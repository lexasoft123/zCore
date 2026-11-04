#include "routing_internal.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <unordered_set>

namespace zcore::routing {
namespace {
bool validChannels(uint32_t channels) {
  return channels > 0 && channels <= kMaximumRoutingChannels;
}

bool validGain(float gain) {
  return std::isfinite(gain) && gain >= 0 && gain <= 1;
}

} // namespace

void validateRoutingConfiguration(const RoutingConfiguration &configuration) {
  if (!validChannels(configuration.outputChannels) ||
      !validGain(configuration.gain) ||
      configuration.sources.size() > kMaximumRoutingSources)
    throw std::invalid_argument("Invalid routing output, gain or source count");
  std::unordered_set<std::string> ids;
  for (const auto &source : configuration.sources) {
    if (source.id.empty() || !ids.insert(source.id).second ||
        !validChannels(source.channels) || !validGain(source.gain))
      throw std::invalid_argument("Invalid or duplicate routing source");
    std::array<bool, kMaximumRoutingChannels * kMaximumRoutingChannels>
        connected{};
    for (const auto &connection : source.connections) {
      if (connection.sourceChannel >= source.channels ||
          connection.outputChannel >= configuration.outputChannels ||
          !validGain(connection.gain))
        throw std::invalid_argument("Invalid routing connection");
      const auto index = connection.outputChannel * kMaximumRoutingChannels +
                         connection.sourceChannel;
      if (connected[index])
        throw std::invalid_argument("Duplicate routing connection");
      connected[index] = true;
    }
  }
}

std::vector<ChannelConnection>
defaultChannelConnections(uint32_t sourceChannels, uint32_t outputChannels) {
  if (!validChannels(sourceChannels) || !validChannels(outputChannels))
    throw std::invalid_argument("Invalid channel count for automatic wiring");
  std::vector<ChannelConnection> result;
  for (uint32_t channel = 0; channel < std::min(sourceChannels, outputChannels);
       ++channel)
    result.push_back({channel, channel});
  if (sourceChannels == 1 && outputChannels >= 2)
    result.push_back({0, 1});
  return result;
}

std::vector<ChannelConnection>
discreteStereoConnections(uint32_t sourceChannels) {
  if (!validChannels(sourceChannels))
    throw std::invalid_argument(
        "Invalid channel count for discrete stereo folding");
  if (sourceChannels == 1)
    return {{0, 0}, {0, 1}};
  std::vector<ChannelConnection> result;
  result.reserve(sourceChannels);
  for (uint32_t channel = 0; channel < sourceChannels; ++channel)
    result.push_back({channel, channel % 2});
  return result;
}

zdsp::AudioBusDescriptor
RoutingGraph::Impl::descriptor(uint32_t channels) const {
  return {channels, zdsp::SampleFormat::Float32Planar,
          channels == 1   ? zdsp::AudioChannelLayout::Mono
          : channels == 2 ? zdsp::AudioChannelLayout::Stereo
                          : zdsp::AudioChannelLayout::Discrete,
          channels <= 2 ? nullptr : channelRoles.data()};
}

zdsp::NodeId RoutingGraph::Impl::addProcessor(
    zdsp::BuiltinNodeKind kind, uint32_t inputChannels, uint32_t buses,
    const zdsp::AudioBusDescriptor *inputs, float value, const float *matrix) {
  using namespace zdsp;
  const NodeId id{static_cast<uint32_t>(nodes.size() + 1)};
  BuiltinNodeConfig config{kind,
                           id,
                           inputChannels,
                           configuration.outputChannels,
                           buses,
                           value,
                           0,
                           0,
                           OscillatorWaveform::Sine,
                           matrix,
                           0};
  const auto stateBytes = builtinStateBytes(config);
  auto *state = static_cast<uint8_t *>(arenaAllocate(&arena, stateBytes, 64));
  const auto preparedBytes = builtinPreparedBytes(config, {kBlock});
  auto *prepared =
      preparedBytes ? arenaAllocate(&arena, preparedBytes, 64) : nullptr;
  if (!state || (preparedBytes && !prepared))
    throw std::runtime_error("Routing DSP processor capacity exhausted");
  const auto processor = createBuiltinProcessor(
      config, {state, static_cast<uint32_t>(stateBytes)});
  nodes.push_back({id,
                   {1, static_cast<uint64_t>(kind)},
                   1,
                   GraphNodeRole::Processor,
                   GraphNodeFlagNone,
                   buses,
                   1,
                   inputs,
                   &routedDescriptor,
                   processor,
                   {prepared, preparedBytes, 64}});
  return id;
}

RoutingGraph::Impl::Impl(double sampleRate, const RoutingConfiguration &route)
    : rate(sampleRate), configuration(route) {
  using namespace zdsp;
  validateRoutingConfiguration(configuration);
  if (!std::isfinite(rate) || rate <= 0)
    throw std::invalid_argument("Invalid routing sample rate");
  for (size_t index = 0; index < configuration.sources.size(); ++index) {
    const auto &source = configuration.sources[index];
    if (source.enabled && !source.muted && source.gain > 0 &&
        std::any_of(source.connections.begin(), source.connections.end(),
                    [](const auto &wire) { return wire.gain > 0; }))
      activeInputs.push_back(index);
  }
  silent = !configuration.enabled || configuration.muted ||
           configuration.gain == 0 || activeInputs.empty();
  if (silent)
    return;
  // The pinned zdsp runner accepts sixteen external buses. Compose
  // independent prepared graphs for larger source lists; only their sum
  // receives the device gain/limiter, so groups cannot clip separately.
  if (configuration.sources.size() > kMaximumBusesPerProcessor) {
    RoutingConfiguration sum = configuration;
    sum.sources.clear();
    for (size_t first = 0; first < configuration.sources.size();
         first += kMaximumBusesPerProcessor) {
      const auto count =
          std::min(configuration.sources.size() - first,
                   static_cast<size_t>(kMaximumBusesPerProcessor));
      RoutingConfiguration groupConfiguration;
      groupConfiguration.outputChannels = configuration.outputChannels;
      groupConfiguration.safetyLimiter = false;
      groupConfiguration.sources.assign(configuration.sources.begin() + first,
                                        configuration.sources.begin() + first +
                                            count);
      auto &group = groups.emplace_back();
      group.first = first;
      group.count = count;
      group.samples.resize(static_cast<size_t>(configuration.outputChannels) *
                           kBlock);
      for (uint32_t channel = 0; channel < configuration.outputChannels;
           ++channel) {
        group.writable[channel] = group.samples.data() + channel * kBlock;
        group.readable[channel] = group.writable[channel];
      }
      group.graph = std::make_unique<RoutingGraph>(rate, groupConfiguration);
      sum.sources.push_back(
          {"group-" + std::to_string(first), configuration.outputChannels, 1,
           true, false,
           defaultChannelConnections(configuration.outputChannels,
                                     configuration.outputChannels)});
    }
    sumGraph = std::make_unique<RoutingGraph>(rate, sum);
    return;
  }
  size_t channels = 0;
  for (const auto index : activeInputs) {
    const auto &source = configuration.sources[index];
    channels += source.channels + 2 * configuration.outputChannels;
  }
  // Compilation and processor states have bounded overhead. Size planar
  // scratch for all source/map outputs before entering the audio thread.
  memory.resize(4 * 1024 * 1024 + channels * kBlock * sizeof(float));
  if (!succeeded(initializeArena(
          &arena, {memory.data(), static_cast<uint32_t>(memory.size())})))
    throw std::runtime_error("Routing DSP arena initialization failed");
  channelRoles.fill(AudioChannelRole::Discrete);
  routedDescriptor = descriptor(configuration.outputChannels);
  mixDescriptors.fill(routedDescriptor);
  inputDescriptors.reserve(activeInputs.size());
  matrices.reserve(activeInputs.size());
  nodes.reserve(kMaximumGraphNodes);
  edges.reserve(kMaximumGraphConnections);
  std::vector<NodeId> mapped;
  for (const auto index : activeInputs) {
    const auto &source = configuration.sources[index];
    const NodeId inputId{static_cast<uint32_t>(nodes.size() + 1)};
    inputDescriptors.push_back(descriptor(source.channels));
    nodes.push_back({inputId,
                     {0, inputId.value},
                     1,
                     GraphNodeRole::Input,
                     GraphNodeFlagNone,
                     0,
                     1,
                     nullptr,
                     &inputDescriptors.back(),
                     {},
                     {}});
    const bool identityMapping =
        source.channels == configuration.outputChannels &&
        source.connections.size() == source.channels &&
        std::all_of(source.connections.begin(), source.connections.end(),
                    [](const auto &wire) {
                      return wire.sourceChannel == wire.outputChannel &&
                             wire.gain == 1;
                    });
    if (identityMapping) {
      const float sourceGain =
          source.enabled && !source.muted ? source.gain : 0;
      if (sourceGain == 1) {
        mapped.push_back(inputId);
      } else {
        const auto gain = addProcessor(BuiltinNodeKind::Gain, source.channels,
                                       1, &inputDescriptors.back(), sourceGain);
        edges.push_back({inputId, 0, gain, 0});
        mapped.push_back(gain);
      }
      continue;
    }
    auto &matrix = matrices.emplace_back(
        source.channels * configuration.outputChannels, 0.0f);
    for (const auto &wire : source.connections)
      matrix[wire.outputChannel * source.channels + wire.sourceChannel] =
          source.enabled && !source.muted ? wire.gain * source.gain : 0;
    const auto mapId =
        addProcessor(BuiltinNodeKind::ChannelMap, source.channels, 1,
                     &inputDescriptors.back(), 1, matrix.data());
    edges.push_back({inputId, 0, mapId, 0});
    mapped.push_back(mapId);
  }
  while (mapped.size() > 1) {
    std::vector<NodeId> next;
    for (size_t first = 0; first < mapped.size();
         first += kMaximumBusesPerProcessor) {
      const auto count = static_cast<uint32_t>(
          std::min(mapped.size() - first,
                   static_cast<size_t>(kMaximumBusesPerProcessor)));
      const auto mix =
          addProcessor(BuiltinNodeKind::Mix, configuration.outputChannels,
                       count, mixDescriptors.data());
      for (uint32_t bus = 0; bus < count; ++bus)
        edges.push_back({mapped[first + bus], 0, mix, bus});
      next.push_back(mix);
    }
    mapped = std::move(next);
  }
  auto previous = mapped[0];
  if (configuration.gain != 1 || !configuration.safetyLimiter) {
    const auto gain =
        addProcessor(BuiltinNodeKind::Gain, configuration.outputChannels, 1,
                     &routedDescriptor, configuration.gain);
    edges.push_back({previous, 0, gain, 0});
    previous = gain;
  }
  if (configuration.safetyLimiter) {
    const auto limiter =
        addProcessor(BuiltinNodeKind::SafetyLimiter,
                     configuration.outputChannels, 1, &routedDescriptor, 0.98f);
    edges.push_back({previous, 0, limiter, 0});
    previous = limiter;
  }
  // Public pre/post-gain meters are published by AudioScene. An unused
  // internal meter would add another full multichannel processing pass.
  const NodeId outputId{static_cast<uint32_t>(nodes.size() + 1)};
  nodes.push_back({outputId,
                   {0, outputId.value},
                   1,
                   GraphNodeRole::Output,
                   GraphNodeFlagNone,
                   1,
                   0,
                   &routedDescriptor,
                   nullptr,
                   {},
                   {}});
  edges.push_back({previous, 0, outputId, 0});
  GraphDescription description{kGraphFormatVersion,
                               {rate},
                               {kBlock},
                               nodes.data(),
                               static_cast<uint32_t>(nodes.size()),
                               edges.data(),
                               static_cast<uint32_t>(edges.size())};
  GraphCompileError error{};
  if (!succeeded(compileGraph(description, &arena, &compiled, &error))) {
    (void)cleanupFailedCompile(&compiled);
    throw std::runtime_error("Routing DSP compilation failed: " +
                             std::to_string(static_cast<uint32_t>(error.kind)));
  }
  initializePublisher(&publisher, retired, 2, &diagnostics);
  snapshot.graph = compiled.graph;
  snapshot.generation = 1;
  snapshot.transition = {TransitionKind::HardCut,
                         {0},
                         {0},
                         {0},
                         InfiniteTailPolicy::Cut,
                         {TailKind::None, {0}},
                         {0},
                         0,
                         100,
                         1000,
                         0};
  if (!succeeded(submitSnapshot(&publisher, &snapshot).status)) {
    (void)deactivateCompiledGraph(compiled.graph);
    throw std::runtime_error("Routing DSP publication failed");
  }
  initializeGraphRunner(&runner, &publisher, {}, nullptr, nullptr,
                        &diagnostics);
  runnerInitialized = true;
}

RoutingGraph::Impl::~Impl() {
  if (!runnerInitialized)
    return;
  zdsp::PublishedGraphSnapshot *snapshots[4]{};
  uint32_t count = 0;
  (void)zdsp::shutdownGraphRunner(&runner, snapshots, 4, &count);
  (void)zdsp::deactivateCompiledGraph(compiled.graph);
}

RoutingGraph::RoutingGraph(double sampleRate,
                           const RoutingConfiguration &configuration)
    : impl_(std::make_unique<Impl>(sampleRate, configuration)) {}
RoutingGraph::~RoutingGraph() = default;

} // namespace zcore::routing
