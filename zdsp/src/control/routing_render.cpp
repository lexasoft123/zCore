#include "routing_internal.h"
#include <algorithm>
#include <array>

namespace zcore::routing {
namespace {
void silence(RoutingOutput output, uint32_t frames) noexcept {
  const auto count = std::min(frames, output.capacityFrames);
  for (auto *channel : output.channels)
    if (channel)
      std::fill_n(channel, count, 0.0f);
}
} // namespace

bool RoutingGraph::render(std::span<const RoutingInput> sources,
                          RoutingOutput output, uint32_t frames,
                          uint64_t position) noexcept {
  using namespace zdsp;
  const auto &configuration = impl_->configuration;
  bool valid = frames > 0 && frames <= kBlock &&
               output.capacityFrames >= frames &&
               output.channels.size() == configuration.outputChannels &&
               sources.size() == configuration.sources.size();
  for (auto *channel : output.channels)
    valid = valid && channel;
  if (valid) {
    for (size_t index = 0; index < sources.size(); ++index) {
      valid = valid && sources[index].capacityFrames >= frames &&
              sources[index].channels.size() ==
                  configuration.sources[index].channels;
      for (const auto *channel : sources[index].channels)
        valid = valid && channel;
    }
  }
  if (!valid || impl_->silent) {
    silence(output, frames);
    return valid;
  }
  if (impl_->sumGraph) {
    std::array<RoutingInput, kMaximumRoutingSources> groups{};
    for (size_t index = 0; index < impl_->groups.size(); ++index) {
      auto &group = impl_->groups[index];
      const RoutingOutput intermediate{
          std::span<float *const>(group.writable.data(),
                                  configuration.outputChannels),
          kBlock};
      if (!group.graph->render(sources.subspan(group.first, group.count),
                               intermediate, frames, position)) {
        silence(output, frames);
        return false;
      }
      groups[index] = {std::span<const float *const>(
                           group.readable.data(), configuration.outputChannels),
                       kBlock};
    }
    return impl_->sumGraph->render(
        std::span<const RoutingInput>(groups.data(), impl_->groups.size()),
        output, frames, position);
  }
  std::array<ConstAudioBusView, kMaximumRoutingSources> inputs{};
  for (size_t index = 0; index < impl_->activeInputs.size(); ++index) {
    const auto &source = sources[impl_->activeInputs[index]];
    inputs[index] = {source.channels.data(),
                     static_cast<uint32_t>(source.channels.size()),
                     {frames},
                     {frames},
                     nullptr};
  }
  MutableAudioBusView destination{
      output.channels.data(), configuration.outputChannels, {frames}, {frames}};
  ProcessContext context{kProcessContextInterfaceVersion,
                         kProcessContextV1RequiredSize,
                         {{1}, {1}, {position}, {0}, {0}, RenderTimeNone},
                         nullptr,
                         {impl_->rate},
                         {frames},
                         nullptr,
                         0,
                         nullptr,
                         0,
                         {nullptr, 0},
                         {DiscontinuityReason::None, DiscontinuityFlagNone}};
  const bool passed = succeeded(renderGraphBlock(
      &impl_->runner, context, inputs.data(),
      static_cast<uint32_t>(impl_->activeInputs.size()), &destination, 1));
  if (!passed)
    silence(output, frames);
  return passed;
}
} // namespace zcore::routing
