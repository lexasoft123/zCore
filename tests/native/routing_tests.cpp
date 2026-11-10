#include <zcore/routing/routing.h>

#include <array>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <new>
#include <stdexcept>

namespace {
bool trackHeap = false;
uint64_t heapEvents = 0;
} // namespace

// Preserve heap-hook boundaries when GCC optimizes allocation probes.
#if defined(__GNUC__)
#define LOOPZ_HEAP_HOOK __attribute__((noinline))
#else
#define LOOPZ_HEAP_HOOK
#endif
LOOPZ_HEAP_HOOK void *operator new(std::size_t bytes) {
  if (trackHeap)
    ++heapEvents;
  if (void *memory = std::malloc(bytes ? bytes : 1))
    return memory;
  throw std::bad_alloc();
}
LOOPZ_HEAP_HOOK void *operator new[](std::size_t bytes) {
  return ::operator new(bytes);
}
LOOPZ_HEAP_HOOK void operator delete(void *memory) noexcept {
  if (trackHeap && memory)
    ++heapEvents;
  std::free(memory);
}
LOOPZ_HEAP_HOOK void operator delete[](void *memory) noexcept {
  ::operator delete(memory);
}
LOOPZ_HEAP_HOOK void operator delete(void *memory, std::size_t) noexcept {
  ::operator delete(memory);
}
LOOPZ_HEAP_HOOK void operator delete[](void *memory, std::size_t) noexcept {
  ::operator delete(memory);
}

#undef LOOPZ_HEAP_HOOK

namespace {
bool renderWithoutHeap(zcore::routing::RoutingGraph &graph,
                       std::span<const zcore::routing::RoutingInput> inputs,
                       zcore::routing::RoutingOutput output, uint32_t frames,
                       uint64_t position) {
  heapEvents = 0;
  trackHeap = true;
  const bool passed = graph.render(inputs, output, frames, position);
  trackHeap = false;
  assert(heapEvents == 0);
  return passed;
}

void close(float actual, float expected) {
  assert(std::isfinite(actual) && std::fabs(actual - expected) < 0.00001f);
}

void discreteStereoFold(double rate) {
  using namespace zcore::routing;
  for (auto count : {1U, 2U, 3U, 8U, 16U, 64U}) {
    RoutingConfiguration configuration;
    configuration.safetyLimiter = false;
    configuration.sources = {
        {"discrete", count, 1, true, false, discreteStereoConnections(count)}};
    RoutingGraph graph(rate, configuration);
    std::array<std::array<float, kBlock>, 64> samples{};
    std::array<const float *, 64> planes{};
    std::array<float, kBlock> left{}, right{};
    float *outputs[]{left.data(), right.data()};
    for (uint32_t channel = 0; channel < count; ++channel)
      planes[channel] = samples[channel].data();
    const std::array<RoutingInput, 1> input{{{{planes.data(), count}, kBlock}}};
    // Every native channel reaches its destination at unity, including the
    // last pair of a 64-channel interface. Idle pairs never attenuate it.
    for (uint32_t active = 0; active < count; ++active) {
      samples[active].fill(0.25f);
      assert(renderWithoutHeap(graph, input, {outputs, kBlock}, kBlock,
                               active * kBlock));
      for (uint32_t frame = 0; frame < kBlock; ++frame) {
        close(left[frame], count == 1 || active % 2 == 0 ? 0.25f : 0);
        close(right[frame], count == 1 || active % 2 == 1 ? 0.25f : 0);
      }
      samples[active].fill(0);
    }
    for (uint32_t channel = 0; channel < count; ++channel)
      samples[channel].fill(0.125f);
    assert(
        renderWithoutHeap(graph, input, {outputs, kBlock}, 63, count * kBlock));
    close(left[0], 0.125f * ((count + 1) / 2));
    close(right[0], count == 1 ? 0.125f : 0.125f * (count / 2));
  }
}

template <typename Operation> void invalid(Operation operation) {
  bool rejected = false;
  try {
    operation();
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  assert(rejected);
}

void routingAndControls(double rate) {
  using namespace zcore::routing;
  RoutingConfiguration configuration;
  configuration.outputChannels = 4;
  configuration.gain = 0.5f;
  configuration.sources = {
      {"stereo-app", 2, 0.5f, true, false, {{0, 1}, {1, 0}, {0, 2, 0.5f}}},
      {"mono-mic", 1, 1, true, false, {{0, 2}, {0, 3}}},
      {"muted", 2, 0.8f, true, true, {{0, 0}, {1, 1}}},
      {"disabled", 2, 1, false, false, {{0, 0}, {1, 1}}},
  };
  std::array<float, kBlock> left{}, right{}, microphone{}, loud{};
  left.fill(0.2f);
  right.fill(0.4f);
  microphone.fill(0.1f);
  loud.fill(10);
  const float *app[]{left.data(), right.data()};
  const float *mic[]{microphone.data()};
  const float *other[]{loud.data(), loud.data()};
  std::array<RoutingInput, 4> inputs{
      {{app, kBlock}, {mic, kBlock}, {other, kBlock}, {other, kBlock}}};
  std::array<std::array<float, kBlock + 2>, 4> storage{};
  std::array<float *, 4> outputs{};
  for (size_t channel = 0; channel < outputs.size(); ++channel) {
    storage[channel].fill(99);
    outputs[channel] = storage[channel].data() + 1;
  }
  RoutingGraph graph(rate, configuration);
  assert(renderWithoutHeap(graph, inputs, {outputs, kBlock}, 64, 0));
  const float expected[]{0.1f, 0.05f, 0.075f, 0.05f};
  for (size_t channel = 0; channel < outputs.size(); ++channel) {
    for (uint32_t frame = 0; frame < 64; ++frame)
      close(outputs[channel][frame], expected[channel]);
    assert(storage[channel][0] == 99 && outputs[channel][64] == 99);
  }
  configuration.muted = true;
  RoutingGraph muted(rate, configuration);
  assert(muted.render(inputs, {outputs, kBlock}, 64, 0));
  for (const auto *channel : outputs)
    for (uint32_t frame = 0; frame < 64; ++frame)
      close(channel[frame], 0);
  assert(configuration.sources[0].gain == 0.5f);
  configuration.muted = false;
  configuration.enabled = false;
  RoutingGraph disabled(rate, configuration);
  assert(disabled.render(inputs, {outputs, kBlock}, 64, 0));
  for (const auto *channel : outputs)
    close(channel[0], 0);
}

void allChannelsAndManySources(double rate) {
  using namespace zcore::routing;
  RoutingConfiguration configuration;
  configuration.outputChannels = 64;
  configuration.sources.push_back(
      {"hardware-64", 64, 1, true, false, defaultChannelConnections(64, 64)});
  std::array<std::array<float, kBlock>, 64> input{}, output{};
  std::array<const float *, 64> inputChannels{};
  std::array<float *, 64> outputChannels{};
  for (uint32_t channel = 0; channel < 64; ++channel) {
    input[channel].fill(static_cast<float>(channel + 1) * 0.01f);
    inputChannels[channel] = input[channel].data();
    outputChannels[channel] = output[channel].data();
  }
  RoutingGraph graph(rate, configuration);
  std::array<RoutingInput, 1> inputs{{{inputChannels, kBlock}}};
  assert(renderWithoutHeap(graph, inputs, {outputChannels, kBlock}, kBlock, 0));
  for (uint32_t channel = 0; channel < 64; ++channel)
    for (uint32_t frame = 0; frame < kBlock; ++frame)
      close(output[channel][frame], input[channel][frame]);

  configuration.sources[0].connections[0].outputChannel = 63;
  configuration.sources[0].connections[63].outputChannel = 0;
  RoutingGraph swapped(rate, configuration);
  assert(
      renderWithoutHeap(swapped, inputs, {outputChannels, kBlock}, kBlock, 0));
  for (uint32_t channel = 0; channel < 64; ++channel) {
    const auto source = channel == 0 ? 63 : channel == 63 ? 0 : channel;
    for (uint32_t frame = 0; frame < kBlock; ++frame)
      close(output[channel][frame], input[source][frame]);
  }

  configuration.sources.clear();
  std::array<RoutingInput, kMaximumRoutingSources> manyInputs{};
  for (uint32_t source = 0; source < kMaximumRoutingSources; ++source) {
    configuration.sources.push_back({std::to_string(source), 64, 0.01f, true,
                                     false, defaultChannelConnections(64, 64)});
    manyInputs[source] = {inputChannels, kBlock};
  }
  RoutingGraph many(rate, configuration);
  assert(
      renderWithoutHeap(many, manyInputs, {outputChannels, kBlock}, kBlock, 0));
  for (uint32_t channel = 0; channel < 64; ++channel)
    for (uint32_t frame = 0; frame < kBlock; ++frame)
      close(output[channel][frame], input[channel][frame] * 0.32f);
}

void silentRoutesPreserveInputOrderAndContainNonfinite(double rate) {
  using namespace zcore::routing;
  std::array<float, kBlock> poison{}, left{}, right{};
  poison.fill(NAN);
  left.fill(0.25f);
  right.fill(0.5f);
  const float *discarded[]{poison.data(), poison.data()};
  const float *live[]{left.data(), right.data()};
  std::array<std::array<float, kBlock>, 2> samples{};
  float *output[]{samples[0].data(), samples[1].data()};
  for (const auto count : {3U, 32U}) {
    RoutingConfiguration configuration;
    configuration.safetyLimiter = false;
    std::vector<RoutingInput> inputs(count, {discarded, kBlock});
    const auto active = count == 3 ? 1U : 17U;
    for (unsigned index = 0; index < count; ++index)
      configuration.sources.push_back({std::to_string(index), 2,
                                       index == active ? 1.0f : 0.0f, true,
                                       false, defaultChannelConnections(2, 2)});
    inputs[active] = {live, kBlock};
    RoutingGraph graph(rate, configuration);
    assert(renderWithoutHeap(graph, inputs, {output, kBlock}, 64, 0));
    close(output[0][0], 0.25f);
    close(output[1][63], 0.5f);
    configuration.sources[active].connections[0].gain = 0;
    configuration.sources[active].connections[1].gain = 0;
    RoutingGraph unwired(rate, configuration);
    assert(renderWithoutHeap(unwired, inputs, {output, kBlock}, 64, 0));
    for (auto *channel : output)
      for (unsigned frame = 0; frame < 64; ++frame)
        close(channel[frame], 0);
    inputs[0].capacityFrames = 1;
    assert(!unwired.render(inputs, {output, kBlock}, 64, 0));
  }
  RoutingConfiguration direct;
  direct.safetyLimiter = false;
  direct.sources.push_back(
      {"live", 2, 1, true, false, defaultChannelConnections(2, 2)});
  RoutingGraph sanitized(rate, direct);
  const std::array<RoutingInput, 1> bad{{{discarded, kBlock}}};
  assert(renderWithoutHeap(sanitized, bad, {output, kBlock}, 64, 0));
  for (auto *channel : output)
    for (unsigned frame = 0; frame < 64; ++frame)
      close(channel[frame], 0);
}
void silenceAndBadBuffers(double rate) {
  using namespace zcore::routing;
  RoutingConfiguration configuration;
  std::array<float, kBlock + 2> left{}, right{};
  left.fill(99);
  right.fill(-99);
  float *output[]{left.data() + 1, right.data() + 1};
  RoutingGraph empty(rate, configuration);
  assert(empty.render({}, {output, kBlock}, kBlock, 0));
  for (uint32_t frame = 1; frame <= kBlock; ++frame)
    close(left[frame], 0);
  assert(left.front() == 99 && left.back() == 99);
  left.fill(99);
  assert(!empty.render({}, {output, 32}, 64, 0));
  assert(left[0] == 99 && left[33] == 99);
  for (uint32_t frame = 1; frame <= 32; ++frame)
    close(left[frame], 0);

  configuration.sources.push_back({"app", 2, 1, true, false, {{0, 0}, {1, 1}}});
  RoutingGraph graph(rate, configuration);
  const float *broken[]{left.data(), nullptr};
  const RoutingInput inputs[]{{broken, kBlock}};
  assert(!graph.render(inputs, {output, kBlock}, 64, 0));
  for (uint32_t frame = 1; frame <= 64; ++frame)
    close(left[frame], 0);
}

// Two separately controlled mono mic buses plus a stereo playback bus use one
// prepared output graph. Every mic reaches L/R without folding the song mono.
void microphonesAndSong(double rate) {
  using namespace zcore::routing;
  RoutingConfiguration configuration;
  configuration.safetyLimiter = false;
  configuration.sources = {
      {"song", 2, 0.5f, true, false, defaultChannelConnections(2, 2)},
      {"mic-1", 1, 0.25f, true, false, defaultChannelConnections(1, 2)},
      {"mic-2", 1, 0.125f, true, false, defaultChannelConnections(1, 2)},
  };
  std::array<float, kBlock> left{}, right{}, mic1{}, mic2{}, outLeft{},
      outRight{};
  left.fill(0.2f);
  right.fill(0.4f);
  mic1.fill(0.1f);
  mic2.fill(-0.4f);
  const float *song[]{left.data(), right.data()};
  const float *first[]{mic1.data()};
  const float *second[]{mic2.data()};
  const std::array<RoutingInput, 3> inputs{
      {{song, kBlock}, {first, kBlock}, {second, kBlock}}};
  float *output[]{outLeft.data(), outRight.data()};
  RoutingGraph mixed(rate, configuration);
  assert(renderWithoutHeap(mixed, inputs, {output, kBlock}, kBlock, 0));
  for (uint32_t frame = 0; frame < kBlock; ++frame) {
    close(outLeft[frame], 0.075f);
    close(outRight[frame], 0.175f);
  }
  configuration.sources[1].muted = true;
  RoutingGraph muted(rate, configuration);
  assert(renderWithoutHeap(muted, inputs, {output, kBlock}, kBlock, kBlock));
  close(outLeft[0], 0.05f);
  close(outRight[0], 0.15f);
  configuration.sources[1].muted = false;
  configuration.sources[0].gain = 1;
  configuration.sources[1].gain = 1;
  configuration.sources[2].gain = 1;
  configuration.safetyLimiter = true;
  left.fill(0.8f);
  right.fill(0.7f);
  mic1.fill(0.6f);
  mic2.fill(0.5f);
  RoutingGraph limited(rate, configuration);
  assert(renderWithoutHeap(limited, inputs, {output, kBlock}, kBlock, 0));
  bool audible = false;
  for (uint32_t frame = 0; frame < kBlock; ++frame) {
    for (const auto *plane : output) {
      assert(std::isfinite(plane[frame]) && std::abs(plane[frame]) <= 0.98001f);
      audible |= std::abs(plane[frame]) > 0.1f;
    }
  }
  assert(audible);
}

} // namespace

int main() {
  using namespace zcore::routing;
  for (double rate : {44100.0, 48000.0}) {
    microphonesAndSong(rate);
    routingAndControls(rate);
    silentRoutesPreserveInputOrderAndContainNonfinite(rate);
    discreteStereoFold(rate);
    allChannelsAndManySources(rate);
    silenceAndBadBuffers(rate);
  }
  const auto mono = defaultChannelConnections(1, 4);
  assert(mono.size() == 2 && mono[0].sourceChannel == 0 &&
         mono[0].outputChannel == 0 && mono[1].sourceChannel == 0 &&
         mono[1].outputChannel == 1);
  invalid([] { (void)defaultChannelConnections(65, 2); });
  invalid([] { (void)discreteStereoConnections(0); });
  invalid([] { (void)discreteStereoConnections(65); });
  invalid([] { (void)RoutingGraph(NAN, {}); });
  RoutingConfiguration configuration;
  configuration.sources.push_back({"app", 2, 1, true, false, {{0, 0}}});
  auto broken = configuration;
  broken.sources[0].connections.push_back({0, 0});
  invalid([&] { validateRoutingConfiguration(broken); });
  broken = configuration;
  broken.sources[0].connections.push_back({2, 1});
  invalid([&] { validateRoutingConfiguration(broken); });
  broken = configuration;
  broken.sources.push_back(broken.sources[0]);
  invalid([&] { validateRoutingConfiguration(broken); });
  broken = configuration;
  broken.sources[0].gain = NAN;
  invalid([&] { validateRoutingConfiguration(broken); });
  std::puts(
      "routing: multichannel mapping, controls, hierarchy and silence passed");
}
