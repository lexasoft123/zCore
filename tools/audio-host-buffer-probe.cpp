#include <zcore/device/audio_host.h>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cstdio>
#include <string_view>
#include <thread>

namespace {
bool silence(void*, const singz::AudioHostRenderBlock& block) noexcept {
  for (uint32_t channel = 0; channel < block.outputChannels; ++channel)
    std::fill_n(block.output[channel], block.frames, 0.0F);
  return true;
}
bool frames(const char* text, uint32_t& value) {
  const std::string_view view{text};
  const auto parsed = std::from_chars(view.data(), view.data() + view.size(), value);
  return parsed.ec == std::errc{} && parsed.ptr == view.data() + view.size() && value != 0;
}
void print(const char* label, const singz::AudioHostResult& result) {
  std::printf("%s ok=%d negotiated=%u nominal=%u maximum=%u message=%s\n", label, result.ok,
              result.latency.bufferFrames, result.format.nominalBufferFrames,
              result.format.maximumFrames, result.message.c_str());
}
}  // namespace

int main(int argc, char** argv) {
  uint32_t opening = 0, live = 0;
  if (argc != 4 || std::string_view{argv[1]} != "--live-buffer" ||
      !frames(argv[2], opening) || !frames(argv[3], live)) {
    std::fprintf(stderr, "Usage: %s --live-buffer OPEN_FRAMES LIVE_FRAMES\n"
                         "Opt-in hardware probe; always renders silence. Changes the shared device buffer.\n", argv[0]);
    return 2;
  }
  singz::AudioHost host;
  const auto inventory = host.enumerate();
  for (const auto& device : inventory.devices) {
    if (device.uid != inventory.defaultOutputUid || device.outputChannels == 0) continue;
    std::printf("output=%s range=%u..%u current=%u (shared with other apps)\n",
                device.label.c_str(), device.bufferFrames.minimumFrames,
                device.bufferFrames.maximumFrames, device.bufferFrames.preferredFrames);
    singz::AudioHostConfig config;
    config.outputDeviceUid = device.uid;
    for (uint32_t channel = 0; channel < std::min(2u, device.outputChannels); ++channel)
      config.outputChannels.push_back(channel);
    config.allowSharedDeviceBufferChange = true;
    config.requestedBufferFrames = opening;
    auto opened = host.open(config, silence, nullptr);
    print("open", opened);
    if (!opened.ok) return 1;
    auto started = host.start();
    print("start", started);
    if (!started.ok) return 1;
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto before = host.status();
    const auto changed = host.setBufferFrames(live);
    print("live", changed);
    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto after = host.status();
    std::printf("callbacks=%llu->%llu route=%llu->%llu stream=%llu->%llu "
                "negotiated=%u xruns=%llu deadlineMisses=%llu discontinuities=%llu "
                "invalidCallbacks=%llu renderFailures=%llu bufferExceedsMaximum=%d bufferWarnings=%u "
                "observedCallbackFrames=%u\n",
                static_cast<unsigned long long>(before.callbacks),
                static_cast<unsigned long long>(after.callbacks),
                static_cast<unsigned long long>(before.routeGeneration),
                static_cast<unsigned long long>(after.routeGeneration),
                static_cast<unsigned long long>(before.streamGeneration),
                static_cast<unsigned long long>(after.streamGeneration), after.latency.bufferFrames,
                static_cast<unsigned long long>(after.xruns),
                static_cast<unsigned long long>(after.deadlineMisses),
                static_cast<unsigned long long>(after.discontinuities),
                static_cast<unsigned long long>(after.invalidCallbacks),
                static_cast<unsigned long long>(after.renderFailures), after.bufferExceedsMaximum,
                after.bufferWarnings, after.observedCallbackFrames);
    host.stop();
    std::printf("stop bufferWarnings=%u\n", host.status().bufferWarnings);
    const bool healthy = after.state == singz::AudioHostState::Running &&
        after.callbacks > before.callbacks && after.routeGeneration == before.routeGeneration &&
        after.streamGeneration == before.streamGeneration &&
        after.observedCallbackFrames == changed.latency.bufferFrames;
    return changed.ok && healthy && host.status().state == singz::AudioHostState::Stopped ? 0 : 1;
  }
  std::fprintf(stderr, "No default output device\n");
  return 1;
}
