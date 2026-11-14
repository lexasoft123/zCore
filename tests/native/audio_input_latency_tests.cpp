#include <zcore/device/audio_input_backend.h>
#include "zcore/platform/macos/audio_input_macos_latency.h"

#include <cstdio>
#include <limits>

namespace {
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); ++failures; } } while (false)

class FakeInput final : public singz::AudioInputBackend {
 public:
  singz::AudioInputResult open(const singz::AudioInputConfig& config,
                               singz::AudioInputPush, void*) override {
    channel = config.channel;
    return result(singz::AudioInputState::Starting);
  }
  singz::AudioInputResult start() override { return result(singz::AudioInputState::Running); }
  void stop() override {}
  bool suspendPrepared() override { return true; }
  bool takeFailure(std::string&) override { return false; }
 private:
  singz::AudioInputResult result(singz::AudioInputState state) {
    auto r = singz::AudioInputResult::success(state, 48000, channel);
    r.inputLatencyFrames = 1168;
    r.inputBufferFrames = 512;
    r.inputLatencySource = "reported";
    return r;
  }
  uint32_t channel = 0;
};
std::unique_ptr<singz::AudioInputBackend> fakeInput() { return std::make_unique<FakeInput>(); }
}

int main() {
  using singz::detail::macInputLatency;
  const auto total = macInputLatency(40, 112, 1016, 512, 48000, 48000, 48000);
  CHECK(total.reported && total.frames == 1168 && total.bufferFrames == 512);
  const auto converted = macInputLatency(40, 112, 1016, 512, 48000, 44100, 48000);
  CHECK(converted.reported && converted.frames == 1073 && converted.bufferFrames == 470);
  CHECK(macInputLatency(1, 1, 1, 1, 48000, 24000, 48000).frames == 2);
  CHECK(macInputLatency(0, 0, 0, 64, 48000, 48000, 48000).reported);
  for (int missing = 0; missing < 4; ++missing) {
    std::optional<uint32_t> terms[] = {40, 112, 1016, 512};
    terms[missing].reset();
    const auto unknown = macInputLatency(terms[0], terms[1], terms[2], terms[3], 48000, 48000, 48000);
    CHECK(!unknown.reported && !unknown.frames && !unknown.bufferFrames);
  }
  CHECK(!macInputLatency(0, 0, 0, 0, 48000, 48000, 48000).reported);
  CHECK(!macInputLatency(1, 2, 3, 4, 0, 48000, 48000).reported);
  CHECK(!macInputLatency(1, 2, 3, 4, 48000, std::numeric_limits<double>::quiet_NaN(), 48000).reported);
  CHECK(!macInputLatency(UINT32_MAX, UINT32_MAX, UINT32_MAX, UINT32_MAX, 48000, 48000, 48000).reported);
  CHECK(!macInputLatency(1, 2, 3, 4, 1, std::numeric_limits<double>::max(), 48000).reported);

  // Stream frames have their own physical rate, independently of the device.
  CHECK(macInputLatency(40, 112, 508, 512, 48000, 48000, 24000).frames == 1168);
  CHECK(!macInputLatency(1, 2, 3, 4, 48000, 48000, 0).reported);
  std::atomic<bool> stable{true};
  CHECK(singz::detail::macInputControlChanges(singz::detail::kMacInputLatencyChanged, stable) == 0);
  CHECK(!stable.load());
  stable.store(true);
  CHECK(singz::detail::macInputControlChanges(1u, stable) == 1u && stable.load());
  CHECK(singz::detail::macInputControlChanges(1u | singz::detail::kMacInputLatencyChanged, stable) == 1u);
  CHECK(!stable.load());

  singz::setAudioInputBackendForTests(fakeInput, {{"latency:fake", "Fixture", true, 48000, 1, {}}});
  singz::AudioInput input;
  for (int run = 0; run != 2; ++run) {
    const auto result = input.start({"latency:fake", 0, 4}, [](const auto&) {});
    CHECK(result.ok && result.inputLatencyFrames == 1168 && result.inputBufferFrames == 512 &&
          result.inputLatencySource == "reported");
    input.stop(true);
  }
  input.stop();
  singz::setAudioInputBackendForTests(nullptr, {});
  const singz::AudioInputResult unknown;
  CHECK(unknown.inputLatencyFrames == 0 && unknown.inputBufferFrames == 0 && unknown.inputLatencySource == "unknown");
  if (!failures) std::puts("PASS input latency sum, rate conversion, missing properties and fake backend lifecycle");
  return failures ? 1 : 0;
}
