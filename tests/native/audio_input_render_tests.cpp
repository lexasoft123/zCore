#include <zcore/device/audio_input_backend.h>
#include <zcore/audio/capture_recording.h>
#include "audio_input_macos_render.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

namespace {
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); ++failures; } } while (false)
using namespace singz::detail;
using Disposition = MacInputRenderDisposition;

void classification() {
  MacInputRenderRecovery recovery;
  CHECK(classifyMacInputRender(0, 0, recovery) == Disposition::Deliver);
  for (const int32_t status : {-10863, -50, -1, 1}) {
    CHECK(classifyMacInputRender(kMacInputTooManyFrames, 0, recovery) == Disposition::Drop);
    CHECK(classifyMacInputRender(status, 1, recovery) == Disposition::Fatal);
    CHECK(recovery.consecutive == 0);
  }
  recovery = {};
  for (uint32_t i = 0; i < kMacInputSliceDropLimit; ++i)
    CHECK(classifyMacInputRender(-10874, i, recovery) == Disposition::Drop);
  CHECK(classifyMacInputRender(-10874, 64, recovery) == Disposition::Fatal);
  for (uint32_t i = 0; i < 1000; ++i)
    CHECK(classifyMacInputRender(-10874, 65 + i, recovery) == Disposition::Fatal);
  CHECK(recovery.consecutive == kMacInputSliceDropLimit + 1);

  recovery = {};
  CHECK(classifyMacInputRender(-10874, 100, recovery) == Disposition::Drop);
  CHECK(classifyMacInputRender(-10874, 100 + kMacInputSliceDropWindowNs - 1, recovery) == Disposition::Drop);
  CHECK(classifyMacInputRender(-10874, 100 + kMacInputSliceDropWindowNs, recovery) == Disposition::Fatal);
  CHECK(recovery.consecutive == 3); // wall-clock bound independent of callback count
  CHECK(classifyMacInputRender(0, 900000000, recovery) == Disposition::Deliver);
  CHECK(recovery.consecutive == 0 && recovery.firstDropNs == 0);
  CHECK(classifyMacInputRender(-10874, 1000000000, recovery) == Disposition::Drop);
  // A successful callback ends a burst even when it arrives after the time bound.
  CHECK(classifyMacInputRender(0, 2000000000, recovery) == Disposition::Deliver);
  CHECK(classifyMacInputRender(-10874, 3000000000, recovery) == Disposition::Drop);
  CHECK(recovery.consecutive == 1 && recovery.firstDropNs == 3000000000);
}

void silenceReplacement() {
  std::array<float, 514> storage;
  storage.fill(-99.f);
  CHECK(prepareMacInputRenderBlock(Disposition::Drop, storage.data() + 1, 512));
  CHECK(storage.front() == -99.f && storage.back() == -99.f);
  for (uint32_t i = 1; i <= 512; ++i) CHECK(storage[i] == 0.f);
  storage.fill(0.5f);
  CHECK(prepareMacInputRenderBlock(Disposition::Deliver, storage.data() + 1, 512));
  for (const auto sample : storage) CHECK(sample == 0.5f);
  CHECK(!prepareMacInputRenderBlock(Disposition::Fatal, storage.data() + 1, 512));
  for (const auto sample : storage) CHECK(sample == 0.5f);
}

// FAKE backend through the real AudioInput facade. The exact production status
// classification, accounting and silence helpers execute here; MacAudioInputBackend/AUHAL render,
// callback return values, HAL properties and the physical device do NOT.
// Reproduce 32 -> 512, including a shrink followed by another growth race.
struct Capture {
  MacInputRenderRecovery recovery;
  std::atomic<uint64_t> drops{0};
  std::atomic<int32_t> failure{0};
  singz::AudioInputPush push = nullptr;
  void* context = nullptr;
  void render(int32_t status, uint32_t frames, uint64_t firstNs, float value) {
    const auto disposition = observeMacInputRender(status, firstNs, recovery, drops);
    if (disposition == Disposition::Fatal) {
      failure.store(status, std::memory_order_release);
      return;
    }
    float samples[512];
    CHECK(frames <= 512);
    for (uint32_t i = 0; i < frames; ++i) samples[i] = value;
    if (!prepareMacInputRenderBlock(disposition, samples, frames)) return;
    CHECK(push(context, samples, frames, firstNs, firstNs,
               singz::AudioInputTimestampQuality::Hardware));
  }
};
Capture* capture = nullptr;
class Input final : public singz::AudioInputBackend {
 public:
  singz::AudioInputResult open(const singz::AudioInputConfig&,
                             singz::AudioInputPush push, void* context) override {
    capture->push = push;
    capture->context = context;
    return result();
  }
  singz::AudioInputResult start() override {
    capture->recovery = {};
    capture->drops.store(0);
    capture->failure.store(0);
    return result();
  }
  void stop() override {}
  bool suspendPrepared() override { return true; }
  bool takeFailure(std::string& error) override {
    const auto status = capture->failure.exchange(0, std::memory_order_acq_rel);
    if (!status) return false;
    error = "AUHAL render callback failed (" + std::to_string(status) + ")";
    return true;
  }
  uint64_t renderSliceDrops() const noexcept override { return capture->drops.load(); }
 private:
  static singz::AudioInputResult result() {
    return singz::AudioInputResult::success(singz::AudioInputState::Running, 48000, 0);
  }
};
std::unique_ptr<singz::AudioInputBackend> factory() { return std::make_unique<Input>(); }

template <class Predicate> void waitFor(Predicate ready) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (!ready() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  CHECK(ready());
}

void captureContinues() {
  Capture state;
  capture = &state;
  singz::setAudioInputBackendForTests(factory, {{"input:slice", "Fixture", true, 48000, 1, {}}});
  {
    singz::AudioInput input;
    singz::CaptureRecording recording;
    struct Observed {
      singz::AudioInputCaptureMetadata capture;
      uint32_t frames = 0;
      bool silent = false;
    };
    std::array<Observed, 16> observed;
    std::atomic<uint32_t> blocks{0}, garbage{0};
    auto sink = [&](const singz::AudioInputBlockView& block) {
      bool silent = true;
      for (uint32_t i = 0; i < block.frames; ++i) {
        if (block.mono[i] < 0) garbage.fetch_add(1);
        silent = silent && block.mono[i] == 0.f;
      }
      const auto index = blocks.load(std::memory_order_relaxed);
      if (index < observed.size()) observed[index] = {block.capture, block.frames, silent};
      recording.append(block);
      blocks.store(index + 1, std::memory_order_release);
    };
    constexpr std::array<uint64_t, 6> times{
        1000000000, 1000666667, 1011333333, 1022000000, 1022666667, 1033333333};
    CHECK(input.start({"input:slice", 0, 8}, sink).ok);
    state.render(0, 32, times[0], 0.25f);
    waitFor([&] { return blocks.load(std::memory_order_acquire) == 1; });
    state.render(-10874, 512, times[1], -99.f);
    waitFor([&] { return blocks.load(std::memory_order_acquire) >= 2; });
    CHECK(input.stats().renderSliceDrops == 1);
    CHECK(state.failure.load() == 0 && blocks.load() == 2);
    state.render(0, 512, times[2], 0.5f);
    waitFor([&] { return blocks.load(std::memory_order_acquire) >= 3; });
    CHECK(input.state() == singz::AudioInputState::Running && input.lastError().empty());
    CHECK(blocks.load() == 3 && garbage.load() == 0);
    CHECK(input.stats().deliveredBlocks == 3 && input.stats().deliveredFrames == 1056);
    CHECK(!observed[0].silent && observed[1].silent && !observed[2].silent);
    CHECK(observed[1].frames == 512);
    // Exercise the actual concatenating take writer: the replacement's 512
    // zeros occupy their original positions and later signal starts at 544.
    CHECK(recording.pcm.size() == 1056);
    if (recording.pcm.size() == 1056) {
      CHECK(recording.pcm[0] > 0 && recording.pcm[31] > 0);
      for (uint32_t i = 32; i < 544; ++i) CHECK(recording.pcm[i] == 0);
      CHECK(recording.pcm[544] > 0 && recording.pcm[1055] > 0);
    }
    // A shrink invalidates presets in real AUHAL; a subsequent growth needs
    // the same tolerance again, rather than a once-per-open exception.
    state.render(0, 32, times[3], 0.25f);
    waitFor([&] { return blocks.load() >= 4; });
    state.render(-10874, 512, times[4], -99.f);
    state.render(0, 512, times[5], 0.5f);
    waitFor([&] { return blocks.load() >= 6; });
    CHECK(blocks.load() == 6 && garbage.load() == 0);
    CHECK(input.stats().renderSliceDrops == 2);
    CHECK(observed[4].silent && observed[4].frames == 512);
    uint64_t sourceFrame = 0;
    for (uint32_t i = 0; i < 6; ++i) {
      CHECK(observed[i].capture.sequence == i);
      CHECK(observed[i].capture.sourceFrame == sourceFrame);
      CHECK(observed[i].capture.sampleHostTimeNs == times[i]);
      CHECK(observed[i].capture.callbackHostTimeNs == times[i]);
      CHECK(observed[i].capture.timestampQuality == singz::AudioInputTimestampQuality::Hardware);
      sourceFrame += observed[i].frames;
    }
    CHECK(sourceFrame == 2112 && recording.pcm.size() == 2112);
    input.stop(true);
    CHECK(input.stats().renderSliceDrops == 2);
    CHECK(input.start({"input:slice", 0, 8}, sink).ok);
    CHECK(input.stats().renderSliceDrops == 0); // per-take, including retained re-arm
    state.render(-10874, 512, 100000000, -99.f);
    CHECK(input.stats().renderSliceDrops == 1);
    input.stop(false);
    CHECK(input.stats().renderSliceDrops == 1); // backend destroyed, final count kept
    CHECK(input.start({"input:slice", 0, 8}, sink).ok);
    CHECK(input.stats().renderSliceDrops == 0);
    state.render(-10874, 512, 1000000000, -99.f);
    state.render(-10874, 512, 1250000000, -99.f);
    waitFor([&] { return input.state() == singz::AudioInputState::Error; });
    CHECK(input.lastError().find("-10874") != std::string::npos);
    CHECK(input.stats().renderSliceDrops == 1);
    input.stop(false);
    CHECK(input.stats().renderSliceDrops == 1);
    CHECK(input.start({"input:slice", 0, 8}, sink).ok);
    state.render(-10863, 512, 2000000000, -99.f);
    waitFor([&] { return input.state() == singz::AudioInputState::Error; });
    CHECK(input.lastError().find("-10863") != std::string::npos);
    CHECK(input.stats().renderSliceDrops == 0);
    input.stop(false);
  }
  singz::setAudioInputBackendForTests(nullptr, {});
  capture = nullptr;
}
}  // namespace

int main() {
  classification();
  silenceReplacement();
  captureContinues();
  return failures ? 1 : 0;
}
