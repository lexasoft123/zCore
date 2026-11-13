#include <zcore/device/audio_input_backend.h>
#include "audio_device_macos_buffer.h"
#include "audio_input_macos_latency.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

namespace {
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); ++failures; } } while (false)

struct Device {
  uint32_t frames = 512, writes = 0;
  std::string uid = "input:shared";
  const std::string& deviceUid() const { return uid; }
  uint64_t deviceIdentity() const { return 1; }
  bool identityMatches() const { return true; }
  bool isSettable() const { return true; }
  bool readFrames(uint32_t& value) const { value = frames; return true; }
  bool readRange(uint32_t& low, uint32_t& high) const { low = 32; high = 32768; return true; }
  bool writeFrames(uint32_t value) { frames = value; ++writes; return true; }
};

// This fixture runs a FAKE AudioInputBackend through the real AudioInput facade.
// It calls production capacity, notification, takeFailure-control and fact helpers.
// It does NOT execute MacAudioInputBackend's actual open/listener registration,
// open fence, AUHAL AudioUnitRender call or OSStatus formatting; those need the
// USB hardware checklist in the report. No physical audio device is opened.
singz::detail::MacAudioInputStreamConfiguration configuration(uint32_t topology = 0) {
  singz::detail::MacAudioInputStreamConfiguration facts;
  facts.identity = 1; facts.rate = 48000; facts.inputChannels = {2};
  singz::detail::MacAudioHostStreamFacts stream;
  stream.identity = 7;
  stream.physical = {48000, 1, 2, 8, 1, 8, 2, 32};
  stream.virtualFormat = stream.physical;
  facts.inputStreams = {stream};
  if (topology == 1) facts.inputChannels = {1, 1};
  if (topology == 3) facts.inputStreams[0].identity++;
  if (topology == 4) facts.inputStreams[0].physical.rate = 44100;
  if (topology == 5) facts.inputStreams[0].virtualFormat.bits = 24;
  return facts;
}

struct Unit {
  std::optional<uint32_t> slice = 32;
  uint32_t sets = 0, initializedSlice = 0;
  bool rejectSet = false, initialized = false, missingPostInit = false;
  bool set(uint32_t frames) {
    CHECK(!initialized); // MaximumFramesPerSlice is raised BEFORE initialize.
    ++sets;
    if (rejectSet) return false;
    slice = frames;
    return true;
  }
  std::optional<uint32_t> read() const {
    return initialized && missingPostInit ? std::nullopt : slice;
  }
  void initialize() {
    initialized = true;
    if (initializedSlice) slice = initializedSlice;
  }
};

struct Capture {
  Device device;
  singz::detail::MacAudioBufferLedger ledger;
  Unit unit;
  uint32_t storage = 0, renderMaximum = 0, warnings = 0;
  std::vector<float> renderStorage;
  std::atomic<bool> callbackTooLarge{false}, latencyStable{true};
  std::atomic<uint32_t> notification{0}, tooManyFrames{0};
  std::atomic<uint32_t> topology{0}, processed{0};
  void callback(uint32_t frames) {
    if (singz::detail::macAudioInputCallbackTooLarge(frames, renderMaximum)) {
      callbackTooLarge.store(true);
      return;
    }
    // Simulated AudioUnitRender rejects frames beyond the unit, even when the
    // vector is large enough. This is intentionally independent of storage.
    if (unit.slice && frames > *unit.slice) {
      tooManyFrames.fetch_add(1);
      callbackTooLarge.store(true);
    }
  }
};
Capture* capture = nullptr;

class Input final : public singz::AudioInputBackend {
 public:
  ~Input() override { stop(); }
  singz::AudioInputResult open(const singz::AudioInputConfig& config,
                             singz::AudioInputPush, void*) override {
    const auto initialSlice = capture->unit.read();
    capture->storage = singz::detail::macAudioInputPreparation(initialSlice).frames;
    const auto lease = singz::detail::requestMacAudioBuffer(
        capture->device, config.requestedBufferFrames, capture->storage, ownership,
        config.maximumAcceptableBufferFrames, capture->ledger, false,
        singz::detail::MacAudioInputBufferCapacity{capture->storage, [&](uint32_t storage) {
          const auto capacity = singz::detail::prepareMacAudioInputUnitSlice(storage, initialSlice,
              capture->warnings, [&](uint32_t frames) { return capture->unit.set(frames); },
              [&] { return capture->unit.read(); });
          return config.requestedBufferFrames == 0
              ? std::min(capacity, initialSlice.value_or(0)) : capacity;
        }});
    if (lease.ok()) {
      capture->renderStorage.resize(capture->storage);
      capture->unit.initialize();
      const auto confirmed = capture->unit.read();
      if (!confirmed || *confirmed < capture->storage)
        capture->warnings |= singz::AudioBufferWarningReadFailed;
      const auto capacity = singz::detail::macAudioInputConfirmedCapacity(capture->storage, confirmed);
      CHECK(singz::detail::updateMacAudioBufferCapacity(ownership, capacity, capture->ledger));
      capture->renderMaximum = confirmed ? capacity : capture->storage;
      CHECK(singz::detail::readMacAudioInputConfiguration(opened, [&](auto& facts) {
        facts = configuration(capture->topology.load()); return true;
      }));
    }
    return lease.ok() ? result(singz::AudioInputState::Starting)
        : singz::AudioInputResult::failure(singz::AudioInputState::Error, lease.message, 0);
  }
  singz::AudioInputResult start() override {
    if (!singz::detail::refreshMacAudioInputConfiguration(opened, [&](auto& facts) {
          facts = configuration(capture->topology.load()); return true;
        })) return singz::AudioInputResult::failure(singz::AudioInputState::Error, "configuration changed on re-arm", 0);
    started = true;
    return result(singz::AudioInputState::Running);
  }
  bool suspendPrepared() override {
    return singz::detail::suspendMacAudioInputPrepared(true, started, [] { return true; });
  }
  uint32_t bufferWarnings() const noexcept override { return capture->warnings; }
  void stop() override {
    (void)singz::detail::restoreMacAudioBuffer(capture->device, ownership, capture->ledger);
  }
  bool takeFailure(std::string& error) override {
    bool failed = false;
    const auto selector = capture->notification.exchange(0);
    const auto changes = selector ? singz::detail::macInputNotificationChanges(selector) : 0;
    const auto failure = singz::detail::macInputTakeControlFailure(changes, capture->latencyStable,
        [] { return true; }, [] { return true; }, [&] {
          return singz::detail::refreshMacAudioInputConfiguration(opened, [&](auto& facts) {
            const auto topology = capture->topology.load();
            facts = configuration(topology); return topology != 2;
          });
        });
    failed = failure != singz::detail::MacInputControlFailure::None;
    if (failed) error = "audio input stream configuration changed; restart capture";
    if (capture->callbackTooLarge.exchange(false)) {
      failed = true;
      error = "audio input callback exceeded the prepared buffer";
    }
    capture->processed.fetch_add(1);
    return failed;
  }
 private:
  singz::AudioInputResult result(singz::AudioInputState state) const {
    return singz::AudioInputResult::success(state, 48000, 0);
  }
  singz::detail::MacAudioBufferOwnership ownership;
  singz::detail::MacAudioInputStreamConfiguration opened;
  bool started = false;
};
std::unique_ptr<singz::AudioInputBackend> inputFactory() { return std::make_unique<Input>(); }

void waitForControl(uint32_t previous) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (capture->processed.load() < previous + 2 && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  // A failed capture exits its delivery loop, so it may make only one check.
  CHECK(capture->processed.load() > previous);
}

void runningCapture(Capture& state, bool monitorFirst, uint32_t expectedCapacity,
                    uint32_t expectedSets, bool warning) {
  using namespace singz::detail;
  capture = &state;
  const auto original = state.device.frames;
  MacAudioBufferOwnership monitor;
  if (monitorFirst)
    CHECK(requestMacAudioBuffer(state.device, 32, 32768, monitor, 0, state.ledger).ok());
  singz::setAudioInputBackendForTests(inputFactory, {{"input:shared", "Fixture", true, 48000, 1, {}}});
  {
    singz::AudioInput input;
    const bool started = input.start({"input:shared", 0, 4}, [](const auto&) {}).ok;
    CHECK(started);
    if (started) {
      CHECK(state.storage >= std::min(original, 16384u) && state.renderStorage.size() == state.storage);
      CHECK(state.unit.sets == expectedSets);
      CHECK((state.warnings != 0) == warning);
      CHECK((input.stats().bufferWarnings != 0) == warning);
      for (const auto& [token, owner] : state.ledger.devices.at(state.device.uid).owners)
        if (token != monitor.token) CHECK(owner.preparedMaximum == expectedCapacity);
      state.callback(state.device.frames);
      if (!monitorFirst) {
        CHECK(requestMacAudioBuffer(state.device, 32, 32768, monitor, 0, state.ledger).ok());
        state.callback(32);
      }
      CHECK(state.device.frames == 32);
      const auto writes = state.device.writes;
      const bool fits = original <= expectedCapacity;
      CHECK(restoreMacAudioBuffer(state.device, monitor, state.ledger) ==
          (fits ? singz::AudioBufferWarningNone : singz::AudioBufferWarningRestoreFailed));
      CHECK(state.device.frames == (fits ? original : 32u));
      CHECK(state.device.writes == writes + (fits ? 1 : 0));
      state.callback(state.device.frames);
      const auto previous = state.processed.load();
      state.notification.store(audioHostFourCc('s', 'l', 'a', 'y'));
      waitForControl(previous);
      CHECK(!state.callbackTooLarge.load() && state.tooManyFrames.load() == 0 && !state.latencyStable.load());
      CHECK(input.state() == singz::AudioInputState::Running && input.lastError().empty());
    }
    input.stop(false);
  }
  (void)restoreMacAudioBuffer(state.device, monitor, state.ledger);
  CHECK(state.ledger.devices.empty() && state.device.frames == original);
  singz::setAudioInputBackendForTests(nullptr, {});
  capture = nullptr;
}

void sliceLimits() {
  for (const auto original : {1024u, 2048u}) {
    for (const auto slice : {512u, 1156u}) {
      for (const bool rejected : {false, true}) {
        Capture state;
        state.device.frames = original;
        state.unit.slice = slice;
        state.unit.rejectSet = rejected;
        const auto storage = std::max(original, slice);
        const auto confirmed = rejected ? slice : storage;
        runningCapture(state, true, std::min(storage, confirmed), storage > slice ? 1 : 0,
                       rejected && storage > slice);
      }
    }
  }
  Capture shrunk;
  shrunk.device.frames = 2048; shrunk.unit.slice = 512; shrunk.unit.initializedSlice = 512;
  runningCapture(shrunk, true, 512, 1, true); // post-init slice overrides successful set
  Capture missing;
  missing.device.frames = 2048; missing.unit.slice = 512; missing.unit.missingPostInit = true;
  runningCapture(missing, true, 0, 1, true); // unknown capacity may never permit restoration
  Capture ordinary;
  ordinary.unit.slice = 512;
  runningCapture(ordinary, false, 512, 0, false); // no unnecessary property write
  Capture missingInitial;
  missingInitial.unit.slice.reset(); missingInitial.unit.initializedSlice = 512;
  runningCapture(missingInitial, true, 512, 0, true); // ordinary open keeps its fallback
}

void genuineFailures() {
  using namespace singz::detail;
  const auto opened = configuration();
  CHECK(!macAudioInputStreamConfigurationChanged(opened, opened, true));
  CHECK(macAudioInputStreamConfigurationChanged(opened, configuration(1), true));
  CHECK(macAudioInputStreamConfigurationChanged(opened, opened, false));
  for (const auto topology : {3u, 4u, 5u})
    CHECK(macAudioInputStreamConfigurationChanged(opened, configuration(topology), true));
  for (const bool physical : {false, true}) {
    for (const auto field : {0, 1, 2, 3, 4, 5, 6, 7}) {
      auto changed = opened;
      auto& format = physical ? changed.inputStreams[0].physical : changed.inputStreams[0].virtualFormat;
      switch (field) {
        case 0: format.rate++; break; case 1: format.id++; break; case 2: format.flags++; break;
        case 3: format.bytesPerPacket++; break; case 4: format.framesPerPacket++; break;
        case 5: format.bytesPerFrame++; break; case 6: format.channels++; break; case 7: format.bits++; break;
      }
      CHECK(macAudioInputStreamConfigurationChanged(opened, changed, true));
    }
  }
  auto added = opened; added.inputStreams.push_back(opened.inputStreams[0]);
  CHECK(macAudioInputStreamConfigurationChanged(opened, added, true));
  CHECK(!macAudioInputCallbackTooLarge(512, 512));
  CHECK(macAudioInputCallbackTooLarge(513, 512));
  for (const auto topology : {1u, 2u, 3u, 4u, 5u}) {
    Capture state;
    capture = &state;
    singz::setAudioInputBackendForTests(inputFactory, {{"input:shared", "Fixture", true, 48000, 1, {}}});
    {
      singz::AudioInput input;
      CHECK(input.start({"input:shared", 0, 4}, [](const auto&) {}).ok);
      state.topology.store(topology);
      state.notification.store(audioHostFourCc('s', 'l', 'a', 'y'));
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
      while (input.state() == singz::AudioInputState::Running && std::chrono::steady_clock::now() < deadline)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      CHECK(input.state() == singz::AudioInputState::Error);
      input.stop(false);
    }
    singz::setAudioInputBackendForTests(nullptr, {});
    capture = nullptr;
  }
}

void boundedRestoration() {
  using namespace singz::detail;
  CHECK(macAudioInputPreparation(32, 32, 512).frames == 512);
  CHECK(macAudioInputPreparation(32, 32, 32768).frames == 16384);
  for (const auto original : {512u, 32768u}) {
    Capture state;
    state.device.frames = original;
    MacAudioBufferOwnership monitor, input;
    CHECK(requestMacAudioBuffer(state.device, 32, 32768, monitor, 0, state.ledger).ok());
    // An explicitly undersized/retained owner must still block restoration.
    uint32_t capacity = 32;
    if (original == 512) {
      CHECK(requestMacAudioBuffer(state.device, 0, capacity, input, 0, state.ledger, false).ok());
    } else {
      CHECK(requestMacAudioBuffer(state.device, 0, capacity, input, 0, state.ledger, false,
          MacAudioInputBufferCapacity{capacity, [](uint32_t storage) { return storage; }}).ok());
      CHECK(capacity == 16384);
      CHECK(state.ledger.devices.at(state.device.uid).owners.at(input.token).preparedMaximum == capacity);
    }
    const auto writes = state.device.writes;
    CHECK(restoreMacAudioBuffer(state.device, monitor, state.ledger) == singz::AudioBufferWarningRestoreFailed);
    CHECK(state.device.frames == 32 && state.device.writes == writes);
    CHECK(!macAudioInputCallbackTooLarge(state.device.frames, capacity));
    CHECK(restoreMacAudioBuffer(state.device, input, state.ledger) == 0 && state.device.frames == original);
  }
}
void notificationAndRearm() {
  using namespace singz::detail;
  std::atomic<bool> stable{true};
  int aliveReads = 0, rateReads = 0, configReads = 0;
  const auto check = [&](uint32_t selector, bool alive, bool rate, bool config) {
    return macInputTakeControlFailure(macInputNotificationChanges(selector), stable,
        [&] { ++aliveReads; return alive; }, [&] { ++rateReads; return rate; },
        [&] { ++configReads; return config; });
  };
  CHECK(check(audioHostFourCc('f', 's', 'i', 'z'), false, false, false) == MacInputControlFailure::None);
  CHECK(!stable.load() && aliveReads == 0 && rateReads == 0 && configReads == 0);
  CHECK(check(audioHostFourCc('l', 'i', 'v', 'n'), false, true, true) == MacInputControlFailure::DeviceLost);
  CHECK(check(audioHostFourCc('n', 's', 'r', 't'), true, false, true) == MacInputControlFailure::RateChanged);
  for (const auto selector : {audioHostFourCc('s', 'l', 'a', 'y'), audioHostFourCc('s', 't', 'm', '#'),
                              audioHostFourCc('p', 'f', 't', ' '), audioHostFourCc('s', 'f', 'm', 't')}) {
    CHECK(check(selector, true, true, true) == MacInputControlFailure::None);
    CHECK(check(selector, true, true, false) == MacInputControlFailure::ConfigurationChanged);
  }
  CHECK(configReads == 8);
  auto opened = configuration();
  int reads = 0;
  CHECK(refreshMacAudioInputConfiguration(opened, [&](auto& facts) {
    facts = configuration(); return ++reads == 2;
  }));
  CHECK(reads == 2);
  reads = 0;
  CHECK(!refreshMacAudioInputConfiguration(opened, [&](auto&) { ++reads; return false; }));
  CHECK(reads == 2);
  CHECK(!refreshMacAudioInputConfiguration(opened, [&](auto& facts) {
    facts = configuration(3); return true;
  }));
  CHECK(opened == configuration()); // changed suspended route is never adopted

  Capture state;
  capture = &state;
  singz::setAudioInputBackendForTests(inputFactory, {{"input:shared", "Fixture", true, 48000, 1, {}}});
  {
    singz::AudioInput input;
    CHECK(input.start({"input:shared", 0, 4}, [](const auto&) {}).ok);
    input.stop(true);
    state.topology.store(3); // replaced stream while stopped; no notification bit
    CHECK(!input.start({"input:shared", 0, 4}, [](const auto&) {}).ok);
    CHECK(input.state() == singz::AudioInputState::Error);
    input.stop(false);
  }
  singz::setAudioInputBackendForTests(nullptr, {});
  capture = nullptr;
}
}  // namespace

int main() {
  Capture first;
  runningCapture(first, true, 512, 1, false);
  Capture second;
  runningCapture(second, false, 512, 1, false);
  sliceLimits();
  genuineFailures();
  boundedRestoration();
  notificationAndRearm();
  if (!failures) std::puts("PASS confirmed AUHAL capacity, input stream facts, notification/failure mapping and retained re-arm");
  return failures ? 1 : 0;
}
