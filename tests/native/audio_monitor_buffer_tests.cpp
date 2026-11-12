#include <zcore/device/audio_host.h>
#include <zcore/device/audio_input_backend.h>
#include "audio_device_macos_buffer.h"
#include "zcore/platform/macos/audio_host_macos_helpers.h"
#include "zcore/platform/macos/audio_input_macos_latency.h"

#include <cstdio>
#include <cstdlib>
#include <new>
#include <limits>
#include <utility>
#include <thread>

#if defined(__APPLE__) && !defined(SINGZ_PORTABLE_BUFFER_TESTS)
#include <TargetConditionals.h>
#endif

namespace {
std::atomic<bool> rejectStopAllocation{false};
std::atomic<uint32_t> stopAllocationAttempts{0};
}
void* operator new(std::size_t bytes) {
  if (rejectStopAllocation.load()) {
    stopAllocationAttempts.fetch_add(1);
    throw std::bad_alloc();
  }
  if (auto* value = std::malloc(bytes ? bytes : 1)) return value;
  throw std::bad_alloc();
}
void* operator new[](std::size_t bytes) { return ::operator new(bytes); }
void operator delete(void* value) noexcept { std::free(value); }
void operator delete[](void* value) noexcept { std::free(value); }
void operator delete(void* value, std::size_t) noexcept { std::free(value); }
void operator delete[](void* value, std::size_t) noexcept { std::free(value); }

namespace {
int failures = 0;
#define CHECK(condition) do { if (!(condition)) { std::fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); ++failures; } } while (false)

// Device property seam: no CoreAudio calls or physical audio in this suite.
struct Properties {
  uint32_t frames = 512, minimum = 64, maximum = 1024;
  int reads = 0, writes = 0, rangeReads = 0;
  int failReadAt = 0, failWriteAt = 0;
  uint32_t negotiatedOverride = 0;
  bool readableRange = true, settable = true;
  int identityChecks = 0, failIdentityAt = 0;
  std::string uid = "fake:buffer";
  uint64_t identity = 1;
  const std::string& deviceUid() const { return uid; }
  uint64_t deviceIdentity() const { return identity; }
  bool identityMatches() { return ++identityChecks != failIdentityAt; }
  bool isSettable() const { return settable; }
  bool readFrames(uint32_t& value) {
    if (++reads == failReadAt) return false;
    value = frames;
    return true;
  }
  bool readRange(uint32_t& low, uint32_t& high) {
    ++rangeReads;
    low = minimum; high = maximum;
    return readableRange;
  }
  bool writeFrames(uint32_t value) {
    if (++writes == failWriteAt) return false;
    frames = negotiatedOverride ? std::exchange(negotiatedOverride, 0u) : value;
    return true;
  }
};

class FakeHost : public singz::AudioHostBackend {
 public:
  int opens = 0, starts = 0, stops = 0;
  bool allowSharedChange = false;
  singz::AudioHostStatus current{};
  singz::AudioHostInventory enumerate() const override { return {}; }
  singz::AudioHostResult open(const singz::AudioHostConfig& config, singz::AudioHostRender,
                             void*) override {
    allowSharedChange = config.allowSharedDeviceBufferChange;
    ++opens; current.state = singz::AudioHostState::Open;
    current.routeGeneration = 5; current.streamGeneration = 8;
    current.format = {48000, config.maximumFrames, 512, 0, 2, true, true};
    current.latency.bufferFrames = 512;
    return result();
  }
  singz::AudioHostResult start() override {
    ++starts; current.state = singz::AudioHostState::Running; return result();
  }
  void stop() noexcept override { ++stops; current.state = singz::AudioHostState::Stopped; }
  singz::AudioHostStatus status() const noexcept override { return current; }
  singz::AudioHostResult result() const {
    return {true, singz::AudioHostError::None, current.state, current.format, current.latency, {}};
  }
};

class LiveHost final : public FakeHost {
 public:
  singz::AudioHostResult open(const singz::AudioHostConfig& config,
                             singz::AudioHostRender render, void* context) override {
    (void)FakeHost::open(config, render, context);
    if (config.requestedBufferFrames && !config.allowSharedDeviceBufferChange)
      return {false, singz::AudioHostError::Unsupported, current.state,
              current.format, current.latency, "Shared-device opt-in required"};
    const auto change = singz::detail::requestMacAudioBuffer(properties,
        config.requestedBufferFrames, config.maximumFrames, ownership,
        config.maximumAcceptableBufferFrames, ledger);
    if (change.ok()) {
      current.latency.bufferFrames = change.frames;
      current.format.nominalBufferFrames = change.frames;
    }
    return {change.ok(), change.error, current.state, current.format, current.latency, change.message};
  }
  void stop() noexcept override {
    current.bufferWarnings |= singz::detail::restoreMacAudioBuffer(properties, ownership, ledger);
    FakeHost::stop();
  }
  Properties properties;
  singz::detail::MacAudioBufferOwnership ownership;
  singz::detail::MacAudioBufferLedger ledger;
  std::atomic<uint32_t> callbackObservation{0};
  singz::AudioHostStatus status() const noexcept override {
    auto result = current;
    result.observedCallbackFrames = callbackObservation.load(std::memory_order_relaxed);
    return result;
  }
  void callbackFrames(uint32_t frames) {
    callbackObservation.store(frames, std::memory_order_relaxed);
  }
  singz::AudioHostResult setBufferFrames(std::uint32_t frames, std::uint32_t ceiling = 0) override {
    if (!allowSharedChange)
      return {false, singz::AudioHostError::Unsupported, current.state,
              current.format, current.latency, "Shared-device opt-in required"};
    const auto change = singz::detail::requestMacAudioBuffer(
        properties, frames, current.format.maximumFrames, ownership, ceiling, ledger);
    if (change.ok()) {
      current.latency.bufferFrames = change.frames;
      current.format.nominalBufferFrames = change.frames;
    }
    return {change.ok(), change.error, current.state, current.format, current.latency, change.message};
  }
};

int inputOpens = 0;
uint32_t inputRequest = 0;
bool inputAllowSharedChange = false;
bool inputRestoreWarning = false;
class FakeInput final : public singz::AudioInputBackend {
 public:
  bool started = false;
  singz::AudioInputResult open(const singz::AudioInputConfig& config,
                             singz::AudioInputPush, void*) override {
    ++inputOpens;
    inputRequest = config.requestedBufferFrames;
    inputAllowSharedChange = config.allowSharedDeviceBufferChange;
    return result(singz::AudioInputState::Starting);
  }
  singz::AudioInputResult start() override { started = true; return result(singz::AudioInputState::Running); }
  void stop() override {}
  bool suspendPrepared() override {
    return singz::detail::suspendMacAudioInputPrepared(true, started,
        [] { return true; });
  }
  bool takeFailure(std::string&) override { return false; }
  uint32_t bufferWarnings() const noexcept override {
    return inputRestoreWarning ? singz::AudioBufferWarningRestoreFailed : 0u;
  }
  singz::AudioInputResult result(singz::AudioInputState state) const {
    auto value = singz::AudioInputResult::success(state, 48000, 0);
    value.inputBufferFrames = inputRequest ? inputRequest : 512;
    return value;
  }
};
std::unique_ptr<singz::AudioInputBackend> fakeInput() { return std::make_unique<FakeInput>(); }

// AudioInput facade -> exact macOS control helpers -> property/stop seams.
// No hard-coded retention result: the same suspend helper as the AUHAL backend
// stops the fake unit and keeps its real ledger lease across a retained stop.
struct MacInputSeamState {
  Properties properties;
  singz::detail::MacAudioBufferLedger ledger;
  uint32_t reportedSlice = 128, preparedFrames = 0;
  int opens = 0, starts = 0, suspends = 0, closes = 0;
};
MacInputSeamState* macInputSeamState = nullptr;
class MacInputSeam final : public singz::AudioInputBackend {
 public:
  ~MacInputSeam() override { stop(); }
  singz::AudioInputResult open(const singz::AudioInputConfig& config,
                             singz::AudioInputPush, void*) override {
    ++macInputSeamState->opens;
    macInputSeamState->preparedFrames = singz::detail::macAudioInputPreparation(
        macInputSeamState->reportedSlice, macInputSeamState->properties.frames).frames;
    const auto change = singz::detail::requestMacAudioBuffer(
        macInputSeamState->properties, config.requestedBufferFrames, macInputSeamState->preparedFrames,
        ownership, config.maximumAcceptableBufferFrames, macInputSeamState->ledger, false);
    prepared = change.ok();
    return prepared ? result(singz::AudioInputState::Starting)
                    : singz::AudioInputResult::failure(singz::AudioInputState::Error, change.message, 0);
  }
  singz::AudioInputResult start() override {
    ++macInputSeamState->starts;
    started = true;
    return result(singz::AudioInputState::Running);
  }
  bool suspendPrepared() override {
    return singz::detail::suspendMacAudioInputPrepared(prepared, started, [] {
      ++macInputSeamState->suspends;
      return true;
    });
  }
  void stop() override {
    if (!prepared) return;
    ++macInputSeamState->closes;
    prepared = started = false;
    (void)singz::detail::restoreMacAudioBuffer(
        macInputSeamState->properties, ownership, macInputSeamState->ledger);
  }
  bool takeFailure(std::string&) override { return false; }
 private:
  singz::AudioInputResult result(singz::AudioInputState state) const {
    auto value = singz::AudioInputResult::success(state, 48000, 0);
    value.inputBufferFrames = macInputSeamState->properties.frames;
    return value;
  }
  bool prepared = false, started = false;
  singz::detail::MacAudioBufferOwnership ownership;
};
std::unique_ptr<singz::AudioInputBackend> macInputSeam() { return std::make_unique<MacInputSeam>(); }

void retainedMacInputApi() {
  MacInputSeamState state;
  macInputSeamState = &state;
  singz::setAudioInputBackendForTests(macInputSeam, {{"retain:mac", "Fixture", true, 48000, 1, {}}});
  {
    singz::AudioInput input;
    const singz::AudioInputConfig ordinary{"retain:mac", 0, 4};
    CHECK(input.start(ordinary, [](const auto&) {}).ok);
    CHECK(state.properties.frames > state.reportedSlice && state.preparedFrames == 512);
    CHECK(state.opens == 1 && state.properties.writes == 0 && state.ledger.devices.size() == 1);
    input.stop(true);
    CHECK(state.suspends == 1 && state.closes == 0 && state.ledger.devices.size() == 1);
    CHECK(input.start(ordinary, [](const auto&) {}).ok && state.opens == 1 && state.starts == 2);
    input.stop(false);
    CHECK(state.closes == 1 && state.ledger.devices.empty());
  }
  singz::setAudioInputBackendForTests(nullptr, {});
  macInputSeamState = nullptr;
}

void choices() {
  using singz::chooseMonitorBufferFrames;
  const singz::AudioHostBufferRange small{32, 1024}, medium{64, 512}, tooSlow{257, 1024};
  CHECK(chooseMonitorBufferFrames(small, small) == 32);
  CHECK(chooseMonitorBufferFrames(small, medium) == 64);
  CHECK(chooseMonitorBufferFrames(medium, small) == 64);
  CHECK(chooseMonitorBufferFrames(small, medium, 128) == 128);
  CHECK(chooseMonitorBufferFrames(small, medium, 1000) == 256);
  CHECK(chooseMonitorBufferFrames(small, {32, 128}, 256) == 128);
  CHECK(chooseMonitorBufferFrames({256, 1024}, small) == 256);
  // Negative fixture: returning target=32 unconditionally must fail this suite.
  CHECK(!chooseMonitorBufferFrames(tooSlow, small));
  CHECK(!chooseMonitorBufferFrames(small, tooSlow));
  CHECK(!chooseMonitorBufferFrames({129, 256}, {32, 128}));
  CHECK(!chooseMonitorBufferFrames({}, small));
  CHECK(!chooseMonitorBufferFrames({64, 32}, small));
  CHECK(!chooseMonitorBufferFrames(small, small, 32, 0));
  CHECK(chooseMonitorBufferFrames(small, small, 0) == 32);
  CHECK(chooseMonitorBufferFrames({UINT32_MAX, UINT32_MAX},
                                {UINT32_MAX, UINT32_MAX}, UINT32_MAX, UINT32_MAX) == UINT32_MAX);
}

void transactions() {
  using singz::detail::MacAudioBufferLedger;
  using singz::detail::MacAudioBufferOwnership;
  using singz::detail::requestMacAudioBuffer;
  using singz::detail::restoreMacAudioBuffer;
  MacAudioBufferLedger ledger;
  Properties p;
  MacAudioBufferOwnership owner;
  const auto request = [&](uint32_t desired, uint32_t cap = 0) {
    return requestMacAudioBuffer(p, desired, 1024, owner, cap, ledger);
  };
  CHECK(request(0).frames == 512 && p.writes == 0 && owner.token != 0);
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0 && p.writes == 0);
  CHECK(request(512).ok() && p.writes == 0);
  const auto small = request(32, 256);
  CHECK(small.ok() && small.frames == 64 && !small.message.empty() && p.frames == 64);
  CHECK(request(256).frames == 256);
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0 && p.frames == 512 && owner.token == 0);

  // Request-time driver quantization is accepted only within EVERY active
  // capacity/ceiling. Return the observed size; request-echo must fail.
  p.negotiatedOverride = 128;
  auto negotiated = request(64);
  CHECK(negotiated.ok() && negotiated.frames == 128 && p.frames == 128 && !negotiated.message.empty());
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0 && p.frames == 512);
  p.minimum = 32;
  p.negotiatedOverride = 64;
  negotiated = request(32, 256);
  CHECK(negotiated.ok() && negotiated.frames == 64 && p.frames == 64);
  CHECK(request(128, 256).ok() && p.frames == 128); // quantization never strands later changes
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0 && p.frames == 512);
  p.minimum = 64;
  p.negotiatedOverride = 1024;
  const auto beforeRollbackWrites = p.writes;
  const auto overBudget = request(32, 256);
  CHECK(!overBudget.ok() && overBudget.message.find("maximum acceptable") != std::string::npos);
  CHECK(p.frames == 512 && p.writes == beforeRollbackWrites + 2);
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0);
  p.minimum = 512;
  const auto writesBefore = p.writes;
  CHECK(!request(32, 256).ok() && p.writes == writesBefore && owner.token == 0);
  p.minimum = 64;
  p.failReadAt = p.reads + 3; // initial snapshot, expected check, then read-back
  CHECK(!request(64).ok() && p.frames == 512);
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0);
  p.failReadAt = p.reads + 3;
  p.failWriteAt = p.writes + 2; // mutation succeeds; immediate rollback fails
  CHECK(!request(64).ok() && p.frames == 64 && owner.token != 0);
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == 0 && p.frames == 512);
  p.failReadAt = p.failWriteAt = 0;

  // Missing current/range, unwritable property, identity changes before/after
  // write, and rejected writes all fail conservatively without blind restore.
  for (int failure = 0; failure != 7; ++failure) {
    Properties broken;
    MacAudioBufferOwnership lease;
    if (failure == 0) broken.failReadAt = 1;
    if (failure == 1) broken.readableRange = false;
    if (failure == 2) broken.minimum = 2048;
    if (failure == 3) broken.failWriteAt = 1;
    if (failure == 4) broken.settable = false;
    if (failure == 5) broken.failIdentityAt = 3;
    if (failure == 6) broken.failIdentityAt = 4;
    CHECK(!requestMacAudioBuffer(broken, 32, 1024, lease, 0, ledger).ok());
    (void)restoreMacAudioBuffer(broken, lease, ledger);
    CHECK(broken.frames == 512);
  }
  // The expected-value check itself is necessary even without ledger callers.
  Properties stale;
  stale.frames = 128;
  CHECK(!singz::detail::writeMacAudioBuffer(stale, 64, 32).wrote && stale.frames == 128 && stale.writes == 0);

  // A rounded result exceeding a RUNNING song's capacity must roll back,
  // even if the requester itself prepared more memory.
  p.frames = 512;
  MacAudioBufferOwnership runningSong, monitor;
  CHECK(requestMacAudioBuffer(p, 0, 512, runningSong, 0, ledger).ok());
  p.negotiatedOverride = 1024;
  CHECK(!requestMacAudioBuffer(p, 32, 2048, monitor, 0, ledger).ok());
  CHECK(p.frames == 512);
  CHECK(restoreMacAudioBuffer(p, monitor, ledger) == 0);
  p.negotiatedOverride = 128;
  CHECK(requestMacAudioBuffer(p, 32, 2048, monitor, 256, ledger).frames == 128);
  CHECK(p.frames == 128 && restoreMacAudioBuffer(p, monitor, ledger) == 0 && p.frames == 128);
  CHECK(restoreMacAudioBuffer(p, runningSong, ledger) == 0 && p.frames == 512);

  // Restoration failure is an advisory flag. Third-party clobber is preserved.
  CHECK(request(64).ok());
  p.failWriteAt = p.writes + 1;
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == singz::AudioBufferWarningRestoreFailed);
  p.frames = 512; p.failWriteAt = 0;
  CHECK(request(64).ok());
  p.frames = 128;
  CHECK(!request(256).ok() && p.frames == 128); // stale expected value, no write
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == singz::AudioBufferWarningExternalChange);
  CHECK(p.frames == 128 && ledger.devices.empty());
  p.frames = 512;
  CHECK(request(64).ok());
  p.frames = 512; // LoopZ treats an already-restored original as done.
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == singz::AudioBufferWarningNone);

  // Both stop orders must restore the true original, not the other backend's
  // opening value. An unchanged song host still holds a restoration lease.
  for (bool inputFirst : {false, true}) {
    p.frames = 512;
    MacAudioBufferOwnership input, output;
    CHECK(requestMacAudioBuffer(p, 32, 1024, input, 256, ledger).frames == 64);
    CHECK(requestMacAudioBuffer(p, 128, 1024, output, 256, ledger).frames == 128);
    auto& first = inputFirst ? input : output;
    auto& last = inputFirst ? output : input;
    CHECK(restoreMacAudioBuffer(p, first, ledger) == 0 && p.frames == 128);
    CHECK(restoreMacAudioBuffer(p, last, ledger) == 0 && p.frames == 512);
  }
  p.frames = 512;
  MacAudioBufferOwnership input, song, tooSmall;
  CHECK(requestMacAudioBuffer(p, 32, 1024, input, 256, ledger).ok());
  CHECK(!requestMacAudioBuffer(p, 0, 256, tooSmall, 0, ledger).ok());
  CHECK(tooSmall.token == 0); // reject a host unable to fit the ORIGINAL 512
  CHECK(requestMacAudioBuffer(p, 0, 512, song, 0, ledger).ok());
  CHECK(!requestMacAudioBuffer(p, 512, 512, song, 0, ledger).ok() && p.frames == 64);
  CHECK(restoreMacAudioBuffer(p, input, ledger) == 0 && p.frames == 64);
  CHECK(p.frames <= 512); // monitoring off cannot silence the running song
  CHECK(!requestMacAudioBuffer(p, 1024, 1024, owner, 0, ledger).ok());
  CHECK(requestMacAudioBuffer(p, 512, 512, song, 0, ledger).ok() && p.frames == 512);
  CHECK(restoreMacAudioBuffer(p, song, ledger) == 0 && p.frames == 512);
  CHECK(ledger.devices.empty());

  // Reusing a UID for a different AudioObjectID must not inherit an old
  // device's restoration baseline through another live backend's lease.
  CHECK(request(64).ok());
  Properties replacement;
  replacement.identity = 2; replacement.frames = 256;
  MacAudioBufferOwnership newDevice;
  CHECK(!requestMacAudioBuffer(replacement, 128, 1024, newDevice, 0, ledger).ok());
  p.failIdentityAt = p.identityChecks + 1;
  CHECK(restoreMacAudioBuffer(p, owner, ledger) == singz::AudioBufferWarningRestoreFailed);
  CHECK(requestMacAudioBuffer(replacement, 128, 1024, newDevice, 0, ledger).ok());
  CHECK(restoreMacAudioBuffer(replacement, newDevice, ledger) == 0 && replacement.frames == 256);
  p.frames = 512;

  // Retained input keeps its lease and prepared unit. An unchanged lease must
  // not disable re-arm; a smaller input capacity is safe until REAL close.
  MacAudioBufferOwnership retainedInput, changer;
  CHECK(requestMacAudioBuffer(p, 32, 256, retainedInput, 256, ledger, false).ok() && p.frames == 64);
  CHECK(restoreMacAudioBuffer(p, retainedInput, ledger) == 0 && p.frames == 512);
  CHECK(requestMacAudioBuffer(p, 64, 1024, changer, 256, ledger).ok());
  CHECK(requestMacAudioBuffer(p, 0, 64, retainedInput, 0, ledger, false).ok());
  bool started = true;
  int stops = 0;
  CHECK(singz::detail::suspendMacAudioInputPrepared(true, started, [&] { ++stops; return true; }));
  CHECK(!started && stops == 1 && retainedInput.token != 0);
  CHECK(restoreMacAudioBuffer(p, changer, ledger) == 0 && p.frames == 64);
  CHECK(singz::detail::suspendMacAudioInputPrepared(true, started, [&] { ++stops; return true; }));
  CHECK(stops == 1); // idempotent suspension, no real close or lease release
  CHECK(restoreMacAudioBuffer(p, retainedInput, ledger) == 0 && p.frames == 512);
  const auto ordinary = singz::detail::macAudioInputPreparation(4096);
  CHECK(ordinary.frames == 4096 && ordinary.warnings == 0);
  const auto degraded = singz::detail::macAudioInputPreparation(std::nullopt);
  CHECK(degraded.frames == 4096 && degraded.warnings == singz::AudioBufferWarningReadFailed);
  CHECK(singz::detail::macAudioInputPreparation(32768).frames == 0);
  CHECK(singz::detail::macAudioInputPreparation(128, 8192).frames == 8192);
  CHECK(singz::detail::macAudioInputPreparation(128, 32768).frames == 16384);
  // Even a current property beyond the bounded render storage must not reject
  // unchanged ordinary capture. Its lease only constrains future writes.
  p.frames = 32768;
  const auto ordinaryWrites = p.writes;
  CHECK(requestMacAudioBuffer(p, 0, 16384, retainedInput, 0, ledger, false).ok());
  CHECK(p.writes == ordinaryWrites && retainedInput.token != 0);
  CHECK(restoreMacAudioBuffer(p, retainedInput, ledger) == 0 && p.frames == 32768);

  // Missing ordinary-input metadata retains its capacity reservation. The
  // first known read initializes original before any write; growth is bounded.
  p.frames = 64;
  p.failReadAt = p.reads + 1;
  CHECK(requestMacAudioBuffer(p, 0, 256, retainedInput, 0, ledger, false).ok());
  CHECK(retainedInput.token != 0 && ledger.devices.begin()->second.original == 0);
  CHECK(!requestMacAudioBuffer(p, 512, 1024, changer, 0, ledger).ok() && p.frames == 64);
  CHECK(requestMacAudioBuffer(p, 128, 1024, changer, 0, ledger).ok() && p.frames == 128);
  CHECK(restoreMacAudioBuffer(p, changer, ledger) == 0 && p.frames == 128);
  CHECK(restoreMacAudioBuffer(p, retainedInput, ledger) == 0 && p.frames == 64);
  p.frames = 512;

  std::atomic<bool> concurrentOk{true};
  const auto client = [&](uint32_t desired) {
    for (int run = 0; run < 100; ++run) {
      MacAudioBufferOwnership lease;
      if (!requestMacAudioBuffer(p, desired, 1024, lease, 256, ledger).ok())
        concurrentOk.store(false);
      if (restoreMacAudioBuffer(p, lease, ledger) != 0) concurrentOk.store(false);
    }
  };
  std::thread capture(client, 64), playback(client, 128);
  capture.join(); playback.join();
  CHECK(concurrentOk.load() && p.frames == 512 && ledger.devices.empty());
}

void hostApi() {
  {
    auto duplex = std::make_unique<LiveHost>();
    auto* fresh = duplex.get();
    singz::AudioHost monitor(std::move(duplex));
    singz::AudioHostConfig request;
    request.requestedBufferFrames = 32;
    CHECK(!monitor.open(request, nullptr, nullptr).ok && fresh->properties.writes == 0);
    request.allowSharedDeviceBufferChange = true;
    request.maximumAcceptableBufferFrames = 256;
    CHECK(monitor.open(request, nullptr, nullptr).latency.bufferFrames == 64 && monitor.start().ok);
    CHECK(monitor.status().observedCallbackFrames == 0);
    fresh->callbackFrames(64);
    CHECK(monitor.status().observedCallbackFrames == 64);
  }
  auto backend = std::make_unique<LiveHost>();
  auto* fake = backend.get();
  singz::AudioHost host(std::move(backend));
  singz::AudioHostConfig config;
  CHECK(!config.allowSharedDeviceBufferChange);
  config.allowSharedDeviceBufferChange = true;
  CHECK(host.open(config, nullptr, nullptr).ok && host.start().ok);
  const auto before = host.status();
  auto changed = host.setBufferFrames(32);
  CHECK(changed.ok && changed.latency.bufferFrames == 64 && changed.format.nominalBufferFrames == 64);
  CHECK(changed.state == singz::AudioHostState::Running);
  fake->callbackFrames(512); // negotiated property is 64; provider may retain old callbacks
  const auto retained = host.status();
  CHECK(retained.observedCallbackFrames == 512 && retained.format.nominalBufferFrames == 64);
  CHECK(retained.latency.bufferFrames == 64 && retained.state == singz::AudioHostState::Running);
  CHECK(fake->opens == 1 && fake->starts == 1 && fake->stops == 0);
  fake->callbackFrames(64);
  CHECK(host.status().observedCallbackFrames == 64);
  CHECK(host.status().format.nominalBufferFrames == 64);
  CHECK(host.setBufferFrames(128).latency.bufferFrames == 128);
  CHECK(fake->opens == 1 && fake->starts == 1 && fake->stops == 0);
  CHECK(host.status().routeGeneration == before.routeGeneration &&
        host.status().streamGeneration == before.streamGeneration);
  auto unsupportedBackend = std::make_unique<FakeHost>();
  auto* unsupported = unsupportedBackend.get();
  singz::AudioHost other(std::move(unsupportedBackend));
  CHECK(other.open({}, nullptr, nullptr).ok && other.start().ok);
  auto rejected = other.setBufferFrames(64);
  CHECK(!rejected.ok && rejected.error == singz::AudioHostError::Unsupported &&
        rejected.state == singz::AudioHostState::Running && rejected.latency.bufferFrames == 512);
  CHECK(unsupported->stops == 0 && unsupported->opens == 1);
  auto noOptIn = std::make_unique<LiveHost>();
  auto* guarded = noOptIn.get();
  singz::AudioHost guardedHost(std::move(noOptIn));
  CHECK(guardedHost.open({}, nullptr, nullptr).ok && guardedHost.start().ok);
  CHECK(guardedHost.setBufferFrames(32).error == singz::AudioHostError::Unsupported);
  CHECK(guarded->properties.writes == 0 && guarded->stops == 0);
  fake->properties.failWriteAt = fake->properties.writes + 1;
  host.stop();
  CHECK(host.status().state == singz::AudioHostState::Stopped &&
        host.status().bufferWarnings == singz::AudioBufferWarningRestoreFailed);
  singz::AudioHost absent{std::unique_ptr<singz::AudioHostBackend>{}};
  CHECK(absent.setBufferFrames(64).error == singz::AudioHostError::Unsupported);
  CHECK(!singz::detail::macAudioHostBufferExceedsMaximum(256, 512));
  CHECK(singz::detail::macAudioHostBufferExceedsMaximum(1024, 512));
  CHECK(singz::detail::macAudioHostControlChange(false, false, false) == 0); // Buffer/other.
  CHECK(singz::detail::macAudioHostControlChange(true, false, false) == singz::detail::kMacAudioHostAliveChanged);
  CHECK(singz::detail::macAudioHostControlChange(false, true, false) == singz::detail::kMacAudioHostFormatChanged);
  CHECK(singz::detail::macAudioHostControlChange(false, false, true) == singz::detail::kMacAudioHostFormatChanged);
}

void inputApi() {
  CHECK(singz::AudioInputConfig{}.requestedBufferFrames == 0);
  CHECK(!singz::AudioInputConfig{}.allowSharedDeviceBufferChange);
#if defined(__APPLE__) && !defined(SINGZ_PORTABLE_BUFFER_TESTS) && TARGET_OS_OSX
  // Refuse the opt-out before device resolution, permissions, or any property
  // write. This test does not open audio hardware on the macOS test runner.
  {
    auto platform = singz::createPlatformAudioInputBackend();
    rejectStopAllocation.store(true);
    platform->stop(); // no empty-ledger creation, no stop-time heap allocation
    rejectStopAllocation.store(false);
    CHECK(stopAllocationAttempts.load() == 0);
    singz::AudioInputConfig denied{"not-a-real-device", 0, 4, 32};
    const auto result = platform->open(denied, nullptr, nullptr);
    CHECK(!result.ok && result.state == singz::AudioInputState::Unsupported);
    CHECK(result.error.find("allowSharedDeviceBufferChange") != std::string::npos);
    auto output = singz::createPlatformAudioHostBackend();
    rejectStopAllocation.store(true);
    output->stop();
    rejectStopAllocation.store(false);
    CHECK(stopAllocationAttempts.load() == 0);
    singz::AudioHostConfig deniedOutput;
    deniedOutput.requestedBufferFrames = 32;
    const auto outputDenied = output->open(deniedOutput, nullptr, nullptr);
    CHECK(!outputDenied.ok && outputDenied.error == singz::AudioHostError::Unsupported);
    CHECK(outputDenied.message.find("allowSharedDeviceBufferChange") != std::string::npos);
    output->stop();
    std::thread reader([&] { for (int i = 0; i < 1000; ++i) (void)output->status(); });
    for (int i = 0; i < 1000; ++i) {
      CHECK(output->setBufferFrames(32, 256).error == singz::AudioHostError::InvalidState);
      output->stop();
    }
    reader.join(); // status/lifecycle snapshot check under TSan, no device open
  }
#endif
  singz::setAudioInputBackendForTests(fakeInput, {{"buffer:fake", "Fixture", true, 48000, 1, {}}});
  {
    singz::AudioInput input;
    singz::AudioInputConfig config{"buffer:fake", 0, 4};
    config.requestedBufferFrames = 64;
    config.allowSharedDeviceBufferChange = true;
    CHECK(input.start(config, [](const auto&) {}).inputBufferFrames == 64);
    CHECK(inputRequest == 64 && inputAllowSharedChange && inputOpens == 1);
    input.stop(true);
    CHECK(input.start(config, [](const auto&) {}).ok && inputOpens == 1);
    input.stop(true);
    config.requestedBufferFrames = 128;
    CHECK(input.start(config, [](const auto&) {}).inputBufferFrames == 128);
    CHECK(inputRequest == 128 && inputOpens == 2); // A changed request cannot reuse stale preparation.
    input.stop(true);
    config.allowSharedDeviceBufferChange = false;
    CHECK(input.start(config, [](const auto&) {}).ok && inputOpens == 3);
    CHECK(!inputAllowSharedChange);
    input.stop(); // Policy changes also invalidate retained preparation.
  }
  inputRestoreWarning = true;
  {
    singz::AudioInput input;
    CHECK(input.start({"buffer:fake", 0, 4}, [](const auto&) {}).ok);
    CHECK(input.stats().bufferWarnings == singz::AudioBufferWarningRestoreFailed);
    input.stop();
    CHECK(input.state() == singz::AudioInputState::Stopped && input.lastError().empty());
    CHECK(input.stats().bufferWarnings == singz::AudioBufferWarningRestoreFailed);
  }
  inputRestoreWarning = false;
  singz::setAudioInputBackendForTests(nullptr, {});
}
}  // namespace

int main() {
  choices(); transactions(); hostApi(); inputApi(); retainedMacInputApi();
  if (!failures) std::puts("PASS monitor buffer selection, property transactions and fake backend APIs");
  return failures ? 1 : 0;
}
