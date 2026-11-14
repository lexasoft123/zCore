#pragma once

#include <algorithm>
#include <cmath>
#include <atomic>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <optional>
#include <vector>
#include <zcore/device/audio_buffer.h>
#include "audio_host_macos_helpers.h"

namespace singz::detail {

inline constexpr uint32_t kMacAudioInputMaximumFrames = 16384;
struct MacAudioInputPreparation {
  uint32_t frames;
  uint32_t warnings;
};
inline MacAudioInputPreparation macAudioInputPreparation(
    std::optional<uint32_t> reported, std::optional<uint32_t> deviceFrames = std::nullopt,
    uint32_t originalFrames = 0) {
  const bool missing = !reported || *reported == 0;
  const uint32_t slice = missing ? 4096 : *reported;
  if (slice > kMacAudioInputMaximumFrames) return {0, AudioBufferWarningNone};
  return {std::min(kMacAudioInputMaximumFrames, std::max({slice, deviceFrames.value_or(0), originalFrames})),
          missing ? AudioBufferWarningReadFailed : AudioBufferWarningNone};
}

inline uint32_t macAudioInputConfirmedCapacity(uint32_t storage,
                                              std::optional<uint32_t> confirmedSlice) {
  return std::min(storage, confirmedSlice.value_or(0));
}

// A rejected/rounded setting is advisory. Keep the last readable pre-init
// slice, but never claim that an unreadable post-init slice is confirmed.
template <class Set, class Read>
uint32_t prepareMacAudioInputUnitSlice(uint32_t storage,
                                     std::optional<uint32_t> initialSlice,
                                     uint32_t& warnings, Set set, Read read) {
  if (!initialSlice || *initialSlice == 0) {
    warnings |= AudioBufferWarningReadFailed;
    return 0;
  }
  auto confirmed = initialSlice;
  if (storage > *initialSlice) {
    if (!set(storage)) warnings |= AudioBufferWarningReadFailed;
    const auto observed = read();
    if (observed && *observed != 0) confirmed = observed;
    else warnings |= AudioBufferWarningReadFailed;
  }
  if (*confirmed < storage) warnings |= AudioBufferWarningReadFailed;
  return macAudioInputConfirmedCapacity(storage, confirmed);
}

// Invoked under the ledger mutex before registration; the unit preparation
// callback returns its confirmed render limit, independently of storage size.
template <class PrepareUnit>
struct MacAudioInputBufferCapacity {
  uint32_t& frames;
  PrepareUnit prepareUnit;
  uint32_t operator()(uint32_t prepared, uint32_t current, uint32_t original) const {
    frames = macAudioInputPreparation(prepared, current, original).frames;
    return prepareUnit(frames);
  }
};

constexpr bool macAudioInputCallbackTooLarge(uint32_t frames, std::size_t capacity) noexcept {
  return frames > capacity;
}

// Same identity/rate/layout/physical+virtual stream facts as the host, with
// only the input scope populated. Byte sizes and data pointers are excluded.
using MacAudioInputStreamConfiguration = MacAudioHostDeviceFacts;
inline bool macAudioInputStreamConfigurationChanged(
    const MacAudioInputStreamConfiguration& opened,
    const MacAudioInputStreamConfiguration& observed, bool readable) {
  return !readable || opened != observed;
}

template <class Read>
bool readMacAudioInputConfiguration(MacAudioInputStreamConfiguration& facts, Read read) {
  return read(facts) || read(facts);
}

// Re-arm must inspect even without a pending notification. Refresh only an
// unchanged baseline: accepting a changed one would reuse a stale channel map.
template <class Read>
bool refreshMacAudioInputConfiguration(MacAudioInputStreamConfiguration& opened, Read read) {
  MacAudioInputStreamConfiguration observed;
  const bool readable = readMacAudioInputConfiguration(observed, read);
  if (macAudioInputStreamConfigurationChanged(opened, observed, readable)) return false;
  opened = std::move(observed);
  return true;
}

// Retained resources keep their ledger lease until a real close. Merely
// owning a lease (including an unchanged device) must not disable fast re-arm.
template <class Stop>
bool suspendMacAudioInputPrepared(bool prepared, bool& started, Stop stop) {
  if (!prepared) return false;
  if (started && !stop()) return false;
  started = false;
  return true;
}

inline constexpr uint32_t kMacInputAliveChanged = 1u << 0;
inline constexpr uint32_t kMacInputRateChanged = 1u << 1;
inline constexpr uint32_t kMacInputConfigurationChanged = 1u << 2;
inline constexpr uint32_t kMacInputLatencyChanged = 1u << 3;

// Raw HAL selector values; the macOS glue static_asserts the SDK constants.
// This is the exact listener dispatch, also exercised without CoreAudio.
constexpr uint32_t macInputNotificationChanges(uint32_t selector) {
  if (selector == audioHostFourCc('l', 'i', 'v', 'n')) return kMacInputAliveChanged;
  if (selector == audioHostFourCc('n', 's', 'r', 't')) return kMacInputRateChanged;
  if (selector == audioHostFourCc('s', 'l', 'a', 'y') ||
      selector == audioHostFourCc('s', 't', 'm', '#') ||
      selector == audioHostFourCc('p', 'f', 't', ' ') ||
      selector == audioHostFourCc('s', 'f', 'm', 't'))
    return kMacInputConfigurationChanged | kMacInputLatencyChanged;
  return kMacInputLatencyChanged;
}

// Latency notifications are advisory, not route failures. Preserve the fatal
// alive/rate/channel bits for the existing lifecycle checks and invalidate the
// latency snapshot even if a notification does not change the reported value.
inline uint32_t macInputControlChanges(uint32_t changes, std::atomic<bool>& stable) {
  if (changes & kMacInputLatencyChanged) stable.store(false, std::memory_order_release);
  return changes & ~kMacInputLatencyChanged;
}

enum class MacInputControlFailure { None, DeviceLost, RateChanged, ConfigurationChanged };
template <class Alive, class Rate, class Configuration>
MacInputControlFailure macInputTakeControlFailure(uint32_t changes, std::atomic<bool>& stable,
                                                Alive alive, Rate rate, Configuration configuration) {
  changes = macInputControlChanges(changes, stable);
  if ((changes & kMacInputAliveChanged) && !alive()) return MacInputControlFailure::DeviceLost;
  if ((changes & kMacInputRateChanged) && !rate()) return MacInputControlFailure::RateChanged;
  if ((changes & kMacInputConfigurationChanged) && !configuration())
    return MacInputControlFailure::ConfigurationChanged;
  return MacInputControlFailure::None;
}

struct MacInputLatency {
  uint32_t frames = 0;
  uint32_t bufferFrames = 0;
  bool reported = false;
};

// Control-thread helper. Every term is required, including a reported zero.
// Device/safety properties are nominal-device-rate frames. Stream latency is
// in physical-stream-rate frames (normally the same rate). Public telemetry
// uses capture-rate frames. Timestamps already locate the first buffer sample,
// so the I/O buffer is reported separately, never included in placement delay.
inline MacInputLatency macInputLatency(
    std::optional<uint32_t> device, std::optional<uint32_t> safety,
    std::optional<uint32_t> stream, std::optional<uint32_t> buffer,
    double deviceRate, double captureRate, double streamRate) {
  if (!device || !safety || !stream || !buffer || *buffer == 0 ||
      !std::isfinite(deviceRate) || deviceRate <= 0 ||
      !std::isfinite(captureRate) || captureRate <= 0 ||
      !std::isfinite(streamRate) || streamRate <= 0) return {};
  const uint64_t deviceFrames = uint64_t{*device} + *safety;
  const long double ratio = static_cast<long double>(captureRate) / deviceRate;
  const long double streamRatio = static_cast<long double>(captureRate) / streamRate;
  const long double frames = std::round(deviceFrames * ratio + *stream * streamRatio);
  const long double bufferFrames = std::round(*buffer * ratio);
  if (!std::isfinite(frames) || !std::isfinite(bufferFrames) || frames < 0 ||
      frames > std::numeric_limits<uint32_t>::max() || bufferFrames < 1 ||
      bufferFrames > std::numeric_limits<uint32_t>::max()) return {};
  return {static_cast<uint32_t>(frames), static_cast<uint32_t>(bufferFrames), true};
}

}  // namespace singz::detail
