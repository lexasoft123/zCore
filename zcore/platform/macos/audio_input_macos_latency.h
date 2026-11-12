#pragma once

#include <cmath>
#include <atomic>
#include <cstdint>
#include <limits>
#include <optional>

namespace singz::detail {

inline constexpr uint32_t kMacInputLatencyChanged = 1u << 3;

// Latency notifications are advisory, not route failures. Preserve the fatal
// alive/rate/channel bits for the existing lifecycle checks and invalidate the
// latency snapshot even if a notification does not change the reported value.
inline uint32_t macInputControlChanges(uint32_t changes, std::atomic<bool>& stable) {
  if (changes & kMacInputLatencyChanged) stable.store(false, std::memory_order_release);
  return changes & ~kMacInputLatencyChanged;
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
