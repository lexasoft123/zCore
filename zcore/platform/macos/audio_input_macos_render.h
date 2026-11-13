#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace singz::detail {

// AUHAL can briefly keep the old slice limit after device-buffer growth.
// Raw OSStatus keeps this policy portable; the macOS glue asserts the SDK value.
inline constexpr int32_t kMacInputTooManyFrames = -10874;
inline constexpr uint32_t kMacInputSliceDropLimit = 64;
inline constexpr uint64_t kMacInputSliceDropWindowNs = 250000000;
// Drop discards the failed render's signal, replacing it with equal-length
// silence at this callback's timestamps; it never omits time from the take.
enum class MacInputRenderDisposition { Deliver, Drop, Fatal };

// Callback-owned state. Reset before starting a take, while callbacks are stopped.
struct MacInputRenderRecovery {
  uint32_t consecutive = 0;
  uint64_t firstDropNs = 0;
};

// Pure status/time policy: no HAL access, allocation, locks or clock reads.
inline MacInputRenderDisposition classifyMacInputRender(
    int32_t status, uint64_t callbackNs, MacInputRenderRecovery& recovery) noexcept {
  if (status != kMacInputTooManyFrames) {
    recovery = {};
    return status == 0 ? MacInputRenderDisposition::Deliver : MacInputRenderDisposition::Fatal;
  }
  if (recovery.consecutive == 0) recovery.firstDropNs = callbackNs;
  // Saturate above the limit so even a caller continuing after Fatal cannot wrap.
  if (recovery.consecutive <= kMacInputSliceDropLimit) ++recovery.consecutive;
  const bool countExceeded = recovery.consecutive > kMacInputSliceDropLimit;
  const bool timeExceeded = callbackNs >= recovery.firstDropNs &&
      callbackNs - recovery.firstDropNs >= kMacInputSliceDropWindowNs;
  if (countExceeded || timeExceeded) return MacInputRenderDisposition::Fatal;
  return MacInputRenderDisposition::Drop;
}

// Exact production accounting seam. Only tolerated, silence-replaced blocks count;
// the terminal failure remains visible through the existing backend failure path.
inline MacInputRenderDisposition observeMacInputRender(
    int32_t status, uint64_t callbackNs, MacInputRenderRecovery& recovery,
    std::atomic<uint64_t>& renderSliceDrops) noexcept {
  const auto disposition = classifyMacInputRender(status, callbackNs, recovery);
  if (disposition == MacInputRenderDisposition::Drop)
    renderSliceDrops.fetch_add(1, std::memory_order_relaxed);
  return disposition;
}

// Caller has already bounded frames by prepared storage. Return whether to
// push the normal block: a tolerated failure must advance sequence/sourceFrame
// by frames, just like real signal, so concatenating consumers keep alignment.
inline bool prepareMacInputRenderBlock(MacInputRenderDisposition disposition,
                                      float* samples, uint32_t frames) noexcept {
  if (disposition == MacInputRenderDisposition::Fatal) return false;
  if (disposition == MacInputRenderDisposition::Drop)
    std::memset(samples, 0, static_cast<std::size_t>(frames) * sizeof(float));
  return true;
}

}  // namespace singz::detail
