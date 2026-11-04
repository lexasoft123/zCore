// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once
#include <cmath>
#include <bit>
#include <limits>
#include <cstdint>
#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

namespace zcore::routing::detail {
// AudioHistory stores the encoding directly in atomic integer slots. Classify
// the encoding once, preserving finite values without a CRT floating-point call.
inline uint32_t finiteSampleBits(float value) noexcept {
    static_assert(sizeof(float) == sizeof(uint32_t) && std::numeric_limits<float>::is_iec559);
    const auto bits = std::bit_cast<uint32_t>(value);
    return (bits & 0x7f800000U) == 0x7f800000U ? 0U : bits;
}
// In-place containment preserves every finite bit pattern, including signed
// zero and subnormals. No alignment requirement, allocation or shared state.
inline void sanitizeFinite(float* samples, uint32_t frames) noexcept {
    uint32_t frame = 0;
#if defined(__SSE2__) || defined(_M_X64)
    const auto magnitudeMask = _mm_set1_epi32(0x7fffffff);
    const auto infinityBits = _mm_set1_epi32(0x7f800000);
    for (; frames - frame >= 4; frame += 4) {
        const auto raw = _mm_loadu_ps(samples + frame);
        const auto magnitude = _mm_and_si128(_mm_castps_si128(raw), magnitudeMask);
        const auto finite = _mm_cmpgt_epi32(infinityBits, magnitude);
        _mm_storeu_ps(samples + frame, _mm_and_ps(raw, _mm_castsi128_ps(finite)));
    }
#endif
    for (; frame < frames; ++frame)
        if (!std::isfinite(samples[frame]))
            samples[frame] = 0;
}
} // namespace zcore::routing::detail
