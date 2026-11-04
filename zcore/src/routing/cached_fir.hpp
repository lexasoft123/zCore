// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once
#include "resample_scratch.hpp"
#include "cached_fir_kernels.hpp"
#include <zcore/routing/audio_resampler.h>
#include <zcore/routing/sinc_kernel.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

namespace zcore::routing {
class CachedFir {
    static double dot(const float* samples, const double* weights, uint32_t count) noexcept {
        uint32_t tap = 0;
#if defined(__SSE2__) || defined(_M_X64)
        auto first = _mm_setzero_pd(), second = _mm_setzero_pd();
        for (; tap + 4 <= count; tap += 4) {
            const auto values = _mm_loadu_ps(samples + tap);
            first =
                _mm_add_pd(first, _mm_mul_pd(_mm_loadu_pd(weights + tap), _mm_cvtps_pd(values)));
            second = _mm_add_pd(second, _mm_mul_pd(_mm_loadu_pd(weights + tap + 2),
                                                   _mm_cvtps_pd(_mm_movehl_ps(values, values))));
        }
        std::array<double, 2> a{}, b{};
        _mm_storeu_pd(a.data(), first);
        _mm_storeu_pd(b.data(), second);
        double sum = (a[0] + a[1]) + (b[0] + b[1]);
#else
        std::array<double, 4> partial{};
        for (; tap + 4 <= count; tap += 4)
            for (uint32_t lane = 0; lane < 4; ++lane)
                partial[lane] += weights[tap + lane] * samples[tap + lane];
        double sum = (partial[0] + partial[1]) + (partial[2] + partial[3]);
#endif
        for (; tap < count; ++tap)
            sum += weights[tap] * samples[tap];
        return sum;
    }
    static std::array<double, 2> dotPair(const float* a, const float* b, const double* weights,
                                         uint32_t count) noexcept {
#if defined(__SSE2__) || defined(_M_X64)
        auto a0 = _mm_setzero_pd(), a1 = _mm_setzero_pd();
        auto b0 = _mm_setzero_pd(), b1 = _mm_setzero_pd();
        uint32_t tap = 0;
        for (; tap + 4 <= count; tap += 4) {
            const auto w0 = _mm_loadu_pd(weights + tap);
            const auto w1 = _mm_loadu_pd(weights + tap + 2);
            const auto av = _mm_loadu_ps(a + tap), bv = _mm_loadu_ps(b + tap);
            a0 = _mm_add_pd(a0, _mm_mul_pd(w0, _mm_cvtps_pd(av)));
            a1 = _mm_add_pd(a1, _mm_mul_pd(w1, _mm_cvtps_pd(_mm_movehl_ps(av, av))));
            b0 = _mm_add_pd(b0, _mm_mul_pd(w0, _mm_cvtps_pd(bv)));
            b1 = _mm_add_pd(b1, _mm_mul_pd(w1, _mm_cvtps_pd(_mm_movehl_ps(bv, bv))));
        }
        std::array<double, 2> x{}, y{}, z{}, w{};
        _mm_storeu_pd(x.data(), a0);
        _mm_storeu_pd(y.data(), a1);
        _mm_storeu_pd(z.data(), b0);
        _mm_storeu_pd(w.data(), b1);
        std::array<double, 2> sum{(x[0] + x[1]) + (y[0] + y[1]), (z[0] + z[1]) + (w[0] + w[1])};
        for (; tap < count; ++tap) {
            sum[0] += weights[tap] * a[tap];
            sum[1] += weights[tap] * b[tap];
        }
        return sum;
#else
        return {dot(a, weights, count), dot(b, weights, count)};
#endif
    }

  public:
    // One source lookup per frame across the whole block, then contiguous dot
    // products with double intermediates. Independent partial sums change only
    // rounding order. No ISA beyond mandatory x64 SSE2, no shared mutable state.
    template <unsigned capacity, unsigned channelCapacity, bool fixedPhase, class Provider>
    static ZCORE_ROUTING_READER_NOINLINE std::optional<uint32_t>
    readWindow(ResamplePosition read, ResampleOutput output, uint32_t frames,
               const SincKernel& kernel, double cutoff, Provider& provider,
               std::span<uint8_t> availableFrames) noexcept {
        if (frames < 128 || frames > 256 || output.channels.size() > channelCapacity)
            return std::nullopt;
        const auto radius = static_cast<uint32_t>(std::ceil(SincKernel::kRadius / cutoff));
        const auto taps = 2 * radius + 1;
        if (taps > capacity)
            return std::nullopt;
        struct Target {
            uint64_t center{};
            double fraction{};
        };
        std::array<Target, 256> targets{};
        uint64_t first = UINT64_MAX, last = 0;
        const auto step = read.sourceRate / read.outputRate;
        const bool sameRate = std::abs(read.sourceRate - read.outputRate) <= read.outputRate * 2e-9;
        for (uint32_t frame = 0; frame < frames; ++frame) {
            const auto offset = read.offset + static_cast<long double>(frame) * step;
            const auto whole = std::floor(offset);
            if (whole < -static_cast<long double>(read.anchor) ||
                whole >= static_cast<long double>(UINT64_MAX - read.anchor - 1))
                return std::nullopt;
            const auto center = whole < 0 ? read.anchor - static_cast<uint64_t>(-whole)
                                          : read.anchor + static_cast<uint64_t>(whole);
            if (center < radius || center > UINT64_MAX - radius)
                return std::nullopt;
            const auto fraction = static_cast<double>(offset - whole);
            // Near-integer unity reads use the existing exact sample-copy path.
            // Decline the complete block before reading its provider if any
            // frame needs that behavior; mixed direct/FIR blocks stay tiled.
            if (sameRate && (fraction < .0002 || fraction > .9998))
                return std::nullopt;
            targets[frame] = {center, fraction};
            if constexpr (fixedPhase)
                if (frame && std::abs(targets[frame].fraction - targets[0].fraction) > 1e-12)
                    return std::nullopt;
            first = std::min(first, center - radius);
            last = std::max(last, center + radius);
        }
        if (last - first >= capacity)
            return std::nullopt;
        const auto windowFrames = static_cast<uint32_t>(last - first + 1);
        const auto count = output.channels.size();
#if defined(LOOPZ_OPTIMIZED_FIR_DISPATCH)
        const auto pairKernel = detail::cachedFirPairKernel();
#else
        // Builds without an optimized native adapter retain the inline reader
        // without an atomic load or indirect call.
        constexpr detail::CachedFirPairKernel pairKernel = nullptr;
#endif
        // One cache line between planar rows distributes a wide-channel gather
        // across cache sets instead of repeatedly evicting power-of-two rows.
        // The readable window and FIR support remain bounded by capacity.
        constexpr auto rowPadding = 64 / sizeof(float);
        std::array<std::array<float, capacity + rowPadding>, channelCapacity> samples;
        std::array<bool, channelCapacity> nonzero{};
        std::array<ResampleFrame, capacity> present;
        std::array<float, kMaximumRoutingChannels> source{};
        bool allAvailable = true;
        for (uint32_t frame = 0; frame < windowFrames; ++frame) {
            present[frame] = provider(first + frame, source);
            allAvailable =
                allAvailable && present[frame].valid && present[frame].availableChannels >= count;
            for (size_t channel = 0; channel < count; ++channel) {
                samples[channel][frame] = present[frame].valid ? source[channel] : 0;
                if constexpr (fixedPhase)
                    nonzero[channel] = nonzero[channel] || samples[channel][frame] != 0;
            }
        }
        std::array<double, capacity> weights;
        double fixedNormalization = 0;
        if constexpr (fixedPhase)
            for (uint32_t tap = 0; tap < taps; ++tap) {
                weights[tap] =
                    kernel.coefficient(double(tap) - radius - targets[0].fraction, cutoff);
                fixedNormalization += weights[tap];
            }
        uint32_t missing = 0;
        for (uint32_t frame = 0; frame < frames; ++frame) {
            const auto& target = targets[frame];
            const auto begin = static_cast<uint32_t>(target.center - radius - first);
            double normalization = fixedPhase && allAvailable ? fixedNormalization : 0;
            bool complete = true, available = true;
            const auto scanCoverage = [&] {
                for (uint32_t tap = 0; tap < taps; ++tap) {
                    if constexpr (!fixedPhase)
                        weights[tap] =
                            kernel.coefficient(double(tap) - radius - target.fraction, cutoff);
                    const auto weight = weights[tap];
                    if (weight != 0) {
                        const auto value = present[begin + tap];
                        complete = complete && value.valid;
                        available = available && value.valid && value.availableChannels >= count;
                        normalization += weight;
                    }
                }
            };
            if constexpr (fixedPhase) {
                if (!allAvailable)
                    scanCoverage();
            } else {
                if (allAvailable) {
                    // Coverage is shared by the entire cached window. Avoid
                    // inspecting it again for every tap of every output frame.
                    for (uint32_t tap = 0; tap < taps; ++tap) {
                        weights[tap] =
                            kernel.coefficient(double(tap) - radius - target.fraction, cutoff);
                        normalization += weights[tap];
                    }
                } else {
                    scanCoverage();
                }
            }
            if (!complete || normalization < .5) {
                ++missing;
                continue;
            }
            if (!available)
                ++missing;
            else if (!availableFrames.empty())
                availableFrames[frame] = 1;
            size_t channel = 0;
            if constexpr (!fixedPhase) {
                for (; channel + 1 < count; channel += 2) {
                    const auto* a = samples[channel].data() + begin;
                    const auto* b = samples[channel + 1].data() + begin;
                    const auto pair = pairKernel ? pairKernel(a, b, weights.data(), taps)
                                                 : dotPair(a, b, weights.data(), taps);
                    for (size_t lane = 0; lane < 2; ++lane) {
                        const auto sample = pair[lane] / normalization;
                        output.channels[channel + lane][frame * output.stride] =
                            std::isfinite(sample) &&
                                    std::abs(sample) <= std::numeric_limits<float>::max()
                                ? static_cast<float>(sample)
                                : 0;
                    }
                }
            }
            for (; channel < count; ++channel) {
                const auto sample =
                    (!fixedPhase || nonzero[channel])
                        ? dot(samples[channel].data() + begin, weights.data(), taps) / normalization
                        : 0;
                output.channels[channel][frame * output.stride] =
                    std::isfinite(sample) && std::abs(sample) <= std::numeric_limits<float>::max()
                        ? static_cast<float>(sample)
                        : 0;
            }
        }
        return missing;
    }
    template <class Provider>
    static std::optional<uint32_t> read(ResamplePosition position, ResampleOutput output,
                                        uint32_t frames, const SincKernel& kernel, double cutoff,
                                        Provider& provider,
                                        std::span<uint8_t> availableFrames) noexcept {
        const auto step = position.sourceRate / position.outputRate;
        // Exact integer decimation keeps one filter phase throughout the block.
        // A separate bounded workspace covers the wider 2:1/4:1 support without
        // enlarging wide-channel readers or allocating on the audio thread.
        if (output.channels.size() <= 8 && (step == 2 || step == 4))
            return readWindow<2048, 8, true>(position, output, frames, kernel, cutoff, provider,
                                             availableFrames);
        if (output.channels.size() >= 16)
            return readWindow<512, kMaximumRoutingChannels, false>(
                position, output, frames, kernel, cutoff, provider, availableFrames);
        return std::nullopt;
    }
};
} // namespace zcore::routing
