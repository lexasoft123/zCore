// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once
#include "resample_scratch.hpp"

#include <zcore/routing/audio_resampler.h>
#include <zcore/routing/sinc_kernel.h>
#include <array>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <limits>
#include <bit>
#if defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

namespace zcore::routing {
// Immutable control-prepared FFT plan. A read owns bounded scratch on its user
// thread's stack (under 160 KiB); independent readers share neither samples nor
// transform state. This is user-mode DSP, never part of the audio driver.
class UnityConvolutionPlan {
  public:
    static constexpr uint32_t kSourceRadius =
        static_cast<uint32_t>(SincKernel::kRadius / SincKernel::kCutoff) + 1;
    static constexpr uint32_t kMaximumFrames = 256;

  private:
    struct Complex {
        double real{}, imaginary{};
    };
    static constexpr uint32_t size = 512;
    std::array<Complex, size> roots_{};
    // Forward DIF and inverse DIT share bit-reversed spectra. Convolution
    // multiplies corresponding bins directly, so neither transform needs a
    // permutation pass. Both retain double intermediates and exact FIR support.
    template <bool inverse> void transform(std::array<Complex, size>& values) const noexcept {
        // Fuse adjacent radix-2 stages. Three complex twiddle products replace
        // four, and each group is written once. The forward transform returns
        // bit-reversed bins, which the inverse accepts directly.
#if defined(__SSE2__) || defined(_M_X64)
        // Keep each complex pair packed throughout the butterfly; converting
        // between scalar structs at every operation defeats MSVC vector codegen.
        using Value = __m128d;
        const auto load = [&](size_t index) {
            return std::bit_cast<Value>(values[index]);
        };
        const auto store = [&](size_t index, Value value) {
            values[index] = std::bit_cast<Complex>(value);
        };
        const auto add = [](Value a, Value b) {
            return _mm_add_pd(a, b);
        };
        const auto subtract = [](Value a, Value b) {
            return _mm_sub_pd(a, b);
        };
        const auto rotate = [](Value a) {
            return _mm_xor_pd(_mm_shuffle_pd(a, a, 1),
                              inverse ? _mm_set_pd(0.0, -0.0) : _mm_set_pd(-0.0, 0.0));
        };
        const auto multiply = [&](Value a, uint32_t index) {
            const auto root = roots_[index];
            const auto imaginary = inverse ? -root.imaginary : root.imaginary;
            return _mm_add_pd(
                _mm_mul_pd(a, _mm_set1_pd(root.real)),
                _mm_mul_pd(_mm_shuffle_pd(a, a, 1), _mm_set_pd(imaginary, -imaginary)));
        };
#else
        const auto load = [&](size_t index) {
            return values[index];
        };
        const auto store = [&](size_t index, Complex value) {
            values[index] = value;
        };
        const auto add = [](Complex a, Complex b) {
            return Complex{a.real + b.real, a.imaginary + b.imaginary};
        };
        const auto subtract = [](Complex a, Complex b) {
            return Complex{a.real - b.real, a.imaginary - b.imaginary};
        };
        const auto rotate = [](Complex a) {
            return inverse ? Complex{-a.imaginary, a.real} : Complex{a.imaginary, -a.real};
        };
        const auto multiply = [&](Complex a, uint32_t index) {
            const auto root = roots_[index];
            const auto imaginary = inverse ? -root.imaginary : root.imaginary;
            return Complex{a.real * root.real - a.imaginary * imaginary,
                           a.real * imaginary + a.imaginary * root.real};
        };
#endif
        if constexpr (!inverse) {
            // The 512-point forward transform starts with its unpaired stage.
            for (uint32_t index = 0; index < size / 2; ++index) {
                const auto a = load(index), b = load(index + size / 2);
                store(index, add(a, b));
                store(index + size / 2, multiply(subtract(a, b), index));
            }
            for (uint32_t length = size / 4; length >= 4; length /= 4) {
                const auto half = length / 2, stride = size / (2 * length);
                for (uint32_t begin = 0; begin < size; begin += 2 * length)
                    for (uint32_t index = 0; index < half; ++index) {
                        const auto a = load(begin + index);
                        const auto b = load(begin + index + half);
                        const auto c = load(begin + index + length);
                        const auto d = load(begin + index + length + half);
                        const auto left = add(a, c), right = add(b, d);
                        const auto difference = subtract(a, c), cross = rotate(subtract(b, d));
                        store(begin + index, add(left, right));
                        store(begin + index + half,
                              multiply(subtract(left, right), index * 2 * stride));
                        store(begin + index + length,
                              multiply(add(difference, cross), index * stride));
                        store(begin + index + length + half,
                              multiply(subtract(difference, cross), index * 3 * stride));
                    }
            }
            for (uint32_t begin = 0; begin < size; begin += 4) {
                const auto a = load(begin), b = load(begin + 1), c = load(begin + 2),
                           d = load(begin + 3);
                const auto left = add(a, c), right = add(b, d);
                const auto difference = subtract(a, c), cross = rotate(subtract(b, d));
                store(begin, add(left, right));
                store(begin + 1, subtract(left, right));
                store(begin + 2, add(difference, cross));
                store(begin + 3, subtract(difference, cross));
            }
            return;
        }
        for (uint32_t begin = 0; begin < size; begin += 4) {
            const auto a = load(begin), b = load(begin + 1), c = load(begin + 2),
                       d = load(begin + 3);
            const auto left = add(a, b), right = add(c, d);
            const auto difference = subtract(a, b), cross = rotate(subtract(c, d));
            store(begin, add(left, right));
            store(begin + 1, add(difference, cross));
            store(begin + 2, subtract(left, right));
            store(begin + 3, subtract(difference, cross));
        }
        for (uint32_t length = 8; length * 2 <= size; length *= 4) {
            const auto half = length / 2, stride = size / (2 * length);
            for (uint32_t begin = 0; begin < size; begin += 2 * length)
                for (uint32_t index = 0; index < half; ++index) {
                    const auto a = load(begin + index);
                    const auto b = multiply(load(begin + index + half), index * 2 * stride);
                    const auto c = multiply(load(begin + index + length), index * stride);
                    const auto d =
                        multiply(load(begin + index + length + half), index * 3 * stride);
                    const auto left = add(a, b), right = add(c, d);
                    const auto difference = subtract(a, b), cross = rotate(subtract(c, d));
                    store(begin + index, add(left, right));
                    store(begin + index + half, add(difference, cross));
                    store(begin + index + length, subtract(left, right));
                    store(begin + index + length + half, subtract(difference, cross));
                }
        }
        // 512 points have one unpaired final stage.
        for (uint32_t index = 0; index < size / 2; ++index) {
            const auto a = load(index), b = multiply(load(index + size / 2), index);
            store(index, add(a, b));
            store(index + size / 2, subtract(a, b));
        }
    }

    UnityConvolutionPlan() {
        for (uint32_t index = 0; index < roots_.size(); ++index) {
            const auto angle = -2 * std::numbers::pi_v<double> * index / size;
            roots_[index] = {std::cos(angle), std::sin(angle)};
        }
    }

  public:
    static const UnityConvolutionPlan& prepared() {
        static const UnityConvolutionPlan plan;
        return plan;
    }
    // Caller proves exact unity step, constant fractional phase, 128–256 output
    // frames and representable center +/- support. All other shapes use FIR.
    template <class Provider>
    ZCORE_ROUTING_READER_NOINLINE uint32_t read(uint64_t center, double fraction, ResampleOutput output,
                                        uint32_t frames, const SincKernel& kernel,
                                        Provider& provider,
                                        std::span<uint8_t> availableFrames) const noexcept {
        constexpr auto radius = kSourceRadius;
        constexpr auto taps = 2 * radius + 1;
        static_assert(256 + 2 * radius <= size);
        const auto windowFrames = frames + 2 * radius;
        const auto count = output.channels.size();
        std::array<std::array<float, kMaximumRoutingChannels>, size> samples;
        std::array<ResampleFrame, size> present;
        std::array<double, kMaximumRoutingChannels> peaks{};
        std::array<double, taps> weights{};
        std::array<Complex, size> spectrum{};
        double normalization = 0;
        for (uint32_t tap = 0; tap < taps; ++tap) {
            const auto weight = kernel.coefficient(static_cast<double>(tap) - radius - fraction,
                                                    SincKernel::kCutoff);
            weights[tap] = weight;
            normalization += weight;
            spectrum[taps - 1 - tap].real = weight;
        }
        if (normalization < .5)
            return frames;
        bool allAvailable = true;
        for (uint32_t frame = 0; frame < windowFrames; ++frame) {
            present[frame] = provider(center - radius + frame, samples[frame]);
            allAvailable = allAvailable && present[frame].valid &&
                           present[frame].availableChannels >= count;
            if (!present[frame].valid)
                std::fill_n(samples[frame].begin(), count, 0.0f);
            for (size_t channel = 0; channel < count; ++channel)
                peaks[channel] = std::max(peaks[channel], std::abs(double(samples[frame][channel])));
        }
        std::array<bool, 256> complete{};
        uint32_t missing = 0;
        // A valid full source window proves coverage for every overlapping
        // output filter. Partial windows still need the per-tap checks below:
        // zero coefficients must not turn irrelevant gaps into missing audio.
        if (allAvailable) {
            std::fill_n(complete.begin(), frames, true);
            if (!availableFrames.empty())
                std::fill_n(availableFrames.begin(), frames, uint8_t{1});
        } else {
            for (uint32_t frame = 0; frame < frames; ++frame) {
                bool valid = true, available = true;
                for (uint32_t tap = 0; tap < taps; ++tap)
                    if (weights[tap] != 0) {
                        const auto source = present[frame + tap];
                        valid = valid && source.valid;
                        available = available && source.valid && source.availableChannels >= count;
                    }
                complete[frame] = valid;
                if (!valid || !available)
                    ++missing;
                else if (!availableFrames.empty())
                    availableFrames[frame] = 1;
            }
        }
        transform<false>(spectrum);
        const auto scale = 1 / (size * normalization);
        for (size_t channel = 0; channel < count;) {
            if (peaks[channel] == 0) {
                ++channel;
                continue;
            }
            // Two real channels can share a complex convolution. Keep widely
            // differing magnitudes separate to bound numerical cross-talk.
            const bool pair = channel + 1 < count && peaks[channel + 1] != 0 &&
                std::max(peaks[channel], peaks[channel + 1]) <=
                    4096 * std::min(peaks[channel], peaks[channel + 1]);
            std::array<Complex, size> values{};
            for (uint32_t frame = 0; frame < windowFrames; ++frame)
                values[frame] = {samples[frame][channel], pair ? samples[frame][channel + 1] : 0.0};
            transform<false>(values);
            for (uint32_t bin = 0; bin < size; ++bin) {
                const auto a = values[bin], b = spectrum[bin];
                values[bin] = {a.real * b.real - a.imaginary * b.imaginary,
                               a.real * b.imaginary + a.imaginary * b.real};
            }
            transform<true>(values);
            for (uint32_t frame = 0; frame < frames; ++frame)
                if (complete[frame]) {
                    const auto write = [&](size_t target, double sample) {
                        sample *= scale;
                        output.channels[target][frame * output.stride] =
                            // The ordered range comparison also rejects NaN and
                            // infinity; a separate finite check repeats work on
                            // every sample (an out-of-line CRT call on MSVC).
                            std::abs(sample) <= std::numeric_limits<float>::max()
                                ? static_cast<float>(sample) : 0;
                    };
                    write(channel, values[frame + 2 * radius].real);
                    if (pair)
                        write(channel + 1, values[frame + 2 * radius].imaginary);
                }
            channel += pair ? 2 : 1;
        }
        return missing;
    }
};
} // namespace zcore::routing
