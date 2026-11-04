// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#include <zcore/routing/audio_resampler.h>
#include <zcore/routing/sinc_kernel.h>
#include <zcore/routing/minimum_phase_kernel.h>
#include "unity_convolution.hpp"
#include "cached_fir.hpp"
#include "finite_samples.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace zcore::routing {
namespace {
bool validClockRate(double rate) noexcept {
    // Nominal endpoint formats stay at 8–192 kHz. Hardware clock estimates
    // may drift by up to the same 0.5% admitted by AudioHistory.
    return std::isfinite(rate) && rate >= 8000 * 0.995 && rate <= 192000 * 1.005;
}
void silence(ResampleOutput output, uint32_t frames) noexcept {
    if (!output.stride || output.stride > kMaximumRoutingChannels)
        return;
    const auto count = std::min({frames, output.capacityFrames, AudioResampler::kMaximumFrames});
    for (auto* plane :
         output.channels.first(std::min(output.channels.size(), size_t(kMaximumRoutingChannels))))
        if (plane)
            for (uint32_t frame = 0; frame < count; ++frame)
                plane[frame * output.stride] = 0;
}
} // namespace
AudioResampler::AudioResampler() : AudioResampler(ResamplingFilter::LinearPhase) {}
AudioResampler::AudioResampler(ResamplingFilter filter)
    : linear_(filter == ResamplingFilter::LinearPhase ? &SincKernel::prepared() : nullptr),
      minimum_(filter == ResamplingFilter::MinimumPhase ? &MinimumPhaseKernel::prepared()
                                                        : nullptr),
      convolution_(linear_ ? &UnityConvolutionPlan::prepared() : nullptr) {
    if (!linear_ && !minimum_)
        throw std::invalid_argument("Unsupported resampling filter");
}
uint32_t AudioResampler::read(ResamplePosition read, ResampleOutput output, uint32_t frames,
                              ReadFrame provider, const void* context,
                              std::span<uint8_t> availableFrames) const noexcept {
    silence(output, frames);
    std::fill_n(availableFrames.begin(), std::min<size_t>(frames, availableFrames.size()),
                uint8_t{0});
    if (!provider || !frames || frames > kMaximumFrames || frames > output.capacityFrames ||
        (!availableFrames.empty() && availableFrames.size() < frames) || !output.stride ||
        output.stride > kMaximumRoutingChannels || output.channels.empty() ||
        output.channels.size() > kMaximumRoutingChannels || !std::isfinite(read.offset) ||
        read.anchor == UINT64_MAX ||
        (minimum_ && read.nominalSourceRate != 0 &&
         (!std::isfinite(read.nominalSourceRate) || read.nominalSourceRate < 8000 ||
          read.nominalSourceRate > 192000)) ||
        !(read.localClockCorrection
              ? std::isfinite(read.sourceRate) && read.sourceRate >= 8000 * 0.8 &&
                    read.sourceRate <= 192000 * 1.25
              : validClockRate(read.sourceRate)) ||
        !validClockRate(read.outputRate) ||
        std::any_of(output.channels.begin(), output.channels.end(), [](auto* plane) {
            return !plane;
        }))
        return frames;
    const auto step = read.sourceRate / read.outputRate;
    if (minimum_ && read.nominalSourceRate &&
        (read.sourceRate < read.nominalSourceRate * (read.localClockCorrection ? .8 : .995) ||
         read.sourceRate > read.nominalSourceRate * (read.localClockCorrection ? 1.25 : 1.005)))
        return frames;
    const auto filterRate =
        minimum_ && read.nominalSourceRate ? read.nominalSourceRate : read.sourceRate;
    const auto cutoff = SincKernel::kCutoff * std::min(1.0, read.outputRate / filterRate);
    const bool sameRate = std::abs(read.sourceRate - read.outputRate) <= read.outputRate * 2e-9;
    const auto getFrame = [&](uint64_t position, std::span<float> values) {
        const auto result = provider(context, position, values.first(output.channels.size()));
        if (result.valid) {
            const auto count = static_cast<uint32_t>(output.channels.size());
            const auto available = std::min(count, result.availableChannels);
            detail::sanitizeFinite(values.data(), available);
            std::fill(values.begin() + available, values.begin() + count, 0.0f);
        }
        return result;
    };

    if (convolution_ && step == 1 && frames >= 128 &&
        frames <= UnityConvolutionPlan::kMaximumFrames) {
        const auto integral = std::floor(read.offset);
        const auto fraction = static_cast<double>(read.offset - integral);
        if (fraction >= .0002 && fraction <= .9998 &&
            integral >= -static_cast<long double>(read.anchor) &&
            integral < static_cast<long double>(UINT64_MAX - read.anchor - 1)) {
            const auto center = integral < 0 ? read.anchor - static_cast<uint64_t>(-integral)
                                             : read.anchor + static_cast<uint64_t>(integral);
            constexpr uint64_t radius = UnityConvolutionPlan::kSourceRadius;
            bool constant = center >= radius && center <= UINT64_MAX - radius - frames;
            for (uint32_t index = 1; constant && index < frames; ++index) {
                const auto offset = read.offset + static_cast<long double>(index);
                const auto whole = std::floor(offset);
                // Adding an integer can round the fractional bits of a
                // double-precision long double (MSVC). Admit only that tiny
                // arithmetic noise, never a changing clock ratio or phase.
                constant = whole == integral + index &&
                           std::abs(static_cast<double>(offset - whole) - fraction) <= 1e-12;
            }
            if (constant)
                return convolution_->read(center, fraction, output, frames, *linear_, getFrame,
                                          availableFrames);
        }
    }

    // A nearly equal measured clock is still a fractional FIR read. The cached
    // reader preserves its phase/rate exactly and declines direct-copy frames.
    if (linear_)
        if (const auto cached =
                CachedFir::read(read, output, frames, *linear_, cutoff, getFrame, availableFrames))
            return *cached;

    // A 32-frame output tile needs 16 KiB of double accumulators at 64
    // channels. Read each source frame once per tile, rather than rereading its
    // atomic payload for every overlapping output filter. No reader workspace
    // allocation or shared mutable conversion state is needed.
    constexpr uint32_t tileFrames = 32;
    struct Target {
        uint64_t center{};
        double fraction{};
        bool complete{false}, available{true}, direct{false};
    };
    const auto count = output.channels.size();
    const auto radius = static_cast<uint32_t>(std::ceil(SincKernel::kRadius / cutoff));
    const auto past = minimum_ ? 2 * radius : radius;
    const auto future = minimum_ ? 0 : radius;
    uint32_t missing = 0;
    for (uint32_t begin = 0; begin < frames; begin += tileFrames) {
        const auto length = std::min(frames - begin, tileFrames);
        std::array<Target, tileFrames> targets{};
        std::array<std::array<double, kMaximumRoutingChannels>, tileFrames> sums{};
        std::array<double, tileFrames> normalization{};
        std::array<float, kMaximumRoutingChannels> source{};
        uint64_t first = UINT64_MAX, last = 0;
        for (uint32_t index = 0; index < length; ++index) {
            auto& target = targets[index];
            const auto offset = read.offset + static_cast<long double>(begin + index) * step;
            const auto integral = std::floor(offset);
            if (integral < -static_cast<long double>(read.anchor) ||
                integral >= static_cast<long double>(UINT64_MAX - read.anchor - 1))
                continue;
            target.center = integral < 0 ? read.anchor - static_cast<uint64_t>(-integral)
                                         : read.anchor + static_cast<uint64_t>(integral);
            target.fraction = static_cast<double>(offset - integral);
            target.direct =
                linear_ && sameRate && (target.fraction < 0.0002 || target.fraction > 0.9998);
            if (target.direct) {
                if (target.fraction > 0.9998)
                    ++target.center;
                const auto value = getFrame(target.center, source);
                target.complete = value.valid;
                target.available = value.availableChannels >= count;
                if (value.valid)
                    for (size_t channel = 0; channel < count; ++channel)
                        output.channels[channel][(begin + index) * output.stride] = source[channel];
            } else if (target.center >= past && target.center <= UINT64_MAX - future) {
                target.complete = true;
                first = std::min(first, target.center - past);
                last = std::max(last, target.center + future);
            }
        }
        if (first != UINT64_MAX) {
            // Even a locally corrected 30:1 source/output ratio has bounded
            // filter support (under 7300 frames). Avoid end++ wrap.
            const auto windowFrames = static_cast<uint32_t>(last - first + 1);
            for (uint32_t index = 0; index < windowFrames; ++index) {
                const auto position = first + index;
                const auto value = getFrame(position, source);
                for (uint32_t outputFrame = 0; outputFrame < length; ++outputFrame) {
                    auto& target = targets[outputFrame];
                    if (!target.complete || target.direct)
                        continue;
                    const auto distance = position >= target.center
                                              ? static_cast<double>(position - target.center)
                                              : -static_cast<double>(target.center - position);
                    const auto weight =
                        minimum_ ? minimum_->coefficient(target.fraction - distance, cutoff)
                                 : linear_->coefficient(distance - target.fraction, cutoff);
                    if (weight == 0)
                        continue;
                    if (!value.valid) {
                        target.complete = false;
                        continue;
                    }
                    target.available = target.available && value.availableChannels >= count;
                    normalization[outputFrame] += weight;
                    auto& accumulated = sums[outputFrame];
                    // Common hardware layouts have fixed widths. Specializing
                    // the independent channel sums lets the compiler vectorize
                    // them without a per-tap dynamic loop or changing precision.
                    const auto accumulate = [&]<size_t Channels>() {
                        for (size_t channel = 0; channel < Channels; ++channel)
                            accumulated[channel] += weight * source[channel];
                    };
                    switch (count) {
                    case 1:
                        accumulate.template operator()<1>();
                        break;
                    case 2:
                        accumulate.template operator()<2>();
                        break;
                    case 8:
                        accumulate.template operator()<8>();
                        break;
                    case 16:
                        accumulate.template operator()<16>();
                        break;
                    default:
                        for (size_t channel = 0; channel < count; ++channel)
                            accumulated[channel] += weight * source[channel];
                    }
                }
            }
        }
        for (uint32_t index = 0; index < length; ++index) {
            const auto& target = targets[index];
            if (!target.complete || (!target.direct && normalization[index] < 0.5)) {
                ++missing;
                continue;
            }
            if (!target.available)
                ++missing;
            else if (!availableFrames.empty())
                availableFrames[begin + index] = 1;
            if (!target.direct)
                for (size_t channel = 0; channel < count; ++channel) {
                    const auto sample = sums[index][channel] / normalization[index];
                    output.channels[channel][(begin + index) * output.stride] =
                        std::isfinite(sample) &&
                                std::abs(sample) <= std::numeric_limits<float>::max()
                            ? static_cast<float>(sample)
                            : 0;
                }
        }
    }
    return missing;
}
} // namespace zcore::routing
