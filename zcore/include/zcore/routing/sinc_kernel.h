// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <span>
#if defined(__aarch64__)
#include <arm_neon.h>
#endif

namespace zcore::routing {
// Kaiser-windowed sinc, following the filter design used by SingZ's
// zcore/legacy/resample.cpp. Timestamp-addressed histories need arbitrary
// fractional positions and measured clock ratios rather than a streaming FIFO.
// Prepare on control; coefficient() performs only bounded table interpolation.
class SincKernel {
  public:
    static constexpr uint32_t kRadius = 96;
    static constexpr double kCutoff = 0.94;
    static const SincKernel& prepared();
    SincKernel(const SincKernel&) = delete;
    SincKernel& operator=(const SincKernel&) = delete;
    double coefficient(double sourceDistance, double cutoff) const noexcept {
        const auto distance = std::abs(sourceDistance) * cutoff;
        if (!std::isfinite(distance) || cutoff <= 0 || cutoff > 1 || distance >= kRadius)
            return 0;
        const auto position = distance * kDensity;
        // The guards bound position to [0, 96 * 2048). Retain the integral
        // value in floating point for interpolation, avoiding a 64-bit integer
        // round trip on AArch64 while keeping the table index exactly bounded.
        const auto integral = std::floor(position);
        const auto index = static_cast<uint32_t>(integral);
        const auto fraction = position - integral;
        return values_[index] + (values_[index + 1] - values_[index]) * fraction;
    }

    // Fill the caller's bounded FIR workspace without allocation. Tap positions
    // retain (double(tap) - radius) - fraction and normalization retains scalar
    // tap order; the original scalar lookup remains the portable/tail oracle.
    double fillCoefficients(uint32_t radius, double fraction, double cutoff,
                            std::span<double> weights) const noexcept {
      size_t tap = 0;
      double normalization = 0;
#if defined(__aarch64__)
      if (std::isfinite(fraction) && cutoff > 0 && cutoff <= 1) {
        const auto pair = [&](size_t index) {
          const float64x2_t positions{double(index) - radius,
                                      double(index + 1) - radius};
          const auto source = vsubq_f64(positions, vdupq_n_f64(fraction));
          const auto distance =
              vmulq_f64(vabsq_f64(source), vdupq_n_f64(cutoff));
          const auto valid = vcltq_f64(distance, vdupq_n_f64(kRadius));
          const auto safe = vbslq_f64(valid, distance, vdupq_n_f64(0));
          const auto position = vmulq_f64(safe, vdupq_n_f64(kDensity));
          const auto integral = vrndmq_f64(position);
          const auto cellFraction = vsubq_f64(position, integral);
          const auto indices = vcvtq_u64_f64(integral);
          const auto first = vgetq_lane_u64(indices, 0);
          const auto second = vgetq_lane_u64(indices, 1);
          const float32x2_t low{values_[first], values_[second]};
          const float32x2_t high{values_[first + 1], values_[second + 1]};
          // Float subtraction is part of the existing table contract.
          const auto delta = vcvt_f64_f32(vsub_f32(high, low));
          const auto result =
              vaddq_f64(vcvt_f64_f32(low), vmulq_f64(delta, cellFraction));
          const auto masked = vbslq_f64(valid, result, vdupq_n_f64(0));
          vst1q_f64(weights.data() + index, masked);
          return masked;
        };
        for (; tap + 4 <= weights.size(); tap += 4) {
          const auto first = pair(tap);
          const auto second = pair(tap + 2);
          normalization += vgetq_lane_f64(first, 0);
          normalization += vgetq_lane_f64(first, 1);
          normalization += vgetq_lane_f64(second, 0);
          normalization += vgetq_lane_f64(second, 1);
        }
      }
#endif
      for (; tap < weights.size(); ++tap) {
        weights[tap] = coefficient((double(tap) - radius) - fraction, cutoff);
        normalization += weights[tap];
      }
      return normalization;
    }

  private:
    static constexpr uint32_t kDensity = 2048;
    SincKernel();
    std::array<float, kRadius * kDensity + 1> values_{};
};
} // namespace zcore::routing
