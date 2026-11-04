// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <array>
#include <cstdint>
#include <cmath>

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
        const auto index = static_cast<size_t>(position);
        const auto fraction = position - index;
        return values_[index] + (values_[index + 1] - values_[index]) * fraction;
    }

  private:
    static constexpr uint32_t kDensity = 2048;
    SincKernel();
    std::array<float, kRadius * kDensity + 1> values_{};
};
} // namespace zcore::routing
