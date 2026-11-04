// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <zcore/routing/sinc_kernel.h>
#include <array>

namespace zcore::routing {
// Experimental causal interpolation kernel. Prepare on control; lookup needs
// only samples at/before the target. Its nonlinear phase adds frequency-dependent
// group delay, so callers must not mix it with an integer direct-read shortcut.
class MinimumPhaseKernel {
  public:
    static constexpr uint32_t kPastSupport = 2 * SincKernel::kRadius;
    static const MinimumPhaseKernel& prepared();
    MinimumPhaseKernel(const MinimumPhaseKernel&) = delete;
    MinimumPhaseKernel& operator=(const MinimumPhaseKernel&) = delete;
    double coefficient(double pastDistance, double cutoff) const noexcept;

  private:
    static constexpr uint32_t kDensity = 512;
    MinimumPhaseKernel();
    std::array<float, kPastSupport * kDensity + 1> values_{};
};
} // namespace zcore::routing
