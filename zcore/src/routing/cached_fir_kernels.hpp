// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once
#include <array>
#include <cstdint>
namespace zcore::routing::detail {
using CachedFirPairKernel = std::array<double, 2> (*)(const float*, const float*, const double*,
                                                      uint32_t) noexcept;
// Native adapters configure a kernel on the control thread only after proving
// its CPU/OS requirements. Null preserves the portable baseline. The realtime
// getter never initializes, allocates, locks or probes the operating system.
void setCachedFirPairKernel(CachedFirPairKernel) noexcept;
CachedFirPairKernel cachedFirPairKernel() noexcept;
} // namespace zcore::routing::detail
