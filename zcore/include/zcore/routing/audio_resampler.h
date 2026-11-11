// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <zcore/routing/routing.h>

namespace zcore::routing {
class SincKernel;
class SincPhaseTables;
class MinimumPhaseKernel;
class UnityConvolutionPlan;
enum class ResamplingFilter { LinearPhase, MinimumPhase };

struct ResamplePosition {
    uint64_t anchor;
    long double offset;
    // Nominal formats are 8–192 kHz. Either clock estimate may differ from
    // those endpoint limits by at most 0.5% to account for physical drift.
    double sourceRate, outputRate;
    // Only a bounded, sample-counter-verified closed timestamp interval may
    // use a locally corrected slope. Endpoint/output rate limits do not change.
    bool localClockCorrection{false};
    // Optional fixed source format for minimum-phase filter shape. Sample
    // positions still follow sourceRate; the linear-phase reader ignores it.
    double nominalSourceRate{0};
};
struct ResampleFrame {
    bool valid;
    // Tail planes beyond this count are always forced to zero by the reader.
    uint32_t availableChannels;
};
struct ResampleOutput {
    std::span<float* const> channels;
    // Capacity is measured in logical frames. Each plane must span
    // (capacityFrames - 1) * stride + 1 floats; stride supports interleaving.
    uint32_t capacityFrames;
    uint32_t stride{1};
};

// Construct on control to prepare the immutable sinc tables. read() owns only
// bounded stack state; concurrent callers need no shared cursor or workspace.
// The provider supplies timestamp-addressed frames, including all filter wings.
// Missing support is silent and counted, without a linear fallback or time shift.
// The default keeps the existing symmetric kernel and aligned direct reads.
// Wide equal-rate fractional blocks may evaluate that same kernel by FFT
// convolution, with double intermediates and independent reader scratch.
// Cached blocks whose phase changes per frame (clocks that drift, or 44.1 kHz
// and 48 kHz either way) take that kernel's weights from phase tables when
// their whole filter window is present, and sum them in single precision: for
// samples within full scale, within 1e-6 of the double-precision sum.
// Experimental minimum-phase mode uses past support only and always filters,
// including integer-aligned targets, to keep its nonlinear phase continuous.
class AudioResampler {
  public:
    using ReadFrame = ResampleFrame (*)(const void*, uint64_t, std::span<float>) noexcept;
    static constexpr uint32_t kMaximumFrames = 2048;
    AudioResampler();
    explicit AudioResampler(ResamplingFilter filter);
    uint32_t read(ResamplePosition position, ResampleOutput output, uint32_t frames,
                  ReadFrame provider, const void* context,
                  std::span<uint8_t> availableFrames = {}) const noexcept;

  private:
    const SincKernel* const linear_;
    const SincPhaseTables* const phases_;
    const MinimumPhaseKernel* const minimum_;
    const UnityConvolutionPlan* const convolution_;
};
} // namespace zcore::routing
