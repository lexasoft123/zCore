// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <zcore/routing/routing.h>
#include <atomic>
#include <array>
#include <bit>

namespace zcore::routing {
class SincKernel;
class SincPhaseTables;
class MinimumPhaseKernel;
class UnityConvolutionPlan;
enum class ResamplingFilter { LinearPhase, MinimumPhase };

enum class ResamplePath : uint32_t {
    Direct,
    PhaseTable,
    CachedExact,
    Convolution,
    TiledExact,
    Invalid,
    Count
};
// Successful immutable table constructions; increments only on the control thread.
uint64_t sincCoefficientTableConstructions() noexcept;
struct ResampleDiagnosticsSnapshot {
    std::array<uint64_t, static_cast<size_t>(ResamplePath::Count)> blocks{};
    uint32_t lastPathMask{};
    uint64_t tableMisses{};
    double cutoff{}, sourceRate{}, outputRate{};
    // Tables are constructed once on control and remain immutable. There is
    // no runtime cache eviction, rebuild, or ratio-keyed mutable coefficient cache.
    uint64_t coefficientTableConstructions{0};
};
struct ResampleDiagnostics {
    std::array<std::atomic<uint64_t>, static_cast<size_t>(ResamplePath::Count)> blocks{};
    std::atomic<uint32_t> lastPathMask{0};
    std::atomic<uint64_t> tableMisses{0}, cutoffBits{0}, sourceBits{0}, outputBits{0};
    void record(uint32_t mask) noexcept {
        lastPathMask.store(mask, std::memory_order_relaxed);
        for (size_t index = 0; index < blocks.size(); ++index)
            if (mask & (1u << index))
                blocks[index].fetch_add(1, std::memory_order_relaxed);
    }
    ResampleDiagnosticsSnapshot snapshot() const noexcept {
        ResampleDiagnosticsSnapshot result;
        result.coefficientTableConstructions = sincCoefficientTableConstructions();
        for (size_t index = 0; index < blocks.size(); ++index)
            result.blocks[index] = blocks[index].load(std::memory_order_relaxed);
        result.lastPathMask = lastPathMask.load(std::memory_order_relaxed);
        result.tableMisses = tableMisses.load(std::memory_order_relaxed);
        result.cutoff = std::bit_cast<double>(cutoffBits.load(std::memory_order_relaxed));
        result.sourceRate = std::bit_cast<double>(sourceBits.load(std::memory_order_relaxed));
        result.outputRate = std::bit_cast<double>(outputBits.load(std::memory_order_relaxed));
        return result;
    }
};
inline constexpr uint32_t resamplePath(ResamplePath value) noexcept {
    return 1u << static_cast<uint32_t>(value);
}
struct ResamplePosition {
    uint64_t anchor;
    long double offset;
    // Nominal formats are 8–192 kHz. Either clock estimate may differ from
    // those endpoint limits by at most 0.5% to account for physical drift.
    double sourceRate, outputRate;
    // Only a bounded closed timestamp interval approved by the sample clock
    // (counter verification or phase tolerance) may use a corrected slope.
    // Endpoint/output rate limits do not change.
    bool localClockCorrection{false};
    // Optional fixed source format for minimum-phase filter shape. Sample
    // positions still follow sourceRate; the linear-phase reader ignores it.
    double nominalSourceRate{0};
    // Optional per-reader instrumentation, allocated by its control owner.
    ResampleDiagnostics* diagnostics{nullptr};
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
