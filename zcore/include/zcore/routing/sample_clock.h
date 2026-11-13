// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <cstdint>
#include <memory>

namespace zcore::routing {
// Prepared on control; zero disables unverified timestamp-phase tolerance.
// The effective sample bound is min(maximumStepSamples, nominalRate * 0.0001).
// A rolling one-second budget prevents a persistently wrong clock being hidden.
struct SampleClockPhaseConfig {
    double maximumStepSamples{2};
    uint32_t maximumCorrectionsPerSecond{8};
};
struct SampleClockPhaseDiagnostics {
    uint64_t phaseCorrections{}, correctedSamples{}, rejectedIntervals{};
};
struct SampleClockMapping {
    bool valid{false};
    uint64_t frame{0};
    long double offset{0};
    double rate{0};
    uint64_t endNs{UINT64_MAX};
    bool localClockCorrection{false};
};

// One serial producer records callback anchors; independent readers locate
// historical host times without reinterpreting them with a newer clock rate.
// Allocate on control. All publication and lookup work is bounded and lock-free.
class SampleClockHistory {
  public:
    struct Snapshot {
        uint64_t begin{0}, end{0};
    };
    explicit SampleClockHistory(uint32_t capacity);
    SampleClockHistory(uint32_t capacity, SampleClockPhaseConfig phaseConfig);
    SampleClockPhaseDiagnostics phaseDiagnostics() const noexcept;
    ~SampleClockHistory();
    SampleClockHistory(const SampleClockHistory&) = delete;
    SampleClockHistory& operator=(const SampleClockHistory&) = delete;
    // Small unverified phase steps use the prepared phaseConfig's bounded closed
    // interval slope; they never change PCM storage or extrapolate that slope.
    // Set only when an authoritative source sample counter proves contiguous
    // frames since the preceding anchor. It is not inferred from receipt time.
    // Valid rates permit only those continuous intervals to replace an otherwise
    // unusable reported timestamp interval with the measured-rate interval.
    // Cumulative correction is bounded; a breach requires a valid raw interval
    // or reset before another interval can be replaced.
    bool append(uint64_t frame, uint64_t hostNs, bool contiguousSourceFrames = false,
                double nominalRate = 0, double fallbackRate = 0) noexcept;
    // Effective timestamp of the newest anchor, including a verified correction.
    uint64_t lastHostNs() const noexcept;
    void reset() noexcept;
    Snapshot snapshot() const noexcept;
    // fractionNs retains sub-nanosecond output-grid positions. Only the newest
    // unclosed interval may extrapolate using its prior interval/fallback rate.
    // throughNs optionally coalesces stable intervals through a requested read,
    // after checking every intervening anchor against timestamp precision.
    SampleClockMapping locate(Snapshot snapshot, uint64_t hostNs, long double fractionNs,
                              double nominalRate, double fallbackRate,
                              uint64_t throughNs = 0) const noexcept;

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace zcore::routing
