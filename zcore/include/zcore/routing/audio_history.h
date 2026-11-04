// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once

#include <zcore/routing/audio_source.h>
#include <array>

namespace zcore::routing {
enum class ResamplingFilter;

enum class AudioHistoryMissingReason : uint32_t {
    InvalidRequest,
    Offline,
    AnchorUnavailable,
    ClockMapping,
    ResamplingSupport,
    GenerationChanged,
    Count
};

struct AudioHistoryReadSnapshot {
    bool enabled{false};
    uint64_t reads{}, requestedFrames{};
    // Actual resampler calls after clock segmentation: 1–31, 32–127,
    // 128–256 and >256 logical frames. Diagnostic only.
    std::array<uint64_t, 4> resampleBlockCounts{};
    std::array<uint64_t, static_cast<size_t>(AudioHistoryMissingReason::Count)> missingFrames{};
    // Failed source-sample lookups, not missing output frames. Several filter
    // taps may fail for one output frame, so these counters are not additive
    // with missingFrames. They distinguish late publication from expired data.
    uint64_t unpublishedSampleReads{}, expiredSampleReads{}, racingSampleReads{},
        absentSampleReads{};
    // Host-clock phase steps with an authoritative continuous sample counter.
    uint64_t maximumVerifiedClockStepNs{}, verifiedClockSteps{};
};

struct AudioHistoryStatus {
    bool online;
    uint32_t availableChannels;
    // Effective clock anchor, which can differ from the reported callback time.
    uint64_t lastHostNs, callbacks, discontinuities;
    double measuredRate;
};

// A fixed logical channel layout, one serial producer and independent readers.
// All storage is allocated at construction. A narrower hardware profile zeros
// absent channels without changing saved wiring. stop() is serialized with the
// producer, after its callbacks have joined; readers may continue concurrently.
// By default, rate/fractional-clock conversion uses a prepared symmetric bandlimited
// kernel, so the caller must queue enough history on both sides of the desired
// timestamp (96 / (0.94 * min(1, outputRate/sourceRate)) source frames). Missing
// support is reported as missing audio; resampling never shifts timestamps to
// conceal absent source frames. Equal-rate aligned reads use the direct path.
// Past host times use retained callback clock intervals, rather than a newer
// rate estimate. Stable intervals coalesce only within timestamp precision.
// Only authoritative continuous source counters may replace an otherwise unusable
// reported interval with the current measured-rate interval. Correction is bounded
// at two milliseconds cumulatively; sustained unsupported clocks are not relabeled
// as nominal. Missing source frames still reset the generation and are never hidden.
// Diagnostic rate estimates use overlapping half-second observations; the
// newest interval can extrapolate until its next callback anchor is available.
// A reader's output clock may also drift within the 0.5% measured-rate bounds.
class AudioHistory final : public AudioSource {
  public:
    static constexpr uint32_t kDefaultCapacityFrames = 32768;
    explicit AudioHistory(uint32_t channels, uint32_t capacityFrames = kDefaultCapacityFrames);
    // Experimental filter choice; prepare on control. Minimum-phase history
    // pins its cutoff to the source format while positions follow its clock.
    AudioHistory(uint32_t channels, uint32_t capacityFrames, ResamplingFilter filter,
                 bool readDiagnostics = false);
    ~AudioHistory();
    AudioHistory(const AudioHistory&) = delete;
    AudioHistory& operator=(const AudioHistory&) = delete;
    uint32_t channels() const noexcept override;
    bool append(RoutingInput input, uint32_t frames, uint64_t firstHostNs, double nominalRate,
                double sourceFrame = 0, bool sourceFrameValid = false) noexcept;
    void stop() noexcept;
    AudioHistoryStatus status() const noexcept;
    // Exclusive nominal-clock endpoint of the last fully published block.
    // Zero means offline or an overlapping publication. This is a readiness
    // hint, not a guarantee about retained samples, gaps or filter support.
    uint64_t publishedBlockEndHostNs() const noexcept;
    // Optional counters allocate on construction, remain disabled by default,
    // and survive stop/reopen. Read after joining readers for a coherent final
    // snapshot; live snapshots are independent atomic observations.
    AudioHistoryReadSnapshot readDiagnostics() const noexcept;
    AudioReadResult read(uint64_t firstHostNs, double rate, RoutingOutput output, uint32_t frames,
                         AudioReadCursor& cursor) const noexcept override;
    // Minimum-phase histories also prepare a linear reader on control, so a
    // later mixed scene can preserve phase without reopening this producer.
    // A linear-only history rejects minimum-phase requests as missing audio.
    AudioReadResult readWithFilter(uint64_t firstHostNs, double rate, RoutingOutput output,
                                   uint32_t frames, AudioReadCursor& cursor,
                                   ResamplingFilter filter) const noexcept override;
    std::optional<AudioReadResult>
    readChannels(uint64_t firstHostNs, double rate, RoutingOutput output, uint32_t frames,
                 AudioReadCursor& cursor, std::span<const uint32_t> channels,
                 std::optional<ResamplingFilter> filter) const noexcept override;
    // Optional per-frame coverage distinguishes valid silence from missing
    // support when combining independently clocked captures. Nonempty storage
    // must cover frames; output and coverage are zero on invalid/stopped reads.
    AudioReadResult readWithCoverage(uint64_t firstHostNs, double rate, RoutingOutput output,
                                     uint32_t frames, AudioReadCursor& cursor,
                                     std::span<uint8_t> availableFrames) const noexcept;

  private:
    AudioReadResult readUsing(uint64_t firstHostNs, double rate, RoutingOutput output,
                              uint32_t frames, AudioReadCursor& cursor,
                              std::span<uint8_t> availableFrames,
                              const class AudioResampler& resampler,
                              std::span<const uint32_t> channels = {}) const noexcept;
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
} // namespace zcore::routing
