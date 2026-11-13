// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#include <zcore/routing/sample_clock.h>
#include "timing.h"
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <stdexcept>

namespace zcore::routing {
namespace {
struct Anchor {
    uint64_t frame{0}, ns{0}, reportedNs{0};
    bool contiguous{false}, phaseCorrected{false};
};
struct Record {
    std::atomic<uint64_t> stamp{0}, frame{0}, ns{0}, reportedNs{0};
    std::atomic<bool> contiguous{false}, phaseCorrected{false};
};
bool supportedNominal(double rate) noexcept {
    return std::isfinite(rate) && rate >= 8000 && rate <= 192000;
}
bool supportedMeasured(double rate, double nominal) noexcept {
    return std::isfinite(rate) && rate >= nominal * 0.995 && rate <= nominal * 1.005;
}
constexpr uint64_t maximumPhaseCorrectionNs = 2000000;
double intervalRate(Anchor a, Anchor b, double nominal) noexcept {
    if (b.ns <= a.ns || b.frame <= a.frame)
        return 0;
    const auto frames = b.frame - a.frame;
    const auto elapsed = b.ns - a.ns;
    // Integer-nanosecond host conversion can round a nominal interval by a
    // nanosecond. Preserve aligned direct reads within that timestamp precision.
    if (std::abs(static_cast<long double>(frames) * 1e9L / nominal - elapsed) <= 2)
        return nominal;
    const auto rate = static_cast<double>(frames) * 1e9 / elapsed;
    if (supportedMeasured(rate, nominal))
        return rate;
    // A device may correct its host-time projection by a few samples while
    // its hardware sample counter stays continuous. Keep the measured local
    // interval for bandlimited conversion instead of replacing a whole block
    // with silence. Unverified intervals require explicit phase admission
    // below; larger unverified jumps still fail.
    const auto phaseError = std::abs(static_cast<long double>(frames) * 1e9L / nominal - elapsed);
    // Quad Cortex produces verified ~2.2-sample host-clock steps. Admit at
    // most three samples in a CLOSED interval with continuous source frames;
    // keep the slope limits and never extrapolate that corrected slope.
    return ((b.contiguous && phaseError <= 3e9L / nominal) || b.phaseCorrected) && rate >= nominal * 0.8 &&
                   rate <= nominal * 1.25
               ? rate
               : 0;
}
} // namespace
struct SampleClockHistory::Impl {
    const uint32_t capacity;
    std::unique_ptr<Record[]> records;
    std::atomic<uint64_t> begin{0}, end{0};
    bool correctionBlocked{false};
    const SampleClockPhaseConfig phaseConfig;
    std::array<uint64_t, 64> correctionTimes{};
    uint32_t correctionCount{0}, correctionNext{0};
    std::atomic<uint64_t> phaseCorrections{0}, correctedSamples{0}, rejectedIntervals{0};
    Impl(uint32_t count, SampleClockPhaseConfig config)
        : capacity(count), records(std::make_unique<Record[]>(count)), phaseConfig(config) {}
    bool canCorrectPhase(uint64_t ns) const noexcept {
        const auto limit = phaseConfig.maximumCorrectionsPerSecond;
        if (!limit) return false;
        if (correctionCount == limit &&
            (ns < correctionTimes[correctionNext] ||
             ns - correctionTimes[correctionNext] < 1000000000ULL))
            return false;
        return true;
    }
    void commitPhaseCorrection(uint64_t ns) noexcept {
        const auto limit = phaseConfig.maximumCorrectionsPerSecond;
        correctionTimes[correctionNext] = ns;
        correctionNext = (correctionNext + 1) % limit;
        correctionCount = std::min(correctionCount + 1, limit);
    }
    bool read(uint64_t ticket, Anchor& result) const noexcept {
        if (ticket == UINT64_MAX)
            return false;
        const auto& record = records[ticket % capacity];
        if (record.stamp.load() != ticket + 1)
            return false;
        result = {record.frame.load(), record.ns.load(), record.reportedNs.load(),
                  record.contiguous.load(), record.phaseCorrected.load()};
        return record.stamp.load() == ticket + 1;
    }
};
SampleClockHistory::SampleClockHistory(uint32_t capacity)
    : SampleClockHistory(capacity, {}) {}
SampleClockHistory::SampleClockHistory(uint32_t capacity, SampleClockPhaseConfig phaseConfig) {
    if (capacity < 2 || capacity > kCapacity + 1)
        throw std::invalid_argument("Invalid sample clock history capacity");
    if (!std::isfinite(phaseConfig.maximumStepSamples) || phaseConfig.maximumStepSamples < 0 ||
        phaseConfig.maximumStepSamples > 16 || phaseConfig.maximumCorrectionsPerSecond > 64)
        throw std::invalid_argument("Invalid sample clock phase configuration");
    impl_ = std::make_unique<Impl>(capacity, phaseConfig);
}
SampleClockHistory::~SampleClockHistory() = default;
bool SampleClockHistory::append(uint64_t frame, uint64_t hostNs,
                                bool contiguousSourceFrames, double nominalRate,
                                double fallbackRate) noexcept {
    auto& self = *impl_;
    const auto ticket = self.end.load();
    if (!hostNs || ticket == UINT64_MAX - 1)
        return false;
    auto mappedNs = hostNs;
    bool phaseCorrected = false;
    uint64_t correctedSamples = 0;
    if (ticket > self.begin.load()) {
        Anchor previous;
        if (!self.read(ticket - 1, previous) || frame <= previous.frame ||
            hostNs <= previous.reportedNs)
            return false;
        const bool nominalValid = supportedNominal(nominalRate);
        const bool reportedValid = nominalValid &&
            intervalRate(previous, {frame, hostNs, hostNs, contiguousSourceFrames}, nominalRate);
        if (reportedValid)
            self.correctionBlocked = false;
        if (!reportedValid && !contiguousSourceFrames && nominalValid &&
            supportedMeasured(fallbackRate, nominalRate)) {
            // Re-anchor phase over this closed interval, using a fractional
            // resampler slope. No PCM is removed or inserted. The total slew
            // in source samples is bounded, as is its frequency. Keep the raw
            // endpoint so a persistent phase step consumes only one event.
            const auto duration = static_cast<long double>(frame - previous.frame) *
                                  1e9L / fallbackRate;
            const auto errorNs = std::abs(duration - (static_cast<long double>(hostNs) - previous.ns));
            const auto errorSamples = errorNs * nominalRate / 1e9L;
            const auto bound = std::min(self.phaseConfig.maximumStepSamples,
                                        nominalRate * 0.0001);
            const auto localRate = static_cast<double>(frame - previous.frame) * 1e9 /
                                   (hostNs - previous.ns);
            if (bound > 0 && errorSamples <= bound + 2e-9 * nominalRate &&
                localRate >= nominalRate * 0.8 && localRate <= nominalRate * 1.25 &&
                self.canCorrectPhase(hostNs)) {
                phaseCorrected = true;
                correctedSamples = static_cast<uint64_t>(std::max(1.0L,
                    std::ceil(errorSamples - 2e-9L * nominalRate)));
            }
        }
        if (contiguousSourceFrames && nominalValid &&
            supportedMeasured(fallbackRate, nominalRate) && !reportedValid &&
            !self.correctionBlocked) {
            // Continuous source frames prove that this is timestamp noise, not
            // absent audio. Retain the mapped phase and use the measured slope.
            const long double duration =
                static_cast<long double>(frame - previous.frame) * 1e9L / fallbackRate;
            if (duration < 1 || duration >= static_cast<long double>(UINT64_MAX))
                return false;
            const auto whole = static_cast<uint64_t>(duration);
            if (previous.ns > UINT64_MAX - whole)
                return false;
            const auto candidate = previous.ns + whole;
            const auto correction = candidate >= hostNs ? candidate - hostNs : hostNs - candidate;
            // Match AudioHistory's two-millisecond discontinuity boundary, and
            // require a valid raw interval or reset after a breached bound.
            if (correction > maximumPhaseCorrectionNs)
                self.correctionBlocked = true;
            else
                mappedNs = candidate;
        }
        if (mappedNs <= previous.ns)
            return false;
        if (nominalValid && !intervalRate(previous,
                {frame, mappedNs, hostNs, contiguousSourceFrames, phaseCorrected}, nominalRate)) {
            phaseCorrected = false;
            self.rejectedIntervals.fetch_add(1, std::memory_order_relaxed);
        }
    }
    auto& record = self.records[ticket % self.capacity];
    record.stamp.store(0);
    record.frame.store(frame);
    record.ns.store(mappedNs);
    record.reportedNs.store(hostNs);
    record.contiguous.store(contiguousSourceFrames);
    record.phaseCorrected.store(phaseCorrected);
    record.stamp.store(ticket + 1);
    self.end.store(ticket + 1);
    // Only a fully published, usable interval spends budget or counts as a
    // correction. A failed append/rejected mapping must leave these untouched.
    if (phaseCorrected) {
        self.commitPhaseCorrection(hostNs);
        self.phaseCorrections.fetch_add(1, std::memory_order_relaxed);
        self.correctedSamples.fetch_add(correctedSamples, std::memory_order_relaxed);
    }
    return true;
}
SampleClockPhaseDiagnostics SampleClockHistory::phaseDiagnostics() const noexcept {
    return {impl_->phaseCorrections.load(std::memory_order_relaxed),
            impl_->correctedSamples.load(std::memory_order_relaxed),
            impl_->rejectedIntervals.load(std::memory_order_relaxed)};
}
uint64_t SampleClockHistory::lastHostNs() const noexcept {
    auto& self = *impl_;
    const auto end = self.end.load();
    if (end == self.begin.load())
        return 0;
    Anchor last;
    return self.read(end - 1, last) ? last.ns : 0;
}
void SampleClockHistory::reset() noexcept {
    impl_->correctionBlocked = false;
    impl_->correctionCount = impl_->correctionNext = 0;
    impl_->begin.store(impl_->end.load());
}
SampleClockHistory::Snapshot SampleClockHistory::snapshot() const noexcept {
    const auto end = impl_->end.load();
    const auto begin = impl_->begin.load();
    return {std::max(begin, end > impl_->capacity ? end - impl_->capacity : 0), end};
}
SampleClockMapping SampleClockHistory::locate(Snapshot snapshot, uint64_t hostNs,
                                              long double fractionNs, double nominalRate,
                                              double fallbackRate,
                                              uint64_t throughNs) const noexcept {
    const auto& self = *impl_;
    if (snapshot.begin >= snapshot.end || snapshot.end - snapshot.begin > self.capacity ||
        snapshot.begin < self.begin.load() || snapshot.end > self.end.load() ||
        !supportedNominal(nominalRate) || !std::isfinite(fractionNs) || fractionNs < 0 ||
        fractionNs >= 1)
        return {};
    Anchor last;
    if (!self.read(snapshot.end - 1, last))
        return {};
    Anchor a = last, b;
    double rate = fallbackRate;
    uint64_t endNs = UINT64_MAX;
    if (hostNs < last.ns) {
        // Find the interval containing this past host time. At most sixteen
        // comparisons for the maximum prepared capacity, independent of uptime.
        auto low = snapshot.begin, high = snapshot.end - 1;
        while (high - low > 1) {
            const auto middle = low + (high - low) / 2;
            Anchor point;
            if (!self.read(middle, point))
                return {};
            if (point.ns <= hostNs)
                low = middle;
            else
                high = middle;
        }
        if (!self.read(low, a) || !self.read(high, b) || hostNs < a.ns || hostNs >= b.ns)
            return {};
        rate = intervalRate(a, b, nominalRate);
        endNs = b.ns;
        if (rate && endNs <= throughNs && high < snapshot.end - 1) {
            auto endpoint = snapshot.end - 1;
            if (throughNs < last.ns) {
                auto left = high, right = endpoint;
                while (right - left > 1) {
                    const auto middle = left + (right - left) / 2;
                    Anchor point;
                    if (!self.read(middle, point))
                        return {};
                    if (point.ns <= throughNs)
                        left = middle;
                    else
                        right = middle;
                }
                endpoint = right;
            }
            // Keep validation bounded even for an unusual one-sample callback
            // profile. Normal >=64-frame callbacks fit within this budget for
            // a full 256-output-frame read at the maximum conversion ratio.
            if (endpoint - low <= 256) {
                Anchor stop;
                if (!self.read(endpoint, stop))
                    return {};
                const auto candidate = intervalRate(a, stop, nominalRate);
                bool stable = candidate != 0;
                for (auto ticket = high; stable && ticket <= endpoint; ++ticket) {
                    Anchor point;
                    if (!self.read(ticket, point) || point.frame <= a.frame || point.ns <= a.ns) {
                        stable = false;
                        break;
                    }
                    const long double predicted =
                        static_cast<long double>(point.frame - a.frame) * 1e9L / candidate;
                    // One nanosecond of timestamp quantization, with a small
                    // floating-point margin; do not smooth real clock curvature.
                    stable = std::abs(predicted - (point.ns - a.ns)) <= 1.01L;
                }
                if (stable) {
                    rate = candidate;
                    endNs = stop.ns;
                }
            }
        }
    } else if (snapshot.end - snapshot.begin >= 2) {
        Anchor prior;
        if (!self.read(snapshot.end - 2, prior))
            return {};
        const auto closedRate = intervalRate(prior, last, nominalRate);
        // A one-interval phase correction is not a new physical sample rate.
        // Never extrapolate its slope into future, not-yet-captured samples.
        rate = supportedMeasured(closedRate, nominalRate) ? closedRate
               : closedRate                               ? fallbackRate
                                                          : 0;
    }
    const bool correction = !supportedMeasured(rate, nominalRate);
    if (!rate || !std::isfinite(rate) || (endNs == UINT64_MAX && correction) ||
        snapshot.begin < self.begin.load())
        return {};
    return {true, a.frame, (static_cast<long double>(hostNs - a.ns) + fractionNs) * rate / 1e9L,
            rate, endNs,   correction};
}
} // namespace zcore::routing
