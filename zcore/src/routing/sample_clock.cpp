// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#include <zcore/routing/sample_clock.h>
#include "timing.h"
#include <algorithm>
#include <atomic>
#include <cmath>
#include <stdexcept>

namespace zcore::routing {
namespace {
struct Anchor {
    uint64_t frame{0}, ns{0};
    bool contiguous{false};
};
struct Record {
    std::atomic<uint64_t> stamp{0}, frame{0}, ns{0};
    std::atomic<bool> contiguous{false};
};
bool supportedNominal(double rate) noexcept {
    return std::isfinite(rate) && rate >= 8000 && rate <= 192000;
}
bool supportedMeasured(double rate, double nominal) noexcept {
    return std::isfinite(rate) && rate >= nominal * 0.995 && rate <= nominal * 1.005;
}
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
    // with silence. Unproven continuity and larger clock jumps still fail.
    const auto phaseError = std::abs(static_cast<long double>(frames) * 1e9L / nominal - elapsed);
    // Quad Cortex produces verified ~2.2-sample host-clock steps. Admit at
    // most three samples in a CLOSED interval with continuous source frames;
    // keep the slope limits and never extrapolate that corrected slope.
    return b.contiguous && phaseError <= 3e9L / nominal && rate >= nominal * 0.8 &&
                   rate <= nominal * 1.25
               ? rate
               : 0;
}
} // namespace
struct SampleClockHistory::Impl {
    const uint32_t capacity;
    std::unique_ptr<Record[]> records;
    std::atomic<uint64_t> begin{0}, end{0};
    explicit Impl(uint32_t count) : capacity(count), records(std::make_unique<Record[]>(count)) {}
    bool read(uint64_t ticket, Anchor& result) const noexcept {
        if (ticket == UINT64_MAX)
            return false;
        const auto& record = records[ticket % capacity];
        if (record.stamp.load() != ticket + 1)
            return false;
        result = {record.frame.load(), record.ns.load(), record.contiguous.load()};
        return record.stamp.load() == ticket + 1;
    }
};
SampleClockHistory::SampleClockHistory(uint32_t capacity) {
    if (capacity < 2 || capacity > kCapacity + 1)
        throw std::invalid_argument("Invalid sample clock history capacity");
    impl_ = std::make_unique<Impl>(capacity);
}
SampleClockHistory::~SampleClockHistory() = default;
bool SampleClockHistory::append(uint64_t frame, uint64_t hostNs,
                                bool contiguousSourceFrames) noexcept {
    auto& self = *impl_;
    const auto ticket = self.end.load();
    if (!hostNs || ticket == UINT64_MAX - 1)
        return false;
    if (ticket > self.begin.load()) {
        Anchor previous;
        if (!self.read(ticket - 1, previous) || frame <= previous.frame || hostNs <= previous.ns)
            return false;
    }
    auto& record = self.records[ticket % self.capacity];
    record.stamp.store(0);
    record.frame.store(frame);
    record.ns.store(hostNs);
    record.contiguous.store(contiguousSourceFrames);
    record.stamp.store(ticket + 1);
    self.end.store(ticket + 1);
    return true;
}
void SampleClockHistory::reset() noexcept {
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
