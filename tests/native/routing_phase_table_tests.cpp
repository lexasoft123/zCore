// The phase-table reader against the exact SincKernel reader and against the
// kernel's own definition. Every acceptance check is also run on a deliberately
// degraded input (a coarse table, or naive placement) and must reject it.
#include <zcore/routing/audio_history.h>
#include <zcore/routing/audio_resampler.h>
#include <zcore/routing/sinc_kernel.h>
#include <zcore/routing/sinc_phase_table.h>
#include "cached_fir.hpp"
#include "frame_positions.hpp"

#include <algorithm>
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <numbers>
#include <optional>
#include <span>
#include <stdexcept>
#include <string_view>
#include <utility>
#include <vector>

namespace {
bool trackHeap = false;
uint64_t heapEvents = 0;
} // namespace
#if defined(__GNUC__)
#define HEAP_HOOK __attribute__((noinline))
#else
#define HEAP_HOOK
#endif
HEAP_HOOK void* operator new(std::size_t bytes) {
    if (trackHeap)
        ++heapEvents;
    if (void* p = std::malloc(bytes ? bytes : 1))
        return p;
    throw std::bad_alloc();
}
HEAP_HOOK void* operator new[](std::size_t bytes) {
    return ::operator new(bytes);
}
HEAP_HOOK void operator delete(void* p) noexcept {
    std::free(p);
}
HEAP_HOOK void operator delete[](void* p) noexcept {
    std::free(p);
}
HEAP_HOOK void operator delete(void* p, std::size_t) noexcept {
    std::free(p);
}
HEAP_HOOK void operator delete[](void* p, std::size_t) noexcept {
    std::free(p);
}

#define CHECK(value)                                                                               \
    do {                                                                                           \
        if (!(value)) {                                                                            \
            std::fprintf(stderr, "CHECK %s:%d: %s\n", __FILE__, __LINE__, #value);                 \
            std::abort();                                                                          \
        }                                                                                          \
    } while (false)

namespace {
using namespace zcore::routing;

template <class F> auto noHeap(F operation) {
    const auto before = heapEvents;
    trackHeap = true;
    const auto result = operation();
    trackHeap = false;
    CHECK(heapEvents == before);
    return result;
}

// The kernel's definition in double precision, independent of both tables.
double idealKernel(double distance) {
    if (!(std::abs(distance) < SincKernel::kRadius))
        return 0;
    // I0(2 sqrt(z)) = sum z^k / (k!)^2.
    static const auto inverseSquares = [] {
        std::array<double, 64> values{};
        for (unsigned order = 1; order < values.size(); ++order)
            values[order] = 1 / (double(order) * order);
        return values;
    }();
    const auto bessel = [](double z) {
        double sum = 1, term = 1;
        for (unsigned order = 1; order < inverseSquares.size(); ++order) {
            term *= z * inverseSquares[order];
            sum += term;
            if (term < 1e-17 * sum)
                break;
        }
        return sum;
    };
    static const double normalization = bessel(81.0 / 4);
    const auto x = std::numbers::pi_v<double> * distance;
    const auto sinc = std::abs(x) < 1e-8 ? 1.0 : std::sin(x) / x;
    const auto relative = distance / SincKernel::kRadius;
    return sinc * bessel(81.0 / 4 * (1 - relative * relative)) / normalization;
}
uint32_t radiusFor(double cutoff) {
    return static_cast<uint32_t>(std::ceil(SincKernel::kRadius / cutoff));
}
// The linear-phase reader's cutoff, as AudioResampler::read derives it.
double cutoffFor(double sourceRate, double outputRate) {
    return SincKernel::kCutoff * std::min(1.0, outputRate / sourceRate);
}
struct Random {
    uint64_t state = 0x243f6a8885a308d3ULL;
    uint64_t next() {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return state >> 11;
    }
    double unit() { // [0, 1)
        return static_cast<double>(next()) * 0x1p-53;
    }
    float sample() { // [-0.5, 0.5)
        return static_cast<float>(unit() - .5);
    }
};

// ---- Weights ----------------------------------------------------------------

struct WeightError {
    double ideal = 0, lookup = 0, sum = 0;
};
// Largest weight error of a table against the kernel's definition and against
// SincKernel's lookup, over fractions and cutoffs it covers. Also checks the
// exact contract every reader relies on: support, padding and the sum.
WeightError weightError(const SincPhaseTable& table, bool strict) {
    const auto& kernel = SincKernel::prepared();
    std::array<float, SincPhaseTable::kMaximumTaps + 16> weights{};
    WeightError worst;
    Random random;
    // Either side of the table's cutoff, including both sides of the slope at
    // which a reader stops sharing one phase cell across half the filter.
    for (double scale : {0.0, 1e-9, 1e-6, 3e-5, 1e-4, 6.5e-4, 7e-4, 1e-3, 3e-3, 9.9e-3})
        for (double sign : {1.0, -1.0}) {
            const double cutoff = table.cutoff() * (1 - sign * scale);
            if (cutoff > 1 || !table.covers(cutoff) || (sign < 0 && scale == 0))
                continue;
            const auto radius = radiusFor(cutoff);
            const uint32_t taps = 2 * radius + 1, padded = SincPhaseTable::paddedTaps(radius);
            for (unsigned trial = 0; trial < 200; ++trial) {
                const double fraction = trial == 0   ? 0.0
                                        : trial == 1 ? 1.0
                                        : trial == 2 ? std::nextafter(1.0, 0.0)
                                        : trial == 3 ? std::numeric_limits<double>::min()
                                        : trial == 4 ? .5
                                                     : random.unit();
                weights.fill(7);
                const auto sum = noHeap([&] {
                    return table.weights(radius, fraction, cutoff,
                                         std::span<float>(weights.data(), padded));
                });
                double expected = 0, actual = 0;
                for (uint32_t tap = 0; tap < taps; ++tap) {
                    const double distance = (double(tap) - radius) - fraction;
                    const double lookup = kernel.coefficient(distance, cutoff);
                    CHECK(std::isfinite(weights[tap]));
                    // The kernel's support is exactly SincKernel's.
                    if (std::abs(distance) * cutoff >= SincKernel::kRadius)
                        CHECK(weights[tap] == 0 && lookup == 0);
                    worst.ideal = std::max(worst.ideal,
                                           std::abs(weights[tap] - idealKernel(distance * cutoff)));
                    worst.lookup = std::max(worst.lookup, std::abs(weights[tap] - lookup));
                    expected += lookup;
                    actual += weights[tap];
                }
                for (uint32_t tap = taps; tap < padded; ++tap)
                    CHECK(weights[tap] == 0);
                CHECK(weights[padded] == 7);
                worst.sum = std::max(worst.sum, std::abs(sum - actual));
                if (strict)
                    CHECK(std::abs(sum - expected) < 2e-6);
            }
        }
    return worst;
}
void weightsMatchKernel() {
    const auto& tables = SincPhaseTables::prepared();
    const double reduced = SincKernel::kCutoff * (44100.0 / 48000.0);
    for (double cutoff : {SincKernel::kCutoff, reduced}) {
        const auto* table = tables.find(cutoff);
        CHECK(table && table->cutoff() == cutoff);
        const auto error = weightError(*table, true);
        // Single-precision weights: the bound is rounding, not the fit.
        CHECK(error.ideal < 1e-7);
        // SincKernel's own interpolated lookup is 1.2e-7 from the definition.
        CHECK(error.lookup < 2.5e-7);
        CHECK(error.sum < 2e-7);
        // Negative control: two phase cells instead of thirty-two fail the
        // same bound by orders of magnitude.
        const auto coarse = std::make_unique<SincPhaseTable>(cutoff, 2);
        const auto coarseError = weightError(*coarse, false);
        CHECK(coarseError.ideal > 100 * 1e-7);
        std::printf("Phase table %.6f: weights within %.3g of the kernel, %.3g of the lookup; "
                    "coarse control %.3g\n",
                    cutoff, error.ideal, error.lookup, coarseError.ideal);
    }
    // Lookup bounds: one table per cutoff neighbourhood, none elsewhere.
    CHECK(tables.find(SincKernel::kCutoff * .9901) == tables.find(SincKernel::kCutoff));
    CHECK(tables.find(reduced * 1.0099) == tables.find(reduced));
    CHECK(tables.find(reduced * .9901) == tables.find(reduced));
    CHECK(tables.find(SincKernel::kCutoff) != tables.find(reduced));
    for (double cutoff : {SincKernel::kCutoff * .98, reduced * 1.02, reduced * .98, .5, 1.0, 0.0,
                          -1.0, double(NAN), double(INFINITY)})
        CHECK(!tables.find(cutoff));
    // Requests outside a table return no weights, so callers keep SincKernel.
    const auto& unity = *tables.find(SincKernel::kCutoff);
    std::array<float, 544> weights{};
    for (double fraction : {-1e-9, 1.0000001, double(NAN), double(INFINITY)})
        CHECK(unity.weights(103, fraction, SincKernel::kCutoff, weights) == 0);
    CHECK(unity.weights(103, .5, .5, weights) == 0);
    CHECK(unity.weights(300, .5, SincKernel::kCutoff, weights) == 0);
    CHECK(unity.weights(103, .5, SincKernel::kCutoff, std::span<float>(weights.data(), 207)) == 0);
    CHECK(unity.convolve(300, .5, SincKernel::kCutoff, weights.data()).weights == 0);
    CHECK(unity.convolve(103, 2, SincKernel::kCutoff, weights.data(), weights.data()).weights == 0);
    for (uint32_t cells : {0u, 3u, 64u})
        try {
            const auto invalid = std::make_unique<SincPhaseTable>(SincKernel::kCutoff, cells);
            CHECK(false);
        } catch (const std::invalid_argument&) {
        }
    for (double cutoff : {0.0, -.5, 1.5, .3, double(NAN)})
        try {
            const auto invalid = std::make_unique<SincPhaseTable>(cutoff);
            CHECK(false);
        } catch (const std::invalid_argument&) {
        }
}
// The fused readers are the stored weights applied to the same planes.
void convolveMatchesWeights() {
    const auto& tables = SincPhaseTables::prepared();
    Random random;
    std::array<float, 600> a{}, b{}, weights{};
    for (auto& value : a)
        value = random.sample();
    for (auto& value : b)
        value = random.sample();
    double worst = 0;
    for (double base : {SincKernel::kCutoff, SincKernel::kCutoff * (44100.0 / 48000.0)})
        for (double scale : {0.0, 2e-5, 9e-4, -4e-3, 8e-3}) {
            const double cutoff = base * (1 - scale);
            const auto* table = tables.find(cutoff);
            if (!table)
                continue;
            const auto radius = radiusFor(cutoff);
            const auto padded = SincPhaseTable::paddedTaps(radius);
            for (unsigned trial = 0; trial < 200; ++trial) {
                const double fraction = random.unit();
                const uint32_t begin = static_cast<uint32_t>(random.next() % 32);
                const auto sum = table->weights(radius, fraction, cutoff, weights);
                const auto pair = noHeap([&] {
                    return table->convolve(radius, fraction, cutoff, a.data() + begin,
                                           b.data() + begin);
                });
                const auto single = noHeap(
                    [&] { return table->convolve(radius, fraction, cutoff, a.data() + begin); });
                const auto dots = SincPhaseTable::dotPair(a.data() + begin, b.data() + begin,
                                                          weights.data(), padded);
                const auto dot = SincPhaseTable::dot(b.data() + begin, weights.data(), padded);
                double expectedA = 0, expectedB = 0;
                for (uint32_t tap = 0; tap < padded; ++tap) {
                    expectedA += double(weights[tap]) * a[begin + tap];
                    expectedB += double(weights[tap]) * b[begin + tap];
                }
                for (double error : {pair.a - expectedA, pair.b - expectedB, single.a - expectedA,
                                     dots[0] - expectedA, dots[1] - expectedB, dot - expectedB,
                                     pair.weights - sum, single.weights - sum})
                    worst = std::max(worst, std::abs(error));
            }
        }
    // Single-precision accumulation of about 200 products of magnitude <= 0.5.
    CHECK(worst < 2e-7);
    std::printf("Fused sums within %.3g of the stored weights in double\n", worst);
}

// ---- Frame placement --------------------------------------------------------

// Places frames with plain double arithmetic: the negative control.
struct NaivePositions {
    uint64_t anchor;
    double offset, step;
    std::optional<detail::FramePositions::Position> at(uint32_t frame) const {
        const double value = offset + frame * step;
        const double whole = std::floor(value);
        return detail::FramePositions::Position{anchor + static_cast<uint64_t>(whole),
                                                value - whole};
    }
};
// Offsets and steps that are multiples of 2^-40 have exact positions in
// 64-bit integers. Returns how many frames a placement gets wrong.
template <class Make> uint64_t placementErrors(Make&& make) {
    constexpr uint64_t anchor = 1000000;
    constexpr int bits = 40;
    Random random;
    uint64_t errors = 0;
    for (unsigned trial = 0; trial < 4000; ++trial) {
        // Up to 2^12 frames of offset, steps from 0.25 to 31 with 45 bits.
        const auto offsetUnits =
            static_cast<int64_t>(random.next() >> 1) % (int64_t{1} << (bits + 12));
        const auto stepUnits = (int64_t{1} << (bits - 2)) +
                               static_cast<int64_t>(random.next() % ((int64_t{31} << bits) -
                                                                     (int64_t{1} << (bits - 2))));
        const long double offset = std::ldexp(static_cast<long double>(offsetUnits), -bits);
        const double step = std::ldexp(static_cast<double>(stepUnits), -bits);
        const auto positions = make(anchor, offset, step);
        for (uint32_t frame : {0u, 1u, 2u, 31u, 100u, 255u, 256u, 2047u}) {
            const int64_t units = offsetUnits + static_cast<int64_t>(frame) * stepUnits;
            const uint64_t center = anchor + static_cast<uint64_t>(units >> bits);
            const double fraction =
                std::ldexp(static_cast<double>(units & ((int64_t{1} << bits) - 1)), -bits);
            const auto position = positions.at(frame);
            errors += !position || position->center != center || position->fraction != fraction;
        }
    }
    return errors;
}
void framePlacementIsExact() {
    using detail::FramePositions;
    CHECK(placementErrors([](uint64_t anchor, long double offset, double step) {
              return FramePositions(anchor, offset, step);
          }) == 0);
    // Negative control: one rounding per frame misplaces many of them.
    const auto naive = placementErrors([](uint64_t anchor, long double offset, double step) {
        return NaivePositions{anchor, static_cast<double>(offset), step};
    });
    CHECK(naive > 1000);

    // The reference sum in long double, wherever it carries enough bits to be
    // exact here; random steps and offsets, including negative ones.
    Random random;
    uint64_t compared = 0;
    for (unsigned trial = 0; trial < 20000; ++trial) {
        const uint64_t anchor = trial % 3 == 0 ? 5000 : 1000000000000ULL;
        const long double offset = (static_cast<long double>(random.unit()) - .5L) * 9000;
        const double step = trial % 4 == 0 ? 48000.0 / 44100 : .25 + 30 * random.unit();
        const FramePositions positions(anchor, offset, step);
        for (uint32_t frame : {0u, 1u, 77u, 255u}) {
            const auto sum = offset + static_cast<long double>(frame) * step;
            const auto whole = std::floor(sum);
            const auto position = positions.at(frame);
            if (whole < -static_cast<long double>(anchor)) {
                CHECK(!position);
                continue;
            }
            CHECK(position);
            const auto center = whole < 0 ? anchor - static_cast<uint64_t>(-whole)
                                          : anchor + static_cast<uint64_t>(whole);
            const auto fraction = static_cast<double>(sum - whole);
            CHECK(position->fraction >= 0 && position->fraction <= 1);
            // Same source position; a fraction that rounds to one may instead
            // be zero at the next frame.
            const long double difference =
                (static_cast<long double>(position->center) - static_cast<long double>(center)) +
                (static_cast<long double>(position->fraction) - fraction);
            const long double tolerance = LDBL_MANT_DIG >= 113  ? 3e-16L
                                          : LDBL_MANT_DIG >= 64 ? 1e-15L
                                                                : 4e-12L;
            CHECK(std::abs(difference) <= tolerance);
            ++compared;
        }
    }
    CHECK(compared > 70000);
    // A position that rounds up to a whole frame it has not reached keeps the
    // frame below, with a fraction just under one; just past a whole frame it
    // keeps the excess. Both sums are exact in a pair of doubles.
    const FramePositions under(1000, 0x1.8p-52L, std::nextafter(3.0, 0.0));
    CHECK(under.at(1)->center == 1002 && under.at(1)->fraction == std::nextafter(1.0, 0.0));
    const FramePositions over(1000, 0x1p-60L, 3);
    CHECK(over.at(1)->center == 1003 && over.at(1)->fraction == 0x1p-60);
    // Limits beside the anchor match the direct sum: before the first source
    // frame, and at the top of the frame counter.
    CHECK(!FramePositions(10, -10.5L, 1).at(0));
    CHECK(FramePositions(10, -10.0L, 1).at(0)->center == 0);
    CHECK(FramePositions(10, -10.5L, 1).at(1)->center == 0);
    CHECK(FramePositions(UINT64_MAX - 8, 0.25L, 1).at(6)->center == UINT64_MAX - 2);
    CHECK(!FramePositions(UINT64_MAX - 8, 0.25L, 1).at(7));
    // An anchor at the counter's limit has nothing at or above it.
    CHECK(!FramePositions(UINT64_MAX, 0.5L, 1).at(0));
    CHECK(FramePositions(UINT64_MAX, -1.5L, 1).at(0)->center == UINT64_MAX - 2);
    // Arguments outside the reduced form use the direct sum.
    const FramePositions far((uint64_t{1} << 63) + 100, -0x1p63L, 1);
    CHECK(far.at(0) && far.at(0)->center == 100 && far.at(0)->fraction == 0);
    CHECK(!FramePositions(5, static_cast<long double>(INFINITY), 1).at(0));
    CHECK(!FramePositions(UINT64_MAX - 1, -0x1p64L, 1).at(0));
    CHECK(!FramePositions(0, 0x1p64L, 1).at(0));
    // The limits are compared as integers. A 53-bit long double would round
    // this room down and this anchor up: the last usable frame is still
    // reached, and a frame before the first is still refused.
    const auto top = FramePositions((uint64_t{1} << 63) - 3, 0x1p63L, 1).at(0);
    CHECK(top && top->center == UINT64_MAX - 2 && top->fraction == 0);
    CHECK(!FramePositions((uint64_t{1} << 63) + 2047, -0x1.0000000000001p63L, 1).at(0));
    std::printf("Frame placement exact on %llu dyadic and %llu reference frames; naive control "
                "misplaces %llu\n",
                4000ULL * 8, static_cast<unsigned long long>(compared),
                static_cast<unsigned long long>(naive));
}

// ---- Resampled audio --------------------------------------------------------

enum class Wave { Noise, Sine997, Sine10007, Sine19997, Sweep };
const char* name(Wave wave) {
    switch (wave) {
    case Wave::Noise:
        return "noise";
    case Wave::Sine997:
        return "997 Hz";
    case Wave::Sine10007:
        return "10007 Hz";
    case Wave::Sine19997:
        return "19997 Hz";
    case Wave::Sweep:
        return "sweep";
    }
    return "";
}
struct Signal {
    static constexpr uint64_t origin = 1000000000000ULL;
    static constexpr uint32_t length = 8000;
    uint32_t channels;
    std::vector<float> samples; // interleaved
    Signal(Wave wave, uint32_t count, double rate)
        : channels(count), samples(size_t(length) * count) {
        Random random;
        for (uint32_t frame = 0; frame < length; ++frame)
            for (uint32_t channel = 0; channel < channels; ++channel) {
                const double time = frame / rate;
                double value = 0;
                switch (wave) {
                case Wave::Noise:
                    value = random.sample();
                    break;
                case Wave::Sine997:
                    value = .5 * std::sin(2 * std::numbers::pi * 997 * time + channel);
                    break;
                case Wave::Sine10007:
                    value = .5 * std::sin(2 * std::numbers::pi * 10007 * time + channel);
                    break;
                case Wave::Sine19997:
                    value = .5 * std::sin(2 * std::numbers::pi * 19997 * time + channel);
                    break;
                case Wave::Sweep: {
                    // 20 Hz to 20 kHz, linear in time, ending just past the
                    // last frame any stream reads: half of it is above 10 kHz.
                    const double span = 5300 / rate;
                    value =
                        .5 * std::sin(2 * std::numbers::pi *
                                          (20 * time + (20000 - 20) * time * time / (2 * span)) +
                                      channel);
                    break;
                }
                }
                samples[size_t(frame) * channels + channel] = static_cast<float>(value);
            }
    }
    ResampleFrame frame(uint64_t position, std::span<float> values) const noexcept {
        if (position < origin || position - origin >= length)
            return {false, 0};
        for (uint32_t channel = 0; channel < channels; ++channel)
            values[channel] = samples[(position - origin) * channels + channel];
        return {true, channels};
    }
    static ResampleFrame read(const void* context, uint64_t position,
                              std::span<float> values) noexcept {
        return static_cast<const Signal*>(context)->frame(position, values);
    }
};
// A stream of blocks whose source rate moves between blocks, as measured
// device clocks do, while its phase stays continuous.
struct Stream {
    const char* name;
    double sourceRate, outputRate;
    double offsetPpm = 0, wanderPpm = 0, jitterPpm = 0;
    uint32_t frames = 256;
};
struct Quality {
    double exactDb, idealDb, lookupIdealDb, peak;
    uint64_t different;
};
double decibels(double energy, double error) {
    return 10 * std::log10(energy / std::max(error, 1e-300));
}
// Runs a stream through read() and compares every frame with the
// double-precision sum of SincKernel's weights, and every sixteenth frame
// with the sum of the kernel's definition.
template <class Read>
Quality measure(const Stream& stream, Wave wave, uint32_t channels, Read&& read) {
    const Signal signal(wave, channels, stream.sourceRate);
    const auto& kernel = SincKernel::prepared();
    std::array<std::array<float, 260>, 16> planes{};
    std::array<float*, 16> pointers{};
    for (uint32_t channel = 0; channel < channels; ++channel)
        pointers[channel] = planes[channel].data();
    Random random;
    long double offset = 600.375L;
    double exactError = 0, exactEnergy = 0, idealError = 0, lookupIdealError = 0, idealEnergy = 0,
           peak = 0;
    uint64_t different = 0;
    std::array<double, 16> exact{}, ideal{};
    std::array<double, 512> lookup{};
    for (uint32_t block = 0; block < 16; ++block) {
        const double ppm = stream.offsetPpm +
                           stream.wanderPpm * std::sin(2 * std::numbers::pi * block / 17.0) +
                           stream.jitterPpm * (2 * random.unit() - 1);
        ResamplePosition position{};
        position.anchor = Signal::origin;
        position.offset = offset;
        position.sourceRate = stream.sourceRate * (1 + ppm * 1e-6);
        position.outputRate = stream.outputRate;
        const double step = position.sourceRate / position.outputRate;
        const double cutoff = cutoffFor(position.sourceRate, position.outputRate);
        const auto radius = radiusFor(cutoff);
        for (auto& plane : planes)
            plane.fill(99);
        const ResampleOutput output{std::span<float* const>(pointers.data(), channels),
                                    stream.frames};
        CHECK(noHeap([&] { return read(position, output, stream.frames, signal, cutoff); }) == 0);
        for (uint32_t frame = 0; frame < stream.frames; ++frame) {
            const auto place = position.offset + static_cast<long double>(frame) * step;
            const auto whole = std::floor(place);
            const auto center = Signal::origin + static_cast<uint64_t>(whole);
            const auto fraction = static_cast<double>(place - whole);
            const bool defined = frame % 16 == 5;
            exact.fill(0);
            ideal.fill(0);
            // The exact reader's weights: SincKernel::coefficient for each tap.
            const double exactSum = kernel.fillCoefficients(
                radius, fraction, cutoff, std::span<double>(lookup.data(), 2 * radius + 1));
            double idealSum = 0;
            std::array<float, 16> source{};
            for (uint32_t tap = 0; tap <= 2 * radius; ++tap) {
                const double definition =
                    defined ? idealKernel(((double(tap) - radius) - fraction) * cutoff) : 0;
                CHECK(signal.frame(center - radius + tap, source).valid);
                idealSum += definition;
                for (uint32_t channel = 0; channel < channels; ++channel) {
                    exact[channel] += lookup[tap] * source[channel];
                    ideal[channel] += definition * source[channel];
                }
            }
            for (uint32_t channel = 0; channel < channels; ++channel) {
                const double actual = planes[channel][frame];
                const double expected = exact[channel] / exactSum;
                CHECK(std::isfinite(actual));
                exactError += (actual - expected) * (actual - expected);
                exactEnergy += expected * expected;
                peak = std::max(peak, std::abs(actual - expected));
                different += static_cast<float>(actual) != static_cast<float>(expected);
                if (defined) {
                    const double definition = ideal[channel] / idealSum;
                    idealError += (actual - definition) * (actual - definition);
                    // What the exact reader itself loses, once its sum is a float.
                    const double rounded = static_cast<float>(expected);
                    lookupIdealError += (rounded - definition) * (rounded - definition);
                    idealEnergy += definition * definition;
                }
            }
        }
        for (uint32_t channel = 0; channel < channels; ++channel)
            CHECK(planes[channel][stream.frames] == 99);
        offset += static_cast<long double>(stream.frames) * step;
    }
    return {decibels(exactEnergy, exactError), decibels(idealEnergy, idealError),
            decibels(idealEnergy, lookupIdealError), peak, different};
}
const auto publicReader = [](const AudioResampler& resampler) {
    return [&resampler](const ResamplePosition& position, const ResampleOutput& output,
                        uint32_t frames, const Signal& signal, double) {
        return resampler.read(position, output, frames, Signal::read, &signal);
    };
};
// The cached reader with a chosen table, or with none for the exact weights.
const auto tableReader = [](const SincPhaseTable* table) {
    return [table](const ResamplePosition& position, const ResampleOutput& output, uint32_t frames,
                   const Signal& signal, double cutoff) {
        const auto provider = [&](uint64_t at, std::span<float> values) {
            return signal.frame(at, values);
        };
        const auto missing =
            output.channels.size() <= 2
                ? CachedFir::readWindow<512, 2, false>(position, output, frames,
                                                       SincKernel::prepared(), cutoff, provider,
                                                       std::span<uint8_t>{}, table)
                : CachedFir::readWindow<512, 16, false>(position, output, frames,
                                                        SincKernel::prepared(), cutoff, provider,
                                                        std::span<uint8_t>{}, table);
        CHECK(missing);
        return *missing;
    };
};
// Minimum signal-to-difference ratio against the exact reader, in dB. The
// difference is SincKernel's own lookup error, which grows with frequency;
// measured values are 5 dB or more above these on AArch64 and x64.
double requiredExactDb(Wave wave) {
    switch (wave) {
    case Wave::Sine997:
        return 142;
    case Wave::Sine10007:
        return 136;
    case Wave::Sweep:
        return 134;
    case Wave::Noise:
        return 133;
    case Wave::Sine19997:
        return 129;
    }
    return 0;
}
void resampledAudioMatchesExactReader(bool printEveryCase) {
    const AudioResampler resampler;
    const auto& tables = SincPhaseTables::prepared();
    const bool verbose = printEveryCase;
    // Fixed ratios, then source clocks that wander and jitter between blocks
    // (ppm: offset, slow wander, per-block jitter) across the tables' range.
    const Stream streams[] = {
        {"48k->48k +100 ppm", 48000, 48000, -100},
        {"48k->48k -100 ppm", 48000, 48000, 100},
        {"48k->48k +1000 ppm", 48000, 48000, -1000},
        {"48k->48k -1000 ppm", 48000, 48000, 1000},
        {"48k->48k drifting", 48000, 48000, 0, 400, 300},
        {"48k->48k drifting wide", 48000, 48000, 2000, 4000, 2500},
        {"44.1k->48k", 44100, 48000},
        {"44.1k->48k drifting", 44100, 48000, 0, 600, 400},
        {"48k->44.1k", 48000, 44100},
        {"48k->44.1k drifting", 48000, 44100, 0, 1000, 800},
        {"48k->48k -100 ppm short", 48000, 48000, 100, 0, 0, 48},
    };
    const std::unique_ptr<const SincPhaseTable> coarse[] = {
        std::make_unique<SincPhaseTable>(SincKernel::kCutoff, 2),
        std::make_unique<SincPhaseTable>(SincKernel::kCutoff * (44100.0 / 48000.0), 2)};
    const auto withTable = [](auto&& choose) {
        return [choose](const ResamplePosition& position, const ResampleOutput& output,
                        uint32_t frames, const Signal& signal, double cutoff) {
            return tableReader(choose(cutoff))(position, output, frames, signal, cutoff);
        };
    };
    double margin = 1e9, worstIdeal = 1e9, worstPeak = 0, bestCoarse = 0;
    for (const auto& stream : streams)
        for (Wave wave :
             {Wave::Noise, Wave::Sine997, Wave::Sine10007, Wave::Sine19997, Wave::Sweep})
            for (uint32_t channels : {2u, 1u, 3u}) {
                // One and three channels (the single-plane and stored-weight
                // readers) on noise; three keep the exact reader below 128 frames.
                if (channels != 2 &&
                    (wave != Wave::Noise || (channels == 3 && stream.frames < 128)))
                    continue;
                const auto quality = measure(stream, wave, channels, publicReader(resampler));
                if (verbose)
                    std::printf("  %-24s %-8s %uch: vs exact %.1f dB (peak %.3g), vs kernel "
                                "%.1f dB, exact vs kernel %.1f dB\n",
                                stream.name, name(wave), channels, quality.exactDb, quality.peak,
                                quality.idealDb, quality.lookupIdealDb);
                CHECK(quality.exactDb > requiredExactDb(wave));
                CHECK(quality.peak < 4e-7);
                // As faithful to the kernel's definition as the exact reader,
                // to within the rounding of single-precision sums.
                CHECK(quality.idealDb > 140 && quality.idealDb > quality.lookupIdealDb - 9);
                margin = std::min(margin, quality.exactDb - requiredExactDb(wave));
                worstIdeal = std::min(worstIdeal, quality.idealDb);
                worstPeak = std::max(worstPeak, quality.peak);
                if (wave == Wave::Sine997 || wave == Wave::Sine19997 ||
                    (channels != 2 && stream.offsetPpm != 100))
                    continue;
                // The public reader is the cached reader with the prepared
                // table, frame for frame, and not the exact weights, which
                // round to the reference almost everywhere.
                const auto direct =
                    measure(stream, wave, channels,
                            withTable([&](double cutoff) { return tables.find(cutoff); }));
                CHECK(direct.exactDb == quality.exactDb && direct.peak == quality.peak &&
                      direct.different == quality.different);
                const auto exact = measure(stream, wave, channels, tableReader(nullptr));
                CHECK(exact.exactDb > 148 && exact.peak < 8e-8);
                CHECK(quality.different > 20 * exact.different + 100);
                // Negative control: two phase cells per table instead of
                // thirty-two. Low frequencies survive that, so the sweep is
                // judged where it reaches its upper half.
                const auto control = measure(stream, wave, channels, withTable([&](double cutoff) {
                                                 const auto* table =
                                                     coarse[0]->covers(cutoff)   ? coarse[0].get()
                                                     : coarse[1]->covers(cutoff) ? coarse[1].get()
                                                                                 : nullptr;
                                                 CHECK(table);
                                                 return table;
                                             }));
                if (verbose)
                    std::printf(
                        "  %-24s %-8s %uch: exact weights %.1f dB, coarse control %.1f dB\n",
                        stream.name, name(wave), channels, exact.exactDb, control.exactDb);
                if (wave != Wave::Sweep || stream.frames >= 256) {
                    CHECK(control.exactDb < requiredExactDb(wave) - 6);
                    bestCoarse = std::max(bestCoarse, control.exactDb);
                }
            }
    std::printf("Table reader vs exact reader: at least %.1f dB above the required SNR, peak "
                "%.3g; vs kernel definition %.1f dB; coarse control at most %.1f dB\n",
                margin, worstPeak, worstIdeal, bestCoarse);
}
// Total harmonic distortion plus noise of a resampled sine: everything that
// is not the tone, found by a least-squares fit of its sine and cosine.
double thdNoiseDb(const std::vector<double>& samples, double cyclesPerSample) {
    double ss = 0, cc = 0, sc = 0, xs = 0, xc = 0, energy = 0;
    for (size_t index = 0; index < samples.size(); ++index) {
        const double s = std::sin(2 * std::numbers::pi * cyclesPerSample * index);
        const double c = std::cos(2 * std::numbers::pi * cyclesPerSample * index);
        ss += s * s, cc += c * c, sc += s * c;
        xs += samples[index] * s, xc += samples[index] * c;
    }
    const double determinant = ss * cc - sc * sc;
    const double a = (xs * cc - xc * sc) / determinant, b = (xc * ss - xs * sc) / determinant;
    double residual = 0;
    for (size_t index = 0; index < samples.size(); ++index) {
        const double tone = a * std::sin(2 * std::numbers::pi * cyclesPerSample * index) +
                            b * std::cos(2 * std::numbers::pi * cyclesPerSample * index);
        residual += (samples[index] - tone) * (samples[index] - tone);
        energy += tone * tone;
    }
    return 10 * std::log10(std::max(residual, 1e-300) / energy);
}
void sineDistortionIsUnchanged() {
    const AudioResampler resampler;
    const std::unique_ptr<const SincPhaseTable> coarse[] = {
        std::make_unique<SincPhaseTable>(SincKernel::kCutoff, 2),
        std::make_unique<SincPhaseTable>(SincKernel::kCutoff * (44100.0 / 48000.0), 2)};
    struct Case {
        const char* name;
        double sourceRate, outputRate, hertz;
    };
    for (const auto& test : {Case{"48k->48k -200 ppm 997 Hz", 48000 * 1.0002, 48000, 997},
                             Case{"48k->48k -200 ppm 10007 Hz", 48000 * 1.0002, 48000, 10007},
                             Case{"48k->48k +200 ppm 19997 Hz", 48000, 48000 * 1.0002, 19997},
                             Case{"44.1k->48k 10007 Hz", 44100, 48000, 10007},
                             Case{"48k->44.1k 10007 Hz", 48000, 44100, 10007},
                             Case{"48k->44.1k -1000 ppm 997 Hz", 48000 * 1.001, 44100, 997}}) {
        const Wave wave = test.hertz == 997     ? Wave::Sine997
                          : test.hertz == 10007 ? Wave::Sine10007
                                                : Wave::Sine19997;
        // The signal is generated at its nominal source rate; the tone seen at
        // the output is scaled by both clocks.
        const double nominal = test.sourceRate > 46000 ? 48000 : 44100;
        const Signal signal(wave, 1, nominal);
        const double step = test.sourceRate / test.outputRate;
        const double cutoff = cutoffFor(test.sourceRate, test.outputRate);
        const auto run = [&](auto&& read) {
            std::vector<double> output;
            std::array<float, 256> plane{};
            const std::array<float*, 1> pointers{plane.data()};
            long double offset = 600.375L;
            for (uint32_t block = 0; block < 24; ++block) {
                ResamplePosition position{};
                position.anchor = Signal::origin;
                position.offset = offset;
                position.sourceRate = test.sourceRate;
                position.outputRate = test.outputRate;
                CHECK(read(position, ResampleOutput{pointers, 256}, 256, signal, cutoff) == 0);
                output.insert(output.end(), plane.begin(), plane.end());
                offset += 256.0L * step;
            }
            return thdNoiseDb(output, test.hertz / nominal * step);
        };
        const double table = run(publicReader(resampler));
        const double exact = run(tableReader(nullptr));
        const double control = run(tableReader(coarse[cutoff < .9 ? 1 : 0].get()));
        std::printf("THD+N %-28s table %.1f dB, exact %.1f dB, coarse control %.1f dB\n", test.name,
                    table, exact, control);
        // What is left beside the tone is the kernel's own image rejection,
        // 108 to 128 dB down; the table reader must not add to it.
        CHECK(table < -105 && std::abs(table - exact) < .5);
        // Negative control. A coarse table is still accurate at 1 kHz, so it
        // is judged at 10 kHz and above.
        if (test.hertz > 10000)
            CHECK(control > table + 6);
    }
}

// ---- Fallbacks --------------------------------------------------------------

// Samples near the float limit overflow a single-precision sum; such frames
// must come from the exact reader, like every frame of an uncovered ratio.
void fallbacksKeepExactReader() {
    const AudioResampler resampler;
    struct Loud {
        float scale;
        static ResampleFrame read(const void* context, uint64_t position,
                                  std::span<float> values) noexcept {
            const float scale = static_cast<const Loud*>(context)->scale;
            for (size_t channel = 0; channel < values.size(); ++channel)
                values[channel] = (position + channel) % 5 < 3 ? scale : -scale * .5f;
            return {true, static_cast<uint32_t>(values.size())};
        }
    };
    for (float scale : {1e30f, 2.5e38f, std::numeric_limits<float>::max()})
        for (size_t channels : {size_t{1}, size_t{2}, size_t{3}}) {
            const Loud loud{scale};
            std::array<std::array<float, 256>, 3> block{}, tiles{};
            std::array<float*, 3> blockPointers{block[0].data(), block[1].data(), block[2].data()};
            ResamplePosition position{};
            position.anchor = 1000000;
            position.offset = 4096.375L;
            position.sourceRate = 48000 * 1.0002;
            position.outputRate = 48000;
            const double step = position.sourceRate / position.outputRate;
            CHECK(noHeap([&] {
                      return resampler.read(
                          position, {std::span<float* const>(blockPointers.data(), channels), 256},
                          256, Loud::read, &loud);
                  }) == 0);
            // Sixteen-frame reads never use the cached window or the tables.
            for (uint32_t begin = 0; begin < 256; begin += 16) {
                std::array<float*, 3> pointers{tiles[0].data() + begin, tiles[1].data() + begin,
                                               tiles[2].data() + begin};
                auto tile = position;
                tile.offset = position.offset + static_cast<long double>(begin) * step;
                CHECK(resampler.read(tile, {std::span<float* const>(pointers.data(), channels), 16},
                                     16, Loud::read, &loud) == 0);
            }
            uint32_t nonzero = 0;
            for (size_t channel = 0; channel < channels; ++channel)
                for (uint32_t frame = 0; frame < 256; ++frame) {
                    const double actual = block[channel][frame], expected = tiles[channel][frame];
                    CHECK(std::isfinite(actual));
                    // Finite where the exact reader is, and equal to it far
                    // more closely than float sums of such values can be.
                    CHECK((actual == 0) == (expected == 0));
                    CHECK(std::abs(actual - expected) <= std::abs(expected) * 1e-6);
                    nonzero += actual != 0;
                }
            CHECK(nonzero > 0);
        }
    // 48 kHz to 40 kHz has no table: bit-for-bit the exact cached reader.
    const Signal signal(Wave::Noise, 2, 48000);
    std::array<std::array<float, 64>, 2> first{}, second{};
    const std::array<float*, 2> a{first[0].data(), first[1].data()};
    const std::array<float*, 2> b{second[0].data(), second[1].data()};
    ResamplePosition position{};
    position.anchor = Signal::origin;
    position.offset = 800.625L;
    position.sourceRate = 48000;
    position.outputRate = 40000;
    const double cutoff = cutoffFor(position.sourceRate, position.outputRate);
    CHECK(!SincPhaseTables::prepared().find(cutoff));
    CHECK(resampler.read(position, {a, 64}, 64, Signal::read, &signal) == 0);
    CHECK(tableReader(nullptr)(position, {b, 64}, 64, signal, cutoff) == 0);
    for (uint32_t channel = 0; channel < 2; ++channel)
        for (uint32_t frame = 0; frame < 64; ++frame)
            CHECK(first[channel][frame] == second[channel][frame] && first[channel][frame] != 0);
    // A window holding a nonzero sample below -300 dB keeps the exact reader
    // (single-precision products of such samples go subnormal): bit for bit
    // the exact cached reader. The same signal 100 dB louder does not, which
    // shows that the comparison can tell the two readers apart.
    for (float scale : {1e-18f, 1e-8f}) {
        struct Faint {
            float scale;
            const Signal* signal;
            static ResampleFrame read(const void* context, uint64_t at,
                                      std::span<float> values) noexcept {
                const auto& self = *static_cast<const Faint*>(context);
                const auto result = self.signal->frame(at, values);
                for (uint32_t channel = 0; channel < self.signal->channels; ++channel)
                    values[channel] *= self.scale;
                return result;
            }
        };
        const Faint faint{scale, &signal};
        position.outputRate = 48000 * .9999;
        const double near = cutoffFor(position.sourceRate, position.outputRate);
        CHECK(SincPhaseTables::prepared().find(near));
        CHECK(noHeap([&] { return resampler.read(position, {a, 64}, 64, Faint::read, &faint); }) ==
              0);
        const auto provider = [&](uint64_t at, std::span<float> values) {
            return Faint::read(&faint, at, values);
        };
        const auto exact = CachedFir::readWindow<512, 2, false>(
            position, {b, 64}, 64, SincKernel::prepared(), near, provider, std::span<uint8_t>{});
        CHECK(exact && *exact == 0);
        uint32_t different = 0;
        for (uint32_t channel = 0; channel < 2; ++channel)
            for (uint32_t frame = 0; frame < 64; ++frame) {
                CHECK(first[channel][frame] != 0);
                different += first[channel][frame] != second[channel][frame];
            }
        CHECK(scale < CachedFir::kFaintSample ? different == 0 : different > 20);
    }
    // Silence in, exact silence out, on the table path.
    struct Quiet {
        static ResampleFrame read(const void*, uint64_t, std::span<float> values) noexcept {
            std::fill(values.begin(), values.end(), 0.0f);
            return {true, static_cast<uint32_t>(values.size())};
        }
    };
    first[0].fill(1);
    first[1].fill(1);
    position.outputRate = 48000 * 1.0001;
    CHECK(resampler.read(position, {a, 64}, 64, Quiet::read, nullptr) == 0);
    for (uint32_t frame = 0; frame < 64; ++frame)
        CHECK(first[0][frame] == 0 && first[1][frame] == 0);
}

// ---- Capture history --------------------------------------------------------

// Two tones through a capture history whose clock and reader each run 1000 ppm
// off nominal, in both directions: the path the output thread takes.
void historyFollowsDriftingClocks() {
    constexpr uint32_t nominal = 48000, captureFrames = 256, readFrames = 256;
    constexpr uint64_t origin = 9000000000ULL;
    // Seconds as double: a few seconds resolve to well below the nanosecond
    // timestamps, and quad sines would dominate the test on AArch64 Linux.
    const auto tone = [](double time, uint32_t channel) {
        return .25 * std::sin(2 * std::numbers::pi * (997 + 211 * channel) * time) +
               .2 * std::sin(2 * std::numbers::pi * 5003 * time + channel);
    };
    double worst = 1e9;
    for (const auto& [captureRate, readerRate] :
         {std::pair{nominal * 1.001, nominal * .999}, std::pair{nominal * .999, nominal * 1.001},
          std::pair{nominal * 1.00003, double(nominal)}}) {
        AudioHistory history(2);
        std::array<std::array<float, captureFrames>, 2> input{};
        std::array<std::array<float, readFrames>, 2> planes{};
        const std::array<const float*, 2> source{input[0].data(), input[1].data()};
        const std::array<float*, 2> destination{planes[0].data(), planes[1].data()};
        AudioReadCursor cursor;
        long double captured = 0, reader = 0;
        uint64_t sourceFrame = 0, checked = 0;
        double error = 0, energy = 0;
        for (uint32_t block = 0; block < 700; ++block) {
            for (uint32_t frame = 0; frame < captureFrames; ++frame)
                for (uint32_t channel = 0; channel < 2; ++channel)
                    input[channel][frame] = static_cast<float>(
                        tone(static_cast<double>(captured) + frame / captureRate, channel));
            CHECK(noHeap([&] {
                return history.append({source, captureFrames}, captureFrames,
                                      origin + static_cast<uint64_t>(captured * 1e9L), nominal,
                                      static_cast<double>(sourceFrame), true);
            }));
            sourceFrame += captureFrames;
            captured += static_cast<long double>(captureFrames) / captureRate;
            while (reader + static_cast<long double>(readFrames + 2048) / readerRate < captured) {
                const auto first = origin + static_cast<uint64_t>(reader * 1e9L);
                const auto result = noHeap([&] {
                    return history.read(first, readerRate, {destination, readFrames}, readFrames,
                                        cursor);
                });
                // The rate estimate needs about a second of capture to settle.
                if (block > 400) {
                    CHECK(result.online && result.missingFrames == 0);
                    for (uint32_t channel = 0; channel < 2; ++channel)
                        for (uint32_t frame = 0; frame < readFrames; ++frame) {
                            const double expected =
                                tone(static_cast<double>(first - origin) / 1e9 + frame / readerRate,
                                     channel);
                            const double actual = planes[channel][frame];
                            error += (actual - expected) * (actual - expected);
                            energy += expected * expected;
                        }
                    ++checked;
                }
                reader += static_cast<long double>(readFrames) / readerRate;
            }
        }
        CHECK(checked > 200);
        worst = std::min(worst, decibels(energy, error));
    }
    std::printf("History with clocks 1000 ppm apart: %.1f dB SNR against the analytic tones\n",
                worst);
    // Limited by nanosecond timestamps and the measured clock, as before.
    CHECK(worst > 85);
}
} // namespace

// Pass "verbose" to print the measurement of every stream and signal.
int main(int argc, char** argv) {
    weightsMatchKernel();
    convolveMatchesWeights();
    framePlacementIsExact();
    resampledAudioMatchesExactReader(argc > 1 && std::string_view(argv[1]) == "verbose");
    sineDistortionIsUnchanged();
    fallbacksKeepExactReader();
    historyFollowsDriftingClocks();
    std::puts("Phase table weights, frame placement, resampled audio and fallbacks passed");
}
