#pragma once

#include <zcore/routing/sinc_kernel.h>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <span>
#if defined(__aarch64__)
#include <arm_neon.h>
#elif defined(__SSE2__) || defined(_M_X64)
#include <emmintrin.h>
#endif

// The weight generator below is instantiated per caller with its accumulators
// in registers; an outlined copy would keep them in memory for every tap.
#if defined(_MSC_VER)
#define ZCORE_ROUTING_ALWAYS_INLINE __forceinline
#elif defined(__GNUC__)
#define ZCORE_ROUTING_ALWAYS_INLINE inline __attribute__((always_inline))
#else
#define ZCORE_ROUTING_ALWAYS_INLINE inline
#endif

namespace zcore::routing {
namespace detail {
// Four single-precision lanes: AArch64 Advanced SIMD, baseline x64 SSE2, or a
// portable array. multiplyAdd4(a, b, c) is a + b * c; AArch64 fuses it.
#if defined(__aarch64__)
using Float4 = float32x4_t;
inline Float4 load4(const float* values) noexcept {
    return vld1q_f32(values);
}
inline void store4(float* values, Float4 value) noexcept {
    vst1q_f32(values, value);
}
inline Float4 splat4(float value) noexcept {
    return vdupq_n_f32(value);
}
inline Float4 add4(Float4 a, Float4 b) noexcept {
    return vaddq_f32(a, b);
}
inline Float4 multiply4(Float4 a, Float4 b) noexcept {
    return vmulq_f32(a, b);
}
inline Float4 multiplyAdd4(Float4 a, Float4 b, Float4 c) noexcept {
    return vfmaq_f32(a, b, c);
}
inline double sum4(Float4 value) noexcept {
    return vaddvq_f64(vaddq_f64(vcvt_f64_f32(vget_low_f32(value)), vcvt_high_f64_f32(value)));
}
#elif defined(__SSE2__) || defined(_M_X64)
using Float4 = __m128;
inline Float4 load4(const float* values) noexcept {
    return _mm_loadu_ps(values);
}
inline void store4(float* values, Float4 value) noexcept {
    _mm_storeu_ps(values, value);
}
inline Float4 splat4(float value) noexcept {
    return _mm_set1_ps(value);
}
inline Float4 add4(Float4 a, Float4 b) noexcept {
    return _mm_add_ps(a, b);
}
inline Float4 multiply4(Float4 a, Float4 b) noexcept {
    return _mm_mul_ps(a, b);
}
inline Float4 multiplyAdd4(Float4 a, Float4 b, Float4 c) noexcept {
    return _mm_add_ps(a, _mm_mul_ps(b, c));
}
inline double sum4(Float4 value) noexcept {
    const auto pairs = _mm_add_pd(_mm_cvtps_pd(value), _mm_cvtps_pd(_mm_movehl_ps(value, value)));
    return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
}
#else
struct Float4 {
    float lane[4];
};
inline Float4 load4(const float* values) noexcept {
    return {{values[0], values[1], values[2], values[3]}};
}
inline void store4(float* values, Float4 value) noexcept {
    for (int lane = 0; lane < 4; ++lane)
        values[lane] = value.lane[lane];
}
inline Float4 splat4(float value) noexcept {
    return {{value, value, value, value}};
}
inline Float4 add4(Float4 a, Float4 b) noexcept {
    for (int lane = 0; lane < 4; ++lane)
        a.lane[lane] += b.lane[lane];
    return a;
}
inline Float4 multiply4(Float4 a, Float4 b) noexcept {
    for (int lane = 0; lane < 4; ++lane)
        a.lane[lane] *= b.lane[lane];
    return a;
}
inline Float4 multiplyAdd4(Float4 a, Float4 b, Float4 c) noexcept {
    for (int lane = 0; lane < 4; ++lane)
        a.lane[lane] += b.lane[lane] * c.lane[lane];
    return a;
}
inline double sum4(Float4 value) noexcept {
    return (double(value.lane[0]) + value.lane[1]) + (double(value.lane[2]) + value.lane[3]);
}
#endif
} // namespace detail

// The SincKernel filter for one cutoff, stored by fractional phase so that a
// whole filter is read from contiguous rows instead of one scattered lookup per
// tap. Each of kCells phase cells holds a quartic per tap. A reader whose
// cutoff differs from the table's by a measured clock ratio needs the same
// kernel at a phase that advances linearly from tap to tap, so drifting
// cutoffs are evaluated as given instead of being rounded to the table's.
//
// Construct on control: the constructor evaluates the windowed sinc in double
// precision and throws for a filter it cannot hold. Afterwards the object is
// immutable; readers need no lock, allocation or shared scratch. The weights
// are single precision and match the kernel's definition to within 1e-7 of its
// peak, which is closer than SincKernel's own interpolated lookup.
class SincPhaseTable {
  public:
    static constexpr uint32_t kCells = 32;
    // Largest supported |1 - cutoff / cutoff()|: capture and reader clocks
    // that are each within 0.5% of one nominal rate.
    static constexpr double kMaximumScale = .01;
    // Filter lengths are rounded up to whole groups of sixteen taps.
    static constexpr uint32_t kMaximumTaps = 512;

    explicit SincPhaseTable(double cutoff, uint32_t cells = kCells);
    SincPhaseTable(const SincPhaseTable&) = delete;
    SincPhaseTable& operator=(const SincPhaseTable&) = delete;

    double cutoff() const noexcept {
        return cutoff_;
    }
    bool covers(double cutoff) const noexcept {
        const auto scale = 1 - cutoff * inverseCutoff_;
        return scale >= -kMaximumScale && scale <= kMaximumScale;
    }
    static constexpr uint32_t paddedTaps(uint32_t radius) noexcept {
        return (2 * radius + 1 + 15) & ~15u;
    }

    // The readers below take the tap layout of SincKernel::fillCoefficients:
    // tap k weighs the source frame (k - radius) - fraction away from the
    // target, and is zero where that distance times cutoff reaches kRadius.
    // A returned weight sum below one half means the request is outside this
    // table (cutoff, radius or fraction); the caller then uses SincKernel.

    // Writes paddedTaps(radius) weights, zero beyond the filter, and returns
    // their sum.
    double weights(uint32_t radius, double fraction, double cutoff,
                   std::span<float> output) const noexcept {
        if (output.size() < paddedTaps(radius) || !supports(radius, fraction, cutoff)) {
            for (auto& weight : output)
                weight = 0;
            return 0;
        }
        StoreSink head{output.data()}, tail{output.data()};
        generate(radius, fraction, cutoff, head, tail);
        using detail::sum4;
        return sum4(head.sum0) + sum4(head.sum1) + sum4(tail.sum0) + sum4(tail.sum1);
    }

    // Filter sums without a weight workspace. Each plane must be readable for
    // paddedTaps(radius) floats, with finite values beyond the filter's taps.
    struct Pair {
        double a, b, weights;
    };
    Pair convolve(uint32_t radius, double fraction, double cutoff, const float* a,
                  const float* b) const noexcept {
        if (!supports(radius, fraction, cutoff))
            return {0, 0, 0};
        PairSink head{a, b}, tail{a, b};
        generate(radius, fraction, cutoff, head, tail);
        using detail::sum4;
        return {sum4(head.a0) + sum4(head.a1) + sum4(tail.a0) + sum4(tail.a1),
                sum4(head.b0) + sum4(head.b1) + sum4(tail.b0) + sum4(tail.b1),
                sum4(head.sum0) + sum4(head.sum1) + sum4(tail.sum0) + sum4(tail.sum1)};
    }
    struct Single {
        double a, weights;
    };
    Single convolve(uint32_t radius, double fraction, double cutoff,
                    const float* a) const noexcept {
        if (!supports(radius, fraction, cutoff))
            return {0, 0};
        SingleSink head{a}, tail{a};
        generate(radius, fraction, cutoff, head, tail);
        using detail::sum4;
        return {sum4(head.a0) + sum4(head.a1) + sum4(tail.a0) + sum4(tail.a1),
                sum4(head.sum0) + sum4(head.sum1) + sum4(tail.sum0) + sum4(tail.sum1)};
    }

    // Filter sums for further planes that share weights(). Like the readers
    // above, both halves of the filter are accumulated towards its centre, so
    // the largest products are added last and single-precision partial sums
    // lose the least.
    static double dot(const float* samples, const float* weights, uint32_t padded) noexcept {
        using namespace detail;
        auto a0 = splat4(0), a1 = a0, c0 = a0, c1 = a0;
        const uint32_t half = padded / 2;
        for (uint32_t tap = 0; tap < half; tap += 8) {
            a0 = multiplyAdd4(a0, load4(weights + tap), load4(samples + tap));
            a1 = multiplyAdd4(a1, load4(weights + tap + 4), load4(samples + tap + 4));
        }
        for (uint32_t tap = padded; tap > half;) {
            tap -= 8;
            c0 = multiplyAdd4(c0, load4(weights + tap), load4(samples + tap));
            c1 = multiplyAdd4(c1, load4(weights + tap + 4), load4(samples + tap + 4));
        }
        return sum4(a0) + sum4(a1) + sum4(c0) + sum4(c1);
    }
    static std::array<double, 2> dotPair(const float* a, const float* b, const float* weights,
                                         uint32_t padded) noexcept {
        using namespace detail;
        auto a0 = splat4(0), a1 = a0, b0 = a0, b1 = a0, c0 = a0, c1 = a0, d0 = a0, d1 = a0;
        const uint32_t half = padded / 2;
        for (uint32_t tap = 0; tap < half; tap += 8) {
            const auto w0 = load4(weights + tap), w1 = load4(weights + tap + 4);
            a0 = multiplyAdd4(a0, w0, load4(a + tap));
            a1 = multiplyAdd4(a1, w1, load4(a + tap + 4));
            b0 = multiplyAdd4(b0, w0, load4(b + tap));
            b1 = multiplyAdd4(b1, w1, load4(b + tap + 4));
        }
        for (uint32_t tap = padded; tap > half;) {
            tap -= 8;
            const auto w0 = load4(weights + tap), w1 = load4(weights + tap + 4);
            c0 = multiplyAdd4(c0, w0, load4(a + tap));
            c1 = multiplyAdd4(c1, w1, load4(a + tap + 4));
            d0 = multiplyAdd4(d0, w0, load4(b + tap));
            d1 = multiplyAdd4(d1, w1, load4(b + tap + 4));
        }
        return {sum4(a0) + sum4(a1) + sum4(c0) + sum4(c1),
                sum4(b0) + sum4(b1) + sum4(d0) + sum4(d1)};
    }

  private:
    static constexpr uint32_t kCoefficients = 5;
    // Columns per row: enough for both prepared cutoffs (see the constructor).
    // A constant keeps the five rows of a column at fixed offsets.
    static constexpr uint32_t kStride = 248;

    // Sinks consume the filter eight taps at a time, at a cursor that moves
    // with the generator's direction.
    struct StoreSink {
        float* base;
        float* cursor{};
        detail::Float4 sum0{detail::splat4(0)}, sum1{detail::splat4(0)};
        ZCORE_ROUTING_ALWAYS_INLINE void seek(uint32_t tap) noexcept {
            cursor = base + tap;
        }
        ZCORE_ROUTING_ALWAYS_INLINE void operator()(detail::Float4 w0, detail::Float4 w1,
                                                    std::ptrdiff_t step) noexcept {
            detail::store4(cursor, w0);
            detail::store4(cursor + 4, w1);
            sum0 = detail::add4(sum0, w0);
            sum1 = detail::add4(sum1, w1);
            cursor += step;
        }
    };
    struct SingleSink {
        const float* base;
        const float* cursor{};
        detail::Float4 a0{detail::splat4(0)}, a1{detail::splat4(0)}, sum0{detail::splat4(0)},
            sum1{detail::splat4(0)};
        ZCORE_ROUTING_ALWAYS_INLINE void seek(uint32_t tap) noexcept {
            cursor = base + tap;
        }
        ZCORE_ROUTING_ALWAYS_INLINE void operator()(detail::Float4 w0, detail::Float4 w1,
                                                    std::ptrdiff_t step) noexcept {
            using namespace detail;
            a0 = multiplyAdd4(a0, w0, load4(cursor));
            a1 = multiplyAdd4(a1, w1, load4(cursor + 4));
            sum0 = add4(sum0, w0);
            sum1 = add4(sum1, w1);
            cursor += step;
        }
    };
    struct PairSink {
        const float* baseA;
        const float* baseB;
        const float* a{};
        const float* b{};
        detail::Float4 a0{detail::splat4(0)}, a1{detail::splat4(0)}, b0{detail::splat4(0)},
            b1{detail::splat4(0)}, sum0{detail::splat4(0)}, sum1{detail::splat4(0)};
        ZCORE_ROUTING_ALWAYS_INLINE void seek(uint32_t tap) noexcept {
            a = baseA + tap;
            b = baseB + tap;
        }
        ZCORE_ROUTING_ALWAYS_INLINE void operator()(detail::Float4 w0, detail::Float4 w1,
                                                    std::ptrdiff_t step) noexcept {
            using namespace detail;
            a0 = multiplyAdd4(a0, w0, load4(a));
            a1 = multiplyAdd4(a1, w1, load4(a + 4));
            b0 = multiplyAdd4(b0, w0, load4(b));
            b1 = multiplyAdd4(b1, w1, load4(b + 4));
            sum0 = add4(sum0, w0);
            sum1 = add4(sum1, w1);
            a += step;
            b += step;
        }
    };

    bool supports(uint32_t radius, double fraction, double cutoff) const noexcept {
        return covers(cutoff) && cutoff > 0 && cutoff <= 1 && radius <= maximumRadius_ &&
               fraction >= 0 && fraction <= 1;
    }

    // Tap k needs the tabulated kernel at phase
    //   fraction + scale * ((k - radius) - fraction),  scale = 1 - cutoff / cutoff(),
    // a straight line in k, here in cells from tap zero. Taps are produced in
    // groups of eight. Each phase cell's quartic is fitted on a margin around
    // the cell, so one cell serves every tap whose phase lies within reach_
    // cells of it: half the filter when the line is flat enough, otherwise
    // one cell per group, which the steepest supported line still satisfies.
    struct Line {
        const float* table;
        int64_t cellMask, cellShift, column;
        double origin, slope;
        // The kernel's support [first, last) in taps, and the groups
        // [inner, outer) that lie wholly inside it.
        uint32_t first, last, inner, outer;
    };
    // Groups [start, start + count) in one cell: the cell's row at group
    // `start`, and the phase of that group's first tap from the middle of
    // the cell.
    struct Chunk {
        const float* row;
        uint32_t start, count;
        float phase;
    };
    static ZCORE_ROUTING_ALWAYS_INLINE Chunk chunkAt(const Line& line, uint32_t start,
                                                     uint32_t count) noexcept {
        // The phase at the middle of the chunk selects its cell.
        const double span = (8.0 * count - 1) / 2;
        const double center = line.origin + (8.0 * start + span) * line.slope;
        const double whole = std::floor(center);
        const auto index = static_cast<int64_t>(whole);
        return {line.table + size_t(index & line.cellMask) * (size_t(kCoefficients) * kStride) +
                    (line.column - (index >> line.cellShift)) + 8 * int64_t(start),
                start, count, static_cast<float>(center - whole - .5 - span * line.slope)};
    }
    // The quartics of eight adjacent columns, each at its own phase.
    static ZCORE_ROUTING_ALWAYS_INLINE void evaluate(const float* row, detail::Float4 phase0,
                                                     detail::Float4 phase1, detail::Float4& w0,
                                                     detail::Float4& w1) noexcept {
        using namespace detail;
        w0 = multiplyAdd4(load4(row + 3 * kStride), load4(row + 4 * kStride), phase0);
        w1 = multiplyAdd4(load4(row + 3 * kStride + 4), load4(row + 4 * kStride + 4), phase1);
        w0 = multiplyAdd4(load4(row + 2 * kStride), w0, phase0);
        w1 = multiplyAdd4(load4(row + 2 * kStride + 4), w1, phase1);
        w0 = multiplyAdd4(load4(row + kStride), w0, phase0);
        w1 = multiplyAdd4(load4(row + kStride + 4), w1, phase1);
        w0 = multiplyAdd4(load4(row), w0, phase0);
        w1 = multiplyAdd4(load4(row + 4), w1, phase1);
    }
    // Tap offsets {first, first + 1, first + 2, first + 3}. The constant comes
    // from read-only data: lanes assembled on the stack would stall the load.
    static ZCORE_ROUTING_ALWAYS_INLINE detail::Float4 lanes(uint32_t first) noexcept {
        static constexpr float offsets[4] = {0, 1, 2, 3};
        return detail::add4(detail::splat4(static_cast<float>(first)), detail::load4(offsets));
    }
    // Zeroes the taps of the group at `tap` that are outside [first, last):
    // the kernel's edge crosses the group, or the group is padding.
    static ZCORE_ROUTING_ALWAYS_INLINE void mask(const Line& line, uint32_t tap, detail::Float4& w0,
                                                 detail::Float4& w1) noexcept {
        using namespace detail;
        static constexpr float keep[24] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1,
                                           1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0};
        // Lanes [low, high) of this group are inside the support.
        const uint32_t low = line.first > tap ? (line.first - tap < 8 ? line.first - tap : 8) : 0;
        const uint32_t high = line.last > tap ? (line.last - tap < 8 ? line.last - tap : 8) : 0;
        const float* const above = keep + 8 - low;
        const float* const below = keep + 16 - high;
        w0 = multiply4(w0, multiply4(load4(above), load4(below)));
        w1 = multiply4(w1, multiply4(load4(above + 4), load4(below + 4)));
    }
    // Groups [from, to) of a chunk, masked: for the few that are not wholly
    // inside the support.
    template <class Sink>
    static ZCORE_ROUTING_ALWAYS_INLINE void edges(const Line& line, const Chunk& chunk,
                                                  uint32_t from, uint32_t to, bool rising,
                                                  Sink& sink) noexcept {
        using namespace detail;
        const auto slope = static_cast<float>(line.slope);
        for (uint32_t left = to > from ? to - from : 0; left; --left) {
            const uint32_t group = rising ? to - left : from + left - 1;
            const auto phase0 =
                multiplyAdd4(splat4(chunk.phase), splat4(slope), lanes(8 * (group - chunk.start)));
            Float4 w0, w1;
            evaluate(chunk.row + 8 * (group - chunk.start), phase0, add4(phase0, splat4(4 * slope)),
                     w0, w1);
            mask(line, 8 * group, w0, w1);
            sink.seek(8 * group);
            sink(w0, w1, 0);
        }
    }
    // One chunk, in rising or falling tap order.
    template <class Sink>
    static ZCORE_ROUTING_ALWAYS_INLINE void run(const Line& line, const Chunk& chunk, bool rising,
                                                Sink& sink) noexcept {
        using namespace detail;
        const uint32_t start = chunk.start, stop = chunk.start + chunk.count;
        const uint32_t inner = line.inner < start ? start : line.inner < stop ? line.inner : stop;
        const uint32_t outer = line.outer < inner ? inner : line.outer < stop ? line.outer : stop;
        edges(line, chunk, rising ? start : outer, rising ? inner : stop, rising, sink);
        if (inner < outer) {
            const std::ptrdiff_t step = rising ? 8 : -8;
            const uint32_t group = rising ? inner : outer - 1;
            const float* row = chunk.row + 8 * (group - start);
            const auto phase = splat4(chunk.phase);
            sink.seek(8 * group);
            if (line.slope != 0) {
                const auto slope = splat4(static_cast<float>(line.slope));
                const auto slope4 = splat4(static_cast<float>(4 * line.slope));
                const auto stepLane = splat4(static_cast<float>(step));
                auto offset = lanes(8 * (group - start));
                for (uint32_t left = outer - inner; left; --left, row += step) {
                    const auto phase0 = multiplyAdd4(phase, slope, offset);
                    Float4 w0, w1;
                    evaluate(row, phase0, add4(phase0, slope4), w0, w1);
                    sink(w0, w1, step);
                    offset = add4(offset, stepLane);
                }
            } else {
                for (uint32_t left = outer - inner; left; --left, row += step) {
                    Float4 w0, w1;
                    evaluate(row, phase, phase, w0, w1);
                    sink(w0, w1, step);
                }
            }
        }
        edges(line, chunk, rising ? outer : start, rising ? stop : inner, rising, sink);
    }
    // Groups [begin, end) with a cell each, in rising or falling tap order.
    // The phase at the middle of each group is tracked in 32.32 fixed point:
    // an integer cell and address keep the groups independent of each other,
    // where a floating-point rounding per group would queue them.
    template <class Sink>
    static ZCORE_ROUTING_ALWAYS_INLINE void scatter(const Line& line, uint32_t begin, uint32_t end,
                                                    bool rising, Sink& sink) noexcept {
        using namespace detail;
        if (begin >= end)
            return;
        constexpr double unit = 4294967296.0;
        const auto slope = static_cast<float>(line.slope);
        const auto slopes = multiply4(splat4(slope), lanes(0));
        const auto slope4 = splat4(4 * slope);
        // From the middle of a cell to the first tap of a group centred there.
        const auto first = static_cast<float>(-.5 - 3.5 * line.slope);
        const std::ptrdiff_t step = rising ? 8 : -8;
        uint32_t group = rising ? begin : end - 1;
        auto position = static_cast<int64_t>(
            std::floor((line.origin + (8.0 * group + 3.5) * line.slope) * unit));
        const auto advance =
            static_cast<int64_t>(std::llround(8 * line.slope * unit)) * (rising ? 1 : -1);
        const float* column = line.table + line.column + 8 * int64_t(group);
        sink.seek(8 * group);
        for (uint32_t left = end - begin; left;
             --left, group += rising ? 1 : -1, position += advance, column += step) {
            const int64_t index = position >> 32;
            const float* row = column +
                               size_t(index & line.cellMask) * (size_t(kCoefficients) * kStride) -
                               (index >> line.cellShift);
            const auto phase0 =
                add4(splat4(static_cast<float>(static_cast<uint32_t>(position)) * 0x1p-32f + first),
                     slopes);
            Float4 w0, w1;
            evaluate(row, phase0, add4(phase0, slope4), w0, w1);
            if (group < line.inner || group >= line.outer)
                mask(line, 8 * group, w0, w1);
            sink(w0, w1, step);
        }
    }
    // First half of the filter in rising order into head, second half in
    // falling order into tail: both end at the centre taps.
    template <class Sink>
    ZCORE_ROUTING_ALWAYS_INLINE void generate(uint32_t radius, double fraction, double cutoff,
                                              Sink& head, Sink& tail) const noexcept {
        const uint32_t taps = 2 * radius + 1, groups = paddedTaps(radius) / 8, half = groups / 2;
        // An equal cutoff must give a slope of exactly zero.
        const double scale = cutoff == cutoff_ ? 0.0 : 1 - cutoff * inverseCutoff_;
        Line line{data_.data(),
                  int64_t(cells_) - 1,
                  cellShift_,
                  int64_t(margin_) - radius,
                  cells_ * (fraction * (1 - scale) - scale * radius),
                  scale * cells_,
                  0,
                  taps,
                  0,
                  0};
        // Both halves are placed before either runs, so that locating the
        // second cell does not wait behind the first half's sums.
        const bool flat = std::abs(line.slope) * (8.0 * (groups - half) - 1) <= 2 * reach_;
        const auto headChunk = chunkAt(line, 0, half);
        const auto tailChunk = chunkAt(line, half, groups - half);
        // The kernel's support, by SincKernel's own test.
        const auto outside = [&](uint32_t tap) {
            return std::abs((double(tap) - radius) - fraction) * cutoff >= SincKernel::kRadius;
        };
        while (line.first < line.last && outside(line.first))
            ++line.first;
        while (line.last > line.first && outside(line.last - 1))
            --line.last;
        line.inner = (line.first + 7) / 8;
        line.outer = line.last / 8;
        if (flat) {
            run(line, headChunk, true, head);
            run(line, tailChunk, false, tail);
        } else {
            scatter(line, 0, half, true, head);
            scatter(line, half, groups, false, tail);
        }
    }

    double cutoff_{}, inverseCutoff_{}, reach_{};
    uint32_t cells_{}, cellShift_{}, maximumRadius_{}, margin_{};
    // [cell][coefficient][column], rows of kStride floats.
    std::array<float, kCells * kCoefficients * kStride> data_{};
};

// The tables every linear-phase reader shares: the unity cutoff, which serves
// all ratios that do not lower the output rate, and 48 kHz to 44.1 kHz. Other
// reductions exceed the cached FIR window for whole blocks and keep SincKernel.
class SincPhaseTables {
  public:
    // Prepare on control; the first call builds the tables.
    static const SincPhaseTables& prepared();
    SincPhaseTables(const SincPhaseTables&) = delete;
    SincPhaseTables& operator=(const SincPhaseTables&) = delete;
    // Bounded lookup for the audio thread; null when no table covers cutoff.
    const SincPhaseTable* find(double cutoff) const noexcept {
        if (unity_.covers(cutoff))
            return &unity_;
        return reduced_.covers(cutoff) ? &reduced_ : nullptr;
    }

  private:
    SincPhaseTables();
    const SincPhaseTable unity_, reduced_;
};
} // namespace zcore::routing
