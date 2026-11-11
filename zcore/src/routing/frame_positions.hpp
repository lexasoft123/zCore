#pragma once
#include <bit>
#include <cmath>
#include <cstdint>
#include <optional>

namespace zcore::routing::detail {
// Source positions of a block's output frames, anchor + offset + frame * step,
// each split into a whole source frame and a fraction in [0, 1].
//
// The block offset is a long double so that a long capture keeps sub-sample
// phase. On AArch64 Linux that type is IEEE quad in software, and a product,
// a sum, a floor and their conversions for every output frame cost more than
// the filter itself. The offset is therefore reduced once per block, and each
// frame is placed with error-free double arithmetic: about 100 significant
// bits, on every platform, where long double has 113, 64 or 53.
class FramePositions {
  public:
    struct Position {
        uint64_t center;
        double fraction;
    };
    static constexpr uint32_t kMaximumFrames = 1u << 26;

    FramePositions(uint64_t anchor, long double offset, double step) noexcept
        : anchor_(anchor), room_(anchor == UINT64_MAX ? 0 : UINT64_MAX - anchor - 1),
          offset_(offset), step_(step) {
        const long double whole = std::floor(offset);
        // Offsets this large have no fraction left; the direct sum handles
        // them, with every other unusual argument.
        if (!(whole > -0x1p62L && whole < 0x1p62L) || !(step > 0 && step < 0x1p20))
            return;
        const long double rest = offset - whole;
        whole_ = static_cast<int64_t>(whole);
        fraction_ = static_cast<double>(rest);
        fractionTail_ = static_cast<double>(rest - fraction_);
        // step = high + low exactly, each short enough that its product with
        // a frame index below kMaximumFrames is exact.
        high_ = std::bit_cast<double>(std::bit_cast<uint64_t>(step) & ~((uint64_t{1} << 27) - 1));
        low_ = step - high_;
        reduced_ = true;
    }
    // Empty when the frame's source position cannot sit beside the anchor.
    std::optional<Position> at(uint32_t frame) const noexcept {
        if (!reduced_ || frame >= kMaximumFrames)
            return direct(frame);
        // fraction + tail + frame * (high + low) as an unevaluated sum of two
        // doubles; both products are exact, and each sum keeps its error.
        const auto major = sum(fraction_, frame * high_);
        const auto minor = sum(frame * low_, fractionTail_);
        const auto upper = sum(major.value, minor.value);
        const double tail = upper.error + (major.error + minor.error);
        const double value = upper.value + tail;
        const double error = tail - (value - upper.value);
        double frames = std::floor(value);
        // The sum can round up to a whole frame that it has not reached.
        if (error < 0 && value == frames)
            frames -= 1;
        const int64_t whole = whole_ + static_cast<int64_t>(frames);
        const double fraction = (value - frames) + error;
        return whole < 0 ? below(uint64_t{0} - static_cast<uint64_t>(whole), fraction)
                         : above(static_cast<uint64_t>(whole), fraction);
    }

  private:
    struct Sum {
        double value, error;
    };
    // value + error == a + b exactly.
    static Sum sum(double a, double b) noexcept {
        const double value = a + b;
        const double part = value - a;
        return {value, (a - (value - part)) + (b - part)};
    }
    // The source frame `back` frames below the anchor, or `ahead` frames above
    // it; the last two frames of the counter stay unused, as before.
    std::optional<Position> below(uint64_t back, double fraction) const noexcept {
        if (back > anchor_)
            return std::nullopt;
        return Position{anchor_ - back, fraction};
    }
    std::optional<Position> above(uint64_t ahead, double fraction) const noexcept {
        if (ahead >= room_)
            return std::nullopt;
        return Position{anchor_ + ahead, fraction};
    }
    std::optional<Position> direct(uint32_t frame) const noexcept {
        const auto offset = offset_ + static_cast<long double>(frame) * step_;
        const auto whole = std::floor(offset);
        // Beyond the 64-bit counter (or not a number) is beside no anchor.
        if (!(whole > -0x1p64L && whole < 0x1p64L))
            return std::nullopt;
        const auto fraction = static_cast<double>(offset - whole);
        return whole < 0 ? below(static_cast<uint64_t>(-whole), fraction)
                         : above(static_cast<uint64_t>(whole), fraction);
    }

    uint64_t anchor_, room_;
    long double offset_;
    double step_;
    int64_t whole_{};
    double fraction_{}, fractionTail_{}, high_{}, low_{};
    bool reduced_{false};
};
} // namespace zcore::routing::detail
