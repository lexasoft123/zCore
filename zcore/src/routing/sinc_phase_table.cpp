#include <zcore/routing/sinc_phase_table.h>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace zcore::routing {
namespace {
constexpr double kBeta = 9;
constexpr uint32_t kNodes = 5;

// sum z^k / (k!)^2, which is I0(2 sqrt(z)) for z >= 0.
double besselSeries(double z) {
    double sum = 1, term = 1;
    for (unsigned order = 1; order < 96; ++order) {
        term *= z / (double(order) * order);
        sum += term;
        if (std::abs(term) < 1e-18 * std::abs(sum))
            break;
    }
    return sum;
}
// SincKernel's Kaiser-windowed sinc, continued smoothly past its support: the
// series is entire in z, where the window's own formula stops at a square
// root. A quartic then fits cells that straddle the support edge as well as
// any other; readers zero the taps beyond the edge explicitly.
double kernel(double distance) {
    static const double normalization = besselSeries(kBeta * kBeta / 4);
    const auto x = std::numbers::pi_v<double> * distance;
    const auto sinc = std::abs(x) < 1e-8 ? 1.0 : std::sin(x) / x;
    const auto relative = distance / SincKernel::kRadius;
    return sinc * besselSeries(kBeta * kBeta / 4 * (1 - relative * relative)) / normalization;
}
} // namespace

SincPhaseTable::SincPhaseTable(double cutoff, uint32_t cells) : cutoff_(cutoff), cells_(cells) {
    if (!std::isfinite(cutoff) || cutoff <= 0 || cutoff > 1 || !cells || cells > kCells ||
        (cells & (cells - 1)))
        throw std::invalid_argument("Unsupported sinc phase table");
    inverseCutoff_ = 1 / cutoff_;
    while ((1u << cellShift_) < cells_)
        ++cellShift_;
    // Readers within kMaximumScale of this cutoff need at most this radius.
    // A reader's phase line leaves [0, 1) by whole source frames; each one
    // moves it a column, so rows carry that margin on both sides, plus the
    // padding of the last group of taps.
    const double widest = std::ceil(SincKernel::kRadius / (cutoff * (1 - kMaximumScale)));
    if (2 * widest + 1 > kMaximumTaps)
        throw std::invalid_argument("Sinc phase table filter is too long");
    maximumRadius_ = static_cast<uint32_t>(widest);
    const auto shift = static_cast<uint32_t>(kMaximumScale * (maximumRadius_ + 16)) + 1;
    margin_ = maximumRadius_ + shift;
    if (margin_ + maximumRadius_ + 16 + shift + 1 > kStride)
        throw std::invalid_argument("Sinc phase table filter is too long");
    // A group of eight taps on the steepest phase line spans 3.5 cells' worth
    // of slope either side of its centre.
    reach_ = 3.5 * kMaximumScale * cells_;

    // Interpolate each cell at Chebyshev nodes of its reach, in a variable
    // centred on the cell: near-minimax error and well-scaled monomials.
    std::array<double, kNodes> nodes{};
    for (uint32_t node = 0; node < kNodes; ++node)
        nodes[node] =
            (.5 + reach_) * std::cos(std::numbers::pi_v<double> * (2 * node + 1) / (2 * kNodes));
    std::array<std::array<double, kNodes>, kNodes> inverse{};
    {
        std::array<std::array<long double, 2 * kNodes>, kNodes> system{};
        for (uint32_t row = 0; row < kNodes; ++row) {
            long double power = 1;
            for (uint32_t column = 0; column < kNodes; ++column) {
                system[row][column] = power;
                power *= nodes[row];
            }
            system[row][kNodes + row] = 1;
        }
        for (uint32_t pivot = 0; pivot < kNodes; ++pivot) {
            uint32_t best = pivot;
            for (uint32_t row = pivot + 1; row < kNodes; ++row)
                if (std::abs(system[row][pivot]) > std::abs(system[best][pivot]))
                    best = row;
            std::swap(system[pivot], system[best]);
            const auto scale = system[pivot][pivot];
            for (auto& value : system[pivot])
                value /= scale;
            for (uint32_t row = 0; row < kNodes; ++row)
                if (row != pivot) {
                    const auto factor = system[row][pivot];
                    for (uint32_t column = 0; column < 2 * kNodes; ++column)
                        system[row][column] -= factor * system[pivot][column];
                }
        }
        for (uint32_t row = 0; row < kNodes; ++row)
            for (uint32_t column = 0; column < kNodes; ++column)
                inverse[row][column] = static_cast<double>(system[row][kNodes + column]);
    }
    static_assert(kNodes == kCoefficients);
    std::array<double, kNodes> values{};
    for (uint32_t cell = 0; cell < cells_; ++cell)
        for (uint32_t column = 0; column < kStride; ++column) {
            // Column c holds the source frame c - margin_ away, before the
            // fractional phase is subtracted.
            const double frames = double(column) - margin_;
            for (uint32_t node = 0; node < kNodes; ++node)
                values[node] = kernel((frames - (cell + .5 + nodes[node]) / cells_) * cutoff_);
            for (uint32_t power = 0; power < kNodes; ++power) {
                double coefficient = 0;
                for (uint32_t node = 0; node < kNodes; ++node)
                    coefficient += inverse[power][node] * values[node];
                data_[(size_t(cell) * kCoefficients + power) * kStride + column] =
                    static_cast<float>(coefficient);
            }
        }
}

// The reduced cutoff is written as AudioResampler derives it for an exact
// 48 kHz to 44.1 kHz read, so that read has no slope at all.
SincPhaseTables::SincPhaseTables()
    : unity_(SincKernel::kCutoff), reduced_(SincKernel::kCutoff * (44100.0 / 48000.0)) {}
const SincPhaseTables& SincPhaseTables::prepared() {
    static const SincPhaseTables value;
    return value;
}
} // namespace zcore::routing
