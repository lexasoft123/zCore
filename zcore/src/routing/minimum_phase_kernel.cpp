// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#include <zcore/routing/minimum_phase_kernel.h>

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace zcore::routing {
namespace {
// The control-only radix-2 transform follows SingZ's legacy/courts.cpp FFT.
// Upstream source bytes remain unchanged. No transform runs on an audio thread.
void transform(std::vector<std::complex<double>>& values, bool inverse) {
    const auto count = values.size();
    for (size_t i = 1, j = 0; i < count; ++i) {
        auto bit = count >> 1;
        for (; j & bit; bit >>= 1)
            j ^= bit;
        j ^= bit;
        if (i < j)
            std::swap(values[i], values[j]);
    }
    for (size_t length = 2; length <= count; length <<= 1) {
        const auto angle = (inverse ? 2 : -2) * std::numbers::pi_v<double> / length;
        const auto step = std::polar(1.0, angle);
        for (size_t begin = 0; begin < count; begin += length) {
            std::complex<double> rotation{1, 0};
            for (size_t offset = 0; offset < length / 2; ++offset) {
                const auto first = values[begin + offset];
                const auto second = values[begin + offset + length / 2] * rotation;
                values[begin + offset] = first + second;
                values[begin + offset + length / 2] = first - second;
                rotation *= step;
            }
        }
    }
    if (inverse)
        for (auto& value : values)
            value /= static_cast<double>(count);
}
double besselI0(double value) {
    double sum = 1, term = 1;
    for (unsigned order = 1; order < 64; ++order) {
        term *= value * value / (4 * order * order);
        sum += term;
        if (term < 1e-12 * sum)
            break;
    }
    return sum;
}
} // namespace

MinimumPhaseKernel::MinimumPhaseKernel() {
    constexpr size_t fftSize = 1 << 20;
    constexpr double beta = 9;
    const auto normalization = besselI0(beta);
    std::vector<std::complex<double>> values(fftSize);
    for (size_t index = 0; index < values_.size(); ++index) {
        const auto time = double(index) / kDensity - SincKernel::kRadius;
        const auto relative = time / SincKernel::kRadius;
        const auto sinc = time == 0 ? 1
                                    : std::sin(std::numbers::pi_v<double> * time) /
                                          (std::numbers::pi_v<double> * time);
        const auto window = besselI0(beta * std::sqrt(std::max(0.0, 1 - relative * relative)));
        values[index] = sinc * window / (normalization * kDensity);
    }
    // Real-cepstrum spectral factorization of the sampled magnitude:
    // fold negative quefrencies into positive time, then exponentiate.
    transform(values, false);
    for (auto& value : values)
        value = std::log(std::max(std::abs(value), 1e-14));
    transform(values, true);
    for (size_t index = 0; index < values.size(); ++index)
        values[index] = index > fftSize / 2            ? 0
                        : index && index < fftSize / 2 ? 2 * values[index].real()
                                                       : values[index].real();
    transform(values, false);
    for (auto& value : values)
        value = std::exp(value);
    transform(values, true);
    for (size_t index = 0; index < values_.size(); ++index) {
        const auto sample = values[index].real() * kDensity;
        if (!std::isfinite(sample) || std::abs(sample) > 2)
            throw std::runtime_error("Minimum-phase kernel preparation failed");
        values_[index] = static_cast<float>(sample);
    }
}
const MinimumPhaseKernel& MinimumPhaseKernel::prepared() {
    static const MinimumPhaseKernel value;
    return value;
}
double MinimumPhaseKernel::coefficient(double pastDistance, double cutoff) const noexcept {
    if (!std::isfinite(pastDistance) || pastDistance < 0 || !std::isfinite(cutoff) || cutoff <= 0 ||
        cutoff > 1)
        return 0;
    const auto position = pastDistance * cutoff * kDensity;
    if (position >= values_.size() - 1)
        return 0;
    const auto index = static_cast<size_t>(position);
    const auto fraction = position - index;
    return values_[index] + (values_[index + 1] - values_[index]) * fraction;
}
} // namespace zcore::routing
