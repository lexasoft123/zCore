#pragma once
#include <array>
#include <atomic>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <limits>

namespace zcore::routing {
struct AudioEnergySnapshot {
    double rms{}, highBandRms{};
    uint64_t blocks{}, frames{};
    bool available{false};
    uint64_t skippedPublications{};
};
// Diagnostic only: block-local two-pole Butterworth high-pass at 8 kHz.
// This is an approximate band-energy indicator, not a brick-wall spectrum.
// Resetting local filter scratch makes independent history readers safe. The
// first 16 frames are excluded from high-band energy to reject reset transients.
// Concurrent observers calculate only local scratch. Publication admits exactly
// one writer per instance with a nonblocking flag; losers drop telemetry only.
// Bounded versioned readers reject a collision rather than mix block payloads.
class AudioEnergyMeter {
  public:
    void observe(std::span<float* const> planes, uint32_t frames, double rate,
                 uint32_t stride = 1) noexcept {
        if (!frames || !stride || !std::isfinite(rate) || rate <= 16000 || planes.size() > 64 ||
            uint64_t(frames - 1) * stride > std::numeric_limits<size_t>::max())
            return;
        const double k = std::tan(3.14159265358979323846 * 8000 / rate);
        const double norm = 1 / (1 + 1.4142135623730951 * k + k * k);
        const double b0 = norm, b1 = -2 * norm, b2 = norm;
        const double a1 = 2 * (k * k - 1) * norm, a2 = (1 - 1.4142135623730951 * k + k * k) * norm;
        double energy = 0, high = 0;
        uint64_t count = 0, highCount = 0;
        for (auto* plane : planes) {
            if (!plane)
                continue;
            double z1 = 0, z2 = 0;
            for (uint32_t frame = 0; frame < frames; ++frame) {
                const double x = std::isfinite(plane[uint64_t(frame) * stride])
                                     ? plane[uint64_t(frame) * stride]
                                     : 0;
                energy += x * x;
                ++count;
                const double y = b0 * x + z1;
                z1 = b1 * x - a1 * y + z2;
                z2 = b2 * x - a2 * y;
                if (frame >= 16) {
                    high += y * y;
                    ++highCount;
                }
            }
        }
        if (publishing_.test_and_set(std::memory_order_acquire)) {
            skipped_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        const uint64_t odd = version_.fetch_add(1, std::memory_order_acq_rel) + 1;
        rms_.store(std::bit_cast<uint64_t>(count ? std::sqrt(energy / count) : 0.),
                   std::memory_order_release);
        high_.store(std::bit_cast<uint64_t>(highCount ? std::sqrt(high / highCount) : 0.),
                    std::memory_order_release);
        frames_.store(frames, std::memory_order_release);
        blocks_.fetch_add(1, std::memory_order_release);
        version_.store(odd + 1, std::memory_order_release);
        publishing_.clear(std::memory_order_release);
    }
    AudioEnergySnapshot snapshot() const noexcept {
        for (unsigned attempt = 0; attempt < 2; ++attempt) {
            const auto before = version_.load(std::memory_order_acquire);
            if (before & 1)
                continue;
            AudioEnergySnapshot result{std::bit_cast<double>(rms_.load(std::memory_order_acquire)),
                                       std::bit_cast<double>(high_.load(std::memory_order_acquire)),
                                       blocks_.load(std::memory_order_acquire),
                                       frames_.load(std::memory_order_acquire)};
            result.skippedPublications = skipped_.load(std::memory_order_relaxed);
            if (version_.load(std::memory_order_acquire) == before) {
                result.available = result.blocks != 0;
                return result;
            }
        }
        AudioEnergySnapshot unavailable;
        unavailable.skippedPublications = skipped_.load(std::memory_order_relaxed);
        return unavailable;
    }

  private:
    std::atomic_flag publishing_ = ATOMIC_FLAG_INIT;
    std::atomic<uint64_t> version_{0}, skipped_{0};
    std::atomic<uint64_t> rms_{0}, high_{0}, blocks_{0}, frames_{0};
};
} // namespace zcore::routing
