// Cost of the timestamp-addressed resampler, per output frame.
//
// Direct cases call AudioResampler::read with a provider that only copies
// samples, so they time target placement, filter weights and the FIR. History
// cases add the clock lookup and AudioHistory's lock-free frame reads, with a
// capture clock and a reader clock that drift like two real devices.
//
// Only the public routing API is used, so the same file builds against older
// revisions for before/after comparisons. Run it pinned to one core:
//   taskset -c 7 ./zcore_routing_resampler_benchmark [filter-substring]
#include <zcore/routing/audio_history.h>
#include <zcore/routing/audio_resampler.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

namespace {
using namespace zcore::routing;
using Clock = std::chrono::steady_clock;

[[noreturn]] void fail(const char* message) {
    std::fprintf(stderr, "%s\n", message);
    std::abort();
}

// Deterministic white noise in [-0.5, 0.5): every tap carries signal, as in
// music, and results do not depend on the platform's random library.
struct Noise {
    uint64_t state = 0x9e3779b97f4a7c15ULL;
    float next() noexcept {
        state = state * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<float>(static_cast<int32_t>(state >> 32)) * 0x1p-32f;
    }
};

struct Signal {
    uint32_t channels;
    uint64_t origin, frames;
    std::vector<float> samples; // interleaved
    static ResampleFrame read(const void* context, uint64_t position,
                              std::span<float> values) noexcept {
        const auto& self = *static_cast<const Signal*>(context);
        if (position < self.origin || position - self.origin >= self.frames)
            return {false, 0};
        const auto* frame = self.samples.data() + (position - self.origin) * self.channels;
        for (size_t channel = 0; channel < values.size(); ++channel)
            values[channel] = frame[channel];
        return {true, static_cast<uint32_t>(values.size())};
    }
};

struct Timing {
    double minimum, median;
};
template <class Run> Timing measure(uint64_t frames, Run&& run) {
    std::array<double, 7> samples{};
    for (auto& sample : samples) {
        const auto start = Clock::now();
        run();
        sample = std::chrono::duration<double, std::nano>(Clock::now() - start).count() /
                 static_cast<double>(frames);
    }
    std::sort(samples.begin(), samples.end());
    return {samples.front(), samples[samples.size() / 2]};
}
void report(const char* name, Timing timing, double outputRate, double checksum) {
    // Share of one core needed to produce this stream in real time.
    std::printf("%-36s %9.1f ns/frame (median %9.1f)  %6.3f %% of a core  checksum %.9g\n", name,
                timing.minimum, timing.median, timing.minimum * outputRate / 1e7, checksum);
    std::fflush(stdout);
}

struct DirectCase {
    const char* name;
    uint32_t channels, frames;
    double sourceRate, outputRate;
};
void runDirect(const DirectCase& test) {
    constexpr uint64_t origin = 1'000'000'000'000ULL, length = 1 << 16, margin = 4096;
    Signal signal{test.channels, origin, length, {}};
    signal.samples.resize(length * test.channels);
    Noise noise;
    for (auto& sample : signal.samples)
        sample = noise.next();
    std::array<std::array<float, 256>, kMaximumRoutingChannels> planes{};
    std::array<float*, kMaximumRoutingChannels> pointers{};
    for (uint32_t channel = 0; channel < test.channels; ++channel)
        pointers[channel] = planes[channel].data();
    const ResampleOutput output{std::span<float* const>(pointers.data(), test.channels),
                                test.frames};
    const AudioResampler resampler;
    const long double step = static_cast<long double>(test.sourceRate) / test.outputRate;
    const long double span = static_cast<long double>(length - 2 * margin) -
                             static_cast<long double>(test.frames) * step;
    const uint32_t blocks = std::max<uint32_t>(64, 200'000 / test.frames);
    double checksum = 0;
    uint64_t missing = 0;
    const auto timing = measure(static_cast<uint64_t>(blocks) * test.frames, [&] {
        // Consecutive blocks continue one stream, so the fractional phase
        // walks exactly as it does for a device; wrap inside the test signal.
        long double offset = margin + .375L;
        for (uint32_t block = 0; block < blocks; ++block) {
            const ResamplePosition position{origin, offset, test.sourceRate, test.outputRate};
            missing += resampler.read(position, output, test.frames, Signal::read, &signal);
            checksum += planes[0][test.frames / 2];
            offset += static_cast<long double>(test.frames) * step;
            if (offset >= margin + span)
                offset -= span;
        }
    });
    if (missing)
        fail("direct benchmark lost resampling support");
    report(test.name, timing, test.outputRate, checksum);
}

struct HistoryCase {
    const char* name;
    uint32_t channels;
    double captureRate, readerRate; // actual clocks; the format is 48 kHz
};
void runHistory(const HistoryCase& test) {
    constexpr uint32_t nominal = 48000, captureFrames = 256, readFrames = 256;
    constexpr uint64_t origin = 9'000'000'000ULL;
    constexpr uint32_t warmBlocks = 400, blocks = 1200;
    std::array<std::array<float, captureFrames>, 2> input{};
    std::array<std::array<float, readFrames>, 2> planes{};
    const std::array<const float*, 2> source{input[0].data(), input[1].data()};
    const std::array<float*, 2> destination{planes[0].data(), planes[1].data()};
    std::array<double, 7> samples{};
    double checksum = 0;
    for (auto& sample : samples) {
        AudioHistory history(test.channels);
        AudioReadCursor cursor;
        Noise noise;
        uint64_t sourceFrame = 0, missing = 0;
        long double captured = 0, reader = 0;
        std::chrono::nanoseconds elapsed{0};
        for (uint32_t block = 0; block < warmBlocks + blocks; ++block) {
            for (auto& plane : input)
                for (auto& value : plane)
                    value = noise.next();
            if (!history.append({{source.data(), test.channels}, captureFrames}, captureFrames,
                                origin + static_cast<uint64_t>(captured * 1e9L), nominal,
                                static_cast<double>(sourceFrame), true))
                fail("history append failed");
            sourceFrame += captureFrames;
            captured += static_cast<long double>(captureFrames) / test.captureRate;
            // The reader trails the capture by about 2048 frames, like a
            // monitoring path that keeps the filter's future wing available.
            while (reader + static_cast<long double>(readFrames + 2048) / test.readerRate <
                   captured) {
                const auto first = origin + static_cast<uint64_t>(reader * 1e9L);
                const bool timed = block >= warmBlocks;
                const auto start = timed ? Clock::now() : Clock::time_point{};
                const auto result = history.read(first, test.readerRate,
                                                 {{destination.data(), test.channels}, readFrames},
                                                 readFrames, cursor);
                if (timed) {
                    elapsed += Clock::now() - start;
                    missing += result.missingFrames;
                    checksum += planes[0][readFrames / 2];
                }
                reader += static_cast<long double>(readFrames) / test.readerRate;
            }
        }
        if (missing)
            fail("history benchmark lost resampling support");
        // Reads and capture blocks are both 256 frames at nearly one rate.
        sample = static_cast<double>(elapsed.count()) /
                 (static_cast<double>(blocks) * readFrames * test.readerRate / test.captureRate);
    }
    std::sort(samples.begin(), samples.end());
    report(test.name, {samples.front(), samples[samples.size() / 2]}, test.readerRate, checksum);
}
} // namespace

int main(int argc, char** argv) {
    const char* filter = argc > 1 ? argv[1] : "";
    const DirectCase direct[] = {
        // Two devices at one nominal rate: the reader is faster, so the
        // filter keeps the source cutoff.
        {"direct 2ch 48k->48k +100ppm x256", 2, 256, 48000, 48000 * 1.0001},
        // The source is faster: the cutoff follows the measured ratio.
        {"direct 2ch 48k->48k -100ppm x256", 2, 256, 48000 * 1.0001, 48000},
        {"direct 2ch 48k->48k -1000ppm x256", 2, 256, 48000 * 1.001, 48000},
        {"direct 2ch 48k->48k +1000ppm x256", 2, 256, 48000, 48000 * 1.001},
        {"direct 2ch 48k->48k -5000ppm x256", 2, 256, 48000 * 1.005, 48000},
        {"direct 1ch 48k->48k -100ppm x256", 1, 256, 48000 * 1.0001, 48000},
        {"direct 2ch 48k->48k -100ppm x64", 2, 64, 48000 * 1.0001, 48000},
        {"direct 2ch 48k->48k -100ppm x32", 2, 32, 48000 * 1.0001, 48000},
        {"direct 2ch 48k->48k -100ppm x16", 2, 16, 48000 * 1.0001, 48000},
        {"direct 10ch 48k->48k -100ppm x256", 10, 256, 48000 * 1.0001, 48000},
        {"direct 2ch 44.1k->48k x256", 2, 256, 44100, 48000},
        {"direct 2ch 44.1k->48k +200ppm x256", 2, 256, 44100, 48000 * 1.0002},
        {"direct 2ch 48k->44.1k x256", 2, 256, 48000, 44100},
        {"direct 2ch 48k->44.1k -200ppm x256", 2, 256, 48000 * 1.0002, 44100},
        {"direct 2ch 48k->48k exact x256", 2, 256, 48000, 48000},
        {"direct 2ch 96k->48k exact x256", 2, 256, 96000, 48000},
    };
    for (const auto& test : direct)
        if (std::strstr(test.name, filter))
            runDirect(test);
    const HistoryCase history[] = {
        {"history 2ch capture +30ppm", 2, 48000 * 1.00003, 48000},
        {"history 2ch capture -30ppm", 2, 48000 * .99997, 48000},
        {"history 2ch reader +50ppm", 2, 48000, 48000 * 1.00005},
        {"history 2ch reader -50ppm", 2, 48000, 48000 * .99995},
        {"history 1ch capture +30ppm", 1, 48000 * 1.00003, 48000},
    };
    for (const auto& test : history)
        if (std::strstr(test.name, filter))
            runHistory(test);
    return 0;
}
