#include <zcore/routing/audio_history.h>
#include "allocation_trap.h"
#include <zcore/routing/audio_resampler.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace zcore::routing;
namespace {
constexpr uint64_t origin = 9000000000ULL;
uint64_t ns(uint64_t frame) { return static_cast<uint64_t>(frame * 1e9L / 48000); }
void require(bool value, const char* message) {
    if (!value) { std::fprintf(stderr, "FAIL: %s\n", message); std::exit(1); }
}
template<class F> auto noHeap(F operation) {
    zdsp::test::resetAllocationTrap();
    zdsp::test::setAllocationTrapEnabled(true);
    const auto result = operation();
    zdsp::test::setAllocationTrapEnabled(false);
    require(zdsp::test::trappedAllocationCount() == 0, "phase tolerance: callback allocated");
    return result;
}
uint64_t phaseStep(uint32_t period, int step, SampleClockPhaseConfig config, bool checkOutput) {
    AudioHistory history(1, 32768, ResamplingFilter::LinearPhase, true, config);
    std::array<float, 128> input{}, output{};
    const std::array<const float*, 1> in{input.data()};
    const std::array<float*, 1> out{output.data()};
    AudioReadCursor cursor;
    uint64_t missing = 0;
    float previous = 0;
    bool havePrevious = false;
    for (uint64_t frame = 0; frame < 8192; frame += period) {
        for (uint32_t i = 0; i < period; ++i) input[i] = float(frame + i) / 16384;
        const auto timestamp = origin + ns(frame) +
            (frame >= 4096 ? static_cast<int64_t>(ns(1)) * step : 0);
        require(noHeap([&] { return history.append({in, period}, period, timestamp, 48000); }), "phase append");
        if (frame <= 2048) continue;
        const auto result = noHeap([&] {
            return history.read(origin + ns(frame - 1024), 48000, {out, period}, period, cursor);
        });
        missing += result.missingFrames;
        if (checkOutput) {
            require(result.missingFrames == 0, "phase tolerance: one-sample step lost audio");
            for (uint32_t i = 0; i < period; ++i) {
                const auto expected = float(frame - 1024 + i) / 16384;
                require(std::isfinite(output[i]) &&
                        std::abs(output[i] - expected) <= 2.01F / 16384,
                        "phase tolerance: output displacement exceeded sample bound");
                if (havePrevious)
                    require(output[i] >= previous && output[i] - previous <= 3.01F / 16384,
                            "phase tolerance: unbounded sample duplication or skip");
                previous = output[i]; havePrevious = true;
            }
        }
    }
    const auto status = history.status();
    const auto diagnostics = history.readDiagnostics();
    require(status.discontinuities == 0, "phase tolerance: unexpected reset");
    require(status.phaseCorrections == diagnostics.phaseCorrections &&
            status.correctedSamples == diagnostics.correctedSamples &&
            status.rejectedIntervals == diagnostics.rejectedIntervals,
            "phase tolerance: diagnostic counters disagree");
    if (checkOutput)
        require(status.phaseCorrections == 1 && status.correctedSamples == 1 &&
                status.rejectedIntervals == 0, "phase tolerance: correction accounting");
    return missing;
}
void reproductions() {
    for (uint32_t period : {32u, 64u, 128u}) {
        SampleClockPhaseConfig disabled; disabled.maximumStepSamples = 0;
        const auto before = phaseStep(period, 1, disabled, false);
        require(before == period + 1, "negative control must reproduce 33/65/129 missing frames");
        const auto after = phaseStep(period, 1, {}, true);
        require(after == 0, "phase tolerance: positive step missing frames");
        require(phaseStep(period, -1, {}, true) == 0,
                "phase tolerance: negative step missing frames");
        std::printf("period %u: missing %llu -> %llu; resets 0; corrections 1; correctedSamples 1\n",
                    period, static_cast<unsigned long long>(before),
                    static_cast<unsigned long long>(after));
    }
}
void bounds() {
    // 64 samples is below the old 2 ms generation-reset threshold, but must
    // remain an unusable interval. Use 512 frames so the local slope guard
    // alone cannot rescue a mutant which removes the actual sample bound.
    SampleClockHistory clock(16);
    require(clock.append(0, origin, false, 48000, 48000), "bound first append");
    require(clock.append(512, origin + ns(576), false, 48000, 48000), "bound second append");
    const auto result = clock.locate(clock.snapshot(), origin + ns(256), 0, 48000, 48000);
    require(!result.valid, "phase bound: 64-sample interval was incorrectly accepted");
    require(clock.phaseDiagnostics().phaseCorrections == 0 &&
            clock.phaseDiagnostics().rejectedIntervals == 1, "phase bound: rejected interval accounting");
    AudioHistory history(1, 2048, ResamplingFilter::LinearPhase, true);
    std::array<float, 512> input{}; input.fill(.25F);
    std::array<float, 64> output{};
    const std::array<const float*, 1> in{input.data()};
    const std::array<float*, 1> out{output.data()};
    AudioReadCursor cursor;
    require(history.append({in, 512}, 512, origin, 48000), "large step first block");
    require(history.append({in, 512}, 512, origin + ns(576), 48000), "large step second block");
    const auto missing = history.read(origin + ns(256), 48000, {out, 64}, 64, cursor);
    require(missing.missingFrames == 64 && history.status().rejectedIntervals == 1,
            "phase bound: larger timestamp discontinuity lost its rejection path");
    require(history.append({in, 512}, 512, origin + ns(1280), 48000), "large gap block");
    require(history.status().discontinuities == 1, "phase bound: >2 ms discontinuity must reset");
    // The absolute time bound is 0.1 ms even with a larger sample setting.
    SampleClockHistory timeBound(16, {16, 8});
    require(timeBound.append(0, origin, false, 48000, 48000), "time bound first append");
    require(timeBound.append(512, origin + ns(520), false, 48000, 48000), "time bound second append");
    require(!timeBound.locate(timeBound.snapshot(), origin + ns(256), 0, 48000, 48000).valid,
            "phase bound: absolute 0.1 ms bound exceeded");
}
void budget() {
    SampleClockHistory clock(32, {2, 2});
    for (uint64_t block = 0; block <= 6; ++block) {
        const auto shifts = (block + 1) / 2;
        require(clock.append(block * 32, origin + ns(block * 32 + shifts), false, 48000, 48000),
                "budget append");
    }
    auto d = clock.phaseDiagnostics();
    require(d.phaseCorrections == 2 && d.correctedSamples == 2 && d.rejectedIntervals == 1,
            "phase budget: third correction in rolling second must be rejected");
    // At the same timestamp, neither valid raw intervals nor a reset replenish
    // the rolling budget. A fresh second does, without changing the config.
    clock.reset();
    require(clock.append(0, origin + 500000000ULL, false, 48000, 48000), "budget reset anchor");
    require(clock.append(32, origin + 500000000ULL + ns(33), false, 48000, 48000), "budget boundary append");
    require(clock.phaseDiagnostics().phaseCorrections == 2, "phase budget: rolling second reopened early");
    require(clock.append(48032, origin + 1500000000ULL + ns(33), false, 48000, 48000), "budget normal append");
    require(clock.append(48064, origin + 1500000000ULL + ns(66), false, 48000, 48000), "budget replenished append");
    require(clock.phaseDiagnostics().phaseCorrections == 3, "phase budget: did not replenish after second");
}
void sourceCounter() {
    AudioHistory history(1, 2048, ResamplingFilter::LinearPhase, true);
    std::array<float, 64> input{}; input.fill(.25F);
    const std::array<const float*, 1> in{input.data()};
    // A real counter gap must win over timestamp tolerance, including a
    // timestamp which would otherwise qualify for a one-sample correction.
    require(history.append({in, 64}, 64, origin, 48000, -64, true), "counter first append");
    require(history.append({in, 64}, 64, origin + ns(65), 48000, 0, true), "counter continuous append");
    require(history.status().discontinuities == 0 && history.status().rejectedIntervals == 0,
            "counter continuity: verified phase interval rejected");
    require(history.append({in, 64}, 64, origin + ns(130), 48000, 65, true), "counter gap append");
    require(history.status().discontinuities == 1,
            "counter continuity: hardware gap hidden by phase tolerance");
}
}
int main(int argc, char** argv) {
    if (argc == 2 && std::strcmp(argv[1], "bound") == 0) { bounds(); return 0; }
    reproductions(); bounds(); budget(); sourceCounter();
    std::puts("Bounded capture phase tolerance checks passed");
}
