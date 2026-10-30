// Compare the generated steady-state traversal with its upstream automation path.
#include "../src/runtime/builtin_nodes.cpp"
#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <cstdio>
#include <limits>

namespace {
void checkLimiterHostModes() {
    using namespace zdsp;
    const auto check = [] {
        constexpr std::array<uint32_t, 8> encodings{
            1, 0x80000001U, 0x007fffffU, 0x807fffffU, 0, 0x80000000U, 0x3f800000U, 0xbf800000U};
        for (float threshold : {0.0f, 0.5f, std::numeric_limits<float>::denorm_min(),
                                std::numeric_limits<float>::min()}) {
            std::array<float, 8> source{}, actual{}, expected{};
            for (size_t i = 0; i < source.size(); ++i)
                source[i] = std::bit_cast<float>(encodings[i]);
            const float* read = source.data();
            float* write = actual.data();
            float* reference = expected.data();
            ConstAudioBusView in{&read, 1, {8}, {8}, nullptr};
            MutableAudioBusView out{&write, 1, {8}, {8}}, ref{&reference, 1, {8}, {8}};
            BuiltinState a{}, b{};
            a.config.node = b.config.node = {1};
            a.config.kind = b.config.kind = BuiltinNodeKind::SafetyLimiter;
            a.currentValue = b.currentValue = threshold;
            ProcessContext context{};
            context.frames = {8};
            // Prevent the compiler from specializing calls across mode changes.
            auto* volatile process = &processLimiter;
            process(&a, &context, &in, out);
            ParameterEvent unrelated{};
            unrelated.node = {2};
            context.parameters = &unrelated;
            context.parameterCount = 1;
            process(&b, &context, &in, ref);
            for (size_t i = 0; i < actual.size(); ++i)
                assert(std::bit_cast<uint32_t>(actual[i]) == std::bit_cast<uint32_t>(expected[i]));
            const auto meter = builtinMeter({&a, &kFunctions});
            const auto referenceMeter = builtinMeter({&b, &kFunctions});
            assert(meter.frames == referenceMeter.frames && meter.peak == referenceMeter.peak);
            assert(std::abs(meter.rms - referenceMeter.rms) <=
                   2 * std::numeric_limits<float>::epsilon() * std::max(1.f, referenceMeter.rms));
        }
    };
#if defined(__SSE2__) || defined(_M_X64)
    const auto original = _mm_getcsr();
    for (uint32_t mode : {0U, 0x8000U, 0x0040U, 0x8040U}) {
        _mm_setcsr((original & ~0x8040U) | mode);
        check();
    }
    _mm_setcsr(original);
    assert(_mm_getcsr() == original);
#elif defined(__aarch64__)
    uint64_t original;
    asm volatile("mrs %0, fpcr" : "=r"(original));
    for (uint64_t mode : {uint64_t(0), uint64_t(1) << 24}) {
        const auto control = (original & ~(uint64_t(1) << 24)) | mode;
        asm volatile("msr fpcr, %0\n\tisb" : : "r"(control) : "memory");
        check();
    }
    asm volatile("msr fpcr, %0\n\tisb" : : "r"(original) : "memory");
    uint64_t restored;
    asm volatile("mrs %0, fpcr" : "=r"(restored));
    assert(restored == original);
#else
    check();
#endif
}
} // namespace

int main() {
    using namespace zdsp;
    // Independent CRT oracle: changing the common helper also changes both
    // processing paths below, so their agreement alone cannot verify it.
    const auto checkEncoding = [](uint32_t bits) {
        const float value = std::bit_cast<float>(bits);
        const bool finite = std::isfinite(value);
        for (const auto initial : {0U, UINT32_MAX - 1, UINT32_MAX}) {
            BuiltinState state{};
            state.nonFinite = initial;
            const auto result = finiteSample(value, &state);
            assert(std::bit_cast<uint32_t>(result) == (finite ? bits : 0U));
            assert(state.nonFinite == initial + uint32_t(!finite && initial != UINT32_MAX));
        }
    };
    for (uint32_t sign : {0U, 0x80000000U})
        for (uint32_t exponent = 0; exponent < 256; ++exponent) {
            for (uint32_t mantissa = 0; mantissa < 512; ++mantissa)
                checkEncoding(sign | (exponent << 23) | (mantissa << 14));
            checkEncoding(sign | (exponent << 23) | 0x7fffffU);
            checkEncoding(sign | (exponent << 23) | 1U);
        }
    constexpr std::array<uint32_t, 12> patterns{0,          0x80000000, 0x3d4ccccd, 0xbd4ccccd,
                                                0x7f800000, 0xff800000, 0x7fc12345, 0x7f812345,
                                                1,          0x80000001, 0x7f7fffff, 0xff7fffff};
    uint32_t cases = 0;
    for (const bool limiter : {false, true})
        for (const auto channels : {1U, 2U, 8U, 64U})
            for (const auto frames : {0U, 1U, 3U, 4U, 5U, 7U, 32U, 64U, 128U, 256U})
                for (const auto initial : {0U, UINT32_MAX - 3, UINT32_MAX})
                    for (const float value : {0.0f, 0.5f, 0.98f, 2.0f})
                        for (const bool pendingRamp : {false, true}) {
                            std::array<std::array<float, 256>, 64> input{}, actual{}, expected{};
                            std::array<const float *, 64> read{};
                            std::array<float *, 64> write{}, reference{};
                            for (uint32_t channel = 0; channel < channels; ++channel) {
                                read[channel] = input[channel].data();
                                write[channel] = actual[channel].data();
                                reference[channel] = expected[channel].data();
                                for (uint32_t frame = 0; frame < 256; ++frame)
                                    input[channel][frame] = std::bit_cast<float>(
                                        patterns[(frame + channel) % patterns.size()]);
                            }
                            BuiltinState a{}, b{};
                            a.config.node = b.config.node = {1};
                            a.currentValue = b.currentValue = value;
                            a.nonFinite = b.nonFinite = initial;
                            if (pendingRamp) {
                                a.rampTarget = b.rampTarget = .25f;
                                a.rampStep = b.rampStep = (.25f - value) / 5;
                                a.rampRemaining = b.rampRemaining = 5;
                            }
                            ConstAudioBusView in{read.data(), channels, {frames}, {256}, nullptr};
                            MutableAudioBusView out{write.data(), channels, {frames}, {256}};
                            MutableAudioBusView ref{reference.data(), channels, {frames}, {256}};
                            ProcessContext context{};
                            context.frames = {frames};
                            const auto process = limiter ? processLimiter : processGain;
                            process(&a, &context, &in, out);
                            // An event for another node selects the unchanged upstream
                            // loop without altering this processor's gain or threshold.
                            ParameterEvent unrelated{};
                            unrelated.node = {2};
                            context.parameters = &unrelated;
                            context.parameterCount = 1;
                            process(&b, &context, &in, ref);
                            assert(a.nonFinite == b.nonFinite);
                            assert(a.currentValue == b.currentValue);
                            assert(a.rampRemaining == b.rampRemaining);
                            if (limiter) {
                                const auto fast = builtinMeter({&a, &kFunctions});
                                const auto referenceMeter = builtinMeter({&b, &kFunctions});
                                assert(fast.frames == frames && referenceMeter.frames == frames);
                                assert(std::bit_cast<uint32_t>(fast.peak) ==
                                       std::bit_cast<uint32_t>(referenceMeter.peak));
                                // Independent double SIMD sums may associate positive
                                // squares differently; final RMS retains float precision (relative
                                // epsilon bound).
                                const auto scale =
                                    std::max(double(fast.rms), double(referenceMeter.rms));
                                assert(std::isfinite(fast.rms) &&
                                       std::isfinite(referenceMeter.rms));
                                assert(std::abs(double(fast.rms) - referenceMeter.rms) <=
                                       2 * std::numeric_limits<float>::epsilon() * scale);
                            }
                            for (uint32_t channel = 0; channel < channels; ++channel)
                                for (uint32_t frame = 0; frame < 256; ++frame)
                                    assert(std::bit_cast<uint32_t>(actual[channel][frame]) ==
                                           std::bit_cast<uint32_t>(expected[channel][frame]));
                            ++cases;
                        }
    // A separate finite-input oracle catches accidentally metering the clipped
    // output, and covers in-place processing, nonfinite containment and -0.
    {
        std::array<float, 8> samples{-2.f,
                                     3.f,
                                     .25f,
                                     -0.f,
                                     std::numeric_limits<float>::infinity(),
                                     std::numeric_limits<float>::quiet_NaN(),
                                     -1.f,
                                     .5f};
        const auto input = samples;
        const float *read = samples.data();
        float *write = samples.data();
        ConstAudioBusView in{&read, 1, {8}, {8}, nullptr};
        MutableAudioBusView out{&write, 1, {8}, {8}};
        BuiltinState state{};
        state.config.kind = BuiltinNodeKind::SafetyLimiter;
        state.currentValue = .5f;
        state.nonFinite = UINT32_MAX - 1;
        ProcessContext context{};
        context.frames = {8};
        processLimiter(&state, &context, &in, out);
        double energy = 0;
        for (uint32_t i = 0; i < samples.size(); ++i) {
            const auto finite = std::isfinite(input[i]) ? input[i] : 0.f;
            energy += double(finite) * finite;
            float expected = finite;
            if (expected > .5f)
                expected = .5f;
            if (expected < -.5f)
                expected = -.5f;
            assert(std::bit_cast<uint32_t>(samples[i]) == std::bit_cast<uint32_t>(expected));
        }
        const auto meter = builtinMeter({&state, &kFunctions});
        assert(meter.peak == 3.f && meter.frames == 8 && state.nonFinite == UINT32_MAX);
        assert(std::abs(double(meter.rms) - std::sqrt(energy / samples.size())) < 1e-6);
    }
    // A real threshold ramp must retain frame-exact automation and meter the
    // unchanged input rather than the progressively clipped output.
    {
        std::array<float, 6> source{1, 1, 1, 1, 1, 1}, output{};
        const float *read = source.data();
        float *write = output.data();
        ConstAudioBusView in{&read, 1, {6}, {6}, nullptr};
        MutableAudioBusView out{&write, 1, {6}, {6}};
        BuiltinState state{};
        state.config.node = {1};
        state.config.kind = BuiltinNodeKind::SafetyLimiter;
        state.currentValue = .5f;
        ParameterEvent event{};
        event.node = {1};
        event.parameter = kLimiterThresholdParameter;
        event.sampleOffset = {2};
        event.value = .25f;
        event.curve = ParameterCurve::Linear;
        event.rampFrames = {2};
        ProcessContext context{};
        context.frames = {6};
        context.parameters = &event;
        context.parameterCount = 1;
        processLimiter(&state, &context, &in, out);
        const std::array<float, 6> expected{.5f, .5f, .5f, .375f, .25f, .25f};
        assert(output == expected && state.rampRemaining == 0 && state.currentValue == .25f);
        const auto meter = builtinMeter({&state, &kFunctions});
        assert(meter.peak == 1.f && meter.rms == 1.f && meter.frames == 6);
    }
    checkLimiterHostModes();
    std::printf("SingZ planar builtins: %u exact output/diagnostic comparisons passed\n", cases);
}
