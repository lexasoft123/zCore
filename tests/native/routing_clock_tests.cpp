#include <zcore/routing/audio_history.h>
#include <zcore/routing/audio_resampler.h>
#include <zcore/routing/sample_clock.h>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <new>
#include <numbers>
#include <vector>

namespace {
bool trackHeap = false;
uint64_t heapEvents = 0;
}
#if defined(__GNUC__)
#define HEAP_HOOK __attribute__((noinline))
#else
#define HEAP_HOOK
#endif
HEAP_HOOK void* operator new(std::size_t bytes) {
    if (trackHeap) ++heapEvents;
    if (void* p = std::malloc(bytes ? bytes : 1)) return p;
    throw std::bad_alloc();
}
HEAP_HOOK void* operator new[](std::size_t bytes) { return ::operator new(bytes); }
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

#define CHECK(value) do { if (!(value)) { \
  std::fprintf(stderr, "CHECK %s:%d: %s\n", __FILE__, __LINE__, #value); \
  std::abort(); } } while (false)

namespace {
using namespace zcore::routing;
uint64_t ns(uint64_t frame, double rate) {
    return static_cast<uint64_t>(static_cast<long double>(frame) * 1e9L / rate);
}
double tone(long double time, uint32_t channel) {
    return static_cast<double>(0.25L * std::sin(2 * std::numbers::pi_v<long double> *
                                              (997 + channel * 74) * time));
}
template<class F> auto noHeap(F operation) {
    const auto before = heapEvents;
    trackHeap = true;
    const auto result = operation();
    trackHeap = false;
    CHECK(heapEvents == before);
    return result;
}
void driftingCaptureAndOutput() {
    AudioHistory history(2);
    constexpr uint32_t nominal = 48000, captureFrames = 480, renderFrames = 64;
    std::array<float, captureFrames> left{}, right{};
    const std::array<const float*, 2> input{left.data(), right.data()};
    std::array<float, renderFrames> a{}, b{};
    const std::array<float*, 2> output{a.data(), b.data()};
    constexpr uint64_t origin = 9000000000ULL;
    uint64_t sourceFrame = 0;
    long double elapsed = 0;
    AudioReadCursor reader;
    double minimumSnr = 300, maximumRateErrorPpm = 0, maximumError = 0;
    uint32_t verified = 0;
    for (double ppm : {200.0, -200.0, 100.0}) {
        const double actual = nominal * (1 + ppm * 1e-6);
        const double outputRate = nominal * (1 - ppm * 0.5e-6);
        const auto phaseStart = elapsed;
        for (uint32_t block = 0; block < 250; ++block) {
            for (uint32_t frame = 0; frame < captureFrames; ++frame) {
                const auto time = elapsed + static_cast<long double>(frame) / actual;
                left[frame] = static_cast<float>(tone(time, 0));
                right[frame] = static_cast<float>(tone(time, 1));
            }
            const auto timestamp = origin + static_cast<uint64_t>(elapsed * 1e9L);
            CHECK(noHeap([&] { return history.append({input, captureFrames}, captureFrames,
                timestamp, nominal, static_cast<double>(sourceFrame), true); }));
            if (elapsed > .25L && block % 10 == 0) {
                const bool settled = elapsed - phaseStart > 1.1L;
                const auto rateError = std::abs(history.status().measuredRate / actual - 1) * 1e6;
                if (settled) {
                    CHECK(rateError < .02);
                    maximumRateErrorPpm = std::max(maximumRateErrorPpm, rateError);
                }
                const auto first = timestamp - ns(1024, nominal);
                const auto result = noHeap([&] {
                    return history.read(first, outputRate, {output, renderFrames}, renderFrames, reader);
                });
                CHECK(result.online && result.missingFrames == 0);
                for (uint32_t channel = 0; channel < 2; ++channel) {
                    double error = 0, reference = 0;
                    for (uint32_t frame = 0; frame < renderFrames; ++frame) {
                        const auto expected = tone(static_cast<long double>(first-origin)/1e9L +
                            static_cast<long double>(frame)/outputRate, channel);
                        const auto value = channel ? b[frame] : a[frame];
                        CHECK(std::isfinite(value) && std::abs(value) < .251);
                        maximumError = std::max(maximumError, std::abs(value-expected));
                        error += (value-expected)*(value-expected);
                        reference += expected*expected;
                    }
                    if (settled) minimumSnr = std::min(minimumSnr,
                        10*std::log10(reference/std::max(error,1e-30)));
                }
                verified += settled;
            }
            sourceFrame += captureFrames;
            elapsed += static_cast<long double>(captureFrames)/actual;
        }
    }
    CHECK(verified >= 39 && minimumSnr > 90 && maximumError < .00001);
    CHECK(history.status().discontinuities == 0);
    std::printf("Split capture/output clocks: %.6f ppm error, %.2f dB SNR, %u checked blocks\n",
        maximumRateErrorPpm, minimumSnr, verified);
}
void silenceChannelsAndRestart() {
    AudioHistory history(2, 2048);
    std::array<float,256> input{};input.fill(.125f);
    const std::array<const float*,1> in{input.data()};
    std::array<float,64> left{},right{};
    const std::array<float*,2> out{left.data(),right.data()};
    AudioReadCursor cursor;
    auto checkZero = [&] { for (size_t i=0;i<64;++i) CHECK(left[i]==0 && right[i]==0); };
    left.fill(1);right.fill(1);
    auto result=noHeap([&] { return history.read(1000000000,48000,{out,64},64,cursor); });
    CHECK(!result.online && result.missingFrames==64);checkZero();
    constexpr uint64_t origin=1000000000;
    CHECK(history.append({in,256},256,origin,48000,0,true));
    CHECK(history.append({in,256},256,origin+ns(256,48000),48000,256,true));
    result=noHeap([&] { return history.read(origin+ns(128,48000),48000,{out,64},64,cursor); });
    CHECK(result.online && result.missingFrames==64);
    for(size_t i=0;i<64;++i) CHECK(std::abs(left[i]-.125f)<.00001f && right[i]==0);
    left.fill(1);right.fill(1);
    result=noHeap([&] { return history.read(origin+100000000,48000,{out,64},64,cursor); });
    CHECK(result.missingFrames==64);checkZero();
    history.stop();
    left.fill(1);right.fill(1);
    result=history.read(origin+ns(128,48000),48000,{out,64},64,cursor);
    CHECK(!result.online && result.missingFrames==64);checkZero();
    const auto restart=origin+1000000000;
    input.fill(-.25f);
    CHECK(history.append({in,256},256,restart,48000,0,true));
    CHECK(history.append({in,256},256,restart+ns(256,48000),48000,256,true));
    result=history.read(origin+ns(128,48000),48000,{out,64},64,cursor);
    CHECK(result.missingFrames==64);checkZero();
    result=noHeap([&] { return history.read(restart+ns(128,48000),48000,{out,64},64,cursor); });
    CHECK(result.online && result.missingFrames==64);
    for(size_t i=0;i<64;++i)CHECK(std::abs(left[i]+.25f)<.00001f && right[i]==0);
    input.fill(std::numeric_limits<float>::quiet_NaN());
    const auto gap=restart+1000000000;
    CHECK(history.append({in,256},256,gap,48000,512,true));
    CHECK(history.append({in,256},256,gap+ns(256,48000),48000,768,true));
    CHECK(history.status().discontinuities>=1);
    result=history.read(gap+ns(128,48000),48000,{out,64},64,cursor);
    CHECK(result.online && result.missingFrames==64);checkZero();
    result=history.read(restart+ns(128,48000),48000,{out,64},64,cursor);
    CHECK(result.missingFrames==64);checkZero();
}
}
int main() {
    driftingCaptureAndOutput();
    silenceChannelsAndRestart();
    std::puts("Capture history drift, silence, absent channels, restart and nonfinite checks passed");
}
