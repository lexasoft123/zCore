#include <zcore/audio/audio_input_transport.h>
#include <zcore/audio/audio_input_timestamp.h>
#include <limits>
#include <zcore/device/audio_input_backend.h>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

namespace {
using namespace singz;
int failures = 0;
#define CHECK(v) do { if (!(v)) { std::fprintf(stderr, "FAIL %d: %s\n", __LINE__, #v); ++failures; } } while (false)
void sameLegacyBlock(const AudioInputBlockView& a, const AudioInputBlockView& b) {
    CHECK(a.frames == b.frames && a.sampleRate == b.sampleRate);
    CHECK(a.sequence == b.sequence && a.sampleHostTimeNs == b.sampleHostTimeNs &&
          a.callbackHostTimeNs == b.callbackHostTimeNs && a.timestampQuality == b.timestampQuality);
    CHECK(a.capture.clockDomainId == b.capture.clockDomainId &&
          a.capture.streamGeneration == b.capture.streamGeneration &&
          a.capture.sourceFrame == b.capture.sourceFrame && a.capture.sequence == b.capture.sequence &&
          a.capture.sampleHostTimeNs == b.capture.sampleHostTimeNs &&
          a.capture.callbackHostTimeNs == b.capture.callbackHostTimeNs &&
          a.capture.timestampQuality == b.capture.timestampQuality &&
          a.capture.discontinuity == b.capture.discontinuity && a.capture.flags == b.capture.flags);
    for (uint32_t i = 0; i < a.frames; ++i) CHECK(a.mono[i] == b.mono[i]);
}
void transportCompatibility() {
    CHECK(resolveAudioInputSourcePosition(true, -128).valid);
    CHECK(resolveAudioInputSourcePosition(true, -128).sampleFrame == -128);
    CHECK(!resolveAudioInputSourcePosition(false, 64).valid);
    CHECK(!resolveAudioInputSourcePosition(true, std::numeric_limits<double>::infinity()).valid);
    CHECK(!resolveAudioInputSourcePosition(true, std::numeric_limits<double>::quiet_NaN()).valid);
    AudioInputRing legacy(2, 64, 11, 22), extended(2, 64, 11, 22);
    std::array<float, 64> pcm{}; pcm.fill(.25F);
    // Hardware host-time gap padding consumers see exactly the same times,
    // metadata, samples and sequences with and without hardware positions.
    for (uint32_t block = 0; block < 12; ++block) {
        const auto ns = 9000000000ULL + static_cast<uint64_t>((block * 64 + (block >= 3 ? 64 : 0)) * 1e9L / 48000);
        const auto position = -128.0 + block * 64;
        CHECK(legacy.push(pcm.data(), 64, ns, ns + 1000000, AudioInputTimestampQuality::Hardware));
        CHECK(extended.producer().pushWithPosition(pcm.data(), 64, ns, ns + 1000000,
              AudioInputTimestampQuality::Hardware, {position, true}));
        AudioInputBlockView a, b;
        CHECK(legacy.peek(a, 48000) && extended.peek(b, 48000));
        sameLegacyBlock(a, b);
        CHECK(!a.sourcePosition.valid && b.sourcePosition.valid && b.sourcePosition.sampleFrame == position);
        CHECK(b.capture.sourceFrame == block * 64); // transport provenance is not AUHAL's clock
        legacy.consume(); extended.consume();
    }
    // Reused slots clear optional metadata on the legacy producer path.
    CHECK(extended.push(pcm.data(), 64, 10000000000ULL));
    AudioInputBlockView b; CHECK(extended.peek(b, 48000)); CHECK(!b.sourcePosition.valid);
    if constexpr (sizeof(void*) == 8) {
        CHECK(sizeof(AudioInputSourcePosition) == 16);
        CHECK(sizeof(AudioInputBlockView) == 128);
        CHECK(offsetof(AudioInputBlockView, sourcePosition) == 112);
        CHECK(offsetof(AudioInputBlockView, frames) == 104);
    }
    std::printf("AudioInputBlockView size %zu, sourcePosition offset %zu, AudioInputSourcePosition size %zu\n",
                sizeof(AudioInputBlockView), offsetof(AudioInputBlockView, sourcePosition), sizeof(AudioInputSourcePosition));
}
struct State {
    AudioInputPush legacy{};
    AudioInputPositionPush positioned{};
    void* context{};
} state;
class Backend final : public AudioInputBackend {
    static AudioInputResult result() {
        AudioInputResult r; r.ok = true; r.sampleRate = 48000;
        r.state = AudioInputState::Running; return r;
    }
public:
    AudioInputResult open(const AudioInputConfig&, AudioInputPush push, void* context) override {
        state = {push, nullptr, context}; return result();
    }
    void setPositionPush(AudioInputPositionPush push) override { state.positioned = push; }
    AudioInputResult start() override { return result(); }
    void stop() override {}
    bool takeFailure(std::string&) override { return false; }
};
std::unique_ptr<AudioInputBackend> backend() { return std::make_unique<Backend>(); }
void facadeDelivery() {
    AudioInputDevice device; device.uid = "position:fake"; device.channels = 1; device.sampleRate = 48000;
    setAudioInputBackendForTests(backend, {device});
    std::atomic<bool> delivered{false};
    AudioInputBlockView received;
    float receivedSample = 0;
    AudioInput input;
    CHECK(input.start({device.uid, 0, 8}, [&](const AudioInputBlockView& block) {
        received = block; receivedSample = block.mono[0];
        delivered.store(true, std::memory_order_release);
    }).ok);
    CHECK(state.positioned != nullptr);
    std::array<float, 64> pcm{}; pcm.fill(.375F);
    CHECK(state.positioned(state.context, pcm.data(), 64, 9000000000ULL, 9001000000ULL,
                            AudioInputTimestampQuality::Hardware, {-64, true}));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!delivered.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    input.stop();
    CHECK(delivered.load() && received.sourcePosition.valid && received.sourcePosition.sampleFrame == -64);
    CHECK(received.sampleHostTimeNs == 9000000000ULL && received.capture.sampleHostTimeNs == 9000000000ULL);
    CHECK(received.callbackHostTimeNs == 9001000000ULL && receivedSample == .375F && received.capture.sourceFrame == 0);
    setAudioInputBackendForTests(nullptr, {});
}
}
int main() { transportCompatibility(); facadeDelivery(); return failures ? 1 : 0; }
