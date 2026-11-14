// Optional consumer integration: supply ZCORE_CAPTURE_TAKE_WRITER_SOURCE.
#include "take_writer.h"
#include <zcore/audio/audio_input_transport.h>
#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <vector>

int main(int argc, char** argv) {
    using namespace singz;
    if (argc != 2) return 2;
    const std::filesystem::path directory(argv[1]);
    std::filesystem::create_directories(directory);
    TakeWriter baseline(32, 64), extended(32, 64);
    baseline.start(directory.string(), "baseline", 48000);
    extended.start(directory.string(), "extended", 48000);
    AudioInputRing oldRing(2, 64, 1, 1), newRing(2, 64, 1, 1);
    std::array<float, 64> pcm{};
    for (uint32_t block = 0; block < 12; ++block) {
        for (uint32_t i = 0; i < 64; ++i) pcm[i] = float(block * 64 + i) / 1024;
        // A genuine hardware sample counter remains continuous, while host
        // time steps 512 frames. Recording must retain its existing gap rule.
        const auto frame = block * 64 + (block >= 4 ? 512 : 0);
        const auto host = 9000000000ULL + static_cast<uint64_t>(frame * 1e9L / 48000);
        if (!oldRing.push(pcm.data(), 64, host, host + 1000000, AudioInputTimestampQuality::Hardware) ||
            !newRing.producer().pushWithPosition(pcm.data(), 64, host, host + 1000000,
                AudioInputTimestampQuality::Hardware, {double(block * 64), true})) return 3;
        AudioInputBlockView oldBlock, newBlock;
        if (!oldRing.peek(oldBlock, 48000) || !newRing.peek(newBlock, 48000) ||
            !baseline.append(oldBlock) || !extended.append(newBlock)) return 4;
        oldRing.consume(); newRing.consume();
    }
    const auto a = baseline.finish(), b = extended.finish();
    auto bytes = [](const std::string& path) {
        std::ifstream file(path, std::ios::binary);
        return std::vector<char>(std::istreambuf_iterator<char>(file), {});
    };
    const bool same = a.error.empty() && b.error.empty() && a.gapPaddedFrames == 512 &&
        b.gapPaddedFrames == 512 && a.frames == 1280 && b.frames == 1280 &&
        a.firstSampleHostNs == b.firstSampleHostNs && a.firstSampleQuality == b.firstSampleQuality &&
        bytes(a.path) == bytes(b.path);
    std::filesystem::remove(a.path); std::filesystem::remove(b.path);
    std::filesystem::remove(directory);
    if (!same) { std::fprintf(stderr, "FAIL: hardware position changed take PCM or gap padding\n"); return 1; }
    std::puts("Take compatibility: identical WAV bytes; 512 padded frames; 1280 total frames");
}
