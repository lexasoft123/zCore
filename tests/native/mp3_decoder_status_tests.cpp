// Status regressions ported from SingZ 8bf8a86c native decoder tests, plus
// the security regressions that belong to the same status surface: the MP3
// frame index's TOCTOU revalidation and the WAV header walk's bounded fmt
// read (this is the media layer's one decoder-status host; a per-format
// split would need a CMakeLists that is not ours to touch).
#include <zcore/media/decoded_audio.h>
#include <zcore/media/streaming_audio_source.h>
#include <zcore/media/wav.h>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>
#if defined(_WIN32)
#include <fcntl.h>
#include <io.h>
#else
#include <fcntl.h>
#include <unistd.h>
#endif
namespace {
int failures = 0;
void check(bool ok, const std::string& label) {
  if (!ok) { std::fprintf(stderr, "FAIL: %s\n", label.c_str()); ++failures; }
}
int openRead(const std::string& path) {
#if defined(_WIN32)
  return _open(path.c_str(), _O_RDONLY | _O_BINARY);
#else
  return ::open(path.c_str(), O_RDONLY);
#endif
}
struct Fixture {
  std::string path;
  Fixture(const char* name, const std::vector<unsigned char>& bytes) {
    path = (std::filesystem::temp_directory_path() / name).string();
    auto* file = std::fopen(path.c_str(), "wb");
    check(file != nullptr, "create fixture");
    if (file) {
      check(std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size(), "write fixture");
      std::fclose(file);
    }
  }
  ~Fixture() { std::remove(path.c_str()); }
};
void rejects(const std::vector<unsigned char>& bytes, singz::DecodedAudioStatus expected,
             const std::string& label) {
  Fixture file("zcore-id3-status.mp3", bytes);
  for (auto format : {singz::DecodedAudioSourceFormat::Auto, singz::DecodedAudioSourceFormat::Mp3}) {
    singz::DecodedAudioPrepareOptions options;
    options.sourceFormat = format;
    auto result = singz::prepareDecodedAudio(singz::OwnedFileDescriptor(openRead(file.path)), options);
    check(result.status == expected && !result.audio, label + ": whole-file status/no publication");
    singz::StreamingAudioOpenOptions streaming;
    streaming.sourceFormat = format;
    singz::DecodedAudioStatus status{};
    auto source = singz::openStreamingAudioSource(singz::OwnedFileDescriptor(openRead(file.path)), streaming, &status);
    check(!source && status == expected, label + ": streaming status/no publication");
  }
}
// Rewrites `bytes.size()` bytes at `offset` in place, the same-length swap the
// MP3 TOCTOU regression needs between open and the first read.
void poke(const std::string& path, long offset, const std::vector<unsigned char>& bytes) {
  auto* file = std::fopen(path.c_str(), "r+b");
  check(file != nullptr, "reopen fixture");
  if (file) {
    check(std::fseek(file, offset, SEEK_SET) == 0, "seek fixture");
    check(std::fwrite(bytes.data(), 1, bytes.size(), file) == bytes.size(), "rewrite fixture bytes");
    std::fclose(file);
  }
}
void put16(std::vector<unsigned char>& v, uint16_t x) {
  v.push_back(static_cast<unsigned char>(x & 0xff));
  v.push_back(static_cast<unsigned char>((x >> 8) & 0xff));
}
void put32(std::vector<unsigned char>& v, uint32_t x) {
  for (int i = 0; i < 4; i++) v.push_back(static_cast<unsigned char>((x >> (8 * i)) & 0xff));
}
} // namespace
int main() {
    const std::vector<unsigned char> emptyTag{'I', 'D', '3', 4, 0, 0, 0, 0, 0, 0};
    const std::vector<unsigned char> declaredTag{'I', 'D', '3', 4, 0, 0, 0, 0, 0, 35};
    for (size_t length = 3; length <= declaredTag.size(); ++length) {
      rejects(std::vector<unsigned char>(declaredTag.begin(), declaredTag.begin() + length),
                   singz::DecodedAudioStatus::MalformedData,
                   "truncated recognized ID3 header/tag at " + std::to_string(length) + " bytes");
    }
    auto corrupt = declaredTag;
    corrupt.resize(48);
    for (size_t index = 10; index < corrupt.size(); ++index)
      corrupt[index] = static_cast<unsigned char>(0xa5 ^ index);
    rejects(corrupt, singz::DecodedAudioStatus::MalformedData,
                 "the canonical corrupted tag leaves fewer than four audio bytes");
    for (size_t remaining = 0; remaining < 4; ++remaining) {
      auto noHeader = emptyTag;
      noHeader.insert(noHeader.end(), remaining, 0xff);
      rejects(noHeader, singz::DecodedAudioStatus::MalformedData,
                   "a complete ID3 tag followed by " + std::to_string(remaining) + " header bytes");
    }
    auto chained = emptyTag;
    chained.insert(chained.end(), declaredTag.begin(), declaredTag.end());
    rejects(chained, singz::DecodedAudioStatus::MalformedData,
                 "a complete ID3 tag does not hide a truncated following tag");
    auto footer = emptyTag;
    footer[5] = 0x10;
    footer.insert(footer.end(), 9, 0);
    rejects(footer, singz::DecodedAudioStatus::MalformedData,
                 "an ID3 footer must fit inside the encoded source");

    // Metadata alone does not make an unsupported MPEG profile malformed.
    auto freeFormat = emptyTag;
    freeFormat.insert(freeFormat.end(), {0xff, 0xfb, 0x00, 0x44});
    freeFormat.resize(2048, 0);
    rejects(freeFormat, singz::DecodedAudioStatus::UnsupportedFormat,
                 "ID3-prefixed free-format data remains unsupported");

    // L5 regression: the frame index is built at open and every decode buffer
    // is sized from it, while dr_mp3 sizes its OUTPUT from the header it
    // parses at decode time. A same-length in-place rewrite of the file
    // between open and the first read must therefore be refused BEFORE any
    // decode — not after dr_mp3 has already written past the buffer. Eight
    // synthetic MPEG-2 Layer III frames (128 kbps, 22050 Hz, stereo, 417
    // bytes each, zero payload) pass the open walk; every rewrite below keeps
    // the frame length at 417 bytes, so only the decode-time revalidation
    // catches it.
    std::vector<unsigned char> mp3Frames;
    for (int frame = 0; frame < 8; frame++) {
      mp3Frames.insert(mp3Frames.end(), {0xff, 0xf3, 0xc0, 0x00});
      mp3Frames.insert(mp3Frames.end(), 413, 0);
    }
    float left[64], right[64];
    float* mp3Channels[2] = {left, right};
    size_t framesRead = 0;
    auto openSyntheticMp3 = [&](const std::string& path) {
      singz::StreamingAudioOpenOptions streaming;
      streaming.sourceFormat = singz::DecodedAudioSourceFormat::Mp3;
      singz::DecodedAudioStatus status{};
      auto source = singz::openStreamingAudioSource(singz::OwnedFileDescriptor(openRead(path)), streaming, &status);
      check(source != nullptr && status == singz::DecodedAudioStatus::Ok, "synthetic MPEG-2 stream opens");
      return source;
    };
    {
      Fixture file("zcore-mp3-toctou-intact.mp3", mp3Frames);
      auto source = openSyntheticMp3(file.path);
      check(source && source->read(mp3Channels, 64, &framesRead) == singz::DecodedAudioStatus::Ok &&
                framesRead == 64,
            "the unmodified synthetic stream still decodes");
    }
    auto rewriteMustBeRefused = [&](const char* name, long offset,
                                    const std::vector<unsigned char>& bytes, const std::string& label) {
      Fixture file(name, mp3Frames);
      auto source = openSyntheticMp3(file.path);
      if (!source) return;
      poke(file.path, offset, bytes);
      framesRead = 999;
      const singz::DecodedAudioStatus status = source->read(mp3Channels, 64, &framesRead);
      check(status == singz::DecodedAudioStatus::MalformedData && framesRead == 0,
            "in-place rewrite refused before decode: " + label);
    };
    rewriteMustBeRefused("zcore-mp3-toctou-mode.mp3", 3, {0xc0},
                         "channel-mode flip keeps the frame length, changes the channel count");
    // The overflow direction: a MONO layout whose bytes flip to stereo would
    // make dr_mp3 write 2x the staged buffer (sized 576*1 from the layout).
    {
      std::vector<unsigned char> monoFrames;
      for (int frame = 0; frame < 8; frame++) {
        monoFrames.insert(monoFrames.end(), {0xff, 0xf3, 0xc0, 0xc0});
        monoFrames.insert(monoFrames.end(), 413, 0);
      }
      Fixture file("zcore-mp3-toctou-mode-overflow.mp3", monoFrames);
      auto source = openSyntheticMp3(file.path);
      if (source) {
        poke(file.path, 3, {0x00});
        float mono[64];
        float* monoChannels[1] = {mono};
        size_t monoRead = 999;
        const singz::DecodedAudioStatus status = source->read(monoChannels, 64, &monoRead);
        check(status == singz::DecodedAudioStatus::MalformedData && monoRead == 0,
              "in-place rewrite refused before decode: mono-to-stereo flip is the overflow direction");
      }
    }
    rewriteMustBeRefused("zcore-mp3-toctou-version.mp3", 0, {0xff, 0xfb, 0x90, 0x00},
                         "version flip keeps the frame length (417), doubles samples per frame");
    rewriteMustBeRefused("zcore-mp3-toctou-invalid.mp3", 0, {0x00, 0x00, 0x00, 0x00},
                         "bytes no longer a Layer III header");

    // The WAV header walk's bounded fmt read: a chunk larger than the read cap
    // but physically present (a streaming encoder's extension blob) must be
    // accepted exactly as before, with the walk landing on `data` at the same
    // offset; a stated size that runs past the file must still be refused.
    auto wavWithFmtSize = [&](uint32_t fmtSize) {
      std::vector<unsigned char> wav{'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E',
                                     'f', 'm', 't', ' '};
      put32(wav, fmtSize);
      put16(wav, 1);          // PCM
      put16(wav, 2);          // stereo
      put32(wav, 44100);      // rate
      put32(wav, 44100 * 4);  // byte rate
      put16(wav, 4);          // block align
      put16(wav, 16);         // bits
      wav.resize(wav.size() + (fmtSize - 16), 0);  // extension bytes up to the stated size
      if ((fmtSize & 1u) != 0) wav.push_back(0);   // the RIFF pad an odd chunk carries
      wav.insert(wav.end(), {'d', 'a', 't', 'a'});
      put32(wav, 8);
      wav.insert(wav.end(), 8, 0);                 // two stereo frames
      return wav;
    };
    for (uint32_t fmtSize : {18u, 4098u, 4099u}) {
      Fixture file(("zcore-wav-fmt-" + std::to_string(fmtSize) + ".wav").c_str(), wavWithFmtSize(fmtSize));
      const singz::WavInfo info = singz::readWavInfo(file.path);
      check(info.ok && info.sampleRate == 44100 && info.channels == 2 && info.frames == 2,
            "a physically present fmt chunk of " + std::to_string(fmtSize) +
                " bytes still walks to data");
    }
    {
      // States ~4 GiB, holds 6 KiB: the bounded read succeeds, the remainder
      // seek lands past EOF, and the walk must refuse the file rather than
      // allocate the stated size.
      std::vector<unsigned char> lying{'R', 'I', 'F', 'F', 0, 0, 0, 0, 'W', 'A', 'V', 'E',
                                       'f', 'm', 't', ' ', 0, 0, 0, 0xf0};
      lying.insert(lying.end(), 6000, 0);
      Fixture file("zcore-wav-fmt-lying.wav", lying);
      check(!singz::readWavInfo(file.path).ok, "a fmt size past the end of the file is refused");
    }
  return failures ? 1 : 0;
}
