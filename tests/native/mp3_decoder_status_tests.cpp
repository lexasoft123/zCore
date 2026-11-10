// Status regressions ported from SingZ 8bf8a86c native decoder tests.
#include <zcore/media/decoded_audio.h>
#include <zcore/media/streaming_audio_source.h>
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
  return failures ? 1 : 0;
}
