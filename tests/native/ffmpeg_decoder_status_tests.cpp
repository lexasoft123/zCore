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
void rejects(const std::vector<unsigned char>& bytes, const std::string& label) {
  Fixture file("zcore-ffmpeg-status.aiff", bytes);
  singz::DecodedAudioPrepareOptions options;
  options.sourceFormat = singz::DecodedAudioSourceFormat::Aiff;
  auto result = singz::prepareDecodedAudio(singz::OwnedFileDescriptor(openRead(file.path)), options);
  check(result.status == singz::DecodedAudioStatus::MalformedData && !result.audio, label);
}
} // namespace
int main() {
  // A tiny mono 44.1 kHz PCM AIFF, built in memory so no encoder is required.
  const std::vector<unsigned char> aiff{
    'F','O','R','M',0,0,0,48,'A','I','F','F',
    'C','O','M','M',0,0,0,18,0,1,0,0,0,1,0,16,
    0x40,0x0e,0xac,0x44,0,0,0,0,0,0,
    'S','S','N','D',0,0,0,10,0,0,0,0,0,0,0,0,0,0};
  {
    Fixture file("zcore-ffmpeg-valid.aiff", aiff);
    singz::DecodedAudioPrepareOptions options;
    options.sourceFormat = singz::DecodedAudioSourceFormat::Aiff;
    auto good = singz::prepareDecodedAudio(singz::OwnedFileDescriptor(openRead(file.path)), options);
    check(good.ok() && good.audio->channelCount() == 1 && good.audio->sampleRate() == 44100,
          "AIFF positive control decodes");
#if defined(_WIN32)
    const int fd = _open(file.path.c_str(), _O_WRONLY | _O_BINARY);
#else
    const int fd = ::open(file.path.c_str(), O_WRONLY);
#endif
    auto unreadable = singz::prepareDecodedAudio(singz::OwnedFileDescriptor(fd), options);
    check(unreadable.status == singz::DecodedAudioStatus::IoError && !unreadable.audio,
          "real descriptor read error remains IoError");
  }
  // codec_provisioning_tests retains the signature then truncates or corrupts
  // every structural field: demuxer EIO alone is not descriptor I/O evidence.
  rejects(std::vector<unsigned char>(aiff.begin(), aiff.begin()+12), "truncated AIFF is malformed");
  auto corrupt = std::vector<unsigned char>(aiff.begin(), aiff.begin()+12);
  corrupt.resize(48, 0xa5);
  for (size_t i=12; i<corrupt.size(); ++i) corrupt[i] = static_cast<unsigned char>(0xa5 ^ i);
  rejects(corrupt, "corrupt AIFF is malformed");
  auto channels = aiff;
  channels[21] = 0;
  rejects(channels, "missing channel count is malformed");
  auto rate = aiff;
  for (size_t i=28; i<38; ++i) rate[i] = 0;
  rejects(rate, "missing sample rate is malformed");
  return failures ? 1 : 0;
}
