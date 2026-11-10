// Media files under non-ASCII paths: write, resume, read, compact to FLAC and
// read back, all through UTF-8 std::string paths. On Windows the CRT's narrow
// fopen/remove/rename read those bytes in the ANSI code page, so a stem under
// a Cyrillic, CJK or emoji folder failed to open there; POSIX passes the bytes
// through and must keep doing so. Windows CI is the leg that has teeth.
#include <zcore/base/utf8_file.h>
#include <zcore/media/flac_io.h>
#include <zcore/media/wav.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

int failures = 0;

void check(bool ok, const char* what) {
  if (!ok) {
    std::fprintf(stderr, "FAIL: %s\n", what);
    failures++;
  }
}

std::string utf8(const fs::path& p) {
  const std::u8string u = p.u8string();
  return std::string(u.begin(), u.end());
}

std::vector<float> tone(int frames, int rate) {
  std::vector<float> out(static_cast<size_t>(frames));
  for (int i = 0; i < frames; i++) {
    out[static_cast<size_t>(i)] = 0.5f * std::sin(2.0 * 3.14159265358979 * 440.0 * i / rate);
  }
  return out;
}

bool samePcm16(const std::vector<float>& a, const std::vector<float>& b) {
  if (a.size() != b.size()) return false;
  for (size_t i = 0; i < a.size(); i++) {
    if (std::fabs(a[i] - b[i]) > 1.5f / 32768.0f) return false;
  }
  return true;
}

void runIn(const fs::path& dir) {
  const int rate = 48000;
  const int half = 4800;
  const std::vector<float> pcm = tone(2 * half, rate);
  const std::string wavPath = utf8(dir / u8"голос-歌-🎤.wav");
  const std::string flacPath = utf8(dir / u8"голос-歌-🎤.flac");

  {
    singz::WavWriter w;
    check(w.open(wavPath, rate, 1), "open a new wav");
    check(w.append(pcm.data(), half), "append the first half");
    check(w.finalize(), "finalize the first half");
  }
  {
    singz::WavWriter w;  // resume path: "r+b" on the same file
    check(w.open(wavPath, rate, 1, half), "reopen the wav to resume");
    check(w.append(pcm.data() + half, half), "append the second half");
    check(w.finalize(), "finalize the resumed wav");
  }
  check(singz::statUtf8(wavPath), "statUtf8 sees the wav");

  const singz::WavInfo info = singz::readWavInfo(wavPath);
  check(info.ok && info.frames == 2 * half && info.sampleRate == rate, "readWavInfo");
  const singz::MonoWav wav = singz::readWavMono(wavPath);
  check(wav.ok && samePcm16(wav.samples, pcm), "readWavMono returns the written samples");

  // A failed output open must return an error without asserting or deleting
  // the source stem. A nonexistent parent works without permission assumptions.
  const std::string blocked = utf8(dir / u8"missing-目录" / u8"stem.flac");
  check(!singz::compactStem(wavPath, blocked).ok, "compactStem reports an output open failure");
  check(singz::statUtf8(wavPath), "failed compactStem preserves the wav");

  const singz::CompactResult c = singz::compactStem(wavPath, flacPath);
  check(c.ok && !c.skipped && c.bytes > 0, "compactStem encodes the flac");
  check(!singz::statUtf8(wavPath), "compactStem removes the wav");
  int64_t size = 0;
  check(singz::statUtf8(flacPath, &size) && size == c.bytes, "the flac is on disk at its size");

  const singz::WavInfo finfo = singz::readWavInfo(flacPath);  // sniffs FLAC
  check(finfo.ok && finfo.frames == 2 * half && finfo.sampleRate == rate, "readWavInfo on the flac");
  const singz::MonoWav flac = singz::readFlacMono(flacPath);
  check(flac.ok && samePcm16(flac.samples, pcm), "readFlacMono returns the same samples");

  const std::string missing = utf8(dir / u8"нет-无.flac");
  check(!singz::readFlacMono(missing).ok, "readFlacMono reports a missing file");
  check(!singz::readFlacInfo(missing).ok, "readFlacInfo reports a missing file");
  check(!singz::readWavMono(missing).ok, "readWavMono reports a missing file");

  const std::string renamed = utf8(dir / u8"переименован-改名.flac");
  check(singz::renameUtf8(flacPath, renamed) == 0, "renameUtf8");
  check(singz::removeUtf8(renamed) == 0 && !singz::statUtf8(renamed), "removeUtf8");

#if defined(_WIN32)
  check(singz::openUtf8(std::string("bad-\xff\xfe.wav"), "rb") == nullptr && errno == EINVAL,
        "invalid UTF-8 is refused, not guessed at");
#endif
}

}  // namespace

int main() {
  const fs::path dir = fs::temp_directory_path() / fs::path(u8"zcore-utf8-Андрей-歌手-🎤");
  std::error_code ec;
  fs::remove_all(dir, ec);
  fs::create_directories(dir, ec);
  if (ec) {
    std::fprintf(stderr, "FAIL: cannot create the test folder: %s\n", ec.message().c_str());
    return 1;
  }
  runIn(dir);
  fs::remove_all(dir, ec);
  if (failures != 0) return 1;
  std::printf("utf8_path_tests: ok\n");
  return 0;
}
