#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <zcore/device/audio_input.h>

namespace singz {

using AudioInputPush = bool (*)(void*, const float*, uint32_t, uint64_t,
                                uint64_t, AudioInputTimestampQuality) noexcept;

struct AudioInputBusConfig {
  std::string deviceUid;
  std::vector<uint32_t> channels;
};
struct AudioInputBusBlock {
  // Planes follow the selected-channel order and remain valid only for this
  // callback. Each non-null plane contains at least frames contiguous floats;
  // frames never exceeds capacityFrames and all planes share these timestamps.
  const float* const* planes = nullptr;
  uint32_t channelCount = 0, frames = 0, capacityFrames = 0;
  uint64_t firstNs = 0, deliveryNs = 0;
  AudioInputTimestampQuality quality = AudioInputTimestampQuality::Unknown;
};
using AudioInputBusPush = bool (*)(void*, const AudioInputBusBlock&) noexcept;
enum class AudioInputBusOpenStatus { Unsupported, Ready, Error };
struct AudioInputBusResult {
  AudioInputBusOpenStatus status = AudioInputBusOpenStatus::Unsupported;
  AudioInputResult result;
};

class AudioInputBackend {
 public:
  virtual ~AudioInputBackend() = default;
  // open() resolves/configures hardware but MUST NOT start callbacks. Its
  // returned rate is published into the shared delivery state before start().
  virtual AudioInputResult open(const AudioInputConfig& config, AudioInputPush push,
                                void* context) = 0;
  // Selected-channel buses are optional. Unsupported has no side effects and
  // permits mono fallback; Error reports a real open failure and forbids it.
  virtual AudioInputBusResult openBus(const AudioInputBusConfig&, AudioInputBusPush, void*) {
    return {};
  }
  virtual AudioInputResult start() = 0;
  virtual void stop() = 0;
  // Stops hardware callbacks while retaining prepared device resources. A
  // false result requests ordinary teardown; other platforms keep that default.
  virtual bool suspendPrepared() { return false; }
  // Lock-free backend flags are rendered into a message here, off RT.
  virtual bool takeFailure(std::string& error) = 0;
};

std::unique_ptr<AudioInputBackend> createPlatformAudioInputBackend();
std::vector<AudioInputDevice> enumeratePlatformAudioInputDevices(std::string* error);

#if defined(SINGZ_CORE_TESTS)
// Host-test injection: compiled out of every production core.
using AudioInputBackendFactoryForTests = std::unique_ptr<AudioInputBackend> (*)();
void setAudioInputBackendForTests(AudioInputBackendFactoryForTests factory,
                                  std::vector<AudioInputDevice> devices);
#endif

}  // namespace singz
