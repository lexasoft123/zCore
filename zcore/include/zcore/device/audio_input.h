#pragma once

#include <cstdint>
#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <zcore/audio/audio_input_transport.h>

namespace singz {

struct AudioInputDevice {
  std::string uid;
  std::string label;
  bool isDefault = false;
  double sampleRate = 0;
  uint32_t channels = 0;
  std::vector<std::string> channelLabels;
};

struct AudioInputConfig {
  std::string deviceUid;
  uint32_t channel = 0;
  uint32_t ringBlocks = 32;
};

enum class AudioInputState {
  Idle,
  Starting,
  Running,
  Stopping,
  Stopped,
  Unsupported,
  Error,
};

struct AudioInputResult {
  AudioInputResult() = default;

  static AudioInputResult success(AudioInputState resultState,
                                  double negotiatedSampleRate,
                                  uint32_t selectedChannel) {
    AudioInputResult result;
    result.ok = true;
    result.state = resultState;
    result.sampleRate = negotiatedSampleRate;
    result.channel = selectedChannel;
    return result;
  }

  static AudioInputResult failure(AudioInputState resultState,
                                  std::string message,
                                  uint32_t selectedChannel) {
    AudioInputResult result;
    result.ok = false;
    result.state = resultState;
    result.error = std::move(message);
    result.channel = selectedChannel;
    return result;
  }

  bool ok = false;
  AudioInputState state = AudioInputState::Error;
  std::string error;
  double sampleRate = 0;
  uint32_t channel = 0;
  std::string deviceUid;
  uint32_t deviceChannels = 0;
  std::string sampleFormat;
  std::string sharingMode;
  std::string performanceMode;
  std::string inputPreset;
  std::string timestampSource;
  // Input hardware/safety/stream delay and separate negotiated I/O buffer,
  // both in capture-rate frames. Delay assumes a FIRST-SAMPLE timestamp (HAL
  // mHostTime or the already buffer-adjusted CallbackEstimate). Never subtract
  // the buffer again from those timestamps. A raw callback-entry timestamp
  // needs its buffer duration removed first, before applying this delay.
  // Unknown means at least one required property was unavailable; zero alone
  // does not indicate whether the driver reported a delay. BoxInputCapture /
  // box_routing_owner and non-macOS backends leave these fields unknown/zero.
  uint32_t inputLatencyFrames = 0;
  uint32_t inputBufferFrames = 0;
  std::string inputLatencySource = "unknown";
  // Optional live validity of the opening snapshot. Property notifications
  // invalidate it without stopping capture; null leaves source authoritative.
  std::shared_ptr<const std::atomic<bool>> inputLatencyStable;
};

struct AudioInputStats {
  uint64_t deliveredBlocks = 0;
  uint64_t deliveredFrames = 0;
  uint64_t overruns = 0;
  uint64_t deliveryWakeups = 0;
};

using AudioInputSink = std::function<void(const AudioInputBlockView&)>;

std::vector<AudioInputDevice> enumerateAudioInputDevices(
    std::string* error = nullptr);
bool audioInputBackendSupported();
bool validateAudioInputConfig(const AudioInputConfig& config,
                              const std::vector<AudioInputDevice>& devices,
                              std::string& error);
bool makeAudioInputChannelMap(uint32_t selectedChannel,
                              uint32_t deviceChannels,
                              int32_t& sourceChannel, std::string& error);
const char* audioInputStateName(AudioInputState state);

// Compatibility lifecycle owner. This target mixes control/delivery threads
// with provider integration; only the backend callback-to-ring path carries
// the no-allocation/no-blocking callback contract.
class AudioInput {
 public:
  AudioInput();
  ~AudioInput();
  AudioInput(const AudioInput&) = delete;
  AudioInput& operator=(const AudioInput&) = delete;

  // A process-lifetime device service may supply its current inventory. The
  // provider still validates the exact UID/channel at open; no second full
  // inventory is read during activation in that case.
  AudioInputResult start(const AudioInputConfig& config, AudioInputSink sink,
                        const std::vector<AudioInputDevice>* inventory = nullptr);
  // Retention is opt-in and never leaves hardware capture running. Reuse is
  // permitted only for an unchanged device/channel/configuration.
  void stop(bool retainPrepared = false);
  AudioInputState state() const;
  AudioInputStats stats() const;
  std::string lastError() const;

 private:
  struct Impl;
  std::shared_ptr<Impl> impl_;
};

}  // namespace singz
