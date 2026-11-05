#include "alsa_pcm.h"

#include <zcore/device/audio_input_backend.h>

#include <algorithm>
#include <array>

namespace singz {
namespace {
class AlsaInput final : public AudioInputBackend {
 public:
  AlsaInput() = default;
#if defined(SINGZ_CORE_TESTS)
  AlsaInput(std::string pcm, uint32_t channels)
      : testPcm_(std::move(pcm)), testChannels_(channels) {}
#endif
  ~AlsaInput() override { stop(); }
  AudioInputResult open(const AudioInputConfig& config, AudioInputPush push,
                        void* context) override {
    stop();
    if (!push) return fail("Capture callback is missing", config.channel);
#if defined(SINGZ_CORE_TESTS)
    if (!testPcm_.empty()) return openMono(config, push, context, testPcm_, testChannels_);
#endif
    const auto endpoints = detail::alsaEndpoints();
    for (const auto& endpoint : endpoints) {
      if (endpoint.info.uid == config.deviceUid)
        return openMono(config, push, context, endpoint.pcm, endpoint.info.inputChannels);
    }
    return fail("ALSA capture endpoint was not found", config.channel);
  }
  AudioInputBusResult openBus(const AudioInputBusConfig& config, AudioInputBusPush push,
                              void* context) override {
    stop();
    if (!push || config.channels.empty() || config.channels.size() > 8)
      return busFail("Capture bus requires 1-8 selected channels and a callback",
                     config.channels.empty() ? 0 : config.channels.front());
    for (size_t index = 0; index < config.channels.size(); ++index)
      for (size_t prior = 0; prior < index; ++prior)
        if (config.channels[index] == config.channels[prior])
          return busFail("Capture bus selected a duplicate channel", config.channels[index]);
#if defined(SINGZ_CORE_TESTS)
    if (!testPcm_.empty()) return openBusEndpoint(config, push, context, testPcm_, testChannels_);
#endif
    const auto endpoints = detail::alsaEndpoints();
    for (const auto& endpoint : endpoints) {
      if (endpoint.info.uid == config.deviceUid)
        return openBusEndpoint(config, push, context, endpoint.pcm, endpoint.info.inputChannels);
    }
    return busFail("ALSA capture endpoint was not found", config.channels.front());
  }
  AudioInputResult start() override {
    std::string error;
    if (!lease_ || !lease_->start(error))
      return fail(error.empty() ? "Capture endpoint is not prepared" : error, channel_);
    auto result = prepared_; result.state = AudioInputState::Running; return result;
  }
  void stop() override {
    lease_.reset();
    push_ = nullptr; busPush_ = nullptr; context_ = nullptr;
    busChannelCount_ = busCapacityFrames_ = 0;
    busChannels_.fill(0); busPlanes_.fill(nullptr);
  }
  bool suspendPrepared() override {
    if (!lease_ || lease_->failure()) return false;
    lease_->stop(); return true;
  }
  bool takeFailure(std::string& error) override {
    const int code = lease_ ? lease_->failure() : 0;
    if (!code) return false;
    error = std::string("ALSA capture stopped: ") + snd_strerror(code);
    return true;
  }
 private:
  static AudioInputResult fail(std::string error, uint32_t channel) {
    return AudioInputResult::failure(AudioInputState::Error, std::move(error), channel);
  }
  static AudioInputBusResult busFail(std::string error, uint32_t channel) {
    return {AudioInputBusOpenStatus::Error,
            AudioInputResult::failure(AudioInputState::Error, std::move(error), channel)};
  }
  AudioInputResult openMono(const AudioInputConfig& config, AudioInputPush push, void* context,
                            const std::string& pcm, uint32_t deviceChannels) {
    if (config.channel >= deviceChannels)
      return fail("Capture channel is outside the device inventory", config.channel);
    channel_ = config.channel; push_ = push; context_ = context;
    std::string error;
    lease_ = detail::acquireAlsaCapture(config.deviceUid, pcm,
        config.channel + 1, 48000, 256, deliver, this, error);
    if (!lease_) return fail(error, config.channel);
    mono_.resize(lease_->period());
    prepare(config.deviceUid);
    return prepared_;
  }
  AudioInputBusResult openBusEndpoint(const AudioInputBusConfig& config, AudioInputBusPush push,
                                      void* context, const std::string& pcm,
                                      uint32_t deviceChannels) {
    const auto maximum = *std::max_element(config.channels.begin(), config.channels.end());
    if (maximum >= deviceChannels)
      return busFail("Capture bus channel is outside the device inventory", maximum);
    const auto needed = maximum + 1;
    busChannelCount_ = static_cast<uint32_t>(config.channels.size());
    for (uint32_t index = 0; index < busChannelCount_; ++index)
      busChannels_[index] = config.channels[index];
    channel_ = config.channels.front(); busPush_ = push; context_ = context;
    std::string error;
    lease_ = detail::acquireAlsaCapture(config.deviceUid, pcm, needed, 48000, 256, deliver,
                                        this, error);
    if (!lease_) return busFail(error, channel_);
    busCapacityFrames_ = lease_->period();
    for (uint32_t index = 0; index < busChannelCount_; ++index) {
      busSamples_[index].resize(busCapacityFrames_);
      busPlanes_[index] = busSamples_[index].data();
    }
    prepare(config.deviceUid);
    return {AudioInputBusOpenStatus::Ready, prepared_};
  }
  void prepare(const std::string& deviceUid) {
    prepared_ = AudioInputResult::success(AudioInputState::Starting, 48000, channel_);
    prepared_.deviceUid = deviceUid;
    prepared_.deviceChannels = lease_->channels();
    prepared_.sampleFormat = "converted planar float32";
    prepared_.sharingMode = "exclusive ALSA; shared capture lease";
    prepared_.timestampSource = "ALSA monotonic htimestamp availability (estimated)";
  }
  static bool deliver(void* context, const float* input, uint32_t channels,
                      uint32_t frames, uint64_t, uint64_t firstSampleNs) noexcept {
    auto& self = *static_cast<AlsaInput*>(context);
    if (self.busPush_) {
      for (uint32_t selected = 0; selected < self.busChannelCount_; ++selected)
        for (uint32_t frame = 0; frame < frames; ++frame)
          self.busSamples_[selected][frame] =
              input[size_t(frame) * channels + self.busChannels_[selected]];
      AudioInputBusBlock block;
      block.planes = self.busPlanes_.data();
      block.channelCount = self.busChannelCount_;
      block.frames = frames;
      block.capacityFrames = self.busCapacityFrames_;
      block.firstNs = firstSampleNs;
      block.deliveryNs = detail::alsaMonotonicNs();
      block.quality = firstSampleNs ? AudioInputTimestampQuality::CallbackEstimate :
          AudioInputTimestampQuality::Unknown;
      return self.busPush_(self.context_, block);
    }
    for (uint32_t frame = 0; frame < frames; ++frame)
      self.mono_[frame] = input[size_t(frame) * channels + self.channel_];
    return self.push_(self.context_, self.mono_.data(), frames,
        firstSampleNs, detail::alsaMonotonicNs(),
        firstSampleNs ? AudioInputTimestampQuality::CallbackEstimate :
            AudioInputTimestampQuality::Unknown);
  }
  std::unique_ptr<detail::AlsaCaptureLease> lease_;
  std::vector<float> mono_;
  uint32_t channel_{0};
  AudioInputPush push_{nullptr};
  AudioInputBusPush busPush_{nullptr};
  void* context_{nullptr};
  AudioInputResult prepared_;
  std::array<uint32_t, 8> busChannels_{};
  std::array<std::vector<float>, 8> busSamples_{};
  std::array<float*, 8> busPlanes_{};
  uint32_t busChannelCount_{0}, busCapacityFrames_{0};
#if defined(SINGZ_CORE_TESTS)
  std::string testPcm_;
  uint32_t testChannels_{0};
#endif
};
}
std::unique_ptr<AudioInputBackend> createPlatformAudioInputBackend() {
  return std::make_unique<AlsaInput>();
}
#if defined(SINGZ_CORE_TESTS)
std::unique_ptr<AudioInputBackend> makeAlsaAudioInputBackendForTests(
    const std::string& pcm, uint32_t channels) {
  return std::make_unique<AlsaInput>(pcm, channels);
}
#endif
std::vector<AudioInputDevice> enumeratePlatformAudioInputDevices(std::string* error) {
  if (error) error->clear();
  std::vector<AudioInputDevice> devices;
  for (const auto& endpoint : detail::alsaEndpoints()) {
    const auto& row = endpoint.info;
    if (row.inputChannels) devices.push_back({row.uid, row.label, row.defaultInput,
        row.nominalSampleRate, row.inputChannels, row.inputChannelLabels});
  }
  return devices;
}
}  // namespace singz
