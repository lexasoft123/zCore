#include <zcore/device/audio_host.h>

#include <algorithm>

namespace singz {

std::optional<uint32_t> chooseMonitorBufferFrames(
    const AudioHostBufferRange& inRange, const AudioHostBufferRange& outRange,
    uint32_t target, uint32_t maximum) noexcept {
  if (inRange.minimumFrames == 0 || outRange.minimumFrames == 0 ||
      inRange.maximumFrames < inRange.minimumFrames ||
      outRange.maximumFrames < outRange.minimumFrames) return std::nullopt;
  const auto lower = std::max(inRange.minimumFrames, outRange.minimumFrames);
  const auto upper = std::min({inRange.maximumFrames, outRange.maximumFrames, maximum});
  if (lower > upper) return std::nullopt;
  return std::clamp(target, lower, upper);
}

AudioHost::AudioHost() : backend_(createPlatformAudioHostBackend()) {}
AudioHost::AudioHost(std::unique_ptr<AudioHostBackend> backend)
    : backend_(std::move(backend)) {}
AudioHost::~AudioHost() { stop(); }
AudioHost::AudioHost(AudioHost&&) noexcept = default;
AudioHost& AudioHost::operator=(AudioHost&& other) noexcept {
  if (this == &other) return *this;
  stop();
  backend_ = std::move(other.backend_);
  return *this;
}

AudioHostInventory AudioHost::enumerate() const {
  return backend_ != nullptr ? backend_->enumerate() : AudioHostInventory{};
}

std::optional<AudioHostDeviceInfo> AudioHost::describeOutputDevice(
    const std::string& uid) const {
  if (backend_ == nullptr) return std::nullopt;
  return backend_->describeOutputDevice(uid);
}

AudioHostResult AudioHost::open(const AudioHostConfig& config,
                                AudioHostRender render, void* renderContext) {
  if (backend_ == nullptr) {
    return {false, AudioHostError::Unsupported, AudioHostState::Unsupported,
            {}, {}, "No audio host provider is available"};
  }
  return backend_->open(config, render, renderContext);
}

AudioHostResult AudioHost::start() {
  if (backend_ == nullptr) {
    return {false, AudioHostError::Unsupported, AudioHostState::Unsupported,
            {}, {}, "No audio host provider is available"};
  }
  return backend_->start();
}

AudioHostResult AudioHost::suspend() {
  if (backend_ == nullptr) {
    return {false, AudioHostError::Unsupported, AudioHostState::Unsupported,
            {}, {}, "No audio host provider is available"};
  }
  return backend_->suspend();
}

AudioHostResult AudioHost::setBufferFrames(std::uint32_t frames,
                                          std::uint32_t maximumAcceptableBufferFrames) {
  if (backend_ == nullptr) {
    return {false, AudioHostError::Unsupported, AudioHostState::Unsupported,
            {}, {}, "No audio host provider is available"};
  }
  return backend_->setBufferFrames(frames, maximumAcceptableBufferFrames);
}

AudioHostResult AudioHost::resume() {
  if (backend_ == nullptr) {
    return {false, AudioHostError::Unsupported, AudioHostState::Unsupported,
            {}, {}, "No audio host provider is available"};
  }
  return backend_->resume();
}

void AudioHost::stop() noexcept {
  if (backend_ != nullptr) backend_->stop();
}

AudioHostStatus AudioHost::status() const noexcept {
  return backend_ != nullptr ? backend_->status()
                             : AudioHostStatus{AudioHostState::Unsupported};
}

}  // namespace singz
