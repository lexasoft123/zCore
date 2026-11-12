#pragma once

#include <CoreAudio/CoreAudio.h>

#include "audio_host_macos_helpers.h"

namespace singz::detail {

struct MacAudioBufferProperties {
  AudioDeviceID device;
  const std::string& uid;

  const std::string& deviceUid() const { return uid; }
  uint64_t deviceIdentity() const { return device; }
  bool identityMatches() const {
    CFStringRef value = CFStringCreateWithCString(nullptr, uid.c_str(), kCFStringEncodingUTF8);
    if (!value) return false;
    AudioDeviceID resolved = kAudioObjectUnknown;
    UInt32 size = sizeof(resolved);
    const AudioObjectPropertyAddress address{kAudioHardwarePropertyTranslateUIDToDevice,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    const auto status = AudioObjectGetPropertyData(kAudioObjectSystemObject, &address,
        sizeof(value), &value, &size, &resolved);
    CFRelease(value);
    return status == noErr && size == sizeof(resolved) && resolved == device;
  }
  bool isSettable() const {
    Boolean value = false;
    const AudioObjectPropertyAddress address{kAudioDevicePropertyBufferFrameSize,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    return AudioObjectIsPropertySettable(device, &address, &value) == noErr && value;
  }

  template <class T>
  bool read(AudioObjectPropertySelector selector, T& value) const {
    const AudioObjectPropertyAddress address{
        selector, kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    UInt32 size = sizeof(value);
    return AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, &value) == noErr &&
           size == sizeof(value);
  }
  bool readFrames(uint32_t& frames) const {
    UInt32 value = 0;
    if (!read(kAudioDevicePropertyBufferFrameSize, value)) return false;
    frames = value;
    return true;
  }
  bool readRange(uint32_t& minimum, uint32_t& maximum) const {
    AudioValueRange range{};
    return read(kAudioDevicePropertyBufferFrameSizeRange, range) &&
           checkedAudioHostBufferRange(range.mMinimum, range.mMaximum, &minimum, &maximum) &&
           minimum != 0;
  }
  bool writeFrames(uint32_t frames) const {
    const UInt32 value = frames;
    const AudioObjectPropertyAddress address{kAudioDevicePropertyBufferFrameSize,
        kAudioObjectPropertyScopeGlobal, kAudioObjectPropertyElementMain};
    return AudioObjectSetPropertyData(device, &address, 0, nullptr, sizeof(value), &value) == noErr;
  }
};

}  // namespace singz::detail
