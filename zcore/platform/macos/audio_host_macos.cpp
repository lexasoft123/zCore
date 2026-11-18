#include <zcore/device/audio_host.h>
#include <zcore/device/audio_input_callback_gate.h>

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>
#include <Block.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstring>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <zcore/device/audio_host_callback.h>
#include <zcore/audio/audio_input_timestamp.h>

#include "audio_host_macos_helpers.h"
#include "audio_device_macos_buffer.h"
#include "audio_device_macos_buffer_properties.h"

namespace singz {
namespace {

constexpr uint32_t kAliveChanged = detail::kMacAudioHostAliveChanged;
constexpr uint32_t kFormatChanged = detail::kMacAudioHostFormatChanged;

struct HostDeviceListenerContext {
  std::atomic<uint32_t> pendingChanges{0};
  std::atomic<uint64_t> routeGeneration{1};
  std::atomic<uint32_t> routeChanged{0};
  AudioDeviceID device{kAudioObjectUnknown};
  std::string uid;
  bool hasInput = false;
  detail::MacAudioHostDeviceFacts opened;
  std::atomic<uint32_t> bufferFrames{0};
  std::atomic<uint32_t> bufferWarnings{AudioBufferWarningNone};
  std::atomic<uint32_t> deviceLost{0};
  // Independent token snapshot: only the serial listener queue mutates this
  // copy; the backend releases its own copy under the control mutex.
  detail::MacAudioBufferOwnership bufferOwnership;
};

// Admission gate for renderCallback: entry is counted and admitted in one
// atomic word, so a callback that races the close in stop() either enters
// before the drain observes zero or is refused without touching any member.
// The gate is heap-allocated and registered with the HAL in place of `this`
// so an UNCLEAN teardown can quarantine it: the leaked context keeps the
// gate readable, so a late callback there is refused instead of touching
// freed memory. A fully clean teardown frees it and accepts the platform's
// Dispose contract as the boundary beyond this defense (the iOS host's
// poison-lease idiom). The scope covers the whole callback —
// preamble, endpoint invocation, and postamble — including the parts
// endpoint_.inFlight never counted.

// The copied block retains its context. Exceptional failed removal keeps the
// already-allocated block/queue alive, avoiding stop-time quarantine allocation.
AudioObjectPropertyAddress property(AudioObjectPropertySelector selector,
                                    AudioObjectPropertyScope scope,
                                    AudioObjectPropertyElement element =
                                        kAudioObjectPropertyElementMain) {
  return {selector, scope, element};
}

template <typename T>
bool readProperty(AudioObjectID object, AudioObjectPropertyAddress address,
                  T* value) {
  UInt32 size = sizeof(T);
  return AudioObjectGetPropertyData(object, &address, 0, nullptr, &size, value) == noErr &&
         size == sizeof(T);
}

std::string readString(AudioObjectID object, AudioObjectPropertyAddress address) {
  CFStringRef value = nullptr;
  UInt32 size = sizeof(value);
  if (AudioObjectGetPropertyData(object, &address, 0, nullptr, &size, &value) != noErr ||
      value == nullptr) {
    return {};
  }
  const CFIndex characters = CFStringGetLength(value);
  const CFIndex capacity =
      CFStringGetMaximumSizeForEncoding(characters, kCFStringEncodingUTF8) + 1;
  std::string result;
  if (capacity > 1 && capacity <= 65536) {
    std::vector<char> bytes(static_cast<size_t>(capacity));
    if (CFStringGetCString(value, bytes.data(), capacity, kCFStringEncodingUTF8)) {
      result = bytes.data();
    }
  }
  CFRelease(value);
  return result;
}

uint32_t channelCount(AudioDeviceID device, AudioObjectPropertyScope scope) {
  auto address = property(kAudioDevicePropertyStreamConfiguration, scope);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(device, &address, 0, nullptr, &size) != noErr ||
      size < offsetof(AudioBufferList, mBuffers) + sizeof(AudioBuffer) || size > 1024 * 1024) {
    return 0;
  }
  std::vector<uint8_t> storage(size);
  auto* list = reinterpret_cast<AudioBufferList*>(storage.data());
  if (AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, list) != noErr) return 0;
  uint64_t channels = 0;
  for (UInt32 index = 0; index < list->mNumberBuffers; ++index) {
    channels += list->mBuffers[index].mNumberChannels;
  }
  return channels <= kAudioHostMaxChannels ? static_cast<uint32_t>(channels) : 0;
}

std::vector<std::string> channelLabels(AudioDeviceID device,
                                       AudioObjectPropertyScope scope,
                                       uint32_t count,
                                       const char* fallbackPrefix) {
  std::vector<std::string> result;
  result.reserve(count);
  for (uint32_t channel = 0; channel < count; ++channel) {
    const auto element = static_cast<AudioObjectPropertyElement>(channel + 1);
    std::string label = readString(
        device, property(kAudioObjectPropertyElementName, scope, element));
    if (label.empty()) {
      const std::string category = readString(
          device, property(kAudioObjectPropertyElementCategoryName, scope, element));
      const std::string number = readString(
          device, property(kAudioObjectPropertyElementNumberName, scope, element));
      if (!category.empty())
        label = category + (number.empty() ? " " + std::to_string(channel + 1)
                                           : " " + number);
      else
        label = number;
    }
    if (label.empty()) label = std::string(fallbackPrefix) + " " +
                               std::to_string(channel + 1);
    result.push_back(std::move(label));
  }
  return result;
}

std::vector<AudioDeviceID> devices() {
  auto address = property(kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &address, 0, nullptr,
                                     &size) != noErr ||
      size % sizeof(AudioDeviceID) != 0 || size > 4096 * sizeof(AudioDeviceID)) {
    return {};
  }
  std::vector<AudioDeviceID> result(size / sizeof(AudioDeviceID));
  if (size != 0 && AudioObjectGetPropertyData(kAudioObjectSystemObject, &address, 0,
                                               nullptr, &size, result.data()) != noErr) {
    return {};
  }
  return result;
}

AudioDeviceID findDevice(const std::string& uid) {
  CFStringRef value = CFStringCreateWithCString(nullptr, uid.c_str(), kCFStringEncodingUTF8);
  if (!value) return kAudioObjectUnknown;
  AudioDeviceID device = kAudioObjectUnknown;
  UInt32 size = sizeof(device);
  const auto address = property(kAudioHardwarePropertyTranslateUIDToDevice,
                                kAudioObjectPropertyScopeGlobal);
  const auto result = AudioObjectGetPropertyData(kAudioObjectSystemObject,
      &address, sizeof(value), &value, &size, &device);
  CFRelease(value);
  return result == noErr && size == sizeof(device) ? device : kAudioObjectUnknown;
}

detail::MacAudioHostStreamFormat streamFormatFacts(const AudioStreamBasicDescription& f) {
  return {f.mSampleRate, f.mFormatID, f.mFormatFlags, f.mBytesPerPacket,
          f.mFramesPerPacket, f.mBytesPerFrame, f.mChannelsPerFrame, f.mBitsPerChannel};
}

bool readHostScopeFacts(AudioDeviceID device, AudioObjectPropertyScope scope,
                        std::vector<uint32_t>& channels,
                        std::vector<detail::MacAudioHostStreamFacts>& streams) {
  auto address = property(kAudioDevicePropertyStreamConfiguration, scope);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(device, &address, 0, nullptr, &size) != noErr ||
      size < offsetof(AudioBufferList, mBuffers) || size > 1024 * 1024) return false;
  std::vector<uint8_t> storage(size);
  if (AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, storage.data()) != noErr ||
      size < offsetof(AudioBufferList, mBuffers)) return false;
  const auto* list = reinterpret_cast<const AudioBufferList*>(storage.data());
  if (list->mNumberBuffers > (size - offsetof(AudioBufferList, mBuffers)) / sizeof(AudioBuffer)) return false;
  for (UInt32 i = 0; i < list->mNumberBuffers; ++i) channels.push_back(list->mBuffers[i].mNumberChannels);
  address = property(kAudioDevicePropertyStreams, scope);
  if (AudioObjectGetPropertyDataSize(device, &address, 0, nullptr, &size) != noErr ||
      size % sizeof(AudioStreamID) != 0 || size > 4096 * sizeof(AudioStreamID)) return false;
  std::vector<AudioStreamID> ids(size / sizeof(AudioStreamID));
  const UInt32 expectedSize = size;
  if (size && (AudioObjectGetPropertyData(device, &address, 0, nullptr, &size, ids.data()) != noErr ||
               size != expectedSize)) return false;
  for (const auto id : ids) {
    AudioStreamBasicDescription physical{}, virtualFormat{};
    if (!readProperty(id, property(kAudioStreamPropertyPhysicalFormat, kAudioObjectPropertyScopeGlobal), &physical) ||
        !readProperty(id, property(kAudioStreamPropertyVirtualFormat, kAudioObjectPropertyScopeGlobal), &virtualFormat)) return false;
    streams.push_back({id, streamFormatFacts(physical), streamFormatFacts(virtualFormat)});
  }
  return true;
}

bool readHostDeviceFacts(AudioDeviceID device, bool hasInput, detail::MacAudioHostDeviceFacts& facts) {
  facts = {};
  facts.identity = device;
  return readProperty(device, property(kAudioDevicePropertyNominalSampleRate,
                                      kAudioObjectPropertyScopeGlobal), &facts.rate) &&
      std::isfinite(facts.rate) && facts.rate > 0 &&
      (!detail::macAudioHostUsesScope(hasInput, true) ||
       readHostScopeFacts(device, kAudioDevicePropertyScopeInput, facts.inputChannels, facts.inputStreams)) &&
      readHostScopeFacts(device, kAudioDevicePropertyScopeOutput, facts.outputChannels, facts.outputStreams);
}

detail::MacAudioHostRouteObservation readHostRouteObservation(const HostDeviceListenerContext& listener) {
  using Identity = detail::MacAudioHostIdentity;
  detail::MacAudioHostRouteObservation observed;
  observed.facts.identity = listener.device;
  CFStringRef expected = CFStringCreateWithCString(nullptr, listener.uid.c_str(), kCFStringEncodingUTF8);
  AudioDeviceID resolved = kAudioObjectUnknown;
  UInt32 size = sizeof(resolved);
  const auto translate = property(kAudioHardwarePropertyTranslateUIDToDevice, kAudioObjectPropertyScopeGlobal);
  const bool translated = expected &&
      AudioObjectGetPropertyData(kAudioObjectSystemObject, &translate, sizeof(expected), &expected,
                                 &size, &resolved) == noErr && size == sizeof(resolved);
  if (translated && resolved != listener.device) {
    CFRelease(expected);
    observed.identity = Identity::Absent;
    return observed;
  }
  UInt32 alive = 0;
  size = sizeof(alive);
  const auto aliveAddress = property(kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal);
  const auto aliveStatus = AudioObjectGetPropertyData(listener.device, &aliveAddress, 0, nullptr, &size, &alive);
  const bool aliveKnown = aliveStatus == noErr && size == sizeof(alive);
  if (aliveStatus == kAudioHardwareBadObjectError || (aliveKnown && !alive)) {
    if (expected) CFRelease(expected);
    observed.identity = Identity::Absent;
    return observed;
  }
  bool identityKnown = translated;
  // A translation read failure is not absence. Verify the existing object
  // directly if possible; still retry once so transient lookup faults recover.
  if (!translated && expected) {
    CFStringRef actual = nullptr;
    size = sizeof(actual);
    const auto uidAddress = property(kAudioDevicePropertyDeviceUID, kAudioObjectPropertyScopeGlobal);
    const auto uidStatus = AudioObjectGetPropertyData(listener.device, &uidAddress, 0, nullptr, &size, &actual);
    if (uidStatus == kAudioHardwareBadObjectError ||
        (uidStatus == noErr && size == sizeof(actual) && actual && !CFEqual(expected, actual))) {
      if (actual) CFRelease(actual);
      CFRelease(expected);
      observed.identity = Identity::Absent;
      return observed;
    }
    identityKnown = uidStatus == noErr && size == sizeof(actual) && actual;
    if (actual) CFRelease(actual);
  }
  if (expected) CFRelease(expected);
  observed.identity = identityKnown && aliveKnown ? Identity::Present : Identity::Unknown;
  observed.retry = !translated || !aliveKnown;
  observed.readable = readHostDeviceFacts(listener.device, listener.hasInput, observed.facts);
  return observed;
}

detail::MacAudioHostRouteCheck hostDeviceChanges(const HostDeviceListenerContext& listener) {
  return detail::checkMacAudioHostRoute(listener.opened, listener.hasInput,
      [&] { return readHostRouteObservation(listener); });
}

bool validMap(const std::vector<uint32_t>& channels, uint32_t available) {
  if (channels.empty() || channels.size() > kAudioHostMaxChannels) return false;
  for (size_t index = 0; index < channels.size(); ++index) {
    if (channels[index] >= available) return false;
    for (size_t previous = 0; previous < index; ++previous) {
      if (channels[index] == channels[previous]) return false;
    }
  }
  return true;
}

bool outputOnly(const AudioHostConfig& config) noexcept {
  return config.inputDeviceUid.empty() && config.inputChannels.empty();
}

bool validInputSelection(const AudioHostConfig& config) noexcept {
  return config.inputDeviceUid.empty() == config.inputChannels.empty();
}

AudioHostResult failure(AudioHostError error, const std::string& message,
                        AudioHostState state = AudioHostState::Error) {
  return {false, error, state, {}, {}, message};
}

AudioStreamBasicDescription planarFormat(double rate, uint32_t channels) {
  AudioStreamBasicDescription format{};
  format.mSampleRate = rate;
  format.mFormatID = kAudioFormatLinearPCM;
  format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked |
                        kAudioFormatFlagIsNonInterleaved |
                        kAudioFormatFlagsNativeEndian;
  format.mBytesPerPacket = sizeof(float);
  format.mFramesPerPacket = 1;
  format.mBytesPerFrame = sizeof(float);
  format.mChannelsPerFrame = channels;
  format.mBitsPerChannel = 32;
  return format;
}

uint32_t latency(AudioDeviceID device, AudioObjectPropertyScope scope) {
  UInt32 latencyFrames = 0;
  UInt32 safetyFrames = 0;
  readProperty(device, property(kAudioDevicePropertyLatency, scope), &latencyFrames);
  readProperty(device, property(kAudioDevicePropertySafetyOffset, scope), &safetyFrames);
  return detail::saturatedAudioHostLatency(latencyFrames, safetyFrames);
}

class MacAudioHostBackend final : public AudioHostBackend {
 public:
  ~MacAudioHostBackend() override { stop(); }

  AudioHostInventory enumerate() const override {
    AudioDeviceID defaultInput = kAudioObjectUnknown;
    AudioDeviceID defaultOutput = kAudioObjectUnknown;
    readProperty(kAudioObjectSystemObject,
                 property(kAudioHardwarePropertyDefaultInputDevice,
                          kAudioObjectPropertyScopeGlobal),
                 &defaultInput);
    readProperty(kAudioObjectSystemObject,
                 property(kAudioHardwarePropertyDefaultOutputDevice,
                          kAudioObjectPropertyScopeGlobal),
                 &defaultOutput);
    AudioHostInventory inventory;
    for (AudioDeviceID device : devices()) {
      AudioHostDeviceInfo info;
      info.uid = readString(device, property(kAudioDevicePropertyDeviceUID,
                                             kAudioObjectPropertyScopeGlobal));
      info.label = readString(device, property(kAudioObjectPropertyName,
                                               kAudioObjectPropertyScopeGlobal));
      info.inputChannels = channelCount(device, kAudioDevicePropertyScopeInput);
      info.outputChannels = channelCount(device, kAudioDevicePropertyScopeOutput);
      if (info.uid.empty() || (info.inputChannels == 0 && info.outputChannels == 0)) continue;
      info.inputChannelLabels = channelLabels(
          device, kAudioDevicePropertyScopeInput, info.inputChannels, "Input");
      info.outputChannelLabels = channelLabels(
          device, kAudioDevicePropertyScopeOutput, info.outputChannels, "Output");
      info.defaultInput = device == defaultInput;
      info.defaultOutput = device == defaultOutput;
      UInt32 transport = kAudioDeviceTransportTypeUnknown;
      if (readProperty(device,
                       property(kAudioDevicePropertyTransportType,
                                kAudioObjectPropertyScopeGlobal),
                       &transport)) {
        const detail::MacAudioHostTransportCapability capability =
            detail::classifyMacAudioHostTransport(transport);
        info.transport = capability.transport;
        info.monitoringSuitability = capability.monitoringSuitability;
      }
      Float64 rate = 0.0;
      if (readProperty(device, property(kAudioDevicePropertyNominalSampleRate,
                                        kAudioObjectPropertyScopeGlobal),
                       &rate) && std::isfinite(rate) && rate > 0.0) {
        info.nominalSampleRate = rate;
      }
      auto rateAddress = property(kAudioDevicePropertyAvailableNominalSampleRates,
                                  kAudioObjectPropertyScopeGlobal);
      UInt32 rateBytes = 0;
      if (AudioObjectGetPropertyDataSize(device, &rateAddress, 0, nullptr,
                                         &rateBytes) == noErr &&
          rateBytes % sizeof(AudioValueRange) == 0 &&
          rateBytes <= sizeof(AudioValueRange) * 128) {
        std::vector<AudioValueRange> ranges(rateBytes / sizeof(AudioValueRange));
        if (rateBytes != 0 &&
            AudioObjectGetPropertyData(device, &rateAddress, 0, nullptr,
                                       &rateBytes, ranges.data()) == noErr) {
          for (const auto& available : ranges) {
            if (std::isfinite(available.mMinimum) &&
                std::isfinite(available.mMaximum) &&
                available.mMinimum > 0.0 &&
                available.mMaximum >= available.mMinimum) {
              info.sampleRateRanges.push_back(
                  {available.mMinimum, available.mMaximum});
            }
          }
        }
      }
      if (info.sampleRateRanges.empty() && info.nominalSampleRate > 0.0) {
        info.sampleRateRanges.push_back(
            {info.nominalSampleRate, info.nominalSampleRate});
      }
      UInt32 frames = 0;
      readProperty(device, property(kAudioDevicePropertyBufferFrameSize,
                                    kAudioObjectPropertyScopeGlobal),
                   &frames);
      AudioValueRange range{};
      readProperty(device, property(kAudioDevicePropertyBufferFrameSizeRange,
                                    kAudioObjectPropertyScopeGlobal),
                   &range);
      uint32_t minimumFrames = 0;
      uint32_t maximumFrames = 0;
      detail::checkedAudioHostBufferRange(range.mMinimum, range.mMaximum,
                                          &minimumFrames, &maximumFrames);
      info.bufferFrames = {minimumFrames, maximumFrames, frames, 0};
      info.direction = info.inputChannels != 0 && info.outputChannels != 0
                           ? AudioHostEndpointDirection::Duplex
                           : (info.inputChannels != 0
                                  ? AudioHostEndpointDirection::Input
                                  : AudioHostEndpointDirection::Output);
      if (info.direction != AudioHostEndpointDirection::Duplex)
        info.monitoringSuitability =
            AudioHostMonitoringSuitability::Unsupported;
      if (info.defaultInput) inventory.defaultInputUid = info.uid;
      if (info.defaultOutput) inventory.defaultOutputUid = info.uid;
      inventory.devices.push_back(std::move(info));
    }
    return inventory;
  }

  // The same answer enumerate() gives for this one device, without the rest
  // of the inventory. enumerate() reads every channel label of every device,
  // each a round trip to coreaudiod (11 devices and 115 channels, 36-48 ms,
  // on the Mac this was measured on). The membership and direction rules are
  // enumerate()'s: a device is listed only with a UID and at least one
  // channel, and it is an output endpoint only with output channels.
  std::optional<AudioHostDeviceInfo> describeOutputDevice(
      const std::string& uid) const override {
    if (uid.empty()) return std::nullopt;
    const AudioDeviceID device = findDevice(uid);
    if (device == kAudioObjectUnknown) return std::nullopt;
    AudioHostDeviceInfo info;
    info.uid = uid;
    info.inputChannels = channelCount(device, kAudioDevicePropertyScopeInput);
    info.outputChannels = channelCount(device, kAudioDevicePropertyScopeOutput);
    if (info.outputChannels == 0) return std::nullopt;
    info.direction = info.inputChannels != 0
                         ? AudioHostEndpointDirection::Duplex
                         : AudioHostEndpointDirection::Output;
    Float64 rate = 0.0;
    if (readProperty(device, property(kAudioDevicePropertyNominalSampleRate,
                                      kAudioObjectPropertyScopeGlobal),
                     &rate) && std::isfinite(rate) && rate > 0.0) {
      info.nominalSampleRate = rate;
    }
    return info;
  }

  AudioHostResult open(const AudioHostConfig& config, AudioHostRender render,
                       void* renderContext) override {
    std::lock_guard lock(controlMutex_);
    stop();
    if (config.requestedBufferFrames != 0 && !config.allowSharedDeviceBufferChange)
      return reject(AudioHostError::Unsupported,
          "Shared CoreAudio output buffer changes require allowSharedDeviceBufferChange",
          AudioHostState::Unsupported);
    allowSharedDeviceBufferChange_ = config.allowSharedDeviceBufferChange;
    if (config.exclusive) {
      return reject(AudioHostError::Unsupported,
                    "CoreAudio exclusive mode is not implemented by this AudioHost provider",
                    AudioHostState::Unsupported);
    }
    const bool hasInput = !outputOnly(config);
    if (config.outputDeviceUid.empty()) {
      return reject(AudioHostError::InvalidConfiguration,
                    "AudioHost output device UID must not be empty");
    }
    if (!validInputSelection(config)) {
      return reject(AudioHostError::InvalidConfiguration,
                    "AudioHost input device UID and channel map must both be empty or both be non-empty");
    }
    if (hasInput && config.inputDeviceUid != config.outputDeviceUid) {
      return reject(AudioHostError::DifferentDevicesUnsupported,
                    "AudioHost requires one physical CoreAudio device for input and output");
    }
    bufferWarnings_ = AudioBufferWarningNone;
    selectedDeviceUid_ = config.outputDeviceUid;
    acceptableBufferMaximum_ = config.maximumAcceptableBufferFrames;
    if (!detail::readMacAudioHostFactsWithRetry([&] {
          selectedDevice_ = findDevice(config.outputDeviceUid);
          return selectedDevice_ != kAudioObjectUnknown;
        })) {
      return reject(AudioHostError::DeviceNotFound,
                    "The selected CoreAudio device disappeared");
    }
    UInt32 alive = 0;
    Float64 rate = 0.0;
    if (!readProperty(selectedDevice_, property(kAudioDevicePropertyDeviceIsAlive,
                                                kAudioObjectPropertyScopeGlobal),
                      &alive) ||
        alive == 0 ||
        !readProperty(selectedDevice_, property(kAudioDevicePropertyNominalSampleRate,
                                                kAudioObjectPropertyScopeGlobal),
                      &rate) ||
        !std::isfinite(rate) || rate <= 0.0) {
      return fail(AudioHostError::ProviderFailure, "CoreAudio device format is unavailable");
    }
    if (config.requestedSampleRate != 0.0 && config.requestedSampleRate != rate) {
      return fail(AudioHostError::InvalidConfiguration,
                  "Changing a shared CoreAudio device sample rate is not supported in Phase 3A");
    }
    const uint32_t deviceInputs = hasInput ? channelCount(selectedDevice_, kAudioDevicePropertyScopeInput) : 0u;
    const uint32_t deviceOutputs = channelCount(selectedDevice_, kAudioDevicePropertyScopeOutput);
    std::vector<int32_t> outputMap;
    if ((hasInput && !validMap(config.inputChannels, deviceInputs)) ||
        !detail::buildMacAudioHostOutputChannelMap(
            config.outputChannels, deviceOutputs, &outputMap) ||
        render == nullptr ||
        config.maximumFrames == 0 || config.maximumFrames > kAudioHostMaxFrames) {
      return fail(AudioHostError::InvalidConfiguration,
                  "Invalid physical channel map, frame bound, or render thunk");
    }
    detail::MacAudioBufferProperties bufferProperties{selectedDevice_, selectedDeviceUid_};
    const auto buffer = detail::requestMacAudioBuffer(
        bufferProperties, config.requestedBufferFrames, config.maximumFrames,
        bufferOwnership_, acceptableBufferMaximum_, detail::macAudioBufferLedger(),
        true, detail::MacAudioBufferFixedCapacity{}, config.holdSharedDeviceBuffer);
    bufferWarnings_ |= buffer.warnings;
    if (!buffer.ok()) return fail(buffer.error, buffer.message);
    const UInt32 nominalFrames = buffer.frames;

    AudioComponentDescription description{};
    description.componentType = kAudioUnitType_Output;
    description.componentSubType = kAudioUnitSubType_HALOutput;
    description.componentManufacturer = kAudioUnitManufacturer_Apple;
    AudioComponent component = AudioComponentFindNext(nullptr, &description);
    if (component == nullptr || AudioComponentInstanceNew(component, &unit_) != noErr) {
      return fail(AudioHostError::ProviderFailure, "CoreAudio AUHAL is unavailable");
    }
    UInt32 inputEnabled = hasInput ? 1u : 0u;
    UInt32 outputEnabled = 1;
    if (AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Input, 1, &inputEnabled,
                             sizeof(inputEnabled)) != noErr ||
        AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_EnableIO,
                             kAudioUnitScope_Output, 0, &outputEnabled,
                             sizeof(outputEnabled)) != noErr ||
        AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_CurrentDevice,
                             kAudioUnitScope_Global, 0, &selectedDevice_,
                             sizeof(selectedDevice_)) != noErr) {
      return fail(AudioHostError::ProviderFailure,
                  hasInput ? "Could not configure AUHAL duplex I/O"
                           : "Could not configure AUHAL output-only I/O (singz-auhal-output-only-v1)");
    }
    const uint32_t inputChannels = static_cast<uint32_t>(config.inputChannels.size());
    const uint32_t outputChannels = static_cast<uint32_t>(config.outputChannels.size());
    AudioStreamBasicDescription inputFormat = planarFormat(rate, inputChannels);
    AudioStreamBasicDescription outputFormat = planarFormat(rate, outputChannels);
    if ((hasInput &&
         AudioUnitSetProperty(unit_, kAudioUnitProperty_StreamFormat,
                              kAudioUnitScope_Output, 1, &inputFormat,
                              sizeof(inputFormat)) != noErr) ||
        AudioUnitSetProperty(unit_, kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input, 0, &outputFormat,
                             sizeof(outputFormat)) != noErr) {
      return fail(AudioHostError::ProviderFailure, "Could not set AUHAL float32 planar formats");
    }
    std::vector<SInt32> inputMap(inputChannels);
    for (uint32_t index = 0; index < inputChannels; ++index) {
      inputMap[index] = static_cast<SInt32>(config.inputChannels[index]);
    }
    if ((hasInput &&
         AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_ChannelMap,
                              kAudioUnitScope_Output, 1, inputMap.data(),
                              static_cast<UInt32>(inputMap.size() *
                                                  sizeof(SInt32))) != noErr) ||
        AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_ChannelMap,
                             kAudioUnitScope_Input, 0, outputMap.data(),
                             static_cast<UInt32>(outputMap.size() *
                                                 sizeof(outputMap[0]))) != noErr) {
      return fail(AudioHostError::InvalidConfiguration,
                  "CoreAudio rejected the explicit physical channel maps");
    }
    UInt32 maximumFrames = config.maximumFrames;
    if (AudioUnitSetProperty(unit_, kAudioUnitProperty_MaximumFramesPerSlice,
                             kAudioUnitScope_Global, 0, &maximumFrames,
                             sizeof(maximumFrames)) != noErr) {
      return fail(AudioHostError::ProviderFailure, "Could not set AUHAL callback frame bound");
    }
    if (hasInput) {
      inputSamples_.resize(static_cast<size_t>(inputChannels) * maximumFrames);
      const size_t inputListBytes = offsetof(AudioBufferList, mBuffers) +
                                    sizeof(AudioBuffer) * inputChannels;
      inputListStorage_.resize((inputListBytes + sizeof(std::max_align_t) - 1) /
                               sizeof(std::max_align_t));
      inputList_ = reinterpret_cast<AudioBufferList*>(inputListStorage_.data());
      inputList_->mNumberBuffers = inputChannels;
      for (uint32_t channel = 0; channel < inputChannels; ++channel) {
        inputPointers_[channel] = inputSamples_.data() +
                                  static_cast<size_t>(channel) * maximumFrames;
        inputList_->mBuffers[channel].mNumberChannels = 1;
        inputList_->mBuffers[channel].mDataByteSize = maximumFrames * sizeof(float);
        inputList_->mBuffers[channel].mData =
            const_cast<float*>(inputPointers_[channel]);
      }
    }
    mach_timebase_info_data_t timebase{};
    if (mach_timebase_info(&timebase) != KERN_SUCCESS || timebase.denom == 0) {
      return fail(AudioHostError::ProviderFailure, "Mach host-time conversion is unavailable");
    }
    nanosecondsPerTick_ = static_cast<double>(timebase.numer) / timebase.denom;
    format_ = {rate, maximumFrames, nominalFrames, inputChannels, outputChannels, true, true};
    latency_ = {hasInput
                    ? latency(selectedDevice_, kAudioDevicePropertyScopeInput)
                    : 0u,
                latency(selectedDevice_, kAudioDevicePropertyScopeOutput),
                nominalFrames, 0};
    listenerContext_ = std::make_shared<HostDeviceListenerContext>();
    listenerContext_->device = selectedDevice_;
    listenerContext_->uid = selectedDeviceUid_;
    listenerContext_->hasInput = hasInput;
    listenerContext_->bufferOwnership = bufferOwnership_;
    listenerContext_->bufferFrames.store(nominalFrames, std::memory_order_relaxed);
    listenerContext_->routeGeneration.store(
        lastRouteGeneration_.fetch_add(1, std::memory_order_relaxed) + 1,
        std::memory_order_relaxed);
    streamGeneration_.fetch_add(1, std::memory_order_relaxed);
    observedCallbackFrames_.store(0, std::memory_order_relaxed);
    fallbackOutputFrame_ = 0;
    outputTimeline_ = {};
    callbackSequence_ = 0;
    inputSourceFrame_ = 0;
    endpoint_.callbacks.store(0, std::memory_order_relaxed);
    endpoint_.renderedFrames.store(0, std::memory_order_relaxed);
    endpoint_.xruns.store(0, std::memory_order_relaxed);
    endpoint_.deadlineMisses.store(0, std::memory_order_relaxed);
    endpoint_.discontinuities.store(0, std::memory_order_relaxed);
    endpoint_.invalidCallbacks.store(0, std::memory_order_relaxed);
    endpoint_.renderFailures.store(0, std::memory_order_relaxed);
    prepareAudioHostCallback(&endpoint_, render, renderContext);
    // The heap gate is the HAL's callback context so it can be quarantined
    // past backend destruction on an unclean teardown.
    callbackGate_ = std::make_unique<CallbackGate>();
    callbackGate_->open(this);
    AURenderCallbackStruct callback{renderCallback, callbackGate_.get()};
    if (AudioUnitSetProperty(unit_, kAudioUnitProperty_SetRenderCallback,
                             kAudioUnitScope_Input, 0, &callback,
                             sizeof(callback)) != noErr ||
        AudioUnitInitialize(unit_) != noErr) {
      return fail(AudioHostError::ProviderFailure, "Could not initialize AUHAL rendering");
    }
    initialized_ = true;
    // Snapshot the configuration this initialized unit actually opened with.
    // Its negotiated stream formats may differ from the pre-open defaults.
    const auto matchesChannels = [](const std::vector<uint32_t>& channels, uint32_t expected) {
      uint64_t total = 0;
      for (const auto count : channels) total += count;
      return total == expected;
    };
    detail::MacAudioHostDeviceFacts openedFacts;
    if (!detail::readMacAudioHostFactsWithRetry(
            [&] { return readHostDeviceFacts(selectedDevice_, hasInput, openedFacts); }) || openedFacts.rate != rate ||
        !matchesChannels(openedFacts.inputChannels, deviceInputs) ||
        !matchesChannels(openedFacts.outputChannels, deviceOutputs))
      return fail(AudioHostError::ProviderFailure, "CoreAudio stream configuration is unavailable");
    listenerContext_->opened = std::move(openedFacts);
    listenerQueue_ = dispatch_queue_create("com.lexasoft.singz.audio-host-device", nullptr);
    if (listenerQueue_ == nullptr) {
      return fail(AudioHostError::ProviderFailure,
                  "Could not create the CoreAudio route listener queue");
    }
    const auto listener = listenerContext_;
    listenerBlock_ = Block_copy(^(UInt32 count,
                                  const AudioObjectPropertyAddress addresses[]) {
      bool inspectConfiguration = false;
      for (UInt32 index = 0; index < count; ++index) {
        const auto selector = addresses[index].mSelector;
        inspectConfiguration |= selector != kAudioDevicePropertyBufferFrameSize;
      }
      const uint32_t changes = detail::macAudioHostControlNotification(inspectConfiguration, [&] {
        const auto checked = hostDeviceChanges(*listener);
        listener->bufferWarnings.fetch_or(checked.warnings, std::memory_order_relaxed);
        return checked.changes;
      });
      // Dedicated serial listener queue is an ordinary control domain, never
      // the render callback. Publish HAL observations; status() only reads.
      for (UInt32 index = 0; index < count; ++index) {
        if (addresses[index].mSelector == kAudioDevicePropertyBufferFrameSize) {
          UInt32 frames = 0;
          if (readProperty(listener->device,
                property(kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal),
                &frames) && frames != 0)
            listener->bufferFrames.store(frames, std::memory_order_release);
          else
            listener->bufferWarnings.fetch_or(AudioBufferWarningReadFailed, std::memory_order_relaxed);
        }
      }
      if ((changes & kAliveChanged) != 0) {
        // Device loss ends the lease without waiting for a status poll/stop.
        // This queue is a control domain; callbacks never touch the ledger.
        detail::MacAudioBufferProperties properties{listener->device, listener->uid};
        try {
          listener->bufferWarnings.fetch_or(
              detail::restoreMacAudioBuffer(properties, listener->bufferOwnership),
              std::memory_order_relaxed);
        } catch (...) {
          listener->bufferWarnings.fetch_or(AudioBufferWarningRestoreFailed,
                                            std::memory_order_relaxed);
        }
        listener->deviceLost.store(1, std::memory_order_release);
      }
      if (changes != 0) {
        listener->pendingChanges.fetch_or(changes, std::memory_order_release);
        listener->routeGeneration.fetch_add(1, std::memory_order_relaxed);
        listener->routeChanged.store(1, std::memory_order_release);
      }
    });
    if (listenerBlock_ == nullptr) {
      return fail(AudioHostError::ProviderFailure,
                  "Could not allocate the CoreAudio route listener block");
    }
    // Reserve before registering so a failed allocation cannot leave an
    // untracked listener. All registrations use the same serial queue/block.
    std::vector<std::pair<AudioObjectID, AudioObjectPropertyAddress>> desiredWatched;
    for (auto selector : std::array<AudioObjectPropertySelector, 3>{
             kAudioDevicePropertyDeviceIsAlive, kAudioDevicePropertyNominalSampleRate,
             kAudioDevicePropertyBufferFrameSize})
      desiredWatched.emplace_back(selectedDevice_, property(selector, kAudioObjectPropertyScopeGlobal));
    desiredWatched.emplace_back(kAudioObjectSystemObject,
        property(kAudioHardwarePropertyDevices, kAudioObjectPropertyScopeGlobal));
    for (auto scope : {kAudioDevicePropertyScopeInput, kAudioDevicePropertyScopeOutput}) {
      if (!detail::macAudioHostUsesScope(hasInput, scope == kAudioDevicePropertyScopeInput)) continue;
      desiredWatched.emplace_back(selectedDevice_, property(kAudioDevicePropertyStreamConfiguration, scope));
      desiredWatched.emplace_back(selectedDevice_, property(kAudioDevicePropertyStreams, scope));
    }
    for (const auto* streams : {&listener->opened.inputStreams, &listener->opened.outputStreams})
      for (const auto& stream : *streams)
        for (auto selector : {kAudioStreamPropertyPhysicalFormat, kAudioStreamPropertyVirtualFormat})
          desiredWatched.emplace_back(stream.identity, property(selector, kAudioObjectPropertyScopeGlobal));
    watched_.reserve(desiredWatched.size());
    for (const auto& [object, address] : desiredWatched) {
      if (AudioObjectAddPropertyListenerBlock(object, &address, listenerQueue_, listenerBlock_) != noErr)
        return fail(AudioHostError::ProviderFailure, "Could not watch the CoreAudio route");
      watched_.emplace_back(object, address);
    }
    // Fence changes between the opening snapshot and listener installation.
    const auto openingChanges = hostDeviceChanges(*listener);
    listener->bufferWarnings.fetch_or(openingChanges.warnings, std::memory_order_relaxed);
    if (openingChanges.changes != 0)
      return fail(AudioHostError::ProviderFailure, "CoreAudio route changed while opening");
    firstCallback_.store(1, std::memory_order_relaxed);
    state_.store(AudioHostState::Open, std::memory_order_release);
    return {true, AudioHostError::None, AudioHostState::Open, format_, latency_, buffer.message};
  }

  AudioHostResult start() override {
    std::lock_guard lock(controlMutex_);
    if (!initialized_ || state_.load(std::memory_order_acquire) != AudioHostState::Open) {
      return failure(AudioHostError::InvalidState,
                     "Open AUHAL before starting it",
                     state_.load(std::memory_order_acquire));
    }
    activateAudioHostCallback(&endpoint_);
    if (AudioOutputUnitStart(unit_) != noErr) {
      if (callbackGate_) callbackGate_->beginClose();
      deactivateAudioHostCallback(&endpoint_);
      return fail(AudioHostError::ProviderFailure, "Could not start AUHAL");
    }
    started_ = true;
    state_.store(AudioHostState::Running, std::memory_order_release);
    return {true, AudioHostError::None, AudioHostState::Running, reportedFormat(), reportedLatency(), {}};
  }

  AudioHostResult setBufferFrames(std::uint32_t frames,
                                   std::uint32_t maximumAcceptableBufferFrames = 0) override {
    std::lock_guard lock(controlMutex_);
    const auto current = status();
    if (!initialized_ || (current.state != AudioHostState::Open &&
                          current.state != AudioHostState::Running)) {
      return {false, AudioHostError::InvalidState, current.state, current.format,
              current.latency, "Open AUHAL before changing its buffer size"};
    }
    if (frames == 0) {
      return {false, AudioHostError::InvalidConfiguration, current.state,
              current.format, current.latency, "A live buffer request must be nonzero"};
    }
    if (!allowSharedDeviceBufferChange_) {
      return {false, AudioHostError::Unsupported, current.state, current.format,
              current.latency, "Shared CoreAudio output buffer changes require allowSharedDeviceBufferChange"};
    }
    detail::MacAudioBufferProperties properties{selectedDevice_, selectedDeviceUid_};
    const auto buffer = detail::requestMacAudioBuffer(
        properties, frames, format_.maximumFrames, bufferOwnership_,
        detail::macAudioBufferCeiling(acceptableBufferMaximum_, maximumAcceptableBufferFrames));
    bufferWarnings_ |= buffer.warnings;
    // Only control-domain telemetry changes. The callback's format and storage
    // stay immutable, prepared for maximumFrames even while the device changes.
    if (buffer.ok()) {
      latency_.bufferFrames = buffer.frames;
    } else {
      uint32_t negotiated = 0;
      if (properties.readFrames(negotiated) && negotiated != 0) latency_.bufferFrames = negotiated;
    }
    listenerContext_->bufferFrames.store(latency_.bufferFrames, std::memory_order_release);
    const auto updated = status();
    return {buffer.ok(), buffer.error, updated.state, updated.format, updated.latency, buffer.message};
  }

  void stop() noexcept override {
    std::lock_guard lock(controlMutex_);
    // Close admission first: a callback that has not yet counted itself is
    // refused here instead of slipping past the drain below, whatever
    // AudioOutputUnitStop returns. Every callback that did enter must finish
    // — including its preamble and postamble outside endpoint_.inFlight —
    // before listener teardown, unit disposal, or any member release below.
    if (callbackGate_) callbackGate_->beginClose();
    deactivateAudioHostCallback(&endpoint_);
    bool teardownClean = true;
    if (started_ && unit_ != nullptr && AudioOutputUnitStop(unit_) != noErr) teardownClean = false;
    if (callbackGate_) {
      while (callbackGate_->inFlight() != 0) std::this_thread::yield();
    }
    while (endpoint_.inFlight.load(std::memory_order_acquire) != 0) std::this_thread::yield();
    bool removedAll = true;
    for (const auto& [object, address] : watched_) {
      const auto removal = AudioObjectRemovePropertyListenerBlock(object, &address, listenerQueue_, listenerBlock_);
      const bool streamObject = address.mSelector == kAudioStreamPropertyPhysicalFormat ||
                                address.mSelector == kAudioStreamPropertyVirtualFormat;
      bool objectGone = removal == kAudioHardwareBadObjectError;
      if (removal != noErr && streamObject && !objectGone) {
        UInt32 objectClass = 0;
        UInt32 size = sizeof(objectClass);
        const auto classAddress = property(kAudioObjectPropertyClass, kAudioObjectPropertyScopeGlobal);
        objectGone = AudioObjectGetPropertyData(object, &classAddress, 0, nullptr, &size,
                                                &objectClass) == kAudioHardwareBadObjectError;
      }
      if (!detail::macAudioHostListenerRemoved(removal == noErr, streamObject, objectGone))
        removedAll = false;
    }
    if (listenerContext_ != nullptr && listenerQueue_ != nullptr &&
        listenerBlock_ != nullptr) {
      if (removedAll) {
        dispatch_sync_f(listenerQueue_, nullptr, [](void*) {});
        Block_release(listenerBlock_);
        dispatch_release(listenerQueue_);
      }
      // Failed removal: retain our existing block/queue references. The block
      // owns the shared context, so no allocating quarantine container is needed.

    } else if (listenerQueue_ != nullptr) {
      dispatch_release(listenerQueue_);
    }
    watched_.clear();
    if (listenerContext_ != nullptr) {
      const uint64_t finalGeneration = listenerContext_->routeGeneration.load(
          std::memory_order_relaxed);
      if (finalGeneration > lastRouteGeneration_.load(std::memory_order_relaxed)) {
        lastRouteGeneration_.store(finalGeneration, std::memory_order_relaxed);
      }
    }
    if (listenerContext_) {
      latency_.bufferFrames = listenerContext_->bufferFrames.load(std::memory_order_acquire);
      bufferWarnings_ |= listenerContext_->bufferWarnings.load(std::memory_order_relaxed);
    }
    listenerContext_.reset();
    listenerQueue_ = nullptr;
    listenerBlock_ = nullptr;
    if (initialized_ && unit_ != nullptr && AudioUnitUninitialize(unit_) != noErr)
      teardownClean = false;
    if (unit_ != nullptr && AudioComponentInstanceDispose(unit_) != noErr)
      teardownClean = false;
    if (callbackGate_) {
      // Free the HAL-reachable context only after a fully clean teardown; a
      // callback past that point is beyond this defense, and the platform's
      // Dispose contract is the accepted boundary there. Any failure (or a
      // non-quiescent gate) leaks the context on purpose so a misbehaving
      // HAL reads a closed gate instead of freed memory — the iOS host's
      // poison-lease idiom. The leaked gate still drops its owner pointer
      // when quiescent so nothing dangles behind the quarantine.
      teardownClean = teardownClean && callbackGate_->clearOwnerIfQuiescent();
      if (teardownClean)
        callbackGate_.reset();
      else {
        (void)callbackGate_->clearOwnerIfQuiescent();
        callbackGate_.release();
      }
    }
    detail::MacAudioBufferProperties bufferProperties{selectedDevice_, selectedDeviceUid_};
    if (bufferOwnership_.token != 0) {
      try {
        bufferWarnings_ |= detail::restoreMacAudioBuffer(bufferProperties, bufferOwnership_);
      } catch (...) {
        bufferWarnings_ |= AudioBufferWarningRestoreFailed;
      }
    }
    unit_ = nullptr;
    initialized_ = false;
    started_ = false;
    inputList_ = nullptr;
    inputListStorage_.clear();
    inputSamples_.clear();
    if (bufferOwnership_.token == 0) selectedDevice_ = kAudioObjectUnknown;
    const AudioHostState previous = state_.load(std::memory_order_acquire);
    state_.store(previous == AudioHostState::Closed
                     ? AudioHostState::Closed : AudioHostState::Stopped,
                 std::memory_order_release);
  }

  AudioHostStatus status() const noexcept override {
    std::lock_guard lock(controlMutex_);
    const uint32_t changes = listenerContext_ != nullptr
                                 ? listenerContext_->pendingChanges.load(
                                       std::memory_order_acquire)
                                 : 0;
    AudioHostStatus result;
    result.latency = reportedLatency();
    result.bufferExceedsMaximum = initialized_ && detail::macAudioHostBufferExceedsMaximum(
        result.latency.bufferFrames, format_.maximumFrames);
    result.bufferWarnings = bufferWarnings_ | (listenerContext_ != nullptr
        ? listenerContext_->bufferWarnings.load(std::memory_order_relaxed) : 0u);
    result.state = listenerContext_ && listenerContext_->deviceLost.load(std::memory_order_acquire)
                       ? AudioHostState::DeviceLost
                       : (((changes & kFormatChanged) != 0 || result.bufferExceedsMaximum)
                              ? AudioHostState::Error
                              : state_.load(std::memory_order_acquire));
    result.observedCallbackFrames = observedCallbackFrames_.load(std::memory_order_relaxed);
    result.format = format_;
    result.format.nominalBufferFrames = result.latency.bufferFrames;
    result.routeGeneration = listenerContext_ != nullptr
                                 ? listenerContext_->routeGeneration.load(
                                       std::memory_order_relaxed)
                                 : lastRouteGeneration_.load(
                                       std::memory_order_relaxed);
    result.streamGeneration = streamGeneration_.load(std::memory_order_relaxed);
    result.callbacks = endpoint_.callbacks.load(std::memory_order_relaxed);
    result.renderedFrames = endpoint_.renderedFrames.load(std::memory_order_relaxed);
    result.xruns = endpoint_.xruns.load(std::memory_order_relaxed);
    result.deadlineMisses = endpoint_.deadlineMisses.load(std::memory_order_relaxed);
    result.discontinuities = endpoint_.discontinuities.load(std::memory_order_relaxed);
    result.invalidCallbacks = endpoint_.invalidCallbacks.load(std::memory_order_relaxed);
    result.renderFailures = endpoint_.renderFailures.load(std::memory_order_relaxed);
    return result;
  }

 private:
  AudioHostFormat reportedFormat() const noexcept {
    auto result = format_;
    result.nominalBufferFrames = reportedLatency().bufferFrames;
    return result;
  }

  AudioHostLatency reportedLatency() const noexcept {
    auto result = latency_;
    if (listenerContext_) result.bufferFrames = listenerContext_->bufferFrames.load(std::memory_order_acquire);
    return result;
  }

  static OSStatus renderCallback(void* context, AudioUnitRenderActionFlags* flags,
                                 const AudioTimeStamp* timestamp, UInt32,
                                 UInt32 frames, AudioBufferList* output) noexcept {
    auto* gate = static_cast<CallbackGate*>(context);
    if (gate == nullptr || output == nullptr) return noErr;
    // Admit-and-count through the outliving gate, then resolve the owner.
    // A refused entry means stop() is tearing down (or the backend is gone):
    // silence the outputs, which need no backend state, and return.
    AudioInputCallbackOwnerScope admission(*gate);
    if (!admission) {
      for (UInt32 channel = 0; channel < output->mNumberBuffers; ++channel) {
        auto* bytes = static_cast<uint8_t*>(output->mBuffers[channel].mData);
        if (bytes != nullptr && output->mBuffers[channel].mDataByteSize > 0)
          std::memset(bytes, 0, output->mBuffers[channel].mDataByteSize);
      }
      return noErr;
    }
    MacAudioHostBackend* self = admission.get();
    self->observedCallbackFrames_.store(frames, std::memory_order_relaxed);
    const uint64_t callbackTicks = mach_absolute_time();
    if (frames == 0 || frames > self->format_.maximumFrames ||
        output->mNumberBuffers < self->format_.outputChannels) {
      AudioHostRenderBlock invalid{};
      for (UInt32 channel = 0; channel < output->mNumberBuffers; ++channel) {
        auto* bytes = static_cast<uint8_t*>(output->mBuffers[channel].mData);
        if (bytes == nullptr) continue;
        for (UInt32 byte = 0; byte < output->mBuffers[channel].mDataByteSize;
             ++byte) {
          bytes[byte] = 0;
        }
      }
      invalid.outputChannels = std::min<uint32_t>(output->mNumberBuffers,
                                                  kAudioHostMaxChannels);
      invalid.frames = std::min<uint32_t>(frames, self->format_.maximumFrames);
      invalid.maximumFrames = self->format_.maximumFrames;
      invalid.sampleRate = self->format_.sampleRate;
      invalid.outputClockMaster = false;
      for (uint32_t c = 0; c < invalid.outputChannels; ++c) {
        self->outputPointers_[c] = static_cast<float*>(output->mBuffers[c].mData);
      }
      invalid.output = self->outputPointers_.data();
      invokeAudioHostCallback(&self->endpoint_, invalid);
      if (flags != nullptr) {
        *flags = static_cast<AudioUnitRenderActionFlags>(
            audioHostFinalActionFlags(*flags,
                                      kAudioUnitRenderAction_OutputIsSilence,
                                      true));
      }
      return noErr;
    }
    OSStatus inputStatus = noErr;
    AudioUnitRenderActionFlags inputFlags = 0;
    if (detail::shouldPullMacAudioHostInput(self->format_.inputChannels)) {
      for (uint32_t channel = 0; channel < self->format_.inputChannels;
           ++channel) {
        self->inputList_->mBuffers[channel].mDataByteSize =
            frames * sizeof(float);
      }
      inputStatus = AudioUnitRender(self->unit_, &inputFlags, timestamp, 1, frames,
                                    self->inputList_);
    }
    const bool inputFailed = audioHostInputPullFailed(
        inputStatus, inputFlags, kAudioUnitRenderAction_PostRenderError);
    if (inputFailed) {
      recordAudioHostXRun(&self->endpoint_);
      for (uint32_t channel = 0; channel < self->format_.inputChannels; ++channel) {
        std::fill_n(const_cast<float*>(self->inputPointers_[channel]), frames, 0.0F);
      }
    }
    for (uint32_t channel = 0; channel < self->format_.outputChannels; ++channel) {
      self->outputPointers_[channel] = static_cast<float*>(output->mBuffers[channel].mData);
    }
    uint32_t discontinuity = AudioHostDiscontinuityNone;
    if (self->firstCallback_.exchange(0, std::memory_order_relaxed) != 0) {
      discontinuity |= AudioHostDiscontinuityStart;
    }
    if (self->listenerContext_ != nullptr &&
        self->listenerContext_->routeChanged.exchange(
            0, std::memory_order_acq_rel) != 0) {
      discontinuity |= AudioHostDiscontinuityRouteChanged;
    }
    if (self->listenerContext_ && self->listenerContext_->deviceLost.load(std::memory_order_acquire) != 0) {
      discontinuity |= AudioHostDiscontinuityDeviceLost;
    }
    if (inputFailed) {
      discontinuity |= AudioHostDiscontinuityXRun;
    }
    const uint64_t callbackNs = static_cast<uint64_t>(
        static_cast<long double>(callbackTicks) * self->nanosecondsPerTick_);
    const bool outputHostTimeValid = timestamp != nullptr &&
                                     (timestamp->mFlags &
                                      kAudioTimeStampHostTimeValid) != 0 &&
                                     timestamp->mHostTime != 0;
    const uint64_t outputNs = outputHostTimeValid
                                  ? static_cast<uint64_t>(
                                        static_cast<long double>(timestamp->mHostTime) *
                                        self->nanosecondsPerTick_)
                                  : callbackNs;
    const bool hasInput = detail::shouldPullMacAudioHostInput(
        self->format_.inputChannels);
    const bool inputHardwareValid = hasInput && timestamp != nullptr &&
                                    (timestamp->mFlags &
                                     kAudioTimeStampHostTimeValid) != 0 &&
                                    timestamp->mHostTime != 0;
    const uint64_t inputHardwareNs = inputHardwareValid
                                         ? static_cast<uint64_t>(
                                               static_cast<long double>(
                                                   timestamp->mHostTime) *
                                               self->nanosecondsPerTick_)
                                         : 0;
    const AudioInputTimestampProjection inputProjection = hasInput
        ? resolveAudioInputTimestamp(inputHardwareValid, inputHardwareNs,
                                     callbackNs, frames,
                                     self->format_.sampleRate)
        : AudioInputTimestampProjection{};
    const uint64_t inputSourceFrame = hasInput ? self->inputSourceFrame_ : 0;
    if (hasInput) {
      self->inputSourceFrame_ =
          advanceAudioHostFrame(self->inputSourceFrame_, frames);
    }
    const bool outputFrameValid = timestamp != nullptr &&
                                  (timestamp->mFlags &
                                   kAudioTimeStampSampleTimeValid) != 0 &&
                                  validAudioHostSampleFrame(
                                      timestamp->mSampleTime, frames);
    const auto timeline = resolveAudioHostOutputTimeline(
        &self->outputTimeline_, outputFrameValid,
        outputFrameValid ? static_cast<uint64_t>(timestamp->mSampleTime) : 0,
        outputHostTimeValid, frames, self->fallbackOutputFrame_);
    const uint64_t outputFrame = timeline.outputFrame;
    discontinuity |= timeline.discontinuity;
    self->fallbackOutputFrame_ = advanceAudioHostFrame(outputFrame, frames);
    const uint64_t callbackSequence = self->callbackSequence_;
    self->callbackSequence_ = advanceAudioHostFrame(self->callbackSequence_, 1);
    AudioHostRenderBlock block{
        hasInput ? self->inputPointers_.data() : nullptr,
        self->outputPointers_.data(),
        self->format_.inputChannels, self->format_.outputChannels, frames,
        self->format_.maximumFrames, self->format_.sampleRate, 1,
        self->listenerContext_ != nullptr
            ? self->listenerContext_->routeGeneration.load(
                  std::memory_order_relaxed)
            : self->lastRouteGeneration_.load(std::memory_order_relaxed),
        self->streamGeneration_.load(std::memory_order_relaxed),
        callbackSequence,
        inputSourceFrame, inputProjection.sampleHostTimeNs,
        detail::macAudioHostInputTimestampValid(
            self->format_.inputChannels),
        detail::macAudioHostInputTimestampHardware(
            self->format_.inputChannels,
            inputProjection.usedHardwareAnchor),
        outputFrame, outputNs, outputNs != 0, outputHostTimeValid, callbackNs,
        discontinuity, true};
    const bool rendered = invokeAudioHostCallback(&self->endpoint_, block);
    if (flags != nullptr) {
      *flags = static_cast<AudioUnitRenderActionFlags>(
          audioHostFinalActionFlags(*flags,
                                    kAudioUnitRenderAction_OutputIsSilence,
                                    !rendered));
    }
    const uint64_t elapsedTicks = mach_absolute_time() - callbackTicks;
    const long double elapsedNs = static_cast<long double>(elapsedTicks) *
                                  self->nanosecondsPerTick_;
    const long double deadlineNs = static_cast<long double>(frames) * 1000000000.0L /
                                   self->format_.sampleRate;
    if (elapsedNs > deadlineNs) recordAudioHostDeadlineMiss(&self->endpoint_);
    return noErr;
  }

  AudioHostResult fail(AudioHostError error, const std::string& message) {
    stop();
    state_.store(AudioHostState::Error, std::memory_order_release);
    return failure(error, message);
  }

  AudioHostResult reject(
      AudioHostError error, const std::string& message,
      AudioHostState state = AudioHostState::Error) noexcept {
    state_.store(state, std::memory_order_release);
    return failure(error, message, state);
  }

  AudioUnit unit_{nullptr};
  AudioDeviceID selectedDevice_{kAudioObjectUnknown};
  bool initialized_{false};
  bool started_{false};
  detail::MacAudioBufferOwnership bufferOwnership_;
  AudioHostFormat format_{};
  AudioHostLatency latency_{};
  mutable std::recursive_mutex controlMutex_;
  std::string selectedDeviceUid_;
  uint32_t acceptableBufferMaximum_{0};
  uint32_t bufferWarnings_{AudioBufferWarningNone};
  bool allowSharedDeviceBufferChange_{false};
  std::atomic<uint32_t> observedCallbackFrames_{0};
  AudioHostCallbackEndpoint endpoint_{};
  // Registered with the HAL in place of `this`; see the comment above the
  // backend for the outliving/quarantine contract.
  using CallbackGate = AudioInputCallbackOwnerGate<MacAudioHostBackend>;
  std::unique_ptr<CallbackGate> callbackGate_;
  std::atomic<AudioHostState> state_{AudioHostState::Closed};
  std::atomic<uint64_t> lastRouteGeneration_{0};
  std::atomic<uint64_t> streamGeneration_{0};
  uint64_t fallbackOutputFrame_{0};
  AudioHostOutputTimeline outputTimeline_{};
  uint64_t callbackSequence_{0};
  std::atomic<uint32_t> firstCallback_{0};
  uint64_t inputSourceFrame_{0};
  double nanosecondsPerTick_{0.0};
  std::vector<float> inputSamples_;
  std::vector<std::max_align_t> inputListStorage_;
  AudioBufferList* inputList_{nullptr};
  std::array<const float*, kAudioHostMaxChannels> inputPointers_{};
  std::array<float*, kAudioHostMaxChannels> outputPointers_{};
  std::vector<std::pair<AudioObjectID, AudioObjectPropertyAddress>> watched_;
  std::shared_ptr<HostDeviceListenerContext> listenerContext_;
  dispatch_queue_t listenerQueue_{nullptr};
  AudioObjectPropertyListenerBlock listenerBlock_{nullptr};
};

}  // namespace

std::unique_ptr<AudioHostBackend> createPlatformAudioHostBackend() {
  return std::make_unique<MacAudioHostBackend>();
}

}  // namespace singz
