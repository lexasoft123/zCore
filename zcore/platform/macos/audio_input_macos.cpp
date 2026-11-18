#include <zcore/device/audio_input_backend.h>
#include <zcore/device/audio_input_callback_gate.h>

#if defined(__APPLE__)
#include <TargetConditionals.h>
#endif

#if defined(__APPLE__) && TARGET_OS_OSX

#include <AudioToolbox/AudioToolbox.h>
#include <CoreAudio/CoreAudio.h>
#include <CoreFoundation/CoreFoundation.h>
#include <Block.h>
#include <dispatch/dispatch.h>
#include <mach/mach_time.h>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <zcore/audio/audio_input_timestamp.h>
#include "audio_input_macos_latency.h"
#include "audio_input_macos_render.h"
#include "audio_device_macos_buffer.h"
#include "audio_device_macos_buffer_properties.h"

namespace singz {
namespace {

static_assert(kAudioUnitErr_TooManyFramesToProcess == detail::kMacInputTooManyFrames);

static_assert(kAudioDevicePropertyDeviceIsAlive == detail::audioHostFourCc('l', 'i', 'v', 'n'));
static_assert(kAudioDevicePropertyNominalSampleRate == detail::audioHostFourCc('n', 's', 'r', 't'));
static_assert(kAudioDevicePropertyStreamConfiguration == detail::audioHostFourCc('s', 'l', 'a', 'y'));
static_assert(kAudioDevicePropertyStreams == detail::audioHostFourCc('s', 't', 'm', '#'));
static_assert(kAudioDevicePropertyBufferFrameSize == detail::audioHostFourCc('f', 's', 'i', 'z'));
static_assert(kAudioStreamPropertyPhysicalFormat == detail::audioHostFourCc('p', 'f', 't', ' '));
static_assert(kAudioStreamPropertyVirtualFormat == detail::audioHostFourCc('s', 'f', 'm', 't'));

struct DeviceListenerContext {
  std::atomic<uint32_t> pendingChanges{0};
  std::atomic<bool> latencyStable{true};
  std::atomic<uint64_t> latencyRevision{0};
};

// Admission gate for renderCallback: entry is counted and admitted in one
// atomic word, so a callback that races the close in stop()/suspendPrepared()
// either enters before the drain observes zero or is refused without reading
// unit_, push_ or renderBuffer_.

// The copied listener block owns its shared context. Failed removal retains
// the already-allocated block/queue until process exit, without allocating on stop.
AudioObjectPropertyAddress address(AudioObjectPropertySelector selector,
                                   AudioObjectPropertyScope scope,
                                   AudioObjectPropertyElement element = kAudioObjectPropertyElementMain) {
  return {selector, scope, element};
}

template <typename T>
bool getProperty(AudioObjectID object, AudioObjectPropertyAddress property, T& value) {
  UInt32 size = sizeof(value);
  return AudioObjectGetPropertyData(object, &property, 0, nullptr, &size, &value) == noErr &&
         size == sizeof(value);
}

std::optional<uint32_t> frameProperty(AudioObjectID object,
                                     AudioObjectPropertySelector selector,
                                     AudioObjectPropertyScope scope) {
  UInt32 value = 0;
  if (!getProperty(object, address(selector, scope), value)) return std::nullopt;
  return value;
}

// Select the stream containing the physical input channel, rather than using
// the first stream (multi-stream interfaces can have different latencies).
AudioStreamID inputStream(AudioDeviceID device, uint32_t channel) {
  const auto property = address(kAudioDevicePropertyStreams, kAudioDevicePropertyScopeInput);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(device, &property, 0, nullptr, &size) != noErr ||
      !size || size % sizeof(AudioStreamID) || size > 4096 * sizeof(AudioStreamID))
    return kAudioObjectUnknown;
  std::vector<AudioStreamID> streams(size / sizeof(AudioStreamID));
  const UInt32 capacity = size;
  if (AudioObjectGetPropertyData(device, &property, 0, nullptr, &size, streams.data()) != noErr ||
      size > capacity || size % sizeof(AudioStreamID)) return kAudioObjectUnknown;
  streams.resize(size / sizeof(AudioStreamID));
  for (const auto stream : streams) {
    UInt32 first = 0;
    AudioStreamBasicDescription format{};
    if (!getProperty(stream, address(kAudioStreamPropertyStartingChannel,
                                     kAudioObjectPropertyScopeGlobal), first) || !first ||
        !getProperty(stream, address(kAudioStreamPropertyVirtualFormat,
                                     kAudioObjectPropertyScopeGlobal), format)) continue;
    if (channel >= first - 1 && uint64_t{channel} < uint64_t{first - 1} + format.mChannelsPerFrame)
      return stream;
  }
  return kAudioObjectUnknown;
}

std::string cfString(CFStringRef value) {
  if (!value) return {};
  const CFIndex length = CFStringGetLength(value);
  const CFIndex maxBytes = CFStringGetMaximumSizeForEncoding(length, kCFStringEncodingUTF8) + 1;
  if (maxBytes <= 1 || maxBytes > 65536) return {};
  std::vector<char> bytes(static_cast<size_t>(maxBytes));
  if (!CFStringGetCString(value, bytes.data(), maxBytes, kCFStringEncodingUTF8)) return {};
  return bytes.data();
}

std::string stringProperty(AudioObjectID object, AudioObjectPropertyAddress property) {
  CFStringRef value = nullptr;
  UInt32 size = sizeof(value);
  if (AudioObjectGetPropertyData(object, &property, 0, nullptr, &size, &value) != noErr || !value)
    return {};
  const std::string result = cfString(value);
  CFRelease(value);
  return result;
}

bool inputChannelConfiguration(AudioDeviceID device, std::vector<uint32_t>& channels) {
  AudioObjectPropertyAddress property = address(kAudioDevicePropertyStreamConfiguration,
                                                kAudioDevicePropertyScopeInput);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(device, &property, 0, nullptr, &size) != noErr ||
      size < offsetof(AudioBufferList, mBuffers) + sizeof(AudioBuffer) ||
      size > offsetof(AudioBufferList, mBuffers) + 4096 * sizeof(AudioBuffer)) {
    return false;
  }
  std::vector<uint8_t> storage(size);
  AudioBufferList* list = reinterpret_cast<AudioBufferList*>(storage.data());
  const UInt32 capacity = size;
  if (AudioObjectGetPropertyData(device, &property, 0, nullptr, &size, list) != noErr ||
      size > capacity || size < offsetof(AudioBufferList, mBuffers) ||
      list->mNumberBuffers > (size - offsetof(AudioBufferList, mBuffers)) / sizeof(AudioBuffer))
    return false;
  uint64_t total = 0;
  std::vector<uint32_t> observed;
  for (UInt32 i = 0; i < list->mNumberBuffers; ++i) {
    total += list->mBuffers[i].mNumberChannels;
    observed.push_back(list->mBuffers[i].mNumberChannels);
  }
  // A bounded public/device protocol and a bounded channel-label loop. No
  // real interface approaches this; reject corrupt driver metadata instead
  // of allocating from it.
  if (total > 4096) return false;
  channels = std::move(observed);
  return true;
}

uint32_t inputChannelCount(AudioDeviceID device) {
  std::vector<uint32_t> channels;
  if (!inputChannelConfiguration(device, channels)) return 0;
  uint32_t total = 0;
  for (const auto count : channels) total += count;
  return total;
}

bool inputStreamConfiguration(AudioDeviceID device,
                              detail::MacAudioInputStreamConfiguration& facts) {
  facts = {};
  facts.identity = device;
  if (!getProperty(device, address(kAudioDevicePropertyNominalSampleRate,
                                   kAudioObjectPropertyScopeGlobal), facts.rate) ||
      !std::isfinite(facts.rate) || facts.rate <= 0 ||
      !inputChannelConfiguration(device, facts.inputChannels)) return false;
  auto property = address(kAudioDevicePropertyStreams, kAudioDevicePropertyScopeInput);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(device, &property, 0, nullptr, &size) != noErr ||
      !size || size % sizeof(AudioStreamID) || size > 4096 * sizeof(AudioStreamID)) return false;
  std::vector<AudioStreamID> streams(size / sizeof(AudioStreamID));
  const UInt32 capacity = size;
  if (AudioObjectGetPropertyData(device, &property, 0, nullptr, &size, streams.data()) != noErr ||
      size != capacity) return false;
  const auto formatFacts = [](const AudioStreamBasicDescription& f) {
    return detail::MacAudioHostStreamFormat{f.mSampleRate, f.mFormatID, f.mFormatFlags,
        f.mBytesPerPacket, f.mFramesPerPacket, f.mBytesPerFrame, f.mChannelsPerFrame, f.mBitsPerChannel};
  };
  for (const auto stream : streams) {
    AudioStreamBasicDescription physical{}, virtualFormat{};
    if (!getProperty(stream, address(kAudioStreamPropertyPhysicalFormat,
                                      kAudioObjectPropertyScopeGlobal), physical) ||
        !getProperty(stream, address(kAudioStreamPropertyVirtualFormat,
                                      kAudioObjectPropertyScopeGlobal), virtualFormat)) return false;
    facts.inputStreams.push_back({stream, formatFacts(physical), formatFacts(virtualFormat)});
  }
  return true;
}

std::vector<AudioDeviceID> allDeviceIds(std::string* error) {
  AudioObjectPropertyAddress property = address(kAudioHardwarePropertyDevices,
                                                kAudioObjectPropertyScopeGlobal);
  UInt32 size = 0;
  if (AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &property, 0, nullptr, &size) != noErr ||
      size % sizeof(AudioDeviceID) != 0 || size > sizeof(AudioDeviceID) * 4096u) {
    if (error) *error = "CoreAudio could not enumerate devices";
    return {};
  }
  std::vector<AudioDeviceID> devices(size / sizeof(AudioDeviceID));
  if (size && AudioObjectGetPropertyData(kAudioObjectSystemObject, &property, 0, nullptr, &size,
                                         devices.data()) != noErr) {
    if (error) *error = "CoreAudio could not read devices";
    return {};
  }
  return devices;
}

std::string osStatusMessage(const char* operation, OSStatus status) {
  char fourcc[5] = {};
  const uint32_t code = static_cast<uint32_t>(status);
  fourcc[0] = static_cast<char>((code >> 24) & 0xff);
  fourcc[1] = static_cast<char>((code >> 16) & 0xff);
  fourcc[2] = static_cast<char>((code >> 8) & 0xff);
  fourcc[3] = static_cast<char>(code & 0xff);
  bool printable = true;
  for (int i = 0; i < 4; ++i) printable = printable && fourcc[i] >= 32 && fourcc[i] <= 126;
  return std::string(operation) + " failed (" +
         (printable ? std::string("'") + fourcc + "'" : std::to_string(status)) + ")";
}

class MacAudioInputBackend final : public AudioInputBackend {
 public:
  ~MacAudioInputBackend() override { stop(); }

  void setPositionPush(AudioInputPositionPush push) override { positionPush_ = push; }

  AudioInputResult open(const AudioInputConfig& config, AudioInputPush push,
                        void* context) override {
    stop();
    if (config.requestedBufferFrames != 0 && !config.allowSharedDeviceBufferChange)
      return AudioInputResult::failure(AudioInputState::Unsupported,
          "Shared CoreAudio input buffer changes require allowSharedDeviceBufferChange",
          config.channel);
    bufferWarnings_ = AudioBufferWarningNone;
    push_ = push;
    context_ = context;

    AudioDeviceID selected = kAudioObjectUnknown;
    CFStringRef uid = CFStringCreateWithCString(nullptr, config.deviceUid.c_str(),
                                               kCFStringEncodingUTF8);
    if (uid) {
      UInt32 size = sizeof(selected);
      const auto property = address(kAudioHardwarePropertyTranslateUIDToDevice,
                                    kAudioObjectPropertyScopeGlobal);
      if (AudioObjectGetPropertyData(kAudioObjectSystemObject, &property, sizeof(uid),
          &uid, &size, &selected) != noErr) selected = kAudioObjectUnknown;
      CFRelease(uid);
    }
    if (selected == kAudioObjectUnknown)
      return AudioInputResult::failure(
          AudioInputState::Error,
          "audio input device disappeared",
          config.channel);
    std::vector<uint32_t> channelLayout;
    if (!inputChannelConfiguration(selected, channelLayout))
      return AudioInputResult::failure(
          AudioInputState::Error, "audio input stream configuration is unavailable", config.channel);
    uint32_t channels = 0;
    for (const auto count : channelLayout) channels += count;
    if (config.channel >= channels)
      return AudioInputResult::failure(
          AudioInputState::Error, "audio input channel disappeared", config.channel);
    UInt32 alive = 0;
    if (!getProperty(selected,
                     address(kAudioDevicePropertyDeviceIsAlive,
                             kAudioObjectPropertyScopeGlobal),
                     alive) || !alive)
      return AudioInputResult::failure(
          AudioInputState::Error, "audio input device is not alive", config.channel);
    selectedDevice_ = selected;
    selectedDeviceUid_ = config.deviceUid;
    channel_ = config.channel;

    Float64 nominalRate = 0;
    if (!getProperty(selected,
                     address(kAudioDevicePropertyNominalSampleRate,
                             kAudioObjectPropertyScopeGlobal),
                    nominalRate) || !std::isfinite(nominalRate) || nominalRate <= 0) {
      return AudioInputResult::failure(
          AudioInputState::Error, "audio input sample rate is unavailable", config.channel);
    }

    AudioComponentDescription description{};
    description.componentType = kAudioUnitType_Output;
    description.componentSubType = kAudioUnitSubType_HALOutput;
    description.componentManufacturer = kAudioUnitManufacturer_Apple;
    const AudioComponent component = AudioComponentFindNext(nullptr, &description);
    if (!component)
      return AudioInputResult::failure(
          AudioInputState::Error, "CoreAudio HAL output unit is unavailable", config.channel);
    OSStatus status = AudioComponentInstanceNew(component, &unit_);
    if (status != noErr) return fail("AudioComponentInstanceNew", status, config.channel);

    UInt32 enabled = 1;
    status = AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Input,
                                  1, &enabled, sizeof(enabled));
    if (status != noErr) return fail("enable AUHAL input", status, config.channel);
    UInt32 disabled = 0;
    status = AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_EnableIO, kAudioUnitScope_Output,
                                  0, &disabled, sizeof(disabled));
    if (status != noErr) return fail("disable AUHAL output", status, config.channel);
    status = AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_CurrentDevice,
                                  kAudioUnitScope_Global, 0, &selected, sizeof(selected));
    if (status != noErr) return fail("select AUHAL device", status, config.channel);


    AudioStreamBasicDescription format{};
    format.mSampleRate = nominalRate;
    format.mFormatID = kAudioFormatLinearPCM;
    format.mFormatFlags = kAudioFormatFlagIsFloat | kAudioFormatFlagIsPacked |
                          kAudioFormatFlagIsNonInterleaved | kAudioFormatFlagsNativeEndian;
    format.mBytesPerPacket = sizeof(float);
    format.mFramesPerPacket = 1;
    format.mBytesPerFrame = sizeof(float);
    format.mChannelsPerFrame = 1;
    format.mBitsPerChannel = 32;
    status = AudioUnitSetProperty(unit_, kAudioUnitProperty_StreamFormat,
                                  kAudioUnitScope_Output, 1, &format, sizeof(format));
    if (status != noErr) return fail("set AUHAL client format", status, config.channel);

    SInt32 mappedChannel = 0;
    std::string mapError;
    if (!makeAudioInputChannelMap(config.channel, channels, mappedChannel, mapError))
      return failMessage(std::move(mapError), config.channel);
    const SInt32 channelMap[1] = {mappedChannel};
    status = AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_ChannelMap,
                                  kAudioUnitScope_Output, 1, channelMap, sizeof(channelMap));
    if (status != noErr) return fail("set AUHAL input channel map", status, config.channel);

    UInt32 maxFrames = 0;
    UInt32 propertySize = sizeof(maxFrames);
    status = AudioUnitGetProperty(unit_, kAudioUnitProperty_MaximumFramesPerSlice,
                                  kAudioUnitScope_Global, 0, &maxFrames, &propertySize);
    const auto initialSlice = status == noErr && propertySize == sizeof(maxFrames) && maxFrames != 0
        ? std::optional<uint32_t>{maxFrames} : std::nullopt;
    const auto preparation = detail::macAudioInputPreparation(
        initialSlice);
    bufferWarnings_ |= preparation.warnings;
    maxFrames = preparation.frames;
    if (maxFrames == 0)
      return failMessage("audio device callback is larger than the core limit", config.channel);
    {
      detail::MacAudioBufferProperties properties{selectedDevice_, selectedDeviceUid_};
      // Ordinary capture may hold a capacity-only lease when optional buffer
      // metadata is missing. Retention keeps that reservation until real close.
      const auto buffer = detail::requestMacAudioBuffer(
          properties, config.requestedBufferFrames, maxFrames, bufferOwnership_,
          config.maximumAcceptableBufferFrames, detail::macAudioBufferLedger(), false,
          detail::MacAudioInputBufferCapacity{maxFrames, [&](uint32_t storage) {
            const auto capacity = detail::prepareMacAudioInputUnitSlice(storage, initialSlice,
                bufferWarnings_, [&](uint32_t frames) {
                  return AudioUnitSetProperty(unit_, kAudioUnitProperty_MaximumFramesPerSlice,
                      kAudioUnitScope_Global, 0, &frames, sizeof(frames)) == noErr;
                }, [&] { return unitSlice(); });
            // Ordinary capture reserves the old limit until post-init readback;
            // another owner cannot restore into an initialize-time limit change.
            return config.requestedBufferFrames == 0
                ? std::min(capacity, initialSlice.value_or(0)) : capacity;
          }});
      bufferWarnings_ |= buffer.warnings;
      if (!buffer.ok()) return failMessage(buffer.message, config.channel);
      if (buffer.frames == 0) bufferWarnings_ |= AudioBufferWarningReadFailed;
    }
    renderBuffer_.resize(maxFrames);
    mach_timebase_info_data_t timebase{};
    if (mach_timebase_info(&timebase) != KERN_SUCCESS || !timebase.denom)
      return failMessage("mach host-time conversion is unavailable", config.channel);
    nsPerHostTick_ = static_cast<double>(timebase.numer) / timebase.denom;

    AURenderCallbackStruct callback{renderCallback, this};
    status = AudioUnitSetProperty(unit_, kAudioOutputUnitProperty_SetInputCallback,
                                  kAudioUnitScope_Global, 0, &callback, sizeof(callback));
    if (status != noErr) return fail("set AUHAL input callback", status, config.channel);
    status = AudioUnitInitialize(unit_);
    if (status != noErr) return fail("initialize AUHAL", status, config.channel);
    initialized_ = true;
    const auto confirmedSlice = unitSlice();
    if (!confirmedSlice || *confirmedSlice < maxFrames) bufferWarnings_ |= AudioBufferWarningReadFailed;
    const auto confirmedCapacity = detail::macAudioInputConfirmedCapacity(maxFrames, confirmedSlice);
    if (!detail::updateMacAudioBufferCapacity(bufferOwnership_, confirmedCapacity))
      return failMessage("audio input buffer lease disappeared", config.channel);
    renderMaximumFrames_ = confirmedSlice ? confirmedCapacity : maxFrames;
    propertySize = sizeof(format);
    status = AudioUnitGetProperty(unit_, kAudioUnitProperty_StreamFormat,
                                  kAudioUnitScope_Output, 1, &format, &propertySize);
    if (status != noErr || propertySize != sizeof(format) ||
        !std::isfinite(format.mSampleRate) || format.mSampleRate <= 0)
      return failMessage("AUHAL negotiated sample rate is unavailable", config.channel);
    deviceSampleRate_ = nominalRate;
    sampleRate_ = format.mSampleRate;
    // Snapshot the configured/initialized unit's device facts, not the earlier
    // metadata used while choosing its channel map.
    if (!detail::readMacAudioInputConfiguration(openedConfiguration_, [&](auto& facts) {
          return inputStreamConfiguration(selectedDevice_, facts);
        }) || openedConfiguration_.inputChannels != channelLayout ||
        openedConfiguration_.rate != nominalRate)
      return failMessage("audio input stream configuration changed while opening", config.channel);
    listenerContext_ = std::make_shared<DeviceListenerContext>();
    listenerQueue_ = dispatch_queue_create("com.lexasoft.singz.audio-input-device", nullptr);
    if (!listenerQueue_)
      return failMessage("could not create audio device listener queue", config.channel);
    const auto listener = listenerContext_;
    listenerBlock_ = Block_copy(^(UInt32 count,
                                  const AudioObjectPropertyAddress properties[]) {
      uint32_t changes = 0;
      for (UInt32 i = 0; i < count; ++i) {
        changes |= detail::macInputNotificationChanges(properties[i].mSelector);
      }
      if (changes & detail::kMacInputLatencyChanged)
        listener->latencyRevision.fetch_add(1, std::memory_order_seq_cst);
      (void)detail::macInputControlChanges(changes, listener->latencyStable);
      if (changes) listener->pendingChanges.fetch_or(changes, std::memory_order_release);
    });
    const AudioObjectPropertyAddress watched[] = {
        address(kAudioDevicePropertyDeviceIsAlive, kAudioObjectPropertyScopeGlobal),
        address(kAudioDevicePropertyNominalSampleRate, kAudioObjectPropertyScopeGlobal),
        address(kAudioDevicePropertyStreams, kAudioDevicePropertyScopeInput),
        address(kAudioDevicePropertyStreamConfiguration, kAudioDevicePropertyScopeInput)};
    for (const AudioObjectPropertyAddress& property : watched) {
      status = AudioObjectAddPropertyListenerBlock(selectedDevice_, &property,
                                                   listenerQueue_, listenerBlock_);
      if (status != noErr) return fail("watch AUHAL device", status, config.channel);
      watchedProperties_.push_back({selectedDevice_, property});
    }
    selectedStream_ = inputStream(selectedDevice_, channel_);
    // These are optional driver properties. Missing queries/listeners disable
    // reported compensation, but must not make an otherwise usable mic fail.
    const AudioObjectPropertyAddress latencyProperties[] = {
        address(kAudioDevicePropertyLatency, kAudioDevicePropertyScopeInput),
        address(kAudioDevicePropertySafetyOffset, kAudioDevicePropertyScopeInput),
        address(kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal)};
    latencyWatchable_ = true;
    for (const auto& property : latencyProperties) {
      if (AudioObjectAddPropertyListenerBlock(selectedDevice_, &property,
                                               listenerQueue_, listenerBlock_) == noErr)
        watchedProperties_.push_back({selectedDevice_, property});
      else latencyWatchable_ = false;
    }
    for (const auto& stream : openedConfiguration_.inputStreams) {
      for (const auto selector : {kAudioStreamPropertyVirtualFormat, kAudioStreamPropertyPhysicalFormat}) {
        const auto property = address(selector, kAudioObjectPropertyScopeGlobal);
        status = AudioObjectAddPropertyListenerBlock(stream.identity, &property, listenerQueue_, listenerBlock_);
        if (status != noErr) return fail("watch AUHAL input stream format", status, config.channel);
        watchedProperties_.push_back({stream.identity, property});
      }
    }
    const auto streamLatency = address(kAudioStreamPropertyLatency, kAudioObjectPropertyScopeGlobal);
    if (selectedStream_ != kAudioObjectUnknown && AudioObjectAddPropertyListenerBlock(
        selectedStream_, &streamLatency, listenerQueue_, listenerBlock_) == noErr)
      watchedProperties_.push_back({selectedStream_, streamLatency});
    else latencyWatchable_ = false;
    // Fence configuration changes between channel-map selection and listener
    // installation; redundant buffer-driven notifications remain harmless.
    if (streamConfigurationChanged())
      return failMessage("audio input stream configuration changed while opening", config.channel);
    callbackFailure_.store(0, std::memory_order_relaxed);
    return resultWithLatency(AudioInputState::Starting);
  }

  AudioInputResult start() override {
    if (!unit_ || !initialized_)
      return AudioInputResult::failure(
          AudioInputState::Error, "AUHAL is not prepared", channel_);
    // Also fences retained-prepared suspension, even when no listener bit was
    // delivered. A new map/format requires reopening rather than rebasing it.
    if (streamConfigurationChanged())
      return failMessage("audio input stream configuration changed; restart capture", channel_);
    renderRecovery_ = {};
    renderSliceDrops_.store(0, std::memory_order_relaxed);
    admission_.open();
    const OSStatus status = AudioOutputUnitStart(unit_);
    if (status != noErr) {
      admission_.beginClose();
      return fail("start AUHAL", status, channel_);
    }
    started_ = true;
    return resultWithLatency(AudioInputState::Running);
  }

  bool suspendPrepared() override {
    // Close admission before the stop inside the transaction: a callback that
    // has not yet counted itself is refused instead of slipping past the
    // drain, whatever AudioOutputUnitStop returns.
    admission_.beginClose();
    const bool suspended = detail::suspendMacAudioInputPrepared(unit_ && initialized_, started_, [this] {
      return AudioOutputUnitStop(unit_) == noErr;
    });
    // The suspended unit's last callback must finish before control returns:
    // the caller then replaces the ring/endpoint that callback wrote into.
    while (admission_.inFlight() != 0) std::this_thread::yield();
    return suspended;
  }

  void stop() noexcept override {
    bool removedAll = true;
    // Mirror the host backend: a failed dispatch_queue_create/Block_copy in
    // open() leaves a null queue/block with no registered listeners, so
    // dispatch_sync_f/dispatch_release/Block_release must all be skipped.
    if (selectedDevice_ != kAudioObjectUnknown && listenerContext_ &&
        listenerQueue_ != nullptr && listenerBlock_ != nullptr) {
      for (const auto& [object, property] : watchedProperties_) {
        if (AudioObjectRemovePropertyListenerBlock(object, &property,
                                                   listenerQueue_, listenerBlock_) != noErr)
          removedAll = false;
      }
      // A successful remove prevents new submissions. A synchronous no-op on
      // the dedicated serial queue is the execution barrier for callbacks
      // submitted before removal returned, closing the listener entry race.
      if (removedAll) {
        dispatch_sync_f(listenerQueue_, nullptr, [](void*) {});
        Block_release(listenerBlock_);
        dispatch_release(listenerQueue_);
      }
      // On failed removal the retained block pins its captured shared context;
      // deliberately retain our block/queue references, without a heap push.
    } else if (listenerQueue_ != nullptr) {
      // A queue that never received its block has nothing registered to it;
      // only the queue reference remains, so release it (host backend parity).
      dispatch_release(listenerQueue_);
    }
    watchedProperties_.clear();
    listenerContext_.reset();
    listenerQueue_ = nullptr;
    listenerBlock_ = nullptr;
    if (unit_) {
      // Close admission before the stop: a callback that has not yet counted
      // itself is refused instead of slipping past the drain, whatever the
      // stop returns or whether the unit was ever started. Any callback that
      // did enter must finish before the unit is disposed or the members it
      // reads are cleared.
      admission_.beginClose();
      if (started_) AudioOutputUnitStop(unit_);
      while (admission_.inFlight() != 0) std::this_thread::yield();
      if (initialized_) AudioUnitUninitialize(unit_);
      AudioComponentInstanceDispose(unit_);
    }
    detail::MacAudioBufferProperties properties{selectedDevice_, selectedDeviceUid_};
    if (bufferOwnership_.token != 0) {
      try {
        bufferWarnings_ |= detail::restoreMacAudioBuffer(properties, bufferOwnership_);
      } catch (...) {
        bufferWarnings_ |= AudioBufferWarningRestoreFailed;
      }
    }
    unit_ = nullptr;
    started_ = false;
    initialized_ = false;
    renderBuffer_.clear();
    push_ = nullptr;
    positionPush_ = nullptr;
    context_ = nullptr;
    if (bufferOwnership_.token == 0) selectedDevice_ = kAudioObjectUnknown;
    selectedStream_ = kAudioObjectUnknown;
    latencyWatchable_ = false;
  }

  uint32_t bufferWarnings() const noexcept override { return bufferWarnings_; }
  uint64_t renderSliceDrops() const noexcept override {
    return renderSliceDrops_.load(std::memory_order_relaxed);
  }

  bool takeFailure(std::string& error) override {
    const uint32_t changes = listenerContext_
        ? listenerContext_->pendingChanges.exchange(0, std::memory_order_acq_rel)
        : 0;
    const auto failure = listenerContext_ ? detail::macInputTakeControlFailure(
        changes, listenerContext_->latencyStable, [&] {
      UInt32 alive = 0;
      return getProperty(selectedDevice_,
                       address(kAudioDevicePropertyDeviceIsAlive,
                               kAudioObjectPropertyScopeGlobal),
                       alive) && alive != 0;
    }, [&] {
      Float64 nominalRate = 0;
      return getProperty(selectedDevice_,
                       address(kAudioDevicePropertyNominalSampleRate,
                               kAudioObjectPropertyScopeGlobal),
                       nominalRate) && std::isfinite(nominalRate) && nominalRate > 0 &&
          nominalRate == deviceSampleRate_;
    }, [&] { return !streamConfigurationChanged(); }) : detail::MacInputControlFailure::None;
    switch (failure) {
      case detail::MacInputControlFailure::DeviceLost:
        error = "audio input device disconnected";
        return true;
      case detail::MacInputControlFailure::RateChanged:
        error = "audio input sample rate changed; restart capture";
        return true;
      case detail::MacInputControlFailure::ConfigurationChanged:
        error = "audio input stream configuration changed; restart capture";
        return true;
      case detail::MacInputControlFailure::None: break;
    }
    if (latencyBasis_.reported && listenerContext_ && (changes & detail::kMacInputLatencyChanged))
      detail::refreshMacInputLatency(latencyBasis_, listenerContext_->latencyStable,
          listenerContext_->latencyRevision, [&] { return readLatency(); });
    // Buffer/latency notifications are advisory; stream selectors inspect
    // facts and only a real difference (or persistent unreadability) fails.
    const int32_t status = callbackFailure_.exchange(0, std::memory_order_acq_rel);
    if (!status) return false;
    error = status == kCallbackTooLarge
                ? "audio input callback exceeded the prepared buffer"
                : osStatusMessage("AUHAL render callback", static_cast<OSStatus>(status));
    return true;
  }

 private:
  std::optional<uint32_t> unitSlice() const {
    UInt32 frames = 0, size = sizeof(frames);
    if (AudioUnitGetProperty(unit_, kAudioUnitProperty_MaximumFramesPerSlice,
        kAudioUnitScope_Global, 0, &frames, &size) != noErr || size != sizeof(frames) || !frames)
      return std::nullopt;
    return frames;
  }

  bool streamConfigurationChanged() {
    return !detail::refreshMacAudioInputConfiguration(openedConfiguration_, [&](auto& facts) {
      return inputStreamConfiguration(selectedDevice_, facts);
    });
  }

  detail::MacInputLatency readLatency() const {
    Float64 deviceRate = 0;
    AudioStreamBasicDescription physicalFormat{};
    if (!latencyWatchable_ ||
        !getProperty(selectedDevice_, address(kAudioDevicePropertyNominalSampleRate,
                                               kAudioObjectPropertyScopeGlobal), deviceRate) ||
        !getProperty(selectedStream_, address(kAudioStreamPropertyPhysicalFormat,
                                               kAudioObjectPropertyScopeGlobal), physicalFormat))
      return {};
    return detail::macInputLatency(
        frameProperty(selectedDevice_, kAudioDevicePropertyLatency, kAudioDevicePropertyScopeInput),
        frameProperty(selectedDevice_, kAudioDevicePropertySafetyOffset, kAudioDevicePropertyScopeInput),
        frameProperty(selectedStream_, kAudioStreamPropertyLatency, kAudioObjectPropertyScopeGlobal),
        frameProperty(selectedDevice_, kAudioDevicePropertyBufferFrameSize, kAudioObjectPropertyScopeGlobal),
        deviceRate, sampleRate_, physicalFormat.mSampleRate);
  }

  AudioInputResult resultWithLatency(AudioInputState state) {
    auto result = AudioInputResult::success(state, sampleRate_, channel_);
    const auto buffer = frameProperty(selectedDevice_, kAudioDevicePropertyBufferFrameSize,
                                      kAudioObjectPropertyScopeGlobal);
    // Buffer negotiation is useful even when optional latency properties or
    // their listeners are unavailable. Preserve capture-rate frame units.
    result.inputBufferFrames = detail::macInputLatency(
        0, 0, 0, buffer, deviceSampleRate_, sampleRate_, deviceSampleRate_).bufferFrames;
    result.inputLatencyStable = std::shared_ptr<const std::atomic<bool>>(
        listenerContext_, &listenerContext_->latencyStable);
    const auto latency = readLatency();
    latencyBasis_ = latency;
    result.inputLatencyFrames = latency.frames;
    if (latency.reported) result.inputBufferFrames = latency.bufferFrames;
    result.inputLatencySource = latency.reported ? "reported" : "unknown";
    return result;
  }

  static constexpr int32_t kCallbackTooLarge = std::numeric_limits<int32_t>::min();

  static OSStatus renderCallback(void* context, AudioUnitRenderActionFlags* flags,
                                 const AudioTimeStamp* timestamp, UInt32,
                                 UInt32 frames, AudioBufferList*) {
    const uint64_t callbackTicks = mach_absolute_time();
    MacAudioInputBackend* self = static_cast<MacAudioInputBackend*>(context);
    if (!self) return noErr;
    // Admit-and-count the whole callback before touching any member: stop()
    // and suspendPrepared() close admission and drain before disposing the
    // unit or clearing unit_/renderBuffer_/push_/context_ read below. A
    // refused entry means teardown is under way; drop the block.
    AudioInputCallbackScope admission(self->admission_);
    if (!admission) return noErr;
    if (!self->unit_ || !self->push_ || frames == 0)
      return noErr;
    if (detail::macAudioInputCallbackTooLarge(frames, self->renderMaximumFrames_)) {
      self->callbackFailure_.store(kCallbackTooLarge, std::memory_order_release);
      return noErr;
    }
    AudioBufferList list{};
    list.mNumberBuffers = 1;
    list.mBuffers[0].mNumberChannels = 1;
    list.mBuffers[0].mDataByteSize = frames * sizeof(float);
    list.mBuffers[0].mData = self->renderBuffer_.data();
    const OSStatus status = AudioUnitRender(self->unit_, flags, timestamp, 1, frames, &list);
    const uint64_t callbackNs = static_cast<uint64_t>(
        static_cast<long double>(callbackTicks) *
        static_cast<long double>(self->nsPerHostTick_));
    const auto disposition = detail::observeMacInputRender(
        status, callbackNs, self->renderRecovery_, self->renderSliceDrops_);
    if (disposition == detail::MacInputRenderDisposition::Fatal) {
      // Other errors still fail immediately; persistent slice errors use the
      // identical status publication/return path after the bounded grace.
      self->callbackFailure_.store(static_cast<int32_t>(status), std::memory_order_release);
      return status;
    }
    // Discard failed signal, not elapsed time: substitute bounded silence and
    // use this callback's normal timestamps/push so take alignment survives.
    // No unit property query/set or lifecycle repair is allowed here.
    if (!detail::prepareMacInputRenderBlock(disposition, self->renderBuffer_.data(), frames))
      return noErr;
    const bool hardwareValid =
        timestamp && (timestamp->mFlags & kAudioTimeStampHostTimeValid) &&
        timestamp->mHostTime != 0;
    const uint64_t hardwareNs = hardwareValid
        ? static_cast<uint64_t>(static_cast<long double>(timestamp->mHostTime) *
                                static_cast<long double>(self->nsPerHostTick_))
        : 0;
    const AudioInputTimestampProjection projected = resolveAudioInputTimestamp(
        hardwareValid, hardwareNs, callbackNs, frames, self->sampleRate_);
    if (self->positionPush_) {
      self->positionPush_(self->context_, self->renderBuffer_.data(), frames,
                          projected.sampleHostTimeNs, callbackNs,
                          projected.usedHardwareAnchor
                              ? AudioInputTimestampQuality::Hardware
                              : AudioInputTimestampQuality::CallbackEstimate,
                          resolveAudioInputSourcePosition(
                              timestamp && (timestamp->mFlags & kAudioTimeStampSampleTimeValid),
                              timestamp ? timestamp->mSampleTime : 0));
    } else {
      self->push_(self->context_, self->renderBuffer_.data(), frames,
                  projected.sampleHostTimeNs, callbackNs,
                  projected.usedHardwareAnchor
                      ? AudioInputTimestampQuality::Hardware
                      : AudioInputTimestampQuality::CallbackEstimate);
    }
    return noErr;
  }

  AudioInputResult fail(const char* operation, OSStatus status, uint32_t channel) {
    return failMessage(osStatusMessage(operation, status), channel);
  }

  AudioInputResult failMessage(std::string message, uint32_t channel) {
    stop();
    return AudioInputResult::failure(AudioInputState::Error, std::move(message), channel);
  }

  AudioUnit unit_ = nullptr;
  bool initialized_ = false;
  bool started_ = false;
  AudioDeviceID selectedDevice_ = kAudioObjectUnknown;
  AudioStreamID selectedStream_ = kAudioObjectUnknown;
  bool latencyWatchable_ = false;
  detail::MacInputLatency latencyBasis_;
  detail::MacAudioBufferOwnership bufferOwnership_;
  uint32_t bufferWarnings_ = AudioBufferWarningNone;
  std::string selectedDeviceUid_;
  double sampleRate_ = 0;
  double deviceSampleRate_ = 0;
  uint32_t channel_ = 0;
  detail::MacAudioInputStreamConfiguration openedConfiguration_;
  double nsPerHostTick_ = 0;
  std::vector<float> renderBuffer_;
  uint32_t renderMaximumFrames_ = 0;
  AudioInputPush push_ = nullptr;
  AudioInputPositionPush positionPush_ = nullptr;
  void* context_ = nullptr;
  std::atomic<int32_t> callbackFailure_{0};
  detail::MacInputRenderRecovery renderRecovery_;
  std::atomic<uint64_t> renderSliceDrops_{0};
  AudioInputCallbackGate admission_{};
  std::shared_ptr<DeviceListenerContext> listenerContext_;
  dispatch_queue_t listenerQueue_ = nullptr;
  AudioObjectPropertyListenerBlock listenerBlock_ = nullptr;
  std::vector<std::pair<AudioObjectID, AudioObjectPropertyAddress>> watchedProperties_;
};

}  // namespace

std::vector<AudioInputDevice> enumeratePlatformAudioInputDevices(std::string* error) {
  if (error) error->clear();
  AudioDeviceID defaultInput = kAudioObjectUnknown;
  getProperty(kAudioObjectSystemObject,
              address(kAudioHardwarePropertyDefaultInputDevice,
                      kAudioObjectPropertyScopeGlobal),
              defaultInput);
  std::vector<AudioInputDevice> result;
  for (const AudioDeviceID device : allDeviceIds(error)) {
    const uint32_t channels = inputChannelCount(device);
    if (channels == 0) continue;
    AudioInputDevice item;
    item.uid = stringProperty(
        device, address(kAudioDevicePropertyDeviceUID, kAudioObjectPropertyScopeGlobal));
    item.label = stringProperty(
        device, address(kAudioObjectPropertyName, kAudioObjectPropertyScopeGlobal));
    Float64 sampleRate = 0;
    getProperty(device,
                address(kAudioDevicePropertyNominalSampleRate,
                        kAudioObjectPropertyScopeGlobal),
                sampleRate);
    item.sampleRate = sampleRate;
    item.channels = channels;
    item.isDefault = device == defaultInput;
    if (item.uid.empty()) continue;
    if (item.label.empty()) item.label = "Audio input";
    item.channelLabels.reserve(channels);
    for (uint32_t channel = 0; channel < channels; ++channel) {
      const auto element = static_cast<AudioObjectPropertyElement>(channel + 1);
      std::string label = stringProperty(
          device, address(kAudioObjectPropertyElementName,
                          kAudioDevicePropertyScopeInput, element));
      if (label.empty()) {
        const std::string category = stringProperty(
            device, address(kAudioObjectPropertyElementCategoryName,
                            kAudioDevicePropertyScopeInput, element));
        const std::string number = stringProperty(
            device, address(kAudioObjectPropertyElementNumberName,
                            kAudioDevicePropertyScopeInput, element));
        if (!category.empty())
          label = category + (number.empty() ? " " + std::to_string(channel + 1)
                                             : " " + number);
        else
          label = number;
      }
      if (label.empty()) label = "Channel " + std::to_string(channel + 1);
      item.channelLabels.push_back(std::move(label));
    }
    result.push_back(std::move(item));
  }
  return result;
}

std::unique_ptr<AudioInputBackend> createPlatformAudioInputBackend() {
  return std::make_unique<MacAudioInputBackend>();
}

}  // namespace singz

#endif  // __APPLE__ && TARGET_OS_OSX
