#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include <zcore/device/audio_host.h>

namespace singz::detail {

inline constexpr uint32_t kMacAudioHostAliveChanged = 1u << 0;
inline constexpr uint32_t kMacAudioHostFormatChanged = 1u << 1;
inline constexpr uint32_t kMacAudioHostBufferChanged = 1u << 2;

// Immutable opening facts, queried only on the ordinary listener/control queue.
// Stream order/identity, both physical and virtual formats, and per-buffer
// channel counts matter; the shared device buffer period deliberately does not.
struct MacAudioHostStreamFormat {
  double rate = 0;
  uint32_t id = 0, flags = 0, bytesPerPacket = 0, framesPerPacket = 0;
  uint32_t bytesPerFrame = 0, channels = 0, bits = 0;
  bool operator==(const MacAudioHostStreamFormat&) const = default;
};
struct MacAudioHostStreamFacts {
  uint32_t identity = 0;
  MacAudioHostStreamFormat physical, virtualFormat;
  bool operator==(const MacAudioHostStreamFacts&) const = default;
};
struct MacAudioHostDeviceFacts {
  uint32_t identity = 0;
  double rate = 0;
  std::vector<uint32_t> inputChannels, outputChannels;
  std::vector<MacAudioHostStreamFacts> inputStreams, outputStreams;
  bool operator==(const MacAudioHostDeviceFacts&) const = default;
};

constexpr bool macAudioHostUsesScope(bool hasInput, bool inputScope) noexcept {
  return !inputScope || hasInput;
}

// Notification selectors only decide whether to query. Latch solely on a real
// difference or an unreadable/removed device, including an AudioObjectID fence.
inline uint32_t macAudioHostControlChange(
    const MacAudioHostDeviceFacts& opened,
    const MacAudioHostDeviceFacts& observed, bool alive, bool readable, bool hasInput = true) noexcept {
  if (!alive || opened.identity != observed.identity) return kMacAudioHostAliveChanged;
  const bool same = opened.rate == observed.rate &&
      opened.outputChannels == observed.outputChannels && opened.outputStreams == observed.outputStreams &&
      (!macAudioHostUsesScope(hasInput, true) ||
       (opened.inputChannels == observed.inputChannels && opened.inputStreams == observed.inputStreams));
  return !readable || !same ? kMacAudioHostFormatChanged : 0u;
}

enum class MacAudioHostIdentity { Present, Absent, Unknown };
struct MacAudioHostRouteObservation {
  MacAudioHostDeviceFacts facts;
  MacAudioHostIdentity identity = MacAudioHostIdentity::Unknown;
  bool readable = false;
  bool retry = false;
};
struct MacAudioHostRouteCheck {
  uint32_t changes = 0;
  uint32_t warnings = AudioBufferWarningNone;
};

// Two immediate attempts at most, never a sleep or a callback operation.
// Unknown identity is not evidence of removal. Still compare readable stream
// facts, and fail closed for persistent format-read failures.
template <class Read>
MacAudioHostRouteCheck checkMacAudioHostRoute(
    const MacAudioHostDeviceFacts& opened, bool hasInput, Read&& read) {
  auto observed = read();
  if (observed.identity != MacAudioHostIdentity::Absent &&
      (observed.retry || !observed.readable || observed.identity == MacAudioHostIdentity::Unknown))
    observed = read();
  if (observed.identity == MacAudioHostIdentity::Absent)
    return {kMacAudioHostAliveChanged, AudioBufferWarningNone};
  // Partial format reads carry no new object identity. An unknown lookup or
  // an unfilled facts object must not be mistaken for a confirmed replacement.
  if (!observed.readable || observed.identity == MacAudioHostIdentity::Unknown)
    observed.facts.identity = opened.identity;
  return {macAudioHostControlChange(opened, observed.facts, true, observed.readable, hasInput),
          observed.identity == MacAudioHostIdentity::Unknown || observed.retry
              ? AudioBufferWarningReadFailed : AudioBufferWarningNone};
}

template <class Read>
bool readMacAudioHostFactsWithRetry(Read&& read) {
  return read() || read();
}

// A destroyed stream cannot retain its registration. Genuine removal failures
// on live objects must still retain the copied block/queue for safety.
constexpr bool macAudioHostListenerRemoved(bool removed, bool streamObject, bool objectGone) noexcept {
  return removed || (streamObject && objectGone);
}

// Buffer-only notifications must neither query HAL configuration nor latch a
// route change. Keep the selector dispatch at the same tested control seam.
template <class Inspect>
uint32_t macAudioHostControlNotification(bool configurationNotification,
                                        Inspect&& inspect) {
  return configurationNotification ? inspect() : 0u;
}

constexpr bool macAudioHostBufferExceedsMaximum(uint32_t frames, uint32_t maximum) noexcept {
  return frames > maximum;
}

// The AUHAL render callback uses this exact predicate before its only
// AudioUnitRender call. Keeping it trivial and testable makes the output-only
// no-input-pull invariant explicit without introducing a callback indirection.
constexpr bool shouldPullMacAudioHostInput(uint32_t inputChannels) noexcept {
  return inputChannels != 0;
}

constexpr bool macAudioHostInputTimestampValid(
    uint32_t inputChannels) noexcept {
  return shouldPullMacAudioHostInput(inputChannels);
}

constexpr bool macAudioHostInputTimestampHardware(
    uint32_t inputChannels, bool hardwareAnchorUsed) noexcept {
  return shouldPullMacAudioHostInput(inputChannels) && hardwareAnchorUsed;
}

// AudioUnitProperties.h defines kAudioOutputUnitProperty_ChannelMap as one
// source-channel index per destination channel, with -1 meaning silence. AUHAL
// output's destination is the physical device, so the map is device-sized and
// each selected physical lane points back to its client/source lane.
inline bool buildMacAudioHostOutputChannelMap(
    const std::vector<uint32_t>& selectedPhysicalChannels,
    uint32_t physicalDestinationChannels,
    std::vector<int32_t>* destinationMap) {
  if (destinationMap == nullptr) return false;
  destinationMap->clear();
  if (physicalDestinationChannels == 0 ||
      physicalDestinationChannels > kAudioHostMaxChannels ||
      selectedPhysicalChannels.empty() ||
      selectedPhysicalChannels.size() > kAudioHostMaxChannels) {
    return false;
  }
  std::vector<int32_t> candidate(physicalDestinationChannels, -1);
  for (std::size_t source = 0; source < selectedPhysicalChannels.size();
       ++source) {
    const uint32_t destination = selectedPhysicalChannels[source];
    if (destination >= physicalDestinationChannels ||
        candidate[destination] != -1) {
      return false;
    }
    candidate[destination] = static_cast<int32_t>(source);
  }
  *destinationMap = std::move(candidate);
  return true;
}

constexpr uint32_t audioHostFourCc(char a, char b, char c, char d) noexcept {
  return (static_cast<uint32_t>(static_cast<uint8_t>(a)) << 24u) |
         (static_cast<uint32_t>(static_cast<uint8_t>(b)) << 16u) |
         (static_cast<uint32_t>(static_cast<uint8_t>(c)) << 8u) |
         static_cast<uint32_t>(static_cast<uint8_t>(d));
}

struct MacAudioHostTransportCapability {
  AudioHostTransport transport{AudioHostTransport::Unknown};
  AudioHostMonitoringSuitability monitoringSuitability{
      AudioHostMonitoringSuitability::Unknown};
};

inline MacAudioHostTransportCapability classifyMacAudioHostTransport(
    uint32_t value) noexcept {
  switch (value) {
    case audioHostFourCc('b', 'l', 't', 'n'):
      return {AudioHostTransport::BuiltIn,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('p', 'c', 'i', ' '):
      return {AudioHostTransport::Pci,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('u', 's', 'b', ' '):
      return {AudioHostTransport::Usb,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('1', '3', '9', '4'):
      return {AudioHostTransport::FireWire,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('t', 'h', 'u', 'n'):
      return {AudioHostTransport::Thunderbolt,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('c', 'c', 'w', 'd'):
      return {AudioHostTransport::ContinuityWired,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('h', 'd', 'm', 'i'):
      return {AudioHostTransport::Hdmi,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('d', 'p', 'r', 't'):
      return {AudioHostTransport::DisplayPort,
              AudioHostMonitoringSuitability::LowLatency};
    case audioHostFourCc('b', 'l', 'u', 'e'):
      return {AudioHostTransport::Bluetooth,
              AudioHostMonitoringSuitability::HighLatency};
    case audioHostFourCc('b', 'l', 'e', 'a'):
      return {AudioHostTransport::BluetoothLowEnergy,
              AudioHostMonitoringSuitability::HighLatency};
    case audioHostFourCc('a', 'i', 'r', 'p'):
      return {AudioHostTransport::AirPlay,
              AudioHostMonitoringSuitability::HighLatency};
    case audioHostFourCc('c', 'c', 'w', 'l'):
    case audioHostFourCc('c', 'c', 'a', 'p'):
      return {AudioHostTransport::ContinuityWireless,
              AudioHostMonitoringSuitability::HighLatency};
    case audioHostFourCc('g', 'r', 'u', 'p'):
    case audioHostFourCc('f', 'g', 'r', 'p'):
      return {AudioHostTransport::Aggregate,
              AudioHostMonitoringSuitability::Unknown};
    case audioHostFourCc('v', 'i', 'r', 't'):
      return {AudioHostTransport::Virtual,
              AudioHostMonitoringSuitability::Unknown};
    case audioHostFourCc('e', 'a', 'v', 'b'):
      return {AudioHostTransport::Avb,
              AudioHostMonitoringSuitability::Unknown};
    default:
      return {};
  }
}

inline uint32_t saturatedAudioHostLatency(uint32_t deviceFrames,
                                          uint32_t safetyFrames) noexcept {
  const uint64_t total = static_cast<uint64_t>(deviceFrames) + safetyFrames;
  return total > std::numeric_limits<uint32_t>::max()
             ? std::numeric_limits<uint32_t>::max()
             : static_cast<uint32_t>(total);
}

inline bool checkedAudioHostFrameCount(double value, uint32_t* result) noexcept {
  if (result == nullptr || !std::isfinite(value) || value < 0.0 ||
      value > static_cast<double>(std::numeric_limits<uint32_t>::max()) ||
      std::floor(value) != value) {
    return false;
  }
  *result = static_cast<uint32_t>(value);
  return true;
}

inline bool checkedAudioHostBufferRange(double minimum, double maximum,
                                        uint32_t* minimumFrames,
                                        uint32_t* maximumFrames) noexcept {
  if (minimumFrames == nullptr || maximumFrames == nullptr) return false;
  uint32_t checkedMinimum = 0;
  uint32_t checkedMaximum = 0;
  if (!checkedAudioHostFrameCount(minimum, &checkedMinimum) ||
      !checkedAudioHostFrameCount(maximum, &checkedMaximum) ||
      checkedMinimum > checkedMaximum) {
    return false;
  }
  *minimumFrames = checkedMinimum;
  *maximumFrames = checkedMaximum;
  return true;
}

}  // namespace singz::detail
