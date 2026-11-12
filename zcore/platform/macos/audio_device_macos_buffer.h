#pragma once

#include <algorithm>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

#include <zcore/device/audio_host.h>

namespace singz::detail {

struct MacAudioBufferOwnership {
  std::string uid;
  uint64_t token = 0;
};

// A single ledger is shared by all zCore input/output backends. Only ordinary
// control threads lock it; callbacks never reference it. Register unchanged
// streams too: they must keep a smaller buffer alive until they close.
struct MacAudioBufferLedger {
  struct Owner {
    uint32_t preparedMaximum;
    uint32_t acceptableMaximum;
  };
  struct Entry {
    uint64_t identity = 0;
    uint32_t original = 0;
    uint32_t lastWritten = 0;
    bool changed = false;
    std::unordered_map<uint64_t, Owner> owners;
  };
  std::mutex mutex;
  std::unordered_map<std::string, Entry> devices;
  uint64_t nextToken = 1;
};
MacAudioBufferLedger& macAudioBufferLedger();

struct MacAudioBufferResult {
  AudioHostError error = AudioHostError::None;
  uint32_t frames = 0;
  std::string message;
  bool ok() const { return error == AudioHostError::None; }
};

struct MacAudioBufferWrite {
  bool wrote = false;
  bool observed = false;
  bool confirmed = false;
  uint32_t frames = 0;
};

// LoopZ DeviceInventory::writeBuffer's checked-write pattern: settable, expected
// current value, UID resolution before/after, exact read-back. CoreAudio has no
// atomic cross-process CAS; another app can still race between these calls.
// Request-time negotiation may accept a rounded observation after validating
// ALL limits; release-time restoration requires exact confirmation.
// Every subsequent write/restore rechecks the expected value rather than
// blindly overwriting a newly observed third-party setting.
template <class Access>
MacAudioBufferWrite writeMacAudioBuffer(Access& access, uint32_t expected,
                                        uint32_t desired) {
  MacAudioBufferWrite result;
  uint32_t current = 0;
  if (!access.identityMatches() || !access.isSettable() ||
      !access.readFrames(current) || current != expected || !access.identityMatches()) return result;
  if (!access.writeFrames(desired)) return result;
  result.wrote = true;
  if (!access.identityMatches() || !access.readFrames(result.frames)) return result;
  result.observed = true;
  if (result.frames != desired) return result;
  result.confirmed = true;
  return result;
}

inline uint32_t macAudioBufferCeiling(uint32_t first, uint32_t second) {
  return first == 0 ? second : (second == 0 ? first : std::min(first, second));
}

// Control-domain property transaction. Access supplies UID identity and HAL
// property operations. The same seam exercises the real ledger in unit tests.
template <class Access>
MacAudioBufferResult requestMacAudioBuffer(
    Access& access, uint32_t requested, uint32_t preparedMaximum,
    MacAudioBufferOwnership& ownership, uint32_t acceptableMaximum = 0,
    MacAudioBufferLedger& ledger = macAudioBufferLedger(),
    bool requireOriginalCapacity = true) {
  std::lock_guard lock(ledger.mutex);
  uint32_t before = 0;
  if (!access.identityMatches())
    return {AudioHostError::ProviderFailure, 0, "CoreAudio device identity or buffer size is unavailable"};
  const bool sizeKnown = access.readFrames(before) && before != 0;
  if (!sizeKnown) {
    // Ordinary input can retain a capacity-only lease without optional buffer
    // metadata. It still constrains future writes if the property recovers.
    if (requireOriginalCapacity || requested != 0 || acceptableMaximum != 0)
      return {AudioHostError::ProviderFailure, 0, "CoreAudio buffer size is unavailable"};
    before = 0;
  }
  const std::string& uid = access.deviceUid();
  const auto existing = ledger.devices.find(uid);
  if (existing != ledger.devices.end() && existing->second.identity != access.deviceIdentity())
    return {AudioHostError::InvalidState, before, "CoreAudio device identity changed while buffer leases remain"};
  const uint32_t original = existing == ledger.devices.end() || existing->second.original == 0
      ? before : existing->second.original;
  // Hosts must fit the original/current stream. Input configures its buffer
  // BEFORE starting, so a reduction only needs to fit the future candidate;
  // its lease then prevents growth beyond the prepared input slice.
  if (requireOriginalCapacity && (original > preparedMaximum || before > preparedMaximum))
    return {AudioHostError::InvalidConfiguration, before,
            "Callback maximumFrames must fit the original and current shared CoreAudio buffer"};
  uint32_t candidate = before;
  uint32_t minimum = 0, maximum = 0;
  if (requested != 0) {
    if (!access.readRange(minimum, maximum) || minimum == 0 || maximum < minimum)
      return {AudioHostError::ProviderFailure, before, "CoreAudio buffer range is unavailable"};
    candidate = std::clamp(requested, minimum, maximum);
  }
  // An unchanged ordinary input is a capacity-only reservation. Optional
  // pre-initialize slice metadata must never make that existing capture fail;
  // the callback bounds-checks storage and future writes honor its capacity.
  const bool capacityOnly = !requireOriginalCapacity && requested == 0 && acceptableMaximum == 0;
  if ((!capacityOnly && candidate > preparedMaximum) ||
      (acceptableMaximum != 0 && candidate > acceptableMaximum))
    return {AudioHostError::InvalidConfiguration, before,
            "CoreAudio buffer exceeds the maximum acceptable buffer frames"};
  if (existing != ledger.devices.end()) {
    for (const auto& owner : existing->second.owners)
      if (candidate != before && (candidate > owner.second.preparedMaximum ||
          (owner.second.acceptableMaximum != 0 && candidate > owner.second.acceptableMaximum))
         )
        return {AudioHostError::InvalidConfiguration, before,
                "CoreAudio buffer exceeds an active stream's capacity or acceptable ceiling"};
    if (candidate != before && existing->second.lastWritten != 0 && before != existing->second.lastWritten)
      return {AudioHostError::ProviderFailure, before,
              "Shared CoreAudio buffer changed externally; close leases and refresh before writing"};
  }
  if (ownership.token != 0 && (ownership.uid != uid || existing == ledger.devices.end() ||
                               !existing->second.owners.contains(ownership.token)))
    return {AudioHostError::InvalidState, before, "CoreAudio buffer lease identity changed"};
  auto& entry = ledger.devices.try_emplace(uid,
      MacAudioBufferLedger::Entry{access.deviceIdentity(), original, before, false, {}}).first->second;
  if (sizeKnown && entry.original == 0) entry.original = before;
  if (sizeKnown && entry.lastWritten == 0) entry.lastWritten = before;
  if (ownership.token == 0) {
    ownership = {uid, ledger.nextToken++};
    entry.owners.emplace(ownership.token,
                         MacAudioBufferLedger::Owner{preparedMaximum, acceptableMaximum});
  }
  if (candidate == before) return {AudioHostError::None, before, {}};
  const bool previouslyChanged = entry.changed;
  const auto write = writeMacAudioBuffer(access, before, candidate);
  bool fits = write.observed && write.frames >= minimum && write.frames <= maximum &&
      write.frames <= preparedMaximum &&
      (acceptableMaximum == 0 || write.frames <= acceptableMaximum);
  if (fits) {
    for (const auto& owner : entry.owners)
      if (write.frames > owner.second.preparedMaximum ||
          (owner.second.acceptableMaximum != 0 && write.frames > owner.second.acceptableMaximum))
        fits = false;
  }
  // A successful write can be quantized by the driver. Its immediate bounded
  // read-back is the negotiated value, not necessarily the requested value.
  if (write.wrote && fits) {
    entry.changed = previouslyChanged || write.frames != before;
    entry.lastWritten = write.frames;
    return {AudioHostError::None, write.frames,
            write.frames == requested ? std::string{}
                : "CoreAudio negotiated " + std::to_string(write.frames) +
                      " buffer frames for request " + std::to_string(requested)};
  }
  bool restored = !write.wrote;
  if (write.wrote) {
    uint32_t observed = write.frames;
    bool known = write.observed;
    // A transient read failure must not strand an oversized driver value.
    // Recover an immediate expected value for rollback when identity is valid.
    if (!known && access.identityMatches() && access.readFrames(observed) && access.identityMatches())
      known = true;
    entry.changed = true;
    entry.lastWritten = known ? observed : candidate;
    if (known && observed == before) {
      restored = true;
    } else {
      // Request-time repair deliberately uses the immediate observed value.
      // HAL has no cross-process CAS, so writeMacAudioBuffer rechecks it and
      // device identity before writing. Release-time restore stays strict.
      const auto rollback = writeMacAudioBuffer(access, entry.lastWritten, before);
      restored = rollback.confirmed;
      if (rollback.wrote && rollback.observed) entry.lastWritten = rollback.frames;
    }
    if (restored) {
      entry.lastWritten = before;
      entry.changed = previouslyChanged;
    }
  }
  const bool unsafeReadBack = write.observed && !fits;
  return {AudioHostError::ProviderFailure, write.observed ? write.frames : 0,
          unsafeReadBack
              ? (restored ? "CoreAudio negotiated buffer exceeds range, capacity or maximum acceptable frames; previous size restored"
                          : "CoreAudio negotiated buffer exceeds range, capacity or maximum acceptable frames; rollback failed")
              : (restored ? "CoreAudio checked write or read-back failed; previous size retained/restored"
                          : "CoreAudio checked write or read-back and rollback failed")};
}

// Soft advisory release: keep the current setting until the LAST in-process
// owner closes; never restore over a third-party change. No callback calls here.
template <class Access>
uint32_t restoreMacAudioBuffer(Access& access, MacAudioBufferOwnership& ownership,
                               MacAudioBufferLedger& ledger = macAudioBufferLedger()) {
  if (ownership.token == 0) return AudioBufferWarningNone;
  std::lock_guard lock(ledger.mutex);
  const auto found = ledger.devices.find(ownership.uid);
  if (found == ledger.devices.end()) { ownership = {}; return AudioBufferWarningNone; }
  auto& entry = found->second;
  entry.owners.erase(ownership.token);
  ownership = {};
  if (!entry.owners.empty()) return AudioBufferWarningNone;
  uint32_t warning = AudioBufferWarningNone;
  if (entry.changed) {
    uint32_t current = 0;
    if (entry.identity != access.deviceIdentity() || access.deviceUid() != found->first ||
        !access.identityMatches() || !access.readFrames(current)) {
      warning = AudioBufferWarningRestoreFailed;
    } else if (current == entry.original) {
      // LoopZ BufferPeriod's already-restored case: no further write needed.
    } else if (current != entry.lastWritten) {
      warning = AudioBufferWarningExternalChange;
    } else if (entry.original != entry.lastWritten) {
      const auto restored = writeMacAudioBuffer(access, entry.lastWritten, entry.original);
      if (!restored.confirmed) warning = AudioBufferWarningRestoreFailed;
    }
  }
  ledger.devices.erase(found);
  return warning;
}

}  // namespace singz::detail
