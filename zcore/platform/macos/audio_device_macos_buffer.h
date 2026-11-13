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
// streams too: their prepared capacities constrain writes and restoration.
struct MacAudioBufferLedger {
  struct Owner {
    uint32_t preparedMaximum;
    uint32_t acceptableMaximum;
    bool changed = false;
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
  uint32_t warnings = AudioBufferWarningNone;
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

// An observed external value becomes the next lifetime's restoration baseline
// only after all writers have gone. Capacity-only leases do not own a value.
inline bool macAudioBufferHasWriter(const MacAudioBufferLedger::Entry& entry) {
  return entry.changed && std::any_of(entry.owners.begin(), entry.owners.end(),
      [](const auto& owner) { return owner.second.changed; });
}
inline void rebaselineMacAudioBuffer(MacAudioBufferLedger::Entry& entry, uint32_t current) {
  entry.original = entry.lastWritten = current;
  entry.changed = false;
}

struct MacAudioBufferFixedCapacity {
  uint32_t operator()(uint32_t prepared, uint32_t, uint32_t) const { return prepared; }
};

// Post-initialize input confirmation only revises this owner's reservation;
// it never writes the device or changes any other owner's ceiling.
inline bool updateMacAudioBufferCapacity(const MacAudioBufferOwnership& ownership,
                                        uint32_t capacity,
                                        MacAudioBufferLedger& ledger = macAudioBufferLedger()) {
  std::lock_guard lock(ledger.mutex);
  const auto device = ledger.devices.find(ownership.uid);
  if (device == ledger.devices.end()) return false;
  const auto owner = device->second.owners.find(ownership.token);
  if (owner == device->second.owners.end()) return false;
  owner->second.preparedMaximum = capacity;
  return true;
}

// Control-domain property transaction. Access supplies UID identity and HAL
// property operations. The same seam exercises the real ledger in unit tests.
template <class Access, class PrepareCapacity = MacAudioBufferFixedCapacity>
MacAudioBufferResult requestMacAudioBuffer(
    Access& access, uint32_t requested, uint32_t preparedMaximum,
    MacAudioBufferOwnership& ownership, uint32_t acceptableMaximum = 0,
    MacAudioBufferLedger& ledger = macAudioBufferLedger(),
    bool requireOriginalCapacity = true, PrepareCapacity prepareCapacity = {}) {
  std::lock_guard lock(ledger.mutex);
  uint32_t warnings = AudioBufferWarningNone;
  uint32_t before = 0;
  if (!access.identityMatches())
    return {AudioHostError::ProviderFailure, 0, "CoreAudio device identity or buffer size is unavailable", warnings};
  const bool sizeKnown = access.readFrames(before) && before != 0;
  if (!sizeKnown) {
    // Ordinary input can retain a capacity-only lease without optional buffer
    // metadata. It still constrains future writes if the property recovers.
    if (requireOriginalCapacity || requested != 0 || acceptableMaximum != 0)
      return {AudioHostError::ProviderFailure, 0, "CoreAudio buffer size is unavailable", warnings};
    before = 0;
  }
  const std::string& uid = access.deviceUid();
  const auto existing = ledger.devices.find(uid);
  if (existing != ledger.devices.end() && existing->second.identity != access.deviceIdentity())
    return {AudioHostError::InvalidState, before, "CoreAudio device identity changed while buffer leases remain", warnings};
  if (ownership.token != 0 && (ownership.uid != uid || existing == ledger.devices.end() ||
                               !existing->second.owners.contains(ownership.token)))
    return {AudioHostError::InvalidState, before, "CoreAudio buffer lease identity changed", warnings};
  if (existing != ledger.devices.end() && sizeKnown &&
      existing->second.lastWritten != 0 && before != existing->second.lastWritten &&
      !macAudioBufferHasWriter(existing->second)) {
    rebaselineMacAudioBuffer(existing->second, before);
    warnings |= AudioBufferWarningExternalChange;
  }
  const uint32_t original = existing == ledger.devices.end() || existing->second.original == 0
      ? before : existing->second.original;
  // Choose storage and reserve that exact capacity in the same transaction.
  // In particular, an input opened under a lowered buffer must also fit the
  // ledger baseline that another owner's release can restore.
  preparedMaximum = prepareCapacity(preparedMaximum, before, original);
  // Hosts must fit the original/current stream. Input configures its buffer
  // BEFORE starting, so a reduction only needs to fit the future candidate;
  // its lease then prevents growth beyond the prepared input slice.
  if (requireOriginalCapacity && (original > preparedMaximum || before > preparedMaximum))
    return {AudioHostError::InvalidConfiguration, before,
            "Callback maximumFrames must fit the original and current shared CoreAudio buffer", warnings};
  uint32_t candidate = before;
  uint32_t minimum = 0, maximum = 0;
  if (requested != 0) {
    if (!access.readRange(minimum, maximum) || minimum == 0 || maximum < minimum)
      return {AudioHostError::ProviderFailure, before, "CoreAudio buffer range is unavailable", warnings};
    candidate = std::clamp(requested, minimum, maximum);
  }
  // An unchanged ordinary input is a capacity-only reservation. Optional
  // pre-initialize slice metadata must never make that existing capture fail;
  // the callback bounds-checks storage and future writes honor its capacity.
  const bool capacityOnly = !requireOriginalCapacity && requested == 0 && acceptableMaximum == 0;
  if ((!capacityOnly && candidate > preparedMaximum) ||
      (acceptableMaximum != 0 && candidate > acceptableMaximum))
    return {AudioHostError::InvalidConfiguration, before,
            "CoreAudio buffer exceeds the maximum acceptable buffer frames", warnings};
  if (existing != ledger.devices.end()) {
    for (const auto& owner : existing->second.owners)
      if (candidate != before && (candidate > owner.second.preparedMaximum ||
          (owner.second.acceptableMaximum != 0 && candidate > owner.second.acceptableMaximum))
         )
        return {AudioHostError::InvalidConfiguration, before,
                "CoreAudio buffer exceeds an active stream's capacity or acceptable ceiling", warnings};
    if (candidate != before && macAudioBufferHasWriter(existing->second) &&
        existing->second.lastWritten != 0 && before != existing->second.lastWritten)
      return {AudioHostError::ProviderFailure, before,
              "Shared CoreAudio buffer changed externally; close leases and refresh before writing", warnings};
  }
  auto& entry = ledger.devices.try_emplace(uid,
      MacAudioBufferLedger::Entry{access.deviceIdentity(), original, before, false, {}}).first->second;
  if (sizeKnown && entry.original == 0) entry.original = before;
  if (sizeKnown && entry.lastWritten == 0) entry.lastWritten = before;
  if (ownership.token == 0) {
    ownership = {uid, ledger.nextToken++};
    entry.owners.emplace(ownership.token,
                         MacAudioBufferLedger::Owner{preparedMaximum, acceptableMaximum});
  }
  if (candidate == before) return {AudioHostError::None, before, {}, warnings};
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
    if (write.frames != before) entry.owners.at(ownership.token).changed = true;
    return {AudioHostError::None, write.frames,
            write.frames == requested ? std::string{}
                : "CoreAudio negotiated " + std::to_string(write.frames) +
                      " buffer frames for request " + std::to_string(requested), warnings};
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
  if (write.wrote && !restored) entry.owners.at(ownership.token).changed = true;
  const bool unsafeReadBack = write.observed && !fits;
  return {AudioHostError::ProviderFailure, write.observed ? write.frames : 0,
          unsafeReadBack
              ? (restored ? "CoreAudio negotiated buffer exceeds range, capacity or maximum acceptable frames; previous size restored"
                          : "CoreAudio negotiated buffer exceeds range, capacity or maximum acceptable frames; rollback failed")
              : (restored ? "CoreAudio checked write or read-back failed; previous size retained/restored"
                          : "CoreAudio checked write or read-back and rollback failed"), warnings};
}

// Soft advisory release: the last writer restores while capacity-only leases
// remain, if their budgets permit. Never overwrite a third-party change.
// A refused/failed restore retains the baseline for a later release to retry.
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
  for (const auto& owner : entry.owners)
    if (owner.second.changed) return AudioBufferWarningNone;
  uint32_t warning = AudioBufferWarningNone;
  if (entry.changed) {
    uint32_t current = 0;
    if (entry.identity != access.deviceIdentity() || access.deviceUid() != found->first ||
        !access.identityMatches() || !access.readFrames(current)) {
      warning = AudioBufferWarningRestoreFailed;
    } else if (current == entry.original) {
      // LoopZ BufferPeriod's already-restored case: no further write needed.
    } else if (current != entry.lastWritten) {
      // Preserve and adopt the external value before checking restore budgets;
      // no process writer remains, so the old baseline must not poison re-arm.
      warning = AudioBufferWarningExternalChange;
      rebaselineMacAudioBuffer(entry, current);
    } else if (entry.original != entry.lastWritten) {
      for (const auto& owner : entry.owners)
        if (entry.original > owner.second.preparedMaximum ||
            (owner.second.acceptableMaximum != 0 && entry.original > owner.second.acceptableMaximum))
          return AudioBufferWarningRestoreFailed;
      const auto restored = writeMacAudioBuffer(access, entry.lastWritten, entry.original);
      if (!restored.confirmed) warning = AudioBufferWarningRestoreFailed;
    }
  }
  if (warning == AudioBufferWarningNone) {
    entry.lastWritten = entry.original;
    entry.changed = false;
  }
  if (entry.owners.empty()) ledger.devices.erase(found);
  return warning;
}

}  // namespace singz::detail
