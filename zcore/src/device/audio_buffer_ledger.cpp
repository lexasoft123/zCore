#include "../../platform/macos/audio_device_macos_buffer.h"

namespace singz::detail {
MacAudioBufferLedger& macAudioBufferLedger() {
  // Hosts can themselves be process-lifetime services. Avoid a static teardown
  // order race if a host releases its final lease after this function's static
  // would otherwise have been destroyed. Callbacks never touch the ledger.
  static auto* ledger = new MacAudioBufferLedger();
  return *ledger;
}
}  // namespace singz::detail
