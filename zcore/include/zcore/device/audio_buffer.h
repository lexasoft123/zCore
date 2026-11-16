#pragma once

#include <cstdint>

namespace singz {
// Advisory lifecycle outcomes; these never make a stopped stream an Error.
enum AudioBufferWarning : uint32_t {
  AudioBufferWarningNone = 0,
  AudioBufferWarningExternalChange = 1u << 0,
  AudioBufferWarningRestoreFailed = 1u << 1,
  AudioBufferWarningReadFailed = 1u << 2,
};
}  // namespace singz
