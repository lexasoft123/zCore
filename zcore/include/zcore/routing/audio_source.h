#pragma once
#include <zcore/routing/routing.h>
#include <optional>
namespace zcore::routing {
enum class ResamplingFilter;
// A reader cache belongs to one serial render owner. Keeping it shared across
// prepared scene replacements preserves unchanged source clock anchors. Neither
// the control thread nor a second render thread may mutate it concurrently.
struct AudioReadCursor {
    uint64_t frame{0}, hostNs{0}, generation{0};
    double rate{0};
    uint32_t frames{0};
};
struct AudioReadResult {
    uint32_t missingFrames;
    bool online;
};

class AudioSource {
  public:
    virtual ~AudioSource() = default;
    virtual uint32_t channels() const noexcept = 0;
    // Render-domain contract: no allocation, locks, logging or OS operations.
    // Fill all requested planes; absent frames are zero. Independent readers
    // use separate cursors and do not consume one another's history.
    virtual AudioReadResult read(uint64_t firstHostNs, double rate, RoutingOutput output,
                                 uint32_t frames, AudioReadCursor& cursor) const noexcept = 0;
    // Optional sparse read. Channels are unique, increasing source indices;
    // output planes correspond to that list. Unsupported adapters return nullopt
    // without touching output, allowing the caller to use the full-layout path.
    virtual std::optional<AudioReadResult>
    readChannels(uint64_t, double, RoutingOutput, uint32_t, AudioReadCursor&,
                 std::span<const uint32_t>, std::optional<ResamplingFilter>) const noexcept {
        return std::nullopt;
    }
    // A prepared physical reader may select its filter per scene. The default
    // preserves adapters without selectable resampling. Selection must never
    // prepare coefficients, allocate or change shared producer state here.
    virtual AudioReadResult readWithFilter(uint64_t firstHostNs, double rate, RoutingOutput output,
                                           uint32_t frames, AudioReadCursor& cursor,
                                           ResamplingFilter) const noexcept {
        return read(firstHostNs, rate, output, frames, cursor);
    }
};

} // namespace zcore::routing
