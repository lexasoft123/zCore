#pragma once

#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace zcore::routing {

inline constexpr uint32_t kBlock = 256;
inline constexpr uint32_t kMaximumRoutingChannels = 64;
inline constexpr uint32_t kMaximumRoutingSources = 32;

struct ChannelConnection {
  uint32_t sourceChannel;
  uint32_t outputChannel;
  float gain{1};
};

struct SourceRouting {
  std::string id;
  uint32_t channels{2};
  float gain{1};
  bool enabled{true};
  bool muted{false};
  std::vector<ChannelConnection> connections;
};

struct RoutingConfiguration {
  uint32_t outputChannels{2};
  std::vector<SourceRouting> sources;
  float gain{1};
  bool enabled{true};
  bool muted{false};
  bool safetyLimiter{true};
};

// Portable routing adapted from LoopZ; see zcore/src/routing/PROVENANCE.md.
// All allocation and validation happen before the graph reaches the renderer.
// Channel numbers are zero-based; an empty connection list means no wiring.
void validateRoutingConfiguration(const RoutingConfiguration &configuration);
std::vector<ChannelConnection>
defaultChannelConnections(uint32_t sourceChannels, uint32_t outputChannels);
// Fold discrete stereo pairs at unity gain: 1/3/5... to left, 2/4/6... to
// right. Mono is duplicated. This is not a surround-speaker downmix or a
// limiter.
std::vector<ChannelConnection>
discreteStereoConnections(uint32_t sourceChannels);

struct RoutingInput {
  std::span<const float *const> channels;
  uint32_t capacityFrames;
};

struct RoutingOutput {
  std::span<float *const> channels;
  uint32_t capacityFrames;
};

class RoutingGraph {
public:
  RoutingGraph(double sampleRate, const RoutingConfiguration &configuration);
  ~RoutingGraph();
  RoutingGraph(const RoutingGraph &) = delete;
  RoutingGraph &operator=(const RoutingGraph &) = delete;

  // Single audio-thread caller. Invalid blocks fail to silence within the
  // declared output capacity. Sources are ordered as in the configuration.
  bool render(std::span<const RoutingInput> sources, RoutingOutput output,
              uint32_t frames, uint64_t position) noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

} // namespace zcore::routing
