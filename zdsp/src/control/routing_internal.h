#pragma once

#include <array>
#include <zcore/routing/routing.h>
#include <zdsp/graph_runner.h>

namespace zcore::routing {
struct RoutingGraph::Impl {
  double rate;
  RoutingConfiguration configuration;
  std::vector<uint8_t> memory;
  zdsp::RealtimeArena arena{};
  std::array<zdsp::AudioChannelRole, kMaximumRoutingChannels> channelRoles{};
  std::vector<zdsp::AudioBusDescriptor> inputDescriptors;
  zdsp::AudioBusDescriptor routedDescriptor{};
  std::array<zdsp::AudioBusDescriptor, zdsp::kMaximumBusesPerProcessor>
      mixDescriptors{};
  std::vector<std::vector<float>> matrices;
  std::vector<zdsp::GraphNodeDescription> nodes;
  std::vector<zdsp::GraphConnection> edges;
  zdsp::GraphCompileResult compiled{};
  zdsp::SnapshotPublisher publisher{};
  zdsp::RetirementSlot retired[2]{};
  zdsp::PublishedGraphSnapshot snapshot{};
  zdsp::GraphRunner runner{};
  zdsp::RuntimeDiagnostics diagnostics{};
  bool runnerInitialized{false};
  bool silent{false};
  std::vector<size_t> activeInputs;

  struct Group {
    std::unique_ptr<RoutingGraph> graph;
    std::vector<float> samples;
    std::array<float *, kMaximumRoutingChannels> writable{};
    std::array<const float *, kMaximumRoutingChannels> readable{};
    size_t first;
    size_t count;
  };
  std::vector<Group> groups;
  std::unique_ptr<RoutingGraph> sumGraph;

  zdsp::AudioBusDescriptor descriptor(uint32_t channels) const;

  zdsp::NodeId addProcessor(zdsp::BuiltinNodeKind kind, uint32_t inputChannels,
                            uint32_t buses,
                            const zdsp::AudioBusDescriptor *inputs,
                            float value = 1, const float *matrix = nullptr);

  explicit Impl(double sampleRate, const RoutingConfiguration &route);

  ~Impl();
};
} // namespace zcore::routing
