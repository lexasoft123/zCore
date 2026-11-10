# Routing provenance

The portable routing API and graph implementation are adapted from LoopZ
commit `e818b011ccb2160921f72188f9e35a36b431d57f`:

- `native/include/loopz/routing.hpp`
- `native/core/routing.cpp`
- Portable routing cases from `tests/routing_tests.cpp`

The namespace is `zcore::routing`; headers use the zCore include tree and the
256-frame maximum block is defined by this API rather than LoopZ's timeline.
No virtual-device documents, OS adapters, service or UI code is imported.
The graph builder is under `zdsp/src/control/routing.cpp` so the foundational
`zcore/` layer never includes zdsp headers.
Prepared routing remains immutable: prepare a replacement graph on control,
retire the previous render caller, then destroy the old graph off the audio thread.
