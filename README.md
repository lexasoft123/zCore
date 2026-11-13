# zCore

[![Build and tests](https://github.com/lexasoft123/zCore/actions/workflows/zcore.yml/badge.svg)](https://github.com/lexasoft123/zCore/actions/workflows/zcore.yml)

**zCore is a C++20 audio engine for desktop and mobile applications.** It combines
realtime DSP graphs, audio-device input and output, streaming media, and live
input analysis in a library you can embed with CMake.

zCore powers [SingZ](https://github.com/lexasoft123/SingZ) and
[LoopZ](https://github.com/lexasoft123/LoopZ). Applications build their own playback
and recording workflows on top of it, with control over permissions, user
interfaces, and project storage.

## What it provides

- **DSP graphs:** graph compilation and execution, audio buses, gain, mixing,
  limiting, metering, and latency handling.
- **Audio sources and scheduling:** decoded buffers, streaming windows,
  scheduled cues, and gain events.
- **Device I/O:** capture and playback, device discovery, stream configuration,
  timestamps, and callback transport behind shared interfaces.
- **Media and resampling:** WAV, FLAC, and MP3 decoding and streaming sources,
  plus sample-rate conversion. Extended codecs can use a consumer-supplied
  FFmpeg integration.
- **Live analysis:** optional level and pitch analysis with adapters for
  delivering captured audio outside the realtime callback.
- **Offline rendering and validation:** offline graph rendering, reusable tests,
  and DSP benchmarks.

Graph preparation, device lifecycle, and media work are separated from realtime
processing. Callback and graph-runtime sources have compile restrictions and
source-policy checks to keep unsafe operations out of those paths.

## Platforms

| Platform | Audio backend | Build requirements |
| --- | --- | --- |
| macOS | Core Audio | Xcode command-line tools and the macOS SDK |
| Windows | WASAPI | Visual Studio C++ tools and the Windows SDK |
| Linux | ALSA, with udev device support | ALSA and libudev development packages, plus pkg-config |
| iOS | AudioToolbox and AVAudioSession | Xcode, the iOS SDK, and an Apple toolchain configured by the host project |
| Android | Oboe | Android NDK and an `oboe::oboe` CMake package supplied by the host project |

Standalone CI builds and tests macOS, Windows, and Linux. Mobile integrations
supply their platform toolchains and handle application permissions and lifecycle.

## Build and test

Requires **CMake 3.20+** and a **C++20 compiler**. The test utilities also require
Python 3. On Ubuntu or Debian, install the Linux audio dependencies first:

```sh
sudo apt-get update
sudo apt-get install -y libasound2-dev libudev-dev pkg-config
```

Build all optional library components and the test suite:

```sh
git clone https://github.com/lexasoft123/zCore.git
cd zCore
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release \
  -DZCORE_BUILD_MEDIA=ON \
  -DZCORE_BUILD_ANALYSIS=ON \
  -DZCORE_BUILD_TESTS=ON
cmake --build build --config Release --parallel 4
ctest --test-dir build -C Release --output-on-failure
```

The graph runtime and device components are available in the default build.
Optional components are controlled independently:

| Option | Default | Enables |
| --- | --- | --- |
| `ZCORE_BUILD_MEDIA` | `OFF` | File decoding and streaming media |
| `ZCORE_BUILD_ANALYSIS` | `OFF` | Live-input analysis adapters |
| `ZCORE_BUILD_TESTS` | `OFF` | Reusable CTest suites |
| `ZCORE_BUILD_BENCHMARKS` | `OFF` | DSP benchmark executables |
| `ZCORE_BUILD_TOOLS` | `OFF` | Opt-in, muted device buffer probe |

On macOS, `AudioInputConfig::requestedBufferFrames` requests an input device I/O
buffer size (zero leaves it unchanged). A nonzero request requires the explicit
`AudioInputConfig::allowSharedDeviceBufferChange = true` opt-in; its default is
false, and an unapproved request returns `Unsupported` before device resolution.
The host also requires `AudioHostConfig::allowSharedDeviceBufferChange = true`
for nonzero opening requests and live writes.
`AudioHost::setBufferFrames(uint32_t, uint32_t maximumAcceptableBufferFrames = 0)`
changes an open or running output without stopping or reopening it. Both clamp
to the current device range and report the read-back size in
`AudioInputResult::inputBufferFrames` or `AudioHostResult::latency.bufferFrames`.
Input latency continues to exclude the I/O buffer. These CoreAudio settings are
device-wide and shared with other apps. A shared process-wide ledger retains the
original and restores when the last lease that wrote the buffer closes, even
while capacity-only leases remain, if the original fits every remaining
prepared capacity and ceiling. A refused restore is a soft warning and can be
retried on a later release. Restoration only writes while the current value
still matches this process's last write. Stop-time restore failures
and external changes are soft `bufferWarnings` in host status or input stats.
Once no writer remains, an external value becomes the next restoration baseline,
so a capacity-only song lease does not block re-enabling monitoring.
An ordinary input stop releases its lease; an explicit retained-prepared stop
keeps the stopped unit and lease for fast re-arm until a real close. This opt-in per-lifetime policy differs from LoopZ's
explicit persistent per-device setting.

Input and host configs accept `maximumAcceptableBufferFrames` (zero = no extra
limit). The host live setter accepts an additional ceiling, combined with its
opening limit. Use 256 for monitoring: a device minimum above this is rejected
before writing. Rounded read-backs are accepted within the device range and
every active capacity/ceiling; unsafe observations trigger checked rollback using
the immediate observed value as the expected value.
Callers must still verify the reported size. Host `maximumFrames` is callback
capacity: it must fit the original device buffer even if another lease already
lowered the current size. Raising a live buffer beyond any active prepared
capacity or opening ceiling is rejected. External changes publish
`AudioHostStatus::bufferExceedsMaximum` and `Error` if they exceed callback
capacity; `status()` reads a synchronized snapshot and performs no HAL query.

A shared property change does not prove that an already-open provider adopted
the period. `observedCallbackFrames` is a plain diagnostic published by one
bounded lock-free callback store. The parent compares it with the negotiated
buffer and chooses its own timeout or controlled reopen. No automatic
verification state or reopen is maintained here. `format.nominalBufferFrames`
and `latency.bufferFrames` retain the device property's negotiated size.
Call lifecycle and status methods from an ordinary control worker. Status takes
the control mutex and can wait behind blocking HAL transactions; do not call it
from a UI or realtime-sensitive thread. Ordinary input capture sizes its render
storage from the larger of the reported slice (or 4096-frame fallback with a
warning) and current device buffer, capped at 16384 frames. No AUHAL capacity
is forced. Unchanged capture holds a capacity-only lease and can open when the
device property exceeds that cap; an oversized callback is dropped safely and
reported as a capture failure. A 4096-frame mono render plane uses 16 KiB,
with the existing hard limit bounding it at 64 KiB.
The writer must be the process that owns the stream. LoopZ observed a 32-frame
setter in one process while another still read/used 512, and deliberately reopens
its monitor streams after hardware-buffer settings changes.
Non-macOS input buffer requests, negotiation ceilings and live output changes
report `Unsupported`; callers should set these options only on macOS.

`chooseMonitorBufferFrames(inRange, outRange, target = 32, maximum = 256)` returns
a candidate in both reported `AudioHostBufferRange` bounds, or `nullopt` when
the ranges are unknown, disjoint, or exceed the budget. Discrete period
restrictions still require provider negotiation; check the reported size before
enabling monitoring.

Build with `-DZCORE_BUILD_TOOLS=ON`, then explicitly run
`zcore_audio_host_buffer_probe --live-buffer 256 32` to try the default output.
It renders only silence, opens at the first requested size, changes live to the
second, and prints negotiated sizes, callback progress, route/stream generations,
xruns, deadline misses, discontinuities, invalid callbacks, actual callback size,
negotiated/observed comparison and stop warnings. These are host
telemetry counters, not an acoustic glitch measurement. Hardware testing with
song playback and loopback measurement remains a consumer integration check;
unit tests cover property negotiation/rollback and fake backends, while macOS
integration is compile-checked.

## Add to your project

Add zCore as a Git submodule and commit the dependency at a tested revision:

```sh
git submodule add https://github.com/lexasoft123/zCore.git external/zcore
```

Then include it in your CMake project. This example enables media support and
links an application whose sources live in `main.cpp`:

```cmake
cmake_minimum_required(VERSION 3.20)
project(MyAudioApp LANGUAGES C CXX)

set(ZCORE_BUILD_MEDIA ON CACHE BOOL "" FORCE)
add_subdirectory(external/zcore)

add_executable(my_audio_app main.cpp)
target_link_libraries(my_audio_app PRIVATE
  zCore::runtime
  zCore::device
  SingZ::zcore_media
)
```

Choose the components your application needs:

| Component | CMake target | Public headers |
| --- | --- | --- |
| Graph runtime | `zCore::runtime` | [`zdsp`](zdsp/include/zdsp) |
| Audio devices | `zCore::device` | [`zcore/device`](zcore/include/zcore/device) |
| Capture transport | `zCore::audio` | [`zcore/audio`](zcore/include/zcore/audio) |
| Resampling | `zCore::resample` | [`resample.h`](zcore/include/zcore/legacy/resample.h) |
| Device-to-graph adapter | `SingZ::zdsp_host_adapter` | [`audio_host_graph_adapter.h`](zdsp/include/zdsp/audio_host_graph_adapter.h) |
| Media, when enabled | `SingZ::zcore_media` | [`media`](zcore/include/zcore/media) |
| Live analysis, when enabled | `SingZ::zdsp_analysis` | [`live_input_analysis.h`](zdsp/include/zdsp/analysis/live_input_analysis.h) |
| Offline rendering | `SingZ::zdsp_offline` | [`offline_renderer.h`](zdsp/include/zdsp/offline_renderer.h) |

Some component targets currently use the `SingZ::` namespace; use the target
names listed above when linking.

For an existing consumer checkout, initialize or update its pinned dependency:

```sh
git submodule update --init --recursive
```

## License and third-party notices

A project-wide license has not yet been declared. Bundled third-party components
retain their own license notices, including
[libFLAC](third_party/native/flac/COPYING.Xiph) and
[dr_mp3](third_party/native/dr_mp3/dr_mp3.h).
