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
