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
original and restores when the last writer or explicit hold lease ends, even
while capacity-only leases remain, if the original fits every remaining
prepared capacity and ceiling. `AudioHostConfig::holdSharedDeviceBuffer = true`
registers a hold at open even if the requested period is zero or already current.
For example, a monitor lowers 512 to 32 frames, then a song opens with a hold:
monitor stop keeps 32 without a restore-failure warning; song stop restores 512,
even if independent ordinary capture remains open. Set a hold on each host that
needs this lifetime, including a monitor that may join an already-small buffer.
The default is false; ordinary capture capacity alone does not retain the value.
A hold is independent of `allowSharedDeviceBufferChange` write consent and
`maximumAcceptableBufferFrames`; it neither writes nor pins a specific period.
It is released on stop, destruction/reopen, opening/start failure, or device loss;
live buffer requests preserve the opening hold. Other platforms ignore the field.
The field is appended to the config; no existing member or virtual method changes.
Rebuild C++ consumers with the updated header. A refused restore is a soft warning
and can be retried on a later release. Restoration only writes while the current value
still matches this process's last write. Stop-time restore failures
and external changes are soft `bufferWarnings` in host status or input stats.
Once no writer or hold remains, an external value becomes the next restoration
baseline, so a capacity-only song lease does not block re-enabling monitoring.
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
warning), current device buffer and the ledger's original buffer, capped at
16384 frames. Capacity selection and lease registration share one ledger
transaction. Before initialization, AUHAL's maximum slice is raised only when
the chosen storage needs it; rejected settings produce an advisory warning.
After initialization, the lease reserves the smaller of storage and the
confirmed unit slice. An unreadable unit limit reserves zero until close;
restoration above a remaining reservation is refused softly. Ordinary capture
still opens when this optional setting/readback fails.
AUHAL updates its render slice asynchronously after device-buffer changes,
including shrink-then-grow sequences that invalidate a pre-initialize preset.
Input tolerates only `kAudioUnitErr_TooManyFramesToProcess` (-10874): that
callback delivers an equal-length block of zeros at its own normal timestamps
and returns success so a later callback can recover. The failed signal is
discarded; silence preserves take duration, sequence and source-frame progress
without presenting stale signal to pitch analysis.
More than 64 consecutive such callbacks, or a failed callback at least 250 ms
after the first in the burst, uses the existing fatal render-error path. A
successful render resets the burst; every other render status remains fatal.
`AudioInputStats::renderSliceDrops` counts these tolerated silence-replaced blocks per
take, including after stop, separately from ring overruns and application event
drops. The callback performs no unit property queries or repair transactions.
Portable tests exercise this policy and accounting through a fake backend and
the real input facade; actual AUHAL render/return behavior and sample continuity
still require the Mac hardware checklist.
Raw transport does not mark host-time jumps caused by missing callbacks with
otherwise successful renders. Timestamp quality alone does not indicate this
gap; recording consumers must compare successive sample host times and block
durations. The analysis adapter's existing clock-reanchor detection resets
analysis windows but does not insert missing time into a concatenating recorder.
Unchanged input stream-configuration notifications are benign; input stream
IDs, physical/virtual formats, rate and per-buffer channel layout are compared
after one retry of a failed read. Buffer-only notifications remain advisory.
Retained re-arm re-reads these facts and rejects a changed route before start,
even without a pending notification. Unchanged capture holds a capacity-only lease and can open when the
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

## Capture timestamp phase tolerance

`SampleClockPhaseConfig` configures `SampleClockHistory(capacity, config)` or
`AudioHistory(channels, capacity, filter, readDiagnostics, config)` on control.
`maximumStepSamples` defaults to 2 (valid range 0..16); the effective bound is
also capped at 0.1 ms at the nominal rate. `maximumCorrectionsPerSecond`
defaults to 8 (range 0..64), in a rolling one-second window. Zero for either
field disables unverified phase tolerance. Invalid configurations throw at
construction. Neither setting changes the existing sample-counter-verified
correction policy described in `audio_history.h`.

A small host-time phase step which would fail the ordinary ±0.5% interval-rate
check now re-anchors the resampler phase over that closed callback interval.
It retains the raw timestamp endpoint and uses a fractional slope: at most the
configured sample displacement across the interval, additionally limited to
0.8..1.25 of nominal rate. No stored PCM frames are removed, inserted or copied.
The corrected slope is never extrapolated into the next open interval. A
persistent one-sample step therefore consumes one correction, without silently
shifting the queue or altering host timestamps. Larger/unbudgeted intervals
retain the existing missing-audio behavior; >2 ms jumps and hardware sample
counter gaps retain the generation-reset path. Timestamp quantization has a
2 ns comparison allowance. The rolling budget survives a history reset.

`AudioHistoryStatus` and the opt-in `AudioHistoryReadSnapshot` append
`phaseCorrections`, `correctedSamples`, and `rejectedIntervals`.
`SampleClockHistory::phaseDiagnostics()` returns the same producer counters.
`phaseCorrections` counts accepted unverified phase intervals;
`correctedSamples` sums the absolute displacement rounded up in samples (after
the 2 ns quantization allowance); `rejectedIntervals` counts closed intervals
which still cannot be mapped, once per publication, rather than per reader or
missing output frame. These cumulative counters survive stop/reset/reopen on
the same history object. The existing `verifiedClockSteps` diagnostics continue
to describe the separate authoritative-counter correction path. Status counters
are available even when optional read diagnostics are disabled.

macOS `AudioInput` now delivers `AudioInputBlockView::sourcePosition`, an
`AudioInputSourcePosition { double sampleFrame; bool valid; }` copied from the
AUHAL `mSampleTime` when its validity flag is set and the value is finite.
Negative hardware positions are valid. This is independent of the transport's
`capture.sourceFrame` counter, which counts *offered* PCM and cannot prove
hardware continuity. Feed `sourcePosition.sampleFrame` and `.valid` to the
existing `AudioHistory::append` source-frame arguments; a counter gap takes
precedence over timestamp tolerance. Missing sample-time validity stays false;
no callback-derived counter is promoted to hardware evidence. The additive
`AudioInputBackend::setPositionPush(AudioInputPositionPush)` extension installs
an optional replacement push before start; unsupported backends and existing
`AudioInputPush` callbacks retain their original behavior. Both push paths keep
all host timestamps, timestamp quality, capture metadata, sequence and PCM
identical, including silence supplied after tolerated AUHAL render failures.
Take gap-padding consumers continue to use the original hardware host times.

ABI layout (64-bit macOS, bytes): `AudioInputSourcePosition` is 16 bytes;
`AudioInputBlockView` grows 112 -> 128, appending `sourcePosition` at offset 112;
`AudioHistoryStatus` grows 40 -> 64 and `AudioHistoryReadSnapshot` grows
152 -> 176. Their counters start at offsets 40 and 152 respectively and are
three uint64 values in the order above. `SampleClockPhaseConfig` is 16 bytes and
`SampleClockPhaseDiagnostics` is 24 bytes.
Existing field offsets and legacy function signatures are unchanged; the
original constructor symbols remain. This static C++ library requires consumers
to rebuild when exchanging extended structs or subclassing `AudioInputBackend`
(whose optional virtual is appended); mixed old/new binary layouts are not safe.

Portable `zcore_routing_phase_tolerance_tests` reproduces the original 33/65/129
missing-frame holes at periods 32/64/128 with a 1024-frame queue and tolerance
disabled, then verifies 0/0/0 with tolerance enabled, both step signs, zero
resets, bounded output displacement, a 64-sample rejection, absolute time bound,
rolling budget and hardware-counter gaps. Two compiled production-code mutants
must fail with their own messages when tolerance or the sample bound is removed.
`zcore_audio_input_position_tests` verifies the real facade delivery extension,
negative hardware positions, reused-ring-slot validity and identical
legacy timestamp/PCM/metadata values for gap-padding consumers.

For an application take-writer regression, optionally configure with
`-DZCORE_CAPTURE_TAKE_WRITER_SOURCE=/path/to/consumer/native/electron/take_writer.cpp`.
`zcore_capture_take_compatibility_tests` feeds legacy and hardware-position
capture blocks into that actual writer and requires identical WAV bytes,
including a 512-frame host-time gap with a continuous sample counter. The
standalone package has no default dependency on a consumer implementation.
