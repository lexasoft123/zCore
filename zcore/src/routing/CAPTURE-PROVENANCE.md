# Independent-clock capture provenance

Ported from LoopZ commit `e818b011ccb2160921f72188f9e35a36b431d57f` on 5 November 2026. Source was read-only.

Behavior is retained: timestamp-addressed multichannel history, measured clock mapping, missing support as silence, prepared bandlimited linear/minimum-phase readers, optional diagnostic counters. Changes are namespace/include paths, a private timing helper, extracting AudioSource/read contracts without product documents/meters, and a local reader macro name. No macOS routing service or platform capture code is copied.

| Source | SHA-256 at port |
| --- | --- |
| `native/include/loopz/audio_history.hpp` | `905328c27f2e251c81b49e0fa97be3d6cbe0324517ef64152fd5b3a72d0d25f1` |
| `native/include/loopz/audio_resampler.hpp` | `b0055a7b851119a9ec2f134b5f428315936cf81a5534de504e00aa97dbf2e5da` |
| `native/include/loopz/sample_clock.hpp` | `08373813c0ce5f5d3dd2f3a505583c7426cda9cea2aa38172f9e392d546bc622` |
| `native/include/loopz/sinc_kernel.hpp` | `4b0ef123de73d14028f6e5d7263cd6033883fdcd04ac78d1ff8208f4bbb6b8f2` |
| `native/include/loopz/minimum_phase_kernel.hpp` | `e7e429993b34278661c18b897ae50f47248d128f0d275274b3ed387da84139fa` |
| `native/core/audio_history.cpp` | `4c45c3b49b225b7fd6f8e3fe7d14b1c4aae2f1ef4f50785728145866bcad0d89` |
| `native/core/audio_resampler.cpp` | `5ddd2099ab20e4426666bd4e23366646d3baf65c23c36b86b88ba0b1ff423085` |
| `native/core/sample_clock.cpp` | `8300561584bbd086afc5b4c106a771f6f73616a31b815ff27c8966ffcb89566d` |
| `native/core/sinc_kernel.cpp` | `dd8a1f786d7614f6a68fc0487169442f4671598e5d0757b2dfd3fd3914c79670` |
| `native/core/minimum_phase_kernel.cpp` | `1b0cac6535f5c4294034d2b729d86ee4ebeb012c97d6ffd14610dfef0daa9e85` |
| `native/core/cached_fir_kernels.cpp` | `7e649c6d62c28c13d2d66edbad087f531fd75c397609bdfa721216036e16678e` |
| `native/core/unity_convolution.hpp` | `009d74528ae8344c3752b9a4f7a270cd52b67cb29e5f22e984d0d4ac925cff4c` |
| `native/core/cached_fir.hpp` | `715b9f70dba6bc3483254dc2e9477b46d27952ba98910e09790b59bbdefc8edb` |
| `native/core/cached_fir_kernels.hpp` | `c961f27961357a3057a7691022e54be940d7915b42995f6d2e4ff598d877aa9e` |
| `native/core/resample_scratch.hpp` | `5b809b0301e94961d4a8ec6c8994ee3b4205c22ce65a78ee57dbdc73e21376e3` |
| `native/core/finite_samples.hpp` | `e369d09470fb2acf08e9f4bac0be5e69f1d981776ecdad92a7dce9ae1c00b1c6` |
| `native/include/loopz/audio_scene.hpp` | `988130274aff2be6ff44b21d14a9486964abaaef4ae81e425e82ea17d673eca6` |
| `native/include/loopz/timeline.hpp` | `ddd00d0c24169b4b4189f55267662359e72d8da74dfec875bcc814d9354c2bfd` |
