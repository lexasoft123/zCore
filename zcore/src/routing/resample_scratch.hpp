// Adapted from LoopZ portable routing; see zcore/src/routing/CAPTURE-PROVENANCE.md.
#pragma once
// Keep each bounded reader workspace in its own stack frame. Inlining two
// mutually exclusive large readers into their caller can multiply stack usage.
#if defined(_MSC_VER)
#define ZCORE_ROUTING_READER_NOINLINE __declspec(noinline)
#elif defined(__GNUC__)
#define ZCORE_ROUTING_READER_NOINLINE __attribute__((noinline))
#else
#define ZCORE_ROUTING_READER_NOINLINE
#endif
