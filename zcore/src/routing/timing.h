#pragma once
#include <zcore/routing/routing.h>
#include <cstdint>
#include <atomic>
namespace zcore::routing {
static_assert(std::atomic<uint64_t>::is_always_lock_free);
static_assert(std::atomic<uint32_t>::is_always_lock_free);
inline constexpr uint32_t kCapacity = 32768;
inline uint64_t frameNs(uint64_t frame, double rate) noexcept {
    return static_cast<uint64_t>(static_cast<long double>(frame) * 1e9L / rate);
}
}
