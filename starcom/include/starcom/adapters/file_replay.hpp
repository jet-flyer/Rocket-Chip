#pragma once

#include "starcom/adapters/loopback.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <span>

namespace starcom::adapters {

using PltuSink = ccsds::Result<std::size_t> (*)(void* ctx,
                                                std::span<const std::byte> pltu);

// Caller owns path and scratch. scratch.size() must be >= kAdapterFrameMax.
ccsds::Result<std::size_t> replay_pltu_file(char const* path,
                                            std::span<std::byte> scratch,
                                            PltuSink sink, void* ctx) noexcept;

}  // namespace starcom::adapters
