#pragma once

#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace starcom::adapters {

// One complete PLTU. Envelope sizes from 211.2 Fig 3-1 (ASM + max frame + CRC-32).
inline constexpr std::size_t kAdapterFrameMax =
    ccsds::kPltuAsmSize + ccsds::kTransferFrameMax + ccsds::kPltuCrcSize;

// One outstanding unit. Extra buffering is the caller's (same idea as FOP-1 Wait_Queue).
struct FrameSlot {
  std::array<std::byte, kAdapterFrameMax> buf{};
  std::size_t n = 0;
};

ccsds::Result<std::size_t> slot_write(FrameSlot& slot,
                                      std::span<const std::byte> octets) noexcept;
ccsds::Result<std::size_t> slot_read(FrameSlot& slot, std::span<std::byte> out) noexcept;

// Two simplex lanes. No sockets. Core still never sees this type.
struct HostLoopback {
  FrameSlot a_to_b{};
  FrameSlot b_to_a{};
};

}  // namespace starcom::adapters
