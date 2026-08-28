#pragma once

#include "starcom/result.hpp"

#include <cstddef>
#include <span>

namespace starcom::adapters {

// PIO-shaped PLTU symbol pipe (IVP 17). Octets to bits, MSB first (same order
// as 211.2 Annex C). Caller owns the clock. No hardware/pio.h, no pin map.
// 211.1 Bi-Phase-L / residual-carrier PM is increment 18, not this port.
struct PioOps {
  void* ctx = nullptr;
  void (*put_bit)(void* ctx, bool bit) noexcept = nullptr;
  bool (*get_bit)(void* ctx) noexcept = nullptr;
};

ccsds::Result<std::size_t> pio_shift_out(PioOps const& pio,
                                         std::span<const std::byte> octets) noexcept;
ccsds::Result<std::size_t> pio_shift_in(PioOps const& pio, std::span<std::byte> out,
                                        std::size_t n_octets) noexcept;

}  // namespace starcom::adapters
