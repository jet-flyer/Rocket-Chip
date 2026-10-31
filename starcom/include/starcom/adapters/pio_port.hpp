#pragma once

#include "starcom/result.hpp"

#include <cstddef>
#include <span>

namespace starcom::adapters {

// PIO-shaped PLTU symbol pipe (IVP 17). Octets to bits, MSB first (same order
// as 211.2 Annex C). Caller owns the clock. No hardware/pio.h, no pin map.
// 211.1 Bi-Phase-L / residual-carrier PM is increment 18, not this port.
// Pio is a compile-time policy (P10-9). Required noexcept members:
// putBit(bool) -> void, getBit() -> bool. The pio object holds the call state.

template <class Pio>
ccsds::Result<std::size_t> pioShiftOut(Pio& pio,
                                         std::span<const std::byte> octets) noexcept {
  if (octets.empty()) {
    return std::size_t{0};
  }
  for (std::byte b : octets) {
    const auto v = static_cast<unsigned>(b);
    for (int i = 7; i >= 0; --i) {
      pio.putBit(((v >> i) & 1u) != 0);
    }
  }
  return octets.size();
}

template <class Pio>
ccsds::Result<std::size_t> pioShiftIn(Pio& pio, std::span<std::byte> out,
                                        std::size_t n_octets) noexcept {
  if (n_octets == 0) {
    return std::size_t{0};
  }
  if (n_octets > out.size()) {
    return tl::unexpected(ccsds::Error::buffer_too_small);
  }
  for (std::size_t i = 0; i < n_octets; ++i) {
    unsigned v = 0;
    for (int b = 0; b < 8; ++b) {
      v = (v << 1) | (pio.getBit() ? 1u : 0u);
    }
    out[i] = static_cast<std::byte>(v);
  }
  return n_octets;
}

}  // namespace starcom::adapters
