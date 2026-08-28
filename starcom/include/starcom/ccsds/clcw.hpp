#pragma once

#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kClcwSize = 4;

// COP-1 CLCW. VCID here is the COP-1 virtual channel, not USLP MapId.
struct Clcw32 {
  std::uint8_t status = 0;           // 3-bit, mission
  std::uint8_t cop_in_effect = 0b01; // 2-bit; 01 = COP-1
  std::uint8_t vcid = 0;             // 6-bit
  bool no_rf_available = false;
  bool no_bit_lock = false;
  bool lockout = false;
  bool wait = false;
  bool retransmit = false;
  std::uint8_t farm_b_counter = 0;  // 2-bit
  std::uint8_t report_value = 0;    // N(R)
};

Result<Clcw32> decodeClcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodeClcw(std::span<std::byte> out,
                                Clcw32 const& fields) noexcept;

}  // namespace starcom::ccsds
