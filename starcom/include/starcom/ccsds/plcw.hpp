#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kPlcwSize = 2;

struct Plcw16 {
  bool retransmit = false;
  Pcid pcid{};
  std::uint8_t expedited_counter = 0;  // 3-bit
  std::uint8_t report_value = 0;       // V(R)
};

Result<Plcw16> decodePlcw(std::span<const std::byte> octets) noexcept;
Result<std::size_t> encodePlcw(std::span<std::byte> out,
                                Plcw16 const& fields) noexcept;

}  // namespace starcom::ccsds
