#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kSpHeaderSize = 6;
inline constexpr std::size_t kSpMinSize = 7;       // header + mandatory 1-octet data field
inline constexpr std::size_t kSpDataMax = 65536;   // Packet Data Length is 16-bit, C = N-1

struct SpFields {
  bool telecommand = false;       // Packet Type; 1 = telecommand
  bool secondary_header = false;
  Apid apid{};
  std::uint8_t seq_flags = 0b11;  // unsegmented
  std::uint16_t seq_count = 0;    // 14-bit
};

struct SpView {
  SpFields fields;
  std::span<const std::byte> data;  // Packet Data Field
};

Result<SpView> decode_sp(std::span<const std::byte> packet) noexcept;
Result<std::size_t> encode_sp(std::span<std::byte> out, SpFields const& fields,
                              std::span<const std::byte> data) noexcept;

}  // namespace starcom::ccsds
