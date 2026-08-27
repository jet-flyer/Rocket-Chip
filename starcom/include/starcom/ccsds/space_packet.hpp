#pragma once

#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kSpacePacketHeaderSize = 6;
inline constexpr std::size_t kSpacePacketMinSize = 7;  // header + mandatory 1-octet data field
inline constexpr std::size_t kSpacePacketDataMax = 65536;

struct SpacePacketFields {
  bool telecommand = false;       // Packet Type; 1 = telecommand
  bool secondary_header = false;
  Apid apid{};
  std::uint8_t seq_flags = 0b11;  // unsegmented
  std::uint16_t seq_count = 0;    // 14-bit
};

struct SpacePacketView {
  SpacePacketFields fields;
  std::span<const std::byte> data;  // Packet Data Field
};

Result<SpacePacketView> decode_space_packet(std::span<const std::byte> packet) noexcept;
Result<std::size_t> encode_space_packet(std::span<std::byte> out,
                                        SpacePacketFields const& fields,
                                        std::span<const std::byte> data) noexcept;

}  // namespace starcom::ccsds
