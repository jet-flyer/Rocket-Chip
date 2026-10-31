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

// CCSDS 133.0-B-2 4.1.3.2.1. Packet Version Number is bits 0-2.
inline constexpr std::uint32_t kSpVersionShift = 5U;
// CCSDS 133.0-B-2 4.1.3.3.4.1. APID is bits 5-15.
inline constexpr std::uint32_t kSpApidHighMask = 0x07U;
inline constexpr std::uint32_t kSpApidMask = 0x7FFU;
// CCSDS 133.0-B-2 4.1.3.4.2.1. Sequence Flags are bits 16-17.
inline constexpr std::uint32_t kSpSeqFlagsShift = 6U;
// CCSDS 133.0-B-2 4.1.3.4.1.2. Packet Sequence Count is 14 bits.
inline constexpr std::uint32_t kSpSeqCountHighMask = 0x3FU;

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

[[nodiscard]] Result<SpacePacketView> decodeSpacePacket(std::span<const std::byte> packet) noexcept;
[[nodiscard]] Result<std::size_t> encodeSpacePacket(std::span<std::byte> out,
                                        SpacePacketFields const& fields,
                                        std::span<const std::byte> data) noexcept;

}  // namespace starcom::ccsds
