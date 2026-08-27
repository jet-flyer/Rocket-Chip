#pragma once

#include "starcom/ccsds/crc.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/result.hpp"

#include <array>
#include <cstddef>
#include <span>

namespace starcom::ccsds {

inline constexpr std::size_t kPltuAsmSize = 3;
inline constexpr std::size_t kPltuCrcSize = 4;

// 211.2 §3.2.3. Not covered by the CRC.
inline constexpr std::array<std::byte, kPltuAsmSize> kPltuAsm{
    std::byte{0xFA}, std::byte{0xF3}, std::byte{0x20}};

struct PltuView {
  std::span<const std::byte> frame;  // Transfer Frame, no ASM, no CRC
};

// Complete candidate starting at ASM. Locates CRC-32 via 211.2 §3.6.4 (V-3
// Frame Length). USLP TFVN 1100 is tfvn_unknown until increment 3.
Result<PltuView> decode_pltu(std::span<const std::byte> octets) noexcept;

// Writes ASM + frame + CRC-32. Envelope cap is kTransferFrameMin/Max.
Result<std::size_t> encode_pltu(std::span<std::byte> out,
                                std::span<const std::byte> frame) noexcept;

}  // namespace starcom::ccsds
