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

// Complete candidate starting at ASM. Locates CRC-32 via 211.2 §3.6.4:
// V-3 11-bit Frame Length, or non-truncated USLP 16-bit Frame Length.
// Truncated USLP (flag = 1) is Error::uslp_truncated (MIB length, IVP 9).
// uslp_truncated_len: 732.1 D1.3.2 Truncated Transfer Frame Length (MIB).
// 0 = not supplied; truncated USLP then returns Error::uslp_truncated.
Result<PltuView> decodePltu(std::span<const std::byte> octets,
                             std::size_t uslp_truncated_len = 0) noexcept;

// Writes ASM + frame + CRC-32. Envelope cap is kTransferFrameMin/Max.
Result<std::size_t> encodePltu(std::span<std::byte> out,
                                std::span<const std::byte> frame) noexcept;

// Regenerative: decodePltu must pass; copy ASM+frame+CRC bit-exact. No V-3
// or Space Packet decode. No COP. Trailing octets after a complete PLTU ignored.
Result<std::size_t> repeatPltu(std::span<std::byte> out,
                                std::span<const std::byte> octets) noexcept;

// 211.2 §3.6: search span for exact ASM. One PLTU per call. leftover is the
// caller's span (no library buffer). Handshake: ICD.
struct PltuHunt {
  std::size_t consumed;
  Result<PltuView> pltu;
};

PltuHunt huntPltu(std::span<const std::byte> octets,
                   std::size_t uslp_truncated_len = 0) noexcept;

// IVP 12: caller-owned store-and-forward. slots.size() is the depth — not a
// Starcom constant. seq is V-3 FSN or USLP VC Frame Count.
struct PltuRepeatSlot {
  std::span<std::byte> buf;
  std::size_t len = 0;
  std::uint64_t seq = 0;
};

struct PltuRepeatQ {
  std::span<PltuRepeatSlot> slots;
  std::size_t head = 0;
  std::size_t count = 0;
};

Result<std::size_t> enqueuePltu(PltuRepeatQ& q, std::span<const std::byte> octets,
                                 bool dedup) noexcept;
Result<std::size_t> dequeuePltu(PltuRepeatQ& q, std::span<std::byte> out) noexcept;

}  // namespace starcom::ccsds
