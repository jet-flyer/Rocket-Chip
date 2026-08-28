#include "starcom/ccsds/crc.hpp"

namespace starcom::ccsds {
namespace {

// 211.2-B-3 Annex C C1.3: G(X) without the X^32 term.
constexpr std::uint32_t kPoly = 0x00A00805u;

}  // namespace

std::uint32_t crc32(std::span<const std::byte> frame) noexcept {
  // R(X) = X^32 · M(X) mod G(X). First transferred bit is M0, then R0 (MSB).
  // Shift register preset all-zero (Annex C note, unlike CCSDS CRC-16).
  std::uint32_t crc = 0;
  for (std::byte octet : frame) {
    const unsigned value = std::to_integer<unsigned>(octet);
    for (int i = 7; i >= 0; --i) {
      const unsigned bit = (value >> i) & 1u;
      const unsigned msb = (crc >> 31) & 1u;
      crc <<= 1;
      if ((msb ^ bit) != 0u) {
        crc ^= kPoly;
      }
    }
  }
  return crc;
}

std::uint16_t crc16Fecf(std::span<const std::byte> covered) noexcept {
  // 732.1 Annex B: G(X) = X^16 + X^12 + X^5 + 1; L(X) presets all-ones.
  constexpr std::uint16_t kPoly16 = 0x1021u;
  std::uint16_t crc = 0xFFFFu;
  for (std::byte octet : covered) {
    const unsigned value = std::to_integer<unsigned>(octet);
    for (int i = 7; i >= 0; --i) {
      const unsigned bit = (value >> i) & 1u;
      const unsigned msb = (crc >> 15) & 1u;
      crc = static_cast<std::uint16_t>(crc << 1);
      if ((msb ^ bit) != 0u) {
        crc = static_cast<std::uint16_t>(crc ^ kPoly16);
      }
    }
  }
  return crc;
}

}  // namespace starcom::ccsds
