#include "starcom/ccsds/crc.hpp"

#include "starcom/ccsds/types.hpp"

namespace starcom::ccsds {
namespace {

// 211.2-B-3 Annex C C1.3: G(X) without the X^32 term.
constexpr std::uint32_t kPoly = 0x00A00805U;

}  // namespace

std::uint32_t crc32(std::span<const std::byte> frame) noexcept {
  // R(X) = X^32 · M(X) mod G(X). First transferred bit is M0, then R0 (MSB).
  // Shift register preset all-zero (Annex C note, unlike CCSDS CRC-16).
  std::uint32_t crc = 0;
  for (std::byte octet : frame) {
    const std::uint32_t value = std::to_integer<std::uint32_t>(octet);
    for (std::int32_t i = static_cast<std::int32_t>(kOctetMsb); i >= 0; --i) {
      const std::uint32_t bit = (value >> i) & 1U;
      const std::uint32_t msb = (crc >> 31) & 1U;
      crc <<= 1;
      if ((msb ^ bit) != 0U) {
        crc ^= kPoly;
      }
    }
  }
  return crc;
}

std::uint16_t crc16Fecf(std::span<const std::byte> covered) noexcept {
  // 732.1 Annex B: G(X) = X^16 + X^12 + X^5 + 1; L(X) presets all-ones.
  constexpr std::uint16_t kPoly16 = 0x1021U;
  std::uint16_t crc = kCrc16Init;
  for (std::byte octet : covered) {
    const std::uint32_t value = std::to_integer<std::uint32_t>(octet);
    for (std::int32_t i = static_cast<std::int32_t>(kOctetMsb); i >= 0; --i) {
      const std::uint32_t bit = (value >> i) & 1U;
      const std::uint32_t msb = (crc >> 15) & 1U;
      crc = static_cast<std::uint16_t>(crc << 1);
      if ((msb ^ bit) != 0U) {
        crc = static_cast<std::uint16_t>(crc ^ kPoly16);
      }
    }
  }
  return crc;
}

}  // namespace starcom::ccsds
