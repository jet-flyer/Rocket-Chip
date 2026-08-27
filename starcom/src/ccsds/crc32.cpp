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

}  // namespace starcom::ccsds
