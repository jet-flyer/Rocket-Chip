#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 211.2-B-3 Annex C (normative). Covers the Transfer Frame only, never the ASM.
// G(X) = X^32 + X^23 + X^21 + X^11 + X^2 + 1; shift register init all-zero.
// Return is the on-wire remainder; first transmitted bit is the MSB.
std::uint32_t crc32(std::span<const std::byte> frame) noexcept;

// 732.1-B-3 Annex B (USLP FECF). Covers the Transfer Frame except the 2-octet
// FECF. G(X) = X^16 + X^12 + X^5 + 1; shift register init all-ones.
std::uint16_t crc16_fecf(std::span<const std::byte> covered) noexcept;

}  // namespace starcom::ccsds
