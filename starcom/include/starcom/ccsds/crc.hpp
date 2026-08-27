#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 211.2-B-3 Annex C (normative). Covers the Transfer Frame only, never the ASM.
// G(X) = X^32 + X^23 + X^21 + X^11 + X^2 + 1; shift register init all-zero.
// Return is the on-wire remainder; first transmitted bit is the MSB.
std::uint32_t crc32(std::span<const std::byte> frame) noexcept;

}  // namespace starcom::ccsds
