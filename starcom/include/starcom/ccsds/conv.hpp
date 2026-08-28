#pragma once

#include "starcom/result.hpp"

#include <cstddef>
#include <cstdint>
#include <span>

namespace starcom::ccsds {

// 211.2 §3.4.3 → 131.0-B-5 §3.3. Rate 1/2, K=7, non-punctured.
// G1 = 171 octal, G2 = 133 octal, G2 inverted. C1 then C2. MSB-first.
// Encodes the whole C&S stream (ASM and Idle included). Decode is not this
// increment (GCS/Pi).

struct ConvEnc {
  std::uint8_t mem = 0;  // 6 delay elements; bit5 = newest, bit0 = oldest
};

// One-shot. Encoder memory starts at 0. out size = 2 * in size.
Result<std::size_t> convEncode(std::span<std::byte> out,
                                std::span<const std::byte> in) noexcept;

// Continues memory across calls (ASM+PLTU+Idle as one stream).
Result<std::size_t> convEncode(std::span<std::byte> out,
                                std::span<const std::byte> in,
                                ConvEnc& enc) noexcept;

// One input octet → two output octets; updates enc.
Result<std::size_t> convEncodeStep(std::span<std::byte> out, std::byte in,
                                     ConvEnc& enc) noexcept;

}  // namespace starcom::ccsds
