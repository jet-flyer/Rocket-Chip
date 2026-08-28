#include "starcom/ccsds/conv.hpp"

#include <array>
#include <bit>

namespace starcom::ccsds {
namespace {

// 131.0-B-5 §3.3: G1 = 171 octal = 0b1111001, G2 = 133 octal = 0b1011011.
constexpr unsigned kG1 = 0b1111001u;
constexpr unsigned kG2 = 0b1011011u;

void encodeBit(ConvEnc& enc, unsigned in_bit, unsigned& c1, unsigned& c2) noexcept {
  // 7-bit register: bit6 = input, bits5..0 = delay line (newest..oldest).
  const unsigned v = ((in_bit & 1u) << 6) | (enc.mem & 0x3Fu);
  c1 = std::popcount(v & kG1) & 1u;
  c2 = (std::popcount(v & kG2) & 1u) ^ 1u;  // G2 symbol inversion
  enc.mem = static_cast<std::uint8_t>(v >> 1);
}

}  // namespace

Result<std::size_t> convEncode(std::span<std::byte> out,
                                std::span<const std::byte> in,
                                ConvEnc& enc) noexcept {
  const std::size_t need = in.size() * 2u;
  if (out.size() < need) {
    return tl::unexpected(Error::buffer_too_small);
  }
  std::size_t o = 0;
  for (std::byte octet : in) {
    unsigned acc = 0;
    unsigned nbits = 0;
    const unsigned value = std::to_integer<unsigned>(octet);
    for (int i = 7; i >= 0; --i) {
      unsigned c1 = 0;
      unsigned c2 = 0;
      encodeBit(enc, (value >> i) & 1u, c1, c2);
      acc = (acc << 2) | (c1 << 1) | c2;
      nbits += 2;
      if (nbits == 8) {
        out[o++] = std::byte{static_cast<std::uint8_t>(acc)};
        acc = 0;
        nbits = 0;
      }
    }
  }
  return need;
}

Result<std::size_t> convEncode(std::span<std::byte> out,
                                std::span<const std::byte> in) noexcept {
  ConvEnc enc{};
  return convEncode(out, in, enc);
}

Result<std::size_t> convEncodeStep(std::span<std::byte> out, std::byte in,
                                     ConvEnc& enc) noexcept {
  std::array<std::byte, 1> one{in};
  return convEncode(out, one, enc);
}

}  // namespace starcom::ccsds
