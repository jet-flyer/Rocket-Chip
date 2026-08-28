#include "starcom/ccsds/clcw.hpp"

namespace starcom::ccsds {

Result<Clcw32> decodeClcw(std::span<const std::byte> octets) noexcept {
  if (octets.size() < kClcwSize) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned b0 = std::to_integer<unsigned>(octets[0]);
  const unsigned b1 = std::to_integer<unsigned>(octets[1]);
  const unsigned b2 = std::to_integer<unsigned>(octets[2]);
  Clcw32 w{};
  w.status = static_cast<std::uint8_t>((b0 >> 2) & 0x07u);
  w.cop_in_effect = static_cast<std::uint8_t>(b0 & 0x03u);
  w.vcid = static_cast<std::uint8_t>((b1 >> 2) & 0x3Fu);
  w.no_rf_available = (b2 & 0x80u) != 0;
  w.no_bit_lock = (b2 & 0x40u) != 0;
  w.lockout = (b2 & 0x20u) != 0;
  w.wait = (b2 & 0x10u) != 0;
  w.retransmit = (b2 & 0x08u) != 0;
  w.farm_b_counter = static_cast<std::uint8_t>((b2 >> 1) & 0x03u);
  w.report_value = std::to_integer<std::uint8_t>(octets[3]);
  return w;
}

Result<std::size_t> encodeClcw(std::span<std::byte> out,
                                Clcw32 const& fields) noexcept {
  if (out.size() < kClcwSize) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const unsigned status = static_cast<unsigned>(fields.status) & 0x07u;
  const unsigned cop = static_cast<unsigned>(fields.cop_in_effect) & 0x03u;
  const unsigned vcid = static_cast<unsigned>(fields.vcid) & 0x3Fu;
  const unsigned farm = static_cast<unsigned>(fields.farm_b_counter) & 0x03u;
  // Control Word Type 0, CLCW version 00 (232.0 Fig 4-6).
  out[0] = std::byte((status << 2) | cop);
  out[1] = std::byte(vcid << 2);  // spare bits 14–15 = 00
  out[2] = std::byte((fields.no_rf_available ? 0x80u : 0u) |
                     (fields.no_bit_lock ? 0x40u : 0u) |
                     (fields.lockout ? 0x20u : 0u) | (fields.wait ? 0x10u : 0u) |
                     (fields.retransmit ? 0x08u : 0u) | (farm << 1));
  out[3] = std::byte(fields.report_value);
  return kClcwSize;
}

}  // namespace starcom::ccsds
