#include "starcom/ccsds/plcw.hpp"

namespace starcom::ccsds {

Result<Plcw16> decodePlcw(std::span<const std::byte> octets) noexcept {
  if (octets.size() < kPlcwSize) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned hi = std::to_integer<unsigned>(octets[0]);
  Plcw16 w{};
  w.retransmit = (hi & 0x20u) != 0;
  w.pcid = Pcid{static_cast<std::uint8_t>((hi >> 4) & 0x01u)};
  w.expedited_counter = static_cast<std::uint8_t>(hi & 0x07u);
  w.report_value = std::to_integer<std::uint8_t>(octets[1]);
  return w;
}

Result<std::size_t> encodePlcw(std::span<std::byte> out,
                                Plcw16 const& fields) noexcept {
  if (out.size() < kPlcwSize) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const unsigned pcid = static_cast<unsigned>(fields.pcid) & 0x01u;
  const unsigned exp = static_cast<unsigned>(fields.expedited_counter) & 0x07u;
  // Format ID 1, Type ID 0, spare 0 (211.0 Fig 3-5).
  out[0] = std::byte(0x80u | (fields.retransmit ? 0x20u : 0u) | (pcid << 4) | exp);
  out[1] = std::byte(fields.report_value);
  return kPlcwSize;
}

}  // namespace starcom::ccsds
