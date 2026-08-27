#include "starcom/ccsds/sp.hpp"

#include <algorithm>

namespace starcom::ccsds {

Result<SpView> decode_sp(std::span<const std::byte> packet) noexcept {
  if (packet.size() < kSpMinSize) {
    return tl::unexpected(Error::sp_too_short);
  }
  const unsigned b0 = std::to_integer<unsigned>(packet[0]);
  if ((b0 >> 5) != 0u) {
    return tl::unexpected(Error::sp_pvn);
  }
  const unsigned length_c = (std::to_integer<unsigned>(packet[4]) << 8) |
                            std::to_integer<unsigned>(packet[5]);
  const std::size_t data_len = static_cast<std::size_t>(length_c) + 1u;
  const std::size_t need = kSpHeaderSize + data_len;
  if (packet.size() < need) {
    return tl::unexpected(Error::sp_too_short);
  }

  SpView view{};
  view.fields.telecommand = (b0 & 0x10u) != 0;
  view.fields.secondary_header = (b0 & 0x08u) != 0;
  const unsigned apid =
      ((b0 & 0x07u) << 8) | std::to_integer<unsigned>(packet[1]);
  view.fields.apid = Apid{static_cast<std::uint16_t>(apid)};
  const unsigned b2 = std::to_integer<unsigned>(packet[2]);
  view.fields.seq_flags = static_cast<std::uint8_t>((b2 >> 6) & 0x03u);
  view.fields.seq_count = static_cast<std::uint16_t>(
      ((b2 & 0x3Fu) << 8) | std::to_integer<unsigned>(packet[3]));
  view.data = packet.subspan(kSpHeaderSize, data_len);
  return view;
}

Result<std::size_t> encode_sp(std::span<std::byte> out, SpFields const& fields,
                              std::span<const std::byte> data) noexcept {
  if (data.size() < 1 || data.size() > kSpDataMax) {
    return tl::unexpected(Error::sp_too_short);
  }
  const std::size_t need = kSpHeaderSize + data.size();
  if (out.size() < need) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const unsigned apid = static_cast<unsigned>(fields.apid) & 0x7FFu;
  const unsigned seq = static_cast<unsigned>(fields.seq_count) & 0x3FFFu;
  const unsigned flags = static_cast<unsigned>(fields.seq_flags) & 0x03u;
  const unsigned c = static_cast<unsigned>(data.size() - 1u);

  out[0] = std::byte((fields.telecommand ? 0x10u : 0u) |
                     (fields.secondary_header ? 0x08u : 0u) |
                     ((apid >> 8) & 0x07u));
  out[1] = std::byte(apid & 0xFFu);
  out[2] = std::byte((flags << 6) | ((seq >> 8) & 0x3Fu));
  out[3] = std::byte(seq & 0xFFu);
  out[4] = std::byte((c >> 8) & 0xFFu);
  out[5] = std::byte(c & 0xFFu);
  std::copy(data.begin(), data.end(),
            out.begin() + static_cast<std::ptrdiff_t>(kSpHeaderSize));
  return need;
}

}  // namespace starcom::ccsds
