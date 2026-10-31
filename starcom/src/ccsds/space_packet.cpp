#include "starcom/ccsds/space_packet.hpp"

#include <algorithm>

namespace starcom::ccsds {

[[nodiscard]] Result<SpacePacketView> decodeSpacePacket(std::span<const std::byte> packet) noexcept {
  if (packet.size() < kSpacePacketMinSize) {
    return tl::unexpected(Error::sp_too_short);
  }
  const std::uint32_t b0 = std::to_integer<std::uint32_t>(packet[0]);
  if ((b0 >> 5) != 0U) {
    return tl::unexpected(Error::sp_pvn);
  }
  const std::uint32_t length_c = (std::to_integer<std::uint32_t>(packet[4]) << 8) |
                            std::to_integer<std::uint32_t>(packet[5]);
  const std::size_t data_len = static_cast<std::size_t>(length_c) + 1U;
  const std::size_t need = kSpacePacketHeaderSize + data_len;
  if (packet.size() < need) {
    return tl::unexpected(Error::sp_too_short);
  }

  SpacePacketView view{};
  view.fields.telecommand = (b0 & 0x10U) != 0;
  view.fields.secondary_header = (b0 & 0x08U) != 0;
  const std::uint32_t apid =
      ((b0 & 0x07U) << 8) | std::to_integer<std::uint32_t>(packet[1]);
  view.fields.apid = Apid{static_cast<std::uint16_t>(apid)};
  const std::uint32_t b2 = std::to_integer<std::uint32_t>(packet[2]);
  view.fields.seq_flags = static_cast<std::uint8_t>((b2 >> 6) & 0x03U);
  view.fields.seq_count = static_cast<std::uint16_t>(
      ((b2 & 0x3FU) << 8) | std::to_integer<std::uint32_t>(packet[3]));
  view.data = packet.subspan(kSpacePacketHeaderSize, data_len);
  return view;
}

[[nodiscard]] Result<std::size_t> encodeSpacePacket(std::span<std::byte> out,
                                        SpacePacketFields const& fields,
                                        std::span<const std::byte> data) noexcept {
  if (data.size() < 1 || data.size() > kSpacePacketDataMax) {
    return tl::unexpected(Error::sp_too_short);
  }
  const std::size_t need = kSpacePacketHeaderSize + data.size();
  if (out.size() < need) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const std::uint32_t apid = static_cast<std::uint32_t>(fields.apid) & 0x7FFU;
  const std::uint32_t seq = static_cast<std::uint32_t>(fields.seq_count) & 0x3FFFU;
  const std::uint32_t flags = static_cast<std::uint32_t>(fields.seq_flags) & 0x03U;
  const std::uint32_t c = static_cast<std::uint32_t>(data.size() - 1U);
  // 133.0-B-2 §4.1.3.3.3.4: Idle Packet secondary header flag is 0.
  const bool secondary =
      fields.secondary_header && apid != static_cast<std::uint32_t>(kIdleApid);

  out[0] = std::byte((fields.telecommand ? 0x10U : 0U) |
                     (secondary ? 0x08U : 0U) |
                     ((apid >> 8) & 0x07U));
  out[1] = std::byte(apid & 0xFFU);
  out[2] = std::byte((flags << 6) | ((seq >> 8) & 0x3FU));
  out[3] = std::byte(seq & 0xFFU);
  out[4] = std::byte((c >> 8) & 0xFFU);
  out[5] = std::byte(c & 0xFFU);
  std::copy(data.begin(), data.end(),
            out.begin() + static_cast<std::ptrdiff_t>(kSpacePacketHeaderSize));
  return need;
}

}  // namespace starcom::ccsds
