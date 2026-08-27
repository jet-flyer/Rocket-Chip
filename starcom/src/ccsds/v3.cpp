#include "starcom/ccsds/v3.hpp"

#include <algorithm>

namespace starcom::ccsds {

Result<V3View> decode_v3(std::span<const std::byte> frame) noexcept {
  if (frame.size() < kV3HeaderSize) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned b0 = std::to_integer<unsigned>(frame[0]);
  if ((b0 >> 6) != kTfvnV3) {
    return tl::unexpected(Error::tfvn_unknown);
  }
  const unsigned length_c =
      ((std::to_integer<unsigned>(frame[2]) & 0x07u) << 8) |
      std::to_integer<unsigned>(frame[3]);
  const std::size_t frame_len = static_cast<std::size_t>(length_c) + 1u;
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  if (frame.size() < frame_len) {
    return tl::unexpected(Error::truncated);
  }

  V3View view{};
  view.fields.qos_expedited = (b0 & 0x20u) != 0;
  view.fields.p_frame = (b0 & 0x10u) != 0;
  view.fields.dfc_id = static_cast<std::uint8_t>((b0 >> 2) & 0x03u);
  const unsigned scid = ((b0 & 0x03u) << 8) | std::to_integer<unsigned>(frame[1]);
  view.fields.scid = Scid{static_cast<std::uint16_t>(scid)};
  const unsigned b2 = std::to_integer<unsigned>(frame[2]);
  view.fields.pcid = Pcid{static_cast<std::uint8_t>((b2 >> 7) & 0x01u)};
  view.fields.port_id = PortId{static_cast<std::uint8_t>((b2 >> 4) & 0x07u)};
  view.fields.destination = (b2 & 0x08u) != 0;
  view.fields.fsn = std::to_integer<std::uint8_t>(frame[4]);
  view.data = frame.subspan(kV3HeaderSize, frame_len - kV3HeaderSize);
  return view;
}

Result<std::size_t> encode_v3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept {
  const std::size_t frame_len = kV3HeaderSize + data.size();
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  if (out.size() < frame_len) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const unsigned scid = static_cast<unsigned>(fields.scid) & 0x3FFu;
  const unsigned c = static_cast<unsigned>(frame_len - 1u);
  const unsigned dfc = static_cast<unsigned>(fields.dfc_id) & 0x03u;
  const unsigned pcid = static_cast<unsigned>(fields.pcid) & 0x01u;
  const unsigned port = static_cast<unsigned>(fields.port_id) & 0x07u;

  out[0] = std::byte((kTfvnV3 << 6) | (fields.qos_expedited ? 0x20u : 0u) |
                     (fields.p_frame ? 0x10u : 0u) | (dfc << 2) |
                     ((scid >> 8) & 0x03u));
  out[1] = std::byte(scid & 0xFFu);
  out[2] = std::byte((pcid << 7) | (port << 4) |
                     (fields.destination ? 0x08u : 0u) | ((c >> 8) & 0x07u));
  out[3] = std::byte(c & 0xFFu);
  out[4] = std::byte(fields.fsn);
  if (!data.empty()) {
    std::copy(data.begin(), data.end(),
              out.begin() + static_cast<std::ptrdiff_t>(kV3HeaderSize));
  }
  return frame_len;
}

}  // namespace starcom::ccsds
