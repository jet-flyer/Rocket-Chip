#include "starcom/ccsds/v3.hpp"

#include <algorithm>

namespace starcom::ccsds {

[[nodiscard]] Result<V3View> decodeV3(std::span<const std::byte> frame) noexcept {
  if (frame.size() < kV3HeaderSize) {
    return tl::unexpected(Error::truncated);
  }
  const std::uint32_t b0 = std::to_integer<std::uint32_t>(frame[0]);
  if ((b0 >> kV3TfvnShift) != kTfvnV3) {
    return tl::unexpected(Error::tfvn_unknown);
  }
  const std::uint32_t length_c =
      ((std::to_integer<std::uint32_t>(frame[2]) & kV3LengthHighMask) << 8) |
      std::to_integer<std::uint32_t>(frame[3]);
  const std::size_t frame_len = static_cast<std::size_t>(length_c) + 1U;
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  if (frame.size() < frame_len) {
    return tl::unexpected(Error::truncated);
  }

  V3View view{};
  view.fields.qos_expedited = (b0 & 0x20U) != 0;
  view.fields.p_frame = (b0 & 0x10U) != 0;
  view.fields.dfc_id = static_cast<std::uint8_t>((b0 >> 2) & 0x03U);
  const std::uint32_t scid = ((b0 & 0x03U) << 8) | std::to_integer<std::uint32_t>(frame[1]);
  view.fields.scid = Scid{static_cast<std::uint16_t>(scid)};
  const std::uint32_t b2 = std::to_integer<std::uint32_t>(frame[2]);
  view.fields.pcid = Pcid{static_cast<std::uint8_t>((b2 >> kV3PcidShift) & 0x01U)};
  view.fields.port_id = PortId{static_cast<std::uint8_t>((b2 >> 4) & kV3PortMask)};
  view.fields.destination = (b2 & 0x08U) != 0;
  view.fields.fsn = std::to_integer<std::uint8_t>(frame[4]);
  view.data = frame.subspan(kV3HeaderSize, frame_len - kV3HeaderSize);
  return view;
}

[[nodiscard]] Result<std::size_t> encodeV3(std::span<std::byte> out, V3Fields const& fields,
                              std::span<const std::byte> data) noexcept {
  const std::size_t frame_len = kV3HeaderSize + data.size();
  if (frame_len < kTransferFrameMin || frame_len > kTransferFrameMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  if (out.size() < frame_len) {
    return tl::unexpected(Error::buffer_too_small);
  }
  const std::uint32_t scid = static_cast<std::uint32_t>(fields.scid) & 0x3FFU;
  const std::uint32_t c = static_cast<std::uint32_t>(frame_len - 1U);
  std::uint32_t dfc = static_cast<std::uint32_t>(fields.dfc_id) & 0x03U;
  const std::uint32_t pcid = static_cast<std::uint32_t>(fields.pcid) & 0x01U;
  std::uint32_t port = static_cast<std::uint32_t>(fields.port_id) & kV3PortMask;
  if (fields.p_frame) {
    dfc = kDfcPackets;  // 3.2.2.5.2
    port = 0U;          // 3.2.2.8.2
  }

  out[0] = std::byte((kTfvnV3 << kV3TfvnShift) | (fields.qos_expedited ? 0x20U : 0U) |
                     (fields.p_frame ? 0x10U : 0U) | (dfc << 2) |
                     ((scid >> 8) & 0x03U));
  out[1] = std::byte(scid & 0xFFU);
  out[2] = std::byte((pcid << kV3PcidShift) | (port << 4) |
                     (fields.destination ? 0x08U : 0U) | ((c >> 8) & kV3LengthHighMask));
  out[3] = std::byte(c & 0xFFU);
  out[4] = std::byte(fields.fsn);
  if (!data.empty()) {
    std::copy(data.begin(), data.end(),
              out.begin() + static_cast<std::ptrdiff_t>(kV3HeaderSize));
  }
  return frame_len;
}

[[nodiscard]] Result<std::size_t> encodeV3UserDefined(std::span<std::byte> out,
                                          V3Fields const& fields,
                                          std::span<const std::byte> data) noexcept {
  if (data.size() > kV3DataMax) {
    return tl::unexpected(Error::v3_length_oob);
  }
  V3Fields f = fields;
  f.p_frame = false;
  f.dfc_id = kDfcUserDefined;
  return encodeV3(out, f, data);
}

}  // namespace starcom::ccsds
