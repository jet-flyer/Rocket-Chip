#include "starcom/ccsds/uslp.hpp"

#include "starcom/ccsds/crc.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

bool construction_has_pointer(std::uint8_t rule) noexcept {
  return rule <= 0b010u;
}

std::size_t primary_header_size(std::uint8_t vcf_len) noexcept {
  return kUslpPrimaryHeaderMin + static_cast<std::size_t>(vcf_len);
}

std::size_t tfdf_header_size(std::uint8_t rule) noexcept {
  return construction_has_pointer(rule) ? 3u : 1u;
}

void store_be_n(std::span<std::byte> dest, std::uint64_t value) noexcept {
  for (std::size_t i = 0; i < dest.size(); ++i) {
    const auto shift = 8u * (dest.size() - 1u - i);
    dest[i] = std::byte(static_cast<unsigned>(value >> shift) & 0xFFu);
  }
}

std::uint64_t load_be_n(std::span<const std::byte> src) noexcept {
  std::uint64_t v = 0;
  for (auto b : src) {
    v = (v << 8) | std::to_integer<std::uint64_t>(b);
  }
  return v;
}

void store_be16(std::span<std::byte> dest, std::uint16_t value) noexcept {
  dest[0] = std::byte((value >> 8) & 0xFFu);
  dest[1] = std::byte(value & 0xFFu);
}

void fill_primary_ids(UslpView& view, std::span<const std::byte> frame) noexcept {
  const unsigned b0 = std::to_integer<unsigned>(frame[0]);
  const unsigned b2 = std::to_integer<unsigned>(frame[2]);
  const unsigned b3 = std::to_integer<unsigned>(frame[3]);
  const unsigned scid = ((b0 & 0x0Fu) << 12) |
                        (std::to_integer<unsigned>(frame[1]) << 4) | (b2 >> 4);
  view.fields.scid = UslpScid{static_cast<std::uint16_t>(scid)};
  view.fields.destination = (b2 & 0x08u) != 0;
  const unsigned vcid = ((b2 & 0x07u) << 3) | (b3 >> 5);
  view.fields.vcid = Vcid{static_cast<std::uint8_t>(vcid)};
  view.fields.map_id = MapId{static_cast<std::uint8_t>((b3 >> 1) & 0x0Fu)};
}

}  // namespace

Result<UslpView> decode_uslp(std::span<const std::byte> frame,
                             UslpMib const& mib) noexcept {
  if (frame.size() < 4) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned b0 = std::to_integer<unsigned>(frame[0]);
  if ((b0 >> 4) != kTfvnUslp) {
    return tl::unexpected(Error::tfvn_unknown);
  }
  const bool truncated_hdr = (std::to_integer<unsigned>(frame[3]) & 0x01u) != 0;
  if (truncated_hdr) {
    if (mib.truncated_frame_length == 0) {
      return tl::unexpected(Error::uslp_truncated);
    }
    const std::size_t flen = mib.truncated_frame_length;
    if (flen < kUslpTruncatedMin || flen > kUslpTruncatedMax) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    if (frame.size() < flen) {
      return tl::unexpected(Error::truncated);
    }
    UslpView view{};
    fill_primary_ids(view, frame);
    view.fields.truncated = true;
    view.fields.expedited = true;  // 732.1 D1.3 note 3: BD / expedited
    const std::size_t th = kUslpTfdfHeaderMin;
    if (flen < kUslpTruncatedHeaderSize + th) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    const auto tfdf = frame.subspan(kUslpTruncatedHeaderSize, th);
    const unsigned tfdf0 = std::to_integer<unsigned>(tfdf[0]);
    view.fields.tfdz_construction = static_cast<std::uint8_t>(tfdf0 >> 5);
    view.fields.upid = static_cast<std::uint8_t>(tfdf0 & 0x1Fu);
    view.tfdz = frame.subspan(kUslpTruncatedHeaderSize + th,
                              flen - kUslpTruncatedHeaderSize - th);
    return view;
  }

  if (frame.size() < 6) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned length_c = (std::to_integer<unsigned>(frame[4]) << 8) |
                            std::to_integer<unsigned>(frame[5]);
  const std::size_t frame_len = static_cast<std::size_t>(length_c) + 1u;
  if (frame_len < kUslpFrameMin || frame_len > kUslpFrameMax) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (frame.size() < frame_len) {
    return tl::unexpected(Error::truncated);
  }
  if (frame.size() < kUslpPrimaryHeaderMin) {
    return tl::unexpected(Error::truncated);
  }

  const unsigned b6 = std::to_integer<unsigned>(frame[6]);
  UslpView view{};
  fill_primary_ids(view, frame);
  view.fields.expedited = (b6 & 0x80u) != 0;
  view.fields.protocol_control = (b6 & 0x40u) != 0;
  view.fields.ocf_present = (b6 & 0x08u) != 0;
  view.fields.vcf_count_len = static_cast<std::uint8_t>(b6 & 0x07u);
  const std::size_t ph = primary_header_size(view.fields.vcf_count_len);
  if (frame_len < ph + kUslpTfdfHeaderMin) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (view.fields.vcf_count_len != 0) {
    view.fields.vcf_count =
        load_be_n(frame.subspan(kUslpPrimaryHeaderMin, view.fields.vcf_count_len));
  }

  const std::size_t iz = mib.insert_zone_length;
  std::size_t trailer = 0;
  if (view.fields.ocf_present) {
    trailer += kUslpOcfSize;
  }
  if (mib.fecf_present) {
    trailer += kUslpFecfSize;
  }
  if (frame_len < ph + iz + kUslpTfdfHeaderMin + trailer) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (iz != 0) {
    view.insert_zone = frame.subspan(ph, iz);
  }
  const auto body = frame.subspan(ph + iz, frame_len - ph - iz - trailer);
  if (body.empty()) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned tfdf0 = std::to_integer<unsigned>(body[0]);
  view.fields.tfdz_construction = static_cast<std::uint8_t>(tfdf0 >> 5);
  view.fields.upid = static_cast<std::uint8_t>(tfdf0 & 0x1Fu);
  const std::size_t th = tfdf_header_size(view.fields.tfdz_construction);
  if (body.size() < th) {
    return tl::unexpected(Error::truncated);
  }
  if (construction_has_pointer(view.fields.tfdz_construction)) {
    view.fields.tfdz_pointer =
        static_cast<std::uint16_t>((std::to_integer<unsigned>(body[1]) << 8) |
                                   std::to_integer<unsigned>(body[2]));
  }
  view.tfdz = body.subspan(th);
  if (view.fields.ocf_present) {
    view.ocf = frame.subspan(frame_len - trailer, kUslpOcfSize);
  }
  if (mib.fecf_present) {
    view.fecf = frame.subspan(frame_len - kUslpFecfSize, kUslpFecfSize);
    const auto covered = frame.subspan(0, frame_len - kUslpFecfSize);
    const std::uint16_t got =
        (std::to_integer<std::uint16_t>(view.fecf[0]) << 8) |
        std::to_integer<std::uint16_t>(view.fecf[1]);
    if (crc16_fecf(covered) != got) {
      return tl::unexpected(Error::uslp_bad_fecf);
    }
  }
  return view;
}

Result<std::size_t> encode_uslp(std::span<std::byte> out, UslpFields const& fields,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf,
                                UslpMib const& mib,
                                std::span<const std::byte> insert) noexcept {
  if (fields.truncated) {
    if (mib.insert_zone_length != 0 || mib.fecf_present || fields.ocf_present ||
        !insert.empty() || !ocf.empty()) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    if (tfdz.empty()) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    const std::size_t frame_len =
        kUslpTruncatedHeaderSize + kUslpTfdfHeaderMin + tfdz.size();
    if (frame_len < kUslpTruncatedMin || frame_len > kUslpTruncatedMax) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    if (mib.truncated_frame_length != 0 &&
        mib.truncated_frame_length != frame_len) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    if (out.size() < frame_len) {
      return tl::unexpected(Error::buffer_too_small);
    }
    const unsigned scid = static_cast<unsigned>(fields.scid);
    const unsigned vcid = static_cast<unsigned>(fields.vcid) & 0x3Fu;
    const unsigned map = static_cast<unsigned>(fields.map_id) & 0x0Fu;
    out[0] = std::byte((kTfvnUslp << 4) | ((scid >> 12) & 0x0Fu));
    out[1] = std::byte((scid >> 4) & 0xFFu);
    out[2] = std::byte(((scid & 0x0Fu) << 4) | (fields.destination ? 0x08u : 0u) |
                       ((vcid >> 3) & 0x07u));
    out[3] = std::byte(((vcid & 0x07u) << 5) | (map << 1) | 0x01u);
    out[4] = std::byte((static_cast<unsigned>(kUslpConstructionNoSeg) << 5) |
                       (fields.upid & 0x1Fu));
    std::copy(tfdz.begin(), tfdz.end(), out.begin() + 5);
    return frame_len;
  }

  std::uint8_t vcf_len = fields.vcf_count_len;
  if (vcf_len > 7u) {
    vcf_len = 7;
  }
  const std::uint8_t rule = static_cast<std::uint8_t>(fields.tfdz_construction & 0x07u);
  const std::size_t ph = primary_header_size(vcf_len);
  const std::size_t th = tfdf_header_size(rule);
  const bool want_ocf = fields.ocf_present;
  if (want_ocf && ocf.size() < kUslpOcfSize) {
    return tl::unexpected(Error::truncated);
  }
  if (mib.insert_zone_length != insert.size()) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  const std::size_t ocf_n = want_ocf ? kUslpOcfSize : 0;
  const std::size_t fecf_n = mib.fecf_present ? kUslpFecfSize : 0;
  const std::size_t iz = mib.insert_zone_length;
  const std::size_t frame_len = ph + iz + th + tfdz.size() + ocf_n + fecf_n;
  if (frame_len < kUslpFrameMin || frame_len > kUslpFrameMax) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (out.size() < frame_len) {
    return tl::unexpected(Error::buffer_too_small);
  }

  const unsigned scid = static_cast<unsigned>(fields.scid);
  const unsigned vcid = static_cast<unsigned>(fields.vcid) & 0x3Fu;
  const unsigned map = static_cast<unsigned>(fields.map_id) & 0x0Fu;
  const unsigned c = static_cast<unsigned>(frame_len - 1u);

  out[0] = std::byte((kTfvnUslp << 4) | ((scid >> 12) & 0x0Fu));
  out[1] = std::byte((scid >> 4) & 0xFFu);
  out[2] = std::byte(((scid & 0x0Fu) << 4) | (fields.destination ? 0x08u : 0u) |
                     ((vcid >> 3) & 0x07u));
  out[3] = std::byte(((vcid & 0x07u) << 5) | (map << 1));  // truncated flag = 0
  out[4] = std::byte((c >> 8) & 0xFFu);
  out[5] = std::byte(c & 0xFFu);
  out[6] = std::byte((fields.expedited ? 0x80u : 0u) |
                     (fields.protocol_control ? 0x40u : 0u) |
                     (want_ocf ? 0x08u : 0u) | (vcf_len & 0x07u));
  if (vcf_len != 0) {
    store_be_n(out.subspan(kUslpPrimaryHeaderMin, vcf_len), fields.vcf_count);
  }
  if (iz != 0) {
    std::copy(insert.begin(), insert.end(),
              out.begin() + static_cast<std::ptrdiff_t>(ph));
  }
  auto tfdf = out.subspan(ph + iz, th);
  tfdf[0] = std::byte((static_cast<unsigned>(rule) << 5) | (fields.upid & 0x1Fu));
  if (th == 3) {
    tfdf[1] = std::byte((fields.tfdz_pointer >> 8) & 0xFFu);
    tfdf[2] = std::byte(fields.tfdz_pointer & 0xFFu);
  }
  if (!tfdz.empty()) {
    std::copy(tfdz.begin(), tfdz.end(),
              out.begin() + static_cast<std::ptrdiff_t>(ph + iz + th));
  }
  if (want_ocf) {
    std::copy(ocf.begin(), ocf.begin() + static_cast<std::ptrdiff_t>(kUslpOcfSize),
              out.begin() + static_cast<std::ptrdiff_t>(frame_len - fecf_n - ocf_n));
  }
  if (mib.fecf_present) {
    const auto covered = std::span<const std::byte>(out.data(), frame_len - fecf_n);
    store_be16(out.subspan(frame_len - fecf_n, kUslpFecfSize), crc16_fecf(covered));
  }
  return frame_len;
}

}  // namespace starcom::ccsds
