#include "starcom/ccsds/uslp.hpp"

#include "starcom/ccsds/crc.hpp"

#include <algorithm>

namespace starcom::ccsds {
namespace {

bool constructionHasPointer(std::uint8_t rule) noexcept {
  return rule <= 0b010u;
}

std::size_t primaryHeaderSize(std::uint8_t vcf_len) noexcept {
  return kUslpPrimaryHeaderMin + static_cast<std::size_t>(vcf_len);
}

std::size_t tfdfHeaderSize(std::uint8_t rule) noexcept {
  return constructionHasPointer(rule) ? 3u : 1u;
}

void storeBeN(std::span<std::byte> dest, std::uint64_t value) noexcept {
  for (std::size_t i = 0; i < dest.size(); ++i) {
    const auto shift = 8u * (dest.size() - 1u - i);
    dest[i] = std::byte(static_cast<unsigned>(value >> shift) & 0xFFu);
  }
}

std::uint64_t loadBeN(std::span<const std::byte> src) noexcept {
  std::uint64_t v = 0;
  for (auto b : src) {
    v = (v << 8) | std::to_integer<std::uint64_t>(b);
  }
  return v;
}

void storeBe16(std::span<std::byte> dest, std::uint16_t value) noexcept {
  dest[0] = std::byte((value >> 8) & 0xFFu);
  dest[1] = std::byte(value & 0xFFu);
}

void fillPrimaryIds(UslpView& view, std::span<const std::byte> frame) noexcept {
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

void writeUslpIds(std::span<std::byte> out, UslpFields const& fields,
                  bool truncated) noexcept {
  const unsigned scid = static_cast<unsigned>(fields.scid);
  const unsigned vcid = static_cast<unsigned>(fields.vcid) & 0x3Fu;
  const unsigned map = static_cast<unsigned>(fields.map_id) & 0x0Fu;
  out[0] = std::byte((kTfvnUslp << 4) | ((scid >> 12) & 0x0Fu));
  out[1] = std::byte((scid >> 4) & 0xFFu);
  out[2] = std::byte(((scid & 0x0Fu) << 4) | (fields.destination ? 0x08u : 0u) |
                     ((vcid >> 3) & 0x07u));
  out[3] = std::byte(((vcid & 0x07u) << 5) | (map << 1) | (truncated ? 0x01u : 0u));
}

Result<UslpView> decodeUslpTruncated(std::span<const std::byte> frame,
                                    UslpMib const& mib) noexcept {
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
  const std::size_t th = kUslpTfdfHeaderMin;
  if (flen < kUslpTruncatedHeaderSize + th) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  UslpView view{};
  fillPrimaryIds(view, frame);
  view.fields.truncated = true;
  view.fields.expedited = true;  // 732.1 D1.3 note 3: BD / expedited
  const auto tfdf = frame.subspan(kUslpTruncatedHeaderSize, th);
  const unsigned tfdf0 = std::to_integer<unsigned>(tfdf[0]);
  view.fields.tfdz_construction = static_cast<std::uint8_t>(tfdf0 >> 5);
  view.fields.upid = static_cast<std::uint8_t>(tfdf0 & 0x1Fu);
  view.tfdz = frame.subspan(kUslpTruncatedHeaderSize + th,
                            flen - kUslpTruncatedHeaderSize - th);
  return view;
}

bool fillTfdf(UslpView& view, std::span<const std::byte> body) noexcept {
  if (body.empty()) {
    return false;
  }
  const unsigned tfdf0 = std::to_integer<unsigned>(body[0]);
  view.fields.tfdz_construction = static_cast<std::uint8_t>(tfdf0 >> 5);
  view.fields.upid = static_cast<std::uint8_t>(tfdf0 & 0x1Fu);
  const std::size_t th = tfdfHeaderSize(view.fields.tfdz_construction);
  if (body.size() < th) {
    return false;
  }
  if (constructionHasPointer(view.fields.tfdz_construction)) {
    view.fields.tfdz_pointer =
        static_cast<std::uint16_t>((std::to_integer<unsigned>(body[1]) << 8) |
                                   std::to_integer<unsigned>(body[2]));
  }
  view.tfdz = body.subspan(th);
  return true;
}

bool checkFecf(UslpView& view, std::span<const std::byte> frame,
               std::size_t frame_len) noexcept {
  view.fecf = frame.subspan(frame_len - kUslpFecfSize, kUslpFecfSize);
  const auto covered = frame.subspan(0, frame_len - kUslpFecfSize);
  const std::uint16_t got =
      (std::to_integer<std::uint16_t>(view.fecf[0]) << 8) |
      std::to_integer<std::uint16_t>(view.fecf[1]);
  return crc16Fecf(covered) == got;
}

Result<std::size_t> uslpFrameLen(std::span<const std::byte> frame) noexcept {
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
  return frame_len;
}

Result<UslpView> decodeUslpNonTruncated(std::span<const std::byte> frame,
                                       UslpMib const& mib) noexcept {
  const auto flen = uslpFrameLen(frame);
  if (!flen) {
    return tl::unexpected(flen.error());
  }
  const std::size_t frame_len = *flen;
  if (frame.size() < kUslpPrimaryHeaderMin) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned b6 = std::to_integer<unsigned>(frame[6]);
  UslpView view{};
  fillPrimaryIds(view, frame);
  view.fields.expedited = (b6 & 0x80u) != 0;
  view.fields.protocol_control = (b6 & 0x40u) != 0;
  view.fields.ocf_present = (b6 & 0x08u) != 0;
  view.fields.vcf_count_len = static_cast<std::uint8_t>(b6 & 0x07u);
  const std::size_t ph = primaryHeaderSize(view.fields.vcf_count_len);
  const std::size_t iz = mib.insert_zone_length;
  std::size_t trailer = view.fields.ocf_present ? kUslpOcfSize : 0;
  if (mib.fecf_present) {
    trailer += kUslpFecfSize;
  }
  if (frame_len < ph + kUslpTfdfHeaderMin ||
      frame_len < ph + iz + kUslpTfdfHeaderMin + trailer) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (view.fields.vcf_count_len != 0) {
    view.fields.vcf_count =
        loadBeN(frame.subspan(kUslpPrimaryHeaderMin, view.fields.vcf_count_len));
  }
  if (iz != 0) {
    view.insert_zone = frame.subspan(ph, iz);
  }
  if (!fillTfdf(view, frame.subspan(ph + iz, frame_len - ph - iz - trailer))) {
    return tl::unexpected(Error::truncated);
  }
  if (view.fields.ocf_present) {
    view.ocf = frame.subspan(frame_len - trailer, kUslpOcfSize);
  }
  if (mib.fecf_present && !checkFecf(view, frame, frame_len)) {
    return tl::unexpected(Error::uslp_bad_fecf);
  }
  return view;
}

Result<std::size_t> encodeUslpTruncated(std::span<std::byte> out,
                                       UslpFields const& fields,
                                       std::span<const std::byte> tfdz,
                                       UslpMib const& mib) noexcept {
  if (mib.insert_zone_length != 0 || mib.fecf_present || fields.ocf_present ||
      tfdz.empty()) {
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
  writeUslpIds(out, fields, true);
  out[4] = std::byte((static_cast<unsigned>(kUslpConstructionNoSeg) << 5) |
                     (fields.upid & 0x1Fu));
  std::copy(tfdz.begin(), tfdz.end(), out.begin() + 5);
  return frame_len;
}

struct UslpLayout {
  std::uint8_t vcf_len;
  std::uint8_t rule;
  std::size_t ph;
  std::size_t th;
  std::size_t iz;
  std::size_t frame_len;
  std::size_t ocf_n;
  std::size_t fecf_n;
  bool want_ocf;
  bool fecf;
};

void writeUslpBody(std::span<std::byte> out, UslpFields const& fields,
                   UslpLayout const& lay, std::span<const std::byte> tfdz,
                   std::span<const std::byte> ocf,
                   std::span<const std::byte> insert) noexcept {
  const unsigned c = static_cast<unsigned>(lay.frame_len - 1u);
  writeUslpIds(out, fields, false);
  out[4] = std::byte((c >> 8) & 0xFFu);
  out[5] = std::byte(c & 0xFFu);
  out[6] = std::byte((fields.expedited ? 0x80u : 0u) |
                     (fields.protocol_control ? 0x40u : 0u) |
                     (lay.want_ocf ? 0x08u : 0u) | (lay.vcf_len & 0x07u));
  if (lay.vcf_len != 0) {
    storeBeN(out.subspan(kUslpPrimaryHeaderMin, lay.vcf_len), fields.vcf_count);
  }
  if (lay.iz != 0) {
    std::copy(insert.begin(), insert.end(),
              out.begin() + static_cast<std::ptrdiff_t>(lay.ph));
  }
  auto tfdf = out.subspan(lay.ph + lay.iz, lay.th);
  tfdf[0] = std::byte((static_cast<unsigned>(lay.rule) << 5) | (fields.upid & 0x1Fu));
  if (lay.th == 3) {
    tfdf[1] = std::byte((fields.tfdz_pointer >> 8) & 0xFFu);
    tfdf[2] = std::byte(fields.tfdz_pointer & 0xFFu);
  }
  if (!tfdz.empty()) {
    std::copy(tfdz.begin(), tfdz.end(),
              out.begin() + static_cast<std::ptrdiff_t>(lay.ph + lay.iz + lay.th));
  }
  if (lay.want_ocf) {
    std::copy(ocf.begin(), ocf.begin() + static_cast<std::ptrdiff_t>(kUslpOcfSize),
              out.begin() + static_cast<std::ptrdiff_t>(lay.frame_len - lay.fecf_n -
                                                        lay.ocf_n));
  }
  if (lay.fecf) {
    const auto covered =
        std::span<const std::byte>(out.data(), lay.frame_len - lay.fecf_n);
    storeBe16(out.subspan(lay.frame_len - lay.fecf_n, kUslpFecfSize),
              crc16Fecf(covered));
  }
}

}  // namespace

Result<UslpView> decodeUslp(std::span<const std::byte> frame,
                             UslpMib const& mib) noexcept {
  if (frame.size() < 4) {
    return tl::unexpected(Error::truncated);
  }
  const unsigned b0 = std::to_integer<unsigned>(frame[0]);
  if ((b0 >> 4) != kTfvnUslp) {
    return tl::unexpected(Error::tfvn_unknown);
  }
  if ((std::to_integer<unsigned>(frame[3]) & 0x01u) != 0) {
    return decodeUslpTruncated(frame, mib);
  }
  return decodeUslpNonTruncated(frame, mib);
}

Result<std::size_t> encodeUslp(std::span<std::byte> out, UslpFields const& fields,
                                std::span<const std::byte> tfdz,
                                std::span<const std::byte> ocf,
                                UslpMib const& mib,
                                std::span<const std::byte> insert) noexcept {
  if (fields.truncated) {
    if (!insert.empty() || !ocf.empty()) {
      return tl::unexpected(Error::uslp_length_oob);
    }
    return encodeUslpTruncated(out, fields, tfdz, mib);
  }
  std::uint8_t vcf_len = fields.vcf_count_len;
  if (vcf_len > 7u) {
    vcf_len = 7;
  }
  UslpLayout lay{};
  lay.vcf_len = vcf_len;
  lay.rule = static_cast<std::uint8_t>(fields.tfdz_construction & 0x07u);
  lay.ph = primaryHeaderSize(lay.vcf_len);
  lay.th = tfdfHeaderSize(lay.rule);
  lay.want_ocf = fields.ocf_present;
  if (lay.want_ocf && ocf.size() < kUslpOcfSize) {
    return tl::unexpected(Error::truncated);
  }
  if (mib.insert_zone_length != insert.size()) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  lay.ocf_n = lay.want_ocf ? kUslpOcfSize : 0;
  lay.fecf_n = mib.fecf_present ? kUslpFecfSize : 0;
  lay.iz = mib.insert_zone_length;
  lay.fecf = mib.fecf_present;
  lay.frame_len = lay.ph + lay.iz + lay.th + tfdz.size() + lay.ocf_n + lay.fecf_n;
  if (lay.frame_len < kUslpFrameMin || lay.frame_len > kUslpFrameMax) {
    return tl::unexpected(Error::uslp_length_oob);
  }
  if (out.size() < lay.frame_len) {
    return tl::unexpected(Error::buffer_too_small);
  }
  writeUslpBody(out, fields, lay, tfdz, ocf, insert);
  return lay.frame_len;
}

}  // namespace starcom::ccsds
