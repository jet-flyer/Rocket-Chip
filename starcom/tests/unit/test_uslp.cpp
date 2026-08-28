// IVP increment 3 + 9 — USLP Version-4 (732.1 §4.1 / annex D / annex B).

#include "heap_trap.hpp"
#include "starcom/ccsds/clcw.hpp"
#include "starcom/ccsds/crc.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/uslp.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::Clcw32;
using starcom::ccsds::decodePltu;
using starcom::ccsds::decodeUslp;
using starcom::ccsds::encodeClcw;
using starcom::ccsds::encodePltu;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::encodeUslp;
using starcom::ccsds::crc16Fecf;
using starcom::ccsds::Error;
using starcom::ccsds::kUslpFecfSize;
using starcom::ccsds::kUslpTruncatedMin;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kUslpConstructionNoSeg;
using starcom::ccsds::kUslpFrameMin;
using starcom::ccsds::kUslpUpidSpacePacket;
using starcom::ccsds::MapId;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::UslpFields;
using starcom::ccsds::UslpMib;
using starcom::ccsds::UslpScid;
using starcom::ccsds::Vcid;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

std::span<const std::byte> asSpan(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

// Non-truncated, VCF Length 0, empty TFDZ, rule 111, UPID 0. C = 7.
constexpr std::array<std::byte, 8> kUslpMin{
    std::byte{0xC0}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
    std::byte{0x00}, std::byte{0x07}, std::byte{0x00}, std::byte{0xE0}};

void test_decode_min() {
  const auto v = decodeUslp(asSpan(kUslpMin));
  CHECK(v.has_value());
  CHECK(v->fields.scid == UslpScid{0});
  CHECK(!v->fields.destination);
  CHECK(v->fields.vcid == Vcid{0});
  CHECK(v->fields.map_id == MapId{0});
  CHECK(!v->fields.expedited);
  CHECK(!v->fields.protocol_control);
  CHECK(!v->fields.ocf_present);
  CHECK(v->fields.vcf_count_len == 0);
  CHECK(v->fields.tfdz_construction == kUslpConstructionNoSeg);
  CHECK(v->fields.upid == kUslpUpidSpacePacket);
  CHECK(v->tfdz.empty());
  CHECK(v->ocf.empty());
}

void test_encode_min_matches_golden() {
  std::array<std::byte, 16> out{};
  const auto n = encodeUslp(out, UslpFields{}, {});
  CHECK(n.has_value());
  CHECK(*n == kUslpFrameMin);
  CHECK(std::equal(kUslpMin.begin(), kUslpMin.end(), out.begin()));
}

void test_roundtrip_populated() {
  UslpFields f{};
  f.scid = UslpScid{0xABCD};
  f.destination = true;
  f.vcid = Vcid{0x15};
  f.map_id = MapId{0xA};
  f.expedited = true;
  f.vcf_count_len = 2;
  f.vcf_count = 0x1234;
  f.upid = kUslpUpidSpacePacket;
  const std::array<std::byte, 3> tfdz{std::byte{0x11}, std::byte{0x22},
                                      std::byte{0x33}};
  std::array<std::byte, 32> out{};
  const auto n = encodeUslp(out, f, asSpan(tfdz));
  CHECK(n.has_value());
  // 7 + 2 VCF + 1 TFDF + 3 TFDZ = 13
  CHECK(*n == 13u);
  const auto v = decodeUslp(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.scid == f.scid);
  CHECK(v->fields.destination);
  CHECK(v->fields.vcid == f.vcid);
  CHECK(v->fields.map_id == f.map_id);
  CHECK(v->fields.expedited);
  CHECK(v->fields.vcf_count_len == 2);
  CHECK(v->fields.vcf_count == 0x1234);
  CHECK(v->tfdz.size() == 3);
  CHECK(std::equal(tfdz.begin(), tfdz.end(), v->tfdz.begin()));
}

void test_pointer_rule_and_ocf() {
  UslpFields f{};
  f.tfdz_construction = 0b001;
  f.tfdz_pointer = 0x0002;
  f.ocf_present = true;
  const std::array<std::byte, 4> tfdz{std::byte{0xAA}, std::byte{0xBB},
                                      std::byte{0xCC}, std::byte{0xDD}};
  std::array<std::byte, 4> ocf_buf{};
  Clcw32 w{};
  w.report_value = 9;
  CHECK(encodeClcw(ocf_buf, w).has_value());
  std::array<std::byte, 32> out{};
  const auto n = encodeUslp(out, f, asSpan(tfdz), asSpan(ocf_buf));
  CHECK(n.has_value());
  // 7 + 3 TFDF + 4 TFDZ + 4 OCF = 18
  CHECK(*n == 18u);
  const auto v = decodeUslp(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.tfdz_construction == 0b001);
  CHECK(v->fields.tfdz_pointer == 0x0002);
  CHECK(v->fields.ocf_present);
  CHECK(v->tfdz.size() == 4);
  CHECK(v->ocf.size() == 4);
  CHECK(v->ocf[3] == std::byte{9});
}

void test_pltu_composition() {
  std::array<std::byte, 8> frame{};
  const auto fn = encodeUslp(frame, UslpFields{}, {});
  CHECK(fn.has_value());
  std::array<std::byte, 32> pltu{};
  const auto pn = encodePltu(pltu, std::span<const std::byte>(frame.data(), *fn));
  CHECK(pn.has_value());
  CHECK(*pn == kPltuAsmSize + kUslpFrameMin + kPltuCrcSize);
  const auto view =
      decodePltu(std::span<const std::byte>(pltu.data(), *pn));
  CHECK(view.has_value());
  const auto u = decodeUslp(view->frame);
  CHECK(u.has_value());
  CHECK(u->tfdz.empty());
}

void test_space_packet_in_uslp_pltu() {
  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encodeSpacePacket(pkt, sp, asSpan(user));
  CHECK(pn.has_value());
  UslpFields f{};
  std::array<std::byte, 32> tf{};
  const auto fn =
      encodeUslp(tf, f, std::span<const std::byte>(pkt.data(), *pn));
  CHECK(fn.has_value());
  // 7 + 1 + 7 packet = 15; PLTU 3+15+4 = 22 = 21+N with N=1
  CHECK(*fn == 15u);
  std::array<std::byte, 64> wire{};
  const auto wn = encodePltu(wire, std::span<const std::byte>(tf.data(), *fn));
  CHECK(wn.has_value());
  CHECK(*wn == 22u);
  const auto pl = decodePltu(std::span<const std::byte>(wire.data(), *wn));
  CHECK(pl.has_value());
  const auto u = decodeUslp(pl->frame);
  CHECK(u.has_value());
  CHECK(u->tfdz.size() == *pn);
}

void test_reject_tfvn() {
  auto bad = kUslpMin;
  bad[0] = std::byte{0x80};  // V-3 TFVN
  CHECK(decodeUslp(asSpan(bad)).error() == Error::tfvn_unknown);
  bad[0] = std::byte{0xD0};  // 1101
  CHECK(decodeUslp(asSpan(bad)).error() == Error::tfvn_unknown);
}

void test_reject_truncated_header() {
  auto t = kUslpMin;
  t[3] = std::byte{0x01};  // End of Frame Primary Header Flag
  CHECK(decodeUslp(asSpan(t)).error() == Error::uslp_truncated);

  std::array<std::byte, 16> pltu{};
  // ASM + 4 header octets with flag=1 is enough for decodePltu to see the flag.
  pltu[0] = std::byte{0xFA};
  pltu[1] = std::byte{0xF3};
  pltu[2] = std::byte{0x20};
  pltu[3] = std::byte{0xC0};
  pltu[4] = std::byte{0x00};
  pltu[5] = std::byte{0x00};
  pltu[6] = std::byte{0x01};
  const auto r = decodePltu(std::span<const std::byte>(pltu.data(), 7));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::uslp_truncated);
}

void test_reject_short_and_length() {
  CHECK(decodeUslp({}).error() == Error::truncated);
  auto short_c = kUslpMin;
  short_c[5] = std::byte{0x06};  // C = 6 → 7-octet frame, below min 8
  CHECK(decodeUslp(asSpan(short_c)).error() == Error::uslp_length_oob);

  std::array<std::byte, 4> tiny{};
  CHECK(encodeUslp(tiny, UslpFields{}, {}).error() == Error::buffer_too_small);
}

void test_truncated_roundtrip_and_pltu() {
  UslpFields f{};
  f.truncated = true;
  const std::array<std::byte, 1> tfdz{std::byte{0xAA}};
  std::array<std::byte, 16> tf{};
  UslpMib mib{};
  mib.truncated_frame_length = kUslpTruncatedMin;
  const auto n = encodeUslp(tf, f, asSpan(tfdz), {}, mib);
  CHECK(n.has_value());
  CHECK(*n == kUslpTruncatedMin);
  CHECK((std::to_integer<unsigned>(tf[3]) & 0x01u) == 1u);

  CHECK(decodeUslp(std::span<const std::byte>(tf.data(), *n)).error() ==
        Error::uslp_truncated);
  const auto v =
      decodeUslp(std::span<const std::byte>(tf.data(), *n), mib);
  CHECK(v.has_value());
  CHECK(v->fields.truncated);
  CHECK(v->fields.expedited);
  CHECK(v->tfdz.size() == 1);
  CHECK(v->tfdz[0] == std::byte{0xAA});

  std::array<std::byte, 32> pltu{};
  const auto pn =
      encodePltu(pltu, std::span<const std::byte>(tf.data(), *n));
  CHECK(pn.has_value());
  CHECK(decodePltu(std::span<const std::byte>(pltu.data(), *pn)).error() ==
        Error::uslp_truncated);
  const auto pv = decodePltu(std::span<const std::byte>(pltu.data(), *pn),
                              mib.truncated_frame_length);
  CHECK(pv.has_value());
  CHECK(pv->frame.size() == kUslpTruncatedMin);
}

void test_insert_zone() {
  UslpMib mib{};
  mib.insert_zone_length = 2;
  const std::array<std::byte, 2> iz{std::byte{0x11}, std::byte{0x22}};
  std::array<std::byte, 32> out{};
  const auto n = encodeUslp(out, UslpFields{}, {}, {}, mib, asSpan(iz));
  CHECK(n.has_value());
  CHECK(*n == 10u);  // 7 + 2 insert + 1 TFDF
  const auto v =
      decodeUslp(std::span<const std::byte>(out.data(), *n), mib);
  CHECK(v.has_value());
  CHECK(v->insert_zone.size() == 2);
  CHECK(v->insert_zone[0] == std::byte{0x11});
  CHECK(v->tfdz.empty());
}

void test_fecf_roundtrip_and_bad() {
  UslpMib mib{};
  mib.fecf_present = true;
  std::array<std::byte, 32> out{};
  const auto n = encodeUslp(out, UslpFields{}, {}, {}, mib);
  CHECK(n.has_value());
  CHECK(*n == kUslpFrameMin + kUslpFecfSize);
  // Covered prefix is min frame with C = 9 (not the no-FECF C = 7 vector).
  CHECK(out[5] == std::byte{0x09});
  CHECK(crc16Fecf(std::span<const std::byte>(out.data(), kUslpFrameMin)) ==
        0x59D0u);
  CHECK(out[8] == std::byte{0x59});
  CHECK(out[9] == std::byte{0xD0});
  const auto v =
      decodeUslp(std::span<const std::byte>(out.data(), *n), mib);
  CHECK(v.has_value());
  CHECK(v->fecf.size() == kUslpFecfSize);
  out[9] ^= std::byte{0x01};
  CHECK(decodeUslp(std::span<const std::byte>(out.data(), *n), mib).error() ==
        Error::uslp_bad_fecf);
}

void test_truncated_rejects_insert_or_fecf() {
  UslpFields f{};
  f.truncated = true;
  const std::array<std::byte, 1> tfdz{std::byte{0xAA}};
  UslpMib mib{};
  mib.truncated_frame_length = kUslpTruncatedMin;
  mib.fecf_present = true;
  std::array<std::byte, 16> out{};
  CHECK(encodeUslp(out, f, asSpan(tfdz), {}, mib).error() ==
        Error::uslp_length_oob);
}

void test_heap() {
  std::array<std::byte, 32> out{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  const auto n = encodeUslp(out, UslpFields{}, {});
  if (n.has_value()) {
    (void)decodeUslp(std::span<const std::byte>(out.data(), *n));
    UslpFields tr{};
    tr.truncated = true;
    const std::array<std::byte, 1> z{std::byte{0xAA}};
    UslpMib mib{};
    mib.truncated_frame_length = kUslpTruncatedMin;
    (void)encodeUslp(out, tr, asSpan(z), {}, mib);
  }
  starcom::test::heapTrapDisarm();
  CHECK(n.has_value());
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_uslp_tests() {
  test_decode_min();
  test_encode_min_matches_golden();
  test_roundtrip_populated();
  test_pointer_rule_and_ocf();
  test_pltu_composition();
  test_space_packet_in_uslp_pltu();
  test_reject_tfvn();
  test_reject_truncated_header();
  test_truncated_roundtrip_and_pltu();
  test_insert_zone();
  test_fecf_roundtrip_and_bad();
  test_truncated_rejects_insert_or_fecf();
  test_reject_short_and_length();
  test_heap();
  return g_fails;
}
