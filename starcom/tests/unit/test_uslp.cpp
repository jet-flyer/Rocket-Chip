// IVP increment 3 — USLP Version-4 in the same PLTU (732.1 §4.1, 211.2 §3.6.4).
// Non-truncated header. Truncated / Insert Zone / FECF not this sitting.

#include "heap_trap.hpp"
#include "starcom/ccsds/clcw.hpp"
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
using starcom::ccsds::decode_pltu;
using starcom::ccsds::decode_uslp;
using starcom::ccsds::encode_clcw;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::encode_space_packet;
using starcom::ccsds::encode_uslp;
using starcom::ccsds::Error;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kUslpConstructionNoSeg;
using starcom::ccsds::kUslpFrameMin;
using starcom::ccsds::kUslpUpidSpacePacket;
using starcom::ccsds::MapId;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::UslpFields;
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

std::span<const std::byte> as_span(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

// Non-truncated, VCF Length 0, empty TFDZ, rule 111, UPID 0. C = 7.
constexpr std::array<std::byte, 8> kUslpMin{
    std::byte{0xC0}, std::byte{0x00}, std::byte{0x00}, std::byte{0x00},
    std::byte{0x00}, std::byte{0x07}, std::byte{0x00}, std::byte{0xE0}};

void test_decode_min() {
  const auto v = decode_uslp(as_span(kUslpMin));
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
  const auto n = encode_uslp(out, UslpFields{}, {});
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
  const auto n = encode_uslp(out, f, as_span(tfdz));
  CHECK(n.has_value());
  // 7 + 2 VCF + 1 TFDF + 3 TFDZ = 13
  CHECK(*n == 13u);
  const auto v = decode_uslp(std::span<const std::byte>(out.data(), *n));
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
  CHECK(encode_clcw(ocf_buf, w).has_value());
  std::array<std::byte, 32> out{};
  const auto n = encode_uslp(out, f, as_span(tfdz), as_span(ocf_buf));
  CHECK(n.has_value());
  // 7 + 3 TFDF + 4 TFDZ + 4 OCF = 18
  CHECK(*n == 18u);
  const auto v = decode_uslp(std::span<const std::byte>(out.data(), *n));
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
  const auto fn = encode_uslp(frame, UslpFields{}, {});
  CHECK(fn.has_value());
  std::array<std::byte, 32> pltu{};
  const auto pn = encode_pltu(pltu, std::span<const std::byte>(frame.data(), *fn));
  CHECK(pn.has_value());
  CHECK(*pn == kPltuAsmSize + kUslpFrameMin + kPltuCrcSize);
  const auto view =
      decode_pltu(std::span<const std::byte>(pltu.data(), *pn));
  CHECK(view.has_value());
  const auto u = decode_uslp(view->frame);
  CHECK(u.has_value());
  CHECK(u->tfdz.empty());
}

void test_space_packet_in_uslp_pltu() {
  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encode_space_packet(pkt, sp, as_span(user));
  CHECK(pn.has_value());
  UslpFields f{};
  std::array<std::byte, 32> tf{};
  const auto fn =
      encode_uslp(tf, f, std::span<const std::byte>(pkt.data(), *pn));
  CHECK(fn.has_value());
  // 7 + 1 + 7 packet = 15; PLTU 3+15+4 = 22 = 21+N with N=1
  CHECK(*fn == 15u);
  std::array<std::byte, 64> wire{};
  const auto wn = encode_pltu(wire, std::span<const std::byte>(tf.data(), *fn));
  CHECK(wn.has_value());
  CHECK(*wn == 22u);
  const auto pl = decode_pltu(std::span<const std::byte>(wire.data(), *wn));
  CHECK(pl.has_value());
  const auto u = decode_uslp(pl->frame);
  CHECK(u.has_value());
  CHECK(u->tfdz.size() == *pn);
}

void test_reject_tfvn() {
  auto bad = kUslpMin;
  bad[0] = std::byte{0x80};  // V-3 TFVN
  CHECK(decode_uslp(as_span(bad)).error() == Error::tfvn_unknown);
  bad[0] = std::byte{0xD0};  // 1101
  CHECK(decode_uslp(as_span(bad)).error() == Error::tfvn_unknown);
}

void test_reject_truncated_header() {
  auto t = kUslpMin;
  t[3] = std::byte{0x01};  // End of Frame Primary Header Flag
  CHECK(decode_uslp(as_span(t)).error() == Error::uslp_truncated);

  std::array<std::byte, 16> pltu{};
  // ASM + 4 header octets with flag=1 is enough for decode_pltu to see the flag.
  pltu[0] = std::byte{0xFA};
  pltu[1] = std::byte{0xF3};
  pltu[2] = std::byte{0x20};
  pltu[3] = std::byte{0xC0};
  pltu[4] = std::byte{0x00};
  pltu[5] = std::byte{0x00};
  pltu[6] = std::byte{0x01};
  const auto r = decode_pltu(std::span<const std::byte>(pltu.data(), 7));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::uslp_truncated);
}

void test_reject_short_and_length() {
  CHECK(decode_uslp({}).error() == Error::truncated);
  auto short_c = kUslpMin;
  short_c[5] = std::byte{0x06};  // C = 6 → 7-octet frame, below min 8
  CHECK(decode_uslp(as_span(short_c)).error() == Error::uslp_length_oob);

  std::array<std::byte, 4> tiny{};
  CHECK(encode_uslp(tiny, UslpFields{}, {}).error() == Error::buffer_too_small);
}

void test_heap() {
  std::array<std::byte, 32> out{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  const auto n = encode_uslp(out, UslpFields{}, {});
  if (n.has_value()) {
    (void)decode_uslp(std::span<const std::byte>(out.data(), *n));
  }
  starcom::test::heap_trap_disarm();
  CHECK(n.has_value());
  CHECK(starcom::test::heap_trap_count() == 0);
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
  test_reject_short_and_length();
  test_heap();
  return g_fails;
}
