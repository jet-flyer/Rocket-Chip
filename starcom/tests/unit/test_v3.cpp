// IVP increment 0+1 — Version-3 (docs/TESTING.md, docs/IVP.md)
// Accept: v3-header-only field unpack; round-trip; PLTU composition
// Reject: truncated, tfvn-unknown, v3-length-oob, buffer_too_small

#include "heap_trap.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

using starcom::ccsds::decodePltu;
using starcom::ccsds::decodeV3;
using starcom::ccsds::encodePltu;
using starcom::ccsds::encodeV3;
using starcom::ccsds::Error;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kV3HeaderSize;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::V3Fields;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

constexpr std::array<std::byte, 5> kV3HeaderOnly{
    std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x04},
    std::byte{0x00}};

std::span<const std::byte> asSpan(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

void test_decode_header_only() {
  // IVP v3-header-only
  const auto v = decodeV3(asSpan(kV3HeaderOnly));
  CHECK(v.has_value());
  CHECK(!v->fields.qos_expedited);
  CHECK(!v->fields.p_frame);
  CHECK(v->fields.dfc_id == 0);
  CHECK(v->fields.scid == Scid{0});
  CHECK(v->fields.pcid == Pcid{0});
  CHECK(v->fields.port_id == PortId{0});
  CHECK(!v->fields.destination);
  CHECK(v->fields.fsn == 0);
  CHECK(v->data.empty());
}

void test_roundtrip_populated() {
  V3Fields f{};
  f.qos_expedited = true;
  f.p_frame = false;
  f.dfc_id = 0b11;
  f.scid = Scid{0x2AA};
  f.pcid = Pcid{1};
  f.port_id = PortId{7};
  f.destination = true;
  f.fsn = 0x5A;
  const std::array<std::byte, 2> data{std::byte{0x11}, std::byte{0x22}};
  std::array<std::byte, 16> out{};
  const auto n = encodeV3(out, f, asSpan(data));
  CHECK(n.has_value());
  CHECK(*n == 7u);
  const auto v = decodeV3(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.qos_expedited);
  CHECK(!v->fields.p_frame);
  CHECK(v->fields.dfc_id == 0b11);
  CHECK(v->fields.scid == Scid{0x2AA});
  CHECK(v->fields.pcid == Pcid{1});
  CHECK(v->fields.port_id == PortId{7});
  CHECK(v->fields.destination);
  CHECK(v->fields.fsn == 0x5A);
  CHECK(v->data.size() == 2u);
  CHECK(v->data[0] == std::byte{0x11});
  CHECK(v->data[1] == std::byte{0x22});
}

void test_encode_matches_golden() {
  V3Fields f{};
  std::array<std::byte, 5> out{};
  const auto n = encodeV3(out, f, {});
  CHECK(n.has_value());
  CHECK(*n == 5u);
  CHECK(std::equal(out.begin(), out.end(), kV3HeaderOnly.begin()));
}

void test_reject_truncated() {
  // IVP truncated
  const auto empty = decodeV3({});
  CHECK(!empty.has_value());
  CHECK(empty.error() == Error::truncated);

  const std::array<std::byte, 4> four{std::byte{0x80}, std::byte{0x00},
                                      std::byte{0x00}, std::byte{0x04}};
  const auto short_hdr = decodeV3(asSpan(four));
  CHECK(!short_hdr.has_value());
  CHECK(short_hdr.error() == Error::truncated);

  // C = 5 implies 6-octet frame; only 5 octets present
  const std::array<std::byte, 5> claims_six{
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x05},
      std::byte{0x00}};
  const auto cut = decodeV3(asSpan(claims_six));
  CHECK(!cut.has_value());
  CHECK(cut.error() == Error::truncated);
}

void test_reject_tfvn_unknown() {
  // IVP tfvn-unknown
  auto frame = kV3HeaderOnly;
  frame[0] = std::byte{0x00};
  const auto r = decodeV3(asSpan(frame));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::tfvn_unknown);
}

void test_reject_v3_length_oob() {
  // IVP v3-length-oob
  std::array<std::byte, 5> too_small{
      std::byte{0x80}, std::byte{0x00}, std::byte{0x00}, std::byte{0x03},
      std::byte{0x00}};
  const auto r = decodeV3(asSpan(too_small));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::v3_length_oob);
}

void test_encode_buffer_too_small() {
  V3Fields f{};
  std::array<std::byte, 4> too_small{};
  const auto r = encodeV3(too_small, f, {});
  CHECK(!r.has_value());
  CHECK(r.error() == Error::buffer_too_small);
}

void test_pltu_composition() {
  V3Fields f{};
  std::array<std::byte, 5> frame{};
  const auto n = encodeV3(frame, f, {});
  CHECK(n.has_value());
  std::array<std::byte, 16> pltu{};
  const auto p = encodePltu(pltu, std::span<const std::byte>(frame.data(), *n));
  CHECK(p.has_value());
  CHECK(*p == kPltuAsmSize + kV3HeaderSize + kPltuCrcSize);
  const auto env = decodePltu(std::span<const std::byte>(pltu.data(), *p));
  CHECK(env.has_value());
  const auto v = decodeV3(env->frame);
  CHECK(v.has_value());
  CHECK(v->data.empty());
}

void test_heap() {
  V3Fields f{};
  std::array<std::byte, 8> out{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  (void)encodeV3(out, f, {});
  (void)decodeV3(asSpan(kV3HeaderOnly));
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_v3_tests() {
  test_decode_header_only();
  test_roundtrip_populated();
  test_encode_matches_golden();
  test_reject_truncated();
  test_reject_tfvn_unknown();
  test_reject_v3_length_oob();
  test_encode_buffer_too_small();
  test_pltu_composition();
  test_heap();
  return g_fails;
}
