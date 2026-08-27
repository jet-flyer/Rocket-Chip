// IVP increment 0+1 — Space Packet (docs/TESTING.md, docs/IVP.md)
// Accept: sp-idle; v3-one-sp-n (PLTU 18+N)
// Reject: sp-too-short, sp-pvn, buffer_too_small

#include "heap_trap.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/sp.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::Apid;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::decode_sp;
using starcom::ccsds::decode_v3;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::encode_sp;
using starcom::ccsds::encode_v3;
using starcom::ccsds::Error;
using starcom::ccsds::kIdleApid;
using starcom::ccsds::kSpHeaderSize;
using starcom::ccsds::kSpMinSize;
using starcom::ccsds::SpFields;
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

std::span<const std::byte> as_span(const auto& a) {
  return std::span<const std::byte>(a.data(), a.size());
}

void test_sp_idle() {
  // IVP sp-idle: APID all-ones, secondary header 0
  SpFields f{};
  f.apid = kIdleApid;
  f.seq_flags = 0b11;
  const std::array<std::byte, 1> data{std::byte{0x00}};
  std::array<std::byte, 16> out{};
  const auto n = encode_sp(out, f, as_span(data));
  CHECK(n.has_value());
  CHECK(*n == kSpMinSize);
  CHECK(out[0] == std::byte{0x07});
  CHECK(out[1] == std::byte{0xFF});
  CHECK(out[2] == std::byte{0xC0});
  CHECK(out[3] == std::byte{0x00});
  CHECK(out[4] == std::byte{0x00});
  CHECK(out[5] == std::byte{0x00});
  CHECK(out[6] == std::byte{0x00});

  const auto v = decode_sp(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(!v->fields.telecommand);
  CHECK(!v->fields.secondary_header);
  CHECK(v->fields.apid == kIdleApid);
  CHECK(v->fields.seq_flags == 0b11);
  CHECK(v->data.size() == 1u);
}

void test_roundtrip() {
  SpFields f{};
  f.telecommand = true;
  f.secondary_header = true;
  f.apid = Apid{0x123};
  f.seq_flags = 0b01;
  f.seq_count = 0x2ABC;
  const std::array<std::byte, 3> data{std::byte{0xAA}, std::byte{0xBB},
                                      std::byte{0xCC}};
  std::array<std::byte, 32> out{};
  const auto n = encode_sp(out, f, as_span(data));
  CHECK(n.has_value());
  CHECK(*n == 9u);
  const auto v = decode_sp(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.telecommand);
  CHECK(v->fields.secondary_header);
  CHECK(v->fields.apid == Apid{0x123});
  CHECK(v->fields.seq_flags == 0b01);
  CHECK(v->fields.seq_count == 0x2ABC);
  CHECK(v->data.size() == 3u);
  CHECK(v->data[0] == std::byte{0xAA});
}

void test_reject_sp_too_short() {
  // IVP sp-too-short
  const auto empty = decode_sp({});
  CHECK(!empty.has_value());
  CHECK(empty.error() == Error::sp_too_short);

  std::array<std::byte, 6> hdr_only{};
  const auto six = decode_sp(as_span(hdr_only));
  CHECK(!six.has_value());
  CHECK(six.error() == Error::sp_too_short);

  // C = 1 implies 2 data octets; only 1 present after header
  std::array<std::byte, 7> claims_two{
      std::byte{0x00}, std::byte{0x00}, std::byte{0xC0}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x01}, std::byte{0x00}};
  const auto cut = decode_sp(as_span(claims_two));
  CHECK(!cut.has_value());
  CHECK(cut.error() == Error::sp_too_short);

  SpFields f{};
  std::array<std::byte, 16> out{};
  const auto enc = encode_sp(out, f, {});
  CHECK(!enc.has_value());
  CHECK(enc.error() == Error::sp_too_short);
}

void test_reject_sp_pvn() {
  // IVP sp-pvn
  std::array<std::byte, 7> pkt{
      std::byte{0x20}, std::byte{0x00}, std::byte{0xC0}, std::byte{0x00},
      std::byte{0x00}, std::byte{0x00}, std::byte{0x00}};  // PVN 001
  const auto r = decode_sp(as_span(pkt));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::sp_pvn);
}

void test_encode_buffer_too_small() {
  SpFields f{};
  const std::array<std::byte, 1> data{std::byte{0x00}};
  std::array<std::byte, 6> too_small{};
  const auto r = encode_sp(too_small, f, as_span(data));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::buffer_too_small);
}

void test_v3_one_sp_n() {
  // IVP v3-one-sp-n: PLTU = 18+N
  constexpr std::size_t n_user = 4;
  SpFields sp{};
  sp.apid = Apid{0x042};
  const std::array<std::byte, n_user> user{std::byte{1}, std::byte{2},
                                           std::byte{3}, std::byte{4}};
  std::array<std::byte, 32> packet{};
  const auto pn = encode_sp(packet, sp, as_span(user));
  CHECK(pn.has_value());
  CHECK(*pn == kSpHeaderSize + n_user);

  V3Fields v3{};
  v3.dfc_id = 0;  // integer packets
  std::array<std::byte, 64> frame{};
  const auto fn =
      encode_v3(frame, v3, std::span<const std::byte>(packet.data(), *pn));
  CHECK(fn.has_value());
  CHECK(*fn == 5 + *pn);

  std::array<std::byte, 80> pltu{};
  const auto plen =
      encode_pltu(pltu, std::span<const std::byte>(frame.data(), *fn));
  CHECK(plen.has_value());
  CHECK(*plen == 18 + n_user);

  const auto env = decode_pltu(std::span<const std::byte>(pltu.data(), *plen));
  CHECK(env.has_value());
  const auto vf = decode_v3(env->frame);
  CHECK(vf.has_value());
  const auto spv = decode_sp(vf->data);
  CHECK(spv.has_value());
  CHECK(spv->fields.apid == Apid{0x042});
  CHECK(spv->data.size() == n_user);
}

void test_heap() {
  SpFields f{};
  const std::array<std::byte, 1> data{std::byte{0x00}};
  std::array<std::byte, 16> out{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  const auto n = encode_sp(out, f, as_span(data));
  if (n.has_value()) {
    (void)decode_sp(std::span<const std::byte>(out.data(), *n));
  }
  starcom::test::heap_trap_disarm();
  CHECK(n.has_value());
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_sp_tests() {
  test_sp_idle();
  test_roundtrip();
  test_reject_sp_too_short();
  test_reject_sp_pvn();
  test_encode_buffer_too_small();
  test_v3_one_sp_n();
  test_heap();
  return g_fails;
}
