// IVP increment 14 — V-3 DFC 11 User Defined Data + simplex without hailing.
// 211.0-B-6 Table 3-1, 3.2.3.5, 2.2.2.3; simplex 211.0 section 6.

#include "heap_trap.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/mac.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/v3.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::copp_bytes_to_send;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::copp_init;
using starcom::ccsds::CoppMib;
using starcom::ccsds::copp_receive_bytes;
using starcom::ccsds::copp_submit_sdu;
using starcom::ccsds::copp_submit_user_defined;
using starcom::ccsds::copp_take_sdu;
using starcom::ccsds::decode_pltu;
using starcom::ccsds::decode_space_packet;
using starcom::ccsds::decode_v3;
using starcom::ccsds::encode_pltu;
using starcom::ccsds::encode_v3;
using starcom::ccsds::encode_v3_user_defined;
using starcom::ccsds::Error;
using starcom::ccsds::kDfcPackets;
using starcom::ccsds::kDfcReserved;
using starcom::ccsds::kDfcUserDefined;
using starcom::ccsds::kPltuAsmSize;
using starcom::ccsds::kPltuCrcSize;
using starcom::ccsds::kV3DataMax;
using starcom::ccsds::kV3HeaderSize;
using starcom::ccsds::mac_init;
using starcom::ccsds::mac_phy;
using starcom::ccsds::mac_set_mode;
using starcom::ccsds::MacDuplex;
using starcom::ccsds::MacMib;
using starcom::ccsds::MacMode;
using starcom::ccsds::MacSession;
using starcom::ccsds::MacState;
using starcom::ccsds::Pcid;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::v3_is_user_defined;
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

void test_constants() {
  CHECK(kDfcPackets == 0);
  CHECK(kDfcReserved == 2);
  CHECK(kDfcUserDefined == 3);
  CHECK(kV3DataMax == 2043u);
  CHECK(kV3HeaderSize + kV3DataMax == 2048u);
}

void test_dfc11_opaque_and_empty() {
  V3Fields f{};
  f.scid = Scid{0x15};
  f.pcid = Pcid{1};
  f.port_id = PortId{3};
  f.destination = true;
  f.fsn = 0x42;
  f.dfc_id = kDfcPackets;
  f.p_frame = true;

  std::array<std::byte, 16> out{};
  const auto empty_n = encode_v3_user_defined(out, f, {});
  CHECK(empty_n.has_value());
  CHECK(*empty_n == kV3HeaderSize);
  const auto empty_v = decode_v3(std::span<const std::byte>(out.data(), *empty_n));
  CHECK(empty_v.has_value());
  CHECK(!empty_v->fields.p_frame);
  CHECK(empty_v->fields.dfc_id == kDfcUserDefined);
  CHECK(empty_v->data.empty());
  CHECK(v3_is_user_defined(empty_v->fields));
  CHECK(empty_v->fields.port_id == PortId{3});
  CHECK(empty_v->fields.fsn == 0x42);

  const std::array<std::byte, 2> opaque{std::byte{0xDE}, std::byte{0xAD}};
  const auto n = encode_v3_user_defined(out, f, as_span(opaque));
  CHECK(n.has_value());
  CHECK(*n == kV3HeaderSize + 2u);
  const auto v = decode_v3(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.dfc_id == kDfcUserDefined);
  CHECK(!v->fields.p_frame);
  CHECK(v3_is_user_defined(v->fields));
  CHECK(v->data.size() == 2u);
  CHECK(v->data[0] == std::byte{0xDE});
  CHECK(v->data[1] == std::byte{0xAD});
}

void test_pltu_wrap_dfc11() {
  V3Fields f{};
  f.port_id = PortId{1};
  const std::array<std::byte, 2> opaque{std::byte{0x11}, std::byte{0x22}};
  std::array<std::byte, 16> frame{};
  const auto n = encode_v3_user_defined(frame, f, as_span(opaque));
  CHECK(n.has_value());
  std::array<std::byte, 32> pltu{};
  const auto p = encode_pltu(pltu, std::span<const std::byte>(frame.data(), *n));
  CHECK(p.has_value());
  CHECK(*p == kPltuAsmSize + kV3HeaderSize + opaque.size() + kPltuCrcSize);
  const auto env = decode_pltu(std::span<const std::byte>(pltu.data(), *p));
  CHECK(env.has_value());
  const auto v = decode_v3(env->frame);
  CHECK(v.has_value());
  CHECK(v->fields.dfc_id == kDfcUserDefined);
  CHECK(v->data.size() == 2u);
}

void test_not_space_packet() {
  V3Fields f{};
  const std::array<std::byte, 3> garbage{std::byte{0xFF}, std::byte{0x00},
                                        std::byte{0x01}};
  std::array<std::byte, 16> frame{};
  const auto n = encode_v3_user_defined(frame, f, as_span(garbage));
  CHECK(n.has_value());
  const auto v = decode_v3(std::span<const std::byte>(frame.data(), *n));
  CHECK(v.has_value());
  CHECK(v3_is_user_defined(v->fields));
  const auto sp = decode_space_packet(v->data);
  CHECK(!sp.has_value());
}

void test_split_sdu_no_reassembly() {
  V3Fields f{};
  f.fsn = 0;
  const std::array<std::byte, 3> a{std::byte{0xAA}, std::byte{0xBB},
                                  std::byte{0xCC}};
  const std::array<std::byte, 3> b{std::byte{0xDD}, std::byte{0xEE},
                                  std::byte{0xFF}};
  std::array<std::byte, 16> fa{};
  std::array<std::byte, 16> fb{};
  const auto na = encode_v3_user_defined(fa, f, as_span(a));
  f.fsn = 1;
  const auto nb = encode_v3_user_defined(fb, f, as_span(b));
  CHECK(na.has_value());
  CHECK(nb.has_value());
  const auto va = decode_v3(std::span<const std::byte>(fa.data(), *na));
  const auto vb = decode_v3(std::span<const std::byte>(fb.data(), *nb));
  CHECK(va.has_value());
  CHECK(vb.has_value());
  CHECK(va->data.size() == 3u);
  CHECK(vb->data.size() == 3u);
  CHECK(va->data[0] == std::byte{0xAA});
  CHECK(vb->data[0] == std::byte{0xDD});
  CHECK(va->data.size() != 6u);
}

void test_pframe_forces_dfc_packets() {
  V3Fields f{};
  f.p_frame = true;
  f.dfc_id = kDfcUserDefined;
  f.port_id = PortId{7};
  const std::array<std::byte, 2> data{std::byte{0x01}, std::byte{0x02}};
  std::array<std::byte, 16> out{};
  const auto n = encode_v3(out, f, as_span(data));
  CHECK(n.has_value());
  const auto v = decode_v3(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.p_frame);
  CHECK(v->fields.dfc_id == kDfcPackets);
  CHECK(v->fields.port_id == PortId{0});
  CHECK(!v3_is_user_defined(v->fields));
}

void test_reserved_dfc_not_a_service() {
  V3Fields f{};
  f.dfc_id = kDfcReserved;
  const std::array<std::byte, 1> data{std::byte{0x99}};
  std::array<std::byte, 16> out{};
  const auto n = encode_v3(out, f, as_span(data));
  CHECK(n.has_value());
  const auto v = decode_v3(std::span<const std::byte>(out.data(), *n));
  CHECK(v.has_value());
  CHECK(v->fields.dfc_id == kDfcReserved);
  CHECK(!v3_is_user_defined(v->fields));
}

void test_user_defined_length_oob() {
  V3Fields f{};
  std::array<std::byte, kV3DataMax + 1> too_big{};
  std::array<std::byte, 8> tiny{};
  const auto r = encode_v3_user_defined(tiny, f, as_span(too_big));
  CHECK(!r.has_value());
  CHECK(r.error() == Error::v3_length_oob);
}

void test_simplex_no_hail() {
  MacMib mib{};
  MacSession s{};
  mac_init(s, mib, MacDuplex::simplex_transmit, nullptr);
  mac_set_mode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s71);
  CHECK(s.duplex == MacDuplex::simplex_transmit);
  CHECK(s.state != MacState::s31);
  CHECK(s.state != MacState::s11);

  mac_init(s, mib, MacDuplex::simplex_receive, nullptr);
  mac_set_mode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s72);
  CHECK(mac_phy(s).receive);
  CHECK(!mac_phy(s).transmit);
  CHECK(s.state != MacState::s31);
  CHECK(s.state != MacState::s11);

  mac_init(s, mib, MacDuplex::simplex_transmit, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s1);
  CHECK(s.state != MacState::s31);
  CHECK(s.state != MacState::s11);

  mac_init(s, mib, MacDuplex::simplex_receive, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s1);
  CHECK(s.state != MacState::s31);
  CHECK(s.state != MacState::s11);
}

void drain_opening_plcw(CoppEndpoint& a, CoppEndpoint& b,
                        std::span<std::byte> wire) {
  const auto n0 = copp_bytes_to_send(a, wire);
  CHECK(n0.has_value());
  CHECK(*n0 > 0);
  copp_receive_bytes(b, std::span<const std::byte>(wire.data(), *n0));
  const auto n1 = copp_bytes_to_send(b, wire);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  copp_receive_bytes(a, std::span<const std::byte>(wire.data(), *n1));
}

void test_copp_user_defined_host_loop() {
  CoppMib mib{};
  mib.transmission_window = 4;
  CoppEndpoint rx{};
  CoppEndpoint tx{};
  copp_init(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{1});
  copp_init(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{1});

  std::array<std::byte, 128> wire{};
  drain_opening_plcw(rx, tx, wire);

  const std::array<std::byte, 3> part0{std::byte{0xAA}, std::byte{0xBB},
                                      std::byte{0xCC}};
  const std::array<std::byte, 3> part1{std::byte{0xDD}, std::byte{0xEE},
                                      std::byte{0xFF}};
  CHECK(copp_submit_user_defined(tx, as_span(part0), false).has_value());
  CHECK(copp_submit_user_defined(tx, as_span(part1), false).has_value());

  const auto u0 = copp_bytes_to_send(tx, wire);
  CHECK(u0.has_value());
  CHECK(*u0 > 0);
  const auto env0 = decode_pltu(std::span<const std::byte>(wire.data(), *u0));
  CHECK(env0.has_value());
  const auto v0 = decode_v3(env0->frame);
  CHECK(v0.has_value());
  CHECK(v3_is_user_defined(v0->fields));
  CHECK(v0->data.size() == 3u);
  CHECK(!decode_space_packet(v0->data).has_value());
  copp_receive_bytes(rx, std::span<const std::byte>(wire.data(), *u0));

  std::array<std::byte, 16> sdu{};
  const auto t0 = copp_take_sdu(rx, sdu);
  CHECK(t0.has_value());
  CHECK(*t0 == 3u);
  CHECK(std::equal(part0.begin(), part0.end(), sdu.begin()));

  const auto ack = copp_bytes_to_send(rx, wire);
  CHECK(ack.has_value());
  CHECK(*ack > 0);
  copp_receive_bytes(tx, std::span<const std::byte>(wire.data(), *ack));

  const auto u1 = copp_bytes_to_send(tx, wire);
  CHECK(u1.has_value());
  CHECK(*u1 > 0);
  const auto env1 = decode_pltu(std::span<const std::byte>(wire.data(), *u1));
  CHECK(env1.has_value());
  const auto v1 = decode_v3(env1->frame);
  CHECK(v1.has_value());
  CHECK(v3_is_user_defined(v1->fields));
  copp_receive_bytes(rx, std::span<const std::byte>(wire.data(), *u1));

  const auto t1 = copp_take_sdu(rx, sdu);
  CHECK(t1.has_value());
  CHECK(*t1 == 3u);
  CHECK(std::equal(part1.begin(), part1.end(), sdu.begin()));
  CHECK(*t0 != 6u);
  CHECK(*t1 != 6u);
}

void test_copp_default_still_packets() {
  CoppMib mib{};
  CoppEndpoint rx{};
  CoppEndpoint tx{};
  copp_init(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{0});
  copp_init(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  std::array<std::byte, 128> wire{};
  drain_opening_plcw(rx, tx, wire);
  const std::array<std::byte, 1> pkt{std::byte{0x42}};
  CHECK(copp_submit_sdu(tx, as_span(pkt), false).has_value());
  const auto n = copp_bytes_to_send(tx, wire);
  CHECK(n.has_value());
  const auto env = decode_pltu(std::span<const std::byte>(wire.data(), *n));
  CHECK(env.has_value());
  const auto v = decode_v3(env->frame);
  CHECK(v.has_value());
  CHECK(v->fields.dfc_id == kDfcPackets);
  CHECK(!v3_is_user_defined(v->fields));
}

void test_copp_empty_user_defined_on_wire() {
  CoppMib mib{};
  CoppEndpoint rx{};
  CoppEndpoint tx{};
  copp_init(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{0});
  copp_init(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  std::array<std::byte, 128> wire{};
  drain_opening_plcw(rx, tx, wire);
  CHECK(copp_submit_user_defined(tx, {}, false).has_value());
  const auto n = copp_bytes_to_send(tx, wire);
  CHECK(n.has_value());
  CHECK(*n > 0);
  const auto env = decode_pltu(std::span<const std::byte>(wire.data(), *n));
  CHECK(env.has_value());
  const auto v = decode_v3(env->frame);
  CHECK(v.has_value());
  CHECK(v3_is_user_defined(v->fields));
  CHECK(v->data.empty());
}

void test_heap() {
  V3Fields f{};
  const std::array<std::byte, 2> opaque{std::byte{0x01}, std::byte{0x02}};
  std::array<std::byte, 32> frame{};
  std::array<std::byte, 64> pltu{};
  CoppMib mib{};
  CoppEndpoint e{};
  copp_init(e, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  const auto n = encode_v3_user_defined(frame, f, as_span(opaque));
  if (n) {
    (void)decode_v3(std::span<const std::byte>(frame.data(), *n));
    (void)encode_pltu(pltu, std::span<const std::byte>(frame.data(), *n));
  }
  (void)copp_submit_user_defined(e, as_span(opaque), false);
  (void)copp_bytes_to_send(e, pltu);
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_user_defined_tests() {
  test_constants();
  test_dfc11_opaque_and_empty();
  test_pltu_wrap_dfc11();
  test_not_space_packet();
  test_split_sdu_no_reassembly();
  test_pframe_forces_dfc_packets();
  test_reserved_dfc_not_a_service();
  test_user_defined_length_oob();
  test_simplex_no_hail();
  test_copp_user_defined_host_loop();
  test_copp_default_still_packets();
  test_copp_empty_user_defined_on_wire();
  test_heap();
  return g_fails;
}
