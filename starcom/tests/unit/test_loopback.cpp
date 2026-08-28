// IVP increment 5 — host loopback, then generic radio port. Core stays sans-I/O.

#include "heap_trap.hpp"
#include "starcom/adapters/loopback.hpp"
#include "starcom/adapters/radio_port.hpp"
#include "starcom/ccsds/cop1.hpp"
#include "starcom/ccsds/space_packet.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::adapters::HostLoopback;
using starcom::adapters::kAdapterFrameMax;
using starcom::adapters::radio_begin_tx;
using starcom::adapters::radio_offer_rx;
using starcom::adapters::radio_poll_rx;
using starcom::adapters::radio_take_tx;
using starcom::adapters::RadioPort;
using starcom::adapters::slot_read;
using starcom::adapters::slot_write;
using starcom::ccsds::cop1_bytes_to_send;
using starcom::ccsds::Cop1Endpoint;
using starcom::ccsds::Cop1Event;
using starcom::ccsds::cop1_init;
using starcom::ccsds::cop1_initiate_ad;
using starcom::ccsds::Cop1Mib;
using starcom::ccsds::cop1_poll_event;
using starcom::ccsds::cop1_receive_bytes;
using starcom::ccsds::cop1_submit_sdu;
using starcom::ccsds::cop1_take_sdu;
using starcom::ccsds::encode_space_packet;
using starcom::ccsds::Error;
using starcom::ccsds::MapId;
using starcom::ccsds::SpacePacketFields;
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

void test_slot_full_and_empty() {
  starcom::adapters::FrameSlot s{};
  const std::array<std::byte, 3> a{std::byte{1}, std::byte{2}, std::byte{3}};
  CHECK(slot_write(s, a).value() == 3u);
  CHECK(slot_write(s, a).error() == Error::buffer_too_small);
  std::array<std::byte, 8> out{};
  CHECK(slot_read(s, out).value() == 3u);
  CHECK(slot_read(s, out).value() == 0u);
}

void test_cop1_over_loopback() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.farm.w = 8;
  Cop1Endpoint tx{};
  Cop1Endpoint rx{};
  cop1_init(tx, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});
  cop1_init(rx, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  CHECK(cop1_initiate_ad(tx));

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encode_space_packet(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(cop1_submit_sdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());

  HostLoopback wire{};
  std::array<std::byte, kAdapterFrameMax> scratch{};

  const auto n1 = cop1_bytes_to_send(tx, scratch);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  CHECK(slot_write(wire.a_to_b, std::span<const std::byte>(scratch.data(), *n1))
            .has_value());
  const auto r1 = slot_read(wire.a_to_b, scratch);
  CHECK(r1.has_value());
  cop1_receive_bytes(rx, std::span<const std::byte>(scratch.data(), *r1));
  CHECK(cop1_poll_event(rx) == Cop1Event::farm_accepted);
  std::array<std::byte, 16> sdu{};
  CHECK(cop1_take_sdu(rx, sdu).value() == *pn);

  const auto n2 = cop1_bytes_to_send(rx, scratch);
  CHECK(n2.has_value());
  CHECK(*n2 > 0);
  CHECK(slot_write(wire.b_to_a, std::span<const std::byte>(scratch.data(), *n2))
            .has_value());
  const auto r2 = slot_read(wire.b_to_a, scratch);
  CHECK(r2.has_value());
  cop1_receive_bytes(tx, std::span<const std::byte>(scratch.data(), *r2));
  CHECK(tx.fop.nn_r == 1);
}

void test_radio_port_cross() {
  RadioPort a{};
  RadioPort b{};
  const std::array<std::byte, 4> frame{std::byte{0xFA}, std::byte{0xF3},
                                       std::byte{0x20}, std::byte{0x80}};
  CHECK(radio_begin_tx(a, frame).has_value());
  std::array<std::byte, 16> moved{};
  const auto t = radio_take_tx(a, moved);
  CHECK(t.has_value());
  CHECK(radio_offer_rx(b, std::span<const std::byte>(moved.data(), *t)).has_value());
  std::array<std::byte, 16> got{};
  const auto p = radio_poll_rx(b, got);
  CHECK(p.has_value());
  CHECK(*p == frame.size());
  CHECK(got[0] == std::byte{0xFA});
  CHECK(radio_take_tx(a, moved).value() == 0u);
}

void test_heap() {
  starcom::adapters::FrameSlot s{};
  const std::array<std::byte, 1> one{std::byte{0x11}};
  std::array<std::byte, 4> out{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  (void)slot_write(s, one);
  (void)slot_read(s, out);
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_loopback_tests() {
  test_slot_full_and_empty();
  test_cop1_over_loopback();
  test_radio_port_cross();
  test_heap();
  return g_fails;
}
