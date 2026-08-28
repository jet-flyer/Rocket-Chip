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
using starcom::adapters::radioBeginTx;
using starcom::adapters::radioOfferRx;
using starcom::adapters::radioPollRx;
using starcom::adapters::radioTakeTx;
using starcom::adapters::RadioPort;
using starcom::adapters::slotRead;
using starcom::adapters::slotWrite;
using starcom::ccsds::cop1BytesToSend;
using starcom::ccsds::Cop1Endpoint;
using starcom::ccsds::Cop1Event;
using starcom::ccsds::cop1Init;
using starcom::ccsds::cop1InitiateAd;
using starcom::ccsds::Cop1Mib;
using starcom::ccsds::cop1PollEvent;
using starcom::ccsds::cop1ReceiveBytes;
using starcom::ccsds::cop1SubmitSdu;
using starcom::ccsds::cop1TakeSdu;
using starcom::ccsds::encodeSpacePacket;
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
  CHECK(slotWrite(s, a).value() == 3u);
  CHECK(slotWrite(s, a).error() == Error::buffer_too_small);
  std::array<std::byte, 8> out{};
  CHECK(slotRead(s, out).value() == 3u);
  CHECK(slotRead(s, out).value() == 0u);
}

void test_cop1_over_loopback() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.farm.w = 8;
  Cop1Endpoint tx{};
  Cop1Endpoint rx{};
  cop1Init(tx, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});
  cop1Init(rx, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  CHECK(cop1InitiateAd(tx));

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encodeSpacePacket(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(cop1SubmitSdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());

  HostLoopback wire{};
  std::array<std::byte, kAdapterFrameMax> scratch{};

  const auto n1 = cop1BytesToSend(tx, scratch);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  CHECK(slotWrite(wire.a_to_b, std::span<const std::byte>(scratch.data(), *n1))
            .has_value());
  const auto r1 = slotRead(wire.a_to_b, scratch);
  CHECK(r1.has_value());
  cop1ReceiveBytes(rx, std::span<const std::byte>(scratch.data(), *r1));
  CHECK(cop1PollEvent(rx) == Cop1Event::farm_accepted);
  std::array<std::byte, 16> sdu{};
  CHECK(cop1TakeSdu(rx, sdu).value() == *pn);

  const auto n2 = cop1BytesToSend(rx, scratch);
  CHECK(n2.has_value());
  CHECK(*n2 > 0);
  CHECK(slotWrite(wire.b_to_a, std::span<const std::byte>(scratch.data(), *n2))
            .has_value());
  const auto r2 = slotRead(wire.b_to_a, scratch);
  CHECK(r2.has_value());
  cop1ReceiveBytes(tx, std::span<const std::byte>(scratch.data(), *r2));
  CHECK(tx.fop.nn_r == 1);
}

void test_radio_port_cross() {
  RadioPort a{};
  RadioPort b{};
  const std::array<std::byte, 4> frame{std::byte{0xFA}, std::byte{0xF3},
                                       std::byte{0x20}, std::byte{0x80}};
  CHECK(radioBeginTx(a, frame).has_value());
  std::array<std::byte, 16> moved{};
  const auto t = radioTakeTx(a, moved);
  CHECK(t.has_value());
  CHECK(radioOfferRx(b, std::span<const std::byte>(moved.data(), *t)).has_value());
  std::array<std::byte, 16> got{};
  const auto p = radioPollRx(b, got);
  CHECK(p.has_value());
  CHECK(*p == frame.size());
  CHECK(got[0] == std::byte{0xFA});
  CHECK(radioTakeTx(a, moved).value() == 0u);
}

void test_heap() {
  starcom::adapters::FrameSlot s{};
  const std::array<std::byte, 1> one{std::byte{0x11}};
  std::array<std::byte, 4> out{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  (void)slotWrite(s, one);
  (void)slotRead(s, out);
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_loopback_tests() {
  test_slot_full_and_empty();
  test_cop1_over_loopback();
  test_radio_port_cross();
  test_heap();
  return g_fails;
}
