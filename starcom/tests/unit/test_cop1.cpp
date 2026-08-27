// IVP increment 4 — COP-1 (232.1 Tables 6-1 / 5-1 subset). docs/TESTING.md
// FARM-1 E1–E11; FOP-1 E23 + AD send/ack + E8 retransmit; USLP+CLCW host loop.
// S4/S5 BC-init and full 46-event FOP-1 table are not this sitting.

#include "heap_trap.hpp"
#include "starcom/ccsds/cop1.hpp"
#include "starcom/ccsds/space_packet.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::Clcw32;
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
using starcom::ccsds::Farm1;
using starcom::ccsds::Farm1Disposition;
using starcom::ccsds::farm_1_buffer_release;
using starcom::ccsds::farm_1_init;
using starcom::ccsds::Farm1Mib;
using starcom::ccsds::farm_1_on_ad;
using starcom::ccsds::farm_1_on_bd;
using starcom::ccsds::farm_1_on_set_vr;
using starcom::ccsds::farm_1_on_unlock;
using starcom::ccsds::farm_1_report;
using starcom::ccsds::Farm1State;
using starcom::ccsds::Fop1;
using starcom::ccsds::fop_1_init;
using starcom::ccsds::fop_1_initiate_ad;
using starcom::ccsds::fop_1_need_frame;
using starcom::ccsds::fop_1_on_clcw;
using starcom::ccsds::Fop1Send;
using starcom::ccsds::Fop1State;
using starcom::ccsds::fop_1_tick;
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

void test_farm_1_table() {
  Farm1Mib mib{};
  mib.w = 8;  // PW = NW = 4
  Farm1 f{};
  farm_1_init(f, mib);
  CHECK(f.state == Farm1State::s1_open);
  CHECK(f.v_r == 0);

  CHECK(farm_1_on_ad(f, 0, true) == Farm1Disposition::accepted);  // E1
  CHECK(f.v_r == 1);
  CHECK(!f.retransmit_flag);

  CHECK(farm_1_on_ad(f, 3, true) == Farm1Disposition::discarded);  // E3 gap
  CHECK(f.retransmit_flag);

  CHECK(farm_1_on_ad(f, 0, true) == Farm1Disposition::discarded);  // E4 negative

  CHECK(farm_1_on_ad(f, 1, true) == Farm1Disposition::accepted);
  CHECK(f.v_r == 2);
  CHECK(!f.retransmit_flag);

  CHECK(farm_1_on_bd(f) == Farm1Disposition::accepted);  // E6
  CHECK((f.farm_b_counter & 0x03) == 1);

  farm_1_on_ad(f, 100, true);  // E5 lockout
  CHECK(f.state == Farm1State::s3_lockout);
  CHECK(f.lockout_flag);

  farm_1_on_set_vr(f, 5);  // E8 in lockout: count only
  CHECK(f.state == Farm1State::s3_lockout);

  farm_1_on_unlock(f);  // E7
  CHECK(f.state == Farm1State::s1_open);
  CHECK(!f.lockout_flag);

  farm_1_on_set_vr(f, 10);
  CHECK(f.v_r == 10);

  Farm1 w{};
  farm_1_init(w, mib);
  CHECK(farm_1_on_ad(w, 0, false) == Farm1Disposition::discarded);  // E2
  CHECK(w.state == Farm1State::s2_wait);
  farm_1_buffer_release(w);  // E10
  CHECK(w.state == Farm1State::s1_open);

  const auto clcw = farm_1_report(f, 3);
  CHECK(clcw.cop_in_effect == 0b01);
  CHECK(clcw.vcid == 3);
  CHECK(clcw.report_value == 10);
}

void test_fop_1_initiate_and_ack() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop_1_init(f, mib);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(fop_1_initiate_ad(f));
  CHECK(f.state == Fop1State::s1_active);
  CHECK(!fop_1_initiate_ad(f));

  CHECK(fop_1_need_frame(f, false, true) == Fop1Send::ad);
  CHECK(f.last_send_ns == 0);
  CHECK(f.v_s == 1);

  Clcw32 ack{};
  ack.cop_in_effect = 0b01;
  ack.report_value = 1;
  fop_1_on_clcw(f, ack, true);
  CHECK(f.nn_r == 1);
  CHECK(f.sent_n == 0);
}

void test_fop_1_retransmit_flag() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop_1_init(f, mib);
  CHECK(fop_1_initiate_ad(f));
  CHECK(fop_1_need_frame(f, false, true) == Fop1Send::ad);

  Clcw32 nak{};
  nak.cop_in_effect = 0b01;
  nak.retransmit = true;
  nak.report_value = 0;  // N(R)=NN(R)=0, N(R)<V(S)
  fop_1_on_clcw(f, nak, true);
  CHECK(f.state == Fop1State::s2_retransmit);
  CHECK(fop_1_need_frame(f, false, false) == Fop1Send::resend_ad);
  CHECK(f.last_send_ns == 0);
}

void test_host_loop() {
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

  std::array<std::byte, 256> wire{};
  const auto n1 = cop1_bytes_to_send(tx, wire);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  cop1_receive_bytes(rx, std::span<const std::byte>(wire.data(), *n1));
  CHECK(cop1_poll_event(rx) == Cop1Event::farm_accepted);
  CHECK(rx.farm.v_r == 1);
  std::array<std::byte, 16> sdu{};
  const auto tn = cop1_take_sdu(rx, sdu);
  CHECK(tn.has_value());
  CHECK(*tn == *pn);

  const auto n2 = cop1_bytes_to_send(rx, wire);
  CHECK(n2.has_value());
  CHECK(*n2 > 0);
  cop1_receive_bytes(tx, std::span<const std::byte>(wire.data(), *n2));
  CHECK(tx.fop.nn_r == 1);
  CHECK(tx.fop.sent_n == 0);
}

void test_heap() {
  Cop1Mib mib{};
  Farm1 farm{};
  Fop1 fop{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  farm_1_init(farm, mib.farm);
  fop_1_init(fop, mib);
  (void)fop_1_initiate_ad(fop);
  (void)farm_1_on_ad(farm, 0, true);
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 1;
  fop_1_on_clcw(fop, w, true);
  fop_1_tick(fop, 1);
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_cop1_tests() {
  test_farm_1_table();
  test_fop_1_initiate_and_ack();
  test_fop_1_retransmit_flag();
  test_host_loop();
  test_heap();
  return g_fails;
}
