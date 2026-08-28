// IVP increment 4 + 10 — COP-1 (232.1 Tables 6-1 / 5-1). docs/TESTING.md
// FARM-1 E1–E11; FOP-1 E23 + S4/S5 + E29; Resume E30–E34; E35–E39; LLIF E41–E46.

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
using starcom::ccsds::cop1BytesToSend;
using starcom::ccsds::Cop1Endpoint;
using starcom::ccsds::Cop1Event;
using starcom::ccsds::cop1Init;
using starcom::ccsds::cop1InitiateAd;
using starcom::ccsds::cop1InitiateAdSetVr;
using starcom::ccsds::cop1InitiateAdUnlock;
using starcom::ccsds::cop1InitiateAdWithClcwCheck;
using starcom::ccsds::cop1TerminateAd;
using starcom::ccsds::Cop1Mib;
using starcom::ccsds::cop1PollEvent;
using starcom::ccsds::cop1ReceiveBytes;
using starcom::ccsds::cop1SubmitSdu;
using starcom::ccsds::cop1TakeSdu;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::Farm1;
using starcom::ccsds::Farm1Disposition;
using starcom::ccsds::farm1BufferRelease;
using starcom::ccsds::farm1Init;
using starcom::ccsds::Farm1Mib;
using starcom::ccsds::farm1OnAd;
using starcom::ccsds::farm1OnBd;
using starcom::ccsds::farm1OnSetVr;
using starcom::ccsds::farm1OnUnlock;
using starcom::ccsds::farm1Report;
using starcom::ccsds::Farm1State;
using starcom::ccsds::Fop1;
using starcom::ccsds::fop1Init;
using starcom::ccsds::fop1InitiateAd;
using starcom::ccsds::fop1InitiateAdSetVr;
using starcom::ccsds::fop1InitiateAdUnlock;
using starcom::ccsds::fop1InitiateAdWithClcwCheck;
using starcom::ccsds::fop1TerminateAd;
using starcom::ccsds::fop1ResumeAd;
using starcom::ccsds::fop1SetVs;
using starcom::ccsds::fop1SetK;
using starcom::ccsds::fop1SetTimeoutType;
using starcom::ccsds::fop1AdReject;
using starcom::ccsds::fop1NeedFrame;
using starcom::ccsds::fop1OnClcw;
using starcom::ccsds::Fop1Send;
using starcom::ccsds::Fop1State;
using starcom::ccsds::fop1Tick;
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
  farm1Init(f, mib);
  CHECK(f.state == Farm1State::s1_open);
  CHECK(f.v_r == 0);

  CHECK(farm1OnAd(f, 0, true) == Farm1Disposition::accepted);  // E1
  CHECK(f.v_r == 1);
  CHECK(!f.retransmit_flag);

  CHECK(farm1OnAd(f, 3, true) == Farm1Disposition::discarded);  // E3 gap
  CHECK(f.retransmit_flag);

  CHECK(farm1OnAd(f, 0, true) == Farm1Disposition::discarded);  // E4 negative

  CHECK(farm1OnAd(f, 1, true) == Farm1Disposition::accepted);
  CHECK(f.v_r == 2);
  CHECK(!f.retransmit_flag);

  CHECK(farm1OnBd(f) == Farm1Disposition::accepted);  // E6
  CHECK((f.farm_b_counter & 0x03) == 1);

  farm1OnAd(f, 100, true);  // E5 lockout
  CHECK(f.state == Farm1State::s3_lockout);
  CHECK(f.lockout_flag);

  farm1OnSetVr(f, 5);  // E8 in lockout: count only
  CHECK(f.state == Farm1State::s3_lockout);

  farm1OnUnlock(f);  // E7
  CHECK(f.state == Farm1State::s1_open);
  CHECK(!f.lockout_flag);

  farm1OnSetVr(f, 10);
  CHECK(f.v_r == 10);

  Farm1 w{};
  farm1Init(w, mib);
  CHECK(farm1OnAd(w, 0, false) == Farm1Disposition::discarded);  // E2
  CHECK(w.state == Farm1State::s2_wait);
  farm1BufferRelease(w);  // E10
  CHECK(w.state == Farm1State::s1_open);

  const auto clcw = farm1Report(f, 3);
  CHECK(clcw.cop_in_effect == 0b01);
  CHECK(clcw.vcid == 3);
  CHECK(clcw.report_value == 10);
}

void test_fop_1_initiate_and_ack() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(fop1InitiateAd(f));
  CHECK(f.state == Fop1State::s1_active);
  CHECK(!fop1InitiateAd(f));

  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  CHECK(f.last_send_ns == 0);
  CHECK(f.v_s == 1);

  Clcw32 ack{};
  ack.cop_in_effect = 0b01;
  ack.report_value = 1;
  fop1OnClcw(f, ack, true);
  CHECK(f.nn_r == 1);
  CHECK(f.sent_n == 0);
}

void test_fop_1_retransmit_flag() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);

  Clcw32 nak{};
  nak.cop_in_effect = 0b01;
  nak.retransmit = true;
  nak.report_value = 0;  // N(R)=NN(R)=0, N(R)<V(S)
  fop1OnClcw(f, nak, true);
  CHECK(f.state == Fop1State::s2_retransmit);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::resend_ad);
  CHECK(f.last_send_ns == 0);
}

void test_fop_1_s4_clcw_check() {
  Cop1Mib mib{};
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdWithClcwCheck(f));
  CHECK(f.state == Fop1State::s4_init_no_bc);
  CHECK(!fop1InitiateAd(f));
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 0;
  fop1OnClcw(f, w, true);
  CHECK(f.state == Fop1State::s1_active);
}

void test_fop_1_s5_unlock_and_terminate() {
  Cop1Mib mib{};
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdUnlock(f));
  CHECK(f.state == Fop1State::s5_init_bc);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::bc_unlock);
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 0;
  fop1OnClcw(f, w, true);
  CHECK(f.state == Fop1State::s1_active);
  fop1TerminateAd(f);
  CHECK(f.state == Fop1State::s6_initial);
}

void test_fop_1_s5_set_vr() {
  Cop1Mib mib{};
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdSetVr(f, 7));
  CHECK(f.v_s == 7);
  CHECK(f.nn_r == 7);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::bc_set_vr);
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 7;
  fop1OnClcw(f, w, true);
  CHECK(f.state == Fop1State::s1_active);
}

void test_host_loop_unlock() {
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.farm.w = 8;
  Cop1Endpoint tx{};
  Cop1Endpoint rx{};
  cop1Init(tx, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});
  cop1Init(rx, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  CHECK(cop1InitiateAdUnlock(tx));

  std::array<std::byte, 256> wire{};
  const auto n_bc = cop1BytesToSend(tx, wire);
  CHECK(n_bc.has_value());
  CHECK(*n_bc > 0);
  cop1ReceiveBytes(rx, std::span<const std::byte>(wire.data(), *n_bc));
  CHECK(rx.farm.state == Farm1State::s1_open);

  const auto n_clcw = cop1BytesToSend(rx, wire);
  CHECK(n_clcw.has_value());
  CHECK(*n_clcw > 0);
  cop1ReceiveBytes(tx, std::span<const std::byte>(wire.data(), *n_clcw));
  CHECK(tx.fop.state == Fop1State::s1_active);

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encodeSpacePacket(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(cop1SubmitSdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());
  const auto n_ad = cop1BytesToSend(tx, wire);
  CHECK(n_ad.has_value());
  CHECK(*n_ad > 0);
  cop1ReceiveBytes(rx, std::span<const std::byte>(wire.data(), *n_ad));
  CHECK(cop1PollEvent(rx) == Cop1Event::farm_accepted);
  cop1TerminateAd(tx);
  CHECK(tx.fop.state == Fop1State::s6_initial);
}

void test_host_loop() {
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

  std::array<std::byte, 256> wire{};
  const auto n1 = cop1BytesToSend(tx, wire);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  cop1ReceiveBytes(rx, std::span<const std::byte>(wire.data(), *n1));
  CHECK(cop1PollEvent(rx) == Cop1Event::farm_accepted);
  CHECK(rx.farm.v_r == 1);
  std::array<std::byte, 16> sdu{};
  const auto tn = cop1TakeSdu(rx, sdu);
  CHECK(tn.has_value());
  CHECK(*tn == *pn);

  const auto n2 = cop1BytesToSend(rx, wire);
  CHECK(n2.has_value());
  CHECK(*n2 > 0);
  cop1ReceiveBytes(tx, std::span<const std::byte>(wire.data(), *n2));
  CHECK(tx.fop.nn_r == 1);
  CHECK(tx.fop.sent_n == 0);
}

void test_fop_1_suspend_resume() {
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 1;
  mib.timeout_type = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(f.state == Fop1State::s1_active);
  CHECK(!fop1ResumeAd(f));  // E30 SS=0
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  fop1Tick(f, 1);   // arm deadline
  fop1Tick(f, 20);  // E18 S1 SS:=1 Suspend
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.ss == 1);
  CHECK(!f.alert_latched);
  CHECK(f.suspend_latched);
  CHECK(fop1ResumeAd(f));  // E31
  CHECK(f.ss == 0);
  CHECK(f.state == Fop1State::s1_active);
}

void test_fop_1_setup_and_llif() {
  Fop1 f{};
  Cop1Mib mib{};
  fop1Init(f, mib);
  CHECK(fop1SetVs(f, 7));  // E35 S6 SS=0
  CHECK(f.v_s == 7);
  CHECK(f.nn_r == 7);
  fop1SetK(f, 4);
  CHECK(f.mib.k == 4);
  fop1SetTimeoutType(f, 1);
  CHECK(f.mib.timeout_type == 1);
  CHECK(fop1InitiateAd(f));
  CHECK(!fop1SetVs(f, 0));  // E35 reject in S1
  fop1AdReject(f);          // E42 Alert [LLIF]
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.alert_latched);
}

void test_heap() {
  Cop1Mib mib{};
  Farm1 farm{};
  Fop1 fop{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  farm1Init(farm, mib.farm);
  fop1Init(fop, mib);
  (void)fop1InitiateAdUnlock(fop);
  (void)fop1NeedFrame(fop, false, false);
  (void)fop1InitiateAd(fop);
  (void)farm1OnAd(farm, 0, true);
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 1;
  fop1OnClcw(fop, w, true);
  fop1Tick(fop, 1);
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_cop1_tests() {
  test_farm_1_table();
  test_fop_1_initiate_and_ack();
  test_fop_1_retransmit_flag();
  test_fop_1_s4_clcw_check();
  test_fop_1_s5_unlock_and_terminate();
  test_fop_1_s5_set_vr();
  test_host_loop_unlock();
  test_host_loop();
  test_fop_1_suspend_resume();
  test_fop_1_setup_and_llif();
  test_heap();
  return g_fails;
}
