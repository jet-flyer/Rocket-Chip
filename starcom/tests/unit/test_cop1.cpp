// Host checks for COP-1.
// This file owns the check-fail count.
// No protocol state.
// Book: none.

// IVP increment 4 + 10 - COP-1 (232.1 Tables 6-1 / 5-1). docs/TESTING.md
// FARM-1 E1-E11; FOP-1 E23 + S4/S5 + E29; Resume E30-E34; E35-E39; LLIF E41-E46.

#include "heap_trap.h"
#include "starcom/ccsds/cop1.h"
#include "starcom/ccsds/pltu.h"
#include "starcom/ccsds/space_packet.h"
#include "starcom/ccsds/uslp.h"

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
using starcom::ccsds::decodePltu;
using starcom::ccsds::decodeUslp;
using starcom::ccsds::encodeClcw;
using starcom::ccsds::encodePltu;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::encodeUslp;
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
using starcom::ccsds::fop1SetT1Initial;
using starcom::ccsds::fop1SetTimeoutType;
using starcom::ccsds::fop1AdReject;
using starcom::ccsds::fop1NeedFrame;
using starcom::ccsds::fop1OnClcw;
using starcom::ccsds::Fop1Send;
using starcom::ccsds::Fop1State;
using starcom::ccsds::fop1Tick;
using starcom::ccsds::MapId;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::kCop1Hold;
using starcom::ccsds::kCop1SeqSlots;
using starcom::ccsds::kCop1SetVr0;
using starcom::ccsds::kCop1SetVr1;
using starcom::ccsds::kCop1Unlock;
using starcom::ccsds::kUslpConstructionNoSeg;
using starcom::ccsds::kUslpUpidCop1Control;
using starcom::ccsds::kUslpUpidSpacePacket;
using starcom::ccsds::UslpFields;
using starcom::ccsds::UslpScid;
using starcom::ccsds::Vcid;

namespace
{

int g_fails = 0;

#define CHECK(cond)                                                            \
  do                                                                           \
  {                                                                            \
    if (!(cond))                                                               \
    {                                                                          \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  }                                                                            \
  while (0)

void test_farm_1_table()
{
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

void test_fop_1_initiate_and_ack()
{
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

void test_fop_1_retransmit_flag()
{
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

void test_fop_1_s4_clcw_check()
{
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

void test_fop_1_s5_unlock_and_terminate()
{
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

void test_fop_1_s5_set_vr()
{
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

void test_host_loop_unlock()
{
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

void test_host_loop()
{
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

void test_fop_1_suspend_resume()
{
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

void test_fop_1_setup_and_llif()
{
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

bool resentTfdzIs(std::span<const std::byte> pltu,
                  std::span<const std::byte> packet)
{
  const auto view = decodePltu(pltu);
  if (!view)
  {
    return false;
  }
  const auto u = decodeUslp(view->frame);
  if (!u || u->tfdz.size() != packet.size())
  {
    return false;
  }
  return std::equal(packet.begin(), packet.end(), u->tfdz.begin());
}

void test_resend_ad_match_then_skip()
{
  // First resend matches sent[0] and leaves the loop. Second resend
  // skips sent[0] and matches sent[1]. Each TFDZ is that slot's packet.
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.farm.w = 8;
  Cop1Endpoint tx{};
  cop1Init(tx, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});
  CHECK(cop1InitiateAd(tx));

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user_a{std::byte{0xA1}};
  const std::array<std::byte, 1> user_b{std::byte{0xB2}};
  std::array<std::byte, 16> pkt_a{};
  std::array<std::byte, 16> pkt_b{};
  const auto na = encodeSpacePacket(pkt_a, sp, user_a);
  const auto nb = encodeSpacePacket(pkt_b, sp, user_b);
  CHECK(na.has_value());
  CHECK(nb.has_value());
  CHECK(cop1SubmitSdu(tx, std::span<const std::byte>(pkt_a.data(), *na), false)
            .has_value());
  std::array<std::byte, 256> wire{};
  const auto n0 = cop1BytesToSend(tx, wire);
  CHECK(n0.has_value());
  CHECK(*n0 > 0);
  CHECK(cop1SubmitSdu(tx, std::span<const std::byte>(pkt_b.data(), *nb), false)
            .has_value());
  const auto n1 = cop1BytesToSend(tx, wire);
  CHECK(n1.has_value());
  CHECK(*n1 > 0);
  CHECK(tx.fop.sent_n == 2);
  CHECK(tx.fop.sent[0] == 0);
  CHECK(tx.fop.sent[1] == 1);

  Clcw32 nak{};
  nak.cop_in_effect = 0b01;
  nak.retransmit = true;
  nak.report_value = 0;
  fop1OnClcw(tx.fop, nak, true);
  CHECK(tx.fop.state == Fop1State::s2_retransmit);

  const auto r0 = cop1BytesToSend(tx, wire);
  CHECK(r0.has_value());
  CHECK(*r0 > 0);
  CHECK(resentTfdzIs(std::span<const std::byte>(wire.data(), *r0),
                     std::span<const std::byte>(pkt_a.data(), *na)));

  const auto r1 = cop1BytesToSend(tx, wire);
  CHECK(r1.has_value());
  CHECK(*r1 > 0);
  CHECK(resentTfdzIs(std::span<const std::byte>(wire.data(), *r1),
                     std::span<const std::byte>(pkt_b.data(), *nb)));
  CHECK(tx.fop.sent_n == 2);
}

// First tick arms the deadline. The second tick is the deadline.
void expireArmedT1(Fop1& f)
{
  fop1Tick(f, 1);
  fop1Tick(f, 1 + f.mib.t1_initial);
}

void test_fop1_tick_t1_initial_zero()
{
  // T1_Initial 0 returns before a deadline is armed.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdWithClcwCheck(f));
  CHECK(f.t1_running);
  fop1SetT1Initial(f, 0);
  fop1Tick(f, 100);
  CHECK(f.state == Fop1State::s4_init_no_bc);
  CHECK(f.t1_deadline == 0);
  CHECK(!f.t1_expired_latched);
  CHECK(!f.alert_latched);
}

void test_fop1_tick_before_deadline()
{
  // A tick before the deadline does not expire T1.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  fop1Tick(f, 1);
  CHECK(f.t1_deadline == 11);
  fop1Tick(f, 10);
  CHECK(f.state == Fop1State::s1_active);
  CHECK(f.t1_deadline == 11);
  CHECK(f.t1_running);
  CHECK(!f.t1_expired_latched);
  CHECK(f.transmission_count == 1);
}

void test_fop1_tick_s5_retransmit()
{
  // E16/E104 S5 Initiate BC retransmission. The first Unlock goes out, then T1 sends it again.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdUnlock(f));
  CHECK(f.bc_to_send);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::bc_unlock);
  CHECK(!f.bc_to_send);
  fop1Tick(f, 1);
  fop1Tick(f, 11);
  CHECK(f.state == Fop1State::s5_init_bc);
  CHECK(f.transmission_count == 2);
  CHECK(f.bc_to_send);
  CHECK(f.t1_running);
  CHECK(f.t1_deadline == 0);
  CHECK(f.t1_expired_latched);
  CHECK(!f.alert_latched);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::bc_unlock);
}

void test_fop1_tick_s5_alert()
{
  // E17/E18 S5 Alert [T1], default timeout type: alerts and does not suspend.
  Cop1Mib mib_default{};
  mib_default.t1_initial = 10;
  mib_default.transmission_limit = 1;
  Fop1 f_default{};
  fop1Init(f_default, mib_default);
  CHECK(fop1InitiateAdUnlock(f_default));
  expireArmedT1(f_default);
  CHECK(f_default.state == Fop1State::s6_initial);
  CHECK(f_default.alert_latched);
  CHECK(!f_default.suspend_latched);
  CHECK(f_default.ss == 0);
  CHECK(!f_default.bc_to_send);
  CHECK(!f_default.t1_running);

  // E17/E18 S5 Alert [T1]. Timeout type 1 still alerts and does not suspend.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 1;
  mib.timeout_type = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdUnlock(f));
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.alert_latched);
  CHECK(!f.suspend_latched);
  CHECK(f.ss == 0);
  CHECK(!f.bc_to_send);
  CHECK(!f.t1_running);
}

void test_fop1_tick_s3_ignore()
{
  // Known deviation from 232.1 (noted for later, not fixed in this step):
  // S3 T1 at the limit is ignored. The book says Alert (E17) or SS:=3 Suspend (E18).
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  CHECK(f.transmission_count >= f.mib.transmission_limit);
  f.state = Fop1State::s3_retransmit_wait;
  const std::uint8_t count = f.transmission_count;
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s3_retransmit_wait);
  CHECK(f.transmission_count == count);
  CHECK(f.t1_expired_latched);
  CHECK(!f.t1_running);
  CHECK(f.ss == 0);
  CHECK(!f.alert_latched);
  CHECK(!f.suspend_latched);
}

void test_fop1_tick_s4_suspend()
{
  // E104/E18 S4. Suspend_State is 4.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.timeout_type = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdWithClcwCheck(f));
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.ss == 4);
  CHECK(f.suspend_latched);
  CHECK(!f.alert_latched);
  CHECK(!f.t1_running);
}

void test_fop1_tick_s4_alert()
{
  // E16/E17 S4 Alert [T1].
  Cop1Mib mib{};
  mib.t1_initial = 10;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAdWithClcwCheck(f));
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.alert_latched);
  CHECK(!f.suspend_latched);
  CHECK(f.ss == 0);
  CHECK(!f.t1_running);
}

void test_fop1_tick_at_limit_alert()
{
  // E17. The timeout type is not 1, so the limit alerts.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.alert_latched);
  CHECK(!f.suspend_latched);
  CHECK(f.ss == 0);
}

void test_fop1_tick_ss2_suspend()
{
  // E18. Suspend_State is 2 when the limit is reached in S2.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 4;
  mib.timeout_type = 1;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  // Setup sets state and transmission_count by hand; a real link may not reach this state.
  f.state = Fop1State::s2_retransmit;
  f.transmission_count = 4;
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s6_initial);
  CHECK(f.ss == 2);
  CHECK(f.suspend_latched);
  CHECK(!f.alert_latched);
}

void test_fop1_tick_s1_retransmit()
{
  // Known deviation from 232.1 (noted for later, not fixed in this step):
  // E16/E104 marks the sent frames and stays in S1, but fop1NeedFrame resends only in S2.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  CHECK(f.sent_n == 1);
  CHECK(!f.to_retransmit[0]);
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s1_active);
  CHECK(f.transmission_count == 2);
  CHECK(f.to_retransmit[0]);
  CHECK(f.t1_running);
  CHECK(f.t1_expired_latched);
  CHECK(!f.alert_latched);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::none);
  CHECK(f.to_retransmit[0]);
}

void test_fop1_tick_s2_retransmit()
{
  // E16/E104 S1/S2. Contrast for the S1 test: S2 does resend the marked AD frame.
  Cop1Mib mib{};
  mib.t1_initial = 10;
  mib.transmission_limit = 4;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  // Setup sets state by hand; a real link may not reach this state.
  f.state = Fop1State::s2_retransmit;
  expireArmedT1(f);
  CHECK(f.state == Fop1State::s2_retransmit);
  CHECK(f.transmission_count == 2);
  CHECK(f.to_retransmit[0]);
  CHECK(f.t1_running);
  CHECK(!f.alert_latched);
  CHECK(fop1NeedFrame(f, false, false) == Fop1Send::resend_ad);
  CHECK(f.last_send_ns == 0);
  CHECK(!f.to_retransmit[0]);
}

UslpFields rxHdr(std::uint64_t ns)
{
  UslpFields hdr{};
  hdr.scid = UslpScid{1};
  hdr.destination = true;
  hdr.vcid = Vcid{1};
  hdr.map_id = MapId{0};
  hdr.vcf_count_len = 1;
  hdr.vcf_count = ns;
  hdr.tfdz_construction = kUslpConstructionNoSeg;
  hdr.upid = kUslpUpidSpacePacket;
  return hdr;
}

bool deliverFrame(Cop1Endpoint& e, UslpFields const& hdr,
                  std::span<const std::byte> tfdz,
                  std::span<const std::byte> ocf)
{
  std::array<std::byte, 160> frame{};
  const auto n = encodeUslp(frame, hdr, tfdz, ocf);
  if (!n)
  {
    return false;
  }
  std::array<std::byte, 192> wire{};
  const auto p = encodePltu(wire, std::span<const std::byte>(frame.data(), *n));
  if (!p)
  {
    return false;
  }
  cop1ReceiveBytes(e, std::span<const std::byte>(wire.data(), *p));
  return true;
}

Cop1Endpoint freshRx()
{
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.farm.w = 8;
  Cop1Endpoint e{};
  cop1Init(e, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  return e;
}

void test_cop1_rx_bad_pltu()
{
  // A bad PLTU calls FARM-1 invalid and changes nothing.
  Cop1Endpoint e = freshRx();
  e.farm.v_r = 3;
  const std::array<std::byte, 4> junk{std::byte{1}, std::byte{2}, std::byte{3},
                                      std::byte{4}};
  cop1ReceiveBytes(e, junk);
  CHECK(e.farm.v_r == 3);
  CHECK(e.farm.state == Farm1State::s1_open);
  CHECK(e.rx_n == 0);
  CHECK(!e.farm_accepted_latched);
}

void test_cop1_rx_bad_uslp()
{
  // A PLTU whose USLP frame is too short to decode calls FARM-1 invalid.
  Cop1Endpoint e = freshRx();
  e.farm.v_r = 3;
  std::array<std::byte, 7> frame{};
  frame[0] = std::byte{0xC0};
  frame[5] = std::byte{6};
  std::array<std::byte, 32> wire{};
  const auto p = encodePltu(wire, frame);
  CHECK(p.has_value());
  cop1ReceiveBytes(e, std::span<const std::byte>(wire.data(), *p));
  CHECK(e.farm.v_r == 3);
  CHECK(e.rx_n == 0);
  CHECK(!e.farm_accepted_latched);
}

void test_cop1_rx_no_clcw()
{
  // An AD frame with no OCF is accepted and leaves the outstanding FOP-1 frame unchanged.
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.t1_initial = 10;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  CHECK(f.state == Fop1State::s1_active);
  CHECK(f.sent_n == 1);
  Cop1Endpoint e = freshRx();
  e.fop = f;
  const std::array<std::byte, 1> user{std::byte{0x11}};
  CHECK(deliverFrame(e, rxHdr(0), user, {}));
  CHECK(e.farm.v_r == 1);
  CHECK(e.rx_n == 1);
  CHECK(e.rx_len[0] == 1);
  CHECK(e.rx_q[0][0] == std::byte{0x11});
  CHECK(e.farm_accepted_latched);
  CHECK(e.fop.state == Fop1State::s1_active);
  CHECK(e.fop.sent_n == 1);
  CHECK(e.fop.v_s == 1);
  CHECK(e.fop.nn_r == 0);
  CHECK(e.fop.last_send_ns == 0);
  CHECK(!e.fop.alert_latched);
}

void test_cop1_rx_clcw_valid()
{
  // E2 in S1. N(R) equals V(S) and is not NN(R), so NN(R) advances and T1 is cancelled.
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.t1_initial = 10;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  CHECK(f.t1_running);
  Cop1Endpoint e = freshRx();
  e.fop = f;
  Clcw32 w{};
  w.cop_in_effect = 0b01;
  w.report_value = 1;
  std::array<std::byte, 4> ocf{};
  CHECK(encodeClcw(ocf, w).has_value());
  UslpFields hdr = rxHdr(0);
  hdr.ocf_present = true;
  const std::array<std::byte, 1> user{std::byte{0x22}};
  CHECK(deliverFrame(e, hdr, user, ocf));
  CHECK(e.fop.nn_r == 1);
  CHECK(e.fop.sent_n == 0);
  CHECK(e.fop.state == Fop1State::s1_active);
  CHECK(!e.fop.t1_running);
  CHECK(e.fop.t1_deadline == 0);
  CHECK(!e.fop.alert_latched);
  CHECK(e.farm.v_r == 1);
  CHECK(e.rx_n == 1);
  CHECK(e.rx_q[0][0] == std::byte{0x22});
}

void test_cop1_rx_clcw_bad_format()
{
  // COP in effect is not 01 (pre-check, 232.1 5.3.2.1). The book gives no action; the code alerts as E15 [CLCW]. Project behavior. FARM-1 still accepts the AD frame.
  Cop1Mib mib{};
  mib.k = 4;
  mib.transmission_limit = 4;
  mib.t1_initial = 10;
  Fop1 f{};
  fop1Init(f, mib);
  CHECK(fop1InitiateAd(f));
  CHECK(fop1NeedFrame(f, false, true) == Fop1Send::ad);
  Cop1Endpoint e = freshRx();
  e.fop = f;
  Clcw32 w{};
  w.cop_in_effect = 0;
  w.report_value = 1;
  std::array<std::byte, 4> ocf{};
  CHECK(encodeClcw(ocf, w).has_value());
  UslpFields hdr = rxHdr(0);
  hdr.ocf_present = true;
  const std::array<std::byte, 1> user{std::byte{0x33}};
  CHECK(deliverFrame(e, hdr, user, ocf));
  CHECK(e.fop.state == Fop1State::s6_initial);
  CHECK(e.fop.alert_latched);
  CHECK(e.farm.v_r == 1);
  CHECK(e.rx_n == 1);
  CHECK(e.rx_q[0][0] == std::byte{0x33});

  // COP in effect pre-check failure in S6 is ignored (project behavior). FARM-1 still accepts the AD frame.
  Cop1Endpoint idle = freshRx();
  CHECK(idle.fop.state == Fop1State::s6_initial);
  CHECK(deliverFrame(idle, hdr, user, ocf));
  CHECK(idle.fop.state == Fop1State::s6_initial);
  CHECK(!idle.fop.alert_latched);
  CHECK(idle.farm.v_r == 1);
  CHECK(idle.rx_n == 1);
  CHECK(idle.rx_q[0][0] == std::byte{0x33});
}

void test_cop1_rx_bc_set_vr()
{
  // BC SET V(R) sets V(R) and does not queue the frame.
  Cop1Endpoint e = freshRx();
  UslpFields hdr = rxHdr(0);
  hdr.expedited = true;
  hdr.protocol_control = true;
  hdr.upid = kUslpUpidCop1Control;
  const std::array<std::byte, 3> dir{kCop1SetVr0, kCop1SetVr1, std::byte{9}};
  CHECK(deliverFrame(e, hdr, dir, {}));
  CHECK(e.farm.v_r == 9);
  CHECK(e.farm.state == Farm1State::s1_open);
  CHECK((e.farm.farm_b_counter & 0x03U) == 1);
  CHECK(e.rx_n == 0);
  CHECK(!e.farm_accepted_latched);
}

void test_cop1_rx_bc_invalid()
{
  // An unknown BC directive is invalid and does not queue the frame.
  UslpFields hdr = rxHdr(0);
  hdr.expedited = true;
  hdr.protocol_control = true;
  hdr.upid = kUslpUpidCop1Control;
  const std::array<std::byte, 1> one{std::byte{0x7F}};
  const std::array<std::byte, 3> bad_first{std::byte{0x11}, kCop1SetVr1, std::byte{9}};
  const std::array<std::byte, 3> bad_second{kCop1SetVr0, std::byte{0x01}, std::byte{9}};
  const std::span<const std::byte> dirs[] = {one, bad_first, bad_second};
  for (const std::span<const std::byte> dir : dirs)
  {
    Cop1Endpoint e = freshRx();
    CHECK(deliverFrame(e, hdr, dir, {}));
    CHECK(e.farm.v_r == 0);
    CHECK(e.farm.state == Farm1State::s1_open);
    CHECK(e.farm.farm_b_counter == 0);
    CHECK(e.rx_n == 0);
    CHECK(!e.farm_accepted_latched);
  }
}

void test_cop1_rx_bd()
{
  // A BD frame is accepted and queued.
  Cop1Endpoint e = freshRx();
  UslpFields hdr = rxHdr(0);
  hdr.expedited = true;
  const std::array<std::byte, 1> user{std::byte{0xBD}};
  CHECK(deliverFrame(e, hdr, user, {}));
  CHECK(e.farm.v_r == 0);
  CHECK((e.farm.farm_b_counter & 0x03U) == 1);
  CHECK(e.rx_n == 1);
  CHECK(e.rx_q[0][0] == std::byte{0xBD});
  CHECK(e.farm_accepted_latched);
}

void test_cop1_rx_ad_not_accepted()
{
  // E3. V(R)=1, N(S)=3, and PW=4, so the frame is inside the positive window.
  Cop1Endpoint e = freshRx();
  const std::array<std::byte, 1> user{std::byte{0x01}};
  CHECK(deliverFrame(e, rxHdr(0), user, {}));
  CHECK(e.farm.v_r == 1);
  (void)cop1PollEvent(e);
  CHECK(deliverFrame(e, rxHdr(3), user, {}));
  CHECK(e.farm.v_r == 1);
  CHECK(e.farm.retransmit_flag);
  CHECK(e.rx_n == 1);
  CHECK(!e.farm_accepted_latched);
}

void test_cop1_rx_reserved()
{
  // Bypass 0 with control 1 is reserved (232.0 Table 4-1), so the frame is discarded.
  Cop1Endpoint e = freshRx();
  UslpFields hdr = rxHdr(0);
  hdr.protocol_control = true;
  const std::array<std::byte, 1> user{std::byte{0x44}};
  CHECK(deliverFrame(e, hdr, user, {}));
  CHECK(e.farm.v_r == 0);
  CHECK(e.farm.state == Farm1State::s1_open);
  CHECK(e.farm.farm_b_counter == 0);
  CHECK(e.rx_n == 0);
  CHECK(!e.farm_accepted_latched);
}

void test_cop1_rx_queue_full()
{
  // Known deviation from 232.1 (noted for later, not fixed in this step):
  // a BD frame accepted with the queue full is dropped. 232.1 3.3 says the new BD FDU replaces the buffered data.
  Cop1Endpoint e = freshRx();
  const std::array<std::byte, 1> user{std::byte{0x01}};
  for (std::uint64_t ns = 0; ns < kCop1SeqSlots; ++ns)
  {
    CHECK(deliverFrame(e, rxHdr(ns), user, {}));
  }
  CHECK(e.rx_n == kCop1SeqSlots);
  (void)cop1PollEvent(e);
  UslpFields hdr = rxHdr(0);
  hdr.expedited = true;
  CHECK(deliverFrame(e, hdr, user, {}));
  CHECK(e.rx_n == kCop1SeqSlots);
  CHECK(e.farm_accepted_latched);
  CHECK((e.farm.farm_b_counter & 0x03U) == 1);
}

void test_cop1_rx_oversize()
{
  // Known deviation from 232.1 (noted for later, not fixed in this step):
  // An AD frame longer than 64 bytes is accepted, V(R) advances, and the data is dropped.
  Cop1Endpoint e = freshRx();
  std::array<std::byte, kCop1Hold + 1> user{};
  user[0] = std::byte{0x55};
  CHECK(deliverFrame(e, rxHdr(0), user, {}));
  CHECK(e.farm.v_r == 1);
  CHECK(e.farm_accepted_latched);
  CHECK(e.rx_n == 0);
}

void test_cop1_rx_wait_no_release()
{
  // Known deviation from 232.1 (noted for later, not fixed in this step):
  // After E2 the FARM stays in Wait when a slot is freed. Unlock leaves Wait.
  Cop1Endpoint e = freshRx();
  const std::array<std::byte, 1> user{std::byte{0x01}};
  for (std::uint64_t ns = 0; ns < kCop1SeqSlots; ++ns)
  {
    CHECK(deliverFrame(e, rxHdr(ns), user, {}));
  }
  CHECK(e.rx_n == kCop1SeqSlots);
  CHECK(e.farm.v_r == kCop1SeqSlots);
  CHECK(deliverFrame(e, rxHdr(kCop1SeqSlots), user, {}));
  CHECK(e.farm.state == Farm1State::s2_wait);
  CHECK(e.farm.wait_flag);
  CHECK(e.farm.v_r == kCop1SeqSlots);
  CHECK(e.rx_n == kCop1SeqSlots);
  std::array<std::byte, 8> out{};
  const auto taken = cop1TakeSdu(e, out);
  CHECK(taken.has_value());
  CHECK(*taken == 1);
  CHECK(e.rx_n == kCop1SeqSlots - 1);
  CHECK(e.farm.state == Farm1State::s2_wait);
  CHECK(e.farm.wait_flag);
  UslpFields hdr = rxHdr(0);
  hdr.expedited = true;
  hdr.protocol_control = true;
  hdr.upid = kUslpUpidCop1Control;
  const std::array<std::byte, 1> unlock{kCop1Unlock};
  CHECK(deliverFrame(e, hdr, unlock, {}));
  CHECK(e.farm.state == Farm1State::s1_open);
  CHECK(!e.farm.wait_flag);
}

void test_heap()
{
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

int run_cop1_tests()
{
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
  test_resend_ad_match_then_skip();
  test_fop1_tick_t1_initial_zero();
  test_fop1_tick_before_deadline();
  test_fop1_tick_s5_retransmit();
  test_fop1_tick_s5_alert();
  test_fop1_tick_s3_ignore();
  test_fop1_tick_s4_suspend();
  test_fop1_tick_s4_alert();
  test_fop1_tick_at_limit_alert();
  test_fop1_tick_ss2_suspend();
  test_fop1_tick_s1_retransmit();
  test_fop1_tick_s2_retransmit();
  test_cop1_rx_bad_pltu();
  test_cop1_rx_bad_uslp();
  test_cop1_rx_no_clcw();
  test_cop1_rx_clcw_valid();
  test_cop1_rx_clcw_bad_format();
  test_cop1_rx_bc_set_vr();
  test_cop1_rx_bc_invalid();
  test_cop1_rx_bd();
  test_cop1_rx_ad_not_accepted();
  test_cop1_rx_reserved();
  test_cop1_rx_queue_full();
  test_cop1_rx_oversize();
  test_cop1_rx_wait_no_release();
  test_heap();
  return g_fails;
}
