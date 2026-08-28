// IVP increment 2 — COP-P (211.0 §7 tables). docs/TESTING.md
// FARM-P RE0–RE6; FOP-P SE0–SE4, SE7; canned PLTU → PLCW; host loop.

#include "heap_trap.hpp"
#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/space_packet.hpp"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

using starcom::ccsds::coppBytesToSend;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppEvent;
using starcom::ccsds::coppInit;
using starcom::ccsds::coppInitUslp;
using starcom::ccsds::CoppMib;
using starcom::ccsds::coppPollEvent;
using starcom::ccsds::coppReceiveBytes;
using starcom::ccsds::coppSubmitSdu;
using starcom::ccsds::coppTakeSdu;
using starcom::ccsds::encodeSpacePacket;
using starcom::ccsds::FarmP;
using starcom::ccsds::FarmPDisposition;
using starcom::ccsds::farmPInit;
using starcom::ccsds::farmPOnFrame;
using starcom::ccsds::farmPReport;
using starcom::ccsds::farmPSetVr;
using starcom::ccsds::FopP;
using starcom::ccsds::FopPSend;
using starcom::ccsds::FopPState;
using starcom::ccsds::fopPInit;
using starcom::ccsds::fopPNeedFrame;
using starcom::ccsds::fopPOnPlcw;
using starcom::ccsds::fopPReset;
using starcom::ccsds::fopPTick;
using starcom::ccsds::Pcid;
using starcom::ccsds::Plcw16;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::seqLt;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::Tick;
using starcom::ccsds::UslpScid;
using starcom::ccsds::Vcid;
using starcom::ccsds::MapId;

namespace {

int g_fails = 0;

#define CHECK(cond)                                                            \
  do {                                                                         \
    if (!(cond)) {                                                             \
      std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);     \
      ++g_fails;                                                               \
    }                                                                          \
  } while (0)

void test_seq_compare() {
  CHECK(!seqLt(0, 0));
  CHECK(seqLt(0, 1));
  CHECK(seqLt(255, 0));
  CHECK(!seqLt(0, 255));
  CHECK(seqLt(10, 20));
  CHECK(!seqLt(20, 10));
}

void test_farm_p_table() {
  FarmP f{};
  farmPInit(f);
  CHECK(f.v_r == 0);
  CHECK(!f.r_s);
  CHECK(f.need_plcw);

  CHECK(farmPOnFrame(f, false, false, 0) == FarmPDisposition::discarded);

  CHECK(farmPOnFrame(f, true, true, 9) == FarmPDisposition::accepted);
  CHECK(f.expedited_frame_counter == 1);

  CHECK(farmPOnFrame(f, true, false, 0) == FarmPDisposition::accepted);
  CHECK(f.v_r == 1);
  CHECK(!f.r_s);

  CHECK(farmPOnFrame(f, true, false, 3) == FarmPDisposition::discarded);
  CHECK(f.r_s);

  CHECK(farmPOnFrame(f, true, false, 0) == FarmPDisposition::discarded);

  CHECK(farmPOnFrame(f, true, false, 1) == FarmPDisposition::accepted);
  CHECK(f.v_r == 2);
  CHECK(!f.r_s);

  farmPSetVr(f, 10);
  CHECK(f.v_r == 10);
  CHECK(!f.r_s);

  const auto w = farmPReport(f, Pcid{1});
  CHECK(w.report_value == 10);
  CHECK(w.pcid == Pcid{1});
  CHECK(!w.retransmit);
}

void test_fop_p_send_and_ack() {
  CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 10;
  FopP f{};
  fopPInit(f, mib);
  CHECK(f.state == FopPState::s1_active);

  CHECK(fopPNeedFrame(f, false, true) == FopPSend::new_seq);
  CHECK(f.last_send_fsn == 0);
  CHECK(f.v_s == 1);
  CHECK(f.sent_n == 1);

  CHECK(fopPNeedFrame(f, true, false) == FopPSend::expedited);
  CHECK(f.last_send_fsn == 0);
  CHECK(f.ve_s == 1);

  // SE1 progressive retransmission: window full or no new SEQ → resend from NN(R).
  FopP g{};
  CoppMib w1{};
  w1.transmission_window = 1;
  fopPInit(g, w1);
  CHECK(fopPNeedFrame(g, false, true) == FopPSend::new_seq);
  CHECK(fopPNeedFrame(g, false, true) == FopPSend::resend_seq);
  CHECK(g.last_send_fsn == 0);
  CHECK(g.v_s == 1);

  Plcw16 ack{};
  ack.report_value = 1;
  fopPOnPlcw(f, ack, true);
  CHECK(f.nn_r == 1);
  CHECK(f.sent_n == 0);
}

void test_fop_p_invalid_plcw_and_timer() {
  CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 5;
  mib.resync_local = true;
  FopP f{};
  fopPInit(f, mib);
  CHECK(fopPNeedFrame(f, false, true) == FopPSend::new_seq);

  Plcw16 bad{};
  bad.report_value = 9;  // > V(S)=1
  fopPOnPlcw(f, bad, true);
  CHECK(f.synch_running);

  fopPTick(f, 0);
  CHECK(f.state == FopPState::s1_active);
  fopPTick(f, 5);
  CHECK(f.synch_expired_latched);
  CHECK(f.state == FopPState::s2_resync);
  CHECK(fopPNeedFrame(f, false, true) == FopPSend::none);

  Plcw16 ok{};
  ok.report_value = 0;
  fopPOnPlcw(f, ok, true);
  CHECK(f.state == FopPState::s1_active);

  fopPReset(f);
  CHECK(f.v_s == 0);
  CHECK(f.state == FopPState::s1_active);
}

void test_canned_pltu_to_plcw() {
  CoppMib mib{};
  mib.transmission_window = 4;
  CoppEndpoint rx{};
  coppInit(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{1});
  CoppEndpoint tx{};
  coppInit(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{1});

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encodeSpacePacket(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(coppSubmitSdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());

  std::array<std::byte, 128> wire{};
  // Both sides start with NEED_PLCW (RE0 / SE0). Drain those P-frames first.
  const auto p0 = coppBytesToSend(rx, wire);
  CHECK(p0.has_value());
  CHECK(*p0 > 0);
  coppReceiveBytes(tx, std::span<const std::byte>(wire.data(), *p0));
  const auto t0 = coppBytesToSend(tx, wire);
  CHECK(t0.has_value());
  CHECK(*t0 > 0);
  coppReceiveBytes(rx, std::span<const std::byte>(wire.data(), *t0));

  const auto p1 = coppBytesToSend(tx, wire);
  CHECK(p1.has_value());
  CHECK(*p1 > 0);
  coppReceiveBytes(rx, std::span<const std::byte>(wire.data(), *p1));
  CHECK(coppPollEvent(rx) == CoppEvent::farm_accepted);
  CHECK(rx.farm.v_r == 1);
  std::array<std::byte, 16> sdu{};
  const auto tn = coppTakeSdu(rx, sdu);
  CHECK(tn.has_value());
  CHECK(*tn == *pn);
  CHECK(std::equal(pkt.begin(), pkt.begin() + static_cast<std::ptrdiff_t>(*pn),
                   sdu.begin()));

  const auto p2 = coppBytesToSend(rx, wire);
  CHECK(p2.has_value());
  coppReceiveBytes(tx, std::span<const std::byte>(wire.data(), *p2));
  CHECK(tx.fop.sent_n == 0);
  CHECK(tx.fop.nn_r == 1);
}

void test_uslp_host_loop() {
  CoppMib mib{};
  mib.transmission_window = 4;
  CoppEndpoint rx{};
  CoppEndpoint tx{};
  coppInitUslp(rx, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  coppInitUslp(tx, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encodeSpacePacket(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(coppSubmitSdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());

  std::array<std::byte, 128> wire{};
  const auto p0 = coppBytesToSend(rx, wire);
  CHECK(p0.has_value());
  CHECK(*p0 > 0);
  coppReceiveBytes(tx, std::span<const std::byte>(wire.data(), *p0));
  const auto t0 = coppBytesToSend(tx, wire);
  CHECK(t0.has_value());
  CHECK(*t0 > 0);
  coppReceiveBytes(rx, std::span<const std::byte>(wire.data(), *t0));

  const auto p1 = coppBytesToSend(tx, wire);
  CHECK(p1.has_value());
  CHECK(*p1 > 0);
  coppReceiveBytes(rx, std::span<const std::byte>(wire.data(), *p1));
  CHECK(coppPollEvent(rx) == CoppEvent::farm_accepted);
  std::array<std::byte, 16> sdu{};
  const auto tn = coppTakeSdu(rx, sdu);
  CHECK(tn.has_value());
  CHECK(*tn == *pn);
}

void test_heap() {
  CoppMib mib{};
  FarmP farm{};
  FopP fop{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  farmPInit(farm);
  fopPInit(fop, mib);
  (void)farmPOnFrame(farm, true, false, 0);
  (void)fopPNeedFrame(fop, false, true);
  Plcw16 w{};
  w.report_value = 1;
  fopPOnPlcw(fop, w, true);
  fopPTick(fop, 1);
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
}

}  // namespace

int run_copp_tests() {
  test_seq_compare();
  test_farm_p_table();
  test_fop_p_send_and_ack();
  test_fop_p_invalid_plcw_and_timer();
  test_canned_pltu_to_plcw();
  test_uslp_host_loop();
  test_heap();
  return g_fails;
}
