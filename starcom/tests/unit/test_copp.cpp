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
using starcom::ccsds::coppReset;
using starcom::ccsds::coppRetransmitCount;
using starcom::ccsds::coppSubmitSdu;
using starcom::ccsds::coppSyncLossCount;
using starcom::ccsds::coppTakeSdu;
using starcom::ccsds::coppTick;
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

// 211.0-B-6 3.2.4.3.2.6.3: Retransmit Flag 1 asks for the expected frame again.
// 7.3.1 RE4 delivers only N(S) = V(R). RE5 drops a gap and sets R(S). RE6 drops a copy.
// 7.2.3.3 SE2, when R(R) is true, sets VV(S) = N(R). SE1 then resends while VV(S) < V(S).
// Turn order is one frame from B, then one from A.
// The second new SEQ PLTU of the four-SDU transfer is lost.
// The next SEQ PLTU opens the gap, so SE1 resends that lost frame and the frame that opened it.
// 7.2.2 d) and 7.3.2 a): after the ack, V(S) = N(R) = V(R).
// 7.2.3.1.3: Synch_Timeout 0 never expires. The timeout here is longer than the loop,
// so SE4 is not the path that recovers the loss.
// Report Value 3 is not used. A PLCW with second octet 0x03 matches decodeSetVr,
// and that decode runs before decodePlcw, so FARM-P would take SET V(R) instead.
void test_copp_lossy_loop() {
  constexpr int kWarm = 2;
  constexpr int kLoss = 4;
  constexpr int kBound = 48;
  constexpr std::uint8_t kBase = 4;
  constexpr std::uint8_t kSeqAfter =
      static_cast<std::uint8_t>(kBase + kWarm + kLoss);

  CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 80;
  CoppEndpoint b{};
  CoppEndpoint a{};
  coppInitUslp(b, mib, UslpScid{2}, UslpScid{1}, Vcid{1}, MapId{0});
  coppInitUslp(a, mib, UslpScid{1}, UslpScid{2}, Vcid{1}, MapId{0});

  Tick now = 1;
  std::array<std::byte, 128> wire{};
  const auto pass_frame = [&](CoppEndpoint& from, CoppEndpoint& to) -> std::size_t {
    const auto n = coppBytesToSend(from, wire);
    CHECK(n.has_value());
    if (!n.has_value() || *n == 0) {
      return 0;
    }
    coppReceiveBytes(to, std::span<const std::byte>(wire.data(), *n));
    return *n;
  };
  const auto require_frame = [&](CoppEndpoint& from, CoppEndpoint& to) {
    CHECK(pass_frame(from, to) > 0);
  };

  coppTick(a, now);
  coppTick(b, now);
  require_frame(b, a);
  require_frame(a, b);
  CHECK(!a.farm.need_plcw);
  CHECK(!b.farm.need_plcw);

  // Both ends already agree at 0. Move the base to 4 before any user SDU.
  a.fop.v_s = kBase;
  a.fop.vv_s = kBase;
  a.fop.n_r = kBase;
  a.fop.nn_r = kBase;
  farmPSetVr(b.farm, kBase);
  require_frame(b, a);
  CHECK(a.fop.nn_r == kBase);
  CHECK(b.farm.v_r == kBase);
  CHECK(!b.farm.need_plcw);

  struct Held {
    std::array<std::byte, 16> bytes{};
    std::size_t n = 0;
  };
  std::array<Held, kWarm + kLoss> want{};
  int got = 0;
  std::array<Held, kWarm + kLoss> have{};

  const auto submit = [&](int index, std::uint8_t mark) {
    SpacePacketFields sp{};
    sp.seq_count = static_cast<std::uint16_t>(index);
    const std::array<std::byte, 1> user{std::byte{mark}};
    const auto pn = encodeSpacePacket(want[static_cast<std::size_t>(index)].bytes, sp,
                                       user);
    CHECK(pn.has_value());
    if (!pn.has_value()) {
      return;
    }
    want[static_cast<std::size_t>(index)].n = *pn;
    const auto sub = coppSubmitSdu(
        a,
        std::span<const std::byte>(want[static_cast<std::size_t>(index)].bytes.data(),
                                   *pn),
        false);
    CHECK(sub.has_value());
  };
  const auto drain = [&]() {
    for (int k = 0; k < kLoss; ++k) {
      std::array<std::byte, 16> sdu{};
      const auto tn = coppTakeSdu(b, sdu);
      CHECK(tn.has_value());
      if (!tn.has_value() || *tn == 0) {
        return;
      }
      CHECK(got < kWarm + kLoss);
      if (got >= kWarm + kLoss) {
        return;
      }
      std::copy_n(sdu.begin(), *tn, have[static_cast<std::size_t>(got)].bytes.begin());
      have[static_cast<std::size_t>(got)].n = *tn;
      ++got;
    }
  };

  for (int i = 0; i < kWarm; ++i) {
    submit(i, static_cast<std::uint8_t>(0x10u + i));
    bool acked = false;
    for (int step = 0; step < kBound; ++step) {
      ++now;
      coppTick(a, now);
      coppTick(b, now);
      (void)pass_frame(b, a);
      const bool plc = a.farm.need_plcw;
      const std::uint8_t vs = a.fop.v_s;
      const auto n = coppBytesToSend(a, wire);
      CHECK(n.has_value());
      if (n.has_value() && *n > 0) {
        CHECK(plc || a.fop.v_s != vs);
        coppReceiveBytes(b, std::span<const std::byte>(wire.data(), *n));
      }
      drain();
      if (got == i + 1 && a.fop.sent_n == 0 && a.fop.v_s == a.fop.nn_r) {
        acked = true;
        break;
      }
    }
    CHECK(acked);
  }
  CHECK(got == kWarm);
  CHECK(a.fop.v_s == static_cast<std::uint8_t>(kBase + kWarm));
  CHECK(b.farm.v_r == static_cast<std::uint8_t>(kBase + kWarm));

  const std::uint32_t rtx_before = coppRetransmitCount(a);
  for (int i = 0; i < kLoss; ++i) {
    submit(kWarm + i, static_cast<std::uint8_t>(0x40u + i));
  }

  int new_seen = 0;
  int resend_n = 0;
  std::uint8_t dropped_fsn = 0;
  std::array<std::uint8_t, 4> resent_fsn{};
  bool done = false;
  for (int step = 0; step < kBound; ++step) {
    ++now;
    coppTick(a, now);
    coppTick(b, now);
    (void)pass_frame(b, a);
    const bool plc = a.farm.need_plcw;
    const std::uint8_t vs = a.fop.v_s;
    const auto n = coppBytesToSend(a, wire);
    CHECK(n.has_value());
    if (n.has_value() && *n > 0 && !plc) {
      if (a.fop.v_s != vs) {
        ++new_seen;
        if (new_seen == 2) {
          dropped_fsn = a.fop.last_send_fsn;
        } else {
          coppReceiveBytes(b, std::span<const std::byte>(wire.data(), *n));
        }
      } else {
        CHECK(resend_n < 4);
        if (resend_n < 4) {
          resent_fsn[static_cast<std::size_t>(resend_n)] = a.fop.last_send_fsn;
        }
        ++resend_n;
        coppReceiveBytes(b, std::span<const std::byte>(wire.data(), *n));
      }
    } else if (n.has_value() && *n > 0) {
      coppReceiveBytes(b, std::span<const std::byte>(wire.data(), *n));
    }
    drain();
    if (got == kWarm + kLoss && new_seen == kLoss && a.fop.sent_n == 0 &&
        a.fop.v_s == a.fop.nn_r) {
      done = true;
      break;
    }
  }
  CHECK(done);

  const std::uint32_t rtx_delta = coppRetransmitCount(a) - rtx_before;
  std::fprintf(stderr,
               "t0 lossy resend_pltu=%d counter_delta=%u dropped_fsn=%u "
               "resent_fsn=%u,%u v_s=%u n_r=%u v_r=%u\n",
               resend_n, rtx_delta, dropped_fsn,
               resend_n > 0 ? resent_fsn[0] : 0u, resend_n > 1 ? resent_fsn[1] : 0u,
               a.fop.v_s, a.fop.n_r, b.farm.v_r);

  CHECK(got == kWarm + kLoss);
  for (int i = 0; i < kWarm + kLoss; ++i) {
    CHECK(have[static_cast<std::size_t>(i)].n == want[static_cast<std::size_t>(i)].n);
    CHECK(std::equal(want[static_cast<std::size_t>(i)].bytes.begin(),
                     want[static_cast<std::size_t>(i)].bytes.begin() +
                         static_cast<std::ptrdiff_t>(want[static_cast<std::size_t>(i)].n),
                     have[static_cast<std::size_t>(i)].bytes.begin()));
  }
  CHECK(resend_n >= 1);
  CHECK(resend_n == 2);
  CHECK(dropped_fsn == static_cast<std::uint8_t>(kBase + kWarm + 1));
  CHECK(resent_fsn[0] == static_cast<std::uint8_t>(kBase + kWarm + 1));
  CHECK(resent_fsn[1] == static_cast<std::uint8_t>(kBase + kWarm + 2));
  CHECK(rtx_delta == static_cast<std::uint32_t>(resend_n));
  CHECK(a.fop.v_s == kSeqAfter);
  CHECK(a.fop.n_r == kSeqAfter);
  CHECK(a.fop.nn_r == kSeqAfter);
  CHECK(b.farm.v_r == kSeqAfter);
  CHECK(a.fop.state == FopPState::s1_active);
  CHECK(coppRetransmitCount(b) == 0u);
  CHECK(coppRetransmitCount(a) == static_cast<std::uint32_t>(resend_n));
  const std::uint32_t a_rtx = coppRetransmitCount(a);
  const std::uint32_t a_sync = coppSyncLossCount(a);
  coppReset(a);
  CHECK(coppRetransmitCount(a) == a_rtx);
  CHECK(coppSyncLossCount(a) == a_sync);
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
  test_copp_lossy_loop();
  test_heap();
  return g_fails;
}
