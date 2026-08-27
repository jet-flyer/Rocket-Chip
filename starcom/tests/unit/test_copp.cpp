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

using starcom::ccsds::copp_bytes_to_send;
using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppEvent;
using starcom::ccsds::copp_init;
using starcom::ccsds::CoppMib;
using starcom::ccsds::copp_poll_event;
using starcom::ccsds::copp_receive_bytes;
using starcom::ccsds::copp_submit_sdu;
using starcom::ccsds::copp_take_sdu;
using starcom::ccsds::encode_space_packet;
using starcom::ccsds::FarmP;
using starcom::ccsds::FarmPDisposition;
using starcom::ccsds::farm_p_init;
using starcom::ccsds::farm_p_on_frame;
using starcom::ccsds::farm_p_report;
using starcom::ccsds::farm_p_set_vr;
using starcom::ccsds::FopP;
using starcom::ccsds::FopPSend;
using starcom::ccsds::FopPState;
using starcom::ccsds::fop_p_init;
using starcom::ccsds::fop_p_need_frame;
using starcom::ccsds::fop_p_on_plcw;
using starcom::ccsds::fop_p_reset;
using starcom::ccsds::fop_p_tick;
using starcom::ccsds::Pcid;
using starcom::ccsds::Plcw16;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
using starcom::ccsds::seq_lt;
using starcom::ccsds::SpacePacketFields;
using starcom::ccsds::Tick;

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
  CHECK(!seq_lt(0, 0));
  CHECK(seq_lt(0, 1));
  CHECK(seq_lt(255, 0));
  CHECK(!seq_lt(0, 255));
  CHECK(seq_lt(10, 20));
  CHECK(!seq_lt(20, 10));
}

void test_farm_p_table() {
  FarmP f{};
  farm_p_init(f);
  CHECK(f.v_r == 0);
  CHECK(!f.r_s);
  CHECK(f.need_plcw);

  CHECK(farm_p_on_frame(f, false, false, 0) == FarmPDisposition::discarded);

  CHECK(farm_p_on_frame(f, true, true, 9) == FarmPDisposition::accepted);
  CHECK(f.expedited_frame_counter == 1);

  CHECK(farm_p_on_frame(f, true, false, 0) == FarmPDisposition::accepted);
  CHECK(f.v_r == 1);
  CHECK(!f.r_s);

  CHECK(farm_p_on_frame(f, true, false, 3) == FarmPDisposition::discarded);
  CHECK(f.r_s);

  CHECK(farm_p_on_frame(f, true, false, 0) == FarmPDisposition::discarded);

  CHECK(farm_p_on_frame(f, true, false, 1) == FarmPDisposition::accepted);
  CHECK(f.v_r == 2);
  CHECK(!f.r_s);

  farm_p_set_vr(f, 10);
  CHECK(f.v_r == 10);
  CHECK(!f.r_s);

  const auto w = farm_p_report(f, Pcid{1});
  CHECK(w.report_value == 10);
  CHECK(w.pcid == Pcid{1});
  CHECK(!w.retransmit);
}

void test_fop_p_send_and_ack() {
  CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 10;
  FopP f{};
  fop_p_init(f, mib);
  CHECK(f.state == FopPState::s1_active);

  CHECK(fop_p_need_frame(f, false, true) == FopPSend::new_seq);
  CHECK(f.last_send_fsn == 0);
  CHECK(f.v_s == 1);
  CHECK(f.sent_n == 1);

  CHECK(fop_p_need_frame(f, true, false) == FopPSend::expedited);
  CHECK(f.last_send_fsn == 0);
  CHECK(f.ve_s == 1);

  // SE1 progressive retransmission: window full or no new SEQ → resend from NN(R).
  FopP g{};
  CoppMib w1{};
  w1.transmission_window = 1;
  fop_p_init(g, w1);
  CHECK(fop_p_need_frame(g, false, true) == FopPSend::new_seq);
  CHECK(fop_p_need_frame(g, false, true) == FopPSend::resend_seq);
  CHECK(g.last_send_fsn == 0);
  CHECK(g.v_s == 1);

  Plcw16 ack{};
  ack.report_value = 1;
  fop_p_on_plcw(f, ack, true);
  CHECK(f.nn_r == 1);
  CHECK(f.sent_n == 0);
}

void test_fop_p_invalid_plcw_and_timer() {
  CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 5;
  mib.resync_local = true;
  FopP f{};
  fop_p_init(f, mib);
  CHECK(fop_p_need_frame(f, false, true) == FopPSend::new_seq);

  Plcw16 bad{};
  bad.report_value = 9;  // > V(S)=1
  fop_p_on_plcw(f, bad, true);
  CHECK(f.synch_running);

  fop_p_tick(f, 0);
  CHECK(f.state == FopPState::s1_active);
  fop_p_tick(f, 5);
  CHECK(f.synch_expired_latched);
  CHECK(f.state == FopPState::s2_resync);
  CHECK(fop_p_need_frame(f, false, true) == FopPSend::none);

  Plcw16 ok{};
  ok.report_value = 0;
  fop_p_on_plcw(f, ok, true);
  CHECK(f.state == FopPState::s1_active);

  fop_p_reset(f);
  CHECK(f.v_s == 0);
  CHECK(f.state == FopPState::s1_active);
}

void test_canned_pltu_to_plcw() {
  CoppMib mib{};
  mib.transmission_window = 4;
  CoppEndpoint rx{};
  copp_init(rx, mib, Pcid{0}, Scid{2}, Scid{1}, PortId{1});
  CoppEndpoint tx{};
  copp_init(tx, mib, Pcid{0}, Scid{1}, Scid{2}, PortId{1});

  SpacePacketFields sp{};
  const std::array<std::byte, 1> user{std::byte{0xAA}};
  std::array<std::byte, 16> pkt{};
  const auto pn = encode_space_packet(pkt, sp, user);
  CHECK(pn.has_value());
  CHECK(copp_submit_sdu(tx, std::span<const std::byte>(pkt.data(), *pn), false)
            .has_value());

  std::array<std::byte, 128> wire{};
  // Both sides start with NEED_PLCW (RE0 / SE0). Drain those P-frames first.
  const auto p0 = copp_bytes_to_send(rx, wire);
  CHECK(p0.has_value());
  CHECK(*p0 > 0);
  copp_receive_bytes(tx, std::span<const std::byte>(wire.data(), *p0));
  const auto t0 = copp_bytes_to_send(tx, wire);
  CHECK(t0.has_value());
  CHECK(*t0 > 0);
  copp_receive_bytes(rx, std::span<const std::byte>(wire.data(), *t0));

  const auto p1 = copp_bytes_to_send(tx, wire);
  CHECK(p1.has_value());
  CHECK(*p1 > 0);
  copp_receive_bytes(rx, std::span<const std::byte>(wire.data(), *p1));
  CHECK(copp_poll_event(rx) == CoppEvent::farm_accepted);
  CHECK(rx.farm.v_r == 1);
  std::array<std::byte, 16> sdu{};
  const auto tn = copp_take_sdu(rx, sdu);
  CHECK(tn.has_value());
  CHECK(*tn == *pn);
  CHECK(std::equal(pkt.begin(), pkt.begin() + static_cast<std::ptrdiff_t>(*pn),
                   sdu.begin()));

  const auto p2 = copp_bytes_to_send(rx, wire);
  CHECK(p2.has_value());
  copp_receive_bytes(tx, std::span<const std::byte>(wire.data(), *p2));
  CHECK(tx.fop.sent_n == 0);
  CHECK(tx.fop.nn_r == 1);
}

void test_heap() {
  CoppMib mib{};
  FarmP farm{};
  FopP fop{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  farm_p_init(farm);
  fop_p_init(fop, mib);
  (void)farm_p_on_frame(farm, true, false, 0);
  (void)fop_p_need_frame(fop, false, true);
  Plcw16 w{};
  w.report_value = 1;
  fop_p_on_plcw(fop, w, true);
  fop_p_tick(fop, 1);
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_copp_tests() {
  test_seq_compare();
  test_farm_p_table();
  test_fop_p_send_and_ack();
  test_fop_p_invalid_plcw_and_timer();
  test_canned_pltu_to_plcw();
  test_heap();
  return g_fails;
}
