// IVP increment 13 — Prox-1 211.0 §6 MAC / DUPLEX + SET V(R) persistent.

#include "heap_trap.hpp"
#include "starcom/ccsds/mac.hpp"

#include <array>
#include <cstdio>
#include <span>

using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppMib;
using starcom::ccsds::copp_init;
using starcom::ccsds::decode_set_vr;
using starcom::ccsds::encode_set_vr;
using starcom::ccsds::farm_p_report;
using starcom::ccsds::FopPState;
using starcom::ccsds::kSetVrDirectiveType;
using starcom::ccsds::mac_drive_set_vr;
using starcom::ccsds::mac_fifo_source;
using starcom::ccsds::mac_init;
using starcom::ccsds::mac_on_fifo_empty;
using starcom::ccsds::mac_on_hail_received;
using starcom::ccsds::mac_on_plcw;
using starcom::ccsds::mac_on_set_vr_directive;
using starcom::ccsds::mac_on_valid_frame;
using starcom::ccsds::mac_phy;
using starcom::ccsds::mac_poll_notify;
using starcom::ccsds::mac_set_carrier_acquired;
using starcom::ccsds::mac_set_duplex;
using starcom::ccsds::mac_set_initialize_mode;
using starcom::ccsds::mac_set_mode;
using starcom::ccsds::mac_tick;
using starcom::ccsds::MacDuplex;
using starcom::ccsds::MacFifoSource;
using starcom::ccsds::MacMib;
using starcom::ccsds::MacMode;
using starcom::ccsds::MacNotify;
using starcom::ccsds::MacSession;
using starcom::ccsds::MacState;
using starcom::ccsds::Pcid;
using starcom::ccsds::Plcw16;
using starcom::ccsds::PortId;
using starcom::ccsds::Scid;
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

MacMib test_mib() {
  MacMib m{};
  m.carrier_only_duration = 2;
  m.acquisition_idle_duration = 2;
  m.tail_idle_duration = 2;
  m.hail_wait_duration = 3;
  m.drop_carrier_duration = 2;
  m.carrier_loss_timer_duration = 4;
  m.persistence_wait_time = 2;
  m.send_duration = 5;
  m.receive_duration = 5;
  m.resync_waiting_period = 2;
  m.resync_lifetime = 20;
  m.local_scid = Scid{1};
  m.local_pcid = Pcid{0};
  return m;
}

void test_set_vr_codec() {
  std::array<std::byte, 2> buf{};
  auto n = encode_set_vr(buf, 0x2A, Pcid{0});
  CHECK(n.has_value());
  CHECK(*n == 2);
  CHECK(static_cast<unsigned>(buf[0]) == 0x2A);
  CHECK(static_cast<unsigned>(buf[1]) == kSetVrDirectiveType);
  auto fsn = decode_set_vr(buf, nullptr);
  CHECK(fsn.has_value());
  CHECK(*fsn == 0x2A);
  std::array<std::byte, 1> tiny{};
  CHECK(!encode_set_vr(tiny, 0, Pcid{0}).has_value());
}

void test_s1_initialize() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  copp_init(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  mac_init(s, test_mib(), MacDuplex::full, &e);
  CHECK(s.state == MacState::s1);
  CHECK(s.mode == MacMode::inactive);
  CHECK(!mac_phy(s).receive);
  CHECK(!mac_phy(s).transmit);
  CHECK(s.ss == 0);
  CHECK(!s.persistence);
  mac_set_initialize_mode(s, 10);
  CHECK(s.state == MacState::s1);
  CHECK(e.farm.v_r == 0);
  CHECK(e.fop.state == FopPState::s1_active);
}

void test_full_hail_tables() {
  MacSession s{};
  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_mode(s, MacMode::connecting_l, 0);
  CHECK(s.state == MacState::s2);
  CHECK(mac_phy(s).receive);
  CHECK(!mac_phy(s).transmit);

  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s31);
  CHECK(s.ss == 1);
  CHECK(s.persistence);
  CHECK(mac_phy(s).transmit);
  CHECK(!mac_phy(s).modulation);
  CHECK(mac_fifo_source(s) == MacFifoSource::carrier_only);

  mac_tick(s, 2);
  CHECK(s.state == MacState::s32);
  CHECK(s.modulation);
  mac_tick(s, 4);
  CHECK(s.state == MacState::s33);
  CHECK(s.ss == 3);
  CHECK(s.mac_frame_pending);
  mac_on_fifo_empty(s, 4);
  CHECK(s.state == MacState::s34);
  mac_tick(s, 6);
  CHECK(s.state == MacState::s35);
  CHECK(!s.transmit_on);
  mac_on_valid_frame(s, 6);
  CHECK(s.state == MacState::s41);
  CHECK(!s.persistence);
  CHECK(mac_poll_notify(s) == MacNotify::hail_ok);
  mac_tick(s, 8);
  CHECK(s.state == MacState::s42);
  mac_tick(s, 10);
  CHECK(s.state == MacState::s40);
  CHECK(s.ss == 0);
}

void test_full_responder_hail() {
  MacSession s{};
  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_mode(s, MacMode::connecting_l, 0);
  mac_on_hail_received(s, 1);
  CHECK(s.state == MacState::s41);
  CHECK(s.need_plcw);
}

void test_half_and_simplex() {
  MacSession s{};
  mac_init(s, test_mib(), MacDuplex::half, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s11);
  CHECK(!mac_phy(s).receive);
  CHECK(mac_phy(s).transmit);
  mac_tick(s, 2);
  CHECK(s.state == MacState::s12);
  mac_tick(s, 4);
  CHECK(s.state == MacState::s13);
  mac_on_fifo_empty(s, 4);
  CHECK(s.state == MacState::s14);
  mac_tick(s, 6);
  CHECK(s.state == MacState::s36);
  mac_on_valid_frame(s, 6);
  CHECK(s.state == MacState::s60);
  CHECK(mac_phy(s).receive);
  CHECK(!mac_phy(s).transmit);

  mac_init(s, test_mib(), MacDuplex::simplex_transmit, nullptr);
  mac_set_mode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s71);
  CHECK(s.duplex == MacDuplex::simplex_transmit);
  mac_set_mode(s, MacMode::inactive, 1);
  CHECK(s.state == MacState::s1);

  mac_init(s, test_mib(), MacDuplex::simplex_receive, nullptr);
  mac_set_mode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s72);
  CHECK(mac_phy(s).receive);
  CHECK(!mac_phy(s).transmit);
}

void test_set_duplex_s1_only() {
  MacSession s{};
  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_duplex(s, MacDuplex::half);
  CHECK(s.duplex == MacDuplex::half);
  mac_set_mode(s, MacMode::connecting_t, 0);
  mac_set_duplex(s, MacDuplex::full);
  CHECK(s.duplex == MacDuplex::half);
}

void test_set_vr_persistent() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  cm.resync_local = true;
  copp_init(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  mac_init(s, test_mib(), MacDuplex::full, &e);
  e.fop.nn_r = 7;
  e.fop.v_s = 7;  // note 5c: N(R) > V(S) is invalid
  e.fop.vv_s = 7;
  e.fop.resync = true;
  e.fop.state = FopPState::s2_resync;
  mac_drive_set_vr(s, 0);
  CHECK(s.persistence);
  CHECK(s.mac_frame_pending);
  CHECK(s.mac_queue_len == 2);
  CHECK(static_cast<unsigned>(s.mac_queue[0]) == 7);
  CHECK(static_cast<unsigned>(s.mac_queue[1]) == kSetVrDirectiveType);

  Plcw16 w = farm_p_report(e.farm, Pcid{0});
  w.report_value = 7;
  w.retransmit = false;
  mac_on_plcw(s, w, true, 1);
  CHECK(!e.fop.resync);
  CHECK(e.fop.state == FopPState::s1_active);
  CHECK(mac_poll_notify(s) == MacNotify::resync_ok);
  CHECK(!s.persistence);
}

void test_inbound_set_vr() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  copp_init(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  mac_init(s, test_mib(), MacDuplex::full, &e);
  mac_on_set_vr_directive(s, 42);
  CHECK(e.farm.v_r == 42);
  CHECK(!e.farm.r_s);
  CHECK(e.farm.need_plcw);
}

void test_carrier_loss_caller() {
  MacSession s{};
  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  mac_tick(s, 2);
  mac_tick(s, 4);
  mac_on_fifo_empty(s, 4);
  mac_tick(s, 6);
  mac_on_valid_frame(s, 6);
  mac_tick(s, 8);
  mac_tick(s, 10);
  CHECK(s.state == MacState::s40);
  mac_set_carrier_acquired(s, true, 10);
  mac_set_carrier_acquired(s, false, 11);
  mac_tick(s, 15);
  CHECK(s.state == MacState::s80);
}

void test_heap() {
  MacSession s{};
  starcom::test::heap_trap_reset();
  starcom::test::heap_trap_arm();
  mac_init(s, test_mib(), MacDuplex::full, nullptr);
  mac_set_mode(s, MacMode::connecting_t, 0);
  mac_tick(s, 2);
  mac_tick(s, 4);
  mac_on_fifo_empty(s, 4);
  std::array<std::byte, 2> buf{};
  (void)encode_set_vr(buf, 1, Pcid{0});
  starcom::test::heap_trap_disarm();
  CHECK(starcom::test::heap_trap_count() == 0);
}

}  // namespace

int run_mac_tests() {
  test_set_vr_codec();
  test_s1_initialize();
  test_full_hail_tables();
  test_full_responder_hail();
  test_half_and_simplex();
  test_set_duplex_s1_only();
  test_set_vr_persistent();
  test_inbound_set_vr();
  test_carrier_loss_caller();
  test_heap();
  return g_fails;
}
