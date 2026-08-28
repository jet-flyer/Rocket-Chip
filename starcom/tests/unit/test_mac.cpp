// IVP increment 13 — Prox-1 211.0 §6 MAC / DUPLEX + SET V(R) persistent.

#include "heap_trap.hpp"
#include "starcom/ccsds/mac.hpp"

#include <array>
#include <cstdio>
#include <span>

using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppMib;
using starcom::ccsds::coppInit;
using starcom::ccsds::decodeSetVr;
using starcom::ccsds::encodeSetVr;
using starcom::ccsds::farmPReport;
using starcom::ccsds::FopPState;
using starcom::ccsds::kSetVrDirectiveType;
using starcom::ccsds::macDriveSetVr;
using starcom::ccsds::macFifoSource;
using starcom::ccsds::macInit;
using starcom::ccsds::macOnFifoEmpty;
using starcom::ccsds::macOnHailReceived;
using starcom::ccsds::macOnPlcw;
using starcom::ccsds::macOnSetVrDirective;
using starcom::ccsds::macOnValidFrame;
using starcom::ccsds::macPhy;
using starcom::ccsds::macPollNotify;
using starcom::ccsds::macSetCarrierAcquired;
using starcom::ccsds::macSetDuplex;
using starcom::ccsds::macSetInitializeMode;
using starcom::ccsds::macSetMode;
using starcom::ccsds::macTick;
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
  auto n = encodeSetVr(buf, 0x2A, Pcid{0});
  CHECK(n.has_value());
  CHECK(*n == 2);
  CHECK(static_cast<unsigned>(buf[0]) == 0x2A);
  CHECK(static_cast<unsigned>(buf[1]) == kSetVrDirectiveType);
  auto fsn = decodeSetVr(buf, nullptr);
  CHECK(fsn.has_value());
  CHECK(*fsn == 0x2A);
  std::array<std::byte, 1> tiny{};
  CHECK(!encodeSetVr(tiny, 0, Pcid{0}).has_value());
}

void test_s1_initialize() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  coppInit(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  macInit(s, test_mib(), MacDuplex::full, &e);
  CHECK(s.state == MacState::s1);
  CHECK(s.mode == MacMode::inactive);
  CHECK(!macPhy(s).receive);
  CHECK(!macPhy(s).transmit);
  CHECK(s.ss == 0);
  CHECK(!s.persistence);
  macSetInitializeMode(s, 10);
  CHECK(s.state == MacState::s1);
  CHECK(e.farm.v_r == 0);
  CHECK(e.fop.state == FopPState::s1_active);
}

void test_full_hail_tables() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  CHECK(s.state == MacState::s2);
  CHECK(macPhy(s).receive);
  CHECK(!macPhy(s).transmit);

  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s31);
  CHECK(s.ss == 1);
  CHECK(s.persistence);
  CHECK(macPhy(s).transmit);
  CHECK(!macPhy(s).modulation);
  CHECK(macFifoSource(s) == MacFifoSource::carrier_only);

  macTick(s, 2);
  CHECK(s.state == MacState::s32);
  CHECK(s.modulation);
  macTick(s, 4);
  CHECK(s.state == MacState::s33);
  CHECK(s.ss == 3);
  CHECK(s.mac_frame_pending);
  macOnFifoEmpty(s, 4);
  CHECK(s.state == MacState::s34);
  macTick(s, 6);
  CHECK(s.state == MacState::s35);
  CHECK(!s.transmit_on);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s41);
  CHECK(!s.persistence);
  CHECK(macPollNotify(s) == MacNotify::hail_ok);
  macTick(s, 8);
  CHECK(s.state == MacState::s42);
  macTick(s, 10);
  CHECK(s.state == MacState::s40);
  CHECK(s.ss == 0);
}

void test_full_responder_hail() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  CHECK(s.state == MacState::s41);
  CHECK(s.need_plcw);
}

void test_half_and_simplex() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  CHECK(s.state == MacState::s11);
  CHECK(!macPhy(s).receive);
  CHECK(macPhy(s).transmit);
  macTick(s, 2);
  CHECK(s.state == MacState::s12);
  macTick(s, 4);
  CHECK(s.state == MacState::s13);
  macOnFifoEmpty(s, 4);
  CHECK(s.state == MacState::s14);
  macTick(s, 6);
  CHECK(s.state == MacState::s36);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s60);
  CHECK(macPhy(s).receive);
  CHECK(!macPhy(s).transmit);

  macInit(s, test_mib(), MacDuplex::simplex_transmit, nullptr);
  macSetMode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s71);
  CHECK(s.duplex == MacDuplex::simplex_transmit);
  macSetMode(s, MacMode::inactive, 1);
  CHECK(s.state == MacState::s1);

  macInit(s, test_mib(), MacDuplex::simplex_receive, nullptr);
  macSetMode(s, MacMode::active, 0);
  CHECK(s.state == MacState::s72);
  CHECK(macPhy(s).receive);
  CHECK(!macPhy(s).transmit);
}

void test_set_duplex_s1_only() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetDuplex(s, MacDuplex::half);
  CHECK(s.duplex == MacDuplex::half);
  macSetMode(s, MacMode::connecting_t, 0);
  macSetDuplex(s, MacDuplex::full);
  CHECK(s.duplex == MacDuplex::half);
}

void test_set_vr_persistent() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  cm.resync_local = true;
  coppInit(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  macInit(s, test_mib(), MacDuplex::full, &e);
  e.fop.nn_r = 7;
  e.fop.v_s = 7;  // note 5c: N(R) > V(S) is invalid
  e.fop.vv_s = 7;
  e.fop.resync = true;
  e.fop.state = FopPState::s2_resync;
  macDriveSetVr(s, 0);
  CHECK(s.persistence);
  CHECK(s.mac_frame_pending);
  CHECK(s.mac_queue_len == 2);
  CHECK(static_cast<unsigned>(s.mac_queue[0]) == 7);
  CHECK(static_cast<unsigned>(s.mac_queue[1]) == kSetVrDirectiveType);

  Plcw16 w = farmPReport(e.farm, Pcid{0});
  w.report_value = 7;
  w.retransmit = false;
  macOnPlcw(s, w, true, 1);
  CHECK(!e.fop.resync);
  CHECK(e.fop.state == FopPState::s1_active);
  CHECK(macPollNotify(s) == MacNotify::resync_ok);
  CHECK(!s.persistence);
}

void test_inbound_set_vr() {
  MacSession s{};
  CoppEndpoint e{};
  CoppMib cm{};
  coppInit(e, cm, Pcid{0}, Scid{1}, Scid{2}, PortId{0});
  macInit(s, test_mib(), MacDuplex::full, &e);
  macOnSetVrDirective(s, 42);
  CHECK(e.farm.v_r == 42);
  CHECK(!e.farm.r_s);
  CHECK(e.farm.need_plcw);
}

void test_carrier_loss_caller() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  macOnValidFrame(s, 6);
  macTick(s, 8);
  macTick(s, 10);
  CHECK(s.state == MacState::s40);
  macSetCarrierAcquired(s, true, 10);
  macSetCarrierAcquired(s, false, 11);
  macTick(s, 15);
  CHECK(s.state == MacState::s80);
}

void test_heap() {
  MacSession s{};
  starcom::test::heapTrapReset();
  starcom::test::heapTrapArm();
  macInit(s, test_mib(), MacDuplex::full, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  std::array<std::byte, 2> buf{};
  (void)encodeSetVr(buf, 1, Pcid{0});
  starcom::test::heapTrapDisarm();
  CHECK(starcom::test::heapTrapCount() == 0);
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
