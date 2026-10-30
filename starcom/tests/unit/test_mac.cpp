// IVP increment 13 — Prox-1 211.0 §6 MAC / DUPLEX + SET V(R) persistent.

#include "heap_trap.hpp"
#include "starcom/ccsds/mac.hpp"

#include <array>
#include <cstdio>
#include <span>

using starcom::ccsds::CoppEndpoint;
using starcom::ccsds::CoppMib;
using starcom::ccsds::coppInit;
using starcom::ccsds::decodeSetControl;
using starcom::ccsds::decodeSetPhy;
using starcom::ccsds::decodeSetPlExt;
using starcom::ccsds::decodeSetVr;
using starcom::ccsds::encodeSetControl;
using starcom::ccsds::encodeSetPhy;
using starcom::ccsds::encodeSetPlExt;
using starcom::ccsds::encodeSetVr;
using starcom::ccsds::kPhyEncodingBypass;
using starcom::ccsds::kSetControlDirectiveType;
using starcom::ccsds::kSetPlExtDirectiveType;
using starcom::ccsds::kSetRxDirectiveType;
using starcom::ccsds::kSetTxDirectiveType;
using starcom::ccsds::farmPReport;
using starcom::ccsds::FopPState;
using starcom::ccsds::kSetVrDirectiveType;
using starcom::ccsds::macDriveSetVr;
using starcom::ccsds::macFifoSource;
using starcom::ccsds::macCopySpdu;
using starcom::ccsds::macInit;
using starcom::ccsds::macLoadPendingCommValue;
using starcom::ccsds::macLocalCommChange;
using starcom::ccsds::macLocalNoMoreData;
using starcom::ccsds::macOnFifoEmpty;
using starcom::ccsds::macOnRnmd;
using starcom::ccsds::macOnHailReceived;
using starcom::ccsds::macOnNoFramesPending;
using starcom::ccsds::macOnPlcw;
using starcom::ccsds::macOnRemoteCommChange;
using starcom::ccsds::macOnSetVrDirective;
using starcom::ccsds::macOnToken;
using starcom::ccsds::macOnValidFrame;
using starcom::ccsds::macPhy;
using starcom::ccsds::macPollNotify;
using starcom::ccsds::macSetCarrierAcquired;
using starcom::ccsds::macSetDuplex;
using starcom::ccsds::macSetInitializeMode;
using starcom::ccsds::macSetMode;
using starcom::ccsds::macTick;
using starcom::ccsds::MacCommValue;
using starcom::ccsds::MacControlParams;
using starcom::ccsds::MacDuplex;
using starcom::ccsds::MacPhyParams;
using starcom::ccsds::MacPlExt;
using starcom::ccsds::spduDirectiveType;
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
  CHECK(!s.mac_frame_pending);
  CHECK(s.mac_queue_len == 0);
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

void test_annex_b_codecs() {
  MacPhyParams tx{};
  tx.encoding = kPhyEncodingBypass;
  tx.mode = 1;
  tx.modulation = 1;
  std::array<std::byte, 2> buf{};
  auto n = encodeSetPhy(buf, tx, true);
  CHECK(n.has_value());
  CHECK(*n == 2);
  CHECK(spduDirectiveType(buf) == kSetTxDirectiveType);
  bool is_tx = false;
  auto back = decodeSetPhy(buf, &is_tx);
  CHECK(back.has_value());
  CHECK(is_tx);
  CHECK(back->encoding == kPhyEncodingBypass);
  CHECK(back->mode == 1);

  n = encodeSetPhy(buf, tx, false);
  CHECK(n.has_value());
  CHECK(spduDirectiveType(buf) == kSetRxDirectiveType);
  back = decodeSetPhy(buf, &is_tx);
  CHECK(back.has_value());
  CHECK(!is_tx);

  MacControlParams ctl{};
  ctl.pass = true;
  n = encodeSetControl(buf, ctl);
  CHECK(n.has_value());
  CHECK(spduDirectiveType(buf) == kSetControlDirectiveType);
  auto cback = decodeSetControl(buf);
  CHECK(cback.has_value());
  CHECK(cback->pass);
  CHECK(!cback->rnmd);

  MacPlExt pl{};
  pl.rate_table = true;
  n = encodeSetPlExt(buf, pl);
  CHECK(n.has_value());
  CHECK(spduDirectiveType(buf) == kSetPlExtDirectiveType);
  auto pback = decodeSetPlExt(buf);
  CHECK(pback.has_value());
  CHECK(pback->rate_table);
}

void test_half_hail_octets() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  CHECK(s.state == MacState::s13);
  CHECK(s.mac_frame_pending);
  CHECK(s.mac_queue_len == 4);
  CHECK(spduDirectiveType(std::span<const std::byte>(s.mac_queue.data(), 2)) ==
        kSetTxDirectiveType);
  CHECK(spduDirectiveType(std::span<const std::byte>(s.mac_queue.data() + 2, 2)) ==
        kSetRxDirectiveType);
  std::array<std::byte, 16> copy{};
  auto n = macCopySpdu(s, copy);
  CHECK(n.has_value());
  CHECK(*n == 4);
}

void test_half_e48_e49_carrier() {
  // 211.0 table 6-12: E49 token swap; E48 missed-token backup; E44
  // sender-overran only while CARRIER_ACQUIRED (6.5.2) is still true.
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  CHECK(s.state == MacState::s36);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s60);
  CHECK(s.mac_queue_len == 0);
  macOnToken(s, 7);
  CHECK(s.state == MacState::s51);  // E49

  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s60);
  macSetCarrierAcquired(s, false, 6);
  macTick(s, 11);  // receive_duration = 5
  CHECK(s.state == MacState::s51);  // E48

  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  macOnValidFrame(s, 6);
  macSetCarrierAcquired(s, true, 6);
  macTick(s, 11);
  CHECK(s.state == MacState::s60);  // E44
  CHECK(macPollNotify(s) == MacNotify::sender_overran);
}

void test_half_e42_tail_e50_counts_miss() {
  // 211.0 table 6-10 E42 is the tail only. Annex C counts S62→E50.
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  macTick(s, 3);
  macTick(s, 5);
  macTick(s, 10);
  s.need_plcw = false;
  macOnNoFramesPending(s, 15);
  CHECK(s.state == MacState::s56);
  CHECK(s.token_fail_n == 0);
  macOnFifoEmpty(s, 15);
  CHECK(s.state == MacState::s58);
  CHECK(s.token_fail_n == 0);
  macTick(s, 17);
  CHECK(s.state == MacState::s62);
  macTick(s, 22);
  CHECK(s.state == MacState::s51);
  CHECK(s.token_fail_n == 1);
}

void test_half_e83_rehail() {
  MacMib mb = test_mib();
  mb.maximum_failed_token_passes = 2;
  mb.drop_carrier_duration = 2;
  MacSession s{};
  macInit(s, mb, MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s60);
  s.role = starcom::ccsds::MacRole::caller;
  s.token_fail_n = 2;
  macSetCarrierAcquired(s, false, 6);
  macTick(s, 11);  // E48 S51
  CHECK(s.state == MacState::s51);
  macTick(s, 13);
  macTick(s, 15);
  CHECK(s.state == MacState::s50);
  macTick(s, 20);  // E83 not E38
  CHECK(s.state == MacState::s80);
  macTick(s, 22);  // E84
  CHECK(s.state == MacState::s11);
  CHECK(s.mode == MacMode::connecting_t);
}

void test_half_e83_responder_listens() {
  MacMib mb = test_mib();
  mb.maximum_failed_token_passes = 2;
  MacSession s{};
  macInit(s, mb, MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  CHECK(s.state == MacState::s51);
  CHECK(s.role == starcom::ccsds::MacRole::responder);
  macTick(s, 3);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  s.token_fail_n = 2;
  macTick(s, 10);
  CHECK(s.state == MacState::s2);
  CHECK(s.mode == MacMode::connecting_l);
  CHECK(!s.transmit_on);
}

void test_half_rehail_while_active() {
  // Station reboot hails again while the responder is still S50.
  // 211.0 E85/E82 → S2, then E30 → S51 (not ignore / E69).
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  CHECK(s.state == MacState::s51);
  macTick(s, 3);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  CHECK(s.role == starcom::ccsds::MacRole::responder);
  CHECK(s.mode == MacMode::active);
  (void)macPollNotify(s);
  macOnHailReceived(s, 6);
  CHECK(s.state == MacState::s51);
  CHECK(s.mode == MacMode::active);
  CHECK(macPollNotify(s) == MacNotify::hail_ok);
}

void test_half_token_octets() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  CHECK(s.state == MacState::s51);
  macTick(s, 3);
  CHECK(s.state == MacState::s52);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  macTick(s, 10);  // E38 sets PERSISTENCE; NEED_PLCW still blocks E39
  CHECK(s.persistence);
  CHECK(s.need_plcw);
  CHECK(s.state == MacState::s50);
  CHECK(macFifoSource(s) == MacFifoSource::plcw);
  s.need_plcw = false;
  s.sdu_pending = true;
  CHECK(macFifoSource(s) == MacFifoSource::idle);
  macOnNoFramesPending(s, 15);  // E39
  CHECK(s.state == MacState::s56);
  CHECK(s.mac_queue_len == 2);
  CHECK(spduDirectiveType(std::span<const std::byte>(s.mac_queue.data(), 2)) ==
        kSetControlDirectiveType);
  auto ctl = decodeSetControl(std::span<const std::byte>(s.mac_queue.data(), 2));
  CHECK(ctl.has_value());
  CHECK(ctl->pass);
}

void test_half_comm_change_and_revert() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  macTick(s, 3);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  cv.tx.frequency = 1;
  cv.rx.frequency = 1;
  macLoadPendingCommValue(s, cv);
  macLocalCommChange(s, 5);
  CHECK(s.y == 2);
  CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);
  s.need_plcw = false;
  macOnNoFramesPending(s, 6);
  CHECK(s.state == MacState::s56);
  CHECK(s.mac_queue_len == 4);
  bool is_tx = false;
  auto phy = decodeSetPhy(std::span<const std::byte>(s.mac_queue.data(), 2), &is_tx);
  CHECK(phy.has_value());
  CHECK(is_tx);
  CHECK(phy->frequency == 1);
  macOnNoFramesPending(s, 7);
  CHECK(s.state == MacState::s58);
  macTick(s, 9);
  CHECK(s.state == MacState::s62);
  CHECK(s.y == 3);
  macTick(s, 14);
  CHECK(s.state == MacState::s51);
  CHECK(macPollNotify(s) == MacNotify::comm_change_revert);
  CHECK(!s.pending_cv_valid);
}

void test_half_comm_change_ok() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  macTick(s, 3);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  macLocalCommChange(s, 5);
  CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);
  s.need_plcw = false;
  macOnNoFramesPending(s, 6);
  macOnNoFramesPending(s, 7);
  CHECK(s.state == MacState::s58);
  macTick(s, 9);
  CHECK(s.state == MacState::s62);
  CHECK(s.y == 3);
  macOnValidFrame(s, 10);
  CHECK(s.state == MacState::s60);
  CHECK(macPollNotify(s) == MacNotify::comm_change_ok);
  CHECK(!s.pending_cv_valid);
}

void test_half_remote_comm_change_echo() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  macTick(s, 2);
  macTick(s, 4);
  macOnFifoEmpty(s, 4);
  macTick(s, 6);
  macOnValidFrame(s, 6);
  CHECK(s.state == MacState::s60);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  macOnRemoteCommChange(s, 6);
  CHECK(s.state == MacState::s51);
  CHECK(s.y == 2);
  macTick(s, 8);
  CHECK(s.state == MacState::s52);
  macTick(s, 10);
  CHECK(s.state == MacState::s56);
  CHECK(s.mac_frame_pending);
  macOnFifoEmpty(s, 10);
  CHECK(s.state == MacState::s58);
  CHECK(!s.mac_frame_pending);
  macTick(s, 12);
  CHECK(s.state == MacState::s62);
  CHECK(s.y == 3);
  CHECK(macPhy(s).receive);
  CHECK(!macPhy(s).transmit);
}

void test_half_comm_change_fifo_empty_leaves_s56() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 1);
  macTick(s, 3);
  macTick(s, 5);
  CHECK(s.state == MacState::s50);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  macLocalCommChange(s, 5);
  s.need_plcw = false;
  macOnNoFramesPending(s, 6);
  CHECK(s.state == MacState::s56);
  macOnFifoEmpty(s, 6);
  CHECK(s.state == MacState::s58);
  CHECK(!s.mac_frame_pending);
}

// Caller hail through S60, missed receive (E48), then S52 into S50.
// Send_Duration is armed. last_now is the returned tick.
Tick half_caller_to_s50(MacSession& s) {
  macSetMode(s, MacMode::connecting_t, 0);
  Tick now = s.mib.carrier_only_duration;
  macTick(s, now);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  macOnFifoEmpty(s, now);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  macOnValidFrame(s, now);
  CHECK(s.state == MacState::s60);
  now += s.mib.receive_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s51);
  now += s.mib.carrier_only_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s52);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s50);
  CHECK(s.y == 0);
  CHECK(!s.persistence);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.send_duration);
  return now;
}

// Responder hail into S50. Send_Duration is armed.
Tick half_responder_to_s50(MacSession& s) {
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 0);
  Tick now = s.mib.carrier_only_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s52);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s50);
  CHECK(s.y == 0);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.send_duration);
  return now;
}

// S50, Y=0, NEED_PLCW clear: E39 to S56, then E42 to S58. Tail wait is armed.
// Setup sets need_plcw by hand; a real link may not reach this state.
Tick half_responder_to_s58(MacSession& s) {
  Tick now = half_responder_to_s50(s);
  s.need_plcw = false;
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s56);
  macOnFifoEmpty(s, now);
  CHECK(s.state == MacState::s58);
  CHECK(s.y == 0);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.tail_idle_duration);
  return now;
}

// S50 local COMM_CHANGE (E63, Y=2), then E65/E66 into S58. Tail wait is armed.
// Setup sets need_plcw by hand; a real link may not reach this state.
Tick half_responder_comm_to_s58(MacSession& s) {
  Tick now = half_responder_to_s50(s);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  macLocalCommChange(s, now);
  CHECK(s.state == MacState::s50);
  CHECK(s.y == 2);
  CHECK(s.persistence);
  CHECK(s.pending_cv_valid);
  s.need_plcw = false;
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s56);
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s58);
  CHECK(s.y == 2);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.tail_idle_duration);
  return now;
}

void check_half_expiry(MacSession& s, MacState state, std::uint8_t y, bool persistence,
                       bool modulation, bool need_plcw, bool transmit_on,
                       std::uint8_t token_fail_n, bool pending, bool wait_armed,
                       Tick wait_left, MacNotify note) {
  CHECK(s.state == state);
  CHECK(s.y == y);
  CHECK(s.persistence == persistence);
  CHECK(s.modulation == modulation);
  CHECK(s.need_plcw == need_plcw);
  CHECK(s.transmit_on == transmit_on);
  CHECK(s.token_fail_n == token_fail_n);
  CHECK(s.pending_cv_valid == pending);
  CHECK(s.wait_armed == wait_armed);
  CHECK(s.wait_left == wait_left);
  CHECK(macPollNotify(s) == note);
}

// Setup sets token_fail_n and y by hand; a real link may not reach this state.
void test_half_s50_e38_token_below_and_off() {
  // Limit is on. token_fail_n is one below it. Y=0: E38 sets PERSISTENCE.
  // The wait is not reloaded.
  {
    MacMib mb = test_mib();
    mb.maximum_failed_token_passes = 2;
    MacSession s{};
    macInit(s, mb, MacDuplex::half, nullptr);
    Tick now = half_caller_to_s50(s);
    s.token_fail_n = 1;
    (void)macPollNotify(s);
    now += s.mib.send_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s50, 0, true, true, true, true, 1, false, false, 0,
                      MacNotify::none);
  }
  // Project behavior: 211.0-B-6 does not define this case.
  // Same limit, one below, Y!=0: the Y==0 assignment does not run.
  {
    MacMib mb = test_mib();
    mb.maximum_failed_token_passes = 2;
    MacSession s{};
    macInit(s, mb, MacDuplex::half, nullptr);
    Tick now = half_caller_to_s50(s);
    s.token_fail_n = 1;
    s.y = 1;
    (void)macPollNotify(s);
    now += s.mib.send_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s50, 1, false, true, true, true, 1, false, false, 0,
                      MacNotify::none);
  }
  // Project behavior: 211.0-B-6 does not define this case.
  // Limit 0 disables the check. A high token_fail_n still takes E38 (Y=0).
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_caller_to_s50(s);
    CHECK(s.mib.maximum_failed_token_passes == 0);
    s.token_fail_n = 9;
    (void)macPollNotify(s);
    now += s.mib.send_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s50, 0, true, true, true, true, 9, false, false, 0,
                      MacNotify::none);
  }
  // Project behavior: 211.0-B-6 does not define this case.
  // Limit 0 and Y!=0: no token-fail and no PERSISTENCE write.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_caller_to_s50(s);
    s.token_fail_n = 9;
    s.y = 1;
    (void)macPollNotify(s);
    now += s.mib.send_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s50, 1, false, true, true, true, 9, false, false, 0,
                      MacNotify::none);
  }
}

// Pins current behavior. FAILS 211.0-B-6 Table 6-11 E65 (needs No Frames Pending AND Y=2). Fix is a later step.
void test_half_s52_e64_y1_and_y3() {
  // Y=1 at S52: E41, then E64, then E65. Ends in S56. Wait stays Send_Duration.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    macSetMode(s, MacMode::connecting_t, 0);
    Tick now = s.mib.carrier_only_duration;
    macTick(s, now);
    now += s.mib.acquisition_idle_duration;
    macTick(s, now);
    macOnFifoEmpty(s, now);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    macOnValidFrame(s, now);
    CHECK(s.state == MacState::s60);
    macLocalCommChange(s, now);  // E62: not S50, so Y=1
    CHECK(s.y == 1);
    macOnToken(s, now);
    CHECK(s.state == MacState::s51);
    CHECK(s.y == 1);
    now += s.mib.carrier_only_duration;
    macTick(s, now);
    CHECK(s.state == MacState::s52);
    CHECK(s.y == 1);
    (void)macPollNotify(s);
    now += s.mib.acquisition_idle_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s56, 2, true, true, true, true, 0, false, true,
                      s.mib.send_duration, MacNotify::comm_change_apply_rx);
    CHECK(s.mac_frame_pending);
  }
  // Y=3 at S52: same E64 then E65. pending_cv_valid stays set.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_comm_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    CHECK(s.state == MacState::s62);
    CHECK(s.y == 3);
    macSetCarrierAcquired(s, true, now);  // E47 S61, Y stays 3
    CHECK(s.state == MacState::s61);
    macOnToken(s, now);
    CHECK(s.state == MacState::s51);
    CHECK(s.y == 3);
    now += s.mib.carrier_only_duration;
    macTick(s, now);
    CHECK(s.state == MacState::s52);
    CHECK(s.y == 3);
    (void)macPollNotify(s);
    now += s.mib.acquisition_idle_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s56, 2, true, true, false, true, 0, true, true,
                      s.mib.send_duration, MacNotify::comm_change_apply_rx);
    CHECK(s.mac_frame_pending);
  }
}

void test_half_s55_e57_enter_s1() {
  // E51 then remote RNMD (X=4), S50, E55, E56, then the S55 wait (E57).
  MacMib mb = test_mib();
  mb.hail_lifetime = 1000;
  mb.plcw_repeat_interval = 1000;
  mb.carrier_loss_timer_duration = 1000;
  MacSession s{};
  macInit(s, mb, MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_t, 0);
  Tick now = s.mib.carrier_only_duration;
  macTick(s, now);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  macOnFifoEmpty(s, now);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  macOnValidFrame(s, now);
  CHECK(s.state == MacState::s60);
  macSetCarrierAcquired(s, true, now);
  macSetCarrierAcquired(s, false, now);
  CHECK(s.carrier_loss_left == 1000);
  macLocalNoMoreData(s, now);
  CHECK(s.x == 1);
  macOnRnmd(s, now);
  CHECK(s.state == MacState::s51);
  CHECK(s.x == 4);
  now += s.mib.carrier_only_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s52);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s50);
  CHECK(s.x == 4);
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s54);
  CHECK(s.x == 5);
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s55);
  CHECK(s.x == 0);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.tail_idle_duration);
  CHECK(s.hail_life_left != 0);
  CHECK(s.plcw_left != 0);
  CHECK(s.carrier_loss_left != 0);
  CHECK(s.mac_frame_pending);
  // Sentinels. enterS1 clears these. It does not clear pending_cv_valid.
  // Setup sets y, x, z, persistence, modulation, need_plcw, need_status_report, token_fail_n, and resync timers by hand; a real link may not reach this state.
  s.y = 4;
  s.x = 7;
  s.z = 9;
  s.persistence = true;
  s.modulation = true;
  s.need_plcw = false;
  s.need_status_report = false;
  s.token_fail_n = 4;
  s.resync_life_left = 1000;
  s.resync_wait_left = 1000;
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  (void)macPollNotify(s);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s1);
  CHECK(s.mode == MacMode::inactive);
  CHECK(s.duplex == MacDuplex::half);
  CHECK(s.y == 0);
  CHECK(s.x == 0);
  CHECK(s.z == 0);
  CHECK(s.ss == 0);
  CHECK(!s.persistence);
  CHECK(!s.modulation);
  CHECK(s.need_plcw);
  CHECK(s.need_status_report);
  CHECK(!s.transmit_on);
  CHECK(s.token_fail_n == 0);
  CHECK(s.pending_cv_valid);
  CHECK(!s.wait_armed);
  CHECK(s.wait_left == 0);
  CHECK(s.carrier_loss_left == 0);
  CHECK(s.plcw_left == 0);
  CHECK(s.hail_life_left == 0);
  CHECK(s.resync_life_left == 0);
  CHECK(s.resync_wait_left == 0);
  CHECK(!s.carrier_acquired);
  CHECK(s.last_now == now);
  CHECK(!s.mac_frame_pending);
  CHECK(s.mac_queue_len == 0);
  CHECK(s.fifo_empty);
  CHECK(s.role == starcom::ccsds::MacRole::caller);
  CHECK(macPollNotify(s) == MacNotify::end_session);
}

void test_half_s61_e45_and_e50() {
  // E45: carrier still held when the S61 receive wait ends.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    CHECK(s.state == MacState::s62);
    macSetCarrierAcquired(s, true, now);
    CHECK(s.state == MacState::s61);
    CHECK(s.carrier_acquired);
    CHECK(s.wait_left == s.mib.receive_duration);
    (void)macPollNotify(s);
    now += s.mib.receive_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s61, 0, false, false, false, false, 0, false, true,
                      s.mib.receive_duration, MacNotify::no_data_this_contact);
  }
  // E50 from S61, Y!=3.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    macSetCarrierAcquired(s, true, now);
    CHECK(s.state == MacState::s61);
    CHECK(s.y == 0);
    macSetCarrierAcquired(s, false, now);
    (void)macPollNotify(s);
    now += s.mib.receive_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s51, 0, false, false, false, true, 1, false, true,
                      s.mib.carrier_only_duration, MacNotify::no_data_this_contact);
  }
  // Project behavior: 211.0-B-6 does not define this case.
  // E50 from S61, Y=3: revert, clear the pending COMM value.
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_comm_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    CHECK(s.state == MacState::s62);
    CHECK(s.y == 3);
    macSetCarrierAcquired(s, true, now);
    CHECK(s.state == MacState::s61);
    CHECK(s.y == 3);
    macSetCarrierAcquired(s, false, now);
    (void)macPollNotify(s);
    now += s.mib.receive_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s51, 0, true, true, false, true, 1, false, true,
                      s.mib.carrier_only_duration, MacNotify::comm_change_revert);
  }
}

// Pins current behavior. FAILS 211.0-B-6 Table 6-10 E47 (Carrier Lock true in S62 should go to S61). Fix is a later step.
void test_half_s62_carrier_no_e50() {
  // Carrier is already true in S58, so S62 does not leave on E47.
  // The receive wait then ends with the carrier still held: E50 does not run.
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  Tick now = half_responder_to_s58(s);
  macSetCarrierAcquired(s, true, now);
  CHECK(s.state == MacState::s58);
  (void)macPollNotify(s);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s62);
  CHECK(s.carrier_acquired);
  CHECK(s.y == 0);
  (void)macPollNotify(s);
  now += s.mib.receive_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s62, 0, false, false, false, false, 0, false, false, 0,
                    MacNotify::none);
  CHECK(s.carrier_acquired);
}

// Project behavior: 211.0-B-6 does not define this case.
// Setup sets need_plcw by hand; a real link may not reach this state.
void test_half_wait_expired_other_state() {
  // S56 is not a case in macWaitExpiredHalf. The armed send wait still ends.
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  Tick now = half_responder_to_s50(s);
  s.need_plcw = false;
  macOnNoFramesPending(s, now);
  CHECK(s.state == MacState::s56);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.send_duration);
  CHECK(s.mac_frame_pending);
  (void)macPollNotify(s);
  now += s.mib.send_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s56, 0, false, true, false, true, 0, false, false, 0,
                    MacNotify::none);
  CHECK(s.mac_frame_pending);
}

// Caller hail stops in S60. receive_duration is armed.
Tick half_caller_to_s60(MacSession& s) {
  macSetMode(s, MacMode::connecting_t, 0);
  Tick now = s.mib.carrier_only_duration;
  macTick(s, now);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  macOnFifoEmpty(s, now);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  macOnValidFrame(s, now);
  CHECK(s.state == MacState::s60);
  CHECK(s.wait_armed);
  CHECK(s.wait_left == s.mib.receive_duration);
  return now;
}

// Setup sets token_fail_n by hand; a real link may not reach this state.
void test_half_s50_token_fail_caller_full() {
  MacMib mb = test_mib();
  mb.maximum_failed_token_passes = 2;
  MacSession s{};
  macInit(s, mb, MacDuplex::half, nullptr);
  Tick now = half_caller_to_s50(s);
  s.token_fail_n = 2;
  (void)macPollNotify(s);
  now += s.mib.send_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s80, 0, false, true, true, false, 2, false, true,
                    s.mib.drop_carrier_duration, MacNotify::none);
}

// Project behavior: 211.0-B-6 does not define this case.
// Setup sets token_fail_n by hand; a real link may not reach this state.
void test_half_s50_token_fail_responder_full() {
  MacMib mb = test_mib();
  mb.maximum_failed_token_passes = 2;
  MacSession s{};
  macInit(s, mb, MacDuplex::half, nullptr);
  Tick now = half_responder_to_s50(s);
  s.token_fail_n = 2;
  (void)macPollNotify(s);
  now += s.mib.send_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s2, 0, false, true, true, false, 0, false, false, 0,
                    MacNotify::none);
}

void test_half_s51_e40_and_s52_y0_full() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  macOnHailReceived(s, 0);
  CHECK(s.state == MacState::s51);
  (void)macPollNotify(s);
  Tick now = s.mib.carrier_only_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s52, 0, false, true, true, true, 0, false, true,
                    s.mib.acquisition_idle_duration, MacNotify::none);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s50, 0, false, true, true, true, 0, false, true,
                    s.mib.send_duration, MacNotify::none);
}

// Pins current behavior. FAILS 211.0-B-6 Table 6-11 E65 (needs No Frames Pending AND Y=2). Fix is a later step.
void test_half_s52_y2_full() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  Tick now = half_caller_to_s60(s);
  MacCommValue cv{};
  cv.tx.encoding = kPhyEncodingBypass;
  cv.rx.encoding = kPhyEncodingBypass;
  macLoadPendingCommValue(s, cv);
  macOnRemoteCommChange(s, now);
  CHECK(s.state == MacState::s51);
  CHECK(s.y == 2);
  now += s.mib.carrier_only_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s52);
  CHECK(s.y == 2);
  (void)macPollNotify(s);
  now += s.mib.acquisition_idle_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s56, 2, true, true, true, true, 0, true, true,
                    s.mib.send_duration, MacNotify::none);
  CHECK(s.mac_frame_pending);
}

void test_half_s58_full() {
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_comm_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s62, 3, true, true, false, false, 0, true, true,
                      s.mib.receive_duration, MacNotify::none);
  }
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_responder_to_s58(s);
    (void)macPollNotify(s);
    now += s.mib.tail_idle_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s62, 0, false, false, false, false, 0, false, true,
                      s.mib.receive_duration, MacNotify::none);
  }
}

void test_half_s60_full() {
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_caller_to_s60(s);
    macSetCarrierAcquired(s, true, now);
    (void)macPollNotify(s);
    now += s.mib.receive_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s60, 0, false, false, true, false, 0, false, true,
                      s.mib.receive_duration, MacNotify::sender_overran);
  }
  {
    MacSession s{};
    macInit(s, test_mib(), MacDuplex::half, nullptr);
    Tick now = half_caller_to_s60(s);
    CHECK(!s.carrier_acquired);
    (void)macPollNotify(s);
    now += s.mib.receive_duration;
    macTick(s, now);
    check_half_expiry(s, MacState::s51, 0, false, false, true, true, 0, false, true,
                      s.mib.carrier_only_duration, MacNotify::none);
  }
}

// Project behavior: 211.0-B-6 does not define this case.
void test_half_s62_e50_y3_full() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  Tick now = half_responder_comm_to_s58(s);
  (void)macPollNotify(s);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s62);
  CHECK(s.y == 3);
  (void)macPollNotify(s);
  now += s.mib.receive_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s51, 0, true, true, false, true, 1, false, true,
                    s.mib.carrier_only_duration, MacNotify::comm_change_revert);
}

void test_half_s62_e50_y0_full() {
  MacSession s{};
  macInit(s, test_mib(), MacDuplex::half, nullptr);
  Tick now = half_responder_to_s58(s);
  (void)macPollNotify(s);
  now += s.mib.tail_idle_duration;
  macTick(s, now);
  CHECK(s.state == MacState::s62);
  (void)macPollNotify(s);
  now += s.mib.receive_duration;
  macTick(s, now);
  check_half_expiry(s, MacState::s51, 0, false, false, false, true, 1, false, true,
                    s.mib.carrier_only_duration, MacNotify::no_carrier_this_contact);
}

void test_plcw_repeat_arms() {
  MacSession s{};
  MacMib m = test_mib();
  m.plcw_repeat_interval = 4;
  macInit(s, m, MacDuplex::half, nullptr);
  macSetMode(s, MacMode::connecting_l, 0);
  s.need_plcw = false;
  macTick(s, 1);
  CHECK(!s.need_plcw);
  CHECK(s.plcw_left == 4);
  macTick(s, 5);
  CHECK(s.need_plcw);
  CHECK(s.plcw_left == 4);
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
  test_annex_b_codecs();
  test_half_hail_octets();
  test_half_e48_e49_carrier();
  test_half_e42_tail_e50_counts_miss();
  test_half_e83_rehail();
  test_half_e83_responder_listens();
  test_half_rehail_while_active();
  test_half_token_octets();
  test_half_comm_change_and_revert();
  test_half_comm_change_ok();
  test_half_remote_comm_change_echo();
  test_half_comm_change_fifo_empty_leaves_s56();
  test_half_s50_e38_token_below_and_off();
  test_half_s52_e64_y1_and_y3();
  test_half_s55_e57_enter_s1();
  test_half_s61_e45_and_e50();
  test_half_s62_carrier_no_e50();
  test_half_wait_expired_other_state();
  test_half_s50_token_fail_caller_full();
  test_half_s50_token_fail_responder_full();
  test_half_s51_e40_and_s52_y0_full();
  test_half_s52_y2_full();
  test_half_s58_full();
  test_half_s60_full();
  test_half_s62_e50_y3_full();
  test_half_s62_e50_y0_full();
  test_plcw_repeat_arms();
  test_heap();
  return g_fails;
}
