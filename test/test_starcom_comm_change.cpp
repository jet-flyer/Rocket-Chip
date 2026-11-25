// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// R-37: station-started COMM_CHANGE (211.0 table 6-11) through two byte pumps.
// The Side model copies the ao_telemetry adapter: starcom_poll_mac_radio and
// AO_Radio_apply_config_now (PHY change = pump re-init and a new hail).
// Each frame stays on air for its preset airtime (radio port query). A side
// that sends does not receive (half duplex), and does not start a new frame
// before TxDone (AO_Radio_tx_active). A frame reaches the peer at its end of
// air, only on the same catalog, and only if the peer did not send while it
// was on air. No CRC loss.

#include <gtest/gtest.h>

#include "starcom_adapt/byte_pump.h"
#include "starcom_adapt/nav_sdu.h"
#include "starcom/ccsds/mac.h"
#include "starcom/ccsds/pltu.h"
#include "starcom/ccsds/v3.h"
#include "rocketchip/radio_config_table.h"
#include "rocketchip/radio_config_airtime.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace {

namespace sc = starcom::ccsds;
namespace sa = rc::starcom_adapt;

constexpr std::uint8_t kIdx250 = 2;  // kRadioConfigTable 250/10 (hail)
constexpr std::uint8_t kIdx500 = 3;  // 500/10
constexpr std::uint8_t kIdx125 = 4;  // 125/2
constexpr sc::Tick kStep = 10;        // kMacIdleTickMs
constexpr sc::Tick kNavPeriod = 100;  // 10 Hz nav submit
constexpr sc::Tick kSessionUp = 2000; // hail and first contacts are done

// One frame on air from one end.
struct Air {
  bool busy = false;
  bool lost = false;  // the peer does not hear this frame
  sc::Tick end = 0;   // end of air (TxDone)
  std::uint8_t phy = 0;
  std::size_t n = 0;
  std::array<std::byte, 255> buf{};
};

// One end and the adapter state that ao_telemetry.cpp keeps for it.
struct Side {
  sa::BytePump p{};
  Air air{};
  bool station = false;
  std::uint8_t phy = kIdx250;
  bool armed = false;
  bool remote_on_rx = false;
  std::uint8_t pend = 0;
  int ok = 0;
  int revert = 0;
  int cancelled = 0;
  int remote_armed = 0;
  int remote_apply = 0;
};

void boot(Side& s, sc::Tick t) {
  if (s.station) {
    sa::pump_init(s.p, sc::Scid{2}, sc::Scid{1});
  } else {
    sa::pump_init(s.p, sc::Scid{1}, sc::Scid{2});
  }
  // boot_starcom_session: the hail catalog is the runtime PHY.
  s.p.hail_catalog_idx = s.phy;
  sc::macLoadHailCommValue(s.p.mac, sa::pump_comm_value_for_catalog(s.phy));
  // pump_init_for_this_job: the firmware runs FIFO-empty at PHY TxDone.
  s.p.defer_spdu_fifo_empty = true;
  sa::pump_start_session(s.p, s.station, t);
}

// Frame airtime in whole sim steps (ceil), from the radio port query.
sc::Tick air_steps_ms(std::uint8_t phy, std::size_t octets) {
  const auto us = rc::radio_preset_airtime_us(
      rc::kRadioConfigTable[phy], static_cast<std::uint8_t>(octets));
  const sc::Tick ms = static_cast<sc::Tick>((us + 999U) / 1000U);
  return ((ms + kStep - 1U) / kStep) * kStep;
}

// AO_Radio_apply_config_now: only a PHY change re-inits the pump.
void apply(Side& s, std::uint8_t idx, sc::Tick t) {
  if (idx == s.phy) {
    return;
  }
  s.phy = idx;
  s.armed = false;
  s.remote_on_rx = false;
  boot(s, t);
}

// starcom_poll_mac_radio.
void poll(Side& s, sc::Tick t) {
  const auto n = sa::pump_poll_mac_notify(s.p);
  if (n == sc::MacNotify::comm_change_apply_rx && s.p.pending_catalog_valid) {
    s.pend = s.p.pending_catalog_idx;
    s.armed = true;
  } else if (n == sc::MacNotify::comm_change_revert) {
    s.armed = false;
    s.remote_on_rx = false;
    s.p.local_comm_change = false;
    ++s.revert;
    apply(s, s.p.hail_catalog_idx, t);
  } else if (n == sc::MacNotify::hail_fail) {
    sa::pump_start_session(s.p, s.station, t);
  } else if (n == sc::MacNotify::comm_change_failed) {
    sa::pump_cancel_comm_change(s.p);
    s.armed = false;
    s.remote_on_rx = false;
    ++s.cancelled;
  } else {
    /* other notifies: no adapter action */
  }
  if (s.armed && !s.p.local_comm_change) {
    s.armed = false;
  }
  if (s.p.remote_apply_now && s.p.pending_catalog_valid) {
    s.p.remote_apply_now = false;
    s.pend = s.p.pending_catalog_idx;
    s.remote_on_rx = true;
    ++s.remote_armed;
  }
  if (s.p.peer_comm_change && s.armed) {
    s.p.peer_comm_change = false;
    s.p.local_comm_change = false;
    s.armed = false;
    s.remote_on_rx = false;
    ++s.ok;
    apply(s, s.pend, t);
    return;
  }
  const auto phy = sa::pump_mac_phy(s.p);
  if (s.remote_on_rx && phy.receive && !phy.transmit) {
    s.remote_on_rx = false;
    ++s.remote_apply;
    apply(s, s.pend, t);
  }
}

struct Sim {
  Side stn{};
  Side veh{};
  sc::Tick t = 0;
  int drop_station_requests = 0;  // lose this many station request frames
  int drop_vehicle_echoes = 0;    // lose this many vehicle echo frames
  int requests_aired = 0;
  int echoes_aired = 0;
  // How the station got the send turn for each request it aired.
  std::array<sa::TurnCause, 8> request_turns{};
  sc::MacState veh_state_at_request = sc::MacState::s1;
  rc::TelemetryState telem{};
  // Operator `n` is taken after the ticks of a step, before the sends, the
  // same place as a CLI key between AO events.
  bool press_pending = false;
  std::uint8_t press_idx = 0;
  bool press_any_state = true;
  sc::MacState press_state = sc::MacState::s1;
  bool press_done = false;
  bool press_accepted = false;

  void start() {
    stn.station = true;
    veh.station = false;
    boot(stn, 0);
    boot(veh, 0);
    telem.q_w = 32767;
  }

  void step() {
    t += kStep;
    sa::pump_tick(stn.p, t);
    poll(stn, t);
    sa::pump_tick(veh.p, t);
    poll(veh, t);
    if (press_pending && stn.p.mac.mode == sc::MacMode::active &&
        (press_any_state || stn.p.mac.state == press_state)) {
      press_pending = false;
      press_done = true;
      // AO_Telemetry_request_comm_change.
      press_accepted = sa::pump_begin_comm_change(stn.p, press_idx, t);
      poll(stn, t);
    }
    if ((t % kNavPeriod) == 0) {
      std::array<std::byte, 64> pkt{};
      const auto pn = sa::pump_pack_nav_packet(veh.p, pkt, telem);
      if (pn) {
        (void)sa::pump_submit_sdu(
            veh.p, std::span<const std::byte>(pkt.data(), *pn), true);
      }
    }
    end_air(stn, veh);
    end_air(veh, stn);
    start_station_air();
    start_vehicle_air();
  }

  // TxDone of a frame on air: the peer hears it if nothing spoiled it.
  void end_air(Side& tx, Side& rx) {
    if (!tx.air.busy || t < tx.air.end) {
      return;
    }
    tx.air.busy = false;
    sa::pump_spdu_air_complete(tx.p, t);
    poll(tx, t);
    if (!tx.air.lost && rx.phy == tx.air.phy) {
      sa::pump_handle_air(rx.p,
                          std::span<const std::byte>(tx.air.buf.data(),
                                                     tx.air.n));
      poll(rx, t);
    }
  }

  // ao_radio.cpp handle_tx_done: in an active session with TRANSMIT on
  // (211.0 6.5.1, until E43) the radio does not open RX.
  static bool rx_closed(Side const& s) {
    return s.p.mac.mode == sc::MacMode::active &&
           sa::pump_mac_phy(s.p).transmit;
  }

  // A new frame from tx, if its radio is free. Half duplex: if the peer is
  // on air, neither frame is heard.
  bool start_air(Side& tx, Side& rx) {
    if (tx.air.busy) {
      return false;
    }
    const auto n = sa::pump_air_to_send(tx.p, tx.air.buf);
    if (!n.has_value() || *n == 0) {
      return false;
    }
    poll(tx, t);
    tx.air.busy = true;
    tx.air.n = *n;
    tx.air.phy = tx.phy;
    tx.air.end = t + air_steps_ms(tx.phy, *n);
    tx.air.lost = rx.air.busy || rx_closed(rx) || rx.phy != tx.phy;
    if (rx.air.busy) {
      rx.air.lost = true;
    }
    return true;
  }

  void start_station_air() {
    const bool request = stn.p.local_comm_change && stn.p.mac.y == 2 &&
                         sa::pump_fifo_source(stn.p) == sc::MacFifoSource::spdu;
    const sa::TurnCause turn = stn.p.turn_cause;
    // Count only a request on the vehicle PHY (a repeat on the old PHY
    // after the vehicle retuned does not reach it).
    if (!start_air(stn, veh) || !request || veh.phy != stn.phy) {
      return;
    }
    if (static_cast<std::size_t>(requests_aired) < request_turns.size()) {
      request_turns[static_cast<std::size_t>(requests_aired)] = turn;
    }
    ++requests_aired;
    veh_state_at_request = veh.p.mac.state;
    if (drop_station_requests > 0) {
      --drop_station_requests;
      stn.air.lost = true;
    }
  }

  void start_vehicle_air() {
    // The vehicle echo: E69 armed the remote apply, and the SPDU leaves.
    const bool echo = veh.remote_on_rx && veh.p.mac.y == 2 &&
                      sa::pump_fifo_source(veh.p) == sc::MacFifoSource::spdu;
    if (!start_air(veh, stn) || !echo || veh.phy != stn.phy) {
      return;
    }
    ++echoes_aired;
    if (drop_vehicle_echoes > 0) {
      --drop_vehicle_echoes;
      veh.air.lost = true;
    }
  }

  void run_until(sc::Tick end) {
    while (t < end) {
      step();
    }
  }

  // Queue operator `n` for the next step (any state) or for state s.
  void arm_press(std::uint8_t idx) {
    press_pending = true;
    press_idx = idx;
    press_any_state = true;
    press_done = false;
    press_accepted = false;
  }

  void arm_press_in(std::uint8_t idx, sc::MacState s) {
    arm_press(idx);
    press_any_state = false;
    press_state = s;
  }

  // Step until the queued press is taken. False if it is not taken in time.
  bool run_until_pressed(sc::Tick limit) {
    while (t < limit && !press_done) {
      step();
    }
    return press_done;
  }

  // One hop: press in state s, then run long enough to finish.
  bool hop(std::uint8_t idx, sc::MacState s) {
    arm_press_in(idx, s);
    if (!run_until_pressed(t + 5000)) {
      return false;
    }
    run_until(t + 3000);
    return press_accepted;
  }
};

// Station COMM_CHANGE P-frame, the same SPDU order as queueFromCv.
std::size_t station_request(std::uint8_t idx, std::span<std::byte> wire) {
  const auto cv = sa::pump_comm_value_for_catalog(idx);
  std::array<std::byte, 8> spdu{};
  std::size_t n = 0;
  std::array<std::byte, 2> b{};
  const auto add = [&](sc::Result<std::size_t> r) {
    if (r && *r == b.size()) {
      spdu[n] = b[0];
      spdu[n + 1] = b[1];
      n += b.size();
    }
  };
  add(sc::encodeSetPlExt(b, cv.pl_tx));
  add(sc::encodeSetPhy(b, cv.tx, true));
  add(sc::encodeSetPlExt(b, cv.pl_rx));
  add(sc::encodeSetPhy(b, cv.rx, false));
  sc::V3Fields hdr{};
  hdr.p_frame = true;
  hdr.qos_expedited = true;
  hdr.pcid = sa::kSoakPcid;
  hdr.scid = sc::Scid{1};
  hdr.destination = true;
  std::array<std::byte, 32> frame{};
  const auto vn =
      sc::encodeV3(frame, hdr, std::span<const std::byte>(spdu.data(), n));
  if (!vn) {
    return 0;
  }
  const auto pn =
      sc::encodePltu(wire, std::span<const std::byte>(frame.data(), *vn));
  return pn ? *pn : 0;
}

// Vehicle after its own send contact: token passed, S62, receive on.
void vehicle_in_s62(sa::BytePump& v) {
  sa::pump_init(v, sc::Scid{1}, sc::Scid{2});
  sa::pump_start_session(v, false, 0);
  sc::macOnHailReceived(v.mac, 1);
  (void)sa::pump_poll_mac_notify(v);
  const sc::Tick t_end = v.mac.mib.send_duration + 200;
  for (sc::Tick t = kStep; t <= t_end; t += kStep) {
    sa::pump_tick(v, t);
    std::array<std::byte, 255> dump{};
    (void)sa::pump_air_to_send(v, dump);
    if (sa::pump_mac_phy(v).receive && v.mac.state == sc::MacState::s62) {
      return;
    }
  }
}

// Airtime in whole ms (ceil), the same rounding as flight_mac_mib.
sc::Tick airtime_ms(rc::RadioConfigEntry const& e, std::uint8_t octets) {
  const auto us = rc::radio_config_nav_airtime_us(e.sf, e.bw_khz, octets);
  return static_cast<sc::Tick>((us + 999U) / 1000U);
}

// Station P-frame with SET CONTROL PARAMETERS Token = Transmit (E39 token).
std::size_t token_frame(std::span<std::byte> wire) {
  std::array<std::byte, 2> b{};
  sc::MacControlParams cp{};
  cp.pass = true;
  if (!sc::encodeSetControl(b, cp)) {
    return 0;
  }
  sc::V3Fields hdr{};
  hdr.p_frame = true;
  hdr.qos_expedited = true;
  hdr.pcid = sa::kSoakPcid;
  hdr.scid = sc::Scid{2};
  hdr.destination = true;
  std::array<std::byte, 16> frame{};
  const auto vn = sc::encodeV3(frame, hdr, b);
  if (!vn) {
    return 0;
  }
  const auto pn =
      sc::encodePltu(wire, std::span<const std::byte>(frame.data(), *vn));
  return pn ? *pn : 0;
}

// Step the Sim until the station opens a new receive window (E67 or E43:
// S58 to S62, WT = Receive_Duration). Returns false on timeout.
bool run_to_station_rx_open(Sim& sim, sc::Tick limit) {
  sc::MacState prev = sim.stn.p.mac.state;
  while (sim.t < limit) {
    sim.step();
    const auto now = sim.stn.p.mac.state;
    if (now == sc::MacState::s62 && prev == sc::MacState::s58) {
      return true;
    }
    prev = now;
  }
  return false;
}

struct ContactResult {
  bool held = false;  // station stayed in receive until the token arrived
  sc::MacState end = sc::MacState::s1;
  sa::TurnCause cause = sa::TurnCause::none;
};

// One worst-case vehicle contact on preset e, against the station receive
// window (211.0-B-6 6.2.4.17 and 6.2.4.18). The vehicle got the token when
// the station FIFO went empty, Tail_Idle_Duration before the window opened.
// It sends nav PLTUs at the preset rate. The last one starts lead ms before
// its Send_Duration ends (E38 does not cut a frame on air). Then the PLCW and
// the E39 token leave. Frames reach the station at their end of air. The
// station runs alone at 1 ms steps from the window open.
ContactResult run_contact(Sim& sim, rc::RadioConfigEntry const& e,
                          sc::Tick lead) {
  ContactResult r{};
  if (!run_to_station_rx_open(sim, sim.t + 10000)) {
    return r;
  }
  sa::BytePump& stn = sim.stn.p;
  const sc::Tick t0 = sim.t;
  const auto& vm = sim.veh.p.mac.mib;
  const sc::Tick nav_ms = 1000U / e.nav_rate_hz;
  const sc::Tick frame_ms = airtime_ms(e, rc::kRadioConfigNavPltuBytes);
  const sc::Tick report_ms = airtime_ms(e, sa::kReportPltuBytes);
  const sc::Tick s50 = t0 - stn.mac.mib.tail_idle_duration +
                       vm.carrier_only_duration +
                       vm.acquisition_idle_duration;
  const sc::Tick e38 = s50 + vm.send_duration;
  const sc::Tick last_start = e38 - lead;
  const sc::Tick plcw_done = last_start + frame_ms + report_ms;
  const sc::Tick token_done = plcw_done + report_ms;
  std::array<std::byte, 255> nav{};
  const auto nn = sa::pump_encode_nav(sim.veh.p, nav, sim.telem);
  std::array<std::byte, 32> tok{};
  const auto tn = token_frame(tok);
  if (!nn || *nn == 0 || tn == 0) {
    return r;
  }
  const auto nav_span = std::span<const std::byte>(nav.data(), *nn);
  // End-of-air times of the nav PLTUs, last one first.
  std::array<sc::Tick, 64> nav_done{};
  std::size_t n_nav = 0;
  for (sc::Tick start = last_start; start >= s50 && n_nav < nav_done.size();
       start -= nav_ms) {
    nav_done[n_nav++] = start + frame_ms;
    if (start < nav_ms) {
      break;
    }
  }
  r.held = true;
  for (sc::Tick t = t0 + 1U; t <= token_done + 50U; ++t) {
    sa::pump_tick(stn, t);
    bool nav_end = false;
    for (std::size_t k = 0; k < n_nav; ++k) {
      nav_end = nav_end || (nav_done[k] == t);
    }
    if (nav_end || t == plcw_done) {
      sa::pump_handle_air(stn, nav_span);
    }
    if (t == token_done) {
      sa::pump_handle_air(stn, std::span<const std::byte>(tok.data(), tn));
      r.end = stn.mac.state;
      r.cause = stn.turn_cause;
      break;
    }
    const auto st = stn.mac.state;
    if (st != sc::MacState::s60 && st != sc::MacState::s61 &&
        st != sc::MacState::s62) {
      r.held = false;
    }
  }
  return r;
}

}  // namespace

// fe197949 put macOnValidFrame after dispatch. A request heard in S62 then
// fell into the S62 branch and E69 did not run. Fix: 211.0 table 6-10 E47
// (S62 to S61) before dispatch, then table 6-11 E69 in S61.
TEST(StarcomCommChange, VehicleInS62TakesStationRequest) {
  static sa::BytePump veh{};
  vehicle_in_s62(veh);
  ASSERT_EQ(veh.mac.state, sc::MacState::s62);
  ASSERT_EQ(veh.mac.y, 0);
  std::array<std::byte, 255> wire{};
  const auto n = station_request(kIdx500, wire);
  ASSERT_GT(n, 0U);
  sa::pump_handle_air(veh, std::span<const std::byte>(wire.data(), n));
  EXPECT_TRUE(veh.remote_apply_now);
  EXPECT_TRUE(veh.pending_catalog_valid);
  EXPECT_EQ(veh.pending_catalog_idx, kIdx500);
  EXPECT_FALSE(veh.peer_comm_change);
  EXPECT_EQ(veh.mac.state, sc::MacState::s51);  // E69
  EXPECT_EQ(veh.mac.y, 2);
  // Table 6-11 E69 loads the TX and the RX values in one event. The first
  // SPDU (SET PL) gives the catalog idx, and pump_comm_value_for_catalog
  // makes all four values from it. SPDUs 2 to 4 then meet S51 and drop.
  // They carry the same catalog and the fixed bypass encoding.
  ASSERT_TRUE(veh.mac.pending_cv_valid);
  EXPECT_TRUE(veh.mac.pending_cv.has_pl_tx);
  EXPECT_TRUE(veh.mac.pending_cv.has_pl_rx);
  EXPECT_EQ(sa::pump_catalog_from_pl(veh.mac.pending_cv.pl_tx), kIdx500);
  EXPECT_EQ(sa::pump_catalog_from_pl(veh.mac.pending_cv.pl_rx), kIdx500);
  EXPECT_EQ(veh.cc_drop_n, 3U);
}

TEST(StarcomCommChange, VehicleInS60TakesStationRequest) {
  static sa::BytePump veh{};
  vehicle_in_s62(veh);
  sc::macOnValidFrame(veh.mac, veh.mac.last_now);
  ASSERT_EQ(veh.mac.state, sc::MacState::s60);
  std::array<std::byte, 255> wire{};
  const auto n = station_request(kIdx500, wire);
  sa::pump_handle_air(veh, std::span<const std::byte>(wire.data(), n));
  EXPECT_TRUE(veh.remote_apply_now);
  EXPECT_EQ(veh.mac.state, sc::MacState::s51);
}

// A reconnect hail heard in S62 is not E69 (fe197949 behavior kept). E47
// runs first, then the hail check in S61.
TEST(StarcomCommChange, HailCatalogInS62IsNotE69) {
  static sa::BytePump veh{};
  vehicle_in_s62(veh);
  std::array<std::byte, 255> wire{};
  const auto n = station_request(veh.hail_catalog_idx, wire);
  sa::pump_handle_air(veh, std::span<const std::byte>(wire.data(), n));
  EXPECT_FALSE(veh.remote_apply_now);
  EXPECT_FALSE(veh.peer_comm_change);
}

// Operator `n` while the station receives (the usual case), three hops.
TEST(StarcomCommChange, StationInitiatedHopEndToEnd) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  const std::array<std::uint8_t, 3> path{kIdx500, kIdx125, kIdx250};
  int hops = 0;
  for (const std::uint8_t idx : path) {
    ASSERT_TRUE(sim.hop(idx, sc::MacState::s60));
    ++hops;
    EXPECT_EQ(sim.stn.phy, idx);
    EXPECT_EQ(sim.veh.phy, idx);
    EXPECT_EQ(sim.stn.ok, hops);
    EXPECT_EQ(sim.veh.remote_apply, hops);
    EXPECT_FALSE(sim.stn.armed);
    EXPECT_EQ(sim.stn.p.hail_catalog_idx, idx);
    EXPECT_EQ(sim.veh.p.hail_catalog_idx, idx);
  }
  EXPECT_EQ(sim.veh_state_at_request, sc::MacState::s62);
  EXPECT_EQ(sim.stn.revert, 0);
  EXPECT_EQ(sim.stn.cancelled, 0);
}

// Same path with `n` in the station send contact (E63).
TEST(StarcomCommChange, StationInitiatedHopFromS50) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  const std::array<std::uint8_t, 3> path{kIdx500, kIdx125, kIdx250};
  for (const std::uint8_t idx : path) {
    ASSERT_TRUE(sim.hop(idx, sc::MacState::s50));
    EXPECT_EQ(sim.stn.phy, idx);
    EXPECT_EQ(sim.veh.phy, idx);
  }
  EXPECT_EQ(sim.stn.ok, 3);
  EXPECT_EQ(sim.veh.remote_apply, 3);
}

// Bench 2026-11-25 shape: hop 1 then more hops pressed while receiving.
TEST(StarcomCommChange, SecondHopAfterFirstCompletes) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s50));
  ASSERT_EQ(sim.veh.phy, kIdx500);
  ASSERT_TRUE(sim.hop(kIdx125, sc::MacState::s60));
  EXPECT_EQ(sim.stn.phy, kIdx125);
  EXPECT_EQ(sim.veh.phy, kIdx125);
  EXPECT_EQ(sim.veh.remote_armed, 2);
  EXPECT_FALSE(sim.veh.p.peer_comm_change);
}

// Press time over one full contact cycle. Every accepted press completes.
TEST(StarcomCommChange, PressTimeSweep) {
  constexpr sc::Tick kCycleSpan = 1500;
  int accepted = 0;
  int completed = 0;
  int refused = 0;
  for (sc::Tick off = 0; off < kCycleSpan; off += kStep) {
    SCOPED_TRACE(off);
    static Sim sim{};
    sim = Sim{};
    sim.start();
    sim.run_until(kSessionUp + off);
    sim.arm_press(kIdx500);
    ASSERT_TRUE(sim.run_until_pressed(sim.t + 5000));
    if (sim.press_accepted) {
      ++accepted;
      sim.run_until(sim.t + 3000);
      if (sim.stn.phy == kIdx500 && sim.veh.phy == kIdx500) {
        ++completed;
      }
    } else {
      ++refused;
      // A refused press leaves the link up and unchanged.
      sim.run_until(sim.t + 3000);
      EXPECT_EQ(sim.stn.phy, kIdx250);
      EXPECT_FALSE(sim.stn.armed);
    }
  }
  EXPECT_EQ(completed, accepted);
  // A press in S56 with Y=0 is refused (PressInS56IsRefused). S56 holds
  // the E39 token until its TxDone, so the sweep meets it for the token
  // airtime in steps plus the step that queues it, in one station contact.
  const sc::Tick s56_steps =
      (air_steps_ms(kIdx250, sa::kReportPltuBytes) / kStep) + 1U;
  EXPECT_LE(refused, static_cast<int>(s56_steps));
  EXPECT_GT(accepted, 140);
}

// Step until the station waits for the echo (S62, Y=3).
void run_until_station_waits(Sim& sim) {
  const sc::Tick limit = sim.t + 10000;
  while (!(sim.stn.p.mac.state == sc::MacState::s62 &&
           sim.stn.p.mac.y == 3) &&
         sim.t < limit) {
    sim.step();
  }
}

// One P-frame or U-frame from the vehicle (SCID 2 to the station).
std::size_t vehicle_frame(bool p_frame, std::span<const std::byte> data,
                          std::span<std::byte> wire) {
  sc::V3Fields hdr{};
  hdr.p_frame = p_frame;
  hdr.qos_expedited = true;
  hdr.pcid = sa::kSoakPcid;
  hdr.scid = sc::Scid{2};
  hdr.destination = true;
  std::array<std::byte, 64> frame{};
  const auto fn = sc::encodeV3(frame, hdr, data);
  if (!fn) {
    return 0;
  }
  const auto n =
      sc::encodePltu(wire, std::span<const std::byte>(frame.data(), *fn));
  return n ? *n : 0;
}

// Adapter gate: a SET CONTROL token is not the COMM_CHANGE echo. E47 and
// E49 run (211.0 table 6-10); E68 does not.
TEST(StarcomCommChange, TokenDoesNotConfirmHop) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.arm_press_in(kIdx500, sc::MacState::s60);
  ASSERT_TRUE(sim.run_until_pressed(sim.t + 5000));
  ASSERT_TRUE(sim.press_accepted);
  run_until_station_waits(sim);
  ASSERT_EQ(sim.stn.p.mac.y, 3);
  ASSERT_FALSE(sim.stn.p.peer_comm_change);
  sc::MacControlParams ctl{};
  ctl.pass = true;
  std::array<std::byte, 2> spdu{};
  ASSERT_TRUE(sc::encodeSetControl(spdu, ctl).has_value());
  std::array<std::byte, 255> wire{};
  const auto n = vehicle_frame(true, spdu, wire);
  ASSERT_GT(n, 0U);
  sa::pump_handle_air(sim.stn.p, std::span<const std::byte>(wire.data(), n));
  EXPECT_FALSE(sim.stn.p.peer_comm_change);
  EXPECT_EQ(sim.stn.p.mac.y, 3);
  EXPECT_EQ(sim.stn.p.mac.state, sc::MacState::s51);  // E49
  EXPECT_NE(sa::pump_poll_mac_notify(sim.stn.p),
            sc::MacNotify::comm_change_ok);
}

// Adapter gate for all frame types: a U-frame (nav data) on the old PHY is
// not the echo. E68 does not run; the station keeps waiting.
TEST(StarcomCommChange, DataFrameDoesNotConfirmHop) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.arm_press_in(kIdx500, sc::MacState::s60);
  ASSERT_TRUE(sim.run_until_pressed(sim.t + 5000));
  run_until_station_waits(sim);
  ASSERT_EQ(sim.stn.p.mac.y, 3);
  const std::array<std::byte, 8> nav{};
  std::array<std::byte, 255> wire{};
  const auto n = vehicle_frame(false, nav, wire);
  ASSERT_GT(n, 0U);
  sa::pump_handle_air(sim.stn.p, std::span<const std::byte>(wire.data(), n));
  EXPECT_EQ(sim.stn.p.mac.y, 3);
  EXPECT_EQ(sim.stn.p.mac.state, sc::MacState::s61);  // E47 only
  EXPECT_TRUE(sim.stn.p.local_comm_change);
  EXPECT_NE(sa::pump_poll_mac_notify(sim.stn.p),
            sc::MacNotify::comm_change_ok);
}

// The request is lost. The vehicle keeps sending on the old PHY. No frame
// runs E68. The station window holds the vehicle timer contact, so the
// station gets the vehicle token (E49) with Comm_Change_Lifetime 0 used up.
// It aborts (comm_change_failed, 4.2.2.5.4), keeps the old PHY and the hail
// value, and the next press completes.
TEST(StarcomCommChange, LostRequestCancelsHop) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.stn.p.mac.mib.comm_change_lifetime = 0;  // send once, no repeat
  sim.drop_station_requests = 1;
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.requests_aired, 1);
  EXPECT_EQ(sim.stn.revert, 0);
  EXPECT_EQ(sim.stn.cancelled, 1);
  EXPECT_EQ(sim.stn.ok, 0);
  EXPECT_FALSE(sim.stn.armed);
  EXPECT_FALSE(sim.stn.p.local_comm_change);
  EXPECT_EQ(sim.stn.p.mac.y, 0);
  EXPECT_EQ(sim.stn.phy, kIdx250);
  EXPECT_EQ(sim.veh.phy, kIdx250);
  EXPECT_EQ(sa::pump_catalog_from_pl(sim.stn.p.mac.hail_cv.pl_tx), kIdx250);
  EXPECT_EQ(sim.veh.remote_armed, 0);
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.stn.phy, kIdx500);
  EXPECT_EQ(sim.veh.phy, kIdx500);
}

// 211.0 6.2.4.10.2: with Comm_Change_Lifetime 1 (one repeat) the station
// sends the COMM_CHANGE again after a lost request. The repeat follows the
// vehicle token (E49, then E64 and E65). Here the repeat is lost too. After
// the next token the lifetime is used up: the station aborts with
// comm_change_failed (4.2.2.5.4) and both ends keep the old PHY.
TEST(StarcomCommChange, LostRequestRepeatsWithinLifetime) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.stn.p.mac.mib.comm_change_lifetime = 1;
  sim.drop_station_requests = 2;
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  sim.run_until(sim.t + 3000);
  EXPECT_EQ(sim.requests_aired, 2);
  EXPECT_EQ(sim.request_turns[1], sa::TurnCause::token);
  EXPECT_EQ(sim.stn.revert, 0);
  EXPECT_EQ(sim.stn.cancelled, 1);
  EXPECT_EQ(sim.stn.ok, 0);
  EXPECT_FALSE(sim.stn.p.local_comm_change);
  EXPECT_EQ(sim.stn.phy, kIdx250);
  EXPECT_EQ(sim.veh.phy, kIdx250);
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.stn.phy, kIdx500);
  EXPECT_EQ(sim.veh.phy, kIdx500);
}

// Lost echo: the vehicle takes the request (E69), sends the echo on the old
// PHY, then retunes on its next receive. The station does not hear the echo.
// It repeats the request on the old PHY inside Comm_Change_Lifetime (2
// repeats, 211.0-B-6 6.2.4.10), but the vehicle already left. Then it
// aborts (comm_change_failed, 4.2.2.5.4) and stays on the old PHY.
// OPEN DEFECT (R-37): the ends split. The vehicle re-inits on the new PHY,
// so its hail value is the new PHY and no revert brings it back. Each end
// hails or listens on its own PHY. This test records that end state; change
// it when the split is fixed.
TEST(StarcomCommChange, LostEchoSplitsPresets) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.drop_vehicle_echoes = 1;
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.requests_aired, 1);  // the repeats leave on the old PHY
  EXPECT_EQ(sim.echoes_aired, 1);
  EXPECT_EQ(sim.veh.remote_apply, 1);
  EXPECT_EQ(sim.stn.ok, 0);
  EXPECT_EQ(sim.stn.phy, kIdx250);
  EXPECT_EQ(sim.veh.phy, kIdx500);
  EXPECT_EQ(sim.stn.p.hail_catalog_idx, kIdx250);
  EXPECT_EQ(sim.veh.p.hail_catalog_idx, kIdx500);
  sim.run_until(sim.t + 60000);
  EXPECT_EQ(sim.stn.revert + sim.stn.cancelled, 1);  // one abort
  EXPECT_EQ(sim.stn.phy, kIdx250);
  EXPECT_EQ(sim.veh.phy, kIdx500);
  EXPECT_EQ(sim.veh.revert, 0);
  EXPECT_NE(sim.stn.p.mac.mode, sc::MacMode::active);
  EXPECT_NE(sim.veh.p.mac.mode, sc::MacMode::active);
}

// 211.0 table 6-10 E42 leaves S56 only with Y=0. A press there is refused.
TEST(StarcomCommChange, PressInS56IsRefused) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.arm_press_in(kIdx500, sc::MacState::s56);
  ASSERT_TRUE(sim.run_until_pressed(sim.t + 5000));
  EXPECT_FALSE(sim.press_accepted);
  EXPECT_FALSE(sim.stn.p.local_comm_change);
  // The contact still closes and the link keeps its PHY.
  sim.run_until(sim.t + 3000);
  EXPECT_EQ(sim.stn.phy, kIdx250);
  EXPECT_NE(sim.stn.p.mac.state, sc::MacState::s56);
  // The next press completes.
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.veh.phy, kIdx500);
}

// A new session ends a hop in flight.
TEST(StarcomCommChange, StartSessionClearsHop) {
  static sa::BytePump stn{};
  sa::pump_init(stn, sc::Scid{2}, sc::Scid{1});
  stn.local_comm_change = true;
  stn.peer_comm_change = true;
  stn.remote_apply_now = true;
  sa::pump_start_session(stn, true, 0);
  EXPECT_FALSE(stn.local_comm_change);
  EXPECT_FALSE(stn.peer_comm_change);
  EXPECT_FALSE(stn.remote_apply_now);
}

// R-37 log: a COMM_CHANGE that reaches the vehicle while it holds the send
// turn (S50) does not start E69 (211.0-B-6 table 6-11). The pump records
// the drop and its state for the AO log.
TEST(StarcomCommChange, DropInS50IsRecorded) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  while (sim.veh.p.mac.state != sc::MacState::s50 && sim.t < kSessionUp + 5000) {
    sim.step();
  }
  ASSERT_EQ(sim.veh.p.mac.state, sc::MacState::s50);
  std::array<std::byte, 255> wire{};
  const auto n = station_request(kIdx500, wire);
  ASSERT_GT(n, 0U);
  sim.veh.p.cc_drop_new = false;
  sa::pump_handle_air(sim.veh.p, std::span<const std::byte>(wire.data(), n));
  EXPECT_TRUE(sim.veh.p.cc_drop_new);
  EXPECT_EQ(sim.veh.p.cc_drop_state, sc::MacState::s50);
  EXPECT_EQ(sim.veh.p.cc_drop_why, sa::CommChangeDrop::not_s60_61);
  EXPECT_FALSE(sim.veh.p.remote_apply_now);
  EXPECT_EQ(sim.veh.p.mac.y, 0);
}

// R-37 log: each dropped SPDU is one event with the frame catalog idx, its
// position and its directive type (queueFromCv order), and the device ms.
TEST(StarcomCommChange, DropEventsNameEachSpdu) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  while (sim.veh.p.mac.state != sc::MacState::s50 && sim.t < kSessionUp + 5000) {
    sim.step();
  }
  ASSERT_EQ(sim.veh.p.mac.state, sc::MacState::s50);
  sa::PumpEvent e{};
  while (sa::pump_take_event(sim.veh.p, e)) {
  }
  std::array<std::byte, 255> wire{};
  const auto n = station_request(kIdx500, wire);
  ASSERT_GT(n, 0U);
  sa::pump_handle_air(sim.veh.p, std::span<const std::byte>(wire.data(), n));
  const std::array<std::uint8_t, 4> types{
      sc::kSetPlExtDirectiveType, sc::kSetTxDirectiveType,
      sc::kSetPlExtDirectiveType, sc::kSetRxDirectiveType};
  for (std::size_t i = 0; i < types.size(); ++i) {
    ASSERT_TRUE(sa::pump_take_event(sim.veh.p, e));
    EXPECT_EQ(e.turn, sa::TurnEvent::none);
    EXPECT_EQ(e.state, sc::MacState::s50);
    EXPECT_EQ(e.why, sa::CommChangeDrop::not_s60_61);
    EXPECT_EQ(e.idx, kIdx500);
    EXPECT_EQ(e.cmd_pos, i + 1U);
    EXPECT_EQ(e.cmd_n, 4U);
    EXPECT_EQ(e.cmd_type, types[i]);
    EXPECT_EQ(e.t, sim.veh.p.mac.last_now);
  }
  EXPECT_FALSE(sa::pump_take_event(sim.veh.p, e));
}

// R-37 log: the ring keeps the turn events of a contact cycle in order.
// A full ring counts the events it does not keep.
TEST(StarcomCommChange, TurnEventsReachTheRing) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sa::PumpEvent e{};
  while (sa::pump_take_event(sim.stn.p, e)) {
  }
  const std::uint32_t lost0 = sim.stn.p.ev_lost;
  bool e49 = false;
  bool s50 = false;
  bool s62 = false;
  sc::Tick last_t = 0;
  for (int k = 0; k < 400; ++k) {
    sim.step();
    while (sa::pump_take_event(sim.stn.p, e)) {
      EXPECT_GE(e.t, last_t);
      last_t = e.t;
      e49 = e49 || e.turn == sa::TurnEvent::e49;
      s50 = s50 || e.turn == sa::TurnEvent::enter_s50;
      s62 = s62 || e.turn == sa::TurnEvent::enter_s62;
    }
  }
  EXPECT_TRUE(e49);
  EXPECT_TRUE(s50);
  EXPECT_TRUE(s62);
  EXPECT_EQ(sim.stn.p.ev_lost, lost0);
  sim.run_until(sim.t + 30000);  // no drain: the ring fills
  EXPECT_EQ(sim.stn.p.ev_len, sa::kPumpEventCap);
  EXPECT_GT(sim.stn.p.ev_lost, lost0);
}

// R-37 log: the station records the COMM_CHANGE send (table 6-11 E65), its
// state and how it got the send turn.
TEST(StarcomCommChange, SendIsRecorded) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  sim.stn.p.cc_sent_new = false;
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  EXPECT_EQ(sim.stn.phy, kIdx500);
  // The hop re-inits the pump. Check the record from a hop that fails.
  sim.drop_station_requests = 1;
  sim.stn.p.mac.mib.comm_change_lifetime = 0;
  sim.stn.p.cc_sent_new = false;
  sim.arm_press_in(kIdx250, sc::MacState::s60);
  ASSERT_TRUE(sim.run_until_pressed(sim.t + 5000));
  sim.run_until(sim.t + 500);
  EXPECT_TRUE(sim.stn.p.cc_sent_new);
  EXPECT_EQ(sim.stn.p.cc_sent_n, 0U);
  EXPECT_NE(sim.stn.p.cc_sent_turn, sa::TurnCause::none);
}

// R-37 overrun. The station receive window must hold one worst-case vehicle
// contact on every preset in the rate table, so the timer back-up (E48 or
// E50) does not take the turn while the vehicle still sends. 211.0-B-6
// 6.2.4.18; 210.0-G-2 4.1.6.5.3 (the token is the primary turnaround).
// Bench 134db12d: hops from 125/5 and 250/10 failed.
// R-37 hop 12 (bench 2026-11-25): the window must also hold a vehicle
// timer contact after a lost station frame. That is the vehicle
// Receive_Duration (from its own S62 entry, before the station E67), E48 or
// E50, S51 and S52, Send_Duration, one nav PLTU still on air, the PLCW and
// the token.
TEST(StarcomCommChange, StationWindowHoldsWorstContactAllPresets) {
  {
    static Sim sim{};
    sim = Sim{};
    sim.start();
    const auto& sm = sim.stn.p.mac.mib;
    const auto& vm = sim.veh.p.mac.mib;
    for (std::size_t i = 0; i < rc::kRadioConfigTableSize; ++i) {
      const auto& e = rc::kRadioConfigTable[i];
      const sc::Tick timer_contact =
          vm.receive_duration + vm.carrier_only_duration +
          vm.acquisition_idle_duration + vm.send_duration +
          airtime_ms(e, rc::kRadioConfigNavPltuBytes) +
          (2U * airtime_ms(e, sa::kReportPltuBytes));
      EXPECT_GE(sm.receive_duration, timer_contact) << "preset idx " << i;
      RecordProperty("margin_ms_idx" + std::to_string(i),
                     static_cast<int>(sm.receive_duration) -
                         static_cast<int>(timer_contact));
    }
  }
  // The phase of the last nav PLTU against E38 sets where the gaps fall,
  // so sweep it over one nav period (lead 1 ms .. one period).
  constexpr sc::Tick kLeadStep = 3;
  for (std::size_t i = 0; i < rc::kRadioConfigTableSize; ++i) {
    const auto& e = rc::kRadioConfigTable[i];
    const sc::Tick nav_ms = 1000U / e.nav_rate_hz;
    int bad = 0;
    for (sc::Tick lead = 1; lead <= nav_ms && bad < 3; lead += kLeadStep) {
      static Sim sim{};
      sim = Sim{};
      sim.start();
      sim.run_until(kSessionUp);
      const auto r = run_contact(sim, e, lead);
      const bool ok = r.held && r.end == sc::MacState::s51 &&
                      r.cause == sa::TurnCause::token;
      if (!ok) {
        ++bad;
        ADD_FAILURE() << "preset idx " << i << " lead " << lead
                      << " ms: held=" << r.held
                      << " end=S" << static_cast<unsigned>(r.end)
                      << " cause=" << static_cast<unsigned>(r.cause);
      }
    }
  }
}

// 211.0-B-6 table 6-10. Receive_Duration ends with carrier lock true: E44,
// stay in S60 and keep receiving. With carrier lock false: E48, take the
// turn. The RC carrier input is "a frame ended inside Tail_Idle_Duration".
TEST(StarcomCommChange, TimeoutCarrierTrueIsE44CarrierFalseIsE48) {
  for (int carrier = 0; carrier < 2; ++carrier) {
    static Sim sim{};
    sim = Sim{};
    sim.start();
    sim.run_until(kSessionUp);
    ASSERT_TRUE(run_to_station_rx_open(sim, sim.t + 10000));
    sa::BytePump& stn = sim.stn.p;
    std::array<std::byte, 255> nav{};
    const auto nn = sa::pump_encode_nav(sim.veh.p, nav, sim.telem);
    ASSERT_TRUE(nn && *nn > 0);
    const auto nav_span = std::span<const std::byte>(nav.data(), *nn);
    const sc::Tick t0 = sim.t;
    const sc::Tick rd = stn.mac.mib.receive_duration;
    const sc::Tick last = (carrier != 0) ? (t0 + rd - 2U) : (t0 + rd - 50U);
    (void)sa::pump_poll_mac_notify(stn);
    sc::MacNotify seen = sc::MacNotify::none;
    for (sc::Tick t = t0 + 1U; t <= t0 + rd + 1U; ++t) {
      sa::pump_tick(stn, t);
      if (t == t0 + 100U || t == last) {
        sa::pump_handle_air(stn, nav_span);
      }
      const auto n = sa::pump_poll_mac_notify(stn);
      if (n != sc::MacNotify::none) {
        seen = n;
      }
    }
    if (carrier != 0) {
      EXPECT_EQ(stn.mac.state, sc::MacState::s60);  // E44
      EXPECT_EQ(seen, sc::MacNotify::sender_overran);
    } else {
      EXPECT_EQ(stn.mac.state, sc::MacState::s51);  // E48
      EXPECT_EQ(stn.turn_cause, sa::TurnCause::timer_e48);
    }
  }
}

// R-37 hop 12 (bench 2026-11-25): a lost request on any preset. The
// vehicle takes its next turn by the E48/E50 timer. The station window
// holds that contact, so the repeat follows the vehicle token (E49), not the
// station E50 timer, and the vehicle hears it. The hop completes at the
// flight Comm_Change_Lifetime (211.0-B-6 6.2.4.10).
TEST(StarcomCommChange, LostRequestRecoversAtFlightLifetime) {
  for (std::size_t i = 0; i < rc::kRadioConfigTableSize; ++i) {
    const auto from = static_cast<std::uint8_t>(i);
    if (!sa::pump_catalog_fits(from)) {
      continue;  // 125/10: no nav fit, NAV_PRESET skips it
    }
    SCOPED_TRACE(static_cast<unsigned>(from));
    const std::uint8_t to = (from == kIdx125) ? kIdx500 : kIdx125;
    static Sim sim{};
    sim = Sim{};
    sim.start();
    sim.run_until(kSessionUp);
    ASSERT_EQ(sim.stn.p.mac.mib.comm_change_lifetime, sa::kCommChangeLifetime);
    if (from != sim.stn.phy) {
      ASSERT_TRUE(sim.hop(from, sc::MacState::s60));
      ASSERT_EQ(sim.stn.phy, from);
      ASSERT_EQ(sim.veh.phy, from);
    }
    const int ok0 = sim.stn.ok;
    sim.requests_aired = 0;
    sim.drop_station_requests = 1;
    ASSERT_TRUE(sim.hop(to, sc::MacState::s60));
    sim.run_until(sim.t + 3000);
    EXPECT_EQ(sim.requests_aired, 2);
    EXPECT_EQ(sim.request_turns[0], sa::TurnCause::token);
    EXPECT_EQ(sim.request_turns[1], sa::TurnCause::token);  // E49, not E50
    EXPECT_EQ(sim.stn.cancelled, 0);
    EXPECT_EQ(sim.stn.revert, 0);
    EXPECT_EQ(sim.stn.ok, ok0 + 1);
    EXPECT_EQ(sim.stn.phy, to);
    EXPECT_EQ(sim.veh.phy, to);
  }
}

// R-37: Send_Duration and Receive_Duration are session static (211.0-B-6
// annex C; 210.0-G-2 2.3.4.3). They are sized once for the slowest preset
// and do not change when a hop re-inits the pumps.
TEST(StarcomCommChange, SessionWindowsStayConstantAcrossHops) {
  static Sim sim{};
  sim = Sim{};
  sim.start();
  sim.run_until(kSessionUp);
  const sc::MacMib s0 = sim.stn.p.mac.mib;
  const sc::MacMib v0 = sim.veh.p.mac.mib;
  // Golden values (ms) for the rate table at 134db12d: worst nav PLTU
  // 129 ms and report PLTU 47 ms (125 kHz SF7), hail preset 250/10.
  // Station: vehicle receive 318 + turn 30 + vehicle send 1200 + contact
  // tail 253 (R-37 hop 12).
  EXPECT_EQ(s0.receive_duration, 1801U);
  EXPECT_EQ(v0.receive_duration, 318U);
  EXPECT_EQ(s0.send_duration, 65U);
  EXPECT_EQ(v0.send_duration, 1200U);
  ASSERT_TRUE(sim.hop(kIdx500, sc::MacState::s60));
  ASSERT_EQ(sim.stn.phy, kIdx500);
  ASSERT_TRUE(sim.hop(kIdx125, sc::MacState::s60));
  ASSERT_EQ(sim.stn.phy, kIdx125);
  EXPECT_EQ(sim.stn.p.mac.mib.receive_duration, s0.receive_duration);
  EXPECT_EQ(sim.stn.p.mac.mib.send_duration, s0.send_duration);
  EXPECT_EQ(sim.veh.p.mac.mib.receive_duration, v0.receive_duration);
  EXPECT_EQ(sim.veh.p.mac.mib.send_duration, v0.send_duration);
  // The fast preset traffic also fits: one worst contact on 500/10.
  const auto r = run_contact(sim, rc::kRadioConfigTable[kIdx500], 1U);
  EXPECT_TRUE(r.held);
  EXPECT_EQ(r.cause, sa::TurnCause::token);
}
