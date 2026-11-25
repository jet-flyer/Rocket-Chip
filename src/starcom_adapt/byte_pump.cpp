// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/byte_pump.h"

#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#include "flight_director/mission_profile_data.h"
#include "rocketchip/radio_config_table.h"
#include "rocketchip/radio_config_airtime.h"
#include "rocketchip/job.h"
#include "rocketchip/host_build.h"
#include "starcom/error.h"

#include <tl/expected.hpp>

#include <cstring>

namespace rc::starcom_adapt {

namespace {

// Airtime in whole ms (ceil) from the radio port query. The port answers
// per preset (rocketchip/radio_config_airtime.h); no modem formula here.
starcom::ccsds::Tick preset_airtime_ms(rc::RadioConfigEntry const& e,
                                       std::uint8_t octets) noexcept {
  const std::uint32_t us = rc::radio_preset_airtime_us(e, octets);
  return static_cast<starcom::ccsds::Tick>((us + 999U) / 1000U);
}

// The slowest preset in the rate table for this frame size.
starcom::ccsds::Tick worst_preset_airtime_ms(std::uint8_t octets) noexcept {
  starcom::ccsds::Tick worst = 0;
  for (std::size_t i = 0; i < rc::kRadioConfigTableSize; ++i) {
    const auto t = preset_airtime_ms(rc::kRadioConfigTable[i], octets);
    if (t > worst) {
      worst = t;
    }
  }
  return worst;
}

// The boot hail preset as a rate-table row (kRadioConfigTable 250/10 if
// the default is not in the table).
rc::RadioConfigEntry const& hail_preset() noexcept {
  const auto& d = rc::kDefaultRocketRadioConfig;
  std::uint8_t idx = rc::radio_config_catalog_index(
      d.bandwidth_khz, d.nav_rate_hz, d.spreading_factor, d.coding_rate,
      d.power_dbm);
  if (idx >= rc::kRadioConfigTableSize) {
    idx = 2;
  }
  return rc::kRadioConfigTable[idx];
}

// Session-static MAC windows in ms. Annex C: these timers are session
// static and set before the session (210.0-G-2 2.3.4.3), so they are sized
// once for the slowest preset in the rate table and do not change at a hop.
struct MacWindows {
  starcom::ccsds::Tick stn_send = 0;
  starcom::ccsds::Tick veh_send = 0;
  starcom::ccsds::Tick stn_recv = 0;
  starcom::ccsds::Tick veh_recv = 0;
  starcom::ccsds::Tick carrier_loss = 0;
};

MacWindows session_windows(starcom::ccsds::Tick nav_ms,
                           starcom::ccsds::Tick turn_ms) noexcept {
  // PLCW (Fig 3-5) and the E39 SET CONTROL token are each a 2-octet
  // P-frame. Both leave after Send_Duration (table 6-10 E38 then E39).
  constexpr unsigned kReportOctets = 2U;
  constexpr uint8_t kReportPltuSum = static_cast<uint8_t>(
      starcom::ccsds::kPltuAsmSize + starcom::ccsds::kV3HeaderSize +
      kReportOctets + starcom::ccsds::kPltuCrcSize);
  static_assert(kReportPltuSum == kReportPltuBytes, "report PLTU layout drifted");
  MacWindows w{};
  // 211.0 6.2.4.17: each side's Send_Duration is local. Sized on the hail
  // preset, as before. N=11 (~90% packing). Table: starcom_adapt/README.md.
  w.stn_send = preset_airtime_ms(hail_preset(), rc::kRadioConfigNavPltuBytes);
  const starcom::ccsds::Tick tax_ms = w.stn_send + (2U * turn_ms);
  w.veh_send = ((9U * tax_ms) + (nav_ms - 1U)) / nav_ms * nav_ms;
  // R-37. Receive_Duration (6.2.4.18) is the timer back-up for a missed
  // token (table 6-10 E48/E50); the token is the primary turnaround
  // (210.0-G-2 4.1.6.5.3). The contact tail is the open and tail idles,
  // one frame still on air at E38, and the PLCW and the token.
  const starcom::ccsds::Tick contact_tail =
      turn_ms + worst_preset_airtime_ms(rc::kRadioConfigNavPltuBytes) +
      (2U * worst_preset_airtime_ms(kReportPltuSum));
  w.veh_recv = w.stn_send + contact_tail;
  // R-37 hop 12 (bench 2026-11-25). A lost station frame leaves the
  // vehicle without a token. The vehicle waits its Receive_Duration, takes
  // the turn by E48 or E50, then sends one whole contact. The station window
  // holds all of that, so the station repeat follows the vehicle token (E49)
  // and does not leave by E50 while the vehicle sends. One more turn_ms is
  // slack for the 10 ms tick of each end (project choice).
  w.stn_recv = w.veh_recv + turn_ms + w.veh_send + contact_tail;
  // One vehicle data-services contact, including the frames after E38.
  // Only the responder uses it (table 6-10 E85, S60 to S2).
  w.carrier_loss = w.veh_send + contact_tail;
  return w;
}

starcom::ccsds::MacMib flight_mac_mib(starcom::ccsds::Scid local) noexcept {
  starcom::ccsds::MacMib m{};
  const uint8_t hz = rc::kDefaultRocketRadioConfig.nav_rate_hz;
  const starcom::ccsds::Tick nav_ms =
      (hz == 0U) ? 200U : static_cast<starcom::ccsds::Tick>(1000U / hz);
  // SX1276 §4.1.1.6 SF7 / 250 kHz: Tsym = 512 us; 8-symbol preamble
  // 12.25 Tsym = 6.272 ms. 10 ms is ceil-to-tick.
  m.carrier_only_duration = kMacIdleTickMs;
  m.acquisition_idle_duration = kMacIdleTickMs;
  m.tail_idle_duration = kMacIdleTickMs;
  const starcom::ccsds::Tick turn_ms =
      m.carrier_only_duration + m.acquisition_idle_duration +
      m.tail_idle_duration;
  const MacWindows w = session_windows(nav_ms, turn_ms);
  const bool station = (local == starcom::ccsds::Scid{2});
  if (station) {
    m.send_duration = w.stn_send;
    m.receive_duration = w.stn_recv;
  } else {
    m.send_duration = w.veh_send;
    m.receive_duration = w.veh_recv;
  }
  m.hail_wait_duration = nav_ms + kHailWaitSlackMs;
  m.hail_lifetime = 0;  // 211.0 6.2.4.14.2: 0 = no abort
  m.drop_carrier_duration = kDropCarrierMs;
  m.maximum_failed_token_passes = 4;  // 211.0 table 6-12 E83; 0 = unlimited
  m.comm_change_lifetime = kCommChangeLifetime;
  m.carrier_loss_timer_duration = w.carrier_loss;
  m.plcw_repeat_interval = w.veh_send;
  m.local_scid = local;
  m.local_pcid = kSoakPcid;
  return m;
}

starcom::ccsds::Result<std::size_t> wrap_mac_p_frame(
    BytePump& p, std::span<std::byte> out,
    std::span<const std::byte> spdu) noexcept {
  starcom::ccsds::V3Fields hdr{};
  hdr.p_frame = true;
  hdr.qos_expedited = true;
  hdr.pcid = kSoakPcid;
  hdr.scid = p.remote_scid;
  hdr.destination = true;
  std::array<std::byte, 5U + starcom::ccsds::kMacQueueCap> frame{};
  const auto vn = starcom::ccsds::encodeV3(frame, hdr, spdu);
  if (!vn) {
    return vn;
  }
  return starcom::ccsds::encodePltu(
      out, std::span<const std::byte>(frame.data(), *vn));
}

std::uint8_t catalog_from_pl(starcom::ccsds::MacPlExt const& pl) noexcept {
  return static_cast<std::uint8_t>((pl.mode_select & 0x03U) |
                                   ((pl.scrambler & 0x03U) << 2));
}

starcom::ccsds::MacPlExt pl_from_catalog(std::uint8_t idx) noexcept {
  starcom::ccsds::MacPlExt pl{};
  pl.mode_select = static_cast<std::uint8_t>(idx & 0x03U);
  pl.scrambler = static_cast<std::uint8_t>((idx >> 2) & 0x03U);
  return pl;
}

void load_pending_from_idx(BytePump& p, std::uint8_t idx) noexcept {
  p.pending_catalog_idx = idx;
  p.pending_catalog_valid = true;
  starcom::ccsds::macLoadPendingCommValue(p.mac,
                                          pump_comm_value_for_catalog(idx));
}

// R-37 logs. Keep the event if the ring has space, else count it as lost.
void push_event(BytePump& p, PumpEvent const& e) noexcept {
  if (p.ev_len >= kPumpEventCap) {
    ++p.ev_lost;
    return;
  }
  const auto slot = static_cast<std::uint8_t>((p.ev_head + p.ev_len) %
                                              kPumpEventCap);
  p.ev[slot] = e;
  ++p.ev_len;
}

void push_turn(BytePump& p, TurnEvent turn) noexcept {
  PumpEvent e{};
  e.t = p.mac.last_now;
  e.turn = turn;
  e.state = p.mac.state;
  push_event(p, e);
}

// R-37 logs. Record entry to S50 (send) or S60, S61, S62 (receive).
void note_state(BytePump& p) noexcept {
  const auto st = p.mac.state;
  if (st == p.ev_state) {
    return;
  }
  p.ev_state = st;
  switch (st) {
    case starcom::ccsds::MacState::s50:
      push_turn(p, TurnEvent::enter_s50);
      break;
    case starcom::ccsds::MacState::s60:
      push_turn(p, TurnEvent::enter_s60);
      break;
    case starcom::ccsds::MacState::s61:
      push_turn(p, TurnEvent::enter_s61);
      break;
    case starcom::ccsds::MacState::s62:
      push_turn(p, TurnEvent::enter_s62);
      break;
    default:
      break;
  }
}

// Where one SPDU sits in its P-frame, for the drop log.
struct SpduPos {
  std::uint8_t idx = kNoCatalogIdx;  // catalog idx from the frame SET PL
  std::uint8_t pos = 0;              // 1-based
  std::uint8_t n = 0;
  std::uint8_t type = 0;
};

void note_comm_change_drop(BytePump& p, CommChangeDrop why,
                           SpduPos const& at) noexcept {
  p.cc_drop_new = true;
  p.cc_drop_state = p.mac.state;
  p.cc_drop_why = why;
  ++p.cc_drop_n;
  PumpEvent e{};
  e.t = p.mac.last_now;
  e.state = p.mac.state;
  e.why = why;
  e.idx = at.idx;
  e.cmd_pos = at.pos;
  e.cmd_n = at.n;
  e.cmd_type = at.type;
  e.drop_n = p.cc_drop_n;
  push_event(p, e);
}

void on_comm_change_spdu(BytePump& p, starcom::ccsds::Tick now,
                         bool have_pl, starcom::ccsds::MacPlExt const* pl,
                         SpduPos const& at) noexcept {
  // 211.0 6.2.3.2: Y is set on the transmit side and reset on the receive
  // side. Y=3 (table 6-11 E67) means this side sent a COMM_CHANGE and waits
  // for the response. Tell the echo from a new request by Y, not by the MAC
  // state (R-37). Only the echo with the catalog this side asked for
  // confirms the hop (project echo, CONFORMANCE.md "COMM_CHANGE echo").
  if (p.mac.y == 3) {
    if (p.local_comm_change && have_pl && pl != nullptr &&
        p.pending_catalog_valid &&
        catalog_from_pl(*pl) == p.pending_catalog_idx) {
      p.peer_comm_change = true;
    }
    return;
  }
  // This side has a hop of its own that it did not send yet (Y=1 or Y=2).
  // Do not take a peer request now; the peer hop ends at its lifetime.
  if (p.local_comm_change) {
    note_comm_change_drop(p, CommChangeDrop::own_hop, at);
    return;
  }
  const auto st = p.mac.state;
  if (have_pl && pl != nullptr &&
      (st == starcom::ccsds::MacState::s60 ||
       st == starcom::ccsds::MacState::s61)) {
    load_pending_from_idx(p, catalog_from_pl(*pl));
  }
  if (st == starcom::ccsds::MacState::s2) {
    starcom::ccsds::macOnHailReceived(p.mac, now);
  } else if (st == starcom::ccsds::MacState::s60 ||
             st == starcom::ccsds::MacState::s61) {
    // Same catalog as the session hail is station reconnect (E81/E84),
    // not E69 COMM_CHANGE. macOnHailReceived moves the responder to S2
    // (project step, not E85/E82: no CARRIER_LOSS_TIMER expiry), then E30.
    if (p.pending_catalog_valid &&
        p.pending_catalog_idx == p.hail_catalog_idx) {
      starcom::ccsds::macOnHailReceived(p.mac, now);
      return;
    }
    starcom::ccsds::macOnRemoteCommChange(p.mac, now);
    if (p.mac.state == starcom::ccsds::MacState::s51) {
      p.turn_cause = TurnCause::remote_comm_change;
    }
    p.remote_apply_now = p.pending_catalog_valid;
  } else if (st == starcom::ccsds::MacState::s50 ||
             st == starcom::ccsds::MacState::s51 ||
             st == starcom::ccsds::MacState::s52) {
    if (p.pending_catalog_valid &&
        p.pending_catalog_idx == p.hail_catalog_idx) {
      starcom::ccsds::macOnHailReceived(p.mac, now);
    } else {
      // This side holds the send turn and does not listen. Table 6-11
      // E69 starts only in S60 or S61, so the request is lost.
      note_comm_change_drop(p, CommChangeDrop::not_s60_61, at);
    }
  } else {
    /* no action: MAC state is not S2, S50, S51, S52, S60 or S61. A frame
       in S62 ran E47 (S62 to S61) before dispatch. */
  }
}

bool p_frame_is_mac_spdu(std::span<const std::byte> data) noexcept {
  if (data.empty()) {
    return false;
  }
  const unsigned hi = std::to_integer<unsigned>(data[0]);
  const auto spdu_t = starcom::ccsds::spduDirectiveType(data);
  // 211.0 Fig 3-5 Format ID 1 (octet0 bit7) + report_value in octet[1]
  // is PLCW. V(R)=0 looks like SET TX (type 0); V(R)=1 looks like SET
  // CONTROL (type 001). Annex B Type-1 SET PL/RX also use bit7 as a
  // field (mode_select / modulation) — those stay MAC.
  const bool format1 = (hi & 0x80U) != 0U;
  // 211.0 Fig 3-5 PLCW: V(R) is octet[1], so V(R)&7 collides with Type-1
  // 0/1/2 (SET TX / CONTROL / RX). SET PL (type 6) stays MAC.
  if (format1 &&
      (spdu_t == starcom::ccsds::kSetTxDirectiveType ||
       spdu_t == starcom::ccsds::kSetControlDirectiveType ||
       spdu_t == starcom::ccsds::kSetRxDirectiveType)) {
    return false;
  }
  return spdu_t == starcom::ccsds::kSetPlExtDirectiveType ||
         spdu_t == starcom::ccsds::kSetControlDirectiveType ||
         spdu_t == starcom::ccsds::kSetRxDirectiveType ||
         spdu_t == starcom::ccsds::kSetTxDirectiveType;
}

// The catalog idx of the first SET PL EXTENSIONS SPDU in the frame.
std::uint8_t frame_catalog_idx(std::span<const std::byte> data) noexcept {
  for (std::size_t i = 0; i + 2 <= data.size(); i += 2) {
    const auto chunk = data.subspan(i, 2);
    if (starcom::ccsds::spduDirectiveType(chunk) ==
        starcom::ccsds::kSetPlExtDirectiveType) {
      const auto pl = starcom::ccsds::decodeSetPlExt(chunk);
      if (pl) {
        return catalog_from_pl(*pl);
      }
    }
  }
  return kNoCatalogIdx;
}

void dispatch_set_control(BytePump& p, std::span<const std::byte> chunk,
                          starcom::ccsds::Tick now, bool& consumed) noexcept {
  const auto ctl = starcom::ccsds::decodeSetControl(chunk);
  if (!ctl) {
    return;
  }
  consumed = true;
  if (ctl->pass) {
    // Table 6-10 E47 before E49. Other directives still see S62.
    starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
    starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
    const auto before = p.mac.state;
    starcom::ccsds::macOnToken(p.mac, now);
    if (before != p.mac.state &&
        p.mac.state == starcom::ccsds::MacState::s51) {
      p.turn_cause = TurnCause::token;  // E49
      push_turn(p, TurnEvent::e49);
    }
  }
  if (ctl->rnmd) {
    starcom::ccsds::macOnRnmd(p.mac, now);
  }
}

bool dispatch_p_frame_spdu(BytePump& p, std::span<const std::byte> data,
                           starcom::ccsds::Tick now) noexcept {
  bool consumed = false;
  SpduPos at{};
  at.idx = frame_catalog_idx(data);
  at.n = static_cast<std::uint8_t>(data.size() / 2U);
  for (std::size_t i = 0; i + 2 <= data.size(); i += 2) {
    const auto chunk = data.subspan(i, 2);
    const auto type = starcom::ccsds::spduDirectiveType(chunk);
    at.pos = static_cast<std::uint8_t>((i / 2U) + 1U);
    at.type = type;
    if (type == starcom::ccsds::kSetPlExtDirectiveType) {
      const auto pl = starcom::ccsds::decodeSetPlExt(chunk);
      if (pl) {
        consumed = true;
        on_comm_change_spdu(p, now, true, &*pl, at);
      }
    } else if (type == starcom::ccsds::kSetTxDirectiveType ||
               type == starcom::ccsds::kSetRxDirectiveType) {
      bool is_tx = false;
      if (starcom::ccsds::decodeSetPhy(chunk, &is_tx)) {
        consumed = true;
        on_comm_change_spdu(p, now, false, nullptr, at);
      }
    } else if (type == starcom::ccsds::kSetControlDirectiveType) {
      dispatch_set_control(p, chunk, now, consumed);
    } else {
      /* no action: directive type is not SET PL, SET TX, SET RX, or SET CONTROL */
    }
  }
  return consumed;
}

}  // namespace

void pump_init(BytePump& p, starcom::ccsds::Scid local,
               starcom::ccsds::Scid remote) noexcept {
  p.local_scid = local;
  p.remote_scid = remote;
  p.pending_catalog_valid = false;
  p.remote_apply_now = false;
  p.peer_comm_change = false;
  p.local_comm_change = false;
  p.air_heard = false;
  p.last_air_tick = 0;
  p.defer_spdu_fifo_empty = false;
  p.spdu_on_air = false;
  p.turn_cause = TurnCause::none;
  p.cc_sent_new = false;
  p.cc_drop_new = false;
  starcom::ccsds::CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 0;
  // coppInit clears the endpoint. A radio setting change calls pump_init again
  // (boot_starcom_session), so keep the link counts since boot, as before.
  const std::uint32_t retransmit_n = p.copp.retransmit_n;
  const std::uint32_t sync_loss_n = p.copp.sync_loss_n;
  starcom::ccsds::coppInit(p.copp, mib, kSoakPcid, local, remote, kSoakPort);
  p.copp.retransmit_n = retransmit_n;
  p.copp.sync_loss_n = sync_loss_n;
  starcom::ccsds::macInit(p.mac, flight_mac_mib(local),
                          starcom::ccsds::MacDuplex::half, &p.copp);
  const auto& hail = rc::kDefaultRocketRadioConfig;
  p.hail_catalog_idx = rc::radio_config_catalog_index(
      hail.bandwidth_khz, hail.nav_rate_hz, hail.spreading_factor,
      hail.coding_rate, hail.power_dbm);
  if (p.hail_catalog_idx == rc::kRadioConfigNoIndex) {
    p.hail_catalog_idx = 2;  // kRadioConfigTable 250/10
  }
  starcom::ccsds::macLoadHailCommValue(
      p.mac, pump_comm_value_for_catalog(p.hail_catalog_idx));
  p.ev_state = p.mac.state;
}

void pump_init_for_this_job(BytePump& p) noexcept {
  // Station local SCID is 2. Vehicle and relay local SCID is 1.
  // job::kRole is the compile-time role from job.h.
  if constexpr (job::kRole == job::DeviceRole::kStation) {
    pump_init(p, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
  } else {
    pump_init(p, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
  }
  // Firmware waits for PHY TxDone before the FIFO-empty event.
  // The host test build clears the FIFO at post.
  if constexpr (!rc::kHostTest) {
    p.defer_spdu_fifo_empty = true;
  }
}

starcom::ccsds::Result<std::size_t> pump_encode_pltu(
    std::span<std::byte> out, std::span<const std::byte> frame) noexcept {
  return starcom::ccsds::encodePltu(out, frame);
}

starcom::ccsds::Result<std::size_t> pump_repeat_pltu(
    std::span<std::byte> out, std::span<const std::byte> octets) noexcept {
  return starcom::ccsds::repeatPltu(out, octets);
}

namespace {

starcom::ccsds::Result<std::size_t> pack_user_packet(
    std::span<std::byte> out, starcom::ccsds::SpacePacketFields const& sp,
    std::span<const std::byte> user) noexcept {
  return starcom::ccsds::encodeSpacePacket(out, sp, user);
}

void copy_user(std::span<std::byte> dst, const uint8_t* src, std::size_t n) noexcept {
  for (std::size_t i = 0; i < n; ++i) {
    dst[i] = std::byte{src[i]};
  }
}

SpaceSeqLane* seq_lane(BytePump& p, starcom::ccsds::Apid apid) noexcept {
  if (apid == kNavApid) {
    return &p.seq_nav;
  }
  if (apid == kCmdApid) {
    return &p.seq_cmd;
  }
  return nullptr;
}

std::uint16_t peek_tx_seq(BytePump& p, starcom::ccsds::Apid apid) noexcept {
  SpaceSeqLane* lane = seq_lane(p, apid);
  if (lane == nullptr) {
    return 0;
  }
  return static_cast<std::uint16_t>(lane->tx & kSpaceSeqMask);
}

void commit_tx_seq(BytePump& p, starcom::ccsds::Apid apid) noexcept {
  SpaceSeqLane* lane = seq_lane(p, apid);
  if (lane == nullptr) {
    return;
  }
  lane->tx = static_cast<std::uint16_t>(
      (static_cast<unsigned>(lane->tx) + 1U) & kSpaceSeqMask);
}

}  // namespace

void pump_note_rx_seq(BytePump& p, starcom::ccsds::Apid apid,
                      std::uint16_t seq_count) noexcept {
  SpaceSeqLane* lane = seq_lane(p, apid);
  if (lane == nullptr) {
    return;
  }
  const std::uint16_t got =
      static_cast<std::uint16_t>(seq_count & kSpaceSeqMask);
  const std::uint16_t gap = static_cast<std::uint16_t>(
      (static_cast<unsigned>(got) - lane->rx_expect) & kSpaceSeqMask);
  if (gap != 0U) {
    lane->rx_lost += gap;
  }
  lane->rx_expect = static_cast<std::uint16_t>(
      (static_cast<unsigned>(got) + 1U) & kSpaceSeqMask);
}

starcom::ccsds::Result<std::size_t> pump_pack_nav_packet(
    BytePump& p, std::span<std::byte> out, const TelemetryState& telem) noexcept {
  uint8_t user[kNavSduUserBytes] = {};
  if (pack_nav_sdu_user(user, sizeof(user), telem) != kNavSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kNavSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.apid = kNavApid;
  sp.seq_count = peek_tx_seq(p, kNavApid);
  const auto n = pack_user_packet(out, sp, user_b);
  if (n.has_value()) {
    commit_tx_seq(p, kNavApid);
  }
  return n;
}

starcom::ccsds::Result<std::size_t> pump_pack_cmd_packet(
    BytePump& p, std::span<std::byte> out, const CmdSduFields& f) noexcept {
  uint8_t user[kCmdSduUserBytes] = {};
  if (pack_cmd_sdu_user(user, sizeof(user), f) != kCmdSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kCmdSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.telecommand = true;
  sp.apid = kCmdApid;
  sp.seq_count = peek_tx_seq(p, kCmdApid);
  const auto n = pack_user_packet(out, sp, user_b);
  if (n.has_value()) {
    commit_tx_seq(p, kCmdApid);
  }
  return n;
}

starcom::ccsds::Result<std::size_t> pump_pack_ack_packet(
    BytePump& p, std::span<std::byte> out,
    const ccsds::CommandAckPayload& ack) noexcept {
  uint8_t user[kAckSduUserBytes] = {};
  if (pack_ack_sdu_user(user, sizeof(user), ack) != kAckSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kAckSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.apid = kCmdApid;
  sp.seq_count = peek_tx_seq(p, kCmdApid);
  const auto n = pack_user_packet(out, sp, user_b);
  if (n.has_value()) {
    commit_tx_seq(p, kCmdApid);
  }
  return n;
}

starcom::ccsds::Result<std::size_t> pump_encode_nav(
    BytePump& p, std::span<std::byte> out,
    const TelemetryState& telem) noexcept {
  std::array<std::byte, 6U + kNavSduUserBytes> packet{};
  const auto pn = pump_pack_nav_packet(p, packet, telem);
  if (!pn) {
    return pn;
  }

  starcom::ccsds::V3Fields v3{};
  v3.scid = p.local_scid;
  std::array<std::byte, 5U + 6U + kNavSduUserBytes> frame{};
  const auto fn = starcom::ccsds::encodeV3(
      frame, v3, std::span<const std::byte>(packet.data(), *pn));
  if (!fn) {
    return fn;
  }
  return starcom::ccsds::encodePltu(
      out, std::span<const std::byte>(frame.data(), *fn));
}

starcom::ccsds::Result<std::size_t> pump_submit_sdu(
    BytePump& p, std::span<const std::byte> packet, bool expedited) noexcept {
  return starcom::ccsds::coppSubmitSdu(p.copp, packet, expedited);
}

starcom::ccsds::Result<std::size_t> pump_bytes_to_send(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::coppBytesToSend(p.copp, out);
}

void pump_receive_bytes(BytePump& p, std::span<const std::byte> octets) noexcept {
  starcom::ccsds::coppReceiveBytes(p.copp, octets);
}

namespace {

// This side sent a COMM_CHANGE and waits for the response (211.0 table 6-11
// E67: Y=3 in S62, or S61 after E47).
bool hop_wait(BytePump const& p) noexcept {
  return p.local_comm_change && p.mac.y == 3 &&
         (p.mac.state == starcom::ccsds::MacState::s61 ||
          p.mac.state == starcom::ccsds::MacState::s62);
}

void handle_air_frame(BytePump& p, std::span<const std::byte> octets) noexcept {
  const auto pltu = starcom::ccsds::decodePltu(octets);
  if (!pltu) {
    starcom::ccsds::coppReceiveBytes(p.copp, octets);
    return;
  }
  const auto v3 = starcom::ccsds::decodeV3(pltu->frame);
  if (!v3) {
    starcom::ccsds::coppReceiveBytes(p.copp, octets);
    return;
  }
  const auto now = p.mac.last_now;
  p.air_heard = true;
  p.last_air_tick = now;
  // R-37 adapter gate (CONFORMANCE.md "COMM_CHANGE echo"). The SX1276 has
  // one modem setting for TX and RX, so the caller cannot listen on the new
  // settings before E63/E64 as 211.0 table 6-11 expects. A frame on the old
  // settings then proves nothing. While this side waits for the response,
  // only the peer echo may run E68 (211.0 6.2.4.8). Any other frame (nav,
  // PLCW, token, hail) does not.
  const bool waiting = hop_wait(p);
  if (v3->fields.p_frame && !v3->data.empty()) {
    // 211.0 table 6-10 E30: hail is SET TX/RX/PL chunks, not a whole-P-frame type.
    const bool s2_hail = (p.mac.state == starcom::ccsds::MacState::s2);
    if (s2_hail || p_frame_is_mac_spdu(v3->data)) {
      // R-37: a received frame means carrier. 211.0 table 6-10 E47 (S62 to
      // S61) runs before the SPDUs, so a COMM_CHANGE request meets E69 in
      // S61 (table 6-11), the same as the token path (E47 before E49).
      starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
      starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
      if (dispatch_p_frame_spdu(p, v3->data, now)) {
        // Reconnect hail (E30 S51) must not run macOnValidFrame (that reset
        // token_fail and kept active).
        if ((p.mac.state == starcom::ccsds::MacState::s60 ||
             p.mac.state == starcom::ccsds::MacState::s61) &&
            (!waiting || p.peer_comm_change)) {
          starcom::ccsds::macOnValidFrame(p.mac, now);
        }
        return;
      }
    }
  }
  if (!waiting) {
    starcom::ccsds::macOnValidFrame(p.mac, now);
  }
  starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
  starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
  starcom::ccsds::coppReceiveBytes(p.copp, octets);
}

}  // namespace

void pump_handle_air(BytePump& p, std::span<const std::byte> octets) noexcept {
  handle_air_frame(p, octets);
  note_state(p);
}

void pump_cancel_comm_change(BytePump& p) noexcept {
  p.local_comm_change = false;
  p.peer_comm_change = false;
  p.remote_apply_now = false;
  p.pending_catalog_valid = false;
  // Project choice: 211.0 4.2.2.5.4 aborts the activity but does not say
  // which settings the caller keeps. Keep the session hail value.
  starcom::ccsds::macLoadHailCommValue(
      p.mac, pump_comm_value_for_catalog(p.hail_catalog_idx));
}

starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::coppTakeSdu(p.copp, out);
}

// Table 6-10 timer events of one MAC tick, for turn_cause and the R-37 log.
static void note_tick_turn(BytePump& p, starcom::ccsds::MacState before,
                           starcom::ccsds::MacNotify notify_before) noexcept {
  using starcom::ccsds::MacState;
  const auto st = p.mac.state;
  if (st == MacState::s51 && before == MacState::s60) {
    p.turn_cause = TurnCause::timer_e48;
    push_turn(p, TurnEvent::e48);
  } else if (st == MacState::s51 &&
             (before == MacState::s61 || before == MacState::s62)) {
    p.turn_cause = TurnCause::timer_e50;
    push_turn(p, TurnEvent::e50);
  } else if (st == before && p.mac.notify != notify_before) {
    // E44 and E45 reload Receive_Duration and keep the state.
    if (st == MacState::s60 &&
        p.mac.notify == starcom::ccsds::MacNotify::sender_overran) {
      push_turn(p, TurnEvent::e44);
    } else if (st == MacState::s61 &&
               p.mac.notify ==
                   starcom::ccsds::MacNotify::no_data_this_contact) {
      push_turn(p, TurnEvent::e45);
    } else {
      /* no action: another notify, not a turn event */
    }
  } else {
    /* no action: no timer turn event this tick */
  }
  note_state(p);
}

void pump_tick(BytePump& p, starcom::ccsds::Tick now) noexcept {
  // 211.0 6.5.2: CARRIER_ACQUIRED follows the PHY, not a hold timer.
  if (p.mac.carrier_acquired) {
    const starcom::ccsds::Tick gap =
        (now > p.last_air_tick) ? (now - p.last_air_tick)
                                : starcom::ccsds::Tick{0};
    const starcom::ccsds::Tick hold =
        (p.mac.mib.tail_idle_duration != 0)
            ? p.mac.mib.tail_idle_duration
            : starcom::ccsds::Tick{1};
    if (gap >= hold) {
      starcom::ccsds::macSetCarrierAcquired(p.mac, false, now);
      p.air_heard = false;
    }
  }
  const auto before = p.mac.state;
  const auto notify_before = p.mac.notify;
  starcom::ccsds::macTick(p.mac, now);
  note_tick_turn(p, before, notify_before);
  // 211.0 table 6-10 E39: NEED_PLCW clear and an empty queue => token.
  if (p.mac.mode == starcom::ccsds::MacMode::active && p.mac.persistence &&
      !p.mac.mac_frame_pending) {
    starcom::ccsds::macOnNoFramesPending(p.mac, now);
  }
  note_state(p);
}

void pump_start_session(BytePump& p, bool caller,
                        starcom::ccsds::Tick now) noexcept {
  // A new session ends any hop in flight (R-37).
  p.local_comm_change = false;
  p.peer_comm_change = false;
  p.remote_apply_now = false;
  starcom::ccsds::macSetMode(p.mac, starcom::ccsds::MacMode::inactive, now);
  starcom::ccsds::coppReset(p.copp);
  starcom::ccsds::macSetMode(
      p.mac,
      caller ? starcom::ccsds::MacMode::connecting_t
             : starcom::ccsds::MacMode::connecting_l,
      now);
  note_state(p);
}

static bool pump_copp_blocked(BytePump& p,
                              starcom::ccsds::MacPhy const& phy,
                              starcom::ccsds::MacFifoSource src) noexcept {
  if (p.mac.mode == starcom::ccsds::MacMode::connecting_l ||
      p.mac.mode == starcom::ccsds::MacMode::connecting_t) {
    return true;
  }
  if (p.mac.mode == starcom::ccsds::MacMode::active && !phy.transmit) {
    return true;
  }
  // Table 6-10 E38 stops user data. An owed PLCW or queued SPDU still leaves.
  if (src == starcom::ccsds::MacFifoSource::plcw ||
      src == starcom::ccsds::MacFifoSource::spdu) {
    return false;
  }
  const bool active = p.mac.mode == starcom::ccsds::MacMode::active;
  if (active && (p.mac.persistence ||
                 src == starcom::ccsds::MacFifoSource::none ||
                 src == starcom::ccsds::MacFifoSource::idle ||
                 src == starcom::ccsds::MacFifoSource::carrier_only)) {
    if (p.mac.persistence && !p.mac.mac_frame_pending) {
      starcom::ccsds::macOnNoFramesPending(p.mac, p.mac.last_now);
    }
    return true;
  }
  return false;
}

static starcom::ccsds::Result<std::size_t> air_to_send(
    BytePump& p, std::span<std::byte> out) noexcept {
  const bool sdu_pending =
      p.copp.exp_full || p.copp.seq_n != 0;
  starcom::ccsds::macSetSduPending(p.mac, sdu_pending);
  const auto phy = starcom::ccsds::macPhy(p.mac);
  const auto src = starcom::ccsds::macFifoSource(p.mac);
  if (src == starcom::ccsds::MacFifoSource::spdu) {
    if (p.spdu_on_air) {
      return std::size_t{0};
    }
    std::array<std::byte, starcom::ccsds::kMacQueueCap> spdu{};
    const auto n = starcom::ccsds::macCopySpdu(p.mac, spdu);
    if (!n || *n == 0) {
      return std::size_t{0};
    }
    const auto pltu = wrap_mac_p_frame(
        p, out, std::span<const std::byte>(spdu.data(), *n));
    if (pltu && *pltu > 0) {
      // Table 6-11 E65/E66: Y=2 means this P-frame is the COMM_CHANGE.
      if (p.mac.y == 2) {
        p.cc_sent_new = true;
        p.cc_sent_state = p.mac.state;
        p.cc_sent_turn = p.turn_cause;
        p.cc_sent_n = p.mac.comm_change_sends;
      }
      p.spdu_on_air = true;
      if (!p.defer_spdu_fifo_empty) {
        p.spdu_on_air = false;
        starcom::ccsds::macOnFifoEmpty(p.mac, p.mac.last_now);
      }
    }
    return pltu;
  }
  // 211.0 6.5.1 / table 6-14: COP-P only in an active send contact.
  if (pump_copp_blocked(p, phy, src)) {
    return std::size_t{0};
  }
  const bool seq_busy =
      p.copp.seq_n != 0 ||
      starcom::ccsds::seqLt(p.copp.fop.nn_r, p.copp.fop.v_s);
  const bool fop_busy = sdu_pending || seq_busy;
  // 211.0 table 6-14 / 7.2.2 h: NEED_PLCW is independent of FOP seq.
  if (p.mac.need_plcw) {
    p.copp.farm.need_plcw = true;
  }
  if (src == starcom::ccsds::MacFifoSource::plcw ||
      src == starcom::ccsds::MacFifoSource::sdu ||
      sdu_pending || p.copp.farm.need_plcw || fop_busy) {
    const bool want_plcw = p.copp.farm.need_plcw;
    const auto n = starcom::ccsds::coppBytesToSend(p.copp, out);
    if (want_plcw && !p.copp.farm.need_plcw) {
      p.mac.need_plcw = false;
    }
    return n;
  }
  return std::size_t{0};
}

starcom::ccsds::Result<std::size_t> pump_air_to_send(
    BytePump& p, std::span<std::byte> out) noexcept {
  const auto n = air_to_send(p, out);
  note_state(p);
  return n;
}

void pump_spdu_air_complete(BytePump& p,
                            starcom::ccsds::Tick now) noexcept {
  if (!p.spdu_on_air) {
    return;
  }
  p.spdu_on_air = false;
  starcom::ccsds::macOnFifoEmpty(p.mac, now);
  note_state(p);
}

starcom::ccsds::MacPhy pump_mac_phy(BytePump const& p) noexcept {
  return starcom::ccsds::macPhy(p.mac);
}

starcom::ccsds::MacFifoSource pump_fifo_source(BytePump const& p) noexcept {
  return starcom::ccsds::macFifoSource(p.mac);
}

starcom::ccsds::MacNotify pump_poll_mac_notify(BytePump& p) noexcept {
  return starcom::ccsds::macPollNotify(p.mac);
}

starcom::ccsds::MacCommValue pump_comm_value_for_catalog(
    std::uint8_t idx) noexcept {
  starcom::ccsds::MacCommValue cv{};
  cv.tx.encoding = starcom::ccsds::kPhyEncodingBypass;
  cv.rx.encoding = starcom::ccsds::kPhyEncodingBypass;
  cv.pl_tx = pl_from_catalog(idx);
  cv.pl_rx = pl_from_catalog(idx);
  cv.has_pl_tx = true;
  cv.has_pl_rx = true;
  return cv;
}

bool pump_catalog_fits(std::uint8_t idx) noexcept {
  if (idx >= rc::kRadioConfigTableSize) {
    return false;
  }
  const auto& e = rc::kRadioConfigTable[idx];
  return rc::radio_config_nav_fits_hz(e.bw_khz, e.nav_rate_hz, e.sf,
                                      rc::kRadioConfigNavPltuBytes);
}

bool pump_begin_comm_change(BytePump& p, std::uint8_t idx,
                            starcom::ccsds::Tick now) noexcept {
  if (!pump_catalog_fits(idx)) {
    return false;
  }
  // R-37: E39 already queued the token in S56. E42 needs Y=0 to leave S56
  // (211.0 table 6-10), so E62 (Y=1) here would hold this side in S56.
  if (p.mac.state == starcom::ccsds::MacState::s56 && p.mac.y == 0) {
    return false;
  }
  load_pending_from_idx(p, idx);
  p.local_comm_change = true;
  p.remote_apply_now = false;
  p.peer_comm_change = false;
  starcom::ccsds::macLocalCommChange(p.mac, now);
  // Table 6-11 E65: NoFramesPending in S50 Y=2 queues COMM_CHANGE this contact.
  starcom::ccsds::macOnNoFramesPending(p.mac, now);
  note_state(p);
  return true;
}

std::uint8_t pump_catalog_from_pl(
    starcom::ccsds::MacPlExt const& pl) noexcept {
  return catalog_from_pl(pl);
}

bool pump_take_event(BytePump& p, PumpEvent& out) noexcept {
  if (p.ev_len == 0U) {
    return false;
  }
  out = p.ev[p.ev_head];
  p.ev_head = static_cast<std::uint8_t>((p.ev_head + 1U) % kPumpEventCap);
  --p.ev_len;
  return true;
}

}  // namespace rc::starcom_adapt
