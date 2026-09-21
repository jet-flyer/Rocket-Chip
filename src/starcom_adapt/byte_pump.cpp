// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/byte_pump.h"

#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#include "flight_director/mission_profile_data.h"
#include "rocketchip/radio_config_table.h"
#include "starcom/error.hpp"

#include <tl/expected.hpp>

#include <cstring>

namespace rc::starcom_adapt {

namespace {

starcom::ccsds::MacMib flight_mac_mib(starcom::ccsds::Scid local) noexcept {
  starcom::ccsds::MacMib m{};
  const uint8_t hz = rc::kDefaultRocketRadioConfig.nav_rate_hz;
  const starcom::ccsds::Tick nav_ms =
      (hz == 0U) ? 200U : static_cast<starcom::ccsds::Tick>(1000U / hz);
  // SX1276 §4.1.1.6 SF7 / 250 kHz: Tsym = 512 us; 8-symbol preamble
  // 12.25 Tsym = 6.272 ms. 10 ms is ceil-to-tick.
  m.carrier_only_duration = 10;
  m.acquisition_idle_duration = 10;
  m.tail_idle_duration = 10;
  const starcom::ccsds::Tick turn_ms =
      m.carrier_only_duration + m.acquisition_idle_duration +
      m.tail_idle_duration;
  const uint32_t toa_us = rc::radio_config_nav_airtime_us(
      rc::kDefaultRocketRadioConfig.spreading_factor,
      rc::kDefaultRocketRadioConfig.bandwidth_khz,
      rc::kRadioConfigNavPltuBytes);
  const starcom::ccsds::Tick toa_ms =
      static_cast<starcom::ccsds::Tick>((toa_us + 999U) / 1000U);
  // 211.0 6.2.4.17–18 / table 6-10: each side's Send_Duration is local.
  // N=11 (~90% packing). Table: starcom_adapt/README.md. Later user preset.
  const starcom::ccsds::Tick stn_send = toa_ms;
  const starcom::ccsds::Tick tax_ms = stn_send + (2U * turn_ms);
  const starcom::ccsds::Tick veh_send =
      ((9U * tax_ms) + (nav_ms - 1U)) / nav_ms * nav_ms;
  const bool station = (local == starcom::ccsds::Scid{2});
  if (station) {
    m.send_duration = stn_send;
    m.receive_duration = veh_send + turn_ms;
  } else {
    m.send_duration = veh_send;
    m.receive_duration = stn_send + turn_ms;
  }
  m.hail_wait_duration = nav_ms + 20U;
  m.hail_lifetime = 0;  // 211.0 6.2.4.14.2: 0 = no abort
  m.drop_carrier_duration = 20;
  m.maximum_failed_token_passes = 4;  // 211.0 table 6-12 E83; 0 = unlimited
  // Cover one peer data-services contact (not 8*nav from symmetric MIB).
  m.carrier_loss_timer_duration = veh_send + turn_ms;
  m.plcw_repeat_interval = veh_send;
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
  std::array<std::byte, 5u + starcom::ccsds::kMacQueueCap> frame{};
  const auto vn = starcom::ccsds::encodeV3(frame, hdr, spdu);
  if (!vn) {
    return vn;
  }
  return starcom::ccsds::encodePltu(
      out, std::span<const std::byte>(frame.data(), *vn));
}

std::uint8_t catalog_from_pl(starcom::ccsds::MacPlExt const& pl) noexcept {
  return static_cast<std::uint8_t>((pl.mode_select & 0x03u) |
                                   ((pl.scrambler & 0x03u) << 2));
}

starcom::ccsds::MacPlExt pl_from_catalog(std::uint8_t idx) noexcept {
  starcom::ccsds::MacPlExt pl{};
  pl.mode_select = static_cast<std::uint8_t>(idx & 0x03u);
  pl.scrambler = static_cast<std::uint8_t>((idx >> 2) & 0x03u);
  return pl;
}

void load_pending_from_idx(BytePump& p, std::uint8_t idx) noexcept {
  p.pending_catalog_idx = idx;
  p.pending_catalog_valid = true;
  starcom::ccsds::macLoadPendingCommValue(p.mac,
                                          pump_comm_value_for_catalog(idx));
}

void on_comm_change_spdu(BytePump& p, starcom::ccsds::Tick now,
                         bool have_pl,
                         starcom::ccsds::MacPlExt const* pl) noexcept {
  // This side started the hop. E68 moves S62→S60 before dispatch, so a
  // peer echo looks like E69. Confirm only — do not retune as remote.
  if (p.local_comm_change) {
    p.peer_comm_change = true;
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
    // not E69 COMM_CHANGE. Accept via E85/E82 → S2 → E30.
    if (p.pending_catalog_valid &&
        p.pending_catalog_idx == p.hail_catalog_idx) {
      starcom::ccsds::macOnHailReceived(p.mac, now);
      return;
    }
    starcom::ccsds::macOnRemoteCommChange(p.mac, now);
    p.remote_apply_now = p.pending_catalog_valid;
  } else if (st == starcom::ccsds::MacState::s62 ||
             st == starcom::ccsds::MacState::s50 ||
             st == starcom::ccsds::MacState::s51 ||
             st == starcom::ccsds::MacState::s52) {
    if (p.pending_catalog_valid &&
        p.pending_catalog_idx == p.hail_catalog_idx) {
      starcom::ccsds::macOnHailReceived(p.mac, now);
      return;
    }
    if (st == starcom::ccsds::MacState::s62) {
      p.peer_comm_change = true;
    }
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
  const bool format1 = (hi & 0x80u) != 0u;
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

bool dispatch_p_frame_spdu(BytePump& p, std::span<const std::byte> data,
                           starcom::ccsds::Tick now) noexcept {
  bool consumed = false;
  for (std::size_t i = 0; i + 2 <= data.size(); i += 2) {
    const auto chunk = data.subspan(i, 2);
    const auto type = starcom::ccsds::spduDirectiveType(chunk);
    if (type == starcom::ccsds::kSetPlExtDirectiveType) {
      const auto pl = starcom::ccsds::decodeSetPlExt(chunk);
      if (!pl) {
        continue;
      }
      consumed = true;
      on_comm_change_spdu(p, now, true, &*pl);
      continue;
    }
    if (type == starcom::ccsds::kSetTxDirectiveType ||
        type == starcom::ccsds::kSetRxDirectiveType) {
      bool is_tx = false;
      if (!starcom::ccsds::decodeSetPhy(chunk, &is_tx)) {
        continue;
      }
      consumed = true;
      on_comm_change_spdu(p, now, false, nullptr);
      continue;
    }
    if (type == starcom::ccsds::kSetControlDirectiveType) {
      const auto ctl = starcom::ccsds::decodeSetControl(chunk);
      if (!ctl) {
        continue;
      }
      consumed = true;
      if (ctl->pass) {
        starcom::ccsds::macOnToken(p.mac, now);
      }
      if (ctl->rnmd) {
        starcom::ccsds::macOnRnmd(p.mac, now);
      }
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
  starcom::ccsds::CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 0;
  starcom::ccsds::coppInit(p.copp, mib, kSoakPcid, local, remote, kSoakPort);
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
}

void pump_init_for_this_job(BytePump& p) noexcept {
#ifdef ROCKETCHIP_JOB_STATION
  pump_init(p, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
#else
  pump_init(p, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
#endif
#ifndef ROCKETCHIP_HOST_TEST
  p.defer_spdu_fifo_empty = true;
#endif
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

}  // namespace

starcom::ccsds::Result<std::size_t> pump_pack_nav_packet(
    std::span<std::byte> out, const TelemetryState& telem) noexcept {
  uint8_t user[kNavSduUserBytes] = {};
  if (pack_nav_sdu_user(user, sizeof(user), telem) != kNavSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kNavSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.apid = kNavApid;
  return pack_user_packet(out, sp, user_b);
}

// COMMAND_LONG is id + seq + 5 floats (JPL-25 ParameterThreshold is 6).
// NOLINTNEXTLINE(readability-function-size)
starcom::ccsds::Result<std::size_t> pump_pack_cmd_packet(
    std::span<std::byte> out, uint16_t cmd_id, uint8_t seq, float p1, float p2,
    float p3, float p4, float p5) noexcept {
  uint8_t user[kCmdSduUserBytes] = {};
  if (pack_cmd_sdu_user(user, sizeof(user), cmd_id, seq, p1, p2, p3, p4, p5) !=
      kCmdSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kCmdSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.telecommand = true;
  sp.apid = kCmdApid;
  return pack_user_packet(out, sp, user_b);
}

starcom::ccsds::Result<std::size_t> pump_pack_ack_packet(
    std::span<std::byte> out, const ccsds::CommandAckPayload& ack) noexcept {
  uint8_t user[kAckSduUserBytes] = {};
  if (pack_ack_sdu_user(user, sizeof(user), ack) != kAckSduUserBytes) {
    return starcom::ccsds::Result<std::size_t>{
        tl::unexpect, starcom::ccsds::Error::buffer_too_small};
  }
  std::array<std::byte, kAckSduUserBytes> user_b{};
  copy_user(user_b, user, user_b.size());
  starcom::ccsds::SpacePacketFields sp{};
  sp.apid = kCmdApid;
  return pack_user_packet(out, sp, user_b);
}

starcom::ccsds::Result<std::size_t> pump_encode_nav(
    BytePump& p, std::span<std::byte> out,
    const TelemetryState& telem) noexcept {
  std::array<std::byte, 6u + kNavSduUserBytes> packet{};
  const auto pn = pump_pack_nav_packet(packet, telem);
  if (!pn) {
    return pn;
  }

  starcom::ccsds::V3Fields v3{};
  v3.scid = p.local_scid;
  std::array<std::byte, 5u + 6u + kNavSduUserBytes> frame{};
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

void pump_handle_air(BytePump& p, std::span<const std::byte> octets) noexcept {
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
  const bool comm_wait = (p.mac.state == starcom::ccsds::MacState::s62 &&
                          p.mac.y == 3);
  p.air_heard = true;
  p.last_air_tick = now;
  if (v3->fields.p_frame && !v3->data.empty()) {
    // 211.0 table 6-10 E30: hail is SET TX/RX/PL chunks, not a whole-P-frame type.
    const bool s2_hail = (p.mac.state == starcom::ccsds::MacState::s2);
    if ((s2_hail || p_frame_is_mac_spdu(v3->data)) &&
        dispatch_p_frame_spdu(p, v3->data, now)) {
      starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
      starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
      if (comm_wait) {
        p.peer_comm_change = true;
      }
      // COMM_CHANGE echo still needs E43 S62→S60. Reconnect hail (E30 S51)
      // must not run macOnValidFrame (that reset token_fail and kept active).
      if (p.mac.state == starcom::ccsds::MacState::s60 ||
          p.mac.state == starcom::ccsds::MacState::s61 ||
          p.mac.state == starcom::ccsds::MacState::s62) {
        starcom::ccsds::macOnValidFrame(p.mac, now);
      }
      return;
    }
  }
  starcom::ccsds::macOnValidFrame(p.mac, now);
  starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
  starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
  starcom::ccsds::coppReceiveBytes(p.copp, octets);
}

starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::coppTakeSdu(p.copp, out);
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
  starcom::ccsds::macTick(p.mac, now);
  // 211.0 table 6-12 E38→E39: persistence + empty MAC queue => token.
  if (p.mac.mode == starcom::ccsds::MacMode::active && p.mac.persistence &&
      !p.mac.mac_frame_pending) {
    starcom::ccsds::macOnNoFramesPending(p.mac, now);
  }
}

void pump_start_session(BytePump& p, bool caller,
                        starcom::ccsds::Tick now) noexcept {
  starcom::ccsds::macSetMode(p.mac, starcom::ccsds::MacMode::inactive, now);
  starcom::ccsds::coppReset(p.copp);
  starcom::ccsds::macSetMode(
      p.mac,
      caller ? starcom::ccsds::MacMode::connecting_t
             : starcom::ccsds::MacMode::connecting_l,
      now);
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

starcom::ccsds::Result<std::size_t> pump_air_to_send(
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

void pump_spdu_air_complete(BytePump& p,
                            starcom::ccsds::Tick now) noexcept {
  if (!p.spdu_on_air) {
    return;
  }
  p.spdu_on_air = false;
  starcom::ccsds::macOnFifoEmpty(p.mac, now);
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
  load_pending_from_idx(p, idx);
  p.local_comm_change = true;
  p.remote_apply_now = false;
  p.peer_comm_change = false;
  starcom::ccsds::macLocalCommChange(p.mac, now);
  // Table 6-11 E65: NoFramesPending in S50 Y=2 queues COMM_CHANGE this contact.
  starcom::ccsds::macOnNoFramesPending(p.mac, now);
  return true;
}

std::uint8_t pump_catalog_from_pl(
    starcom::ccsds::MacPlExt const& pl) noexcept {
  return catalog_from_pl(pl);
}

}  // namespace rc::starcom_adapt
