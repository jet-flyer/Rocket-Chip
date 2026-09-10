// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/byte_pump.h"

#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#include "flight_director/mission_profile_data.h"
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
  m.send_duration = nav_ms;
  m.receive_duration = nav_ms;
  m.hail_wait_duration = nav_ms + 20U;
  m.hail_lifetime = static_cast<starcom::ccsds::Tick>(10U * m.hail_wait_duration);
  m.drop_carrier_duration = 20;
  m.carrier_loss_timer_duration = static_cast<starcom::ccsds::Tick>(2U * nav_ms);
  m.plcw_repeat_interval = nav_ms;
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

void dispatch_p_frame_spdu(BytePump& p, std::span<const std::byte> data,
                           starcom::ccsds::Tick now) noexcept {
  for (std::size_t i = 0; i + 2 <= data.size(); i += 2) {
    const auto chunk = data.subspan(i, 2);
    const auto type = starcom::ccsds::spduDirectiveType(chunk);
    if (type == starcom::ccsds::kSetTxDirectiveType ||
        type == starcom::ccsds::kSetRxDirectiveType ||
        type == starcom::ccsds::kSetPlExtDirectiveType) {
      if (p.mac.state == starcom::ccsds::MacState::s2) {
        starcom::ccsds::macOnHailReceived(p.mac, now);
      } else if (p.mac.state == starcom::ccsds::MacState::s60 ||
                 p.mac.state == starcom::ccsds::MacState::s61) {
        starcom::ccsds::macOnRemoteCommChange(p.mac, now);
      }
      continue;
    }
    if (type == starcom::ccsds::kSetControlDirectiveType) {
      const auto ctl = starcom::ccsds::decodeSetControl(chunk);
      if (ctl) {
        if (ctl->pass) {
          starcom::ccsds::macOnToken(p.mac, now);
        }
        if (ctl->rnmd) {
          starcom::ccsds::macOnRnmd(p.mac, now);
        }
      }
    }
  }
}

}  // namespace

void pump_init(BytePump& p, starcom::ccsds::Scid local,
               starcom::ccsds::Scid remote) noexcept {
  p.local_scid = local;
  p.remote_scid = remote;
  starcom::ccsds::CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 0;
  starcom::ccsds::coppInit(p.copp, mib, kSoakPcid, local, remote, kSoakPort);
  starcom::ccsds::macInit(p.mac, flight_mac_mib(local),
                          starcom::ccsds::MacDuplex::half, &p.copp);
}

void pump_init_for_this_job(BytePump& p) noexcept {
#ifdef ROCKETCHIP_JOB_STATION
  pump_init(p, starcom::ccsds::Scid{2}, starcom::ccsds::Scid{1});
#else
  pump_init(p, starcom::ccsds::Scid{1}, starcom::ccsds::Scid{2});
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
  starcom::ccsds::macOnValidFrame(p.mac, now);
  starcom::ccsds::macSetCarrierAcquired(p.mac, true, now);
  starcom::ccsds::macSetSymbolInlock(p.mac, true, now);
  if (v3->fields.p_frame && !v3->data.empty()) {
    const auto type = starcom::ccsds::spduDirectiveType(v3->data);
    if (type == starcom::ccsds::kSetTxDirectiveType ||
        type == starcom::ccsds::kSetRxDirectiveType ||
        type == starcom::ccsds::kSetControlDirectiveType ||
        type == starcom::ccsds::kSetPlExtDirectiveType) {
      dispatch_p_frame_spdu(p, v3->data, now);
      return;
    }
  }
  starcom::ccsds::coppReceiveBytes(p.copp, octets);
}

starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::coppTakeSdu(p.copp, out);
}

void pump_tick(BytePump& p, starcom::ccsds::Tick now) noexcept {
  starcom::ccsds::macTick(p.mac, now);
}

void pump_start_session(BytePump& p, bool caller,
                        starcom::ccsds::Tick now) noexcept {
  starcom::ccsds::macSetMode(
      p.mac,
      caller ? starcom::ccsds::MacMode::connecting_t
             : starcom::ccsds::MacMode::connecting_l,
      now);
}

starcom::ccsds::Result<std::size_t> pump_air_to_send(
    BytePump& p, std::span<std::byte> out) noexcept {
  const bool sdu_pending =
      p.copp.exp_full || p.copp.seq_n != 0;
  starcom::ccsds::macSetSduPending(p.mac, sdu_pending);
  const auto src = starcom::ccsds::macFifoSource(p.mac);
  if (src == starcom::ccsds::MacFifoSource::none ||
      src == starcom::ccsds::MacFifoSource::carrier_only ||
      src == starcom::ccsds::MacFifoSource::idle) {
    return std::size_t{0};
  }
  if (src == starcom::ccsds::MacFifoSource::spdu) {
    std::array<std::byte, starcom::ccsds::kMacQueueCap> spdu{};
    const auto n = starcom::ccsds::macCopySpdu(p.mac, spdu);
    if (!n || *n == 0) {
      return std::size_t{0};
    }
    const auto pltu = wrap_mac_p_frame(
        p, out, std::span<const std::byte>(spdu.data(), *n));
    if (pltu && *pltu > 0) {
      starcom::ccsds::macOnFifoEmpty(p.mac, p.mac.last_now);
    }
    return pltu;
  }
  return starcom::ccsds::coppBytesToSend(p.copp, out);
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

}  // namespace rc::starcom_adapt
