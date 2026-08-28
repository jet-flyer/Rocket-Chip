// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include "starcom_adapt/byte_pump.h"

#ifdef ROCKETCHIP_USE_STARCOM

#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/cmd_sdu.h"
#include "starcom/error.hpp"

#include <tl/expected.hpp>

#include <cstring>

namespace rc::starcom_adapt {

void pump_init(BytePump& p, starcom::ccsds::Scid local,
               starcom::ccsds::Scid remote) noexcept {
  p.local_scid = local;
  p.remote_scid = remote;
  starcom::ccsds::CoppMib mib{};
  mib.transmission_window = 4;
  mib.synch_timeout = 0;
  starcom::ccsds::copp_init(p.copp, mib, kSoakPcid, local, remote, kSoakPort);
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
  return starcom::ccsds::encode_pltu(out, frame);
}

starcom::ccsds::Result<std::size_t> pump_repeat_pltu(
    std::span<std::byte> out, std::span<const std::byte> octets) noexcept {
  return starcom::ccsds::repeat_pltu(out, octets);
}

namespace {

starcom::ccsds::Result<std::size_t> pack_user_packet(
    std::span<std::byte> out, starcom::ccsds::SpacePacketFields const& sp,
    std::span<const std::byte> user) noexcept {
  return starcom::ccsds::encode_space_packet(out, sp, user);
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
  const auto fn = starcom::ccsds::encode_v3(
      frame, v3, std::span<const std::byte>(packet.data(), *pn));
  if (!fn) {
    return fn;
  }
  return starcom::ccsds::encode_pltu(
      out, std::span<const std::byte>(frame.data(), *fn));
}

starcom::ccsds::Result<std::size_t> pump_submit_sdu(
    BytePump& p, std::span<const std::byte> packet, bool expedited) noexcept {
  return starcom::ccsds::copp_submit_sdu(p.copp, packet, expedited);
}

starcom::ccsds::Result<std::size_t> pump_bytes_to_send(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::copp_bytes_to_send(p.copp, out);
}

void pump_receive_bytes(BytePump& p, std::span<const std::byte> octets) noexcept {
  starcom::ccsds::copp_receive_bytes(p.copp, octets);
}

starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept {
  return starcom::ccsds::copp_take_sdu(p.copp, out);
}

void pump_tick(BytePump& p, starcom::ccsds::Tick now) noexcept {
  starcom::ccsds::copp_tick(p.copp, now);
}

}  // namespace rc::starcom_adapt

#endif  // ROCKETCHIP_USE_STARCOM
