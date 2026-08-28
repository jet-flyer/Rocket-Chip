// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// AO byte pump (IVP 21–22). RC-owned. RadioScheduler / SX1276 stay in RC.
// No Starcom default pin map. Soak SCIDs are RC IDs, not a Starcom MIB.
// IVP 22: air path is COP-P (submit_sdu / bytes_to_send / receive_bytes).

#ifndef ROCKETCHIP_STARCOM_BYTE_PUMP_H
#define ROCKETCHIP_STARCOM_BYTE_PUMP_H

#ifdef ROCKETCHIP_USE_STARCOM

#include "starcom/ccsds/copp.hpp"
#include "starcom/ccsds/pltu.hpp"
#include "starcom/ccsds/space_packet.hpp"
#include "starcom/ccsds/types.hpp"
#include "starcom/ccsds/v3.hpp"
#include "starcom/result.hpp"
#include "rocketchip/telemetry_state.h"
#include "rocketchip/telemetry_encoder.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rc::starcom_adapt {

// SX1276 FIFO is 255 octets. Consumer MTU, not a Starcom book cap.
inline constexpr std::size_t kAirMtu = 255;
inline constexpr starcom::ccsds::Apid kNavApid{0x001};
// RC kApidCmdAck (include/rocketchip/telemetry_encoder.h). TC=command, TM=ACK.
inline constexpr starcom::ccsds::Apid kCmdApid{0x003};
inline constexpr starcom::ccsds::Pcid kSoakPcid{0};
inline constexpr starcom::ccsds::PortId kSoakPort{1};

struct BytePump {
  starcom::ccsds::CoppEndpoint copp{};
  starcom::ccsds::Scid local_scid{};
  starcom::ccsds::Scid remote_scid{};
};

void pump_init(BytePump& p, starcom::ccsds::Scid local,
               starcom::ccsds::Scid remote) noexcept;
void pump_init_for_this_job(BytePump& p) noexcept;

starcom::ccsds::Result<std::size_t> pump_encode_pltu(
    std::span<std::byte> out, std::span<const std::byte> frame) noexcept;
starcom::ccsds::Result<std::size_t> pump_repeat_pltu(
    std::span<std::byte> out, std::span<const std::byte> octets) noexcept;
starcom::ccsds::Result<std::size_t> pump_encode_nav(
    BytePump& p, std::span<std::byte> out,
    const TelemetryState& telem) noexcept;

starcom::ccsds::Result<std::size_t> pump_pack_nav_packet(
    std::span<std::byte> out, const TelemetryState& telem) noexcept;
starcom::ccsds::Result<std::size_t> pump_pack_cmd_packet(
    std::span<std::byte> out, uint16_t cmd_id, uint8_t seq, float p1, float p2,
    float p3, float p4, float p5) noexcept;
starcom::ccsds::Result<std::size_t> pump_pack_ack_packet(
    std::span<std::byte> out, const ccsds::CommandAckPayload& ack) noexcept;

starcom::ccsds::Result<std::size_t> pump_submit_sdu(
    BytePump& p, std::span<const std::byte> packet, bool expedited) noexcept;
starcom::ccsds::Result<std::size_t> pump_bytes_to_send(
    BytePump& p, std::span<std::byte> out) noexcept;
void pump_receive_bytes(BytePump& p, std::span<const std::byte> octets) noexcept;
starcom::ccsds::Result<std::size_t> pump_take_sdu(
    BytePump& p, std::span<std::byte> out) noexcept;
void pump_tick(BytePump& p, starcom::ccsds::Tick now) noexcept;

}  // namespace rc::starcom_adapt

#endif  // ROCKETCHIP_USE_STARCOM
#endif  // ROCKETCHIP_STARCOM_BYTE_PUMP_H
