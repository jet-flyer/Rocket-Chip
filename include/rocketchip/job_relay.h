// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job_relay.h
 * @brief Relay role — range extender configuration (IVP-95)
 *
 * RX continuous, validate CCSDS CRC, re-TX. Link-layer only — no payload
 * decode, no AO_Telemetry, no ESKF, no Flight Director.
 * Council 3 [C3-R2]: relay is link-layer only in AO_Radio.
 */

#ifndef ROCKETCHIP_JOB_RELAY_H
#define ROCKETCHIP_JOB_RELAY_H

namespace job {

inline constexpr DeviceRole kRole = DeviceRole::kRelay;

// Radio mode: RX continuous (relay receives, then re-TXes)
inline constexpr bool kRadioModeRx = true;

// No MAVLink output on relay (no AO_Telemetry)
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_RELAY_H
