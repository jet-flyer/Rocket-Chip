// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Relay role — range extender
// RX continuous and re-TX. Link-layer only — no payload decode,
// no AO_Telemetry, no ESKF, no Flight Director.

#ifndef ROCKETCHIP_JOB_RELAY_H
#define ROCKETCHIP_JOB_RELAY_H

namespace job {

inline constexpr DeviceRole kRole = DeviceRole::kRelay;

// kRadioModeRx is "radio stays in RX". Relay re-TX is this job, not this flag.
inline constexpr bool kRadioModeRx = true;

// No MAVLink output on relay (no AO_Telemetry)
inline constexpr bool kDefaultMavlinkOutput = false;

} // namespace job

#endif // ROCKETCHIP_JOB_RELAY_H
