// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file job_capabilities.h
 * @brief Compile-time capability predicates for each device role.
 *
 * Role-scoped "does this role do X?" constexprs, consumed by shared code
 * paths that need to mask behavior without scattering `if (kRole == ...)`
 * checks across the codebase.
 *
 * Stage 16C IVP-142c introduces this file as the single home for
 * capability masking. Stage 16C IVP-143 will generalize into board-level
 * capability flags (peripheral presence, etc.); role-level capabilities
 * stay here.
 *
 * Do NOT add per-peripheral presence flags here (IMU/baro/GPS presence
 * is a board property, not a role property — a station with an optional
 * baro is still "station role"). Those live in board_*.h.
 */

#ifndef ROCKETCHIP_JOB_CAPABILITIES_H
#define ROCKETCHIP_JOB_CAPABILITIES_H

#include "rocketchip/job.h"

namespace job {

// Does this role run Core 1 in sensor-sampling mode?
// - Vehicle: yes (IMU/baro/GPS + mag at 1 kHz).
// - Station: no (Core 1 launched but idles on g_startSensorPhase forever).
// - Relay:   no (same as station).
//
// Consumers: health_monitor Core1 vitality check — the check is
// meaningless when this role never advances core1_loop_count.
inline constexpr bool kRoleSamplesCore1 = (kRole == DeviceRole::kVehicle);

// Does this role run AO_Logger with the flight table in flash?
// - Vehicle: yes (mission log).
// - Station: not yet (log file format / replay scope is a Stage 16C
//   follow-up; keep off until that's done).
// - Relay:   no (link-layer only per job_relay.h).
//
// Consumers: health_monitor kHealthFlashOk bit — only assert when
// the role actually initializes and writes to flash log.
inline constexpr bool kRoleRunsLogger = (kRole == DeviceRole::kVehicle);

// Does this role run the full vehicle Go/No-Go matrix (IMU + baro +
// ESKF + flash + watchdog)?
// - Vehicle: yes.
// - Station: no — station's readiness is a single condensed bit
//   (radio + watchdog + flash_if_present + mcu_not_critical) that
//   plugs into the vehicle's Go/No-Go as one item over the radio.
// - Relay:   no — link-layer only, no go/no-go participation.
inline constexpr bool kRoleHasFullGoNogo = (kRole == DeviceRole::kVehicle);

} // namespace job

#endif // ROCKETCHIP_JOB_CAPABILITIES_H
