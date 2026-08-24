// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flight phase guard functions (IVP-70). One-sample predicates; sustain is GuardEvaluator.
// Primitive float args only — this leaf does not read FusedState or MissionProfile.

#ifndef ROCKETCHIP_GUARD_FUNCTIONS_H
#define ROCKETCHIP_GUARD_FUNCTIONS_H

#include "rocketchip/fused_state.h"

namespace rc {

// Launch: |accel_z| > threshold.
bool guard_launch_accel(float accel_z, float threshold);

// Burnout: accel_mag < threshold.
bool guard_burnout_accel(float accel_mag, float threshold);

// Apogee: vel_d > -threshold (level check, not a zero-crossing). NED down-positive.
bool guard_apogee_velocity(float vel_d, float threshold);

// Backup apogee: vert_vel <= 0. Not a baro peak. Callers pass fused.vert_vel_eskf.
bool guard_baro_peak(float vert_vel);

// Main deploy: baro_alt_agl < threshold. Altitude only.
bool guard_main_deploy_altitude(float baro_alt_agl, float threshold);

// Landing: NED velocity norm < threshold.
bool guard_stationary(float vel_n, float vel_e, float vel_d, float threshold);

// Landing: |baro_alt_rate_mps| < threshold.
bool guard_baro_stationary(float baro_alt_rate_mps, float threshold);

} // namespace rc

#endif // ROCKETCHIP_GUARD_FUNCTIONS_H
