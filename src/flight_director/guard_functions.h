// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_functions.h
 * @brief Flight phase guard functions (IVP-70)
 *
 * Pure functions that evaluate sensor data against thresholds.
 * Each guard returns true when its condition is met for a single sample.
 * Sustain timing (N consecutive true samples) is handled by the
 * GuardEvaluator, not here.
 *
 * Guards read from FusedState (ESKF output) — not raw sensor data.
 * Thresholds come from MissionProfile.
 */

#ifndef ROCKETCHIP_GUARD_FUNCTIONS_H
#define ROCKETCHIP_GUARD_FUNCTIONS_H

#include "rocketchip/fused_state.h"

namespace rc {

// Launch detection: body-Z acceleration exceeds threshold.
// In NED frame with rocket pointing up, launch produces large negative vel_d
// and large positive body-Z accel. We use |accel_z| from the calibrated
// accel reading stashed in FusedState (via seqlock snapshot).
// This guard checks accel magnitude since body-Z orientation varies.
//
// @param accel_z Calibrated body-Z acceleration (m/s^2), from seqlock
// @param threshold MissionProfile::launch_accel_threshold
bool guard_launch_accel(float accel_z, float threshold);

// Burnout detection: acceleration magnitude drops below threshold.
// During powered flight, accel > 1g from thrust. At burnout, only
// drag + gravity remain, so |A| drops toward ~1g then below as
// drag decreases. We check total accel magnitude.
//
// @param accel_mag sqrt(ax^2 + ay^2 + az^2) in m/s^2
// @param threshold MissionProfile::burnout_accel_threshold
bool guard_burnout_accel(float accel_mag, float threshold);

// Apogee detection: vertical velocity crosses zero (going negative).
// ESKF vel_d is NED — negative = ascending, positive = descending.
// Apogee is when vel_d crosses from negative to positive (or near zero).
//
// @param vel_d NED down velocity (m/s), negative = ascending
// @param threshold MissionProfile::apogee_velocity_threshold (positive value)
bool guard_apogee_velocity(float vel_d, float threshold);

// Backup apogee: barometric altitude rate is non-positive.
// If baro_vvel <= 0 for sustained period, rocket is descending.
// Uses baro vertical velocity from BaroKF, not ESKF.
//
// @param baro_vvel Baro-derived vertical velocity (m/s), positive = ascending
bool guard_baro_peak(float baro_vvel);

// Main deploy: altitude AGL below threshold.
// Rocket is descending through the main chute deployment altitude.
//
// @param baro_alt_agl Barometric altitude above ground level (m)
// @param threshold MissionProfile::main_deploy_altitude_m
bool guard_main_deploy_altitude(float baro_alt_agl, float threshold);

// Landing detection: velocity magnitude near zero.
// All three NED velocity components must be small. Uses velocity norm.
//
// @param vel_n, vel_e, vel_d NED velocity components (m/s)
// @param threshold MissionProfile::landing_velocity_threshold
bool guard_stationary(float vel_n, float vel_e, float vel_d, float threshold);

} // namespace rc

#endif // ROCKETCHIP_GUARD_FUNCTIONS_H
