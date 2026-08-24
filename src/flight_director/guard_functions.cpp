// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Flight phase guard function implementations (IVP-70)

#include "guard_functions.h"
#include <cmath>

namespace rc {

bool guard_launch_accel(float accel_z, float threshold) {
    return fabsf(accel_z) > threshold;
}

bool guard_burnout_accel(float accel_mag, float threshold) {
    // Burnout: total acceleration drops below thrust threshold
    return accel_mag < threshold;
}

bool guard_apogee_velocity(float vel_d, float threshold) {
    // Level check, not a crossing. NED down-positive.
    return vel_d > -threshold;
}

bool guard_baro_peak(float vert_vel) {
    // Sign test on the caller float (currently vert_vel_eskf, NED down-positive).
    return vert_vel <= 0.0F;
}

bool guard_main_deploy_altitude(float baro_alt_agl, float threshold) {
    return baro_alt_agl < threshold;
}

bool guard_stationary(float vel_n, float vel_e, float vel_d, float threshold) {
    float vel_mag = sqrtf(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d);
    return vel_mag < threshold;
}

bool guard_baro_stationary(float baro_alt_rate_mps, float threshold) {
    return fabsf(baro_alt_rate_mps) < threshold;
}

} // namespace rc
