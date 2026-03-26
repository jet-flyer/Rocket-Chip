// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_functions.cpp
 * @brief Flight phase guard function implementations (IVP-70)
 */

#include "guard_functions.h"
#include <cmath>

namespace rc {

bool guard_launch_accel(float accel_z, float threshold) {
    // Launch produces large body-Z accel. Use absolute value since
    // orientation sign depends on mounting.
    return fabsf(accel_z) > threshold;
}

bool guard_burnout_accel(float accel_mag, float threshold) {
    // Burnout: total acceleration drops below thrust threshold
    return accel_mag < threshold;
}

bool guard_apogee_velocity(float vel_d, float threshold) {
    // NED: vel_d > 0 = descending. Apogee = velocity near zero or positive.
    // threshold is a positive value (e.g., 0.5 m/s).
    return vel_d > -threshold;
}

bool guard_baro_peak(float baro_vvel) {
    // Baro vertical velocity <= 0 means altitude is no longer increasing
    return baro_vvel <= 0.0f;
}

bool guard_main_deploy_altitude(float baro_alt_agl, float threshold) {
    // Descending through deployment altitude
    return baro_alt_agl < threshold;
}

bool guard_stationary(float vel_n, float vel_e, float vel_d, float threshold) {
    float vel_mag = sqrtf(vel_n * vel_n + vel_e * vel_e + vel_d * vel_d);
    return vel_mag < threshold;
}

} // namespace rc
