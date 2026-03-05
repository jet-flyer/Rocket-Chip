// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file fused_state.h
 * @brief ESKF-internal float32 snapshot — full-precision for local logging
 *
 * FusedState captures the complete ESKF output at a single time step.
 * Populated by eskf_to_fused_state() on Core 0 after each propagation.
 * Source data: ESKF nominal state + shared_sensor_data_t seqlock fields.
 *
 * IVP-49: Data Model & ICD (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_FUSED_STATE_H
#define ROCKETCHIP_FUSED_STATE_H

#include <stdint.h>

namespace rc {

struct FusedState {
    // Attitude quaternion (body-to-NED)
    float q_w;
    float q_x;
    float q_y;
    float q_z;

    // Position NED (m, relative to origin)
    float pos_n;
    float pos_e;
    float pos_d;

    // Velocity NED (m/s)
    float vel_n;
    float vel_e;
    float vel_d;

    // Accelerometer bias (m/s^2)
    float accel_bias_x;
    float accel_bias_y;
    float accel_bias_z;

    // Gyroscope bias (rad/s)
    float gyro_bias_x;
    float gyro_bias_y;
    float gyro_bias_z;

    // Covariance diagnostics: sqrt of P diagonal (1-sigma)
    float sig_att;     // rad
    float sig_pos;     // m
    float sig_vel;     // m/s

    // Barometric altitude (AGL, m) and vertical velocity (m/s)
    float baro_alt_agl;
    float baro_vvel;

    // Temperature sensors (both flow through seqlock pipeline)
    float baro_temperature_c;      // DPS310 — best ambient proxy
    float imu_temperature_c;       // ICM-20948 die — drift correlation

    // Mahony AHRS divergence (degrees)
    float mahony_div_deg;

    // GPS (from seqlock)
    int32_t gps_lat_1e7;
    int32_t gps_lon_1e7;
    float gps_alt_msl_m;
    float gps_ground_speed_mps;
    uint8_t gps_fix_type;
    uint8_t gps_satellites;

    // Health flags
    bool eskf_healthy;
    bool zupt_active;

    // Flight state (0 = IDLE until IVP-67 state machine)
    uint8_t flight_state;

    // Mission Elapsed Time (milliseconds from time_us_64()/1000)
    uint32_t met_ms;
};

} // namespace rc

#endif // ROCKETCHIP_FUSED_STATE_H
