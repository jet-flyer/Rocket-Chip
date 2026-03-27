// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file log_decimator.cpp
 * @brief Box-car averaging decimator for FusedState
 *
 * IVP-52c: Decimation + Main Loop Integration (Stage 6: Data Logging)
 */

#include "log_decimator.h"
#include <cmath>
#include <cstring>

namespace rc {

void decimator_init(LogDecimator* dec, uint32_t ratio) {
    if (dec == nullptr || ratio == 0) {
        return;
    }
    std::memset(dec, 0, sizeof(*dec));
    dec->ratio = ratio;
    dec->initialized = true;
}

// Accumulate float fields by addition (quaternion handled separately)
static void accumulate_floats(FusedState& accum, const FusedState& input) {
    accum.pos_n += input.pos_n;
    accum.pos_e += input.pos_e;
    accum.pos_d += input.pos_d;
    accum.vel_n += input.vel_n;
    accum.vel_e += input.vel_e;
    accum.vel_d += input.vel_d;
    accum.accel_bias_x += input.accel_bias_x;
    accum.accel_bias_y += input.accel_bias_y;
    accum.accel_bias_z += input.accel_bias_z;
    accum.gyro_bias_x += input.gyro_bias_x;
    accum.gyro_bias_y += input.gyro_bias_y;
    accum.gyro_bias_z += input.gyro_bias_z;
    accum.sig_att += input.sig_att;
    accum.sig_pos += input.sig_pos;
    accum.sig_vel += input.sig_vel;
    accum.baro_alt_agl += input.baro_alt_agl;
    accum.baro_vvel += input.baro_vvel;
    accum.baro_temperature_c += input.baro_temperature_c;
    accum.imu_temperature_c += input.imu_temperature_c;
    accum.mahony_div_deg += input.mahony_div_deg;
    accum.gps_alt_msl_m += input.gps_alt_msl_m;
    accum.gps_ground_speed_mps += input.gps_ground_speed_mps;
}

// Average float fields by dividing by count
static void average_floats(FusedState& out, float inv) {
    out.pos_n *= inv;
    out.pos_e *= inv;
    out.pos_d *= inv;
    out.vel_n *= inv;
    out.vel_e *= inv;
    out.vel_d *= inv;
    out.accel_bias_x *= inv;
    out.accel_bias_y *= inv;
    out.accel_bias_z *= inv;
    out.gyro_bias_x *= inv;
    out.gyro_bias_y *= inv;
    out.gyro_bias_z *= inv;
    out.sig_att *= inv;
    out.sig_pos *= inv;
    out.sig_vel *= inv;
    out.baro_alt_agl *= inv;
    out.baro_vvel *= inv;
    out.baro_temperature_c *= inv;
    out.imu_temperature_c *= inv;
    out.mahony_div_deg *= inv;
    out.gps_alt_msl_m *= inv;
    out.gps_ground_speed_mps *= inv;
}

// Overwrite discrete/integer fields from latest sample
static void copy_discrete_fields(FusedState& accum, const FusedState& input) {
    accum.gps_lat_1e7 = input.gps_lat_1e7;
    accum.gps_lon_1e7 = input.gps_lon_1e7;
    accum.gps_fix_type = input.gps_fix_type;
    accum.gps_satellites = input.gps_satellites;
    accum.eskf_healthy = input.eskf_healthy;
    accum.zupt_active = input.zupt_active;
    accum.flight_state = input.flight_state;
    accum.met_ms = input.met_ms;
}

bool decimator_push(LogDecimator* dec, const FusedState& input, FusedState& out) {
    if (dec == nullptr || !dec->initialized) {
        return false;
    }

    if (dec->count == 0) {
        dec->accum = input;
        dec->count = 1;
    } else {
        // Quaternion: flip sign if dot product < 0 (Markley 2007 antipodal check)
        float dot = dec->accum.q_w * input.q_w +
                    dec->accum.q_x * input.q_x +
                    dec->accum.q_y * input.q_y +
                    dec->accum.q_z * input.q_z;
        float sign = (dot < 0.0F) ? -1.0F : 1.0F;
        dec->accum.q_w += sign * input.q_w;
        dec->accum.q_x += sign * input.q_x;
        dec->accum.q_y += sign * input.q_y;
        dec->accum.q_z += sign * input.q_z;

        accumulate_floats(dec->accum, input);
        copy_discrete_fields(dec->accum, input);
        dec->count++;
    }

    if (dec->count < dec->ratio) {
        return false;
    }

    // Output: average all accumulated fields
    float inv = 1.0F / static_cast<float>(dec->count);
    out = dec->accum;

    // Quaternion: divide then normalize
    out.q_w *= inv;
    out.q_x *= inv;
    out.q_y *= inv;
    out.q_z *= inv;
    float qnorm = std::sqrt(out.q_w * out.q_w + out.q_x * out.q_x +
                             out.q_y * out.q_y + out.q_z * out.q_z);
    static constexpr float kQuatNormEpsilon = 1e-6F;  // Guard against zero-norm quaternion
    if (qnorm > kQuatNormEpsilon) {
        float qinv = 1.0F / qnorm;
        out.q_w *= qinv;
        out.q_x *= qinv;
        out.q_y *= qinv;
        out.q_z *= qinv;
    }

    average_floats(out, inv);

    dec->count = 0;
    return true;
}

} // namespace rc
