// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file data_convert.cpp
 * @brief FusedState <-> TelemetryState conversion implementations
 */

#include "data_convert.h"
#include <cmath>

namespace rc {

// Integer type limits as float for clamping
static constexpr float kInt16MaxF  =  32767.0F;
static constexpr float kInt16MinF  = -32768.0F;
static constexpr int16_t kInt16Max =  32767;

// Largest float exactly representable below INT32_MAX
static constexpr float kInt32MaxF  =  2147483520.0F;
static constexpr float kInt32MinF  = -2147483520.0F;
static constexpr int32_t kInt32Max = INT32_MAX;

static constexpr float kUint16MaxF    = 65535.0F;
static constexpr uint16_t kUint16Max  = 65535;

static constexpr float kInt8MaxF  =  127.0F;
static constexpr float kInt8MinF  = -128.0F;
static constexpr int8_t kInt8Max  =  127;

// Clamp float to int16 range and round
static int16_t clamp_round_i16(float val) {
    if (val >= kInt16MaxF) { return kInt16Max; }
    if (val <= kInt16MinF) { return -kInt16Max - 1; }
    return static_cast<int16_t>(std::roundf(val));
}

// Clamp float to int32 range and round
static int32_t clamp_round_i32(float val) {
    if (val >= kInt32MaxF) { return kInt32Max; }
    if (val <= kInt32MinF) { return -kInt32Max; }
    return static_cast<int32_t>(std::roundf(val));
}

// Clamp float to uint16 range and round
static uint16_t clamp_round_u16(float val) {
    if (val >= kUint16MaxF) { return kUint16Max; }
    if (val <= 0.0F) { return 0; }
    return static_cast<uint16_t>(std::roundf(val));
}

// Clamp float to int8 range and round
static int8_t clamp_round_i8(float val) {
    if (val >= kInt8MaxF) { return kInt8Max; }
    if (val <= kInt8MinF) { return -kInt8Max - 1; }
    return static_cast<int8_t>(std::roundf(val));
}

void fused_to_telemetry(const FusedState& f, TelemetryState& t) {
    // Quaternion: Q15 encoding (val * 32767)
    static constexpr float k_q15_scale = 32767.0F;
    t.q_w = clamp_round_i16(f.q_w * k_q15_scale);
    t.q_x = clamp_round_i16(f.q_x * k_q15_scale);
    t.q_y = clamp_round_i16(f.q_y * k_q15_scale);
    t.q_z = clamp_round_i16(f.q_z * k_q15_scale);

    // GPS position (pass-through, already in 1e7 integer format)
    t.lat_1e7 = f.gps_lat_1e7;
    t.lon_1e7 = f.gps_lon_1e7;

    // MSL altitude in millimeters
    t.alt_mm = clamp_round_i32(f.gps_alt_msl_m * 1000.0F);

    // NED velocity in cm/s
    static constexpr float k_ms_to_cms = 100.0F;
    t.vel_n_cms = clamp_round_i16(f.vel_n * k_ms_to_cms);
    t.vel_e_cms = clamp_round_i16(f.vel_e * k_ms_to_cms);
    t.vel_d_cms = clamp_round_i16(f.vel_d * k_ms_to_cms);

    // Baro altitude AGL in mm
    t.baro_alt_mm = clamp_round_i32(f.baro_alt_agl * 1000.0F);

    // Baro vertical velocity in cm/s
    t.baro_vvel_cms = clamp_round_i16(f.vert_vel_eskf * k_ms_to_cms);

    // GPS ground speed in cm/s (unsigned)
    t.gps_speed_cms = clamp_round_u16(f.gps_ground_speed_mps * k_ms_to_cms);

    // GPS fix/sats packed: [7:4]=fix_type, [3:0]=sats (capped to 4-bit max)
    static constexpr uint8_t k_nibble_max = 15;  // 4-bit field maximum
    uint8_t fix = (f.gps_fix_type > k_nibble_max) ? k_nibble_max : f.gps_fix_type;
    uint8_t sats = (f.gps_satellites > k_nibble_max) ? k_nibble_max : f.gps_satellites;
    t.gps_fix_sats = static_cast<uint8_t>((fix << 4) | sats);

    t.flight_state = f.flight_state;

    // Health: direct copy of 2-bit packed primary byte (IVP-107)
    t.health = f.health_primary;

    // Temperature (DPS310 baro, rounded to nearest degree)
    t.temperature_c = clamp_round_i8(f.baro_temperature_c);

    // Battery: not measured (no ADC wired)
    t.battery_mv = 0;

    t.met_ms = f.met_ms;
    t.flags = 0;
    if (f.zupt_active) { t.flags |= kFlagsZuptActive; }
}

void telemetry_to_fused_approx(const TelemetryState& t, FusedState& f) {
    // Zero everything first
    f = {};

    // Quaternion: Q15 decode
    static constexpr float k_q15_inv = 1.0F / 32767.0F;
    f.q_w = static_cast<float>(t.q_w) * k_q15_inv;
    f.q_x = static_cast<float>(t.q_x) * k_q15_inv;
    f.q_y = static_cast<float>(t.q_y) * k_q15_inv;
    f.q_z = static_cast<float>(t.q_z) * k_q15_inv;

    static constexpr float k_mm_to_m = 0.001F;  // millimeters to meters
    f.gps_lat_1e7 = t.lat_1e7;
    f.gps_lon_1e7 = t.lon_1e7;
    f.gps_alt_msl_m = static_cast<float>(t.alt_mm) * k_mm_to_m;

    static constexpr float k_cms_to_ms = 0.01F;
    f.vel_n = static_cast<float>(t.vel_n_cms) * k_cms_to_ms;
    f.vel_e = static_cast<float>(t.vel_e_cms) * k_cms_to_ms;
    f.vel_d = static_cast<float>(t.vel_d_cms) * k_cms_to_ms;

    f.baro_alt_agl = static_cast<float>(t.baro_alt_mm) * k_mm_to_m;
    f.vert_vel_eskf = static_cast<float>(t.baro_vvel_cms) * k_cms_to_ms;
    f.gps_ground_speed_mps = static_cast<float>(t.gps_speed_cms) * k_cms_to_ms;

    static constexpr uint8_t k_nibble_mask = 0x0F;  // 4-bit field mask
    f.gps_fix_type = (t.gps_fix_sats >> 4) & k_nibble_mask;
    f.gps_satellites = t.gps_fix_sats & k_nibble_mask;

    f.flight_state = t.flight_state;
    f.health_primary = t.health;  // Direct copy of 2-bit packed byte (IVP-107)
    f.zupt_active = (t.flags & kFlagsZuptActive) != 0;

    f.baro_temperature_c = static_cast<float>(t.temperature_c);
    f.met_ms = t.met_ms;
}

} // namespace rc
