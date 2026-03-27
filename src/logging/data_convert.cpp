// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file data_convert.cpp
 * @brief FusedState <-> TelemetryState conversion implementations
 *
 * IVP-49: Data Model & ICD (Stage 6: Data Logging)
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
    static constexpr float kQ15Scale = 32767.0F;
    t.q_w = clamp_round_i16(f.q_w * kQ15Scale);
    t.q_x = clamp_round_i16(f.q_x * kQ15Scale);
    t.q_y = clamp_round_i16(f.q_y * kQ15Scale);
    t.q_z = clamp_round_i16(f.q_z * kQ15Scale);

    // GPS position (pass-through, already in 1e7 integer format)
    t.lat_1e7 = f.gps_lat_1e7;
    t.lon_1e7 = f.gps_lon_1e7;

    // MSL altitude in millimeters
    t.alt_mm = clamp_round_i32(f.gps_alt_msl_m * 1000.0F);

    // NED velocity in cm/s
    static constexpr float kMsToCms = 100.0F;
    t.vel_n_cms = clamp_round_i16(f.vel_n * kMsToCms);
    t.vel_e_cms = clamp_round_i16(f.vel_e * kMsToCms);
    t.vel_d_cms = clamp_round_i16(f.vel_d * kMsToCms);

    // Baro altitude AGL in mm
    t.baro_alt_mm = clamp_round_i32(f.baro_alt_agl * 1000.0F);

    // Baro vertical velocity in cm/s
    t.baro_vvel_cms = clamp_round_i16(f.baro_vvel * kMsToCms);

    // GPS ground speed in cm/s (unsigned)
    t.gps_speed_cms = clamp_round_u16(f.gps_ground_speed_mps * kMsToCms);

    // GPS fix/sats packed: [7:4]=fix_type, [3:0]=sats (capped to 4-bit max)
    static constexpr uint8_t kNibbleMax = 15;  // 4-bit field maximum
    uint8_t fix = (f.gps_fix_type > kNibbleMax) ? kNibbleMax : f.gps_fix_type;
    uint8_t sats = (f.gps_satellites > kNibbleMax) ? kNibbleMax : f.gps_satellites;
    t.gps_fix_sats = static_cast<uint8_t>((fix << 4) | sats);

    t.flight_state = f.flight_state;

    // Health bitfield
    t.health = 0;
    if (f.eskf_healthy) { t.health |= kHealthEskfHealthy; }
    if (f.zupt_active)  { t.health |= kHealthZuptActive; }

    // Temperature (DPS310 baro, rounded to nearest degree)
    t.temperature_c = clamp_round_i8(f.baro_temperature_c);

    // Battery: not measured yet (future ADC integration)
    t.battery_mv = 0;

    t.met_ms = f.met_ms;
    t._reserved = 0;
}

void telemetry_to_fused_approx(const TelemetryState& t, FusedState& f) {
    // Zero everything first
    f = {};

    // Quaternion: Q15 decode
    static constexpr float kQ15Inv = 1.0F / 32767.0F;
    f.q_w = static_cast<float>(t.q_w) * kQ15Inv;
    f.q_x = static_cast<float>(t.q_x) * kQ15Inv;
    f.q_y = static_cast<float>(t.q_y) * kQ15Inv;
    f.q_z = static_cast<float>(t.q_z) * kQ15Inv;

    static constexpr float kMmToM = 0.001F;  // millimeters to meters
    f.gps_lat_1e7 = t.lat_1e7;
    f.gps_lon_1e7 = t.lon_1e7;
    f.gps_alt_msl_m = static_cast<float>(t.alt_mm) * kMmToM;

    static constexpr float kCmsToMs = 0.01F;
    f.vel_n = static_cast<float>(t.vel_n_cms) * kCmsToMs;
    f.vel_e = static_cast<float>(t.vel_e_cms) * kCmsToMs;
    f.vel_d = static_cast<float>(t.vel_d_cms) * kCmsToMs;

    f.baro_alt_agl = static_cast<float>(t.baro_alt_mm) * kMmToM;
    f.baro_vvel = static_cast<float>(t.baro_vvel_cms) * kCmsToMs;
    f.gps_ground_speed_mps = static_cast<float>(t.gps_speed_cms) * kCmsToMs;

    static constexpr uint8_t kNibbleMask = 0x0F;  // 4-bit field mask
    f.gps_fix_type = (t.gps_fix_sats >> 4) & kNibbleMask;
    f.gps_satellites = t.gps_fix_sats & kNibbleMask;

    f.flight_state = t.flight_state;
    f.eskf_healthy = (t.health & kHealthEskfHealthy) != 0;
    f.zupt_active = (t.health & kHealthZuptActive) != 0;

    f.baro_temperature_c = static_cast<float>(t.temperature_c);
    f.met_ms = t.met_ms;
}

} // namespace rc
