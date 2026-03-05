// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file telemetry_state.h
 * @brief Fixed-point wire-ready telemetry payload — 45 bytes packed
 *
 * TelemetryState is the canonical wire format for PCM frames and radio
 * telemetry. All fields quantized from FusedState float32 to fixed-point
 * integers for compact transport.
 *
 * Also defines FlightMetadata (UTC epoch anchor) and FlightSummary
 * (per-flight running statistics).
 *
 * IVP-49: Data Model & ICD (Stage 6: Data Logging)
 */

#ifndef ROCKETCHIP_TELEMETRY_STATE_H
#define ROCKETCHIP_TELEMETRY_STATE_H

#include <stdint.h>

namespace rc {

/**
 * @brief Fixed-point telemetry payload — packed, 45 bytes
 *
 * Quantization bounds (max roundtrip error):
 *   Quaternion Q15: ±3.05e-5
 *   Velocity cm/s:  ±0.005 m/s
 *   Altitude mm:    ±0.001 m
 *   Temperature:    ±0.5 C
 */
struct __attribute__((packed)) TelemetryState {
    int16_t  q_w;               // Q15: val * 32767
    int16_t  q_x;
    int16_t  q_y;
    int16_t  q_z;               // 8B

    int32_t  lat_1e7;           // 4B  degrees * 1e7
    int32_t  lon_1e7;           // 4B
    int32_t  alt_mm;            // 4B  MSL, millimeters

    int16_t  vel_n_cms;         // cm/s
    int16_t  vel_e_cms;
    int16_t  vel_d_cms;         // 6B

    int32_t  baro_alt_mm;       // 4B  AGL, millimeters
    int16_t  baro_vvel_cms;     // 2B  baro-derived vertical vel, cm/s

    uint16_t gps_speed_cms;     // 2B  ground speed, cm/s
    uint8_t  gps_fix_sats;      // 1B  [7:4]=fix_type, [3:0]=sats (capped at 15)
    uint8_t  flight_state;      // 1B
    uint8_t  health;            // 1B  bitfield: [0]=eskf_healthy, [1]=zupt_active
    int8_t   temperature_c;     // 1B  DPS310 baro temp, rounded to nearest C
    uint16_t battery_mv;        // 2B  0 = not measured
    uint32_t met_ms;            // 4B
    uint8_t  _reserved;         // 1B  pad to 45
};
static_assert(sizeof(TelemetryState) == 45, "TelemetryState must be 45 bytes");

// Health bitfield definitions
static constexpr uint8_t kHealthEskfHealthy = (1U << 0);
static constexpr uint8_t kHealthZuptActive  = (1U << 1);

/**
 * @brief UTC epoch anchor — captured on first GPS fix
 *
 * Enables MET-to-wall-clock reconstruction:
 *   UTC = anchor_UTC + (frame_MET - met_at_gps_epoch_ms)
 *
 * If met_at_gps_epoch_ms == 0, no GPS fix was acquired — MET is boot-relative.
 */
struct FlightMetadata {
    uint32_t met_at_gps_epoch_ms;    // MET when UTC anchor was captured
    uint16_t utc_year;
    uint8_t  utc_month;
    uint8_t  utc_day;
    uint8_t  utc_hour;
    uint8_t  utc_minute;
    uint8_t  utc_second;
    uint8_t  gps_fix_at_anchor;      // Fix quality when anchor was set
    uint8_t  _pad[2];                // Align to 14 bytes
};

/**
 * @brief Per-flight summary statistics (computed as running values)
 *
 * User-configurable display: max (default), min, average.
 * Updated on every frame during an active flight.
 */
struct FlightSummary {
    float    max_alt_m;              // Running max baro altitude AGL
    float    max_speed_mps;          // Running max velocity magnitude
    float    max_accel_g;            // Running max accel magnitude (g)
    float    min_alt_m;              // Running min altitude
    float    avg_alt_m;              // Running average (sum / frame_count)
    uint32_t duration_ms;            // last_frame_MET - first_frame_MET
    int32_t  landing_lat_1e7;        // Last valid GPS fix
    int32_t  landing_lon_1e7;
    uint32_t frame_count;            // For average computation
};

} // namespace rc

#endif // ROCKETCHIP_TELEMETRY_STATE_H
