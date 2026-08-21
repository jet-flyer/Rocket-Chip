// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * PARKED 2026-08-20 — not production firmware.
 * Was: include/rocketchip/sensor_snapshot.h
 * Why: no src/ producer or consumer (WN-045). IVP-55 raw-sensor logging was
 * a Stage 6 stretch that never shipped (DF-001). Kept here so the 40-byte
 * layout can be recovered if IVP-55 is implemented later. Do not re-add to
 * include/rocketchip/ claiming a live log path.
 *
 * @file sensor_snapshot.h
 * @brief Raw pre-calibration sensor data — 40 bytes packed
 *
 * Contains ADC counts and raw values before calibration offset/scale
 * application. Intended for IVP-55 raw sensor logging (never wired).
 * Field units are inconsistent in-file (ADC counts vs Pa*100 / GPS deg);
 * resolve before any future producer.
 */
#ifndef ROCKETCHIP_SENSOR_SNAPSHOT_H
#define ROCKETCHIP_SENSOR_SNAPSHOT_H

#include <stdint.h>

namespace rc {

struct __attribute__((packed)) SensorSnapshot {
    int16_t  accel_raw[3];           // 6B  ICM-20948 ADC counts
    int16_t  gyro_raw[3];           // 6B
    int16_t  mag_raw[3];            // 6B  AK09916 ADC counts
    int32_t  baro_pressure_raw;     // 4B  DPS310 Pa * 100
    int16_t  baro_temp_raw;         // 2B  DPS310 C * 100
    int32_t  gps_lat_1e7;          // 4B
    int32_t  gps_lon_1e7;          // 4B
    float    gps_alt_msl_m;         // 4B
    uint32_t met_us;                // 4B  microsecond MET for high-rate replay
};
static_assert(sizeof(SensorSnapshot) == 40, "SensorSnapshot must be 40 bytes");

} // namespace rc

#endif // ROCKETCHIP_SENSOR_SNAPSHOT_H
