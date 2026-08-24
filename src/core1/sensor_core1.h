// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Core 1 Sensor Loop — Public Interface
//
// Entry point for Core 1: high-rate sensor sampling (~1kHz IMU, ~31Hz baro,
// ~10Hz GPS). Publishes calibrated data via seqlock to Core 0.
//============================================================================
#ifndef ROCKETCHIP_SENSOR_CORE1_H
#define ROCKETCHIP_SENSOR_CORE1_H

#include <stdint.h>
#include <atomic>

#include "rocketchip/shared_state.h"  // cross-core init flags, GPS, seqlock (OPT-IVP-02)

// ============================================================================
// Core 1 Entry Point
// ============================================================================

// Launch target for multicore_launch_core1(). Sets up MPU stack guard,
// registers as multicore lockout victim, waits for sensor phase signal,
// then enters the sensor loop (never returns).
void core1_entry();

// Best-GPS diagnostic (non-atomic). Mutator: core1_update_best_gps_fix. Flag is visibility only.
struct best_gps_fix_t {
    int32_t lat_1e7;
    int32_t lon_1e7;
    float alt_msl_m;
    float hdop;
    uint8_t satellites;
    uint8_t fix_type;
};

extern best_gps_fix_t g_bestGpsFix;
extern std::atomic<bool> g_bestGpsValid;

// Update best-fix diagnostic when satellite count or HDOP improves.
// Shared by vehicle Core 1 sensor loop and station idle-bridge tick
// (Stage 16C IVP-141) so both roles maintain one authoritative
// implementation. Safe to call with invalid fix — no-op in that case.
void core1_update_best_gps_fix(const shared_sensor_data_t* localData);

// Poll the bound GPS transport into seqlock-shaped fields in localData.
// Rate-limited by *lastGpsReadUs (caller owns). Vehicle Core 1 loop and
// station idle-bridge share this body. Side effects: may busy-wait SDA
// settle (I2C), increment gps_error_count, and request a Core-0 UART
// reinit after 10 s of UART staleness (non-blocking on this core). Caller
// seqlock_writes localData after return.
void core1_read_gps(shared_sensor_data_t* localData,
                    uint32_t* lastGpsReadUs);

#endif // ROCKETCHIP_SENSOR_CORE1_H
