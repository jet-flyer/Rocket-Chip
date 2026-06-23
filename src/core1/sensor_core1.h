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

<<<<<<< Updated upstream
#include "rocketchip/shared_state.h"  // cross-core init flags, GPS, seqlock (OPT-IVP-02)
#include "fusion/eskf.h"              // rc::ESKF
=======
#include "drivers/icm20948.h"         // icm20948_t
#include "drivers/gps.h"             // gps_data_t, gps_transport_t
#include "fusion/eskf.h"             // rc::ESKF
#include "rocketchip/sensor_seqlock.h"  // shared_sensor_data_t (for shared GPS helper)
>>>>>>> Stashed changes

// ============================================================================
// Core 1 Entry Point
// ============================================================================

/// Launch target for multicore_launch_core1(). Sets up MPU stack guard,
/// registers as multicore lockout victim, waits for sensor phase signal,
/// then enters the sensor loop (never returns).
void core1_entry();

// ============================================================================
// Cross-Core Shared State (written by Core 1, read by Core 0)
// ============================================================================

/// Best GPS fix diagnostic: captures the highest-quality fix seen this session.
/// Written by Core 1, read by Core 0 CLI. Atomic flag guards visibility
/// (not struct consistency -- benign for diagnostics, not flight-critical).
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

/// Update best-fix diagnostic when satellite count or HDOP improves.
/// Shared by vehicle Core 1 sensor loop and station idle-bridge tick
/// (Stage 16C IVP-141) so both roles maintain one authoritative
/// implementation. Safe to call with invalid fix — no-op in that case.
void core1_update_best_gps_fix(const shared_sensor_data_t* localData);
<<<<<<< Updated upstream
=======

/// Poll GPS via transport-neutral function pointers + populate seqlock-shape
/// fields in localData. Internally rate-limited by *lastGpsReadUs — caller
/// must pass a stable reference. Same body used by vehicle Core 1 loop and
/// station idle-bridge tick (Stage 16C IVP-141). Callers are responsible
/// for seqlock_write after calling this; this helper only updates the local
/// struct's GPS fields and invokes update_best_gps_fix.
void core1_read_gps(shared_sensor_data_t* localData,
                    uint32_t* lastGpsReadUs);

// ============================================================================
// Extern Globals (owned by main.cpp, accessed by Core 1)
// ============================================================================
// Sensor init flags -- written by Core 0 init_sensors(), read by Core 1.
>>>>>>> Stashed changes

/// Poll GPS via transport-neutral function pointers and populate seqlock-
/// shape GPS fields in localData. Internally rate-limited by
/// *lastGpsReadUs (caller owns the state). Same body used by vehicle
/// Core 1 loop and station idle-bridge tick. Caller is responsible for
/// seqlock_write on localData after calling this — this helper only
/// updates the local struct and invokes update_best_gps_fix.
void core1_read_gps(shared_sensor_data_t* localData,
                    uint32_t* lastGpsReadUs);

// ESKF instance in eskf_runner.cpp — Core 0 fusion; Core 1 reads for GPS
// staleness and related diagnostics.
extern rc::ESKF g_eskf;
extern bool g_eskfInitialized;

#endif // ROCKETCHIP_SENSOR_CORE1_H
