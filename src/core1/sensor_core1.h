// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Core 1 Sensor Loop — Public Interface
//
// Entry point for Core 1: high-rate sensor sampling (~1kHz IMU, ~31Hz baro,
// ~10Hz GPS). Publishes calibrated data via seqlock to Core 0.
//
// Stage 13 AO Architecture: Phase 1 extraction from main.cpp.
//============================================================================
#ifndef ROCKETCHIP_SENSOR_CORE1_H
#define ROCKETCHIP_SENSOR_CORE1_H

#include <stdint.h>
#include <atomic>

#include "drivers/icm20948.h"         // icm20948_t
#include "drivers/gps.h"             // gps_data_t, gps_transport_t
#include "fusion/eskf.h"             // rc::ESKF

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

// ============================================================================
// Extern Globals (owned by main.cpp, accessed by Core 1)
// ============================================================================
// Sensor init flags -- written by Core 0 init_sensors(), read by Core 1.

extern bool g_imuInitialized;
extern bool g_baroInitialized;
extern bool g_gpsInitialized;
extern bool g_baroContinuous;
extern bool g_neopixelInitialized;
extern bool g_eskfInitialized;

// IMU device handle -- init'd on Core 0, used on Core 1 for reads.
extern icm20948_t g_imu;

// ESKF instance -- init'd on Core 0, read by Core 1 for GPS staleness heuristic.
extern rc::ESKF g_eskf;

// GPS transport-neutral function pointers -- set once during init_sensors().
extern gps_transport_t g_gpsTransport;
extern bool (*g_gpsFnUpdate)();
extern bool (*g_gpsFnGetData)(gps_data_t*);
extern bool (*g_gpsFnHasFix)();

#endif // ROCKETCHIP_SENSOR_CORE1_H
