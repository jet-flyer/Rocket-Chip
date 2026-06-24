// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file shared_state.h
 * @brief Centralized cross-core and CLI-visible global state (OPT-IVP-02).
 *
 * Consolidates all init flags, GPS function pointers, seqlock, atomics,
 * and device handles from main.cpp. This reduces duplication and makes
 * ownership clear.
 *
 * Core 0 owns initialization.
 * Core 1 reads most sensor flags and uses the GPS function pointers.
 * CLI reads status for display.
 */

#ifndef ROCKETCHIP_SHARED_STATE_H
#define ROCKETCHIP_SHARED_STATE_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "drivers/icm20948.h"
#include "drivers/gps.h"
#include "rocketchip/sensor_seqlock.h"

// ============================================================================
// Global State (moved from src/main.cpp)
// ============================================================================

// Sensor initialization flags
extern bool g_neopixelInitialized;      // Core 1 reads
extern bool g_i2cInitialized;           // CLI reads
extern bool g_imuInitialized;           // Core 1 reads
extern bool g_baroInitialized;          // Core 1 reads/writes
extern bool g_baroContinuous;           // Core 1 reads
extern bool g_gpsInitialized;           // Core 1 reads/writes
extern bool g_spiInitialized;           // CLI reads

// Init-attempted flags (IVP-142c). Distinguishes "attempted and failed"
// from "not present on this role".
extern bool g_imuInitAttempted;
extern bool g_baroInitAttempted;
extern bool g_gpsInitAttempted;

// PSRAM state
extern size_t g_psramSize;
extern bool g_psramSelfTestPassed;
extern bool g_psramFlashSafePassed;

// Calibration storage
extern bool g_calStorageInitialized;

// GPS transport and function pointers (set once in init_sensors())
extern gps_transport_t g_gpsTransport;
extern bool (*g_gpsFnUpdate)();
extern bool (*g_gpsFnGetData)(gps_data_t*);
extern bool (*g_gpsFnHasFix)();

// IMU device handle (initialized on Core 0, used on Core 1)
extern icm20948_t g_imu;

// Sensor seqlock (Core 1 writer, Core 0 reader)
extern sensor_seqlock_t g_sensorSeqlock;

// Cross-core synchronization atomics
extern std::atomic<bool> g_startSensorPhase;
extern std::atomic<bool> g_sensorPhaseDone;
extern std::atomic<bool> g_calReloadPending;
extern std::atomic<bool> g_core1PauseI2C;
extern std::atomic<bool> g_core1I2CPaused;
extern std::atomic<bool> g_core1LockoutReady;

// Sensor phase flag (Core 0 write, Core 0/Core 1 read for gating)
extern bool g_sensorPhaseActive;

#endif  // ROCKETCHIP_SHARED_STATE_H
