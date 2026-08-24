// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Cross-core and CLI-visible globals. Core 0 initializes; Core 1 reads
// most sensor flags and g_gpsTransport; CLI reads status.

#ifndef ROCKETCHIP_SHARED_STATE_H
#define ROCKETCHIP_SHARED_STATE_H

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "drivers/icm20948.h"
#include "drivers/gps.h"
#include "rocketchip/sensor_seqlock.h"

// ============================================================================
// Global State
// ============================================================================

// Sensor initialization flags
extern bool g_neopixelInitialized;      // Core 1 reads
extern bool g_i2cInitialized;           // CLI reads
extern bool g_imuInitialized;           // Core 1 reads
extern bool g_baroInitialized;          // Core 1 reads/writes
extern bool g_baroContinuous;           // Core 1 reads
extern bool g_gpsInitialized;           // Core 1 reads/writes
extern bool g_spiInitialized;           // CLI reads

// Distinguishes "attempted and failed" from "not present on this role".
extern bool g_imuInitAttempted;
extern bool g_baroInitAttempted;
extern bool g_gpsInitAttempted;

// PSRAM state. Ring uses PSRAM only if size > 0 and both tests pass.
extern size_t g_psramSize;
extern bool g_psramSelfTestPassed;
extern bool g_psramFlashSafePassed;

// Calibration storage
extern bool g_calStorageInitialized;

// GPS transport. Core 0 writes once in init_sensors() before releasing
// g_startSensorPhase; Core 1 reads after that barrier. Callers switch
// on this and invoke gps_uart_* or gps_pa1010d_* by name (P10-9).
extern gps_transport_t g_gpsTransport;

// IMU handle (not seqlock-protected). Core 0 inits before
// g_startSensorPhase; Core 1 reads at 1 kHz (may re-init on consecutive
// fail). After handoff, Core 0 does not icm20948_read this handle:
// CLI 's' uses the seqlock; HW-status config dump is skipped when
// rc_os_i2c_scan_allowed is false (vehicle after Core 1 launch).
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

// Cooperative pause of Core 1 I2C around flash_safe_execute (R-17 / LL 31).
// Distinct from g_calReloadPending (cal-only). May return after ~100 ms
// without an ack; callers still run post-flash i2c_bus_reset().
namespace rc {
void core1_i2c_pause();
void core1_i2c_resume();
}

#endif  // ROCKETCHIP_SHARED_STATE_H
