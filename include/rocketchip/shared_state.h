// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Cross-core and CLI-visible globals. Definitions in shared_state.cpp.
// Seqlock + signaling atomics are declared in sensor_seqlock.h (included
// below) — do not re-extern them here.

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

// Sensor initialization flags. Core 0 writes during bring-up.
// g_neopixelInitialized / g_i2cInitialized / g_spiInitialized / g_baroContinuous:
//   Core 0 write, Core 0 CLI/status read — not a Core 1 object.
// g_imuInitialized: Core 0 write before g_startSensorPhase; Core 1 reads after.
extern bool g_neopixelInitialized;
extern bool g_i2cInitialized;
extern bool g_imuInitialized;
extern bool g_baroContinuous;
extern bool g_spiInitialized;

// Live after handoff (two cores). Release-store / acquire-load.
// g_baroInitialized: Core 0 init; Core 1 may store false (device dead).
// g_gpsInitialized: Core 0 init and UART-reinit fail; Core 1 reads.
extern std::atomic<bool> g_baroInitialized;
extern std::atomic<bool> g_gpsInitialized;

// Distinguishes "attempted and failed" from "not present on this role".
// Core 0 write, Core 0 CLI/boot-summary read.
extern bool g_imuInitAttempted;
extern bool g_baroInitAttempted;
extern bool g_gpsInitAttempted;

// PSRAM state. Core 0 write during boot; logger/CLI read. Ring uses PSRAM
// only if size > 0 and both tests pass.
extern size_t g_psramSize;
extern bool g_psramSelfTestPassed;
extern bool g_psramFlashSafePassed;

// Calibration storage. Core 0 write at boot, Core 0 read.
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

// Sensor phase flag. Core 0 write, Core 0 read (pause, ESKF tick, cal_hooks).
extern bool g_sensorPhaseActive;

// Pause Core 1 I2C around runtime flash_safe_execute (R-17 / LL 31).
// Not nestable. May return after ~100 ms without ack; callers abort/reattach.
// Wrapped: cal save (including AO_RCOS_start_cal_save), flight erase/download.
namespace rc {
void core1_i2c_pause();
void core1_i2c_resume();
}

#endif  // ROCKETCHIP_SHARED_STATE_H
