// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Calibration Hooks — Cross-Core I2C Pause/Resume + Sensor Read Callbacks
//
// Stage 13 AO Architecture: Phase 8 extraction from main.cpp.
//============================================================================

#include "cal_hooks.h"
#include "rocketchip/sensor_seqlock.h"
#include "drivers/icm20948.h"
#include "ao_led_engine.h"
#include "pico/stdlib.h"
#include <stdio.h>

// Extern globals from main.cpp and sensor_core1.cpp
extern icm20948_t g_imu;
extern bool g_sensorPhaseActive;

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t kCalReadDelayMs = 10;           // ~100Hz accel sampling
static constexpr uint32_t kMagDiagPrintModulus = 200;      // Print every N mag failures
static constexpr uint32_t kCore1PauseAckMaxMs = 100;       // Max wait for Core 1 pause

// ============================================================================
// Mag read staleness tracking
// ============================================================================

static uint32_t g_lastMagReadCount = 0;
static uint32_t g_magDiagSeqlockFail = 0;
static uint32_t g_magDiagNotValid = 0;
static uint32_t g_magDiagStale = 0;
static uint32_t g_magDiagLastSeenCount = 0;

// ============================================================================
// Accel Read Callback (for 6-pos calibration via CLI)
// ============================================================================

bool cal_read_accel(float* ax, float* ay, float* az, float* tempC) {
    sleep_ms(kCalReadDelayMs);
    // Use full icm20948_read() instead of icm20948_read_accel() — the accel-only
    // read (6 bytes from ACCEL_XOUT_H) does NOT read through TEMP_OUT_L, so the
    // data-ready flag is never cleared. After ~200 reads the output registers
    // stop updating (all zeros). The full 14-byte read clears data-ready.
    icm20948_data_t data;
    if (!icm20948_read(&g_imu, &data) || !data.accel_valid) {
        return false;
    }
    *ax = data.accel.x;
    *ay = data.accel.y;
    *az = data.accel.z;
    *tempC = data.temperature_c;
    return true;
}

// ============================================================================
// Mag Read Callback (for compass calibration via CLI)
// ============================================================================
// Reads from seqlock — Core 1 keeps running, no I2C contention.

void cal_reset_mag_staleness() {
    g_lastMagReadCount = 0;
    g_magDiagSeqlockFail = 0;
    g_magDiagNotValid = 0;
    g_magDiagStale = 0;
    g_magDiagLastSeenCount = 0;
}

bool cal_read_mag(float* mx, float* my, float* mz) {
    shared_sensor_data_t snap = {};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        g_magDiagSeqlockFail++;
        return false;
    }
    g_magDiagLastSeenCount = snap.mag_read_count;
    if (!snap.mag_valid) {
        g_magDiagNotValid++;
        if (g_magDiagNotValid == 1 || g_magDiagNotValid % kMagDiagPrintModulus == 0) {
            (void)printf("  [mag_valid=false, mag_read_count=%lu, lastAccepted=%lu]\n",
                        (unsigned long)snap.mag_read_count,
                        (unsigned long)g_lastMagReadCount);
            (void)fflush(stdout);
        }
        return false;
    }
    if (snap.mag_read_count == g_lastMagReadCount) {
        g_magDiagStale++;
        return false;
    }
    g_lastMagReadCount = snap.mag_read_count;

    // Return RAW mag data — ellipsoid solver needs uncorrected samples.
    *mx = snap.mag_raw_x;
    *my = snap.mag_raw_y;
    *mz = snap.mag_raw_z;
    return true;
}

// ============================================================================
// Core 1 I2C Pause/Resume Hooks
// ============================================================================
// Bypass mode eliminates the I2C master race condition (LL Entry 21).
// Hooks only need to pause/unpause Core 1 for bus ownership.

void cal_pre_hook() {
    if (g_sensorPhaseActive && !g_core1I2CPaused.load(std::memory_order_acquire)) {
        g_core1PauseI2C.store(true, std::memory_order_release);
        for (uint32_t i = 0; i < kCore1PauseAckMaxMs; i++) {
            if (g_core1I2CPaused.load(std::memory_order_acquire)) {
                break;
            }
            sleep_ms(1);
        }
    }
}

void cal_post_hook() {
    if (g_sensorPhaseActive) {
        g_calReloadPending.store(true, std::memory_order_release);
        g_core1PauseI2C.store(false, std::memory_order_release);
    }
}

// ============================================================================
// NeoPixel + Feed Callbacks
// ============================================================================

void cal_set_neo_override(uint8_t mode) {
    AO_LedEngine_post_override(mode);
}

void cal_feed_active() {
    // Core 1 handles all sensor feeds. Nothing to do here.
}
