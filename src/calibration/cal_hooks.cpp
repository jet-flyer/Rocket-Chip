// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Cal helpers: seqlock mag read, post-save Core 1 reload.
//============================================================================

#include "cal_hooks.h"
#include "rocketchip/shared_state.h"
#include "rocketchip/rc_log.h"

// ============================================================================
// Constants
// ============================================================================

static constexpr uint32_t kMagDiagPrintModulus = 200;      // Print every N mag failures

// ============================================================================
// Mag read staleness tracking
// ============================================================================

static uint32_t g_lastMagReadCount = 0;
static uint32_t g_magDiagSeqlockFail = 0;
static uint32_t g_magDiagNotValid = 0;
static uint32_t g_magDiagStale = 0;
static uint32_t g_magDiagLastSeenCount = 0;

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
            rc::rc_log("  [mag_valid=false, mag_read_count=%lu, lastAccepted=%lu]\n",
                       (unsigned long)snap.mag_read_count,
                       (unsigned long)g_lastMagReadCount);
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
// Post-Calibration Hook
// ============================================================================
// Signal Core 1 to reload calibration data after a successful save. The
// I2C-pause primitive that used to live here (cal_pre_hook) now lives
// with the pause atomics in shared_state (R-17) and is invoked directly
// by every flash_safe_execute callsite.
// cal_pre_hook is gone (R-18 cleanup); only the cal-specific reload-pending
// signal remains here, called from ao_rcos.cpp cal_save_to_flash().

void cal_post_hook() {
    if (g_sensorPhaseActive) {
        g_calReloadPending.store(true, std::memory_order_release);
    }
}
