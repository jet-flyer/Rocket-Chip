// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Health Monitor — System Health Flag Assembly (Phase 6)
//
// Module called from AO_FlightDirector at 10Hz. Reads sensor init flags,
// seqlock error counts, ESKF health, radio/flash/watchdog state.
// Assembles Go/No-Go input for ARM command validation.
//============================================================================

#include "safety/health_monitor.h"
#include "flight_director/go_nogo_checks.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"           // g_imuInitialized, g_baroInitialized, etc.
#include "fusion/eskf_runner.h"           // eskf_runner_get_eskf(), eskf_runner_is_initialized()
#include "active_objects/ao_logger.h"     // AO_Logger_get_flight_table()
#include "logging/flight_table.h"         // flight_table_count, kMaxFlightEntries
#include "calibration/calibration_manager.h"
#include "calibration/calibration_data.h" // CAL_STATUS_MAG
#include "watchdog/watchdog_recovery.h"   // rc::WatchdogRecovery

// ============================================================================
// Extern declarations -- globals owned by main.cpp (global namespace)
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;
extern rc::WatchdogRecovery g_recovery;
extern bool g_radioInitialized;  // NOLINT(readability-redundant-declaration)

namespace rc {

// ============================================================================
// Static state
// ============================================================================

static HealthState g_health{};

// ============================================================================
// health_monitor_init()
// ============================================================================

void health_monitor_init() {
    g_health = {};
    // Set initial flags from current init state
    uint8_t flags = 0;
    if (g_imuInitialized)  { flags |= kHealthImuOk; }
    if (g_baroInitialized) { flags |= kHealthBaroOk; }
    if (g_gpsInitialized)  { flags |= kHealthGpsOk; }
    if (g_radioInitialized){ flags |= kHealthRadioOk; }
    if (eskf_runner_is_initialized() &&
        eskf_runner_get_eskf()->healthy()) {
        flags |= kHealthEskfOk;
    }
    {
        const FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded && (flight_table_count(ft) < kMaxFlightEntries)) {
            flags |= kHealthFlashOk;
        }
    }
    if (!g_recovery.boot_state.safe_mode && !g_recovery.eskf_disabled) {
        flags |= kHealthWatchdogOk;
    }
    g_health.flags = flags;
    g_health.prev_flags = flags;
}

// ============================================================================
// health_monitor_tick()
// ============================================================================

bool health_monitor_tick() {
    g_health.prev_flags = g_health.flags;

    // Read sensor snapshot for validity flags
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    uint8_t flags = 0;

    // IMU: init OK + recent data valid
    if (g_imuInitialized && snap.accel_valid) {
        flags |= kHealthImuOk;
    }

    // Baro: init OK + recent data valid
    if (g_baroInitialized && snap.baro_valid) {
        flags |= kHealthBaroOk;
    }

    // GPS: init OK + valid fix
    if (g_gpsInitialized && snap.gps_fix_type >= 2 && snap.gps_satellites >= 4) {
        flags |= kHealthGpsOk;
    }

    // Radio: init OK
    if (g_radioInitialized) {
        flags |= kHealthRadioOk;
    }

    // ESKF: initialized + healthy
    if (eskf_runner_is_initialized() &&
        eskf_runner_get_eskf()->healthy()) {
        flags |= kHealthEskfOk;
    }

    // Flash: flight table loaded + space remaining
    {
        const FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded && (flight_table_count(ft) < kMaxFlightEntries)) {
            flags |= kHealthFlashOk;
        }
    }

    // Watchdog: no safe-mode, no ESKF disabled
    if (!g_recovery.boot_state.safe_mode && !g_recovery.eskf_disabled) {
        flags |= kHealthWatchdogOk;
    }

    g_health.flags = flags;

    // Tier-1 Go/No-Go ready: IMU + baro + ESKF + flash + watchdog + no launch_abort
    g_health.go_nogo_ready =
        ((flags & kHealthImuOk) != 0) &&
        ((flags & kHealthBaroOk) != 0) &&
        ((flags & kHealthEskfOk) != 0) &&
        ((flags & kHealthFlashOk) != 0) &&
        ((flags & kHealthWatchdogOk) != 0) &&
        !g_recovery.launch_abort;

    return g_health.flags != g_health.prev_flags;
}

// ============================================================================
// health_monitor_get_state()
// ============================================================================

const HealthState* health_monitor_get_state() {
    return &g_health;
}

// ============================================================================
// health_monitor_fill_go_nogo()
// ============================================================================

void health_monitor_fill_go_nogo(GoNoGoInput* gng) {
    // Read fresh sensor snapshot for GPS specifics
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    // Tier 1: Platform -- derive from health flags + recovery state
    gng->imu_healthy     = (g_health.flags & kHealthImuOk) != 0;
    gng->baro_healthy    = (g_health.flags & kHealthBaroOk) != 0;
    gng->eskf_healthy    = (g_health.flags & kHealthEskfOk) != 0;
    gng->flash_available = (g_health.flags & kHealthFlashOk) != 0;
    gng->launch_abort    = g_recovery.launch_abort;
    gng->watchdog_ok     = (g_health.flags & kHealthWatchdogOk) != 0;

    // Tier 2: Profile -- GPS needs fresh snapshot, others from health/cal state
    gng->gps_has_lock = g_gpsInitialized &&
                        snap.gps_fix_type >= 2 &&
                        snap.gps_satellites >= 4;

    const calibration_store_t* cal = calibration_manager_get();
    gng->mag_calibrated = (cal->cal_flags & CAL_STATUS_MAG) != 0;
    gng->radio_linked   = (g_health.flags & kHealthRadioOk) != 0;
}

} // namespace rc
