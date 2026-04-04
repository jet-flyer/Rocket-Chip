// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_FlightDirector — Flight Director Active Object (IVP-78, Phase 3)
//
// Phase 3 migration: owns the FlightDirector QHsm instance, tick logic,
// guard evaluation, and CLI-facing command/status functions. Moved from
// main.cpp where these were extern "C" bridge functions.
//
// Dependencies on main.cpp globals use extern declarations. These will
// migrate to their owning AOs in later phases.
//============================================================================

#include "ao_flight_director.h"
#include "ao_logger.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/fused_state.h"
#include "rocketchip/pcm_frame.h"       // LogEventId
#include "rocketchip/led_patterns.h"    // kCalNeoOff etc.
#include "flight_director/flight_director.h"
#include "flight_director/command_handler.h"
#include "flight_director/go_nogo_checks.h"
#include "flight_director/mission_profile_data.h"
#include "safety/pio_backup_timer.h"
#include "calibration/calibration_manager.h"
#include "calibration/calibration_data.h"
#include "logging/flight_table.h"
#include "core1/sensor_core1.h"         // g_imuInitialized, g_baroInitialized, etc.
#include "fusion/eskf_runner.h"         // eskf_runner_get_eskf() etc.
#include "watchdog/watchdog_recovery.h" // rc::WatchdogRecovery

#include "pico/time.h"
#include <atomic>
#include <stdio.h>
#include <math.h>

// ============================================================================
// Internal signal for the 100Hz tick (private to this AO)
// ============================================================================
enum : uint16_t {
    SIG_FD_TICK_TIMER = rc::SIG_AO_MAX + 3
};

// ============================================================================
// Flight Director period
// ============================================================================
static constexpr uint32_t kFlightDirectorPeriodMs = 10;  // 100Hz

// ============================================================================
// FdAo struct — owns the FlightDirector instance
// ============================================================================
struct FdAo {
    QActive super;
    QTimeEvt tick_timer;            // 100Hz (every 1 tick at 100Hz base)
    rc::FlightDirector director;    // Flight Director QHsm (Phase 3: owned here)
    bool initialized;               // true after ctor + init + callback wiring
    uint32_t last_tick_ms;          // Rate limiter for 100Hz tick
};

static FdAo l_fdAo;

// Queue depth 32: tick events accumulate while telemetry_radio_tick() blocks
// in QV_onIdle (rfm95w_send polls DIO0 for 50-150ms LoRa airtime). At 100Hz,
// 150ms = 15 events. Depth 32 gives 2x margin. Real fix: non-blocking LoRa
// driver (see whiteboard deferred notes). (A6, revised after HW test)
static QEvtPtr l_fdAoQueue[32];

// ============================================================================
// Extern declarations — globals owned by main.cpp, read here.
// These will migrate to their owning AOs in later phases.
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;
extern std::atomic<uint8_t> g_calNeoPixelOverride;
extern rc::WatchdogRecovery g_recovery;
extern bool g_radioInitialized;  // NOLINT(readability-redundant-declaration)

// Phase 4: Flight table, populate_fused_state, log_flight_event moved to AO_Logger.

// ============================================================================
// Forward declarations (QP state handlers)
// ============================================================================
static QState FdAo_initial(FdAo * const me, QEvt const * const e);
static QState FdAo_running(FdAo * const me, QEvt const * const e);

// ============================================================================
// Tick logic (moved from main.cpp flight_director_tick)
// ============================================================================
static void fd_tick(FdAo* me) {
    if (!me->initialized) {
        return;
    }

    uint32_t nowMs = to_ms_since_boot(get_absolute_time());
    // 100Hz tick (10ms period) — matches guard evaluation rate per plan
    if (nowMs - me->last_tick_ms < kFlightDirectorPeriodMs) {
        return;
    }
    me->last_tick_ms = nowMs;

    rc::flight_director_dispatch_tick(&me->director, nowMs);

    // Guard evaluation — read sensor snapshot and evaluate guards
    shared_sensor_data_t snap{};
    if (seqlock_read(&g_sensorSeqlock, &snap)) {
        rc::FusedState fused{};
        AO_Logger_populate_fused_state(fused, snap);

        float accel_mag = sqrtf(snap.accel_x * snap.accel_x +
                                snap.accel_y * snap.accel_y +
                                snap.accel_z * snap.accel_z);
        rc::flight_director_evaluate_guards(&me->director, fused,
                                             snap.accel_z, accel_mag);
    }
}

// ============================================================================
// QP State Handlers
// ============================================================================

static QState FdAo_initial(FdAo * const me, QEvt const * const e) {
    (void)e;
    // 100Hz tick (every 1 tick at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 1U, 1U);
    return Q_TRAN(&FdAo_running);
}

static QState FdAo_running(FdAo * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_FD_TICK_TIMER:
        fd_tick(me);
        return Q_HANDLED();
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public interface
// ============================================================================

QActive * const AO_FlightDirector = &l_fdAo.super;

void AO_FlightDirector_start(uint8_t prio) {
    FdAo* me = &l_fdAo;

    // --- Initialize FlightDirector QHsm (moved from init_flight_director) ---
    rc::flight_director_ctor(&me->director, &rc::kDefaultRocketProfile);
    me->director.set_led_cb = [](uint8_t val) {
        g_calNeoPixelOverride.store(val, std::memory_order_relaxed);
    };
    me->director.log_pyro_cb = [](rc::PyroChannel ch) {
        printf("[FD] PYRO INTENT: %s\n",
               ch == rc::PyroChannel::kDrogue ? "DROGUE" : "MAIN");
        if (ch == rc::PyroChannel::kDrogue) {
            l_fdAo.director.state.drogue_fired = true;
            AO_Logger_log_event(rc::LogEventId::kPyroFiredDrogue, 0, 0, 0, 0);
            rc::pio_backup_timer_cancel(rc::BackupTimerId::kDrogue);
        } else {
            l_fdAo.director.state.main_fired = true;
            AO_Logger_log_event(rc::LogEventId::kPyroFiredMain, 0, 0, 0, 0);
            rc::pio_backup_timer_cancel(rc::BackupTimerId::kMain);
        }
    };
    rc::flight_director_init(&me->director);
    me->initialized = true;
    me->last_tick_ms = 0;

    // --- Start QP Active Object ---
    QActive_ctor(&me->super, Q_STATE_CAST(&FdAo_initial));

    QTimeEvt_ctorX(&me->tick_timer, &me->super,
                   SIG_FD_TICK_TIMER, 0U);

    QActive_start(&me->super,
                  Q_PRIO(prio, 0U),
                  l_fdAoQueue,
                  Q_DIM(l_fdAoQueue),
                  (void *)0, 0U,
                  (void *)0);
}

void AO_FlightDirector_dispatch_signal(int signal) {
    if (!l_fdAo.initialized) {
        printf("[FD] Flight Director not initialized.\n");
        return;
    }
    rc::flight_director_dispatch_signal(&l_fdAo.director,
                                         static_cast<rc::FlightSignal>(signal));
}

bool AO_FlightDirector_process_command(int cmd) {
    if (!l_fdAo.initialized) {
        printf("[FD] Flight Director not initialized.\n");
        return false;
    }

    auto cmdType = static_cast<rc::CommandType>(cmd);
    rc::FlightPhase phase = rc::flight_director_phase(&l_fdAo.director);

    // Build Go/No-Go input from current system state
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    rc::GoNoGoInput gng{};
    // Tier 1: Platform
    gng.imu_healthy = g_imuInitialized && snap.accel_valid;
    gng.baro_healthy = g_baroInitialized && snap.baro_valid;
    gng.eskf_healthy = g_eskfInitialized && eskf_runner_get_eskf()->healthy();
    {
        const rc::FlightTableState* ft = AO_Logger_get_flight_table();
        gng.flash_available = ft->loaded &&
                              (rc::flight_table_count(ft) <
                               rc::kMaxFlightEntries);
    }
    gng.launch_abort = g_recovery.launch_abort;
    gng.watchdog_ok = !g_recovery.boot_state.safe_mode &&
                      !g_recovery.eskf_disabled;
    // Tier 2: Profile
    gng.gps_has_lock = g_gpsInitialized &&
                       snap.gps_fix_type >= 2 &&
                       snap.gps_satellites >= 4;
    const calibration_store_t* cal = calibration_manager_get();
    gng.mag_calibrated = (cal->cal_flags & CAL_STATUS_MAG) != 0;
    gng.radio_linked = g_radioInitialized;

    rc::CommandResult result = rc::command_handler_validate(
        cmdType, phase, &gng);

    if (result.accepted) {
        rc::flight_director_dispatch_signal(&l_fdAo.director, result.signal);

        // IVP-89: PIO backup timer arm/disarm hooks
        if (result.signal == rc::SIG_ARM) {
            // Start backup timers with profile values
            // TODO: read from Mission Profile once fields are wired
            rc::pio_backup_timer_arm(15.0f, 45.0f);  // VALIDATE defaults
            printf("[PIO] Backup timers armed: drogue=15s main=45s\n");
        } else if (result.signal == rc::SIG_ABORT) {
            AO_Logger_log_event(rc::LogEventId::kAbortTriggered,
                             static_cast<uint8_t>(phase), 0, 0, 0);
        } else if (result.signal == rc::SIG_DISARM ||
                   result.signal == rc::SIG_RESET) {
            rc::pio_backup_timer_disarm();
            printf("[PIO] Backup timers disarmed\n");
        }
    } else {
        printf("[FD] Command rejected: %s\n", result.reason);
    }
    return result.accepted;
}

void AO_FlightDirector_print_status() {
    if (!l_fdAo.initialized) {
        printf("[FD] Flight Director not initialized.\n");
        return;
    }
    const rc::FlightState& st = l_fdAo.director.state;
    uint32_t nowMs = to_ms_since_boot(get_absolute_time());
    uint32_t phaseMs = nowMs - st.phase_entry_ms;

    printf("\n--- Flight Director Status ---\n");
    printf("  Profile:     %s\n", l_fdAo.director.profile->name);
    printf("  Phase:       %s\n", rc::flight_phase_name(st.current_phase));
    printf("  Previous:    %s\n", rc::flight_phase_name(st.previous_phase));
    printf("  In-phase:    %lu ms\n", (unsigned long)phaseMs);
    printf("  Transitions: %lu\n", (unsigned long)st.transition_count);

    // Markers
    const rc::FlightMarkers& mk = st.markers;
    if (mk.armed_ms > 0) {
        printf("  Armed:       T+%lu ms\n", (unsigned long)mk.armed_ms);
    }
    if (mk.launch_ms > 0) {
        printf("  Launch:      T+%lu ms\n", (unsigned long)mk.launch_ms);
    }
    if (mk.burnout_ms > 0) {
        printf("  Burnout:     T+%lu ms\n", (unsigned long)mk.burnout_ms);
    }
    if (mk.apogee_ms > 0) {
        printf("  Apogee:      T+%lu ms\n", (unsigned long)mk.apogee_ms);
    }
    if (mk.drogue_deploy_ms > 0) {
        printf("  Drogue:      T+%lu ms\n", (unsigned long)mk.drogue_deploy_ms);
    }
    if (mk.main_deploy_ms > 0) {
        printf("  Main:        T+%lu ms\n", (unsigned long)mk.main_deploy_ms);
    }
    if (mk.landing_ms > 0) {
        printf("  Landing:     T+%lu ms\n", (unsigned long)mk.landing_ms);
    }
    if (mk.abort_ms > 0) {
        printf("  Abort:       T+%lu ms\n", (unsigned long)mk.abort_ms);
    }
    printf("-----------------------------\n");
}

const rc::FlightDirector* AO_FlightDirector_get_director() {
    return &l_fdAo.director;
}

bool AO_FlightDirector_is_initialized() {
    return l_fdAo.initialized;
}
