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
#include "ao_led_engine.h"
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
#include "safety/health_monitor.h"

#include "pico/time.h"
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
static constexpr uint8_t kHealthTickDivider = 10;        // 100Hz / 10 = 10Hz

// ============================================================================
// FdAo struct — owns the FlightDirector instance
// ============================================================================
struct FdAo {
    QActive super;
    QTimeEvt tick_timer;            // 100Hz (every 1 tick at 100Hz base)
    rc::FlightDirector director;    // Flight Director QHsm (Phase 3: owned here)
    bool initialized;               // true after ctor + init + callback wiring
    uint32_t last_tick_ms;          // Rate limiter for 100Hz tick
    uint8_t health_tick_count;      // Divider counter for 10Hz health monitor
    bool pio_drogue_reported;       // PIO backup drogue fire already published
    bool pio_main_reported;         // PIO backup main fire already published
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

// Phase 4: Flight table, populate_fused_state, log_flight_event moved to AO_Logger.

// ============================================================================
// Forward declarations (QP state handlers)
// ============================================================================
static QState FdAo_initial(FdAo * const me, QEvt const * const e);
static QState FdAo_running(FdAo * const me, QEvt const * const e);

// ============================================================================
// PIO backup timer fire detection — publish SIG_PYRO_FIRED (source=1: PIO)
// ============================================================================
static void fd_check_pio_backup(FdAo* me) {
    if (!me->pio_drogue_reported &&
        rc::pio_backup_timer_fired(rc::BackupTimerId::kDrogue)) {
        me->pio_drogue_reported = true;
        me->director.state.drogue_fired = true;
        static rc::PyroFiredEvt pioDrogueEvt;
        pioDrogueEvt.super.sig = rc::SIG_PYRO_FIRED;
        pioDrogueEvt.channel = 0;  // Drogue
        pioDrogueEvt.source = 1;   // PIO backup
        QActive_publish_(&pioDrogueEvt.super, &me->super, me->super.prio);
        printf("[PIO] Backup drogue fired — SIG_PYRO_FIRED published\n");
    }
    if (!me->pio_main_reported &&
        rc::pio_backup_timer_fired(rc::BackupTimerId::kMain)) {
        me->pio_main_reported = true;
        me->director.state.main_fired = true;
        static rc::PyroFiredEvt pioMainEvt;
        pioMainEvt.super.sig = rc::SIG_PYRO_FIRED;
        pioMainEvt.channel = 1;  // Main
        pioMainEvt.source = 1;   // PIO backup
        QActive_publish_(&pioMainEvt.super, &me->super, me->super.prio);
        printf("[PIO] Backup main fired — SIG_PYRO_FIRED published\n");
    }
}

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

    // Health monitor at 10Hz (every 10th tick)
    me->health_tick_count++;
    if (me->health_tick_count >= kHealthTickDivider) {
        me->health_tick_count = 0;
        if (rc::health_monitor_tick()) {
            // Health flags changed — publish SIG_HEALTH_STATUS
            static rc::HealthStatusEvt healthEvt;
            healthEvt.super.sig = rc::SIG_HEALTH_STATUS;
            healthEvt.health_flags =
                rc::health_monitor_get_state()->flags;
            QActive_publish_(&healthEvt.super,
                             &me->super, me->super.prio);
        }
    }

    fd_check_pio_backup(me);
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
        AO_LedEngine_post_pattern(val);
    };
    me->director.phase_change_cb = [](rc::FlightPhase phase, uint32_t ts_ms) {
        static rc::PhaseChangeEvt evt;
        evt.super.sig = rc::SIG_PHASE_CHANGE;
        evt.phase = static_cast<uint8_t>(phase);
        evt.timestamp_ms = ts_ms;
        QActive_publish_(&evt.super,
                         &l_fdAo.super, l_fdAo.super.prio);
    };
    me->director.log_pyro_cb = [](rc::PyroChannel ch) {
        printf("[FD] PYRO FIRED: %s (primary)\n",
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
        // Publish SIG_PYRO_FIRED for AO subscribers (source=0: FD primary)
        static rc::PyroFiredEvt pyroEvt;
        pyroEvt.super.sig = rc::SIG_PYRO_FIRED;
        pyroEvt.channel = static_cast<uint8_t>(ch);
        pyroEvt.source = 0;  // Primary (FD-commanded)
        QActive_publish_(&pyroEvt.super,
                         &l_fdAo.super, l_fdAo.super.prio);
    };
    rc::flight_director_init(&me->director);
    me->initialized = true;
    me->last_tick_ms = 0;
    me->health_tick_count = 0;
    me->pio_drogue_reported = false;
    me->pio_main_reported = false;

    // Phase 6: Initialize health monitor
    rc::health_monitor_init();

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

    // Build Go/No-Go input from health monitor (Phase 6)
    rc::GoNoGoInput gng{};
    rc::health_monitor_fill_go_nogo(&gng);

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
