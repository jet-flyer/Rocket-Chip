// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_FlightDirector — Flight Director Active Object
//
// Owns FlightDirector QHsm, tick logic, guard evaluation, CLI commands.
//============================================================================

#include "ao_flight_director.h"
#include "ao_logger.h"
#include "rocketchip/ao_signals.h"
#include "rocketchip/sensor_seqlock.h"
#include "rocketchip/fused_state.h"
#include "rocketchip/pcm_frame.h"       // LogEventId
#include "flight_director/action_executor.h"  // kLedPhaseBeacon
#include "flight_director/flight_director.h"
#include "flight_director/command_handler.h"
#include "flight_director/go_nogo_checks.h"
#include "safety/test_mode.h"          // R-25-exec: phase-accessor registration + IDLE-exit clearing
#include "flight_director/mission_profile_data.h"
#include "safety/pio_backup_timer.h"
#include "safety/health_monitor.h"
#include "fusion/eskf_runner.h"

#include "pico/time.h"
#include "rocketchip/rc_log.h"
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
// Health monitor moved to AO_HealthMonitor (IVP-105)

// ============================================================================
// FdAo struct — owns the FlightDirector instance
// ============================================================================
struct FdAo {
    QActive super;
    QTimeEvt tick_timer;            // 100Hz (every 1 tick at 100Hz base)
    rc::FlightDirector director;    // Flight Director QHsm
    bool initialized;               // true after ctor + init + callback wiring
    uint32_t last_tick_ms;          // Rate limiter for 100Hz tick
    // health_tick_count removed — health monitor is now AO_HealthMonitor (IVP-105)
    bool pio_drogue_reported;       // PIO backup drogue fire already published
    bool pio_main_reported;         // PIO backup main fire already published
};

static FdAo g_fdAo;

// Queue depth 32: tick events accumulate while telemetry_radio_tick() blocks
// in QV_onIdle (rfm95w_send polls DIO0 for 50-150ms LoRa airtime). At 100Hz,
// 150ms = 15 events. Depth 32 gives 2x margin. Real fix: non-blocking LoRa
// driver (see whiteboard deferred notes). (A6, revised after HW test)
static QEvtPtr g_fdAoQueue[32];

// ============================================================================
// Extern declarations — globals owned by main.cpp, read here.
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;

// ============================================================================
// Forward declarations (QP state handlers)
// ============================================================================
static QState fd_ao_initial(FdAo * const me, QEvt const * const e);
static QState fd_ao_running(FdAo * const me, QEvt const * const e);

// ============================================================================
// PIO backup timer fire detection — publish SIG_PYRO_FIRED (source=1: PIO)
// ============================================================================
static void fd_check_pio_backup(FdAo* me) {
    if (!me->pio_drogue_reported &&
        rc::pio_backup_timer_fired(rc::BackupTimerId::kDrogue)) {
        me->pio_drogue_reported = true;
        me->director.state.drogue_fired = true;
        static rc::PyroFiredEvt g_pioDrogueEvt;
        g_pioDrogueEvt.super.sig = rc::SIG_PYRO_FIRED;
        g_pioDrogueEvt.channel = 0;  // Drogue
        g_pioDrogueEvt.source = 1;   // PIO backup
        QActive_publish_(&g_pioDrogueEvt.super, &me->super, me->super.prio);
        rc::rc_log("[PIO] Backup drogue fired — SIG_PYRO_FIRED published\n");
    }
    if (!me->pio_main_reported &&
        rc::pio_backup_timer_fired(rc::BackupTimerId::kMain)) {
        me->pio_main_reported = true;
        me->director.state.main_fired = true;
        static rc::PyroFiredEvt g_pioMainEvt;
        g_pioMainEvt.super.sig = rc::SIG_PYRO_FIRED;
        g_pioMainEvt.channel = 1;  // Main
        g_pioMainEvt.source = 1;   // PIO backup
        QActive_publish_(&g_pioMainEvt.super, &me->super, me->super.prio);
        rc::rc_log("[PIO] Backup main fired — SIG_PYRO_FIRED published\n");
    }
}

// ============================================================================
// Tick logic
// ============================================================================
static void fd_tick(FdAo* me) {
    if (!me->initialized) {
        return;
    }

    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    // 100Hz tick (10ms period) — matches guard evaluation rate per plan
    if (now_ms - me->last_tick_ms < kFlightDirectorPeriodMs) {
        return;
    }
    me->last_tick_ms = now_ms;

    rc::flight_director_dispatch_tick(&me->director, now_ms);

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

    // Auto-DISARM on critical sensor fault while ARMED (Stage 13 safety)
    // If IMU or ESKF faults on the pad, disarm and latch launch_abort.
    // Launch abort is level 3 in the safety state model — power-cycle-only
    // clear. See docs/USER_GUIDE.md "Safety State Model".
    if (rc::flight_director_phase(&me->director) == rc::FlightPhase::kArmed &&
        rc::health_monitor_critical_fault()) {
        rc::rc_log("[FD] CRITICAL FAULT while ARMED — auto-DISARM + LAUNCH ABORT\n");
        rc::flight_director_dispatch_signal(&me->director, rc::SIG_DISARM);
        rc::flight_director_set_launch_abort();
    }

    fd_check_pio_backup(me);
}

// ============================================================================
// QP State Handlers
// ============================================================================

static QState fd_ao_initial(FdAo * const me, QEvt const * const e) {
    (void)e;
    // 100Hz tick (every 1 tick at 100Hz base)
    QTimeEvt_armX(&me->tick_timer, 1U, 1U);
    return Q_TRAN(&fd_ao_running);
}

static QState fd_ao_running(FdAo * const me, QEvt const * const e) {
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

QActive * const AO_FlightDirector = &g_fdAo.super;

// Pyro fired callback — extracted to keep AO_FlightDirector_start under
// the function-size limit.
static void fd_on_pyro_fired(rc::PyroChannel ch) {
    rc::rc_log("[FD] PYRO FIRED: %s (primary)\n",
               ch == rc::PyroChannel::kDrogue ? "DROGUE" : "MAIN");
    if (ch == rc::PyroChannel::kDrogue) {
        g_fdAo.director.state.drogue_fired = true;
        AO_Logger_log_event(rc::LogEventId::kPyroFiredDrogue, 0, 0, 0, 0);
        rc::pio_backup_timer_cancel(rc::BackupTimerId::kDrogue);
    } else {
        g_fdAo.director.state.main_fired = true;
        AO_Logger_log_event(rc::LogEventId::kPyroFiredMain, 0, 0, 0, 0);
        rc::pio_backup_timer_cancel(rc::BackupTimerId::kMain);
    }
    // Publish SIG_PYRO_FIRED for AO subscribers (source=0: FD primary)
    static rc::PyroFiredEvt g_pyroEvt;
    g_pyroEvt.super.sig = rc::SIG_PYRO_FIRED;
    g_pyroEvt.channel = static_cast<uint8_t>(ch);
    g_pyroEvt.source = 0;  // Primary (FD-commanded)
    QActive_publish_(&g_pyroEvt.super,
                     &g_fdAo.super, g_fdAo.super.prio);
}

// R-25-exec: register phase accessor for test_mode_evaluate's three-
// condition AND gate (condition (b): current phase == kIdle). Lambda
// calls flight_director_phase() against the module-local director
// instance. Extracted from AO_FlightDirector_start to keep that
// function under JSF AV Rule 1 line-count limit.
static void fd_register_test_mode_accessor() {
    rc::test_mode_register_phase_accessor([]() {
        return rc::flight_director_phase(&g_fdAo.director);
    });
}

// Wire up FlightDirector C-style callbacks (HSM is C; AO is C++).
// Extracted from AO_FlightDirector_start to keep that function under
// JSF AV Rule 1 line-count limit.
//
// Callback responsibilities:
//   set_led_cb       — beacon one-shot (IVP-116 — phase LED routing
//                      lives in AO_Notify; this hook is beacon only).
//   phase_change_cb  — publish SIG_PHASE_CHANGE + R-25-exec council
//                      amendment #2 (clear test_mode on IDLE-exit).
//   log_pyro_cb      — pyro fired logger.
//   beacon_cb        — IVP-121 distress beacon on MAIN_DESCENT timeout.
static void fd_wire_callbacks(rc::FlightDirector* director) {
    director->set_led_cb = [](uint8_t val) {
        if (val == rc::kLedPhaseBeacon) {
            static QEvt g_beaconEvt;
            g_beaconEvt.sig = rc::SIG_BEACON_ACTIVE;
            QActive_publish_(&g_beaconEvt,
                             &g_fdAo.super, g_fdAo.super.prio);
        }
    };
    director->phase_change_cb = [](rc::FlightPhase phase, uint32_t ts_ms) {
        if (phase != rc::FlightPhase::kIdle) {
            rc::test_mode_clear_on_idle_exit();
        }
        static rc::PhaseChangeEvt g_evt;
        g_evt.super.sig = rc::SIG_PHASE_CHANGE;
        g_evt.phase = static_cast<uint8_t>(phase);
        g_evt.timestamp_ms = ts_ms;
        QActive_publish_(&g_evt.super,
                         &g_fdAo.super, g_fdAo.super.prio);
    };
    director->log_pyro_cb = fd_on_pyro_fired;
    director->beacon_cb = []() {
        static QEvt g_beaconEvt;
        g_beaconEvt.sig = rc::SIG_BEACON_ACTIVE;
        QActive_publish_(&g_beaconEvt,
                         &g_fdAo.super, g_fdAo.super.prio);
        rc::rc_log("[FD] Distress beacon published (SIG_BEACON_ACTIVE)\n");
    };
    director->reset_subsystems_cb = []() {
        eskf_runner_request_reinit();
    };
}

void AO_FlightDirector_start(uint8_t prio) {
    FdAo* me = &g_fdAo;

    // --- Initialize FlightDirector QHsm + wire callbacks ---
    rc::flight_director_ctor(&me->director, &rc::kDefaultRocketProfile);
    fd_wire_callbacks(&me->director);
    rc::flight_director_init(&me->director);
    me->initialized = true;
    me->last_tick_ms = 0;
    me->pio_drogue_reported = false;
    me->pio_main_reported = false;

    // R-25-exec: register phase accessor for test_mode_evaluate's
    // three-condition AND gate. See fd_register_test_mode_accessor.
    fd_register_test_mode_accessor();

    // --- Start QP Active Object ---
    QActive_ctor(&me->super, Q_STATE_CAST(&fd_ao_initial));
    QTimeEvt_ctorX(&me->tick_timer, &me->super,
                   SIG_FD_TICK_TIMER, 0U);
    QActive_start(&me->super,
                  Q_PRIO(prio, 0U),
                  g_fdAoQueue,
                  Q_DIM(g_fdAoQueue),
                  nullptr, 0U,
                  nullptr);
}

void AO_FlightDirector_dispatch_signal(int signal) {
    if (!g_fdAo.initialized) {
        rc::rc_log("[FD] Flight Director not initialized.\n");
        return;
    }
    rc::flight_director_dispatch_signal(&g_fdAo.director,
                                         static_cast<rc::FlightSignal>(signal));
}

bool AO_FlightDirector_process_command(int cmd) {
    if (!g_fdAo.initialized) {
        rc::rc_log("[FD] Flight Director not initialized.\n");
        return false;
    }

    auto cmd_type = static_cast<rc::CommandType>(cmd);
    rc::FlightPhase phase = rc::flight_director_phase(&g_fdAo.director);

    // Build Go/No-Go input from health monitor
    rc::GoNoGoInput gng{};
    rc::health_monitor_fill_go_nogo(&gng);

    rc::CommandResult result = rc::command_handler_validate(
        cmd_type, phase, &gng);

    if (result.accepted) {
        rc::flight_director_dispatch_signal(&g_fdAo.director, result.signal);

        // PIO backup timer arm/disarm hooks
        if (result.signal == rc::SIG_ARM) {
            const auto* p = g_fdAo.director.profile;
            rc::pio_backup_timer_arm(p->drogue_timer_s, p->main_timer_s);
            rc::rc_log("[PIO] Backup timers armed: drogue=%.0fs main=%.0fs\n",
                       static_cast<double>(p->drogue_timer_s),
                       static_cast<double>(p->main_timer_s));
            eskf_runner_end_mahony_startup();
        } else if (result.signal == rc::SIG_ABORT) {
            AO_Logger_log_event(rc::LogEventId::kAbortTriggered,
                             static_cast<uint8_t>(phase), 0, 0, 0);
        } else if (result.signal == rc::SIG_DISARM ||
                   result.signal == rc::SIG_RESET) {
            rc::pio_backup_timer_disarm();
            rc::rc_log("[PIO] Backup timers disarmed\n");
        }
    } else {
        rc::rc_log("[FD] Command rejected: %s\n", result.reason);
    }
    return result.accepted;
}

void AO_FlightDirector_print_status() {
    if (!g_fdAo.initialized) {
        rc::rc_log("[FD] Flight Director not initialized.\n");
        return;
    }
    const rc::FlightState& st = g_fdAo.director.state;
    uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    uint32_t phase_ms = now_ms - st.phase_entry_ms;

    rc::rc_log("\n--- Flight Director Status ---\n");
    rc::rc_log("  Profile:     %s\n", g_fdAo.director.profile->name);
    rc::rc_log("  Phase:       %s\n", rc::flight_phase_name(st.current_phase));
    rc::rc_log("  Previous:    %s\n", rc::flight_phase_name(st.previous_phase));
    rc::rc_log("  In-phase:    %lu ms\n", (unsigned long)phase_ms);
    rc::rc_log("  Transitions: %lu\n", (unsigned long)st.transition_count);

    // Markers
    const rc::FlightMarkers& mk = st.markers;
    if (mk.armed_ms > 0) {
        rc::rc_log("  Armed:       T+%lu ms\n", (unsigned long)mk.armed_ms);
    }
    if (mk.launch_ms > 0) {
        rc::rc_log("  Launch:      T+%lu ms\n", (unsigned long)mk.launch_ms);
    }
    if (mk.burnout_ms > 0) {
        rc::rc_log("  Burnout:     T+%lu ms\n", (unsigned long)mk.burnout_ms);
    }
    if (mk.apogee_ms > 0) {
        rc::rc_log("  Apogee:      T+%lu ms\n", (unsigned long)mk.apogee_ms);
    }
    if (mk.drogue_deploy_ms > 0) {
        rc::rc_log("  Drogue:      T+%lu ms\n", (unsigned long)mk.drogue_deploy_ms);
    }
    if (mk.main_deploy_ms > 0) {
        rc::rc_log("  Main:        T+%lu ms\n", (unsigned long)mk.main_deploy_ms);
    }
    if (mk.landing_ms > 0) {
        rc::rc_log("  Landing:     T+%lu ms\n", (unsigned long)mk.landing_ms);
    }
    if (mk.abort_ms > 0) {
        rc::rc_log("  Abort:       T+%lu ms\n", (unsigned long)mk.abort_ms);
    }
    rc::rc_log("-----------------------------\n");
}

const rc::FlightDirector* AO_FlightDirector_get_director() {
    return &g_fdAo.director;
}

bool AO_FlightDirector_is_initialized() {
    return g_fdAo.initialized;
}

bool AO_FlightDirector_is_ground_state() {
    if (!g_fdAo.initialized) {
        // Pre-init: whatever the FD will eventually be, we're definitely
        // not in flight. Treat as ground.
        return true;
    }
    return rc::flight_director_phase(&g_fdAo.director) == rc::FlightPhase::kIdle;
}
