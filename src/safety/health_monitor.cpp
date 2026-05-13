// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Health Monitor — Centralized System Health (Stage 13, IVP-104)
//
// 2-bit per-subsystem encoding with sliding window degraded detection.
// Council-reviewed: fault latch during ARMED→DESCENT, auto-recover in
// IDLE/LANDED. Fault-to-healthy transitions emit log event.
//
// Called from AO_HealthMonitor at 10Hz.
//============================================================================

#include "safety/health_monitor.h"
#include "safety/crash_record.h"
#include "drivers/mcu_temp.h"
#include "safety/pio_watchdog.h"
#include "flight_director/go_nogo_checks.h"
#include "flight_director/flight_state.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"           // g_imuInitialized, g_baroInitialized, etc.
#include "fusion/eskf_runner.h"           // eskf_runner_get_eskf(), eskf_runner_is_initialized()
#include "fusion/confidence_gate.h"       // eskf_runner_get_confidence()
#include "active_objects/ao_logger.h"     // AO_Logger_get_flight_table()
#include "active_objects/ao_rf_manager.h" // AO_RfManager_get_state (T14 pre-arm)
#include "active_objects/ao_radio.h"      // AO_Radio_get_state()
#include "logging/flight_table.h"         // flight_table_count, kMaxFlightEntries
#include "calibration/calibration_manager.h"
#include "calibration/calibration_data.h" // CAL_STATUS_MAG
#include "flight_director/flight_director.h"  // flight_director_launch_abort()
#include "fusion/eskf_runner.h"           // eskf_is_disabled()
#include "rocketchip/config.h"            // DBG_PRINT
#include "rocketchip/job_capabilities.h"  // job::kRoleSamplesCore1, kRoleRunsLogger

// ============================================================================
// Extern declarations -- globals owned by main.cpp (global namespace)
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;

namespace rc {

// ============================================================================
// Static state
// ============================================================================

static HealthState g_health{};
static uint8_t g_currentPhase = 0;  // FlightPhase::kIdle

// Sliding windows for degraded detection (circular buffers)
static uint8_t g_imuWindow[kHealthWindowSize]{};    // 1 = valid, 0 = invalid
static uint8_t g_baroWindow[kHealthWindowSize]{};
static uint8_t g_windowIndex = 0;

// Latched fault state (per subsystem, during ARMED→DESCENT)
static uint8_t g_latchedPrimary = 0;  // bits set = fault latched for that subsystem
static bool g_mcuFaultLatched = false;  // MCU has its own latch bit (out-of-band)
// R-3 (audit 2026-05-07): set true by health_monitor_init() when the
// preserved-SRAM crash record from a prior boot is detected. Latched until
// health_monitor_clear_latches(). Surfaces as kHealthCriticalPriorHardfault
// in the critical byte.
static bool g_priorHardfaultLatched = false;

// MCU temp hysteresis state — current HealthLevel we're holding onto
// for the MCU slot. Re-evaluated each tick with 2 °C hysteresis windows
// around each threshold so noise near a boundary doesn't flicker.
static HealthLevel g_mcuTempLevel = kHealthAbsent;

// Consecutive-fault persistence counters (IVP-142b-3). Incremented each
// tick a subsystem is in kHealthFault; reset to 0 on any non-Fault tick.
// Saturate at kCriticalFaultPersistTicks so the counters don't wrap.
static uint8_t g_imuFaultTicks  = 0;
static uint8_t g_baroFaultTicks = 0;
static uint8_t g_eskfFaultTicks = 0;

// ============================================================================
// Internal: check if current phase requires fault latch
// ============================================================================

static bool phase_requires_fault_latch() {
    // Latch faults in IDLE (pre-launch) and ARMED through DESCENT.
    // Only LANDED clears latches (or explicit manual reset).
    // Rationale: a sensor that faults pre-launch may have a loose connector —
    // silent recovery gives false confidence. Can't check mid-flight.
    auto phase = static_cast<FlightPhase>(g_currentPhase);
    return phase != FlightPhase::kLanded;
}

// ============================================================================
// Internal: count valid ticks in sliding window
// ============================================================================

static uint8_t window_valid_count(const uint8_t* window) {
    uint8_t count = 0;
    for (uint8_t i = 0; i < kHealthWindowSize; ++i) {
        count += window[i];
    }
    return count;
}

// ============================================================================
// Internal: evaluate single subsystem with fault latch
// ============================================================================

static HealthLevel apply_fault_latch(HealthLevel measured, uint8_t shift) {
    uint8_t mask = static_cast<uint8_t>(kHealthMask2bit << shift);

    if (measured == kHealthFault) {
        // Record latch
        g_latchedPrimary |= mask;
    }

    if (phase_requires_fault_latch() && (g_latchedPrimary & mask) != 0) {
        // Fault latched mid-flight — stay in fault regardless of measured
        return kHealthFault;
    }

    if (!phase_requires_fault_latch()) {
        // Clear latch in IDLE/LANDED
        g_latchedPrimary &= static_cast<uint8_t>(~mask);
    }

    return measured;
}

// MCU has its own latch path since it lives outside the primary byte.
// Same semantics as apply_fault_latch but with a dedicated bool.
static HealthLevel apply_mcu_fault_latch(HealthLevel measured) {
    if (measured == kHealthFault) {
        g_mcuFaultLatched = true;
    }
    if (phase_requires_fault_latch() && g_mcuFaultLatched) {
        return kHealthFault;
    }
    if (!phase_requires_fault_latch()) {
        g_mcuFaultLatched = false;
    }
    return measured;
}

// ============================================================================
// Internal: evaluate IMU health
// ============================================================================

static HealthLevel evaluate_imu(const shared_sensor_data_t& snap) {
    if (!g_imuInitialized) {
        return kHealthAbsent;
    }

    // Record validity in sliding window
    g_imuWindow[g_windowIndex] = snap.accel_valid ? 1U : 0U;

    uint8_t validCount = window_valid_count(g_imuWindow);
    uint8_t invalidCount = kHealthWindowSize - validCount;

    if (!snap.accel_valid && invalidCount >= kHealthWindowSize) {
        return kHealthFault;  // All invalid = fault
    }
    if (invalidCount >= kImuDegradeThreshold) {
        return kHealthDegraded;
    }
    return kHealthOk;
}

// ============================================================================
// Internal: evaluate baro health
// ============================================================================

static HealthLevel evaluate_baro(const shared_sensor_data_t& snap) {
    if (!g_baroInitialized) {
        return kHealthAbsent;
    }

    g_baroWindow[g_windowIndex] = snap.baro_valid ? 1U : 0U;

    uint8_t validCount = window_valid_count(g_baroWindow);
    uint8_t invalidCount = kHealthWindowSize - validCount;

    if (!snap.baro_valid && invalidCount >= kHealthWindowSize) {
        return kHealthFault;
    }
    if (invalidCount >= kBaroDegradeThreshold) {
        return kHealthDegraded;
    }
    return kHealthOk;
}

// ============================================================================
// Internal: evaluate ESKF health
// ============================================================================

static HealthLevel evaluate_eskf() {
    if (!eskf_runner_is_initialized()) {
        return kHealthAbsent;
    }

    const ESKF* eskf = eskf_runner_get_eskf();
    if (!eskf->healthy()) {
        return kHealthFault;
    }

    // Degraded = healthy but confidence gate uncertain
    const ConfidenceState* conf = eskf_runner_get_confidence();
    if (!conf->confident) {
        return kHealthDegraded;
    }

    return kHealthOk;
}

// ============================================================================
// Internal: evaluate MCU die-temp health (Stage 16C IVP-142b-1)
//
// Hysteresis: 2 °C below each threshold for the fall-back edge.
//   Rising:   OK -> DEGRADED at  70.0,  DEGRADED -> FAULT at  85.0
//   Falling:  FAULT -> DEGRADED at 83.0, DEGRADED -> OK at    68.0
// Safe-mode threshold (105 °C) is evaluated here as FAULT; the actual
// SIG_MCU_OVERTEMP post + FD ABORT wiring lives in IVP-142b-2.
//
// Stuck sensor (mcu_temp_is_stuck() true) -> DEGRADED, never OK. Does
// not escalate to FAULT because a stuck reading is an instrumentation
// issue, not a real over-temperature condition.
// ============================================================================
// Stateful wrapper — pulls prev from g_mcuTempLevel, calls the pure
// classifier in the header, and layers stuck-sensor clamp on top.
static HealthLevel evaluate_mcu_temp(const shared_sensor_data_t& snap) {
    if (!mcu_temp_available() || snap.mcu_temp_read_count == 0) {
        return kHealthAbsent;
    }

    HealthLevel next = mcu_temp_classify(g_mcuTempLevel, snap.mcu_die_temp_c);

    // Stuck sensor clamps to at most DEGRADED — never promote a frozen
    // reading to OK even if the value happens to sit below WARN.
    if (mcu_temp_is_stuck() && next == kHealthOk) {
        next = kHealthDegraded;
    }

    g_mcuTempLevel = next;
    return next;
}

// ============================================================================
// Internal: evaluate GPS health
// ============================================================================

static HealthLevel evaluate_gps(const shared_sensor_data_t& snap) {
    if (!g_gpsInitialized) {
        return kHealthAbsent;
    }

    // Init but no NMEA data yet = absent (not fault). IVP-142c revisits
    // Stage 13 council amendment 4: calling read_count==0 a "fault"
    // conflated "no data yet" with "data stopped." The station's GPS
    // update runs in the idle bridge on Core 0 and seeds the seqlock
    // slightly after the first AO_HealthMonitor tick; naming that
    // startup race a fault was latching GPS stuck at fault on station
    // boot. The real fault case ("had data, stopped") is caught by
    // the persistence counter in health_monitor_critical_fault()
    // (IVP-142b-3) once it's been running.
    if (snap.gps_read_count == 0) {
        return kHealthAbsent;
    }

    // Degraded = NMEA flowing but no usable fix
    if (snap.gps_fix_type < 2 || snap.gps_satellites < 4) {
        return kHealthDegraded;
    }

    return kHealthOk;
}

// ============================================================================
// health_monitor_init()
// ============================================================================

void health_monitor_init() {
    g_health = {};
    g_currentPhase = 0;
    g_windowIndex = 0;
    g_latchedPrimary = 0;
    g_mcuFaultLatched = false;
    g_priorHardfaultLatched = false;
    g_mcuTempLevel = kHealthAbsent;
    g_imuFaultTicks = 0;
    g_baroFaultTicks = 0;
    g_eskfFaultTicks = 0;

    // Pre-fill sliding windows as valid (optimistic at boot)
    for (uint8_t i = 0; i < kHealthWindowSize; ++i) {
        g_imuWindow[i] = 1;
        g_baroWindow[i] = 1;
    }

    // R-3 (audit 2026-05-07): consume any prior-boot crash record. If present,
    // latch kHealthCriticalPriorHardfault so the existing safe-mode / FAULT-
    // health pivot owns the recovery path. Clear-on-IDLE per the existing
    // latch convention.
    CrashRecord prior{};
    if (crash_record_consume_prior(&prior)) {
        g_priorHardfaultLatched = true;
        const char* reason_str = "unknown";
        switch (static_cast<CrashReason>(prior.reason)) {
            case kCrashReasonMemManage:     reason_str = "MemManage";     break;
            case kCrashReasonMpuConfigFail: reason_str = "MpuConfigFail"; break;
            case kCrashReasonCore1BootWait: reason_str = "Core1BootWait"; break;
            case kCrashReasonNone:          reason_str = "none";          break;
        }
        DBG_PRINT("HEALTH: prior-boot hardfault (%s) cfsr=0x%08lx hfsr=0x%08lx pc=0x%08lx lr=0x%08lx",
                  reason_str,
                  static_cast<unsigned long>(prior.cfsr),
                  static_cast<unsigned long>(prior.hfsr),
                  static_cast<unsigned long>(prior.stacked_pc),
                  static_cast<unsigned long>(prior.stacked_lr));
    }

    // Do one initial evaluation
    health_monitor_tick();
}

// ============================================================================
// Core 1 vitality tracking (IVP-117)
//
// At 10Hz, a 500ms stall = 5 ticks without core1_loop_count advancing.
// 6 ticks provides a small margin against normal jitter.
// ============================================================================
static constexpr uint8_t kCore1StallThreshold10Hz = 6U;
static uint32_t g_lastCore1Count = 0;
static uint8_t  g_core1StallTicks = 0;

// ============================================================================
// Internal: check Core 1 vitality from seqlock snapshot
// Returns true if Core 1 is healthy (loop counter advancing).
// ============================================================================

static bool check_core1_vitality() {
    // Capability gate (IVP-142c A1, council 2026-04-18): on roles that
    // don't drive Core 1 into a sampling loop (station, relay), the
    // stall check is meaningless — core1_loop_count never advances by
    // design. Return true so kHealthCore1Ok doesn't oscillate off and
    // pollute secondary/notify consumers.
    if constexpr (!job::kRoleSamplesCore1) {
        return true;
    }

    shared_sensor_data_t snap{};
    if (!seqlock_read(&g_sensorSeqlock, &snap)) {
        return true;  // Can't read — assume alive (seqlock race, transient)
    }
    if (snap.core1_loop_count != g_lastCore1Count) {
        g_lastCore1Count = snap.core1_loop_count;
        g_core1StallTicks = 0;
        return true;
    }
    if (g_core1StallTicks < kCore1StallThreshold10Hz) {
        g_core1StallTicks++;
    }
    return (g_core1StallTicks < kCore1StallThreshold10Hz);
}

// ============================================================================
// Internal: evaluate secondary 1-bit health flags
// ============================================================================

static uint8_t evaluate_secondary() {
    uint8_t secondary = 0;
    if (AO_Radio_get_state()->initialized) {
        secondary |= kHealthRadioOk;
    }
    // Capability gate (IVP-142c A1): only assert kHealthFlashOk when
    // this role actually runs AO_Logger. On roles that don't
    // (station, relay), the flight-table struct stays zero-initialized
    // and ft->loaded is perpetually false — that's a zero-init artifact,
    // not a real fault. Returning "OK by default" when Logger isn't
    // expected keeps secondary byte meaningful for notify/preflight
    // consumers without widening the bit layout.
    if constexpr (job::kRoleRunsLogger) {
        const FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded && (flight_table_count(ft) < kMaxFlightEntries)) {
            secondary |= kHealthFlashOk;
        }
    } else {
        secondary |= kHealthFlashOk;
    }
    // Health contribution: ESKF not runaway-restart-disabled. Station
    // doesn't run ESKF, so the reader is gated by capability.
    bool eskfHealthy = true;
    if constexpr (job::kRoleSamplesCore1) {
        eskfHealthy = !eskf_is_disabled();
    }
    if (eskfHealthy) {
        secondary |= kHealthWatchdogOk;
    }
    if (!pio_watchdog_fault_detected()) {
        secondary |= kHealthPioOk;
    }
    // IVP-117: Core 1 vitality — primary check lives here, LedEngine has
    // a local fallback as defense-in-depth (Council A1).
    if (check_core1_vitality()) {
        secondary |= kHealthCore1Ok;
    }
    return secondary;
}

// ============================================================================
// Internal: log fault transitions (council: JPL + Cubesat)
// ============================================================================

static void log_health_transitions(uint8_t prev, uint8_t curr) {
    static constexpr uint8_t shifts[] = {
        kHealthShiftImu, kHealthShiftBaro, kHealthShiftEskf, kHealthShiftGps
    };
    static constexpr const char* names[] = {"IMU", "Baro", "ESKF", "GPS"};
    for (uint8_t i = 0; i < 4; ++i) {
        auto p = health_get_subsystem(prev, shifts[i]);
        auto c = health_get_subsystem(curr, shifts[i]);
        if (p == kHealthFault && c >= kHealthDegraded) {
            DBG_PRINT("HEALTH: %s fault cleared -> %s",
                      names[i], c == kHealthOk ? "OK" : "DEGRADED");
        } else if (p >= kHealthDegraded && c == kHealthFault) {
            DBG_PRINT("HEALTH: %s -> FAULT", names[i]);
        }
    }
}

// ============================================================================
// Internal: evaluate HealthCritical byte (IVP-142b-2)
//
// Recomputed fresh each tick. 0x00 = no criticals.
// Bits set here are visible to downstream consumers (notify/LED, telemetry,
// preflight, log) but DO NOT trigger automatic state transitions —
// manual abort is the intentional path until flight data justifies
// wiring automatic responses.
// ============================================================================

static uint8_t evaluate_critical(const shared_sensor_data_t& snap) {
    uint8_t critical = 0;

    // MCU die temp at/above safe-mode threshold (RP2350 datasheet §1.4.3
    // abs-max junction temp 125 °C; 105 °C chosen as 20 °C margin).
    // Requires an actual sampled reading — sentinel / not-yet-captured
    // doesn't count.
    if (mcu_temp_available() &&
        snap.mcu_temp_read_count > 0 &&
        snap.mcu_die_temp_c >= kMcuTempSafeModeC) {
        critical |= kHealthCriticalMcu;
    }

    // R-3: latched prior-boot hardfault (set in health_monitor_init() if a
    // crash_record from the previous boot was consumed).
    if (g_priorHardfaultLatched) {
        critical |= kHealthCriticalPriorHardfault;
    }

    return critical;
}

static void log_critical_transitions(uint8_t prev, uint8_t curr) {
    if (prev == curr) return;
    const uint8_t rising  = static_cast<uint8_t>(curr & ~prev);
    const uint8_t falling = static_cast<uint8_t>(prev & ~curr);
    if (rising & kHealthCriticalMcu) {
        DBG_PRINT("HEALTH: MCU CRITICAL (>=%.0fC) — manual abort recommended",
                  static_cast<double>(kMcuTempSafeModeC));
    }
    if (falling & kHealthCriticalMcu) {
        DBG_PRINT("HEALTH: MCU critical cleared");
    }
    if (rising & kHealthCriticalPriorHardfault) {
        DBG_PRINT("HEALTH: PRIOR-BOOT HARDFAULT detected (see crash record)");
    }
    if (falling & kHealthCriticalPriorHardfault) {
        DBG_PRINT("HEALTH: prior-boot hardfault latch cleared");
    }
}

static void log_mcu_transition(HealthLevel prev, HealthLevel curr) {
    if (prev == curr) return;
    if (prev == kHealthFault && curr >= kHealthDegraded) {
        DBG_PRINT("HEALTH: MCU fault cleared -> %s",
                  curr == kHealthOk ? "OK" : "DEGRADED");
    } else if (prev >= kHealthDegraded && curr == kHealthFault) {
        DBG_PRINT("HEALTH: MCU -> FAULT");
    } else if (prev == kHealthOk && curr == kHealthDegraded) {
        DBG_PRINT("HEALTH: MCU -> DEGRADED");
    }
}

// ============================================================================
// Internal: evaluate all 4 primary-byte subsystems with fault-latch applied.
// Returns packed primary byte; out-params expose the individual levels the
// caller needs for persistence counters and go/no-go.
// ============================================================================

struct PrimaryLevels {
    HealthLevel imu;
    HealthLevel baro;
    HealthLevel eskf;
    HealthLevel gps;
};

static uint8_t evaluate_primary_byte(const shared_sensor_data_t& snap,
                                     PrimaryLevels& out) {
    uint8_t primary = 0;
    out.imu = apply_fault_latch(evaluate_imu(snap), kHealthShiftImu);
    primary = health_set_subsystem(primary, kHealthShiftImu, out.imu);
    out.baro = apply_fault_latch(evaluate_baro(snap), kHealthShiftBaro);
    primary = health_set_subsystem(primary, kHealthShiftBaro, out.baro);
    out.eskf = apply_fault_latch(evaluate_eskf(), kHealthShiftEskf);
    primary = health_set_subsystem(primary, kHealthShiftEskf, out.eskf);
    out.gps = apply_fault_latch(evaluate_gps(snap), kHealthShiftGps);
    primary = health_set_subsystem(primary, kHealthShiftGps, out.gps);
    return primary;
}

// ============================================================================
// Internal: log all transitions and return `changed` boolean for tick().
// ============================================================================

static bool finalize_tick_logging() {
    if (g_health.primary != g_health.prev_primary) {
        log_health_transitions(g_health.prev_primary, g_health.primary);
    }
    log_mcu_transition(g_health.prev_mcu, g_health.mcu);
    log_critical_transitions(g_health.prev_critical, g_health.critical);

    return (g_health.primary != g_health.prev_primary) ||
           (g_health.secondary != g_health.prev_secondary) ||
           (g_health.critical != g_health.prev_critical) ||
           (g_health.mcu != g_health.prev_mcu);
}

// ============================================================================
// health_monitor_tick()
// ============================================================================

bool health_monitor_tick() {
    g_health.prev_primary = g_health.primary;
    g_health.prev_secondary = g_health.secondary;
    g_health.prev_critical = g_health.critical;
    g_health.prev_mcu = g_health.mcu;
    g_health.tick_counter++;

    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    PrimaryLevels lvl{};
    uint8_t primary = evaluate_primary_byte(snap, lvl);

    // MCU die-temp lives in HealthState::mcu, not packed into primary.
    HealthLevel mcuLevel = apply_mcu_fault_latch(evaluate_mcu_temp(snap));

    // Per-subsystem persistence counters (IVP-142b-3). Delegates to the
    // pure helper in the header so host tests cover the rule without
    // needing the full hardware-dep surface of this TU.
    g_imuFaultTicks  = critical_fault_ticks_next(g_imuFaultTicks,  lvl.imu);
    g_baroFaultTicks = critical_fault_ticks_next(g_baroFaultTicks, lvl.baro);
    g_eskfFaultTicks = critical_fault_ticks_next(g_eskfFaultTicks, lvl.eskf);

    g_windowIndex = static_cast<uint8_t>((g_windowIndex + 1U) % kHealthWindowSize);

    uint8_t secondary = evaluate_secondary();

    g_health.primary = primary;
    g_health.secondary = secondary;
    g_health.critical = evaluate_critical(snap);
    g_health.mcu = mcuLevel;

    g_health.go_nogo_ready =
        (lvl.imu  >= kHealthDegraded) &&
        (lvl.baro >= kHealthDegraded) &&
        (lvl.eskf >= kHealthDegraded) &&
        ((secondary & kHealthFlashOk) != 0) &&
        ((secondary & kHealthWatchdogOk) != 0) &&
        !flight_director_launch_abort();

    return finalize_tick_logging();
}

// ============================================================================
// health_monitor_get_state()
// ============================================================================

const HealthState* health_monitor_get_state() {
    return &g_health;
}

// ============================================================================
// health_monitor_set_phase()
// ============================================================================

void health_monitor_set_phase(uint8_t phase) {
    uint8_t prevPhase = g_currentPhase;
    g_currentPhase = phase;

    // Clear fault latches only on LANDED (post-flight recovery, GPS beacon).
    // IDLE latches persist until manual clear or reboot — pre-launch safety.
    auto p = static_cast<FlightPhase>(phase);
    if (p == FlightPhase::kLanded) {
        if (g_latchedPrimary != 0 || g_mcuFaultLatched) {
            DBG_PRINT("HEALTH: clearing fault latches (phase=%s)",
                      flight_phase_name(p));
        }
        g_latchedPrimary = 0;
        g_mcuFaultLatched = false;
        // Persistence counters reset on LANDED too — a sensor that
        // faulted in DESCENT shouldn't keep post-landing consumers
        // (beacon LED, GPS-only telemetry) in a critical-fault state.
        g_imuFaultTicks  = 0;
        g_baroFaultTicks = 0;
        g_eskfFaultTicks = 0;
    }
    (void)prevPhase;  // Available for future logging
}

// ============================================================================
// health_monitor_clear_latches()
// ============================================================================

void health_monitor_clear_latches() {
    auto phase = static_cast<FlightPhase>(g_currentPhase);
    if (phase != FlightPhase::kIdle) {
        DBG_PRINT("HEALTH: latch clear ignored (phase=%s)",
                  flight_phase_name(phase));
        return;
    }
    if (g_latchedPrimary != 0 || g_mcuFaultLatched || g_priorHardfaultLatched) {
        DBG_PRINT("HEALTH: latches cleared by manual reset");
    }
    g_latchedPrimary = 0;
    g_mcuFaultLatched = false;
    g_priorHardfaultLatched = false;
    g_imuFaultTicks  = 0;
    g_baroFaultTicks = 0;
    g_eskfFaultTicks = 0;
}

// ============================================================================
// health_monitor_critical_fault()
// ============================================================================

bool health_monitor_critical_fault() {
    // Phase gate: in IDLE the go/no-go NO-GO already blocks ARM, and
    // auto-DISARM is a no-op (not armed). Return false so callers can
    // trust this signal as "fault that demands auto-action." IVP-142b
    // bits in HealthState::critical still propagate via the explicit
    // checks below — those are visibility-only and do not auto-trigger
    // state transitions, so the phase gate doesn't suppress them for
    // the humans-in-the-loop paths (preflight, telemetry, LED, log).
    auto phase = static_cast<FlightPhase>(g_currentPhase);

    // HealthState::critical bits (MCU over-temp, …) always count as a
    // critical condition — they're threshold-bound and don't benefit
    // from persistence smoothing. Keeps preflight/LED/log visibility
    // identical regardless of phase.
    if (g_health.critical != 0) {
        return true;
    }

    // Primary-byte IMU/baro/ESKF faults are persistence-gated in flight
    // phases (ARMED through LANDED) so transient noise (dust in baro
    // vent, closing a door near the pad, transient I2C NACK) doesn't
    // flash "critical" and trip false auto-DISARMs. kCriticalFaultPersistTicks
    // consecutive ticks at 10 Hz = 500 ms.
    if (phase == FlightPhase::kIdle) {
        return false;
    }

    return (g_imuFaultTicks  >= kCriticalFaultPersistTicks) ||
           (g_baroFaultTicks >= kCriticalFaultPersistTicks) ||
           (g_eskfFaultTicks >= kCriticalFaultPersistTicks);
}

// ============================================================================
// health_monitor_fill_go_nogo()
// ============================================================================

void health_monitor_fill_go_nogo(GoNoGoInput* gng) {
    // Read fresh sensor snapshot for GPS specifics
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    // Derive from 2-bit health: OK or degraded counts as healthy for Go/No-Go
    HealthLevel imu = health_imu(g_health.primary);
    HealthLevel baro = health_baro(g_health.primary);
    HealthLevel eskf = health_eskf(g_health.primary);

    gng->imu_healthy     = (imu >= kHealthDegraded);
    gng->baro_healthy    = (baro >= kHealthDegraded);
    gng->eskf_healthy    = (eskf >= kHealthDegraded);
    gng->flash_available = (g_health.secondary & kHealthFlashOk) != 0;
    gng->launch_abort    = flight_director_launch_abort();
    gng->watchdog_ok     = (g_health.secondary & kHealthWatchdogOk) != 0;

    // Tier 2: Profile -- GPS needs fresh snapshot
    gng->gps_has_lock = g_gpsInitialized &&
                        snap.gps_fix_type >= 2 &&
                        snap.gps_satellites >= 4;

    const calibration_store_t* cal = calibration_manager_get();
    gng->mag_calibrated = (cal->cal_flags & CAL_STATUS_MAG) != 0;
    gng->radio_linked   = (g_health.secondary & kHealthRadioOk) != 0;

    // Stage T Batch B IVP-T14: RF link-health for pre-arm check.
    // Reads AO_RfManager's read-only state accessor per AO Commandment V
    // (cooperative-dispatch-only; this fill_go_nogo path runs on Core 0
    // from AO_HealthMonitor or CLI tick context, both compliant).
    const RfManagerState* rf = AO_RfManager_get_state();
    if (rf != nullptr) {
        gng->rf_link_state   = static_cast<uint8_t>(rf->state);
        gng->rf_lq_pct       = rf->lq_pct;
        gng->rf_anchor_valid = rf->anchor_valid;
    } else {
        gng->rf_link_state   = 0;       // ACQ
        gng->rf_lq_pct       = 0;
        gng->rf_anchor_valid = false;
    }
}

} // namespace rc
