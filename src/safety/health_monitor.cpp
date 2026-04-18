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
#include "drivers/mcu_temp.h"
#include "safety/pio_watchdog.h"
#include "flight_director/go_nogo_checks.h"
#include "flight_director/flight_state.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"           // g_imuInitialized, g_baroInitialized, etc.
#include "fusion/eskf_runner.h"           // eskf_runner_get_eskf(), eskf_runner_is_initialized()
#include "fusion/confidence_gate.h"       // eskf_runner_get_confidence()
#include "active_objects/ao_logger.h"     // AO_Logger_get_flight_table()
#include "active_objects/ao_radio.h"      // AO_Radio_get_state()
#include "logging/flight_table.h"         // flight_table_count, kMaxFlightEntries
#include "calibration/calibration_manager.h"
#include "calibration/calibration_data.h" // CAL_STATUS_MAG
#include "watchdog/watchdog_recovery.h"   // rc::WatchdogRecovery
#include "rocketchip/config.h"            // DBG_PRINT

// ============================================================================
// Extern declarations -- globals owned by main.cpp (global namespace)
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;
extern rc::WatchdogRecovery g_recovery;

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

// MCU temp hysteresis state — current HealthLevel we're holding onto
// for the MCU slot. Re-evaluated each tick with 2 °C hysteresis windows
// around each threshold so noise near a boundary doesn't flicker.
static HealthLevel g_mcuTempLevel = kHealthAbsent;

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

    // Init but no NMEA data at all = fault (council amendment 4)
    if (snap.gps_read_count == 0) {
        return kHealthFault;
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
    g_mcuTempLevel = kHealthAbsent;

    // Pre-fill sliding windows as valid (optimistic at boot)
    for (uint8_t i = 0; i < kHealthWindowSize; ++i) {
        g_imuWindow[i] = 1;
        g_baroWindow[i] = 1;
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
    {
        const FlightTableState* ft = AO_Logger_get_flight_table();
        if (ft->loaded && (flight_table_count(ft) < kMaxFlightEntries)) {
            secondary |= kHealthFlashOk;
        }
    }
    if (!g_recovery.boot_state.safe_mode && !g_recovery.eskf_disabled) {
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
// health_monitor_tick()
// ============================================================================

bool health_monitor_tick() {
    g_health.prev_primary = g_health.primary;
    g_health.prev_secondary = g_health.secondary;
    g_health.prev_critical = g_health.critical;
    g_health.prev_mcu = g_health.mcu;
    g_health.tick_counter++;

    // Read sensor snapshot
    shared_sensor_data_t snap{};
    seqlock_read(&g_sensorSeqlock, &snap);

    // --- Primary: 4 subsystems x 2-bit (uint8_t) ---

    uint8_t primary = 0;

    HealthLevel imuLevel = apply_fault_latch(evaluate_imu(snap), kHealthShiftImu);
    primary = health_set_subsystem(primary, kHealthShiftImu, imuLevel);

    HealthLevel baroLevel = apply_fault_latch(evaluate_baro(snap), kHealthShiftBaro);
    primary = health_set_subsystem(primary, kHealthShiftBaro, baroLevel);

    HealthLevel eskfLevel = apply_fault_latch(evaluate_eskf(), kHealthShiftEskf);
    primary = health_set_subsystem(primary, kHealthShiftEskf, eskfLevel);

    HealthLevel gpsLevel = apply_fault_latch(evaluate_gps(snap), kHealthShiftGps);
    primary = health_set_subsystem(primary, kHealthShiftGps, gpsLevel);

    // MCU die-temp lives in HealthState::mcu, not packed into primary.
    HealthLevel mcuLevel = apply_mcu_fault_latch(evaluate_mcu_temp(snap));

    // Advance sliding window index
    g_windowIndex = static_cast<uint8_t>((g_windowIndex + 1U) % kHealthWindowSize);

    uint8_t secondary = evaluate_secondary();

    g_health.primary = primary;
    g_health.secondary = secondary;
    g_health.critical = evaluate_critical(snap);
    g_health.mcu = mcuLevel;

    // Go/No-Go: IMU + baro + ESKF must be OK or degraded, flash + watchdog OK
    g_health.go_nogo_ready =
        (imuLevel >= kHealthDegraded) &&
        (baroLevel >= kHealthDegraded) &&
        (eskfLevel >= kHealthDegraded) &&
        ((secondary & kHealthFlashOk) != 0) &&
        ((secondary & kHealthWatchdogOk) != 0) &&
        !g_recovery.launch_abort;

    // Log fault transitions (council: JPL + Cubesat)
    if (g_health.primary != g_health.prev_primary) {
        log_health_transitions(g_health.prev_primary, primary);
    }
    log_mcu_transition(g_health.prev_mcu, g_health.mcu);
    log_critical_transitions(g_health.prev_critical, g_health.critical);

    bool changed = (g_health.primary != g_health.prev_primary) ||
                   (g_health.secondary != g_health.prev_secondary) ||
                   (g_health.critical != g_health.prev_critical) ||
                   (g_health.mcu != g_health.prev_mcu);
    return changed;
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
    if (g_latchedPrimary != 0 || g_mcuFaultLatched) {
        DBG_PRINT("HEALTH: latches cleared by manual reset");
    }
    g_latchedPrimary = 0;
    g_mcuFaultLatched = false;
}

// ============================================================================
// health_monitor_critical_fault()
// ============================================================================

bool health_monitor_critical_fault() {
    HealthLevel imu = health_imu(g_health.primary);
    HealthLevel baro = health_baro(g_health.primary);
    HealthLevel eskf = health_eskf(g_health.primary);
    // Baro is critical: without it, no altitude-gated main deploy and ESKF
    // vertical axis drifts (LL Entry 34). Same scrub criteria as IMU/ESKF.
    //
    // IVP-142b-3 follow-up: these primary-byte checks are currently
    // single-tick — an intermittent fault reports as critical.
    // Persistence + phase gating refactor (Option D from 2026-04-18
    // council) lives in the next IVP.
    //
    // IVP-142b-2: any bit set in HealthState::critical also counts as
    // a critical condition. Doesn't auto-transition state — consumers
    // (notify/LED/telemetry/preflight) make it visible; humans decide.
    return (imu == kHealthFault) ||
           (baro == kHealthFault) ||
           (eskf == kHealthFault) ||
           (g_health.critical != 0);
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
    gng->launch_abort    = g_recovery.launch_abort;
    gng->watchdog_ok     = (g_health.secondary & kHealthWatchdogOk) != 0;

    // Tier 2: Profile -- GPS needs fresh snapshot
    gng->gps_has_lock = g_gpsInitialized &&
                        snap.gps_fix_type >= 2 &&
                        snap.gps_satellites >= 4;

    const calibration_store_t* cal = calibration_manager_get();
    gng->mag_calibrated = (cal->cal_flags & CAL_STATUS_MAG) != 0;
    gng->radio_linked   = (g_health.secondary & kHealthRadioOk) != 0;
}

} // namespace rc
