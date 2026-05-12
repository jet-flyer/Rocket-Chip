// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Health Monitor — Centralized System Health (Stage 13, IVP-104)
//
// 2-bit per-subsystem encoding: absent/fault/degraded/healthy.
// Council-reviewed: 5 personas, unanimous GO.
// Decision record: docs/decisions/HEALTH_CONTRACT.md
//
// Called from AO_HealthMonitor at 10Hz (promoted from FD module in IVP-105).
// Consumers: AO_LedEngine (fault layer), AO_Telemetry (health byte),
//            AO_Logger (FusedState), CLI preflight (pull model).
//============================================================================
#ifndef ROCKETCHIP_HEALTH_MONITOR_H
#define ROCKETCHIP_HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>

namespace rc {

// ============================================================================
// 2-bit health state per subsystem
//
// Encoding chosen so uninitialized memory (0x00) = all absent (fail-safe).
// ============================================================================
enum HealthLevel : uint8_t {
    kHealthAbsent  = 0b00,  // Not present / not initialized
    kHealthFault   = 0b01,  // Present but failing (errors, diverged)
    kHealthDegraded = 0b10, // Working but reduced quality
    kHealthOk      = 0b11,  // Fully operational
};

// ============================================================================
// Primary health byte: 4 subsystems x 2 bits
//
// bits [1:0] = IMU, [3:2] = Baro, [5:4] = ESKF, [7:6] = GPS
//
// MCU die-temp health (IVP-142b-1) is tracked separately in
// HealthState::mcu rather than widened into primary, to avoid rippling
// the 8-bit assumption through FusedState / PCM log frame /
// telemetry wire format / replay harness. Consumers that care about
// MCU read HealthState::mcu directly.
// ============================================================================
static constexpr uint8_t kHealthShiftImu  = 0;
static constexpr uint8_t kHealthShiftBaro = 2;
static constexpr uint8_t kHealthShiftEskf = 4;
static constexpr uint8_t kHealthShiftGps  = 6;
static constexpr uint8_t kHealthMask2bit  = 0x03;

// ============================================================================
// Secondary flags byte: 1-bit per subsystem
// ============================================================================
enum HealthSecondary : uint8_t {
    kHealthRadioOk    = (1 << 0),
    kHealthFlashOk    = (1 << 1),
    kHealthWatchdogOk = (1 << 2),
    kHealthPioOk      = (1 << 3),
    kHealthCore1Ok    = (1 << 4),  // IVP-117: Core 1 vitality (primary check)
};

// ============================================================================
// Critical flags byte: 1-bit per threshold-bound critical condition
//
// Distinct from HealthSecondary (which is "1-bit OK/not-OK" for absent-able
// subsystems) and distinct from HealthLevel (which is 4-state severity for
// per-subsystem quality). A bit in HealthCritical means "system-wide
// invariant violated, about to damage hardware or lose safety margin."
//
// Encoding: 0 = nominal, 1 = critical. 0x00 default = no criticals = still
// fail-safe since readers check bits individually.
//
// Writer: health_monitor_tick() only, recomputed fresh each tick from the
// underlying signals (materialized for PCM/MAVLink/FD snapshot determinism).
//
// Consumers do NOT auto-trigger state transitions from these bits — bits
// exist so humans (pilot, RSO, ground ops) see the condition via LED/audio/
// telemetry/preflight and can abort manually. Wiring automatic responses
// requires flight-data evidence that the condition is reachable (council
// 2026-04-18: "no dead safe-mode paths").
// ============================================================================
enum HealthCritical : uint8_t {
    // MCU die temperature ≥ kMcuTempSafeModeC (105 °C). RP2350 silicon
    // is at 20 °C margin below absolute-max junction temp (125 °C);
    // continued operation risks permanent damage.
    kHealthCriticalMcu              = (1 << 0),
    // Previous boot hit a hardfault (e.g., MPU stack guard, R-3 audit
    // 2026-05-07). The crash_record consumed at boot triggers this bit,
    // letting the existing safe-mode / FAULT-health pivot machinery own
    // the recovery path. Latched until health_monitor_clear_latches().
    kHealthCriticalPriorHardfault   = (1 << 1),
    // Reserved for future critical conditions (pyro-continuity short,
    // battery undervolt/overvolt, flash-write failure).
    // See docs/decisions/HEALTH_CONTRACT.md pending council note.
};

// ============================================================================
// HealthState — full system health snapshot
// ============================================================================
struct HealthState {
    uint8_t primary;          // 4 subsystems x 2-bit (HealthLevel) — IMU/Baro/ESKF/GPS
    uint8_t secondary;        // 1-bit flags (HealthSecondary)
    uint8_t critical;         // 1-bit flags (HealthCritical) — IVP-142b-2
    uint8_t prev_primary;     // Previous tick (change detection)
    uint8_t prev_secondary;
    uint8_t prev_critical;
    HealthLevel mcu;          // MCU die-temp (IVP-142b-1) — separate from
                              //   primary to keep FusedState/telemetry/PCM
                              //   layouts unchanged.
    HealthLevel prev_mcu;
    bool go_nogo_ready;       // All tier-1 checks pass
    uint8_t tick_counter;     // Staleness: incremented each tick, watchdog-readable
};

// ============================================================================
// Inline helpers for reading/writing 2-bit fields
// ============================================================================

inline HealthLevel health_get_subsystem(uint8_t primary, uint8_t shift) {
    return static_cast<HealthLevel>((primary >> shift) & kHealthMask2bit);
}

inline uint8_t health_set_subsystem(uint8_t primary, uint8_t shift, HealthLevel level) {
    primary &= static_cast<uint8_t>(~(kHealthMask2bit << shift));
    primary |= static_cast<uint8_t>(static_cast<uint8_t>(level) << shift);
    return primary;
}

// Convenience accessors
inline HealthLevel health_imu(uint8_t primary)  { return health_get_subsystem(primary, kHealthShiftImu); }
inline HealthLevel health_baro(uint8_t primary) { return health_get_subsystem(primary, kHealthShiftBaro); }
inline HealthLevel health_eskf(uint8_t primary) { return health_get_subsystem(primary, kHealthShiftEskf); }
inline HealthLevel health_gps(uint8_t primary)  { return health_get_subsystem(primary, kHealthShiftGps); }

// ============================================================================
// Sliding window size for degraded-state detection
// ============================================================================
static constexpr uint8_t kHealthWindowSize         = 10;   // 10 ticks = 1s at 10Hz
static constexpr uint8_t kImuDegradeThreshold      = 5;    // 5/10 invalid → degraded
static constexpr uint8_t kBaroDegradeThreshold     = 3;    // 3/10 invalid → degraded

// ============================================================================
// Critical-fault persistence (Stage 16C IVP-142b-3)
//
// Council rationale (2026-04-18): a single-tick health_monitor_critical_fault()
// returning true on any kHealthFault triggers false auto-DISARMs on noise
// events — dust in the baro vent, transient I2C NACKs, IMU sample drops.
// Rocketeer's field-experience: closing a door near the pad pressure-pulses
// the baro and a naive fault-means-abort path aborts the rocket.
//
// Fix: require N consecutive ticks of fault AND phase != IDLE before a
// primary-byte fault counts as "critical" for auto-action purposes.
// At 10 Hz tick rate, kCriticalFaultPersistTicks = 5 = 500 ms.
//
// Phases:
//   IDLE: go/no-go NO-GO already blocks ARM. auto-DISARM is a no-op by
//         definition (not armed). No persistence check needed — return
//         false so callers don't think they need to act.
//   ARMED / BOOST / COAST / DESCENT / LANDED: persistence-gated. A fault
//         only counts as critical after N consecutive ticks.
//   LANDED: same as flight phases — if a sensor dies mid-descent the
//         health status still reports critical for post-flight review.
// ============================================================================
static constexpr uint8_t kCriticalFaultPersistTicks = 5;  // 500 ms at 10 Hz

// Pure bump helper for the persistence counter — extracted so the
// algorithm is testable at host level without the full health_monitor
// hardware-dep surface. Increments on kHealthFault (saturating at
// kCriticalFaultPersistTicks); resets to 0 on any other level.
inline uint8_t critical_fault_ticks_next(uint8_t prev, HealthLevel lvl) {
    if (lvl != kHealthFault) {
        return 0;
    }
    if (prev < kCriticalFaultPersistTicks) {
        return static_cast<uint8_t>(prev + 1U);
    }
    return kCriticalFaultPersistTicks;
}

// ============================================================================
// MCU die-temp thresholds (Stage 16C IVP-142b-1)
// Sources:
//   - RP2350 datasheet §1.4.3 Absolute Maximum Ratings: Tj max 125 °C
//   - Industrial-grade silicon operating range spec: -40 to +85 °C
//   - 105 °C safe-mode threshold = 20 °C margin below abs-max
// Flight-data validation of the WARN threshold may refine it downward
// in Stage 18 (airframe thermal profile); FAULT/SAFE are datasheet
// ceilings and do not move.
// ============================================================================
static constexpr float kMcuTempWarnC      = 70.0F;   // enter DEGRADED
static constexpr float kMcuTempFaultC     = 85.0F;   // enter FAULT
static constexpr float kMcuTempSafeModeC  = 105.0F;  // trigger safe-mode (IVP-142b-2)
static constexpr float kMcuTempHysteresisC = 2.0F;   // clean exit / re-entry gap

// ============================================================================
// API
// ============================================================================

// Initialize health monitor (call once at boot)
void health_monitor_init();

// Periodic tick — evaluates all subsystem health.
// Returns true if primary or secondary flags changed.
bool health_monitor_tick();

// Read-only accessor (safe under QV cooperative scheduling)
const HealthState* health_monitor_get_state();

// Go/No-Go assembly for ARM validation
struct GoNoGoInput;  // forward declare
void health_monitor_fill_go_nogo(GoNoGoInput* gng);

// Set flight phase for fault-latch behavior.
// Called by AO_HealthMonitor when SIG_PHASE_CHANGE received.
void health_monitor_set_phase(uint8_t phase);

// Manual fault latch clear (CLI reset command or reboot).
// Only works in IDLE — ignored during flight phases.
void health_monitor_clear_latches();

// Check if critical subsystems are faulted (IMU, baro, or ESKF).
// Used by FD for auto-DISARM while ARMED.
bool health_monitor_critical_fault();

// Pure hysteresis FSM for MCU die-temp (IVP-142b-1).
// Given the previous level and a new temperature reading, returns the
// next HealthLevel using the WARN/FAULT thresholds with 2 °C hysteresis
// on fall-back. Exposed for host-test coverage of state transitions.
//
// Does NOT consult mcu_temp_is_stuck() — caller layers that on top.
//   prev=kHealthAbsent seeds to kHealthOk on first valid sample (>= -100).
//   temp_c < -100 is treated as sentinel -> kHealthAbsent.
inline HealthLevel mcu_temp_classify(HealthLevel prev, float temp_c) {
    if (temp_c < -100.0F) {
        return kHealthAbsent;
    }
    HealthLevel base = (prev == kHealthAbsent) ? kHealthOk : prev;

    if (temp_c >= kMcuTempFaultC) {
        return kHealthFault;
    }
    if (temp_c >= kMcuTempWarnC) {
        if (base == kHealthFault &&
            temp_c >= (kMcuTempFaultC - kMcuTempHysteresisC)) {
            return kHealthFault;
        }
        return kHealthDegraded;
    }
    if (base == kHealthFault &&
        temp_c >= (kMcuTempFaultC - kMcuTempHysteresisC)) {
        return kHealthFault;
    }
    if (base == kHealthDegraded &&
        temp_c >= (kMcuTempWarnC - kMcuTempHysteresisC)) {
        return kHealthDegraded;
    }
    return kHealthOk;
}

// ============================================================================
// Legacy HealthFlag — kept temporarily for Go/No-Go compatibility
// Remove after IVP-107 completes telemetry migration.
// ============================================================================
enum HealthFlag : uint8_t {
    kHealthFlagImuOk      = (1 << 0),
    kHealthFlagBaroOk     = (1 << 1),
    kHealthFlagGpsOk      = (1 << 2),
    kHealthFlagRadioOk    = (1 << 3),
    kHealthFlagEskfOk     = (1 << 4),
    kHealthFlagFlashOk    = (1 << 5),
    kHealthFlagWatchdogOk = (1 << 6),
};

} // namespace rc

#endif // ROCKETCHIP_HEALTH_MONITOR_H
