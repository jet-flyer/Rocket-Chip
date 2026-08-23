// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Health monitor — 2-bit per subsystem (absent/fault/degraded/healthy).
// Layout and policy: docs/decisions/HEALTH_CONTRACT.md. Tick at 10 Hz.
//============================================================================
#ifndef ROCKETCHIP_HEALTH_MONITOR_H
#define ROCKETCHIP_HEALTH_MONITOR_H

#include <stdint.h>
#include <stdbool.h>
#include "drivers/mcu_temp.h"

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

// Primary byte: [1:0] IMU, [3:2] Baro, [5:4] ESKF, [7:6] GPS.
// MCU die-temp is HealthState::mcu — not in primary (wire/PCM stay 8-bit).
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

// Critical bits: 0 = nominal. Display/preflight only — do not auto-transition.
enum HealthCritical : uint8_t {
    kHealthCriticalMcu              = (1 << 0),  // die temp ≥ kMcuTempSafeModeC
    kHealthCriticalPriorHardfault   = (1 << 1),  // crash_record at boot; latched
    kHealthCriticalPriorBrownout    = (1 << 2),  // POWMAN HAD_BOR; inspect before ARM
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
    bool go_nogo_ready;       // Cached go_nogo_evaluate().all_go (Tier 1)
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

// N consecutive fault ticks (not IDLE) before auto-action. 5 = 500 ms at 10 Hz.
static constexpr uint8_t kCriticalFaultPersistTicks = 5;

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

// MCU die-temp: datasheet Tj max 125 °C; 105 °C = 20 °C margin. WARN may move.
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
//   prev=kHealthAbsent seeds to kHealthOk on first valid sample
//   (>= kMcuTempAbsentBelowC). temp_c < kMcuTempAbsentBelowC (includes
//   kMcuTempSentinelC) is treated as unset -> kHealthAbsent.
inline HealthLevel mcu_temp_classify(HealthLevel prev, float temp_c) {
    if (temp_c < kMcuTempAbsentBelowC) {
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

// Pre-IVP-107 1-bit HealthFlag enum removed 2026-07-09: zero callers after
// Go/No-Go + preflight migrated to HealthLevel / HealthFlags2 / HealthSecondary
// (IVP-104/107). Do not reintroduce — use health_monitor_fill_go_nogo().

} // namespace rc

#endif // ROCKETCHIP_HEALTH_MONITOR_H
