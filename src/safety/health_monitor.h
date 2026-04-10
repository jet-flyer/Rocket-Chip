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
};

// ============================================================================
// HealthState — full system health snapshot
// ============================================================================
struct HealthState {
    uint8_t primary;          // 4 subsystems x 2-bit (HealthLevel)
    uint8_t secondary;        // 1-bit flags (HealthSecondary)
    uint8_t prev_primary;     // Previous tick (change detection)
    uint8_t prev_secondary;
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
