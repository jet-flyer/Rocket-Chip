// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file go_nogo_checks.h
 * @brief Go/No-Go pre-arm poll (IVP-69)
 *
 * Two-tier Go/No-Go readiness poll (NASA launch terminology):
 *   Tier 1 (platform): IMU, baro, ESKF, flash, LAUNCH_ABORT, watchdog
 *   Tier 2 (profile): GPS, mag cal, radio, battery (stub)
 *
 * Tier 1 "No-Go" blocks arming. Tier 2 "No-Go" warns but allows ARM.
 *
 * All checks are pure functions of a GoNoGoInput snapshot — no globals,
 * no hardware access. main.cpp populates the snapshot from its statics.
 */

#ifndef ROCKETCHIP_GO_NOGO_CHECKS_H
#define ROCKETCHIP_GO_NOGO_CHECKS_H

#include <cstdint>

namespace rc {

// Maximum number of individual check results
static constexpr uint8_t kGoNoGoMaxChecks = 12;

// Maximum reason string length (stack-allocated, no heap)
static constexpr uint8_t kGoNoGoReasonLen = 32;

// Input snapshot — populated by caller from system state.
// Pure data, no pointers to hardware.
struct GoNoGoInput {
    // Tier 1: Platform
    bool imu_healthy;           // ICM-20948 init OK + recent reads succeeding
    bool baro_healthy;          // DPS310 init OK + recent reads succeeding
    bool eskf_healthy;          // ESKF::healthy() == true
    bool flash_available;       // Flight table loaded + space remaining
    bool launch_abort;          // WatchdogRecovery::launch_abort latched
    bool watchdog_ok;           // No safe-mode, no ESKF disabled

    // Tier 2: Profile-specific
    bool gps_has_lock;          // fix_type >= 2 && satellites >= 4
    bool mag_calibrated;        // CAL_STATUS_MAG flag set
    bool radio_linked;          // Radio init OK (TX or RX)
    // Battery: no ADC hardware on Feather RP2350 HSTX — always passes
};

// Individual station poll result
struct GoNoGoCheck {
    char name[16];              // Station name: "IMU", "GPS", etc.
    char reason[kGoNoGoReasonLen]; // "GO", "NO-GO: ...", "not monitored"
    uint8_t tier;               // 1 or 2
    bool go;                    // true = GO, false = NO-GO
};

// Aggregate poll result
struct GoNoGoResult {
    GoNoGoCheck checks[kGoNoGoMaxChecks];
    uint8_t num_checks;
    uint8_t tier1_total;
    uint8_t tier1_go;
    uint8_t tier2_total;
    uint8_t tier2_go;
    bool all_go;                // All Tier 1 GO — clear to ARM
};

// Run the Go/No-Go poll against the input snapshot.
// Returns aggregate result. Does not print — caller decides output.
GoNoGoResult go_nogo_evaluate(const GoNoGoInput& input);

// Print Go/No-Go result to serial (Ground classification).
// Format: [GO/NO-GO] Platform: 6/6 GO | Profile: 3/4 (GPS: NO-GO NO LOCK)
void go_nogo_print(const GoNoGoResult& result);

} // namespace rc

#endif // ROCKETCHIP_GO_NOGO_CHECKS_H
