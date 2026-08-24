// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Go/No-Go pre-arm poll. Pure function of GoNoGoInput — no globals, no HW.
// Snapshot: health_monitor_fill_go_nogo(). ARM: command_handler uses all_go.
//
// Two tiers (NASA pad terminology):
//   Tier 1 platform — IMU, baro, ESKF, flash, launch-abort, watchdog,
//     prior hardfault, prior brownout. Any NO-GO blocks ARM.
//   Tier 2 profile — GPS, mag cal, radio HW, RF link, battery stub.
//     NO-GO warns; ARM still allowed.
// CLI preflight must print this poll and use all_go as VERDICT — not a
// parallel health-byte policy.

#ifndef ROCKETCHIP_GO_NOGO_CHECKS_H
#define ROCKETCHIP_GO_NOGO_CHECKS_H

#include <cstdint>

namespace rc {

// Room for 8 Tier-1 + 5 Tier-2 stations (see go_nogo_evaluate).
static constexpr uint8_t kGoNoGoMaxChecks = 16;

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
    bool watchdog_ok;           // kHealthWatchdogOk (ESKF healthy bit)
    bool prior_hardfault_clear; // kHealthCriticalPriorHardfault not latched
    bool prior_brownout_clear;  // kHealthCriticalPriorBrownout not latched
                                //   (operator-cleared after inspection)

    // Tier 2: Profile-specific
    bool gps_has_lock;          // fix_type >= 2 && satellites >= 4
    bool mag_calibrated;        // CAL_STATUS_MAG flag set
    bool radio_linked;          // Radio init OK (TX or RX)
    // Learned RF link from AO_RfManager, not radio_linked (HW init).
    uint8_t rf_link_state;      // rc::LinkState cast to uint8_t
                                //   0=kAcq, 1=kTentative, 2=kTrack, 3=kTrackDegraded
    uint8_t rf_lq_pct;          // 0-100, sliding-window link quality
    bool rf_anchor_valid;       // AO_RfManager has seen at least 1 valid RX
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

// Print: [GO/NO-GO] Platform: t1_go/t1_total GO|NO-GO | Profile: t2_go/t2_total GO|WARN
// then one line per NO-GO station (name + reason).
void go_nogo_print(const GoNoGoResult& result);

} // namespace rc

#endif // ROCKETCHIP_GO_NOGO_CHECKS_H
