// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file go_nogo_checks.cpp
 * @brief Go/No-Go poll evaluation (IVP-69)
 */

#include "go_nogo_checks.h"
#include <cstdio>
#include <cstring>

namespace rc {

// Helper: add a station poll to the result
static void add_station(GoNoGoResult& r, uint8_t tier, const char* name,
                         bool go, const char* reason) {
    if (r.num_checks >= kGoNoGoMaxChecks) { return; }

    GoNoGoCheck& c = r.checks[r.num_checks];
    strncpy(c.name, name, sizeof(c.name) - 1);
    c.name[sizeof(c.name) - 1] = '\0';
    strncpy(c.reason, reason, sizeof(c.reason) - 1);
    c.reason[sizeof(c.reason) - 1] = '\0';
    c.tier = tier;
    c.go = go;

    if (tier == 1) {
        ++r.tier1_total;
        if (go) { ++r.tier1_go; }
    } else {
        ++r.tier2_total;
        if (go) { ++r.tier2_go; }
    }
    ++r.num_checks;
}

// Stage T Batch B IVP-T14: add "RF Link" Tier-2 station. GO when link is
// in kTrack with LQ >= 65% — matches the dashboard glance-indicator green
// threshold (Round 2 design §12). kTrackDegraded (state 3) is warn-yellow.
//   state: 0=ACQ, 1=Tentative, 2=Track, 3=TrackDegraded
static void add_rf_link_station(GoNoGoResult& r, const GoNoGoInput& input) {
    const bool link_go = input.rf_anchor_valid &&
                         (input.rf_link_state == 2) &&
                         (input.rf_lq_pct >= 65U);
    static char reason[kGoNoGoReasonLen];
    if (!input.rf_anchor_valid) {
        std::snprintf(reason, sizeof(reason), "NO-GO NO RX YET");
    } else if (input.rf_link_state == 0) {
        std::snprintf(reason, sizeof(reason), "NO-GO LINK ACQ");
    } else if (input.rf_link_state == 1) {
        std::snprintf(reason, sizeof(reason), "NO-GO TENTATIVE %u%%",
                      static_cast<unsigned>(input.rf_lq_pct));
    } else if (input.rf_link_state == 3) {
        std::snprintf(reason, sizeof(reason), "NO-GO DEGRADED %u%%",
                      static_cast<unsigned>(input.rf_lq_pct));
    } else if (input.rf_lq_pct < 65U) {
        std::snprintf(reason, sizeof(reason), "NO-GO LQ %u%%",
                      static_cast<unsigned>(input.rf_lq_pct));
    } else {
        std::snprintf(reason, sizeof(reason), "GO LQ %u%%",
                      static_cast<unsigned>(input.rf_lq_pct));
    }
    add_station(r, 2, "RF Link", link_go, reason);
}

GoNoGoResult go_nogo_evaluate(const GoNoGoInput& input) {
    GoNoGoResult r{};

    // Tier 1: Platform (all must be GO to ARM)
    add_station(r, 1, "IMU", input.imu_healthy,
                input.imu_healthy ? "GO" : "NO-GO UNHEALTHY");
    add_station(r, 1, "Baro", input.baro_healthy,
                input.baro_healthy ? "GO" : "NO-GO UNHEALTHY");
    add_station(r, 1, "ESKF", input.eskf_healthy,
                input.eskf_healthy ? "GO" : "NO-GO UNHEALTHY");
    add_station(r, 1, "Flash", input.flash_available,
                input.flash_available ? "GO" : "NO-GO FULL");
    add_station(r, 1, "Safety", !input.launch_abort,
                !input.launch_abort ? "GO" : "NO-GO LAUNCH ABORT");
    add_station(r, 1, "Watchdog", input.watchdog_ok,
                input.watchdog_ok ? "GO" : "NO-GO SAFE MODE");
    // Fault-recovery 2026-05-14: prior-boot fault latches gate ARM.
    // Each requires operator-cleared latch (via CLI reset command in IDLE
    // after physical inspection of the recovery cause) before re-arm.
    add_station(r, 1, "PriorHF", input.prior_hardfault_clear,
                input.prior_hardfault_clear ? "GO" : "NO-GO PRIOR HARDFAULT");
    add_station(r, 1, "PriorBOR", input.prior_brownout_clear,
                input.prior_brownout_clear ? "GO" : "NO-GO PRIOR BROWNOUT");

    // Tier 2: Profile-specific (warn only, don't block ARM)
    add_station(r, 2, "GPS", input.gps_has_lock,
                input.gps_has_lock ? "GO" : "NO-GO NO LOCK");
    add_station(r, 2, "Mag Cal", input.mag_calibrated,
                input.mag_calibrated ? "GO" : "NO-GO NOT CALIBRATED");
    // Stage T Batch B: "Radio HW" = SX1276 init / SPI reachable;
    // "RF Link" below = learned link state from AO_RfManager (IVP-T14).
    add_station(r, 2, "Radio HW", input.radio_linked,
                input.radio_linked ? "GO" : "NO-GO NOT INIT");
    add_rf_link_station(r, input);
    add_station(r, 2, "Battery", true, "GO (not monitored)");

    r.all_go = (r.tier1_go == r.tier1_total);
    return r;
}

void go_nogo_print(const GoNoGoResult& result) {
    printf("[GO/NO-GO] Platform: %u/%u %s | Profile: %u/%u %s\n",
           static_cast<unsigned>(result.tier1_go),
           static_cast<unsigned>(result.tier1_total),
           result.all_go ? "GO" : "NO-GO",
           static_cast<unsigned>(result.tier2_go),
           static_cast<unsigned>(result.tier2_total),
           (result.tier2_go == result.tier2_total) ? "GO" : "WARN");

    // Detail any NO-GO stations
    for (uint8_t i = 0; i < result.num_checks; ++i) {
        const GoNoGoCheck& c = result.checks[i];
        if (!c.go) {
            printf("  %s: %s\n", c.name, c.reason);
        }
    }
}

} // namespace rc
