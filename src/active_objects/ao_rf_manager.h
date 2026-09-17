// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RfManager — RF link health (ACQ / TENTATIVE / TRACK / TRACK_DEGRADED).
//
// Read-only consumers: AO_FlightDirector (pre-arm), rc_os_dashboard
// (row + glance) via AO_RfManager_get_state(). Does not gate radio TX.
// COP-P + R-32 cadence own air. Rekey TRACK/LQ to COP-P lock later.
//============================================================================
#ifndef ROCKETCHIP_AO_RF_MANAGER_H
#define ROCKETCHIP_AO_RF_MANAGER_H

#include <stdint.h>

extern "C" {
#include "qp_port.h"
}

namespace rc {

// LinkState enum + pure state-machine helpers defined in safety/rf_link_health.h.
// Included here so consumers of AO_RfManager see the enum type.
} // namespace rc
#include "safety/rf_link_health.h"
namespace rc {

// Human-readable label for dashboard/log use.
const char* link_state_name(LinkState s);

// ============================================================================
// RfManagerState — read-only snapshot exposed to consumers.
//
// Per AO Commandment V (`docs/decisions/AO_COMMANDMENTS.md`):
//   **Callable only from Core 0 handler context under cooperative QV
//   dispatch.** Never from an ISR, never from Core 1, never cached across
//   AO dispatch boundaries. The pointer returned by
//   AO_RfManager_get_state() is stable; its contents change between ticks.
// ============================================================================
struct RfManagerState {
    LinkState state;              // Current link-health state
    uint8_t   lq_pct;             // Link quality 0-100 (sliding-window %)
    int16_t   last_rx_rssi_dbm;   // dBm
    int8_t    last_rx_snr_db;
    uint32_t  last_rx_ms;         // Wall-clock ms (to_ms_since_boot)
    uint32_t  last_rx_us;         // Microsecond timestamp for anchor math
    uint32_t  packets_good;       // Cumulative valid-RX count
    uint32_t  packets_crc_err;    // Cumulative CRC failures
    uint32_t  packets_missed;     // Cumulative drops (seq gaps detected
                                  //   via inter-arrival > 1.5× nav_period)
    uint8_t   consec_good_rx;     // Used by kTentative → kTrack promotion
    uint8_t   consec_missed_rx;   // Used by forced-ACQ demotion
    int32_t   anchor_estimate_us; // Filtered "when was vehicle's last TX"
                                  //   relative to now. Updated per RX via
                                  //   alpha-filter (§3).
    bool      anchor_valid;       // False until first RX seen
};

// ============================================================================
// Public API — start + read-only snapshot (no TX window gate)
// ============================================================================

extern QActive * const AO_RfManager;

// Start the AO. Priority between FD (highest) and Logger (middle).
// nav_period_ms_init seeds the initial period; updated via set_nav_period_ms
// when SET_RADIO_CONFIG changes cadence.
void AO_RfManager_start(uint8_t prio, uint32_t nav_period_ms_init);

// Read-only snapshot. Pointer stable; contents change between ticks.
// **Cooperative-dispatch-only invariant — see header doc.**
const RfManagerState* AO_RfManager_get_state();

// Called when a radio apply (SET_RADIO_CONFIG or COMM_CHANGE) changes nav rate.
void AO_RfManager_set_nav_period_ms(uint32_t nav_period_ms);

// Test-only: override last_rx_ms so the next 10 Hz tick sees a stale
// anchor and forced-ACQ fires. fault_force_radio_dropout() (R-9b);
// test_mode_active() at entry (SWE-133).
void AO_RfManager_force_last_rx_ms_for_test(uint32_t last_rx_ms);

} // namespace rc

#endif // ROCKETCHIP_AO_RF_MANAGER_H
