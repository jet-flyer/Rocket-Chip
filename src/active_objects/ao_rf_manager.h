// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RfManager — RF link manager (Stage T Batch B, IVP-T14)
//
// Owns link state (ACQ / TENTATIVE / TRACK / TRACK_DEGRADED), filtered
// RxDone anchor estimate, and the "when is my next safe TX window" math.
//
// Station-side consumer: AO_Radio TX scheduler calls
// AO_RfManager_next_tx_window_us() to anchor station TX to vehicle RxDone.
//
// Read-only consumers: AO_FlightDirector (pre-arm aggregator),
// rc_os_dashboard (row + glance indicator) read AO_RfManager_get_state().
//
// Design doc: docs/plans/STAGE_T_T14_DESIGN.md (Round 2 final).
// Council: NASA/JPL, ArduPilot, Rocketeer, Cubesat (2026-04-21 Round 2 GO).
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
// Public API — Start + read-only accessor + scheduler gate
// ============================================================================

extern QActive * const AO_RfManager;

/// Start the AO. Priority between FD (highest) and Logger (middle).
/// nav_period_ms_init seeds the initial period; updated via set_nav_period_ms
/// when SET_RADIO_CONFIG changes cadence.
void AO_RfManager_start(uint8_t prio, uint32_t nav_period_ms_init);

/// Read-only snapshot. Pointer stable; contents change between ticks.
/// **Cooperative-dispatch-only invariant — see header doc.**
const RfManagerState* AO_RfManager_get_state();

/// Station TX scheduler hook: returns the microsecond timestamp of the
/// next safe-to-TX window, OR 0 if the anchor is stale (deadman fired,
/// §4) or not yet acquired. Returns 0 if LinkState is kAcq.
uint32_t AO_RfManager_next_tx_window_us(uint32_t now_us);

/// Station retry gate: "is the link healthy enough to bother retrying?"
/// Returns false in kAcq (no point TXing blind) and during kTrackDegraded
/// early frames (wait for recovery).
bool AO_RfManager_ok_to_retry();

/// Called when SET_RADIO_CONFIG successfully applies a new nav rate.
/// Recomputes deadman + alpha thresholds against the new period.
void AO_RfManager_set_nav_period_ms(uint32_t nav_period_ms);

#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS
/// Test-only: override last_rx_ms so the next 10 Hz tick sees a stale anchor
/// and the deadman / forced-ACQ branches fire. Used by fault_force_radio_
/// dropout() (R-9b) for fault-injection harness; never called from
/// production paths.
void AO_RfManager_force_last_rx_ms_for_test(uint32_t last_rx_ms);
#endif

} // namespace rc

#endif // ROCKETCHIP_AO_RF_MANAGER_H
