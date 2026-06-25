// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// AO_RfManager — RF link manager implementation.
//
// Event flow:
//   AO_Radio posts SIG_RADIO_RX on every valid RX → AO_RfManager handles,
//   updates per-RX state (LQ, anchor_estimate, consec counters, state-machine
//   transitions).
//   10 Hz internal tick drives deadman + idle-drift + forced-ACQ checks that
//   fire on absence of RX, not on RX events.
//
// Design doc: docs/plans/STAGE_T_T14_DESIGN.md (Round 2 final).
//============================================================================

#include "ao_rf_manager.h"
#include "safety/rf_link_health.h"  // pure state-machine helpers (host-testable)
#include "rocketchip/ao_signals.h"  // rc::RadioRxEvt, rc::SIG_RADIO_RX
#include "rocketchip/job.h"         // job::kRole
#include "ao_notify.h"              // IVP-T14 #10: vehicle-lost/found posts
#include <string.h>

#ifndef ROCKETCHIP_HOST_TEST
#include "pico/time.h"
#endif

namespace rc {

// ============================================================================
// Constants — AO-specific (pure state-machine constants live in
// safety/rf_link_health.h, shared with host tests).
// ============================================================================

// 10 Hz internal tick: 100 Hz QF base / 10 = every 10 ticks.
// Drives the deadman/forced-ACQ checks that fire on absence of RX.
static constexpr uint16_t kRfTickInterval = 10U;

// ============================================================================
// Private signal — 10 Hz tick
// ============================================================================
enum : uint16_t {
    SIG_RFMGR_TICK = rc::SIG_AO_MAX + 20
};

// ============================================================================
// AO struct
// ============================================================================

struct RfManager {
    QActive super;                // QP/C base (must be first)
    QTimeEvt tick_timer;
    RfManagerState state;         // Snapshot exposed via get_state()
    uint32_t nav_period_ms;       // Current nav cadence (for deadman math)
    uint16_t alpha_scaled;        // Filter coefficient × kAlphaScale
    // LQ sliding window: 1 bit per RX slot (1=good, 0=miss/crc_err).
    // Window advances per 10 Hz tick (if no RX occurred, slot = 0) AND
    // per RX event (slot = 1, displaces oldest).
    uint16_t lq_window;           // uses low kLqWindowSize bits
    uint8_t  lq_window_count;     // valid bits in window (0..kLqWindowSize)
};

static RfManager g_rf;

// Queue depth 16: 10 Hz tick + bursty SIG_RADIO_RX at up to nav_rate (10 Hz).
// LL Entry 32 / Stage 9 queue-overflow analysis: 16 handles a 160 ms stall
// (e.g., flash op) without overflow.
static QEvtPtr g_rfQueue[16];

// ============================================================================
// Forward declarations
// ============================================================================
static QState rf_initial(RfManager * const me, QEvt const * const e);
static QState rf_running(RfManager * const me, QEvt const * const e);

static void handle_valid_rx(RfManager * const me, const RadioRxEvt * const rx);
static void handle_tick(RfManager * const me);
static void transition_to_acq(RfManager * const me, const char * reason);
static void check_transitions(RfManager * const me);

// ============================================================================
// Utility
// ============================================================================
static uint32_t now_ms_impl() {
#ifndef ROCKETCHIP_HOST_TEST
    return to_ms_since_boot(get_absolute_time());
#else
    return 0;
#endif
}

static uint32_t now_us_impl() {
#ifndef ROCKETCHIP_HOST_TEST
    return time_us_32();
#else
    return 0;
#endif
}

// Advance the LQ window, inserting `good_bit` (1=good RX, 0=miss/crc_err).
// Delegates bit-shift to shared helper; increments count guard.
static void lq_window_push(RfManager * me, uint8_t good_bit) {
    me->lq_window = rf_lq_window_push(me->lq_window, good_bit);
    if (me->lq_window_count < kRfLqWindowSize) {
        me->lq_window_count++;
    }
}

// ============================================================================
// Transition handlers
// ============================================================================

// Force-drop to kAcq. Called on prolonged RX absence, on tx_consec_fail
// in TRACK, or on idle-drift tripwire. Does NOT clear per-RX metadata like
// RSSI (dashboard still shows last-known).
// Stage T Batch B IVP-T14 Round 2 #10: on the transition edge from a
// tracking state (kTrack or kTrackDegraded) to kAcq, post the
// "VEHICLE NOT HEARD" notification. Not posted on startup drops from
// kTentative (we never had a learned link — no lost verdict yet).
static void transition_to_acq(RfManager * const me, const char * /*reason*/) {
    const LinkState prev = me->state.state;
    me->state.state = LinkState::kAcq;
    me->state.anchor_valid = false;
    me->state.consec_good_rx = 0;
    // Leave consec_missed_rx alone — it tracks missed since last good RX,
    // independent of state.
    if (prev == LinkState::kTrack || prev == LinkState::kTrackDegraded) {
        AO_Notify_post_vehicle_lost();
    }
}

// Compute next state per design §2 predicates — delegates to shared pure
// helper in safety/rf_link_health.h (host-testable).
// IVP-T14 Round 2 #10: also detects the re-acquisition edge into kTrack
// and posts SIG_NOTIFY_VEHICLE_FOUND to clear the station's lost-latch.
static void check_transitions(RfManager * const me) {
    RfManagerState& s = me->state;
    s.lq_pct = rf_lq_compute_pct(me->lq_window, me->lq_window_count);

    RfTransitionInput in;
    in.current            = s.state;
    in.consec_good_rx     = s.consec_good_rx;
    in.lq_pct             = s.lq_pct;
    in.lq_window_count    = me->lq_window_count;
    in.consec_missed_rx   = s.consec_missed_rx;
    // Note: forced-ACQ time check is handled in handle_tick (which has
    // direct access to now_ms). rf_next_state sees 0 here so the wallclock
    // branch is inactive — only the consec-missed-frames branch can fire
    // from the per-RX path, which is correct (no drops during valid RX).
    in.time_since_last_rx_ms = 0;

    const LinkState prev = s.state;
    s.state = rf_next_state(in);
    if (s.state == LinkState::kTrack && prev != LinkState::kTrack) {
        AO_Notify_post_vehicle_found();
    }
}

// ============================================================================
// RX event handler — runs on every SIG_RADIO_RX from AO_Radio
// ============================================================================
static void handle_valid_rx(RfManager * const me, const RadioRxEvt * const rx) {
    RfManagerState& s = me->state;

    uint32_t now_us = now_us_impl();
    uint32_t now_ms = now_ms_impl();

    // CRC check: AO_Radio already validated for CCSDS packets. Non-CCSDS
    // (MAVLink) is passed through with CRC unchecked at this layer — treat
    // as good here; telemetry-layer parser will drop if malformed.
    // RadioRxEvt does not expose a crc_ok field; we rely on AO_Radio's
    // pre-publish validation (see validate_rx_packet in ao_radio.cpp).
    const bool good = (rx->len > 0);

    if (good) {
        s.packets_good++;
        lq_window_push(me, 1U);

        // Anchor-estimate filter (§3). On first RX, seed directly.
        if (!s.anchor_valid) {
            s.anchor_estimate_us = static_cast<int32_t>(now_us);
            s.anchor_valid = true;
        } else {
            // Expected next arrival relative to the filter: s.anchor + nav_period.
            int32_t expected_us = s.anchor_estimate_us +
                                  static_cast<int32_t>(me->nav_period_ms) * 1000;
            int32_t error_us = static_cast<int32_t>(now_us) - expected_us;
            s.anchor_estimate_us +=
                rf_anchor_correction_us(error_us, me->alpha_scaled);
        }

        s.last_rx_us = now_us;
        s.last_rx_ms = now_ms;
        s.last_rx_rssi_dbm = rx->rssi;
        s.last_rx_snr_db = rx->snr;
        s.consec_good_rx++;
        s.consec_missed_rx = 0;

        // State promotion on first RX: ACQ → TENTATIVE.
        if (s.state == LinkState::kAcq) {
            s.state = LinkState::kTentative;
        }
    } else {
        s.packets_crc_err++;
        lq_window_push(me, 0U);
        s.consec_good_rx = 0;
    }

    check_transitions(me);
}

// ============================================================================
// 10 Hz tick handler — deadman + forced-ACQ + missed-frame accounting
// ============================================================================
static void handle_tick(RfManager * const me) {
    RfManagerState& s = me->state;
    uint32_t now_ms = now_ms_impl();

    // Have we missed a nav frame since last tick?
    // Detect by time since last RX. If > 1.5 × nav_period, count as missed.
    // Only accumulate when state != kAcq (kAcq has no expected cadence).
    if (s.state != LinkState::kAcq && s.anchor_valid) {
        uint32_t since_rx_ms = now_ms - s.last_rx_ms;
        uint32_t nav_period_ms = me->nav_period_ms;
        if (nav_period_ms == 0) { nav_period_ms = 200U; }  // sanity

        if (since_rx_ms > (nav_period_ms * 15U / 10U)) {
            // Time has passed longer than 1.5 nav periods — tally missing
            // slot. Advance window with a miss.
            lq_window_push(me, 0U);
            s.consec_missed_rx++;
            s.consec_good_rx = 0;
            s.packets_missed++;

            // §2 Forced-ACQ cap: delegated to shared rf_next_state helper
            // (wallclock branch active via time_since_last_rx_ms input).
            RfTransitionInput in;
            in.current               = s.state;
            in.consec_good_rx        = s.consec_good_rx;
            in.lq_pct                = rf_lq_compute_pct(me->lq_window,
                                                         me->lq_window_count);
            in.lq_window_count       = me->lq_window_count;
            in.consec_missed_rx      = s.consec_missed_rx;
            in.time_since_last_rx_ms = since_rx_ms;
            s.lq_pct = in.lq_pct;

            LinkState next = rf_next_state(in);
            if (next == LinkState::kAcq && s.state != LinkState::kAcq) {
                transition_to_acq(me, "forced");
            } else {
                s.state = next;
            }
        }
    }
}

// ============================================================================
// State handlers
// ============================================================================

static QState rf_initial(RfManager * const me, QEvt const * const e) {
    (void)e;

    // Subscribe to RadioRx events from AO_Radio.
    // Both station and vehicle need RF link health tracking.
    QActive_subscribe(&me->super, rc::SIG_RADIO_RX);

    // Arm 10 Hz internal tick
    QTimeEvt_armX(&me->tick_timer, kRfTickInterval, kRfTickInterval);

    return Q_TRAN(&rf_running);
}

static QState rf_running(RfManager * const me, QEvt const * const e) {
    switch (e->sig) {
    case SIG_RFMGR_TICK: {
        handle_tick(me);
        return Q_HANDLED();
    }
    case rc::SIG_RADIO_RX: {
        auto const * rx = rc::evt_cast<rc::RadioRxEvt>(e);
        handle_valid_rx(me, rx);
        return Q_HANDLED();
    }
    default:
        break;
    }
    return Q_SUPER(&QHsm_top);
}

// ============================================================================
// Public interface
// ============================================================================

QActive * const AO_RfManager = &g_rf.super;

void AO_RfManager_start(uint8_t prio, uint32_t nav_period_ms_init) {
    // Initialize AO state before Q_NEW_REF-style init so first RX has
    // sensible defaults.
    memset(&g_rf.state, 0, sizeof(g_rf.state));
    g_rf.state.state = LinkState::kAcq;
    g_rf.state.anchor_valid = false;
    g_rf.nav_period_ms = (nav_period_ms_init != 0) ? nav_period_ms_init : 200U;
    g_rf.alpha_scaled = kRfAlphaInit;
    g_rf.lq_window = 0;
    g_rf.lq_window_count = 0;

    QActive_ctor(&g_rf.super, Q_STATE_CAST(&rf_initial));
    QTimeEvt_ctorX(&g_rf.tick_timer, &g_rf.super, SIG_RFMGR_TICK, 0U);

    QActive_start(&g_rf.super,
                  Q_PRIO(prio, 0U),
                  g_rfQueue,
                  Q_DIM(g_rfQueue),
                  nullptr, 0U,
                  nullptr);
}

const RfManagerState* AO_RfManager_get_state() {
    // Cooperative-dispatch-only invariant documented in header. No runtime
    // guard — invariant is contract, not enforcement. Debug builds could
    // assert-on-Core1-access in future.
    return &g_rf.state;
}

uint32_t AO_RfManager_next_tx_window_us(uint32_t now_us) {
    const RfManagerState& s = g_rf.state;

    // No anchor yet → no window.
    if (!s.anchor_valid || s.state == LinkState::kAcq) {
        return 0;
    }

    // Deadman check: anchor stale? Delegates to shared helper.
    uint32_t elapsed_us = now_us - s.last_rx_us;
    if (rf_deadman_fired(elapsed_us, g_rf.nav_period_ms)) {
        return 0;  // Deadman fired — hold TX, wait for re-sync.
    }

    // Window math: next vehicle TX is expected at last_rx_us + nav_period.
    // Safe station-TX window opens at last_rx_us + small settle (2 ms) and
    // closes at next_vehicle_tx - guard_us.
    //
    // Skeleton: return "next safe moment" simply as last_rx_us + small offset.
    // Full window arithmetic per design §6 lands in a follow-up commit alongside
    // airtime-aware guard computation (needs airtime-formula integration into
    // this AO — currently only AO_Radio knows airtime).
    //
    // For now, return last_rx_us + 2 ms as a conservative "after vehicle
    // settles, before next period" window opener.
    return s.last_rx_us + 2000U;
}

bool AO_RfManager_ok_to_retry() {
    const RfManagerState& s = g_rf.state;
    // Don't retry in kAcq (no point TXing blind).
    // Permit retries in kTentative (builds up link) + kTrack + kTrackDegraded.
    return s.state != LinkState::kAcq;
}

void AO_RfManager_set_nav_period_ms(uint32_t nav_period_ms) {
    if (nav_period_ms == 0) { nav_period_ms = 200U; }
    if (nav_period_ms > 5000U) { nav_period_ms = 5000U; }  // sanity cap
    g_rf.nav_period_ms = nav_period_ms;
}

const char* link_state_name(LinkState s) {
    switch (s) {
    case LinkState::kAcq:           return "ACQ";
    case LinkState::kTentative:     return "TENTATIVE";
    case LinkState::kTrack:         return "TRACK";
    case LinkState::kTrackDegraded: return "TRACK_DEGRADED";
    }
    return "?";
}

// R-25-exec step 3 (2026-05-13): no longer dev-tier gated. Accessed only
// via fault_force_radio_dropout() which checks test_mode_active() at entry.
void AO_RfManager_force_last_rx_ms_for_test(uint32_t last_rx_ms) {
    g_rf.state.last_rx_ms = last_rx_ms;
}

} // namespace rc
