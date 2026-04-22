// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// RF Link Health — pure state-machine logic (Stage T Batch B, IVP-T14)
//
// Extracted from AO_RfManager for host testability. This module owns the
// state transition predicates, LQ sliding-window, anchor filter math, and
// deadman check. Zero QP/C framework dependency, zero hardware dependency —
// all inputs passed explicitly so host tests can drive transitions
// deterministically.
//
// Design doc: docs/plans/STAGE_T_T14_DESIGN.md Round 2 §2-§4.
//============================================================================
#ifndef ROCKETCHIP_RF_LINK_HEALTH_H
#define ROCKETCHIP_RF_LINK_HEALTH_H

#include <stdint.h>

namespace rc {

// ============================================================================
// LinkState — 4-state link-health machine (design §2)
// (Also declared in ao_rf_manager.h; the enum itself is portable — keep
//  both declarations syntactically identical.)
// ============================================================================
enum class LinkState : uint8_t {
    kAcq            = 0,
    kTentative      = 1,
    kTrack          = 2,
    kTrackDegraded  = 3,
};

// ============================================================================
// Tunable parameters (Round 2 council consensus values).
//
// Static-bench defaults. Flight-test MAY revisit per §18 of the design doc.
// ============================================================================
constexpr uint8_t  kRfTentativeToTrackConsecRx     = 5;   // §2
constexpr uint8_t  kRfTrackToDegradedLqPct         = 55;  // §2 Schmitt enter
constexpr uint8_t  kRfDegradedToTrackLqPct         = 65;  // §2 Schmitt exit (10 pp deadband)
constexpr uint8_t  kRfTentativeLqFloorPct          = 65;  // §2 TENTATIVE→TRACK LQ floor
// §2 Forced-ACQ: LOS is TIME-primary with a telemetry-frame minimum floor.
//   Both conditions must be true:
//     (a) time_since_last_rx_ms >= kRfForcedAcqLosMs (universal time axis).
//     (b) consec_missed_rx >= kRfForcedAcqMinFrames (spurious-single-drop
//         protection at low nav rates).
//   Plus an optional accelerator: a long run of consecutive drops triggers
//   even before the time threshold at high nav rates where that many drops
//   already exceeds any legitimate fade.
//
// Rationale (2026-04-21 user direction): LOS must not be implicitly coupled
// to radio config. Frame-count-primary was assumption-laden and broke
// whenever nav_rate_hz changed (pre-fix T12 C1 measured 0% ACK for exactly
// this reason-class of bug). Time is the universal axis; frame-minimum
// just prevents single-drop false LOS on slow links. At 10 Hz nav the two
// conditions coincide trivially; at 0.5 Hz nav the frame-min dominates,
// keeping a single missed packet from declaring LOS.
//
// Tuning rules (all numbers static-bench; field may revisit per §18):
//   kRfForcedAcqLosMs      = 2000  // typical RF fade upper bound
//   kRfForcedAcqMinFrames  = 5     // need 5 consecutive drops to trust the
//                                  //   "something is wrong" signal. User
//                                  //   direction 2026-04-21: "5 for now."
//                                  //   Low-nav-rate implications:
//                                  //     10 Hz: 5 drops = 500 ms  (time dominates)
//                                  //      5 Hz: 5 drops = 1000 ms (time dominates)
//                                  //      2 Hz: 5 drops = 2500 ms (frame-min dominates)
//                                  //      1 Hz: 5 drops = 5000 ms (frame-min dominates)
//   kRfForcedAcqFastFrames = 20    // rarely fires at >=10 Hz; exists for
//                                  //   >=20 Hz configs where many drops
//                                  //   fit inside 2 s
constexpr uint32_t kRfForcedAcqLosMs               = 2000U;
constexpr uint8_t  kRfForcedAcqMinFrames           = 5;
constexpr uint8_t  kRfForcedAcqFastFrames          = 20;
constexpr uint8_t  kRfLqWindowSize                 = 10;  // §2 sliding-window width
constexpr uint32_t kRfDeadmanFloorMs               = 500U;     // §4
constexpr uint32_t kRfDeadmanPeriodMultiple        = 5U;       // §4

// ============================================================================
// Sliding-window LQ helpers — bit 0 is newest.
// ============================================================================

// Push a slot onto a window. good_bit=1 means good RX, 0 means miss/CRC err.
// Returns new window. count is incremented (caller passes in-out).
inline uint16_t rf_lq_window_push(uint16_t window, uint8_t good_bit) {
    const uint16_t mask = static_cast<uint16_t>((1U << kRfLqWindowSize) - 1U);
    return static_cast<uint16_t>(((window << 1) | (good_bit & 1U)) & mask);
}

// Compute LQ% from a filled window. count must be in [0, kRfLqWindowSize].
inline uint8_t rf_lq_compute_pct(uint16_t window, uint8_t count) {
    if (count == 0) { return 0; }
    uint8_t good = 0;
    for (uint8_t i = 0; i < count; ++i) {
        if (window & 0x1U) { good++; }
        window = static_cast<uint16_t>(window >> 1);
    }
    return static_cast<uint8_t>((100U * good) / count);
}

// ============================================================================
// State transition — pure function.
//
// Given current state + all derived inputs, returns the next LinkState.
// Caller owns the counters (consec_good_rx, lq_window_count, etc.) — this
// function does NOT mutate state.
// ============================================================================
struct RfTransitionInput {
    LinkState current;
    uint8_t   consec_good_rx;         // incremented on good RX, reset on miss
    uint8_t   lq_pct;                 // 0-100, computed from sliding window
    uint8_t   lq_window_count;        // bits currently in window (<=kRfLqWindowSize)
    uint8_t   consec_missed_rx;       // incremented per detected miss
    uint32_t  time_since_last_rx_ms;  // wall-clock ms since last good RX
};

inline LinkState rf_next_state(const RfTransitionInput& in) {
    // Forced-ACQ: LOS requires BOTH a time threshold AND a minimum number
    // of missed frames (prevents single-dropped-packet false LOS on slow
    // links). A separate fast-frames accelerator triggers early only at
    // high nav rates where 20 consecutive drops already exceeds any fade.
    // See kRfForcedAcq* constants above for rationale.
    if (in.current != LinkState::kAcq) {
        // Primary LOS: time AND frame-minimum both satisfied.
        if (in.time_since_last_rx_ms >= kRfForcedAcqLosMs &&
            in.consec_missed_rx >= kRfForcedAcqMinFrames) {
            return LinkState::kAcq;
        }
        // Optional accelerator for high nav rates.
        if (in.consec_missed_rx >= kRfForcedAcqFastFrames) {
            return LinkState::kAcq;
        }
    }

    switch (in.current) {
    case LinkState::kAcq:
        // Promotion happens on RX, handled at call site (see rf_on_valid_rx).
        return LinkState::kAcq;

    case LinkState::kTentative:
        if (in.consec_good_rx >= kRfTentativeToTrackConsecRx &&
            in.lq_pct >= kRfTentativeLqFloorPct) {
            return LinkState::kTrack;
        }
        return LinkState::kTentative;

    case LinkState::kTrack:
        // Schmitt-trigger enter: drop to DEGRADED at LQ < 55% (only after
        // window has filled to kRfLqWindowSize to avoid transient drops).
        if (in.lq_pct < kRfTrackToDegradedLqPct &&
            in.lq_window_count >= kRfLqWindowSize) {
            return LinkState::kTrackDegraded;
        }
        return LinkState::kTrack;

    case LinkState::kTrackDegraded:
        // Schmitt-trigger exit: recover at LQ >= 65% (10 pp deadband).
        if (in.lq_pct >= kRfDegradedToTrackLqPct &&
            in.lq_window_count >= kRfLqWindowSize) {
            return LinkState::kTrack;
        }
        return LinkState::kTrackDegraded;
    }
    return in.current;
}

// ============================================================================
// Deadman — returns true if anchor is stale per §4 threshold.
//   threshold = max(500 ms, 5 × nav_period)
// ============================================================================
inline bool rf_deadman_fired(uint32_t time_since_last_rx_us,
                              uint32_t nav_period_ms) {
    uint32_t deadman_us = kRfDeadmanFloorMs * 1000U;
    uint32_t scaled_us = nav_period_ms * kRfDeadmanPeriodMultiple * 1000U;
    if (scaled_us > deadman_us) { deadman_us = scaled_us; }
    return time_since_last_rx_us > deadman_us;
}

// ============================================================================
// Anchor-filter step — scaled fixed-point to avoid FP in AO hot path.
//
// Input error_us = (now_us - (prev_anchor_us + nav_period_us))
// alpha_scaled = α × 10000 (bounded [500, 5000] = [0.05, 0.50])
// Returns the correction in microseconds to add to prev_anchor_us.
// ============================================================================
constexpr uint16_t kRfAlphaScale = 10000U;
constexpr uint16_t kRfAlphaMin   = 500U;   // 0.05
constexpr uint16_t kRfAlphaMax   = 5000U;  // 0.50
constexpr uint16_t kRfAlphaInit  = 2500U;  // 0.25 seed (SiK-style)

inline int32_t rf_anchor_correction_us(int32_t error_us, uint16_t alpha_scaled) {
    int64_t correction = (static_cast<int64_t>(error_us) *
                          static_cast<int64_t>(alpha_scaled)) /
                         static_cast<int64_t>(kRfAlphaScale);
    return static_cast<int32_t>(correction);
}

} // namespace rc

#endif // ROCKETCHIP_RF_LINK_HEALTH_H
