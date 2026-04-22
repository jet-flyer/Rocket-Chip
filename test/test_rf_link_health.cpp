// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_rf_link_health.cpp
 * @brief Host tests for RF link-health state machine (Stage T Batch B, IVP-T14)
 *
 * Covers the pure helpers in src/safety/rf_link_health.h:
 *   - Sliding-window LQ computation.
 *   - State transition predicates: kAcq promotion, kTentative→kTrack,
 *     Schmitt-trigger kTrack↔kTrackDegraded, forced-ACQ cap.
 *   - Anchor-filter scaled arithmetic.
 *   - Deadman threshold.
 *
 * Does NOT test the AO wrapper (ao_rf_manager.cpp) — that exercises the
 * QP framework, verified at bench/HW integration time.
 */

#include <gtest/gtest.h>
#include "safety/rf_link_health.h"

using namespace rc;

// ============================================================================
// Sliding-window LQ
// ============================================================================

TEST(RfLqWindow, EmptyWindowReturnsZero) {
    EXPECT_EQ(rf_lq_compute_pct(0, 0), 0);
}

TEST(RfLqWindow, AllGoodReturns100Pct) {
    uint16_t w = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < kRfLqWindowSize; ++i) {
        w = rf_lq_window_push(w, 1);
        count++;
    }
    EXPECT_EQ(rf_lq_compute_pct(w, count), 100);
}

TEST(RfLqWindow, AllMissReturnsZero) {
    uint16_t w = 0;
    uint8_t count = 0;
    for (uint8_t i = 0; i < kRfLqWindowSize; ++i) {
        w = rf_lq_window_push(w, 0);
        count++;
    }
    EXPECT_EQ(rf_lq_compute_pct(w, count), 0);
}

TEST(RfLqWindow, HalfGoodReturns50Pct) {
    uint16_t w = 0;
    for (uint8_t i = 0; i < 10; ++i) {
        w = rf_lq_window_push(w, i % 2 == 0 ? 1 : 0);
    }
    EXPECT_EQ(rf_lq_compute_pct(w, 10), 50);
}

TEST(RfLqWindow, WindowMasksCorrectly) {
    // Push more bits than window size; oldest should fall off.
    uint16_t w = 0;
    // Push 10 ones to fill window with 100%.
    for (uint8_t i = 0; i < 10; ++i) {
        w = rf_lq_window_push(w, 1);
    }
    EXPECT_EQ(rf_lq_compute_pct(w, 10), 100);
    // Push 5 zeros — should drop to 50%.
    for (uint8_t i = 0; i < 5; ++i) {
        w = rf_lq_window_push(w, 0);
    }
    EXPECT_EQ(rf_lq_compute_pct(w, 10), 50);
    // Push 5 more zeros — should be 0%.
    for (uint8_t i = 0; i < 5; ++i) {
        w = rf_lq_window_push(w, 0);
    }
    EXPECT_EQ(rf_lq_compute_pct(w, 10), 0);
}

// ============================================================================
// State transitions
// ============================================================================

static RfTransitionInput make_input(LinkState s) {
    RfTransitionInput in{};
    in.current = s;
    in.consec_good_rx = 0;
    in.lq_pct = 0;
    in.lq_window_count = 0;
    in.consec_missed_rx = 0;
    in.time_since_last_rx_ms = 0;
    return in;
}

TEST(RfTransition, AcqStaysAcqWithoutRx) {
    // ACQ → ACQ when no RX evidence — promotion happens at the per-RX
    // call site, not in rf_next_state.
    auto in = make_input(LinkState::kAcq);
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq);
}

TEST(RfTransition, TentativeRequires5ConsecutiveGoodAndLq65) {
    auto in = make_input(LinkState::kTentative);
    in.lq_pct = 65;
    in.lq_window_count = 5;

    // Fewer than 5 consec: stay.
    in.consec_good_rx = 4;
    EXPECT_EQ(rf_next_state(in), LinkState::kTentative);

    // 5 consec + LQ 65%: promote.
    in.consec_good_rx = 5;
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfTransition, TentativeDoesNotPromoteIfLqBelow65) {
    auto in = make_input(LinkState::kTentative);
    in.consec_good_rx = 5;
    in.lq_pct = 64;  // just below floor
    in.lq_window_count = 5;
    EXPECT_EQ(rf_next_state(in), LinkState::kTentative);
}

TEST(RfTransition, TrackToDegradedOnLqBelow55) {
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 54;
    in.lq_window_count = kRfLqWindowSize;  // full window required
    EXPECT_EQ(rf_next_state(in), LinkState::kTrackDegraded);
}

TEST(RfTransition, TrackStaysAt55Pct) {
    // Schmitt trigger: the exact boundary LQ=55% must NOT drop (drop is < 55).
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 55;
    in.lq_window_count = kRfLqWindowSize;
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfTransition, TrackStaysWhenWindowNotFilled) {
    // Even at LQ=10%, don't drop until window has filled (prevents
    // spurious drops right after TENTATIVE promotion).
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 10;
    in.lq_window_count = 5;  // not full
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfTransition, DegradedRecoveryAt65Pct) {
    auto in = make_input(LinkState::kTrackDegraded);
    in.lq_pct = 65;
    in.lq_window_count = kRfLqWindowSize;
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfTransition, DegradedStaysAt64Pct) {
    // Schmitt deadband: LQ=64% does not recover (must hit 65).
    auto in = make_input(LinkState::kTrackDegraded);
    in.lq_pct = 64;
    in.lq_window_count = kRfLqWindowSize;
    EXPECT_EQ(rf_next_state(in), LinkState::kTrackDegraded);
}

TEST(RfTransition, DegradedInDeadbandStays) {
    // LQ in [55, 65) must stay in DEGRADED — the deadband.
    for (uint8_t lq = 55; lq < 65; ++lq) {
        auto in = make_input(LinkState::kTrackDegraded);
        in.lq_pct = lq;
        in.lq_window_count = kRfLqWindowSize;
        EXPECT_EQ(rf_next_state(in), LinkState::kTrackDegraded)
            << "at lq=" << int(lq);
    }
}

// ============================================================================
// Forced-ACQ (the bug Round 1 most wanted fixed)
// ============================================================================

TEST(RfForcedAcq, PrimaryRequiresBothTimeAndMinFrames) {
    // Per user direction 2026-04-21: LOS is time-primary WITH a frame
    // minimum to prevent single-dropped-packet false LOS on slow links.
    // Time alone with fewer than N_min drops must NOT trigger.
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = kRfForcedAcqMinFrames - 1;  // 4 (below min)
    in.time_since_last_rx_ms = kRfForcedAcqLosMs + 100;  // past time
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfForcedAcq, TimeAndMinFramesTogetherTrigger) {
    // Both conditions met → LOS.
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = kRfForcedAcqMinFrames;  // 5
    in.time_since_last_rx_ms = kRfForcedAcqLosMs;  // 2000 ms
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq);
}

TEST(RfForcedAcq, MinFramesAloneDoesNotTrigger) {
    // Frame count without time threshold must not trigger primary LOS
    // (only the fast-frames accelerator can fire early).
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = kRfForcedAcqMinFrames;  // 5
    in.time_since_last_rx_ms = 500;  // well under 2 s
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfForcedAcq, FastFramesIsAccelerator) {
    // Optional accelerator: 20 consecutive missed frames triggers ACQ
    // even before the 2 s time threshold. Useful at high nav rates where
    // 20 drops already means something is wrong.
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = kRfForcedAcqFastFrames;  // exactly 20
    in.time_since_last_rx_ms = 500;  // well under 2 s
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq);
}

TEST(RfForcedAcq, StaysBelowBothThresholds) {
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = kRfForcedAcqFastFrames - 1;  // 19
    in.time_since_last_rx_ms = kRfForcedAcqLosMs - 1;  // 1999 ms
    EXPECT_EQ(rf_next_state(in), LinkState::kTrack);
}

TEST(RfForcedAcq, DoesNotTriggerFromAcq) {
    // kAcq → kAcq even with "missed frames" because the concept doesn't
    // apply while we're already acquiring.
    auto in = make_input(LinkState::kAcq);
    in.consec_missed_rx = 100;
    in.time_since_last_rx_ms = 10000;
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq);
}

TEST(RfForcedAcq, LowNavRateFrameMinProtection) {
    // At 1 Hz nav (1000 ms/frame), frame-min=5 means 5 s of silence before
    // LOS declares — longer than the 2 s time threshold alone. Prevents
    // a single dropped packet (1 s silence) from wrongly declaring LOS
    // just because its time slot is long. User direction 2026-04-21:
    // "time with a telem frame minimum might make the most sense."
    auto in = make_input(LinkState::kTrackDegraded);
    in.lq_pct = 55;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = 2;  // 2 missed frames at 1 Hz
    in.time_since_last_rx_ms = 2500;  // over 2 s but only 2 drops
    EXPECT_EQ(rf_next_state(in), LinkState::kTrackDegraded)
        << "2 drops at 1 Hz is not enough to declare LOS — wait for "
           "frame-min";

    // 5 drops at 1 Hz (5 s elapsed): now both conditions satisfied.
    in.consec_missed_rx = 5;
    in.time_since_last_rx_ms = 5000;
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq)
        << "5 drops + 5 s elapsed satisfies both conditions";
}

TEST(RfForcedAcq, HighNavRateTimeTriggers) {
    // At 10 Hz nav (100 ms/frame), the 2 s time threshold corresponds to
    // 20 missed frames. Frame-min (5) is satisfied long before the time
    // threshold, so in practice time dominates at high nav rates.
    auto in = make_input(LinkState::kTrack);
    in.lq_pct = 100;
    in.lq_window_count = kRfLqWindowSize;
    in.consec_missed_rx = 20;  // 2 s worth at 10 Hz
    in.time_since_last_rx_ms = kRfForcedAcqLosMs;
    EXPECT_EQ(rf_next_state(in), LinkState::kAcq);
}

// ============================================================================
// Anchor filter
// ============================================================================

TEST(RfAnchorFilter, ZeroErrorIsZeroCorrection) {
    EXPECT_EQ(rf_anchor_correction_us(0, kRfAlphaInit), 0);
}

TEST(RfAnchorFilter, Alpha25PercentCorrects25PercentOfError) {
    // α = 0.25 (kRfAlphaInit = 2500/10000), error = +1000 us.
    int32_t correction = rf_anchor_correction_us(1000, kRfAlphaInit);
    EXPECT_EQ(correction, 250);
}

TEST(RfAnchorFilter, NegativeErrorNegativeCorrection) {
    int32_t correction = rf_anchor_correction_us(-1000, kRfAlphaInit);
    EXPECT_EQ(correction, -250);
}

TEST(RfAnchorFilter, ConvergesAfterFewSamples) {
    // Starting from 0, with repeated +1000 us errors, α=0.25 filter should
    // converge toward 1000 after several samples.
    int32_t estimate = 0;
    for (int i = 0; i < 20; ++i) {
        int32_t err = 1000 - estimate;
        estimate += rf_anchor_correction_us(err, kRfAlphaInit);
    }
    // After 20 samples at α=0.25, (1 - 0.75^20) ≈ 99.7%, estimate ≈ 997.
    EXPECT_GT(estimate, 990);
    EXPECT_LE(estimate, 1000);
}

// ============================================================================
// Deadman
// ============================================================================

TEST(RfDeadman, FloorAppliesWhenNavRateHigh) {
    // At 10 Hz nav (100 ms period), 5×period = 500 ms = floor.
    // Elapsed 499 ms → not stale.
    EXPECT_FALSE(rf_deadman_fired(499 * 1000U, 100U));
    // Elapsed 501 ms → stale (just past 500 ms floor).
    EXPECT_TRUE(rf_deadman_fired(501 * 1000U, 100U));
}

TEST(RfDeadman, PeriodMultipleDominatesAtLowNav) {
    // At 2 Hz nav (500 ms period), 5×period = 2500 ms > 500 ms floor.
    EXPECT_FALSE(rf_deadman_fired(2499 * 1000U, 500U));
    EXPECT_TRUE(rf_deadman_fired(2501 * 1000U, 500U));
}

TEST(RfDeadman, ZeroElapsedNeverFires) {
    EXPECT_FALSE(rf_deadman_fired(0, 100U));
    EXPECT_FALSE(rf_deadman_fired(0, 500U));
}

// ============================================================================
// Round 2 council consensus specific — Schmitt trigger livelock prevention
// ============================================================================

TEST(RfSchmittTrigger, NoLivelockAtExactlyLq60) {
    // The original pre-Round-2 design used LQ<60 / LQ>=60 which would
    // oscillate at LQ=60. With 55/65 thresholds + deadband, LQ=60 in
    // TRACK stays TRACK and LQ=60 in DEGRADED stays DEGRADED.
    auto track_in = make_input(LinkState::kTrack);
    track_in.lq_pct = 60;
    track_in.lq_window_count = kRfLqWindowSize;
    EXPECT_EQ(rf_next_state(track_in), LinkState::kTrack);

    auto deg_in = make_input(LinkState::kTrackDegraded);
    deg_in.lq_pct = 60;
    deg_in.lq_window_count = kRfLqWindowSize;
    EXPECT_EQ(rf_next_state(deg_in), LinkState::kTrackDegraded);
}

TEST(RfSchmittTrigger, DescendingLqFollowsOneTransitionPath) {
    // From TRACK at LQ=100, descend LQ to 50 — exactly one transition.
    LinkState s = LinkState::kTrack;
    int transitions = 0;
    for (uint8_t lq = 100; lq > 50; --lq) {
        auto in = make_input(s);
        in.lq_pct = lq;
        in.lq_window_count = kRfLqWindowSize;
        LinkState next = rf_next_state(in);
        if (next != s) { transitions++; s = next; }
    }
    EXPECT_EQ(transitions, 1);
    EXPECT_EQ(s, LinkState::kTrackDegraded);
}

TEST(RfSchmittTrigger, AscendingLqFollowsOneTransitionPath) {
    // From DEGRADED at LQ=30, ascend LQ to 80 — exactly one transition.
    LinkState s = LinkState::kTrackDegraded;
    int transitions = 0;
    for (uint8_t lq = 30; lq < 80; ++lq) {
        auto in = make_input(s);
        in.lq_pct = lq;
        in.lq_window_count = kRfLqWindowSize;
        LinkState next = rf_next_state(in);
        if (next != s) { transitions++; s = next; }
    }
    EXPECT_EQ(transitions, 1);
    EXPECT_EQ(s, LinkState::kTrack);
}
