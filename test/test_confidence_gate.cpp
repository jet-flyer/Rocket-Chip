// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-84: Confidence gate unit tests.
// Validates hysteresis, threshold checks, and timing behavior.

#include "../src/fusion/confidence_gate.h"

#include <gtest/gtest.h>

using namespace rc;

// Helper: build a healthy input at a given timestamp
static ConfidenceInput healthy_input(uint32_t now_ms) {
    return {
        .mahony_div_deg = 1.0f,
        .eskf_healthy = true,
        .max_innov_ratio = 0.5f,
        .p_att_max = 0.01f,
        .p_vel_max = 1.0f,
        .now_ms = now_ms,
    };
}

// Helper: build a bad input (AHRS diverged) at a given timestamp
static ConfidenceInput bad_ahrs_input(uint32_t now_ms) {
    ConfidenceInput inp = healthy_input(now_ms);
    inp.mahony_div_deg = 25.0f;  // exceeds kAhrsDivMaxDeg (15°)
    return inp;
}

class ConfidenceGateTest : public ::testing::Test {
protected:
    ConfidenceState cs;

    void SetUp() override {
        confidence_gate_init(&cs);
    }
};

// --- Initial state ---

TEST_F(ConfidenceGateTest, InitiallyConfident) {
    EXPECT_TRUE(cs.confident);
    EXPECT_EQ(cs.time_since_confident_ms, 0u);
    EXPECT_FLOAT_EQ(cs.ahrs_divergence_deg, 0.0f);
}

// --- Normal operation ---

TEST_F(ConfidenceGateTest, StaysConfidentWhenHealthy) {
    for (uint32_t t = 0; t < 10000; t += 10) {
        confidence_gate_evaluate(&cs, healthy_input(t));
    }
    EXPECT_TRUE(cs.confident);
    EXPECT_EQ(cs.time_since_confident_ms, 0u);
}

// --- Loss debounce ---

TEST_F(ConfidenceGateTest, TransientSpikeDoesNotCauseLoss) {
    // 400ms of bad data — below kLossDebounceMs (500ms)
    for (uint32_t t = 0; t < 400; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_TRUE(cs.confident) << "Should survive transient <500ms";

    // Return to healthy
    confidence_gate_evaluate(&cs, healthy_input(500));
    EXPECT_TRUE(cs.confident);
}

TEST_F(ConfidenceGateTest, SustainedBadCausesLoss) {
    // >500ms of bad data
    for (uint32_t t = 0; t <= 600; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_FALSE(cs.confident) << "Should lose confidence after 500ms bad";
    EXPECT_GT(cs.time_since_confident_ms, 0u);
}

// --- Recovery debounce ---

TEST_F(ConfidenceGateTest, RecoveryRequiresDebounce) {
    // Lose confidence
    for (uint32_t t = 0; t <= 600; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_FALSE(cs.confident);

    // 1500ms of good data — below kRecoveryDebounceMs (2000ms)
    for (uint32_t t = 700; t <= 2100; t += 10) {
        confidence_gate_evaluate(&cs, healthy_input(t));
    }
    EXPECT_FALSE(cs.confident) << "Should not recover before 2000ms good";

    // Continue good data to 2800ms (>2000ms good)
    for (uint32_t t = 2200; t <= 2800; t += 10) {
        confidence_gate_evaluate(&cs, healthy_input(t));
    }
    EXPECT_TRUE(cs.confident) << "Should recover after 2000ms good";
}

// --- Individual threshold checks ---

TEST_F(ConfidenceGateTest, EskfUnhealthyCausesLoss) {
    ConfidenceInput inp = healthy_input(0);
    inp.eskf_healthy = false;

    for (uint32_t t = 0; t <= 600; t += 10) {
        inp.now_ms = t;
        confidence_gate_evaluate(&cs, inp);
    }
    EXPECT_FALSE(cs.confident);
}

TEST_F(ConfidenceGateTest, HighInnovRatioCausesLoss) {
    ConfidenceInput inp = healthy_input(0);
    inp.max_innov_ratio = 8.0f;  // exceeds kInnovRatioMax (5.0)

    for (uint32_t t = 0; t <= 600; t += 10) {
        inp.now_ms = t;
        confidence_gate_evaluate(&cs, inp);
    }
    EXPECT_FALSE(cs.confident);
}

TEST_F(ConfidenceGateTest, LargePAttCausesLoss) {
    ConfidenceInput inp = healthy_input(0);
    inp.p_att_max = 1.0f;  // exceeds kPAttMaxRad2 (0.5)

    for (uint32_t t = 0; t <= 600; t += 10) {
        inp.now_ms = t;
        confidence_gate_evaluate(&cs, inp);
    }
    EXPECT_FALSE(cs.confident);
}

TEST_F(ConfidenceGateTest, LargePVelCausesLoss) {
    ConfidenceInput inp = healthy_input(0);
    inp.p_vel_max = 60.0f;  // exceeds kPVelMaxM2s2 (50.0)

    for (uint32_t t = 0; t <= 600; t += 10) {
        inp.now_ms = t;
        confidence_gate_evaluate(&cs, inp);
    }
    EXPECT_FALSE(cs.confident);
}

// --- Time tracking ---

TEST_F(ConfidenceGateTest, TimeSinceConfidentAccumulates) {
    // Lose confidence at t=500
    for (uint32_t t = 0; t <= 600; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_FALSE(cs.confident);

    // Continue bad for 3 more seconds
    for (uint32_t t = 700; t <= 3600; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_GT(cs.time_since_confident_ms, 2500u);
}

TEST_F(ConfidenceGateTest, TimeSinceConfidentResetsOnRecovery) {
    // Lose confidence
    for (uint32_t t = 0; t <= 600; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_FALSE(cs.confident);

    // Recover (>2000ms good)
    for (uint32_t t = 700; t <= 3000; t += 10) {
        confidence_gate_evaluate(&cs, healthy_input(t));
    }
    EXPECT_TRUE(cs.confident);
    EXPECT_EQ(cs.time_since_confident_ms, 0u);
}

// --- AHRS divergence tracking ---

TEST_F(ConfidenceGateTest, AhrsDivergenceTracked) {
    ConfidenceInput inp = healthy_input(100);
    inp.mahony_div_deg = 7.5f;
    confidence_gate_evaluate(&cs, inp);
    EXPECT_FLOAT_EQ(cs.ahrs_divergence_deg, 7.5f);
}

// --- Hysteresis asymmetry ---

TEST_F(ConfidenceGateTest, LossDebounceIsShorterThanRecovery) {
    // This is a design property, not a runtime test — just verify constants
    EXPECT_LT(confidence::kLossDebounceMs, confidence::kRecoveryDebounceMs);
}

// --- Edge: bad data interrupted by single good tick ---

TEST_F(ConfidenceGateTest, InterruptedBadResetsTimer) {
    // 400ms bad
    for (uint32_t t = 0; t < 400; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_TRUE(cs.confident);

    // Single good tick
    confidence_gate_evaluate(&cs, healthy_input(400));

    // 400ms bad again — timer should have reset
    for (uint32_t t = 410; t < 810; t += 10) {
        confidence_gate_evaluate(&cs, bad_ahrs_input(t));
    }
    EXPECT_TRUE(cs.confident) << "Timer should have reset on good tick";
}
