// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-83: Innovation monitor unit tests.
// Validates per-channel sliding-window NIS tracker and Q scale output.

#include "../src/fusion/innovation_monitor.h"

#include <gtest/gtest.h>
#include <cmath>

using namespace rc;

class InnovationMonitorTest : public ::testing::Test {
protected:
    InnovationChannel ch;

    void SetUp() override {
        innovation_channel_init(&ch);
    }
};

// --- Init state ---

TEST_F(InnovationMonitorTest, InitialState) {
    EXPECT_EQ(ch.count, 0);
    EXPECT_EQ(ch.head, 0);
    EXPECT_FLOAT_EQ(ch.alpha, 0.0f);
    EXPECT_FLOAT_EQ(ch.sum, 0.0f);
}

TEST_F(InnovationMonitorTest, EmptyChannelQScale) {
    // Empty channel should return 1.0 (no adaptation)
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

// --- Steady state ---

TEST_F(InnovationMonitorTest, SteadyStateNearOne) {
    // Push 20 samples of NIS=1.0 — alpha should be ~1.0, q_scale=1.0
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 1.0f);
    }
    EXPECT_NEAR(ch.alpha, 1.0f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

TEST_F(InnovationMonitorTest, SteadyStateBelowOne) {
    // Push NIS=0.5 — filter well-tuned, q_scale should be 1.0
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 0.5f);
    }
    EXPECT_NEAR(ch.alpha, 0.5f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

// --- High innovation ---

TEST_F(InnovationMonitorTest, HighInnovationRaisesAlpha) {
    // Push NIS=10.0 — Q is too low
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 10.0f);
    }
    EXPECT_NEAR(ch.alpha, 10.0f, 1e-4f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 10.0f);  // at cap
}

TEST_F(InnovationMonitorTest, ModerateHighInnovation) {
    // Push NIS=3.0 — moderate Q inflation
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 3.0f);
    }
    EXPECT_NEAR(ch.alpha, 3.0f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 3.0f);
}

// --- Cap enforcement ---

TEST_F(InnovationMonitorTest, QScaleCappedAtMax) {
    // Push NIS=100.0 — far above cap
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 100.0f);
    }
    EXPECT_GT(ch.alpha, InnovationChannel::kMaxQInflation);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch),
                    InnovationChannel::kMaxQInflation);
}

// --- Window washout ---

TEST_F(InnovationMonitorTest, WindowWashout) {
    // Fill with NIS=10, then replace with NIS=0.5
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 10.0f);
    }
    EXPECT_NEAR(ch.alpha, 10.0f, 1e-4f);

    // Now push 20 samples of NIS=0.5 — should wash out the 10s
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 0.5f);
    }
    EXPECT_NEAR(ch.alpha, 0.5f, 1e-4f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

TEST_F(InnovationMonitorTest, PartialWashout) {
    // Fill with NIS=1.0
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 1.0f);
    }

    // Replace half with NIS=5.0
    for (int i = 0; i < InnovationChannel::kWindowSize / 2; ++i) {
        innovation_channel_push(&ch, 5.0f);
    }

    // Alpha should be between 1.0 and 5.0
    float expected = (10.0f * 1.0f + 10.0f * 5.0f) / 20.0f;  // 3.0
    EXPECT_NEAR(ch.alpha, expected, 0.1f);
    EXPECT_GT(innovation_channel_q_scale(&ch), 1.0f);
}

// --- Partial fill ---

TEST_F(InnovationMonitorTest, PartialFillAlpha) {
    // Push only 5 samples (window not full)
    for (int i = 0; i < 5; ++i) {
        innovation_channel_push(&ch, 2.0f);
    }
    EXPECT_EQ(ch.count, 5);
    EXPECT_NEAR(ch.alpha, 2.0f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 2.0f);
}

TEST_F(InnovationMonitorTest, SingleSample) {
    innovation_channel_push(&ch, 7.5f);
    EXPECT_EQ(ch.count, 1);
    EXPECT_NEAR(ch.alpha, 7.5f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 7.5f);
}

// --- Edge cases ---

TEST_F(InnovationMonitorTest, RejectsNaN) {
    innovation_channel_push(&ch, 1.0f);
    innovation_channel_push(&ch, std::nanf(""));
    EXPECT_EQ(ch.count, 1);  // NaN rejected, count stays at 1
    EXPECT_NEAR(ch.alpha, 1.0f, 1e-5f);
}

TEST_F(InnovationMonitorTest, RejectsInfinity) {
    innovation_channel_push(&ch, 1.0f);
    innovation_channel_push(&ch, INFINITY);
    EXPECT_EQ(ch.count, 1);
    EXPECT_NEAR(ch.alpha, 1.0f, 1e-5f);
}

TEST_F(InnovationMonitorTest, RejectsNegative) {
    innovation_channel_push(&ch, 1.0f);
    innovation_channel_push(&ch, -1.0f);
    EXPECT_EQ(ch.count, 1);
    EXPECT_NEAR(ch.alpha, 1.0f, 1e-5f);
}

TEST_F(InnovationMonitorTest, ZeroNIS) {
    // NIS=0 is valid (perfect prediction)
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 0.0f);
    }
    EXPECT_NEAR(ch.alpha, 0.0f, 1e-5f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

// --- One-directional property (Council A7) ---

TEST_F(InnovationMonitorTest, NeverDeflatesBelowOne) {
    // Even with alpha well below 1.0, q_scale never goes below 1.0
    for (int i = 0; i < InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 0.1f);
    }
    EXPECT_LT(ch.alpha, 1.0f);
    EXPECT_FLOAT_EQ(innovation_channel_q_scale(&ch), 1.0f);
}

// --- Circular buffer wrap ---

TEST_F(InnovationMonitorTest, CircularWrapCorrectness) {
    // Push 3 full windows worth of data
    for (int i = 0; i < 3 * InnovationChannel::kWindowSize; ++i) {
        innovation_channel_push(&ch, 2.0f);
    }
    EXPECT_EQ(ch.count, InnovationChannel::kWindowSize);
    EXPECT_NEAR(ch.alpha, 2.0f, 1e-4f);
}
