// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// ESKF Runaway-Restart Brake
//
// Migrated 2026-04-22 from watchdog_recovery. Runtime-only brake that
// disables the filter after 5 consecutive divergence events in one
// session. Cleared by eskf_reenable() (CLI-callable subsystem reset).
// Not a safety-posture flag — this is subsystem-internal health.
//============================================================================

#include <gtest/gtest.h>
#include "fusion/eskf_runner.h"

class EskfBrakeFixture : public ::testing::Test {
  protected:
    void SetUp() override {
        eskf_reenable();  // fresh state
    }
    void TearDown() override {
        eskf_reenable();
    }
};

TEST_F(EskfBrakeFixture, DefaultEnabled) {
    EXPECT_FALSE(eskf_is_disabled());
}

TEST_F(EskfBrakeFixture, UnderThresholdStaysEnabled) {
    // 4 divergence events — below the threshold of 5
    eskf_note_divergence();
    eskf_note_divergence();
    eskf_note_divergence();
    eskf_note_divergence();
    EXPECT_FALSE(eskf_is_disabled());
}

TEST_F(EskfBrakeFixture, AtThresholdDisables) {
    // 5 divergence events — hits the threshold
    for (int i = 0; i < 5; ++i) {
        eskf_note_divergence();
    }
    EXPECT_TRUE(eskf_is_disabled());
}

TEST_F(EskfBrakeFixture, ExtraDivergencesStayDisabled) {
    // 10 divergence events — saturates at disabled, no crash
    for (int i = 0; i < 10; ++i) {
        eskf_note_divergence();
    }
    EXPECT_TRUE(eskf_is_disabled());
}

TEST_F(EskfBrakeFixture, ReenableClears) {
    for (int i = 0; i < 5; ++i) {
        eskf_note_divergence();
    }
    ASSERT_TRUE(eskf_is_disabled());

    eskf_reenable();
    EXPECT_FALSE(eskf_is_disabled());

    // Counter also reset: 4 new divergences should not re-disable
    for (int i = 0; i < 4; ++i) {
        eskf_note_divergence();
    }
    EXPECT_FALSE(eskf_is_disabled());
}
