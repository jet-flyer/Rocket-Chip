// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Launch Abort — flight_director module-scope safety flag
//
// Verifies level-3 safety-posture semantics per docs/USER_GUIDE.md
// "Safety State Model":
//   - default false
//   - set sticks
//   - no clear API in production (power-cycle-only)
//   - test-only reset hook for fixture teardown
//============================================================================

#include <gtest/gtest.h>
#include "flight_director/flight_director.h"

namespace rc {
// Test-only reset hook (implementation in flight_director.cpp under
// ROCKETCHIP_HOST_TEST). No production clear path — power cycle only.
void flight_director_test_reset_launch_abort();
}

class LaunchAbortFixture : public ::testing::Test {
  protected:
    void SetUp() override {
        rc::flight_director_test_reset_launch_abort();
    }
    void TearDown() override {
        rc::flight_director_test_reset_launch_abort();
    }
};

TEST_F(LaunchAbortFixture, DefaultFalse) {
    EXPECT_FALSE(rc::flight_director_launch_abort());
}

TEST_F(LaunchAbortFixture, SetSticks) {
    rc::flight_director_set_launch_abort();
    EXPECT_TRUE(rc::flight_director_launch_abort());
    // Repeated reads still true — no auto-clear.
    EXPECT_TRUE(rc::flight_director_launch_abort());
    EXPECT_TRUE(rc::flight_director_launch_abort());
}

TEST_F(LaunchAbortFixture, SetIsIdempotent) {
    rc::flight_director_set_launch_abort();
    rc::flight_director_set_launch_abort();
    rc::flight_director_set_launch_abort();
    EXPECT_TRUE(rc::flight_director_launch_abort());
}
