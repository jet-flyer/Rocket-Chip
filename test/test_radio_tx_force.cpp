// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Pure latch and failure count for one forced radio HW TX timeout. No SPI.

#include <gtest/gtest.h>

#include "active_objects/radio_tx_force.h"

TEST(RadioTxForce, FailedStartLeavesLatchArmed) {
    RadioTxForceLatch lat{};
    radio_tx_force_arm(lat);
    EXPECT_FALSE(radio_tx_force_on_start(lat, false));
    EXPECT_TRUE(lat.armed);
    EXPECT_FALSE(lat.marked);
}

TEST(RadioTxForce, ForcedTimeoutDoesNotCountThenRealCountsOne) {
    RadioTxForceLatch lat{};
    radio_tx_force_arm(lat);
    EXPECT_TRUE(radio_tx_force_on_start(lat, true));
    EXPECT_FALSE(lat.armed);
    EXPECT_TRUE(lat.marked);

    uint8_t consec = 0;
    EXPECT_FALSE(radio_tx_timeout_counts(lat, consec));
    EXPECT_EQ(consec, 0);
    EXPECT_FALSE(lat.marked);

    EXPECT_TRUE(radio_tx_timeout_counts(lat, consec));
    EXPECT_EQ(consec, 1);
}

TEST(RadioTxForce, LatchArmsOnlyTheNextStart) {
    RadioTxForceLatch lat{};
    radio_tx_force_arm(lat);
    EXPECT_TRUE(radio_tx_force_on_start(lat, true));
    EXPECT_FALSE(radio_tx_force_on_start(lat, true));
}
