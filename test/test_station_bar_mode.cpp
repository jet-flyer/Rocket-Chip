// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include <gtest/gtest.h>
#include "active_objects/station_bar_mode.h"

TEST(StationBarMode, NoRxIsNoSignal) {
    EXPECT_EQ(station_bar_mode(false, 0, 0), StationBarMode::NoSignal);
    EXPECT_EQ(station_bar_mode(true, 0, 0), StationBarMode::NoSignal);
}

TEST(StationBarMode, RfNoLockIsRfHeard) {
    EXPECT_EQ(station_bar_mode(false, 3, 0), StationBarMode::RfHeard);
    EXPECT_EQ(station_bar_mode(false, 10, 1999), StationBarMode::RfHeard);
}

TEST(StationBarMode, LockAndFreshRxIsLocked) {
    EXPECT_EQ(station_bar_mode(true, 3, 0), StationBarMode::Locked);
    EXPECT_EQ(station_bar_mode(true, 10, 1999), StationBarMode::Locked);
}

TEST(StationBarMode, DropAfterLiveStaysWaiting) {
    EXPECT_EQ(station_bar_mode(false, 10, 2000), StationBarMode::Waiting);
    EXPECT_EQ(station_bar_mode(true, 10, 2000), StationBarMode::Waiting);
    EXPECT_EQ(station_bar_mode(true, 10, 5000), StationBarMode::Waiting);
    EXPECT_EQ(station_bar_mode(true, 10, 60000), StationBarMode::Waiting);
}
