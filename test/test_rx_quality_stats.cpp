// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// RX packet quality stats: count, min, mean, max of RSSI and SNR.
#include <gtest/gtest.h>

#include "diag/rx_quality_stats.h"

#include <stdint.h>

using rc::RxQualStat;
using rc::RxQuality;

TEST(RxQualStat, EmptyIsZero) {
    const RxQualStat s{};
    EXPECT_EQ(s.n, 0U);
    EXPECT_EQ(rc::rx_qual_mean(s), 0);
}

TEST(RxQualStat, FirstValueSetsMinAndMax) {
    RxQualStat s{};
    rc::rx_qual_add(s, -90);
    EXPECT_EQ(s.n, 1U);
    EXPECT_EQ(s.min, -90);
    EXPECT_EQ(s.max, -90);
    EXPECT_EQ(rc::rx_qual_mean(s), -90);
}

TEST(RxQualStat, MinMeanMaxOfNegativeRssi) {
    RxQualStat s{};
    rc::rx_qual_add(s, -100);
    rc::rx_qual_add(s, -80);
    rc::rx_qual_add(s, -91);
    EXPECT_EQ(s.min, -100);
    EXPECT_EQ(s.max, -80);
    EXPECT_EQ(rc::rx_qual_mean(s), -90);  // -271/3 = -90.33
}

TEST(RxQualStat, MeanRoundsHalfAwayFromZero) {
    RxQualStat s{};
    rc::rx_qual_add(s, -3);
    rc::rx_qual_add(s, -4);
    EXPECT_EQ(rc::rx_qual_mean(s), -4);  // -3.5
    RxQualStat t{};
    rc::rx_qual_add(t, 3);
    rc::rx_qual_add(t, 4);
    EXPECT_EQ(rc::rx_qual_mean(t), 4);   // 3.5
}

TEST(RxQualStat, StopsAtCountLimit) {
    RxQualStat s{};
    s.n = UINT32_MAX;
    s.sum = -5;
    s.min = -5;
    s.max = -5;
    rc::rx_qual_add(s, -120);
    EXPECT_EQ(s.n, UINT32_MAX);
    EXPECT_EQ(s.min, -5);
    EXPECT_EQ(s.sum, -5);
}

TEST(RxQuality, NoteAddsRssiAndSnr) {
    RxQuality q{};
    rc::rx_quality_note(q, -110, -7);
    rc::rx_quality_note(q, -100, 9);
    EXPECT_EQ(q.rssi.n, 2U);
    EXPECT_EQ(q.snr.n, 2U);
    EXPECT_EQ(q.rssi.min, -110);
    EXPECT_EQ(q.rssi.max, -100);
    EXPECT_EQ(q.snr.min, -7);
    EXPECT_EQ(q.snr.max, 9);
    EXPECT_EQ(rc::rx_qual_mean(q.snr), 1);
}
