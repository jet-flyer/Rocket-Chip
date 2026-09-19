// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include <gtest/gtest.h>
#include <string.h>
#include "cli/rc_os_dashboard_format.h"

TEST(DashFormat, MetZuluMissing) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), 12300, false, 0, 0, 0, 0);
    EXPECT_STREQ(buf, "MET: 0:12.3  Zulu: --:--:--Z  Local: --:--:--");
}

TEST(DashFormat, MetZuluAndLocalSameWhenOffsetZero) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), 75400, true, 18, 41, 2, 0);
    EXPECT_STREQ(buf, "MET: 1:15.4  Zulu: 18:41:02Z  Local: 18:41:02");
}

TEST(DashFormat, ClockStripFitsPad) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), 5999900, true, 23, 59, 59, -360);
    // "State: " + %-14s + two spaces = 23 visible prefix.
    EXPECT_LE(23 + static_cast<int>(strlen(buf)), 80);
}

TEST(DashFormat, MetZuluAndLocalMdt) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), 0, true, 18, 41, 2, -360);
    EXPECT_STREQ(buf, "MET: 0:00.0  Zulu: 18:41:02Z  Local: 12:41:02");
}

TEST(DashFormat, UtcOffsetWrapsMidnight) {
    uint8_t h = 0;
    uint8_t m = 0;
    uint8_t s = 0;
    rc::dash::utc_plus_minutes(1, 30, 0, -120, &h, &m, &s);
    EXPECT_EQ(h, 23);
    EXPECT_EQ(m, 30);
    EXPECT_EQ(s, 0);
}

TEST(DashFormat, AirOff) {
    char buf[96];
    rc::dash::format_air_row(buf, sizeof(buf), "starcom", false, false, false,
                             false, 0, 0, 0, 0, 0);
    EXPECT_STREQ(buf, "Air: starcom");
}

TEST(DashFormat, AirWaiting) {
    char buf[96];
    rc::dash::format_air_row(buf, sizeof(buf), "starcom", true, false, true,
                             false, 0, 0, 0, 3, 50);
    EXPECT_STREQ(buf, "Air: starcom  COP-P waiting  MAC A/s50");
}

TEST(DashFormat, AirLockIncludesVr) {
    char buf[96];
    rc::dash::format_air_row(buf, sizeof(buf), "starcom", true, true, true,
                             true, 2, 1, 2, 3, 50);
    EXPECT_STREQ(buf,
                 "Air: starcom  COP-P lock  N(R)=2 V(S)=1 V(R)=2  MAC A/s50  nav");
}

TEST(DashFormat, AirPlcwDuringConnectingIsWaiting) {
    char buf[96];
    rc::dash::format_air_row(buf, sizeof(buf), "starcom", true, true, true,
                             true, 0, 0, 0, 2, 13);
    EXPECT_STREQ(buf, "Air: starcom  COP-P waiting  MAC T/s13");
}

TEST(DashFormat, CrcLqIgnoresEmpty) {
    EXPECT_EQ(rc::dash::crc_lq_pct(0, 0), 0);
    EXPECT_EQ(rc::dash::crc_lq_pct(175, 0), 100);
    EXPECT_EQ(rc::dash::crc_lq_pct(90, 10), 90);
}

TEST(DashFormat, RfLinkCopPDoesNotScoreNavHz) {
    char buf[80];
    rc::dash::format_rf_link_row(buf, sizeof(buf), true, true, 100, true, 23);
    EXPECT_STREQ(buf, "RF Link: COP-P  LQ 100%  RX 2.3 Hz  [OK]");
}
