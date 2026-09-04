// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Nav SDU packer. STOP-GAP reject-as-PLTU is test_telemetry_encoder.cpp.

#include <gtest/gtest.h>
#include "starcom_adapt/nav_sdu.h"
#include "starcom_adapt/sc_air.h"
#include "rocketchip/telemetry_state.h"
#include <cstring>

using namespace rc;

static TelemetryState make_telem() {
    TelemetryState t{};
    t.q_w = 32767;
    t.lat_1e7 = 474000000;
    t.lon_1e7 = -1222000000;
    t.alt_mm = 100000;
    t.vel_n_cms = 100;
    t.gps_fix_sats = 0x3C;
    t.health = 0xFF;
    t.temperature_c = 22;
    t.battery_mv = 3700;
    t.met_ms = 12345;
    t.flags = kFlagsZuptActive;
    return t;
}

TEST(NavSdu, PackRoundTripFullTelemetryState) {
    const TelemetryState in = make_telem();
    uint8_t buf[kNavSduUserBytes] = {};
    ASSERT_EQ(pack_nav_sdu_user(buf, sizeof(buf), in), kNavSduUserBytes);
    EXPECT_EQ(sizeof(TelemetryState), 45u);
    EXPECT_EQ(kNavSduUserBytes, 45);

    TelemetryState out{};
    out.met_ms = 99;
    out.flags = 0x7F;
    ASSERT_TRUE(unpack_nav_sdu_user(buf, sizeof(buf), &out));
    EXPECT_EQ(out.q_w, in.q_w);
    EXPECT_EQ(out.lat_1e7, in.lat_1e7);
    EXPECT_EQ(out.battery_mv, in.battery_mv);
    EXPECT_EQ(out.met_ms, in.met_ms);
    EXPECT_EQ(out.flags, in.flags);
    EXPECT_EQ(out.flags, kFlagsZuptActive);
}

TEST(NavSdu, PackRejectsShortBuffer) {
    const TelemetryState in = make_telem();
    uint8_t buf[8] = {};
    EXPECT_EQ(pack_nav_sdu_user(buf, sizeof(buf), in), 0);
}

TEST(NavSdu, DefaultBuildAllowsLoraCommands) {
#ifndef ROCKETCHIP_USE_STARCOM
    EXPECT_TRUE(kAirLoraCommandsEnabled);
    EXPECT_FALSE(kStarcomPrepBuild);
#endif
}
