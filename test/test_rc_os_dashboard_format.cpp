// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project

#include <gtest/gtest.h>
#include <string.h>
#include "cli/rc_os_dashboard_format.h"
#include "rocketchip/vehicle_met.h"
#include "rocketchip/time_sync.h"

TEST(DashFormat, MetZuluMissing) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), true, 12, false, false, 0, 0, 0, false, 0);
    EXPECT_STREQ(buf, "MET +00:00:12  Zulu: --:--:--Z  Local: --:--:--");
}

TEST(DashFormat, MetHeldUntilLiftoff) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), false, 0, false, true, 18, 41, 2, false, 0);
    EXPECT_STREQ(buf, "MET -:--:--:--  Zulu: 18:41:02Z  Local: --:--:--");
}

TEST(DashFormat, MetZuluAndLocalSameWhenOffsetZero) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), true, 75, false, true, 18, 41, 2, true, 0);
    EXPECT_STREQ(buf, "MET +00:01:15  Zulu: 18:41:02Z  Local: 18:41:02");
}

TEST(DashFormat, ClockStripFitsPad) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), true, 5999, false, true, 23, 59, 59, true, -360);
    // Clock is its own line. State is the line under it.
    EXPECT_LE(static_cast<int>(strlen(buf)), 80);
}

TEST(DashFormat, MetZuluAndLocalMdt) {
    char buf[80];
    rc::dash::format_met_zulu(buf, sizeof(buf), true, 0, false, true, 18, 41, 2, true, -360);
    EXPECT_STREQ(buf, "MET +00:00:00  Zulu: 18:41:02Z  Local: 12:41:02");
}

TEST(DashFormat, VehicleMetFollowsDetectionNotTheStation) {
    rc::VehicleMet clock = {};
    const uint8_t idle = static_cast<uint8_t>(rc::FlightPhase::kIdle);
    const uint8_t armed = static_cast<uint8_t>(rc::FlightPhase::kArmed);
    const uint8_t boost = static_cast<uint8_t>(rc::FlightPhase::kBoost);
    const uint8_t coast = static_cast<uint8_t>(rc::FlightPhase::kCoast);
    const uint8_t abort = static_cast<uint8_t>(rc::FlightPhase::kAbort);
    const uint8_t launch = static_cast<uint8_t>(rc::FlightPhase::kBoost);

    auto joined = rc::vehicle_met_step(&clock, launch, coast, 50000);
    EXPECT_FALSE(joined.mission);

    clock = {};
    auto plug = rc::vehicle_met_step(&clock, rc::kMetStartPlugIn, idle, 4000);
    EXPECT_TRUE(plug.mission);
    EXPECT_EQ(plug.met_ms, 4000U);

    clock = {};
    auto ground = rc::vehicle_met_step(&clock, launch, idle, 1000);
    EXPECT_FALSE(ground.mission);
    ground = rc::vehicle_met_step(&clock, launch, armed, 20000);
    EXPECT_FALSE(ground.mission);
    auto liftoff = rc::vehicle_met_step(&clock, launch, boost, 25000);
    EXPECT_TRUE(liftoff.mission);
    EXPECT_EQ(liftoff.met_ms, 0U);
    auto later = rc::vehicle_met_step(&clock, launch, coast, 26500);
    EXPECT_EQ(later.met_ms, 1500U);
    auto ended = rc::vehicle_met_step(&clock, launch, abort, 30000);
    EXPECT_TRUE(ended.mission);
    EXPECT_EQ(ended.met_ms, 5000U);
    auto reset = rc::vehicle_met_step(&clock, launch, idle, 90000);
    EXPECT_FALSE(reset.mission);
}

TEST(DashFormat, StationGpsRowShowsLockAndUptime) {
    char buf[72];
    rc::dash::format_station_gps_row(buf, sizeof(buf), true, 3, 8, true, 42);
    EXPECT_STREQ(buf, "Stn GPS: 3D      8 sat  Up: 42s");
    rc::dash::format_station_gps_row(buf, sizeof(buf), true, 0, 0, false, 3);
    EXPECT_STREQ(buf, "Stn GPS: none    0 sat  Up: 3s  no time");
    rc::dash::format_station_gps_row(buf, sizeof(buf), false, 0, 0, false, 1);
    EXPECT_STREQ(buf, "Stn GPS: no I2C  0 sat  Up: 1s  no time");
}

TEST(DashFormat, MetTicksBetweenPackets) {
    rc::MetRun run = {};
    rc::met_run_observe(&run, true, 1000, 5000);
    EXPECT_EQ(rc::met_run_at(run, 5000), 1000U);
    EXPECT_EQ(rc::met_run_at(run, 6200), 2200U);
    rc::met_run_observe(&run, true, 1000, 7000);
    EXPECT_EQ(rc::met_run_at(run, 7300), 3300U);
    rc::met_run_observe(&run, true, 5000, 8000);
    EXPECT_EQ(rc::met_run_at(run, 8000), 5000U);
    rc::met_run_observe(&run, false, 0, 9000);
    EXPECT_FALSE(run.on);
}

TEST(DashFormat, LinkAgeFoldsMidnight) {
    EXPECT_EQ(rc::link_latency_s(100, 90), 10);
    EXPECT_EQ(rc::link_latency_s(2, 86398), 4);
    uint32_t accepted = 0;
    uint32_t vehicle_now = 0;
    const uint64_t packed = rc::tminus_pack_echo(12345, 80000);
    rc::tminus_unpack_echo(packed, &accepted, &vehicle_now);
    EXPECT_EQ(accepted, 12345U);
    EXPECT_EQ(vehicle_now, 80000U);
}

TEST(DashFormat, GpsTimeDefersOnlyWhenLocalIsMissing) {
    EXPECT_EQ(rc::dash::choose_time_source(true, true, false, false),
              rc::dash::TimePick::kLocalGps);
    EXPECT_EQ(rc::dash::choose_time_source(false, true, false, false),
              rc::dash::TimePick::kPeerGps);
    EXPECT_EQ(rc::dash::choose_time_source(false, true, false, true),
              rc::dash::TimePick::kPeerGps);
    EXPECT_EQ(rc::dash::choose_time_source(false, true, true, true),
              rc::dash::TimePick::kNone);
    EXPECT_EQ(rc::dash::choose_time_source(false, false, false, false),
              rc::dash::TimePick::kNone);
}

TEST(DashFormat, ZuluAdvancesWhenNmeaSecondSticks) {
    rc::dash::ZuluRun run = {};
    bool ok = false;
    uint8_t h = 0;
    uint8_t m = 0;
    uint8_t s = 0;
    rc::dash::zulu_run_apply(&run, true, 18, 41, 2, 1000, &ok, &h, &m, &s);
    EXPECT_TRUE(ok);
    EXPECT_EQ(h, 18);
    EXPECT_EQ(m, 41);
    EXPECT_EQ(s, 2);
    rc::dash::zulu_run_apply(&run, true, 18, 41, 2, 3500, &ok, &h, &m, &s);
    EXPECT_EQ(s, 4);
    rc::dash::zulu_run_apply(&run, true, 18, 41, 9, 4000, &ok, &h, &m, &s);
    EXPECT_EQ(s, 9);
    rc::dash::zulu_run_apply(&run, false, 0, 0, 0, 6000, &ok, &h, &m, &s);
    EXPECT_TRUE(ok);
    EXPECT_EQ(s, 11);
}

TEST(DashFormat, DenverSeptemberIsMdt) {
    const int32_t lat = 397392000;
    const int32_t lon = -1049903000;
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 9, 24, 18, 0, 0), -360);
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 1, 15, 18, 0, 0), -420);
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 3, 8, 8, 59, 0), -420);
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 3, 8, 9, 0, 0), -360);
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 11, 1, 7, 59, 0), -360);
    EXPECT_EQ(rc::dash::tz_offset_min(lat, lon, 2026, 11, 1, 8, 0, 0), -420);
}

TEST(DashFormat, PhoenixDoesNotDoDst) {
    EXPECT_EQ(rc::dash::tz_offset_min(334480000, -1120740000, 2026, 9, 24, 18, 0, 0),
              -420);
}

TEST(DashFormat, LondonBstAndIndiaChina) {
    EXPECT_EQ(rc::dash::tz_offset_min(515000000, -1000000, 2026, 9, 24, 12, 0, 0), 60);
    EXPECT_EQ(rc::dash::tz_offset_min(515000000, -1000000, 2026, 1, 15, 12, 0, 0), 0);
    EXPECT_EQ(rc::dash::tz_offset_min(515000000, -1000000, 2026, 3, 29, 0, 59, 0), 0);
    EXPECT_EQ(rc::dash::tz_offset_min(515000000, -1000000, 2026, 3, 29, 1, 0, 0), 60);
    EXPECT_EQ(rc::dash::tz_offset_min(287000000, 772000000, 2026, 9, 24, 12, 0, 0), 330);
    EXPECT_EQ(rc::dash::tz_offset_min(306000000, 1040000000, 2026, 1, 15, 4, 0, 0), 480);
    EXPECT_EQ(rc::dash::tz_offset_min(0, -1050000000, 2026, 9, 24, 18, 0, 0), -420);
}

TEST(DashFormat, TMinusParsesMinutesAndZulu) {
    uint16_t minutes = 0;
    EXPECT_TRUE(rc::dash::parse_tminus_minutes("15", &minutes));
    EXPECT_EQ(minutes, 15);
    EXPECT_FALSE(rc::dash::parse_tminus_minutes("0", &minutes));
    EXPECT_FALSE(rc::dash::parse_tminus_minutes("1441", &minutes));
    uint8_t h = 0;
    uint8_t m = 0;
    uint8_t s = 0;
    EXPECT_TRUE(rc::dash::parse_tminus_zulu("183000", &h, &m, &s));
    EXPECT_EQ(h, 18);
    EXPECT_EQ(m, 30);
    EXPECT_EQ(s, 0);
    EXPECT_TRUE(rc::dash::parse_tminus_zulu("18:30:00", &h, &m, &s));
    EXPECT_FALSE(rc::dash::parse_tminus_zulu("246000", &h, &m, &s));
}

TEST(DashFormat, TMinusCountsDownAndRollsToNextDay) {
    rc::dash::TMinus t{};
    rc::dash::tminus_arm(&t, 10000, 90);
    EXPECT_EQ(rc::dash::tminus_remaining_s(t, 10000), 90);
    EXPECT_EQ(rc::dash::tminus_remaining_s(t, 40000), 60);
    EXPECT_EQ(rc::dash::tminus_remaining_s(t, 110000), -10);
    const uint32_t now = 18U * 3600U;
    const uint32_t morning = 6U * 3600U;
    EXPECT_EQ(rc::dash::tminus_zulu_delta_s(now, now + 120U), 120U);
    EXPECT_EQ(rc::dash::tminus_zulu_delta_s(now, morning),
              (24U * 3600U - now) + morning);
    char buf[24];
    rc::dash::format_met_mark(buf, sizeof(buf), false, 0, false);
    EXPECT_STREQ(buf, "MET -:--:--:--");
    rc::dash::format_met_mark(buf, sizeof(buf), true, -754, false);
    EXPECT_STREQ(buf, "MET -00:12:34");
    rc::dash::format_met_mark(buf, sizeof(buf), true, 5, false);
    EXPECT_STREQ(buf, "MET +00:00:05");
    // science.nasa.gov: "2/03:45:18 MET" is 2 days, 3 hours, 45 minutes, 18 seconds.
    rc::dash::format_met_mark(buf, sizeof(buf), true,
                              2 * 86400 + 3 * 3600 + 45 * 60 + 18, true);
    EXPECT_STREQ(buf, "MET +2/03:45:18");
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
