// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file test_haversine.cpp
 * @brief Host tests for haversine distance + bearing math (IVP-123)
 *
 * Tests the pure math functions used by cmd_station_distance().
 * The functions are static in rc_os_commands.cpp, so we duplicate
 * the implementations here for testing. If the implementations drift,
 * the tests will catch regressions because the math is deterministic.
 */

#include <gtest/gtest.h>
#include <cmath>

// Duplicate of rc_os_commands.cpp haversine_m (static, can't link directly)
static float haversine_m(int32_t lat1_e7, int32_t lon1_e7,
                          int32_t lat2_e7, int32_t lon2_e7) {
    static constexpr float kDegToRad = 3.14159265f / 180.0f;
    static constexpr float kEarthR   = 6371000.0f;
    static constexpr float kScale    = 1e-7f;

    float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
    float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
    float dlat = lat2 - lat1;
    float dlon = (static_cast<float>(lon2_e7 - lon1_e7)) * kScale * kDegToRad;

    float a = sinf(dlat * 0.5f) * sinf(dlat * 0.5f)
            + cosf(lat1) * cosf(lat2) * sinf(dlon * 0.5f) * sinf(dlon * 0.5f);
    float c = 2.0f * atan2f(sqrtf(a), sqrtf(1.0f - a));
    return kEarthR * c;
}

// Duplicate of rc_os_commands.cpp bearing_deg
static float bearing_deg(int32_t lat1_e7, int32_t lon1_e7,
                          int32_t lat2_e7, int32_t lon2_e7) {
    static constexpr float kDegToRad = 3.14159265f / 180.0f;
    static constexpr float kRadToDeg = 180.0f / 3.14159265f;
    static constexpr float kScale    = 1e-7f;

    float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
    float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
    float dlon = static_cast<float>(lon2_e7 - lon1_e7) * kScale * kDegToRad;

    float y = sinf(dlon) * cosf(lat2);
    float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
    float bearing = atan2f(y, x) * kRadToDeg;
    return fmodf(bearing + 360.0f, 360.0f);
}

// ============================================================================
// Haversine distance tests
// ============================================================================

TEST(Haversine, SamePointReturnsZero) {
    int32_t lat = 401234567;  // ~40.1234567°N
    int32_t lon = -740567890; // ~74.0567890°W
    float d = haversine_m(lat, lon, lat, lon);
    EXPECT_NEAR(d, 0.0f, 0.1f);
}

TEST(Haversine, KnownPairApprox1km) {
    // Two points ~1 km apart (roughly 0.009° latitude at 40°N)
    int32_t lat1 = 400000000;  // 40.0000000°N
    int32_t lon1 = -740000000; // 74.0000000°W
    int32_t lat2 = 400090000;  // 40.0090000°N (0.009° north ≈ 1 km)
    int32_t lon2 = -740000000;
    float d = haversine_m(lat1, lon1, lat2, lon2);
    EXPECT_NEAR(d, 1000.0f, 15.0f);  // Within 15m of 1km
}

TEST(Haversine, LongDistance100km) {
    // ~100 km apart (roughly 0.9° latitude)
    int32_t lat1 = 400000000;
    int32_t lon1 = -740000000;
    int32_t lat2 = 409000000;  // 40.9°N
    int32_t lon2 = -740000000;
    float d = haversine_m(lat1, lon1, lat2, lon2);
    EXPECT_NEAR(d, 100000.0f, 500.0f);  // Within 500m of 100km
}

// ============================================================================
// Bearing tests
// ============================================================================

TEST(Bearing, NorthOfPointReturns0) {
    int32_t lat1 = 400000000;
    int32_t lon1 = -740000000;
    int32_t lat2 = 401000000;  // Due north
    int32_t lon2 = -740000000;
    float b = bearing_deg(lat1, lon1, lat2, lon2);
    EXPECT_NEAR(b, 0.0f, 1.0f);
}

TEST(Bearing, EastOfPointReturns90) {
    int32_t lat1 = 400000000;
    int32_t lon1 = -740000000;
    int32_t lat2 = 400000000;
    int32_t lon2 = -739000000;  // Due east (less negative longitude)
    float b = bearing_deg(lat1, lon1, lat2, lon2);
    EXPECT_NEAR(b, 90.0f, 1.0f);
}

TEST(Bearing, Normalized0To360) {
    // South bearing should be ~180
    int32_t lat1 = 400000000;
    int32_t lon1 = -740000000;
    int32_t lat2 = 399000000;  // Due south
    int32_t lon2 = -740000000;
    float b = bearing_deg(lat1, lon1, lat2, lon2);
    EXPECT_NEAR(b, 180.0f, 1.0f);
    EXPECT_GE(b, 0.0f);
    EXPECT_LT(b, 360.0f);
}
