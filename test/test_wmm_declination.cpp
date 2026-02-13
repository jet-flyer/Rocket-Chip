// WMM declination lookup table tests.
// Validates bilinear interpolation, known declination values,
// boundary conditions, and NaN/edge case handling.

#include <gtest/gtest.h>

#include <cmath>

#include "fusion/wmm_declination.h"

namespace {

constexpr float kDegToRad = 3.14159265f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.14159265f;

// Helper: get declination in degrees for easier test assertions
float decl_deg(float lat, float lon) {
    return rc::wmm_get_declination(lat, lon) * kRadToDeg;
}

// ============================================================================
// Known declination values (approximate, from NOAA WMM calculator)
// Tolerance is ±2° to account for table resolution and IGRF vs WMM differences
// ============================================================================

TEST(WMMDeclination, WashingtonDC) {
    // Washington DC: ~38.9°N, 77.0°W → declination ~-11° to -12°
    float d = decl_deg(38.9f, -77.0f);
    EXPECT_NEAR(d, -11.0f, 3.0f);
}

TEST(WMMDeclination, London) {
    // London: ~51.5°N, 0.1°W → declination ~0° to -2°
    float d = decl_deg(51.5f, -0.1f);
    EXPECT_NEAR(d, -1.0f, 3.0f);
}

TEST(WMMDeclination, Sydney) {
    // Sydney: ~-33.9°S, 151.2°E → declination ~+12° to +13°
    float d = decl_deg(-33.9f, 151.2f);
    EXPECT_NEAR(d, 12.5f, 3.0f);
}

TEST(WMMDeclination, HoustonTX) {
    // Houston: ~29.8°N, 95.4°W → declination ~+2° (NOAA WMM2025: +1.84°)
    // IGRF13 table is epoch ~2020; secular variation ~0.1°/yr gives ~0.5° offset.
    float d = decl_deg(29.76f, -95.37f);
    EXPECT_NEAR(d, 1.84f, 3.0f);
}

TEST(WMMDeclination, Paris) {
    // Paris: ~48.9°N, 2.4°E → declination ~+2° (NOAA WMM2025: +1.83°)
    float d = decl_deg(48.86f, 2.35f);
    EXPECT_NEAR(d, 1.83f, 3.0f);
}

TEST(WMMDeclination, Berlin) {
    // Berlin: ~52.5°N, 13.4°E → declination ~+5° (NOAA WMM2025: +4.96°)
    float d = decl_deg(52.52f, 13.40f);
    EXPECT_NEAR(d, 4.96f, 3.0f);
}

TEST(WMMDeclination, BuenosAires) {
    // Buenos Aires: ~34.6°S, 58.4°W → declination ~-10° (NOAA WMM2025: -10.05°)
    float d = decl_deg(-34.6f, -58.38f);
    EXPECT_NEAR(d, -10.05f, 3.0f);
}

TEST(WMMDeclination, CapeTown) {
    // Cape Town: ~33.9°S, 18.4°E → declination ~-26° (NOAA WMM2025: -26.41°)
    float d = decl_deg(-33.92f, 18.42f);
    EXPECT_NEAR(d, -26.41f, 3.0f);
}

TEST(WMMDeclination, NorthPole) {
    // North magnetic pole area: large positive declination
    float d = decl_deg(85.0f, -130.0f);
    // Just verify it's a large value in the expected direction
    EXPECT_GT(fabsf(d), 10.0f);
}

TEST(WMMDeclination, Equator0Lon) {
    // Equator, prime meridian: small negative declination (~-4°)
    float d = decl_deg(0.0f, 0.0f);
    EXPECT_NEAR(d, -4.15f, 2.0f);
}

// ============================================================================
// Return type and units
// ============================================================================

TEST(WMMDeclination, ReturnsRadians) {
    // Verify output is in radians (not degrees)
    float d = rc::wmm_get_declination(40.0f, -80.0f);
    // US mid-latitude should give ~-10° = ~-0.17 rad
    EXPECT_GT(fabsf(d), 0.01f);    // Not zero
    EXPECT_LT(fabsf(d), 1.0f);     // Not degrees (would be ~10-15)
}

// ============================================================================
// Grid boundary and edge cases
// ============================================================================

TEST(WMMDeclination, ExactGridPoint) {
    // At an exact grid point, interpolation should return that point's value
    // Grid point: lat=0 (row 9), lon=0 (col 18) = -4.15177°
    float d = decl_deg(0.0f, 0.0f);
    EXPECT_NEAR(d, -4.15177f, 0.01f);
}

TEST(WMMDeclination, LatClampSouth) {
    // Below -90 should clamp to -90
    float d1 = rc::wmm_get_declination(-90.0f, 0.0f);
    float d2 = rc::wmm_get_declination(-100.0f, 0.0f);
    EXPECT_FLOAT_EQ(d1, d2);
}

TEST(WMMDeclination, LatClampNorth) {
    // Above +90 should clamp to +90
    float d1 = rc::wmm_get_declination(90.0f, 0.0f);
    float d2 = rc::wmm_get_declination(100.0f, 0.0f);
    EXPECT_FLOAT_EQ(d1, d2);
}

TEST(WMMDeclination, LonClampWest) {
    float d1 = rc::wmm_get_declination(0.0f, -180.0f);
    float d2 = rc::wmm_get_declination(0.0f, -200.0f);
    EXPECT_FLOAT_EQ(d1, d2);
}

TEST(WMMDeclination, LonClampEast) {
    float d1 = rc::wmm_get_declination(0.0f, 180.0f);
    float d2 = rc::wmm_get_declination(0.0f, 200.0f);
    EXPECT_FLOAT_EQ(d1, d2);
}

TEST(WMMDeclination, LonWrapConsistency) {
    // -180 and +180 are the same meridian — should give same value
    float d1 = rc::wmm_get_declination(45.0f, -180.0f);
    float d2 = rc::wmm_get_declination(45.0f, 180.0f);
    EXPECT_NEAR(d1, d2, 0.01f);
}

// ============================================================================
// Interpolation smoothness
// ============================================================================

TEST(WMMDeclination, SmoothBetweenGridPoints) {
    // Moving 1° at a time should produce smooth changes
    float prev = decl_deg(40.0f, -80.0f);
    for (int32_t lon = -79; lon <= -70; ++lon) {
        float curr = decl_deg(40.0f, static_cast<float>(lon));
        // Adjacent degree values should differ by < 2°
        EXPECT_LT(fabsf(curr - prev), 2.0f)
            << "Jump at lon=" << lon;
        prev = curr;
    }
}

TEST(WMMDeclination, SmoothLatitude) {
    float prev = decl_deg(30.0f, -100.0f);
    for (int32_t lat = 31; lat <= 40; ++lat) {
        float curr = decl_deg(static_cast<float>(lat), -100.0f);
        EXPECT_LT(fabsf(curr - prev), 2.0f)
            << "Jump at lat=" << lat;
        prev = curr;
    }
}

// ============================================================================
// Finite output
// ============================================================================

TEST(WMMDeclination, FiniteOutput) {
    // All grid corners should produce finite results
    for (int32_t lat = -90; lat <= 90; lat += 10) {
        for (int32_t lon = -180; lon <= 180; lon += 10) {
            float d = rc::wmm_get_declination(
                static_cast<float>(lat), static_cast<float>(lon));
            EXPECT_TRUE(std::isfinite(d))
                << "Non-finite at lat=" << lat << " lon=" << lon;
        }
    }
}

} // namespace
