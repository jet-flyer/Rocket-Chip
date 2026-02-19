// IVP-46: ESKF GPS Position & Velocity Measurement Update Tests
//
// Verifies: NED origin management, geodetic-to-NED conversion,
// GPS position update (3 sequential scalars), GPS velocity update
// (2 sequential scalars), innovation gating, convergence, NIS tracking.
//
// Reference: Solà (2017) §7.2, ArduPilot EKF3 FuseVelPosNED(),
// council conditions C-1 through C-8.

#include <gtest/gtest.h>
#include "fusion/eskf.h"
#include "fusion/eskf_state.h"
#include "math/vec3.h"

#include <cmath>
#include <limits>

using rc::ESKF;
using rc::Vec3;

// ============================================================================
// Helpers
// ============================================================================

static constexpr float kG = 9.80665f;
static const Vec3 kAccelStationary(0.0f, 0.0f, -kG);
static const Vec3 kGyroZero(0.0f, 0.0f, 0.0f);
static constexpr float kDt = 0.005f;  // 200 Hz
static constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;

// Reference point: approximately Boulder, CO (40.0°N, 105.2°W, 1600m)
static constexpr double kRefLatDeg = 40.0;
static constexpr double kRefLonDeg = -105.2;
static constexpr double kRefLatRad = kRefLatDeg * kDeg2Rad;
static constexpr double kRefLonRad = kRefLonDeg * kDeg2Rad;
static constexpr float kRefAltM = 1600.0f;

static ESKF make_eskf() {
    ESKF eskf;
    EXPECT_TRUE(eskf.init(kAccelStationary, kGyroZero));
    return eskf;
}

// Helper: make an initialized ESKF with origin set at reference point
static ESKF make_eskf_with_origin() {
    ESKF eskf = make_eskf();
    EXPECT_TRUE(eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 1.0f));
    return eskf;
}

// ============================================================================
// Origin tests
// ============================================================================

TEST(ESKFGpsOrigin, SetOrigin) {
    auto eskf = make_eskf();
    EXPECT_FALSE(eskf.has_origin_);

    bool result = eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 1.0f);
    EXPECT_TRUE(result);
    EXPECT_TRUE(eskf.has_origin_);

    // Geodetic-to-NED at origin should return ~zero
    Vec3 ned = eskf.geodetic_to_ned(kRefLatRad, kRefLonRad, kRefAltM);
    EXPECT_NEAR(ned.x, 0.0f, 0.01f);
    EXPECT_NEAR(ned.y, 0.0f, 0.01f);
    EXPECT_NEAR(ned.z, 0.0f, 0.01f);
}

TEST(ESKFGpsOrigin, RejectsBadHdop) {
    auto eskf = make_eskf();

    // HDOP=5.0 exceeds kGpsMaxHdopForOrigin=4.0 — should reject
    bool result = eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 5.0f);
    EXPECT_FALSE(result);
    EXPECT_FALSE(eskf.has_origin_);
}

TEST(ESKFGpsOrigin, SetOriginTwiceFails) {
    auto eskf = make_eskf();

    EXPECT_TRUE(eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 1.0f));
    // Second call should fail — origin already set
    EXPECT_FALSE(eskf.set_origin(kRefLatRad + 0.001, kRefLonRad, kRefAltM, 1.0f));
}

TEST(ESKFGpsOrigin, ResetOriginContinuity) {
    // Council C-7: after reset_origin, absolute position must be preserved.
    auto eskf = make_eskf_with_origin();

    // Move the filter position to 100m north, 50m east, 10m down
    eskf.p = Vec3(100.0f, 50.0f, 10.0f);

    // Compute absolute geodetic before reset (for verification)
    // abs_lat ≈ origin_lat + p.x / R_earth
    // abs_lon ≈ origin_lon + p.y / (R_earth * cos(lat))
    // abs_alt ≈ origin_alt - p.z
    double abs_lat_before = kRefLatRad
        + static_cast<double>(eskf.p.x) / static_cast<double>(ESKF::kEarthRadius);
    double abs_lon_before = kRefLonRad
        + static_cast<double>(eskf.p.y) / (static_cast<double>(ESKF::kEarthRadius)
            * static_cast<double>(cosf(static_cast<float>(kRefLatRad))));
    float abs_alt_before = kRefAltM - eskf.p.z;

    // Reset origin to a new point (slightly offset)
    double new_lat = kRefLatRad + 0.0001;  // ~11m north
    double new_lon = kRefLonRad + 0.0001;  // ~8.5m east
    float new_alt = kRefAltM + 5.0f;
    eskf.reset_origin(new_lat, new_lon, new_alt);

    // Recompute absolute geodetic after reset
    double abs_lat_after = eskf.origin_lat_rad_
        + static_cast<double>(eskf.p.x) / static_cast<double>(ESKF::kEarthRadius);
    double abs_lon_after = eskf.origin_lon_rad_
        + static_cast<double>(eskf.p.y) / (static_cast<double>(ESKF::kEarthRadius)
            * static_cast<double>(eskf.cos_origin_lat_));
    float abs_alt_after = eskf.origin_alt_m_ - eskf.p.z;

    // Absolute position should be preserved within float precision (~1m)
    double dlat_m = (abs_lat_after - abs_lat_before) * static_cast<double>(ESKF::kEarthRadius);
    double dlon_m = (abs_lon_after - abs_lon_before) * static_cast<double>(ESKF::kEarthRadius)
                     * static_cast<double>(eskf.cos_origin_lat_);
    EXPECT_NEAR(dlat_m, 0.0, 1.0);    // < 1m error
    EXPECT_NEAR(dlon_m, 0.0, 1.0);    // < 1m error
    EXPECT_NEAR(abs_alt_after, abs_alt_before, 0.1f);
}

TEST(ESKFGpsOrigin, GeodeticToNed_KnownOffset) {
    auto eskf = make_eskf_with_origin();

    // 1° north ≈ R_earth * π/180 ≈ 111,319m
    double one_deg_north = kRefLatRad + kDeg2Rad;
    Vec3 ned = eskf.geodetic_to_ned(one_deg_north, kRefLonRad, kRefAltM);

    // Expected: ~111,319m north
    EXPECT_NEAR(ned.x, 111319.0f, 200.0f);  // Allow ~200m tolerance (flat-earth)
    EXPECT_NEAR(ned.y, 0.0f, 1.0f);
    EXPECT_NEAR(ned.z, 0.0f, 0.01f);
}

TEST(ESKFGpsOrigin, GeodeticToNed_DoublePrecision) {
    // Council C-5: verify that small lat offset (1e-7 deg) produces non-zero NED.
    // Float32 subtraction would lose these small differences.
    auto eskf = make_eskf_with_origin();

    // 1e-7 degrees ≈ 0.011m north — below float32 resolution for subtraction
    // at 40° latitude, but double handles it.
    double tiny_offset = 1e-7 * kDeg2Rad;
    Vec3 ned = eskf.geodetic_to_ned(kRefLatRad + tiny_offset, kRefLonRad, kRefAltM);

    // Should be ~0.011m, definitely > 0.001m
    EXPECT_GT(ned.x, 0.001f);
    EXPECT_NEAR(ned.y, 0.0f, 0.001f);
}

TEST(ESKFGpsOrigin, GeodeticToNed_NoOriginReturnsZero) {
    auto eskf = make_eskf();
    EXPECT_FALSE(eskf.has_origin_);

    Vec3 ned = eskf.geodetic_to_ned(kRefLatRad, kRefLonRad, kRefAltM);
    EXPECT_FLOAT_EQ(ned.x, 0.0f);
    EXPECT_FLOAT_EQ(ned.y, 0.0f);
    EXPECT_FLOAT_EQ(ned.z, 0.0f);
}

// Council requirement: Reproduce the previous bNIS explosion failure scenario.
// 100 predict cycles → set_origin → zero p/v → inject baro → verify bNIS < 5.
// Without P reset in set_origin(), stale cross-covariances corrupt baro Kalman gain.
TEST(ESKFGpsOrigin, PostOriginBaroStability) {
    ESKF eskf = make_eskf();

    // Run 100 predict cycles to accumulate drift (like indoor boot)
    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, kDt);
    }

    // Set origin (must also reset P for position/velocity states)
    EXPECT_TRUE(eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 1.0f));
    eskf.p = Vec3();
    eskf.v = Vec3();

    // Inject baro measurement — must NOT explode
    bool accepted = eskf.update_baro(0.0f);  // altitude = 0 (at origin)
    EXPECT_TRUE(accepted);
    EXPECT_LT(eskf.last_baro_nis_, 5.0f);  // bNIS must be sane
    EXPECT_TRUE(eskf.healthy());
}

// ============================================================================
// GPS position update tests
// ============================================================================

TEST(ESKFGpsPosition, Basic) {
    auto eskf = make_eskf_with_origin();

    // GPS says we are 10m north — initial position is (0,0,0)
    const float P33_before = eskf.P(3, 3);
    Vec3 gps_ned(10.0f, 0.0f, 0.0f);

    bool accepted = eskf.update_gps_position(gps_ned, 1.0f);
    EXPECT_TRUE(accepted);

    // Position should move toward GPS measurement
    EXPECT_GT(eskf.p.x, 0.0f);

    // P[3][3] should decrease
    EXPECT_LT(eskf.P(3, 3), P33_before);

    // NIS should be stored and positive
    EXPECT_GT(eskf.last_gps_pos_nis_, 0.0f);
}

TEST(ESKFGpsPosition, InnovationGating) {
    auto eskf = make_eskf_with_origin();

    const Vec3 p_before = eskf.p;

    // 10km outlier — should be gated out (5σ with σ=3.5m → gate ~50m)
    Vec3 gps_far(10000.0f, 0.0f, 0.0f);
    bool accepted = eskf.update_gps_position(gps_far, 1.0f);

    // update_gps_position returns true (it ran), but position shouldn't move much
    // because the individual scalar updates are gated per-axis.
    // With P after set_origin = r_pos^2 = 12.25, R=12.25, S~24.5, gate=5*sqrt(24.5)≈24.7.
    // Innovation=10000 >> 24.7, so gated out.
    EXPECT_NEAR(eskf.p.x, p_before.x, 0.01f);
}

TEST(ESKFGpsPosition, Convergence) {
    auto eskf = make_eskf_with_origin();

    const Vec3 target(5.0f, -3.0f, 2.0f);
    const float P33_initial = eskf.P(3, 3);

    for (int i = 0; i < 50; ++i) {
        eskf.update_gps_position(target, 1.0f);
    }

    // Position should converge within ~5m of target
    EXPECT_NEAR(eskf.p.x, target.x, 5.0f);
    EXPECT_NEAR(eskf.p.y, target.y, 5.0f);
    EXPECT_NEAR(eskf.p.z, target.z, 5.0f);

    // P[3][3] should have shrunk significantly
    EXPECT_LT(eskf.P(3, 3), P33_initial * 0.5f);
}

TEST(ESKFGpsPosition, HdopScaling) {
    // Higher HDOP = larger R = less correction per update
    auto eskf_low = make_eskf_with_origin();
    auto eskf_high = make_eskf_with_origin();

    Vec3 gps_ned(10.0f, 0.0f, 0.0f);

    eskf_low.update_gps_position(gps_ned, 1.0f);   // HDOP=1 → R=12.25
    eskf_high.update_gps_position(gps_ned, 3.0f);   // HDOP=3 → R=110.25

    // Low HDOP should correct more
    EXPECT_GT(eskf_low.p.x, eskf_high.p.x);
}

TEST(ESKFGpsPosition, RejectsNaN) {
    auto eskf = make_eskf_with_origin();
    Vec3 gps_nan(NAN, 0.0f, 0.0f);
    EXPECT_FALSE(eskf.update_gps_position(gps_nan));
}

TEST(ESKFGpsPosition, RejectsWithoutOrigin) {
    auto eskf = make_eskf();
    EXPECT_FALSE(eskf.has_origin_);

    Vec3 gps_ned(10.0f, 0.0f, 0.0f);
    EXPECT_FALSE(eskf.update_gps_position(gps_ned));
}

// ============================================================================
// GPS velocity update tests
// ============================================================================

TEST(ESKFGpsVelocity, Basic) {
    auto eskf = make_eskf();

    // Inject known NE velocity — filter velocity should converge toward it
    const float P66_before = eskf.P(6, 6);

    bool accepted = eskf.update_gps_velocity(2.0f, 1.0f);
    EXPECT_TRUE(accepted);

    // Velocity should move toward measurement
    EXPECT_GT(eskf.v.x, 0.0f);  // v_north > 0
    EXPECT_GT(eskf.v.y, 0.0f);  // v_east > 0

    // P[6][6] should decrease
    EXPECT_LT(eskf.P(6, 6), P66_before);

    // NIS should be stored
    EXPECT_GT(eskf.last_gps_vel_nis_, 0.0f);
}

TEST(ESKFGpsVelocity, InnovationGating) {
    auto eskf = make_eskf();
    const Vec3 v_before = eskf.v;

    // 100 m/s outlier — with P_init_vel=1, R=0.25, S=1.25, gate=5*sqrt(1.25)≈5.6
    // Innovation=100 >> 5.6, gated out.
    eskf.update_gps_velocity(100.0f, 0.0f);

    EXPECT_NEAR(eskf.v.x, v_before.x, 0.01f);
}

TEST(ESKFGpsVelocity, RejectsNaN) {
    auto eskf = make_eskf();
    EXPECT_FALSE(eskf.update_gps_velocity(NAN, 0.0f));
    EXPECT_FALSE(eskf.update_gps_velocity(0.0f, NAN));
}

// ============================================================================
// Full GPS pipeline integration test
// ============================================================================

TEST(ESKFGpsPipeline, FullConvergence) {
    auto eskf = make_eskf();

    // Set origin
    EXPECT_TRUE(eskf.set_origin(kRefLatRad, kRefLonRad, kRefAltM, 1.0f));

    // Target: 20m north, 10m east, 0m down from origin, moving 1 m/s north
    Vec3 gps_target(20.0f, 10.0f, 0.0f);
    float v_north = 1.0f;
    float v_east = 0.0f;

    // 100 cycles: predict + GPS position + GPS velocity
    for (int i = 0; i < 100; ++i) {
        eskf.predict(kAccelStationary, kGyroZero, kDt);
        eskf.update_gps_position(gps_target, 1.0f);
        eskf.update_gps_velocity(v_north, v_east);
    }

    // Position should converge near target
    EXPECT_NEAR(eskf.p.x, gps_target.x, 5.0f);
    EXPECT_NEAR(eskf.p.y, gps_target.y, 5.0f);

    // Velocity should converge near GPS velocity
    EXPECT_NEAR(eskf.v.x, v_north, 0.5f);
    EXPECT_NEAR(eskf.v.y, v_east, 0.5f);

    // Filter should be healthy
    EXPECT_TRUE(eskf.healthy());

    // P should be bounded
    EXPECT_GT(eskf.P(3, 3), 0.0f);
    EXPECT_LT(eskf.P(3, 3), ESKF::kClampPPosition);
}
