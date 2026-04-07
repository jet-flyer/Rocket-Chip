// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// IVP-101: 3-axis magnetometer fusion host tests.
// Validates update_mag_3axis(), magnitude gating, state convergence,
// and auto-enable prerequisites.

#include <gtest/gtest.h>
#include <cmath>

#include "fusion/eskf.h"
#include "fusion/wmm_tables.h"

namespace {

constexpr float kDegToRad = 3.14159265f / 180.0f;

// Dallas TX WMM2025 expected field (approx from tables)
static rc::Vec3 dallas_field_ned() {
    return rc::wmm_get_earth_field_ned(33.0f, -97.0f);
}

// ============================================================================
// Basic functionality
// ============================================================================

TEST(Mag3Axis, RejectsWhenInhibited) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);
    // Mag states inhibited by default
    EXPECT_TRUE(eskf.inhibit_mag_states_);

    rc::Vec3 magBody(20.0f, 5.0f, 40.0f);
    rc::Vec3 field = dallas_field_ned();
    EXPECT_FALSE(eskf.update_mag_3axis(magBody, field));
}

TEST(Mag3Axis, AcceptsWhenEnabled) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    // Enable mag states
    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.set_inhibit_mag(false);

    // Feed the predicted field rotated to body (identity attitude = NED = body)
    // At identity quaternion, body frame = NED frame
    rc::Vec3 magBody = field;  // No bias, identity attitude
    EXPECT_TRUE(eskf.update_mag_3axis(magBody, field));
}

TEST(Mag3Axis, ZeroInnovationAtTruth) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.body_mag_bias = rc::Vec3(0.0f, 0.0f, 0.0f);
    eskf.set_inhibit_mag(false);

    // At identity attitude with correct earth_mag and zero bias,
    // the innovation should be near zero
    rc::Vec3 magBody = field;  // Perfect measurement
    bool ok = eskf.update_mag_3axis(magBody, field);
    EXPECT_TRUE(ok);

    // NIS should be very small (near zero innovation)
    EXPECT_LT(eskf.last_mag_nis_, 0.1f);
}

// ============================================================================
// Magnitude gating
// ============================================================================

TEST(Mag3Axis, RejectsTooWeak) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.set_inhibit_mag(false);

    // Measured field at 50% of expected (below 75% gate)
    rc::Vec3 magWeak = field * 0.5f;
    EXPECT_FALSE(eskf.update_mag_3axis(magWeak, field));
}

TEST(Mag3Axis, RejectsTooStrong) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.set_inhibit_mag(false);

    // Measured field at 150% of expected (above 125% gate)
    rc::Vec3 magStrong = field * 1.5f;
    EXPECT_FALSE(eskf.update_mag_3axis(magStrong, field));
}

TEST(Mag3Axis, AcceptsWithinGate) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.set_inhibit_mag(false);

    // 10% stronger — within 25% gate
    rc::Vec3 magSlightlyStrong = field * 1.1f;
    EXPECT_TRUE(eskf.update_mag_3axis(magSlightlyStrong, field));
}

// ============================================================================
// State convergence
// ============================================================================

TEST(Mag3Axis, EarthMagConverges) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    // Start with wrong earth_mag (10 µT offset on each axis)
    eskf.earth_mag = field + rc::Vec3(10.0f, 10.0f, 10.0f);
    eskf.set_inhibit_mag(false);

    // Feed true field for 100 iterations — earth_mag should converge
    for (int32_t i = 0; i < 100; ++i) {
        rc::Vec3 magBody = field;  // Identity attitude, true field
        eskf.update_mag_3axis(magBody, field);
    }

    // Earth mag should be closer to truth after 100 updates
    float errX = fabsf(eskf.earth_mag.x - field.x);
    float errY = fabsf(eskf.earth_mag.y - field.y);
    float errZ = fabsf(eskf.earth_mag.z - field.z);
    EXPECT_LT(errX, 5.0f);  // Within 5 µT of truth
    EXPECT_LT(errY, 5.0f);
    EXPECT_LT(errZ, 5.0f);
}

TEST(Mag3Axis, BodyBiasConverges) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;
    eskf.set_inhibit_mag(false);

    // Inject a 5 µT hard-iron bias in body frame
    rc::Vec3 bias(5.0f, -3.0f, 2.0f);

    for (int32_t i = 0; i < 200; ++i) {
        rc::Vec3 magBody = field + bias;  // Biased measurement
        eskf.update_mag_3axis(magBody, field);
    }

    // Body bias should converge toward the injected bias
    float errX = fabsf(eskf.body_mag_bias.x - bias.x);
    float errY = fabsf(eskf.body_mag_bias.y - bias.y);
    float errZ = fabsf(eskf.body_mag_bias.z - bias.z);
    EXPECT_LT(errX, 3.0f);
    EXPECT_LT(errY, 3.0f);
    EXPECT_LT(errZ, 3.0f);
}

// ============================================================================
// Input validation
// ============================================================================

TEST(Mag3Axis, RejectsNaN) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);
    eskf.set_inhibit_mag(false);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;

    rc::Vec3 magNaN(NAN, 20.0f, 40.0f);
    EXPECT_FALSE(eskf.update_mag_3axis(magNaN, field));
}

TEST(Mag3Axis, RejectsInf) {
    rc::ESKF eskf;
    rc::Vec3 accel(0.0f, 0.0f, -9.80665f);
    rc::Vec3 gyro(0.0f, 0.0f, 0.0f);
    eskf.init(accel, gyro);
    eskf.set_inhibit_mag(false);

    rc::Vec3 field = dallas_field_ned();
    eskf.earth_mag = field;

    rc::Vec3 magInf(INFINITY, 20.0f, 40.0f);
    EXPECT_FALSE(eskf.update_mag_3axis(magInf, field));
}

// ============================================================================
// WMM table integration
// ============================================================================

TEST(Mag3Axis, WmmFieldNedNonZero) {
    rc::Vec3 field = dallas_field_ned();
    // Dallas should have a reasonable field (~47 µT total)
    float mag = field.norm();
    EXPECT_GT(mag, 30.0f);   // At least 30 µT
    EXPECT_LT(mag, 70.0f);   // At most 70 µT
}

TEST(Mag3Axis, WmmFieldNedDownwardInclination) {
    // In northern hemisphere, Z (down) component should be positive
    rc::Vec3 field = rc::wmm_get_earth_field_ned(45.0f, -90.0f);  // Central US
    EXPECT_GT(field.z, 0.0f);  // Z is down-positive
}

}  // namespace
