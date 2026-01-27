/**
 * @file AP_AHRS.cpp
 * @brief Minimal AHRS implementation for RocketChip
 *
 * Implements accel-based attitude estimation for compass calibration.
 * Uses complementary filter: fast gyro integration + slow accel correction.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#include "AP_AHRS.h"
#include <cmath>

// Singleton instance
static AP_AHRS s_ahrs_instance;

AP_AHRS& AP_AHRS::get_singleton() {
    return s_ahrs_instance;
}

// ArduPilot-style accessor
namespace AP {
    AP_AHRS& ahrs() {
        return AP_AHRS::get_singleton();
    }
}

AP_AHRS::AP_AHRS()
    : _dcm_matrix()
    , _roll_rad(0.0f)
    , _pitch_rad(0.0f)
    , _yaw_rad(0.0f)
    , _gyro_bias()
    , _initialized(false)
    , _healthy(false)
{
    // Initialize DCM to identity
    _dcm_matrix.identity();
}

void AP_AHRS::init() {
    // Reset state
    _dcm_matrix.identity();
    _roll_rad = 0.0f;
    _pitch_rad = 0.0f;
    _yaw_rad = 0.0f;
    _gyro_bias.zero();
    _initialized = true;
    _healthy = false;  // Will become healthy after first valid update
}

void AP_AHRS::estimate_attitude_from_accel(const Vector3f& accel, float& roll, float& pitch) {
    // Estimate pitch and roll from gravity vector
    // Assumes device is not accelerating (only gravity)

    float accel_length = accel.length();

    // Sanity check - accel should be approximately 1g
    if (accel_length < 0.5f * GRAVITY_MSS || accel_length > 2.0f * GRAVITY_MSS) {
        // Accel reading invalid (device accelerating or sensor error)
        return;
    }

    // Normalize accelerometer reading
    Vector3f accel_norm = accel / accel_length;

    // Roll: rotation about X axis
    // atan2(ay, az) gives roll when nose is level
    roll = atan2f(accel_norm.y, accel_norm.z);

    // Pitch: rotation about Y axis
    // asin(-ax) gives pitch
    // Clamp to avoid NaN from asin
    float ax_clamped = constrain_float(-accel_norm.x, -1.0f, 1.0f);
    pitch = asinf(ax_clamped);
}

void AP_AHRS::update(const Vector3f& accel, const Vector3f& gyro, float dt) {
    if (!_initialized) {
        init();
    }

    if (dt <= 0.0f || dt > 1.0f) {
        // Invalid dt
        return;
    }

    // Remove estimated gyro bias
    Vector3f gyro_corrected = gyro - _gyro_bias;

    // =========================================================================
    // Step 1: Gyro integration (fast, but drifts)
    // =========================================================================

    // Integrate gyro to get delta angles
    float delta_roll = gyro_corrected.x * dt;
    float delta_pitch = gyro_corrected.y * dt;
    float delta_yaw = gyro_corrected.z * dt;

    // Update Euler angles from gyro
    // Note: This is a simplified integration - proper DCM update would use
    // rotation matrix multiplication, but for small angles this is adequate
    _roll_rad += delta_roll;
    _pitch_rad += delta_pitch;
    _yaw_rad += delta_yaw;

    // =========================================================================
    // Step 2: Accelerometer correction (slow, but absolute for pitch/roll)
    // =========================================================================

    float accel_roll, accel_pitch;
    estimate_attitude_from_accel(accel, accel_roll, accel_pitch);

    // Complementary filter: blend gyro (fast) with accel (slow)
    // Higher kAccelWeight = trust accel more (slower response, less drift)
    _roll_rad = _roll_rad * (1.0f - kAccelWeight) + accel_roll * kAccelWeight;
    _pitch_rad = _pitch_rad * (1.0f - kAccelWeight) + accel_pitch * kAccelWeight;

    // =========================================================================
    // Step 3: Normalize angles
    // =========================================================================

    // Keep roll in [-pi, pi]
    while (_roll_rad > M_PI) _roll_rad -= 2.0f * M_PI;
    while (_roll_rad < -M_PI) _roll_rad += 2.0f * M_PI;

    // Keep pitch in [-pi/2, pi/2] (gimbal lock protection)
    _pitch_rad = constrain_float(_pitch_rad, -M_PI_2 + 0.01f, M_PI_2 - 0.01f);

    // Keep yaw in [-pi, pi]
    while (_yaw_rad > M_PI) _yaw_rad -= 2.0f * M_PI;
    while (_yaw_rad < -M_PI) _yaw_rad += 2.0f * M_PI;

    // =========================================================================
    // Step 4: Update DCM matrix from Euler angles
    // =========================================================================

    update_dcm_from_euler();

    _healthy = true;
}

void AP_AHRS::update_dcm_from_euler() {
    // Build DCM from Euler angles (ZYX convention: yaw, pitch, roll)
    // This converts body frame to NED (North-East-Down) frame

    float cp = cosf(_pitch_rad);
    float sp = sinf(_pitch_rad);
    float cr = cosf(_roll_rad);
    float sr = sinf(_roll_rad);
    float cy = cosf(_yaw_rad);
    float sy = sinf(_yaw_rad);

    // DCM = Rz(yaw) * Ry(pitch) * Rx(roll)
    // Row 0
    _dcm_matrix.a.x = cp * cy;
    _dcm_matrix.a.y = cp * sy;
    _dcm_matrix.a.z = -sp;

    // Row 1
    _dcm_matrix.b.x = sr * sp * cy - cr * sy;
    _dcm_matrix.b.y = sr * sp * sy + cr * cy;
    _dcm_matrix.b.z = sr * cp;

    // Row 2
    _dcm_matrix.c.x = cr * sp * cy + sr * sy;
    _dcm_matrix.c.y = cr * sp * sy - sr * cy;
    _dcm_matrix.c.z = cr * cp;
}

bool AP_AHRS::get_location(Location& loc) const {
    // No GPS yet - return false
    // This will be updated when GPS is integrated (Milestone 6)
    (void)loc;
    return false;
}

bool AP_AHRS::get_origin(Location& loc) const {
    // No EKF origin set - return false
    // This will be updated when GPS is integrated (Milestone 6)
    (void)loc;
    return false;
}

void AP_AHRS::set_yaw(float yaw_rad) {
    _yaw_rad = yaw_rad;

    // Normalize to [-pi, pi]
    while (_yaw_rad > M_PI) _yaw_rad -= 2.0f * M_PI;
    while (_yaw_rad < -M_PI) _yaw_rad += 2.0f * M_PI;

    update_dcm_from_euler();
}
