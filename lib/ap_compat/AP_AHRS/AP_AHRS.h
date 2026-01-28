/**
 * @file AP_AHRS.h
 * @brief Minimal AHRS implementation for RocketChip
 *
 * Provides attitude estimation using accelerometer for pitch/roll (gravity)
 * and gyroscope integration for yaw. This is the production-correct bootstrap
 * architecture for compass calibration before full EKF is available.
 *
 * Key insight: Compass calibration only needs to know which way is UP (gravity),
 * not heading. Accel-derived attitude breaks the circular dependency between
 * compass calibration and AHRS.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

#include "AP_AHRS_config.h"
#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <cstdint>

/**
 * @brief Minimal AHRS for attitude estimation
 *
 * Implements subset of ArduPilot's AP_AHRS interface needed by CompassCalibrator:
 * - get_DCM_rotation_body_to_ned() for attitude sampling
 * - get_location() / get_origin() for WMM lookup (returns false until GPS)
 *
 * Attitude estimation:
 * - Pitch/roll: Derived from accelerometer (gravity vector)
 * - Yaw: Integrated from gyroscope (relative, not absolute heading)
 */
class AP_AHRS {
public:
    /**
     * @brief Get singleton instance
     * @return Reference to global AHRS instance
     */
    static AP_AHRS& get_singleton();

    /**
     * @brief Initialize AHRS
     * Call once at startup after HAL is initialized
     */
    void init();

    /**
     * @brief Update attitude from sensor data
     *
     * Call this at sensor rate (~100Hz) to update attitude estimate.
     *
     * @param accel Accelerometer reading in m/s² (body frame)
     * @param gyro Gyroscope reading in rad/s (body frame)
     * @param dt Time delta in seconds since last update
     */
    void update(const Vector3f& accel, const Vector3f& gyro, float dt);

    /**
     * @brief Get DCM rotation matrix (body to NED frame)
     *
     * This is the primary interface used by CompassCalibrator for
     * storing attitude samples alongside magnetometer readings.
     *
     * @return Const reference to 3x3 DCM matrix
     */
    const Matrix3f& get_DCM_rotation_body_to_ned() const { return _dcm_matrix; }

    /**
     * @brief Get roll angle
     * @return Roll in radians, positive = right wing down
     */
    float get_roll_rad() const { return _roll_rad; }

    /**
     * @brief Get pitch angle
     * @return Pitch in radians, positive = nose up
     */
    float get_pitch_rad() const { return _pitch_rad; }

    /**
     * @brief Get yaw angle
     * @return Yaw in radians, positive = clockwise from north (relative)
     * @note Without magnetometer or GPS, this is relative yaw from startup
     */
    float get_yaw_rad() const { return _yaw_rad; }

    /**
     * @brief Get current GPS location
     *
     * @param loc Output location structure (unchanged if returns false)
     * @return true if valid location available, false otherwise
     * @note Returns false until GPS is integrated
     */
    bool get_location(Location& loc) const;

    /**
     * @brief Get EKF origin location
     *
     * @param loc Output location structure (unchanged if returns false)
     * @return true if origin is set, false otherwise
     * @note Returns false until GPS is integrated
     */
    bool get_origin(Location& loc) const;

    /**
     * @brief Check if AHRS is healthy
     * @return true if attitude estimate is valid
     */
    bool healthy() const { return _healthy; }

    /**
     * @brief Set yaw to specific value
     *
     * Used to reset yaw reference (e.g., at calibration start).
     *
     * @param yaw_rad Yaw angle in radians
     */
    void set_yaw(float yaw_rad);

    /**
     * @brief Reset gyro drift estimate
     *
     * Called after gyro calibration to reset accumulated drift.
     * @note Stub implementation - does nothing in minimal AHRS
     */
    void reset_gyro_drift() { _gyro_bias.zero(); }

    /**
     * @brief Get attitude trim (level calibration offset)
     * @return Vector3f with roll, pitch trim in radians
     */
    const Vector3f& get_trim() const { return _trim; }

    /**
     * @brief Set attitude trim (level calibration offset)
     * @param trim Vector3f with roll, pitch, yaw trim in radians
     */
    void set_trim(const Vector3f& trim) { _trim = trim; }

    /**
     * @brief Reset AHRS state
     *
     * Called to reset attitude estimation to initial state.
     * @note Stub implementation - resets angles to zero
     */
    void reset() {
        _roll_rad = 0;
        _pitch_rad = 0;
        _yaw_rad = 0;
        _dcm_matrix.identity();
        _healthy = false;
    }

    // Constructor (public for singleton pattern)
    AP_AHRS();

private:

    // Attitude state
    Matrix3f _dcm_matrix;           // Direction Cosine Matrix (body->NED)
    float _roll_rad;                // Roll angle (radians)
    float _pitch_rad;               // Pitch angle (radians)
    float _yaw_rad;                 // Yaw angle (radians, relative)

    // Gyro integration
    Vector3f _gyro_bias;            // Gyro bias estimate (rad/s)

    // Trim (level calibration offset)
    Vector3f _trim;                 // Roll, pitch, yaw trim in radians

    // State flags
    bool _initialized;
    bool _healthy;

    // Complementary filter coefficient
    static constexpr float kAccelWeight = 0.02f;  // Weight for accel correction

    // Internal methods
    void update_dcm_from_euler();
    void estimate_attitude_from_accel(const Vector3f& accel, float& roll, float& pitch);
};

// Global singleton accessor (ArduPilot convention)
namespace AP {
    AP_AHRS& ahrs();
}
