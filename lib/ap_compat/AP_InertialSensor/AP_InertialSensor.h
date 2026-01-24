/**
 * @file AP_InertialSensor.h
 * @brief Inertial sensor wrapper with calibration support
 *
 * Provides an ArduPilot-compatible interface for IMU calibration
 * using AccelCalibrator. Implements AP_AccelCal_Client to receive
 * calibration completion callbacks and persist results via AP_Param.
 *
 * Usage:
 * @code
 * // Initialize
 * AP_InertialSensor& ins = *AP_InertialSensor::get_singleton();
 * ins.init();
 *
 * // Run calibration
 * ins.accel_calibrate_start(6, 2.0f);
 * while (ins.accel_calibrating()) {
 *     // Read IMU and feed samples
 *     Vector3f accel = read_imu();
 *     float dt = 0.01f;
 *     ins.accel_cal_new_sample(accel * dt, dt);
 * }
 *
 * // Calibration saves automatically on success
 * if (ins.accel_cal_success()) {
 *     printf("Calibration saved!\n");
 * }
 * @endcode
 */

#pragma once

#include <cstdint>
#include <AP_Math/vector3.h>
#include <AP_AccelCal/AccelCalibrator.h>
#include <AP_AccelCal/AP_AccelCal.h>
#include <AP_Param/AP_Param.h>
#include <AP_Param/ParamStorageLayout.h>

// Maximum number of IMU instances
#ifndef INS_MAX_INSTANCES
#define INS_MAX_INSTANCES 2
#endif

/**
 * @brief Inertial sensor manager with calibration
 *
 * Manages accelerometer and gyroscope calibration parameters.
 * Implements ArduPilot's AP_AccelCal_Client interface for
 * automatic calibration persistence.
 */
class AP_InertialSensor : public AP_AccelCal_Client {
public:
    /**
     * @brief Get singleton instance
     * @return Pointer to singleton
     */
    static AP_InertialSensor* get_singleton();

    /**
     * @brief Initialize sensor and load calibration
     *
     * Must be called after AP_Param::setup().
     * Loads stored calibration from flash if valid.
     */
    void init();

    // ========================================================================
    // Calibration Status
    // ========================================================================

    /**
     * @brief Check if accelerometer is calibrated
     * @param instance IMU instance (0 or 1)
     */
    bool accel_calibrated(uint8_t instance = 0) const;

    /**
     * @brief Check if gyroscope is calibrated
     * @param instance IMU instance (0 or 1)
     */
    bool gyro_calibrated(uint8_t instance = 0) const;

    // ========================================================================
    // Calibration Parameters (for applying to raw readings)
    // ========================================================================

    /**
     * @brief Get accelerometer offset
     * @param instance IMU instance
     * @return Offset vector (add to raw reading before scaling)
     *
     * The offset is the correction to apply: corrected = raw + offset
     */
    const Vector3f& get_accel_offset(uint8_t instance = 0) const;

    /**
     * @brief Get accelerometer scale factors
     * @param instance IMU instance
     * @return Scale vector (multiply after offset)
     *
     * Diagonal elements of correction matrix.
     */
    const Vector3f& get_accel_scale(uint8_t instance = 0) const;

    /**
     * @brief Get accelerometer off-diagonal terms
     * @param instance IMU instance
     * @return Off-diagonal elements of correction matrix
     *
     * Cross-axis coupling terms [XY, XZ, YZ].
     */
    const Vector3f& get_accel_offdiag(uint8_t instance = 0) const;

    /**
     * @brief Get gyroscope offset
     * @param instance IMU instance
     * @return Offset vector (subtract from raw reading)
     */
    const Vector3f& get_gyro_offset(uint8_t instance = 0) const;

    // ========================================================================
    // Apply Calibration
    // ========================================================================

    /**
     * @brief Apply calibration correction to accelerometer reading
     * @param raw Raw accelerometer value (in any units)
     * @param instance IMU instance
     * @return Corrected accelerometer value (same units)
     *
     * Applies offset, scale, and off-diagonal corrections:
     * corrected = M * (raw + offset)
     * where M is constructed from scale and offdiag.
     */
    Vector3f correct_accel(const Vector3f& raw, uint8_t instance = 0) const;

    /**
     * @brief Apply calibration correction to gyroscope reading
     * @param raw Raw gyroscope value
     * @param instance IMU instance
     * @return Corrected gyroscope value
     *
     * Simple offset subtraction: corrected = raw - offset
     */
    Vector3f correct_gyro(const Vector3f& raw, uint8_t instance = 0) const;

    // ========================================================================
    // Accelerometer Calibration Control
    // ========================================================================

    /**
     * @brief Get calibrator for direct access
     * @param instance IMU instance
     * @return Pointer to AccelCalibrator
     */
    AccelCalibrator* get_calibrator(uint8_t instance = 0);

    /**
     * @brief Start accelerometer calibration
     * @param num_samples Number of orientations (typically 6)
     * @param sample_time Time per orientation in seconds
     * @param fit_type Calibration fit type
     */
    void accel_calibrate_start(
        uint8_t num_samples = 6,
        float sample_time = 2.0f,
        accel_cal_fit_type_t fit_type = ACCEL_CAL_AXIS_ALIGNED_ELLIPSOID);

    /**
     * @brief Signal start of sample collection for current orientation
     */
    void accel_cal_collect_sample();

    /**
     * @brief Feed sample to calibrator
     * @param delta_velocity Acceleration * dt (m/s)
     * @param dt Time delta in seconds
     */
    void accel_cal_new_sample(const Vector3f& delta_velocity, float dt);

    /**
     * @brief Check for calibration timeout
     * Call periodically during calibration.
     */
    void accel_cal_check_timeout();

    /**
     * @brief Get current calibration status
     */
    accel_cal_status_t accel_cal_get_status(uint8_t instance = 0) const;

    /**
     * @brief Check if calibration is in progress
     */
    bool accel_calibrating(uint8_t instance = 0);

    /**
     * @brief Check if calibration succeeded
     */
    bool accel_cal_success(uint8_t instance = 0) const;

    /**
     * @brief Get calibration fitness (quality metric)
     * @return Mean squared residual error
     */
    float accel_cal_get_fitness(uint8_t instance = 0) const;

    /**
     * @brief Get number of samples collected
     */
    uint8_t accel_cal_get_samples_collected(uint8_t instance = 0) const;

    /**
     * @brief Cancel calibration
     */
    void accel_cal_cancel();

    // ========================================================================
    // Gyroscope Calibration
    // ========================================================================

    /**
     * @brief Run simple gyro bias calibration
     * @param samples Number of samples to average
     * @param sample_period_ms Time between samples in ms
     * @param get_gyro_func Callback to read gyroscope
     * @return true on success
     *
     * Simple averaging calibration. Device must be stationary.
     */
    bool gyro_calibrate_simple(
        uint16_t samples,
        uint16_t sample_period_ms,
        Vector3f (*get_gyro_func)());

    /**
     * @brief Set gyro offset manually
     * @param instance IMU instance
     * @param offset Offset to save
     */
    void set_gyro_offset(uint8_t instance, const Vector3f& offset);

    // ========================================================================
    // Save/Load
    // ========================================================================

    /**
     * @brief Force save all calibration parameters
     */
    void save_calibration();

    /**
     * @brief Load calibration from flash
     */
    void load_calibration();

    /**
     * @brief Reset calibration to defaults
     */
    void reset_calibration();

    // ========================================================================
    // AP_AccelCal_Client Interface
    // ========================================================================

    bool _acal_get_saving() override { return _saving; }
    bool _acal_get_ready_to_sample() override { return true; }
    bool _acal_get_fail() override { return false; }
    AccelCalibrator* _acal_get_calibrator(uint8_t instance) override;
    void _acal_save_calibrations() override;
    void _acal_event_success() override;
    void _acal_event_failure() override;

private:
    AP_InertialSensor();

    // Singleton instance
    static AP_InertialSensor* _singleton;

    // Calibration state
    bool _initialized;
    bool _saving;

    // Per-instance calibration parameters
    struct InstanceCalibration {
        AP_Vector3f accel_offset;
        AP_Vector3f accel_scale;
        AP_Vector3f accel_offdiag;
        AP_Vector3f gyro_offset;
        bool accel_calibrated;
        bool gyro_calibrated;
    };

    InstanceCalibration _instance[INS_MAX_INSTANCES];

    // Calibrators
    AccelCalibrator _calibrators[INS_MAX_INSTANCES];

    // Initialize parameter storage offsets
    void init_param_offsets();
};

// ============================================================================
// Global Accessor
// ============================================================================

/**
 * @brief Get AP_InertialSensor singleton
 * @return Reference to singleton instance
 */
inline AP_InertialSensor& AP_Ins() {
    return *AP_InertialSensor::get_singleton();
}
