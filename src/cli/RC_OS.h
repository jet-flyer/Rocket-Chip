/**
 * @file RC_OS.h
 * @brief RocketChip OS - CLI command to MAVLink translation
 *
 * RC_OS provides a terminal-based CLI that internally routes commands
 * through the same MAVLink handlers as external GCS software. This ensures
 * CLI and GCS use identical code paths for all operations.
 *
 * @note Part of RocketChip - Modular Motion Tracking Platform
 */

#pragma once

#include <GCS_MAVLink/GCS.h>

namespace RC_OS {

// ============================================================================
// Calibration Commands - Route to MAV_CMD_PREFLIGHT_CALIBRATION
// ============================================================================

/**
 * @brief Simple level/accel calibration (keep device flat)
 * Equivalent to MAV_CMD_PREFLIGHT_CALIBRATION with param5=1
 */
inline MAV_RESULT cmd_level_cal() {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 0, 0, 1, 0, 0  // param5=1 = simple accel cal
    );
}

/**
 * @brief Full 6-position accelerometer calibration
 * Equivalent to MAV_CMD_PREFLIGHT_CALIBRATION with param5=2
 */
inline MAV_RESULT cmd_accel_cal_6pos() {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 0, 0, 2, 0, 0  // param5=2 = 6-position accel cal
    );
}

/**
 * @brief Compass/magnetometer calibration
 * Equivalent to MAV_CMD_PREFLIGHT_CALIBRATION with param2=1
 */
inline MAV_RESULT cmd_compass_cal() {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 1, 0, 0, 0, 0, 0  // param2=1 = compass cal
    );
}

/**
 * @brief Barometer ground pressure calibration
 * Equivalent to MAV_CMD_PREFLIGHT_CALIBRATION with param3=1
 */
inline MAV_RESULT cmd_baro_cal() {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_PREFLIGHT_CALIBRATION,
        0, 0, 1, 0, 0, 0, 0  // param3=1 = ground pressure cal
    );
}

/**
 * @brief Gyro calibration
 * Equivalent to MAV_CMD_PREFLIGHT_CALIBRATION with param1=1
 */
inline MAV_RESULT cmd_gyro_cal() {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_PREFLIGHT_CALIBRATION,
        1, 0, 0, 0, 0, 0, 0  // param1=1 = gyro cal
    );
}

// ============================================================================
// Position Confirmation (for 6-position accel cal)
// ============================================================================

/**
 * @brief Confirm vehicle is in requested position during 6-pos cal
 * @param position ACCELCAL_VEHICLE_POS enum value
 */
inline MAV_RESULT cmd_confirm_position(int position) {
    return GCS::get_singleton().send_local_command(
        MAV_CMD_ACCELCAL_VEHICLE_POS,
        static_cast<float>(position), 0, 0, 0, 0, 0, 0
    );
}

} // namespace RC_OS
