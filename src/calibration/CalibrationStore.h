/**
 * @file CalibrationStore.h
 * @brief Persistent calibration data storage
 *
 * Wrapper around AP_HAL::Storage for sensor calibration data.
 * Provides simple load/save API with CRC validation.
 *
 * See docs/AP_HAL_RP2350_PLAN.md for storage architecture.
 */

#pragma once

#include <rocketchip/storage_layout.h>

namespace rocketchip {

/**
 * @brief Calibration data persistence
 *
 * Stores sensor calibration in flash via AP_FlashStorage.
 * Data survives power cycles and can be updated after recalibration.
 *
 * Usage:
 * @code
 * CalibrationStore store;
 * store.init();
 *
 * SensorCalibration cal;
 * if (store.load(cal)) {
 *     // Apply calibration to sensors
 * } else {
 *     // Run calibration routine
 *     store.save(cal);
 * }
 * @endcode
 */
class CalibrationStore {
public:
    CalibrationStore() = default;
    ~CalibrationStore() = default;

    // Prevent copying
    CalibrationStore(const CalibrationStore&) = delete;
    CalibrationStore& operator=(const CalibrationStore&) = delete;

    /**
     * @brief Initialize calibration store
     *
     * Must be called after hal.init(). Prepares storage for access.
     */
    void init();

    /**
     * @brief Load calibration from flash
     * @param cal Output: calibration data
     * @return true if valid calibration was loaded
     *
     * Returns false if:
     * - No calibration has been saved
     * - Magic number doesn't match
     * - CRC check fails
     * - Version mismatch (future consideration)
     */
    bool load(SensorCalibration& cal);

    /**
     * @brief Save calibration to flash
     * @param cal Calibration data to save
     * @return true on success
     *
     * Computes CRC and writes to flash. Data persists across
     * power cycles.
     */
    bool save(const SensorCalibration& cal);

    /**
     * @brief Erase stored calibration
     * @return true on success
     *
     * Clears the calibration region. Use with caution.
     */
    bool erase();

    /**
     * @brief Check if valid calibration exists
     * @return true if load() would succeed
     */
    bool has_calibration();

    /**
     * @brief Get default calibration (identity)
     * @param cal Output: default calibration values
     *
     * Sets all offsets to 0, scales to 1.0, etc.
     * Useful for initializing before first calibration.
     */
    static void get_defaults(SensorCalibration& cal);

private:
    bool m_initialized = false;

    /**
     * @brief Compute CRC32 for calibration data
     * @param cal Calibration structure (excluding crc32 field)
     * @return CRC32 value
     */
    static uint32_t compute_crc(const SensorCalibration& cal);
};

}  // namespace rocketchip
