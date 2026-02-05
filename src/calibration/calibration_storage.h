/**
 * @file calibration_storage.h
 * @brief Calibration persistent storage interface
 *
 * Stores calibration data in flash using a dual-sector approach
 * for wear leveling and power-safe writes.
 */

#ifndef ROCKETCHIP_CALIBRATION_STORAGE_H
#define ROCKETCHIP_CALIBRATION_STORAGE_H

#include "calibration_data.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize calibration storage
 *
 * Sets up flash sectors for calibration storage.
 * Call once at boot, before stdio_init_all() per LL Entry 4/12.
 *
 * @return true on success
 */
bool calibration_storage_init(void);

/**
 * @brief Read calibration data from storage
 *
 * @param cal Output calibration structure
 * @return true if valid calibration was read
 */
bool calibration_storage_read(calibration_store_t* cal);

/**
 * @brief Write calibration data to storage
 *
 * Uses flash_safe_execute for dual-core safety.
 * Writes to alternate sector for wear leveling.
 *
 * @param cal Calibration data to write
 * @return true on success
 */
bool calibration_storage_write(const calibration_store_t* cal);

/**
 * @brief Erase calibration storage (factory reset)
 * @return true on success
 */
bool calibration_storage_erase(void);

#ifdef __cplusplus
}
#endif

#endif // ROCKETCHIP_CALIBRATION_STORAGE_H
