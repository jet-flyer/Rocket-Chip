/**
 * @file calibration_storage.h
 * @brief Calibration persistent storage interface
 *
 * Provides a simple interface for storing calibration data in flash.
 * Uses a dual-sector approach for wear leveling and power-safe writes.
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
 * Sets up the flash sectors for calibration storage.
 * Call once at boot.
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
 * Uses dual-sector approach: writes to alternate sector,
 * then marks new sector as valid.
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
