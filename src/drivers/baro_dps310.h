/**
 * @file baro_dps310.h
 * @brief DPS310 Barometer wrapper using ruuvi.dps310.c library
 *
 * Provides Pico SDK integration for the proven ruuvi DPS310 driver.
 */

#ifndef ROCKETCHIP_BARO_DPS310_H
#define ROCKETCHIP_BARO_DPS310_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

#define BARO_DPS310_ADDR_DEFAULT    0x77
#define BARO_DPS310_ADDR_ALT        0x76

// ============================================================================
// Types
// ============================================================================

/**
 * @brief Barometer data
 */
typedef struct {
    float pressure_pa;      ///< Pressure in Pascals
    float temperature_c;    ///< Temperature in Celsius
    float altitude_m;       ///< Altitude in meters (calculated from sea level pressure)
    bool valid;             ///< Data validity flag
} baro_dps310_data_t;

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize the DPS310 barometer
 * @param addr I2C address (BARO_DPS310_ADDR_DEFAULT or BARO_DPS310_ADDR_ALT)
 * @return true on success
 */
bool baro_dps310_init(uint8_t addr);

/**
 * @brief Check if barometer is initialized and ready
 * @return true if ready
 */
bool baro_dps310_ready(void);

/**
 * @brief Start continuous measurement mode
 * @return true on success
 */
bool baro_dps310_start_continuous(void);

/**
 * @brief Stop continuous measurement (enter standby)
 * @return true on success
 */
bool baro_dps310_stop(void);

/**
 * @brief Read latest pressure and temperature data
 * @param data Output data structure
 * @return true on success
 */
bool baro_dps310_read(baro_dps310_data_t* data);

/**
 * @brief Set sea level pressure for altitude calculation
 * @param pressure_pa Sea level pressure in Pascals (default 101325)
 */
void baro_dps310_set_sea_level(float pressure_pa);

/**
 * @brief Calculate altitude from pressure
 * @param pressure_pa Current pressure in Pascals
 * @param sea_level_pa Sea level pressure in Pascals
 * @return Altitude in meters
 */
float baro_dps310_pressure_to_altitude(float pressure_pa, float sea_level_pa);

#endif // ROCKETCHIP_BARO_DPS310_H
