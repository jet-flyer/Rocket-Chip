/**
 * @file AP_Baro_config.h
 * @brief RocketChip barometer configuration override
 *
 * Replaces ArduPilot's AP_Baro_config.h to avoid unnecessary dependencies
 * (AP_Airspeed, AP_ExternalAHRS, AP_MSP) that aren't in our sparse checkout.
 *
 * Only DPS280/DPS310 backend is enabled for RocketChip.
 */

#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// ============================================================================
// Feature Disables - Skip unused dependencies
// ============================================================================

// Disable wind compensation (requires AP_Airspeed)
#ifndef HAL_BARO_WIND_COMP_ENABLED
#define HAL_BARO_WIND_COMP_ENABLED 0
#endif

// Disable MSP integration
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED 0
#endif

// Disable ExternalAHRS
#ifndef AP_EXTERNAL_AHRS_ENABLED
#define AP_EXTERNAL_AHRS_ENABLED 0
#endif

// Disable simulation
#ifndef AP_SIM_ENABLED
#define AP_SIM_ENABLED 0
#endif

// Disable DroneCAN
#ifndef HAL_ENABLE_DRONECAN_DRIVERS
#define HAL_ENABLE_DRONECAN_DRIVERS 0
#endif

// Disable AUAV airspeed
#ifndef AP_AIRSPEED_AUAV_ENABLED
#define AP_AIRSPEED_AUAV_ENABLED 0
#endif

// ============================================================================
// Barometer Core Configuration
// ============================================================================

#ifndef AP_BARO_ENABLED
#define AP_BARO_ENABLED 1
#endif

// Disable default backends - we'll enable only what we need
#ifndef AP_BARO_BACKEND_DEFAULT_ENABLED
#define AP_BARO_BACKEND_DEFAULT_ENABLED 0
#endif

// ============================================================================
// Backend Enable/Disable
// ============================================================================

// Enable DPS280/DPS310 (our sensor)
#ifndef AP_BARO_DPS280_ENABLED
#define AP_BARO_DPS280_ENABLED 1
#endif

// Disable all other backends
#ifndef AP_BARO_BMP085_ENABLED
#define AP_BARO_BMP085_ENABLED 0
#endif

#ifndef AP_BARO_BMP280_ENABLED
#define AP_BARO_BMP280_ENABLED 0
#endif

#ifndef AP_BARO_BMP388_ENABLED
#define AP_BARO_BMP388_ENABLED 0
#endif

#ifndef AP_BARO_BMP581_ENABLED
#define AP_BARO_BMP581_ENABLED 0
#endif

#ifndef AP_BARO_DUMMY_ENABLED
#define AP_BARO_DUMMY_ENABLED 0
#endif

#ifndef AP_BARO_EXTERNALAHRS_ENABLED
#define AP_BARO_EXTERNALAHRS_ENABLED 0
#endif

#ifndef AP_BARO_FBM320_ENABLED
#define AP_BARO_FBM320_ENABLED 0
#endif

#ifndef AP_BARO_ICM20789_ENABLED
#define AP_BARO_ICM20789_ENABLED 0
#endif

#ifndef AP_BARO_ICP101XX_ENABLED
#define AP_BARO_ICP101XX_ENABLED 0
#endif

#ifndef AP_BARO_ICP201XX_ENABLED
#define AP_BARO_ICP201XX_ENABLED 0
#endif

#ifndef AP_BARO_KELLERLD_ENABLED
#define AP_BARO_KELLERLD_ENABLED 0
#endif

#ifndef AP_BARO_LPS2XH_ENABLED
#define AP_BARO_LPS2XH_ENABLED 0
#endif

#ifndef AP_BARO_MS5611_ENABLED
#define AP_BARO_MS5611_ENABLED 0
#endif

#ifndef AP_BARO_MS5607_ENABLED
#define AP_BARO_MS5607_ENABLED 0
#endif

#ifndef AP_BARO_MS5637_ENABLED
#define AP_BARO_MS5637_ENABLED 0
#endif

#ifndef AP_BARO_MS5837_ENABLED
#define AP_BARO_MS5837_ENABLED 0
#endif

#define AP_BARO_MS56XX_ENABLED 0

#ifndef AP_BARO_MSP_ENABLED
#define AP_BARO_MSP_ENABLED 0
#endif

#ifndef AP_SIM_BARO_ENABLED
#define AP_SIM_BARO_ENABLED 0
#endif

#ifndef AP_BARO_SPL06_ENABLED
#define AP_BARO_SPL06_ENABLED 0
#endif

#ifndef AP_BARO_DRONECAN_ENABLED
#define AP_BARO_DRONECAN_ENABLED 0
#endif

#ifndef AP_BARO_AUAV_ENABLED
#define AP_BARO_AUAV_ENABLED 0
#endif

// ============================================================================
// Optional Features
// ============================================================================

#ifndef AP_BARO_PROBE_EXTERNAL_I2C_BUSES
#define AP_BARO_PROBE_EXTERNAL_I2C_BUSES 0
#endif

#ifndef AP_BARO_PROBE_EXT_PARAMETER_ENABLED
#define AP_BARO_PROBE_EXT_PARAMETER_ENABLED 0
#endif

#ifndef AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED
#define AP_BARO_1976_STANDARD_ATMOSPHERE_ENABLED 0
#endif

#ifndef AP_BARO_THST_COMP_ENABLED
#define AP_BARO_THST_COMP_ENABLED 0
#endif
