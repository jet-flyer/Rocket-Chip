/**
 * @file AP_Compass_config.h
 * @brief Compass configuration for RocketChip
 *
 * Enables compass calibration via ArduPilot's CompassCalibrator.
 * Overrides default config to work with our minimal AHRS implementation.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// ============================================================================
// Core Compass Features
// ============================================================================

#ifndef AP_COMPASS_ENABLED
#define AP_COMPASS_ENABLED 1
#endif

// Enable diagonal/off-diagonal soft iron calibration
#ifndef AP_COMPASS_DIAGONALS_ENABLED
#define AP_COMPASS_DIAGONALS_ENABLED 1
#endif

// ============================================================================
// Compass Calibration
// ============================================================================

// Enable compass calibration (CompassCalibrator)
// Note: ArduPilot default requires AP_AHRS_DCM_ENABLED, but we provide
// our own minimal AHRS implementation that satisfies the dependency
#ifndef COMPASS_CAL_ENABLED
#define COMPASS_CAL_ENABLED 1
#endif

// Fixed yaw calibration requires GPS - disabled until GPS integrated
#ifndef AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED
#define AP_COMPASS_CALIBRATION_FIXED_YAW_ENABLED 0
#endif

// Scale factor limits for soft-iron correction
#define COMPASS_MAX_SCALE_FACTOR 1.5f
#define COMPASS_MIN_SCALE_FACTOR (1.0f / COMPASS_MAX_SCALE_FACTOR)

// ============================================================================
// Compass Backends - All Disabled
// ============================================================================
// RocketChip uses its own LIS3MDL driver, not ArduPilot backends

#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

#ifndef AP_COMPASS_EXTERNALAHRS_ENABLED
#define AP_COMPASS_EXTERNALAHRS_ENABLED 0
#endif

#ifndef AP_COMPASS_MSP_ENABLED
#define AP_COMPASS_MSP_ENABLED 0
#endif

#ifndef AP_COMPASS_SITL_ENABLED
#define AP_COMPASS_SITL_ENABLED 0
#endif

#ifndef AP_COMPASS_DRONECAN_ENABLED
#define AP_COMPASS_DRONECAN_ENABLED 0
#endif

#ifndef AP_COMPASS_DRONECAN_HIRES_ENABLED
#define AP_COMPASS_DRONECAN_HIRES_ENABLED 0
#endif

// I2C backends - disabled
#ifndef AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_I2C_BACKEND_DEFAULT_ENABLED 0
#endif

// AK09916 is the magnetometer inside the ICM-20948 chip
#ifndef AP_COMPASS_AK09916_ENABLED
#define AP_COMPASS_AK09916_ENABLED 1
#endif

#ifndef AP_COMPASS_AK8963_ENABLED
#define AP_COMPASS_AK8963_ENABLED 0
#endif

#ifndef AP_COMPASS_BMM150_ENABLED
#define AP_COMPASS_BMM150_ENABLED 0
#endif

#ifndef AP_COMPASS_BMM350_ENABLED
#define AP_COMPASS_BMM350_ENABLED 0
#endif

#ifndef AP_COMPASS_HMC5843_ENABLED
#define AP_COMPASS_HMC5843_ENABLED 0
#endif

// ICM20948 probe enables accessing AK09916 via the ICM-20948's auxiliary I2C bus
#ifndef AP_COMPASS_ICM20948_ENABLED
#define AP_COMPASS_ICM20948_ENABLED 1
#endif

#ifndef AP_COMPASS_IIS2MDC_ENABLED
#define AP_COMPASS_IIS2MDC_ENABLED 0
#endif

#ifndef AP_COMPASS_IST8308_ENABLED
#define AP_COMPASS_IST8308_ENABLED 0
#endif

#ifndef AP_COMPASS_IST8310_ENABLED
#define AP_COMPASS_IST8310_ENABLED 0
#endif

#ifndef AP_COMPASS_LIS3MDL_ENABLED
#define AP_COMPASS_LIS3MDL_ENABLED 0  // We use our own driver
#endif

#ifndef AP_COMPASS_LIS2MDL_ENABLED
#define AP_COMPASS_LIS2MDL_ENABLED 0
#endif

#ifndef AP_COMPASS_LSM303D_ENABLED
#define AP_COMPASS_LSM303D_ENABLED 0
#endif

#ifndef AP_COMPASS_LSM9DS1_ENABLED
#define AP_COMPASS_LSM9DS1_ENABLED 0
#endif

#ifndef AP_COMPASS_MAG3110_ENABLED
#define AP_COMPASS_MAG3110_ENABLED 0
#endif

#ifndef AP_COMPASS_MMC3416_ENABLED
#define AP_COMPASS_MMC3416_ENABLED 0
#endif

#ifndef AP_COMPASS_MMC5XX3_ENABLED
#define AP_COMPASS_MMC5XX3_ENABLED 0
#endif

#ifndef AP_COMPASS_QMC5883P_ENABLED
#define AP_COMPASS_QMC5883P_ENABLED 0
#endif

#ifndef AP_COMPASS_QMC5883L_ENABLED
#define AP_COMPASS_QMC5883L_ENABLED 0
#endif

#ifndef AP_COMPASS_RM3100_ENABLED
#define AP_COMPASS_RM3100_ENABLED 0
#endif

#ifndef AP_COMPASS_PROBING_ENABLED
#define AP_COMPASS_PROBING_ENABLED 0
#endif

#ifndef AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED
#define AP_COMPASS_INTERNAL_BUS_PROBING_ENABLED 0
#endif
