/**
 * @file AP_AHRS_config.h
 * @brief AHRS configuration for RocketChip
 *
 * Enables minimal AHRS implementation for compass calibration support.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// Enable AHRS and DCM attitude estimation
#ifndef AP_AHRS_ENABLED
#define AP_AHRS_ENABLED 1
#endif

#ifndef AP_AHRS_DCM_ENABLED
#define AP_AHRS_DCM_ENABLED 1
#endif

// Disable EKF (requires full ArduPilot infrastructure)
#ifndef HAL_NAVEKF2_AVAILABLE
#define HAL_NAVEKF2_AVAILABLE 0
#endif

#ifndef HAL_NAVEKF3_AVAILABLE
#define HAL_NAVEKF3_AVAILABLE 0
#endif

// Disable external AHRS backends
#ifndef AP_AHRS_SIM_ENABLED
#define AP_AHRS_SIM_ENABLED 0
#endif

#ifndef AP_AHRS_EXTERNAL_ENABLED
#define AP_AHRS_EXTERNAL_ENABLED 0
#endif
