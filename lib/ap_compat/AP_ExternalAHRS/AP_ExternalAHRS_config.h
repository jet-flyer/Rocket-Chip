/**
 * @file AP_ExternalAHRS_config.h
 * @brief External AHRS configuration stub for RocketChip
 *
 * Disables external AHRS support - RocketChip uses internal AHRS.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// Disable external AHRS support
#ifndef HAL_EXTERNAL_AHRS_ENABLED
#define HAL_EXTERNAL_AHRS_ENABLED 0
#endif

#ifndef AP_EXTERNAL_AHRS_ENABLED
#define AP_EXTERNAL_AHRS_ENABLED 0
#endif
