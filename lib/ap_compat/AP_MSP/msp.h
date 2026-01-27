/**
 * @file msp.h
 * @brief MSP protocol stub for RocketChip
 *
 * Disables MSP sensor support - RocketChip doesn't use MSP.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// Disable MSP sensors
#ifndef HAL_MSP_SENSORS_ENABLED
#define HAL_MSP_SENSORS_ENABLED 0
#endif
