/**
 * @file AP_GPS_config.h
 * @brief GPS configuration stub for RocketChip
 *
 * Disables ArduPilot GPS support - RocketChip uses its own GPS driver.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// Disable ArduPilot GPS - we have our own GPS driver
#ifndef AP_GPS_ENABLED
#define AP_GPS_ENABLED 0
#endif

#ifndef AP_GPS_BACKEND_DEFAULT_ENABLED
#define AP_GPS_BACKEND_DEFAULT_ENABLED 0
#endif
