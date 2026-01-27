/**
 * @file AP_Logger.h
 * @brief AP_Logger stub for RocketChip
 *
 * Disables ArduPilot logging - RocketChip uses its own logging system.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

// Disable ArduPilot logging
#ifndef AP_LOGGER_ENABLED
#define AP_LOGGER_ENABLED 0
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

// Empty stub for AP_Logger class references
class AP_Logger {
public:
    static AP_Logger* get_singleton() { return nullptr; }
};
