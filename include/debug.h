/**
 * @file debug.h
 * @brief Compile-time guarded debug output macros
 *
 * Provides DBG_PRINT and DBG_ERROR macros that are enabled only when
 * CONFIG_DEBUG is defined. In release builds, these macros compile to
 * no-ops for clean production firmware.
 *
 * Usage:
 *   #define CONFIG_DEBUG  // Enable in debug builds
 *   #include "debug.h"
 *
 *   DBG_PRINT("[Module] Initialization complete\n");
 *   DBG_ERROR("[Module] Sensor read failed\n");
 *
 * @note Fatal errors (malloc failure, stack overflow) should use direct
 *       LED blink codes rather than these macros, as console output may
 *       be unreliable in fatal error conditions.
 *
 * @see standards/DEBUG_OUTPUT.md for output format conventions
 */

#ifndef ROCKETCHIP_DEBUG_H
#define ROCKETCHIP_DEBUG_H

#include <cstdio>

#ifdef CONFIG_DEBUG

/**
 * @brief Debug print macro - enabled in debug builds
 * @param fmt Printf-style format string
 * @param ... Format arguments
 */
#define DBG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)

/**
 * @brief Debug error macro - enabled in debug builds
 * @param fmt Printf-style format string
 * @param ... Format arguments
 */
#define DBG_ERROR(fmt, ...) printf(fmt, ##__VA_ARGS__)

#else

/**
 * @brief Debug print macro - disabled in release builds
 */
#define DBG_PRINT(fmt, ...) do {} while(0)

/**
 * @brief Debug error macro - disabled in release builds
 */
#define DBG_ERROR(fmt, ...) do {} while(0)

#endif // CONFIG_DEBUG

#endif // ROCKETCHIP_DEBUG_H
