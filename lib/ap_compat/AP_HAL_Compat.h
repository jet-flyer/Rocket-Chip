/**
 * @file AP_HAL_Compat.h
 * @brief ArduPilot compatibility configuration for RocketChip
 *
 * Provides feature flags, utility macros, and configuration for
 * ArduPilot libraries. HAL implementations are in AP_HAL_RP2350/.
 *
 * @note Do NOT define HAL functions here - those are in AP_HAL_RP2350.
 */

#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

// ============================================================================
// FUNCTOR_TYPEDEF - Function pointer type creation
// ============================================================================

// ArduPilot uses FUNCTOR_TYPEDEF for callback function pointers.
// This simplified version creates plain function pointer types.
// Used by AP_FlashStorage for flash operation callbacks.

#ifndef FUNCTOR_TYPEDEF
#define FUNCTOR_TYPEDEF(name, rettype, ...) \
    typedef rettype (*name)(__VA_ARGS__)
#endif

#ifndef FUNCTOR_DECLARE
#define FUNCTOR_DECLARE(rettype, name, ...) \
    rettype (*name)(__VA_ARGS__)
#endif

#ifndef FUNCTOR_BIND
#define FUNCTOR_BIND(obj, func, rettype, ...) (func)
#endif

// ============================================================================
// Board Detection
// ============================================================================

#ifndef HAL_BOARD_ROCKETCHIP
#define HAL_BOARD_ROCKETCHIP 99
#endif

#ifndef HAL_BOARD_NAME
#define HAL_BOARD_NAME "RocketChip-RP2350"
#endif

#ifndef CONFIG_HAL_BOARD
#define CONFIG_HAL_BOARD HAL_BOARD_ROCKETCHIP
#endif

// Memory class (affects some algorithm choices)
#ifndef HAL_MEM_CLASS
#define HAL_MEM_CLASS HAL_MEM_CLASS_500
#endif

// ============================================================================
// Storage Configuration
// ============================================================================

// Size in bytes of the persistent storage area
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE 4096
#endif

// Flash storage type for RP2350 (similar to STM32F4 - can clear bits)
#ifndef AP_FLASHSTORAGE_TYPE
#define AP_FLASHSTORAGE_TYPE 2  // AP_FLASHSTORAGE_TYPE_F4
#endif

// ============================================================================
// Feature Flags - Disable unused ArduPilot features
// ============================================================================

#ifndef AP_PARAM_ENABLED
#define AP_PARAM_ENABLED 0
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

#ifndef HAL_INS_ACCELCAL_ENABLED
#define HAL_INS_ACCELCAL_ENABLED 1
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif

#ifndef AP_AHRS_ENABLED
#define AP_AHRS_ENABLED 0
#endif

#ifndef AP_GPS_ENABLED
#define AP_GPS_ENABLED 0
#endif

#ifndef HAL_NAVEKF2_AVAILABLE
#define HAL_NAVEKF2_AVAILABLE 0
#endif

#ifndef HAL_NAVEKF3_AVAILABLE
#define HAL_NAVEKF3_AVAILABLE 0
#endif

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

// ============================================================================
// AP Namespace - Internal error reporting
// ============================================================================

namespace AP {

/**
 * @brief Internal error reporting stub
 */
inline void internal_error(uint32_t error_code) {
    printf("[AP_InternalError] Code: %lu\n", (unsigned long)error_code);
}

} // namespace AP

// ============================================================================
// Utility macros from AP_Common
// ============================================================================

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) (sizeof(arr) / sizeof((arr)[0]))
#endif

#ifndef PACKED
#define PACKED __attribute__((packed))
#endif

#ifndef UNUSED
#define UNUSED __attribute__((unused))
#endif

#ifndef NOINLINE
#define NOINLINE __attribute__((noinline))
#endif

#ifndef NORETURN
#define NORETURN [[noreturn]]
#endif

#ifndef WARN_IF_UNUSED
#define WARN_IF_UNUSED __attribute__((warn_unused_result))
#endif

// __AP_LINE__ is used for debug output in constrain macros
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif

// INTERNAL_ERROR macro stub (normally reports to AP_InternalError)
#ifndef INTERNAL_ERROR
#define INTERNAL_ERROR(error) do { printf("INTERNAL_ERROR: %d\n", (int)(error)); } while(0)
#endif

// MIN/MAX macros
#ifndef MIN
#define MIN(a,b) ((a)<(b)?(a):(b))
#endif

#ifndef MAX
#define MAX(a,b) ((a)>(b)?(a):(b))
#endif

// ============================================================================
// Float math helpers
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#ifndef M_PI_2
#define M_PI_2 1.57079632679489661923f
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / M_PI)
#endif

// Gravity constant (m/s^2)
#ifndef GRAVITY_MSS
#define GRAVITY_MSS 9.80665f
#endif

// ============================================================================
// Type definitions
// ============================================================================

// Floating point type (ArduPilot uses this for precision selection)
#ifndef FTYPE_DEFINED
#define FTYPE_DEFINED
typedef float ftype;
#endif
