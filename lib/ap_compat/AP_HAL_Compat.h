/**
 * @file AP_HAL_Compat.h
 * @brief ArduPilot HAL compatibility shim for RocketChip
 *
 * Maps ArduPilot's AP_HAL functions to RocketChip's HAL.
 * This allows using ArduPilot libraries (AP_Math, Filter, Calibrators)
 * without porting the full ChibiOS-based AP_HAL.
 */

#pragma once

#include "Timing.h"
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cstdlib>

// ============================================================================
// Board Detection
// ============================================================================

#define HAL_BOARD_ROCKETCHIP 99
#define HAL_BOARD_NAME "RocketChip-RP2350"
#define CONFIG_HAL_BOARD HAL_BOARD_ROCKETCHIP

// Memory class (affects some algorithm choices)
#define HAL_MEM_CLASS HAL_MEM_CLASS_500

// ============================================================================
// Feature Flags - Disable unused ArduPilot features
// ============================================================================

#define AP_PARAM_ENABLED 0
#define HAL_GCS_ENABLED 0
#define AP_INERTIALSENSOR_ENABLED 0
#define HAL_INS_ACCELCAL_ENABLED 1
#define HAL_LOGGING_ENABLED 0
#define HAL_HAVE_IMU_HEATER 0
#define AP_AHRS_ENABLED 0
#define AP_GPS_ENABLED 0
#define HAL_NAVEKF2_AVAILABLE 0
#define HAL_NAVEKF3_AVAILABLE 0
#define AP_SCRIPTING_ENABLED 0

// ============================================================================
// AP_HAL Namespace - Core timing functions
// ============================================================================

namespace AP_HAL {

/**
 * @brief Get milliseconds since boot
 */
inline uint32_t millis() {
    return rocketchip::hal::Timing::millis32();
}

/**
 * @brief Get milliseconds since boot (64-bit)
 */
inline uint64_t millis64() {
    return rocketchip::hal::Timing::millis();
}

/**
 * @brief Get microseconds since boot
 */
inline uint32_t micros() {
    return rocketchip::hal::Timing::micros32();
}

/**
 * @brief Get microseconds since boot (64-bit)
 */
inline uint64_t micros64() {
    return rocketchip::hal::Timing::micros();
}

/**
 * @brief Panic handler - print message and halt
 */
[[noreturn]] inline void panic(const char* fmt, ...) {
    va_list args;
    va_start(args, fmt);
    printf("AP_HAL PANIC: ");
    vprintf(fmt, args);
    printf("\n");
    va_end(args);

    // Halt
    while(1) {
        // Could add LED blink here
    }
}

} // namespace AP_HAL

// ============================================================================
// HAL_Semaphore - Thread synchronization stub
// ============================================================================

/**
 * @brief Semaphore stub for single-threaded operation
 *
 * For Phase 1, calibration runs single-threaded so semaphores are no-ops.
 * TODO: Replace with FreeRTOS mutex for multi-threaded fusion.
 */
class HAL_Semaphore {
public:
    HAL_Semaphore() = default;

    bool take(uint32_t timeout_ms) {
        (void)timeout_ms;
        return true;
    }

    bool take_nonblocking() {
        return true;
    }

    void give() {}
};

/**
 * @brief Scoped semaphore lock (RAII pattern)
 */
class WITH_SEMAPHORE_TYPE {
public:
    WITH_SEMAPHORE_TYPE(HAL_Semaphore& sem) : m_sem(sem) {
        m_sem.take(0);
    }
    ~WITH_SEMAPHORE_TYPE() {
        m_sem.give();
    }
private:
    HAL_Semaphore& m_sem;
};

// Macro for scoped semaphore - expands to no-op with our stub
#define WITH_SEMAPHORE(sem) WITH_SEMAPHORE_TYPE _sem_lock(sem)

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

// constrain_value is defined by AP_Math - don't duplicate

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

// Gravity constant (m/s²)
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
