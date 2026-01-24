/**
 * @file AP_HAL.h
 * @brief AP_HAL include header for RocketChip
 *
 * This header provides the standard <AP_HAL/AP_HAL.h> include path
 * that ArduPilot libraries expect. It provides configuration and
 * base classes needed by ArduPilot code.
 *
 * NOTE: This header does NOT include the full HAL implementation.
 * FreeRTOS-dependent components (Scheduler, Semaphores) must be
 * included separately by code that needs them.
 */

#pragma once

// Configuration and utility macros
#include "../AP_HAL_Compat.h"

// Abstract base classes for HAL interfaces
#include "Storage.h"

// ============================================================================
// Minimal HAL_Semaphore for ArduPilot libraries that need it
// ============================================================================

// Many ArduPilot libraries use HAL_Semaphore but don't actually need
// thread synchronization when compiled standalone. Provide a stub.
#ifndef HAL_SEMAPHORE_DEFINED
#define HAL_SEMAPHORE_DEFINED

class HAL_Semaphore {
public:
    HAL_Semaphore() = default;
    bool take(uint32_t timeout_ms) { (void)timeout_ms; return true; }
    bool take_nonblocking() { return true; }
    void give() {}
};

// Scoped semaphore lock (RAII pattern)
class WITH_SEMAPHORE_TYPE {
public:
    WITH_SEMAPHORE_TYPE(HAL_Semaphore& sem) : m_sem(sem) { m_sem.take(0); }
    ~WITH_SEMAPHORE_TYPE() { m_sem.give(); }
private:
    HAL_Semaphore& m_sem;
};

#define WITH_SEMAPHORE(sem) WITH_SEMAPHORE_TYPE _sem_lock(sem)

#endif // HAL_SEMAPHORE_DEFINED

// ============================================================================
// AP_HAL Namespace - Timing functions stub
// ============================================================================

// ArduPilot libraries expect these timing functions. They're provided by
// the Scheduler, but for libraries compiled without full HAL, we provide
// stubs that use Pico SDK directly.

// Skip stub definitions if full HAL timing functions are available
#ifndef AP_HAL_TIMING_DEFINED
#define AP_HAL_TIMING_DEFINED

#include "pico/time.h"

namespace AP_HAL {

inline uint32_t millis() {
    return to_ms_since_boot(get_absolute_time());
}

inline uint64_t millis64() {
    return to_ms_since_boot(get_absolute_time());
}

inline uint32_t micros() {
    return time_us_32();
}

inline uint64_t micros64() {
    return time_us_64();
}

// Panic handler
[[noreturn]] inline void panic(const char* fmt, ...) {
    (void)fmt;
    while(1) {
        // Halt
    }
}

}  // namespace AP_HAL

#endif // AP_HAL_TIMING_DEFINED
