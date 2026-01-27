/**
 * @file Semaphores.h
 * @brief AP_HAL Semaphore implementation using FreeRTOS
 *
 * Provides mutex and binary semaphore wrappers for ArduPilot libraries.
 * Implements the AP_HAL::Semaphore and AP_HAL::BinarySemaphore interfaces.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

// Prevent the stub HAL_Semaphore from being defined
#define HAL_SEMAPHORE_DEFINED 1

#include "hwdef.h"
#include "FreeRTOS.h"
#include "semphr.h"

namespace RP2350 {

/**
 * @brief Mutex semaphore for mutual exclusion
 *
 * Wraps FreeRTOS recursive mutex. Recursive means the same task can take
 * the mutex multiple times without deadlock.
 *
 * Implements AP_HAL::Semaphore interface.
 */
class Semaphore {
public:
    Semaphore();
    ~Semaphore();

    // Prevent copying
    Semaphore(const Semaphore&) = delete;
    Semaphore& operator=(const Semaphore&) = delete;

    /**
     * @brief Take the semaphore with timeout
     * @param timeout_ms Maximum time to wait in milliseconds
     * @return true if acquired, false on timeout
     */
    bool take(uint32_t timeout_ms);

    /**
     * @brief Try to take the semaphore without blocking
     * @return true if acquired immediately, false if unavailable
     */
    bool take_nonblocking();

    /**
     * @brief Take the semaphore, blocking indefinitely
     */
    void take_blocking();

    /**
     * @brief Release the semaphore
     * @return true on success
     */
    bool give();

    /**
     * @brief Check if semaphore is currently held
     * @return true if semaphore is taken
     *
     * @note For debugging only - state may change immediately after call
     */
    bool is_taken() const;

private:
    void ensure_initialized();  // Lazy init for static objects

    SemaphoreHandle_t m_handle;
    StaticSemaphore_t m_buffer;  // Static allocation for FreeRTOS
};


/**
 * @brief Binary semaphore for thread signaling
 *
 * Used for one-to-one signaling between tasks or from ISR to task.
 * Unlike mutex, this is for notification/synchronization, not protection.
 *
 * Implements AP_HAL::BinarySemaphore interface.
 */
class BinarySemaphore {
public:
    /**
     * @brief Construct binary semaphore
     * @param initial_state true = signaled (first wait won't block)
     */
    explicit BinarySemaphore(bool initial_state = false);
    ~BinarySemaphore();

    // Prevent copying
    BinarySemaphore(const BinarySemaphore&) = delete;
    BinarySemaphore& operator=(const BinarySemaphore&) = delete;

    /**
     * @brief Wait for signal with timeout
     * @param timeout_us Maximum time to wait in microseconds
     * @return true if signaled, false on timeout
     */
    bool wait(uint32_t timeout_us);

    /**
     * @brief Wait indefinitely for signal
     * @return true when signaled
     */
    bool wait_blocking();

    /**
     * @brief Check if signaled without blocking
     * @return true if was signaled (consumes the signal)
     */
    bool wait_nonblocking();

    /**
     * @brief Signal the semaphore (from task context)
     */
    void signal();

    /**
     * @brief Signal the semaphore (from ISR context)
     *
     * Safe to call from interrupt handlers.
     */
    void signal_ISR();

private:
    void ensure_initialized();  // Lazy init for static objects

    SemaphoreHandle_t m_handle;
    StaticSemaphore_t m_buffer;
    bool m_initial_state;
};


/**
 * @brief RAII scoped lock for Semaphore
 *
 * Automatically takes semaphore on construction and releases on destruction.
 *
 * @code
 * {
 *     WithSemaphore lock(my_mutex);
 *     // Protected code here
 * }  // Automatically released
 * @endcode
 */
class WithSemaphore {
public:
    explicit WithSemaphore(Semaphore& sem) : m_sem(sem) {
        m_sem.take_blocking();
    }

    explicit WithSemaphore(Semaphore* sem) : m_sem(*sem) {
        m_sem.take_blocking();
    }

    ~WithSemaphore() {
        m_sem.give();
    }

    // Prevent copying
    WithSemaphore(const WithSemaphore&) = delete;
    WithSemaphore& operator=(const WithSemaphore&) = delete;

private:
    Semaphore& m_sem;
};

}  // namespace RP2350

// ============================================================================
// AP_HAL Compatibility Aliases
// ============================================================================

// Define guard macro BEFORE any includes of AP_HAL/AP_HAL.h
// This prevents the stub HAL_Semaphore from being defined
#define HAL_SEMAPHORE_DEFINED

namespace AP_HAL {
    using Semaphore = RP2350::Semaphore;
    using BinarySemaphore = RP2350::BinarySemaphore;
}

// Global alias for ArduPilot code expecting HAL_Semaphore
using HAL_Semaphore = RP2350::Semaphore;

// Convenience macro for scoped locking
#define WITH_SEMAPHORE(sem) RP2350::WithSemaphore _sem_lock_##__LINE__(sem)
