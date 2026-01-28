/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Adapted from AP_HAL_ESP32/Semaphores.h for RP2350
 */

#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>

namespace RP2350 {

/**
 * @brief Mutex semaphore for mutual exclusion
 *
 * Wraps FreeRTOS recursive mutex. Recursive means the same task can take
 * the mutex multiple times without deadlock.
 *
 * Uses opaque storage to avoid requiring FreeRTOS.h in headers.
 * The actual FreeRTOS types are used only in the implementation.
 */
class Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    ~Semaphore() override;

    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
    void take_blocking() override;

    // Check if current task owns this semaphore
    bool check_owner();

protected:
    // Opaque storage for FreeRTOS SemaphoreHandle_t (void*)
    void* _handle;

    // Opaque storage for StaticSemaphore_t
    // Size based on FreeRTOS StaticQueue_t for SMP config (approximately 128 bytes)
    // This must be large enough to hold StaticSemaphore_t
    alignas(4) uint8_t _buffer[128];
};


/**
 * @brief Binary semaphore for thread signaling
 *
 * Used for one-to-one signaling between tasks or from ISR to task.
 * Unlike mutex, this is for notification/synchronization, not protection.
 */
class BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state = false);
    ~BinarySemaphore() override;

    CLASS_NO_COPY(BinarySemaphore);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking() override;
    void signal() override;
    void signal_ISR() override;

protected:
    // Opaque storage for FreeRTOS SemaphoreHandle_t (void*)
    void* _sem;

    // Opaque storage for StaticSemaphore_t
    alignas(4) uint8_t _buffer[128];
};

}  // namespace RP2350
