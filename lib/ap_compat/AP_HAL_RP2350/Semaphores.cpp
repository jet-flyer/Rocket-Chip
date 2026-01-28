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
 * Adapted from AP_HAL_ESP32/Semaphores.cpp for RP2350
 */

#include "Semaphores.h"

#include <cstdio>
#include <FreeRTOS.h>
#include <semphr.h>
#include <task.h>

// Verify buffer size is sufficient for FreeRTOS static semaphore
static_assert(sizeof(StaticSemaphore_t) <= 128,
    "Semaphore buffer too small for StaticSemaphore_t");

using namespace RP2350;

// ============================================================================
// Semaphore (Mutex) Implementation
// ============================================================================

Semaphore::Semaphore()
    : _handle(nullptr)
{
    StaticSemaphore_t* buf = reinterpret_cast<StaticSemaphore_t*>(_buffer);
    _handle = xSemaphoreCreateRecursiveMutexStatic(buf);
}

Semaphore::~Semaphore()
{
    if (_handle != nullptr) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(_handle));
        _handle = nullptr;
    }
}

bool Semaphore::give()
{
    return xSemaphoreGiveRecursive(static_cast<SemaphoreHandle_t>(_handle));
}

bool Semaphore::take(uint32_t timeout_ms)
{
    // HAL_SEMAPHORE_BLOCK_FOREVER = 0, meaning 0 = block forever
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        take_blocking();
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    // Use FreeRTOS tick-based timeout
    TickType_t ticks = pdMS_TO_TICKS(timeout_ms);
    if (ticks == 0 && timeout_ms > 0) {
        ticks = 1;  // Minimum 1 tick for non-zero timeout
    }
    return xSemaphoreTakeRecursive(static_cast<SemaphoreHandle_t>(_handle), ticks) == pdTRUE;
}

void Semaphore::take_blocking()
{
    xSemaphoreTakeRecursive(static_cast<SemaphoreHandle_t>(_handle), portMAX_DELAY);
}

bool Semaphore::take_nonblocking()
{
    return xSemaphoreTakeRecursive(static_cast<SemaphoreHandle_t>(_handle), 0) == pdTRUE;
}

bool Semaphore::check_owner()
{
    return xSemaphoreGetMutexHolder(static_cast<SemaphoreHandle_t>(_handle)) == xTaskGetCurrentTaskHandle();
}


// ============================================================================
// BinarySemaphore Implementation
// ============================================================================

BinarySemaphore::BinarySemaphore(bool initial_state)
    : _sem(nullptr)
{
    StaticSemaphore_t* buf = reinterpret_cast<StaticSemaphore_t*>(_buffer);
    _sem = xSemaphoreCreateBinaryStatic(buf);
    if (initial_state) {
        xSemaphoreGive(static_cast<SemaphoreHandle_t>(_sem));
    }
}

BinarySemaphore::~BinarySemaphore()
{
    if (_sem != nullptr) {
        vSemaphoreDelete(static_cast<SemaphoreHandle_t>(_sem));
        _sem = nullptr;
    }
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    TickType_t ticks = pdMS_TO_TICKS(timeout_us / 1000U);
    if (ticks == 0 && timeout_us > 0) {
        ticks = 1;  // Minimum 1 tick for non-zero timeout
    }
    return xSemaphoreTake(static_cast<SemaphoreHandle_t>(_sem), ticks) == pdTRUE;
}

bool BinarySemaphore::wait_blocking()
{
    return xSemaphoreTake(static_cast<SemaphoreHandle_t>(_sem), portMAX_DELAY) == pdTRUE;
}

void BinarySemaphore::signal()
{
    xSemaphoreGive(static_cast<SemaphoreHandle_t>(_sem));
}

void BinarySemaphore::signal_ISR()
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(static_cast<SemaphoreHandle_t>(_sem), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
