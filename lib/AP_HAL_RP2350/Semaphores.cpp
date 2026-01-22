/**
 * @file Semaphores.cpp
 * @brief AP_HAL Semaphore implementation using FreeRTOS
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "Semaphores.h"
#include "FreeRTOS.h"
#include "task.h"

namespace RP2350 {

// ============================================================================
// Semaphore (Mutex) Implementation
// ============================================================================

Semaphore::Semaphore() {
    // Create recursive mutex with static allocation
    m_handle = xSemaphoreCreateRecursiveMutexStatic(&m_buffer);
    configASSERT(m_handle != nullptr);
}

Semaphore::~Semaphore() {
    // Static semaphores don't need deletion, but we can still call this
    // for consistency if we later switch to dynamic allocation
    if (m_handle != nullptr) {
        vSemaphoreDelete(m_handle);
        m_handle = nullptr;
    }
}

bool Semaphore::take(uint32_t timeout_ms) {
    if (m_handle == nullptr) {
        return false;
    }

    // Convert milliseconds to ticks
    TickType_t ticks;
    if (timeout_ms == 0) {
        ticks = 0;
    } else if (timeout_ms == UINT32_MAX) {
        ticks = portMAX_DELAY;
    } else {
        ticks = pdMS_TO_TICKS(timeout_ms);
        // Ensure at least 1 tick for non-zero timeout
        if (ticks == 0 && timeout_ms > 0) {
            ticks = 1;
        }
    }

    return xSemaphoreTakeRecursive(m_handle, ticks) == pdTRUE;
}

bool Semaphore::take_nonblocking() {
    if (m_handle == nullptr) {
        return false;
    }
    return xSemaphoreTakeRecursive(m_handle, 0) == pdTRUE;
}

void Semaphore::take_blocking() {
    if (m_handle == nullptr) {
        return;
    }
    xSemaphoreTakeRecursive(m_handle, portMAX_DELAY);
}

bool Semaphore::give() {
    if (m_handle == nullptr) {
        return false;
    }
    return xSemaphoreGiveRecursive(m_handle) == pdTRUE;
}

bool Semaphore::is_taken() const {
    if (m_handle == nullptr) {
        return false;
    }
    // A recursive mutex count > 0 means it's taken
    // We try to take with 0 timeout to check
    if (xSemaphoreTakeRecursive(m_handle, 0) == pdTRUE) {
        // We got it, so it wasn't fully taken (or we already had it)
        xSemaphoreGiveRecursive(m_handle);
        return false;
    }
    return true;
}

// ============================================================================
// BinarySemaphore Implementation
// ============================================================================

BinarySemaphore::BinarySemaphore(bool initial_state) {
    // Create binary semaphore with static allocation
    m_handle = xSemaphoreCreateBinaryStatic(&m_buffer);
    configASSERT(m_handle != nullptr);

    // If initial_state is true, signal it so first wait won't block
    if (initial_state && m_handle != nullptr) {
        xSemaphoreGive(m_handle);
    }
}

BinarySemaphore::~BinarySemaphore() {
    if (m_handle != nullptr) {
        vSemaphoreDelete(m_handle);
        m_handle = nullptr;
    }
}

bool BinarySemaphore::wait(uint32_t timeout_us) {
    if (m_handle == nullptr) {
        return false;
    }

    // Convert microseconds to ticks
    // FreeRTOS tick is typically 1ms, so microsecond precision is limited
    TickType_t ticks;
    if (timeout_us == 0) {
        ticks = 0;
    } else {
        // Convert us to ms, rounding up, then to ticks
        uint32_t timeout_ms = (timeout_us + 999) / 1000;
        if (timeout_ms == 0) {
            timeout_ms = 1;  // Minimum 1ms
        }
        ticks = pdMS_TO_TICKS(timeout_ms);
        if (ticks == 0) {
            ticks = 1;  // Minimum 1 tick
        }
    }

    return xSemaphoreTake(m_handle, ticks) == pdTRUE;
}

bool BinarySemaphore::wait_blocking() {
    if (m_handle == nullptr) {
        return false;
    }
    return xSemaphoreTake(m_handle, portMAX_DELAY) == pdTRUE;
}

bool BinarySemaphore::wait_nonblocking() {
    if (m_handle == nullptr) {
        return false;
    }
    return xSemaphoreTake(m_handle, 0) == pdTRUE;
}

void BinarySemaphore::signal() {
    if (m_handle == nullptr) {
        return;
    }
    xSemaphoreGive(m_handle);
}

void BinarySemaphore::signal_ISR() {
    if (m_handle == nullptr) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(m_handle, &xHigherPriorityTaskWoken);

    // Yield to higher priority task if one was woken
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

}  // namespace RP2350
