/**
 * @file Util.cpp
 * @brief AP_HAL Util implementation for RP2350
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "Util.h"
#include "Scheduler.h"

#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "FreeRTOS.h"
#include "task.h"

#include <cstdio>

namespace RP2350 {

// ============================================================================
// Constructor
// ============================================================================

Util::Util()
    : m_soft_armed(false)
    , m_last_armed_change_ms(0)
{
}

// ============================================================================
// Memory Information
// ============================================================================

uint32_t Util::available_memory() const {
    // With static allocation, xPortGetFreeHeapSize() returns 0 if no heap is used.
    // Fall back to total heap size in that case (all heap is "available").
    uint32_t free_heap = xPortGetFreeHeapSize();
    if (free_heap == 0) {
        // No dynamic allocations - report total heap as available
        return configTOTAL_HEAP_SIZE;
    }
    return free_heap;
}

uint32_t Util::total_memory() const {
    return configTOTAL_HEAP_SIZE;
}

void Util::mem_info(char* buffer, uint16_t buflen) const {
    if (buffer == nullptr || buflen == 0) {
        return;
    }

    uint32_t free_heap = available_memory();
    uint32_t total_heap = total_memory();
    uint32_t used = total_heap - free_heap;
    uint32_t percent = (used * 100) / total_heap;

    snprintf(buffer, buflen,
             "Heap: %lu/%lu bytes (%lu%% used)",
             static_cast<unsigned long>(used),
             static_cast<unsigned long>(total_heap),
             static_cast<unsigned long>(percent));
}

// ============================================================================
// System Identification
// ============================================================================

bool Util::get_system_id(char* buffer, uint8_t buflen) const {
    if (buffer == nullptr || buflen < 25) {
        return false;
    }

    // Get the unique board ID from flash
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    // Format as hex string: "XXXX-XXXX-XXXX-XXXX"
    snprintf(buffer, buflen,
             "%02X%02X-%02X%02X-%02X%02X-%02X%02X",
             id.id[0], id.id[1], id.id[2], id.id[3],
             id.id[4], id.id[5], id.id[6], id.id[7]);

    return true;
}

uint8_t Util::get_system_id_unformatted(uint8_t* buffer, uint8_t buflen) const {
    if (buffer == nullptr || buflen == 0) {
        return 0;
    }

    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);

    uint8_t copy_len = (buflen < PICO_UNIQUE_BOARD_ID_SIZE_BYTES)
                       ? buflen : PICO_UNIQUE_BOARD_ID_SIZE_BYTES;

    memcpy(buffer, id.id, copy_len);
    return copy_len;
}

// ============================================================================
// Safety State
// ============================================================================

SafetyState Util::safety_switch_state() const {
    // RocketChip has no physical safety switch
    return SafetyState::SAFETY_NONE;
}

// ============================================================================
// Arming State
// ============================================================================

bool Util::get_soft_armed() const {
    return m_soft_armed;
}

void Util::set_soft_armed(bool armed) {
    if (m_soft_armed != armed) {
        m_soft_armed = armed;
        m_last_armed_change_ms = Scheduler::millis();
    }
}

uint32_t Util::get_last_armed_change_ms() const {
    return m_last_armed_change_ms;
}

// ============================================================================
// System Load
// ============================================================================

uint16_t Util::get_system_load() const {
    // FreeRTOS doesn't provide direct CPU load measurement.
    // We could use runtime stats if enabled, but for now return 0.
    // TODO: Implement using configGENERATE_RUN_TIME_STATS
    return 0;
}

// ============================================================================
// Diagnostics
// ============================================================================

void Util::thread_info(char* buffer, uint16_t buflen) const {
    if (buffer == nullptr || buflen == 0) {
        return;
    }

#if (configUSE_TRACE_FACILITY == 1)
    // Get task list if trace facility enabled
    vTaskList(buffer);
#else
    // Just report number of tasks
    UBaseType_t num_tasks = uxTaskGetNumberOfTasks();
    snprintf(buffer, buflen, "Tasks: %u (trace disabled)",
             static_cast<unsigned int>(num_tasks));
#endif
}

void Util::timer_info(char* buffer, uint16_t buflen) const {
    if (buffer == nullptr || buflen == 0) {
        return;
    }

    uint64_t uptime_ms = Scheduler::millis64();
    uint32_t seconds = static_cast<uint32_t>(uptime_ms / 1000);
    uint32_t minutes = seconds / 60;
    uint32_t hours = minutes / 60;

    snprintf(buffer, buflen,
             "Uptime: %02lu:%02lu:%02lu",
             static_cast<unsigned long>(hours),
             static_cast<unsigned long>(minutes % 60),
             static_cast<unsigned long>(seconds % 60));
}

}  // namespace RP2350
