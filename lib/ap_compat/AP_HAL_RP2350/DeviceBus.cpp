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
 * Adapted from AP_HAL_ESP32/DeviceBus.cpp for RP2350
 */

#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <cstdio>

#include "Scheduler.h"

#include "FreeRTOS.h"
#include "task.h"

#include <new>  // For placement new

// Debug output for DeviceBus (disable in production)
#define DEVICEBUS_DEBUG 0

#if DEVICEBUS_DEBUG
#define DBG_BUS(fmt, ...) printf("[DeviceBus] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG_BUS(fmt, ...) ((void)0)
#endif

using namespace RP2350;

extern const AP_HAL::HAL& hal;

// Stack size for bus threads - increased to 4KB to prevent sporadic crashes
// during I2C FIFO reads and context switches (hardfault in PendSV_Handler)
// 2KB was causing sporadic UNDEFINSTR faults during IMU callbacks
static constexpr uint32_t kBusThreadStackSize = 4096;

DeviceBus::DeviceBus(uint8_t _thread_priority)
    : semaphore(nullptr)  // Deferred init - call init_semaphore() before use
    , callbacks(nullptr)
    , thread_priority(_thread_priority)
    , bus_thread_handle(nullptr)
    , thread_started(false)
    , hal_device(nullptr)
{
    DBG_BUS("DeviceBus created, priority=%u", _thread_priority);
}

void DeviceBus::init_semaphore()
{
    if (semaphore == nullptr) {
        // Use placement new to construct semaphore in preallocated storage
        // This is called after FreeRTOS is running, when semaphore APIs are safe
        semaphore = new (semaphore_storage) Semaphore();
        DBG_BUS("Semaphore initialized via placement new");
    }
}

/*
 * Per-bus callback thread
 * Runs continuously, invoking callbacks when their time comes
 */
void DeviceBus::bus_thread(void *arg)
{
    DeviceBus *binfo = static_cast<DeviceBus *>(arg);

    DBG_BUS("bus_thread started");

    static uint32_t s_callback_count = 0;
    static uint32_t s_loop_count = 0;

    while (true) {
        s_loop_count++;
        if (s_loop_count <= 5 || s_loop_count % 1000 == 0) {
            DBG_BUS("loop #%lu, callbacks=%p", s_loop_count, (void*)binfo->callbacks);
        }

        uint64_t now = AP_HAL::micros64();
        callback_info *callback;

        // Find and run callbacks that are due
        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                // Advance next_usec (may need multiple advances if we're behind)
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // Call with semaphore held
                if (s_callback_count <= 5 || s_callback_count % 100 == 0) {
                    DBG_BUS("Taking semaphore for callback #%lu...", s_callback_count + 1);
                }
                if (binfo->semaphore->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
                    s_callback_count++;
                    if (s_callback_count <= 5 || s_callback_count % 100 == 0) {
                        DBG_BUS("CB #%lu START (cb=%p)", s_callback_count, (void*)&callback->cb);
                    }
                    callback->cb();
                    if (s_callback_count <= 5 || s_callback_count % 100 == 0) {
                        DBG_BUS("CB #%lu END", s_callback_count);
                    }
                    binfo->semaphore->give();
                }
            }
        }

        // Calculate when next callback is due
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = binfo->callbacks; callback; callback = callback->next) {
            if (next_needed == 0 || callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // Delay for at most 50ms to handle newly added callbacks
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // Clamp minimum delay to 100us
        if (delay < 100) {
            delay = 100;
        }

        // CRITICAL: Must use vTaskDelay() to yield to lower-priority tasks.
        // DeviceBus runs at priority 5, main task at priority 2-5.
        // If we busy-wait, lower-priority tasks (like wait_for_sample) starve.
        // Use minimum 1 tick delay - accepts 1ms rounding but ensures yielding.
        TickType_t ticks = pdMS_TO_TICKS((delay + 500) / 1000);
        if (ticks == 0) {
            ticks = 1;  // Always yield at least 1 tick
        }
        vTaskDelay(ticks);
    }
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(
    uint32_t period_usec,
    AP_HAL::Device::PeriodicCb cb,
    AP_HAL::Device *_hal_device)
{
    DBG_BUS("register_periodic_callback: period=%lu us", period_usec);

    // Initialize semaphore on first use (deferred from constructor)
    init_semaphore();

    // Allocate callback info FIRST (before thread creation)
    callback_info *callback = new callback_info;
    if (callback == nullptr) {
        DBG_BUS("Failed to allocate callback_info");
        return nullptr;
    }

    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // Add to linked list BEFORE creating thread
    // (thread may preempt us immediately if higher priority)
    callback->next = callbacks;
    callbacks = callback;

    // Memory barrier to ensure visibility to other cores
    __sync_synchronize();

    DBG_BUS("Callback added: callbacks=%p, period=%lu us", (void*)callbacks, period_usec);

    if (!thread_started) {
        thread_started = true;
        hal_device = _hal_device;

        // Create thread name based on bus type
        const char *name = "APM_Bus";
        if (hal_device) {
            switch (hal_device->bus_type()) {
            case AP_HAL::Device::BUS_TYPE_I2C:
                name = "APM_I2C";
                break;
            case AP_HAL::Device::BUS_TYPE_SPI:
                name = "APM_SPI";
                break;
            default:
                break;
            }
        }

        DBG_BUS("Creating bus thread: %s", name);

        // Create the bus thread - may preempt immediately!
        BaseType_t result = xTaskCreate(
            DeviceBus::bus_thread,
            name,
            kBusThreadStackSize,
            this,
            thread_priority,
            &bus_thread_handle
        );

        if (result != pdPASS) {
            DBG_BUS("Failed to create bus thread!");
            thread_started = false;
            // Remove the callback we just added
            callbacks = callback->next;
            delete callback;
            return nullptr;
        }

        // Pin bus thread to Core 0 to match main task (SMP only)
        // ArduPilot assumes single-core memory semantics (see PD12)
#if configNUMBER_OF_CORES > 1
        vTaskCoreAffinitySet(bus_thread_handle, (1 << 0));
        DBG_BUS("Bus thread pinned to Core 0");
#endif
    }

    return callback;
}

/*
 * Adjust the timer for the next call
 * Must be called from the bus thread
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    if (xTaskGetCurrentTaskHandle() != bus_thread_handle) {
        DBG_BUS("adjust_timer called from wrong thread");
        return false;
    }

    callback_info *callback = static_cast<callback_info *>(h);
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;
    return true;
}
