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
 * Adapted from AP_HAL_ESP32/DeviceBus.h for RP2350
 */

#pragma once

#include <inttypes.h>
#include <AP_HAL/HAL.h>
#include "Semaphores.h"

#include "FreeRTOS.h"
#include "task.h"

namespace RP2350 {

/**
 * @brief Per-bus callback management (adapted from ESP32 HAL)
 *
 * Creates a dedicated FreeRTOS task per bus that manages periodic callbacks.
 * This is the standard ArduPilot pattern for device polling.
 */
class DeviceBus {
public:
    DeviceBus(uint8_t thread_priority);

    Semaphore semaphore;

    AP_HAL::Device::PeriodicHandle register_periodic_callback(
        uint32_t period_usec,
        AP_HAL::Device::PeriodicCb cb,
        AP_HAL::Device *hal_device);

    bool adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec);

private:
    static void bus_thread(void *arg);

    struct callback_info {
        struct callback_info *next;
        AP_HAL::Device::PeriodicCb cb;
        uint32_t period_usec;
        uint64_t next_usec;
    };

    callback_info *callbacks;
    uint8_t thread_priority;
    TaskHandle_t bus_thread_handle;
    bool thread_started;
    AP_HAL::Device *hal_device;
};

}  // namespace RP2350
