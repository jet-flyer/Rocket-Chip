// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Fault injection hooks for bench testing (IVP-129).
// Callable via GDB `call fault_force_*()` from debug probe.
// Excluded from flight binary (BUILD_FOR_FLIGHT).
// See docs/FAULT_INJECTION.md for usage.

#ifndef BUILD_FOR_FLIGHT

#include "dev/fault_inject.h"
#include "rocketchip/config.h"
#include "rocketchip/ao_signals.h"
#include "safety/pio_backup_timer.h"
#include "hardware/pio.h"
#include <stdio.h>

extern "C" {
#include "qp_port.h"
}

// Globals checked by qv_idle_bridge (main.cpp)
volatile bool g_fault_core0_stall = false;
volatile uint32_t g_fault_watchdog_skip = 0;

// ESKF instance (owned by eskf_runner.cpp)
extern bool g_eskfInitialized;

// extern "C" + __attribute__((used)) prevent C++ mangling and compiler removal.
// --undefined in CMakeLists.txt prevents linker --gc-sections removal.
// These are only called via GDB `call`, not from firmware code.
extern "C" extern "C" __attribute__((used))
void fault_force_eskf_unhealthy() {
    g_eskfInitialized = false;
    printf("[FAULT] ESKF forced unhealthy\n");
}

extern "C" __attribute__((used))
void fault_force_core0_stall() {
    g_fault_core0_stall = true;
    printf("[FAULT] Core 0 stall requested — idle bridge will spin\n");
}

extern "C" __attribute__((used))
void fault_force_core0_stall_clear() {
    g_fault_core0_stall = false;
    printf("[FAULT] Core 0 stall cleared\n");
}

extern "C" __attribute__((used))
void fault_force_watchdog_stall(uint32_t skip_ticks) {
    g_fault_watchdog_skip = skip_ticks;
    printf("[FAULT] Watchdog skip for %lu ticks\n", (unsigned long)skip_ticks);
}

extern "C" __attribute__((used))
void fault_force_health_fail(uint8_t /*subsystem_index*/) {
    printf("[FAULT] Use GDB 'set' for health byte — see FAULT_INJECTION.md\n");
}

extern "C" __attribute__((used))
void fault_force_ao_queue_flood(uint8_t /*ao_priority*/, uint16_t count) {
    printf("[FAULT] Publishing %u dummy events (floods all AO queues)\n", count);
    static QEvt const s_dummyEvt = QEVT_INITIALIZER(rc::SIG_SENSOR_DATA);
    for (uint16_t i = 0; i < count; ++i) {
        QActive_publish_(&s_dummyEvt, (void*)0, 0U);
    }
}

extern "C" __attribute__((used))
void fault_force_pio_sm_halt() {
    pio2->ctrl &= ~0xFU;
    printf("[FAULT] PIO2 all SMs disabled (backup timers halted)\n");
}

#endif // BUILD_FOR_FLIGHT
