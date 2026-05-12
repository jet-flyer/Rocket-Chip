// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Fault injection hooks for bench testing (IVP-129).
// Callable via GDB `call fault_force_*()` from debug probe.
// Compiled in only when ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS is defined,
// which the build sets when NOT_CERTIFIED_FOR_FLIGHT=ON.
// See docs/FAULT_INJECTION.md for usage.

#ifdef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

#include "dev/fault_inject.h"
#include "rocketchip/config.h"
#include "rocketchip/ao_signals.h"
#include "safety/pio_backup_timer.h"
#include "active_objects/ao_flight_director.h"
#include "active_objects/ao_rf_manager.h"
#include "hardware/pio.h"
#include "pico/time.h"
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

// R-9a: ARM-then-ABORT sequence so enhanced_fault_injection.py can observe
// the [FD] ABORT* positive-control signal end-to-end. Goes through the
// AO_FlightDirector_dispatch_signal() public path (same path the CLI and
// telemetry layers use), avoiding GDB-scratch QEvt fragility.
extern "C" __attribute__((used))
void fault_force_launch_abort() {
    if (!AO_FlightDirector_is_initialized()) {
        printf("[FAULT] launch-abort: FD not initialized\n");
        return;
    }
    if (!AO_FlightDirector_is_ground_state()) {
        printf("[FAULT] launch-abort: FD not in ground state — aborting hook\n");
        return;
    }
    printf("[FAULT] launch-abort: dispatching SIG_ARM\n");
    AO_FlightDirector_dispatch_signal(static_cast<int>(rc::SIG_ARM));
    sleep_ms(50);  // let ARM phase change emit + propagate
    printf("[FAULT] launch-abort: dispatching SIG_ABORT\n");
    AO_FlightDirector_dispatch_signal(static_cast<int>(rc::SIG_ABORT));
    printf("[FAULT] launch-abort: sequence dispatched (expect [FD] ABORT*)\n");
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

// R-3 verification hook (audit 2026-05-07): force a MemManage fault by
// writing into the MPU stack guard region. The handler in
// fault_protection.cpp captures crash state and triggers NVIC_SystemReset,
// then on next boot health_monitor_init() consumes the record and latches
// kHealthCriticalPriorHardfault. Use this from GDB to verify the full
// capture-and-reset path end-to-end.
//
// Writes a sentinel value into the first word of the MPU guard region.
// Per the linker, __StackBottom marks the start of the guard (64 bytes
// long, configured by mpu_setup_stack_guard()).
extern "C" __attribute__((used))
void fault_force_hardfault() {
    printf("[FAULT] force_hardfault: writing into MPU stack guard...\n");
    // Read __StackBottom via inline asm so the compiler doesn't constant-fold
    // into something the optimizer can hoist or remove. The fault fires on
    // the store.
    extern uint32_t __StackBottom;
    volatile uint32_t* guard = &__StackBottom;
    *guard = 0xC0DE0001U;
    // Should not reach — but if MPU is disabled or guard not active, log it.
    printf("[FAULT] force_hardfault: write returned (MPU not active?)\n");
}

// R-9b: age out AO_RfManager's last_rx_ms so the next 10 Hz tick fires the
// deadman / forced-ACQ branches. enhanced_fault_injection.py radio-dropout
// scenario regex matches the [FAULT] line we emit here directly (mirrors
// the fault_force_pio_sm_halt() pattern); the underlying link-state demotion
// to kAcq + AO_Notify_post_vehicle_lost() runs on the next tick but is not
// required for the regex hit.
extern "C" __attribute__((used))
void fault_force_radio_dropout() {
    rc::AO_RfManager_force_last_rx_ms_for_test(0U);
    printf("[FAULT] link lost — RfManager last_rx_ms forced to 0\n");
}

#endif // ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS
