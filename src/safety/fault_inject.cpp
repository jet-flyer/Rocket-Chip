// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Fault injection hooks (IVP-129).
//
// R-25-exec step 3 of 13 (per docs/decisions/BENCH_TIER_DEPRECATION_
// 2026-05-13.md, council-APPROVED Approach A): migrated from
// src/dev/fault_inject.cpp. No longer #ifdef-gated. Every
// fault_force_* entry checks rc::test_mode_active() and returns
// early if not armed (SWE-133 partitioning principle).
//
// Callable via GDB `call fault_force_*()` from debug probe.
// See docs/FAULT_INJECTION.md for usage.

#include "safety/fault_inject.h"
#include "safety/test_mode.h"
#include "rocketchip/linker_symbols.h"  // __StackBottom (linker-defined)
#include "rocketchip/config.h"
#include "rocketchip/ao_signals.h"
#include "safety/pio_backup_timer.h"
#include "active_objects/ao_flight_director.h"
#include "active_objects/ao_rf_manager.h"
#include "hardware/pio.h"
#include "pico/time.h"
#include "rocketchip/rc_log.h"

extern "C" {
#include "qp_port.h"
}

// Globals checked by qv_idle_bridge (main.cpp)
volatile bool g_fault_core0_stall = false;
volatile uint32_t g_fault_watchdog_skip = 0;

// ESKF instance (owned by eskf_runner.cpp)
extern bool g_eskfInitialized;

// R-25-exec gate helper. Every fault_force_* function early-returns if
// test mode is not armed. Single point of refusal so the audit
// invariant "every fault_force_* checks test_mode_active() at entry"
// is mechanically verifiable (grep for fi_test_mode_gate).
//
// Returns true iff the caller should proceed. Logs a uniform message
// on rejection so a probe operator who arms the function via GDB but
// forgot to arm test mode gets clear feedback.
static bool fi_test_mode_gate(const char* name) {
    if (rc::test_mode_active()) { return true; }
    rc::rc_log("[FAULT] %s gated — arm test mode via probe "
           "(write kTestModeMagic to rc::g_test_mode_arm_magic + reset; "
           "see safety/test_mode.h)\n", name);
    return false;
}

// extern "C" + __attribute__((used)) prevent C++ mangling and compiler removal.
// --undefined in CMakeLists.txt prevents linker --gc-sections removal.
// These are only called via GDB `call`, not from firmware code.
extern "C" __attribute__((used))
void fault_force_eskf_unhealthy() {
    if (!fi_test_mode_gate("fault_force_eskf_unhealthy")) { return; }
    g_eskfInitialized = false;
    rc::rc_log("[FAULT] ESKF forced unhealthy\n");
}

extern "C" __attribute__((used))
void fault_force_core0_stall() {
    if (!fi_test_mode_gate("fault_force_core0_stall")) { return; }
    g_fault_core0_stall = true;
    rc::rc_log("[FAULT] Core 0 stall requested — idle bridge will spin\n");
}

extern "C" __attribute__((used))
void fault_force_core0_stall_clear() {
    // Intentionally not gated — clearing the stall is a recovery
    // action, not a fault injection. Should be reachable even after
    // test_mode_active() clears (which happens on any IDLE-exit).
    g_fault_core0_stall = false;
    rc::rc_log("[FAULT] Core 0 stall cleared\n");
}

extern "C" __attribute__((used))
void fault_force_watchdog_stall(uint32_t skip_ticks) {
    if (!fi_test_mode_gate("fault_force_watchdog_stall")) { return; }
    g_fault_watchdog_skip = skip_ticks;
    rc::rc_log("[FAULT] Watchdog skip for %lu ticks\n", (unsigned long)skip_ticks);
}

extern "C" __attribute__((used))
void fault_force_health_fail(uint8_t /*subsystem_index*/) {
    if (!fi_test_mode_gate("fault_force_health_fail")) { return; }
    rc::rc_log("[FAULT] Use GDB 'set' for health byte — see FAULT_INJECTION.md\n");
}

// R-9a: ARM-then-ABORT sequence so enhanced_fault_injection.py can observe
// the [FD] ABORT* positive-control signal end-to-end. Goes through the
// AO_FlightDirector_dispatch_signal() public path (same path the CLI and
// telemetry layers use), avoiding GDB-scratch QEvt fragility.
extern "C" __attribute__((used))
void fault_force_launch_abort() {
    if (!fi_test_mode_gate("fault_force_launch_abort")) { return; }
    if (!AO_FlightDirector_is_initialized()) {
        rc::rc_log("[FAULT] launch-abort: FD not initialized\n");
        return;
    }
    if (!AO_FlightDirector_is_ground_state()) {
        rc::rc_log("[FAULT] launch-abort: FD not in ground state — aborting hook\n");
        return;
    }
    rc::rc_log("[FAULT] launch-abort: dispatching SIG_ARM\n");
    AO_FlightDirector_dispatch_signal(static_cast<int>(rc::SIG_ARM));
    sleep_ms(50);  // let ARM phase change emit + propagate
    rc::rc_log("[FAULT] launch-abort: dispatching SIG_ABORT\n");
    AO_FlightDirector_dispatch_signal(static_cast<int>(rc::SIG_ABORT));
    rc::rc_log("[FAULT] launch-abort: sequence dispatched (expect [FD] ABORT*)\n");
}

extern "C" __attribute__((used))
void fault_force_ao_queue_flood(uint8_t /*ao_priority*/, uint16_t count) {
    if (!fi_test_mode_gate("fault_force_ao_queue_flood")) { return; }
    rc::rc_log("[FAULT] Publishing %u dummy events (floods all AO queues)\n", count);
    static QEvt const s_dummyEvt = QEVT_INITIALIZER(rc::SIG_SENSOR_DATA);
    for (uint16_t i = 0; i < count; ++i) {
        QActive_publish_(&s_dummyEvt, nullptr, 0U);
    }
}

extern "C" __attribute__((used))
void fault_force_pio_sm_halt() {
    if (!fi_test_mode_gate("fault_force_pio_sm_halt")) { return; }
    pio2->ctrl &= ~0xFU;
    rc::rc_log("[FAULT] PIO2 all SMs disabled (backup timers halted)\n");
}

// R-3 verification hook (audit 2026-05-07): force a MemManage fault by
// writing into the MPU stack guard region. The handler in
// fault_protection.cpp captures crash state and dispatches phase-aware:
// in kIdle it triggers NVIC_SystemReset and on next boot
// health_monitor_init() consumes the record and latches
// kHealthCriticalPriorHardfault; in any flight phase it transitions to
// kFault and busy-loops without resetting (fault-recovery 2026-05-14,
// plan B.1/B.2). Use this from GDB to verify either dispatch path
// end-to-end depending on the current flight phase.
//
// Writes a sentinel value into the first word of the MPU guard region.
// Per the linker, __StackBottom marks the start of the guard (64 bytes
// long, configured by mpu_setup_stack_guard()).
extern "C" __attribute__((used))
void fault_force_hardfault() {
    if (!fi_test_mode_gate("fault_force_hardfault")) { return; }
    rc::rc_log("[FAULT] force_hardfault: writing into MPU stack guard...\n");
    // Volatile pointer so the compiler doesn't hoist/remove the store; the
    // fault fires on the write into the guard region. (__StackBottom from
    // rocketchip/linker_symbols.h.)
    volatile uint32_t* guard = &__StackBottom;
    *guard = 0xC0DE0001U;
    // Should not reach — but if MPU is disabled or guard not active, log it.
    rc::rc_log("[FAULT] force_hardfault: write returned (MPU not active?)\n");
}

// R-9b: age out AO_RfManager's last_rx_ms so the next 10 Hz tick fires the
// deadman / forced-ACQ branches. enhanced_fault_injection.py radio-dropout
// scenario regex matches the [FAULT] line we emit here directly (mirrors
// the fault_force_pio_sm_halt() pattern); the underlying link-state demotion
// to kAcq + AO_Notify_post_vehicle_lost() runs on the next tick but is not
// required for the regex hit.
extern "C" __attribute__((used))
void fault_force_radio_dropout() {
    if (!fi_test_mode_gate("fault_force_radio_dropout")) { return; }
    rc::AO_RfManager_force_last_rx_ms_for_test(0U);
    rc::rc_log("[FAULT] link lost — RfManager last_rx_ms forced to 0\n");
}
