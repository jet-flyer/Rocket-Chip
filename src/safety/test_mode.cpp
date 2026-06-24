// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// test_mode — runtime gate for test/fault-injection affordances in the
// flight binary. See test_mode.h for full design rationale + council
// decision provenance.

#include "safety/test_mode.h"
#include "flight_director/flight_state.h"

#ifdef ROCKETCHIP_HOST_TEST
// Host test build: no Pico SDK. Section attributes are no-ops; the
// global becomes a plain variable that host tests can read/write
// directly via rc::g_test_mode_arm_magic. The boot-time-window check
// uses a static counter that defaults to 0 ms (tests can advance via
// the test fixture if needed).
static inline uint32_t test_mode_boot_ms() { return 0U; }
#define TEST_MODE_SRAM_ATTR
#else
#include "pico/time.h"
static inline uint32_t test_mode_boot_ms() {
    return to_ms_since_boot(get_absolute_time());
}
#define TEST_MODE_SRAM_ATTR __attribute__((section(".uninitialized_data"), used))
#endif

namespace rc {

// .uninitialized_data SRAM word — probe writes kTestModeMagic here to
// arm. Mirrors crash_record's storage pattern (proven survives reset
// on target; plain global on host).
//
// extern + used so the linker keeps it across LTO + GDB / probe can
// `set var rc::g_test_mode_arm_magic = 0x7E57'BABE` reach the symbol.
TEST_MODE_SRAM_ATTR
volatile uint32_t g_test_mode_arm_magic;

// Runtime flag — true iff all three arming conditions are currently
// satisfied. Cleared aggressively (any IDLE-exit, any failed re-evaluation).
volatile bool g_test_mode_enabled = false;

// One-shot: was an arm magic observed at boot? Captured in
// test_mode_init() so test_mode_evaluate() can re-evaluate the
// boot-time-window + state-machine conditions without re-reading
// the cleared SRAM.
static volatile bool s_magic_observed_at_boot = false;

// Storage for the phase accessor registered by AO_FlightDirector_start.
// Typedef is in the public header so callers can name the pointer type.
// Until registered, test_mode_evaluate() refuses to arm — fail-safe.
static FlightPhaseAccessor s_phase_accessor = nullptr;

void test_mode_register_phase_accessor(FlightPhaseAccessor fn) {
    s_phase_accessor = fn;
}

bool test_mode_magic_observed_at_boot() {
    return s_magic_observed_at_boot;
}

void test_mode_init() {
    // Single-use read of the arm magic. If present, clear it
    // immediately — operator must re-arm next session. This prevents
    // a never-cleared probe write from accidentally re-arming on
    // every reboot.
    if (g_test_mode_arm_magic == kTestModeMagic) {
        s_magic_observed_at_boot = true;
        g_test_mode_arm_magic = 0;
#ifndef ROCKETCHIP_HOST_TEST
        __asm volatile ("dsb" ::: "memory");
#endif
    } else {
        s_magic_observed_at_boot = false;
    }
    g_test_mode_enabled = false;
}

void test_mode_evaluate() {
    // If no magic observed at boot, the flag stays false forever
    // (no path can flip it true without a fresh boot + probe write).
    if (!s_magic_observed_at_boot) {
        g_test_mode_enabled = false;
        return;
    }

    // Condition (b): current phase must be kIdle. Without the
    // accessor wired in (early boot, before FD init), refuse to
    // arm — fail-safe direction.
    if (s_phase_accessor == nullptr) {
        g_test_mode_enabled = false;
        return;
    }
    if (s_phase_accessor() != FlightPhase::kIdle) {
        g_test_mode_enabled = false;
        return;
    }

    // Condition (c): boot-time window.
    uint32_t now_ms = test_mode_boot_ms();
    if (now_ms >= kTestModeArmWindowMs) {
        g_test_mode_enabled = false;
        return;
    }

    // All three conditions satisfied.
    g_test_mode_enabled = true;
}

void test_mode_clear_on_idle_exit() {
    // First clearing gate (per council amendment #2): any state
    // transition out of kIdle force-clears the flag. Once cleared
    // this way, the flag cannot re-arm without a fresh boot + probe
    // write — because s_magic_observed_at_boot was cleared on the
    // single-use init read and test_mode_evaluate() observes that
    // as a hard fail-closed.
    s_magic_observed_at_boot = false;
    g_test_mode_enabled = false;
#ifndef ROCKETCHIP_HOST_TEST
    __asm volatile ("dsb" ::: "memory");
#endif
}

const char* test_mode_status_string() {
    if (g_test_mode_enabled) {
        return "active";
    }
    if (s_magic_observed_at_boot) {
        return "stale-arm";
    }
    return "off";
}

} // namespace rc
