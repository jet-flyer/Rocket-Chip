// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// inject_arm_gate — probe-armed inject/debug gate for the flight binary.
// See inject_arm_gate.h.

#include "safety/inject_arm_gate.h"
#include "flight_director/flight_state.h"

#ifdef ROCKETCHIP_HOST_TEST
// HOST_TEST: no Pico SDK. Magic word is a plain global. Boot time is
// pinned at 0, so the arm-window expiry branch is not host-testable.
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
static volatile bool g_magicObservedAtBoot = false;

// Last phase published by FD (or host tests). Unknown until first note.
static bool g_phaseKnown = false;
static FlightPhase g_notedPhase = FlightPhase::kIdle;

void test_mode_note_phase(FlightPhase phase) {
    g_notedPhase = phase;
    g_phaseKnown = true;
}

bool test_mode_magic_observed_at_boot() {
    return g_magicObservedAtBoot;
}

void test_mode_init() {
    // Single-use read of the arm magic. If present, clear it
    // immediately — operator must re-arm next session. This prevents
    // a never-cleared probe write from accidentally re-arming on
    // every reboot.
    if (g_test_mode_arm_magic == kTestModeMagic) {
        g_magicObservedAtBoot = true;
        g_test_mode_arm_magic = 0;
#ifndef ROCKETCHIP_HOST_TEST
        __asm volatile ("dsb" ::: "memory");
#endif
    } else {
        g_magicObservedAtBoot = false;
    }
    g_test_mode_enabled = false;
    g_phaseKnown = false;
}

void test_mode_evaluate() {
    // If no magic observed at boot, the flag stays false forever
    // (no path can flip it true without a fresh boot + probe write).
    if (!g_magicObservedAtBoot) {
        g_test_mode_enabled = false;
        return;
    }

    // Condition (b): current phase must be kIdle. Without a published
    // phase (early boot, before FD init), refuse to arm — fail-safe.
    if (!g_phaseKnown) {
        g_test_mode_enabled = false;
        return;
    }
    if (g_notedPhase != FlightPhase::kIdle) {
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
    g_magicObservedAtBoot = false;
    g_test_mode_enabled = false;
#ifndef ROCKETCHIP_HOST_TEST
    __asm volatile ("dsb" ::: "memory");
#endif
}

const char* test_mode_status_string() {
    if (g_test_mode_enabled) {
        return "active";
    }
    if (g_magicObservedAtBoot) {
        return "stale-arm";
    }
    return "off";
}

} // namespace rc
