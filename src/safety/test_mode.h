// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// test_mode — runtime gate for test/fault-injection affordances in the
// flight binary. Replaces the parallel bench-tier build pattern that
// caused R-23, F-2026-05-13-004, and R-22 design wall (all surfaced
// during DC-2026-05-13 audit cleanup).
//
// Design (council-approved 2026-05-13 unanimous: NASA/JPL + Prof +
// ArduPilot + Cubesat) per docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md
// "Approach A":
//
//   - Single binary; test affordances coexist with flight code.
//   - g_test_mode_enabled flag gates every test entry point.
//   - Three-condition AND gate to arm:
//       (a) Debug probe writes kTestModeMagic to .uninitialized_data SRAM
//           (probe-only physical-presence signal; adversary cannot reach
//           over LoRa, in-flight stray cannot reach without bench access).
//       (b) Current flight phase == kIdle (state-machine condition).
//       (c) millis() < kTestModeArmWindowMs (boot-time window closes
//           the arming opportunity after init settles).
//   - Two clearing gates (Therac-25 precedent — single gate is
//     insufficient):
//       (a) Any state transition out of kIdle clears the flag.
//       (b) Flight Director refuses to enter kArmed if flag is set.
//   - No persistence across power-on reset (.uninitialized_data
//     survives AIRCR/warm-reboot but operator MUST re-arm each session;
//     clearing on any boot path is intentional defense-in-depth).
//   - No CLI fallback for arming (CSE council amendment — CLI
//     fallbacks get normalized into routine ops use, defeating the
//     physical-presence guarantee).
//
// PX4 SYS_FAILURE_EN is the IRL precedent: failure injection lives in
// the flight binary, parameter-gated, default off. RocketChip's twist
// is the three-condition AND + probe-only arming, suited to a single-
// developer hobbyist project without an authenticated MAVLink link.
//============================================================================
#ifndef ROCKETCHIP_SAFETY_TEST_MODE_H
#define ROCKETCHIP_SAFETY_TEST_MODE_H

#include <stdbool.h>
#include <stdint.h>

#include "flight_director/flight_state.h"   // FlightPhase enum

namespace rc {

// SRAM magic — randomly-chosen, stable. Same arming pattern as
// crash_record.h's kCrashRecordMagic. The probe writes this value
// to g_test_mode_arm_magic; runtime polls it as condition (a).
//
// Distinct from kCrashRecordMagic so a stale crash record cannot
// inadvertently arm test mode.
static constexpr uint32_t kTestModeMagic = 0x7E57'BABEU;  // "TEST BABE" mnemonic

// Boot-time window (milliseconds since boot) during which test mode
// can be armed. Closes after this window — operator must arm within
// the first N seconds after a fresh boot. Defense-in-depth against
// long-running drift (a probe-write that happens hours into operation
// is treated as suspect).
//
// Value sourced from PX4 parameter-arming convention (defaults to
// ~30 s post-boot for parameter-gated test modes).
static constexpr uint32_t kTestModeArmWindowMs = 30000U;  // 30 s

// .uninitialized_data SRAM word — probe writes kTestModeMagic here to
// arm. Layout pattern mirrors crash_record (proven survives AIRCR /
// SWD reset; operator-controllable via GDB `set var`).
extern volatile uint32_t g_test_mode_arm_magic;

// Runtime flag — true iff all three arming conditions are currently
// satisfied. NEVER set directly; only updated by test_mode_evaluate()
// called from the main loop / AO tick.
extern volatile bool g_test_mode_enabled;

// Initialize the module. Reads g_test_mode_arm_magic; if it equals
// kTestModeMagic, clears it (single-use semantics — re-arm requires
// a fresh probe write) and starts the boot-time-window timer. Called
// once from init_hardware() before any AO_FlightDirector start.
void test_mode_init();

// Phase-accessor registration. AO_FlightDirector_start() passes its
// phase accessor here so test_mode_evaluate() can read the current
// phase without a hard link-time dependency on the FD module.
//
// The accessor is a function pointer returning the current FlightPhase.
// Until this is called, test_mode_evaluate() refuses to arm (fail-
// safe direction — no phase accessor means we can't verify condition
// (b) of the three-condition AND gate).
typedef FlightPhase (*FlightPhaseAccessor)();
void test_mode_register_phase_accessor(FlightPhaseAccessor fn);

// Called from the AO tick (probably ao_health_monitor or ao_rcos).
// Re-evaluates the three-condition AND gate. Updates
// g_test_mode_enabled accordingly. Idempotent.
void test_mode_evaluate();

// Called from flight_director state transitions: clears the flag
// (forces it false) when the FD leaves kIdle. Both clearing-gate
// implementations: this function + the FD's refusal to enter kArmed
// if the flag is set.
void test_mode_clear_on_idle_exit();

// Single-source-of-truth read accessor. Use this at every test
// entry point (`if (!rc::test_mode_active()) return;`). NEVER
// reach for g_test_mode_enabled directly outside of test_mode.cpp
// — the accessor exists so future refactors can change the gate
// representation without touching every call site.
inline bool test_mode_active() {
    return g_test_mode_enabled;
}

// Was the arm magic observed at boot? Set once during test_mode_init()
// (single-use read of g_test_mode_arm_magic). Stays true for the whole
// boot session even after test_mode_clear_on_idle_exit() flips
// g_test_mode_enabled false — useful for callers that want to know
// "did the operator arm this boot?" even after the gate clears.
//
// Used by AO_RCOS_start() to choose kMenu output mode on boots where
// test mode was armed (deterministic output for warm_reboot_audit +
// other host-side scripts, both vehicle and station). R-25-exec step 11.
bool test_mode_magic_observed_at_boot();

// Status for boot banner / preflight VERDICT (test-mode-active
// surfaces as forced NO-GO). Returns one of:
//   "off"        — not armed
//   "active"     — three conditions satisfied; gate open
//   "stale-arm"  — magic present but conditions failed (boot-window
//                  expired or non-IDLE state)
const char* test_mode_status_string();

} // namespace rc

#endif // ROCKETCHIP_SAFETY_TEST_MODE_H
