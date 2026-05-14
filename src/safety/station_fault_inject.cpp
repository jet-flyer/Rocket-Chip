// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side fault injection (IVP-132a).
//
// R-25-exec step 6 of 13 (per docs/decisions/BENCH_TIER_DEPRECATION_
// 2026-05-13.md, council-APPROVED Approach A): migrated from
// src/dev/station_fault_inject.cpp. No longer #ifdef-gated. Every
// fault_force_station_* entry checks rc::test_mode_active() and
// returns early if not armed (SWE-133 partitioning principle).
//
// Hook points are wired in ao_telemetry.cpp (RX path) and
// drivers/gps_uart.cpp (GPS valid flag). Call from GDB via
// `call fault_force_station_*()`. Only meaningful when
// ROCKETCHIP_JOB_STATION=1; vehicle builds inherit dead branches
// guarded by the runtime test_mode gate.

#include "safety/station_fault_inject.h"
#include "safety/test_mode.h"
#include <stdio.h>

#ifdef ROCKETCHIP_JOB_STATION
#include "core1/sensor_core1.h"
#endif

// Always defined (unified build) but only honored in the station
// RX path (guarded by kRadioModeRx / ROCKETCHIP_JOB_STATION).
// Naming matches vehicle fault_inject.cpp pattern.
volatile uint8_t g_fault_station_rx_drop_remaining = 0;
volatile uint8_t g_fault_station_ack_suppress_remaining = 0;

// R-25-exec gate helper. Mirrors fi_test_mode_gate() in
// src/safety/fault_inject.cpp.
static bool fis_test_mode_gate(const char* name) {
    if (rc::test_mode_active()) { return true; }
    printf("[FAULT] %s gated — arm test mode via probe "
           "(write kTestModeMagic to rc::g_test_mode_arm_magic + reset; "
           "see safety/test_mode.h)\n", name);
    return false;
}

extern "C" {

__attribute__((used))
void fault_force_station_rx_drop(uint8_t n) {
    if (!fis_test_mode_gate("fault_force_station_rx_drop")) { return; }
    g_fault_station_rx_drop_remaining = n;
}

__attribute__((used))
void fault_force_station_ack_suppress(uint8_t n) {
    if (!fis_test_mode_gate("fault_force_station_ack_suppress")) { return; }
    g_fault_station_ack_suppress_remaining = n;
}

__attribute__((used))
void fault_force_station_gps_loss() {
    if (!fis_test_mode_gate("fault_force_station_gps_loss")) { return; }
#ifdef ROCKETCHIP_JOB_STATION
    g_bestGpsValid.store(false);
#endif
}

__attribute__((used))
void fault_force_station_gps_restore() {
    // Recovery action — NOT gated. Symmetric with vehicle
    // fault_force_core0_stall_clear(): clearing the injected fault
    // must remain reachable even after test_mode_active() clears.
#ifdef ROCKETCHIP_JOB_STATION
    g_bestGpsValid.store(false);
#endif
}

}  // extern "C"
