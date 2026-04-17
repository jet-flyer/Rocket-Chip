// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side fault injection (IVP-132a). Excluded from flight binary.
// Hook points are wired in ao_telemetry.cpp (RX path) and drivers/gps_uart.cpp
// (GPS valid flag). Call from GDB via `call fault_force_station_*()`.

#ifndef BUILD_FOR_FLIGHT

#include "dev/station_fault_inject.h"

// These globals are always defined (unified build) but only honored in the
// station RX path (guarded by kRadioModeRx / ROCKETCHIP_JOB_STATION).
// Naming matches vehicle fault_inject.cpp pattern.
volatile uint8_t g_fault_station_rx_drop_remaining = 0;
volatile uint8_t g_fault_station_ack_suppress_remaining = 0;

#ifdef ROCKETCHIP_JOB_STATION
#include "core1/sensor_core1.h"
#endif

extern "C" {

void fault_force_station_rx_drop(uint8_t n) {
    g_fault_station_rx_drop_remaining = n;
}

void fault_force_station_ack_suppress(uint8_t n) {
    g_fault_station_ack_suppress_remaining = n;
}

void fault_force_station_gps_loss() {
#ifdef ROCKETCHIP_JOB_STATION
    g_bestGpsValid.store(false);
#endif
}

void fault_force_station_gps_restore() {
#ifdef ROCKETCHIP_JOB_STATION
    // No-op helper — Core 1 sensor loop will repopulate on next fix.
    // Exists so GDB users can explicitly end the fault-injected state.
    g_bestGpsValid.store(false);
#endif
}

__attribute__((used))
void fault_station_keep_alive() {
    (void)fault_force_station_rx_drop;
    (void)fault_force_station_ack_suppress;
    (void)fault_force_station_gps_loss;
    (void)fault_force_station_gps_restore;
}

}  // extern "C"

#endif  // !BUILD_FOR_FLIGHT
