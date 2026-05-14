// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side fault injection hooks (IVP-132a).
//
// R-25-exec step 6 (2026-05-13, per council-APPROVED Approach A in
// docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): migrated
// from src/dev/station_fault_inject.h. Lives in the single station
// flight binary; every fault_force_station_* entry checks
// rc::test_mode_active() and returns early if not armed (runtime
// partitioning per SWE-133). Only meaningful on the station
// (ROCKETCHIP_JOB_STATION=1).
//
// Callable via GDB `call fault_force_station_*()` from debug probe.
// See safety/test_mode.h for the gate; docs/FAULT_INJECTION.md for
// per-scenario usage.
#ifndef ROCKETCHIP_SAFETY_STATION_FAULT_INJECT_H
#define ROCKETCHIP_SAFETY_STATION_FAULT_INJECT_H

#include <stdint.h>

extern "C" {
void fault_force_station_rx_drop(uint8_t n);       // drop next N RX packets
void fault_force_station_ack_suppress(uint8_t n);  // ignore next N matching ACKs
void fault_force_station_gps_loss();               // clear station GPS valid flag
void fault_force_station_gps_restore();            // allow GPS fix to repopulate
}

// Checked by handle_rx_packet — when >0, drop the RX and decrement
extern volatile uint8_t g_fault_station_rx_drop_remaining;

// Checked by handle_rx_packet ACK-match block — when >0, ignore match
extern volatile uint8_t g_fault_station_ack_suppress_remaining;

#endif // ROCKETCHIP_SAFETY_STATION_FAULT_INJECT_H
