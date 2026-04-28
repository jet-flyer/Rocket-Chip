// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station-side fault injection hooks for bench testing (IVP-132a).
// Called via GDB `call fault_force_station_*()` from debug probe.
// Compiled in only when ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS is defined,
// which the build sets when NOT_CERTIFIED_FOR_FLIGHT=ON.
// Only meaningful when ROCKETCHIP_JOB_STATION=1 — no-ops on vehicle build.
// See docs/FAULT_INJECTION.md for usage.
#ifndef ROCKETCHIP_DEV_STATION_FAULT_INJECT_H
#define ROCKETCHIP_DEV_STATION_FAULT_INJECT_H

#include <stdint.h>

#ifndef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

static inline void fault_force_station_rx_drop(uint8_t)       {}
static inline void fault_force_station_ack_suppress(uint8_t)  {}
static inline void fault_force_station_gps_loss()             {}
static inline void fault_force_station_gps_restore()          {}

#else

extern "C" {
void fault_force_station_rx_drop(uint8_t n);       // drop next N RX packets
void fault_force_station_ack_suppress(uint8_t n);  // ignore next N matching ACKs
void fault_force_station_gps_loss();               // clear station GPS valid flag
void fault_force_station_gps_restore();            // allow GPS fix to repopulate
}

// Checked by handle_rx_packet — when >0, drop the RX and decrement
extern volatile uint8_t g_fault_station_rx_drop_remaining;

// Checked by handle_rx_packet ACK-match block — when >0, ignore match and decrement
extern volatile uint8_t g_fault_station_ack_suppress_remaining;

#endif

#endif // ROCKETCHIP_DEV_STATION_FAULT_INJECT_H
