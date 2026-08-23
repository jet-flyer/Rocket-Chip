// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Station GDB fault_force_station_* hooks. Flight ELF; no-op unless
// rc::test_mode_active(). Meaningful on JOB_STATION. Gate:
// inject_arm_gate.h. Usage: docs/FAULT_INJECTION.md.
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
