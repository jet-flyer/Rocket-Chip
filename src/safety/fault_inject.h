// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// GDB fault_force_* hooks. Linked into the flight ELF; no-op unless
// rc::test_mode_active() (probe magic AND IDLE AND boot window).
// Gate: inject_arm_gate.h. Usage: docs/FAULT_INJECTION.md.
#ifndef ROCKETCHIP_SAFETY_FAULT_INJECT_H
#define ROCKETCHIP_SAFETY_FAULT_INJECT_H

#include <stdint.h>

extern "C" {
void fault_force_eskf_unhealthy();
void fault_force_core0_stall();
void fault_force_core0_stall_clear();   // Recovery action — NOT gated
void fault_force_watchdog_stall(uint32_t skip_ticks);
void fault_force_health_fail(uint8_t subsystem_index);
void fault_force_launch_abort();
void fault_force_radio_dropout();
void fault_force_ao_queue_flood(uint8_t ao_priority, uint16_t count);
void fault_force_pio_sm_halt();
void fault_force_hardfault();  // Gated. Writes MPU guard; no-op if gate closed.
}

// Checked by QV idle callback — when true, idle spins instead of doing work.
extern volatile bool g_fault_core0_stall;

// Checked by watchdog kick — when >0, skip kick and decrement.
extern volatile uint32_t g_fault_watchdog_skip;

#endif // ROCKETCHIP_SAFETY_FAULT_INJECT_H
