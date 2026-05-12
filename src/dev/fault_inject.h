// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Fault injection hooks for bench testing (IVP-129).
// Called via GDB `call fault_force_*()` from debug probe.
// Compiled in only when ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS is defined,
// which the build sets when NOT_CERTIFIED_FOR_FLIGHT=ON.
// See docs/FAULT_INJECTION.md for usage.
#ifndef ROCKETCHIP_DEV_FAULT_INJECT_H
#define ROCKETCHIP_DEV_FAULT_INJECT_H

#include <stdint.h>

#ifndef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

static inline void fault_force_eskf_unhealthy()                     {}
static inline void fault_force_core0_stall()                        {}
static inline void fault_force_core0_stall_clear()                  {}
static inline void fault_force_watchdog_stall(uint32_t)             {}
static inline void fault_force_health_fail(uint8_t)                 {}
static inline void fault_force_launch_abort()                       {}
static inline void fault_force_radio_dropout()                      {}
static inline void fault_force_ao_queue_flood(uint8_t, uint16_t)    {}
static inline void fault_force_pio_sm_halt()                        {}
static inline void fault_force_hardfault()                          {}

#else

extern "C" {
void fault_force_eskf_unhealthy();
void fault_force_core0_stall();
void fault_force_core0_stall_clear();
void fault_force_watchdog_stall(uint32_t skip_ticks);
void fault_force_health_fail(uint8_t subsystem_index);
void fault_force_launch_abort();
void fault_force_radio_dropout();
void fault_force_ao_queue_flood(uint8_t ao_priority, uint16_t count);
void fault_force_pio_sm_halt();
void fault_force_hardfault();
}

// Checked by QV idle callback — when true, idle spins instead of doing work
extern volatile bool g_fault_core0_stall;

// Checked by watchdog kick — when >0, skip kick and decrement
extern volatile uint32_t g_fault_watchdog_skip;

#endif

#endif // ROCKETCHIP_DEV_FAULT_INJECT_H
