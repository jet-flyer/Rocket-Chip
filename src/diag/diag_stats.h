// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Soak snapshot + T=0 identity. Always compiled.
// Dump: CLI 'd' or GDB. Not ISR-safe. Touches radio SPI (RegVersion).
// AO queue getters have no critical section on this port (QF_CRIT is a no-op).
#ifndef ROCKETCHIP_DIAG_DIAG_STATS_H
#define ROCKETCHIP_DIAG_DIAG_STATS_H

#include <stdint.h>

extern "C" {
// T=0 precondition block — build/role/board identity, RegVersion
// readback, IRQ pin state, SPI error counter. Used at T=0 of every
// soak to catch Frankenstein builds.
void diag_stats_t0_preconditions();

// Serial dump: T=0 block + AO queues + MSP + radio counters + health + temps.
// Performs a radio SPI read. Pause Core 1 I2C first if sensors are sampling.
void diag_stats_dump();
}

// Update MSP high-water mark. Called from QV idle callback every
// iteration. Tracks minimum-seen MSP = deepest stack usage.
void diag_stats_msp_tick();

#endif // ROCKETCHIP_DIAG_DIAG_STATS_H
