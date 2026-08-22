// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Soak snapshot + T=0 identity check. Read-only — always compiled.
// Dump: rc_os_debug 'd' or GDB call diag_stats_dump().
#ifndef ROCKETCHIP_DIAG_DIAG_STATS_H
#define ROCKETCHIP_DIAG_DIAG_STATS_H

#include <stdint.h>

extern "C" {
// T=0 precondition block — build/role/board identity, RegVersion
// readback, IRQ pin state, SPI error counter. Used at T=0 of every
// soak to catch Frankenstein builds.
void diag_stats_t0_preconditions();

// Full diagnostic snapshot to serial. Read-only — safe to run from
// any phase. Includes T=0 block + per-AO queue depths + MSP high-
// water + radio counters + health latches + sensor temps.
void diag_stats_dump();
}

// Update MSP high-water mark. Called from QV idle callback every
// iteration. Tracks minimum-seen MSP = deepest stack usage.
void diag_stats_msp_tick();

#endif // ROCKETCHIP_DIAG_DIAG_STATS_H
