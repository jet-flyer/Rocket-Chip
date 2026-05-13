// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Diagnostic statistics for IVP-132 soak testing.
//
// R-25-exec step 4 (2026-05-13, per council-APPROVED Approach A in
// docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): migrated
// from src/dev/diag_stats.h. Lives in the single flight binary; the
// full diag_stats_dump() runs unconditionally because it is a pure
// read-only snapshot of AO queue depth, MSP high-water, radio
// counters, health, and sensor state — no state mutation, no
// risk. Both diag_stats_dump() and diag_stats_msp_tick() are now
// always available (SWE-133 partitioning principle does not require
// gating reads).
//
// T=0 preconditions block always compiled. Dump callable via:
//   - rc_os_debug 'd' key (always available)
//   - GDB `call diag_stats_dump()`
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
