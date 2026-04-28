// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Diagnostic statistics for IVP-132 soak testing.
// Dumps per-AO queue depth high-water, MSP high-water, flash page count,
// radio retry count, health latch state, sensor temps.
// Dev-only (compiled in only when ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS is defined,
// which the build sets when NOT_CERTIFIED_FOR_FLIGHT=ON).
#ifndef ROCKETCHIP_DEV_DIAG_STATS_H
#define ROCKETCHIP_DEV_DIAG_STATS_H

#include <stdint.h>

// IVP-132a.4 T=0 precondition block — ALWAYS compiled (both bench and
// flight). Prints build/role/board identity, SX1276 RegVersion readback,
// IRQ pin state, SPI error counter. Required before any soak to catch
// Frankenstein builds. Readable by GDB via `call diag_stats_t0_preconditions()`.
extern "C" {
void diag_stats_t0_preconditions();
}

#ifndef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

static inline void diag_stats_dump()     {
    // Flight tier: only the T=0 block is available. Full stats dump is
    // bench-only.
    diag_stats_t0_preconditions();
}
static inline void diag_stats_msp_tick() {}

#else

extern "C" {
// Dump full diagnostic snapshot to serial. Called from debug menu 'd' key
// or via GDB `call diag_stats_dump()`.
void diag_stats_dump();
}

// Update MSP high-water mark. Called from QV idle callback every iteration.
// Tracks minimum-seen MSP across runtime = deepest stack usage.
void diag_stats_msp_tick();

#endif

#endif
