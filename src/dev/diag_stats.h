// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Diagnostic statistics for IVP-132 soak testing.
// Dumps per-AO queue depth high-water, MSP high-water, flash page count,
// radio retry count, health latch state, sensor temps.
// Dev-only (excluded from flight binary via BUILD_FOR_FLIGHT).
#ifndef ROCKETCHIP_DEV_DIAG_STATS_H
#define ROCKETCHIP_DEV_DIAG_STATS_H

#include <stdint.h>

#ifdef BUILD_FOR_FLIGHT

static inline void diag_stats_dump()     {}
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
