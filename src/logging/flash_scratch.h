// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Flash stall scratch loop (FSK pre-plan item 2). TEST BUILD ONLY.
// Enable with -DROCKETCHIP_FLASH_STALL_SCRATCH=ON (CMake option, OFF by
// default). This is a test affordance outside the flight image; field
// images do not contain it.
//
// Scratch area: rc::kFlashScratchStart .. rc::kFlashScratchEnd
// (flash_layout.h). It is the top 384 sectors (1.5 MB) of the default
// flight-log region, directly below the flash-safe test sector. In a
// scratch build the flight-log region ends at kFlashScratchStart, so the
// log and the scratch area do not overlap. Calibration, flight table,
// radio config, and the flash-safe test sector are above the area.
//
// Debug 'j' starts the loop, and 'j' again stops it:
//   1. Erase phase: one 4 KB sector for each 50 ms AO_RCOS tick, but no
//      op on a tick where AO_RCOS has a queue backlog (an erase is longer
//      than the tick period). Expect about one erase each 100 ms (~40 s).
//   2. Program phase: one 256 B page for each tick (20 pages/s) into the
//      erased area, until all pages are done (about 307 s).
// The loop runs only in the ground state (FD kIdle). If the phase is not
// kIdle, the loop stops at the next tick, so it never erases while ARMED.
// The loop refuses to start if a flight-table entry overlaps the area.
#ifndef ROCKETCHIP_LOGGING_FLASH_SCRATCH_H
#define ROCKETCHIP_LOGGING_FLASH_SCRATCH_H

namespace rc {

// Prints "[SCRATCH] ..." status. Default builds print "build=off".
void flash_scratch_dump();

#if defined(ROCKETCHIP_FLASH_STALL_SCRATCH)
void flash_scratch_toggle();  // Debug 'j'
void flash_scratch_tick();    // AO_RCOS 20 Hz tick, task context only
#endif

} // namespace rc

#endif // ROCKETCHIP_LOGGING_FLASH_SCRATCH_H
