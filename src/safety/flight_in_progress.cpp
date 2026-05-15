// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// flight_in_progress sentinel — boot-time evidence that the firmware was
// armed/airborne when something reset it. See safety/crash_record.h for
// the declarations. Implemented in its own file (separate from
// crash_record.cpp) so it can be linked into host-test targets without
// dragging the target-only `dsb` / `wfi` / AIRCR code path from
// crash_record_capture().

#include "safety/crash_record.h"

namespace rc {

#ifdef ROCKETCHIP_HOST_TEST
// Host test build: no Pico SDK, no .uninitialized_data linker section, no
// ARM dsb instruction. Plain global; host tests can read/write directly to
// exercise the sentinel logic without needing reset-survival semantics.
// `dsb` is replaced with a compiler-barrier-only equivalent — host tests
// don't run on a multi-issue Cortex-M33 and don't need real memory-system
// ordering, only compiler-reordering protection.
static volatile uint32_t g_flight_in_progress_magic = 0U;
#define FLIGHT_SENTINEL_DSB() __asm volatile ("" ::: "memory")
#else
__attribute__((section(".uninitialized_data"), used))
static volatile uint32_t g_flight_in_progress_magic;
#define FLIGHT_SENTINEL_DSB() __asm volatile ("dsb" ::: "memory")
#endif

void flight_in_progress_set() {
    g_flight_in_progress_magic = kFlightInProgressMagic;
    FLIGHT_SENTINEL_DSB();
}

void flight_in_progress_clear() {
    g_flight_in_progress_magic = 0;
    FLIGHT_SENTINEL_DSB();
}

bool flight_in_progress_was_set() {
    const bool was_set = (g_flight_in_progress_magic == kFlightInProgressMagic);
    g_flight_in_progress_magic = 0;
    FLIGHT_SENTINEL_DSB();
    return was_set;
}

} // namespace rc
