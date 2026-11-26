// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QV BSP (Board Support Package) — RP2350 Pico SDK
//
// IVP-76: QF+QV BSP Integration (Stage 9: Active Object Architecture)
// Council-reviewed 2026-03-27 (Amendments A1, A2, A4)
//
// QV cooperative scheduler BSP for RP2350. Provides:
//   - QF_onStartup(): 100Hz tick via diag/qf_tick.cpp (SDK alarm, D1)
//   - QV_onIdle(): Bridge to existing superloop tick functions (D10)
//   - QF_onCleanup(): No-op (bare-metal never exits)
//
// The tick timer fires from hardware alarm IRQ on Core 0. Callback posts
// QF time events via QTIMEEVT_TICK_X(). QV dispatches AOs in the main
// loop, then calls QV_onIdle() when all queues are empty.
//
// During Stage 9 migration (IVP-76 through IVP-80), QV_onIdle runs the
// existing superloop tick functions. As modules migrate to Active Objects,
// their tick calls are removed from idle. IVP-81 makes idle pure __wfi().
//
// Reference: QP/C 8.1.3 examples/arm-cm/blinky_nucleo-c031c6/qv/bsp.c
//============================================================================

#include "qp_port.h"

#ifndef ROCKETCHIP_HOST_TEST

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/sync.h"

//----------------------------------------------------------------------------
// Tick timer — 100Hz SDK alarm (D1), in src/diag/qf_tick.cpp.
// The alarm callback runs in hardware timer IRQ context on Core 0 and
// posts QF time events. After a stall it fires one tick, not a burst
// (an SDK repeating timer replays each missed period).
extern bool qf_tick_start(void);

//----------------------------------------------------------------------------
// QF callbacks

void QF_onStartup(void) {
    // Start the 100Hz tick on a fixed 10 ms grid (start-to-start).
    // Return value checked per Council Amendment A1.
    bool ok = qf_tick_start();
    if (!ok) {
        // Timer allocation failed — no tick, no AO scheduling.
        // Enter hard fault: disable interrupts and spin for watchdog reset.
        __asm volatile("cpsid i");
        while (1) { __asm volatile("nop"); }
    }
}

void QF_onCleanup(void) {
    // Never called on bare-metal.
}

//----------------------------------------------------------------------------
// QV idle callback
//
// Called by QV scheduler when all Active Object event queues are empty.
// QV disables interrupts before calling — we must re-enable them.
//
// Council A4 resolved (IVP-81): __wfi() active in bridge after tick
// functions. Tick functions run once per idle call, then WFI suspends
// until next interrupt (100Hz QF tick, USB CDC, etc.). Previous
// sleep_ms(1) was a workaround for misdiagnosed OpenOCD interference.
//
// IMPORTANT (Council A2): watchdog_kick_tick() stays here PERMANENTLY.
// It never moves into an AO. This is a system-level invariant — if QV
// scheduling stalls, the watchdog fires.
//
// Bridge function is provided by main.cpp (application-specific).
// Signature: void qv_idle_bridge(void);
extern void qv_idle_bridge(void);

void QV_onIdle(void) {
    QF_INT_ENABLE();
    qv_idle_bridge();
}

// Q_onError: provided by main.cpp (target) or qp_app.c (host tests)

#else // ROCKETCHIP_HOST_TEST

void QF_onStartup(void) { }
void QF_onCleanup(void) { }
void QV_onIdle(void) { }

// Q_onError: provided by qp_app.c for host tests

#endif // ROCKETCHIP_HOST_TEST
