// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// QV BSP (Board Support Package) — RP2350 Pico SDK
//
// IVP-75: Active Object Migration Planning (Stage 8)
//
// Minimal BSP for QP/C QV cooperative scheduler on RP2350. Provides the
// required callbacks that QF and QV call during initialization and idle.
//
// This file is compiled but NOT called in Stage 8 — the superloop remains
// the runtime model. Stage 9 (IVP-76) will wire QV_onIdle into the main
// loop and replace the superloop with QF_run().
//
// QV is the simplest QP/C kernel: cooperative, non-preemptive, single-stack.
// Active Objects run to completion in priority order. QV_onIdle is called
// when no events are pending — we use WFI (Wait For Interrupt) for power
// savings on RP2350.
//
// Reference: QP/C 8.1.3 examples/arm-cm/blinky_nucleo-c031c6/qv/bsp.c
//============================================================================

#include "qp_port.h"

#ifndef ROCKETCHIP_HOST_TEST

#include "pico/stdlib.h"
#include "hardware/sync.h"

//----------------------------------------------------------------------------
// QF callbacks

void QF_onStartup(void) {
    // Called by QF_run() before entering the event loop.
    // Stage 9: configure SysTick or alarm for QF tick source.
    // Stage 8: no-op (superloop handles timing).
}

void QF_onCleanup(void) {
    // Called by QF_run() on exit (never on bare-metal).
}

//----------------------------------------------------------------------------
// QV callbacks

void QV_onIdle(void) {
    // Called by QV scheduler when all event queues are empty.
    // WFI suspends the core until the next interrupt (timer, USB, I2C, etc.)
    // Power savings: ~15-25mA active → ~1-5mA WFI on RP2350.
    __wfi();
}

// Q_onError: provided by main.cpp (target) or qp_app.c (host tests)

#else // ROCKETCHIP_HOST_TEST

void QF_onStartup(void) { }
void QF_onCleanup(void) { }
void QV_onIdle(void) { }

// Q_onError: provided by qp_app.c for host tests

#endif // ROCKETCHIP_HOST_TEST
