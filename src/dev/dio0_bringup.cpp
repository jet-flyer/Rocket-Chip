// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-132a.4a — active DIO0 bring-up test. See dio0_bringup.h.

#ifndef BUILD_FOR_FLIGHT

#include "dev/dio0_bringup.h"
#include "drivers/rfm95w.h"
#include "rocketchip/config.h"
#include "hardware/gpio.h"
#include "pico/time.h"
#include <stdio.h>

// Edge counter incremented from GPIO IRQ callback during the test.
static volatile uint32_t s_edge_count = 0;

static void dio0_irq_cb(uint gpio, uint32_t events) {
    (void)gpio;
    (void)events;
    s_edge_count = s_edge_count + 1;
}

extern "C" uint32_t dio0_bringup_edge_count() {
    return s_edge_count;
}

extern "C" int dio0_bringup_test() {
    const uint8_t cs  = rocketchip::pins::kRadioCs;
    const uint8_t irq = rocketchip::pins::kRadioIrq;

    printf("\n=== DIO0 bring-up test (IVP-132a.4a) ===\n");
    printf("  cs_pin=%u  irq_pin=%u\n", (unsigned)cs, (unsigned)irq);

    // Step 1: confirm SX1276 is physically reachable.
    uint8_t v = rfm95w_read_version(cs);
    printf("  SX1276 RegVersion = 0x%02x (expect 0x12)\n", (unsigned)v);
    if (v != 0x12) {
        printf("  FAIL: SX1276 unreachable\n");
        return 2;
    }

    // Step 2: capture baseline edge count, install IRQ callback.
    s_edge_count = 0;
    gpio_set_irq_enabled_with_callback(
        irq, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, dio0_irq_cb);

    // Simpler implementation: drive a short busy_wait then check edge count.
    // If the radio was left in RxContinuous by normal station init, DIO0
    // may already be actively indicating RX state (asserted when a packet
    // is being received). In pure idle it should sit low.
    //
    // Strategy: just wait 500 ms with IRQ enabled and observe if any edge
    // happened. If not, the line is likely quiet (expected in an RF-quiet
    // room). That's NOT a fail — it's inconclusive. We escalate by
    // forcing a register-level DIO0 output toggle via RegTest (undocumented)
    // — out of scope for a first pass.
    //
    // For this iteration, report observation-only and let the caller decide.
    busy_wait_ms(500);
    uint32_t n1 = s_edge_count;
    printf("  edges in 500ms window: %lu\n", (unsigned long)n1);

    // Step 4: detach callback, do NOT destroy radio state.
    gpio_set_irq_enabled(irq, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    if (n1 == 0) {
        printf("  INCONCLUSIVE: no edges observed. Could mean IRQ-dead\n");
        printf("  or just RF-quiet. Run again with vehicle transmitting\n");
        printf("  to force RxDone edges.\n");
        return 3;
    }

    printf("  PASS: %lu edges observed — DIO0 -> RP2350 wiring live\n",
           (unsigned long)n1);
    return 0;
}

#endif  // !BUILD_FOR_FLIGHT
