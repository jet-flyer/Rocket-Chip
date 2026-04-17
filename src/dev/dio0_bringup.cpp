// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-132a.4a — active DIO0 bring-up test. See dio0_bringup.h.

#ifndef BUILD_FOR_FLIGHT

#include "dev/dio0_bringup.h"
#include "drivers/rfm95w.h"
#include "rocketchip/config.h"
#include "hardware/gpio.h"
#include <stdio.h>
#include <stdint.h>

// NOTE: Previously used busy_wait_ms() for the observation window.
// That hangs under GDB `call` because the SDK timer infrastructure
// isn't guaranteed to be serviced while the target is in the special
// call-injected state. Replaced with a bounded spin counter that
// doesn't depend on SDK timer state. See LL Entry 37 (TBD) for full
// GDB-call-safe function guidelines.

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

    // Observation window: bounded spin, no SDK timer dependency.
    // Spin for kSpinLimit iterations, polling the edge counter. If any
    // edge fires during the window, return PASS. This is GDB-call-safe
    // because it doesn't touch the SDK timer subsystem.
    //
    // kSpinLimit tuning: ~10-50 million iterations on RP2350 Cortex-M33
    // at 150 MHz is roughly 0.1-0.5 seconds. Exact timing doesn't matter
    // — we just need enough headroom for at least one vehicle TX cycle
    // (at 5Hz that's 200ms per packet).
    constexpr uint32_t kSpinLimit = 30000000;
    uint32_t spin = 0;
    while (spin < kSpinLimit && s_edge_count == 0) {
        __asm volatile("nop");
        spin++;
    }
    uint32_t n1 = s_edge_count;
    printf("  edges observed: %lu (spin=%lu)\n",
           (unsigned long)n1, (unsigned long)spin);

    // Step 4: detach callback, do NOT destroy radio state.
    gpio_set_irq_enabled(irq, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    if (n1 == 0) {
        printf("  INCONCLUSIVE: no edges observed in spin window.\n");
        printf("  Could mean IRQ-dead or just RF-quiet. Run with vehicle\n");
        printf("  transmitting to force RxDone edges.\n");
        return 3;
    }

    printf("  PASS: %lu edges observed -- DIO0 -> RP2350 wiring live\n",
           (unsigned long)n1);
    return 0;
}

#endif  // !BUILD_FOR_FLIGHT
