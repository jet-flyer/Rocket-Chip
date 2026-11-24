// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// [Radio] block of the T=0 preconditions. See diag_radio_precondition.h.

#include "diag/diag_radio_precondition.h"
#include "drivers/rfm95w.h"
#include "rocketchip/board.h"
#include "rocketchip/rc_log.h"
#include "hardware/gpio.h"
#include <stdint.h>

void diag_log_radio_precondition(bool radio_fitted) {
    rc::rc_log("[Radio]\n");
    if (!radio_fitted) {
        rc::rc_log("  fitted=NO (no radio on this board; RegVersion not read)\n");
        return;
    }

    // Radio direct readback: catches the original Frankenstein bug directly.
    const uint8_t radio_version = rfm95w_read_version(board::kRadioCsPin);
    rc::rc_log("  RegVersion=0x%02x (expect 0x12)\n",
               static_cast<unsigned>(radio_version));

    // Passive IRQ wiring evidence (NASA/JPL)
    const int irq_pin = board::kRadioIrqPin;
    const int irq_pin_state = gpio_get(board::kRadioIrqPin) ? 1 : 0;
    rc::rc_log("  radio_irq_pin=%d state=%d\n", irq_pin, irq_pin_state);
}
