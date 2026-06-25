// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#include "pio_backup_timer.h"

#ifndef ROCKETCHIP_HOST_TEST

#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "backup_timer.pio.h"

namespace rc {

static constexpr float kSmClockHz = 1000000.0F;  // 1MHz at div=150

static PIO g_pio = nullptr;
static uint32_t g_offset = 0;
static bool g_initialized = false;
static uint32_t g_drogueSm = 0;
static uint32_t g_mainSm = 0;
static uint8_t g_droguePin = 0xFF;
static uint8_t g_mainPin = 0xFF;
static bool g_drogueArmed = false;
static bool g_mainArmed = false;

bool pio_backup_timer_init(uint8_t drogue_pin, uint8_t main_pin) {
    if (g_initialized) {
        return true;
    }

    g_pio = pio2;
    g_droguePin = drogue_pin;
    g_mainPin = main_pin;

    // Claim available SMs (heartbeat WD already took one)
    int sm1 = pio_claim_unused_sm(g_pio, false);
    if (sm1 < 0) { return false; }
    g_drogueSm = static_cast<uint32_t>(sm1);

    int sm2 = pio_claim_unused_sm(g_pio, false);
    if (sm2 < 0) {
        pio_sm_unclaim(g_pio, g_drogueSm);
        return false;
    }
    g_mainSm = static_cast<uint32_t>(sm2);

    // Load timer program once (shared instruction memory)
    if (!pio_can_add_program(g_pio, &backup_timer_program)) {
        pio_sm_unclaim(g_pio, g_drogueSm);
        pio_sm_unclaim(g_pio, g_mainSm);
        return false;
    }
    g_offset = pio_add_program(g_pio, &backup_timer_program);

    // Initialize SMs but do NOT enable — enabled at ARM time
    if (drogue_pin != 0xFF) {
        backup_timer_program_init(g_pio, g_drogueSm, g_offset, drogue_pin);
    }
    if (main_pin != 0xFF) {
        backup_timer_program_init(g_pio, g_mainSm, g_offset, main_pin);
    }

    g_initialized = true;
    return true;
}

void pio_backup_timer_arm(float drogue_timeout_s, float main_timeout_s) {
    if (!g_initialized) {
        return;
    }

    // Restore SM + pin to known-good state before enabling. On re-arm after a
    // prior disarm/cancel, the pin was returned to SIO LOW and the SM may have
    // been left halted mid-instruction. backup_timer_program_init() is
    // idempotent (pio_gpio_init → gpio_set_function PIO; pio_sm_init resets
    // ISR/shift counters/PC to entry point per SDK docs). See LL Entry 42.
    if (drogue_timeout_s > 0.0F && g_droguePin != 0xFF) {
        backup_timer_program_init(g_pio, g_drogueSm, g_offset, g_droguePin);
    }
    if (main_timeout_s > 0.0F && g_mainPin != 0xFF) {
        backup_timer_program_init(g_pio, g_mainSm, g_offset, g_mainPin);
    }

    // Drogue timer
    if (drogue_timeout_s > 0.0F && g_droguePin != 0xFF) {
        uint32_t countdown = static_cast<uint32_t>(drogue_timeout_s * kSmClockHz);
        pio_sm_set_enabled(g_pio, g_drogueSm, true);
        pio_sm_put_blocking(g_pio, g_drogueSm, countdown);
        g_drogueArmed = true;
    }

    // Main timer
    if (main_timeout_s > 0.0F && g_mainPin != 0xFF) {
        uint32_t countdown = static_cast<uint32_t>(main_timeout_s * kSmClockHz);
        pio_sm_set_enabled(g_pio, g_mainSm, true);
        pio_sm_put_blocking(g_pio, g_mainSm, countdown);
        g_mainArmed = true;
    }
}

void pio_backup_timer_cancel(BackupTimerId id) {
    if (!g_initialized) {
        return;
    }

    uint32_t sm = (id == BackupTimerId::kDrogue) ? g_drogueSm : g_mainSm;
    pio_sm_set_enabled(g_pio, sm, false);

    // Ensure pin is LOW after cancel
    uint8_t pin = (id == BackupTimerId::kDrogue) ? g_droguePin : g_mainPin;
    if (pin != 0xFF) {
        gpio_set_function(pin, GPIO_FUNC_SIO);
        gpio_put(pin, 0);
    }

    if (id == BackupTimerId::kDrogue) {
        g_drogueArmed = false;
    } else {
        g_mainArmed = false;
    }
}

void pio_backup_timer_disarm() {
    if (!g_initialized) {
        return;
    }

    // Stop both SMs
    pio_sm_set_enabled(g_pio, g_drogueSm, false);
    pio_sm_set_enabled(g_pio, g_mainSm, false);

    // Return pins to SIO control, drive LOW. Visible safety signal: pin
    // can't be driven HIGH by a stale SM after disarm. arm() restores PIO
    // function on the pin via backup_timer_program_init() on the next ARM.
    if (g_droguePin != 0xFF) {
        gpio_set_function(g_droguePin, GPIO_FUNC_SIO);
        gpio_put(g_droguePin, 0);
    }
    if (g_mainPin != 0xFF) {
        gpio_set_function(g_mainPin, GPIO_FUNC_SIO);
        gpio_put(g_mainPin, 0);
    }

    // PIO program stays loaded for chip lifetime (pairs with init, not disarm).
    // Earlier code called pio_remove_program here with "defense in depth"
    // framing, but the SDK's metadata bitmap then asserts on the *second*
    // disarm (program slots already marked unused). Per pico-examples
    // hello_pio idiomatic pattern: add at claim/init, remove at unclaim/
    // teardown — never at arm/disarm. See LL Entry 42.
    g_drogueArmed = false;
    g_mainArmed = false;
}

bool pio_backup_timer_fired(BackupTimerId id) {
    if (!g_initialized) {
        return false;
    }
    uint8_t pin = (id == BackupTimerId::kDrogue) ? g_droguePin : g_mainPin;
    if (pin == 0xFF) {
        return false;
    }
    return gpio_get(pin);
}

bool pio_backup_timer_armed(BackupTimerId id) {
    if (id == BackupTimerId::kDrogue) {
        return g_drogueArmed;
    }
    return g_mainArmed;
}

}  // namespace rc

#else  // ROCKETCHIP_HOST_TEST

namespace rc {

static bool g_drogue_armed = false;
static bool g_main_armed = false;
static bool g_drogue_fired = false;
static bool g_main_fired = false;

bool pio_backup_timer_init(uint8_t, uint8_t) { return true; }

void pio_backup_timer_arm(float drogue_s, float main_s) {
    if (drogue_s > 0.0f) g_drogue_armed = true;
    if (main_s > 0.0f) g_main_armed = true;
}

void pio_backup_timer_cancel(BackupTimerId id) {
    if (id == BackupTimerId::kDrogue) g_drogue_armed = false;
    else g_main_armed = false;
}

void pio_backup_timer_disarm() {
    g_drogue_armed = false;
    g_main_armed = false;
}

bool pio_backup_timer_fired(BackupTimerId id) {
    return (id == BackupTimerId::kDrogue) ? g_drogue_fired : g_main_fired;
}

bool pio_backup_timer_armed(BackupTimerId id) {
    return (id == BackupTimerId::kDrogue) ? g_drogue_armed : g_main_armed;
}

}  // namespace rc

#endif
