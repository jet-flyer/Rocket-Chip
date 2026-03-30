// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_SAFETY_PIO_BACKUP_TIMER_H
#define ROCKETCHIP_SAFETY_PIO_BACKUP_TIMER_H

// PIO-based backup deployment timers (IVP-89).
//
// Two autonomous countdown timers (drogue + main) on PIO2.
// Loaded at ARM time, fire pyro GPIOs on expiry.
// Cancelled on successful smart deployment.
// Completely independent of ARM cores — survives crashes.
//
// Timer actions are profile-configurable:
//   0 = disabled (no GPIO drive)
//   1 = fire drogue GPIO
//   2 = fire main GPIO
//
// For bench testing: read GPIO state via gpio_get() or CLI command.

#include <cstdint>

namespace rc {

// Timer IDs
enum class BackupTimerId : uint8_t {
    kDrogue = 0,
    kMain = 1,
};

// Initialize backup timer system on PIO2.
// drogue_pin, main_pin: GPIO pins for pyro channels (0xFF = disabled).
// Returns false if PIO resources unavailable.
bool pio_backup_timer_init(uint8_t drogue_pin, uint8_t main_pin);

// Arm both timers: start countdown.
// drogue_timeout_s, main_timeout_s: countdown in seconds (0 = skip).
// Called on SIG_ARM command.
void pio_backup_timer_arm(float drogue_timeout_s, float main_timeout_s);

// Cancel a specific timer (successful smart deploy).
// Stops the PIO SM. Idempotent — safe to call multiple times.
void pio_backup_timer_cancel(BackupTimerId id);

// Disarm both timers and clear PIO instruction memory.
// Called on SIG_DISARM or SIG_RESET.
void pio_backup_timer_disarm();

// Check if a timer has fired (for diagnostics).
bool pio_backup_timer_fired(BackupTimerId id);

// Check if a timer is armed and counting.
bool pio_backup_timer_armed(BackupTimerId id);

}  // namespace rc

#endif  // ROCKETCHIP_SAFETY_PIO_BACKUP_TIMER_H
