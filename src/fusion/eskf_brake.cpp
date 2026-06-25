// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// ESKF Runaway-Restart Brake
//
// Migrated 2026-04-22 from watchdog_recovery. Runtime-only brake that
// disables the filter after `kEskfMaxFailCycles` consecutive divergence
// events in one session. Cleared by eskf_reenable() — CLI-callable
// subsystem reset. Not a safety posture; see docs/USER_GUIDE.md
// "Safety State Model" for the level-3 launch_abort distinction.
//
// File-local statics on purpose — the brake state is session-scoped
// and BSS-zero-initialized (cleared on power cycle).
//============================================================================

#include "fusion/eskf_runner.h"
#include <stdint.h>

namespace {
constexpr uint8_t kEskfMaxFailCycles = 5;  // ArduPilot EKF3 coreSetupRequired pattern
uint8_t g_eskfFailCount = 0;
bool    g_eskfDisabled  = false;
} // namespace

bool eskf_is_disabled() {
    return g_eskfDisabled;
}

void eskf_reenable() {
    g_eskfFailCount = 0;
    g_eskfDisabled = false;
}

void eskf_note_divergence() {
    if (g_eskfFailCount < UINT8_MAX) {
        g_eskfFailCount++;
    }
    if (g_eskfFailCount >= kEskfMaxFailCycles) {
        g_eskfDisabled = true;
    }
}
