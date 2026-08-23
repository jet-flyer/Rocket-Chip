// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Debug sub-menu (`q`). Reads always available.
// Mutating keys require rc::test_mode_active().
// Usage: docs/FAULT_INJECTION.md.
#ifndef ROCKETCHIP_CLI_RC_OS_DEBUG_H
#define ROCKETCHIP_CLI_RC_OS_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

// Enter the debug sub-menu. Prints the menu banner. Returns true if
// the menu was entered (gating is per mutating command, not at entry).
bool dev_debug_menu_enter();

// Dispatch a single key in the debug sub-menu. Returns true if the
// key was handled.
bool dev_debug_menu_dispatch(int c);

// ESKF live-streaming poll. Called from the main dispatcher when the
// ESKF-live mode is active (entered via `q→e`). Returns true if the
// stream is active and consumed the key; false otherwise.
bool dev_eskf_live_poll();

// R-25-exec step 6 (2026-05-13): dev_station_replay_poll DELETED;
// station replay coverage moves host-side per council amendment #4.

// LED-test submenu routes the next keystroke from the main CLI
// dispatcher here instead of blocking in a handler (LL Entry 32).
// Test-mode-gated.
bool dev_led_test_pending();
void dev_led_test_feed(int c);

#endif // ROCKETCHIP_CLI_RC_OS_DEBUG_H
