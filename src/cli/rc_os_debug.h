// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// rc_os_debug — operator Debug sub-menu (`q` from main).
//
// R-25-exec step 2 (2026-05-13, per council-APPROVED Approach A in
// docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md): migrated from
// src/dev/dev_cli.h. No longer #ifdef-gated by ROCKETCHIP_INCLUDES_
// DEV_DIAGNOSTICS — lives in the single flight binary always.
//
// State-mutating commands (digit-key radio config, LED test, replay
// trigger) are runtime-gated by rc::test_mode_active() at the entry
// points; diagnostic reads (sensors, HW status, etc.) are always
// available.
#ifndef ROCKETCHIP_CLI_RC_OS_DEBUG_H
#define ROCKETCHIP_CLI_RC_OS_DEBUG_H

#include <stdbool.h>
#include <stdint.h>

// Enter the debug sub-menu. Prints the menu banner. Returns true if
// the menu was entered (always true in Approach A — the menu lives in
// the flight binary; gating is at the per-command level via
// test_mode_active(), not at the menu-entry level).
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
