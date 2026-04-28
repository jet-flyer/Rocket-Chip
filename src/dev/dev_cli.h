// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_DEV_CLI_H
#define ROCKETCHIP_DEV_CLI_H

#include <stdbool.h>
#include <stdint.h>

#ifndef ROCKETCHIP_INCLUDES_DEV_DIAGNOSTICS

static inline bool dev_debug_menu_enter()             { return false; }
static inline bool dev_debug_menu_dispatch(int)       { return false; }
static inline bool dev_eskf_live_poll()               { return false; }
static inline bool dev_replay_poll()                  { return false; }
static inline bool dev_station_replay_poll()          { return false; }
static inline bool dev_led_test_pending()             { return false; }
static inline void dev_led_test_feed(int)             { }

#else

bool dev_debug_menu_enter();
bool dev_debug_menu_dispatch(int c);
bool dev_eskf_live_poll();
bool dev_replay_poll();
bool dev_station_replay_poll();
// Stage L: LED-test submenu routes the next keystroke from the main CLI
// dispatcher here instead of blocking in a handler (LL Entry 32).
bool dev_led_test_pending();
void dev_led_test_feed(int c);

#endif

#endif // ROCKETCHIP_DEV_CLI_H
