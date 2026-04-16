// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_DEV_CLI_H
#define ROCKETCHIP_DEV_CLI_H

#include <stdbool.h>
#include <stdint.h>

#ifdef BUILD_FOR_FLIGHT

static inline bool dev_debug_menu_enter()            { return false; }
static inline bool dev_debug_menu_dispatch(int)       { return false; }
static inline bool dev_eskf_live_poll()               { return false; }
static inline bool dev_replay_poll()                  { return false; }

#else

bool dev_debug_menu_enter();
bool dev_debug_menu_dispatch(int c);
bool dev_eskf_live_poll();
bool dev_replay_poll();

#endif

#endif // ROCKETCHIP_DEV_CLI_H
