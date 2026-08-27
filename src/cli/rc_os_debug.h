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

void cli_debug_start_eskf_live();

// ESKF live-streaming poll. Called from the main dispatcher when the
// ESKF-live mode is active (entered via `q→e`). Returns true if the
// stream is active and consumed the key; false otherwise.
bool dev_eskf_live_poll();

// R-25-exec step 6 (2026-05-13): dev_station_replay_poll DELETED;
// station replay coverage moves host-side per council amendment #4.

#endif // ROCKETCHIP_CLI_RC_OS_DEBUG_H
