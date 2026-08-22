// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Live-updating ANSI terminal dashboard for ground station mode
// Renders a color-coded telemetry dashboard using ANSI escape codes.
// Redraws in-place (cursor-home + clear-to-EOL) — no scrolling.
// Works in any ANSI-capable terminal: PuTTY, Windows Terminal, miniterm,
// macOS Terminal, VSCode Serial Monitor (Terminal mode).
// Stage 12B Phase 1: Firmware ANSI dashboard (no companion app needed).

#ifndef ROCKETCHIP_RC_OS_DASHBOARD_H
#define ROCKETCHIP_RC_OS_DASHBOARD_H

#include "rocketchip/telemetry_state.h"
#include <stdint.h>

struct RadioAoState;  // forward declaration

// Builds the entire frame in a static buffer, then writes it in one call
// to avoid CDC buffer tearing. Uses cursor-home + clear-to-EOL per line.
// Forward-decl — avoids pulling ao_telemetry.h into every dashboard user.
struct RxTelemSnapshot;

void ansi_dashboard_render(const rc::TelemetryState& telem,
                            const RadioAoState* rs,
                            uint32_t met_ms, uint16_t seq, bool valid,
                            const RxTelemSnapshot* rx = nullptr);

void ansi_dashboard_render_waiting(const RadioAoState* rs);

// When paused, ansi_dashboard_render() returns immediately without writing.
void rc_os_dashboard_pause();
void rc_os_dashboard_resume();

#endif // ROCKETCHIP_RC_OS_DASHBOARD_H
