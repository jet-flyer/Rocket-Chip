// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Station pad ANSI surface. Not a cli_engine menu.
// Three keys live in ao_rcos poll_dashboard_keys; console is kMenu after 'x'.
// In-place redraw (cursor-home + clear-to-EOL). No full-screen clear.

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
void ansi_dashboard_pause();
void ansi_dashboard_resume();

void ansi_dashboard_set_tminus_minutes(uint16_t minutes, uint32_t now_ms);
// False when the station has no GPS time yet.
bool ansi_dashboard_set_tminus_zulu(uint8_t h, uint8_t m, uint8_t s,
                                    uint32_t now_ms);
void ansi_dashboard_clear_tminus();
// Vehicle ACK'd this UTC second-of-day. Station GPS retargets the
// countdown. Returns false when this board has no GPS time.
bool ansi_dashboard_retarget_tminus_sod(uint32_t accepted_sod, uint32_t now_ms);
void ansi_dashboard_note_latency(bool valid, int32_t latency_s);
bool ansi_dashboard_latency(int32_t* latency_s);

#endif // ROCKETCHIP_RC_OS_DASHBOARD_H
