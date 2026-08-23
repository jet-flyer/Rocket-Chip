// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station idle tick — Core 0 GPS poll from qv_idle_bridge (not an AO).
// Blocking GPS I2C is OK here (LL 32 is AO-only).
//============================================================================
#ifndef ROCKETCHIP_STATION_IDLE_TICK_H
#define ROCKETCHIP_STATION_IDLE_TICK_H

namespace rc {

// Called once from init_application() on the station branch.
void station_idle_tick_init();

// Called from qv_idle_bridge() every idle pass. Rate-limited internally.
void station_idle_tick();

}  // namespace rc

#endif  // ROCKETCHIP_STATION_IDLE_TICK_H
