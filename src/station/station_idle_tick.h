// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station Idle Tick — Core 0 periodic work for station role (Stage 16C IVP-140)
//
// Runs from qv_idle_bridge() under `if constexpr (kRadioModeRx)`. NOT an AO
// handler — LL Entry 32's no-blocking-in-AO rule does not apply. Safe to
// call ~6ms-worst-case GPS I2C here.
//
// IVP-140: scaffolding only. Tick body is a no-op. GPS poll lands in IVP-141.
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
