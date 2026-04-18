// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station Idle Tick — scaffolding (Stage 16C IVP-140)
//
// IVP-140 body is intentionally empty. IVP-141 implements the GPS poll by
// reusing existing g_gpsFnUpdate / g_gpsFnGetData function pointers that
// init_sensors() populates for both roles, plus seqlock_write on the
// existing shared_sensor_data_t. No new GPS code path.
//============================================================================
#include "station/station_idle_tick.h"

namespace rc {

void station_idle_tick_init() {
    // No state to initialize yet (IVP-141 adds rate-limit state).
}

void station_idle_tick() {
    // No-op until IVP-141.
}

}  // namespace rc
