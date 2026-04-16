// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Replay injection: feeds pre-recorded sensor data into the seqlock
// as if Core 1 produced it. Pauses real sensor reads during replay.
// Dev-only (excluded from flight binary via BUILD_FOR_FLIGHT).
#ifndef ROCKETCHIP_DEV_REPLAY_INJECT_H
#define ROCKETCHIP_DEV_REPLAY_INJECT_H

#include <stdint.h>
#include <stdbool.h>

#ifdef BUILD_FOR_FLIGHT

static inline void replay_inject_start() {}
static inline void replay_inject_stop()  {}
static inline bool replay_inject_active() { return false; }
static inline void replay_inject_sample(
    uint32_t, float, float, float,
    float, float, float,
    float, int32_t, int32_t, int32_t, bool) {}

#else

extern "C" {
void replay_inject_start();
void replay_inject_stop();
void replay_inject_sample(
    uint32_t timestamp_us,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float pressure_pa,
    int32_t gps_lat_1e7, int32_t gps_lon_1e7, int32_t gps_alt_mm,
    bool gps_valid);
}
bool replay_inject_active();

#endif

#endif
