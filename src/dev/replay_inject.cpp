// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Replay injection — feeds CSV sensor data into seqlock from Core 0.
// Pauses Core 1 real sensor reads, publishes replay data at caller's rate.

#ifndef BUILD_FOR_FLIGHT

#include "dev/replay_inject.h"
#include "rocketchip/sensor_seqlock.h"
#include "pico/time.h"
#include <stdio.h>
#include <string.h>
#include <atomic>

extern sensor_seqlock_t g_sensorSeqlock;
extern std::atomic<bool> g_core1PauseI2C;
extern std::atomic<bool> g_core1I2CPaused;

static bool s_active = false;
static uint32_t s_sampleCount = 0;

extern "C" __attribute__((used))
void replay_inject_start() {
    g_core1PauseI2C.store(true, std::memory_order_release);
    uint32_t timeout = 500;
    while (!g_core1I2CPaused.load(std::memory_order_acquire) && timeout > 0) {
        sleep_ms(1);
        --timeout;
    }
    s_active = true;
    s_sampleCount = 0;
    printf("[REPLAY] Started — Core 1 paused, injecting from serial\n");
}

extern "C" __attribute__((used))
void replay_inject_stop() {
    s_active = false;
    g_core1PauseI2C.store(false, std::memory_order_release);
    printf("[REPLAY] Stopped — %lu samples injected, Core 1 resumed\n",
           (unsigned long)s_sampleCount);
}

bool replay_inject_active() {
    return s_active;
}

extern "C" __attribute__((used))
void replay_inject_sample(
    uint32_t timestamp_us,
    float ax, float ay, float az,
    float gx, float gy, float gz,
    float pressure_pa,
    int32_t gps_lat_1e7, int32_t gps_lon_1e7, int32_t gps_alt_mm,
    bool gps_valid)
{
    if (!s_active) { return; }

    shared_sensor_data_t data;
    memset(&data, 0, sizeof(data));

    data.accel_x = ax;
    data.accel_y = ay;
    data.accel_z = az;
    data.accel_valid = true;

    data.gyro_x = gx;
    data.gyro_y = gy;
    data.gyro_z = gz;
    data.gyro_valid = true;

    data.imu_timestamp_us = timestamp_us;
    data.imu_read_count = s_sampleCount + 1;

    if (pressure_pa > 0.0f) {
        data.pressure_pa = pressure_pa;
        data.baro_valid = true;
        data.baro_timestamp_us = timestamp_us;
        data.baro_read_count = s_sampleCount + 1;
    }

    if (gps_valid) {
        data.gps_lat_1e7 = gps_lat_1e7;
        data.gps_lon_1e7 = gps_lon_1e7;
        data.gps_alt_msl_m = static_cast<float>(gps_alt_mm) / 1000.0f;
        data.gps_valid = true;
        data.gps_fix_type = 3;
        data.gps_satellites = 8;
        data.gps_timestamp_us = timestamp_us;
        data.gps_read_count = s_sampleCount + 1;
    }

    seqlock_write(&g_sensorSeqlock, &data);
    ++s_sampleCount;
}

#endif // BUILD_FOR_FLIGHT
