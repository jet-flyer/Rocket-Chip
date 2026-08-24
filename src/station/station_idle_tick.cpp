// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station GPS poll — same core1_read_gps() / seqlock_write as vehicle.
// ~10 Hz. Not an AO (LL 32 does not apply). GPS I2C ~6 ms is OK here.
//============================================================================
#include "station/station_idle_tick.h"

#include "pico/time.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"
#include "drivers/mcu_temp.h"

namespace rc {

// ~10 Hz outer cadence. Matches vehicle Core 1 effective GPS rate.
static constexpr uint32_t kStationGpsTickIntervalUs = 100'000U;

// Persistent local snapshot of seqlock-shape fields. core1_read_gps()
// populates GPS fields in place; other fields (IMU/baro/health) remain
// zero on station since Core 1 is idle here. Station readers
// (cmd_station_gps_push, dashboard) consume only GPS columns from the
// seqlock snapshot.
static shared_sensor_data_t g_localData;

// core1_read_gps() owns an "inter-poll floor" via lastGpsReadUs; we pass
// a persistent pointer to it so the helper's internal throttle works.
static uint32_t g_lastGpsReadUs = 0;

// Outer rate-limit gate — separate from s_lastGpsReadUs, which the
// helper updates on every call. This gate prevents invoking the helper
// more often than ~10 Hz.
static uint32_t g_lastTickUs = 0;

// MCU temp capture cadence — every 10th GPS tick = ~1 Hz.
// IVP-142a: on-die RP2350 temp sensor, captured on both roles.
static constexpr uint32_t kStationMcuTempDivider = 10;
static uint32_t g_mcuTempCycle = 0;

void station_idle_tick_init() {
    g_lastGpsReadUs = 0;
    g_lastTickUs = 0;
    g_mcuTempCycle = 0;
    // Sentinel so seqlock readers don't see 0.0°C (a real pad temp)
    // before first capture.
    g_localData.mcu_die_temp_c = kMcuTempSentinelC;
}

void station_idle_tick() {
    const uint32_t now_us = time_us_32();
    if ((now_us - g_lastTickUs) < kStationGpsTickIntervalUs) {
        return;
    }
    g_lastTickUs = now_us;

    // GPS poll is optional; MCU-temp still runs without a GPS (CW-B30-02).
    if (g_gpsInitialized.load(std::memory_order_acquire)) {
        core1_read_gps(&g_localData, &g_lastGpsReadUs);
    }

    // MCU temp at ~1 Hz (every 10th outer tick at 10 Hz).
    g_mcuTempCycle++;
    if (g_mcuTempCycle >= kStationMcuTempDivider && mcu_temp_available()) {
        g_mcuTempCycle = 0;
        g_localData.mcu_die_temp_c = mcu_temp_read_c();
        g_localData.mcu_temp_read_count++;
    }

    // Publish to seqlock so Core 0 readers see fresh GPS state.
    // Same-core writer/reader on station — seqlock ordering still
    // enforced via __dmb inside seqlock_write.
    seqlock_write(&g_sensorSeqlock, &g_localData);
}

}  // namespace rc
