// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station Idle Tick — GPS poll via reused vehicle infrastructure
//
// Stage 16C IVP-141: wires station GPS polling into qv_idle_bridge() by
// reusing the existing vehicle GPS path unchanged. No new GPS code:
//   - g_gpsFnUpdate / g_gpsFnGetData function pointers are populated for
//     both roles at boot (vehicle: init_sensors(); station Fruit Jam:
//     ultra-early in init_early_hw() per the GPS-fix commit 21d4eb1).
//     Whichever transport was bound (UART on Feather/Pico 2, I2C PA1010D
//     on Fruit Jam) is the same transport station uses here.
//   - core1_read_gps() is the shared reader, promoted from file-static
//     so both the vehicle Core 1 loop and this station idle-bridge tick
//     call the same body.
//   - seqlock_write on the shared shared_sensor_data_t is the same
//     primitive vehicle uses; same-core writer/reader on station is
//     memory-order correct without additional sync.
//
// Rate limit: ~10 Hz at the outer gate (kStationGpsTickIntervalUs).
// Matches vehicle Core 1 kCore1GpsDivider cadence (1 kHz loop / 100 =
// ~10 Hz). Vehicle's own inter-poll floor inside core1_read_gps
// (kGpsMinIntervalUs=2ms) remains as a secondary throttle.
//
// Execution context: qv_idle_bridge() — NOT an AO handler. LL Entry 32's
// no-blocking-in-AO rule does not apply. Worst-case GPS I2C cost on
// Fruit Jam (~6 ms for a full MT3333 buffer read) is safe here; bounded
// by picotool-flash experience and the 5 s watchdog (800x margin).
//============================================================================
#include "station/station_idle_tick.h"

#include "pico/time.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"

namespace rc {

// ~10 Hz outer cadence. Matches vehicle Core 1 effective GPS rate.
static constexpr uint32_t kStationGpsTickIntervalUs = 100'000U;

// Persistent local snapshot of seqlock-shape fields. core1_read_gps()
// populates GPS fields in place; other fields (IMU/baro/health) remain
// zero on station since Core 1 is idle here. Station readers
// (cmd_station_gps_push, dashboard) consume only GPS columns from the
// seqlock snapshot.
static shared_sensor_data_t s_localData;

// core1_read_gps() owns an "inter-poll floor" via lastGpsReadUs; we pass
// a persistent pointer to it so the helper's internal throttle works.
static uint32_t s_lastGpsReadUs = 0;

// Outer rate-limit gate — separate from s_lastGpsReadUs, which the
// helper updates on every call. This gate prevents invoking the helper
// more often than ~10 Hz.
static uint32_t s_lastTickUs = 0;

void station_idle_tick_init() {
    s_lastGpsReadUs = 0;
    s_lastTickUs = 0;
    // s_localData zero-initialized at program startup (BSS).
}

void station_idle_tick() {
    if (!g_gpsInitialized) {
        return;
    }

    const uint32_t nowUs = time_us_32();
    if ((nowUs - s_lastTickUs) < kStationGpsTickIntervalUs) {
        return;
    }
    s_lastTickUs = nowUs;

    // Shared helper: drives g_gpsFnUpdate/g_gpsFnGetData, applies the
    // I2C SDA settling delay when the bound transport is I2C, runs the
    // hold-on-valid pattern for burst-then-silent NMEA cadence, and
    // updates the best-fix diagnostic.
    core1_read_gps(&s_localData, &s_lastGpsReadUs);

    // Publish to seqlock so Core 0 readers see fresh GPS state.
    // Same-core writer/reader on station — seqlock ordering still
    // enforced via __dmb inside seqlock_write.
    seqlock_write(&g_sensorSeqlock, &s_localData);
}

}  // namespace rc
