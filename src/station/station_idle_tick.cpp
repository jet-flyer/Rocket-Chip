// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Station Idle Tick — GPS poll via reused vehicle infrastructure
//
// Stage 16C IVP-141: wires station GPS polling into qv_idle_bridge() by
<<<<<<< Updated upstream
// reusing the existing vehicle GPS path unchanged. No new GPS code:
//   - g_gpsFnUpdate / g_gpsFnGetData function pointers are populated for
//     both roles at boot (vehicle: init_sensors(); station Fruit Jam:
//     ultra-early in init_early_hw() per the GPS-fix commit 21d4eb1).
//     Whichever transport was bound (UART on Feather/Pico 2, I2C PA1010D
//     on Fruit Jam) is the same transport station uses here.
//   - core1_read_gps() is the shared reader, promoted from file-static
//     so both the vehicle Core 1 loop and this station idle-bridge tick
=======
// reusing the existing vehicle path unchanged. No new GPS code:
//   - g_gpsFnUpdate / g_gpsFnGetData function pointers are populated by
//     init_sensors() at boot for BOTH roles (transport chosen per-board).
//   - core1_read_gps() is the shared reader, promoted from file-static in
//     this IVP so both vehicle Core 1 loop and station Core 0 idle bridge
>>>>>>> Stashed changes
//     call the same body.
//   - seqlock_write on the shared shared_sensor_data_t is the same
//     primitive vehicle uses; same-core writer/reader on station is
//     memory-order correct without additional sync.
//
<<<<<<< Updated upstream
// Rate limit: ~10 Hz at the outer gate (kStationGpsTickIntervalUs).
// Matches vehicle Core 1 kCore1GpsDivider cadence (1 kHz loop / 100 =
// ~10 Hz). Vehicle's own inter-poll floor inside core1_read_gps
// (kGpsMinIntervalUs=2ms) remains as a secondary throttle.
//
// Execution context: qv_idle_bridge() — NOT an AO handler. LL Entry 32's
// no-blocking-in-AO rule does not apply. Worst-case GPS I2C cost on
// Fruit Jam (~6 ms for a full MT3333 buffer read) is safe here; bounded
// by picotool-flash experience and the 5 s watchdog (800x margin).
=======
// Rate limit: ~10 Hz at the outer gate (kStationGpsTickIntervalUs). Matches
// the vehicle Core 1 kCore1GpsDivider cadence (1kHz loop / 100 = ~10 Hz).
//
// Execution context: qv_idle_bridge() — NOT an AO handler. LL Entry 32's
// no-blocking-in-AO rule does not apply. Worst-case ~6 ms I2C on Fruit Jam
// station is safe here (800× watchdog margin per plan).
>>>>>>> Stashed changes
//============================================================================
#include "station/station_idle_tick.h"

#include "pico/time.h"
#include "rocketchip/sensor_seqlock.h"
#include "core1/sensor_core1.h"
<<<<<<< Updated upstream
#include "drivers/mcu_temp.h"

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

// MCU temp capture cadence — every 10th GPS tick = ~1 Hz.
// IVP-142a: on-die RP2350 temp sensor, captured on both roles.
static constexpr uint32_t kStationMcuTempDivider = 10;
static uint32_t s_mcuTempCycle = 0;

void station_idle_tick_init() {
    s_lastGpsReadUs = 0;
    s_lastTickUs = 0;
    s_mcuTempCycle = 0;
    // Sentinel so seqlock readers don't see 0.0°C before first capture.
    s_localData.mcu_die_temp_c = -999.0F;
=======

extern sensor_seqlock_t g_sensorSeqlock;  // defined in main.cpp

namespace rc {

// ~10 Hz station GPS tick cadence. Matches vehicle Core 1 effective rate.
// (Vehicle uses kCore1GpsDivider=100 over 1kHz loop; station rate-limits
// the outer call here since the idle bridge has no implicit timer.)
static constexpr uint32_t kStationGpsTickIntervalUs = 100'000;

// Persistent local snapshot of seqlock-shape fields. core1_read_gps()
// populates GPS fields in-place; other fields remain zero on station
// (no IMU/baro on this role). Station readers (cmd_station_gps_push,
// dashboard) only consume GPS columns from the seqlock snapshot.
static shared_sensor_data_t s_localData;

// Last-poll timestamp passed into core1_read_gps(). Tracked here (not
// inside the helper) to match the vehicle caller shape.
static uint32_t s_lastGpsReadUs = 0;

// Outer rate-limit gate. Separate from lastGpsReadUs because the helper
// updates that on every call; this gate prevents calling the helper more
// often than ~10 Hz.
static uint32_t s_lastTickUs = 0;

void station_idle_tick_init() {
    s_lastGpsReadUs = 0;
    s_lastTickUs = 0;
    // s_localData zero-initialized at program startup (BSS).
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
    // Shared helper: drives g_gpsFnUpdate/g_gpsFnGetData, applies the
    // I2C SDA settling delay when the bound transport is I2C, runs the
    // hold-on-valid pattern for burst-then-silent NMEA cadence, and
    // updates the best-fix diagnostic.
    core1_read_gps(&s_localData, &s_lastGpsReadUs);

    // MCU temp at ~1 Hz (every 10th GPS tick at 10 Hz).
    s_mcuTempCycle++;
    if (s_mcuTempCycle >= kStationMcuTempDivider && mcu_temp_available()) {
        s_mcuTempCycle = 0;
        s_localData.mcu_die_temp_c = mcu_temp_read_c();
        s_localData.mcu_temp_read_count++;
    }

    // Publish to seqlock so Core 0 readers see fresh GPS state.
    // Same-core writer/reader on station — seqlock ordering still
    // enforced via __dmb inside seqlock_write.
=======
    // Shared helper: drives g_gpsFnUpdate/g_gpsFnGetData, applies SDA
    // settling on I2C transport, handles hold-on-valid pattern for
    // burst-then-silent NMEA cadence, updates best-fix diagnostic.
    core1_read_gps(&s_localData, &s_lastGpsReadUs);

    // Publish to seqlock so Core 0 readers (cmd_station_gps_push,
    // dashboard, future AOs) see fresh GPS state. Same-core writer/reader:
    // seqlock ordering still enforced via __dmb in seqlock_write.
>>>>>>> Stashed changes
    seqlock_write(&g_sensorSeqlock, &s_localData);
}

}  // namespace rc
