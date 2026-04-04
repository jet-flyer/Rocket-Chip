// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Sensor Seqlock — Cross-Core Data Sharing
//
// Shared sensor data struct and lock-free seqlock protocol for Core 1
// (sensor sampling at ~1kHz) to publish calibrated sensor data to Core 0
// (ESKF fusion at 200Hz, AOs, CLI).
//
// Per SEQLOCK_DESIGN.md (council-approved). Explicit __dmb() required
// because memory_order_release only orders the atomic store itself,
// not the non-atomic memcpy data.
//
// Stage 13 AO Architecture: Phase 0A extraction from main.cpp.
//============================================================================
#ifndef ROCKETCHIP_SENSOR_SEQLOCK_H
#define ROCKETCHIP_SENSOR_SEQLOCK_H

#include <stdint.h>
#include <string.h>
#include <atomic>

#ifndef ROCKETCHIP_HOST_TEST
#include "hardware/sync.h"  // __dmb()
#else
// Host test stub: no barrier needed on x86 (strong memory model)
static inline void __dmb() {}
#endif

// ============================================================================
// Shared Sensor Data (per SEQLOCK_DESIGN.md, council-approved)
// All values calibration-applied, body frame, SI units.
// Written by Core 1, read by Core 0 via seqlock.
// ============================================================================

struct shared_sensor_data_t {
    // IMU (68 bytes: 13 floats + 3 uint32_t + 3 bool + 1 pad)
    float accel_x;                          // m/s^2
    float accel_y;
    float accel_z;
    float gyro_x;                           // rad/s
    float gyro_y;
    float gyro_z;
    float mag_x;                            // uT calibrated (when mag_valid)
    float mag_y;
    float mag_z;
    float mag_raw_x;                        // uT raw (for mag recalibration)
    float mag_raw_y;
    float mag_raw_z;
    float imu_temperature_c;
    uint32_t imu_timestamp_us;
    uint32_t imu_read_count;                // Monotonic
    uint32_t mag_read_count;                // Increments only on new mag data
    bool accel_valid;
    bool gyro_valid;
    bool mag_valid;
    uint8_t _pad_imu;

    // Barometer (20 bytes)
    float pressure_pa;
    float baro_temperature_c;
    uint32_t baro_timestamp_us;
    uint32_t baro_read_count;
    bool baro_valid;
    uint8_t _pad_baro[3];

    // GPS (32 bytes)
    int32_t gps_lat_1e7;
    int32_t gps_lon_1e7;
    float gps_alt_msl_m;
    float gps_ground_speed_mps;
    float gps_course_deg;
    uint32_t gps_timestamp_us;
    uint32_t gps_read_count;
    uint8_t gps_fix_type;
    uint8_t gps_satellites;
    bool gps_valid;
    // Diagnostic: raw lwGPS fields for debugging fix detection
    uint8_t gps_gga_fix;       // GGA fix quality (0=none, 1=GPS, 2=DGPS)
    uint8_t gps_gsa_fix_mode;  // GSA fix mode (1=none, 2=2D, 3=3D)
    bool gps_rmc_valid;        // RMC status ('A')
    uint8_t _pad_gps[2];
    float gps_hdop;            // Horizontal DOP (0 = unknown)
    float gps_vdop;            // Vertical DOP (0 = unknown)

    // Health (16 bytes)
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;              // For watchdog/stall detection
};

static_assert(sizeof(shared_sensor_data_t) == 148, // NOLINT(readability-magic-numbers)
              "Struct size changed - update SEQLOCK_DESIGN.md");
static_assert(sizeof(shared_sensor_data_t) % 4 == 0,
              "Struct must be 4-byte aligned for memcpy");

// ============================================================================
// Seqlock Wrapper
// ============================================================================

struct sensor_seqlock_t {
    std::atomic<uint32_t> sequence{0};      // Odd = write in progress
    shared_sensor_data_t data = {};
};

// ============================================================================
// Seqlock Read/Write API
// ============================================================================

static constexpr uint32_t kSeqlockMaxRetries = 4;

inline void seqlock_write(sensor_seqlock_t* sl, const shared_sensor_data_t* src) {
    uint32_t seq = sl->sequence.load(std::memory_order_relaxed);
    // Signal write-in-progress (odd)
    sl->sequence.store(seq + 1, std::memory_order_release);
    __dmb();  // Ensure odd counter visible before data writes
    memcpy(&sl->data, src, sizeof(shared_sensor_data_t));
    __dmb();  // Ensure all data writes complete before even counter
    sl->sequence.store(seq + 2, std::memory_order_release);
}

inline bool seqlock_read(sensor_seqlock_t* sl, shared_sensor_data_t* dst) {
    for (uint32_t attempt = 0; attempt < kSeqlockMaxRetries; attempt++) {
        uint32_t seq1 = sl->sequence.load(std::memory_order_acquire);
        if ((seq1 & 1U) != 0U) {
            continue;  // Write in progress, retry
        }
        __dmb();  // Ensure counter read committed before data reads
        memcpy(dst, &sl->data, sizeof(shared_sensor_data_t));
        __dmb();  // Ensure all data loads complete before re-reading counter
        uint32_t seq2 = sl->sequence.load(std::memory_order_acquire);
        if (seq1 == seq2) {
            return true;  // Consistent snapshot
        }
    }
    return false;  // All retries collided - caller uses previous data
}

// ============================================================================
// Global Seqlock Instance + Cross-Core Signaling
//
// Defined in main.cpp. Will migrate to owning modules in later phases.
// ============================================================================

extern sensor_seqlock_t g_sensorSeqlock;

// Cross-core signaling (atomic flags - FIFO reserved by multicore_lockout)
extern std::atomic<bool> g_startSensorPhase;
extern std::atomic<bool> g_sensorPhaseDone;
extern std::atomic<bool> g_calReloadPending;
extern std::atomic<bool> g_core1PauseI2C;
extern std::atomic<bool> g_core1I2CPaused;
extern std::atomic<bool> g_core1LockoutReady;

// NeoPixel calibration override (Phase M.5 interim).
// Core 0 (CLI) writes, Core 1 reads. Will be replaced by SIG_LED_OVERRIDE
// events in Phase 5 (LED Engine priority compositor).
extern std::atomic<uint8_t> g_calNeoPixelOverride;

#endif // ROCKETCHIP_SENSOR_SEQLOCK_H
