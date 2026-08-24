// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Calibration routine manager
// Manages sensor calibration workflows:
// - Gyro bias calibration (device at rest)
// - Accelerometer level calibration (device flat)
// - Barometer ground reference
// - 6-position accel calibration
// Public API is SI units and named thresholds. Part/SKU notes stay in
// the .cpp (WN-156).

#ifndef ROCKETCHIP_CALIBRATION_MANAGER_H
#define ROCKETCHIP_CALIBRATION_MANAGER_H

#include "calibration_data.h"

// ============================================================================
// Calibration State
// ============================================================================

typedef enum {
    CAL_STATE_IDLE = 0,
    CAL_STATE_GYRO_SAMPLING,
    CAL_STATE_ACCEL_LEVEL_SAMPLING,
    CAL_STATE_BARO_SAMPLING,
    CAL_STATE_ACCEL_6POS_SAMPLING,      // Collecting samples for one 6-pos position
    CAL_STATE_COMPLETE,
    CAL_STATE_FAILED,
} cal_state_t;

typedef enum {
    CAL_RESULT_OK = 0,
    CAL_RESULT_BUSY,            // Calibration already in progress
    CAL_RESULT_NO_DATA,         // Not enough samples
    CAL_RESULT_MOTION_DETECTED, // Device moved during calibration
    CAL_RESULT_TIMEOUT,         // Calibration took too long
    CAL_RESULT_INVALID_DATA,    // Data out of expected range
    CAL_RESULT_STORAGE_ERROR,   // Failed to save
    CAL_RESULT_FIT_FAILED,      // Post-fit range/radius check (not LM divergence)
} cal_result_t;

// ============================================================================
// Initialization
// ============================================================================

// Loads existing calibration from storage, or uses defaults.
void calibration_manager_init(void);

const calibration_store_t* calibration_manager_get(void);

cal_state_t calibration_manager_get_state(void);

// ============================================================================
// Gyro Calibration
// ============================================================================

// Device must be stationary. Takes ~2 seconds.
cal_result_t calibration_start_gyro(void);

void calibration_feed_gyro(float gx, float gy, float gz, float temperatureC);

// ============================================================================
// Accelerometer Level Calibration
// ============================================================================

// Device must be flat and stationary. Takes ~1 second.
cal_result_t calibration_start_accel_level(void);

void calibration_feed_accel(float ax, float ay, float az, float temperatureC);

// ============================================================================
// Barometer Calibration
// ============================================================================

// Sets current pressure as ground reference. Takes ~1 second.
cal_result_t calibration_start_baro(void);

void calibration_feed_baro(float pressurePa, float temperatureC);

// ============================================================================
// Calibration Control
// ============================================================================

void calibration_cancel(void);

void calibration_reset_state(void);

bool calibration_is_active(void);

uint8_t calibration_get_progress(void);

cal_result_t calibration_get_result(void);

// ============================================================================
// 6-Position Accelerometer Calibration
// ============================================================================

// Sets state to CAL_STATE_ACCEL_6POS_SAMPLING. Core 1 feeds samples
// via calibration_feed_accel(). Call calibration_6pos_position_done()
// to check completion, then calibration_finalize_6pos_position() to
// compute the average and mark the position collected.
// CAL_RESULT_INVALID_DATA if pos out of range or already collected
cal_result_t calibration_start_6pos_position(uint8_t pos);

bool calibration_6pos_position_done(void);

uint16_t calibration_6pos_position_sample_count(void);

// CAL_RESULT_OK on success, CAL_RESULT_NO_DATA if not enough samples
cal_result_t calibration_finalize_6pos_position(void);

// Requires all 6 positions. Stores offset + diagonal scale; offdiag written 0.
cal_result_t calibration_compute_6pos(void);

void calibration_reset_6pos(void);

// Pointer to float[3] {ax, ay, az} or NULL if invalid
const float* calibration_get_6pos_avg(uint8_t pos);

const char* calibration_get_6pos_name(uint8_t pos);

// ============================================================================
// Magnetometer Calibration
// ============================================================================

enum class mag_feed_result_t : uint8_t {
    ACCEPTED,
    REJECTED_RANGE,
    REJECTED_CLOSE,
    BUFFER_FULL
};

// Checks magnitude range (15-95 µT) and angular separation from recent samples.
// Updates geodesic coverage mask on acceptance.
mag_feed_result_t calibration_feed_mag_sample(float mx, float my, float mz);

void calibration_reset_mag_cal(void);

uint16_t calibration_get_mag_sample_count(void);

uint8_t calibration_get_mag_coverage_pct(void);

// Sphere then 9-param ellipsoid. FIT_FAILED is range/radius, not LM divergence.
cal_result_t calibration_compute_mag_cal(void);

float calibration_get_mag_fitness(void);

// ============================================================================
// Applying Calibration
// ============================================================================

void calibration_apply_gyro(float gxRaw, float gyRaw, float gzRaw,
                            float* gxCal, float* gyCal, float* gzCal);

void calibration_apply_accel(float axRaw, float ayRaw, float azRaw,
                             float* axCal, float* ayCal, float* azCal);

// For cross-core use where Core 1 has its own calibration copy.
void calibration_apply_gyro_with(const calibration_store_t* cal,
                                  const cal_vec3_t& raw, cal_vec3_t& out);

// For cross-core use where Core 1 has its own calibration copy.
void calibration_apply_accel_with(const calibration_store_t* cal,
                                   const cal_vec3_t& raw, cal_vec3_t& out);

void calibration_apply_mag(float mxRaw, float myRaw, float mzRaw,
                            float* mxCal, float* myCal, float* mzCal);

// For cross-core use where Core 1 has its own calibration copy.
void calibration_apply_mag_with(const calibration_store_t* cal,
                                  const cal_vec3_t& raw, cal_vec3_t& out);

// Copies via calibration_storage_read, not the manager get() snapshot.
bool calibration_load_into(calibration_store_t* dest);

float calibration_get_altitude_agl(float pressurePa);

// ============================================================================
// Storage
// ============================================================================

cal_result_t calibration_save(void);

cal_result_t calibration_load(void);

cal_result_t calibration_reset(void);

#endif // ROCKETCHIP_CALIBRATION_MANAGER_H
