/**
 * @file calibration_manager.c
 * @brief Calibration routine manager implementation
 */

#include "calibration_manager.h"
#include "calibration_storage.h"
#include "accel_calibrator.h"
#include "rocketchip/config.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

#define GYRO_CAL_SAMPLES        200     // ~2 seconds at 100Hz
#define GYRO_CAL_MOTION_THRESH  0.10f   // rad/s - motion detection threshold (~6 deg/s)

#define ACCEL_CAL_SAMPLES       100     // ~1 second at 100Hz
#define ACCEL_CAL_MOTION_THRESH 0.5f    // m/s² - motion detection threshold
#define GRAVITY_NOMINAL         9.80665f

#define BARO_CAL_SAMPLES        50      // ~1 second at 50Hz

// ============================================================================
// Private State
// ============================================================================

static calibration_store_t g_calibration;
static cal_state_t g_cal_state = CAL_STATE_IDLE;
static cal_result_t g_cal_result = CAL_RESULT_OK;

// Sampling accumulators
static struct {
    float sum_x, sum_y, sum_z;
    float sum_temp;
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    uint32_t count;
    uint32_t target_count;
} g_sample_acc;

// 6-position accelerometer calibrator
static accel_calibrator_t g_accel_cal;

// ============================================================================
// Private Function Declarations
// ============================================================================

static void feed_accel_6pos(float ax, float ay, float az, float temperature_c);

// ============================================================================
// Private Functions
// ============================================================================

static void reset_accumulator(uint32_t target_samples) {
    memset(&g_sample_acc, 0, sizeof(g_sample_acc));
    g_sample_acc.target_count = target_samples;
    g_sample_acc.min_x = g_sample_acc.min_y = g_sample_acc.min_z = 1e9f;
    g_sample_acc.max_x = g_sample_acc.max_y = g_sample_acc.max_z = -1e9f;
}

static bool check_gyro_motion(void) {
    // Check if gyro readings varied too much during calibration
    float range_x = g_sample_acc.max_x - g_sample_acc.min_x;
    float range_y = g_sample_acc.max_y - g_sample_acc.min_y;
    float range_z = g_sample_acc.max_z - g_sample_acc.min_z;

    return (range_x > GYRO_CAL_MOTION_THRESH ||
            range_y > GYRO_CAL_MOTION_THRESH ||
            range_z > GYRO_CAL_MOTION_THRESH);
}

static bool check_accel_motion(void) {
    // Check if accel readings varied too much during calibration
    float range_x = g_sample_acc.max_x - g_sample_acc.min_x;
    float range_y = g_sample_acc.max_y - g_sample_acc.min_y;
    float range_z = g_sample_acc.max_z - g_sample_acc.min_z;

    return (range_x > ACCEL_CAL_MOTION_THRESH ||
            range_y > ACCEL_CAL_MOTION_THRESH ||
            range_z > ACCEL_CAL_MOTION_THRESH);
}

// ============================================================================
// Initialization
// ============================================================================

void calibration_manager_init(void) {
    // Try to load from storage
    if (calibration_load() != CAL_RESULT_OK) {
        // Initialize with defaults
        calibration_init_defaults(&g_calibration);
    }

    g_cal_state = CAL_STATE_IDLE;
    g_cal_result = CAL_RESULT_OK;
}

const calibration_store_t* calibration_manager_get(void) {
    return &g_calibration;
}

cal_state_t calibration_manager_get_state(void) {
    return g_cal_state;
}

// ============================================================================
// Gyro Calibration
// ============================================================================

cal_result_t calibration_start_gyro(void) {
    if (g_cal_state != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(GYRO_CAL_SAMPLES);
    g_cal_state = CAL_STATE_GYRO_SAMPLING;
    g_cal_result = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_gyro(float gx, float gy, float gz, float temperature_c) {
    if (g_cal_state != CAL_STATE_GYRO_SAMPLING) {
        return;
    }

    // Accumulate samples
    g_sample_acc.sum_x += gx;
    g_sample_acc.sum_y += gy;
    g_sample_acc.sum_z += gz;
    g_sample_acc.sum_temp += temperature_c;

    // Track min/max for motion detection
    if (gx < g_sample_acc.min_x) g_sample_acc.min_x = gx;
    if (gx > g_sample_acc.max_x) g_sample_acc.max_x = gx;
    if (gy < g_sample_acc.min_y) g_sample_acc.min_y = gy;
    if (gy > g_sample_acc.max_y) g_sample_acc.max_y = gy;
    if (gz < g_sample_acc.min_z) g_sample_acc.min_z = gz;
    if (gz > g_sample_acc.max_z) g_sample_acc.max_z = gz;

    g_sample_acc.count++;

    // Check if complete
    if (g_sample_acc.count >= g_sample_acc.target_count) {
        // Check for motion
        if (check_gyro_motion()) {
            g_cal_state = CAL_STATE_FAILED;
            g_cal_result = CAL_RESULT_MOTION_DETECTED;
            return;
        }

        // Compute bias as average
        float n = (float)g_sample_acc.count;
        g_calibration.gyro.bias.x = g_sample_acc.sum_x / n;
        g_calibration.gyro.bias.y = g_sample_acc.sum_y / n;
        g_calibration.gyro.bias.z = g_sample_acc.sum_z / n;
        g_calibration.gyro.temperature_ref = g_sample_acc.sum_temp / n;
        g_calibration.gyro.status = CAL_STATUS_GYRO;

        g_calibration.cal_flags |= CAL_STATUS_GYRO;
        calibration_update_crc(&g_calibration);

        g_cal_state = CAL_STATE_COMPLETE;
        g_cal_result = CAL_RESULT_OK;
    }
}

// ============================================================================
// Accelerometer Level Calibration
// ============================================================================

cal_result_t calibration_start_accel_level(void) {
    if (g_cal_state != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(ACCEL_CAL_SAMPLES);
    g_cal_state = CAL_STATE_ACCEL_LEVEL_SAMPLING;
    g_cal_result = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_accel(float ax, float ay, float az, float temperature_c) {
    // Handle 6-position calibration separately
    if (g_cal_state == CAL_STATE_ACCEL_6POS) {
        feed_accel_6pos(ax, ay, az, temperature_c);
        return;
    }

    if (g_cal_state != CAL_STATE_ACCEL_LEVEL_SAMPLING) {
        return;
    }

    // Accumulate samples
    g_sample_acc.sum_x += ax;
    g_sample_acc.sum_y += ay;
    g_sample_acc.sum_z += az;
    g_sample_acc.sum_temp += temperature_c;

    // Track min/max for motion detection
    if (ax < g_sample_acc.min_x) g_sample_acc.min_x = ax;
    if (ax > g_sample_acc.max_x) g_sample_acc.max_x = ax;
    if (ay < g_sample_acc.min_y) g_sample_acc.min_y = ay;
    if (ay > g_sample_acc.max_y) g_sample_acc.max_y = ay;
    if (az < g_sample_acc.min_z) g_sample_acc.min_z = az;
    if (az > g_sample_acc.max_z) g_sample_acc.max_z = az;

    g_sample_acc.count++;

    // Check if complete
    if (g_sample_acc.count >= g_sample_acc.target_count) {
        // Check for motion
        if (check_accel_motion()) {
            g_cal_state = CAL_STATE_FAILED;
            g_cal_result = CAL_RESULT_MOTION_DETECTED;
            return;
        }

        // Compute average
        float n = (float)g_sample_acc.count;
        float avg_x = g_sample_acc.sum_x / n;
        float avg_y = g_sample_acc.sum_y / n;
        float avg_z = g_sample_acc.sum_z / n;

        // For level cal, we assume device is flat (Z pointing up or down)
        // Expected: X≈0, Y≈0, Z≈±g
        // Offset = expected - measured (so that raw + offset = expected)
        // This matches ArduPilot convention: calibrated = (raw + offset) * scale
        g_calibration.accel.offset.x = -avg_x;  // Expected 0, so offset = 0 - avg_x
        g_calibration.accel.offset.y = -avg_y;  // Expected 0, so offset = 0 - avg_y

        // For Z, we need to preserve gravity magnitude
        // If Z is positive (upright), expected is +g, offset = g - avg_z
        // If Z is negative (inverted), expected is -g, offset = -g - avg_z
        if (avg_z > 0) {
            g_calibration.accel.offset.z = GRAVITY_NOMINAL - avg_z;
        } else {
            g_calibration.accel.offset.z = -GRAVITY_NOMINAL - avg_z;
        }

        g_calibration.accel.temperature_ref = g_sample_acc.sum_temp / n;
        g_calibration.accel.status = CAL_STATUS_LEVEL;

        g_calibration.cal_flags |= CAL_STATUS_LEVEL;
        calibration_update_crc(&g_calibration);

        g_cal_state = CAL_STATE_COMPLETE;
        g_cal_result = CAL_RESULT_OK;
    }
}

// ============================================================================
// 6-Position Accelerometer Calibration
// ============================================================================

cal_result_t calibration_start_accel_6pos(void) {
    DBG_PRINT("[6pos] start: cal_state=%d (IDLE=%d)", g_cal_state, CAL_STATE_IDLE);

    if (g_cal_state != CAL_STATE_IDLE) {
        DBG_PRINT("[6pos] BUSY: g_cal_state=%d", g_cal_state);
        return CAL_RESULT_BUSY;
    }

    accel_cal_init(&g_accel_cal);
    if (!accel_cal_start(&g_accel_cal)) {
        DBG_PRINT("[6pos] accel_cal_start failed: state=%d", g_accel_cal.state);
        return CAL_RESULT_INVALID_DATA;
    }

    g_cal_state = CAL_STATE_ACCEL_6POS;
    g_cal_result = CAL_RESULT_OK;

    DBG_PRINT("[6pos] started successfully");
    return CAL_RESULT_OK;
}

int8_t calibration_get_6pos_position(void) {
    if (g_cal_state != CAL_STATE_ACCEL_6POS) {
        return -1;
    }
    return (int8_t)accel_cal_get_position(&g_accel_cal);
}

const char* calibration_get_6pos_position_name(void) {
    if (g_cal_state != CAL_STATE_ACCEL_6POS) {
        return "N/A";
    }
    return accel_cal_position_name(accel_cal_get_position(&g_accel_cal));
}

bool calibration_accept_6pos_position(void) {
    if (g_cal_state != CAL_STATE_ACCEL_6POS) {
        return false;
    }
    return accel_cal_accept_position(&g_accel_cal);
}

uint8_t calibration_get_6pos_position_progress(void) {
    if (g_cal_state != CAL_STATE_ACCEL_6POS) {
        return 0;
    }
    return accel_cal_get_position_progress(&g_accel_cal);
}

/**
 * @brief Feed accel sample to 6-position calibration
 * @note Called from calibration_feed_accel when in 6-pos mode
 */
static void feed_accel_6pos(float ax, float ay, float az, float temperature_c) {
    accel_cal_feed_sample(&g_accel_cal, ax, ay, az, temperature_c);

    // Check state changes
    accel_cal_state_t state = accel_cal_get_state(&g_accel_cal);

    if (state == ACCEL_CAL_STATE_SUCCESS) {
        // Copy results to calibration store
        cal_vec3_t offset, scale;
        float fitness;
        if (accel_cal_get_result(&g_accel_cal, &offset, &scale, &fitness)) {
            g_calibration.accel.offset = offset;
            g_calibration.accel.scale = scale;
            g_calibration.accel.temperature_ref = g_accel_cal.temperature_ref;
            g_calibration.accel.status = CAL_STATUS_ACCEL_6POS;

            g_calibration.cal_flags |= CAL_STATUS_ACCEL_6POS;
            // Also set level flag since 6-pos is more accurate
            g_calibration.cal_flags |= CAL_STATUS_LEVEL;
            calibration_update_crc(&g_calibration);
        }

        g_cal_state = CAL_STATE_COMPLETE;
        g_cal_result = CAL_RESULT_OK;
    }
    else if (state == ACCEL_CAL_STATE_FAILED) {
        g_cal_state = CAL_STATE_FAILED;

        // Map failure reason to cal_result
        accel_cal_fail_t fail = accel_cal_get_fail_reason(&g_accel_cal);
        switch (fail) {
            case ACCEL_CAL_FAIL_MOTION:
                g_cal_result = CAL_RESULT_MOTION_DETECTED;
                break;
            case ACCEL_CAL_FAIL_SAMPLE_SIMILAR:
            case ACCEL_CAL_FAIL_FIT_FAILED:
            case ACCEL_CAL_FAIL_OFFSET_OOB:
            case ACCEL_CAL_FAIL_SCALE_OOB:
                g_cal_result = CAL_RESULT_INVALID_DATA;
                break;
            case ACCEL_CAL_FAIL_TIMEOUT:
                g_cal_result = CAL_RESULT_TIMEOUT;
                break;
            default:
                g_cal_result = CAL_RESULT_INVALID_DATA;
                break;
        }
    }
}

// ============================================================================
// Barometer Calibration
// ============================================================================

cal_result_t calibration_start_baro(void) {
    if (g_cal_state != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(BARO_CAL_SAMPLES);
    g_cal_state = CAL_STATE_BARO_SAMPLING;
    g_cal_result = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_baro(float pressure_pa, float temperature_c) {
    if (g_cal_state != CAL_STATE_BARO_SAMPLING) {
        return;
    }

    g_sample_acc.sum_x += pressure_pa;  // Reuse sum_x for pressure
    g_sample_acc.sum_temp += temperature_c;
    g_sample_acc.count++;

    if (g_sample_acc.count >= g_sample_acc.target_count) {
        float n = (float)g_sample_acc.count;
        g_calibration.baro.ground_pressure_pa = g_sample_acc.sum_x / n;
        g_calibration.baro.ground_temperature_c = g_sample_acc.sum_temp / n;
        g_calibration.baro.status = CAL_STATUS_BARO;

        g_calibration.cal_flags |= CAL_STATUS_BARO;
        calibration_update_crc(&g_calibration);

        g_cal_state = CAL_STATE_COMPLETE;
        g_cal_result = CAL_RESULT_OK;
    }
}

// ============================================================================
// Calibration Control
// ============================================================================

void calibration_cancel(void) {
    // Cancel 6-pos calibration if active
    if (g_cal_state == CAL_STATE_ACCEL_6POS) {
        accel_cal_cancel(&g_accel_cal);
    }

    g_cal_state = CAL_STATE_IDLE;
    g_cal_result = CAL_RESULT_OK;
}

void calibration_reset_state(void) {
    // Reset state to IDLE after completion/failure has been acknowledged
    if (g_cal_state == CAL_STATE_COMPLETE || g_cal_state == CAL_STATE_FAILED) {
        g_cal_state = CAL_STATE_IDLE;
    }
}

bool calibration_is_active(void) {
    return (g_cal_state != CAL_STATE_IDLE &&
            g_cal_state != CAL_STATE_COMPLETE &&
            g_cal_state != CAL_STATE_FAILED);
}

uint8_t calibration_get_progress(void) {
    if (!calibration_is_active()) {
        return (g_cal_state == CAL_STATE_COMPLETE) ? 100 : 0;
    }

    // Handle 6-position calibration
    if (g_cal_state == CAL_STATE_ACCEL_6POS) {
        return accel_cal_get_progress(&g_accel_cal);
    }

    if (g_sample_acc.target_count == 0) return 0;

    uint32_t progress = (g_sample_acc.count * 100) / g_sample_acc.target_count;
    return (progress > 100) ? 100 : (uint8_t)progress;
}

cal_result_t calibration_get_result(void) {
    return g_cal_result;
}

// ============================================================================
// Applying Calibration
// ============================================================================

void calibration_apply_gyro(float gx_raw, float gy_raw, float gz_raw,
                            float* gx_cal, float* gy_cal, float* gz_cal) {
    *gx_cal = gx_raw - g_calibration.gyro.bias.x;
    *gy_cal = gy_raw - g_calibration.gyro.bias.y;
    *gz_cal = gz_raw - g_calibration.gyro.bias.z;
}

void calibration_apply_accel(float ax_raw, float ay_raw, float az_raw,
                             float* ax_cal, float* ay_cal, float* az_cal) {
    // Apply offset and scale: calibrated = (raw + offset) * scale
    // Matches ArduPilot convention and ellipsoid fitting algorithm
    *ax_cal = (ax_raw + g_calibration.accel.offset.x) * g_calibration.accel.scale.x;
    *ay_cal = (ay_raw + g_calibration.accel.offset.y) * g_calibration.accel.scale.y;
    *az_cal = (az_raw + g_calibration.accel.offset.z) * g_calibration.accel.scale.z;
}

float calibration_get_altitude_agl(float pressure_pa) {
    // Barometric formula: h = 44330 * (1 - (P/P0)^0.1903)
    float p0 = g_calibration.baro.ground_pressure_pa;
    if (p0 < 10000.0f) p0 = 101325.0f;  // Sanity check

    return 44330.0f * (1.0f - powf(pressure_pa / p0, 0.1903f));
}

// ============================================================================
// Storage
// ============================================================================

cal_result_t calibration_save(void) {
    calibration_update_crc(&g_calibration);
    if (!calibration_storage_write(&g_calibration)) {
        return CAL_RESULT_STORAGE_ERROR;
    }
    return CAL_RESULT_OK;
}

cal_result_t calibration_load(void) {
    calibration_store_t loaded;

    if (!calibration_storage_read(&loaded)) {
        return CAL_RESULT_STORAGE_ERROR;
    }

    if (!calibration_validate(&loaded)) {
        return CAL_RESULT_INVALID_DATA;
    }

    g_calibration = loaded;
    return CAL_RESULT_OK;
}

cal_result_t calibration_reset(void) {
    calibration_init_defaults(&g_calibration);
    return calibration_save();
}
