/**
 * @file calibration_manager.c
 * @brief Calibration routine manager implementation
 */

#include "calibration_manager.h"
#include "calibration_storage.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

#define GYRO_CAL_SAMPLES        200     // ~2 seconds at 100Hz
#define GYRO_CAL_MOTION_THRESH  0.10f   // rad/s - motion detection (~6 deg/s)

#define ACCEL_CAL_SAMPLES       100     // ~1 second at 100Hz
#define ACCEL_CAL_MOTION_THRESH 1.0f    // m/s² - motion detection (relaxed for hand-held placement)
#define GRAVITY_NOMINAL         9.80665f

#define BARO_CAL_SAMPLES        50      // ~1 second at 50Hz

// 6-position accel calibration (IVP-17)
#define ACCEL_6POS_SAMPLES_PER_POS  50
#define ACCEL_6POS_POSITIONS        6
#define ACCEL_6POS_TOTAL_SAMPLES    (ACCEL_6POS_SAMPLES_PER_POS * ACCEL_6POS_POSITIONS)
#define ACCEL_6POS_MOTION_THRESH    0.5f    // m/s² - tighter than level cal
#define ACCEL_6POS_ORIENT_MIN       7.0f    // m/s² - dominant axis minimum
#define ACCEL_6POS_ORIENT_MAX_OFF   3.0f    // m/s² - off-axis maximum
#define ACCEL_6POS_MAX_OFFSET       5.0f    // m/s² - max offset per axis after fit
#define ACCEL_6POS_MIN_DIAG         0.8f    // Minimum diagonal scale factor
#define ACCEL_6POS_MAX_DIAG         1.2f    // Maximum diagonal scale factor
#define ACCEL_6POS_MAX_ITERATIONS   50      // Gauss-Newton iteration limit
#define ACCEL_6POS_NUM_PARAMS       9       // offset[3] + diag[3] + offdiag[3]

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

// 6-position accel calibration state (LL Entry 1: static allocation for large buffers)
static float g_6pos_samples[ACCEL_6POS_TOTAL_SAMPLES][3];  // 3600 bytes
static float g_6pos_avg[ACCEL_6POS_POSITIONS][3];           // 72 bytes
static uint16_t g_6pos_sample_count;
static uint8_t g_6pos_collected;    // Bitmask of completed positions

// Gauss-Newton working arrays
static float g_jtj[ACCEL_6POS_NUM_PARAMS * ACCEL_6POS_NUM_PARAMS];      // 324 bytes
static float g_jtj_inv[ACCEL_6POS_NUM_PARAMS * ACCEL_6POS_NUM_PARAMS];  // 324 bytes

// Position names matching ArduPilot convention
static const char* const kPositionNames[ACCEL_6POS_POSITIONS] = {
    "LEVEL (+Z up)",
    "INVERTED (-Z up)",
    "NOSE DOWN (+X up)",
    "NOSE UP (-X up)",
    "LEFT SIDE (+Y up)",
    "RIGHT SIDE (-Y up)"
};

// Expected dominant axis and sign for each position [axis_index, sign]
// axis_index: 0=X, 1=Y, 2=Z;  sign: +1 or -1
static const int8_t kExpectedOrientation[ACCEL_6POS_POSITIONS][2] = {
    {2, +1},   // LEVEL: +Z
    {2, -1},   // INVERTED: -Z
    {0, +1},   // NOSE DOWN: +X
    {0, -1},   // NOSE UP: -X
    {1, +1},   // LEFT SIDE: +Y
    {1, -1},   // RIGHT SIDE: -Y
};

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
    float range_x = g_sample_acc.max_x - g_sample_acc.min_x;
    float range_y = g_sample_acc.max_y - g_sample_acc.min_y;
    float range_z = g_sample_acc.max_z - g_sample_acc.min_z;

    return (range_x > GYRO_CAL_MOTION_THRESH ||
            range_y > GYRO_CAL_MOTION_THRESH ||
            range_z > GYRO_CAL_MOTION_THRESH);
}

static bool check_accel_motion(void) {
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
    // Storage init happens in main before USB
    // Here we just load from the already-initialized storage
    if (calibration_load() != CAL_RESULT_OK) {
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

    // Accumulate
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

    // Check completion
    if (g_sample_acc.count >= g_sample_acc.target_count) {
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
    if (g_cal_state != CAL_STATE_ACCEL_LEVEL_SAMPLING) {
        return;
    }

    // Accumulate
    g_sample_acc.sum_x += ax;
    g_sample_acc.sum_y += ay;
    g_sample_acc.sum_z += az;
    g_sample_acc.sum_temp += temperature_c;

    // Track min/max
    if (ax < g_sample_acc.min_x) g_sample_acc.min_x = ax;
    if (ax > g_sample_acc.max_x) g_sample_acc.max_x = ax;
    if (ay < g_sample_acc.min_y) g_sample_acc.min_y = ay;
    if (ay > g_sample_acc.max_y) g_sample_acc.max_y = ay;
    if (az < g_sample_acc.min_z) g_sample_acc.min_z = az;
    if (az > g_sample_acc.max_z) g_sample_acc.max_z = az;

    g_sample_acc.count++;

    // Check completion
    if (g_sample_acc.count >= g_sample_acc.target_count) {
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

        // Level cal: device flat, Z pointing up or down
        // Offset = expected - measured
        g_calibration.accel.offset.x = -avg_x;
        g_calibration.accel.offset.y = -avg_y;

        // Preserve gravity magnitude on Z
        if (avg_z > 0) {
            g_calibration.accel.offset.z = GRAVITY_NOMINAL - avg_z;
        } else {
            g_calibration.accel.offset.z = -GRAVITY_NOMINAL - avg_z;
        }

        // Level cal uses simple offset only — reset scale to unity, clear offdiag
        g_calibration.accel.scale = (cal_vec3_t){1.0f, 1.0f, 1.0f};
        g_calibration.accel.offdiag = (cal_vec3_t){0.0f, 0.0f, 0.0f};

        g_calibration.accel.temperature_ref = g_sample_acc.sum_temp / n;
        g_calibration.accel.status = CAL_STATUS_LEVEL;

        g_calibration.cal_flags |= CAL_STATUS_LEVEL;
        g_calibration.cal_flags &= ~CAL_STATUS_ACCEL_6POS;  // Level cal supersedes 6-pos
        calibration_update_crc(&g_calibration);

        g_cal_state = CAL_STATE_COMPLETE;
        g_cal_result = CAL_RESULT_OK;
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
// 6-Position Accelerometer Calibration (IVP-17)
// ============================================================================

// Gauss-Newton helper: compute residual for one sample
// residual = GRAVITY - |M * (sample + offset)|
// params[0..2] = offset, params[3..5] = diag, params[6..8] = offdiag
static float calc_residual_6pos(const float sample[3], const float params[9]) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];

    float A = params[3] * sx + params[6] * sy + params[7] * sz;
    float B = params[6] * sx + params[4] * sy + params[8] * sz;
    float C = params[7] * sx + params[8] * sy + params[5] * sz;

    float len = sqrtf(A*A + B*B + C*C);
    return GRAVITY_NOMINAL - len;
}

// Gauss-Newton helper: compute Jacobian for one sample (9 partial derivatives)
// Based on ArduPilot AccelCalibrator.cpp Jacobian formulas
static void calc_jacobian_6pos(const float sample[3], const float params[9],
                               float jacob[9]) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];

    float A = params[3] * sx + params[6] * sy + params[7] * sz;
    float B = params[6] * sx + params[4] * sy + params[8] * sz;
    float C = params[7] * sx + params[8] * sy + params[5] * sz;

    float len = sqrtf(A*A + B*B + C*C);
    if (len < 1e-6f) {
        memset(jacob, 0, 9 * sizeof(float));
        return;
    }

    // d(residual)/d(offset[0..2])
    jacob[0] = -((params[3]*A + params[6]*B + params[7]*C) / len);
    jacob[1] = -((params[6]*A + params[4]*B + params[8]*C) / len);
    jacob[2] = -((params[7]*A + params[8]*B + params[5]*C) / len);
    // d(residual)/d(diag[0..2])
    jacob[3] = -(sx * A / len);
    jacob[4] = -(sy * B / len);
    jacob[5] = -(sz * C / len);
    // d(residual)/d(offdiag[0..2]) — XY, XZ, YZ
    jacob[6] = -((sy*A + sx*B) / len);
    jacob[7] = -((sz*A + sx*C) / len);
    jacob[8] = -((sz*B + sy*C) / len);
}

// Mean squared residuals across all collected samples
static float calc_mean_sq_residuals(const float params[9]) {
    float sum = 0.0f;
    for (uint16_t i = 0; i < g_6pos_sample_count; i++) {
        float r = calc_residual_6pos(g_6pos_samples[i], params);
        sum += r * r;
    }
    return sum / (float)g_6pos_sample_count;
}

// 9x9 matrix inverse via Gaussian elimination with partial pivoting
// Returns false if singular
static bool mat_inverse_9x9(const float src[81], float dst[81]) {
    // Augmented matrix [src | I] — work in dst as scratch
    float aug[9][18];

    for (uint8_t r = 0; r < 9; r++) {
        for (uint8_t c = 0; c < 9; c++) {
            aug[r][c] = src[r * 9 + c];
            aug[r][c + 9] = (r == c) ? 1.0f : 0.0f;
        }
    }

    // Forward elimination with partial pivoting
    for (uint8_t col = 0; col < 9; col++) {
        // Find pivot
        uint8_t max_row = col;
        float max_val = fabsf(aug[col][col]);
        for (uint8_t r = col + 1; r < 9; r++) {
            float val = fabsf(aug[r][col]);
            if (val > max_val) {
                max_val = val;
                max_row = r;
            }
        }

        if (max_val < 1e-10f) {
            return false;  // Singular
        }

        // Swap rows
        if (max_row != col) {
            for (uint8_t c = 0; c < 18; c++) {
                float tmp = aug[col][c];
                aug[col][c] = aug[max_row][c];
                aug[max_row][c] = tmp;
            }
        }

        // Eliminate below
        float pivot = aug[col][col];
        for (uint8_t r = col + 1; r < 9; r++) {
            float factor = aug[r][col] / pivot;
            for (uint8_t c = col; c < 18; c++) {
                aug[r][c] -= factor * aug[col][c];
            }
        }
    }

    // Back substitution
    for (int8_t col = 8; col >= 0; col--) {
        float pivot = aug[col][col];
        for (uint8_t c = 0; c < 18; c++) {
            aug[col][c] /= pivot;
        }
        for (int8_t r = col - 1; r >= 0; r--) {
            float factor = aug[r][col];
            for (uint8_t c = 0; c < 18; c++) {
                aug[r][c] -= factor * aug[col][c];
            }
        }
    }

    // Extract inverse from right half
    for (uint8_t r = 0; r < 9; r++) {
        for (uint8_t c = 0; c < 9; c++) {
            dst[r * 9 + c] = aug[r][c + 9];
        }
    }

    return true;
}

void calibration_reset_6pos(void) {
    memset(g_6pos_samples, 0, sizeof(g_6pos_samples));
    memset(g_6pos_avg, 0, sizeof(g_6pos_avg));
    g_6pos_sample_count = 0;
    g_6pos_collected = 0;
}

const char* calibration_get_6pos_name(uint8_t pos) {
    if (pos >= ACCEL_6POS_POSITIONS) return "UNKNOWN";
    return kPositionNames[pos];
}

const float* calibration_get_6pos_avg(uint8_t pos) {
    if (pos >= ACCEL_6POS_POSITIONS) return NULL;
    return g_6pos_avg[pos];
}

cal_result_t calibration_collect_6pos_position(uint8_t pos, accel_read_fn read_fn) {
    if (pos >= ACCEL_6POS_POSITIONS) return CAL_RESULT_INVALID_DATA;
    if (read_fn == NULL) return CAL_RESULT_INVALID_DATA;
    if (g_6pos_collected & (1 << pos)) return CAL_RESULT_INVALID_DATA;  // Already done

    uint16_t base_idx = pos * ACCEL_6POS_SAMPLES_PER_POS;
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float min_v[3] = {1e9f, 1e9f, 1e9f};
    float max_v[3] = {-1e9f, -1e9f, -1e9f};
    float temp_unused;

    for (uint16_t i = 0; i < ACCEL_6POS_SAMPLES_PER_POS; i++) {
        float ax, ay, az;
        if (!read_fn(&ax, &ay, &az, &temp_unused)) {
            return CAL_RESULT_NO_DATA;
        }

        g_6pos_samples[base_idx + i][0] = ax;
        g_6pos_samples[base_idx + i][1] = ay;
        g_6pos_samples[base_idx + i][2] = az;

        sum[0] += ax; sum[1] += ay; sum[2] += az;

        for (uint8_t a = 0; a < 3; a++) {
            float v = (a == 0) ? ax : (a == 1) ? ay : az;
            if (v < min_v[a]) min_v[a] = v;
            if (v > max_v[a]) max_v[a] = v;
        }

        // ~100Hz sampling rate: 10ms between reads
        // Caller (CLI) should add sleep_ms(10) between calls if needed,
        // but the read_fn itself may block for a fresh sample.
    }

    // Motion check
    for (uint8_t a = 0; a < 3; a++) {
        if ((max_v[a] - min_v[a]) > ACCEL_6POS_MOTION_THRESH) {
            return CAL_RESULT_MOTION_DETECTED;
        }
    }

    // Compute average
    float n = (float)ACCEL_6POS_SAMPLES_PER_POS;
    g_6pos_avg[pos][0] = sum[0] / n;
    g_6pos_avg[pos][1] = sum[1] / n;
    g_6pos_avg[pos][2] = sum[2] / n;

    // Orientation confirmation: verify dominant axis matches expected
    int8_t expected_axis = kExpectedOrientation[pos][0];
    int8_t expected_sign = kExpectedOrientation[pos][1];

    // Find dominant axis
    float abs_avg[3] = {
        fabsf(g_6pos_avg[pos][0]),
        fabsf(g_6pos_avg[pos][1]),
        fabsf(g_6pos_avg[pos][2])
    };

    uint8_t dominant = 0;
    if (abs_avg[1] > abs_avg[dominant]) dominant = 1;
    if (abs_avg[2] > abs_avg[dominant]) dominant = 2;

    // Check dominant axis matches expected
    if (dominant != (uint8_t)expected_axis) {
        return CAL_RESULT_INVALID_DATA;
    }

    // Check sign matches
    float dominant_val = g_6pos_avg[pos][dominant];
    if ((expected_sign > 0 && dominant_val < 0) ||
        (expected_sign < 0 && dominant_val > 0)) {
        return CAL_RESULT_INVALID_DATA;
    }

    // Check magnitude: dominant > 7.0, off-axes < 3.0
    if (abs_avg[dominant] < ACCEL_6POS_ORIENT_MIN) {
        return CAL_RESULT_INVALID_DATA;
    }
    for (uint8_t a = 0; a < 3; a++) {
        if (a != dominant && abs_avg[a] > ACCEL_6POS_ORIENT_MAX_OFF) {
            return CAL_RESULT_INVALID_DATA;
        }
    }

    g_6pos_collected |= (1 << pos);
    g_6pos_sample_count = 0;
    for (uint8_t p = 0; p < ACCEL_6POS_POSITIONS; p++) {
        if (g_6pos_collected & (1 << p)) {
            g_6pos_sample_count += ACCEL_6POS_SAMPLES_PER_POS;
        }
    }

    return CAL_RESULT_OK;
}

cal_result_t calibration_compute_6pos(void) {
    // Guard: all 6 positions must be collected
    if (g_6pos_collected != 0x3F) return CAL_RESULT_NO_DATA;

    // Initialize parameters: offset={0,0,0}, diag={1,1,1}, offdiag={0,0,0}
    float params[ACCEL_6POS_NUM_PARAMS] = {
        0.0f, 0.0f, 0.0f,   // offset
        1.0f, 1.0f, 1.0f,   // diag (scale)
        0.0f, 0.0f, 0.0f    // offdiag
    };

    float best_params[ACCEL_6POS_NUM_PARAMS];
    memcpy(best_params, params, sizeof(params));
    float best_fitness = calc_mean_sq_residuals(params);

    float jacob[ACCEL_6POS_NUM_PARAMS];
    float jtfi[ACCEL_6POS_NUM_PARAMS];  // J^T * residuals

    for (uint16_t iter = 0; iter < ACCEL_6POS_MAX_ITERATIONS; iter++) {
        // Clear normal equation accumulators
        memset(g_jtj, 0, sizeof(g_jtj));
        memset(jtfi, 0, sizeof(jtfi));

        // Accumulate J^T*J and J^T*r over all samples
        for (uint16_t i = 0; i < g_6pos_sample_count; i++) {
            float r = calc_residual_6pos(g_6pos_samples[i], params);
            calc_jacobian_6pos(g_6pos_samples[i], params, jacob);

            for (uint8_t row = 0; row < ACCEL_6POS_NUM_PARAMS; row++) {
                jtfi[row] += jacob[row] * r;
                for (uint8_t col = 0; col < ACCEL_6POS_NUM_PARAMS; col++) {
                    g_jtj[row * ACCEL_6POS_NUM_PARAMS + col] += jacob[row] * jacob[col];
                }
            }
        }

        // Invert J^T*J
        if (!mat_inverse_9x9(g_jtj, g_jtj_inv)) {
            // Singular matrix — return best so far if reasonable
            break;
        }

        // Compute parameter update: delta = (J^T*J)^-1 * J^T*r
        // Update: params -= delta
        float new_params[ACCEL_6POS_NUM_PARAMS];
        bool has_nan = false;
        for (uint8_t i = 0; i < ACCEL_6POS_NUM_PARAMS; i++) {
            float delta = 0.0f;
            for (uint8_t j = 0; j < ACCEL_6POS_NUM_PARAMS; j++) {
                delta += g_jtj_inv[i * ACCEL_6POS_NUM_PARAMS + j] * jtfi[j];
            }
            new_params[i] = params[i] - delta;
            if (isnan(new_params[i]) || isinf(new_params[i])) {
                has_nan = true;
            }
        }

        if (has_nan) {
            break;  // Use best params found so far
        }

        memcpy(params, new_params, sizeof(params));

        float fitness = calc_mean_sq_residuals(params);
        if (fitness < best_fitness) {
            best_fitness = fitness;
            memcpy(best_params, params, sizeof(params));
        }
    }

    // Validate result
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsf(best_params[i]) > ACCEL_6POS_MAX_OFFSET) {
            return CAL_RESULT_FIT_FAILED;
        }
    }
    for (uint8_t i = 3; i < 6; i++) {
        if (best_params[i] < ACCEL_6POS_MIN_DIAG ||
            best_params[i] > ACCEL_6POS_MAX_DIAG) {
            return CAL_RESULT_FIT_FAILED;
        }
    }

    // Store results
    g_calibration.accel.offset.x = best_params[0];
    g_calibration.accel.offset.y = best_params[1];
    g_calibration.accel.offset.z = best_params[2];
    g_calibration.accel.scale.x  = best_params[3];
    g_calibration.accel.scale.y  = best_params[4];
    g_calibration.accel.scale.z  = best_params[5];
    g_calibration.accel.offdiag.x = best_params[6];
    g_calibration.accel.offdiag.y = best_params[7];
    g_calibration.accel.offdiag.z = best_params[8];
    g_calibration.accel.status = CAL_STATUS_ACCEL_6POS;

    g_calibration.cal_flags |= CAL_STATUS_ACCEL_6POS;
    g_calibration.cal_flags &= ~CAL_STATUS_LEVEL;  // 6-pos supersedes level cal
    calibration_update_crc(&g_calibration);

    return CAL_RESULT_OK;
}

// ============================================================================
// Calibration Control
// ============================================================================

void calibration_cancel(void) {
    g_cal_state = CAL_STATE_IDLE;
    g_cal_result = CAL_RESULT_OK;
}

void calibration_reset_state(void) {
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
    // Bias removal
    float cx = gx_raw - g_calibration.gyro.bias.x;
    float cy = gy_raw - g_calibration.gyro.bias.y;
    float cz = gz_raw - g_calibration.gyro.bias.z;

    // Board rotation: R * corrected
    const float* R = g_calibration.board_rotation.m;
    *gx_cal = R[0]*cx + R[1]*cy + R[2]*cz;
    *gy_cal = R[3]*cx + R[4]*cy + R[5]*cz;
    *gz_cal = R[6]*cx + R[7]*cy + R[8]*cz;
}

void calibration_apply_accel(float ax_raw, float ay_raw, float az_raw,
                             float* ax_cal, float* ay_cal, float* az_cal) {
    // Stage 1: Ellipsoid correction — M * (raw + offset)
    // M is symmetric 3x3 with scale (diagonal) and offdiag terms
    float ox = ax_raw + g_calibration.accel.offset.x;
    float oy = ay_raw + g_calibration.accel.offset.y;
    float oz = az_raw + g_calibration.accel.offset.z;

    float cx = g_calibration.accel.scale.x   * ox
             + g_calibration.accel.offdiag.x * oy
             + g_calibration.accel.offdiag.y * oz;
    float cy = g_calibration.accel.offdiag.x * ox
             + g_calibration.accel.scale.y   * oy
             + g_calibration.accel.offdiag.z * oz;
    float cz = g_calibration.accel.offdiag.y * ox
             + g_calibration.accel.offdiag.z * oy
             + g_calibration.accel.scale.z   * oz;

    // Stage 2: Board rotation — R * corrected
    const float* R = g_calibration.board_rotation.m;
    *ax_cal = R[0]*cx + R[1]*cy + R[2]*cz;
    *ay_cal = R[3]*cx + R[4]*cy + R[5]*cz;
    *az_cal = R[6]*cx + R[7]*cy + R[8]*cz;
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
