/**
 * @file accel_calibrator.c
 * @brief 6-position accelerometer calibrator implementation
 *
 * Implements ArduPilot's ellipsoid-fitting algorithm using Gauss-Newton
 * iteration to find optimal offset and scale parameters.
 */

#include "accel_calibrator.h"
#include "rocketchip/config.h"  // For DBG_PRINT macros
#include "debug/debug_stream.h" // For direct dbg_printf
#include <math.h>
#include <string.h>

// ============================================================================
// Private Constants
// ============================================================================

// Minimum sample distance for accepting new position
// For 90-degree rotation: sqrt(2) * g ≈ 13.86 m/s²
// Use 10 m/s² to allow tolerance for imperfect positioning (~45° minimum)
// The ellipsoid fit will still work with less-than-perfect orientations
static const float kMinSampleDist = 10.0f;

// Validation bounds
static const float kMaxOffset = 5.0f;       // Max offset magnitude (m/s²)
static const float kMinScale = 0.8f;        // Min scale factor
static const float kMaxScale = 1.2f;        // Max scale factor

// Position names - explicit orientations for each face
static const char* kPositionNames[] = {
    "LEVEL: USB port facing you, flat on table",
    "ON LEFT EDGE: tilt left 90deg, USB still toward you",
    "ON RIGHT EDGE: tilt right 90deg, USB still toward you",
    "NOSE DOWN: USB port pointing DOWN",
    "NOSE UP: USB port pointing UP",
    "INVERTED: flip upside down, USB toward you"
};

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Calculate vector length
 */
static float vec_length(float x, float y, float z) {
    return sqrtf(x*x + y*y + z*z);
}

/**
 * @brief Check if new sample is sufficiently different from existing samples
 */
static bool sample_is_distinct(accel_calibrator_t* cal, float x, float y, float z) {
    float mag = vec_length(x, y, z);
    DBG_PRINT("[6pos] New sample: (%.2f, %.2f, %.2f) m/s², mag=%.2f", x, y, z, mag);

    for (uint8_t i = 0; i < cal->positions_completed; i++) {
        float dx = cal->samples[i].x - x;
        float dy = cal->samples[i].y - y;
        float dz = cal->samples[i].z - z;
        float dist = vec_length(dx, dy, dz);

        DBG_PRINT("[6pos]   vs pos %d: dist=%.2f m/s² (need>%.1f)",
               i + 1, dist, kMinSampleDist);

        if (dist < kMinSampleDist) {
            DBG_PRINT("[6pos] FAIL: Too similar to position %d - rotate device more!", i + 1);
            return false;
        }
    }
    DBG_PRINT("[6pos] OK: Position %d accepted", cal->positions_completed + 1);
    return true;
}

/**
 * @brief Check for motion during sampling
 */
static bool motion_detected(accel_calibrator_t* cal) {
    float range_x = cal->sample_max_x - cal->sample_min_x;
    float range_y = cal->sample_max_y - cal->sample_min_y;
    float range_z = cal->sample_max_z - cal->sample_min_z;

    bool motion = (range_x > ACCEL_CAL_MOTION_THRESHOLD ||
                   range_y > ACCEL_CAL_MOTION_THRESHOLD ||
                   range_z > ACCEL_CAL_MOTION_THRESHOLD);

    // Only print if motion detected (diagnostic for failures)
    if (motion) {
        DBG_PRINT("[6pos] Motion detected! ranges (%.3f, %.3f, %.3f) > thresh %.1f",
               range_x, range_y, range_z, ACCEL_CAL_MOTION_THRESHOLD);
    }

    return motion;
}

/**
 * @brief Compute residual for one sample with current parameters
 */
static float compute_residual(const accel_cal_params_t* params,
                              const accel_sample_t* sample) {
    // Apply calibration: calibrated = (raw + offset) * diag
    float cx = (sample->x + params->offset.x) * params->diag.x;
    float cy = (sample->y + params->offset.y) * params->diag.y;
    float cz = (sample->z + params->offset.z) * params->diag.z;

    // Residual = expected magnitude - actual magnitude
    float mag = vec_length(cx, cy, cz);
    return GRAVITY_MSS - mag;
}

/**
 * @brief Compute mean squared residuals (fitness)
 */
static float compute_fitness(accel_calibrator_t* cal) {
    float sum_sq = 0.0f;

    for (uint8_t i = 0; i < cal->positions_completed; i++) {
        float r = compute_residual(&cal->params, &cal->samples[i]);
        sum_sq += r * r;
    }

    return sum_sq / (float)cal->positions_completed;
}

/**
 * @brief Run Gauss-Newton iteration to fit ellipsoid
 *
 * Minimizes sum of squared residuals where:
 * residual_i = g - |M * (sample_i + offset)|
 *
 * Parameters: offset.x, offset.y, offset.z, diag.x, diag.y, diag.z
 */
static bool run_ellipsoid_fit(accel_calibrator_t* cal) {
    // Initialize parameters
    cal->params.offset.x = 0.0f;
    cal->params.offset.y = 0.0f;
    cal->params.offset.z = 0.0f;
    cal->params.diag.x = 1.0f;
    cal->params.diag.y = 1.0f;
    cal->params.diag.z = 1.0f;

    // Debug: print input samples
    DBG_PRINT("[6pos] Ellipsoid fit input samples:");
    for (uint8_t i = 0; i < cal->positions_completed; i++) {
        float mag = vec_length(cal->samples[i].x, cal->samples[i].y, cal->samples[i].z);
        DBG_PRINT("[6pos]   %d: (%.3f, %.3f, %.3f) mag=%.3f",
               i+1, cal->samples[i].x, cal->samples[i].y, cal->samples[i].z, mag);
    }

    float best_fitness = 1e10f;

    for (int iter = 0; iter < ACCEL_CAL_MAX_ITERATIONS; iter++) {
        // 6x6 normal equations: J^T * J * delta = J^T * r
        // J is Nx6 Jacobian, r is Nx1 residuals, N=6 samples

        // Accumulate J^T * J (6x6) and J^T * r (6x1)
        float JTJ[6][6] = {0};
        float JTr[6] = {0};

        for (uint8_t s = 0; s < cal->positions_completed; s++) {
            accel_sample_t* sample = &cal->samples[s];

            // Compute calibrated sample
            float cx = (sample->x + cal->params.offset.x) * cal->params.diag.x;
            float cy = (sample->y + cal->params.offset.y) * cal->params.diag.y;
            float cz = (sample->z + cal->params.offset.z) * cal->params.diag.z;
            float mag = vec_length(cx, cy, cz);

            if (mag < 0.001f) mag = 0.001f;  // Avoid division by zero

            // Residual
            float r = GRAVITY_MSS - mag;

            // Jacobian row: partial derivatives of residual w.r.t. parameters
            // d(residual)/d(param) = d(-mag)/d(param)
            // mag = sqrt(cx^2 + cy^2 + cz^2)
            // d(mag)/d(offset.x) = cx * diag.x / mag
            // d(mag)/d(diag.x) = cx * (sample.x + offset.x) / mag

            float J[6];
            // Offset partials (note: residual = g - mag, so d(r)/d(param) = -d(mag)/d(param))
            J[0] = -cx * cal->params.diag.x / mag;  // d/d(offset.x)
            J[1] = -cy * cal->params.diag.y / mag;  // d/d(offset.y)
            J[2] = -cz * cal->params.diag.z / mag;  // d/d(offset.z)

            // Diagonal (scale) partials
            J[3] = -cx * (sample->x + cal->params.offset.x) / mag;  // d/d(diag.x)
            J[4] = -cy * (sample->y + cal->params.offset.y) / mag;  // d/d(diag.y)
            J[5] = -cz * (sample->z + cal->params.offset.z) / mag;  // d/d(diag.z)

            // Accumulate J^T * J and J^T * r
            for (int i = 0; i < 6; i++) {
                JTr[i] += J[i] * r;
                for (int j = 0; j < 6; j++) {
                    JTJ[i][j] += J[i] * J[j];
                }
            }
        }

        // Solve 6x6 system: JTJ * delta = JTr using Gaussian elimination
        // Create augmented matrix
        float aug[6][7];
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                aug[i][j] = JTJ[i][j];
            }
            aug[i][6] = JTr[i];
        }

        // Forward elimination with partial pivoting
        for (int k = 0; k < 6; k++) {
            // Find pivot
            int max_row = k;
            float max_val = fabsf(aug[k][k]);
            for (int i = k + 1; i < 6; i++) {
                if (fabsf(aug[i][k]) > max_val) {
                    max_val = fabsf(aug[i][k]);
                    max_row = i;
                }
            }

            // Swap rows
            if (max_row != k) {
                for (int j = 0; j < 7; j++) {
                    float tmp = aug[k][j];
                    aug[k][j] = aug[max_row][j];
                    aug[max_row][j] = tmp;
                }
            }

            // Check for singular matrix
            if (fabsf(aug[k][k]) < 1e-10f) {
                return false;  // Fit failed
            }

            // Eliminate column
            for (int i = k + 1; i < 6; i++) {
                float factor = aug[i][k] / aug[k][k];
                for (int j = k; j < 7; j++) {
                    aug[i][j] -= factor * aug[k][j];
                }
            }
        }

        // Back substitution
        float delta[6];
        for (int i = 5; i >= 0; i--) {
            delta[i] = aug[i][6];
            for (int j = i + 1; j < 6; j++) {
                delta[i] -= aug[i][j] * delta[j];
            }
            delta[i] /= aug[i][i];

            // Check for NaN
            if (isnan(delta[i]) || isinf(delta[i])) {
                return false;
            }
        }

        // Apply update
        cal->params.offset.x += delta[0];
        cal->params.offset.y += delta[1];
        cal->params.offset.z += delta[2];
        cal->params.diag.x += delta[3];
        cal->params.diag.y += delta[4];
        cal->params.diag.z += delta[5];

        // Compute new fitness
        float fitness = compute_fitness(cal);

        // Check convergence
        if (fitness < ACCEL_CAL_FITNESS_TOLERANCE) {
            cal->fitness = fitness;
            DBG_PRINT("[6pos] Converged at iter %d, fitness=%.4f", iter, fitness);
            return true;
        }

        if (fitness < best_fitness) {
            best_fitness = fitness;
        }

        // Debug every 10 iterations
        if (iter % 10 == 0) {
            DBG_PRINT("[6pos] Iter %d: fitness=%.4f, off=(%.3f,%.3f,%.3f), sc=(%.3f,%.3f,%.3f)",
                   iter, fitness,
                   cal->params.offset.x, cal->params.offset.y, cal->params.offset.z,
                   cal->params.diag.x, cal->params.diag.y, cal->params.diag.z);
        }

        // Check for divergence
        if (fitness > best_fitness * 10.0f) {
            DBG_PRINT("[6pos] Diverging at iter %d, fitness=%.4f", iter, fitness);
            break;  // Getting worse, stop
        }
    }

    cal->fitness = compute_fitness(cal);
    DBG_PRINT("[6pos] Final fitness=%.4f (threshold=%.4f)", cal->fitness, ACCEL_CAL_FITNESS_TOLERANCE * 5.0f);
    // Accept if fitness is reasonable (5x tolerance allows for sensor noise)
    return cal->fitness < ACCEL_CAL_FITNESS_TOLERANCE * 5.0f;
}

/**
 * @brief Validate calibration results
 */
static bool validate_result(accel_calibrator_t* cal) {
    // Check offset bounds
    float offset_mag = vec_length(cal->params.offset.x,
                                  cal->params.offset.y,
                                  cal->params.offset.z);
    if (offset_mag > kMaxOffset) {
        cal->fail_reason = ACCEL_CAL_FAIL_OFFSET_OOB;
        return false;
    }

    // Check scale bounds
    if (cal->params.diag.x < kMinScale || cal->params.diag.x > kMaxScale ||
        cal->params.diag.y < kMinScale || cal->params.diag.y > kMaxScale ||
        cal->params.diag.z < kMinScale || cal->params.diag.z > kMaxScale) {
        cal->fail_reason = ACCEL_CAL_FAIL_SCALE_OOB;
        return false;
    }

    return true;
}

// ============================================================================
// Public API
// ============================================================================

void accel_cal_init(accel_calibrator_t* cal) {
    memset(cal, 0, sizeof(accel_calibrator_t));
    cal->state = ACCEL_CAL_STATE_IDLE;
    cal->params.diag.x = 1.0f;
    cal->params.diag.y = 1.0f;
    cal->params.diag.z = 1.0f;
}

bool accel_cal_start(accel_calibrator_t* cal) {
    if (cal->state != ACCEL_CAL_STATE_IDLE) {
        return false;
    }

    accel_cal_init(cal);
    cal->state = ACCEL_CAL_STATE_WAITING;
    cal->current_position = ACCEL_CAL_POS_LEVEL;
    cal->positions_completed = 0;
    cal->fail_reason = ACCEL_CAL_FAIL_NONE;

    return true;
}

accel_cal_state_t accel_cal_get_state(accel_calibrator_t* cal) {
    return cal->state;
}

accel_cal_position_t accel_cal_get_position(accel_calibrator_t* cal) {
    return cal->current_position;
}

const char* accel_cal_position_name(accel_cal_position_t pos) {
    if (pos < ACCEL_CAL_POS_COUNT) {
        return kPositionNames[pos];
    }
    return "UNKNOWN";
}

void accel_cal_feed_sample(accel_calibrator_t* cal,
                           float x, float y, float z, float temp_c) {
    if (cal->state != ACCEL_CAL_STATE_COLLECTING) {
        return;
    }

    // Accumulate sample
    cal->sample_sum_x += x;
    cal->sample_sum_y += y;
    cal->sample_sum_z += z;
    cal->temperature_sum += temp_c;

    // Track min/max for motion detection
    if (cal->sample_count == 0) {
        cal->sample_min_x = cal->sample_max_x = x;
        cal->sample_min_y = cal->sample_max_y = y;
        cal->sample_min_z = cal->sample_max_z = z;
    } else {
        if (x < cal->sample_min_x) cal->sample_min_x = x;
        if (x > cal->sample_max_x) cal->sample_max_x = x;
        if (y < cal->sample_min_y) cal->sample_min_y = y;
        if (y > cal->sample_max_y) cal->sample_max_y = y;
        if (z < cal->sample_min_z) cal->sample_min_z = z;
        if (z > cal->sample_max_z) cal->sample_max_z = z;
    }

    cal->sample_count++;

    // Check if we have enough samples for this position
    if (cal->sample_count >= ACCEL_CAL_SAMPLES_PER_POS) {
        // Check for motion
        if (motion_detected(cal)) {
            cal->state = ACCEL_CAL_STATE_FAILED;
            cal->fail_reason = ACCEL_CAL_FAIL_MOTION;
            return;
        }

        // Compute average
        float n = (float)cal->sample_count;
        float avg_x = cal->sample_sum_x / n;
        float avg_y = cal->sample_sum_y / n;
        float avg_z = cal->sample_sum_z / n;

        // Check if sample is sufficiently different from prior samples
        if (!sample_is_distinct(cal, avg_x, avg_y, avg_z)) {
            cal->state = ACCEL_CAL_STATE_FAILED;
            cal->fail_reason = ACCEL_CAL_FAIL_SAMPLE_SIMILAR;
            return;
        }

        // Store sample
        cal->samples[cal->positions_completed].x = avg_x;
        cal->samples[cal->positions_completed].y = avg_y;
        cal->samples[cal->positions_completed].z = avg_z;
        cal->temperature_ref += cal->temperature_sum / n;

        cal->positions_completed++;

        // Check if all positions done
        if (cal->positions_completed >= ACCEL_CAL_NUM_POSITIONS) {
            // Average temperature
            cal->temperature_ref /= ACCEL_CAL_NUM_POSITIONS;

            // Run ellipsoid fit
            cal->state = ACCEL_CAL_STATE_FITTING;
            DBG_PRINT("[6pos] Running ellipsoid fit...");
            if (!run_ellipsoid_fit(cal)) {
                DBG_PRINT("[6pos] FAIL: Ellipsoid fit failed! fitness=%.4f (max=%.4f)",
                       cal->fitness, ACCEL_CAL_FITNESS_TOLERANCE * 5.0f);
                DBG_PRINT("[6pos]   Offset: (%.4f, %.4f, %.4f)",
                       cal->params.offset.x, cal->params.offset.y, cal->params.offset.z);
                DBG_PRINT("[6pos]   Scale:  (%.4f, %.4f, %.4f)",
                       cal->params.diag.x, cal->params.diag.y, cal->params.diag.z);
                cal->state = ACCEL_CAL_STATE_FAILED;
                cal->fail_reason = ACCEL_CAL_FAIL_FIT_FAILED;
                return;
            }

            // Validate result
            if (!validate_result(cal)) {
                DBG_PRINT("[6pos] FAIL: Validation failed! reason=%d", cal->fail_reason);
                DBG_PRINT("[6pos]   Offset: (%.4f, %.4f, %.4f) mag=%.4f (max=%.1f)",
                       cal->params.offset.x, cal->params.offset.y, cal->params.offset.z,
                       vec_length(cal->params.offset.x, cal->params.offset.y, cal->params.offset.z),
                       kMaxOffset);
                DBG_PRINT("[6pos]   Scale:  (%.4f, %.4f, %.4f) (range: %.1f-%.1f)",
                       cal->params.diag.x, cal->params.diag.y, cal->params.diag.z,
                       kMinScale, kMaxScale);
                cal->state = ACCEL_CAL_STATE_FAILED;
                return;
            }

            cal->state = ACCEL_CAL_STATE_SUCCESS;

            // Print final calibration result for verification
            DBG_PRINT("[6pos] SUCCESS! Fitness=%.4f m/s²", cal->fitness);
            DBG_PRINT("[6pos]   Offset: (%.4f, %.4f, %.4f) m/s²",
                   cal->params.offset.x, cal->params.offset.y, cal->params.offset.z);
            DBG_PRINT("[6pos]   Scale:  (%.4f, %.4f, %.4f)",
                   cal->params.diag.x, cal->params.diag.y, cal->params.diag.z);
            DBG_PRINT("[6pos]   Temp ref: %.1f C", cal->temperature_ref);
        } else {
            // Move to next position
            cal->current_position = (accel_cal_position_t)cal->positions_completed;
            cal->state = ACCEL_CAL_STATE_POSITION_DONE;
        }
    }
}

bool accel_cal_accept_position(accel_calibrator_t* cal) {
    if (cal->state != ACCEL_CAL_STATE_WAITING &&
        cal->state != ACCEL_CAL_STATE_POSITION_DONE) {
        return false;
    }

    // Reset sample accumulation
    cal->sample_count = 0;
    cal->sample_sum_x = cal->sample_sum_y = cal->sample_sum_z = 0.0f;
    cal->sample_min_x = cal->sample_min_y = cal->sample_min_z = 1e10f;
    cal->sample_max_x = cal->sample_max_y = cal->sample_max_z = -1e10f;
    cal->temperature_sum = 0.0f;

    cal->state = ACCEL_CAL_STATE_COLLECTING;
    return true;
}

uint8_t accel_cal_get_progress(accel_calibrator_t* cal) {
    if (cal->state == ACCEL_CAL_STATE_SUCCESS) {
        return 100;
    }
    if (cal->state == ACCEL_CAL_STATE_IDLE ||
        cal->state == ACCEL_CAL_STATE_FAILED) {
        return 0;
    }

    // Progress based on completed positions
    return (cal->positions_completed * 100) / ACCEL_CAL_NUM_POSITIONS;
}

uint8_t accel_cal_get_position_progress(accel_calibrator_t* cal) {
    if (cal->state != ACCEL_CAL_STATE_COLLECTING) {
        return 0;
    }
    return (cal->sample_count * 100) / ACCEL_CAL_SAMPLES_PER_POS;
}

accel_cal_fail_t accel_cal_get_fail_reason(accel_calibrator_t* cal) {
    return cal->fail_reason;
}

bool accel_cal_get_result(accel_calibrator_t* cal,
                          cal_vec3_t* offset, cal_vec3_t* scale, float* fitness) {
    if (cal->state != ACCEL_CAL_STATE_SUCCESS) {
        return false;
    }

    if (offset) *offset = cal->params.offset;
    if (scale) *scale = cal->params.diag;
    if (fitness) *fitness = cal->fitness;

    return true;
}

void accel_cal_cancel(accel_calibrator_t* cal) {
    accel_cal_init(cal);
}

void accel_cal_apply(const accel_cal_params_t* params,
                     const cal_vec3_t* raw, cal_vec3_t* calibrated) {
    // calibrated = (raw + offset) * diag
    calibrated->x = (raw->x + params->offset.x) * params->diag.x;
    calibrated->y = (raw->y + params->offset.y) * params->diag.y;
    calibrated->z = (raw->z + params->offset.z) * params->diag.z;
}
