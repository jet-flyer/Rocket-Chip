/**
 * @file calibration_manager.cpp
 * @brief Calibration routine manager implementation
 */

#include "calibration_manager.h"
#include "calibration_storage.h"
#include "pico/rand.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint16_t kGyroCalSamples        = 200;      // ~2 seconds at 100Hz
constexpr float    kGyroCalMotionThresh   = 0.10F;    // rad/s - motion detection (~6 deg/s)

constexpr uint16_t kAccelCalSamples       = 100;      // ~1 second at 100Hz
constexpr float    kAccelCalMotionThresh  = 1.0F;     // m/s² - motion detection (relaxed for hand-held placement)
constexpr float    kGravityNominal        = 9.80665F;

constexpr uint16_t kBaroCalSamples        = 50;       // ~1 second at 50Hz

// 6-position accel calibration (IVP-17)
constexpr uint16_t kAccel6posSamplesPerPos = 50;
constexpr uint8_t  kAccel6posPositions     = 6;
constexpr uint16_t kAccel6posTotalSamples  = kAccel6posSamplesPerPos * kAccel6posPositions;
constexpr float    kAccel6posMaxOffset     = 5.0F;    // m/s² - max offset per axis after fit
constexpr float    kAccel6posMinDiag       = 0.8F;    // Minimum diagonal scale factor
constexpr float    kAccel6posMaxDiag       = 1.2F;    // Maximum diagonal scale factor
constexpr uint8_t  kAccel6posMaxIterations = 50;      // Gauss-Newton iteration limit
// 6 parameters: offset[3] + diag[3]. Offdiag dropped because 6 axis-aligned
// positions provide only 6 independent constraints — the 3 offdiag parameters
// sit in the Jacobian null space (underdetermined). A full 9-param model is valid
// with richer data (e.g., 24+ arbitrary orientations), but from 6 cardinal
// positions the offdiag values are arbitrary. ArduPilot's AccelCalibrator solves
// 9 params but discards offdiag at runtime for the same reason.
constexpr uint8_t  kAccel6posNumParams     = 6;       // offset[3] + diag[3]
constexpr uint8_t  kAccel6posOffsetEnd     = 3;       // params[0..2] = offset
constexpr uint8_t  kAccel6posDiagEnd       = 6;       // params[3..5] = diagonal scale
constexpr uint8_t  kParamDiagZ             = 5;       // Index of diagonal Z scale in params[]
constexpr uint8_t  kAccel6posAllMask       = 0x3F;    // Bitmask: all 6 positions collected

// Magnetometer calibration (IVP-35/36)
// Field magnitude bounds (µT) — Earth's field ranges ~25-65 µT, allow margin for distortion
constexpr float    kMagMinFieldUt         = 15.0F;    // Reject samples below this magnitude
constexpr float    kMagMaxFieldUt         = 95.0F;    // Reject samples above this magnitude
constexpr uint16_t kMagMaxSamples         = 300;      // Static buffer capacity
constexpr uint16_t kMagMinSamplesForFit   = 50;       // Minimum for reliable fit
constexpr uint8_t  kMagRecentWindow       = 20;       // Angular separation check window
constexpr uint8_t  kMagLatBands           = 10;       // Coverage grid: latitude divisions
constexpr uint8_t  kMagLonSectors         = 8;        // Coverage grid: longitude divisions
constexpr uint8_t  kMagCoverageSections   = kMagLatBands * kMagLonSectors;  // 80 total
constexpr uint8_t  kMagCoverageBytes      = (kMagCoverageSections + 7) / 8; // 10 bytes for bitmask

// Sphere fit (Step 1) — 4 parameters: radius, offset_x, offset_y, offset_z
constexpr uint8_t  kMagSphereParams       = 4;
constexpr uint8_t  kMagSphereIterations   = 10;

// Ellipsoid fit (Step 2) — 9 parameters: offset[3] + diag[3] + offdiag[3]
constexpr uint8_t  kMagEllipsoidParams    = 9;
constexpr uint8_t  kMagEllipsoidIterations = 20;

// LM damping — ArduPilot + council consensus: start at 1.0
constexpr float    kMagLmLambdaInit       = 1.0F;
constexpr float    kMagLmLambdaUp         = 10.0F;    // Multiply on worse fit
constexpr float    kMagLmLambdaDown       = 0.1F;     // Multiply on better fit
constexpr uint8_t  kLmMaxParams           = 9;        // Largest param vector (ellipsoid)

// Post-fit validation bounds (from IVP.md)
constexpr float    kMagMaxOffset          = 85.0F;    // µT per axis
constexpr float    kMagMinDiag            = 0.2F;     // Minimum diagonal scale
constexpr float    kMagMaxDiag            = 5.0F;     // Maximum diagonal scale
constexpr float    kMagMaxOffdiag         = 1.0F;     // Maximum |offdiag| per element
constexpr float    kMagMaxFitness         = 5.0F;     // RMS residual threshold (µT)

// Gauss-Newton solver tolerances
constexpr float    kMinVectorLength        = 1e-6F;   // Below this, treat as zero-length
constexpr float    kSingularityThreshold   = 1e-10F;  // Matrix pivot below this = singular
constexpr float    kMinMaxInitializer      = 1e9F;    // Initial min/max accumulator bound

// Atmospheric constants (barometric formula)
constexpr float    kMinValidPressurePa     = 10000.0F;   // Sanity floor for ground pressure
constexpr float    kStdAtmPressurePa       = 101325.0F;  // Standard sea-level pressure (Pa)
constexpr float    kHypsometricScale       = 44330.0F;    // Barometric formula coefficient
constexpr float    kHypsometricExponent    = 0.1903F;     // Barometric formula exponent (1/5.255)

// ============================================================================
// Private State
// ============================================================================

static calibration_store_t g_calibration;
static cal_state_t g_calState = CAL_STATE_IDLE;
static cal_result_t g_calResult = CAL_RESULT_OK;

// Sampling accumulators
static struct {
    float sum_x, sum_y, sum_z;
    float sum_temp;
    float min_x, max_x;
    float min_y, max_y;
    float min_z, max_z;
    uint32_t count;
    uint32_t target_count;
} g_sampleAcc;

// 6-position accel calibration state (LL Entry 1: static allocation for large buffers)
static float g_6posSamples[kAccel6posTotalSamples][3];  // 3600 bytes
static float g_6posAvg[kAccel6posPositions][3];           // 72 bytes
static uint16_t g_6posSampleCount;
static uint8_t g_6posCollected;    // Bitmask of completed positions

// Gauss-Newton working arrays (shared between 6-pos accel and mag ellipsoid — never concurrent)
// Sized for largest user: mag ellipsoid (9x9). Accel 6-pos uses 6x6 subset.
static float g_jtj[kMagEllipsoidParams * kMagEllipsoidParams];      // 324 bytes
static float g_jtjInv[kMagEllipsoidParams * kMagEllipsoidParams];  // 324 bytes

// Mag calibration state (LL Entry 1: static allocation for large buffers)
static float g_magSamples[kMagMaxSamples][3];         // 3600 bytes
static uint16_t g_magSampleCount;
static uint8_t g_magCoverageMask[kMagCoverageBytes]; // 10 bytes — 80-section bitmask
static float g_magExpectedRadius;                     // Set by sphere fit
static float g_magFitness;                             // RMS residual after fit

// Position names — QGroundControl order (easiest first, inverted last)
static const char* const kPositionNames[kAccel6posPositions] = {
    "LEVEL (+Z up)",
    "LEFT SIDE (+Y up)",
    "RIGHT SIDE (-Y up)",
    "NOSE DOWN (+X up)",
    "NOSE UP (-X up)",
    "INVERTED (-Z up)"
};


// ============================================================================
// Private Functions
// ============================================================================

static void reset_accumulator(uint32_t targetSamples) {
    memset(&g_sampleAcc, 0, sizeof(g_sampleAcc));
    g_sampleAcc.target_count = targetSamples;
    g_sampleAcc.min_x = g_sampleAcc.min_y = g_sampleAcc.min_z = kMinMaxInitializer;
    g_sampleAcc.max_x = g_sampleAcc.max_y = g_sampleAcc.max_z = -kMinMaxInitializer;
}

static bool check_gyro_motion() {
    float rangeX = g_sampleAcc.max_x - g_sampleAcc.min_x;
    float rangeY = g_sampleAcc.max_y - g_sampleAcc.min_y;
    float rangeZ = g_sampleAcc.max_z - g_sampleAcc.min_z;

    return (rangeX > kGyroCalMotionThresh ||
            rangeY > kGyroCalMotionThresh ||
            rangeZ > kGyroCalMotionThresh);
}

static bool check_accel_motion() {
    float rangeX = g_sampleAcc.max_x - g_sampleAcc.min_x;
    float rangeY = g_sampleAcc.max_y - g_sampleAcc.min_y;
    float rangeZ = g_sampleAcc.max_z - g_sampleAcc.min_z;

    return (rangeX > kAccelCalMotionThresh ||
            rangeY > kAccelCalMotionThresh ||
            rangeZ > kAccelCalMotionThresh);
}

// ============================================================================
// Initialization
// ============================================================================

void calibration_manager_init() {
    // Storage init happens in main before USB
    // Here we just load from the already-initialized storage
    if (calibration_load() != CAL_RESULT_OK) {
        calibration_init_defaults(&g_calibration);
    }

    g_calState = CAL_STATE_IDLE;
    g_calResult = CAL_RESULT_OK;
}

const calibration_store_t* calibration_manager_get() {
    return &g_calibration;
}

cal_state_t calibration_manager_get_state() {
    return g_calState;
}

// ============================================================================
// Gyro Calibration
// ============================================================================

cal_result_t calibration_start_gyro() {
    if (g_calState != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(kGyroCalSamples);
    g_calState = CAL_STATE_GYRO_SAMPLING;
    g_calResult = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_gyro(float gx, float gy, float gz, float temperatureC) {
    if (g_calState != CAL_STATE_GYRO_SAMPLING) {
        return;
    }

    // Accumulate
    g_sampleAcc.sum_x += gx;
    g_sampleAcc.sum_y += gy;
    g_sampleAcc.sum_z += gz;
    g_sampleAcc.sum_temp += temperatureC;

    // Track min/max for motion detection
    if (gx < g_sampleAcc.min_x) {
        g_sampleAcc.min_x = gx;
    }
    if (gx > g_sampleAcc.max_x) {
        g_sampleAcc.max_x = gx;
    }
    if (gy < g_sampleAcc.min_y) {
        g_sampleAcc.min_y = gy;
    }
    if (gy > g_sampleAcc.max_y) {
        g_sampleAcc.max_y = gy;
    }
    if (gz < g_sampleAcc.min_z) {
        g_sampleAcc.min_z = gz;
    }
    if (gz > g_sampleAcc.max_z) {
        g_sampleAcc.max_z = gz;
    }

    g_sampleAcc.count++;

    // Check completion
    if (g_sampleAcc.count >= g_sampleAcc.target_count) {
        if (check_gyro_motion()) {
            g_calState = CAL_STATE_FAILED;
            g_calResult = CAL_RESULT_MOTION_DETECTED;
            return;
        }

        // Compute bias as average
        float n = static_cast<float>(g_sampleAcc.count);
        g_calibration.gyro.bias.x = g_sampleAcc.sum_x / n;
        g_calibration.gyro.bias.y = g_sampleAcc.sum_y / n;
        g_calibration.gyro.bias.z = g_sampleAcc.sum_z / n;
        g_calibration.gyro.temperature_ref = g_sampleAcc.sum_temp / n;
        g_calibration.gyro.status = CAL_STATUS_GYRO;

        g_calibration.cal_flags |= CAL_STATUS_GYRO;
        calibration_update_crc(&g_calibration);

        g_calState = CAL_STATE_COMPLETE;
        g_calResult = CAL_RESULT_OK;
    }
}

// ============================================================================
// Accelerometer Level Calibration
// ============================================================================

// Compute level offset from accumulated samples and store in g_calibration
static void compute_level_offset() {
    float n = static_cast<float>(g_sampleAcc.count);
    float avgX = g_sampleAcc.sum_x / n;
    float avgY = g_sampleAcc.sum_y / n;
    float avgZ = g_sampleAcc.sum_z / n;

    // Level cal: device flat, Z pointing up or down
    // Offset = expected - measured
    g_calibration.accel.offset.x = -avgX;
    g_calibration.accel.offset.y = -avgY;

    // Preserve gravity magnitude on Z
    if (avgZ > 0) {
        g_calibration.accel.offset.z = kGravityNominal - avgZ;
    } else {
        g_calibration.accel.offset.z = -kGravityNominal - avgZ;
    }

    // Level cal uses simple offset only — reset scale to unity, clear offdiag
    g_calibration.accel.scale = cal_vec3_t{1.0F, 1.0F, 1.0F};
    g_calibration.accel.offdiag = cal_vec3_t{0.0F, 0.0F, 0.0F};

    g_calibration.accel.temperature_ref = g_sampleAcc.sum_temp / n;
    g_calibration.accel.status = CAL_STATUS_LEVEL;

    g_calibration.cal_flags |= CAL_STATUS_LEVEL;
    g_calibration.cal_flags &= ~CAL_STATUS_ACCEL_6POS;  // Level cal supersedes 6-pos
    calibration_update_crc(&g_calibration);
}

cal_result_t calibration_start_accel_level() {
    if (g_calState != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(kAccelCalSamples);
    g_calState = CAL_STATE_ACCEL_LEVEL_SAMPLING;
    g_calResult = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_accel(float ax, float ay, float az, float temperatureC) {
    if (g_calState != CAL_STATE_ACCEL_LEVEL_SAMPLING) {
        return;
    }

    // Accumulate
    g_sampleAcc.sum_x += ax;
    g_sampleAcc.sum_y += ay;
    g_sampleAcc.sum_z += az;
    g_sampleAcc.sum_temp += temperatureC;

    // Track min/max
    if (ax < g_sampleAcc.min_x) {
        g_sampleAcc.min_x = ax;
    }
    if (ax > g_sampleAcc.max_x) {
        g_sampleAcc.max_x = ax;
    }
    if (ay < g_sampleAcc.min_y) {
        g_sampleAcc.min_y = ay;
    }
    if (ay > g_sampleAcc.max_y) {
        g_sampleAcc.max_y = ay;
    }
    if (az < g_sampleAcc.min_z) {
        g_sampleAcc.min_z = az;
    }
    if (az > g_sampleAcc.max_z) {
        g_sampleAcc.max_z = az;
    }

    g_sampleAcc.count++;

    // Check completion
    if (g_sampleAcc.count >= g_sampleAcc.target_count) {
        if (check_accel_motion()) {
            g_calState = CAL_STATE_FAILED;
            g_calResult = CAL_RESULT_MOTION_DETECTED;
            return;
        }

        compute_level_offset();

        g_calState = CAL_STATE_COMPLETE;
        g_calResult = CAL_RESULT_OK;
    }
}

// ============================================================================
// Barometer Calibration
// ============================================================================

cal_result_t calibration_start_baro() {
    if (g_calState != CAL_STATE_IDLE) {
        return CAL_RESULT_BUSY;
    }

    reset_accumulator(kBaroCalSamples);
    g_calState = CAL_STATE_BARO_SAMPLING;
    g_calResult = CAL_RESULT_OK;

    return CAL_RESULT_OK;
}

void calibration_feed_baro(float pressurePa, float temperatureC) {
    if (g_calState != CAL_STATE_BARO_SAMPLING) {
        return;
    }

    g_sampleAcc.sum_x += pressurePa;  // Reuse sum_x for pressure
    g_sampleAcc.sum_temp += temperatureC;
    g_sampleAcc.count++;

    if (g_sampleAcc.count >= g_sampleAcc.target_count) {
        float n = static_cast<float>(g_sampleAcc.count);
        g_calibration.baro.ground_pressure_pa = g_sampleAcc.sum_x / n;
        g_calibration.baro.ground_temperature_c = g_sampleAcc.sum_temp / n;
        g_calibration.baro.status = CAL_STATUS_BARO;

        g_calibration.cal_flags |= CAL_STATUS_BARO;
        calibration_update_crc(&g_calibration);

        g_calState = CAL_STATE_COMPLETE;
        g_calResult = CAL_RESULT_OK;
    }
}

// ============================================================================
// 6-Position Accelerometer Calibration (IVP-17)
// ============================================================================

// Gauss-Newton helper: compute residual for one sample
// residual = GRAVITY - |diag * (sample + offset)|
// params[0..2] = offset, params[3..5] = diagonal scale
static float calc_residual_6pos(const float sample[3], const float params[kAccel6posNumParams]) {
    float a = params[3] * (sample[0] + params[0]);
    float b = params[4] * (sample[1] + params[1]);
    float c = params[kParamDiagZ] * (sample[2] + params[2]);

    float len = sqrtf(a*a + b*b + c*c);
    return kGravityNominal - len;
}

// Gauss-Newton helper: compute Jacobian for one sample (6 partial derivatives)
// params[0..2] = offset, params[3..5] = diagonal scale
static void calc_jacobian_6pos(const float sample[3], const float params[kAccel6posNumParams],
                               float jacob[kAccel6posNumParams]) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];

    float a = params[3] * sx;
    float b = params[4] * sy;
    float c = params[kParamDiagZ] * sz;

    float len = sqrtf(a*a + b*b + c*c);
    if (len < kMinVectorLength) {
        memset(jacob, 0, kAccel6posNumParams * sizeof(float));
        return;
    }

    // d(residual)/d(offset[0..2])
    jacob[0] = -(params[3] * a / len);
    jacob[1] = -(params[4] * b / len);
    jacob[2] = -(params[kParamDiagZ] * c / len);
    // d(residual)/d(diag[0..2])
    jacob[3] = -(sx * a / len);
    jacob[4] = -(sy * b / len);
    jacob[kParamDiagZ] = -(sz * c / len);
}

// Mean squared residuals across all collected samples
static float calc_mean_sq_residuals(const float params[kAccel6posNumParams]) {
    float sum = 0.0F;
    for (uint16_t i = 0; i < g_6posSampleCount; i++) {
        float r = calc_residual_6pos(g_6posSamples[i], params);
        sum += r * r;
    }
    return sum / static_cast<float>(g_6posSampleCount);
}

// Forward elimination with partial pivoting on flat augmented matrix [A|I].
// aug: row-major flat array of n rows × augWidth columns
// Returns false if matrix is singular.
static bool forward_eliminate(float* aug, uint8_t n, uint8_t augWidth) {
    for (uint8_t col = 0; col < n; col++) {
        // Find pivot
        uint8_t maxRow = col;
        float maxVal = fabsf(aug[col * augWidth + col]);
        for (uint8_t r = col + 1; r < n; r++) {
            float val = fabsf(aug[r * augWidth + col]);
            if (val > maxVal) {
                maxVal = val;
                maxRow = r;
            }
        }

        if (maxVal < kSingularityThreshold) {
            return false;  // Singular
        }

        // Swap rows
        if (maxRow != col) {
            for (uint8_t c = 0; c < augWidth; c++) {
                float tmp = aug[col * augWidth + c];
                aug[col * augWidth + c] = aug[maxRow * augWidth + c];
                aug[maxRow * augWidth + c] = tmp;
            }
        }

        // Eliminate below
        float pivot = aug[col * augWidth + col];
        for (uint8_t r = col + 1; r < n; r++) {
            float factor = aug[r * augWidth + col] / pivot;
            for (uint8_t c = col; c < augWidth; c++) {
                aug[r * augWidth + c] -= factor * aug[col * augWidth + c];
            }
        }
    }
    return true;
}

// Back substitution on upper-triangular flat augmented matrix
static void back_substitute(float* aug, uint8_t n, uint8_t augWidth) {
    for (int8_t col = static_cast<int8_t>(n - 1); col >= 0; col--) {
        float pivot = aug[col * augWidth + col];
        for (uint8_t c = 0; c < augWidth; c++) {
            aug[col * augWidth + c] /= pivot;
        }
        for (int8_t r = static_cast<int8_t>(col - 1); r >= 0; r--) {
            float factor = aug[r * augWidth + col];
            for (uint8_t c = 0; c < augWidth; c++) {
                aug[r * augWidth + c] -= factor * aug[col * augWidth + c];
            }
        }
    }
}

// NxN matrix inverse via Gaussian elimination with partial pivoting
// Uses a static working buffer sized for the largest supported dimension (9x9)
// Returns false if singular
constexpr uint8_t kMaxMatDim    = 9;
constexpr uint8_t kMaxAugWidth  = kMaxMatDim * 2;

static bool mat_inverse(const float* src, float* dst, uint8_t n) {
    if (n > kMaxMatDim) {
        return false;
    }
    uint8_t augWidth = n * 2;

    // Static augmented matrix [src | I] — sized for largest case (9x18)
    static float g_aug[kMaxMatDim * kMaxAugWidth];

    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            g_aug[r * augWidth + c] = src[r * n + c];
            g_aug[r * augWidth + c + n] = (r == c) ? 1.0F : 0.0F;
        }
    }

    if (!forward_eliminate(g_aug, n, augWidth)) {
        return false;
    }
    back_substitute(g_aug, n, augWidth);

    // Extract inverse from right half
    for (uint8_t r = 0; r < n; r++) {
        for (uint8_t c = 0; c < n; c++) {
            dst[r * n + c] = g_aug[r * augWidth + c + n];
        }
    }
    return true;
}

void calibration_reset_6pos() {
    memset(g_6posSamples, 0, sizeof(g_6posSamples));
    memset(g_6posAvg, 0, sizeof(g_6posAvg));
    g_6posSampleCount = 0;
    g_6posCollected = 0;
}

const char* calibration_get_6pos_name(uint8_t pos) {
    if (pos >= kAccel6posPositions) {
        return "UNKNOWN";
    }
    return kPositionNames[pos];
}

const float* calibration_get_6pos_avg(uint8_t pos) {
    if (pos >= kAccel6posPositions) {
        return nullptr;
    }
    return g_6posAvg[pos];
}

cal_result_t calibration_collect_6pos_position(uint8_t pos, accel_read_fn readFn) {
    if (pos >= kAccel6posPositions) {
        return CAL_RESULT_INVALID_DATA;
    }
    if (readFn == nullptr) {
        return CAL_RESULT_INVALID_DATA;
    }
    if ((g_6posCollected & (1 << pos)) != 0) {
        return CAL_RESULT_INVALID_DATA;  // Already done
    }

    uint16_t baseIdx = pos * kAccel6posSamplesPerPos;
    float sum[3] = {0.0F, 0.0F, 0.0F};
    float tempUnused = 0.0F;

    for (uint16_t i = 0; i < kAccel6posSamplesPerPos; i++) {
        float ax = 0.0F;
        float ay = 0.0F;
        float az = 0.0F;
        if (!readFn(&ax, &ay, &az, &tempUnused)) {
            return CAL_RESULT_NO_DATA;
        }

        g_6posSamples[baseIdx + i][0] = ax;
        g_6posSamples[baseIdx + i][1] = ay;
        g_6posSamples[baseIdx + i][2] = az;

        sum[0] += ax; sum[1] += ay; sum[2] += az;
    }

    // No motion check — ArduPilot doesn't either. Solver handles noisy data;
    // if samples are bad the fit won't converge (caught at gate check).

    // Compute average
    float n = static_cast<float>(kAccel6posSamplesPerPos);
    g_6posAvg[pos][0] = sum[0] / n;
    g_6posAvg[pos][1] = sum[1] / n;
    g_6posAvg[pos][2] = sum[2] / n;

    // No orientation pre-check — ArduPilot doesn't do this either.
    // If the user places the board wrong, the Gauss-Newton fit won't converge
    // and the gate checks (offset, scale, gravity magnitude) catch the error.

    g_6posCollected |= (1 << pos);
    g_6posSampleCount = 0;
    for (uint8_t p = 0; p < kAccel6posPositions; p++) {
        if ((g_6posCollected & (1 << p)) != 0) {
            g_6posSampleCount += kAccel6posSamplesPerPos;
        }
    }

    return CAL_RESULT_OK;
}

// Accumulate normal equations (J^T*J and J^T*r) over all samples for current params
static void accumulate_normal_equations(const float* params, float* jtfi) {
    float jacob[kAccel6posNumParams];
    memset(g_jtj, 0, sizeof(g_jtj));
    memset(jtfi, 0, kAccel6posNumParams * sizeof(float));

    for (uint16_t i = 0; i < g_6posSampleCount; i++) {
        float r = calc_residual_6pos(g_6posSamples[i], params);
        calc_jacobian_6pos(g_6posSamples[i], params, jacob);

        for (uint8_t row = 0; row < kAccel6posNumParams; row++) {
            jtfi[row] += jacob[row] * r;
            for (uint8_t col = 0; col < kAccel6posNumParams; col++) {
                g_jtj[row * kAccel6posNumParams + col] += jacob[row] * jacob[col];
            }
        }
    }
}

// Apply Gauss-Newton parameter update. Returns false if NaN/Inf encountered.
static bool gauss_newton_update(float* params, const float* jtfi) {
    float newParams[kAccel6posNumParams];
    for (uint8_t i = 0; i < kAccel6posNumParams; i++) {
        float delta = 0.0F;
        for (uint8_t j = 0; j < kAccel6posNumParams; j++) {
            delta += g_jtjInv[i * kAccel6posNumParams + j] * jtfi[j];
        }
        newParams[i] = params[i] - delta;
        if (isnan(newParams[i]) || isinf(newParams[i])) {
            return false;
        }
    }
    memcpy(params, newParams, sizeof(newParams));
    return true;
}

// Validate 6-pos fit result: offsets and diagonal within bounds
static bool validate_6pos_params(const float* params) {
    for (uint8_t i = 0; i < kAccel6posOffsetEnd; i++) {
        if (fabsf(params[i]) > kAccel6posMaxOffset) {
            return false;
        }
    }
    for (uint8_t i = kAccel6posOffsetEnd; i < kAccel6posDiagEnd; i++) {
        if (params[i] < kAccel6posMinDiag || params[i] > kAccel6posMaxDiag) {
            return false;
        }
    }
    return true;
}

// Store 6-pos fit results into global calibration
static void store_6pos_results(const float* params) {
    // params[0..2] = offset, params[3..5] = diagonal scale
    g_calibration.accel.offset.x = params[0];
    g_calibration.accel.offset.y = params[1];
    g_calibration.accel.offset.z = params[2];
    g_calibration.accel.scale.x  = params[3];
    g_calibration.accel.scale.y  = params[4];
    g_calibration.accel.scale.z  = params[kParamDiagZ];
    // Offdiag zeroed — underdetermined from 6 axis-aligned positions.
    // A 9-param model (offset + diag + offdiag) is valid in principle, but the
    // 3 offdiag parameters sit in the Jacobian null space when only axis-aligned
    // data is available. ArduPilot solves 9 params but discards offdiag at runtime.
    // We solve only the 6 that the data constrains. Cross-axis coupling from the
    // ICM-20948 is ±2% (DS-000189 Table 2) — negligible for our use case.
    g_calibration.accel.offdiag = cal_vec3_t{0.0F, 0.0F, 0.0F};
    g_calibration.accel.status = CAL_STATUS_ACCEL_6POS;

    g_calibration.cal_flags |= CAL_STATUS_ACCEL_6POS;
    g_calibration.cal_flags &= ~CAL_STATUS_LEVEL;  // 6-pos supersedes level cal
    calibration_update_crc(&g_calibration);
}

cal_result_t calibration_compute_6pos() {
    if (g_6posCollected != kAccel6posAllMask) {
        return CAL_RESULT_NO_DATA;
    }

    // Initialize parameters: offset={0,0,0}, diag={1,1,1}
    float params[kAccel6posNumParams] = {
        0.0F, 0.0F, 0.0F,
        1.0F, 1.0F, 1.0F
    };

    float bestParams[kAccel6posNumParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = calc_mean_sq_residuals(params);
    float jtfi[kAccel6posNumParams];

    for (uint16_t iter = 0; iter < kAccel6posMaxIterations; iter++) {
        accumulate_normal_equations(params, jtfi);

        if (!mat_inverse(g_jtj, g_jtjInv, kAccel6posNumParams)) {
            break;  // Singular — use best so far
        }

        if (!gauss_newton_update(params, jtfi)) {
            break;  // NaN — use best so far
        }

        float fitness = calc_mean_sq_residuals(params);
        if (fitness < bestFitness) {
            bestFitness = fitness;
            memcpy(bestParams, params, sizeof(params));
        }
    }

    if (!validate_6pos_params(bestParams)) {
        return CAL_RESULT_FIT_FAILED;
    }

    store_6pos_results(bestParams);
    return CAL_RESULT_OK;
}

// ============================================================================
// Magnetometer Calibration (IVP-35/36)
// ============================================================================

// Coverage grid: map a unit vector to a lat/lon section index (0-79)
static uint8_t mag_coverage_section(float nx, float ny, float nz) {
    // Latitude: asin(nz) maps [-1,1] to [-pi/2, pi/2], scale to [0, kMagLatBands-1]
    float lat = asinf(nz);
    auto latBin = static_cast<uint8_t>((lat + (static_cast<float>(M_PI) / 2.0F))
                  * static_cast<float>(kMagLatBands) / static_cast<float>(M_PI));
    if (latBin >= kMagLatBands) {
        latBin = kMagLatBands - 1;
    }

    // Longitude: atan2(ny, nx) maps to [-pi, pi], scale to [0, kMagLonSectors-1]
    float lon = atan2f(ny, nx);
    auto lonBin = static_cast<uint8_t>((lon + static_cast<float>(M_PI))
                  * static_cast<float>(kMagLonSectors) / (2.0F * static_cast<float>(M_PI)));
    if (lonBin >= kMagLonSectors) {
        lonBin = kMagLonSectors - 1;
    }

    return latBin * kMagLonSectors + lonBin;
}

static void mag_coverage_set(uint8_t section) {
    g_magCoverageMask[section / 8] |= (1 << (section % 8));
}

static uint8_t mag_coverage_count() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < kMagCoverageSections; i++) {
        if ((g_magCoverageMask[i / 8] & (1 << (i % 8))) != 0) {
            count++;
        }
    }
    return count;
}

void calibration_reset_mag_cal() {
    memset(g_magSamples, 0, sizeof(g_magSamples));
    memset(g_magCoverageMask, 0, sizeof(g_magCoverageMask));
    g_magSampleCount = 0;
    g_magExpectedRadius = 0.0F;
    g_magFitness = 0.0F;
}

uint16_t calibration_get_mag_sample_count() {
    return g_magSampleCount;
}

uint8_t calibration_get_mag_coverage_pct() {
    return (mag_coverage_count() * 100) / kMagCoverageSections;
}

float calibration_get_mag_fitness() {
    return g_magFitness;
}

mag_feed_result_t calibration_feed_mag_sample(float mx, float my, float mz) {
    if (g_magSampleCount >= kMagMaxSamples) {
        return mag_feed_result_t::BUFFER_FULL;
    }

    // Range check: reject if magnitude outside expected Earth field range
    float mag = sqrtf(mx*mx + my*my + mz*mz);
    if (mag < kMagMinFieldUt || mag > kMagMaxFieldUt) {
        return mag_feed_result_t::REJECTED_RANGE;
    }

    // Angular separation check against recent samples
    // Minimum angular separation based on TARGET count (kMagMaxSamples), not running count.
    // Using running count requires near-orthogonal orientations for early samples (90° for
    // sample 2), rejecting everything during smooth hand rotation. ArduPilot uses fixed
    // minimum distance based on target sample count.
    if (g_magSampleCount > 0) {
        float nx = mx / mag;
        float ny = my / mag;
        float nz = mz / mag;

        // Check against the most recent kMagRecentWindow samples
        uint16_t checkStart = (g_magSampleCount > kMagRecentWindow)
                              ? (g_magSampleCount - kMagRecentWindow) : 0;

        float minSepCos = 1.0F - 2.0F / static_cast<float>(kMagMaxSamples);

        for (uint16_t i = checkStart; i < g_magSampleCount; i++) {
            float rmag = sqrtf(g_magSamples[i][0]*g_magSamples[i][0]
                             + g_magSamples[i][1]*g_magSamples[i][1]
                             + g_magSamples[i][2]*g_magSamples[i][2]);
            if (rmag < kMinVectorLength) {
                continue;
            }
            float dot = (nx * g_magSamples[i][0]
                       + ny * g_magSamples[i][1]
                       + nz * g_magSamples[i][2]) / rmag;
            if (dot > minSepCos) {
                return mag_feed_result_t::REJECTED_CLOSE;
            }
        }
    }

    // Accept sample
    g_magSamples[g_magSampleCount][0] = mx;
    g_magSamples[g_magSampleCount][1] = my;
    g_magSamples[g_magSampleCount][2] = mz;
    g_magSampleCount++;

    // Update coverage grid
    float invMag = 1.0F / mag;
    uint8_t section = mag_coverage_section(mx * invMag, my * invMag, mz * invMag);
    mag_coverage_set(section);

    return mag_feed_result_t::ACCEPTED;
}

// ============================================================================
// Levenberg-Marquardt Solver (shared between sphere and ellipsoid fits)
// ============================================================================

// Function pointer types for residual and Jacobian evaluation
using ResidualFn = float (*)(const float sample[3], const float* params);
using JacobianFn = void (*)(const float sample[3], const float* params, float* jacob);

// Compute mean squared residuals over g_magSamples[0..numSamples-1]
static float lm_mean_sq_residuals(const float* params, uint16_t numSamples,
                                   ResidualFn residualFn) {
    float sum = 0.0F;
    for (uint16_t i = 0; i < numSamples; i++) {
        float r = residualFn(g_magSamples[i], params);
        sum += r * r;
    }
    return sum / static_cast<float>(numSamples);
}

// Run LM iterations on params[0..numParams-1] using g_magSamples[0..numSamples-1].
// Uses g_jtj / g_jtjInv as working buffers (sized for 9x9, sufficient for 4x4).
// On return, bestParams holds the best fit and bestFitness holds RMS^2.
// JSF AV Rule 170 deviation: function pointers accepted for Ground-classified code.
// See STANDARDS_DEVIATIONS.md FP-1.
static void lm_solve(float* params, float* bestParams, float* bestFitness,
                     uint8_t numParams, uint16_t numSamples, uint8_t maxIter,
                     ResidualFn residualFn, JacobianFn jacobianFn) {
    float lambda = kMagLmLambdaInit;
    float jacob[kLmMaxParams];
    float jtfi[kLmMaxParams];

    for (uint8_t iter = 0; iter < maxIter; iter++) {
        // Accumulate J^T*J and J^T*r
        memset(g_jtj, 0, numParams * numParams * sizeof(float));
        memset(jtfi, 0, numParams * sizeof(float));

        for (uint16_t i = 0; i < numSamples; i++) {
            float r = residualFn(g_magSamples[i], params);
            jacobianFn(g_magSamples[i], params, jacob);

            for (uint8_t row = 0; row < numParams; row++) {
                jtfi[row] += jacob[row] * r;
                for (uint8_t col = 0; col < numParams; col++) {
                    g_jtj[row * numParams + col] += jacob[row] * jacob[col];
                }
            }
        }

        // LM damping: add lambda to diagonal
        for (uint8_t d = 0; d < numParams; d++) {
            g_jtj[d * numParams + d] += lambda;
        }

        if (!mat_inverse(g_jtj, g_jtjInv, numParams)) {
            break;  // Singular
        }

        // Compute parameter update
        float newParams[kLmMaxParams];
        bool valid = true;
        for (uint8_t i = 0; i < numParams; i++) {
            float delta = 0.0F;
            for (uint8_t j = 0; j < numParams; j++) {
                delta += g_jtjInv[i * numParams + j] * jtfi[j];
            }
            newParams[i] = params[i] - delta;
            if (isnan(newParams[i]) || isinf(newParams[i])) {
                valid = false;
                break;
            }
        }
        if (!valid) {
            break;
        }

        // Evaluate new fitness
        float fitness = lm_mean_sq_residuals(newParams, numSamples, residualFn);

        if (fitness < *bestFitness) {
            *bestFitness = fitness;
            memcpy(bestParams, newParams, numParams * sizeof(float));
            memcpy(params, newParams, numParams * sizeof(float));
            lambda *= kMagLmLambdaDown;
        } else {
            lambda *= kMagLmLambdaUp;
        }
    }
}

// --- Sphere fit (Step 1): 4 params [radius, offset_x, offset_y, offset_z] ---

static float calc_sphere_residual(const float sample[3], const float params[kMagSphereParams]) {
    float sx = sample[0] + params[1];
    float sy = sample[1] + params[2];
    float sz = sample[2] + params[3];
    float len = sqrtf(sx*sx + sy*sy + sz*sz);
    return params[0] - len;  // radius - |raw + offset|
}

// NOLINTBEGIN(readability-magic-numbers) — param indices: [0]=radius, [1-3]=offset
static void calc_sphere_jacobian(const float sample[3], const float params[kMagSphereParams],
                                  float jacob[kMagSphereParams]) {
    float sx = sample[0] + params[1];
    float sy = sample[1] + params[2];
    float sz = sample[2] + params[3];
    float len = sqrtf(sx*sx + sy*sy + sz*sz);

    if (len < kMinVectorLength) {
        memset(jacob, 0, kMagSphereParams * sizeof(float));
        return;
    }

    jacob[0] = 1.0F;           // d(r)/d(radius) = 1
    jacob[1] = -sx / len;      // d(r)/d(offset_x)
    jacob[2] = -sy / len;      // d(r)/d(offset_y)
    jacob[3] = -sz / len;      // d(r)/d(offset_z)
}
// NOLINTEND(readability-magic-numbers)

// Run sphere fit LM solver on the first numSamples entries of g_magSamples
static bool mag_sphere_fit(uint16_t numSamples, float* outRadius, float outOffset[3]) {
    // Initial guess: radius = mean magnitude, offset = 0
    float sumMag = 0.0F;
    for (uint16_t i = 0; i < numSamples; i++) {
        sumMag += sqrtf(g_magSamples[i][0]*g_magSamples[i][0]
                      + g_magSamples[i][1]*g_magSamples[i][1]
                      + g_magSamples[i][2]*g_magSamples[i][2]);
    }

    float params[kMagSphereParams] = {
        sumMag / static_cast<float>(numSamples),  // radius
        0.0F, 0.0F, 0.0F                           // offset
    };

    float bestParams[kMagSphereParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(params, numSamples, calc_sphere_residual);

    lm_solve(params, bestParams, &bestFitness,
             kMagSphereParams, numSamples, kMagSphereIterations,
             calc_sphere_residual, calc_sphere_jacobian);

    // Validate sphere fit
    if (bestParams[0] < kMagMinFieldUt || bestParams[0] > kMagMaxFieldUt) {
        return false;
    }

    *outRadius = bestParams[0];
    outOffset[0] = bestParams[1];
    outOffset[1] = bestParams[2];
    outOffset[2] = bestParams[3];
    return true;
}

// --- Ellipsoid fit (Step 2): same 9-param model as accel 6-pos but with mag radius ---

// NOLINTBEGIN(readability-magic-numbers) — matrix element indices match param vector layout
static float calc_residual_mag(const float sample[3], const float params[kMagEllipsoidParams]) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];

    float a = params[3] * sx + params[6] * sy + params[7] * sz;
    float b = params[6] * sx + params[4] * sy + params[8] * sz;
    float c = params[7] * sx + params[8] * sy + params[5] * sz;

    float len = sqrtf(a*a + b*b + c*c);
    return g_magExpectedRadius - len;
}

static void calc_jacobian_mag(const float sample[3], const float params[kMagEllipsoidParams],
                               float jacob[kMagEllipsoidParams]) {
    float sx = sample[0] + params[0];
    float sy = sample[1] + params[1];
    float sz = sample[2] + params[2];

    float a = params[3] * sx + params[6] * sy + params[7] * sz;
    float b = params[6] * sx + params[4] * sy + params[8] * sz;
    float c = params[7] * sx + params[8] * sy + params[5] * sz;

    float len = sqrtf(a*a + b*b + c*c);
    if (len < kMinVectorLength) {
        memset(jacob, 0, kMagEllipsoidParams * sizeof(float));
        return;
    }

    // d(residual)/d(offset[0..2])
    jacob[0] = -((params[3]*a + params[6]*b + params[7]*c) / len);
    jacob[1] = -((params[6]*a + params[4]*b + params[8]*c) / len);
    jacob[2] = -((params[7]*a + params[8]*b + params[5]*c) / len);
    // d(residual)/d(diag[0..2])
    jacob[3] = -(sx * a / len);
    jacob[4] = -(sy * b / len);
    jacob[kParamDiagZ] = -(sz * c / len);
    // d(residual)/d(offdiag[0..2]) — XY, XZ, YZ
    jacob[6] = -((sy*a + sx*b) / len);
    jacob[7] = -((sz*a + sx*c) / len);
    jacob[8] = -((sz*b + sy*c) / len);
}
// NOLINTEND(readability-magic-numbers)

// Fisher-Yates shuffle using RP2350 hardware TRNG, then truncate to 2/3
static uint16_t mag_thin_samples(uint16_t count) {
    // Shuffle using get_rand_32() from pico/rand.h
    for (uint16_t i = count - 1; i > 0; i--) {
        uint32_t r = get_rand_32() % (i + 1);
        if (r != i) {
            float tmp[3];
            memcpy(tmp, g_magSamples[i], sizeof(tmp));
            memcpy(g_magSamples[i], g_magSamples[r], sizeof(tmp));
            memcpy(g_magSamples[r], tmp, sizeof(tmp));
        }
    }

    return (count * 2) / 3;
}

static bool mag_ellipsoid_fit(uint16_t numSamples, const float sphereOffset[3],
                               float outParams[kMagEllipsoidParams]) {
    // Seed from sphere fit results
    // NOLINTBEGIN(readability-magic-numbers) — param layout: offset[0-2], diag[3-5], offdiag[6-8]
    float params[kMagEllipsoidParams] = {
        sphereOffset[0], sphereOffset[1], sphereOffset[2],
        1.0F, 1.0F, 1.0F,
        0.0F, 0.0F, 0.0F
    };
    // NOLINTEND(readability-magic-numbers)

    float bestParams[kMagEllipsoidParams];
    memcpy(bestParams, params, sizeof(params));
    float bestFitness = lm_mean_sq_residuals(params, numSamples, calc_residual_mag);

    lm_solve(params, bestParams, &bestFitness,
             kMagEllipsoidParams, numSamples, kMagEllipsoidIterations,
             calc_residual_mag, calc_jacobian_mag);

    memcpy(outParams, bestParams, sizeof(bestParams));
    g_magFitness = sqrtf(bestFitness);  // RMS residual
    return true;
}

// Validate ellipsoid fit parameters
static bool validate_mag_params(const float params[kMagEllipsoidParams]) {
    // NOLINTBEGIN(readability-magic-numbers) — param layout: offset[0-2], diag[3-5], offdiag[6-8]
    // Offset bounds
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsf(params[i]) > kMagMaxOffset) {
            return false;
        }
    }
    // Diagonal scale bounds
    for (uint8_t i = 3; i < 6; i++) {
        if (params[i] < kMagMinDiag || params[i] > kMagMaxDiag) {
            return false;
        }
    }
    // Off-diagonal bounds
    for (uint8_t i = 6; i < 9; i++) {
        if (fabsf(params[i]) > kMagMaxOffdiag) {
            return false;
        }
    }
    // NOLINTEND(readability-magic-numbers)

    // Radius validation
    if (g_magExpectedRadius < kMagMinFieldUt || g_magExpectedRadius > kMagMaxFieldUt) {
        return false;
    }

    // Fitness check
    if (g_magFitness > kMagMaxFitness) {
        return false;
    }

    return true;
}

cal_result_t calibration_compute_mag_cal() {
    if (g_magSampleCount < kMagMinSamplesForFit) {
        return CAL_RESULT_NO_DATA;
    }

    // Step 1: Sphere fit on all samples
    float sphereRadius = 0.0F;
    float sphereOffset[3] = {0.0F, 0.0F, 0.0F};

    if (!mag_sphere_fit(g_magSampleCount, &sphereRadius, sphereOffset)) {
        return CAL_RESULT_FIT_FAILED;
    }

    g_magExpectedRadius = sphereRadius;

    // Step 1.5: Fisher-Yates thinning to 2/3
    uint16_t thinned = mag_thin_samples(g_magSampleCount);

    // Step 2: Ellipsoid fit on thinned samples
    float ellParams[kMagEllipsoidParams];
    if (!mag_ellipsoid_fit(thinned, sphereOffset, ellParams)) {
        return CAL_RESULT_FIT_FAILED;
    }

    // Validate
    if (!validate_mag_params(ellParams)) {
        return CAL_RESULT_FIT_FAILED;
    }

    // Store results
    // NOLINTBEGIN(readability-magic-numbers) — param layout: offset[0-2], diag[3-5], offdiag[6-8]
    g_calibration.mag.offset.x  = ellParams[0];
    g_calibration.mag.offset.y  = ellParams[1];
    g_calibration.mag.offset.z  = ellParams[2];
    g_calibration.mag.scale.x   = ellParams[3];
    g_calibration.mag.scale.y   = ellParams[4];
    g_calibration.mag.scale.z   = ellParams[5];
    g_calibration.mag.offdiag.x = ellParams[6];
    g_calibration.mag.offdiag.y = ellParams[7];
    g_calibration.mag.offdiag.z = ellParams[8];
    // NOLINTEND(readability-magic-numbers)
    g_calibration.mag.expected_radius = g_magExpectedRadius;
    g_calibration.mag.status = CAL_STATUS_MAG;

    g_calibration.cal_flags |= CAL_STATUS_MAG;
    calibration_update_crc(&g_calibration);

    return CAL_RESULT_OK;
}

// ============================================================================
// Calibration Control
// ============================================================================

void calibration_cancel() {
    g_calState = CAL_STATE_IDLE;
    g_calResult = CAL_RESULT_OK;
}

void calibration_reset_state() {
    if (g_calState == CAL_STATE_COMPLETE || g_calState == CAL_STATE_FAILED) {
        g_calState = CAL_STATE_IDLE;
    }
}

bool calibration_is_active() {
    return (g_calState != CAL_STATE_IDLE &&
            g_calState != CAL_STATE_COMPLETE &&
            g_calState != CAL_STATE_FAILED);
}

uint8_t calibration_get_progress() {
    if (!calibration_is_active()) {
        return (g_calState == CAL_STATE_COMPLETE) ? 100 : 0;
    }

    if (g_sampleAcc.target_count == 0) {
        return 0;
    }

    uint32_t progress = (g_sampleAcc.count * 100) / g_sampleAcc.target_count;
    return (progress > 100) ? 100 : static_cast<uint8_t>(progress);
}

cal_result_t calibration_get_result() {
    return g_calResult;
}

// ============================================================================
// Applying Calibration
// ============================================================================

void calibration_apply_gyro_with(const calibration_store_t* cal,
                                  float gxRaw, float gyRaw, float gzRaw,
                                  float* gxCal, float* gyCal, float* gzCal) {
    // Bias removal
    float cx = gxRaw - cal->gyro.bias.x;
    float cy = gyRaw - cal->gyro.bias.y;
    float cz = gzRaw - cal->gyro.bias.z;

    // Board rotation: rotMat * corrected
    // NOLINTBEGIN(readability-magic-numbers) — 3x3 rotation matrix indices
    const float* rotMat = cal->board_rotation.m;
    *gxCal = rotMat[0]*cx + rotMat[1]*cy + rotMat[2]*cz;
    *gyCal = rotMat[3]*cx + rotMat[4]*cy + rotMat[5]*cz;
    *gzCal = rotMat[6]*cx + rotMat[7]*cy + rotMat[8]*cz;
    // NOLINTEND(readability-magic-numbers)
}

void calibration_apply_gyro(float gxRaw, float gyRaw, float gzRaw,
                            float* gxCal, float* gyCal, float* gzCal) {
    calibration_apply_gyro_with(&g_calibration, gxRaw, gyRaw, gzRaw,
                                gxCal, gyCal, gzCal);
}

void calibration_apply_accel_with(const calibration_store_t* cal,
                                   float axRaw, float ayRaw, float azRaw,
                                   float* axCal, float* ayCal, float* azCal) {
    // Stage 1: Ellipsoid correction — M * (raw + offset)
    // M is symmetric 3x3 with scale (diagonal) and offdiag terms
    float ox = axRaw + cal->accel.offset.x;
    float oy = ayRaw + cal->accel.offset.y;
    float oz = azRaw + cal->accel.offset.z;

    float cx = cal->accel.scale.x   * ox
             + cal->accel.offdiag.x * oy
             + cal->accel.offdiag.y * oz;
    float cy = cal->accel.offdiag.x * ox
             + cal->accel.scale.y   * oy
             + cal->accel.offdiag.z * oz;
    float cz = cal->accel.offdiag.y * ox
             + cal->accel.offdiag.z * oy
             + cal->accel.scale.z   * oz;

    // Stage 2: Board rotation — rotMat * corrected
    // NOLINTBEGIN(readability-magic-numbers) — 3x3 rotation matrix indices
    const float* rotMat = cal->board_rotation.m;
    *axCal = rotMat[0]*cx + rotMat[1]*cy + rotMat[2]*cz;
    *ayCal = rotMat[3]*cx + rotMat[4]*cy + rotMat[5]*cz;
    *azCal = rotMat[6]*cx + rotMat[7]*cy + rotMat[8]*cz;
    // NOLINTEND(readability-magic-numbers)
}

void calibration_apply_accel(float axRaw, float ayRaw, float azRaw,
                             float* axCal, float* ayCal, float* azCal) {
    calibration_apply_accel_with(&g_calibration, axRaw, ayRaw, azRaw,
                                  axCal, ayCal, azCal);
}

void calibration_apply_mag_with(const calibration_store_t* cal,
                                  float mxRaw, float myRaw, float mzRaw,
                                  float* mxCal, float* myCal, float* mzCal) {
    // Stage 1: Ellipsoid correction — M * (raw + offset)
    float ox = mxRaw + cal->mag.offset.x;
    float oy = myRaw + cal->mag.offset.y;
    float oz = mzRaw + cal->mag.offset.z;

    float cx = cal->mag.scale.x   * ox
             + cal->mag.offdiag.x * oy
             + cal->mag.offdiag.y * oz;
    float cy = cal->mag.offdiag.x * ox
             + cal->mag.scale.y   * oy
             + cal->mag.offdiag.z * oz;
    float cz = cal->mag.offdiag.y * ox
             + cal->mag.offdiag.z * oy
             + cal->mag.scale.z   * oz;

    // Stage 2: Board rotation — rotMat * corrected
    // NOLINTBEGIN(readability-magic-numbers) — 3x3 rotation matrix indices
    const float* rotMat = cal->board_rotation.m;
    *mxCal = rotMat[0]*cx + rotMat[1]*cy + rotMat[2]*cz;
    *myCal = rotMat[3]*cx + rotMat[4]*cy + rotMat[5]*cz;
    *mzCal = rotMat[6]*cx + rotMat[7]*cy + rotMat[8]*cz;
    // NOLINTEND(readability-magic-numbers)
}

void calibration_apply_mag(float mxRaw, float myRaw, float mzRaw,
                            float* mxCal, float* myCal, float* mzCal) {
    calibration_apply_mag_with(&g_calibration, mxRaw, myRaw, mzRaw,
                                mxCal, myCal, mzCal);
}

bool calibration_load_into(calibration_store_t* dest) {
    if (dest == nullptr) {
        return false;
    }
    calibration_store_t loaded;
    if (!calibration_storage_read(&loaded)) {
        return false;
    }
    if (!calibration_validate(&loaded)) {
        return false;
    }
    *dest = loaded;
    return true;
}

float calibration_get_altitude_agl(float pressurePa) {
    // Barometric formula: h = 44330 * (1 - (P/P0)^0.1903)
    float p0 = g_calibration.baro.ground_pressure_pa;
    if (p0 < kMinValidPressurePa) {
        p0 = kStdAtmPressurePa;  // Sanity check
    }

    return kHypsometricScale * (1.0F - powf(pressurePa / p0, kHypsometricExponent));
}

// ============================================================================
// Storage
// ============================================================================

cal_result_t calibration_save() {
    calibration_update_crc(&g_calibration);
    if (!calibration_storage_write(&g_calibration)) {
        return CAL_RESULT_STORAGE_ERROR;
    }
    return CAL_RESULT_OK;
}

cal_result_t calibration_load() {
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

cal_result_t calibration_reset() {
    calibration_init_defaults(&g_calibration);
    return calibration_save();
}
