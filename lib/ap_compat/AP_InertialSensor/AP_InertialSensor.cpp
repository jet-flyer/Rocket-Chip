/**
 * @file AP_InertialSensor.cpp
 * @brief Inertial sensor calibration implementation
 */

// Include HAL_RP2350 FIRST to set macros before AP_HAL.h fallbacks
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include "AP_InertialSensor.h"
#include <cstring>

// ============================================================================
// Singleton
// ============================================================================

AP_InertialSensor* AP_InertialSensor::_singleton = nullptr;

AP_InertialSensor* AP_InertialSensor::get_singleton()
{
    if (_singleton == nullptr) {
        _singleton = new AP_InertialSensor();
    }
    return _singleton;
}

// ============================================================================
// Constructor
// ============================================================================

AP_InertialSensor::AP_InertialSensor()
    : _initialized(false)
    , _saving(false)
{
    // Zero-initialize instance calibration
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        _instance[i].accel_calibrated = false;
        _instance[i].gyro_calibrated = false;
    }
}

// ============================================================================
// Initialization
// ============================================================================

void AP_InertialSensor::init()
{
    if (_initialized) {
        return;
    }

    // Ensure AP_Param is setup
    if (!AP_Param::initialized()) {
        AP_Param::setup();
    }

    // Initialize parameter storage offsets
    init_param_offsets();

    // Load calibration from flash
    load_calibration();

    _initialized = true;
}

void AP_InertialSensor::init_param_offsets()
{
    // Instance 0
    _instance[0].accel_offset.set_offset(RocketChip::kAccelOffset0);
    _instance[0].accel_scale.set_offset(RocketChip::kAccelScale0);
    _instance[0].accel_offdiag.set_offset(RocketChip::kAccelOffdiag0);
    _instance[0].gyro_offset.set_offset(RocketChip::kGyroOffset0);

    // Instance 1
    _instance[1].accel_offset.set_offset(RocketChip::kAccelOffset1);
    _instance[1].accel_scale.set_offset(RocketChip::kAccelScale1);
    _instance[1].accel_offdiag.set_offset(RocketChip::kAccelOffdiag1);
    _instance[1].gyro_offset.set_offset(RocketChip::kGyroOffset1);

    // Set default values (identity calibration)
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        _instance[i].accel_offset.set(Vector3f(0, 0, 0));
        _instance[i].accel_scale.set(Vector3f(1, 1, 1));
        _instance[i].accel_offdiag.set(Vector3f(0, 0, 0));
        _instance[i].gyro_offset.set(Vector3f(0, 0, 0));
    }
}

// ============================================================================
// Calibration Status
// ============================================================================

bool AP_InertialSensor::accel_calibrated(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }
    return _instance[instance].accel_calibrated;
}

bool AP_InertialSensor::gyro_calibrated(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }
    return _instance[instance].gyro_calibrated;
}

// ============================================================================
// Calibration Parameters
// ============================================================================

const Vector3f& AP_InertialSensor::get_accel_offset(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        static Vector3f zero;
        return zero;
    }
    return _instance[instance].accel_offset.get();
}

const Vector3f& AP_InertialSensor::get_accel_scale(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        static Vector3f one(1, 1, 1);
        return one;
    }
    return _instance[instance].accel_scale.get();
}

const Vector3f& AP_InertialSensor::get_accel_offdiag(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        static Vector3f zero;
        return zero;
    }
    return _instance[instance].accel_offdiag.get();
}

const Vector3f& AP_InertialSensor::get_gyro_offset(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        static Vector3f zero;
        return zero;
    }
    return _instance[instance].gyro_offset.get();
}

// ============================================================================
// Apply Calibration
// ============================================================================

Vector3f AP_InertialSensor::correct_accel(const Vector3f& raw, uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return raw;
    }

    const Vector3f& offset = _instance[instance].accel_offset.get();
    const Vector3f& scale = _instance[instance].accel_scale.get();
    const Vector3f& offdiag = _instance[instance].accel_offdiag.get();

    // Apply offset first
    Vector3f corrected = raw + offset;

    // Apply scale and off-diagonal corrections
    // Full correction matrix M is:
    // | scale.x    offdiag.x  offdiag.y |
    // | offdiag.x  scale.y    offdiag.z |
    // | offdiag.y  offdiag.z  scale.z   |
    //
    // For axis-aligned calibration (offdiag = 0), this simplifies to:
    // corrected = corrected .* scale (element-wise multiply)
    //
    // For full ellipsoid, we do full matrix multiply:
    Vector3f result;
    result.x = corrected.x * scale.x + corrected.y * offdiag.x + corrected.z * offdiag.y;
    result.y = corrected.x * offdiag.x + corrected.y * scale.y + corrected.z * offdiag.z;
    result.z = corrected.x * offdiag.y + corrected.y * offdiag.z + corrected.z * scale.z;

    return result;
}

Vector3f AP_InertialSensor::correct_gyro(const Vector3f& raw, uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return raw;
    }

    // Simple offset subtraction for gyro
    return raw - _instance[instance].gyro_offset.get();
}

// ============================================================================
// Accelerometer Calibration Control
// ============================================================================

AccelCalibrator* AP_InertialSensor::get_calibrator(uint8_t instance)
{
    if (instance >= INS_MAX_INSTANCES) {
        return nullptr;
    }
    return &_calibrators[instance];
}

void AP_InertialSensor::accel_calibrate_start(
    uint8_t num_samples,
    float sample_time,
    accel_cal_fit_type_t fit_type)
{
    // Start calibration on instance 0
    // For warm-start, we could pass current calibration as initial estimate
    _calibrators[0].start(fit_type, num_samples, sample_time);
}

void AP_InertialSensor::accel_cal_collect_sample()
{
    _calibrators[0].collect_sample();
}

void AP_InertialSensor::accel_cal_new_sample(const Vector3f& delta_velocity, float dt)
{
    _calibrators[0].new_sample(delta_velocity, dt);
}

void AP_InertialSensor::accel_cal_check_timeout()
{
    _calibrators[0].check_for_timeout();
}

accel_cal_status_t AP_InertialSensor::accel_cal_get_status(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return ACCEL_CAL_NOT_STARTED;
    }
    return _calibrators[instance].get_status();
}

bool AP_InertialSensor::accel_calibrating(uint8_t instance)
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }
    return _calibrators[instance].running();
}

bool AP_InertialSensor::accel_cal_success(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return false;
    }
    return _calibrators[instance].get_status() == ACCEL_CAL_SUCCESS;
}

float AP_InertialSensor::accel_cal_get_fitness(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return 0.0f;
    }
    return _calibrators[instance].get_fitness();
}

uint8_t AP_InertialSensor::accel_cal_get_samples_collected(uint8_t instance) const
{
    if (instance >= INS_MAX_INSTANCES) {
        return 0;
    }
    return _calibrators[instance].get_num_samples_collected();
}

void AP_InertialSensor::accel_cal_cancel()
{
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        _calibrators[i].clear();
    }
}

// ============================================================================
// Gyroscope Calibration
// ============================================================================

bool AP_InertialSensor::gyro_calibrate_simple(
    uint16_t samples,
    uint16_t sample_period_ms,
    Vector3f (*get_gyro_func)())
{
    if (get_gyro_func == nullptr || samples == 0) {
        return false;
    }

    // Accumulate samples
    Vector3f sum(0, 0, 0);

    for (uint16_t i = 0; i < samples; i++) {
        Vector3f gyro = get_gyro_func();
        sum += gyro;

        // Wait for next sample
        // Note: In actual implementation, would use proper delay
        // For now, caller is responsible for timing
    }

    // Average is the offset
    Vector3f offset = sum / static_cast<float>(samples);

    // Save
    _instance[0].gyro_offset.set_and_save(offset);
    _instance[0].gyro_calibrated = true;

    // Set flag
    AP_Param::set_cal_flag(RocketChip::kCalFlagGyro0);

    return true;
}

void AP_InertialSensor::set_gyro_offset(uint8_t instance, const Vector3f& offset)
{
    if (instance >= INS_MAX_INSTANCES) {
        return;
    }

    _instance[instance].gyro_offset.set_and_save(offset);
    _instance[instance].gyro_calibrated = true;

    // Set appropriate flag
    uint16_t flag = (instance == 0) ? RocketChip::kCalFlagGyro0 : RocketChip::kCalFlagGyro1;
    AP_Param::set_cal_flag(flag);
}

// ============================================================================
// Save/Load
// ============================================================================

void AP_InertialSensor::save_calibration()
{
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        _instance[i].accel_offset.save();
        _instance[i].accel_scale.save();
        _instance[i].accel_offdiag.save();
        _instance[i].gyro_offset.save();
    }

    // Flush to flash
    AP_Param::flush();
}

void AP_InertialSensor::load_calibration()
{
    uint16_t flags = AP_Param::get_cal_flags();

    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        // Load parameters
        _instance[i].accel_offset.load();
        _instance[i].accel_scale.load();
        _instance[i].accel_offdiag.load();
        _instance[i].gyro_offset.load();

        // Check if calibrated (based on flags)
        uint16_t accel_flag = (i == 0) ? RocketChip::kCalFlagAccel0 : RocketChip::kCalFlagAccel1;
        uint16_t gyro_flag = (i == 0) ? RocketChip::kCalFlagGyro0 : RocketChip::kCalFlagGyro1;

        _instance[i].accel_calibrated = (flags & accel_flag) != 0;
        _instance[i].gyro_calibrated = (flags & gyro_flag) != 0;

        // If not calibrated, ensure identity values
        if (!_instance[i].accel_calibrated) {
            _instance[i].accel_offset.set(Vector3f(0, 0, 0));
            _instance[i].accel_scale.set(Vector3f(1, 1, 1));
            _instance[i].accel_offdiag.set(Vector3f(0, 0, 0));
        }

        if (!_instance[i].gyro_calibrated) {
            _instance[i].gyro_offset.set(Vector3f(0, 0, 0));
        }
    }
}

void AP_InertialSensor::reset_calibration()
{
    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        _instance[i].accel_offset.set_and_save(Vector3f(0, 0, 0));
        _instance[i].accel_scale.set_and_save(Vector3f(1, 1, 1));
        _instance[i].accel_offdiag.set_and_save(Vector3f(0, 0, 0));
        _instance[i].gyro_offset.set_and_save(Vector3f(0, 0, 0));
        _instance[i].accel_calibrated = false;
        _instance[i].gyro_calibrated = false;
    }

    // Clear flags
    AP_Param::clear_cal_flag(RocketChip::kCalFlagAccel0 | RocketChip::kCalFlagAccel1 |
                             RocketChip::kCalFlagGyro0 | RocketChip::kCalFlagGyro1);

    AP_Param::flush();
}

// ============================================================================
// AP_AccelCal_Client Interface
// ============================================================================

AccelCalibrator* AP_InertialSensor::_acal_get_calibrator(uint8_t instance)
{
    if (instance >= INS_MAX_INSTANCES) {
        return nullptr;
    }
    return &_calibrators[instance];
}

void AP_InertialSensor::_acal_save_calibrations()
{
    _saving = true;

    for (uint8_t i = 0; i < INS_MAX_INSTANCES; i++) {
        AccelCalibrator& cal = _calibrators[i];

        if (cal.get_status() == ACCEL_CAL_SUCCESS) {
            // Get calibration results
            Vector3f offset, diag, offdiag;
            cal.get_calibration(offset, diag, offdiag);

            // Save to parameters
            _instance[i].accel_offset.set_and_save(offset);
            _instance[i].accel_scale.set_and_save(diag);
            _instance[i].accel_offdiag.set_and_save(offdiag);
            _instance[i].accel_calibrated = true;

            // Set calibration flag
            uint16_t flag = (i == 0) ? RocketChip::kCalFlagAccel0 : RocketChip::kCalFlagAccel1;
            AP_Param::set_cal_flag(flag);
        }
    }

    // Flush to flash
    AP_Param::flush();

    _saving = false;
}

void AP_InertialSensor::_acal_event_success()
{
    // Called after calibration succeeds
    // _acal_save_calibrations() handles the actual saving
}

void AP_InertialSensor::_acal_event_failure()
{
    // Called after calibration fails
    // Could log error or notify user
}
