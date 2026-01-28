/**
 * @file CalibrationStore.cpp
 * @brief Calibration storage implementation
 */

#include "CalibrationStore.h"
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <cstring>

// ArduPilot pattern: use global hal reference
extern const AP_HAL::HAL& hal;

namespace rocketchip {

// ============================================================================
// Initialization
// ============================================================================

void CalibrationStore::init()
{
    // Storage is initialized via hal.init()
    // We just mark ourselves as ready
    m_initialized = true;
}

// ============================================================================
// Load/Save
// ============================================================================

bool CalibrationStore::load(SensorCalibration& cal)
{
    if (!m_initialized) {
        return false;
    }

    // Read from storage
    hal.storage->read_block(&cal, kCalibrationOffset, sizeof(SensorCalibration));

    // Validate magic number
    if (cal.magic != kCalibrationMagic) {
        return false;
    }

    // Validate version (allow loading older versions in future)
    if (cal.version != kCalibrationVersion) {
        return false;
    }

    // Validate CRC
    uint32_t expected_crc = compute_crc(cal);
    if (cal.crc32 != expected_crc) {
        return false;
    }

    return true;
}

bool CalibrationStore::save(const SensorCalibration& cal)
{
    if (!m_initialized) {
        return false;
    }

    // Make a copy to add header fields
    SensorCalibration to_save = cal;
    to_save.magic = kCalibrationMagic;
    to_save.version = kCalibrationVersion;
    to_save.crc32 = compute_crc(to_save);

    // Write to storage
    hal.storage->write_block(kCalibrationOffset, &to_save, sizeof(SensorCalibration));

    // Trigger immediate flush (optional - storage flushes periodically anyway)
    hal.storage->_timer_tick();

    return true;
}

bool CalibrationStore::erase()
{
    if (!m_initialized) {
        return false;
    }

    // Zero out the calibration region
    uint8_t zeros[kCalibrationSize];
    memset(zeros, 0, sizeof(zeros));

    hal.storage->write_block(kCalibrationOffset, zeros, sizeof(zeros));
    hal.storage->_timer_tick();

    return true;
}

bool CalibrationStore::has_calibration()
{
    SensorCalibration cal;
    return load(cal);
}

// ============================================================================
// Default Values
// ============================================================================

void CalibrationStore::get_defaults(SensorCalibration& cal)
{
    memset(&cal, 0, sizeof(cal));

    cal.magic = kCalibrationMagic;
    cal.version = kCalibrationVersion;
    cal.flags = 0;  // No sensors calibrated

    // Accelerometer defaults (identity)
    cal.accel.offset[0] = 0.0f;
    cal.accel.offset[1] = 0.0f;
    cal.accel.offset[2] = 0.0f;
    cal.accel.scale[0] = 1.0f;
    cal.accel.scale[1] = 1.0f;
    cal.accel.scale[2] = 1.0f;
    cal.accel.cross_axis[0] = 0.0f;
    cal.accel.cross_axis[1] = 0.0f;
    cal.accel.cross_axis[2] = 0.0f;

    // Gyroscope defaults
    cal.gyro.offset[0] = 0.0f;
    cal.gyro.offset[1] = 0.0f;
    cal.gyro.offset[2] = 0.0f;

    // Magnetometer defaults (identity)
    // Hard iron offset: zero
    cal.mag.offset[0] = 0.0f;
    cal.mag.offset[1] = 0.0f;
    cal.mag.offset[2] = 0.0f;
    // Soft iron diagonal: identity (1,1,1)
    cal.mag.diag[0] = 1.0f;
    cal.mag.diag[1] = 1.0f;
    cal.mag.diag[2] = 1.0f;
    // Soft iron off-diagonal: zero
    cal.mag.offdiag[0] = 0.0f;
    cal.mag.offdiag[1] = 0.0f;
    cal.mag.offdiag[2] = 0.0f;
    // Scale factor: 1.0
    cal.mag.scale_factor = 1.0f;

    // Barometer defaults
    cal.baro.pressure_offset = 0.0f;
    cal.baro.temp_offset = 0.0f;

    // Compute CRC
    cal.crc32 = compute_crc(cal);
}

// ============================================================================
// CRC Computation
// ============================================================================

uint32_t CalibrationStore::compute_crc(const SensorCalibration& cal)
{
    // CRC32 over all fields except the crc32 field itself
    // Using simple polynomial CRC (same as used in many embedded systems)

    const uint8_t* data = reinterpret_cast<const uint8_t*>(&cal);
    size_t length = offsetof(SensorCalibration, crc32);  // Exclude crc32 field

    uint32_t crc = 0xFFFFFFFF;

    for (size_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xEDB88320;  // IEEE 802.3 polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return ~crc;
}

}  // namespace rocketchip
