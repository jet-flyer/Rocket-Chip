/**
 * @file IMU_ISM330DHCX.cpp
 * @brief ISM330DHCX driver wrapper implementation
 *
 * Bridges the ST platform-independent driver to RocketChip's SensorBus.
 */

#include "IMU_ISM330DHCX.h"
#include "Timing.h"

namespace rocketchip {
namespace hal {

// ============================================================================
// Platform Callbacks - Bridge ST driver to SensorBus
// ============================================================================

/**
 * @brief Platform read callback for ST driver
 *
 * The ST driver calls this function for all register reads. We cast the
 * handle back to our SensorBus pointer and forward the request.
 *
 * @param handle Pointer to SensorBus (passed via stmdev_ctx_t.handle)
 * @param reg Register address to read
 * @param data Buffer to store read data
 * @param len Number of bytes to read
 * @return 0 on success, non-zero on error
 */
int32_t IMU_ISM330DHCX::platformRead(void* handle, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (handle == nullptr) {
        return -1;
    }

    SensorBus* bus = static_cast<SensorBus*>(handle);
    BusResult result = bus->readRegisters(reg, data, static_cast<size_t>(len));

    return (result == BusResult::OK) ? 0 : -1;
}

/**
 * @brief Platform write callback for ST driver
 */
int32_t IMU_ISM330DHCX::platformWrite(void* handle, uint8_t reg, const uint8_t* data, uint16_t len)
{
    if (handle == nullptr) {
        return -1;
    }

    SensorBus* bus = static_cast<SensorBus*>(handle);

    // ST driver passes const data, but our interface takes non-const
    // This is safe - we're only reading from the buffer
    BusResult result = bus->writeRegisters(reg, data, static_cast<size_t>(len));

    return (result == BusResult::OK) ? 0 : -1;
}

/**
 * @brief Platform delay callback for ST driver
 */
void IMU_ISM330DHCX::platformDelay(uint32_t ms)
{
    Timing::delayMs(ms);
}

// ============================================================================
// Constructor / Initialization
// ============================================================================

IMU_ISM330DHCX::IMU_ISM330DHCX(SensorBus* bus)
    : m_bus(bus)
    , m_ctx{}
    , m_initialized(false)
    , m_accel_sensitivity(0.244f / 1000.0f)  // Default ±8g
    , m_gyro_sensitivity(35.0f / 1000.0f)    // Default ±1000dps
    , m_accel_range(AccelRange::RANGE_8G)
    , m_gyro_range(GyroRange::RANGE_1000DPS)
{
    // Wire up ST driver context to our callbacks
    m_ctx.write_reg = platformWrite;
    m_ctx.read_reg = platformRead;
    m_ctx.mdelay = platformDelay;
    m_ctx.handle = bus;  // Pass SensorBus pointer through to callbacks
}

bool IMU_ISM330DHCX::begin()
{
    if (m_bus == nullptr) {
        return false;
    }

    // Initialize the bus
    if (!m_bus->begin()) {
        return false;
    }

    // Verify device identity using ST driver function
    uint8_t who_am_i = 0;
    if (ism330dhcx_device_id_get(&m_ctx, &who_am_i) != 0) {
        return false;
    }
    if (who_am_i != DEVICE_ID) {
        return false;
    }

    // Perform software reset
    if (!softReset()) {
        return false;
    }

    // Wait for boot (datasheet: 10ms typical)
    Timing::delayMs(15);

    // Enable block data update (prevents reading partial samples)
    if (ism330dhcx_block_data_update_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    // Enable auto-increment for multi-byte reads
    if (ism330dhcx_auto_increment_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    // Configure accelerometer: ±8g at 416 Hz
    if (!setAccelRange(AccelRange::RANGE_8G)) {
        return false;
    }
    if (ism330dhcx_xl_data_rate_set(&m_ctx, ISM330DHCX_XL_ODR_416Hz) != 0) {
        return false;
    }

    // Configure gyroscope: ±1000 dps at 416 Hz
    if (!setGyroRange(GyroRange::RANGE_1000DPS)) {
        return false;
    }
    if (ism330dhcx_gy_data_rate_set(&m_ctx, ISM330DHCX_GY_ODR_416Hz) != 0) {
        return false;
    }

    m_initialized = true;
    return true;
}

bool IMU_ISM330DHCX::isConnected()
{
    uint8_t who_am_i = 0;
    if (ism330dhcx_device_id_get(&m_ctx, &who_am_i) != 0) {
        return false;
    }
    return (who_am_i == DEVICE_ID);
}

bool IMU_ISM330DHCX::softReset()
{
    if (ism330dhcx_reset_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    // Wait for reset to complete (self-clearing bit)
    uint8_t rst = 1;
    for (int i = 0; i < 10; i++) {
        Timing::delayMs(2);
        if (ism330dhcx_reset_get(&m_ctx, &rst) == 0 && rst == 0) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// Configuration
// ============================================================================

bool IMU_ISM330DHCX::setAccelRange(AccelRange range)
{
    ism330dhcx_fs_xl_t st_range = toStAccelRange(range);
    if (ism330dhcx_xl_full_scale_set(&m_ctx, st_range) != 0) {
        return false;
    }
    m_accel_range = range;
    updateAccelSensitivity(range);
    return true;
}

bool IMU_ISM330DHCX::setGyroRange(GyroRange range)
{
    ism330dhcx_fs_g_t st_range = toStGyroRange(range);
    if (ism330dhcx_gy_full_scale_set(&m_ctx, st_range) != 0) {
        return false;
    }
    m_gyro_range = range;
    updateGyroSensitivity(range);
    return true;
}

bool IMU_ISM330DHCX::setODR(ODR rate)
{
    ism330dhcx_odr_xl_t xl_rate = toStAccelODR(rate);
    ism330dhcx_odr_g_t g_rate = toStGyroODR(rate);

    if (ism330dhcx_xl_data_rate_set(&m_ctx, xl_rate) != 0) {
        return false;
    }
    if (ism330dhcx_gy_data_rate_set(&m_ctx, g_rate) != 0) {
        return false;
    }
    return true;
}

// ============================================================================
// Data Reading
// ============================================================================

bool IMU_ISM330DHCX::dataReady(bool& accel_ready, bool& gyro_ready)
{
    ism330dhcx_status_reg_t status;
    if (ism330dhcx_status_reg_get(&m_ctx, &status) != 0) {
        return false;
    }
    accel_ready = (status.xlda != 0);
    gyro_ready = (status.gda != 0);
    return true;
}

bool IMU_ISM330DHCX::read(Vector3f& accel, Vector3f& gyro)
{
    // Use ST driver's data read functions
    int16_t accel_raw[3];
    int16_t gyro_raw[3];

    if (ism330dhcx_acceleration_raw_get(&m_ctx, accel_raw) != 0) {
        return false;
    }
    if (ism330dhcx_angular_rate_raw_get(&m_ctx, gyro_raw) != 0) {
        return false;
    }

    // Convert to physical units
    accel.x = static_cast<float>(accel_raw[0]) * m_accel_sensitivity;
    accel.y = static_cast<float>(accel_raw[1]) * m_accel_sensitivity;
    accel.z = static_cast<float>(accel_raw[2]) * m_accel_sensitivity;

    gyro.x = static_cast<float>(gyro_raw[0]) * m_gyro_sensitivity;
    gyro.y = static_cast<float>(gyro_raw[1]) * m_gyro_sensitivity;
    gyro.z = static_cast<float>(gyro_raw[2]) * m_gyro_sensitivity;

    return true;
}

bool IMU_ISM330DHCX::readAccel(Vector3f& accel)
{
    int16_t raw[3];
    if (ism330dhcx_acceleration_raw_get(&m_ctx, raw) != 0) {
        return false;
    }

    accel.x = static_cast<float>(raw[0]) * m_accel_sensitivity;
    accel.y = static_cast<float>(raw[1]) * m_accel_sensitivity;
    accel.z = static_cast<float>(raw[2]) * m_accel_sensitivity;

    return true;
}

bool IMU_ISM330DHCX::readGyro(Vector3f& gyro)
{
    int16_t raw[3];
    if (ism330dhcx_angular_rate_raw_get(&m_ctx, raw) != 0) {
        return false;
    }

    gyro.x = static_cast<float>(raw[0]) * m_gyro_sensitivity;
    gyro.y = static_cast<float>(raw[1]) * m_gyro_sensitivity;
    gyro.z = static_cast<float>(raw[2]) * m_gyro_sensitivity;

    return true;
}

bool IMU_ISM330DHCX::readTemperature(float& temp_c)
{
    int16_t raw;
    if (ism330dhcx_temperature_raw_get(&m_ctx, &raw) != 0) {
        return false;
    }

    // Temperature sensor: 256 LSB/°C, 0 LSB at 25°C
    temp_c = (static_cast<float>(raw) / 256.0f) + 25.0f;
    return true;
}

// ============================================================================
// Internal Helpers
// ============================================================================

void IMU_ISM330DHCX::updateAccelSensitivity(AccelRange range)
{
    // Sensitivity values from datasheet (mg/LSB converted to g/LSB)
    switch (range) {
        case AccelRange::RANGE_2G:  m_accel_sensitivity = 0.061f / 1000.0f; break;
        case AccelRange::RANGE_4G:  m_accel_sensitivity = 0.122f / 1000.0f; break;
        case AccelRange::RANGE_8G:  m_accel_sensitivity = 0.244f / 1000.0f; break;
        case AccelRange::RANGE_16G: m_accel_sensitivity = 0.488f / 1000.0f; break;
        default: m_accel_sensitivity = 0.244f / 1000.0f; break;
    }
}

void IMU_ISM330DHCX::updateGyroSensitivity(GyroRange range)
{
    // Sensitivity values from datasheet (mdps/LSB converted to dps/LSB)
    switch (range) {
        case GyroRange::RANGE_125DPS:  m_gyro_sensitivity = 4.375f / 1000.0f;  break;
        case GyroRange::RANGE_250DPS:  m_gyro_sensitivity = 8.75f / 1000.0f;   break;
        case GyroRange::RANGE_500DPS:  m_gyro_sensitivity = 17.5f / 1000.0f;   break;
        case GyroRange::RANGE_1000DPS: m_gyro_sensitivity = 35.0f / 1000.0f;   break;
        case GyroRange::RANGE_2000DPS: m_gyro_sensitivity = 70.0f / 1000.0f;   break;
        case GyroRange::RANGE_4000DPS: m_gyro_sensitivity = 140.0f / 1000.0f;  break;
        default: m_gyro_sensitivity = 35.0f / 1000.0f; break;
    }
}

ism330dhcx_fs_xl_t IMU_ISM330DHCX::toStAccelRange(AccelRange range)
{
    switch (range) {
        case AccelRange::RANGE_2G:  return ISM330DHCX_2g;
        case AccelRange::RANGE_4G:  return ISM330DHCX_4g;
        case AccelRange::RANGE_8G:  return ISM330DHCX_8g;
        case AccelRange::RANGE_16G: return ISM330DHCX_16g;
        default: return ISM330DHCX_8g;
    }
}

ism330dhcx_fs_g_t IMU_ISM330DHCX::toStGyroRange(GyroRange range)
{
    switch (range) {
        case GyroRange::RANGE_125DPS:  return ISM330DHCX_125dps;
        case GyroRange::RANGE_250DPS:  return ISM330DHCX_250dps;
        case GyroRange::RANGE_500DPS:  return ISM330DHCX_500dps;
        case GyroRange::RANGE_1000DPS: return ISM330DHCX_1000dps;
        case GyroRange::RANGE_2000DPS: return ISM330DHCX_2000dps;
        case GyroRange::RANGE_4000DPS: return ISM330DHCX_4000dps;
        default: return ISM330DHCX_1000dps;
    }
}

ism330dhcx_odr_xl_t IMU_ISM330DHCX::toStAccelODR(ODR rate)
{
    switch (rate) {
        case ODR::ODR_OFF:     return ISM330DHCX_XL_ODR_OFF;
        case ODR::ODR_12_5HZ:  return ISM330DHCX_XL_ODR_12Hz5;
        case ODR::ODR_26HZ:    return ISM330DHCX_XL_ODR_26Hz;
        case ODR::ODR_52HZ:    return ISM330DHCX_XL_ODR_52Hz;
        case ODR::ODR_104HZ:   return ISM330DHCX_XL_ODR_104Hz;
        case ODR::ODR_208HZ:   return ISM330DHCX_XL_ODR_208Hz;
        case ODR::ODR_416HZ:   return ISM330DHCX_XL_ODR_416Hz;
        case ODR::ODR_833HZ:   return ISM330DHCX_XL_ODR_833Hz;
        case ODR::ODR_1666HZ:  return ISM330DHCX_XL_ODR_1666Hz;
        case ODR::ODR_3330HZ:  return ISM330DHCX_XL_ODR_3332Hz;
        case ODR::ODR_6660HZ:  return ISM330DHCX_XL_ODR_6667Hz;
        default: return ISM330DHCX_XL_ODR_416Hz;
    }
}

ism330dhcx_odr_g_t IMU_ISM330DHCX::toStGyroODR(ODR rate)
{
    switch (rate) {
        case ODR::ODR_OFF:     return ISM330DHCX_GY_ODR_OFF;
        case ODR::ODR_12_5HZ:  return ISM330DHCX_GY_ODR_12Hz5;
        case ODR::ODR_26HZ:    return ISM330DHCX_GY_ODR_26Hz;
        case ODR::ODR_52HZ:    return ISM330DHCX_GY_ODR_52Hz;
        case ODR::ODR_104HZ:   return ISM330DHCX_GY_ODR_104Hz;
        case ODR::ODR_208HZ:   return ISM330DHCX_GY_ODR_208Hz;
        case ODR::ODR_416HZ:   return ISM330DHCX_GY_ODR_416Hz;
        case ODR::ODR_833HZ:   return ISM330DHCX_GY_ODR_833Hz;
        case ODR::ODR_1666HZ:  return ISM330DHCX_GY_ODR_1666Hz;
        case ODR::ODR_3330HZ:  return ISM330DHCX_GY_ODR_3332Hz;
        case ODR::ODR_6660HZ:  return ISM330DHCX_GY_ODR_6667Hz;
        default: return ISM330DHCX_GY_ODR_416Hz;
    }
}

} // namespace hal
} // namespace rocketchip
