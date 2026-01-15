/**
 * @file Mag_LIS3MDL.cpp
 * @brief LIS3MDL magnetometer driver wrapper implementation
 *
 * Bridges the ST platform-independent driver to RocketChip's SensorBus.
 */

#include "Mag_LIS3MDL.h"
#include "Timing.h"

namespace rocketchip {
namespace hal {

// ============================================================================
// Platform Callbacks - Bridge ST driver to SensorBus
// ============================================================================

int32_t Mag_LIS3MDL::platformRead(void* handle, uint8_t reg, uint8_t* data, uint16_t len)
{
    if (handle == nullptr) {
        return -1;
    }

    SensorBus* bus = static_cast<SensorBus*>(handle);
    BusResult result = bus->readRegisters(reg, data, static_cast<size_t>(len));

    return (result == BusResult::OK) ? 0 : -1;
}

int32_t Mag_LIS3MDL::platformWrite(void* handle, uint8_t reg, const uint8_t* data, uint16_t len)
{
    if (handle == nullptr) {
        return -1;
    }

    SensorBus* bus = static_cast<SensorBus*>(handle);
    BusResult result = bus->writeRegisters(reg, data, static_cast<size_t>(len));

    return (result == BusResult::OK) ? 0 : -1;
}

void Mag_LIS3MDL::platformDelay(uint32_t ms)
{
    Timing::delayMs(ms);
}

// ============================================================================
// Constructor / Initialization
// ============================================================================

Mag_LIS3MDL::Mag_LIS3MDL(SensorBus* bus)
    : m_bus(bus)
    , m_ctx{}
    , m_initialized(false)
    , m_sensitivity(6842.0f)  // Default +/-4 gauss: 1/6842 gauss/LSB
    , m_range(MagRange::RANGE_4GAUSS)
{
    // Wire up ST driver context to our callbacks
    m_ctx.write_reg = platformWrite;
    m_ctx.read_reg = platformRead;
    m_ctx.mdelay = platformDelay;
    m_ctx.handle = bus;
}

bool Mag_LIS3MDL::begin()
{
    if (m_bus == nullptr) {
        return false;
    }

    // Initialize the bus
    if (!m_bus->begin()) {
        return false;
    }

    // Verify device identity
    uint8_t who_am_i = 0;
    if (lis3mdl_device_id_get(&m_ctx, &who_am_i) != 0) {
        return false;
    }
    if (who_am_i != DEVICE_ID) {
        return false;
    }

    // Perform software reset
    if (!softReset()) {
        return false;
    }

    // Wait for boot
    Timing::delayMs(10);

    // Enable block data update
    if (lis3mdl_block_data_update_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    // Set full-scale range: +/-4 gauss
    if (!setRange(MagRange::RANGE_4GAUSS)) {
        return false;
    }

    // Set data rate: 80 Hz, high performance mode
    if (!setDataRate(MagDataRate::HP_80Hz)) {
        return false;
    }

    // Set continuous conversion mode
    if (lis3mdl_operating_mode_set(&m_ctx, LIS3MDL_CONTINUOUS_MODE) != 0) {
        return false;
    }

    // Enable temperature sensor
    if (lis3mdl_temperature_meas_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    m_initialized = true;
    return true;
}

bool Mag_LIS3MDL::isConnected()
{
    uint8_t who_am_i = 0;
    if (lis3mdl_device_id_get(&m_ctx, &who_am_i) != 0) {
        return false;
    }
    return (who_am_i == DEVICE_ID);
}

bool Mag_LIS3MDL::softReset()
{
    if (lis3mdl_reset_set(&m_ctx, PROPERTY_ENABLE) != 0) {
        return false;
    }

    // Wait for reset to complete
    uint8_t rst = 1;
    for (int i = 0; i < 10; i++) {
        Timing::delayMs(2);
        if (lis3mdl_reset_get(&m_ctx, &rst) == 0 && rst == 0) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// Configuration
// ============================================================================

bool Mag_LIS3MDL::setRange(MagRange range)
{
    lis3mdl_fs_t st_range = toStRange(range);
    if (lis3mdl_full_scale_set(&m_ctx, st_range) != 0) {
        return false;
    }
    m_range = range;
    updateSensitivity(range);
    return true;
}

bool Mag_LIS3MDL::setDataRate(MagDataRate rate)
{
    lis3mdl_om_t st_rate = toStDataRate(rate);
    if (lis3mdl_data_rate_set(&m_ctx, st_rate) != 0) {
        return false;
    }
    return true;
}

bool Mag_LIS3MDL::enableTemperature(bool enable)
{
    return lis3mdl_temperature_meas_set(&m_ctx, enable ? PROPERTY_ENABLE : PROPERTY_DISABLE) == 0;
}

// ============================================================================
// Data Reading
// ============================================================================

bool Mag_LIS3MDL::dataReady()
{
    uint8_t ready = 0;
    if (lis3mdl_mag_data_ready_get(&m_ctx, &ready) != 0) {
        return false;
    }
    return ready != 0;
}

bool Mag_LIS3MDL::read(Vector3f& mag)
{
    int16_t raw[3];
    if (lis3mdl_magnetic_raw_get(&m_ctx, raw) != 0) {
        return false;
    }

    // Convert to gauss using sensitivity (LSB/gauss -> gauss = raw / sensitivity)
    mag.x = static_cast<float>(raw[0]) / m_sensitivity;
    mag.y = static_cast<float>(raw[1]) / m_sensitivity;
    mag.z = static_cast<float>(raw[2]) / m_sensitivity;

    return true;
}

bool Mag_LIS3MDL::readTemperature(float& temp_c)
{
    int16_t raw;
    if (lis3mdl_temperature_raw_get(&m_ctx, &raw) != 0) {
        return false;
    }

    // Temperature sensor: 8 LSB/C, offset at 25C
    temp_c = (static_cast<float>(raw) / 8.0f) + 25.0f;
    return true;
}

// ============================================================================
// Internal Helpers
// ============================================================================

void Mag_LIS3MDL::updateSensitivity(MagRange range)
{
    // Sensitivity in LSB/gauss from datasheet
    switch (range) {
        case MagRange::RANGE_4GAUSS:  m_sensitivity = 6842.0f; break;
        case MagRange::RANGE_8GAUSS:  m_sensitivity = 3421.0f; break;
        case MagRange::RANGE_12GAUSS: m_sensitivity = 2281.0f; break;
        case MagRange::RANGE_16GAUSS: m_sensitivity = 1711.0f; break;
        default: m_sensitivity = 6842.0f; break;
    }
}

lis3mdl_fs_t Mag_LIS3MDL::toStRange(MagRange range)
{
    switch (range) {
        case MagRange::RANGE_4GAUSS:  return LIS3MDL_4_GAUSS;
        case MagRange::RANGE_8GAUSS:  return LIS3MDL_8_GAUSS;
        case MagRange::RANGE_12GAUSS: return LIS3MDL_12_GAUSS;
        case MagRange::RANGE_16GAUSS: return LIS3MDL_16_GAUSS;
        default: return LIS3MDL_4_GAUSS;
    }
}

lis3mdl_om_t Mag_LIS3MDL::toStDataRate(MagDataRate rate)
{
    switch (rate) {
        // Low power mode rates
        case MagDataRate::LP_0_625Hz: return LIS3MDL_LP_Hz625;
        case MagDataRate::LP_1_25Hz:  return LIS3MDL_LP_1Hz25;
        case MagDataRate::LP_2_5Hz:   return LIS3MDL_LP_2Hz5;
        case MagDataRate::LP_5Hz:     return LIS3MDL_LP_5Hz;
        case MagDataRate::LP_10Hz:    return LIS3MDL_LP_10Hz;
        case MagDataRate::LP_20Hz:    return LIS3MDL_LP_20Hz;
        case MagDataRate::LP_40Hz:    return LIS3MDL_LP_40Hz;
        case MagDataRate::LP_80Hz:    return LIS3MDL_LP_80Hz;
        case MagDataRate::LP_1000Hz:  return LIS3MDL_LP_1kHz;
        // Medium performance mode rates
        case MagDataRate::MP_0_625Hz: return LIS3MDL_LP_Hz625;  // No direct MP_0.625Hz
        case MagDataRate::MP_1_25Hz:  return LIS3MDL_MP_1Hz25;
        case MagDataRate::MP_2_5Hz:   return LIS3MDL_MP_2Hz5;
        case MagDataRate::MP_5Hz:     return LIS3MDL_MP_5Hz;
        case MagDataRate::MP_10Hz:    return LIS3MDL_MP_10Hz;
        case MagDataRate::MP_20Hz:    return LIS3MDL_MP_20Hz;
        case MagDataRate::MP_40Hz:    return LIS3MDL_MP_40Hz;
        case MagDataRate::MP_80Hz:    return LIS3MDL_MP_80Hz;
        case MagDataRate::MP_560Hz:   return LIS3MDL_MP_560Hz;
        // High performance mode rates
        case MagDataRate::HP_0_625Hz: return LIS3MDL_LP_Hz625;  // No direct HP_0.625Hz
        case MagDataRate::HP_1_25Hz:  return LIS3MDL_HP_1Hz25;
        case MagDataRate::HP_2_5Hz:   return LIS3MDL_HP_2Hz5;
        case MagDataRate::HP_5Hz:     return LIS3MDL_HP_5Hz;
        case MagDataRate::HP_10Hz:    return LIS3MDL_HP_10Hz;
        case MagDataRate::HP_20Hz:    return LIS3MDL_HP_20Hz;
        case MagDataRate::HP_40Hz:    return LIS3MDL_HP_40Hz;
        case MagDataRate::HP_80Hz:    return LIS3MDL_HP_80Hz;
        case MagDataRate::HP_300Hz:   return LIS3MDL_HP_300Hz;
        // Ultra high performance mode rates
        case MagDataRate::UHP_0_625Hz: return LIS3MDL_LP_Hz625;  // No direct UHP_0.625Hz
        case MagDataRate::UHP_1_25Hz:  return LIS3MDL_UHP_1Hz25;
        case MagDataRate::UHP_2_5Hz:   return LIS3MDL_UHP_2Hz5;
        case MagDataRate::UHP_5Hz:     return LIS3MDL_UHP_5Hz;
        case MagDataRate::UHP_10Hz:    return LIS3MDL_UHP_10Hz;
        case MagDataRate::UHP_20Hz:    return LIS3MDL_UHP_20Hz;
        case MagDataRate::UHP_40Hz:    return LIS3MDL_UHP_40Hz;
        case MagDataRate::UHP_80Hz:    return LIS3MDL_UHP_80Hz;
        case MagDataRate::UHP_155Hz:   return LIS3MDL_UHP_155Hz;
        default: return LIS3MDL_HP_80Hz;
    }
}

} // namespace hal
} // namespace rocketchip
