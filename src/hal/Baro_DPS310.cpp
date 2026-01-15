/**
 * @file Baro_DPS310.cpp
 * @brief DPS310 barometer driver wrapper implementation
 *
 * Bridges the Ruuvi DPS310 driver to RocketChip's SensorBus.
 * Uses the ruuvi driver functions for all sensor operations.
 */

#include "Baro_DPS310.h"
#include "Timing.h"
#include <cmath>
#include <cstring>
#include <cstdlib>

// Include DPS310 driver with C linkage
extern "C" {
#include "dps310.h"
}

namespace rocketchip {
namespace hal {

// ============================================================================
// Platform Callbacks (static, file-scope)
// ============================================================================

// We need to pass the SensorBus pointer through the comm_ctx
// These match the ruuvi driver's callback signatures

static uint32_t platformRead(const void* const comm_ctx, const uint8_t reg_addr,
                              uint8_t* const data, const uint8_t data_len)
{
    if (comm_ctx == nullptr || data == nullptr) {
        return 1;
    }

    SensorBus* bus = const_cast<SensorBus*>(static_cast<const SensorBus*>(comm_ctx));
    BusResult result = bus->readRegisters(reg_addr, data, static_cast<size_t>(data_len));

    return (result == BusResult::OK) ? 0 : 1;
}

static uint32_t platformWrite(const void* const comm_ctx, const uint8_t reg_addr,
                               const uint8_t* const data, const uint8_t data_len)
{
    if (comm_ctx == nullptr || data == nullptr) {
        return 1;
    }

    SensorBus* bus = const_cast<SensorBus*>(static_cast<const SensorBus*>(comm_ctx));
    BusResult result = bus->writeRegisters(reg_addr, data, static_cast<size_t>(data_len));

    return (result == BusResult::OK) ? 0 : 1;
}

static void platformSleep(const uint32_t ms)
{
    Timing::delayMs(ms);
}

// ============================================================================
// Context Creation Helper
// ============================================================================

// The ruuvi dps310_ctx_t has const members for function pointers and comm_ctx.
// We need to create a properly initialized context. Using malloc + memcpy
// is the cleanest way to handle const member initialization in C++.

static dps310_ctx_t* createContext(SensorBus* bus)
{
    // Allocate raw memory
    dps310_ctx_t* ctx = static_cast<dps310_ctx_t*>(malloc(sizeof(dps310_ctx_t)));
    if (ctx == nullptr) {
        return nullptr;
    }

    // Create a temporary initialized context
    // Using C99 compound literal with designated initializers
    dps310_ctx_t init_ctx = {
        .device_status = DPS310_NOT_INITIALIZED,
        .product_id = 0,
        .revision_id = 0,
        .temp_mr = DPS310_MR_1,
        .temp_osr = DPS310_OS_1,
        .pres_mr = DPS310_MR_1,
        .pres_osr = DPS310_OS_1,
        .c0 = 0, .c1 = 0, .c00 = 0, .c10 = 0,
        .c01 = 0, .c11 = 0, .c20 = 0, .c21 = 0, .c30 = 0,
        .last_temp_scal = 0.0f,
        .comm_ctx = bus,
        .sleep = platformSleep,
        .read = platformRead,
        .write = platformWrite
    };

    // Copy to allocated memory (this handles the const members)
    memcpy(ctx, &init_ctx, sizeof(dps310_ctx_t));

    return ctx;
}

// ============================================================================
// Constructor / Destructor
// ============================================================================

Baro_DPS310::Baro_DPS310(SensorBus* bus)
    : m_bus(bus)
    , m_ctx(nullptr)
    , m_initialized(false)
    , m_last_pressure(0.0f)
    , m_last_temperature(0.0f)
{
}

Baro_DPS310::~Baro_DPS310()
{
    if (m_ctx != nullptr) {
        dps310_uninit(m_ctx);
        free(m_ctx);
        m_ctx = nullptr;
    }
}

// ============================================================================
// Initialization
// ============================================================================

bool Baro_DPS310::begin()
{
    if (m_bus == nullptr) {
        return false;
    }

    // Initialize the bus
    if (!m_bus->begin()) {
        return false;
    }

    // Create the driver context with our platform callbacks
    m_ctx = createContext(m_bus);
    if (m_ctx == nullptr) {
        return false;
    }

    // Call the ruuvi driver initialization
    // This reads product ID, calibration coefficients, and sets up the sensor
    dps310_status_t status = dps310_init(m_ctx);
    if (status != DPS310_SUCCESS) {
        free(m_ctx);
        m_ctx = nullptr;
        return false;
    }

    // Configure default settings: 1 Hz rate, 16x oversampling for good precision
    status = dps310_config_temp(m_ctx, DPS310_MR_1, DPS310_OS_16);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    status = dps310_config_pres(m_ctx, DPS310_MR_1, DPS310_OS_16);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    m_initialized = true;
    return true;
}

bool Baro_DPS310::isConnected()
{
    if (m_ctx == nullptr) {
        return false;
    }
    return (m_ctx->device_status & DPS310_READY) != 0;
}

// ============================================================================
// Configuration
// ============================================================================

bool Baro_DPS310::configureTemperature(BaroRate rate, BaroOversample osr)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_mr_t mr = static_cast<dps310_mr_t>(rate);
    dps310_os_t os = static_cast<dps310_os_t>(osr);

    dps310_status_t status = dps310_config_temp(m_ctx, mr, os);
    return (status == DPS310_SUCCESS);
}

bool Baro_DPS310::configurePressure(BaroRate rate, BaroOversample osr)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_mr_t mr = static_cast<dps310_mr_t>(rate);
    dps310_os_t os = static_cast<dps310_os_t>(osr);

    dps310_status_t status = dps310_config_pres(m_ctx, mr, os);
    return (status == DPS310_SUCCESS);
}

// ============================================================================
// Data Reading
// ============================================================================

bool Baro_DPS310::read(float& pressure_pa, float& temp_c)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    // First read temperature (needed for pressure compensation)
    dps310_status_t status = dps310_measure_temp_once_sync(m_ctx, &temp_c);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    // Then read pressure
    status = dps310_measure_pres_once_sync(m_ctx, &pressure_pa);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    m_last_temperature = temp_c;
    m_last_pressure = pressure_pa;

    return true;
}

bool Baro_DPS310::readPressure(float& pressure_pa)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_status_t status = dps310_measure_pres_once_sync(m_ctx, &pressure_pa);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    m_last_pressure = pressure_pa;
    return true;
}

bool Baro_DPS310::readTemperature(float& temp_c)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_status_t status = dps310_measure_temp_once_sync(m_ctx, &temp_c);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    m_last_temperature = temp_c;
    return true;
}

// ============================================================================
// Continuous Mode
// ============================================================================

bool Baro_DPS310::startContinuous()
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_status_t status = dps310_measure_continuous_async(m_ctx);
    return (status == DPS310_SUCCESS);
}

bool Baro_DPS310::standby()
{
    if (m_ctx == nullptr) {
        return false;
    }

    dps310_status_t status = dps310_standby(m_ctx);
    return (status == DPS310_SUCCESS);
}

bool Baro_DPS310::getLastResult(float& temp_c, float& pressure_pa)
{
    if (m_ctx == nullptr || !m_initialized) {
        return false;
    }

    dps310_status_t status = dps310_get_last_result(m_ctx, &temp_c, &pressure_pa);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    m_last_temperature = temp_c;
    m_last_pressure = pressure_pa;
    return true;
}

// ============================================================================
// Utility
// ============================================================================

float Baro_DPS310::pressureToAltitude(float pressure_pa, float sea_level_pa)
{
    // Barometric formula: h = 44330 * (1 - (P/P0)^(1/5.255))
    return 44330.0f * (1.0f - powf(pressure_pa / sea_level_pa, 0.1903f));
}

} // namespace hal
} // namespace rocketchip
