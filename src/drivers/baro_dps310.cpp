/**
 * @file baro_dps310.cpp
 * @brief DPS310 Barometer wrapper using ruuvi.dps310.c library
 *
 * Implements Pico SDK I2C callbacks for the ruuvi DPS310 driver.
 */

#include "baro_dps310.h"
#include "i2c_bus.h"
extern "C" {
#include "dps310.h"
}
#include "pico/time.h"
#include <math.h>
#include <string.h>

// Atmospheric constants (barometric formula)
constexpr float kStdAtmPressurePa    = 101325.0F;  // Standard sea-level pressure (Pa)
constexpr float kHypsometricScale    = 44330.0F;    // Barometric formula coefficient
constexpr float kHypsometricExponent = 0.1903F;     // Barometric formula exponent (1/5.255)

// I2C write buffer limit
constexpr uint8_t kMaxMultiByteWrite = 16;

// ============================================================================
// Private State
// ============================================================================

static uint8_t g_i2c_addr = 0;
static float g_sea_level_pa = kStdAtmPressurePa;

// Forward declarations for callbacks
static void pico_sleep(uint32_t ms);
static uint32_t pico_read(const void* comm_ctx, uint8_t reg_addr, uint8_t* data, uint8_t data_len);
static uint32_t pico_write(const void* comm_ctx, uint8_t reg_addr, const uint8_t* data, uint8_t data_len);

// DPS310 context - suppress missing-field-initializers for third-party struct
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static dps310_ctx_t g_dps310_ctx = {
    .device_status = DPS310_NOT_INITIALIZED,
    .comm_ctx = &g_i2c_addr,
    .sleep = pico_sleep,
    .read = pico_read,
    .write = pico_write,
};
#pragma GCC diagnostic pop

// ============================================================================
// Pico SDK Callbacks for ruuvi DPS310
// ============================================================================

static void pico_sleep(uint32_t ms) {
    sleep_ms(ms);
}

static uint32_t pico_read(const void* comm_ctx, uint8_t reg_addr, uint8_t* data, uint8_t data_len) {
    const uint8_t* addr = (const uint8_t*)comm_ctx;
    int ret = i2c_bus_read_regs(*addr, reg_addr, data, data_len);
    return (ret == data_len) ? 0 : DPS310_BUS_ERROR;
}

static uint32_t pico_write(const void* comm_ctx, uint8_t reg_addr, const uint8_t* data, uint8_t data_len) {
    const uint8_t* addr = (const uint8_t*)comm_ctx;

    // For single byte writes, use the simple register write
    if (data_len == 1) {
        int ret = i2c_bus_write_reg(*addr, reg_addr, data[0]);
        return (ret == 0) ? 0 : DPS310_BUS_ERROR;
    }

    // For multi-byte writes, need to prepend register address
    uint8_t buf[kMaxMultiByteWrite + 1];
    if (data_len > kMaxMultiByteWrite) return DPS310_BUS_ERROR;

    buf[0] = reg_addr;
    memcpy(&buf[1], data, data_len);

    int ret = i2c_bus_write(*addr, buf, data_len + 1);
    return (ret == (int)(data_len + 1)) ? 0 : DPS310_BUS_ERROR;
}

// ============================================================================
// Public API
// ============================================================================

bool baro_dps310_init(uint8_t addr) {
    g_i2c_addr = addr;

    // Initialize the ruuvi driver
    dps310_status_t status = dps310_init(&g_dps310_ctx);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    // Configure for 8x oversampling, 8 Hz rate (good balance of precision/speed)
    status = dps310_config_temp(&g_dps310_ctx, DPS310_MR_8, DPS310_OS_8);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    status = dps310_config_pres(&g_dps310_ctx, DPS310_MR_8, DPS310_OS_8);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    return true;
}

bool baro_dps310_ready() {
    return (g_dps310_ctx.device_status & DPS310_READY) != 0;
}

bool baro_dps310_start_continuous() {
    dps310_status_t status = dps310_measure_continuous_async(&g_dps310_ctx);
    return (status == DPS310_SUCCESS);
}

bool baro_dps310_stop() {
    dps310_status_t status = dps310_standby(&g_dps310_ctx);
    return (status == DPS310_SUCCESS);
}

bool baro_dps310_read(baro_dps310_data_t* data) {
    if (data == nullptr) {
        return false;
    }

    float temp = 0, pres = 0;
    dps310_status_t status = dps310_get_last_result(&g_dps310_ctx, &temp, &pres);

    if (status != DPS310_SUCCESS) {
        data->valid = false;
        return false;
    }

    data->temperature_c = temp;
    data->pressure_pa = pres;
    data->altitude_m = baro_dps310_pressure_to_altitude(pres, g_sea_level_pa);
    data->valid = true;

    return true;
}

void baro_dps310_set_sea_level(float pressure_pa) {
    g_sea_level_pa = pressure_pa;
}

float baro_dps310_pressure_to_altitude(float pressure_pa, float sea_level_pa) {
    // Barometric formula: h = 44330 * (1 - (P/P0)^0.1903)
    return kHypsometricScale * (1.0F - powf(pressure_pa / sea_level_pa, kHypsometricExponent));
}
