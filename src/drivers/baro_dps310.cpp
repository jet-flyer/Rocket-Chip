// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
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

// Map integer oversampling/rate values to ruuvi driver enums.
static dps310_os_t os_from_int(uint8_t val) {
    switch (val) {
        case 1:   return DPS310_OS_1;
        case 2:   return DPS310_OS_2;
        case 4:   return DPS310_OS_4;
        case 8:   return DPS310_OS_8;
        case 16:  return DPS310_OS_16;
        case 32:  return DPS310_OS_32;
        case 64:  return DPS310_OS_64;
        case 128: return DPS310_OS_128;
        default:  return DPS310_OS_8;  // Fall back to safe default
    }
}

static dps310_mr_t mr_from_int(uint8_t val) {
    switch (val) {
        case 1:   return DPS310_MR_1;
        case 2:   return DPS310_MR_2;
        case 4:   return DPS310_MR_4;
        case 8:   return DPS310_MR_8;
        case 16:  return DPS310_MR_16;
        case 32:  return DPS310_MR_32;
        case 64:  return DPS310_MR_64;
        case 128: return DPS310_MR_128;
        default:  return DPS310_MR_8;  // Fall back to safe default
    }
}

// ============================================================================
// Private State
// ============================================================================

static uint8_t g_i2cAddr = 0;
static float g_seaLevelPa = kStdAtmPressurePa;

// Forward declarations for callbacks
// NOLINTBEGIN(readability-identifier-naming) — params match ruuvi dps310 callback typedef
static void pico_sleep(uint32_t ms);
static uint32_t pico_read(const void* comm_ctx, uint8_t reg_addr, uint8_t* data, uint8_t data_len);
static uint32_t pico_write(const void* comm_ctx, uint8_t reg_addr, const uint8_t* data, uint8_t data_len);
// NOLINTEND(readability-identifier-naming)

// DPS310 context - suppress missing-field-initializers for third-party struct
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
static dps310_ctx_t g_dps310Ctx = {
    .device_status = DPS310_NOT_INITIALIZED,
    .comm_ctx = &g_i2cAddr,
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

// NOLINTBEGIN(readability-identifier-naming) — params match ruuvi dps310 callback typedef
static uint32_t pico_read(const void* comm_ctx, uint8_t reg_addr, uint8_t* data, uint8_t data_len) {
    const auto* addr = static_cast<const uint8_t*>(comm_ctx);
    int ret = i2c_bus_read_regs(*addr, reg_addr, data, data_len);
    return (ret == data_len) ? 0 : DPS310_BUS_ERROR;
}

static uint32_t pico_write(const void* comm_ctx, uint8_t reg_addr, const uint8_t* data, uint8_t data_len) {
    const auto* addr = static_cast<const uint8_t*>(comm_ctx);

    // For single byte writes, use the simple register write
    if (data_len == 1) {
        int ret = i2c_bus_write_reg(*addr, reg_addr, data[0]);
        return (ret == 0) ? 0 : DPS310_BUS_ERROR;
    }

    // For multi-byte writes, need to prepend register address
    uint8_t buf[kMaxMultiByteWrite + 1];
    if (data_len > kMaxMultiByteWrite) {
        return DPS310_BUS_ERROR;
    }

    buf[0] = reg_addr;
    memcpy(&buf[1], data, data_len);

    int ret = i2c_bus_write(*addr, buf, data_len + 1);
    return (ret == (int)(data_len + 1)) ? 0 : DPS310_BUS_ERROR;
}
// NOLINTEND(readability-identifier-naming)

// ============================================================================
// Public API
// ============================================================================

bool baro_dps310_init(uint8_t addr) {
    g_i2cAddr = addr;

    // Initialize the ruuvi driver
    dps310_status_t status = dps310_init(&g_dps310Ctx);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    // Configure oversampling and measurement rate from header constants.
    // See baro_dps310.h for the full tradeoff table (precision vs latency vs power).
    const dps310_mr_t mr = mr_from_int(kBaroDps310MeasRate);
    const dps310_os_t os = os_from_int(kBaroDps310Oversampling);

    status = dps310_config_temp(&g_dps310Ctx, mr, os);
    if (status != DPS310_SUCCESS) {
        return false;
    }

    status = dps310_config_pres(&g_dps310Ctx, mr, os);
    return (status == DPS310_SUCCESS);
}

bool baro_dps310_ready() {
    return (g_dps310Ctx.device_status & DPS310_READY) != 0;
}

bool baro_dps310_start_continuous() {
    dps310_status_t status = dps310_measure_continuous_async(&g_dps310Ctx);
    return (status == DPS310_SUCCESS);
}

bool baro_dps310_stop() {
    dps310_status_t status = dps310_standby(&g_dps310Ctx);
    return (status == DPS310_SUCCESS);
}

bool baro_dps310_read(baro_dps310_data_t* data) {
    if (data == nullptr) {
        return false;
    }

    float temp = 0.0F;
    float pres = 0.0F;
    dps310_status_t status = dps310_get_last_result(&g_dps310Ctx, &temp, &pres);

    if (status != DPS310_SUCCESS) {
        data->valid = false;
        return false;
    }

    data->temperature_c = temp;
    data->pressure_pa = pres;
    data->altitude_m = baro_dps310_pressure_to_altitude(pres, g_seaLevelPa);
    data->valid = true;

    return true;
}

void baro_dps310_set_sea_level(float pressurePa) {
    g_seaLevelPa = pressurePa;
}

float baro_dps310_pressure_to_altitude(float pressurePa, float seaLevelPa) {
    // Barometric formula: h = 44330 * (1 - (P/P0)^0.1903)
    return kHypsometricScale * (1.0F - powf(pressurePa / seaLevelPa, kHypsometricExponent));
}
