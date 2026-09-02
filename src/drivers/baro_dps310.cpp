// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// DPS310 from Infineon register map (no ruuvi in the link).
// Prior Art: IFXDS_DPS310 v1.1 compensation; Adafruit DPS310 Arduino.

#include "baro_dps310.h"
#include "i2c_master.h"
#include "rocketchip/isa_atmosphere.h"
#include "pico/time.h"
#include <math.h>
#include <stddef.h>

constexpr uint32_t kXferUs = kI2cMasterDefaultTimeoutUs;
constexpr uint8_t kRegPsr = 0x00;
constexpr uint8_t kRegTmp = 0x03;
constexpr uint8_t kRegPrsCfg = 0x06;
constexpr uint8_t kRegTmpCfg = 0x07;
constexpr uint8_t kRegMeasCfg = 0x08;
constexpr uint8_t kRegCfg = 0x09;
constexpr uint8_t kRegRst = 0x0C;
constexpr uint8_t kRegProdId = 0x0D;
constexpr uint8_t kRegCoef = 0x10;
constexpr uint8_t kCoefLen = 18;
constexpr uint8_t kSoftRst = 0x09;
constexpr uint8_t kProdIdMask = 0x0F;
constexpr uint8_t kProdIdDps310 = 0x00;  // IFXDS_DPS310 product nibble
constexpr uint8_t kModeContBoth = 0x07;
constexpr uint8_t kMeasSensorRdy = 0x40;
constexpr uint8_t kMeasCoefRdy = 0x80;
constexpr uint8_t kCfgShiftPrs = 0x04;
constexpr uint8_t kMrShift = 4;
constexpr uint8_t kTmpExt = 0x80;  // TMP_CFG MEMS source
constexpr uint8_t kStandby = 0x00;
constexpr uint8_t kRawBurst = 6;
constexpr uint8_t kCoefReadyTries = 50;
constexpr uint32_t kSoftRstWaitMs = 40;   // Infineon startup + coef window
constexpr uint32_t kCoefReadyPollMs = 10;
constexpr uint32_t kScaleOs1 = 524288;    // IFXDS OS=1
constexpr uint32_t kScaleOs8 = 7864320;   // IFXDS OS=8

static uint8_t g_addr = 0;
static bool g_ready = false;
static float g_seaLevelPa = rc::kStdAtmPressurePa;
static int32_t g_c0 = 0;
static int32_t g_c1 = 0;
static int32_t g_c00 = 0;
static int32_t g_c10 = 0;
static int32_t g_c01 = 0;
static int32_t g_c11 = 0;
static int32_t g_c20 = 0;
static int32_t g_c21 = 0;
static int32_t g_c30 = 0;


static int32_t twos(uint32_t value, uint8_t bits) {
    const uint32_t sign = 1U << (bits - 1U);
    if (value >= sign) {
        return static_cast<int32_t>(value) - static_cast<int32_t>(2U * sign);
    }
    return static_cast<int32_t>(value);
}

static uint8_t os_bits(uint8_t os) {
    switch (os) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        default: return 3;
    }
}

static uint8_t mr_bits(uint8_t mr) {
    switch (mr) {
        case 1: return 0;
        case 2: return 1;
        case 4: return 2;
        case 8: return 3;
        case 16: return 4;
        case 32: return 5;
        default: return 5;
    }
}

static bool write_reg(uint8_t reg, uint8_t val) {
    return i2c_master_write_reg(g_addr, reg, val, kXferUs) == 0;
}

static bool read_regs(uint8_t reg, uint8_t* data, size_t len) {
    return i2c_master_read_regs(g_addr, reg, data, len, kXferUs) ==
           static_cast<int>(len);
}

static bool load_coefs() {
    uint8_t c[kCoefLen];
    if (!read_regs(kRegCoef, c, kCoefLen)) {
        return false;
    }
    g_c0 = twos((static_cast<uint32_t>(c[0]) << 4) | (c[1] >> 4), 12);
    g_c1 = twos((static_cast<uint32_t>(c[1] & 0x0FU) << 8) | c[2], 12);
    g_c00 = twos((static_cast<uint32_t>(c[3]) << 12) | (static_cast<uint32_t>(c[4]) << 4) |
                 (c[5] >> 4), 20);
    g_c10 = twos((static_cast<uint32_t>(c[5] & 0x0FU) << 16) |
                 (static_cast<uint32_t>(c[6]) << 8) | c[7], 20);
    g_c01 = twos((static_cast<uint32_t>(c[8]) << 8) | c[9], 16);
    g_c11 = twos((static_cast<uint32_t>(c[10]) << 8) | c[11], 16);
    g_c20 = twos((static_cast<uint32_t>(c[12]) << 8) | c[13], 16);
    g_c21 = twos((static_cast<uint32_t>(c[14]) << 8) | c[15], 16);
    g_c30 = twos((static_cast<uint32_t>(c[16]) << 8) | c[17], 16);
    return true;
}

bool baro_dps310_init(uint8_t addr) {
    g_addr = addr;
    g_ready = false;
    if (!write_reg(kRegRst, kSoftRst)) {
        return false;
    }
    sleep_ms(kSoftRstWaitMs);
    uint8_t id = 0;
    if (i2c_master_read_reg(g_addr, kRegProdId, &id, kXferUs) != 0) {
        return false;
    }
    if ((id & kProdIdMask) != kProdIdDps310) {
        return false;
    }
    uint8_t meas = 0;
    bool coef_ready = false;
    for (uint8_t i = 0; i < kCoefReadyTries; i++) {
        if (i2c_master_read_reg(g_addr, kRegMeasCfg, &meas, kXferUs) == 0) {
            if ((meas & (kMeasSensorRdy | kMeasCoefRdy)) ==
                (kMeasSensorRdy | kMeasCoefRdy)) {
                coef_ready = true;
                break;
            }
        }
        sleep_ms(kCoefReadyPollMs);
    }
    if (!coef_ready) {
        return false;
    }
    if (!load_coefs()) {
        return false;
    }
    const uint8_t prs = static_cast<uint8_t>(
        (mr_bits(kBaroDps310PresMeasRate) << kMrShift) |
        os_bits(kBaroDps310PresOversampling));
    const uint8_t tmp = static_cast<uint8_t>(
        (mr_bits(kBaroDps310TempMeasRate) << kMrShift) |
        os_bits(kBaroDps310TempOversampling) | kTmpExt);
    if (!write_reg(kRegPrsCfg, prs)) {
        return false;
    }
    if (!write_reg(kRegTmpCfg, tmp)) {
        return false;
    }
    uint8_t cfg = 0;
    if (kBaroDps310PresOversampling > 8) {
        cfg = static_cast<uint8_t>(cfg | kCfgShiftPrs);
    }
    if (!write_reg(kRegCfg, cfg)) {
        return false;
    }
    g_ready = true;
    return true;
}

bool baro_dps310_ready() {
    return g_ready;
}

bool baro_dps310_start_continuous() {
    if (!g_ready) {
        return false;
    }
    return write_reg(kRegMeasCfg, kModeContBoth);
}

bool baro_dps310_stop() {
    return write_reg(kRegMeasCfg, kStandby);
}

bool baro_dps310_read(baro_dps310_data_t* data) {
    if (data == nullptr || !g_ready) {
        if (data != nullptr) {
            data->valid = false;
        }
        return false;
    }
    uint8_t raw[kRawBurst];
    if (!read_regs(kRegPsr, raw, kRawBurst)) {
        data->valid = false;
        return false;
    }
    int32_t praw = twos((static_cast<uint32_t>(raw[0]) << 16) |
                        (static_cast<uint32_t>(raw[1]) << 8) | raw[2], 24);
    int32_t traw = twos((static_cast<uint32_t>(raw[3]) << 16) |
                        (static_cast<uint32_t>(raw[4]) << 8) | raw[5], 24);
    const float tsc = static_cast<float>(traw) / static_cast<float>(kScaleOs1);
    const float psc = static_cast<float>(praw) / static_cast<float>(kScaleOs8);
    const float temp = (static_cast<float>(g_c0) * 0.5F) +
                       (static_cast<float>(g_c1) * tsc);
    const float pres = static_cast<float>(g_c00) +
                       psc * (static_cast<float>(g_c10) +
                              psc * (static_cast<float>(g_c20) +
                                     psc * static_cast<float>(g_c30))) +
                       tsc * static_cast<float>(g_c01) +
                       tsc * psc * (static_cast<float>(g_c11) +
                                    psc * static_cast<float>(g_c21));
    data->temperature_c = temp;
    data->pressure_pa = pres;
    data->altitude_m = baro_dps310_pressure_to_altitude(pres, g_seaLevelPa);
    data->valid = true;
    return true;
}

void baro_dps310_set_sea_level(float pressure_pa) {
    g_seaLevelPa = pressure_pa;
}

float baro_dps310_pressure_to_altitude(float pressure_pa, float sea_level_pa) {
    return rc::kHypsometricScale *
           (1.0F - powf(pressure_pa / sea_level_pa, rc::kHypsometricExponent));
}
