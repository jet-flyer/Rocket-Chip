// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// ICM-20948 driver. Bypass mag at 0x0C (ArduPilot Invensensev2).
// Prior Art: DS-000189; AKD09916; SparkFun bank cache.

#include "icm20948.h"
#include "i2c_master.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

constexpr uint8_t kRegBankSel = 0x7F;
constexpr uint32_t kXferUs = kI2cMasterDefaultTimeoutUs;
constexpr uint32_t kResetSettleMs = 100;  // DS-000189 start-up 11-100 ms
constexpr uint32_t kWakeSettleMs = 50;
constexpr uint32_t kStepDelayMs = 10;
constexpr uint8_t kImuBurst = 14;
constexpr uint8_t kMagBurst = 9;
constexpr uint8_t kMagDivider = 10;
constexpr uint8_t kUnknownBank = 0xFF;
constexpr uint8_t kFsSelMask = 0xF9;
constexpr uint8_t kMagWiaTries = 10;
constexpr uint8_t kPwrMgmt2On = 0x00;
constexpr float kTempSensitivity = 333.87F;  // DS-000189
constexpr float kTempOffset = 21.0F;         // DS-000189
constexpr float kMagScale = 0.15F;           // AKD09916 0.15 uT/LSB
constexpr float kStdGravityMs2 = 9.80665F;   // ISO 80000-3
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0F;
constexpr float kGyroLsb250Dps = 131.0F;     // DS-000189
constexpr float kGyroLsb500Dps = 65.5F;
constexpr float kGyroLsb1000Dps = 32.8F;
constexpr float kGyroLsb2000Dps = 16.4F;
constexpr float kAccelLsb2G = 16384.0F;
constexpr float kAccelLsb4G = 8192.0F;
constexpr float kAccelLsb8G = 4096.0F;
constexpr float kAccelLsb16G = 2048.0F;

namespace bank0 {
    constexpr uint8_t kWhoAmI    = 0x00;
    constexpr uint8_t kUserCtrl  = 0x03;
    constexpr uint8_t kPwrMgmt1  = 0x06;
    constexpr uint8_t kPwrMgmt2  = 0x07;
    constexpr uint8_t kIntPinCfg = 0x0F;
    constexpr uint8_t kAccelXoutH = 0x2D;
}  // namespace bank0

namespace bank2 {
    constexpr uint8_t kGyroSmplrtDiv = 0x00;
    constexpr uint8_t kGyroConfig1   = 0x01;
    constexpr uint8_t kAccelConfig   = 0x14;
}  // namespace bank2

namespace bit {
    constexpr uint8_t kDeviceReset = (1U << 7);
    constexpr uint8_t kClkselAuto  = 0x01;
    constexpr uint8_t kI2cMstEn    = (1U << 5);
    constexpr uint8_t kI2cIfDis    = (1U << 4);
    constexpr uint8_t kI2cMstRst   = (1U << 1);
    constexpr uint8_t kBypassEn    = (1U << 1);
}  // namespace bit

namespace ak {
    constexpr uint8_t kWia2  = 0x01;
    constexpr uint8_t kSt1   = 0x10;
    constexpr uint8_t kCntl2 = 0x31;
    constexpr uint8_t kCntl3 = 0x32;
    constexpr uint8_t kModeCont100 = 0x08;
    constexpr uint8_t kModePwrDown = 0x00;
    constexpr uint8_t kSrst = 0x01;
    constexpr uint8_t kSt1Drdy = 0x01;
    constexpr uint8_t kSt2Hofl = 0x08;
}  // namespace ak

static const float kAccelScale[] = {
    kStdGravityMs2 / kAccelLsb2G,
    kStdGravityMs2 / kAccelLsb4G,
    kStdGravityMs2 / kAccelLsb8G,
    kStdGravityMs2 / kAccelLsb16G,
};

static const float kGyroScale[] = {
    kDegToRad / kGyroLsb250Dps,
    kDegToRad / kGyroLsb500Dps,
    kDegToRad / kGyroLsb1000Dps,
    kDegToRad / kGyroLsb2000Dps,
};

static uint8_t g_magDivCount = 0;

static bool select_bank(icm20948_t* dev, uint8_t bank) {
    if (dev->current_bank == bank) {
        return true;
    }
    if (i2c_master_write_reg(dev->addr, kRegBankSel,
                             static_cast<uint8_t>(bank << 4), kXferUs) != 0) {
        dev->current_bank = kUnknownBank;
        return false;
    }
    dev->current_bank = bank;
    return true;
}

static bool write_bank(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t val) {
    if (!select_bank(dev, bank)) {
        return false;
    }
    return i2c_master_write_reg(dev->addr, reg, val, kXferUs) == 0;
}

static bool read_bank(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t* val) {
    if (!select_bank(dev, bank)) {
        return false;
    }
    return i2c_master_read_reg(dev->addr, reg, val, kXferUs) == 0;
}

static bool enable_bypass(icm20948_t* dev) {
    // Write known-good USER_CTRL (I2C_IF_DIS=0). RMW of a 0xFF bus-error
    // byte can still leave SPI-only mode until VDD POR (DS-000189).
    if (!write_bank(dev, 0, bank0::kUserCtrl, 0x00)) {
        return false;
    }
    if (!write_bank(dev, 0, bank0::kIntPinCfg, bit::kBypassEn)) {
        return false;
    }
    sleep_ms(kStepDelayMs);
    return true;
}

static bool init_mag(icm20948_t* dev) {
    if (!enable_bypass(dev)) {
        return false;
    }
    if (i2c_master_write_reg(kI2cAddrAk09916, ak::kCntl3, ak::kSrst, kXferUs) != 0) {
        return false;
    }
    sleep_ms(kResetSettleMs);
    bool found = false;
    for (uint8_t i = 0; i < kMagWiaTries; i++) {
        uint8_t wia = 0;
        if (i2c_master_read_reg(kI2cAddrAk09916, ak::kWia2, &wia, kXferUs) == 0 &&
            wia == kAk09916WhoAmI) {
            found = true;
            break;
        }
        sleep_ms(kStepDelayMs);
    }
    if (!found) {
        return false;
    }
    (void)i2c_master_write_reg(kI2cAddrAk09916, ak::kCntl2, ak::kModePwrDown,
                               kXferUs);
    sleep_ms(1);
    if (i2c_master_write_reg(kI2cAddrAk09916, ak::kCntl2, ak::kModeCont100,
                             kXferUs) != 0) {
        return false;
    }
    dev->mag_initialized = true;
    dev->mag_scale = kMagScale;
    return true;
}

static bool reset_and_wake(icm20948_t* dev) {
    if (!select_bank(dev, 0)) {
        return false;
    }
    uint8_t who = 0;
    if (i2c_master_read_reg(dev->addr, bank0::kWhoAmI, &who, kXferUs) != 0) {
        return false;
    }
    if (who != kIcm20948WhoAmI) {
        return false;
    }
    // WHO_AM_I already ACKs: chip is an I2C slave. DEVICE_RESET on a warm
    // STEMMA (no nRESET) is how I2C_IF_DIS latches until USB POR.
    // DS-000189: CLKSEL=auto also clears SLEEP.
    if (!write_bank(dev, 0, bank0::kPwrMgmt1, bit::kClkselAuto)) {
        return false;
    }
    sleep_ms(kWakeSettleMs);
    return write_bank(dev, 0, bank0::kPwrMgmt2, kPwrMgmt2On);
}

bool icm20948_init(icm20948_t* dev, uint8_t addr) {
    if (dev == nullptr) {
        return false;
    }
    memset(dev, 0, sizeof(*dev));
    dev->addr = addr;
    dev->current_bank = kUnknownBank;
    if (!reset_and_wake(dev)) {
        return false;
    }
    dev->accel_fs = ICM20948_ACCEL_FS_4G;
    dev->gyro_fs = ICM20948_GYRO_FS_500DPS;
    uint8_t accel_cfg = 0;
    if (!read_bank(dev, 2, bank2::kAccelConfig, &accel_cfg)) {
        return false;
    }
    accel_cfg = static_cast<uint8_t>((accel_cfg & kFsSelMask) |
                                     static_cast<uint8_t>(dev->accel_fs << 1));
    if (!write_bank(dev, 2, bank2::kAccelConfig, accel_cfg)) {
        return false;
    }
    dev->accel_scale = kAccelScale[dev->accel_fs];
    uint8_t gyro_cfg = 0;
    if (!read_bank(dev, 2, bank2::kGyroConfig1, &gyro_cfg)) {
        return false;
    }
    gyro_cfg = static_cast<uint8_t>((gyro_cfg & kFsSelMask) |
                                    static_cast<uint8_t>(dev->gyro_fs << 1));
    if (!write_bank(dev, 2, bank2::kGyroConfig1, gyro_cfg)) {
        return false;
    }
    dev->gyro_scale = kGyroScale[dev->gyro_fs];
    (void)init_mag(dev);
    dev->initialized = true;
    return true;
}

bool icm20948_ready(icm20948_t* dev) {
    if (dev == nullptr || !dev->initialized) {
        return false;
    }
    uint8_t who = 0;
    if (!read_bank(dev, 0, bank0::kWhoAmI, &who)) {
        return false;
    }
    return who == kIcm20948WhoAmI;
}

static void parse_imu(const uint8_t* buf, icm20948_t* dev, icm20948_data_t* data) {
    const int16_t ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    const int16_t ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    const int16_t az = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    const int16_t gx = static_cast<int16_t>((buf[6] << 8) | buf[7]);
    const int16_t gy = static_cast<int16_t>((buf[8] << 8) | buf[9]);
    const int16_t gz = static_cast<int16_t>((buf[10] << 8) | buf[11]);
    const int16_t tr = static_cast<int16_t>((buf[12] << 8) | buf[13]);
    data->accel.x = static_cast<float>(ax) * dev->accel_scale;
    data->accel.y = static_cast<float>(ay) * dev->accel_scale;
    data->accel.z = static_cast<float>(az) * dev->accel_scale;
    data->gyro.x = static_cast<float>(gx) * dev->gyro_scale;
    data->gyro.y = static_cast<float>(gy) * dev->gyro_scale;
    data->gyro.z = static_cast<float>(gz) * dev->gyro_scale;
    data->temperature_c = (static_cast<float>(tr) / kTempSensitivity) + kTempOffset;
    data->accel_valid = true;
    data->gyro_valid = true;
}

static void read_mag(icm20948_t* dev, icm20948_data_t* data) {
    g_magDivCount++;
    data->mag_valid = false;
    if (!dev->mag_initialized || g_magDivCount < kMagDivider) {
        return;
    }
    g_magDivCount = 0;
    uint8_t buf[kMagBurst];
    if (i2c_master_read_regs(kI2cAddrAk09916, ak::kSt1, buf, kMagBurst, kXferUs) !=
        static_cast<int>(kMagBurst)) {
        return;
    }
    if ((buf[0] & ak::kSt1Drdy) == 0) {
        return;
    }
    if ((buf[8] & ak::kSt2Hofl) != 0) {
        return;
    }
    const int16_t mx = static_cast<int16_t>((buf[2] << 8) | buf[1]);
    const int16_t my = static_cast<int16_t>((buf[4] << 8) | buf[3]);
    const int16_t mz = static_cast<int16_t>((buf[6] << 8) | buf[5]);
    data->mag.x = static_cast<float>(mx) * dev->mag_scale;
    data->mag.y = static_cast<float>(my) * dev->mag_scale;
    data->mag.z = static_cast<float>(mz) * dev->mag_scale;
    data->mag_valid = true;
}

bool icm20948_ensure_awake(icm20948_t* dev) {
    if (dev == nullptr || !dev->initialized) {
        return false;
    }
    // Bank cache can say 0 while the chip is in bank 2 — 0x2D then reads
    // as zeros (I2C ACK, gravity-floor Z=). Force a real BANK_SEL write.
    dev->current_bank = kUnknownBank;
    if (!write_bank(dev, 0, bank0::kPwrMgmt1, bit::kClkselAuto)) {
        return false;
    }
    if (!write_bank(dev, 0, bank0::kPwrMgmt2, kPwrMgmt2On)) {
        return false;
    }
    return enable_bypass(dev);
}

bool icm20948_read(icm20948_t* dev, icm20948_data_t* data) {
    if (dev == nullptr || data == nullptr || !dev->initialized) {
        if (data != nullptr) {
            memset(data, 0, sizeof(*data));
        }
        return false;
    }
    // Always write BANK_SEL. Skipping it after a desync is the Z= path.
    dev->current_bank = kUnknownBank;
    if (!select_bank(dev, 0)) {
        data->accel_valid = false;
        data->gyro_valid = false;
        data->mag_valid = false;
        return false;
    }
    uint8_t buf[kImuBurst];
    if (i2c_master_read_regs(dev->addr, bank0::kAccelXoutH, buf, kImuBurst,
                             kXferUs) != static_cast<int>(kImuBurst)) {
        data->accel_valid = false;
        data->gyro_valid = false;
        data->mag_valid = false;
        return false;
    }
    parse_imu(buf, dev, data);
    read_mag(dev, data);
    return true;
}

bool icm20948_read_config_registers(icm20948_t* dev, uint8_t* accel_config,
                                    uint8_t* gyro_config1, uint8_t* gyro_smplrt) {
    if (dev == nullptr || accel_config == nullptr || gyro_config1 == nullptr ||
        gyro_smplrt == nullptr) {
        return false;
    }
    if (!read_bank(dev, 2, bank2::kAccelConfig, accel_config)) {
        return false;
    }
    if (!read_bank(dev, 2, bank2::kGyroConfig1, gyro_config1)) {
        return false;
    }
    return read_bank(dev, 2, bank2::kGyroSmplrtDiv, gyro_smplrt);
}
