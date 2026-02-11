/**
 * @file icm20948.cpp
 * @brief ICM-20948 9-axis IMU driver implementation
 *
 * Reference: ICM-20948 datasheet, AK09916 datasheet
 *
 * Note: ICM-20948 uses a bank-switching register architecture.
 * Registers 0x00-0x7F exist in each of 4 banks (0-3).
 *
 * AK09916 magnetometer is accessed via I2C bypass mode (BYPASS_EN=1),
 * which connects the internal auxiliary bus to the external I2C bus.
 * The AK09916 appears directly at address 0x0C. This eliminates the
 * ICM-20948's internal I2C master (and its bank-switching race, stall
 * after disable/enable, and fragile SLV0 configuration).
 * ArduPilot uses the same approach (AP_InertialSensor_Invensensev2.cpp).
 */

#include "icm20948.h"
#include "i2c_bus.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Register Definitions (constexpr — JSF AV Rules 29/30/31)
// ============================================================================

// Bank select register (same address in all banks)
constexpr uint8_t kRegBankSel = 0x7F;

namespace bank0 {
    constexpr uint8_t kWhoAmI           = 0x00;
    constexpr uint8_t kUserCtrl         = 0x03;
    constexpr uint8_t kLpConfig         = 0x05;
    constexpr uint8_t kPwrMgmt1         = 0x06;
    constexpr uint8_t kPwrMgmt2         = 0x07;
    constexpr uint8_t kIntPinCfg        = 0x0F;
    constexpr uint8_t kIntEnable        = 0x10;
    constexpr uint8_t kIntEnable1       = 0x11;
    constexpr uint8_t kIntStatus        = 0x19;
    constexpr uint8_t kIntStatus1       = 0x1A;
    constexpr uint8_t kAccelXoutH       = 0x2D;
    constexpr uint8_t kAccelXoutL       = 0x2E;
    constexpr uint8_t kAccelYoutH       = 0x2F;
    constexpr uint8_t kAccelYoutL       = 0x30;
    constexpr uint8_t kAccelZoutH       = 0x31;
    constexpr uint8_t kAccelZoutL       = 0x32;
    constexpr uint8_t kGyroXoutH        = 0x33;
    constexpr uint8_t kGyroXoutL        = 0x34;
    constexpr uint8_t kGyroYoutH        = 0x35;
    constexpr uint8_t kGyroYoutL        = 0x36;
    constexpr uint8_t kGyroZoutH        = 0x37;
    constexpr uint8_t kGyroZoutL        = 0x38;
    constexpr uint8_t kTempOutH         = 0x39;
    constexpr uint8_t kTempOutL         = 0x3A;
} // namespace bank0

namespace bank2 {
    constexpr uint8_t kGyroSmplrtDiv    = 0x00;
    constexpr uint8_t kGyroConfig1      = 0x01;
    constexpr uint8_t kGyroConfig2      = 0x02;
    constexpr uint8_t kAccelSmplrtDiv1  = 0x10;
    constexpr uint8_t kAccelSmplrtDiv2  = 0x11;
    constexpr uint8_t kAccelConfig      = 0x14;
    constexpr uint8_t kAccelConfig2     = 0x15;
} // namespace bank2

// bank3 namespace removed — bypass mode does not use the I2C master
// (Bank 3 contained SLV0/SLV4 config, I2C master clock, delay ctrl)

// ============================================================================
// Bit Definitions
// ============================================================================

namespace bit {
    // PWR_MGMT_1
    constexpr uint8_t kDeviceReset      = (1U << 7);
    constexpr uint8_t kSleep            = (1U << 6);
    constexpr uint8_t kLpEn             = (1U << 5);
    constexpr uint8_t kClkselAuto       = 0x01;

    // USER_CTRL
    constexpr uint8_t kI2cMstEn         = (1U << 5);
    constexpr uint8_t kI2cIfDis         = (1U << 4);
    constexpr uint8_t kI2cMstRst        = (1U << 1);

    // INT_PIN_CFG
    constexpr uint8_t kBypassEn         = (1U << 1);
} // namespace bit

// ============================================================================
// AK09916 Magnetometer Registers
// ============================================================================

namespace ak09916 {
    constexpr uint8_t kI2cAddr          = 0x0C;

    constexpr uint8_t kWia2             = 0x01;  // Device ID (should read 0x09)
    constexpr uint8_t kSt1              = 0x10;  // Status 1 (data ready)
    constexpr uint8_t kHxl              = 0x11;  // Mag X low byte
    constexpr uint8_t kHxh              = 0x12;
    constexpr uint8_t kHyl              = 0x13;
    constexpr uint8_t kHyh              = 0x14;
    constexpr uint8_t kHzl              = 0x15;
    constexpr uint8_t kHzh              = 0x16;
    constexpr uint8_t kSt2              = 0x18;  // Status 2 (overflow)
    constexpr uint8_t kCntl2            = 0x31;  // Control 2 (mode)
    constexpr uint8_t kCntl3            = 0x32;  // Control 3 (reset)

    // Status bits
    constexpr uint8_t kSt1Drdy          = (1U << 0);
    constexpr uint8_t kSt2Hofl          = (1U << 3);  // Magnetic overflow
} // namespace ak09916

// ============================================================================
// Scale Factors
// ============================================================================

// Accelerometer scale factors (LSB to m/s²)
// Sensitivity: 16384, 8192, 4096, 2048 LSB/g for ±2, ±4, ±8, ±16g
static const float kAccelScale[] = {
    9.80665F / 16384.0F,  // ±2g
    9.80665F / 8192.0F,   // ±4g
    9.80665F / 4096.0F,   // ±8g
    9.80665F / 2048.0F,   // ±16g
};

// Gyroscope scale factors (LSB to rad/s)
// Sensitivity: 131, 65.5, 32.8, 16.4 LSB/dps for ±250, ±500, ±1000, ±2000 dps
static const float kGyroScale[] = {
    (M_PI / 180.0F) / 131.0F,   // ±250 dps
    (M_PI / 180.0F) / 65.5F,    // ±500 dps
    (M_PI / 180.0F) / 32.8F,    // ±1000 dps
    (M_PI / 180.0F) / 16.4F,    // ±2000 dps
};

// Magnetometer scale: 0.15 µT/LSB (fixed for AK09916)
constexpr float kMagScale = 0.15F;

// Temperature conversion (ICM-20948 datasheet Section 11.1)
constexpr float kTempSensitivity = 333.87F;  // LSB/°C
constexpr float kTempOffset      = 21.0F;    // Room temperature offset (°C)

// FS_SEL bitmask: bits [2:1] in ACCEL_CONFIG / GYRO_CONFIG1
constexpr uint8_t kFsSelMask = 0xF9;  // Clear bits [2:1], preserve rest

// Read sizes
constexpr uint8_t kImuReadSize        = 14;  // ACCEL_XOUT_H through TEMP_OUT_L
constexpr uint8_t kAccelGyroTempSize  = 6;   // 3-axis × 2 bytes
constexpr uint8_t kMagReadSize        = 9;   // ST1 + 6 data + TMPS(dummy) + ST2

// AK09916 outputs at 100Hz in continuous mode. When icm20948_read() is called
// at ~1kHz, only every Nth call reads the mag to avoid unnecessary bus traffic
// through the bypass bridge. AK09916 datasheet: 100Hz mode = 10ms per sample.
constexpr uint8_t kMagReadDivider     = 10;  // Read mag every 10th call (~100Hz)

// Timing delays (datasheet minimums)
constexpr uint32_t kInitStepDelayMs   = 10;   // Between bank switches / config steps
constexpr uint32_t kResetSettleMs     = 100;  // Post-device-reset stabilization
constexpr uint32_t kWakeSettleMs      = 50;   // Post-wake from sleep
constexpr uint32_t kMagRetries        = 10;   // SparkFun WHO_AM_I retry count
constexpr uint32_t kMagInitRetries    = 3;    // Magnetometer init retry count

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Select register bank
 */
static bool select_bank(icm20948_t* dev, uint8_t bank) {
    return i2c_bus_write_reg(dev->addr, kRegBankSel, (bank << 4)) == 0;
}

/**
 * @brief Write to a register in a specific bank
 */
static bool write_bank_reg(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t value) {
    if (!select_bank(dev, bank)) {
        return false;
    }
    return i2c_bus_write_reg(dev->addr, reg, value) == 0;
}

/**
 * @brief Read from a register in a specific bank
 */
static bool read_bank_reg(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t* value) {
    if (!select_bank(dev, bank)) {
        return false;
    }
    return i2c_bus_read_reg(dev->addr, reg, value) == 0;
}

// Direct I2C read from AK09916 (bypass mode — device at 0x0C on external bus)
static bool mag_direct_read_reg(uint8_t reg, uint8_t* value) {
    return i2c_bus_read_reg(ak09916::kI2cAddr, reg, value) == 0;
}

// Direct I2C write to AK09916
static bool mag_direct_write_reg(uint8_t reg, uint8_t value) {
    return i2c_bus_write_reg(ak09916::kI2cAddr, reg, value) == 0;
}

// Enable I2C bypass mode — connects AK09916 to external I2C bus at 0x0C.
// Must disable I2C master FIRST (I2C_MST_EN=0), then set BYPASS_EN=1.
// Reversing this order causes the internal master and external host to
// both drive the auxiliary bus simultaneously (undefined behavior).
static bool enable_bypass_mode(icm20948_t* dev) {
    // Disable I2C master if it was enabled
    uint8_t userCtrl = 0;
    if (!read_bank_reg(dev, 0, bank0::kUserCtrl, &userCtrl)) {
        return false;
    }
    userCtrl &= ~bit::kI2cMstEn;
    if (!write_bank_reg(dev, 0, bank0::kUserCtrl, userCtrl)) {
        return false;
    }

    // Enable BYPASS_EN — connects AK09916 to external I2C bus
    uint8_t intPinCfg = 0;
    if (!read_bank_reg(dev, 0, bank0::kIntPinCfg, &intPinCfg)) {
        return false;
    }
    intPinCfg |= bit::kBypassEn;
    if (!write_bank_reg(dev, 0, bank0::kIntPinCfg, intPinCfg)) {
        return false;
    }

    sleep_ms(kInitStepDelayMs);
    return true;
}

// Reset AK09916, verify WHO_AM_I, set continuous 100Hz mode (all via direct I2C)
static bool configure_magnetometer(icm20948_t* dev) {
    (void)dev;  // dev not needed — AK09916 is on external bus at fixed address

    // Reset magnetometer via direct I2C
    if (!mag_direct_write_reg(ak09916::kCntl3, 0x01)) {
        return false;
    }
    sleep_ms(kResetSettleMs);

    // Verify WHO_AM_I with retry
    bool magFound = false;
    for (uint8_t tries = 0; tries < kMagRetries; tries++) {
        uint8_t wia2 = 0;
        if (mag_direct_read_reg(ak09916::kWia2, &wia2) && wia2 == kAk09916WhoAmI) {
            magFound = true;
            break;
        }
        sleep_ms(kInitStepDelayMs);
    }
    if (!magFound) {
        return false;
    }

    // Set continuous 100Hz mode (shutdown first per Adafruit pattern)
    if (!mag_direct_write_reg(ak09916::kCntl2, AK09916_MODE_POWER_DOWN)) {
        return false;
    }
    sleep_ms(1);
    if (!mag_direct_write_reg(ak09916::kCntl2, AK09916_MODE_CONT_100HZ)) {
        return false;
    }
    sleep_ms(kInitStepDelayMs);

    return true;
}

static bool init_magnetometer(icm20948_t* dev) {
    if (!enable_bypass_mode(dev)) {
        return false;
    }
    if (!configure_magnetometer(dev)) {
        return false;
    }

    dev->mag_initialized = true;
    dev->mag_mode = AK09916_MODE_CONT_100HZ;
    dev->mag_scale = kMagScale;
    return true;
}

// ============================================================================
// Public Functions
// ============================================================================

// Verify WHO_AM_I, reset device, and wake into active mode
static bool reset_and_wake(icm20948_t* dev) {
    uint8_t whoAmI = 0;
    if (!select_bank(dev, 0)) {
        return false;
    }
    if (i2c_bus_read_reg(dev->addr, bank0::kWhoAmI, &whoAmI) != 0) {
        return false;
    }
    if (whoAmI != kIcm20948WhoAmI) {
        return false;
    }

    if (!icm20948_reset(dev)) {
        return false;
    }
    sleep_ms(kResetSettleMs);

    // Wake up device, set auto clock source
    if (!write_bank_reg(dev, 0, bank0::kPwrMgmt1, bit::kClkselAuto)) {
        return false;
    }
    sleep_ms(kWakeSettleMs);

    // Enable all sensors
    return write_bank_reg(dev, 0, bank0::kPwrMgmt2, 0x00);
}

bool icm20948_init(icm20948_t* dev, uint8_t addr) {
    if (dev == nullptr) {
        return false;
    }

    memset(dev, 0, sizeof(icm20948_t));
    dev->addr = addr;

    if (!reset_and_wake(dev)) {
        return false;
    }

    // Set default ranges (per IVP-09: ±4g, ±500dps)
    dev->accel_fs = ICM20948_ACCEL_FS_4G;
    dev->gyro_fs = ICM20948_GYRO_FS_500DPS;

    if (!icm20948_set_accel_fs(dev, dev->accel_fs)) {
        return false;
    }
    if (!icm20948_set_gyro_fs(dev, dev->gyro_fs)) {
        return false;
    }

    // Initialize magnetometer (bypass mode) with retries
    for (uint8_t magAttempt = 0; magAttempt < kMagInitRetries; magAttempt++) {
        if (init_magnetometer(dev)) {
            break;
        }
        sleep_ms(kWakeSettleMs);
    }

    dev->initialized = true;
    return true;
}

bool icm20948_ready(icm20948_t* dev) {
    if (dev == nullptr || !dev->initialized) {
        return false;
    }

    uint8_t whoAmI = 0;
    if (!select_bank(dev, 0)) {
        return false;
    }
    if (i2c_bus_read_reg(dev->addr, bank0::kWhoAmI, &whoAmI) != 0) {
        return false;
    }

    return whoAmI == kIcm20948WhoAmI;
}

bool icm20948_reset(icm20948_t* dev) {
    if (dev == nullptr) {
        return false;
    }

    if (!write_bank_reg(dev, 0, bank0::kPwrMgmt1, bit::kDeviceReset)) {
        return false;
    }

    sleep_ms(kResetSettleMs);
    dev->initialized = false;
    // Device reset clears BYPASS_EN — icm20948_init() must be called
    // to restore bypass mode and mag functionality.
    dev->mag_initialized = false;
    return true;
}

bool icm20948_set_accel_fs(icm20948_t* dev, icm20948_accel_fs_t fs) {
    if (dev == nullptr) {
        return false;
    }

    uint8_t config = 0;
    if (!read_bank_reg(dev, 2, bank2::kAccelConfig, &config)) {
        return false;
    }

    config = (config & kFsSelMask) | (fs << 1);  // Bits [2:1] = FS_SEL
    if (!write_bank_reg(dev, 2, bank2::kAccelConfig, config)) {
        return false;
    }

    dev->accel_fs = fs;
    dev->accel_scale = kAccelScale[fs];

    return true;
}

bool icm20948_set_gyro_fs(icm20948_t* dev, icm20948_gyro_fs_t fs) {
    if (dev == nullptr) {
        return false;
    }

    uint8_t config = 0;
    if (!read_bank_reg(dev, 2, bank2::kGyroConfig1, &config)) {
        return false;
    }

    config = (config & kFsSelMask) | (fs << 1);  // Bits [2:1] = FS_SEL
    if (!write_bank_reg(dev, 2, bank2::kGyroConfig1, config)) {
        return false;
    }

    dev->gyro_fs = fs;
    dev->gyro_scale = kGyroScale[fs];

    return true;
}

bool icm20948_set_mag_mode(icm20948_t* dev, ak09916_mode_t mode) {
    if (dev == nullptr || !dev->mag_initialized) {
        return false;
    }

    if (!mag_direct_write_reg(ak09916::kCntl2, mode)) {
        return false;
    }
    dev->mag_mode = mode;
    return true;
}

bool icm20948_set_low_power(icm20948_t* dev, bool enable) {
    if (dev == nullptr || !dev->initialized) {
        return false;
    }

    uint8_t pwr = 0;
    if (!read_bank_reg(dev, 0, bank0::kPwrMgmt1, &pwr)) {
        return false;
    }

    if (enable) {
        pwr |= bit::kLpEn;
    } else {
        pwr &= ~bit::kLpEn;
    }

    return write_bank_reg(dev, 0, bank0::kPwrMgmt1, pwr);
}

bool icm20948_read(icm20948_t* dev, icm20948_data_t* data) {
    if (dev == nullptr || data == nullptr || !dev->initialized) {
        memset(data, 0, sizeof(icm20948_data_t));
        return false;
    }

    // Select bank 0 for sensor data
    if (!select_bank(dev, 0)) {
        data->accel_valid = false;
        data->gyro_valid = false;
        data->mag_valid = false;
        return false;
    }

    // Read 14 bytes: ACCEL_XOUT_H (0x2D) through TEMP_OUT_L (0x3A)
    uint8_t buf[kImuReadSize];
    if (i2c_bus_read_regs(dev->addr, bank0::kAccelXoutH, buf, sizeof(buf)) != sizeof(buf)) {
        data->accel_valid = false;
        data->gyro_valid = false;
        data->mag_valid = false;
        return false;
    }

    // Parse accel/gyro/temp from burst read buffer
    // Byte layout per ICM-20948 datasheet register map:
    //   [0..5]   accel XH/XL/YH/YL/ZH/ZL
    //   [6..11]  gyro  XH/XL/YH/YL/ZH/ZL
    //   [12..13] temp  H/L
    // NOLINTBEGIN(readability-magic-numbers) — buffer byte offsets per datasheet register map
    int16_t ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    int16_t az = static_cast<int16_t>((buf[4] << 8) | buf[5]);

    // int16_t sensor values fit in float 24-bit mantissa (max ±32768)
    data->accel.x = static_cast<float>(ax) * dev->accel_scale;
    data->accel.y = static_cast<float>(ay) * dev->accel_scale;
    data->accel.z = static_cast<float>(az) * dev->accel_scale;
    data->accel_valid = true;

    int16_t gx = static_cast<int16_t>((buf[6] << 8) | buf[7]);
    int16_t gy = static_cast<int16_t>((buf[8] << 8) | buf[9]);
    int16_t gz = static_cast<int16_t>((buf[10] << 8) | buf[11]);

    data->gyro.x = static_cast<float>(gx) * dev->gyro_scale;
    data->gyro.y = static_cast<float>(gy) * dev->gyro_scale;
    data->gyro.z = static_cast<float>(gz) * dev->gyro_scale;
    data->gyro_valid = true;

    int16_t tempRaw = static_cast<int16_t>((buf[12] << 8) | buf[13]);
    data->temperature_c = (static_cast<float>(tempRaw) / kTempSensitivity) + kTempOffset;

    // Read mag from AK09916 at 0x0C (bypass mode) at reduced rate.
    // AK09916 outputs at 100Hz — reading at 1kHz wastes 90% of bus time
    // on DRDY=0 results. Divider matches mag output rate.
    static uint8_t magDivCount = 0;
    magDivCount++;
    if (dev->mag_initialized && magDivCount >= kMagReadDivider) {
        magDivCount = 0;
        uint8_t magBuf[kMagReadSize];
        if (i2c_bus_read_regs(ak09916::kI2cAddr, ak09916::kSt1,
                              magBuf, sizeof(magBuf)) == sizeof(magBuf)) {
            uint8_t st1 = magBuf[0];
            uint8_t st2 = magBuf[8];

            if ((st1 & ak09916::kSt1Drdy) != 0 && (st2 & ak09916::kSt2Hofl) == 0) {
                int16_t mx = static_cast<int16_t>((magBuf[2] << 8) | magBuf[1]);  // Little-endian
                int16_t my = static_cast<int16_t>((magBuf[4] << 8) | magBuf[3]);
                int16_t mz = static_cast<int16_t>((magBuf[6] << 8) | magBuf[5]);

                data->mag.x = static_cast<float>(mx) * dev->mag_scale;
                data->mag.y = static_cast<float>(my) * dev->mag_scale;
                data->mag.z = static_cast<float>(mz) * dev->mag_scale;
                data->mag_valid = true;
            } else {
                data->mag_valid = false;
            }
        } else {
            data->mag_valid = false;
        }
    } else if (!dev->mag_initialized && magDivCount >= kMagReadDivider) {
        // Mag lost after device reset — attempt lazy re-init once per divider cycle.
        // init_magnetometer() re-enables bypass + configures AK09916 (~220ms).
        // On success, subsequent calls resume normal mag reads.
        magDivCount = 0;
        init_magnetometer(dev);
        data->mag.x = data->mag.y = data->mag.z = 0;
        data->mag_valid = false;
    }
    // NOLINTEND(readability-magic-numbers)

    return true;
}

bool icm20948_read_accel(icm20948_t* dev, icm20948_vec3_t* accel) {
    if (dev == nullptr || accel == nullptr || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) {
        return false;
    }

    uint8_t buf[kAccelGyroTempSize];
    if (i2c_bus_read_regs(dev->addr, bank0::kAccelXoutH, buf, sizeof(buf)) != sizeof(buf)) {
        return false;
    }

    // NOLINTBEGIN(readability-magic-numbers) — XH/XL/YH/YL/ZH/ZL byte pairs
    int16_t ax = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t ay = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    int16_t az = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    // NOLINTEND(readability-magic-numbers)

    // int16_t sensor values fit in float 24-bit mantissa (max ±32768)
    accel->x = static_cast<float>(ax) * dev->accel_scale;
    accel->y = static_cast<float>(ay) * dev->accel_scale;
    accel->z = static_cast<float>(az) * dev->accel_scale;

    return true;
}

bool icm20948_read_gyro(icm20948_t* dev, icm20948_vec3_t* gyro) {
    if (dev == nullptr || gyro == nullptr || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) {
        return false;
    }

    uint8_t buf[kAccelGyroTempSize];
    if (i2c_bus_read_regs(dev->addr, bank0::kGyroXoutH, buf, sizeof(buf)) != sizeof(buf)) {
        return false;
    }

    // NOLINTBEGIN(readability-magic-numbers) — XH/XL/YH/YL/ZH/ZL byte pairs
    int16_t gx = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    int16_t gy = static_cast<int16_t>((buf[2] << 8) | buf[3]);
    int16_t gz = static_cast<int16_t>((buf[4] << 8) | buf[5]);
    // NOLINTEND(readability-magic-numbers)

    // int16_t sensor values fit in float 24-bit mantissa (max ±32768)
    gyro->x = static_cast<float>(gx) * dev->gyro_scale;
    gyro->y = static_cast<float>(gy) * dev->gyro_scale;
    gyro->z = static_cast<float>(gz) * dev->gyro_scale;

    return true;
}

bool icm20948_read_mag(icm20948_t* dev, icm20948_vec3_t* mag) {
    if (dev == nullptr || mag == nullptr || !dev->initialized || !dev->mag_initialized) {
        return false;
    }

    // Read directly from AK09916 at 0x0C (bypass mode)
    uint8_t buf[kMagReadSize];
    if (i2c_bus_read_regs(ak09916::kI2cAddr, ak09916::kSt1, buf, sizeof(buf)) != sizeof(buf)) {
        return false;
    }

    // NOLINTBEGIN(readability-magic-numbers) — AK09916 register byte offsets
    uint8_t st1 = buf[0];
    uint8_t st2 = buf[8];

    if ((st1 & ak09916::kSt1Drdy) == 0 || (st2 & ak09916::kSt2Hofl) != 0) {
        return false;  // Not ready or overflow
    }

    int16_t mx = static_cast<int16_t>((buf[2] << 8) | buf[1]);  // Little-endian
    int16_t my = static_cast<int16_t>((buf[4] << 8) | buf[3]);
    int16_t mz = static_cast<int16_t>((buf[6] << 8) | buf[5]);
    // NOLINTEND(readability-magic-numbers)

    // int16_t sensor values fit in float 24-bit mantissa (max ±32768)
    mag->x = static_cast<float>(mx) * dev->mag_scale;
    mag->y = static_cast<float>(my) * dev->mag_scale;
    mag->z = static_cast<float>(mz) * dev->mag_scale;

    return true;
}

bool icm20948_read_temperature(icm20948_t* dev, float* temp_c) {
    if (dev == nullptr || temp_c == nullptr || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) {
        return false;
    }

    uint8_t buf[2];
    if (i2c_bus_read_regs(dev->addr, bank0::kTempOutH, buf, 2) != 2) {
        return false;
    }

    int16_t tempRaw = static_cast<int16_t>((buf[0] << 8) | buf[1]);
    *temp_c = (static_cast<float>(tempRaw) / kTempSensitivity) + kTempOffset;

    return true;
}

bool icm20948_dataReady(icm20948_t* dev, bool* accelReady, bool* gyroReady) {
    if (dev == nullptr || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) {
        return false;
    }

    uint8_t status = 0;
    if (i2c_bus_read_reg(dev->addr, bank0::kIntStatus1, &status) != 0) {
        return false;
    }

    // Bit 0 is RAW_DATA_0_RDY_INT (accel and gyro share this)
    bool dataReady = (status & 0x01) != 0;

    if (accelReady != nullptr) {
        *accelReady = dataReady;
    }
    if (gyroReady != nullptr) {
        *gyroReady = dataReady;
    }

    return true;
}

