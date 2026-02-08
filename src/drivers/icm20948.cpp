/**
 * @file icm20948.c
 * @brief ICM-20948 9-axis IMU driver implementation
 *
 * Reference: ICM-20948 datasheet, AK09916 datasheet
 *
 * Note: ICM-20948 uses a bank-switching register architecture.
 * Registers 0x00-0x7F exist in each of 4 banks (0-3).
 */

#include "icm20948.h"
#include "i2c_bus.h"
#include "pico/time.h"
#include <math.h>
#include <string.h>

// ============================================================================
// Register Definitions - Bank 0 (User Bank 0)
// ============================================================================

#define REG_BANK_SEL        0x7F    // Bank select (same address in all banks)

// Bank 0 registers
#define B0_WHO_AM_I         0x00
#define B0_USER_CTRL        0x03
#define B0_LP_CONFIG        0x05
#define B0_PWR_MGMT_1       0x06
#define B0_PWR_MGMT_2       0x07
#define B0_INT_PIN_CFG      0x0F
#define B0_INT_ENABLE       0x10
#define B0_INT_ENABLE_1     0x11
#define B0_INT_STATUS       0x19
#define B0_INT_STATUS_1     0x1A
#define B0_ACCEL_XOUT_H     0x2D
#define B0_ACCEL_XOUT_L     0x2E
#define B0_ACCEL_YOUT_H     0x2F
#define B0_ACCEL_YOUT_L     0x30
#define B0_ACCEL_ZOUT_H     0x31
#define B0_ACCEL_ZOUT_L     0x32
#define B0_GYRO_XOUT_H      0x33
#define B0_GYRO_XOUT_L      0x34
#define B0_GYRO_YOUT_H      0x35
#define B0_GYRO_YOUT_L      0x36
#define B0_GYRO_ZOUT_H      0x37
#define B0_GYRO_ZOUT_L      0x38
#define B0_TEMP_OUT_H       0x39
#define B0_TEMP_OUT_L       0x3A
#define B0_EXT_SLV_SENS_DATA_00 0x3B  // Magnetometer data comes here

// Bank 2 registers (accel/gyro config)
#define B2_GYRO_SMPLRT_DIV  0x00
#define B2_GYRO_CONFIG_1    0x01
#define B2_GYRO_CONFIG_2    0x02
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B2_ACCEL_CONFIG     0x14
#define B2_ACCEL_CONFIG_2   0x15

// Bank 3 registers (I2C master)
#define B3_I2C_MST_CTRL     0x01
#define B3_I2C_MST_DELAY_CTRL 0x02
#define B3_I2C_SLV0_ADDR    0x03
#define B3_I2C_SLV0_REG     0x04
#define B3_I2C_SLV0_CTRL    0x05
#define B3_I2C_SLV0_DO      0x06

// ============================================================================
// PWR_MGMT_1 bits
// ============================================================================

#define PWR_MGMT_1_DEVICE_RESET (1 << 7)
#define PWR_MGMT_1_SLEEP        (1 << 6)
#define PWR_MGMT_1_LP_EN        (1 << 5)
#define PWR_MGMT_1_CLKSEL_AUTO  0x01

// ============================================================================
// USER_CTRL bits
// ============================================================================

#define USER_CTRL_I2C_MST_EN    (1 << 5)
#define USER_CTRL_I2C_MST_RST   (1 << 1)
#define USER_CTRL_I2C_IF_DIS    (1 << 4)

// ============================================================================
// INT_PIN_CFG bits
// ============================================================================

#define INT_PIN_CFG_BYPASS_EN   (1 << 1)

// ============================================================================
// I2C_SLV_CTRL bits
// ============================================================================

#define I2C_SLV_EN              (1 << 7)
#define I2C_SLV_READ            (1 << 7)  // For SLV_ADDR register

// ============================================================================
// AK09916 Magnetometer Registers
// ============================================================================

#define AK09916_I2C_ADDR    0x0C

#define AK_WIA2             0x01    // Device ID (should read 0x09)
#define AK_ST1              0x10    // Status 1 (data ready)
#define AK_HXL              0x11    // Mag X low byte
#define AK_HXH              0x12
#define AK_HYL              0x13
#define AK_HYH              0x14
#define AK_HZL              0x15
#define AK_HZH              0x16
#define AK_ST2              0x18    // Status 2 (overflow)
#define AK_CNTL2            0x31    // Control 2 (mode)
#define AK_CNTL3            0x32    // Control 3 (reset)

#define AK_ST1_DRDY         (1 << 0)
#define AK_ST2_HOFL         (1 << 3)  // Magnetic overflow

// ============================================================================
// Scale Factors
// ============================================================================

// Accelerometer scale factors (LSB to m/s²)
// Sensitivity: 16384, 8192, 4096, 2048 LSB/g for ±2, ±4, ±8, ±16g
static const float kAccelScale[] = {
    9.80665f / 16384.0f,  // ±2g
    9.80665f / 8192.0f,   // ±4g
    9.80665f / 4096.0f,   // ±8g
    9.80665f / 2048.0f,   // ±16g
};

// Gyroscope scale factors (LSB to rad/s)
// Sensitivity: 131, 65.5, 32.8, 16.4 LSB/dps for ±250, ±500, ±1000, ±2000 dps
static const float kGyroScale[] = {
    (M_PI / 180.0f) / 131.0f,   // ±250 dps
    (M_PI / 180.0f) / 65.5f,    // ±500 dps
    (M_PI / 180.0f) / 32.8f,    // ±1000 dps
    (M_PI / 180.0f) / 16.4f,    // ±2000 dps
};

// Magnetometer scale: 0.15 µT/LSB (fixed for AK09916)
#define MAG_SCALE 0.15f

// ============================================================================
// Private Functions
// ============================================================================

/**
 * @brief Select register bank
 */
static bool select_bank(icm20948_t* dev, uint8_t bank) {
    return i2c_bus_write_reg(dev->addr, REG_BANK_SEL, (bank << 4)) == 0;
}

/**
 * @brief Write to a register in a specific bank
 */
static bool write_bank_reg(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t value) {
    if (!select_bank(dev, bank)) return false;
    return i2c_bus_write_reg(dev->addr, reg, value) == 0;
}

/**
 * @brief Read from a register in a specific bank
 */
static bool read_bank_reg(icm20948_t* dev, uint8_t bank, uint8_t reg, uint8_t* value) {
    if (!select_bank(dev, bank)) return false;
    return i2c_bus_read_reg(dev->addr, reg, value) == 0;
}

/**
 * @brief Write to magnetometer via I2C master
 */
static bool mag_write_reg(icm20948_t* dev, uint8_t reg, uint8_t value) {
    // Configure SLV0 for write
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_REG, reg)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_DO, value)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_CTRL, I2C_SLV_EN | 1)) return false;

    sleep_ms(1);  // Wait for transaction
    return true;
}

/**
 * @brief Configure magnetometer read via I2C master
 */
static bool mag_config_read(icm20948_t* dev, uint8_t reg, uint8_t len) {
    // Configure SLV0 for read
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_SLV_READ)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_REG, reg)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_CTRL, I2C_SLV_EN | len)) return false;
    return true;
}

/**
 * @brief Read a single register from magnetometer via I2C master
 */
static bool mag_read_reg(icm20948_t* dev, uint8_t reg, uint8_t* value) {
    // Configure SLV0 for a one-shot single-byte read
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_ADDR, AK09916_I2C_ADDR | I2C_SLV_READ)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_REG, reg)) return false;
    if (!write_bank_reg(dev, 3, B3_I2C_SLV0_CTRL, I2C_SLV_EN | 1)) return false;

    sleep_ms(1);  // Wait for I2C master transaction

    // Read result from EXT_SLV_SENS_DATA_00
    return read_bank_reg(dev, 0, B0_EXT_SLV_SENS_DATA_00, value);
}

/**
 * @brief Initialize the AK09916 magnetometer
 *
 * Sequence per Adafruit/SparkFun reference implementations:
 * 1. Clear BYPASS_EN (disconnect external bus from aux bus)
 * 2. Configure I2C master clock + P_NSR
 * 3. Enable I2C master
 * 4. Reset magnetometer
 * 5. Verify WHO_AM_I with retry loop
 * 6. Set continuous measurement mode
 * 7. Configure SLV0 for automatic reads
 */
static bool init_magnetometer(icm20948_t* dev) {
    // 1. Clear BYPASS_EN — required before enabling I2C master
    uint8_t int_pin_cfg;
    if (!read_bank_reg(dev, 0, B0_INT_PIN_CFG, &int_pin_cfg)) return false;
    int_pin_cfg &= ~INT_PIN_CFG_BYPASS_EN;
    if (!write_bank_reg(dev, 0, B0_INT_PIN_CFG, int_pin_cfg)) return false;

    // 2. Configure I2C master: CLK=7 (~345kHz), P_NSR=1 (stop between reads)
    // 0x17 = Adafruit/SparkFun standard value
    if (!write_bank_reg(dev, 3, B3_I2C_MST_CTRL, 0x17)) return false;

    // 3. Enable I2C master
    if (!write_bank_reg(dev, 0, B0_USER_CTRL, USER_CTRL_I2C_MST_EN)) return false;
    sleep_ms(10);

    // 4. Reset magnetometer
    if (!mag_write_reg(dev, AK_CNTL3, 0x01)) return false;
    sleep_ms(100);

    // 5. Verify WHO_AM_I with retry (per SparkFun: up to 10 attempts)
    bool mag_found = false;
    for (uint8_t tries = 0; tries < 10; tries++) {
        uint8_t wia2;
        if (mag_read_reg(dev, AK_WIA2, &wia2) && wia2 == AK09916_WHO_AM_I) {
            mag_found = true;
            break;
        }
        // Reset I2C master and retry
        if (!write_bank_reg(dev, 0, B0_USER_CTRL,
                            USER_CTRL_I2C_MST_EN | USER_CTRL_I2C_MST_RST)) return false;
        sleep_ms(10);
    }
    if (!mag_found) return false;

    // 6. Set continuous 100Hz mode (shutdown first per Adafruit pattern)
    if (!mag_write_reg(dev, AK_CNTL2, AK09916_MODE_POWER_DOWN)) return false;
    sleep_ms(1);
    if (!mag_write_reg(dev, AK_CNTL2, AK09916_MODE_CONT_100HZ)) return false;
    sleep_ms(10);

    // 7. Configure SLV0 for continuous reads: 9 bytes from ST1
    // ST1(1) + HXL/HXH/HYL/HYH/HZL/HZH(6) + dummy(1) + ST2(1)
    if (!mag_config_read(dev, AK_ST1, 9)) return false;

    dev->mag_initialized = true;
    dev->mag_mode = AK09916_MODE_CONT_100HZ;
    dev->mag_scale = MAG_SCALE;

    return true;
}

// ============================================================================
// Public Functions
// ============================================================================

bool icm20948_init(icm20948_t* dev, uint8_t addr) {
    if (dev == NULL) {
        return false;
    }

    memset(dev, 0, sizeof(icm20948_t));
    dev->addr = addr;

    // Check WHO_AM_I
    uint8_t who_am_i;
    if (!select_bank(dev, 0)) return false;
    if (i2c_bus_read_reg(addr, B0_WHO_AM_I, &who_am_i) != 0) {
        return false;
    }

    if (who_am_i != ICM20948_WHO_AM_I) {
        return false;
    }

    // Reset device
    if (!icm20948_reset(dev)) {
        return false;
    }
    sleep_ms(100);

    // Wake up device, set auto clock source
    if (!write_bank_reg(dev, 0, B0_PWR_MGMT_1, PWR_MGMT_1_CLKSEL_AUTO)) return false;
    sleep_ms(50);

    // Enable all sensors
    if (!write_bank_reg(dev, 0, B0_PWR_MGMT_2, 0x00)) return false;

    // Set default ranges (per IVP-09: ±4g, ±500dps)
    dev->accel_fs = ICM20948_ACCEL_FS_4G;
    dev->gyro_fs = ICM20948_GYRO_FS_500DPS;

    if (!icm20948_set_accel_fs(dev, dev->accel_fs)) return false;
    if (!icm20948_set_gyro_fs(dev, dev->gyro_fs)) return false;

    // Initialize magnetometer
    // Initialize magnetometer with retries (intermittent after reboot)
    for (uint8_t mag_attempt = 0; mag_attempt < 3; mag_attempt++) {
        if (init_magnetometer(dev)) {
            break;
        }
        // Reset I2C master and retry
        write_bank_reg(dev, 0, B0_USER_CTRL,
                       USER_CTRL_I2C_MST_EN | USER_CTRL_I2C_MST_RST);
        sleep_ms(50);
    }
    if (!dev->mag_initialized) {
        // Mag init failure is not fatal - continue without mag
        dev->mag_initialized = false;
    }

    dev->initialized = true;
    return true;
}

bool icm20948_ready(icm20948_t* dev) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }

    uint8_t who_am_i;
    if (!select_bank(dev, 0)) return false;
    if (i2c_bus_read_reg(dev->addr, B0_WHO_AM_I, &who_am_i) != 0) {
        return false;
    }

    return who_am_i == ICM20948_WHO_AM_I;
}

bool icm20948_reset(icm20948_t* dev) {
    if (dev == NULL) {
        return false;
    }

    if (!write_bank_reg(dev, 0, B0_PWR_MGMT_1, PWR_MGMT_1_DEVICE_RESET)) {
        return false;
    }

    sleep_ms(100);
    dev->initialized = false;
    dev->mag_initialized = false;
    return true;
}

bool icm20948_set_accel_fs(icm20948_t* dev, icm20948_accel_fs_t fs) {
    if (dev == NULL) {
        return false;
    }

    uint8_t config;
    if (!read_bank_reg(dev, 2, B2_ACCEL_CONFIG, &config)) return false;

    config = (config & 0xF9) | (fs << 1);  // Bits [2:1] = FS_SEL
    if (!write_bank_reg(dev, 2, B2_ACCEL_CONFIG, config)) return false;

    dev->accel_fs = fs;
    dev->accel_scale = kAccelScale[fs];

    return true;
}

bool icm20948_set_gyro_fs(icm20948_t* dev, icm20948_gyro_fs_t fs) {
    if (dev == NULL) {
        return false;
    }

    uint8_t config;
    if (!read_bank_reg(dev, 2, B2_GYRO_CONFIG_1, &config)) return false;

    config = (config & 0xF9) | (fs << 1);  // Bits [2:1] = FS_SEL
    if (!write_bank_reg(dev, 2, B2_GYRO_CONFIG_1, config)) return false;

    dev->gyro_fs = fs;
    dev->gyro_scale = kGyroScale[fs];

    return true;
}

bool icm20948_set_mag_mode(icm20948_t* dev, ak09916_mode_t mode) {
    if (dev == NULL || !dev->mag_initialized) {
        return false;
    }

    if (!mag_write_reg(dev, AK_CNTL2, mode)) return false;
    dev->mag_mode = mode;
    return true;
}

bool icm20948_set_low_power(icm20948_t* dev, bool enable) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }

    uint8_t pwr;
    if (!read_bank_reg(dev, 0, B0_PWR_MGMT_1, &pwr)) return false;

    if (enable) {
        pwr |= PWR_MGMT_1_LP_EN;
    } else {
        pwr &= ~PWR_MGMT_1_LP_EN;
    }

    return write_bank_reg(dev, 0, B0_PWR_MGMT_1, pwr);
}

bool icm20948_read(icm20948_t* dev, icm20948_data_t* data) {
    if (dev == NULL || data == NULL || !dev->initialized) {
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

    // Read accel, gyro, temp, and ext sensor data in one burst
    // ACCEL_XOUT_H (0x2D) through EXT_SLV_SENS_DATA_08 (0x43) = 23 bytes
    // 14 bytes accel/gyro/temp + 9 bytes mag (ST1 + 6 data + dummy + ST2)
    uint8_t buf[23];
    if (i2c_bus_read_regs(dev->addr, B0_ACCEL_XOUT_H, buf, sizeof(buf)) != sizeof(buf)) {
        data->accel_valid = false;
        data->gyro_valid = false;
        data->mag_valid = false;
        return false;
    }

    // Parse accelerometer (bytes 0-5)
    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    data->accel.x = ax * dev->accel_scale;
    data->accel.y = ay * dev->accel_scale;
    data->accel.z = az * dev->accel_scale;
    data->accel_valid = true;

    // Parse gyroscope (bytes 6-11)
    int16_t gx = (int16_t)((buf[6] << 8) | buf[7]);
    int16_t gy = (int16_t)((buf[8] << 8) | buf[9]);
    int16_t gz = (int16_t)((buf[10] << 8) | buf[11]);

    data->gyro.x = gx * dev->gyro_scale;
    data->gyro.y = gy * dev->gyro_scale;
    data->gyro.z = gz * dev->gyro_scale;
    data->gyro_valid = true;

    // Parse temperature (bytes 12-13)
    int16_t temp_raw = (int16_t)((buf[12] << 8) | buf[13]);
    // Temperature formula from datasheet: Temp_degC = ((TEMP_OUT - RoomTemp_Offset) / Temp_Sensitivity) + 21
    // Room temp offset and sensitivity vary; using typical values
    data->temperature_c = (temp_raw / 333.87f) + 21.0f;

    // Parse magnetometer from EXT_SLV_SENS_DATA (bytes 14-22)
    // Format: ST1, HXL, HXH, HYL, HYH, HZL, HZH, dummy(0x17), ST2
    if (dev->mag_initialized) {
        uint8_t st1 = buf[14];
        uint8_t st2 = buf[22];  // ST2 at offset 8 (after dummy byte)

        if ((st1 & AK_ST1_DRDY) && !(st2 & AK_ST2_HOFL)) {
            // Data ready and no overflow
            int16_t mx = (int16_t)((buf[16] << 8) | buf[15]);  // Little-endian
            int16_t my = (int16_t)((buf[18] << 8) | buf[17]);
            int16_t mz = (int16_t)((buf[20] << 8) | buf[19]);

            data->mag.x = mx * dev->mag_scale;
            data->mag.y = my * dev->mag_scale;
            data->mag.z = mz * dev->mag_scale;
            data->mag_valid = true;
        } else {
            data->mag_valid = false;
        }
    } else {
        data->mag.x = data->mag.y = data->mag.z = 0;
        data->mag_valid = false;
    }

    return true;
}

bool icm20948_read_accel(icm20948_t* dev, icm20948_vec3_t* accel) {
    if (dev == NULL || accel == NULL || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) return false;

    uint8_t buf[6];
    if (i2c_bus_read_regs(dev->addr, B0_ACCEL_XOUT_H, buf, 6) != 6) {
        return false;
    }

    int16_t ax = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t ay = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t az = (int16_t)((buf[4] << 8) | buf[5]);

    accel->x = ax * dev->accel_scale;
    accel->y = ay * dev->accel_scale;
    accel->z = az * dev->accel_scale;

    return true;
}

bool icm20948_read_gyro(icm20948_t* dev, icm20948_vec3_t* gyro) {
    if (dev == NULL || gyro == NULL || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) return false;

    uint8_t buf[6];
    if (i2c_bus_read_regs(dev->addr, B0_GYRO_XOUT_H, buf, 6) != 6) {
        return false;
    }

    int16_t gx = (int16_t)((buf[0] << 8) | buf[1]);
    int16_t gy = (int16_t)((buf[2] << 8) | buf[3]);
    int16_t gz = (int16_t)((buf[4] << 8) | buf[5]);

    gyro->x = gx * dev->gyro_scale;
    gyro->y = gy * dev->gyro_scale;
    gyro->z = gz * dev->gyro_scale;

    return true;
}

bool icm20948_read_mag(icm20948_t* dev, icm20948_vec3_t* mag) {
    if (dev == NULL || mag == NULL || !dev->initialized || !dev->mag_initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) return false;

    // Read from external sensor data registers
    // 9 bytes: ST1 + 6 data + dummy + ST2
    uint8_t buf[9];
    if (i2c_bus_read_regs(dev->addr, B0_EXT_SLV_SENS_DATA_00, buf, 9) != 9) {
        return false;
    }

    uint8_t st1 = buf[0];
    uint8_t st2 = buf[8];  // ST2 after dummy byte

    if (!(st1 & AK_ST1_DRDY) || (st2 & AK_ST2_HOFL)) {
        return false;  // Not ready or overflow
    }

    int16_t mx = (int16_t)((buf[2] << 8) | buf[1]);  // Little-endian
    int16_t my = (int16_t)((buf[4] << 8) | buf[3]);
    int16_t mz = (int16_t)((buf[6] << 8) | buf[5]);

    mag->x = mx * dev->mag_scale;
    mag->y = my * dev->mag_scale;
    mag->z = mz * dev->mag_scale;

    return true;
}

bool icm20948_read_temperature(icm20948_t* dev, float* temp_c) {
    if (dev == NULL || temp_c == NULL || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) return false;

    uint8_t buf[2];
    if (i2c_bus_read_regs(dev->addr, B0_TEMP_OUT_H, buf, 2) != 2) {
        return false;
    }

    int16_t temp_raw = (int16_t)((buf[0] << 8) | buf[1]);
    *temp_c = (temp_raw / 333.87f) + 21.0f;

    return true;
}

bool icm20948_data_ready(icm20948_t* dev, bool* accel_ready, bool* gyro_ready) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }

    if (!select_bank(dev, 0)) return false;

    uint8_t status;
    if (i2c_bus_read_reg(dev->addr, B0_INT_STATUS_1, &status) != 0) {
        return false;
    }

    // Bit 0 is RAW_DATA_0_RDY_INT (accel and gyro share this)
    bool data_ready = (status & 0x01) != 0;

    if (accel_ready) *accel_ready = data_ready;
    if (gyro_ready) *gyro_ready = data_ready;

    return true;
}

bool icm20948_set_i2c_master_enable(icm20948_t* dev, bool enable) {
    if (dev == NULL || !dev->initialized) {
        return false;
    }

    // Read current USER_CTRL, modify I2C_MST_EN bit
    uint8_t user_ctrl;
    if (!read_bank_reg(dev, 0, B0_USER_CTRL, &user_ctrl)) return false;

    if (enable) {
        user_ctrl |= USER_CTRL_I2C_MST_EN;
    } else {
        user_ctrl &= ~USER_CTRL_I2C_MST_EN;
    }

    return write_bank_reg(dev, 0, B0_USER_CTRL, user_ctrl);
}
