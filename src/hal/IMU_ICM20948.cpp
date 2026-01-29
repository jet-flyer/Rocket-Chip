/**
 * @file IMU_ICM20948.cpp
 * @brief ICM-20948 9-DoF IMU driver implementation
 */

#include "IMU_ICM20948.h"
#include "pico/stdlib.h"
#include <cstdio>

namespace rocketchip {
namespace hal {

// Sensitivity values from ICM-20948 datasheet
static constexpr float kAccelSensitivity2G  = 16384.0f;  // LSB/g
static constexpr float kAccelSensitivity4G  = 8192.0f;
static constexpr float kAccelSensitivity8G  = 4096.0f;
static constexpr float kAccelSensitivity16G = 2048.0f;

static constexpr float kGyroSensitivity250  = 131.0f;    // LSB/(deg/s)
static constexpr float kGyroSensitivity500  = 65.5f;
static constexpr float kGyroSensitivity1000 = 32.8f;
static constexpr float kGyroSensitivity2000 = 16.4f;

IMU_ICM20948::IMU_ICM20948(SensorBus* bus)
    : m_bus(bus)
    , m_initialized(false)
    , m_currentBank(0xFF)  // Invalid, force bank switch on first access
    , m_accel_sensitivity(1.0f / kAccelSensitivity8G)
    , m_gyro_sensitivity(1.0f / kGyroSensitivity1000)
    , m_accel_range(AccelRange::RANGE_8G)
    , m_gyro_range(GyroRange::RANGE_1000DPS)
{
}

bool IMU_ICM20948::begin() {
    if (!m_bus) {
        printf("ICM20948: No bus\n");
        return false;
    }

    // Initialize the bus first (match DPS310 pattern)
    if (!m_bus->begin()) {
        printf("ICM20948: Bus init failed\n");
        return false;
    }

    // Probe the device
    if (!m_bus->probe()) {
        printf("ICM20948: Device not responding at I2C address\n");
        return false;
    }
    printf("ICM20948: Device found via probe\n");

    // Check WHO_AM_I
    uint8_t whoami = 0;
    if (!readRegister(kBank0, kRegWhoAmI, &whoami)) {
        printf("ICM20948: WHO_AM_I read failed\n");
        return false;
    }

    printf("ICM20948: WHO_AM_I = 0x%02X (expected 0x%02X)\n", whoami, DEVICE_ID);
    if (whoami != DEVICE_ID) {
        printf("ICM20948: Wrong device ID\n");
        return false;
    }

    // Soft reset
    if (!softReset()) {
        printf("ICM20948: Soft reset failed\n");
        return false;
    }

    sleep_ms(100);  // Wait for reset to complete

    // Wake up device, auto-select best clock
    if (!writeRegister(kBank0, kRegPwrMgmt1, kBitClkAuto)) {
        printf("ICM20948: PWR_MGMT_1 write failed\n");
        return false;
    }

    sleep_ms(10);

    // Enable accel and gyro (disable sleep for all axes)
    if (!writeRegister(kBank0, kRegPwrMgmt2, 0x00)) {
        printf("ICM20948: PWR_MGMT_2 write failed\n");
        return false;
    }

    sleep_ms(10);

    // Configure default ranges
    if (!setAccelRange(AccelRange::RANGE_8G)) {
        printf("ICM20948: setAccelRange failed\n");
        return false;
    }

    if (!setGyroRange(GyroRange::RANGE_1000DPS)) {
        printf("ICM20948: setGyroRange failed\n");
        return false;
    }

    printf("ICM20948: Init complete\n");
    m_initialized = true;
    return true;
}

bool IMU_ICM20948::isConnected() {
    uint8_t whoami = 0;
    if (!readRegister(kBank0, kRegWhoAmI, &whoami)) {
        return false;
    }
    return (whoami == DEVICE_ID);
}

bool IMU_ICM20948::setAccelRange(AccelRange range) {
    uint8_t config = 0;

    switch (range) {
        case AccelRange::RANGE_2G:
            config = 0x00;  // FS_SEL = 0
            break;
        case AccelRange::RANGE_4G:
            config = 0x02;  // FS_SEL = 1
            break;
        case AccelRange::RANGE_8G:
            config = 0x04;  // FS_SEL = 2
            break;
        case AccelRange::RANGE_16G:
            config = 0x06;  // FS_SEL = 3
            break;
        default:
            return false;
    }

    // Enable DLPF with 136Hz bandwidth
    config |= 0x01;  // ACCEL_FCHOICE = 1
    config |= (0x02 << 3);  // ACCEL_DLPFCFG = 2 (136Hz)

    if (!writeRegister(kBank2, kRegAccelConfig, config)) {
        return false;
    }

    m_accel_range = range;
    updateAccelSensitivity(range);
    return true;
}

bool IMU_ICM20948::setGyroRange(GyroRange range) {
    uint8_t config = 0;

    switch (range) {
        case GyroRange::RANGE_250DPS:
            config = 0x00;  // FS_SEL = 0
            break;
        case GyroRange::RANGE_500DPS:
            config = 0x02;  // FS_SEL = 1
            break;
        case GyroRange::RANGE_1000DPS:
            config = 0x04;  // FS_SEL = 2
            break;
        case GyroRange::RANGE_2000DPS:
            config = 0x06;  // FS_SEL = 3
            break;
        default:
            // ICM-20948 doesn't support 125DPS or 4000DPS
            return false;
    }

    // Enable DLPF with 119Hz bandwidth
    config |= 0x01;  // GYRO_FCHOICE = 1
    config |= (0x02 << 3);  // GYRO_DLPFCFG = 2 (119.5Hz)

    if (!writeRegister(kBank2, kRegGyroConfig1, config)) {
        return false;
    }

    m_gyro_range = range;
    updateGyroSensitivity(range);
    return true;
}

bool IMU_ICM20948::read(Vector3f& accel, Vector3f& gyro) {
    // Read 12 bytes: accel (6) + gyro (6)
    // Data starts at ACCEL_XOUT_H (0x2D) through GYRO_ZOUT_L (0x38)
    uint8_t data[12];

    if (!readRegisters(kBank0, kRegAccelXOutH, data, 12)) {
        return false;
    }

    // Parse accelerometer data (big-endian)
    int16_t ax = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t ay = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t az = (static_cast<int16_t>(data[4]) << 8) | data[5];

    // Parse gyroscope data (big-endian)
    int16_t gx = (static_cast<int16_t>(data[6]) << 8) | data[7];
    int16_t gy = (static_cast<int16_t>(data[8]) << 8) | data[9];
    int16_t gz = (static_cast<int16_t>(data[10]) << 8) | data[11];

    // Convert to physical units
    accel.x = static_cast<float>(ax) * m_accel_sensitivity;
    accel.y = static_cast<float>(ay) * m_accel_sensitivity;
    accel.z = static_cast<float>(az) * m_accel_sensitivity;

    gyro.x = static_cast<float>(gx) * m_gyro_sensitivity;
    gyro.y = static_cast<float>(gy) * m_gyro_sensitivity;
    gyro.z = static_cast<float>(gz) * m_gyro_sensitivity;

    return true;
}

bool IMU_ICM20948::readAccel(Vector3f& accel) {
    uint8_t data[6];

    if (!readRegisters(kBank0, kRegAccelXOutH, data, 6)) {
        return false;
    }

    int16_t ax = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t ay = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t az = (static_cast<int16_t>(data[4]) << 8) | data[5];

    accel.x = static_cast<float>(ax) * m_accel_sensitivity;
    accel.y = static_cast<float>(ay) * m_accel_sensitivity;
    accel.z = static_cast<float>(az) * m_accel_sensitivity;

    return true;
}

bool IMU_ICM20948::readGyro(Vector3f& gyro) {
    uint8_t data[6];

    if (!readRegisters(kBank0, kRegGyroXOutH, data, 6)) {
        return false;
    }

    int16_t gx = (static_cast<int16_t>(data[0]) << 8) | data[1];
    int16_t gy = (static_cast<int16_t>(data[2]) << 8) | data[3];
    int16_t gz = (static_cast<int16_t>(data[4]) << 8) | data[5];

    gyro.x = static_cast<float>(gx) * m_gyro_sensitivity;
    gyro.y = static_cast<float>(gy) * m_gyro_sensitivity;
    gyro.z = static_cast<float>(gz) * m_gyro_sensitivity;

    return true;
}

bool IMU_ICM20948::softReset() {
    // Write reset bit
    if (!writeRegister(kBank0, kRegPwrMgmt1, kBitDeviceReset)) {
        return false;
    }

    // Reset clears bank selection, so invalidate cached bank
    m_currentBank = 0xFF;

    return true;
}

bool IMU_ICM20948::selectBank(uint8_t bank) {
    if (bank == m_currentBank) {
        return true;
    }

    uint8_t bankValue = (bank << 4);  // Bank is in bits [5:4]

    if (m_bus->writeRegister(kRegBankSel, bankValue) != BusResult::OK) {
        return false;
    }

    m_currentBank = bank;
    return true;
}

bool IMU_ICM20948::writeRegister(uint8_t bank, uint8_t reg, uint8_t value) {
    if (!selectBank(bank)) {
        return false;
    }
    return m_bus->writeRegister(reg, value) == BusResult::OK;
}

bool IMU_ICM20948::readRegister(uint8_t bank, uint8_t reg, uint8_t* value) {
    if (!selectBank(bank)) {
        return false;
    }
    return m_bus->readRegister(reg, *value) == BusResult::OK;
}

bool IMU_ICM20948::readRegisters(uint8_t bank, uint8_t reg, uint8_t* data, uint8_t len) {
    if (!selectBank(bank)) {
        return false;
    }
    return m_bus->readRegisters(reg, data, len) == BusResult::OK;
}

void IMU_ICM20948::updateAccelSensitivity(AccelRange range) {
    switch (range) {
        case AccelRange::RANGE_2G:
            m_accel_sensitivity = 1.0f / kAccelSensitivity2G;
            break;
        case AccelRange::RANGE_4G:
            m_accel_sensitivity = 1.0f / kAccelSensitivity4G;
            break;
        case AccelRange::RANGE_8G:
            m_accel_sensitivity = 1.0f / kAccelSensitivity8G;
            break;
        case AccelRange::RANGE_16G:
            m_accel_sensitivity = 1.0f / kAccelSensitivity16G;
            break;
        default:
            break;
    }
}

void IMU_ICM20948::updateGyroSensitivity(GyroRange range) {
    switch (range) {
        case GyroRange::RANGE_250DPS:
            m_gyro_sensitivity = 1.0f / kGyroSensitivity250;
            break;
        case GyroRange::RANGE_500DPS:
            m_gyro_sensitivity = 1.0f / kGyroSensitivity500;
            break;
        case GyroRange::RANGE_1000DPS:
            m_gyro_sensitivity = 1.0f / kGyroSensitivity1000;
            break;
        case GyroRange::RANGE_2000DPS:
            m_gyro_sensitivity = 1.0f / kGyroSensitivity2000;
            break;
        default:
            break;
    }
}

} // namespace hal
} // namespace rocketchip
