/**
 * @file I2CDevice.cpp
 * @brief AP_HAL I2CDevice implementation for RP2350
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "I2CDevice.h"

// RocketChip HAL
#include "hal/Bus.h"

// Pico SDK
#include "hardware/i2c.h"
#include "pico/stdlib.h"

namespace RP2350 {

// ============================================================================
// I2C Pin Definitions (Feather RP2350 uses I2C1 for Qwiic)
// ============================================================================

// Feather RP2350 Qwiic uses I2C1 on pins 2/3 (PICO_DEFAULT_I2C=1)
// Bus 0 in our API = Qwiic connector = I2C1 hardware
// Bus 1 in our API = Alternate = I2C0 hardware (if available)

#ifndef QWIIC_I2C_INST
#define QWIIC_I2C_INST i2c1
#endif

#ifndef QWIIC_SDA_PIN
#define QWIIC_SDA_PIN 2
#endif

#ifndef QWIIC_SCL_PIN
#define QWIIC_SCL_PIN 3
#endif

// Alternate I2C (if needed)
#ifndef ALT_I2C_INST
#define ALT_I2C_INST i2c0
#endif

#ifndef ALT_SDA_PIN
#define ALT_SDA_PIN 4  // Example alternate pins
#endif

#ifndef ALT_SCL_PIN
#define ALT_SCL_PIN 5
#endif

// ============================================================================
// I2CDevice_RP2350
// ============================================================================

I2CDevice_RP2350::I2CDevice_RP2350(uint8_t bus, uint8_t address,
                                     uint32_t bus_clock, uint32_t timeout_ms)
    : AP_HAL::I2CDevice()  // Calls Device(BUS_TYPE_I2C)
    , m_bus(bus)
    , m_bus_clock(bus_clock)
    , m_timeout_ms(timeout_ms)
    , m_retries(2)
    , m_i2c_bus(nullptr)
    , m_semaphore(nullptr)
    , m_initialized(false)
{
    // Set device identification in base class
    set_device_bus(bus);
    set_device_address(address);

    // Bus 0 = Qwiic (I2C1 on Feather RP2350)
    // Bus 1 = Alternate (I2C0)
    void* i2c_inst = (bus == 0) ? QWIIC_I2C_INST : ALT_I2C_INST;
    uint8_t sda = (bus == 0) ? QWIIC_SDA_PIN : ALT_SDA_PIN;
    uint8_t scl = (bus == 0) ? QWIIC_SCL_PIN : ALT_SCL_PIN;

    m_i2c_bus = new rocketchip::hal::I2CBus(i2c_inst, address, sda, scl, bus_clock);

    if (m_i2c_bus->begin()) {
        m_initialized = true;
    }
}

I2CDevice_RP2350::~I2CDevice_RP2350() {
    delete m_i2c_bus;
}

// ============================================================================
// Core Transfer
// ============================================================================

bool I2CDevice_RP2350::transfer(const uint8_t* send, uint32_t send_len,
                                  uint8_t* recv, uint32_t recv_len) {
    if (!m_initialized || m_i2c_bus == nullptr) {
        return false;
    }

    // Try with retries
    for (uint8_t attempt = 0; attempt <= m_retries; attempt++) {
        bool success = true;

        // Send phase
        if (send != nullptr && send_len > 0) {
            auto result = m_i2c_bus->write(send, send_len);
            if (result != rocketchip::hal::BusResult::OK) {
                success = false;
            }
        }

        // Receive phase
        if (success && recv != nullptr && recv_len > 0) {
            auto result = m_i2c_bus->read(recv, recv_len);
            if (result != rocketchip::hal::BusResult::OK) {
                success = false;
            }
        }

        if (success) {
            return true;
        }

        // Brief delay before retry
        if (attempt < m_retries) {
            sleep_us(100);
        }
    }

    return false;
}

bool I2CDevice_RP2350::read_registers(uint8_t first_reg, uint8_t* recv, uint32_t recv_len) {
    if (!m_initialized || m_i2c_bus == nullptr) {
        return false;
    }

    // Apply read flag if set (from Device base class)
    uint8_t reg = first_reg | _read_flag;

    // I2C read: write register address, then read data
    return transfer(&reg, 1, recv, recv_len);
}

bool I2CDevice_RP2350::write_register(uint8_t reg, uint8_t val) {
    if (!m_initialized || m_i2c_bus == nullptr) {
        return false;
    }

    uint8_t buf[2] = {reg, val};
    return transfer(buf, 2, nullptr, 0);
}

bool I2CDevice_RP2350::read_registers_multiple(uint8_t first_reg, uint8_t* recv,
                                                 uint32_t recv_len, uint8_t times) {
    if (!m_initialized || recv == nullptr) {
        return false;
    }

    // Read the same registers multiple times, advancing buffer each time
    for (uint8_t i = 0; i < times; i++) {
        if (!read_registers(first_reg, recv + (i * recv_len), recv_len)) {
            return false;
        }
    }

    return true;
}

// ============================================================================
// Device Info
// ============================================================================

bool I2CDevice_RP2350::probe() {
    if (!m_initialized || m_i2c_bus == nullptr) {
        return false;
    }

    return m_i2c_bus->probe();
}

// ============================================================================
// Thread Safety
// ============================================================================

AP_HAL::Semaphore* I2CDevice_RP2350::get_semaphore() {
    return m_semaphore;
}

// ============================================================================
// Periodic Callbacks
// ============================================================================

AP_HAL::Device::PeriodicHandle I2CDevice_RP2350::register_periodic_callback(
    uint32_t period_usec, PeriodicCb cb) {
    // TODO: Implement periodic callbacks via Scheduler
    // ArduPilot sensor drivers typically use this for polling sensors at fixed rates
    // For Phase 2, most sensors will be polled directly from SensorTask
    (void)period_usec;
    (void)cb;
    return nullptr;
}

bool I2CDevice_RP2350::adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) {
    // TODO: Implement when register_periodic_callback is implemented
    (void)h;
    (void)period_usec;
    return false;
}

// ============================================================================
// Configuration
// ============================================================================

bool I2CDevice_RP2350::set_speed(Speed speed) {
    // Would need to reinitialize bus with new speed
    // SPEED_HIGH = 400kHz, SPEED_LOW = 100kHz
    (void)speed;
    return true;
}

void I2CDevice_RP2350::set_address(uint8_t address) {
    set_device_address(address);  // Update base class
    if (m_i2c_bus != nullptr) {
        m_i2c_bus->setAddress(address);
    }
}

// ============================================================================
// I2CDeviceManager_RP2350
// ============================================================================

I2CDeviceManager_RP2350::I2CDeviceManager_RP2350()
    : m_devices{}
    , m_device_count{}
    , m_initialized(false)
    , m_bus_semaphores{}
{
}

I2CDeviceManager_RP2350::~I2CDeviceManager_RP2350() {
    // Clean up allocated devices
    for (uint8_t bus = 0; bus < kMaxI2CBuses; bus++) {
        for (uint8_t i = 0; i < m_device_count[bus]; i++) {
            delete m_devices[bus][i];
        }
    }
}

void I2CDeviceManager_RP2350::init() {
    if (m_initialized) {
        return;
    }

    // Initialize bus semaphores
    for (uint8_t bus = 0; bus < kMaxI2CBuses; bus++) {
        m_bus_semaphores[bus].give();  // Start unlocked
    }

    m_initialized = true;
}

AP_HAL::I2CDevice* I2CDeviceManager_RP2350::get_device_ptr(uint8_t bus, uint8_t address,
                                                            uint32_t bus_clock,
                                                            bool use_smbus,
                                                            uint32_t timeout_ms) {
    (void)use_smbus;  // SMBus not supported on RP2350

    if (bus >= kMaxI2CBuses) {
        return nullptr;
    }

    // Check if we already have a device at this address
    for (uint8_t i = 0; i < m_device_count[bus]; i++) {
        if (m_devices[bus][i] != nullptr &&
            m_devices[bus][i]->get_bus_address() == address) {
            return m_devices[bus][i];
        }
    }

    // Create new device if space available
    if (m_device_count[bus] >= kMaxDevicesPerBus) {
        return nullptr;
    }

    auto* device = new I2CDevice_RP2350(bus, address, bus_clock, timeout_ms);

    // Give device access to bus semaphore
    device->m_semaphore = &m_bus_semaphores[bus];

    m_devices[bus][m_device_count[bus]++] = device;

    return device;
}

Semaphore* I2CDeviceManager_RP2350::get_bus_semaphore(uint8_t bus) {
    if (bus >= kMaxI2CBuses) {
        return nullptr;
    }
    return &m_bus_semaphores[bus];
}

}  // namespace RP2350
