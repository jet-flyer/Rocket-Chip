/**
 * @file I2CDevice.cpp
 * @brief AP_HAL I2CDevice implementation for RP2350
 *
 * Uses DeviceBus pattern from ESP32 HAL for periodic callbacks.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "I2CDevice.h"

// Pico SDK
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstdio>

// Debug output for I2C (disable in production)
#define I2C_DEBUG 0

#if I2C_DEBUG
#define DBG_I2C(fmt, ...) printf("[I2C] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG_I2C(fmt, ...) ((void)0)
#endif

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

// Static bus info array
I2CBus I2CDeviceManager_RP2350::businfo[kMaxI2CBuses];

// ============================================================================
// I2C Bus Recovery
// ============================================================================

/**
 * @brief Recover I2C bus from stuck state
 *
 * If a device was in the middle of a transaction when reset occurred,
 * it may be holding SDA low waiting for clocks. Toggle SCL to free it.
 */
static void i2c_bus_recovery(uint8_t sda_pin, uint8_t scl_pin) {
    // Temporarily use GPIO mode
    gpio_init(sda_pin);
    gpio_init(scl_pin);
    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_OUT);
    gpio_pull_up(sda_pin);
    gpio_pull_up(scl_pin);

    // Toggle SCL up to 9 times to release any stuck device
    for (int i = 0; i < 9; i++) {
        // Check if SDA is high (bus is free)
        if (gpio_get(sda_pin)) {
            break;
        }
        // Toggle SCL
        gpio_put(scl_pin, 0);
        busy_wait_us(5);
        gpio_put(scl_pin, 1);
        busy_wait_us(5);
    }

    // Generate STOP condition: SDA low, then SCL high, then SDA high
    gpio_set_dir(sda_pin, GPIO_OUT);
    gpio_put(sda_pin, 0);
    busy_wait_us(5);
    gpio_put(scl_pin, 1);
    busy_wait_us(5);
    gpio_put(sda_pin, 1);
    busy_wait_us(5);

    // Return pins to input to allow I2C peripheral to take over
    gpio_set_dir(sda_pin, GPIO_IN);
    gpio_set_dir(scl_pin, GPIO_IN);
}

// ============================================================================
// I2CDevice_RP2350
// ============================================================================

I2CDevice_RP2350::I2CDevice_RP2350(I2CBus &bus, uint8_t address,
                                     uint32_t bus_clock, uint32_t timeout_ms)
    : AP_HAL::I2CDevice()  // Calls Device(BUS_TYPE_I2C)
    , m_bus(bus)
    , m_address(address)
    , m_timeout_ms(timeout_ms)
    , m_retries(2)
    , m_initialized(true)  // Bus is managed by manager
{
    // Set device identification in base class
    set_device_bus(bus.bus_num);
    set_device_address(address);
}

I2CDevice_RP2350::~I2CDevice_RP2350() {
    // Nothing to clean up - bus is owned by manager
}

// ============================================================================
// Core Transfer
// ============================================================================

bool I2CDevice_RP2350::transfer(const uint8_t* send, uint32_t send_len,
                                  uint8_t* recv, uint32_t recv_len) {
    static uint32_t s_transfer_count = 0;
    static uint32_t s_transfer_ok = 0;
    static uint32_t s_consecutive_fails = 0;
    s_transfer_count++;

    // Log transfers for debugging
    if (s_transfer_count <= 50) {
        DBG_I2C("xfer #%lu (0x%02X) snd=%lu rcv=%lu",
                s_transfer_count, m_address, send_len, recv_len);
    } else if (s_transfer_count % 500 == 0) {
        DBG_I2C("xfer #%lu (periodic)", s_transfer_count);
    }

    if (!m_initialized) {
        DBG_I2C("transfer failed: not initialized");
        return false;
    }

    // Get raw I2C instance based on bus number
    i2c_inst_t* i2c = (m_bus.bus_num == 0) ? QWIIC_I2C_INST : ALT_I2C_INST;
    uint8_t sda = (m_bus.bus_num == 0) ? QWIIC_SDA_PIN : ALT_SDA_PIN;
    uint8_t scl = (m_bus.bus_num == 0) ? QWIIC_SCL_PIN : ALT_SCL_PIN;
    constexpr uint32_t TIMEOUT_US = 10000;

    // Ensure I2C is properly initialized before each transfer
    // Use 100kHz - RP2350 has issues with clock stretching at higher speeds (PD7)
    constexpr uint32_t I2C_FREQ_HZ = 100000;
    i2c_init(i2c, I2C_FREQ_HZ);
    gpio_set_function(sda, GPIO_FUNC_I2C);
    gpio_set_function(scl, GPIO_FUNC_I2C);
    gpio_pull_up(sda);
    gpio_pull_up(scl);

    // Try with retries
    for (uint8_t attempt = 0; attempt <= m_retries; attempt++) {
        bool success = true;

        // If we have both send and receive, use repeated start (nostop=true on send)
        bool use_repeated_start = (send != nullptr && send_len > 0 &&
                                    recv != nullptr && recv_len > 0);

        // Send phase
        if (send != nullptr && send_len > 0) {
            // nostop=true if we need to follow with a read (repeated start)
            int result = i2c_write_timeout_us(i2c, m_address, send, send_len,
                                               use_repeated_start, TIMEOUT_US);
            if (result < 0) {
                success = false;
            }
        }

        // Receive phase
        if (success && recv != nullptr && recv_len > 0) {
            int result = i2c_read_timeout_us(i2c, m_address, recv, recv_len,
                                              false, TIMEOUT_US);
            if (result < 0) {
                success = false;
            }
        }

        if (success) {
            s_transfer_ok++;
            s_consecutive_fails = 0;  // Reset failure counter
            if (s_transfer_ok <= 5 || s_transfer_ok % 50 == 0) {
                DBG_I2C("transfer OK #%lu (addr=0x%02X)",
                        s_transfer_ok, m_address);
            }
            return true;
        }

        // Brief delay before retry
        if (attempt < m_retries) {
            sleep_us(100);
        }
    }

    // Track consecutive failures and attempt bus recovery
    s_consecutive_fails++;
    if (s_consecutive_fails == 3) {
        DBG_I2C("3 consecutive fails - attempting bus recovery");
        i2c_bus_recovery(sda, scl);
        // Reinitialize I2C after recovery
        i2c_init(i2c, I2C_FREQ_HZ);
        gpio_set_function(sda, GPIO_FUNC_I2C);
        gpio_set_function(scl, GPIO_FUNC_I2C);
        gpio_pull_up(sda);
        gpio_pull_up(scl);
    }

    // Log all failures (not just sampled)
    DBG_I2C("transfer FAIL #%lu to 0x%02X after %u retries (consec=%lu)",
            s_transfer_count, m_address, m_retries, s_consecutive_fails);
    return false;
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
    // Simple probe: try to read one byte from address
    uint8_t dummy;
    i2c_inst_t* i2c = (m_bus.bus_num == 0) ? QWIIC_I2C_INST : ALT_I2C_INST;
    int result = i2c_read_timeout_us(i2c, m_address, &dummy, 1, false, 1000);
    return (result >= 0);
}

// ============================================================================
// Thread Safety
// ============================================================================

AP_HAL::Semaphore* I2CDevice_RP2350::get_semaphore() {
    // Return semaphore from our bus (DeviceBus holds it)
    return &m_bus.semaphore;
}

// ============================================================================
// Periodic Callbacks (delegate to DeviceBus)
// ============================================================================

AP_HAL::Device::PeriodicHandle I2CDevice_RP2350::register_periodic_callback(
    uint32_t period_usec, PeriodicCb cb) {
    DBG_I2C("register_periodic_callback: period=%lu us", period_usec);
    return m_bus.register_periodic_callback(period_usec, cb, this);
}

bool I2CDevice_RP2350::adjust_periodic_callback(PeriodicHandle h, uint32_t period_usec) {
    return m_bus.adjust_timer(h, period_usec);
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
    m_address = address;
    set_device_address(address);  // Update base class
}

// ============================================================================
// I2CDeviceManager_RP2350
// ============================================================================

I2CDeviceManager_RP2350::I2CDeviceManager_RP2350()
    : m_initialized(false)
{
}

I2CDeviceManager_RP2350::~I2CDeviceManager_RP2350() {
    // Devices are owned by callers via OwnPtr, nothing to clean up
}

void I2CDeviceManager_RP2350::init() {
    if (m_initialized) {
        return;
    }

    DBG_I2C("Initializing I2C device manager");

    // Recover I2C buses from any stuck state (e.g., after reset during transaction)
    i2c_bus_recovery(QWIIC_SDA_PIN, QWIIC_SCL_PIN);
    i2c_bus_recovery(ALT_SDA_PIN, ALT_SCL_PIN);

    // Initialize bus info
    businfo[0].bus_num = 0;
    businfo[0].bus_clock = kDefaultClockHz;
    businfo[0].semaphore.give();  // Start unlocked

    businfo[1].bus_num = 1;
    businfo[1].bus_clock = kDefaultClockHz;
    businfo[1].semaphore.give();  // Start unlocked

    m_initialized = true;
    DBG_I2C("I2C device manager initialized");
}

AP_HAL::I2CDevice* I2CDeviceManager_RP2350::get_device_ptr(uint8_t bus, uint8_t address,
                                                            uint32_t bus_clock,
                                                            bool use_smbus,
                                                            uint32_t timeout_ms) {
    (void)use_smbus;  // SMBus not supported on RP2350

    // Debug disabled - use probe instead
    // printf("[I2C] get_device_ptr(bus=%u, addr=0x%02X)\n", bus, address);

    if (bus >= kMaxI2CBuses) {
        DBG_I2C("Invalid bus %u (max %u)", bus, kMaxI2CBuses - 1);
        return nullptr;
    }

    DBG_I2C("Creating device: bus=%u addr=0x%02X", bus, address);

    // Return new device that references our static bus info
    return new I2CDevice_RP2350(businfo[bus], address, bus_clock, timeout_ms);
}

}  // namespace RP2350
