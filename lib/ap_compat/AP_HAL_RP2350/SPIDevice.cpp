/**
 * @file SPIDevice.cpp
 * @brief AP_HAL SPIDevice implementation for RP2350
 *
 * @note POLLING-ONLY: No DMA used per platform difference PD8.
 *       RP2350 SPI+DMA stops after ~253 cycles.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "SPIDevice.h"
#include "hwdef.h"

// RocketChip HAL
#include "hal/Bus.h"

// Pico SDK
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

#include <cstring>

namespace RP2350 {

// ============================================================================
// Device Table
// ============================================================================

// Known SPI devices on RocketChip
// Add entries here for new SPI peripherals
const SPIDeviceDesc SPIDeviceManager_RP2350::kDeviceTable[] = {
    // LoRa Radio FeatherWing (RFM95W) - "Feather M0" jumper position = CS on pin 10
    // RST=11, IRQ=6 (handled separately by radio driver)
    {
        .name = "radio:0",
        .bus = 0,
        .cs_pin = 10,
        .sck_pin = Pins::SPI0_SCK,
        .mosi_pin = Pins::SPI0_MOSI,
        .miso_pin = Pins::SPI0_MISO,
        .freq_hz = kDefaultSPIClockHz,
        .mode = 0
    },

    // External flash (if present) - commonly on CS pin 25
    {
        .name = "flash:0",
        .bus = 0,
        .cs_pin = 25,
        .sck_pin = Pins::SPI0_SCK,
        .mosi_pin = Pins::SPI0_MOSI,
        .miso_pin = Pins::SPI0_MISO,
        .freq_hz = kHighSpeedSPIClockHz,
        .mode = 0
    },

    // IMU over SPI (if using SPI instead of I2C)
    // ISM330DHCX supports both - SPI preferred for high-rate sampling
    // Note: FeatherWing only exposes I2C, this is for custom wiring
    {
        .name = "imu:0",
        .bus = 0,
        .cs_pin = 9,  // Example CS pin
        .sck_pin = Pins::SPI0_SCK,
        .mosi_pin = Pins::SPI0_MOSI,
        .miso_pin = Pins::SPI0_MISO,
        .freq_hz = kHighSpeedSPIClockHz,
        .mode = 3  // ISM330DHCX uses SPI mode 3
    },
};

const uint8_t SPIDeviceManager_RP2350::kDeviceTableSize =
    sizeof(kDeviceTable) / sizeof(kDeviceTable[0]);

// ============================================================================
// SPIDevice_RP2350
// ============================================================================

SPIDevice_RP2350::SPIDevice_RP2350(const SPIDeviceDesc& desc)
    : m_bus(desc.bus)
    , m_cs_pin(desc.cs_pin)
    , m_freq_low(kLowSpeedSPIClockHz)
    , m_freq_high(desc.freq_hz)
    , m_freq_current(desc.freq_hz)
    , m_slowdown(1)
    , m_spi_bus(nullptr)
    , m_initialized(false)
{
    // Copy name
    strncpy(m_name, desc.name, kMaxDeviceNameLen - 1);
    m_name[kMaxDeviceNameLen - 1] = '\0';

    // Select SPI instance based on bus number
    void* spi_inst = (desc.bus == 0) ? spi0 : spi1;

    // Create SPIBus with mode
    auto mode = static_cast<rocketchip::hal::SPIBus::Mode>(desc.mode);
    m_spi_bus = new rocketchip::hal::SPIBus(
        spi_inst,
        desc.cs_pin,
        desc.sck_pin,
        desc.mosi_pin,
        desc.miso_pin,
        desc.freq_hz,
        mode
    );

    if (m_spi_bus->begin()) {
        m_initialized = true;
    }
}

SPIDevice_RP2350::~SPIDevice_RP2350() {
    delete m_spi_bus;
}

// ============================================================================
// Core Transfer (Polling-only per PD8)
// ============================================================================

bool SPIDevice_RP2350::transfer(const uint8_t* send, uint32_t send_len,
                                  uint8_t* recv, uint32_t recv_len) {
    if (!m_initialized || m_spi_bus == nullptr) {
        return false;
    }

    // IMPORTANT: Using polling only - no DMA per PD8
    // RP2350 SPI+DMA stops after ~253 cycles

    spi_inst_t* spi = (m_bus == 0) ? spi0 : spi1;

    // Assert CS
    gpio_put(m_cs_pin, 0);

    // Send phase (polling)
    if (send != nullptr && send_len > 0) {
        spi_write_blocking(spi, send, send_len);
    }

    // Receive phase (polling)
    if (recv != nullptr && recv_len > 0) {
        spi_read_blocking(spi, 0x00, recv, recv_len);
    }

    // Deassert CS
    gpio_put(m_cs_pin, 1);

    return true;
}

bool SPIDevice_RP2350::transfer_fullduplex(const uint8_t* send, uint8_t* recv,
                                             uint32_t len) {
    if (!m_initialized || m_spi_bus == nullptr) {
        return false;
    }

    if (send == nullptr || recv == nullptr || len == 0) {
        return false;
    }

    // IMPORTANT: Using polling only - no DMA per PD8

    spi_inst_t* spi = (m_bus == 0) ? spi0 : spi1;

    // Assert CS
    gpio_put(m_cs_pin, 0);

    // Full-duplex transfer (polling)
    spi_write_read_blocking(spi, send, recv, len);

    // Deassert CS
    gpio_put(m_cs_pin, 1);

    return true;
}

bool SPIDevice_RP2350::clock_pulse(uint32_t len) {
    if (!m_initialized) {
        return false;
    }

    // Send clock pulses WITHOUT asserting CS
    // Used for SD card initialization

    spi_inst_t* spi = (m_bus == 0) ? spi0 : spi1;

    // Create dummy buffer
    uint8_t dummy = 0xFF;
    for (uint32_t i = 0; i < len; i++) {
        spi_write_blocking(spi, &dummy, 1);
    }

    return true;
}

// ============================================================================
// Thread Safety
// ============================================================================

Semaphore* SPIDevice_RP2350::get_semaphore() {
    // Note: In a full implementation, this would return the bus semaphore
    // from SPIDeviceManager. For now, return nullptr (caller should handle).
    return nullptr;
}

// ============================================================================
// Configuration
// ============================================================================

bool SPIDevice_RP2350::set_speed(SPISpeed speed) {
    if (!m_initialized) {
        return false;
    }

    uint32_t new_freq = (speed == SPISpeed::HIGH) ? m_freq_high : m_freq_low;
    new_freq /= m_slowdown;

    if (new_freq != m_freq_current) {
        spi_inst_t* spi = (m_bus == 0) ? spi0 : spi1;
        spi_set_baudrate(spi, new_freq);
        m_freq_current = new_freq;
    }

    return true;
}

void SPIDevice_RP2350::set_slowdown(uint8_t slowdown) {
    if (slowdown > 0) {
        m_slowdown = slowdown;
    }
}

// ============================================================================
// SPIDeviceManager_RP2350
// ============================================================================

SPIDeviceManager_RP2350::SPIDeviceManager_RP2350()
    : m_devices{}
    , m_device_count(0)
    , m_initialized(false)
    , m_bus_semaphores{}
{
}

SPIDeviceManager_RP2350::~SPIDeviceManager_RP2350() {
    // Clean up allocated devices
    for (uint8_t i = 0; i < m_device_count; i++) {
        delete m_devices[i];
    }
}

void SPIDeviceManager_RP2350::init() {
    if (m_initialized) {
        return;
    }

    // Initialize bus semaphores
    for (uint8_t bus = 0; bus < kMaxSPIBuses; bus++) {
        m_bus_semaphores[bus].give();  // Start unlocked
    }

    m_initialized = true;
}

const SPIDeviceDesc* SPIDeviceManager_RP2350::find_desc(const char* name) const {
    for (uint8_t i = 0; i < kDeviceTableSize; i++) {
        if (strcmp(kDeviceTable[i].name, name) == 0) {
            return &kDeviceTable[i];
        }
    }
    return nullptr;
}

SPIDevice_RP2350* SPIDeviceManager_RP2350::get_device(const char* name) {
    if (name == nullptr) {
        return nullptr;
    }

    // Check if device already instantiated
    for (uint8_t i = 0; i < m_device_count; i++) {
        if (m_devices[i] != nullptr &&
            strcmp(m_devices[i]->get_name(), name) == 0) {
            return m_devices[i];
        }
    }

    // Find device descriptor
    const SPIDeviceDesc* desc = find_desc(name);
    if (desc == nullptr) {
        return nullptr;
    }

    // Check space available
    if (m_device_count >= kMaxSPIBuses * kMaxSPIDevicesPerBus) {
        return nullptr;
    }

    // Create new device
    auto* device = new SPIDevice_RP2350(*desc);
    m_devices[m_device_count++] = device;

    return device;
}

const char* SPIDeviceManager_RP2350::get_device_name(uint8_t idx) const {
    if (idx >= kDeviceTableSize) {
        return nullptr;
    }
    return kDeviceTable[idx].name;
}

}  // namespace RP2350
