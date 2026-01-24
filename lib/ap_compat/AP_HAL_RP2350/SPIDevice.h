/**
 * @file SPIDevice.h
 * @brief AP_HAL SPIDevice implementation for RP2350
 *
 * Wraps RocketChip's SPIBus class with AP_HAL::SPIDevice interface.
 * Uses polling-only mode (no DMA) per platform difference PD8.
 *
 * Device naming convention:
 * - "imu:0" - IMU on SPI0
 * - "radio:0" - Radio (RFM95W) on SPI0
 * - "flash:0" - External flash on SPI0
 *
 * @note Polling-only due to RP2350 SPI+DMA issue (PD8).
 *       DMA transfers stop after ~253 bytes/cycles.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>
#include "Semaphores.h"

// Forward declare to avoid header conflicts
namespace rocketchip {
namespace hal {
class SPIBus;
}
}

namespace RP2350 {

// ============================================================================
// Constants
// ============================================================================

/** Maximum number of SPI buses */
static constexpr uint8_t kMaxSPIBuses = 2;

/** Maximum devices per bus */
static constexpr uint8_t kMaxSPIDevicesPerBus = 4;

/** Maximum device name length */
static constexpr uint8_t kMaxDeviceNameLen = 16;

/** Default SPI clock speed */
static constexpr uint32_t kDefaultSPIClockHz = 8000000;  // 8MHz

/** Low speed SPI clock */
static constexpr uint32_t kLowSpeedSPIClockHz = 1000000;  // 1MHz

/** High speed SPI clock */
static constexpr uint32_t kHighSpeedSPIClockHz = 10000000;  // 10MHz

// ============================================================================
// Device Speed Enumeration
// ============================================================================

/**
 * @brief SPI speed setting
 */
enum class SPISpeed : uint8_t {
    LOW = 0,
    HIGH = 1
};

// ============================================================================
// SPI Device Descriptor
// ============================================================================

/**
 * @brief SPI device hardware configuration
 *
 * Defines the pin assignments and settings for a named SPI device.
 */
struct SPIDeviceDesc {
    const char* name;       // Device name (e.g., "radio:0")
    uint8_t bus;            // SPI bus number (0 or 1)
    uint8_t cs_pin;         // Chip select GPIO
    uint8_t sck_pin;        // Clock GPIO
    uint8_t mosi_pin;       // MOSI GPIO
    uint8_t miso_pin;       // MISO GPIO
    uint32_t freq_hz;       // Default clock speed
    uint8_t mode;           // SPI mode (0-3)
};

// ============================================================================
// SPIDevice_RP2350
// ============================================================================

/**
 * @brief SPI device wrapper for AP_HAL compatibility
 *
 * Wraps a RocketChip SPIBus instance for a specific device.
 * Uses polling-only transfers (no DMA) per PD8.
 *
 * Usage:
 * @code
 * auto* dev = hal.spi_mgr.get_device("radio:0");
 * if (dev) {
 *     uint8_t tx = 0x42;
 *     uint8_t rx;
 *     if (dev->transfer(&tx, 1, &rx, 1)) {
 *         printf("Received: 0x%02X\n", rx);
 *     }
 * }
 * @endcode
 */
class SPIDevice_RP2350 {
public:
    /**
     * @brief Construct SPI device from descriptor
     * @param desc Device descriptor with pin assignments
     */
    explicit SPIDevice_RP2350(const SPIDeviceDesc& desc);

    ~SPIDevice_RP2350();

    // Prevent copying
    SPIDevice_RP2350(const SPIDevice_RP2350&) = delete;
    SPIDevice_RP2350& operator=(const SPIDevice_RP2350&) = delete;

    // ========================================================================
    // Core Transfer (Polling-only per PD8)
    // ========================================================================

    /**
     * @brief Perform SPI transfer (half-duplex style)
     *
     * Sends data first, then receives. Uses polling mode only.
     *
     * @param send Data to send (can be nullptr)
     * @param send_len Bytes to send
     * @param recv Buffer to receive into (can be nullptr)
     * @param recv_len Bytes to receive
     * @return true on success
     */
    bool transfer(const uint8_t* send, uint32_t send_len,
                  uint8_t* recv, uint32_t recv_len);

    /**
     * @brief Perform full-duplex SPI transfer
     *
     * Simultaneous send and receive of equal-length buffers.
     *
     * @param send Data to send
     * @param recv Buffer to receive into
     * @param len Number of bytes to transfer
     * @return true on success
     */
    bool transfer_fullduplex(const uint8_t* send, uint8_t* recv, uint32_t len);

    /**
     * @brief Send clock pulses without asserting CS
     *
     * Used for SD card initialization.
     *
     * @param len Number of bytes worth of clocks
     * @return true on success
     */
    bool clock_pulse(uint32_t len);

    // ========================================================================
    // Device Info
    // ========================================================================

    /**
     * @brief Get device name
     */
    const char* get_name() const { return m_name; }

    /**
     * @brief Get bus number
     */
    uint8_t bus_num() const { return m_bus; }

    /**
     * @brief Check if device is initialized
     */
    bool is_initialized() const { return m_initialized; }

    // ========================================================================
    // Thread Safety
    // ========================================================================

    /**
     * @brief Get bus semaphore for external locking
     * @return Pointer to semaphore
     */
    Semaphore* get_semaphore();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Set SPI speed
     * @param speed LOW or HIGH speed setting
     * @return true if set successfully
     */
    bool set_speed(SPISpeed speed);

    /**
     * @brief Set bus slowdown factor
     *
     * Divides clock by this factor for debugging.
     *
     * @param slowdown Division factor
     */
    void set_slowdown(uint8_t slowdown);

private:
    char m_name[kMaxDeviceNameLen];
    uint8_t m_bus;
    uint8_t m_cs_pin;
    uint32_t m_freq_low;
    uint32_t m_freq_high;
    uint32_t m_freq_current;
    uint8_t m_slowdown;

    rocketchip::hal::SPIBus* m_spi_bus;
    bool m_initialized;
};

// ============================================================================
// SPIDeviceManager_RP2350
// ============================================================================

/**
 * @brief SPI device manager for AP_HAL compatibility
 *
 * Factory for creating SPIDevice instances. Manages bus initialization
 * and provides access to devices by name.
 *
 * Pre-defined device names:
 * - "radio:0" - LoRa radio on SPI0 (CS=10)
 * - "flash:0" - External flash on SPI0 (if present)
 * - "imu:0" - IMU on SPI0 (if using SPI instead of I2C)
 */
class SPIDeviceManager_RP2350 {
public:
    SPIDeviceManager_RP2350();
    ~SPIDeviceManager_RP2350();

    /**
     * @brief Initialize SPI manager
     */
    void init();

    /**
     * @brief Get device by name
     * @param name Device name (e.g., "radio:0")
     * @return Pointer to device, or nullptr if not found
     */
    SPIDevice_RP2350* get_device(const char* name);

    /**
     * @brief Get number of registered devices
     */
    uint8_t get_count() const { return m_device_count; }

    /**
     * @brief Get device name at index
     * @param idx Device index
     * @return Device name, or nullptr if invalid
     */
    const char* get_device_name(uint8_t idx) const;

    /**
     * @brief Get mask of available buses
     * @return Bitmask (bit 0 = bus 0, bit 1 = bus 1)
     */
    uint32_t get_bus_mask() const { return 0x03; }

private:
    // Device table (built-in known devices)
    static const SPIDeviceDesc kDeviceTable[];
    static const uint8_t kDeviceTableSize;

    // Instantiated devices
    SPIDevice_RP2350* m_devices[kMaxSPIBuses * kMaxSPIDevicesPerBus];
    uint8_t m_device_count;
    bool m_initialized;

    // Bus semaphores
    Semaphore m_bus_semaphores[kMaxSPIBuses];

    // Find device descriptor by name
    const SPIDeviceDesc* find_desc(const char* name) const;
};

}  // namespace RP2350
