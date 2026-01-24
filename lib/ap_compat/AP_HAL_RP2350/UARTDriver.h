/**
 * @file UARTDriver.h
 * @brief AP_HAL UARTDriver implementation for RP2350
 *
 * Wraps RocketChip's UART and USBSerial classes with AP_HAL interface.
 * Supports both USB CDC (console) and hardware UART ports.
 *
 * Port mapping:
 * - Serial 0: USB CDC (console/debug)
 * - Serial 1: UART0 (GPS, radio, etc.)
 * - Serial 2: UART1 (GPS, radio, etc.)
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>
#include <cstddef>

// Forward declare to avoid header conflicts
namespace rocketchip {
namespace hal {
class UART;
}
}

namespace RP2350 {

// ============================================================================
// Constants
// ============================================================================

/** Maximum number of serial ports */
static constexpr uint8_t kMaxSerialPorts = 3;

/** Default TX buffer size */
static constexpr uint16_t kDefaultTxBufferSize = 256;

/** Default RX buffer size */
static constexpr uint16_t kDefaultRxBufferSize = 256;

// ============================================================================
// UARTDriver_RP2350
// ============================================================================

/**
 * @brief UART driver for RP2350
 *
 * Provides AP_HAL::UARTDriver-compatible interface wrapping RocketChip HAL.
 * Can operate in USB CDC mode (for console) or hardware UART mode.
 *
 * Usage:
 * @code
 * // USB CDC (port 0)
 * hal.serial[0]->begin(115200);
 * hal.serial[0]->printf("Hello from USB\n");
 *
 * // Hardware UART (port 1)
 * hal.serial[1]->begin(9600);  // GPS default baud
 * while (hal.serial[1]->available()) {
 *     char c = hal.serial[1]->read();
 * }
 * @endcode
 */
class UARTDriver_RP2350 {
public:
    /**
     * @brief Port type selection
     */
    enum class PortType : uint8_t {
        USB_CDC,    ///< USB CDC (virtual COM port)
        UART0,      ///< Hardware UART0
        UART1       ///< Hardware UART1
    };

    /**
     * @brief Construct UART driver
     * @param type Port type (USB_CDC, UART0, UART1)
     * @param tx_pin TX GPIO pin (ignored for USB_CDC)
     * @param rx_pin RX GPIO pin (ignored for USB_CDC)
     */
    UARTDriver_RP2350(PortType type, uint8_t tx_pin = 0, uint8_t rx_pin = 0);

    /**
     * @brief Destructor
     */
    ~UARTDriver_RP2350();

    // Prevent copying
    UARTDriver_RP2350(const UARTDriver_RP2350&) = delete;
    UARTDriver_RP2350& operator=(const UARTDriver_RP2350&) = delete;

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize UART with specified baud rate
     * @param baud Baud rate (ignored for USB CDC)
     */
    void begin(uint32_t baud);

    /**
     * @brief Initialize with buffer size hints
     * @param baud Baud rate
     * @param rxSpace RX buffer size (hint, may be ignored)
     * @param txSpace TX buffer size (hint, may be ignored)
     */
    void begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace);

    /**
     * @brief Shutdown UART
     */
    void end();

    /**
     * @brief Check if initialized
     * @return true if begin() was called successfully
     */
    bool is_initialized() const { return m_initialized; }

    // ========================================================================
    // Read Operations
    // ========================================================================

    /**
     * @brief Check available bytes to read
     * @return Number of bytes available (may be approximate for USB CDC)
     */
    uint32_t available();

    /**
     * @brief Read single byte
     * @return Byte read (0-255), or -1 if none available
     */
    int16_t read();

    /**
     * @brief Read single byte
     * @param b Reference to store byte
     * @return true if byte was read
     */
    bool read(uint8_t& b);

    /**
     * @brief Read multiple bytes
     * @param buffer Output buffer
     * @param count Maximum bytes to read
     * @return Number of bytes read, or -1 on error
     */
    int32_t read(uint8_t* buffer, uint16_t count);

    /**
     * @brief Discard all pending input
     * @return true if successful
     */
    bool discard_input();

    // ========================================================================
    // Write Operations
    // ========================================================================

    /**
     * @brief Write single byte
     * @param c Byte to write
     * @return 1 if written, 0 if failed
     */
    size_t write(uint8_t c);

    /**
     * @brief Write multiple bytes
     * @param buffer Data to write
     * @param size Number of bytes
     * @return Number of bytes written
     */
    size_t write(const uint8_t* buffer, size_t size);

    /**
     * @brief Write null-terminated string
     * @param str String to write
     * @return Number of bytes written
     */
    size_t write(const char* str);

    /**
     * @brief Printf-style formatted output
     * @param fmt Format string
     * @return Number of bytes written
     */
    void printf(const char* fmt, ...);

    /**
     * @brief Check TX buffer space available
     * @return Bytes available in TX buffer
     */
    uint32_t txspace();

    /**
     * @brief Check if TX data is pending
     * @return true if TX buffer has unsent data
     */
    bool tx_pending();

    /**
     * @brief Flush TX buffer (wait for all data to be sent)
     */
    void flush();

    // ========================================================================
    // Configuration
    // ========================================================================

    /**
     * @brief Get current baud rate
     * @return Baud rate (0 for USB CDC)
     */
    uint32_t get_baud_rate() const { return m_baud; }

    /**
     * @brief Set parity mode
     * @param v Parity: 0=none, 1=odd, 2=even
     */
    void configure_parity(uint8_t v);

    /**
     * @brief Set stop bits
     * @param n Number of stop bits (1 or 2)
     */
    void set_stop_bits(int n);

private:
    PortType m_type;
    uint8_t m_tx_pin;
    uint8_t m_rx_pin;
    uint32_t m_baud;
    bool m_initialized;

    // Hardware UART instance (only for UART0/UART1)
    rocketchip::hal::UART* m_uart;
};

}  // namespace RP2350
