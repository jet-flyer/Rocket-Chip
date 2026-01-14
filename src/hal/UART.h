/**
 * @file UART.h
 * @brief Serial communication abstraction
 * 
 * Provides interface for UART communication, used for debug console,
 * GPS modules (alternative to I2C), and some radio modules.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#ifndef ROCKETCHIP_HAL_UART_H
#define ROCKETCHIP_HAL_UART_H

#include <cstdint>
#include <cstddef>

namespace rocketchip {
namespace hal {

/**
 * @brief UART parity configuration
 */
enum class UartParity : uint8_t {
    NONE,
    EVEN,
    ODD
};

/**
 * @brief UART stop bits configuration
 */
enum class UartStopBits : uint8_t {
    ONE = 1,
    TWO = 2
};

/**
 * @brief UART configuration structure
 */
struct UartConfig {
    uint32_t baud_rate;
    uint8_t data_bits;       // Usually 8
    UartParity parity;
    UartStopBits stop_bits;
};

/**
 * @brief Standard UART configurations
 */
namespace UartPresets {
    /// Standard debug console (115200 8N1)
    constexpr UartConfig DEBUG = {
        .baud_rate = 115200,
        .data_bits = 8,
        .parity = UartParity::NONE,
        .stop_bits = UartStopBits::ONE
    };

    /// Fast GPS (921600 8N1)
    constexpr UartConfig GPS_FAST = {
        .baud_rate = 921600,
        .data_bits = 8,
        .parity = UartParity::NONE,
        .stop_bits = UartStopBits::ONE
    };

    /// Standard GPS (9600 8N1 - uBlox default)
    constexpr UartConfig GPS_DEFAULT = {
        .baud_rate = 9600,
        .data_bits = 8,
        .parity = UartParity::NONE,
        .stop_bits = UartStopBits::ONE
    };

    /// MAVLink telemetry (57600 8N1)
    constexpr UartConfig MAVLINK = {
        .baud_rate = 57600,
        .data_bits = 8,
        .parity = UartParity::NONE,
        .stop_bits = UartStopBits::ONE
    };
}


/**
 * @brief UART serial port
 * 
 * Wraps Pico SDK UART functions with buffered I/O.
 * 
 * @code
 * UART gps(uart1, GPIO_GPS_TX, GPIO_GPS_RX, UartPresets::GPS_DEFAULT);
 * gps.begin();
 * 
 * // Reading GPS data
 * while (gps.available()) {
 *     char c = gps.read();
 *     parseNMEA(c);
 * }
 * @endcode
 */
class UART {
public:
    /**
     * @brief Construct UART instance
     * @param uart_inst Pico SDK UART instance (uart0 or uart1)
     * @param tx_pin TX GPIO pin
     * @param rx_pin RX GPIO pin
     * @param config UART configuration
     */
    UART(void* uart_inst, uint8_t tx_pin, uint8_t rx_pin, const UartConfig& config);

    /**
     * @brief Destructor - releases UART resources
     */
    ~UART();

    /**
     * @brief Initialize UART
     * @return true if initialization successful
     */
    bool begin();

    /**
     * @brief Change baud rate
     * @param baud New baud rate
     * @note UART must be initialized first
     */
    void setBaudRate(uint32_t baud);

    /**
     * @brief Check if data available to read
     * @return Number of bytes available
     */
    size_t available() const;

    /**
     * @brief Read a single byte
     * @return Byte read, or -1 if no data available
     */
    int read();

    /**
     * @brief Read multiple bytes
     * @param buffer Output buffer
     * @param length Maximum bytes to read
     * @return Number of bytes actually read
     */
    size_t read(uint8_t* buffer, size_t length);

    /**
     * @brief Read until delimiter or buffer full
     * @param buffer Output buffer
     * @param length Maximum bytes to read
     * @param delimiter Stop character (e.g., '\n')
     * @return Number of bytes read (including delimiter if found)
     */
    size_t readUntil(uint8_t* buffer, size_t length, char delimiter);

    /**
     * @brief Peek at next byte without removing from buffer
     * @return Next byte, or -1 if no data available
     */
    int peek() const;

    /**
     * @brief Write a single byte
     * @param data Byte to write
     * @return true if successful
     */
    bool write(uint8_t data);

    /**
     * @brief Write multiple bytes
     * @param buffer Data to write
     * @param length Number of bytes to write
     * @return Number of bytes actually written
     */
    size_t write(const uint8_t* buffer, size_t length);

    /**
     * @brief Write null-terminated string
     * @param str String to write
     * @return Number of bytes written
     */
    size_t print(const char* str);

    /**
     * @brief Write string with newline
     * @param str String to write
     * @return Number of bytes written (including newline)
     */
    size_t println(const char* str);

    /**
     * @brief Flush TX buffer (wait for all data to be sent)
     */
    void flush();

    /**
     * @brief Clear RX buffer
     */
    void clear();

    /**
     * @brief Check if UART is initialized
     */
    bool isInitialized() const { return m_initialized; }

private:
    void* m_uart;
    uint8_t m_tx_pin;
    uint8_t m_rx_pin;
    UartConfig m_config;
    bool m_initialized;

    // Non-copyable
    UART(const UART&) = delete;
    UART& operator=(const UART&) = delete;
};


/**
 * @brief USB CDC serial (virtual COM port)
 * 
 * Wrapper for USB serial communication, typically used for debug output
 * during development. Uses TinyUSB CDC interface.
 */
class USBSerial {
public:
    /**
     * @brief Initialize USB CDC
     * @return true if USB enumerated
     */
    static bool begin();

    /**
     * @brief Check if USB host is connected
     * @return true if connected and DTR asserted
     */
    static bool connected();

    /**
     * @brief Check if data available
     * @return Number of bytes available
     */
    static size_t available();

    /**
     * @brief Read a byte
     * @return Byte read, or -1 if none available
     */
    static int read();

    /**
     * @brief Read multiple bytes
     */
    static size_t read(uint8_t* buffer, size_t length);

    /**
     * @brief Write a byte
     */
    static bool write(uint8_t data);

    /**
     * @brief Write multiple bytes
     */
    static size_t write(const uint8_t* buffer, size_t length);

    /**
     * @brief Write null-terminated string
     */
    static size_t print(const char* str);

    /**
     * @brief Write string with newline
     */
    static size_t println(const char* str);

    /**
     * @brief Printf-style formatted output
     */
    static size_t printf(const char* format, ...);

    /**
     * @brief Flush TX buffer
     */
    static void flush();

private:
    USBSerial() = delete;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_UART_H
