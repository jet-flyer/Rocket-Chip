/**
 * @file UART.cpp
 * @brief Serial communication abstraction implementation
 *
 * Implements UART and USB CDC serial communication using Pico SDK
 * and TinyUSB.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "UART.h"

#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"

// TinyUSB for USB CDC - requires project to have tusb_config.h
// Only include if we have a config (check for stdio_usb which provides it)
#if defined(LIB_PICO_STDIO_USB) && LIB_PICO_STDIO_USB
#include "tusb.h"
#define HAS_TINYUSB 1
#else
#define HAS_TINYUSB 0
#endif

#include <cstdarg>
#include <cstdio>
#include <cstring>

namespace rocketchip {
namespace hal {

// ============================================================================
// UART class implementation
// ============================================================================

UART::UART(void* uart_inst, uint8_t tx_pin, uint8_t rx_pin, const UartConfig& config)
    : m_uart(uart_inst)
    , m_tx_pin(tx_pin)
    , m_rx_pin(rx_pin)
    , m_config(config)
    , m_initialized(false)
{
}

UART::~UART() {
    if (m_initialized) {
        uart_deinit(static_cast<uart_inst_t*>(m_uart));
    }
}

bool UART::begin() {
    if (m_initialized) {
        return true;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);

    // Initialize UART at specified baud rate
    uart_init(uart, m_config.baud_rate);

    // Configure GPIO pins for UART
    gpio_set_function(m_tx_pin, GPIO_FUNC_UART);
    gpio_set_function(m_rx_pin, GPIO_FUNC_UART);

    // Set format (data bits, stop bits, parity)
    uart_parity_t parity;
    switch (m_config.parity) {
        case UartParity::EVEN:
            parity = UART_PARITY_EVEN;
            break;
        case UartParity::ODD:
            parity = UART_PARITY_ODD;
            break;
        case UartParity::NONE:
        default:
            parity = UART_PARITY_NONE;
            break;
    }

    uart_set_format(uart, m_config.data_bits,
                    static_cast<uint>(m_config.stop_bits), parity);

    // Enable FIFO
    uart_set_fifo_enabled(uart, true);

    m_initialized = true;
    return true;
}

void UART::setBaudRate(uint32_t baud) {
    if (!m_initialized) {
        return;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);
    uart_set_baudrate(uart, baud);
    m_config.baud_rate = baud;
}

size_t UART::available() const {
    if (!m_initialized) {
        return 0;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);
    return uart_is_readable(uart) ? 1 : 0;  // UART doesn't provide count
}

int UART::read() {
    if (!m_initialized) {
        return -1;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);

    if (!uart_is_readable(uart)) {
        return -1;
    }

    return uart_getc(uart);
}

size_t UART::read(uint8_t* buffer, size_t length) {
    if (!m_initialized || buffer == nullptr || length == 0) {
        return 0;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);
    size_t count = 0;

    while (count < length && uart_is_readable(uart)) {
        buffer[count++] = static_cast<uint8_t>(uart_getc(uart));
    }

    return count;
}

size_t UART::readUntil(uint8_t* buffer, size_t length, char delimiter) {
    if (!m_initialized || buffer == nullptr || length == 0) {
        return 0;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);
    size_t count = 0;

    while (count < length && uart_is_readable(uart)) {
        uint8_t c = static_cast<uint8_t>(uart_getc(uart));
        buffer[count++] = c;

        if (c == static_cast<uint8_t>(delimiter)) {
            break;
        }
    }

    return count;
}

int UART::peek() const {
    // Pico SDK UART doesn't have a peek function
    // Would need to implement with a local buffer
    return -1;
}

bool UART::write(uint8_t data) {
    if (!m_initialized) {
        return false;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);
    uart_putc_raw(uart, data);
    return true;
}

size_t UART::write(const uint8_t* buffer, size_t length) {
    if (!m_initialized || buffer == nullptr) {
        return 0;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);

    for (size_t i = 0; i < length; ++i) {
        uart_putc_raw(uart, buffer[i]);
    }

    return length;
}

size_t UART::print(const char* str) {
    if (str == nullptr) {
        return 0;
    }

    return write(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

size_t UART::println(const char* str) {
    size_t count = print(str);
    count += print("\r\n");
    return count;
}

void UART::flush() {
    if (!m_initialized) {
        return;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);

    // Wait for TX FIFO to drain
    uart_tx_wait_blocking(uart);
}

void UART::clear() {
    if (!m_initialized) {
        return;
    }

    uart_inst_t* uart = static_cast<uart_inst_t*>(m_uart);

    // Read and discard any pending data
    while (uart_is_readable(uart)) {
        uart_getc(uart);
    }
}

// ============================================================================
// USBSerial class implementation
// ============================================================================

#if HAS_TINYUSB

bool USBSerial::begin() {
    // TinyUSB initialization is typically done by pico_stdlib
    // Just ensure USB task is running
    tusb_init();

    // Wait briefly for USB enumeration
    // Note: This is a simplified approach; full enumeration may take longer
    for (int i = 0; i < 100; ++i) {
        tud_task();
        if (tud_cdc_connected()) {
            return true;
        }
        sleep_ms(10);
    }

    return tud_ready();
}

bool USBSerial::connected() {
    tud_task();  // Process USB events
    return tud_cdc_connected();
}

size_t USBSerial::available() {
    tud_task();
    return tud_cdc_available();
}

int USBSerial::read() {
    tud_task();

    if (tud_cdc_available() == 0) {
        return -1;
    }

    uint8_t c;
    if (tud_cdc_read(&c, 1) == 1) {
        return c;
    }

    return -1;
}

size_t USBSerial::read(uint8_t* buffer, size_t length) {
    if (buffer == nullptr || length == 0) {
        return 0;
    }

    tud_task();
    return tud_cdc_read(buffer, length);
}

bool USBSerial::write(uint8_t data) {
    tud_task();

    if (!tud_cdc_connected()) {
        return false;
    }

    // Wait for space in TX buffer
    while (!tud_cdc_write_available()) {
        tud_task();
    }

    return tud_cdc_write(&data, 1) == 1;
}

size_t USBSerial::write(const uint8_t* buffer, size_t length) {
    if (buffer == nullptr || length == 0) {
        return 0;
    }

    tud_task();

    if (!tud_cdc_connected()) {
        return 0;
    }

    size_t written = 0;
    while (written < length) {
        size_t available = tud_cdc_write_available();
        if (available == 0) {
            tud_task();
            tud_cdc_write_flush();
            continue;
        }

        size_t to_write = length - written;
        if (to_write > available) {
            to_write = available;
        }

        size_t n = tud_cdc_write(&buffer[written], to_write);
        written += n;

        tud_cdc_write_flush();
    }

    return written;
}

size_t USBSerial::print(const char* str) {
    if (str == nullptr) {
        return 0;
    }

    return write(reinterpret_cast<const uint8_t*>(str), strlen(str));
}

size_t USBSerial::println(const char* str) {
    size_t count = print(str);
    count += print("\r\n");
    return count;
}

size_t USBSerial::printf(const char* format, ...) {
    char buffer[256];

    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return 0;
    }

    if (static_cast<size_t>(len) >= sizeof(buffer)) {
        len = sizeof(buffer) - 1;
    }

    return write(reinterpret_cast<const uint8_t*>(buffer), len);
}

void USBSerial::flush() {
    tud_task();
    tud_cdc_write_flush();
}

#else // HAS_TINYUSB not defined - provide stub implementations

bool USBSerial::begin() { return false; }
bool USBSerial::connected() { return false; }
size_t USBSerial::available() { return 0; }
int USBSerial::read() { return -1; }
size_t USBSerial::read(uint8_t*, size_t) { return 0; }
bool USBSerial::write(uint8_t) { return false; }
size_t USBSerial::write(const uint8_t*, size_t) { return 0; }
size_t USBSerial::print(const char*) { return 0; }
size_t USBSerial::println(const char*) { return 0; }
size_t USBSerial::printf(const char*, ...) { return 0; }
void USBSerial::flush() {}

#endif // HAS_TINYUSB

} // namespace hal
} // namespace rocketchip
