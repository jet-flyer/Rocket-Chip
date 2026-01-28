/**
 * @file UARTDriver.cpp
 * @brief AP_HAL UARTDriver implementation for RP2350
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "UARTDriver.h"

// RocketChip HAL - use "hal/" prefix to avoid conflicts
#include "hal/UART.h"

// Pico SDK for USB CDC check
#include "pico/stdlib.h"

#include <cstdio>
#include <cstdarg>
#include <cstring>

// Hardware UART instances from Pico SDK
extern "C" {
#include "hardware/uart.h"
}

namespace RP2350 {

// ============================================================================
// Default UART pin assignments (can be overridden in hwdef.h)
// ============================================================================

#ifndef UART0_TX_PIN
#define UART0_TX_PIN 0
#endif

#ifndef UART0_RX_PIN
#define UART0_RX_PIN 1
#endif

#ifndef UART1_TX_PIN
#define UART1_TX_PIN 4
#endif

#ifndef UART1_RX_PIN
#define UART1_RX_PIN 5
#endif

// ============================================================================
// UARTDriver_RP2350
// ============================================================================

UARTDriver_RP2350::UARTDriver_RP2350(PortType type, uint8_t tx_pin, uint8_t rx_pin)
    : AP_HAL::UARTDriver()
    , m_type(type)
    , m_tx_pin(tx_pin)
    , m_rx_pin(rx_pin)
    , m_baud(0)
    , m_initialized(false)
    , m_uart(nullptr)
{
}

UARTDriver_RP2350::~UARTDriver_RP2350() {
    _end();
}

// ============================================================================
// Protected Backend Methods
// ============================================================================

void UARTDriver_RP2350::_begin(uint32_t baud, uint16_t rxSpace, uint16_t txSpace) {
    (void)rxSpace;  // Buffer sizes are fixed in RocketChip HAL
    (void)txSpace;

    if (m_initialized) {
        // Already initialized, just update baud rate for hardware UART
        if (m_type != PortType::USB_CDC && m_uart != nullptr) {
            m_uart->setBaudRate(baud);
            m_baud = baud;
        }
        return;
    }

    m_baud = baud;

    switch (m_type) {
        case PortType::USB_CDC:
            // USB CDC is initialized via stdio_init_all() in main
            // Just mark as ready - actual USB connection may come later
            m_initialized = true;
            break;

        case PortType::UART0: {
            // Use default pins if not specified
            uint8_t tx = (m_tx_pin == 0) ? UART0_TX_PIN : m_tx_pin;
            uint8_t rx = (m_rx_pin == 0) ? UART0_RX_PIN : m_rx_pin;

            rocketchip::hal::UartConfig config = {
                .baud_rate = baud,
                .data_bits = 8,
                .parity = rocketchip::hal::UartParity::NONE,
                .stop_bits = rocketchip::hal::UartStopBits::ONE
            };

            m_uart = new rocketchip::hal::UART(uart0, tx, rx, config);
            if (m_uart->begin()) {
                m_initialized = true;
            }
            break;
        }

        case PortType::UART1: {
            // Use default pins if not specified
            uint8_t tx = (m_tx_pin == 0) ? UART1_TX_PIN : m_tx_pin;
            uint8_t rx = (m_rx_pin == 0) ? UART1_RX_PIN : m_rx_pin;

            rocketchip::hal::UartConfig config = {
                .baud_rate = baud,
                .data_bits = 8,
                .parity = rocketchip::hal::UartParity::NONE,
                .stop_bits = rocketchip::hal::UartStopBits::ONE
            };

            m_uart = new rocketchip::hal::UART(uart1, tx, rx, config);
            if (m_uart->begin()) {
                m_initialized = true;
            }
            break;
        }
    }
}

void UARTDriver_RP2350::_end() {
    if (!m_initialized) {
        return;
    }

    if (m_uart != nullptr) {
        delete m_uart;
        m_uart = nullptr;
    }

    m_initialized = false;
}

void UARTDriver_RP2350::_flush() {
    if (!m_initialized) {
        return;
    }

    if (m_type == PortType::USB_CDC) {
        rocketchip::hal::USBSerial::flush();
        return;
    }

    if (m_uart != nullptr) {
        m_uart->flush();
    }
}

uint32_t UARTDriver_RP2350::_available() {
    if (!m_initialized) {
        return 0;
    }

    if (m_type == PortType::USB_CDC) {
        return rocketchip::hal::USBSerial::available();
    }

    if (m_uart != nullptr) {
        return m_uart->available();
    }

    return 0;
}

bool UARTDriver_RP2350::_discard_input() {
    if (!m_initialized) {
        return false;
    }

    if (m_type == PortType::USB_CDC) {
        // Read and discard all available data
        while (rocketchip::hal::USBSerial::available() > 0) {
            rocketchip::hal::USBSerial::read();
        }
        return true;
    }

    if (m_uart != nullptr) {
        m_uart->clear();
        return true;
    }

    return false;
}

size_t UARTDriver_RP2350::_write(const uint8_t *buffer, size_t size) {
    if (!m_initialized || buffer == nullptr || size == 0) {
        return 0;
    }

    if (m_type == PortType::USB_CDC) {
        return rocketchip::hal::USBSerial::write(buffer, size);
    }

    if (m_uart != nullptr) {
        return m_uart->write(buffer, size);
    }

    return 0;
}

ssize_t UARTDriver_RP2350::_read(uint8_t *buffer, uint16_t count) {
    if (!m_initialized || buffer == nullptr || count == 0) {
        return -1;
    }

    if (m_type == PortType::USB_CDC) {
        return static_cast<ssize_t>(rocketchip::hal::USBSerial::read(buffer, count));
    }

    if (m_uart != nullptr) {
        return static_cast<ssize_t>(m_uart->read(buffer, count));
    }

    return -1;
}

// ============================================================================
// Public Virtual Methods
// ============================================================================

uint32_t UARTDriver_RP2350::txspace() {
    if (!m_initialized) {
        return 0;
    }

    // RocketChip HAL doesn't expose TX buffer space
    // Return a reasonable default
    if (m_type == PortType::USB_CDC) {
        // USB CDC typically has good TX buffering
        return 256;
    }

    // Hardware UART with FIFO
    return 32;
}

bool UARTDriver_RP2350::tx_pending() {
    // RocketChip HAL doesn't expose TX pending status
    // Return false (assume TX completes quickly)
    return false;
}

// ============================================================================
// Configuration
// ============================================================================

void UARTDriver_RP2350::configure_parity(uint8_t v) {
    (void)v;
    // Would need to reinitialize UART with new config
    // Not commonly used, skip for now
}

void UARTDriver_RP2350::set_stop_bits(int n) {
    (void)n;
    // Would need to reinitialize UART with new config
    // Not commonly used, skip for now
}

}  // namespace RP2350
