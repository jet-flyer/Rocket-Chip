/**
 * @file GPIO.cpp
 * @brief AP_HAL GPIO implementation for RP2350
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "GPIO.h"     // AP_HAL_RP2350/GPIO.h (this file's header)
#include "Scheduler.h"

// RocketChip HAL - use "hal/" prefix to avoid conflict with local GPIO.h
#include "hal/GPIO.h"  // rocketchip::hal::GPIO, PinMode, PinEdge

// Pico SDK for USB connected check (only if USB stdio is enabled)
#if LIB_PICO_STDIO_USB
#include "pico/stdio_usb.h"
#endif

namespace RP2350 {

// ============================================================================
// DigitalSource_RP2350
// ============================================================================

DigitalSource_RP2350::DigitalSource_RP2350(uint8_t pin)
    : m_pin(pin)
{
}

void DigitalSource_RP2350::mode(uint8_t output) {
    using namespace rocketchip::hal;
    if (output == HAL_GPIO_OUTPUT) {
        GPIO::pinMode(m_pin, PinMode::OUTPUT);
    } else {
        GPIO::pinMode(m_pin, PinMode::INPUT);
    }
}

uint8_t DigitalSource_RP2350::read() {
    return rocketchip::hal::GPIO::readBool(m_pin) ? 1 : 0;
}

void DigitalSource_RP2350::write(uint8_t value) {
    rocketchip::hal::GPIO::write(m_pin, value != 0);
}

void DigitalSource_RP2350::toggle() {
    rocketchip::hal::GPIO::toggle(m_pin);
}

// ============================================================================
// GPIO_RP2350
// ============================================================================

GPIO_RP2350::GPIO_RP2350()
    : m_channels{}
    , m_channel_pins{}
    , m_num_channels(0)
{
    // Initialize channel pins to invalid
    for (uint8_t i = 0; i < kMaxChannels; i++) {
        m_channel_pins[i] = 0xFF;
    }
}

void GPIO_RP2350::init() {
    // No-op: Pico SDK handles GPIO initialization
    // Individual pins are configured via pinMode()
}

void GPIO_RP2350::pinMode(uint8_t pin, uint8_t output) {
    using namespace rocketchip::hal;

    if (!valid_pin(pin)) {
        return;
    }

    switch (output) {
        case HAL_GPIO_OUTPUT:
            GPIO::pinMode(pin, PinMode::OUTPUT);
            break;
        case HAL_GPIO_INPUT:
        case HAL_GPIO_ALT:
        default:
            GPIO::pinMode(pin, PinMode::INPUT);
            break;
    }
}

uint8_t GPIO_RP2350::read(uint8_t pin) {
    if (!valid_pin(pin)) {
        return 0;
    }
    return rocketchip::hal::GPIO::readBool(pin) ? 1 : 0;
}

void GPIO_RP2350::write(uint8_t pin, uint8_t value) {
    if (!valid_pin(pin)) {
        return;
    }
    rocketchip::hal::GPIO::write(pin, value != 0);
}

void GPIO_RP2350::toggle(uint8_t pin) {
    if (!valid_pin(pin)) {
        return;
    }
    rocketchip::hal::GPIO::toggle(pin);
}

bool GPIO_RP2350::valid_pin(uint8_t pin) const {
    // RP2350 has GPIO 0-29 usable (GPIO 30-47 are QSPI/internal)
    return pin < kMaxPins;
}

DigitalSource_RP2350* GPIO_RP2350::channel(uint16_t n) {
    if (n >= kMaxPins) {
        return nullptr;
    }

    // Check if we already have a channel for this pin
    for (uint8_t i = 0; i < m_num_channels; i++) {
        if (m_channel_pins[i] == n) {
            return &m_channels[i];
        }
    }

    // Create new channel if space available
    if (m_num_channels >= kMaxChannels) {
        // No more space - return nullptr
        // In practice, ArduPilot rarely needs more than a few DigitalSource objects
        return nullptr;
    }

    // Initialize new channel
    uint8_t idx = m_num_channels++;
    m_channel_pins[idx] = static_cast<uint8_t>(n);
    m_channels[idx] = DigitalSource_RP2350(static_cast<uint8_t>(n));

    return &m_channels[idx];
}

bool GPIO_RP2350::usb_connected() {
#if LIB_PICO_STDIO_USB
    return stdio_usb_connected();
#else
    // USB stdio not enabled - can't check connection
    return false;
#endif
}

bool GPIO_RP2350::attach_interrupt(uint8_t pin,
                                    void (*fn)(uint8_t, bool, uint32_t),
                                    uint8_t mode) {
    using namespace rocketchip::hal;

    if (!valid_pin(pin) || fn == nullptr) {
        // Null callback means detach
        if (fn == nullptr) {
            GPIO::detachInterrupt(pin);
            return true;
        }
        return false;
    }

    // Map AP_HAL interrupt mode to RocketChip PinEdge
    PinEdge edge;
    switch (mode) {
        case AP_HAL::GPIO::INTERRUPT_RISING:
            edge = PinEdge::RISING;
            break;
        case AP_HAL::GPIO::INTERRUPT_FALLING:
            edge = PinEdge::FALLING;
            break;
        case AP_HAL::GPIO::INTERRUPT_BOTH:
            edge = PinEdge::BOTH;
            break;
        case AP_HAL::GPIO::INTERRUPT_NONE:
        default:
            GPIO::detachInterrupt(pin);
            return true;
    }

    // RocketChip GPIO callback signature differs slightly
    // AP_HAL: (pin, state, timestamp_us)
    // RocketChip: (pin, state) - we need to add timestamp
    //
    // For now, we create a wrapper. In a full implementation,
    // this would need static callback storage per pin.
    //
    // TODO: Implement proper callback wrapper with timestamp
    // For Phase 2, interrupt support is lower priority - most ArduPilot
    // sensor drivers use polling via periodic callbacks.

    return false;  // Not fully implemented yet
}

}  // namespace RP2350
