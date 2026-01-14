/**
 * @file GPIO.cpp
 * @brief Digital I/O abstraction implementation
 *
 * Implements GPIO operations including input/output, pull configuration,
 * and interrupt handling using Pico SDK.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "GPIO.h"
#include "Timing.h"

#include "hardware/gpio.h"

#include <cstring>

namespace rocketchip {
namespace hal {

// ============================================================================
// Internal callback storage
// ============================================================================

namespace {
    // Maximum number of GPIO pins on RP2350
    constexpr uint8_t MAX_GPIO_PINS = 48;

    // Storage for GPIO callbacks
    GpioCallback s_callbacks[MAX_GPIO_PINS];

    // Master GPIO IRQ handler
    void gpio_irq_handler(uint gpio, uint32_t events) {
        if (gpio < MAX_GPIO_PINS && s_callbacks[gpio]) {
            PinState state = (events & GPIO_IRQ_EDGE_RISE) ? PinState::HIGH : PinState::LOW;
            s_callbacks[gpio](static_cast<uint8_t>(gpio), state);
        }
    }
}

// ============================================================================
// GPIO class implementation
// ============================================================================

void GPIO::pinMode(uint8_t pin, PinMode mode) {
    gpio_init(pin);

    switch (mode) {
        case PinMode::INPUT:
            gpio_set_dir(pin, GPIO_IN);
            gpio_disable_pulls(pin);
            break;

        case PinMode::OUTPUT:
            gpio_set_dir(pin, GPIO_OUT);
            break;

        case PinMode::INPUT_PULLUP:
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_up(pin);
            break;

        case PinMode::INPUT_PULLDOWN:
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_down(pin);
            break;
    }
}

void GPIO::write(uint8_t pin, PinState state) {
    gpio_put(pin, state == PinState::HIGH);
}

void GPIO::write(uint8_t pin, bool high) {
    gpio_put(pin, high);
}

PinState GPIO::read(uint8_t pin) {
    return gpio_get(pin) ? PinState::HIGH : PinState::LOW;
}

bool GPIO::readBool(uint8_t pin) {
    return gpio_get(pin);
}

void GPIO::toggle(uint8_t pin) {
    gpio_xor_mask(1u << pin);
}

void GPIO::attachInterrupt(uint8_t pin, PinEdge edge, GpioCallback callback) {
    if (pin >= MAX_GPIO_PINS) {
        return;
    }

    // Store callback
    s_callbacks[pin] = callback;

    // Convert edge to Pico SDK event mask
    uint32_t events = 0;
    switch (edge) {
        case PinEdge::RISING:
            events = GPIO_IRQ_EDGE_RISE;
            break;
        case PinEdge::FALLING:
            events = GPIO_IRQ_EDGE_FALL;
            break;
        case PinEdge::BOTH:
            events = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
            break;
        case PinEdge::NONE:
        default:
            events = 0;
            break;
    }

    // Enable IRQ with callback
    gpio_set_irq_enabled_with_callback(pin, events, true, gpio_irq_handler);
}

void GPIO::detachInterrupt(uint8_t pin) {
    if (pin >= MAX_GPIO_PINS) {
        return;
    }

    // Disable all edge interrupts on this pin
    gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, false);

    // Clear callback
    s_callbacks[pin] = nullptr;
}

PinState GPIO::readDebounced(uint8_t pin, uint32_t debounce_ms) {
    // Read initial state
    PinState initial_state = read(pin);

    // Wait for debounce period, checking if state remains stable
    uint32_t start_ms = Timing::millis32();
    uint32_t debounce_us = debounce_ms * 1000;
    uint32_t check_interval_us = 1000;  // Check every 1ms

    while (Timing::elapsedMs(start_ms) < debounce_ms) {
        Timing::delayMicros(check_interval_us);

        PinState current_state = read(pin);
        if (current_state != initial_state) {
            // State changed, restart debounce
            initial_state = current_state;
            start_ms = Timing::millis32();
        }
    }

    return initial_state;
}

// ============================================================================
// OutputPin class implementation
// ============================================================================

OutputPin::OutputPin(uint8_t pin, PinState initial_state)
    : m_pin(pin)
{
    GPIO::pinMode(pin, PinMode::OUTPUT);
    GPIO::write(pin, initial_state);
}

// ============================================================================
// InputPin class implementation
// ============================================================================

InputPin::InputPin(uint8_t pin, PinMode mode)
    : m_pin(pin)
{
    GPIO::pinMode(pin, mode);
}

} // namespace hal
} // namespace rocketchip
