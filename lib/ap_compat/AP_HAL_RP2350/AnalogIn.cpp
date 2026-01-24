/**
 * @file AnalogIn.cpp
 * @brief AP_HAL AnalogIn implementation for RP2350
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "AnalogIn.h"

// RocketChip HAL - use "hal/" prefix to avoid conflicts
#include "hal/ADC.h"

namespace RP2350 {

// ============================================================================
// AnalogSource_RP2350
// ============================================================================

AnalogSource_RP2350::AnalogSource_RP2350()
    : m_pin(255)  // ANALOG_INPUT_NONE
    , m_channel(255)
    , m_last_value(0.0f)
    , m_configured(false)
{
}

void AnalogSource_RP2350::set_channel(uint8_t channel) {
    m_channel = channel;
    m_configured = false;
}

bool AnalogSource_RP2350::set_pin(uint8_t pin) {
    // Check for special pins
    if (pin == 254) {  // ANALOG_INPUT_BOARD_VCC
        m_pin = pin;
        m_channel = 255;  // Special marker for board VCC
        m_configured = true;
        return true;
    }

    // Check if valid ADC pin (26-29)
    if (pin < 26 || pin > 29) {
        return false;
    }

    m_pin = pin;
    m_channel = pin - 26;  // GPIO26 = channel 0, etc.
    m_configured = false;  // Will configure on first read
    return true;
}

float AnalogSource_RP2350::read_average() {
    using namespace rocketchip::hal;

    // Board VCC returns nominal value
    if (m_pin == 254) {
        m_last_value = 4095.0f;  // Full scale (represents 3.3V)
        return m_last_value;
    }

    if (m_channel > 4) {
        return 0.0f;
    }

    // Configure pin on first read
    if (!m_configured && m_channel < 4) {
        ADC::configurePin(static_cast<AdcChannel>(m_channel));
        m_configured = true;
    }

    // Read averaged value (16 samples)
    AdcChannel ch = static_cast<AdcChannel>(m_channel);
    uint16_t raw = ADC::readAveraged(ch, 16);
    m_last_value = static_cast<float>(raw);

    return m_last_value;
}

float AnalogSource_RP2350::read_latest() {
    using namespace rocketchip::hal;

    // Board VCC returns nominal value
    if (m_pin == 254) {
        m_last_value = 4095.0f;
        return m_last_value;
    }

    if (m_channel > 4) {
        return 0.0f;
    }

    // Configure pin on first read
    if (!m_configured && m_channel < 4) {
        ADC::configurePin(static_cast<AdcChannel>(m_channel));
        m_configured = true;
    }

    // Read single value
    AdcChannel ch = static_cast<AdcChannel>(m_channel);
    uint16_t raw = ADC::readRaw(ch);
    m_last_value = static_cast<float>(raw);

    return m_last_value;
}

float AnalogSource_RP2350::voltage_average() {
    using namespace rocketchip::hal;

    // Board VCC returns 3.3V
    if (m_pin == 254) {
        return 3.3f;
    }

    if (m_channel > 4) {
        return 0.0f;
    }

    // Configure pin on first read
    if (!m_configured && m_channel < 4) {
        ADC::configurePin(static_cast<AdcChannel>(m_channel));
        m_configured = true;
    }

    // Read averaged voltage directly from ADC
    AdcChannel ch = static_cast<AdcChannel>(m_channel);
    float voltage = ADC::readAveragedVoltage(ch, 16);

    // Store raw equivalent for read_latest consistency
    m_last_value = (voltage / ADC::VREF) * static_cast<float>(ADC::MAX_VALUE);

    return voltage;
}

float AnalogSource_RP2350::voltage_latest() {
    using namespace rocketchip::hal;

    // Board VCC returns 3.3V
    if (m_pin == 254) {
        return 3.3f;
    }

    if (m_channel > 4) {
        return 0.0f;
    }

    // Configure pin on first read
    if (!m_configured && m_channel < 4) {
        ADC::configurePin(static_cast<AdcChannel>(m_channel));
        m_configured = true;
    }

    // Read single voltage
    AdcChannel ch = static_cast<AdcChannel>(m_channel);
    float voltage = ADC::readVoltage(ch);

    m_last_value = (voltage / ADC::VREF) * static_cast<float>(ADC::MAX_VALUE);

    return voltage;
}

float AnalogSource_RP2350::voltage_average_ratiometric() {
    // For RP2350, ratiometric is same as regular voltage reading
    // (no separate ratiometric reference)
    return voltage_average();
}

// ============================================================================
// AnalogIn_RP2350
// ============================================================================

AnalogIn_RP2350::AnalogIn_RP2350()
    : m_channels{}
    , m_channel_pins{}
    , m_num_channels(0)
    , m_power_flags(0)
    , m_initialized(false)
{
    // Initialize channel pins to invalid
    for (uint8_t i = 0; i < kMaxChannels; i++) {
        m_channel_pins[i] = 255;  // ANALOG_INPUT_NONE
    }
}

void AnalogIn_RP2350::init() {
    if (m_initialized) {
        return;
    }

    // Initialize RocketChip ADC
    rocketchip::hal::ADC::init();

    m_initialized = true;
}

bool AnalogIn_RP2350::valid_analog_pin(uint16_t pin) const {
    // Valid pins: 26-29 (ADC channels) or 254 (board VCC)
    return (pin >= kFirstAdcPin && pin <= kLastAdcPin) ||
           (pin == kBoardVccPin);
}

AnalogSource_RP2350* AnalogIn_RP2350::channel(int16_t n) {
    // Validate pin
    if (n < 0) {
        return nullptr;
    }

    uint16_t pin = static_cast<uint16_t>(n);
    if (!valid_analog_pin(pin)) {
        return nullptr;
    }

    // Check if we already have a channel for this pin
    for (uint8_t i = 0; i < m_num_channels; i++) {
        if (m_channel_pins[i] == pin) {
            return &m_channels[i];
        }
    }

    // Create new channel if space available
    if (m_num_channels >= kMaxChannels) {
        return nullptr;
    }

    // Initialize new channel
    uint8_t idx = m_num_channels++;
    m_channel_pins[idx] = static_cast<uint8_t>(pin);
    m_channels[idx].set_pin(static_cast<uint8_t>(pin));

    return &m_channels[idx];
}

float AnalogIn_RP2350::board_voltage() {
    // RP2350 operates at 3.3V
    // In a real implementation, this could read from a voltage divider
    // monitoring the actual supply rail
    return 3.3f;
}

uint16_t AnalogIn_RP2350::power_status_flags() {
    // Minimal implementation - just report USB status
    // Bit 2 (USB_CONNECTED) = 4
    // Bit 0 (BRICK_VALID) = 1 (we're powered)

    uint16_t flags = 1;  // BRICK_VALID - we're running

    // Note: USB connected check would require GPIO module
    // For now, assume USB is primary power source
    flags |= 4;  // USB_CONNECTED

    m_power_flags |= flags;  // Accumulate
    return flags;
}

float AnalogIn_RP2350::mcu_temperature() {
    return rocketchip::hal::ADC::readTemperature();
}

}  // namespace RP2350
