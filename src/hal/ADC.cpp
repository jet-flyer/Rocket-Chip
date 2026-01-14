/**
 * @file ADC.cpp
 * @brief Analog-to-digital converter abstraction implementation
 *
 * Implements ADC operations using Pico SDK for battery monitoring
 * and pyro continuity checking.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "ADC.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"

namespace rocketchip {
namespace hal {

// ============================================================================
// Static member initialization
// ============================================================================

bool ADC::s_initialized = false;

// ============================================================================
// ADC class implementation
// ============================================================================

void ADC::init() {
    if (s_initialized) {
        return;
    }

    adc_init();
    s_initialized = true;
}

void ADC::configurePin(AdcChannel channel) {
    if (channel == AdcChannel::TEMPERATURE) {
        // Temperature sensor doesn't need pin configuration
        adc_set_temp_sensor_enabled(true);
        return;
    }

    // Map channel to GPIO pin
    // ADC channels 0-3 map to GPIO 26-29
    uint8_t gpio_pin = 26 + static_cast<uint8_t>(channel);
    adc_gpio_init(gpio_pin);
}

uint16_t ADC::readRaw(AdcChannel channel) {
    if (!s_initialized) {
        init();
    }

    // Configure pin if external channel
    if (channel != AdcChannel::TEMPERATURE) {
        configurePin(channel);
    } else {
        adc_set_temp_sensor_enabled(true);
    }

    // Select and read ADC channel
    adc_select_input(static_cast<uint8_t>(channel));
    return adc_read();
}

float ADC::readVoltage(AdcChannel channel) {
    uint16_t raw = readRaw(channel);
    return (static_cast<float>(raw) / static_cast<float>(MAX_VALUE)) * VREF;
}

uint16_t ADC::readAveraged(AdcChannel channel, uint8_t samples) {
    if (!s_initialized) {
        init();
    }

    if (samples == 0) {
        samples = 1;
    }

    // Configure pin if external channel
    if (channel != AdcChannel::TEMPERATURE) {
        configurePin(channel);
    } else {
        adc_set_temp_sensor_enabled(true);
    }

    // Select ADC channel
    adc_select_input(static_cast<uint8_t>(channel));

    // Accumulate samples
    uint32_t accumulator = 0;
    for (uint8_t i = 0; i < samples; ++i) {
        accumulator += adc_read();
    }

    return static_cast<uint16_t>(accumulator / samples);
}

float ADC::readAveragedVoltage(AdcChannel channel, uint8_t samples) {
    uint16_t raw = readAveraged(channel, samples);
    return (static_cast<float>(raw) / static_cast<float>(MAX_VALUE)) * VREF;
}

float ADC::readTemperature() {
    if (!s_initialized) {
        init();
    }

    // Enable temperature sensor
    adc_set_temp_sensor_enabled(true);
    adc_select_input(static_cast<uint8_t>(AdcChannel::TEMPERATURE));

    // Read multiple samples for stability
    uint16_t raw = readAveraged(AdcChannel::TEMPERATURE, 16);

    // Convert to voltage
    float voltage = (static_cast<float>(raw) / static_cast<float>(MAX_VALUE)) * VREF;

    // RP2350 temperature sensor formula (from datasheet)
    // T = 27 - (ADC_voltage - 0.706) / 0.001721
    // Note: Calibration may vary between chips
    float temperature = 27.0f - (voltage - 0.706f) / 0.001721f;

    return temperature;
}

// ============================================================================
// BatteryMonitor class implementation
// ============================================================================

BatteryMonitor::BatteryMonitor(AdcChannel channel, float divider_ratio,
                               float v_min, float v_max)
    : m_channel(channel)
    , m_divider_ratio(divider_ratio)
    , m_v_min(v_min)
    , m_v_max(v_max)
{
    // Configure ADC pin
    ADC::configurePin(channel);
}

float BatteryMonitor::readVoltage() const {
    // Read averaged ADC value
    float adc_voltage = ADC::readAveragedVoltage(m_channel, 16);

    // Apply voltage divider ratio
    return adc_voltage * m_divider_ratio;
}

float BatteryMonitor::readPercent() const {
    float voltage = readVoltage();

    // Clamp to valid range
    if (voltage <= m_v_min) {
        return 0.0f;
    }
    if (voltage >= m_v_max) {
        return 100.0f;
    }

    // Linear interpolation (simple approximation)
    // Note: LiPo discharge curve is not linear - this is a rough estimate
    float percent = ((voltage - m_v_min) / (m_v_max - m_v_min)) * 100.0f;

    return percent;
}

bool BatteryMonitor::isCritical(float threshold) const {
    return readVoltage() < threshold;
}

// ============================================================================
// PyroContinuity class implementation
// ============================================================================

PyroContinuity::PyroContinuity(AdcChannel channel, float v_open,
                               float v_good_min, float v_good_max)
    : m_channel(channel)
    , m_v_open(v_open)
    , m_v_good_min(v_good_min)
    , m_v_good_max(v_good_max)
{
    // Configure ADC pin
    ADC::configurePin(channel);
}

PyroContinuity::Status PyroContinuity::check() const {
    float voltage = readVoltage();

    // Define tolerance for open circuit detection
    constexpr float OPEN_TOLERANCE = 0.2f;

    // Check if open circuit (near expected open voltage)
    if (voltage > (m_v_open - OPEN_TOLERANCE) &&
        voltage < (m_v_open + OPEN_TOLERANCE)) {
        return Status::OPEN;
    }

    // Check if good continuity (within expected range)
    if (voltage >= m_v_good_min && voltage <= m_v_good_max) {
        return Status::GOOD;
    }

    // Check for short circuit (very low voltage)
    constexpr float SHORT_THRESHOLD = 0.1f;
    if (voltage < SHORT_THRESHOLD) {
        return Status::SHORT;
    }

    // Cannot determine state
    return Status::UNKNOWN;
}

float PyroContinuity::readVoltage() const {
    return ADC::readAveragedVoltage(m_channel, 8);
}

} // namespace hal
} // namespace rocketchip
