/**
 * @file AnalogIn.h
 * @brief AP_HAL AnalogIn implementation for RP2350
 *
 * Wraps RocketChip hal::ADC to provide AP_HAL::AnalogIn interface.
 *
 * Pin mapping (RP2350):
 * - Pin 26: ADC Channel 0
 * - Pin 27: ADC Channel 1
 * - Pin 28: ADC Channel 2
 * - Pin 29: ADC Channel 3
 * - ANALOG_INPUT_BOARD_VCC (254): Returns 3.3V nominal
 *
 * @note RP2350-E9 Erratum: ADC inputs may experience leakage from floating
 *       GPIO pins. Use external pull-downs on unused ADC channels if precision
 *       is critical. See docs/RP2350_FULL_AP_PORT.md PD9 for details.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>

// AP_HAL base classes for inheritance
#include <AP_HAL/AnalogIn.h>

namespace RP2350 {

// Forward declare to avoid circular includes
class AnalogIn_RP2350;

/**
 * @brief AnalogSource implementation for single ADC channel
 *
 * Provides object-oriented access to a single ADC channel,
 * as required by AP_HAL::AnalogIn::channel().
 */
class AnalogSource_RP2350 : public AP_HAL::AnalogSource {
public:
    friend class AnalogIn_RP2350;

    AnalogSource_RP2350();

    /**
     * @brief Read averaged raw ADC value
     * @return Averaged raw value (0-4095 scaled to float)
     */
    float read_average() override;

    /**
     * @brief Read latest raw ADC value
     * @return Latest raw value (0-4095 scaled to float)
     */
    float read_latest() override;

    /**
     * @brief Set the ADC pin
     * @param pin GPIO pin number (26-29 for ADC, 254 for board VCC)
     * @return true if pin is valid ADC pin
     */
    bool set_pin(uint8_t pin) override;

    /**
     * @brief Read averaged voltage
     * @return Actual voltage in volts (0-3.3V for RP2350)
     * @note Returns true measured voltage per ArduPilot convention for 3.3V boards
     *       (see ArduPilot PR #18754 for ADC scaling discussion)
     */
    float voltage_average() override;

    /**
     * @brief Read latest voltage
     * @return Actual voltage in volts (0-3.3V for RP2350)
     */
    float voltage_latest() override;

    /**
     * @brief Read averaged voltage assuming ratiometric sensor
     * @return Actual voltage in volts
     * @note Same as voltage_average() for RP2350 (no separate ratiometric ref)
     */
    float voltage_average_ratiometric() override;

private:
    void set_channel(uint8_t channel);

    uint8_t m_pin;       // GPIO pin number (26-29, or 254 for VCC)
    uint8_t m_channel;   // ADC channel (0-4)
    float m_last_value;  // Last raw reading (for read_latest)
    bool m_configured;   // Pin has been configured
};


/**
 * @brief AnalogIn implementation for RP2350
 *
 * Wraps RocketChip's static ADC class to provide the AP_HAL::AnalogIn interface.
 */
class AnalogIn_RP2350 : public AP_HAL::AnalogIn {
public:
    AnalogIn_RP2350();
    ~AnalogIn_RP2350() = default;

    // Prevent copying
    AnalogIn_RP2350(const AnalogIn_RP2350&) = delete;
    AnalogIn_RP2350& operator=(const AnalogIn_RP2350&) = delete;

    /**
     * @brief Initialize ADC subsystem
     */
    void init() override;

    /**
     * @brief Get AnalogSource for a pin
     * @param n GPIO pin number (26-29 for ADC, 254 for board VCC)
     * @return Pointer to AnalogSource (caller does NOT own - static storage)
     */
    AP_HAL::AnalogSource* channel(int16_t n) override;

    /**
     * @brief Check if pin is valid for analog input
     * @param pin GPIO pin number
     * @return true if pin can be used for ADC (26-29 or 254)
     */
    bool valid_analog_pin(uint16_t pin) const override;

    /**
     * @brief Read board voltage (3.3V rail)
     * @return Board voltage in volts (nominally 3.3V)
     */
    float board_voltage(void) override;

    /**
     * @brief Read servo rail voltage (not implemented)
     * @return 0 (no servo rail monitoring on this board)
     */
    float servorail_voltage(void) override { return 0.0f; }

    /**
     * @brief Power status flags (minimal implementation)
     * @return MAV_POWER_STATUS flags
     */
    uint16_t power_status_flags(void) override;

    /**
     * @brief Accumulated power status flags
     * @return All flags ever set
     */
    uint16_t accumulated_power_status_flags(void) const override { return m_power_flags; }

#if HAL_WITH_MCU_MONITORING
    /**
     * @brief Read MCU internal temperature
     * @return Temperature in degrees Celsius
     */
    float mcu_temperature(void) override;

    /**
     * @brief Read MCU voltage (same as board_voltage)
     * @return Voltage in volts
     */
    float mcu_voltage(void) override { return board_voltage(); }
#endif

private:
    static constexpr uint8_t kMaxChannels = 6;  // 4 external + VCC + temp
    static constexpr uint8_t kFirstAdcPin = 26;
    static constexpr uint8_t kLastAdcPin = 29;
    static constexpr uint8_t kBoardVccPin = 254;
    static constexpr uint8_t kTempChannel = 4;

    // Static storage for AnalogSource objects
    AnalogSource_RP2350 m_channels[kMaxChannels];
    uint8_t m_channel_pins[kMaxChannels];
    uint8_t m_num_channels;

    uint16_t m_power_flags;
    bool m_initialized;
};

}  // namespace RP2350
