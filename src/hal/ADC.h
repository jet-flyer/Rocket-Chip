/**
 * @file ADC.h
 * @brief Analog-to-digital converter abstraction
 * 
 * Provides interface for reading analog voltages, primarily used for
 * battery monitoring and pyro continuity checking.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 * @note RP2350 has 12-bit ADC with 4 external channels (GPIO26-29)
 */

#ifndef ROCKETCHIP_HAL_ADC_H
#define ROCKETCHIP_HAL_ADC_H

#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief ADC channel mapping for RP2350
 * 
 * ADC channels are fixed to specific GPIO pins on RP2350:
 * - Channel 0: GPIO26
 * - Channel 1: GPIO27
 * - Channel 2: GPIO28
 * - Channel 3: GPIO29
 * - Channel 4: Internal temperature sensor
 */
enum class AdcChannel : uint8_t {
    CH0_GPIO26 = 0,
    CH1_GPIO27 = 1,
    CH2_GPIO28 = 2,
    CH3_GPIO29 = 3,
    TEMPERATURE = 4  // Internal temperature sensor
};

/**
 * @brief ADC interface
 * 
 * Wraps Pico SDK ADC functions. The RP2350 ADC is adequate for monitoring
 * tasks but is not intended for high-precision analog sensing.
 * 
 * @code
 * // Battery voltage monitoring with voltage divider
 * // Assuming 2:1 divider (100k/100k), Vbat = 2 * Vadc
 * ADC::init();
 * float vadc = ADC::readVoltage(AdcChannel::CH0_GPIO26);
 * float vbat = vadc * 2.0f;  // Account for divider
 * @endcode
 */
class ADC {
public:
    /**
     * @brief Initialize ADC subsystem
     * @note Must be called before any other ADC functions
     */
    static void init();

    /**
     * @brief Read raw ADC value
     * @param channel ADC channel to read
     * @return 12-bit raw value (0-4095)
     */
    static uint16_t readRaw(AdcChannel channel);

    /**
     * @brief Read ADC value as voltage
     * @param channel ADC channel to read
     * @return Voltage in volts (0.0 - 3.3V range)
     */
    static float readVoltage(AdcChannel channel);

    /**
     * @brief Read averaged ADC value
     * @param channel ADC channel to read
     * @param samples Number of samples to average (default 16)
     * @return Averaged 12-bit value
     * @note Takes approximately samples * 2µs
     */
    static uint16_t readAveraged(AdcChannel channel, uint8_t samples = 16);

    /**
     * @brief Read averaged ADC value as voltage
     * @param channel ADC channel to read
     * @param samples Number of samples to average (default 16)
     * @return Averaged voltage in volts
     */
    static float readAveragedVoltage(AdcChannel channel, uint8_t samples = 16);

    /**
     * @brief Read internal temperature sensor
     * @return Temperature in degrees Celsius
     */
    static float readTemperature();

    /**
     * @brief Configure ADC pin
     * 
     * Must be called to configure a GPIO pin for ADC use.
     * This disables digital input on the pin.
     * 
     * @param channel ADC channel (CH0-CH3 only, not TEMPERATURE)
     */
    static void configurePin(AdcChannel channel);

    /**
     * @brief ADC reference voltage (RP2350 = 3.3V)
     */
    static constexpr float VREF = 3.3f;

    /**
     * @brief ADC resolution (12-bit = 4096 counts)
     */
    static constexpr uint16_t MAX_VALUE = 4095;

private:
    ADC() = delete;  // Static-only class
    
    static bool s_initialized;
};


/**
 * @brief Battery voltage monitor
 * 
 * Convenience class for monitoring battery voltage through a voltage divider.
 * 
 * @code
 * // For 2:1 divider (Vbat -> 100k -> ADC -> 100k -> GND)
 * BatteryMonitor battery(AdcChannel::CH0_GPIO26, 2.0f);
 * float vbat = battery.readVoltage();
 * float percent = battery.readPercent();  // Estimated SOC
 * @endcode
 */
class BatteryMonitor {
public:
    /**
     * @brief Construct battery monitor
     * @param channel ADC channel connected to voltage divider
     * @param divider_ratio Voltage divider ratio (Vbat / Vadc)
     * @param v_min Minimum voltage (empty battery, default 3.0V for LiPo)
     * @param v_max Maximum voltage (full battery, default 4.2V for LiPo)
     */
    BatteryMonitor(AdcChannel channel, float divider_ratio,
                   float v_min = 3.0f, float v_max = 4.2f);

    /**
     * @brief Read battery voltage
     * @return Battery voltage in volts (averaged reading)
     */
    float readVoltage() const;

    /**
     * @brief Read battery percentage
     * @return Estimated state of charge (0-100%)
     * @note Linear approximation - not accurate for all battery chemistries
     */
    float readPercent() const;

    /**
     * @brief Check if battery voltage is critical
     * @param threshold Voltage threshold (default 3.3V)
     * @return true if battery below threshold
     */
    bool isCritical(float threshold = 3.3f) const;

private:
    AdcChannel m_channel;
    float m_divider_ratio;
    float m_v_min;
    float m_v_max;
};


/**
 * @brief Pyro continuity checker
 * 
 * Checks pyro channel continuity by measuring voltage through ematch.
 * A connected ematch creates a voltage divider, resulting in a
 * measurable voltage. Open circuit reads near zero or full rail
 * depending on circuit topology.
 */
class PyroContinuity {
public:
    /**
     * @brief Continuity status
     */
    enum class Status : uint8_t {
        OPEN,       // No ematch connected
        GOOD,       // Ematch connected, good continuity
        SHORT,      // Short circuit detected
        UNKNOWN     // Could not determine
    };

    /**
     * @brief Construct continuity checker
     * @param channel ADC channel for continuity sense
     * @param v_open Expected voltage when open (depends on circuit)
     * @param v_good_min Minimum voltage for good continuity
     * @param v_good_max Maximum voltage for good continuity
     */
    PyroContinuity(AdcChannel channel, float v_open, 
                   float v_good_min, float v_good_max);

    /**
     * @brief Check continuity status
     * @return Continuity status
     */
    Status check() const;

    /**
     * @brief Read raw sense voltage
     * @return Voltage at sense point
     */
    float readVoltage() const;

private:
    AdcChannel m_channel;
    float m_v_open;
    float m_v_good_min;
    float m_v_good_max;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_ADC_H
