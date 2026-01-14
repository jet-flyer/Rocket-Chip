/**
 * @file PWM.h
 * @brief PIO-based PWM output abstraction
 * 
 * Provides PWM output for servos, ESCs, and other actuators using the RP2350's
 * Programmable I/O (PIO) state machines. PIO offloads timing-critical work
 * from the CPU cores, ensuring consistent pulse timing even under heavy load.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 * @note Preferred over hardware PWM for servo control due to flexibility
 */

#ifndef ROCKETCHIP_HAL_PWM_H
#define ROCKETCHIP_HAL_PWM_H

#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief PWM channel configuration
 */
struct PwmConfig {
    uint32_t frequency_hz;   ///< PWM frequency (e.g., 50Hz for servos, 400Hz for ESCs)
    uint16_t min_pulse_us;   ///< Minimum pulse width in microseconds
    uint16_t max_pulse_us;   ///< Maximum pulse width in microseconds
    uint16_t center_pulse_us;///< Center/neutral pulse width in microseconds
};

/**
 * @brief Standard PWM configurations
 */
namespace PwmPresets {
    /// Standard RC servo (1000-2000µs @ 50Hz)
    constexpr PwmConfig SERVO_STANDARD = {
        .frequency_hz = 50,
        .min_pulse_us = 1000,
        .max_pulse_us = 2000,
        .center_pulse_us = 1500
    };

    /// Digital servo (500-2500µs @ 50Hz, extended range)
    constexpr PwmConfig SERVO_DIGITAL = {
        .frequency_hz = 50,
        .min_pulse_us = 500,
        .max_pulse_us = 2500,
        .center_pulse_us = 1500
    };

    /// Standard ESC (1000-2000µs @ 50Hz)
    constexpr PwmConfig ESC_STANDARD = {
        .frequency_hz = 50,
        .min_pulse_us = 1000,
        .max_pulse_us = 2000,
        .center_pulse_us = 1000  // Typically starts at min for throttle
    };

    /// Fast ESC (1000-2000µs @ 400Hz)
    constexpr PwmConfig ESC_FAST = {
        .frequency_hz = 400,
        .min_pulse_us = 1000,
        .max_pulse_us = 2000,
        .center_pulse_us = 1000
    };

    /// OneShot125 ESC (125-250µs @ 4000Hz)
    constexpr PwmConfig ESC_ONESHOT125 = {
        .frequency_hz = 4000,
        .min_pulse_us = 125,
        .max_pulse_us = 250,
        .center_pulse_us = 125
    };
}


/**
 * @brief PIO-based PWM output manager
 * 
 * Uses RP2350 PIO state machines for precise PWM timing without CPU load.
 * Supports multiple channels with independent configurations.
 * 
 * @note Maximum 8 PWM channels (limited by PIO state machines)
 */
class PwmManager {
public:
    /**
     * @brief Maximum number of PWM channels
     */
    static constexpr uint8_t MAX_CHANNELS = 8;

    /**
     * @brief Initialize PWM manager
     * @return true if PIO initialization successful
     */
    static bool init();

    /**
     * @brief Configure a PWM channel
     * @param channel Channel number (0 to MAX_CHANNELS-1)
     * @param pin GPIO pin for output
     * @param config PWM configuration
     * @return true if configuration successful
     */
    static bool configureChannel(uint8_t channel, uint8_t pin, const PwmConfig& config);

    /**
     * @brief Set PWM pulse width directly
     * @param channel Channel number
     * @param pulse_us Pulse width in microseconds
     */
    static void setPulseWidth(uint8_t channel, uint16_t pulse_us);

    /**
     * @brief Set PWM output as normalized value
     * @param channel Channel number
     * @param value Normalized value (-1.0 to +1.0 for servos, 0.0 to 1.0 for ESCs)
     * @note Maps to configured min/max pulse widths
     */
    static void setNormalized(uint8_t channel, float value);

    /**
     * @brief Set PWM output as percentage
     * @param channel Channel number
     * @param percent Percentage (0-100)
     */
    static void setPercent(uint8_t channel, uint8_t percent);

    /**
     * @brief Enable PWM output on channel
     * @param channel Channel number
     */
    static void enable(uint8_t channel);

    /**
     * @brief Disable PWM output on channel (holds low)
     * @param channel Channel number
     */
    static void disable(uint8_t channel);

    /**
     * @brief Disable all PWM outputs
     * @note Safety function - use in failsafe conditions
     */
    static void disableAll();

    /**
     * @brief Set all servo channels to center
     * @note Convenience for TVC initialization
     */
    static void centerAll();

private:
    PwmManager() = delete;  // Static-only class
    
    static bool s_initialized;
    static PwmConfig s_configs[MAX_CHANNELS];
    static uint8_t s_pins[MAX_CHANNELS];
    static bool s_enabled[MAX_CHANNELS];
};


/**
 * @brief Servo output channel
 * 
 * Convenience wrapper for a single servo output.
 * 
 * @code
 * Servo tvcPitch(0, GPIO_TVC_PITCH, PwmPresets::SERVO_DIGITAL);
 * tvcPitch.enable();
 * tvcPitch.setAngle(0);    // Center
 * tvcPitch.setAngle(45);   // Pitch up
 * tvcPitch.setAngle(-30);  // Pitch down
 * @endcode
 */
class Servo {
public:
    /**
     * @brief Construct servo output
     * @param channel PWM channel number
     * @param pin GPIO pin
     * @param config PWM configuration (default: standard servo)
     */
    Servo(uint8_t channel, uint8_t pin, 
          const PwmConfig& config = PwmPresets::SERVO_STANDARD);

    /**
     * @brief Set servo angle
     * @param degrees Angle in degrees (typically -90 to +90 or 0 to 180)
     * @param min_angle Minimum angle corresponding to min_pulse_us
     * @param max_angle Maximum angle corresponding to max_pulse_us
     */
    void setAngle(float degrees, float min_angle = -90.0f, float max_angle = 90.0f);

    /**
     * @brief Set servo position normalized
     * @param position -1.0 to +1.0 (maps to min to max pulse)
     */
    void setPosition(float position);

    /**
     * @brief Set servo pulse width directly
     * @param pulse_us Pulse width in microseconds
     */
    void setPulseWidth(uint16_t pulse_us);

    /**
     * @brief Move servo to center position
     */
    void center();

    /**
     * @brief Enable servo output
     */
    void enable();

    /**
     * @brief Disable servo output
     */
    void disable();

    /**
     * @brief Check if servo is enabled
     */
    bool isEnabled() const { return m_enabled; }

    /**
     * @brief Get current pulse width
     */
    uint16_t getPulseWidth() const { return m_current_pulse_us; }

private:
    uint8_t m_channel;
    uint8_t m_pin;
    PwmConfig m_config;
    uint16_t m_current_pulse_us;
    bool m_enabled;
};


/**
 * @brief ESC (Electronic Speed Controller) output
 * 
 * Convenience wrapper for ESC throttle control.
 * 
 * @code
 * ESC motor(2, GPIO_MOTOR, PwmPresets::ESC_FAST);
 * motor.arm();      // Required before throttle works
 * motor.setThrottle(0.5f);  // 50% throttle
 * motor.disarm();   // Safety - stops motor
 * @endcode
 */
class ESC {
public:
    /**
     * @brief Construct ESC output
     * @param channel PWM channel number
     * @param pin GPIO pin
     * @param config PWM configuration (default: standard ESC)
     */
    ESC(uint8_t channel, uint8_t pin, 
        const PwmConfig& config = PwmPresets::ESC_STANDARD);

    /**
     * @brief Arm the ESC
     * 
     * Sends minimum throttle signal to arm the ESC.
     * Most ESCs require this before they respond to throttle commands.
     */
    void arm();

    /**
     * @brief Disarm the ESC
     * 
     * Sends minimum throttle and disables output.
     * Safety function - use in failsafe conditions.
     */
    void disarm();

    /**
     * @brief Set throttle
     * @param throttle 0.0 to 1.0 (0% to 100%)
     * @note ESC must be armed first
     */
    void setThrottle(float throttle);

    /**
     * @brief Check if ESC is armed
     */
    bool isArmed() const { return m_armed; }

private:
    uint8_t m_channel;
    uint8_t m_pin;
    PwmConfig m_config;
    bool m_armed;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_PWM_H
