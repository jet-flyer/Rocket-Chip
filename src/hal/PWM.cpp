/**
 * @file PWM.cpp
 * @brief PIO-based PWM output abstraction implementation
 *
 * Implements PWM output for servos and ESCs using RP2350 PIO state machines.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "PWM.h"
#include "PIO.h"

#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include <cstring>

namespace rocketchip {
namespace hal {

// ============================================================================
// PWM PIO program
// ============================================================================

namespace {

// PIO program for PWM generation
// Sets pin high, waits for pulse duration, sets pin low, waits for remainder
//
// .program pwm
// .side_set 1 opt
// pull noblock    side 0     ; Pull pulse width from FIFO (or use X)
// mov x, osr                 ; Copy to X
// mov y, isr                 ; Y = period counter (preset)
// wait_high:
//     jmp x-- wait_high side 1  ; High pulse
// wait_low:
//     jmp y-- wait_low  side 0  ; Low remainder
//     jmp pull          side 0  ; Restart

const uint16_t pwm_program_instructions[] = {
    0x80a0, //  0: pull   noblock         side 0
    0xa027, //  1: mov    x, osr
    0xa046, //  2: mov    y, isr
    0x1843, //  3: jmp    x--, 3          side 1
    0x0084, //  4: jmp    y--, 4          side 0
    0x0000, //  5: jmp    0               side 0
};

const struct pio_program pwm_program = {
    .instructions = pwm_program_instructions,
    .length = 6,
    .origin = -1,
};

pio_sm_config pwm_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 5);
    sm_config_set_sideset(&c, 2, true, false);  // Optional sideset, 1 bit
    return c;
}

// Simpler approach: use hardware PWM slice with PIO for extended precision
// For now, use a straightforward timing approach with periodic updates

// State for each channel
struct PwmChannelState {
    bool configured;
    bool enabled;
    uint8_t pin;
    uint32_t period_us;
    uint16_t pulse_us;
    PwmConfig config;
};

PwmChannelState s_channels[PwmManager::MAX_CHANNELS];
bool s_pwm_initialized = false;

// We'll use a simpler approach: hardware PWM slice
// RP2350 has 8 PWM slices with 2 channels each

} // anonymous namespace

// ============================================================================
// PwmManager class implementation
// ============================================================================

bool PwmManager::s_initialized = false;
PwmConfig PwmManager::s_configs[MAX_CHANNELS];
uint8_t PwmManager::s_pins[MAX_CHANNELS];
bool PwmManager::s_enabled[MAX_CHANNELS];

bool PwmManager::init() {
    if (s_initialized) {
        return true;
    }

    // Initialize all channels to unconfigured
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        s_channels[i].configured = false;
        s_channels[i].enabled = false;
        s_enabled[i] = false;
    }

    s_initialized = true;
    return true;
}

bool PwmManager::configureChannel(uint8_t channel, uint8_t pin, const PwmConfig& config) {
    if (channel >= MAX_CHANNELS) {
        return false;
    }

    if (!s_initialized) {
        init();
    }

    // Store configuration
    s_configs[channel] = config;
    s_pins[channel] = pin;
    s_channels[channel].config = config;
    s_channels[channel].pin = pin;
    s_channels[channel].period_us = 1000000 / config.frequency_hz;
    s_channels[channel].pulse_us = config.center_pulse_us;
    s_channels[channel].configured = true;
    s_channels[channel].enabled = false;

    // Initialize GPIO for PWM
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_OUT);
    gpio_put(pin, 0);

    // Get PWM slice for this pin
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel_num = pwm_gpio_to_channel(pin);

    // Configure PWM
    // System clock is typically 125MHz
    // For 50Hz: period = 20ms = 20000us
    // We need ~1us resolution for servo control

    uint32_t sys_clk = clock_get_hz(clk_sys);
    uint32_t period_us = s_channels[channel].period_us;

    // Calculate divider and wrap value for 1us resolution
    // With 125MHz clock, div=125 gives 1us per tick
    float divider = static_cast<float>(sys_clk) / 1000000.0f;  // 1MHz = 1us ticks
    uint16_t wrap = static_cast<uint16_t>(period_us - 1);

    // Clamp wrap to 16-bit max
    if (period_us > 65535) {
        // Need larger divider for lower frequencies
        divider = static_cast<float>(sys_clk) / (1000000.0f / 16.0f);  // 62.5kHz = 16us ticks
        wrap = static_cast<uint16_t>((period_us / 16) - 1);
    }

    pwm_config pwm_cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&pwm_cfg, divider);
    pwm_config_set_wrap(&pwm_cfg, wrap);

    // Set GPIO function to PWM
    gpio_set_function(pin, GPIO_FUNC_PWM);

    // Initialize PWM with config
    pwm_init(slice_num, &pwm_cfg, false);  // Don't start yet

    return true;
}

void PwmManager::setPulseWidth(uint8_t channel, uint16_t pulse_us) {
    if (channel >= MAX_CHANNELS || !s_channels[channel].configured) {
        return;
    }

    s_channels[channel].pulse_us = pulse_us;

    uint8_t pin = s_pins[channel];
    uint slice_num = pwm_gpio_to_slice_num(pin);
    uint channel_num = pwm_gpio_to_channel(pin);

    // Calculate level based on pulse width
    uint32_t period_us = s_channels[channel].period_us;
    uint16_t level;

    if (period_us <= 65535) {
        // 1us resolution
        level = pulse_us;
    } else {
        // 16us resolution
        level = pulse_us / 16;
    }

    pwm_set_chan_level(slice_num, channel_num, level);
}

void PwmManager::setNormalized(uint8_t channel, float value) {
    if (channel >= MAX_CHANNELS || !s_channels[channel].configured) {
        return;
    }

    const PwmConfig& config = s_configs[channel];

    // Clamp value to -1.0 to 1.0
    if (value < -1.0f) value = -1.0f;
    if (value > 1.0f) value = 1.0f;

    // Map from [-1, 1] or [0, 1] to [min_pulse, max_pulse]
    uint16_t pulse_us;
    if (value < 0.0f) {
        // Map [-1, 0] to [min_pulse, center_pulse]
        pulse_us = static_cast<uint16_t>(
            config.center_pulse_us + value * (config.center_pulse_us - config.min_pulse_us));
    } else {
        // Map [0, 1] to [center_pulse, max_pulse]
        pulse_us = static_cast<uint16_t>(
            config.center_pulse_us + value * (config.max_pulse_us - config.center_pulse_us));
    }

    setPulseWidth(channel, pulse_us);
}

void PwmManager::setPercent(uint8_t channel, uint8_t percent) {
    if (channel >= MAX_CHANNELS || !s_channels[channel].configured) {
        return;
    }

    // Clamp to 0-100
    if (percent > 100) percent = 100;

    const PwmConfig& config = s_configs[channel];

    // Map 0-100% to min-max pulse
    uint16_t range = config.max_pulse_us - config.min_pulse_us;
    uint16_t pulse_us = config.min_pulse_us +
                        static_cast<uint16_t>((range * percent) / 100);

    setPulseWidth(channel, pulse_us);
}

void PwmManager::enable(uint8_t channel) {
    if (channel >= MAX_CHANNELS || !s_channels[channel].configured) {
        return;
    }

    uint8_t pin = s_pins[channel];
    uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_set_enabled(slice_num, true);
    s_enabled[channel] = true;
    s_channels[channel].enabled = true;
}

void PwmManager::disable(uint8_t channel) {
    if (channel >= MAX_CHANNELS) {
        return;
    }

    if (s_channels[channel].configured) {
        uint8_t pin = s_pins[channel];
        uint slice_num = pwm_gpio_to_slice_num(pin);

        pwm_set_enabled(slice_num, false);
        gpio_put(pin, 0);  // Ensure output is low
    }

    s_enabled[channel] = false;
    s_channels[channel].enabled = false;
}

void PwmManager::disableAll() {
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        disable(i);
    }
}

void PwmManager::centerAll() {
    for (uint8_t i = 0; i < MAX_CHANNELS; ++i) {
        if (s_channels[i].configured) {
            setPulseWidth(i, s_configs[i].center_pulse_us);
        }
    }
}

// ============================================================================
// Servo class implementation
// ============================================================================

Servo::Servo(uint8_t channel, uint8_t pin, const PwmConfig& config)
    : m_channel(channel)
    , m_pin(pin)
    , m_config(config)
    , m_current_pulse_us(config.center_pulse_us)
    , m_enabled(false)
{
    PwmManager::configureChannel(channel, pin, config);
}

void Servo::setAngle(float degrees, float min_angle, float max_angle) {
    // Clamp to angle range
    if (degrees < min_angle) degrees = min_angle;
    if (degrees > max_angle) degrees = max_angle;

    // Map angle to pulse width
    float range_angle = max_angle - min_angle;
    float range_pulse = static_cast<float>(m_config.max_pulse_us - m_config.min_pulse_us);

    float normalized = (degrees - min_angle) / range_angle;
    m_current_pulse_us = static_cast<uint16_t>(
        m_config.min_pulse_us + normalized * range_pulse);

    PwmManager::setPulseWidth(m_channel, m_current_pulse_us);
}

void Servo::setPosition(float position) {
    // position: -1.0 to 1.0
    PwmManager::setNormalized(m_channel, position);

    // Update cached pulse width
    if (position < -1.0f) position = -1.0f;
    if (position > 1.0f) position = 1.0f;

    if (position < 0.0f) {
        m_current_pulse_us = static_cast<uint16_t>(
            m_config.center_pulse_us +
            position * (m_config.center_pulse_us - m_config.min_pulse_us));
    } else {
        m_current_pulse_us = static_cast<uint16_t>(
            m_config.center_pulse_us +
            position * (m_config.max_pulse_us - m_config.center_pulse_us));
    }
}

void Servo::setPulseWidth(uint16_t pulse_us) {
    m_current_pulse_us = pulse_us;
    PwmManager::setPulseWidth(m_channel, pulse_us);
}

void Servo::center() {
    m_current_pulse_us = m_config.center_pulse_us;
    PwmManager::setPulseWidth(m_channel, m_current_pulse_us);
}

void Servo::enable() {
    PwmManager::enable(m_channel);
    m_enabled = true;
}

void Servo::disable() {
    PwmManager::disable(m_channel);
    m_enabled = false;
}

// ============================================================================
// ESC class implementation
// ============================================================================

ESC::ESC(uint8_t channel, uint8_t pin, const PwmConfig& config)
    : m_channel(channel)
    , m_pin(pin)
    , m_config(config)
    , m_armed(false)
{
    PwmManager::configureChannel(channel, pin, config);
}

void ESC::arm() {
    // Send minimum throttle signal to arm
    PwmManager::setPulseWidth(m_channel, m_config.min_pulse_us);
    PwmManager::enable(m_channel);
    m_armed = true;
}

void ESC::disarm() {
    // Send minimum throttle and disable
    PwmManager::setPulseWidth(m_channel, m_config.min_pulse_us);
    PwmManager::disable(m_channel);
    m_armed = false;
}

void ESC::setThrottle(float throttle) {
    if (!m_armed) {
        return;  // ESC must be armed first
    }

    // Clamp throttle to 0.0 - 1.0
    if (throttle < 0.0f) throttle = 0.0f;
    if (throttle > 1.0f) throttle = 1.0f;

    // Map to pulse width
    uint16_t range = m_config.max_pulse_us - m_config.min_pulse_us;
    uint16_t pulse_us = m_config.min_pulse_us +
                        static_cast<uint16_t>(throttle * range);

    PwmManager::setPulseWidth(m_channel, pulse_us);
}

} // namespace hal
} // namespace rocketchip
