/**
 * @file GPIO.h
 * @brief AP_HAL GPIO implementation for RP2350
 *
 * Wraps RocketChip hal::GPIO to provide AP_HAL::GPIO interface.
 *
 * @note RP2350-E9 Erratum: Floating GPIO pins may latch at ~2V due to
 *       internal leakage. Use external pull-downs on critical inputs.
 *       Do not rely on internal pull-downs or floating pins alone.
 *       See docs/RP2350_FULL_AP_PORT.md PD9 for details.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

#include <cstdint>

// AP_HAL base classes for inheritance
#include <AP_HAL/GPIO.h>

// Forward declare to avoid circular includes
namespace rocketchip {
namespace hal {
enum class PinMode : uint8_t;
enum class PinState : uint8_t;
enum class PinEdge : uint8_t;
}
}

namespace RP2350 {

/**
 * @brief DigitalSource implementation for single-pin access
 *
 * Provides object-oriented access to a single GPIO pin,
 * as required by AP_HAL::GPIO::channel().
 */
class DigitalSource_RP2350 : public AP_HAL::DigitalSource {
public:
    DigitalSource_RP2350() : m_pin(0xFF) {}  // Default constructor for array init
    explicit DigitalSource_RP2350(uint8_t pin);

    /**
     * @brief Set pin mode
     * @param output HAL_GPIO_INPUT (0) or HAL_GPIO_OUTPUT (1)
     */
    void mode(uint8_t output) override;

    /**
     * @brief Read pin state
     * @return 0 for LOW, 1 for HIGH
     */
    uint8_t read() override;

    /**
     * @brief Write pin state
     * @param value 0 for LOW, non-zero for HIGH
     */
    void write(uint8_t value) override;

    /**
     * @brief Toggle pin state
     */
    void toggle() override;

private:
    uint8_t m_pin;
};


/**
 * @brief GPIO implementation for RP2350
 *
 * Wraps RocketChip's static GPIO class to provide the AP_HAL::GPIO interface.
 */
class GPIO_RP2350 : public AP_HAL::GPIO {
public:
    GPIO_RP2350();
    ~GPIO_RP2350() = default;

    // Prevent copying
    GPIO_RP2350(const GPIO_RP2350&) = delete;
    GPIO_RP2350& operator=(const GPIO_RP2350&) = delete;

    /**
     * @brief Initialize GPIO subsystem
     * @note No-op on RP2350 - SDK handles initialization
     */
    void init() override;

    /**
     * @brief Configure pin mode
     * @param pin GPIO pin number (0-29 valid on RP2350)
     * @param output HAL_GPIO_INPUT (0), HAL_GPIO_OUTPUT (1), or HAL_GPIO_ALT (2)
     */
    void pinMode(uint8_t pin, uint8_t output) override;

    /**
     * @brief Read pin state
     * @param pin GPIO pin number
     * @return 0 for LOW, 1 for HIGH
     */
    uint8_t read(uint8_t pin) override;

    /**
     * @brief Write pin state
     * @param pin GPIO pin number
     * @param value 0 for LOW, non-zero for HIGH
     */
    void write(uint8_t pin, uint8_t value) override;

    /**
     * @brief Toggle pin state
     * @param pin GPIO pin number
     */
    void toggle(uint8_t pin) override;

    /**
     * @brief Check if pin number is valid
     * @param pin GPIO pin number
     * @return true if pin is usable (0-29)
     */
    bool valid_pin(uint8_t pin) const override;

    /**
     * @brief Get DigitalSource for a pin
     * @param n GPIO pin number
     * @return Pointer to DigitalSource (caller does NOT own - static storage)
     */
    AP_HAL::DigitalSource* channel(uint16_t n) override;

    /**
     * @brief Check if USB cable is connected
     * @return true if USB CDC is connected
     */
    bool usb_connected(void) override;

    /**
     * @brief Attach interrupt handler to pin
     * @param pin GPIO pin number
     * @param fn Callback functor (pin, state, timestamp)
     * @param mode Trigger mode (RISING, FALLING, BOTH)
     * @return true on success
     */
    bool attach_interrupt(uint8_t pin,
                          irq_handler_fn_t fn,
                          INTERRUPT_TRIGGER_TYPE mode) override;

    /**
     * @brief Optional timer tick (no-op)
     */
    void timer_tick(void) override {}

private:
    static constexpr uint8_t kMaxPins = 30;  // GPIO0-29 on RP2350
    static constexpr uint8_t kMaxChannels = 8;  // Max simultaneous DigitalSource

    // Static storage for DigitalSource objects
    DigitalSource_RP2350 m_channels[kMaxChannels];
    uint8_t m_channel_pins[kMaxChannels];
    uint8_t m_num_channels;
};

}  // namespace RP2350
