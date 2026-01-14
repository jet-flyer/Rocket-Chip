/**
 * @file GPIO.h
 * @brief Digital I/O abstraction
 * 
 * Provides unified interface for GPIO operations including input/output,
 * pull-up/down configuration, and interrupt handling.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#ifndef ROCKETCHIP_HAL_GPIO_H
#define ROCKETCHIP_HAL_GPIO_H

#include <cstdint>
#include <functional>

namespace rocketchip {
namespace hal {

/**
 * @brief GPIO pin direction
 */
enum class PinMode : uint8_t {
    INPUT,
    OUTPUT,
    INPUT_PULLUP,
    INPUT_PULLDOWN
};

/**
 * @brief GPIO interrupt edge trigger
 */
enum class PinEdge : uint8_t {
    NONE,
    RISING,
    FALLING,
    BOTH
};

/**
 * @brief GPIO pin state
 */
enum class PinState : uint8_t {
    LOW = 0,
    HIGH = 1
};

/**
 * @brief Callback type for GPIO interrupts
 * @param pin The pin that triggered the interrupt
 * @param state Current state of the pin
 */
using GpioCallback = std::function<void(uint8_t pin, PinState state)>;


/**
 * @brief Digital GPIO interface
 * 
 * Wraps Pico SDK GPIO functions with additional features like debouncing
 * for mechanical inputs (arm switches, buttons).
 */
class GPIO {
public:
    /**
     * @brief Configure a pin's mode
     * @param pin GPIO pin number
     * @param mode Pin direction and pull configuration
     */
    static void pinMode(uint8_t pin, PinMode mode);

    /**
     * @brief Write to an output pin
     * @param pin GPIO pin number
     * @param state HIGH or LOW
     */
    static void write(uint8_t pin, PinState state);

    /**
     * @brief Write to an output pin (convenience overload)
     * @param pin GPIO pin number
     * @param high true for HIGH, false for LOW
     */
    static void write(uint8_t pin, bool high);

    /**
     * @brief Read from an input pin
     * @param pin GPIO pin number
     * @return Current pin state
     */
    static PinState read(uint8_t pin);

    /**
     * @brief Read from an input pin (convenience returning bool)
     * @param pin GPIO pin number
     * @return true if HIGH, false if LOW
     */
    static bool readBool(uint8_t pin);

    /**
     * @brief Toggle an output pin
     * @param pin GPIO pin number
     */
    static void toggle(uint8_t pin);

    /**
     * @brief Attach interrupt handler to a pin
     * @param pin GPIO pin number
     * @param edge Trigger edge
     * @param callback Function to call on interrupt
     * @note Callback runs in interrupt context - keep it fast!
     */
    static void attachInterrupt(uint8_t pin, PinEdge edge, GpioCallback callback);

    /**
     * @brief Remove interrupt handler from a pin
     * @param pin GPIO pin number
     */
    static void detachInterrupt(uint8_t pin);

    /**
     * @brief Read input with software debouncing
     * 
     * Useful for mechanical switches (arm switch, buttons).
     * Returns stable state only after pin has been stable for debounce period.
     * 
     * @param pin GPIO pin number
     * @param debounce_ms Debounce time in milliseconds (default 50ms)
     * @return Debounced pin state
     * @note This function is blocking for up to debounce_ms
     */
    static PinState readDebounced(uint8_t pin, uint32_t debounce_ms = 50);

private:
    GPIO() = delete;  // Static-only class
};


/**
 * @brief Convenience class for output pins
 * 
 * RAII wrapper that configures pin as output on construction.
 * 
 * @code
 * OutputPin statusLed(25);
 * statusLed.set();    // Turn on
 * statusLed.clear();  // Turn off
 * statusLed.toggle(); // Toggle state
 * @endcode
 */
class OutputPin {
public:
    /**
     * @brief Construct and configure pin as output
     * @param pin GPIO pin number
     * @param initial_state Initial output state (default LOW)
     */
    explicit OutputPin(uint8_t pin, PinState initial_state = PinState::LOW);

    void set()    { GPIO::write(m_pin, PinState::HIGH); }
    void clear()  { GPIO::write(m_pin, PinState::LOW); }
    void toggle() { GPIO::toggle(m_pin); }
    
    void write(PinState state) { GPIO::write(m_pin, state); }
    void write(bool high)      { GPIO::write(m_pin, high); }

    uint8_t pin() const { return m_pin; }

private:
    uint8_t m_pin;
};


/**
 * @brief Convenience class for input pins
 * 
 * RAII wrapper that configures pin as input on construction.
 * 
 * @code
 * InputPin armSwitch(10, PinMode::INPUT_PULLUP);
 * if (armSwitch.read() == PinState::LOW) {
 *     // Switch is active (pulled to ground)
 * }
 * @endcode
 */
class InputPin {
public:
    /**
     * @brief Construct and configure pin as input
     * @param pin GPIO pin number
     * @param mode Input mode (INPUT, INPUT_PULLUP, INPUT_PULLDOWN)
     */
    explicit InputPin(uint8_t pin, PinMode mode = PinMode::INPUT);

    PinState read() const { return GPIO::read(m_pin); }
    bool readBool() const { return GPIO::readBool(m_pin); }
    PinState readDebounced(uint32_t debounce_ms = 50) const {
        return GPIO::readDebounced(m_pin, debounce_ms);
    }

    uint8_t pin() const { return m_pin; }

private:
    uint8_t m_pin;
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_GPIO_H
