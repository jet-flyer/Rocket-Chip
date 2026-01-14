/**
 * @file PIO.h
 * @brief PIO (Programmable I/O) utilities
 * 
 * Provides interfaces for RP2350 PIO-specific functionality beyond
 * PWM output, including WS2812 (NeoPixel) LED control.
 * 
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 * @note PIO state machine allocation is shared with PWM module
 */

#ifndef ROCKETCHIP_HAL_PIO_H
#define ROCKETCHIP_HAL_PIO_H

#include <cstdint>

namespace rocketchip {
namespace hal {

/**
 * @brief RGB color structure
 */
struct RGB {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    constexpr RGB() : r(0), g(0), b(0) {}
    constexpr RGB(uint8_t red, uint8_t green, uint8_t blue) : r(red), g(green), b(blue) {}

    /**
     * @brief Create from 24-bit packed value (0xRRGGBB)
     */
    static constexpr RGB fromPacked(uint32_t packed) {
        return RGB(
            static_cast<uint8_t>((packed >> 16) & 0xFF),
            static_cast<uint8_t>((packed >> 8) & 0xFF),
            static_cast<uint8_t>(packed & 0xFF)
        );
    }

    /**
     * @brief Convert to packed 24-bit value
     */
    constexpr uint32_t toPacked() const {
        return (static_cast<uint32_t>(r) << 16) |
               (static_cast<uint32_t>(g) << 8) |
               static_cast<uint32_t>(b);
    }

    /**
     * @brief Scale brightness (0-255)
     */
    constexpr RGB scaled(uint8_t brightness) const {
        return RGB(
            static_cast<uint8_t>((r * brightness) >> 8),
            static_cast<uint8_t>((g * brightness) >> 8),
            static_cast<uint8_t>((b * brightness) >> 8)
        );
    }
};

/**
 * @brief Standard colors
 */
namespace Colors {
    constexpr RGB OFF     = {0, 0, 0};
    constexpr RGB RED     = {255, 0, 0};
    constexpr RGB GREEN   = {0, 255, 0};
    constexpr RGB BLUE    = {0, 0, 255};
    constexpr RGB WHITE   = {255, 255, 255};
    constexpr RGB YELLOW  = {255, 255, 0};
    constexpr RGB CYAN    = {0, 255, 255};
    constexpr RGB MAGENTA = {255, 0, 255};
    constexpr RGB ORANGE  = {255, 128, 0};
    constexpr RGB PURPLE  = {128, 0, 255};
}


/**
 * @brief WS2812 (NeoPixel) LED driver using PIO
 * 
 * Drives WS2812/WS2812B/SK6812 addressable LEDs using a PIO state machine.
 * This offloads the precise timing requirements from the CPU.
 * 
 * @code
 * // Single status LED on Feather
 * WS2812 statusLed(GPIO_NEOPIXEL, 1);
 * statusLed.begin();
 * statusLed.setPixel(0, Colors::GREEN);
 * statusLed.show();
 * 
 * // LED strip
 * WS2812 strip(GPIO_STRIP, 8);
 * strip.begin();
 * for (int i = 0; i < 8; i++) {
 *     strip.setPixel(i, RGB(i * 32, 0, 255 - i * 32));
 * }
 * strip.show();
 * @endcode
 */
class WS2812 {
public:
    /**
     * @brief Maximum supported LEDs per strip
     */
    static constexpr uint16_t MAX_LEDS = 256;

    /**
     * @brief LED chip type (affects color order)
     */
    enum class Type : uint8_t {
        WS2812,   // GRB order
        WS2812B,  // GRB order
        SK6812,   // GRB order
        SK6812_RGBW  // RGBW order (4 bytes per pixel)
    };

    /**
     * @brief Construct WS2812 driver
     * @param pin GPIO pin connected to LED data input
     * @param num_leds Number of LEDs in the chain
     * @param type LED chip type (affects color order)
     */
    WS2812(uint8_t pin, uint16_t num_leds, Type type = Type::WS2812);

    /**
     * @brief Destructor - releases PIO resources
     */
    ~WS2812();

    /**
     * @brief Initialize PIO and allocate buffer
     * @return true if successful
     */
    bool begin();

    /**
     * @brief Set a single pixel color
     * @param index Pixel index (0 to num_leds-1)
     * @param color RGB color
     */
    void setPixel(uint16_t index, const RGB& color);

    /**
     * @brief Set a single pixel from packed color
     * @param index Pixel index
     * @param packed 24-bit color (0xRRGGBB)
     */
    void setPixel(uint16_t index, uint32_t packed);

    /**
     * @brief Set all pixels to same color
     * @param color RGB color
     */
    void fill(const RGB& color);

    /**
     * @brief Turn off all pixels
     */
    void clear();

    /**
     * @brief Get current color of a pixel
     * @param index Pixel index
     * @return RGB color
     */
    RGB getPixel(uint16_t index) const;

    /**
     * @brief Update LEDs with current buffer contents
     * 
     * Must be called after setPixel() to actually update the LEDs.
     * Takes approximately 30µs per LED (blocking during data transmission).
     */
    void show();

    /**
     * @brief Set global brightness
     * @param brightness 0-255 (applied to all pixels on show())
     */
    void setBrightness(uint8_t brightness);

    /**
     * @brief Get number of LEDs
     */
    uint16_t getNumLeds() const { return m_num_leds; }

private:
    uint8_t m_pin;
    uint16_t m_num_leds;
    Type m_type;
    uint8_t m_brightness;
    uint8_t* m_buffer;
    bool m_initialized;

    // PIO resources
    int m_pio_sm;  // State machine index (-1 if not allocated)

    // Non-copyable
    WS2812(const WS2812&) = delete;
    WS2812& operator=(const WS2812&) = delete;
};


/**
 * @brief Status LED patterns
 * 
 * Pre-defined blink patterns for system status indication.
 * Used with a single NeoPixel for flight status.
 */
class StatusLED {
public:
    /**
     * @brief Status pattern types
     */
    enum class Pattern : uint8_t {
        OFF,
        SOLID,
        BLINK_SLOW,    // 1 Hz
        BLINK_FAST,    // 4 Hz
        PULSE,         // Breathing effect
        DOUBLE_BLINK,  // Two quick blinks
        SOS            // ... --- ...
    };

    /**
     * @brief Construct status LED controller
     * @param pin GPIO pin for NeoPixel
     */
    explicit StatusLED(uint8_t pin);

    /**
     * @brief Initialize LED
     */
    bool begin();

    /**
     * @brief Set status pattern
     * @param pattern Blink pattern
     * @param color Pattern color
     */
    void setPattern(Pattern pattern, const RGB& color);

    /**
     * @brief Update pattern animation
     * 
     * Call this regularly (at least 50Hz) to update animations.
     */
    void update();

    // Convenience methods for common states
    void setIdle()     { setPattern(Pattern::SOLID, Colors::BLUE); }
    void setArmed()    { setPattern(Pattern::BLINK_SLOW, Colors::GREEN); }
    void setFlight()   { setPattern(Pattern::SOLID, Colors::GREEN); }
    void setRecovery() { setPattern(Pattern::BLINK_FAST, Colors::YELLOW); }
    void setError()    { setPattern(Pattern::BLINK_FAST, Colors::RED); }
    void setLowBatt()  { setPattern(Pattern::DOUBLE_BLINK, Colors::ORANGE); }

private:
    WS2812 m_led;
    Pattern m_pattern;
    RGB m_color;
    uint32_t m_last_update_ms;
    uint8_t m_anim_step;
};


/**
 * @brief PIO state machine manager
 * 
 * Coordinates PIO resource allocation across HAL modules.
 * RP2350 has 2 PIO blocks with 4 state machines each (8 total).
 */
class PIOManager {
public:
    /**
     * @brief Initialize PIO manager
     */
    static void init();

    /**
     * @brief Allocate a state machine
     * @param pio_index Preferred PIO block (0 or 1), or -1 for any
     * @return State machine index (0-7), or -1 if none available
     */
    static int allocateStateMachine(int pio_index = -1);

    /**
     * @brief Release a state machine
     * @param sm State machine index to release
     */
    static void releaseStateMachine(int sm);

    /**
     * @brief Get number of available state machines
     */
    static uint8_t availableStateMachines();

private:
    PIOManager() = delete;
    static uint8_t s_allocated;  // Bitmask of allocated SMs
};

} // namespace hal
} // namespace rocketchip

#endif // ROCKETCHIP_HAL_PIO_H
