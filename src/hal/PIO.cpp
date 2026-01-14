/**
 * @file PIO.cpp
 * @brief PIO (Programmable I/O) utilities implementation
 *
 * Implements WS2812 (NeoPixel) LED control and PIO state machine management
 * using RP2350 PIO hardware.
 *
 * @note Part of RocketChip HAL - Hardware Abstraction Layer
 */

#include "PIO.h"
#include "Timing.h"

#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include <cstring>
#include <cstdlib>

namespace rocketchip {
namespace hal {

// ============================================================================
// WS2812 PIO program
// ============================================================================

// WS2812 timing (in cycles at 800kHz = 1.25us per bit)
// T0H = 0.4us  = 0.32 cycles
// T0L = 0.85us = 0.68 cycles
// T1H = 0.8us  = 0.64 cycles
// T1L = 0.45us = 0.36 cycles
//
// Simplified: bit period = 1.25us
// Using PIO program from pico-examples

namespace {

// Inline PIO program for WS2812
// This is a simplified version based on pico-examples ws2812.pio
//
// .program ws2812
// .side_set 1
// .wrap_target
// bitloop:
//     out x, 1       side 0 [T3 - 1]
//     jmp !x do_zero side 1 [T1 - 1]
// do_one:
//     jmp bitloop    side 1 [T2 - 1]
// do_zero:
//     nop            side 0 [T2 - 1]
// .wrap

// Pre-compiled PIO instructions for WS2812
// Cycles: T1=2, T2=5, T3=3 for 800kHz at 125MHz system clock
const uint16_t ws2812_program_instructions[] = {
    //     .wrap_target
    0x6221, //  0: out    x, 1            side 0 [2]
    0x1123, //  1: jmp    !x, 3           side 1 [1]
    0x1400, //  2: jmp    0               side 1 [4]
    0xa442, //  3: nop                    side 0 [4]
    //     .wrap
};

const struct pio_program ws2812_program = {
    .instructions = ws2812_program_instructions,
    .length = 4,
    .origin = -1,
};

// WS2812 program configuration
pio_sm_config ws2812_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + 0, offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
    pio_gpio_init(pio, pin);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);

    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);  // Shift left, autopull at 24 bits
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);

    // Clock divider for target frequency
    float div = clock_get_hz(clk_sys) / (freq * 10);  // 10 cycles per bit
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}

} // anonymous namespace

// ============================================================================
// PIOManager class implementation
// ============================================================================

uint8_t PIOManager::s_allocated = 0;

void PIOManager::init() {
    // Reset allocation state
    s_allocated = 0;
}

int PIOManager::allocateStateMachine(int pio_index) {
    // RP2350 has 2 PIO blocks with 4 state machines each (8 total)

    if (pio_index >= 0 && pio_index <= 1) {
        // Try specific PIO block
        uint8_t start = static_cast<uint8_t>(pio_index * 4);
        for (uint8_t i = start; i < start + 4; ++i) {
            if (!(s_allocated & (1 << i))) {
                s_allocated |= (1 << i);
                return i;
            }
        }
    } else {
        // Try any available
        for (uint8_t i = 0; i < 8; ++i) {
            if (!(s_allocated & (1 << i))) {
                s_allocated |= (1 << i);
                return i;
            }
        }
    }

    return -1;  // No available state machines
}

void PIOManager::releaseStateMachine(int sm) {
    if (sm >= 0 && sm < 8) {
        s_allocated &= ~(1 << sm);
    }
}

uint8_t PIOManager::availableStateMachines() {
    uint8_t count = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        if (!(s_allocated & (1 << i))) {
            ++count;
        }
    }
    return count;
}

// ============================================================================
// WS2812 class implementation
// ============================================================================

WS2812::WS2812(uint8_t pin, uint16_t num_leds, Type type)
    : m_pin(pin)
    , m_num_leds(num_leds > MAX_LEDS ? MAX_LEDS : num_leds)
    , m_type(type)
    , m_brightness(255)
    , m_buffer(nullptr)
    , m_initialized(false)
    , m_pio_sm(-1)
{
}

WS2812::~WS2812() {
    if (m_buffer != nullptr) {
        free(m_buffer);
        m_buffer = nullptr;
    }

    if (m_pio_sm >= 0) {
        PIOManager::releaseStateMachine(m_pio_sm);
    }
}

bool WS2812::begin() {
    if (m_initialized) {
        return true;
    }

    // Allocate buffer (3 bytes per LED for RGB)
    size_t buffer_size = m_num_leds * 3;
    m_buffer = static_cast<uint8_t*>(malloc(buffer_size));
    if (m_buffer == nullptr) {
        return false;
    }
    memset(m_buffer, 0, buffer_size);

    // Allocate PIO state machine
    m_pio_sm = PIOManager::allocateStateMachine(0);  // Prefer PIO0
    if (m_pio_sm < 0) {
        free(m_buffer);
        m_buffer = nullptr;
        return false;
    }

    // Determine which PIO block and SM
    PIO pio = (m_pio_sm < 4) ? pio0 : pio1;
    uint sm = m_pio_sm % 4;

    // Add program to PIO
    uint offset = pio_add_program(pio, &ws2812_program);

    // Initialize the PIO program
    ws2812_program_init(pio, sm, offset, m_pin, 800000);  // 800kHz

    m_initialized = true;
    return true;
}

void WS2812::setPixel(uint16_t index, const RGB& color) {
    if (!m_initialized || index >= m_num_leds) {
        return;
    }

    // WS2812 uses GRB byte order
    size_t offset = index * 3;
    m_buffer[offset + 0] = color.g;
    m_buffer[offset + 1] = color.r;
    m_buffer[offset + 2] = color.b;
}

void WS2812::setPixel(uint16_t index, uint32_t packed) {
    setPixel(index, RGB::fromPacked(packed));
}

void WS2812::fill(const RGB& color) {
    for (uint16_t i = 0; i < m_num_leds; ++i) {
        setPixel(i, color);
    }
}

void WS2812::clear() {
    if (m_buffer != nullptr) {
        memset(m_buffer, 0, m_num_leds * 3);
    }
}

RGB WS2812::getPixel(uint16_t index) const {
    if (!m_initialized || index >= m_num_leds) {
        return RGB();
    }

    // WS2812 uses GRB byte order
    size_t offset = index * 3;
    return RGB(m_buffer[offset + 1],   // R
               m_buffer[offset + 0],   // G
               m_buffer[offset + 2]);  // B
}

void WS2812::show() {
    if (!m_initialized || m_buffer == nullptr) {
        return;
    }

    PIO pio = (m_pio_sm < 4) ? pio0 : pio1;
    uint sm = m_pio_sm % 4;

    // Send all pixels
    for (uint16_t i = 0; i < m_num_leds; ++i) {
        size_t offset = i * 3;

        // Apply brightness scaling and pack into 24-bit value
        uint8_t g = (m_buffer[offset + 0] * m_brightness) >> 8;
        uint8_t r = (m_buffer[offset + 1] * m_brightness) >> 8;
        uint8_t b = (m_buffer[offset + 2] * m_brightness) >> 8;

        // Pack as GRB with MSB first (shifted left by 8 for autopull alignment)
        uint32_t pixel = (static_cast<uint32_t>(g) << 24) |
                         (static_cast<uint32_t>(r) << 16) |
                         (static_cast<uint32_t>(b) << 8);

        pio_sm_put_blocking(pio, sm, pixel);
    }

    // Wait for reset time (>50us)
    Timing::delayMicros(60);
}

void WS2812::setBrightness(uint8_t brightness) {
    m_brightness = brightness;
}

// ============================================================================
// StatusLED class implementation
// ============================================================================

StatusLED::StatusLED(uint8_t pin)
    : m_led(pin, 1)
    , m_pattern(Pattern::OFF)
    , m_color(Colors::OFF)
    , m_last_update_ms(0)
    , m_anim_step(0)
{
}

bool StatusLED::begin() {
    return m_led.begin();
}

void StatusLED::setPattern(Pattern pattern, const RGB& color) {
    m_pattern = pattern;
    m_color = color;
    m_anim_step = 0;
    m_last_update_ms = Timing::millis32();
}

void StatusLED::update() {
    uint32_t now = Timing::millis32();
    uint32_t elapsed = now - m_last_update_ms;

    switch (m_pattern) {
        case Pattern::OFF:
            m_led.setPixel(0, Colors::OFF);
            m_led.show();
            break;

        case Pattern::SOLID:
            m_led.setPixel(0, m_color);
            m_led.show();
            break;

        case Pattern::BLINK_SLOW:
            // 1 Hz blink (500ms on, 500ms off)
            if (elapsed >= 500) {
                m_anim_step = !m_anim_step;
                m_led.setPixel(0, m_anim_step ? m_color : Colors::OFF);
                m_led.show();
                m_last_update_ms = now;
            }
            break;

        case Pattern::BLINK_FAST:
            // 4 Hz blink (125ms on, 125ms off)
            if (elapsed >= 125) {
                m_anim_step = !m_anim_step;
                m_led.setPixel(0, m_anim_step ? m_color : Colors::OFF);
                m_led.show();
                m_last_update_ms = now;
            }
            break;

        case Pattern::PULSE: {
            // Breathing effect - sine wave approximation
            if (elapsed >= 20) {  // Update at ~50Hz
                // Simple triangle wave for brightness
                static const uint8_t lut[] = {
                    0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 110, 120,
                    130, 140, 150, 160, 170, 180, 190, 200, 210, 220, 230,
                    240, 250, 255, 250, 240, 230, 220, 210, 200, 190, 180,
                    170, 160, 150, 140, 130, 120, 110, 100, 90, 80, 70,
                    60, 50, 40, 30, 20, 10
                };
                constexpr uint8_t LUT_SIZE = sizeof(lut);

                uint8_t brightness = lut[m_anim_step % LUT_SIZE];
                m_led.setPixel(0, m_color.scaled(brightness));
                m_led.show();

                m_anim_step = (m_anim_step + 1) % LUT_SIZE;
                m_last_update_ms = now;
            }
            break;
        }

        case Pattern::DOUBLE_BLINK:
            // Two quick blinks, then pause
            // Pattern: ON-OFF-ON-OFF-pause (100ms each, 600ms pause)
            if (elapsed >= 100) {
                switch (m_anim_step) {
                    case 0: case 2:
                        m_led.setPixel(0, m_color);
                        break;
                    case 1: case 3:
                        m_led.setPixel(0, Colors::OFF);
                        break;
                    default:
                        m_led.setPixel(0, Colors::OFF);
                        break;
                }
                m_led.show();

                m_anim_step++;
                if (m_anim_step >= 10) {  // 4 states + 6 pause states
                    m_anim_step = 0;
                }
                m_last_update_ms = now;
            }
            break;

        case Pattern::SOS: {
            // ... --- ... in morse code
            // S = 3 short, O = 3 long
            // Short = 100ms, Long = 300ms, gap = 100ms, letter gap = 300ms
            static const uint16_t sos_timing[] = {
                100, 100, 100, 100, 100, 300,  // S: dot dot dot + gap
                300, 100, 300, 100, 300, 300,  // O: dash dash dash + gap
                100, 100, 100, 100, 100, 700   // S: dot dot dot + long pause
            };
            static const bool sos_state[] = {
                true, false, true, false, true, false,
                true, false, true, false, true, false,
                true, false, true, false, true, false
            };
            constexpr uint8_t SOS_STEPS = 18;

            if (elapsed >= sos_timing[m_anim_step % SOS_STEPS]) {
                m_led.setPixel(0, sos_state[m_anim_step % SOS_STEPS] ?
                               m_color : Colors::OFF);
                m_led.show();

                m_anim_step = (m_anim_step + 1) % SOS_STEPS;
                m_last_update_ms = now;
            }
            break;
        }
    }
}

} // namespace hal
} // namespace rocketchip
