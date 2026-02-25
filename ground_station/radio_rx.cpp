// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file radio_rx.cpp
 * @brief Ground Station LoRa Receiver (Fruit Jam + RFM95W breakout)
 *
 * Standalone RX bridge: receives LoRa packets and prints to USB serial.
 * Uses the same rfm95w driver as the flight firmware.
 *
 * NeoPixel signal strength: 5 onboard NeoPixels show RSSI as a bar graph.
 *   0 lit = no packets recently    (>2s since last packet)
 *   1 lit = very weak              (RSSI < -110 dBm)
 *   2 lit = weak                   (RSSI < -100 dBm)
 *   3 lit = moderate               (RSSI < -80 dBm)
 *   4 lit = good                   (RSSI < -60 dBm)
 *   5 lit = excellent              (RSSI >= -60 dBm)
 * Color: green (good) → yellow → red (weak)
 *
 * Hardware:
 *   Adafruit Fruit Jam (#6200) — RP2350B
 *   Adafruit RFM95W breakout (#3072) — wired via jumpers
 *
 * Wiring (Fruit Jam header → RFM95W breakout):
 *   3V    → VIN
 *   GND   → GND
 *   SCK  (GPIO30) → SCK
 *   MOSI (GPIO31) → MOSI
 *   MISO (GPIO28) → MISO
 *   D10  (GPIO10) → CS
 *   D6   (GPIO6)  → RST
 *   D7   (GPIO7)  → G0 (IRQ/DIO0)
 *
 * Build: see ground_station/CMakeLists.txt
 */

#include "rfm95w.h"
#include "spi_bus.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "ws2812.pio.h"

#include <cstdio>

// ============================================================================
// Fruit Jam Pin Configuration
// ============================================================================

// SPI1 on Fruit Jam header (SPI0 is wired to onboard SD card)
constexpr uint8_t kGsSpiMiso = 28;   // GPIO28 — header MISO
constexpr uint8_t kGsSpiSck  = 30;   // GPIO30 — header SCK
constexpr uint8_t kGsSpiMosi = 31;   // GPIO31 — header MOSI

// Radio control pins (free GPIOs on header)
constexpr uint8_t kGsRadioCs  = 10;  // GPIO10 — header D10
constexpr uint8_t kGsRadioRst = 6;   // GPIO6  — header D6
constexpr uint8_t kGsRadioIrq = 7;   // GPIO7  — header D7

// Onboard LED — GPIO29, active low on Fruit Jam
constexpr uint8_t kGsLedPin = 29;

// Onboard NeoPixels — 5 on GPIO32
constexpr uint kGsNeoPin = 32;
constexpr uint kGsNeoCount = 5;

// Button 3 — GPIO5, active low (has pull-up)
constexpr uint kGsButton3 = 5;

// ============================================================================
// NeoPixel Signal Strength
// ============================================================================

static PIO g_neoPio;
static uint g_neoSm;
static uint g_neoOffset;

static void neo_init(void) {
    g_neoPio = pio0;
    g_neoSm = 0;
    g_neoOffset = 0;

    // GPIO32 is above the default PIO range (0-31). On RP2350B, each PIO
    // block sees a 32-pin window starting at gpiobase. Set gpiobase=16
    // so the PIO can access GPIOs 16-47 (which includes GPIO32).
    // Reference: https://github.com/raspberrypi/pico-sdk/issues/2030
    if (!pio_can_add_program(pio0, &ws2812_program)) {
        g_neoPio = pio1;
    }

    // Set gpiobase BEFORE program init — SDK pin functions subtract gpiobase
    g_neoPio->gpiobase = 16;

    g_neoOffset = pio_add_program(g_neoPio, &ws2812_program);
    g_neoSm = pio_claim_unused_sm(g_neoPio, true);
    ws2812_program_init(g_neoPio, g_neoSm, g_neoOffset,
                        kGsNeoPin, 800000, false);
}

static void neo_put_pixel(uint32_t grb) {
    pio_sm_put_blocking(g_neoPio, g_neoSm, grb << 8u);
}

static uint32_t neo_rgb(uint8_t r, uint8_t g, uint8_t b) {
    // WS2812 expects GRB order
    return (static_cast<uint32_t>(g) << 16) |
           (static_cast<uint32_t>(r) << 8) |
           static_cast<uint32_t>(b);
}

// Map RSSI to 0-5 bars lit, with color gradient
static void neo_show_rssi(int16_t rssi, bool active) {
    uint bars = 0;
    if (!active) {
        bars = 0;
    } else if (rssi >= -60) {
        bars = 5;
    } else if (rssi >= -80) {
        bars = 4;
    } else if (rssi >= -100) {
        bars = 3;
    } else if (rssi >= -110) {
        bars = 2;
    } else {
        bars = 1;
    }

    // Colors per bar position: bar 0 (weakest) = red, bar 4 (strongest) = green
    // Dim brightness (~25%) to avoid blinding at desk
    static const uint32_t kBarColors[5] = {
        neo_rgb(20, 0, 0),   // bar 0: red
        neo_rgb(20, 8, 0),   // bar 1: orange
        neo_rgb(20, 20, 0),  // bar 2: yellow
        neo_rgb(8, 20, 0),   // bar 3: yellow-green
        neo_rgb(0, 20, 0),   // bar 4: green
    };

    for (uint i = 0; i < kGsNeoCount; i++) {
        if (i < bars) {
            neo_put_pixel(kBarColors[i]);
        } else {
            neo_put_pixel(0);  // off
        }
    }
}

static void neo_all_off(void) {
    for (uint i = 0; i < kGsNeoCount; i++) {
        neo_put_pixel(0);
    }
}

// ============================================================================
// Button 3 — NeoPixel On/Off Toggle
// ============================================================================

static bool g_neoEnabled = true;
static bool g_btn3LastState = true;   // Active low — true = released
static uint32_t g_btn3LastMs = 0;

static void button_init(void) {
    gpio_init(kGsButton3);
    gpio_set_dir(kGsButton3, GPIO_IN);
    gpio_pull_up(kGsButton3);
}

// Poll button, toggle NeoPixels on press. 200ms debounce.
static void button_poll(uint32_t nowMs) {
    bool pressed = !gpio_get(kGsButton3);  // Active low
    if (pressed && g_btn3LastState && (nowMs - g_btn3LastMs > 200)) {
        g_neoEnabled = !g_neoEnabled;
        g_btn3LastMs = nowMs;
        if (!g_neoEnabled) {
            neo_all_off();
        }
    }
    g_btn3LastState = !pressed;
}

// ============================================================================
// Link Quality Statistics
// ============================================================================

static uint32_t g_pkt_count = 0;
static uint32_t g_crc_errors = 0;
static int16_t  g_rssi_min = 0;
static int16_t  g_rssi_max = -200;
static int32_t  g_rssi_sum = 0;
static int8_t   g_snr_min = 127;
static int8_t   g_snr_max = -128;
static int32_t  g_snr_sum = 0;
static uint32_t g_last_pkt_ms = 0;
static int16_t  g_last_rssi = -200;

static void print_link_status(void) {
    printf("\n--- Link Quality ---\n");
    printf("  Packets: %lu  CRC errors: %lu\n",
           (unsigned long)g_pkt_count, (unsigned long)g_crc_errors);
    if (g_pkt_count > 0) {
        printf("  RSSI: %d / %d / %d dBm (min/avg/max)\n",
               g_rssi_min,
               static_cast<int16_t>(g_rssi_sum / static_cast<int32_t>(g_pkt_count)),
               g_rssi_max);
        printf("  SNR:  %d / %d / %d dB (min/avg/max)\n",
               g_snr_min,
               static_cast<int8_t>(g_snr_sum / static_cast<int32_t>(g_pkt_count)),
               g_snr_max);
        if (g_last_pkt_ms > 0) {
            uint32_t elapsed = to_ms_since_boot(get_absolute_time()) - g_last_pkt_ms;
            if (elapsed < 10000) {
                printf("  Last pkt: %lu ms ago\n", (unsigned long)elapsed);
            } else {
                printf("  Last pkt: %lu s ago\n", (unsigned long)(elapsed / 1000));
            }
        }
    }
    printf("--------------------\n\n");
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // LED init — active low on Fruit Jam (LOW = on, HIGH = off)
    gpio_init(kGsLedPin);
    gpio_set_dir(kGsLedPin, GPIO_OUT);
    gpio_put(kGsLedPin, 1);  // Off

    // NeoPixel + button init
    neo_init();
    button_init();
    // Startup sweep — purple across all 5
    for (uint i = 0; i < kGsNeoCount; i++) {
        neo_put_pixel(neo_rgb(10, 0, 15));
    }
    sleep_ms(300);
    for (uint i = 0; i < kGsNeoCount; i++) {
        neo_put_pixel(0);
    }

    // Quick blink to show board is alive
    for (int i = 0; i < 5; i++) {
        gpio_put(kGsLedPin, 0);  // On
        busy_wait_ms(50);
        gpio_put(kGsLedPin, 1);  // Off
        busy_wait_ms(50);
    }

    // Init SPI bus (SPI1 for Fruit Jam — gs_spi.cpp)
    spi_bus_init();

    // Init radio before USB (per LL Entry 4: flash ops before USB)
    static rfm95w_t radio;
    bool radio_ok = rfm95w_init(&radio, kGsRadioCs, kGsRadioRst, kGsRadioIrq);

    // Init USB
    stdio_init_all();

    // Blink while waiting for USB terminal (double blink = RX mode)
    while (!stdio_usb_connected()) {
        gpio_put(kGsLedPin, 0);
        sleep_ms(50);
        gpio_put(kGsLedPin, 1);
        sleep_ms(50);
        gpio_put(kGsLedPin, 0);
        sleep_ms(50);
        gpio_put(kGsLedPin, 1);
        sleep_ms(300);
    }
    sleep_ms(500);

    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Ground Station Receiver\n");
    printf("  Fruit Jam + RFM95W breakout\n");
    printf("  915 MHz LoRa SF7 BW125 CR4/5\n");
    printf("========================================\n\n");

    if (!radio_ok) {
        printf("!!! Radio init FAILED !!!\n");
        printf("Check wiring: SCK=%d MOSI=%d MISO=%d CS=%d RST=%d IRQ=%d\n",
               kGsSpiSck, kGsSpiMosi, kGsSpiMiso,
               kGsRadioCs, kGsRadioRst, kGsRadioIrq);
        // Error pattern: all red
        for (uint i = 0; i < kGsNeoCount; i++) {
            neo_put_pixel(neo_rgb(30, 0, 0));
        }
        // Error blink — fast
        while (true) {
            gpio_put(kGsLedPin, 0);
            sleep_ms(50);
            gpio_put(kGsLedPin, 1);
            sleep_ms(50);
        }
    }

    printf("Radio initialized OK!\n");
    printf("  CS=%d RST=%d IRQ=%d\n", kGsRadioCs, kGsRadioRst, kGsRadioIrq);
    printf("  NeoPixels: %d on GPIO%d (signal strength)\n", kGsNeoCount, kGsNeoPin);
    printf("\n--- Listening for packets ---\n\n");

    // Start continuous receive
    rfm95w_start_rx(&radio);

    // Main RX loop
    uint32_t last_status_ms = 0;
    uint32_t last_neo_ms = 0;
    uint8_t buf[rfm95w::kMaxPayload];

    while (true) {
        if (rfm95w_available(&radio)) {
            uint8_t len = rfm95w_recv(&radio, buf, sizeof(buf));

            if (len > 0) {
                // Packet received
                gpio_put(kGsLedPin, 0);  // LED on
                g_pkt_count++;
                g_last_pkt_ms = to_ms_since_boot(get_absolute_time());

                int16_t rssi = radio.last_rssi;
                int8_t snr = radio.last_snr;
                g_last_rssi = rssi;

                // Update stats
                g_rssi_sum += rssi;
                if (rssi < g_rssi_min || g_rssi_min == 0) { g_rssi_min = rssi; }
                if (rssi > g_rssi_max) { g_rssi_max = rssi; }
                g_snr_sum += snr;
                if (snr < g_snr_min) { g_snr_min = snr; }
                if (snr > g_snr_max) { g_snr_max = snr; }

                // Update NeoPixel signal bar
                if (g_neoEnabled) { neo_show_rssi(rssi, true); }

                // Print packet
                printf("[%lu] RSSI:%d SNR:%d len:%d | ",
                       (unsigned long)g_pkt_count, rssi, snr, len);

                // Print as ASCII if all printable, else hex
                bool printable = true;
                for (uint8_t i = 0; i < len; i++) {
                    if (buf[i] < 32 || buf[i] > 126) {
                        printable = false;
                        break;
                    }
                }

                if (printable) {
                    buf[len] = '\0';
                    printf("%s\n", reinterpret_cast<char*>(buf));
                } else {
                    for (uint8_t i = 0; i < len; i++) {
                        printf("%02X ", buf[i]);
                    }
                    printf("\n");
                }

                gpio_put(kGsLedPin, 1);  // LED off
            } else {
                // CRC error — recv returned 0
                g_crc_errors++;
                printf("[CRC ERROR]\n");
            }

            // Re-arm RX after receiving
            rfm95w_start_rx(&radio);
        }

        uint32_t now = to_ms_since_boot(get_absolute_time());

        // Button 3 toggles NeoPixels on/off
        button_poll(now);

        // Fade NeoPixels to off if no packet in 2 seconds
        if (g_neoEnabled && (now - last_neo_ms >= 500)) {
            last_neo_ms = now;
            bool active = (g_last_pkt_ms > 0) &&
                          (now - g_last_pkt_ms < 2000);
            neo_show_rssi(g_last_rssi, active);
        }

        // Print link quality summary every 30 seconds
        if (now - last_status_ms >= 30000) {
            last_status_ms = now;
            print_link_status();
        }
    }

    return 0;
}
