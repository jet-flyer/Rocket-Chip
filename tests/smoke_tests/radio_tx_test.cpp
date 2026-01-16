/**
 * @file radio_tx_test.cpp
 * @brief Radio TX smoke test - sends periodic debug packets
 *
 * Tests the RFM95W LoRa radio in transmit mode.
 * Sends incrementing counter and sensor-like data for testing.
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX (#6130)
 * - LoRa Radio FeatherWing RFM95W (#3231)
 *
 * Pair with ground_station/radio_rx for receiving.
 */

#include "HAL.h"
#include "Radio_RFM95W.h"

#include "pico/stdlib.h"
#include "hardware/gpio.h"

#include <cstdio>
#include <cstring>

using namespace rocketchip::hal;

// Onboard LED
constexpr uint8_t PIN_LED = PICO_DEFAULT_LED_PIN;

// Packet send interval (ms)
constexpr uint32_t TX_INTERVAL_MS = 500;

// Global radio instance
static Radio_RFM95W* g_radio = nullptr;

// Test statistics
static uint32_t g_tx_count = 0;
static uint32_t g_tx_errors = 0;

void printStatus() {
    printf("\n=== Radio TX Status ===\n");
    printf("  Packets sent:   %lu\n", g_tx_count);
    printf("  Errors:         %lu\n", g_tx_errors);
    printf("  Success rate:   %.1f%%\n",
           g_tx_count > 0 ? 100.0f * (g_tx_count - g_tx_errors) / g_tx_count : 0.0f);
    printf("========================\n\n");
}

int main() {
    // Initialize LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Quick blink to show board is alive
    for (int i = 0; i < 3; i++) {
        gpio_put(PIN_LED, 1);
        busy_wait_ms(100);
        gpio_put(PIN_LED, 0);
        busy_wait_ms(100);
    }

    // Initialize HAL (timing, etc.)
    initHAL();

    // Create radio instance with default pins
    g_radio = new Radio_RFM95W();

    // Initialize USB
    stdio_init_all();

    // Blink while waiting for USB
    while (!stdio_usb_connected()) {
        gpio_put(PIN_LED, 1);
        sleep_ms(100);
        gpio_put(PIN_LED, 0);
        sleep_ms(100);
    }
    sleep_ms(500);

    // Print header
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Radio TX Smoke Test\n");
    printf("  RFM95W @ 915 MHz LoRa\n");
    printf("========================================\n\n");

    // Initialize radio
    printf("Initializing radio...\n");
    printf("  CS=GPIO%d, RST=GPIO%d, IRQ=GPIO%d\n",
           Radio_RFM95W::DEFAULT_CS_PIN,
           Radio_RFM95W::DEFAULT_RST_PIN,
           Radio_RFM95W::DEFAULT_IRQ_PIN);

    RadioConfig config;
    config.frequency_mhz = 915.0f;
    config.tx_power_dbm = 17;        // Good range without exceeding limits
    config.spreading_factor = 7;     // Fast, decent range
    config.bandwidth_hz = 125000;    // Standard LoRa bandwidth
    config.coding_rate = 5;          // 4/5 coding

    if (!g_radio->begin(config)) {
        printf("!!! Radio init FAILED !!!\n");
        printf("Check wiring and FeatherWing connection.\n");

        // Error blink pattern
        while (true) {
            gpio_put(PIN_LED, 1);
            sleep_ms(50);
            gpio_put(PIN_LED, 0);
            sleep_ms(50);
        }
    }

    printf("Radio initialized OK!\n");
    printf("  Frequency: %.1f MHz\n", config.frequency_mhz);
    printf("  TX Power:  %d dBm\n", config.tx_power_dbm);
    printf("  SF%d, BW %lu Hz, CR 4/%d\n",
           config.spreading_factor, config.bandwidth_hz, config.coding_rate);

    printf("\n--- Starting TX loop (every %lu ms) ---\n\n", TX_INTERVAL_MS);

    // Main TX loop
    uint32_t last_tx = 0;
    uint32_t packet_num = 0;

    while (true) {
        uint32_t now = Timing::millis();

        if ((now - last_tx) >= TX_INTERVAL_MS) {
            last_tx = now;
            packet_num++;

            // Build debug packet
            char packet[64];
            int len = snprintf(packet, sizeof(packet),
                              "PKT#%lu T=%lu ms\r\n",
                              packet_num, now);

            // Transmit
            gpio_put(PIN_LED, 1);  // LED on during TX

            RadioResult result = g_radio->send(
                reinterpret_cast<uint8_t*>(packet), len);

            gpio_put(PIN_LED, 0);  // LED off

            g_tx_count++;

            if (result == RadioResult::OK) {
                printf("TX #%lu: \"%s\" (%d bytes)\n", packet_num, packet, len);
            } else {
                g_tx_errors++;
                printf("TX #%lu: ERROR (code %d)\n", packet_num, static_cast<int>(result));
            }

            // Print status every 20 packets
            if (packet_num % 20 == 0) {
                printStatus();
            }
        }

        // Small delay to avoid busy loop
        Timing::delayMs(1);
    }

    return 0;
}
