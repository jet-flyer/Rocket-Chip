/**
 * @file radio_rx.cpp
 * @brief Ground Station Radio Receiver
 *
 * Receives LoRa packets and outputs to USB serial.
 * Acts as a wireless serial bridge for debugging.
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX (#6130) - or any RP2350 Feather
 * - LoRa Radio FeatherWing RFM95W (#3231)
 *
 * Pair with radio_tx_test or flight firmware for transmit side.
 *
 * Output format:
 *   [RSSI dBm, SNR dB] payload_data
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

// Global radio instance
static Radio_RFM95W* g_radio = nullptr;

// Statistics
static uint32_t g_rx_count = 0;
static uint32_t g_rx_errors = 0;
static int16_t g_min_rssi = 0;
static int16_t g_max_rssi = -200;
static int32_t g_rssi_sum = 0;

void printStatus() {
    printf("\n=== Ground Station RX Status ===\n");
    printf("  Packets received: %lu\n", g_rx_count);
    printf("  CRC errors:       %lu\n", g_rx_errors);
    if (g_rx_count > 0) {
        printf("  RSSI range:       %d to %d dBm\n", g_min_rssi, g_max_rssi);
        printf("  RSSI average:     %ld dBm\n", g_rssi_sum / (int32_t)g_rx_count);
    }
    printf("=================================\n\n");
}

int main() {
    // Initialize LED
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Quick blink to show board is alive
    for (int i = 0; i < 5; i++) {
        gpio_put(PIN_LED, 1);
        busy_wait_ms(50);
        gpio_put(PIN_LED, 0);
        busy_wait_ms(50);
    }

    // Initialize HAL (timing, etc.)
    initHAL();

    // Create radio instance with default pins
    g_radio = new Radio_RFM95W();

    // Initialize USB
    stdio_init_all();

    // Blink pattern while waiting for USB (double blink = RX mode)
    while (!stdio_usb_connected()) {
        gpio_put(PIN_LED, 1);
        sleep_ms(50);
        gpio_put(PIN_LED, 0);
        sleep_ms(50);
        gpio_put(PIN_LED, 1);
        sleep_ms(50);
        gpio_put(PIN_LED, 0);
        sleep_ms(300);
    }
    sleep_ms(500);

    // Print header
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip Ground Station Receiver\n");
    printf("  RFM95W @ 915 MHz LoRa\n");
    printf("========================================\n\n");

    // Initialize radio
    printf("Initializing radio...\n");

    RadioConfig config;
    config.frequency_mhz = 915.0f;
    config.tx_power_dbm = 17;        // Not used for RX, but sets PA state
    config.spreading_factor = 7;     // Must match TX
    config.bandwidth_hz = 125000;    // Must match TX
    config.coding_rate = 5;          // Must match TX

    if (!g_radio->begin(config)) {
        printf("!!! Radio init FAILED !!!\n");
        printf("Check wiring and FeatherWing connection.\n");

        // Error blink pattern (fast)
        while (true) {
            gpio_put(PIN_LED, 1);
            sleep_ms(50);
            gpio_put(PIN_LED, 0);
            sleep_ms(50);
        }
    }

    printf("Radio initialized OK!\n");
    printf("  Frequency: %.1f MHz\n", config.frequency_mhz);
    printf("  SF%d, BW %lu Hz, CR 4/%d\n",
           config.spreading_factor, config.bandwidth_hz, config.coding_rate);

    printf("\n--- Listening for packets ---\n\n");

    // Start continuous receive
    g_radio->startReceive();

    // Main RX loop
    RadioPacket packet;
    uint32_t last_status = 0;

    while (true) {
        // Check for incoming packet (non-blocking)
        RadioResult result = g_radio->receive(packet, 100);

        if (result == RadioResult::OK) {
            // Got a packet!
            gpio_put(PIN_LED, 1);

            // Update statistics
            g_rx_count++;
            g_rssi_sum += packet.rssi;
            if (packet.rssi < g_min_rssi || g_min_rssi == 0) g_min_rssi = packet.rssi;
            if (packet.rssi > g_max_rssi) g_max_rssi = packet.rssi;

            // Null-terminate for safe printing
            if (packet.length < sizeof(packet.data)) {
                packet.data[packet.length] = '\0';
            } else {
                packet.data[sizeof(packet.data) - 1] = '\0';
            }

            // Output with signal quality
            printf("[%d dBm, %d dB] %s",
                   packet.rssi, packet.snr, (char*)packet.data);

            // Add newline if packet doesn't have one
            if (packet.length > 0 && packet.data[packet.length - 1] != '\n') {
                printf("\n");
            }

            gpio_put(PIN_LED, 0);
        }
        else if (result == RadioResult::ERR_CRC) {
            g_rx_errors++;
            printf("[CRC ERROR]\n");
        }
        // ERR_TIMEOUT and ERR_NO_PACKET are normal, just means no packet yet

        // Print status every 30 seconds
        uint32_t now = Timing::millis();
        if ((now - last_status) >= 30000) {
            last_status = now;
            printStatus();
        }
    }

    return 0;
}
