/**
 * @file gps_test.cpp
 * @brief Smoke test for PA1010D GPS module
 *
 * Tests initialization and NMEA parsing for:
 * - I2C connectivity at address 0x10
 * - NMEA sentence reception (GGA, RMC, VTG, GSA)
 * - Position, velocity, time, and fix quality parsing
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX (#6130)
 * - PA1010D GPS module (#4415)
 *
 * GPS connected via I2C1 (STEMMA QT / Qwiic)
 *
 * @note GPS requires clear sky view for satellite fix.
 *       Test will still pass connectivity even without fix.
 */

#include "HAL.h"
#include "GPS_PA1010D.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <cstdio>

using namespace rocketchip::hal;

// ============================================================================
// Hardware Configuration
// ============================================================================

// I2C pins for Feather RP2350 (STEMMA QT / Qwiic)
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

// Onboard LED
constexpr uint8_t PIN_LED = PICO_DEFAULT_LED_PIN;

// ============================================================================
// Test Results
// ============================================================================

struct GPSTestResults {
    bool init_ok;
    bool connected;
    uint32_t sentences_received;
    uint32_t fixes_received;
    bool got_position;
    bool got_velocity;
    bool got_time;
};

static GPSTestResults results = {0};

// ============================================================================
// Sensor Instances
// ============================================================================

static I2CBus* g_gps_bus = nullptr;
static GPS_PA1010D* g_gps = nullptr;

// ============================================================================
// Test Functions
// ============================================================================

bool initGPS() {
    printf("\n=== Initializing PA1010D GPS ===\n");

    // Create bus instance
    g_gps_bus = new I2CBus(i2c1, GPS_PA1010D::I2C_ADDR, I2C_SDA, I2C_SCL, 400000);

    // Initialize the I2C bus first!
    printf("  I2C Bus Init:    ");
    if (!g_gps_bus->begin()) {
        printf("FAILED\n");
        return false;
    }
    printf("OK\n");

    // Create GPS instance
    g_gps = new GPS_PA1010D(g_gps_bus);

    // Check connectivity (bus is now initialized)
    printf("  I2C Connection:  ");
    results.connected = g_gps->isConnected();
    printf("%s\n", results.connected ? "OK" : "FAILED");

    if (!results.connected) {
        printf("  !!! GPS not responding at address 0x%02X !!!\n", GPS_PA1010D::I2C_ADDR);
        return false;
    }

    // Initialize GPS
    printf("  GPS Init:        ");
    results.init_ok = g_gps->begin();
    printf("%s\n", results.init_ok ? "OK" : "FAILED");

    if (results.init_ok) {
        // Configure for 10Hz updates
        printf("  Setting 10Hz:    ");
        bool rate_ok = g_gps->setUpdateRate(GPSUpdateRate::RATE_10HZ);
        printf("%s\n", rate_ok ? "OK" : "FAILED");
    }

    return results.init_ok;
}

void printGPSData(const GPSData& data) {
    printf("\n--- GPS Data ---\n");

    // Fix status
    printf("  Fix Quality: %d (%s)\n",
           static_cast<int>(data.fix_quality),
           data.fix_quality == GPSFixQuality::NO_FIX ? "No Fix" :
           data.fix_quality == GPSFixQuality::GPS_FIX ? "GPS" :
           data.fix_quality == GPSFixQuality::DGPS_FIX ? "DGPS" : "Other");

    printf("  Fix Type:    %s\n",
           data.fix_type == GPSFixType::NO_FIX ? "No Fix" :
           data.fix_type == GPSFixType::FIX_2D ? "2D" : "3D");

    printf("  Satellites:  %d\n", data.satellites);
    printf("  HDOP:        %.2f\n", data.hdop);

    // Position
    if (data.position_valid) {
        results.got_position = true;
        printf("  Position:    %.6f, %.6f\n", data.latitude, data.longitude);
        printf("  Altitude:    %.1f m MSL\n", data.altitude_msl);
    } else {
        printf("  Position:    (not valid)\n");
    }

    // Velocity
    if (data.velocity_valid) {
        results.got_velocity = true;
        printf("  Speed:       %.2f m/s (%.2f knots)\n", data.speed_mps, data.speed_knots);
        printf("  Course:      %.1f deg\n", data.course_deg);
    } else {
        printf("  Velocity:    (not valid)\n");
    }

    // Time
    if (data.time_valid) {
        results.got_time = true;
        printf("  UTC Time:    %04d-%02d-%02d %02d:%02d:%02d.%03d\n",
               data.time.year, data.time.month, data.time.day,
               data.time.hour, data.time.minute, data.time.second,
               data.time.millisecond);
    } else {
        printf("  Time:        (not valid)\n");
    }
}

void printSummary() {
    printf("\n========================================\n");
    printf("  PA1010D GPS Smoke Test Results\n");
    printf("========================================\n");
    printf("  I2C Connected:   %s\n", results.connected ? "PASS" : "FAIL");
    printf("  GPS Init:        %s\n", results.init_ok ? "PASS" : "FAIL");
    printf("  Sentences:       %lu\n", results.sentences_received);
    printf("  Fixes:           %lu\n", results.fixes_received);
    printf("  Got Position:    %s\n", results.got_position ? "YES" : "no (need sky view)");
    printf("  Got Velocity:    %s\n", results.got_velocity ? "YES" : "no");
    printf("  Got Time:        %s\n", results.got_time ? "YES" : "no");
    printf("========================================\n");

    // Connectivity is the minimum pass criteria
    // Actual GPS fix requires outdoor/sky view
    bool basic_pass = results.connected && results.init_ok;
    if (basic_pass) {
        printf("*** CONNECTIVITY TEST PASSED ***\n");
        if (results.got_position) {
            printf("*** FULL GPS FIX ACHIEVED ***\n");
        } else {
            printf("(Move outdoors for satellite fix)\n");
        }
    } else {
        printf("!!! TEST FAILED - Check I2C wiring !!!\n");
    }
}

// ============================================================================
// Main
// ============================================================================

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

    // Initialize HAL (includes timing, etc.)
    initHAL();

    // Initialize GPS (before USB, to start receiving data)
    bool gps_ok = initGPS();

    // Now init USB and wait for connection
    stdio_init_all();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Blink while waiting for USB
    // Fast blink if init failed, slow blink if OK
    while (!stdio_usb_connected()) {
        gpio_put(PIN_LED, 1);
        sleep_ms(gps_ok ? 200 : 50);
        gpio_put(PIN_LED, 0);
        sleep_ms(gps_ok ? 200 : 50);
    }
    sleep_ms(500);

    // Print header
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip PA1010D GPS Smoke Test\n");
    printf("========================================\n");

    // Re-run GPS init with output visible
    initGPS();

    // Continuous reading loop
    printf("\n--- Starting GPS monitoring ---\n");
    printf("    (Sentences will stream as received)\n");
    printf("    (Press Ctrl+C or reset to stop)\n\n");

    int cycle = 0;
    uint32_t last_sentence_count = 0;

    while (true) {
        // Poll GPS - call frequently to process incoming NMEA data
        // The GPS sends data continuously, we need to read it often
        for (int i = 0; i < 100; i++) {
            g_gps->update();
            Timing::delayMs(1);  // ~100 updates per 100ms cycle
        }

        // Update statistics
        results.sentences_received = g_gps->getSentenceCount();
        results.fixes_received = g_gps->getFixCount();

        // Check for new data
        if (g_gps->hasNewData()) {
            GPSData data = g_gps->getData();
            printGPSData(data);

            // Blink LED on new data
            gpio_put(PIN_LED, 1);
            Timing::delayMs(50);
            gpio_put(PIN_LED, 0);
        }

        // Show activity if sentences are coming in
        if (results.sentences_received > last_sentence_count) {
            last_sentence_count = results.sentences_received;
            printf(".");  // Progress dot
            fflush(stdout);
        }

        cycle++;

        // Print summary every ~5 seconds (50 cycles at 100ms each)
        if (cycle % 50 == 0) {
            printSummary();
            printf("\nContinuing GPS monitoring...\n\n");
        }
    }

    return 0;
}
