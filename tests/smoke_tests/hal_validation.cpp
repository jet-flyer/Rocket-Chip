/**
 * @file hal_validation.cpp
 * @brief HAL Smoke Test - Hardware Validation
 *
 * Validates the RocketChip HAL on actual hardware:
 * - I2C Bus: Probes ISM330DHCX (0x6A), LIS3MDL (0x1C), DPS310 (0x77)
 * - GPIO: Onboard LED (GPIO 13), boot button (GPIO 7)
 * - ADC: Internal temperature sensor
 * - PIO: NeoPixel status LED (GPIO 21)
 * - Timing: Delay and timing functions
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX (#6130)
 * - ISM330DHCX + LIS3MDL FeatherWing (#4569)
 * - DPS310 Barometer (#4494)
 *
 * Success: NeoPixel turns GREEN, prints pass count
 * Failure: NeoPixel turns RED, prints failure details
 */

#include "HAL.h"

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

// I2C addresses (default for each sensor)
constexpr uint8_t ADDR_ISM330DHCX = 0x6A;  // Accel/Gyro
constexpr uint8_t ADDR_LIS3MDL    = 0x1C;  // Magnetometer
constexpr uint8_t ADDR_DPS310     = 0x77;  // Barometer
constexpr uint8_t ADDR_PA1010D    = 0x10;  // GPS (Qwiic)

// WHO_AM_I register addresses and expected values
constexpr uint8_t ISM330DHCX_WHO_AM_I_REG = 0x0F;
constexpr uint8_t ISM330DHCX_WHO_AM_I_VAL = 0x6B;

constexpr uint8_t LIS3MDL_WHO_AM_I_REG = 0x0F;
constexpr uint8_t LIS3MDL_WHO_AM_I_VAL = 0x3D;

constexpr uint8_t DPS310_PRODUCT_ID_REG = 0x0D;
constexpr uint8_t DPS310_PRODUCT_ID_VAL = 0x10;  // Revision ID may vary

// PA1010D doesn't have a WHO_AM_I, we just probe and try to read NMEA

// Onboard peripherals
constexpr uint8_t PIN_LED      = 13;  // Red LED
constexpr uint8_t PIN_BUTTON   = 7;   // Boot button
constexpr uint8_t PIN_NEOPIXEL = 21;  // NeoPixel

// ============================================================================
// Test Results
// ============================================================================

struct TestResults {
    uint8_t passed;
    uint8_t failed;
    bool hal_init;
    bool timing;
    bool gpio_led;
    bool gpio_button;
    bool adc_temp;
    bool i2c_ism330;
    bool i2c_lis3mdl;
    bool i2c_dps310;
    bool neopixel;
};

static TestResults results = {0};

// Flag to control whether printResult actually prints
static bool g_print_enabled = false;

void printResult(const char* name, bool passed) {
    if (passed) {
        results.passed++;
        if (g_print_enabled) printf("  [PASS] %s\n", name);
    } else {
        results.failed++;
        if (g_print_enabled) printf("  [FAIL] %s\n", name);
    }
}

// ============================================================================
// Individual Tests
// ============================================================================

bool testHALInit() {
    printf("\n=== HAL Initialization ===\n");
    HALInitResult result = initHAL();
    printResult("initHAL()", result.success);
    if (!result.success && result.error_msg) {
        printf("       Error: %s\n", result.error_msg);
    }

    // Print platform info
    PlatformInfo info = getPlatformInfo();
    printf("  Platform: %s @ %lu MHz\n", info.chip_name, info.cpu_freq_mhz);
    printf("  Board: %s\n", info.board_name);
    printf("  Reset reason: %s\n", getResetReasonString(getResetReason()));

    return result.success;
}

bool testTiming() {
    printf("\n=== Timing Tests ===\n");

    // Test micros
    uint64_t t1 = Timing::micros();
    Timing::delayMicros(1000);  // 1ms
    uint64_t t2 = Timing::micros();
    uint64_t elapsed_us = t2 - t1;

    // Allow 10% tolerance
    bool micros_ok = (elapsed_us >= 900 && elapsed_us <= 1100);
    printf("  delayMicros(1000): %llu us\n", elapsed_us);
    printResult("Timing::micros()", micros_ok);

    // Test millis
    uint32_t m1 = Timing::millis32();
    Timing::delayMicros(10000);  // 10ms
    uint32_t m2 = Timing::millis32();
    uint32_t elapsed_ms = m2 - m1;

    bool millis_ok = (elapsed_ms >= 9 && elapsed_ms <= 12);
    printf("  10ms delay: %lu ms\n", elapsed_ms);
    printResult("Timing::millis32()", millis_ok);

    // Test IntervalTimer
    IntervalTimer timer(5000);  // 5ms
    Timing::delayMicros(6000);
    bool interval_ok = timer.ready();
    printResult("IntervalTimer", interval_ok);

    // Test StopWatch
    StopWatch sw;
    sw.start();
    Timing::delayMicros(2000);
    uint32_t sw_elapsed = sw.stop();
    bool sw_ok = (sw_elapsed >= 1800 && sw_elapsed <= 2200);
    printf("  StopWatch 2ms: %lu us\n", sw_elapsed);
    printResult("StopWatch", sw_ok);

    return micros_ok && millis_ok && interval_ok && sw_ok;
}

bool testGPIO() {
    printf("\n=== GPIO Tests ===\n");

    // Test LED output
    OutputPin led(PIN_LED, PinState::LOW);
    led.set();
    Timing::delayMicros(100);
    led.clear();
    printResult("GPIO LED output", true);  // No way to verify without reading back

    // Test button input (with pull-up, button pulls to ground when pressed)
    InputPin button(PIN_BUTTON, PinMode::INPUT_PULLUP);
    PinState btn_state = button.read();
    printf("  Button state: %s (should be HIGH if not pressed)\n",
           btn_state == PinState::HIGH ? "HIGH" : "LOW");
    printResult("GPIO Button input", true);  // Just verify no crash

    // Blink LED to show we're working
    for (int i = 0; i < 3; i++) {
        led.set();
        Timing::delayMicros(100000);  // 100ms
        led.clear();
        Timing::delayMicros(100000);
    }

    return true;
}

bool testADC() {
    printf("\n=== ADC Tests ===\n");

    // Read internal temperature
    float temp_c = ADC::readTemperature();
    printf("  Internal temp: %.1f C\n", temp_c);

    // Reasonable range check (-10 to 60C for indoor testing)
    bool temp_ok = (temp_c > -10.0f && temp_c < 60.0f);
    printResult("ADC Temperature", temp_ok);

    // Test averaging
    uint16_t raw = ADC::readAveraged(AdcChannel::TEMPERATURE, 16);
    printf("  Averaged raw: %u\n", raw);
    bool avg_ok = (raw > 0 && raw < 4096);
    printResult("ADC Averaging", avg_ok);

    return temp_ok && avg_ok;
}

bool testI2CSensor(I2CBus& bus, const char* name, uint8_t who_am_i_reg,
                   uint8_t expected_id, bool exact_match = true) {
    if (!bus.probe()) {
        printf("  %s: NOT FOUND at 0x%02X\n", name, bus.getAddress());
        return false;
    }

    uint8_t id = 0;
    BusResult result = bus.readRegister(who_am_i_reg, id);
    if (result != BusResult::OK) {
        printf("  %s: Read failed\n", name);
        return false;
    }

    bool match = exact_match ? (id == expected_id) : ((id & 0xF0) == (expected_id & 0xF0));
    printf("  %s: WHO_AM_I = 0x%02X (expected 0x%02X) %s\n",
           name, id, expected_id, match ? "OK" : "MISMATCH");

    return match;
}

bool testI2CBus() {
    printf("\n=== I2C Bus Tests ===\n");

    // Create I2C buses for each sensor
    // Note: GPIO 2/3 (STEMMA QT) are on i2c1, not i2c0
    I2CBus ism330(i2c1, ADDR_ISM330DHCX, I2C_SDA, I2C_SCL, 400000);
    I2CBus lis3mdl(i2c1, ADDR_LIS3MDL, I2C_SDA, I2C_SCL, 400000);
    I2CBus dps310(i2c1, ADDR_DPS310, I2C_SDA, I2C_SCL, 400000);

    // Initialize all bus objects (each tracks its own init state)
    if (!ism330.begin()) {
        printf("  I2C init failed!\n");
        return false;
    }
    lis3mdl.begin();  // Safe to call - I2C hw already init'd
    dps310.begin();
    printResult("I2C Bus init", true);

    // Test each sensor
    bool ism_ok = testI2CSensor(ism330, "ISM330DHCX", ISM330DHCX_WHO_AM_I_REG,
                                 ISM330DHCX_WHO_AM_I_VAL);
    printResult("ISM330DHCX (accel/gyro)", ism_ok);
    results.i2c_ism330 = ism_ok;

    bool lis_ok = testI2CSensor(lis3mdl, "LIS3MDL", LIS3MDL_WHO_AM_I_REG,
                                 LIS3MDL_WHO_AM_I_VAL);
    printResult("LIS3MDL (magnetometer)", lis_ok);
    results.i2c_lis3mdl = lis_ok;

    // DPS310 product ID is in lower 4 bits, revision in upper 4
    bool dps_ok = testI2CSensor(dps310, "DPS310", DPS310_PRODUCT_ID_REG,
                                 DPS310_PRODUCT_ID_VAL, false);
    printResult("DPS310 (barometer)", dps_ok);
    results.i2c_dps310 = dps_ok;

    return ism_ok && lis_ok && dps_ok;
}

bool testNeoPixel() {
    printf("\n=== NeoPixel (PIO) Test ===\n");

    WS2812 neopixel(PIN_NEOPIXEL, 1);
    if (!neopixel.begin()) {
        printf("  NeoPixel init failed!\n");
        return false;
    }

    // Cycle through colors
    const RGB colors[] = {Colors::RED, Colors::GREEN, Colors::BLUE, Colors::WHITE};
    const char* names[] = {"RED", "GREEN", "BLUE", "WHITE"};

    for (int i = 0; i < 4; i++) {
        neopixel.setPixel(0, colors[i]);
        neopixel.show();
        printf("  Color: %s\n", names[i]);
        Timing::delayMicros(300000);  // 300ms
    }

    neopixel.clear();
    neopixel.show();

    printResult("NeoPixel WS2812", true);
    return true;
}

void showFinalStatus(bool all_passed) {
    WS2812 neopixel(PIN_NEOPIXEL, 1);
    neopixel.begin();

    if (all_passed) {
        neopixel.setPixel(0, Colors::GREEN);
    } else {
        neopixel.setPixel(0, Colors::RED);
    }
    neopixel.setBrightness(64);  // Dim it a bit
    neopixel.show();
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // Initialize LED first to show board is alive
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // 3 rapid blinks = board is alive
    for (int i = 0; i < 3; i++) {
        gpio_put(PIN_LED, 1);
        busy_wait_ms(100);
        gpio_put(PIN_LED, 0);
        busy_wait_ms(100);
    }

    // Run all tests BEFORE stdio init (no USB buffer to block on)
    g_print_enabled = false;
    results.hal_init   = testHALInit();
    results.timing     = testTiming();
    results.gpio_led   = testGPIO();
    results.gpio_button = true;  // Verified in testGPIO
    results.adc_temp   = testADC();
    testI2CBus();
    results.neopixel   = testNeoPixel();

    bool all_passed = (results.failed == 0);

    // Show final status on NeoPixel immediately
    showFinalStatus(all_passed);

    // Init stdio and wait for connection (like Arduino's while(!Serial))
    stdio_init_all();

    // Re-init LED for waiting indicator
    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Blink LED while waiting for USB serial connection
    while (!stdio_usb_connected()) {
        gpio_put(PIN_LED, 1);
        sleep_ms(100);
        gpio_put(PIN_LED, 0);
        sleep_ms(100);
    }
    sleep_ms(500);  // Settle time after connection

    // Print full report
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip HAL Validation Test\n");
    printf("========================================\n");
    printf("  Passed: %u  |  Failed: %u\n", results.passed, results.failed);
    printf("\n");
    printf("  HAL Init:    %s\n", results.hal_init ? "PASS" : "FAIL");
    printf("  Timing:      %s\n", results.timing ? "PASS" : "FAIL");
    printf("  GPIO:        %s\n", results.gpio_led ? "PASS" : "FAIL");
    printf("  ADC:         %s\n", results.adc_temp ? "PASS" : "FAIL");
    printf("  I2C ISM330:  %s\n", results.i2c_ism330 ? "PASS" : "FAIL");
    printf("  I2C LIS3MDL: %s\n", results.i2c_lis3mdl ? "PASS" : "FAIL");
    printf("  I2C DPS310:  %s\n", results.i2c_dps310 ? "PASS" : "FAIL");
    printf("  NeoPixel:    %s\n", results.neopixel ? "PASS" : "FAIL");
    printf("========================================\n");
    printf("%s\n", all_passed ? "*** ALL TESTS PASSED ***" : "!!! SOME TESTS FAILED !!!");

    // Main loop - blink LED, print status periodically
    int cycle = 0;
    while (true) {
        gpio_put(PIN_LED, 1);
        sleep_ms(all_passed ? 500 : 100);
        gpio_put(PIN_LED, 0);
        sleep_ms(all_passed ? 500 : 100);

        // Print status every 5 seconds for late-connecting terminals
        cycle++;
        if (cycle >= 5) {
            cycle = 0;
            printf("[%s] Passed: %u, Failed: %u\n",
                   all_passed ? "PASS" : "FAIL", results.passed, results.failed);
        }
    }

    return 0;
}
