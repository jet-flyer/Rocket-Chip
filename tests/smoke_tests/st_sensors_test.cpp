/**
 * @file st_sensors_test.cpp
 * @brief Smoke test for ST platform-independent sensor drivers
 *
 * Tests initialization and data reading for:
 * - ISM330DHCX (6-DoF IMU: accelerometer + gyroscope)
 * - LIS3MDL (3-axis magnetometer)
 * - DPS310 (barometric pressure + temperature)
 *
 * Hardware required:
 * - Adafruit Feather RP2350 HSTX (#6130)
 * - ISM330DHCX + LIS3MDL FeatherWing (#4569)
 * - DPS310 Barometer (#4494)
 *
 * All sensors connected via I2C1 (STEMMA QT / Qwiic)
 */

#include "HAL.h"
#include "IMU_ISM330DHCX.h"
#include "Mag_LIS3MDL.h"
#include "Baro_DPS310.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <cstdio>
#include <cmath>

using namespace rocketchip::hal;

// ============================================================================
// Hardware Configuration
// ============================================================================

// I2C pins for Feather RP2350 (STEMMA QT / Qwiic)
constexpr uint8_t I2C_SDA = 2;
constexpr uint8_t I2C_SCL = 3;

// I2C addresses
constexpr uint8_t ADDR_ISM330DHCX = 0x6A;
constexpr uint8_t ADDR_LIS3MDL    = 0x1C;
constexpr uint8_t ADDR_DPS310     = 0x77;

// Onboard LED
constexpr uint8_t PIN_LED = PICO_DEFAULT_LED_PIN;

// ============================================================================
// Test Results
// ============================================================================

struct SensorResults {
    bool imu_init;
    bool mag_init;
    bool baro_init;
    bool imu_read;
    bool mag_read;
    bool baro_read;
    int samples_collected;
};

static SensorResults results = {0};

// ============================================================================
// Sensor Instances
// ============================================================================

static I2CBus* g_imu_bus = nullptr;
static I2CBus* g_mag_bus = nullptr;
static I2CBus* g_baro_bus = nullptr;

static IMU_ISM330DHCX* g_imu = nullptr;
static Mag_LIS3MDL* g_mag = nullptr;
static Baro_DPS310* g_baro = nullptr;

// ============================================================================
// Test Functions
// ============================================================================

bool initSensors() {
    printf("\n=== Initializing ST Driver Sensors ===\n");

    // Create bus instances (all share I2C1 hardware)
    g_imu_bus = new I2CBus(i2c1, ADDR_ISM330DHCX, I2C_SDA, I2C_SCL, 400000);
    g_mag_bus = new I2CBus(i2c1, ADDR_LIS3MDL, I2C_SDA, I2C_SCL, 400000);
    g_baro_bus = new I2CBus(i2c1, ADDR_DPS310, I2C_SDA, I2C_SCL, 400000);

    // Create sensor instances
    g_imu = new IMU_ISM330DHCX(g_imu_bus);
    g_mag = new Mag_LIS3MDL(g_mag_bus);
    g_baro = new Baro_DPS310(g_baro_bus);

    // Initialize IMU
    printf("  ISM330DHCX (IMU):    ");
    results.imu_init = g_imu->begin();
    printf("%s\n", results.imu_init ? "OK" : "FAILED");

    // Initialize Magnetometer
    printf("  LIS3MDL (Mag):       ");
    results.mag_init = g_mag->begin();
    printf("%s\n", results.mag_init ? "OK" : "FAILED");

    // Initialize Barometer
    printf("  DPS310 (Baro):       ");
    results.baro_init = g_baro->begin();
    printf("%s\n", results.baro_init ? "OK" : "FAILED");

    return results.imu_init && results.mag_init && results.baro_init;
}

void readAllSensors() {
    Vector3f accel, gyro, mag;
    float pressure_pa, baro_temp_c;
    float imu_temp_c, mag_temp_c;

    printf("\n=== Sensor Readings ===\n");

    // Read IMU
    if (results.imu_init) {
        if (g_imu->read(accel, gyro)) {
            results.imu_read = true;
            printf("  IMU Accel:  X=%+7.3f  Y=%+7.3f  Z=%+7.3f  g\n",
                   accel.x, accel.y, accel.z);
            printf("  IMU Gyro:   X=%+8.2f Y=%+8.2f Z=%+8.2f dps\n",
                   gyro.x, gyro.y, gyro.z);

            if (g_imu->readTemperature(imu_temp_c)) {
                printf("  IMU Temp:   %.1f C\n", imu_temp_c);
            }
        } else {
            printf("  IMU: Read FAILED\n");
        }
    }

    // Read Magnetometer
    if (results.mag_init) {
        if (g_mag->read(mag)) {
            results.mag_read = true;
            float magnitude = sqrtf(mag.x * mag.x + mag.y * mag.y + mag.z * mag.z);
            printf("  Mag Field:  X=%+7.3f  Y=%+7.3f  Z=%+7.3f  gauss (|M|=%.3f)\n",
                   mag.x, mag.y, mag.z, magnitude);

            if (g_mag->readTemperature(mag_temp_c)) {
                printf("  Mag Temp:   %.1f C\n", mag_temp_c);
            }
        } else {
            printf("  Mag: Read FAILED\n");
        }
    }

    // Read Barometer
    if (results.baro_init) {
        if (g_baro->read(pressure_pa, baro_temp_c)) {
            results.baro_read = true;
            float altitude_m = Baro_DPS310::pressureToAltitude(pressure_pa);
            printf("  Pressure:   %.2f Pa (%.2f hPa)\n", pressure_pa, pressure_pa / 100.0f);
            printf("  Altitude:   %.1f m (relative to 1013.25 hPa)\n", altitude_m);
            printf("  Baro Temp:  %.2f C\n", baro_temp_c);
        } else {
            printf("  Baro: Read FAILED\n");
        }
    }

    results.samples_collected++;
}

void printSummary() {
    printf("\n========================================\n");
    printf("  ST Driver Smoke Test Results\n");
    printf("========================================\n");
    printf("  ISM330DHCX Init: %s\n", results.imu_init ? "PASS" : "FAIL");
    printf("  LIS3MDL Init:    %s\n", results.mag_init ? "PASS" : "FAIL");
    printf("  DPS310 Init:     %s\n", results.baro_init ? "PASS" : "FAIL");
    printf("  ISM330DHCX Read: %s\n", results.imu_read ? "PASS" : "FAIL");
    printf("  LIS3MDL Read:    %s\n", results.mag_read ? "PASS" : "FAIL");
    printf("  DPS310 Read:     %s\n", results.baro_read ? "PASS" : "FAIL");
    printf("========================================\n");

    bool all_passed = results.imu_init && results.mag_init && results.baro_init &&
                      results.imu_read && results.mag_read && results.baro_read;
    printf("%s\n", all_passed ? "*** ALL TESTS PASSED ***" : "!!! SOME TESTS FAILED !!!");
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

    // Initialize sensors (before USB, to avoid blocking)
    bool sensors_ok = initSensors();

    // Wait a bit for sensors to stabilize
    Timing::delayMs(100);

    // Take an initial reading
    readAllSensors();

    // Now init USB and wait for connection
    stdio_init_all();

    gpio_init(PIN_LED);
    gpio_set_dir(PIN_LED, GPIO_OUT);

    // Blink while waiting for USB
    while (!stdio_usb_connected()) {
        gpio_put(PIN_LED, 1);
        sleep_ms(sensors_ok ? 200 : 50);
        gpio_put(PIN_LED, 0);
        sleep_ms(sensors_ok ? 200 : 50);
    }
    sleep_ms(500);

    // Print header
    printf("\n");
    printf("========================================\n");
    printf("  RocketChip ST Driver Smoke Test\n");
    printf("========================================\n");

    // Re-run sensor init with output visible
    initSensors();

    // Continuous reading loop
    printf("\n--- Starting continuous sensor readings ---\n");
    printf("    (Press Ctrl+C or reset to stop)\n\n");

    int cycle = 0;
    while (true) {
        // Read and print all sensors
        readAllSensors();

        // Blink LED to show activity
        gpio_put(PIN_LED, 1);
        Timing::delayMs(50);
        gpio_put(PIN_LED, 0);

        // Wait before next reading
        Timing::delayMs(450);  // ~2 Hz update rate

        cycle++;

        // Print summary every 10 readings
        if (cycle % 10 == 0) {
            printSummary();
            printf("\nSamples collected: %d\n", results.samples_collected);
            printf("Continuing readings...\n\n");
        }
    }

    return 0;
}
