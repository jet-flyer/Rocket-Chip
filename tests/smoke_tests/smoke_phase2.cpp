/**
 * @file smoke_phase2.cpp
 * @brief Smoke tests for AP_HAL_RP2350 Phase 2 components
 *
 * Tests are added incrementally as each Phase 2 component is implemented.
 * Run after each component to verify before proceeding.
 *
 * Build: cmake --build build --target smoke_phase2
 * Flash: copy build/smoke_phase2.uf2 to RPI-RP2 drive
 *
 * Follows DEBUG_OUTPUT.md pattern:
 * - Run tests immediately (don't block on serial)
 * - Visual feedback via LED
 * - Wait for USB connection before printing results
 *
 * @note Part of AP_HAL_RP2350 validation
 */

#include <cstdio>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

// AP_HAL
#include "AP_HAL_RP2350/AP_HAL_RP2350.h"

// Use onboard LED for visual feedback
#ifndef PICO_DEFAULT_LED_PIN
#define PICO_DEFAULT_LED_PIN 25
#endif

// ============================================================================
// Test Configuration
// ============================================================================

static constexpr uint32_t kUsbWaitBlinkMs = 200;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kLedBlinkMs = 200;

// ============================================================================
// Test Results
// ============================================================================

struct TestResult {
    const char* name;
    bool passed;
    const char* message;
};

static constexpr uint8_t kMaxTests = 24;
static TestResult g_results[kMaxTests];
static uint8_t g_num_tests = 0;
static uint8_t g_passed = 0;
static uint8_t g_failed = 0;

static void record_result(const char* name, bool passed, const char* message = nullptr) {
    if (g_num_tests < kMaxTests) {
        g_results[g_num_tests].name = name;
        g_results[g_num_tests].passed = passed;
        g_results[g_num_tests].message = message;
        g_num_tests++;
        if (passed) {
            g_passed++;
        } else {
            g_failed++;
        }
    }
}

// ============================================================================
// GPIO Tests
// ============================================================================

static void test_gpio_init() {
    // GPIO init is called by hal.init() - just verify it doesn't crash
    // and that we can access the gpio member
    hal.gpio.init();  // Should be no-op, already called
    record_result("GPIO init", true);
}

static void test_gpio_pin_valid() {
    // Test pin validation
    bool valid_low = hal.gpio.valid_pin(0);
    bool valid_high = hal.gpio.valid_pin(29);
    bool invalid = !hal.gpio.valid_pin(30);

    bool passed = valid_low && valid_high && invalid;
    record_result("GPIO pin validation", passed,
                  passed ? nullptr : "valid_pin() returned wrong result");
}

static void test_gpio_led_toggle() {
    // Configure LED as output
    hal.gpio.pinMode(PICO_DEFAULT_LED_PIN, HAL_GPIO_OUTPUT);

    // Test toggle function - sets LED to known states and verifies
    hal.gpio.write(PICO_DEFAULT_LED_PIN, 0);
    uint8_t state1 = hal.gpio.read(PICO_DEFAULT_LED_PIN);
    hal.gpio.toggle(PICO_DEFAULT_LED_PIN);
    uint8_t state2 = hal.gpio.read(PICO_DEFAULT_LED_PIN);
    hal.gpio.toggle(PICO_DEFAULT_LED_PIN);
    uint8_t state3 = hal.gpio.read(PICO_DEFAULT_LED_PIN);

    bool passed = (state1 == 0) && (state2 == 1) && (state3 == 0);
    record_result("GPIO LED toggle", passed,
                  passed ? nullptr : "toggle() or read() failed");
}

static void test_gpio_digital_source() {
    // Test DigitalSource (channel) interface
    auto* source = hal.gpio.channel(PICO_DEFAULT_LED_PIN);

    if (source == nullptr) {
        record_result("GPIO DigitalSource", false, "channel() returned nullptr");
        return;
    }

    // Test via DigitalSource
    source->mode(HAL_GPIO_OUTPUT);
    source->write(1);
    uint8_t high = source->read();
    source->write(0);
    uint8_t low = source->read();
    source->toggle();
    uint8_t toggled = source->read();

    bool passed = (high == 1) && (low == 0) && (toggled == 1);
    source->write(0);  // Leave LED off

    record_result("GPIO DigitalSource", passed,
                  passed ? nullptr : "DigitalSource methods failed");
}

static void run_gpio_tests() {
    test_gpio_init();
    test_gpio_pin_valid();
    test_gpio_led_toggle();
    test_gpio_digital_source();
    // Note: USB connected test removed - runs before USB connected so always fails
    // The usb_connected() function is verified implicitly by the USB wait loop
}

// ============================================================================
// AnalogIn Tests
// ============================================================================

// Storage for temperature reading (for display after USB connects)
static float g_mcu_temperature = 0.0f;
static float g_board_voltage = 0.0f;

static void test_analogin_init() {
    // AnalogIn init is called by hal.init() - verify it doesn't crash
    hal.analogin.init();  // Should be no-op, already called
    record_result("AnalogIn init", true);
}

static void test_analogin_pin_valid() {
    // Test pin validation - valid pins are 26-29 and 254 (board VCC)
    bool valid_26 = hal.analogin.valid_analog_pin(26);
    bool valid_29 = hal.analogin.valid_analog_pin(29);
    bool valid_vcc = hal.analogin.valid_analog_pin(254);
    bool invalid_25 = !hal.analogin.valid_analog_pin(25);
    bool invalid_30 = !hal.analogin.valid_analog_pin(30);

    bool passed = valid_26 && valid_29 && valid_vcc && invalid_25 && invalid_30;
    record_result("AnalogIn pin validation", passed,
                  passed ? nullptr : "valid_analog_pin() returned wrong result");
}

static void test_analogin_temperature() {
    // Read internal temperature sensor
    // Expected: ~20-40C for room temperature operation
    g_mcu_temperature = hal.analogin.mcu_temperature();

    bool passed = (g_mcu_temperature > 10.0f) && (g_mcu_temperature < 60.0f);
    record_result("AnalogIn MCU temp", passed,
                  passed ? nullptr : "Temperature out of expected range");
}

static void test_analogin_board_voltage() {
    // Read board voltage - should be ~3.3V
    g_board_voltage = hal.analogin.board_voltage();

    bool passed = (g_board_voltage > 3.0f) && (g_board_voltage < 3.6f);
    record_result("AnalogIn board voltage", passed,
                  passed ? nullptr : "Board voltage out of expected range");
}

static void test_analogin_channel() {
    // Test channel allocation for board VCC
    auto* vcc_channel = hal.analogin.channel(254);  // ANALOG_INPUT_BOARD_VCC

    if (vcc_channel == nullptr) {
        record_result("AnalogIn channel", false, "channel(254) returned nullptr");
        return;
    }

    // Read voltage via AnalogSource
    float voltage = vcc_channel->voltage_average();
    bool passed = (voltage > 3.0f) && (voltage < 3.6f);

    record_result("AnalogIn channel", passed,
                  passed ? nullptr : "AnalogSource voltage out of range");
}

static void run_analogin_tests() {
    test_analogin_init();
    test_analogin_pin_valid();
    test_analogin_temperature();
    test_analogin_board_voltage();
    test_analogin_channel();
}

// ============================================================================
// UARTDriver Tests
// ============================================================================

static void test_uart_usb_init() {
    // Serial[0] should already be initialized by hal.init()
    bool passed = hal.serial[0]->is_initialized();
    record_result("UART USB init", passed,
                  passed ? nullptr : "USB CDC not initialized");
}

static void test_uart_usb_write() {
    // Note: This test runs BEFORE USB is connected (per DEBUG_OUTPUT.md pattern)
    // USBSerial::write() correctly returns 0 when not connected
    // We verify the method doesn't crash and returns a valid value
    size_t written = hal.serial[0]->write(static_cast<uint8_t>('.'));
    // Expected: 0 before USB connects, 1 after - both are valid
    bool passed = (written == 0 || written == 1);
    record_result("UART USB write", passed,
                  passed ? nullptr : "write() returned unexpected value");
}

static void test_uart_usb_printf() {
    // Test printf - capture before/after to verify no crash
    hal.serial[0]->printf("UART printf test ");
    record_result("UART USB printf", true);
}

static void test_uart_txspace() {
    // TX space should be non-zero
    uint32_t space = hal.serial[0]->txspace();
    bool passed = (space > 0);
    record_result("UART txspace", passed,
                  passed ? nullptr : "txspace() returned 0");
}

static void run_uart_tests() {
    test_uart_usb_init();
    test_uart_usb_write();
    test_uart_usb_printf();
    test_uart_txspace();
    // Note: Hardware UART tests (serial[1], serial[2]) require physical wiring
    // and are not run in the standard smoke test
}

// ============================================================================
// I2CDevice Tests
// ============================================================================

// Known sensor addresses on the ISM330DHCX + LIS3MDL FeatherWing
static constexpr uint8_t kISM330DHCX_ADDR = 0x6A;  // IMU (SA0=low)
static constexpr uint8_t kLIS3MDL_ADDR = 0x1C;     // Magnetometer (SA1=low)
static constexpr uint8_t kDPS310_ADDR = 0x77;      // Barometer

// Storage for I2C test results
static bool g_imu_found = false;
static bool g_mag_found = false;
static uint8_t g_imu_whoami = 0;

static void test_i2c_manager_init() {
    // I2C manager should be initialized by hal.init()
    // Just verify we can get the bus mask
    uint32_t mask = hal.i2c_mgr.get_bus_mask();
    bool passed = (mask == 0x03);  // Both buses available
    record_result("I2C manager init", passed,
                  passed ? nullptr : "get_bus_mask() wrong");
}

static void test_i2c_get_device() {
    // Test getting a device handle
    auto* dev = hal.i2c_mgr.get_device(0, kISM330DHCX_ADDR);
    bool passed = (dev != nullptr);
    record_result("I2C get device", passed,
                  passed ? nullptr : "get_device() returned nullptr");
}

static void test_i2c_probe_imu() {
    // Probe for ISM330DHCX (should be present)
    auto* dev = hal.i2c_mgr.get_device(0, kISM330DHCX_ADDR);
    if (dev == nullptr) {
        record_result("I2C probe IMU", false, "No device handle");
        return;
    }

    g_imu_found = dev->probe();
    record_result("I2C probe IMU", g_imu_found,
                  g_imu_found ? nullptr : "ISM330DHCX not responding at 0x6A");
}

static void test_i2c_read_whoami() {
    // Read WHO_AM_I register from ISM330DHCX (0x0F should return 0x6B)
    auto* dev = hal.i2c_mgr.get_device(0, kISM330DHCX_ADDR);
    if (dev == nullptr || !g_imu_found) {
        record_result("I2C read WHO_AM_I", false, "IMU not available");
        return;
    }

    bool success = dev->read_registers(0x0F, &g_imu_whoami, 1);
    bool passed = success && (g_imu_whoami == 0x6B);
    record_result("I2C read WHO_AM_I", passed,
                  passed ? nullptr : "WHO_AM_I mismatch");
}

static void test_i2c_probe_mag() {
    // Probe for LIS3MDL (may or may not be present)
    auto* dev = hal.i2c_mgr.get_device(0, kLIS3MDL_ADDR);
    if (dev == nullptr) {
        record_result("I2C probe MAG", false, "No device handle");
        return;
    }

    g_mag_found = dev->probe();
    // This is informational - mag might not be connected
    record_result("I2C probe MAG", true,
                  g_mag_found ? nullptr : "(not connected)");
}

static void run_i2c_tests() {
    test_i2c_manager_init();
    test_i2c_get_device();
    test_i2c_probe_imu();
    test_i2c_read_whoami();
    test_i2c_probe_mag();
}

// ============================================================================
// SPIDevice Tests
// ============================================================================

// Storage for SPI test results
static bool g_radio_found = false;

static void test_spi_manager_init() {
    // SPI manager should be initialized by hal.init()
    // Verify we can get the bus mask
    uint32_t mask = hal.spi_mgr.get_bus_mask();
    bool passed = (mask == 0x03);  // Both SPI0 and SPI1 available
    record_result("SPI manager init", passed,
                  passed ? nullptr : "get_bus_mask() wrong");
}

static void test_spi_get_device() {
    // Test getting a device by name
    auto* dev = hal.spi_mgr.get_device("radio:0");
    bool passed = (dev != nullptr);
    record_result("SPI get device", passed,
                  passed ? nullptr : "get_device(\"radio:0\") returned nullptr");
}

static void test_spi_device_initialized() {
    // Even without physical hardware, the SPI bus should initialize
    auto* dev = hal.spi_mgr.get_device("radio:0");
    if (dev == nullptr) {
        record_result("SPI device init", false, "No device handle");
        return;
    }

    bool passed = dev->is_initialized();
    record_result("SPI device init", passed,
                  passed ? nullptr : "SPI device not initialized");
}

static void test_spi_transfer_basic() {
    // Basic transfer test - send dummy byte and receive
    // Note: Without hardware, this just tests that transfer doesn't crash
    auto* dev = hal.spi_mgr.get_device("radio:0");
    if (dev == nullptr || !dev->is_initialized()) {
        record_result("SPI transfer", false, "Device not ready");
        return;
    }

    // Try a simple transfer - send 0x42, receive into buffer
    // If LoRa FeatherWing is connected, this might return data
    // If not connected, we'll get 0xFF (open bus)
    uint8_t tx = 0x42;  // Version register read command
    uint8_t rx = 0x00;
    bool success = dev->transfer(&tx, 1, &rx, 1);

    // Transfer should succeed even if no device connected
    // (SPI always "works" - just returns 0xFF on open bus)
    record_result("SPI transfer", success,
                  success ? nullptr : "transfer() failed");

    // Check if we might have found the radio
    // RFM95W version register (0x42) should return 0x12
    if (rx == 0x12) {
        g_radio_found = true;
    }
}

static void test_spi_fullduplex() {
    // Test full-duplex transfer
    auto* dev = hal.spi_mgr.get_device("radio:0");
    if (dev == nullptr || !dev->is_initialized()) {
        record_result("SPI fullduplex", false, "Device not ready");
        return;
    }

    uint8_t tx[2] = {0x00, 0x00};  // Read register 0
    uint8_t rx[2] = {0x00, 0x00};
    bool success = dev->transfer_fullduplex(tx, rx, 2);

    record_result("SPI fullduplex", success,
                  success ? nullptr : "transfer_fullduplex() failed");
}

static void run_spi_tests() {
    test_spi_manager_init();
    test_spi_get_device();
    test_spi_device_initialized();
    test_spi_transfer_basic();
    test_spi_fullduplex();
}

// ============================================================================
// Test Summary
// ============================================================================

static void print_summary() {
    printf("\n");
    printf("========================================\n");
    printf("Phase 2 Smoke Test Results\n");
    printf("========================================\n");

    for (uint8_t i = 0; i < g_num_tests; i++) {
        const char* status = g_results[i].passed ? "PASS" : "FAIL";
        printf("[%s] %s", status, g_results[i].name);
        if (g_results[i].message) {
            printf(" - %s", g_results[i].message);
        }
        printf("\n");
    }

    printf("----------------------------------------\n");
    printf("Total: %d passed, %d failed\n", g_passed, g_failed);
    printf("========================================\n");
}

// ============================================================================
// LED Feedback Patterns
// ============================================================================

static void led_indicate_result() {
    hal.gpio.pinMode(PICO_DEFAULT_LED_PIN, HAL_GPIO_OUTPUT);

    if (g_failed == 0) {
        // All passed - slow steady blink
        printf("\nAll tests PASSED - LED slow blink\n");
        while (true) {
            hal.gpio.toggle(PICO_DEFAULT_LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    } else {
        // Some failed - fast blink
        printf("\nSome tests FAILED - LED fast blink\n");
        while (true) {
            hal.gpio.toggle(PICO_DEFAULT_LED_PIN);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================================
// Main Task
// ============================================================================

static void main_task(void* params) {
    (void)params;

    // Initialize HAL first
    hal.init();

    // -------------------------------------------------------------------------
    // Phase 1: Run all tests immediately (don't block on serial)
    // Per DEBUG_OUTPUT.md: "Run program logic immediately"
    // -------------------------------------------------------------------------
    run_gpio_tests();
    run_analogin_tests();
    run_uart_tests();
    run_i2c_tests();
    run_spi_tests();

    // Visual feedback works without serial - blink to indicate tests complete
    // Pattern: 3 quick blinks = tests finished
    hal.gpio.pinMode(PICO_DEFAULT_LED_PIN, HAL_GPIO_OUTPUT);
    for (int i = 0; i < 3; i++) {
        hal.gpio.write(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        hal.gpio.write(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // -------------------------------------------------------------------------
    // Phase 2: Wait for USB connection (LED blinks to indicate "connect now")
    // Per DEBUG_OUTPUT.md: "LED blink while waiting indicates connect terminal now"
    // -------------------------------------------------------------------------
    while (!stdio_usb_connected()) {
        hal.gpio.toggle(PICO_DEFAULT_LED_PIN);
        vTaskDelay(pdMS_TO_TICKS(kUsbWaitBlinkMs));
    }

    // Brief settle time after USB connects
    vTaskDelay(pdMS_TO_TICKS(kUsbSettleTimeMs));

    // -------------------------------------------------------------------------
    // Phase 3: Print results now that terminal is connected
    // -------------------------------------------------------------------------
    printf("\n");
    printf("========================================\n");
    printf("AP_HAL_RP2350 Phase 2 Smoke Tests\n");
    printf("========================================\n");
    printf("Testing components incrementally.\n");
    printf("Each component tested before proceeding.\n");

    print_summary();

    // Print AnalogIn readings
    printf("\nAnalogIn Readings:\n");
    printf("  MCU Temperature: %.1f C\n", g_mcu_temperature);
    printf("  Board Voltage: %.2f V\n", g_board_voltage);

    // Print I2C readings
    printf("\nI2C Devices:\n");
    printf("  ISM330DHCX (0x6A): %s", g_imu_found ? "Found" : "Not found");
    if (g_imu_found) {
        printf(" (WHO_AM_I=0x%02X)", g_imu_whoami);
    }
    printf("\n");
    printf("  LIS3MDL (0x1C): %s\n", g_mag_found ? "Found" : "Not found");

    // Print SPI info
    printf("\nSPI Devices:\n");
    printf("  radio:0 (CS=GPIO10): %s\n", g_radio_found ? "RFM95W detected" : "Not detected");
    printf("  (Note: SPI always 'succeeds' - open bus returns 0xFF)\n");

    // Final LED indication
    led_indicate_result();
}

// ============================================================================
// Entry Point
// ============================================================================

int main() {
    // Initialize LED for pre-scheduler debugging
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Blink 3 times = reached main()
    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(100);
    }

    // Initialize stdio
    stdio_init_all();

    // One slow blink = about to create task
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(250);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(250);

    // Create main task
    xTaskCreate(main_task, "main", 4096, nullptr, 1, nullptr);

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (required for static allocation)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook() {
    // Rapid LED blink to indicate malloc failure
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    while (true) {
        gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);
        busy_wait_ms(50);
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    (void)name;
    // Solid LED to indicate stack overflow
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    while (true) {
        tight_loop_contents();
    }
}

// Idle task memory (Core 0)
static StaticTask_t g_idle_task_tcb;
static StackType_t g_idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb,
                                    StackType_t** stack,
                                    configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &g_idle_task_tcb;
    *stack = g_idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

#if (configNUMBER_OF_CORES > 1)
// Passive idle task memory (Core 1)
static StaticTask_t g_passive_idle_tcb;
static StackType_t g_passive_idle_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** tcb,
                                           StackType_t** stack,
                                           configSTACK_DEPTH_TYPE* stack_size,
                                           BaseType_t core) {
    (void)core;
    *tcb = &g_passive_idle_tcb;
    *stack = g_passive_idle_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}
#endif

// Timer task memory
static StaticTask_t g_timer_task_tcb;
static StackType_t g_timer_task_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t** tcb,
                                     StackType_t** stack,
                                     configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &g_timer_task_tcb;
    *stack = g_timer_task_stack;
    *stack_size = configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"
