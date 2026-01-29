/**
 * @file ins_probe_test.cpp
 * @brief AP_InertialSensor probe test for ICM-20948
 *
 * Tests the full ArduPilot AP_InertialSensor integration:
 * - AP_HAL I2CDeviceManager
 * - AP_InertialSensor_Invensensev2 probe
 * - Bank selection callback
 * - Basic sensor reading
 *
 * Hardware: Adafruit Feather RP2350 HSTX with ICM-20948 on I2C1 @ 0x69
 */

#include <cstdio>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// ArduPilot HAL
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <AP_HAL/AP_HAL.h>

// ArduPilot INS
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensensev2.h>

// ArduPilot Param (for persistent calibration)
#include <AP_Param/AP_Param.h>

// HAL reference
extern const AP_HAL::HAL& hal;

// ============================================================================
// Configuration
// ============================================================================

static constexpr uint32_t kUsbWaitBlinkMs = 100;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kI2cBus = 0;        // Bus 0 = Qwiic on Feather
static constexpr uint8_t kIcm20948Addr = 0x69; // AD0 = 1
static constexpr uint32_t kTestTaskStackSize = 4096;
static constexpr UBaseType_t kTestTaskPriority = 2;

// ============================================================================
// Global INS instance
// ============================================================================

static AP_InertialSensor g_ins;

// ============================================================================
// Debug monitor task - prints INS state while init() blocks
// ============================================================================

static volatile bool g_monitor_running = false;

static void monitor_task(void* params) {
    (void)params;

    printf("[MONITOR] Started - will print INS state every 100ms\n");
    fflush(stdout);

    uint32_t start_ms = to_ms_since_boot(get_absolute_time());
    uint32_t iteration = 0;

    while (g_monitor_running) {
        iteration++;
        uint32_t elapsed_ms = to_ms_since_boot(get_absolute_time()) - start_ms;

        // Print gyro sample rate - this shows if register_gyro() was called
        uint16_t rate = g_ins.get_raw_gyro_rate_hz(0);
        uint8_t gyro_count = g_ins.get_gyro_count();
        uint8_t accel_count = g_ins.get_accel_count();

        printf("[MONITOR @%lums #%lu] gyro_count=%u accel_count=%u rate=%uHz\n",
               elapsed_ms, iteration, gyro_count, accel_count, rate);
        fflush(stdout);

        vTaskDelay(pdMS_TO_TICKS(100));  // Fast updates to catch state changes
    }

    printf("[MONITOR] Stopped after %lu iterations\n", iteration);
    vTaskDelete(NULL);
}

// ============================================================================
// Test functions
// ============================================================================

/**
 * @brief Reset ICM-20948 to known state
 *
 * The sensor may be in a running state from a previous boot (MCU resets
 * but sensor stays powered via Qwiic). Reset it to ensure clean I2C comms.
 */
static void reset_icm20948() {
    i2c_inst_t* i2c = i2c1;

    // Initialize I2C
    i2c_init(i2c, 100000);
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    // ICM-20948 PWR_MGMT_1 register (Bank 0)
    // Bit 7 = DEVICE_RESET (auto-clears)
    constexpr uint8_t ICM_ADDR = 0x69;
    constexpr uint8_t REG_BANK_SEL = 0x7F;
    constexpr uint8_t PWR_MGMT_1 = 0x06;
    constexpr uint8_t DEVICE_RESET = 0x80;

    // Select Bank 0 first
    uint8_t bank_cmd[2] = {REG_BANK_SEL, 0x00};
    i2c_write_timeout_us(i2c, ICM_ADDR, bank_cmd, 2, false, 5000);

    // Send reset command
    uint8_t reset_cmd[2] = {PWR_MGMT_1, DEVICE_RESET};
    i2c_write_timeout_us(i2c, ICM_ADDR, reset_cmd, 2, false, 5000);

    // Wait for reset to complete (datasheet says 100ms max)
    vTaskDelay(pdMS_TO_TICKS(150));

    printf("[DEBUG] ICM-20948 reset sent\n");
}

static void i2c_scan() {
    printf("I2C Bus Scan (Bus 0 - Qwiic / I2C1 on GPIO2/3):\n");
    printf("  Scanning addresses 0x08-0x77...\n");

    // Use direct SDK calls for reliable probing
    i2c_inst_t* i2c = i2c1;  // Qwiic on Feather RP2350 is I2C1

    // Make sure I2C is initialized
    i2c_init(i2c, 100000);  // 100kHz for scanning
    gpio_set_function(2, GPIO_FUNC_I2C);
    gpio_set_function(3, GPIO_FUNC_I2C);
    gpio_pull_up(2);
    gpio_pull_up(3);

    int found = 0;
    for (uint8_t addr = 0x08; addr <= 0x77; addr++) {
        // Try to read 1 byte - if device ACKs, we'll get data (or timeout)
        // If device NACKs, we get PICO_ERROR_GENERIC immediately
        uint8_t dummy;
        int ret = i2c_read_timeout_us(i2c, addr, &dummy, 1, false, 5000);

        if (ret >= 0) {
            printf("  Found device at 0x%02X (read OK)\n", addr);
            found++;
        } else if (ret == PICO_ERROR_TIMEOUT) {
            // Device ACK'd address but didn't respond with data - still present
            printf("  Found device at 0x%02X (timeout - clock stretch?)\n", addr);
            found++;
        }
        // PICO_ERROR_GENERIC means NACK - no device
    }

    if (found == 0) {
        printf("  No devices found!\n");
        printf("  Check: Is the ICM-20948 connected to the Qwiic port?\n");
    } else {
        printf("  Total: %d device(s) found\n", found);
    }
    printf("\n");
}

static bool test_i2c_device_manager() {
    printf("Testing I2C Device Manager...\n");

    // First, scan the bus to see what's there
    i2c_scan();

    // Try ICM-20948 at 0x69 (confirmed by scan)
    // ICM-20948: WHO_AM_I at 0x00 = 0xEA (in Bank 0)
    // Must select Bank 0 first via REG_BANK_SEL (0x7F)
    printf("  Trying ICM-20948 at 0x%02X...\n", kIcm20948Addr);
    auto dev = hal.i2c_mgr->get_device(kI2cBus, kIcm20948Addr);
    if (!dev) {
        printf("    ERROR: get_device() returned null\n");
    } else {
        printf("    Got device handle\n");

        // ICM-20948 has 4 register banks. WHO_AM_I is in Bank 0.
        // REG_BANK_SEL (0x7F) bits [5:4] select the bank.
        // Bank 0 = 0x00, Bank 1 = 0x10, Bank 2 = 0x20, Bank 3 = 0x30
        constexpr uint8_t REG_BANK_SEL = 0x7F;
        constexpr uint8_t BANK_0 = 0x00;

        // Select Bank 0
        bool bank_ok = dev->write_register(REG_BANK_SEL, BANK_0);
        printf("    write_register(0x7F, 0x00) [select Bank 0]: %s\n",
               bank_ok ? "OK" : "FAIL");

        if (bank_ok) {
            uint8_t whoami = 0xFF;
            bool read_ok = dev->read_registers(0x00, &whoami, 1);
            printf("    read_registers(0x00): %s, value=0x%02X (expected 0xEA)\n",
                   read_ok ? "OK" : "FAIL", whoami);
            if (read_ok && whoami == 0xEA) {
                printf("  PASS: ICM-20948 detected!\n");
                fflush(stdout);
                vTaskDelay(pdMS_TO_TICKS(100));  // Give USB time to transmit
                return true;
            }
        }
    }

    // Also try 0x68 in case AD0 is low
    printf("  Trying ICM-20948 at 0x68...\n");
    dev = hal.i2c_mgr->get_device(kI2cBus, 0x68);
    if (dev) {
        constexpr uint8_t REG_BANK_SEL = 0x7F;
        constexpr uint8_t BANK_0 = 0x00;

        // Select Bank 0 first
        if (dev->write_register(REG_BANK_SEL, BANK_0)) {
            uint8_t whoami = 0xFF;
            bool read_ok = dev->read_registers(0x00, &whoami, 1);
            printf("    read_registers(0x00): %s, value=0x%02X\n",
                   read_ok ? "OK" : "FAIL", whoami);
            if (read_ok && whoami == 0xEA) {
                printf("  PASS: ICM-20948 detected at 0x68!\n");
                return true;
            }
        }
    }

    printf("  FAIL: No supported IMU detected\n");
    return false;
}

static bool test_invensensev2_probe() {
    printf("\nTesting AP_InertialSensor_Invensensev2 probe...\n");
    fflush(stdout);

    // Get a fresh device for the probe (probe takes ownership via OwnPtr)
    printf("  Getting device...\n");
    fflush(stdout);
    auto dev = hal.i2c_mgr->get_device(kI2cBus, kIcm20948Addr);
    if (!dev) {
        printf("  FAIL: Could not get device for probe\n");
        return false;
    }
    printf("  Got device, calling probe()...\n");
    fflush(stdout);

    // Probe the sensor
    auto* backend = AP_InertialSensor_Invensensev2::probe(
        g_ins,
        std::move(dev),
        ROTATION_NONE
    );

    printf("  probe() returned\n");
    fflush(stdout);

    if (backend == nullptr) {
        printf("  FAIL: probe() returned null\n");
        printf("  This usually means:\n");
        printf("    - WHO_AM_I didn't match expected value\n");
        printf("    - Bank selection failed\n");
        printf("    - I2C communication error\n");
        return false;
    }

    printf("  PASS: AP_InertialSensor_Invensensev2 probe successful!\n");

    // Backend is owned by INS now, we don't delete it
    return true;
}

static bool test_ins_init() {
    printf("\nTesting AP_InertialSensor init...\n");
    printf("  HAL_INS_PROBE_LIST is defined: %s\n",
#if defined(HAL_INS_PROBE_LIST)
           "YES"
#else
           "NO"
#endif
    );
    fflush(stdout);

    // CRITICAL CHECK: Verify singleton matches our instance
    // If these differ, register_gyro() modifies the wrong object!
    AP_InertialSensor* singleton = AP_InertialSensor::get_singleton();
    printf("  Singleton check: &g_ins=%p, get_singleton()=%p\n",
           (void*)&g_ins, (void*)singleton);
    if (singleton != &g_ins) {
        printf("  WARNING: SINGLETON MISMATCH! Another instance was created!\n");
        printf("  This means register_gyro() will modify the wrong object.\n");
    }
    fflush(stdout);

    // Check sample rate before init (should be 0)
    printf("  Gyro sample rate BEFORE init: %u Hz\n", g_ins.get_raw_gyro_rate_hz(0));
    fflush(stdout);

    // Start monitor task to print sample rate while init() blocks
    // Use priority 6 (above bus thread at 5) so it actually runs
    g_monitor_running = true;
    TaskHandle_t monitor_handle = NULL;
    xTaskCreate(monitor_task, "monitor", 1024, nullptr, 6, &monitor_handle);

    // Initialize at 100Hz sample rate
    printf("  Calling g_ins.init(100)...\n");
    printf("  NOTE: This may block waiting for sensor data. Watch MONITOR logs.\n");
    fflush(stdout);

    // Start init in a timeout context - we'll print progress every second
    uint32_t start_time = time_us_32();

    // This will block until sensor data is received
    g_ins.init(100);

    // Stop monitor task
    g_monitor_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));  // Let it clean up

    uint32_t elapsed_ms = (time_us_32() - start_time) / 1000;
    printf("  init() returned after %lu ms\n", elapsed_ms);
    fflush(stdout);

    // Check sample rate after init
    printf("  Gyro sample rate AFTER init: %u Hz\n", g_ins.get_raw_gyro_rate_hz(0));
    fflush(stdout);

    uint8_t accel_count = g_ins.get_accel_count();
    uint8_t gyro_count = g_ins.get_gyro_count();

    printf("  Accelerometers detected: %u\n", accel_count);
    printf("  Gyroscopes detected: %u\n", gyro_count);

    if (accel_count == 0 || gyro_count == 0) {
        printf("  FAIL: No sensors detected after init\n");
        return false;
    }

    printf("  PASS: INS initialized with sensors\n");
    return true;
}

static void test_sensor_readings() {
    printf("\nReading sensor data...\n");
    printf("Press any key to stop.\n\n");

    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT) {
        // Wait for new sample
        g_ins.wait_for_sample();
        g_ins.update();

        // Get accel and gyro
        Vector3f accel = g_ins.get_accel(0);
        Vector3f gyro = g_ins.get_gyro(0);

        float accel_mag = accel.length();

        printf("Accel: [%+7.3f, %+7.3f, %+7.3f] |%5.2f| m/s^2  "
               "Gyro: [%+7.2f, %+7.2f, %+7.2f] deg/s\r",
               (double)accel.x, (double)accel.y, (double)accel.z,
               (double)accel_mag,
               (double)(gyro.x * RAD_TO_DEG),
               (double)(gyro.y * RAD_TO_DEG),
               (double)(gyro.z * RAD_TO_DEG));
        fflush(stdout);
    }
    printf("\n");
}

// ============================================================================
// Test Task (runs in FreeRTOS context)
// ============================================================================

static void test_task(void* params) {
    (void)params;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Wait for USB connection first so we can see debug output
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(kUsbWaitBlinkMs));
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(kUsbWaitBlinkMs));
    }
    vTaskDelay(pdMS_TO_TICKS(kUsbSettleTimeMs));

    // Print header
    printf("\n");
    printf("========================================\n");
    printf("  AP_InertialSensor Probe Test\n");
    printf("  ICM-20948 via Invensensev2 driver\n");
    printf("========================================\n\n");

    // Reset ICM-20948 first - sensor may be in running state from previous boot
    printf("Resetting ICM-20948...\n");
    reset_icm20948();

    // Initialize HAL subsystems
    printf("Initializing HAL...\n");
    RP2350::hal_init();

    // Initialize AP_Param system from storage
    // This enables parameter persistence across reboots and firmware updates
    printf("Initializing AP_Param...\n");
    AP_Param::setup();
    printf("HAL + AP_Param initialized.\n\n");

    // Run tests
    int passed = 0;
    int failed = 0;

    // Test 1: Basic I2C communication (verifies hardware layer)
    bool test1 = test_i2c_device_manager();
    printf("[DEBUG] test_i2c_device_manager returned: %s\n", test1 ? "true" : "false");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(50));

    if (test1) passed++; else failed++;

    // Test 2: Full INS init (which calls detect_backends -> HAL_INS_PROBE_LIST)
    // Note: We skip the manual probe test because:
    // 1. Manual probe() doesn't register the backend with ADD_BACKEND
    // 2. A second probe during init() would fail (sensor already configured)
    // The proper ArduPilot way is to let init() handle probing via HAL_INS_PROBE_LIST
    bool test2 = test_ins_init();
    if (test2) passed++; else failed++;

    // Print summary
    printf("\n========================================\n");
    printf("Test Results:\n");
    printf("  I2C Device Manager: %s\n", test1 ? "PASS" : "FAIL");
    printf("  INS Init:           %s\n", test2 ? "PASS" : "FAIL");
    printf("\n  Passed: %d  Failed: %d\n", passed, failed);
    printf("========================================\n\n");

    if (passed == 2) {
        printf("SUCCESS: Full AP_InertialSensor integration working!\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 1);  // Solid LED = success
    } else if (test1) {
        printf("PARTIAL: I2C working, driver integration needs work.\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
    } else {
        printf("FAILED: I2C communication not working.\n");
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
    }

    printf("\n========================================\n");
    printf("  TEST COMPLETE - ALL DONE\n");
    printf("========================================\n");
    fflush(stdout);

    // Idle quietly (no more streaming output)
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    // Initialize USB early for debug output
    stdio_init_all();

    // Create test task
    xTaskCreate(
        test_task,
        "ins_probe_test",
        kTestTaskStackSize,
        nullptr,
        kTestTaskPriority,
        nullptr
    );

    // Start FreeRTOS scheduler
    vTaskStartScheduler();

    // Should never reach here
    for (;;) {}

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (required for static allocation)
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook() {
    printf("FATAL: Malloc failed!\n");
    while (true) {
        tight_loop_contents();
    }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    printf("FATAL: Stack overflow in task: %s\n", name);
    while (true) {
        tight_loop_contents();
    }
}

// Idle task memory (Core 0)
static StaticTask_t idle_task_tcb;
static StackType_t idle_task_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory(StaticTask_t** tcb,
                                    StackType_t** stack,
                                    configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &idle_task_tcb;
    *stack = idle_task_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}

#if (configNUMBER_OF_CORES > 1)
// Passive idle task memory (Core 1)
static StaticTask_t passive_idle_tcb;
static StackType_t passive_idle_stack[configMINIMAL_STACK_SIZE];

void vApplicationGetPassiveIdleTaskMemory(StaticTask_t** tcb,
                                           StackType_t** stack,
                                           configSTACK_DEPTH_TYPE* stack_size,
                                           BaseType_t core) {
    (void)core;
    *tcb = &passive_idle_tcb;
    *stack = passive_idle_stack;
    *stack_size = configMINIMAL_STACK_SIZE;
}
#endif

// Timer task memory
static StaticTask_t timer_task_tcb;
static StackType_t timer_task_stack[configTIMER_TASK_STACK_DEPTH];

void vApplicationGetTimerTaskMemory(StaticTask_t** tcb,
                                     StackType_t** stack,
                                     configSTACK_DEPTH_TYPE* stack_size) {
    *tcb = &timer_task_tcb;
    *stack = timer_task_stack;
    *stack_size = configTIMER_TASK_STACK_DEPTH;
}

}  // extern "C"
