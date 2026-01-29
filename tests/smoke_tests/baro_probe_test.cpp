/**
 * @file baro_probe_test.cpp
 * @brief AP_Baro probe test for DPS310
 *
 * Tests the ArduPilot AP_Baro integration:
 * - AP_HAL I2CDeviceManager
 * - AP_Baro_DPS280 probe (DPS310 uses same driver)
 * - Basic pressure/temperature reading
 *
 * Hardware: Adafruit Feather RP2350 HSTX with DPS310 on I2C1 @ 0x77
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
#include <AP_HAL/I2CDevice.h>

// ArduPilot Baro
#include <AP_Baro/AP_Baro.h>
#include <AP_Baro/AP_Baro_DPS280.h>

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
static constexpr uint8_t kDps310Addr = 0x77;  // Default address
static constexpr uint32_t kTestTaskStackSize = 4096;
static constexpr UBaseType_t kTestTaskPriority = 2;

// DPS310 register addresses
static constexpr uint8_t DPS310_REG_PRODUCT_ID = 0x0D;
static constexpr uint8_t DPS310_PRODUCT_ID = 0x10;  // Expected value

// ============================================================================
// Global Baro instance
// ============================================================================

static AP_Baro g_baro;

// ============================================================================
// Test functions
// ============================================================================

/**
 * @brief Test I2CDeviceManager by reading DPS310 WHO_AM_I
 */
static bool test_i2c_device_manager() {
    printf("Testing I2CDeviceManager with DPS310...\n");
    fflush(stdout);

    printf("  Trying DPS310 at 0x%02X...\n", kDps310Addr);
    auto dev = hal.i2c_mgr->get_device(kI2cBus, kDps310Addr);
    if (!dev) {
        printf("    ERROR: get_device() returned null\n");
        return false;
    }

    printf("    Got device handle\n");

    uint8_t product_id = 0xFF;
    bool read_ok = dev->read_registers(DPS310_REG_PRODUCT_ID, &product_id, 1);
    printf("    read_registers(0x%02X): %s, value=0x%02X (expected 0x%02X)\n",
           DPS310_REG_PRODUCT_ID,
           read_ok ? "OK" : "FAIL",
           product_id,
           DPS310_PRODUCT_ID);

    if (read_ok && product_id == DPS310_PRODUCT_ID) {
        printf("  PASS: DPS310 detected!\n");
        return true;
    }

    // Also try alternate address 0x76
    printf("  Trying DPS310 at 0x76...\n");
    dev = hal.i2c_mgr->get_device(kI2cBus, 0x76);
    if (dev) {
        read_ok = dev->read_registers(DPS310_REG_PRODUCT_ID, &product_id, 1);
        printf("    read_registers(0x%02X): %s, value=0x%02X\n",
               DPS310_REG_PRODUCT_ID,
               read_ok ? "OK" : "FAIL",
               product_id);
        if (read_ok && product_id == DPS310_PRODUCT_ID) {
            printf("  PASS: DPS310 detected at 0x76!\n");
            return true;
        }
    }

    printf("  FAIL: No DPS310 detected\n");
    return false;
}

static bool test_baro_init() {
    printf("\nTesting AP_Baro init...\n");
    printf("  HAL_BARO_PROBE_LIST is defined: %s\n",
#if defined(HAL_BARO_PROBE_LIST)
           "YES"
#else
           "NO"
#endif
    );
    fflush(stdout);

    // Check singleton
    AP_Baro* singleton = AP_Baro::get_singleton();
    printf("  Singleton check: &g_baro=%p, get_singleton()=%p\n",
           (void*)&g_baro, (void*)singleton);
    if (singleton != &g_baro) {
        printf("  WARNING: SINGLETON MISMATCH!\n");
    }
    fflush(stdout);

    // Initialize barometer
    printf("  Calling g_baro.init()...\n");
    fflush(stdout);

    uint32_t start_time = time_us_32();
    g_baro.init();
    uint32_t elapsed_ms = (time_us_32() - start_time) / 1000;

    printf("  init() returned after %lu ms\n", elapsed_ms);
    fflush(stdout);

    uint8_t num_instances = g_baro.num_instances();
    printf("  Barometers detected: %u\n", num_instances);

    if (num_instances == 0) {
        printf("  FAIL: No barometers detected after init\n");
        return false;
    }

    printf("  PASS: Barometer initialized\n");
    return true;
}

static void test_baro_readings() {
    printf("\nReading barometer data...\n");
    printf("Press any key to stop.\n\n");

    // Calibrate the barometer (sets ground level reference)
    printf("Calibrating (setting ground level)...\n");
    g_baro.calibrate(false);  // false = don't save to EEPROM
    printf("Calibration complete.\n\n");

    while (getchar_timeout_us(500000) == PICO_ERROR_TIMEOUT) {
        // Update barometer
        g_baro.update();

        // Get readings
        float pressure = g_baro.get_pressure();
        float temperature = g_baro.get_temperature();
        float altitude = g_baro.get_altitude();
        bool healthy = g_baro.healthy();

        printf("Pressure: %7.1f Pa  Temp: %5.1f C  Alt: %+6.1f m  %s\r",
               (double)pressure,
               (double)temperature,
               (double)altitude,
               healthy ? "OK" : "UNHEALTHY");
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

    // Wait for USB connection
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
    printf("  AP_Baro Probe Test\n");
    printf("  DPS310 via DPS280 driver\n");
    printf("========================================\n\n");

    // Initialize HAL subsystems
    printf("Initializing HAL...\n");
    RP2350::hal_init();

    // Initialize AP_Param system
    printf("Initializing AP_Param...\n");
    AP_Param::setup();
    printf("HAL + AP_Param initialized.\n\n");

    // Run tests
    int passed = 0;
    int failed = 0;

    // Test 1: Basic I2C communication
    if (test_i2c_device_manager()) {
        passed++;
    } else {
        failed++;
    }

    // Test 2: Full baro init
    if (test_baro_init()) {
        passed++;
    } else {
        failed++;
    }

    // Print summary
    printf("\n");
    printf("========================================\n");
    printf("  Results: %d passed, %d failed\n", passed, failed);
    printf("========================================\n\n");

    // If all tests passed, show live readings
    if (failed == 0) {
        test_baro_readings();
    }

    // Indicate done
    printf("Test complete. LED solid = done.\n");
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    // Keep task alive
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// Main entry point
// ============================================================================

int main() {
    // Initialize stdio (USB CDC)
    stdio_init_all();

    // Create test task
    xTaskCreate(
        test_task,
        "baro_test",
        kTestTaskStackSize,
        nullptr,
        kTestTaskPriority,
        nullptr
    );

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
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
