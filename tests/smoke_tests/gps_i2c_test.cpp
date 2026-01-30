/**
 * @file gps_i2c_test.cpp
 * @brief PA1010D GPS I2C communication test
 *
 * Tests basic I2C communication with the PA1010D GPS module.
 * The PA1010D outputs NMEA sentences over I2C when data is available.
 * Returns 0xFF when no data available.
 *
 * Hardware: Adafruit PA1010D on Qwiic @ 0x10
 *
 * Note: ArduPilot's AP_GPS uses UART, not I2C. For I2C GPS we'll need
 * a custom driver that reads NMEA from I2C and feeds it to the GPS parser.
 */

#include <cstdio>
#include <cstring>

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// ArduPilot HAL (for I2CDeviceManager)
#include <AP_HAL_RP2350/HAL_RP2350_Class.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>

// HAL reference
extern const AP_HAL::HAL& hal;

// ============================================================================
// Configuration
// ============================================================================

static constexpr uint32_t kUsbWaitBlinkMs = 100;
static constexpr uint32_t kUsbSettleTimeMs = 500;
static constexpr uint32_t kI2cBus = 0;          // Bus 0 = Qwiic on Feather
static constexpr uint8_t kPa1010dAddr = 0x10;   // PA1010D default address
static constexpr uint32_t kTestTaskStackSize = 4096;
static constexpr UBaseType_t kTestTaskPriority = 2;

// PA1010D returns 0xFF (or sometimes 0x0A) when no data available
static constexpr uint8_t kNoDataByte = 0xFF;

// ============================================================================
// Test functions
// ============================================================================

/**
 * @brief Test basic I2C communication with PA1010D
 */
static bool test_i2c_communication() {
    printf("Testing I2C communication with PA1010D at 0x%02X...\n", kPa1010dAddr);
    fflush(stdout);

    auto dev = hal.i2c_mgr->get_device(kI2cBus, kPa1010dAddr);
    if (!dev) {
        printf("  ERROR: get_device() returned null\n");
        return false;
    }

    printf("  Got device handle\n");

    // Try to read some bytes - PA1010D returns NMEA data or 0xFF
    uint8_t buffer[32];
    memset(buffer, 0, sizeof(buffer));

    bool read_ok = dev->read(buffer, sizeof(buffer));
    printf("  read(%zu bytes): %s\n", sizeof(buffer), read_ok ? "OK" : "FAIL");

    if (read_ok) {
        // Check if we got any real data (not all 0xFF)
        bool has_data = false;
        for (size_t i = 0; i < sizeof(buffer); i++) {
            if (buffer[i] != kNoDataByte && buffer[i] != 0x00) {
                has_data = true;
                break;
            }
        }

        if (has_data) {
            printf("  Data received: ");
            for (size_t i = 0; i < 16; i++) {
                if (buffer[i] >= 0x20 && buffer[i] < 0x7F) {
                    printf("%c", buffer[i]);
                } else {
                    printf(".");
                }
            }
            printf("...\n");
        } else {
            printf("  No NMEA data yet (all 0xFF) - GPS may need time to start\n");
        }

        printf("  PASS: PA1010D responding on I2C\n");
        return true;
    }

    printf("  FAIL: Could not read from PA1010D\n");
    return false;
}

/**
 * @brief Read and display NMEA sentences from GPS
 */
static void read_nmea_stream() {
    printf("\nReading NMEA stream from PA1010D...\n");
    printf("Press any key to stop.\n\n");

    auto dev = hal.i2c_mgr->get_device(kI2cBus, kPa1010dAddr);
    if (!dev) {
        printf("ERROR: Could not get device\n");
        return;
    }

    // Buffer to accumulate NMEA sentence
    char nmea_buffer[256];
    size_t nmea_pos = 0;

    while (getchar_timeout_us(100000) == PICO_ERROR_TIMEOUT) {
        // Read a chunk of data
        uint8_t chunk[64];
        if (!dev->read(chunk, sizeof(chunk))) {
            continue;
        }

        // Process each byte
        for (size_t i = 0; i < sizeof(chunk); i++) {
            uint8_t c = chunk[i];

            // Skip filler bytes
            if (c == 0xFF || c == 0x00) {
                continue;
            }

            // Start of new sentence
            if (c == '$') {
                nmea_pos = 0;
            }

            // Add to buffer
            if (nmea_pos < sizeof(nmea_buffer) - 1) {
                nmea_buffer[nmea_pos++] = (char)c;
            }

            // End of sentence
            if (c == '\n') {
                nmea_buffer[nmea_pos] = '\0';

                // Print the sentence (filter for common types)
                if (strncmp(nmea_buffer, "$GPRMC", 6) == 0 ||
                    strncmp(nmea_buffer, "$GPGGA", 6) == 0 ||
                    strncmp(nmea_buffer, "$GNRMC", 6) == 0 ||
                    strncmp(nmea_buffer, "$GNGGA", 6) == 0) {
                    printf("%s", nmea_buffer);
                    fflush(stdout);
                }

                nmea_pos = 0;
            }
        }
    }
    printf("\n");
}

// ============================================================================
// Test Task
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
    printf("  PA1010D GPS I2C Test\n");
    printf("  Address: 0x%02X (Qwiic)\n", kPa1010dAddr);
    printf("========================================\n\n");

    // Initialize HAL
    printf("Initializing HAL...\n");
    RP2350::hal_init();
    printf("HAL initialized.\n\n");

    // Run tests
    int passed = 0;
    int failed = 0;

    if (test_i2c_communication()) {
        passed++;
    } else {
        failed++;
    }

    // Print summary
    printf("\n");
    printf("========================================\n");
    printf("  Results: %d passed, %d failed\n", passed, failed);
    printf("========================================\n\n");

    // If I2C works, show NMEA stream
    if (failed == 0) {
        read_nmea_stream();
    }

    printf("Test complete.\n");
    gpio_put(PICO_DEFAULT_LED_PIN, 1);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    stdio_init_all();

    xTaskCreate(
        test_task,
        "gps_test",
        kTestTaskStackSize,
        nullptr,
        kTestTaskPriority,
        nullptr
    );

    vTaskStartScheduler();
    return 0;
}

// ============================================================================
// FreeRTOS Hooks
// ============================================================================

extern "C" {

void vApplicationMallocFailedHook() {
    printf("FATAL: Malloc failed!\n");
    while (true) { tight_loop_contents(); }
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    printf("FATAL: Stack overflow in task: %s\n", name);
    while (true) { tight_loop_contents(); }
}

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
