/**
 * @file storage_test.cpp
 * @brief Minimal smoke test for AP_HAL_RP2350 Storage
 *
 * Stripped down to match smoke_ap_hal as closely as possible.
 * Tests only basic Storage operations - no CalibrationStore.
 *
 * Hardware: Adafruit Feather RP2350 HSTX
 */

// Match smoke_ap_hal includes exactly
#include "AP_HAL_RP2350/AP_HAL_RP2350.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

// ============================================================================
// Test Results Storage (matching smoke_ap_hal pattern)
// ============================================================================

struct TestResult {
    const char* name;
    bool passed;
    char details[128];
};

static constexpr int kNumTests = 2;
static TestResult g_results[kNumTests];
static int g_result_index = 0;

static void store_result(const char* name, bool passed, const char* msg) {
    if (g_result_index >= kNumTests) return;
    TestResult& r = g_results[g_result_index++];
    r.name = name;
    r.passed = passed;
    snprintf(r.details, sizeof(r.details), "%s", msg);
}

// ============================================================================
// Tests
// ============================================================================

static bool test_storage_health() {
    bool healthy = hal.storage.healthy();
    store_result("Storage Health", healthy,
                 healthy ? "Storage subsystem healthy" : "Storage NOT healthy");
    return healthy;
}

static bool test_storage_read_write() {
    // Simple read/write test
    uint8_t test_data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    uint8_t read_buf[4] = {0};

    hal.storage.write_block(0, test_data, sizeof(test_data));
    hal.storage.read_block(read_buf, 0, sizeof(read_buf));

    bool match = (memcmp(test_data, read_buf, sizeof(test_data)) == 0);
    store_result("Storage R/W", match,
                 match ? "Read matches write" : "Data mismatch!");
    return match;
}

// ============================================================================
// Main Test Task
// ============================================================================

static void test_task(void* params) {
    (void)params;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Initialize HAL (required before tests)
    hal.init();

    // Run tests - results stored, not printed
    int passed = 0;
    int failed = 0;

    if (test_storage_health()) passed++; else failed++;
    if (test_storage_read_write()) passed++; else failed++;

    // Wait for USB connection with LED blinking
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Brief settle time
    vTaskDelay(pdMS_TO_TICKS(500));

    // Print results
    printf("\n");
    printf("========================================\n");
    printf("Storage Smoke Test (Minimal)\n");
    printf("========================================\n\n");

    for (int i = 0; i < g_result_index; i++) {
        const TestResult& r = g_results[i];
        printf("[%s] %s: %s\n",
               r.passed ? "PASS" : "FAIL",
               r.name,
               r.details);
    }

    printf("\n========================================\n");
    printf("Results: %d passed, %d failed\n", passed, failed);
    printf("========================================\n");

    // LED indicates final status
    while (true) {
        if (failed == 0) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================================
// Main Entry Point (matching smoke_ap_hal exactly)
// ============================================================================

int main() {
    // Initialize LED for debugging FIRST
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // Blink 3 times rapidly = reached main()
    for (int i = 0; i < 3; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        busy_wait_ms(100);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        busy_wait_ms(100);
    }

    // Initialize stdio for USB output
    stdio_init_all();

    // One slow blink = stdio initialized, about to create task
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(250);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    sleep_ms(250);

    // Create test task (static allocation)
    static StaticTask_t task_buffer;
    static StackType_t task_stack[2048];

    TaskHandle_t task_handle = xTaskCreateStatic(
        test_task,
        "StorageTest",
        sizeof(task_stack) / sizeof(StackType_t),
        nullptr,
        tskIDLE_PRIORITY + 1,
        task_stack,
        &task_buffer
    );

    // Two quick blinks = task created
    if (task_handle != nullptr) {
        for (int i = 0; i < 2; i++) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            sleep_ms(100);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            sleep_ms(100);
        }
    } else {
        // Rapid blink = task creation failed
        while (true) {
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            busy_wait_ms(50);
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            busy_wait_ms(50);
        }
    }

    // Two long blinks = about to start scheduler
    for (int i = 0; i < 2; i++) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        sleep_ms(500);
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        sleep_ms(500);
    }

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (matching smoke_ap_hal exactly)
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
