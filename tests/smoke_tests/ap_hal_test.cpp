/**
 * @file ap_hal_test.cpp
 * @brief Smoke test for AP_HAL_RP2350 Phase 1 components
 *
 * Tests:
 * - HAL initialization
 * - Scheduler timing functions (millis, micros)
 * - Scheduler delays
 * - Semaphore operations (take, give, timeout)
 * - Util functions (memory info, system ID)
 * - Timer callbacks
 *
 * @note Part of AP_HAL_RP2350 validation
 */

#include "AP_HAL_RP2350/AP_HAL_RP2350.h"

#include "FreeRTOS.h"
#include "task.h"

#include "pico/stdlib.h"

#include <cstdio>
#include <cstring>

// ============================================================================
// Test Counters
// ============================================================================

static uint32_t g_timer_callback_count = 0;
static uint32_t g_io_callback_count = 0;
static volatile bool g_test_running = true;

// ============================================================================
// Test Callbacks
// ============================================================================

class TestCallbacks {
public:
    void timer_callback() {
        g_timer_callback_count++;
    }

    void io_callback() {
        g_io_callback_count++;
    }
};

static TestCallbacks g_callbacks;

// ============================================================================
// Test Functions
// ============================================================================

static bool test_timing() {
    printf("\n--- Test: Timing Functions ---\n");

    // Test millis
    uint32_t start_ms = AP_HAL::millis();
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t elapsed_ms = AP_HAL::millis() - start_ms;

    printf("  millis() after 100ms delay: %lu ms\n", (unsigned long)elapsed_ms);
    if (elapsed_ms < 90 || elapsed_ms > 150) {
        printf("  FAIL: Expected ~100ms, got %lu ms\n", (unsigned long)elapsed_ms);
        return false;
    }

    // Test micros
    uint32_t start_us = AP_HAL::micros();
    busy_wait_us_32(1000);  // 1ms busy wait
    uint32_t elapsed_us = AP_HAL::micros() - start_us;

    printf("  micros() after 1000us delay: %lu us\n", (unsigned long)elapsed_us);
    if (elapsed_us < 900 || elapsed_us > 1500) {
        printf("  FAIL: Expected ~1000us, got %lu us\n", (unsigned long)elapsed_us);
        return false;
    }

    // Test 64-bit versions
    uint64_t ms64 = AP_HAL::millis64();
    uint64_t us64 = AP_HAL::micros64();
    printf("  millis64(): %llu ms\n", (unsigned long long)ms64);
    printf("  micros64(): %llu us\n", (unsigned long long)us64);

    printf("  PASS: Timing functions working\n");
    return true;
}

static bool test_delay() {
    printf("\n--- Test: Scheduler Delays ---\n");

    // Test millisecond delay
    uint32_t start = AP_HAL::millis();
    hal.scheduler.delay(50);
    uint32_t elapsed = AP_HAL::millis() - start;

    printf("  delay(50) took: %lu ms\n", (unsigned long)elapsed);
    if (elapsed < 40 || elapsed > 100) {
        printf("  FAIL: Expected ~50ms, got %lu ms\n", (unsigned long)elapsed);
        return false;
    }

    // Test microsecond delay
    start = AP_HAL::micros();
    hal.scheduler.delay_microseconds(500);
    elapsed = AP_HAL::micros() - start;

    printf("  delay_microseconds(500) took: %lu us\n", (unsigned long)elapsed);
    if (elapsed < 450 || elapsed > 700) {
        printf("  FAIL: Expected ~500us, got %lu us\n", (unsigned long)elapsed);
        return false;
    }

    printf("  PASS: Delays working\n");
    return true;
}

static bool test_semaphore() {
    printf("\n--- Test: Semaphores ---\n");

    // Create a semaphore
    HAL_Semaphore mutex;

    // Test take and give
    printf("  Testing take_blocking + give...\n");
    mutex.take_blocking();
    printf("    Acquired mutex\n");
    mutex.give();
    printf("    Released mutex\n");

    // Test take with timeout (should succeed immediately)
    printf("  Testing take(100)...\n");
    bool result = mutex.take(100);
    if (!result) {
        printf("  FAIL: take(100) returned false on free mutex\n");
        return false;
    }
    printf("    Acquired mutex with timeout\n");

    // Test take_nonblocking (should fail since we hold it)
    printf("  Testing take_nonblocking() while held...\n");
    // Note: FreeRTOS recursive mutex allows same task to take multiple times
    result = mutex.take_nonblocking();
    printf("    take_nonblocking returned: %s (recursive mutex allows this)\n",
           result ? "true" : "false");

    // Release both takes
    mutex.give();
    if (result) {
        mutex.give();
    }

    printf("  PASS: Semaphore operations working\n");
    return true;
}

static bool test_binary_semaphore() {
    printf("\n--- Test: Binary Semaphores ---\n");

    // Create signaled
    RP2350::BinarySemaphore sem_signaled(true);

    // Should not block since initially signaled
    printf("  Testing wait_nonblocking() on signaled sem...\n");
    bool result = sem_signaled.wait_nonblocking();
    if (!result) {
        printf("  FAIL: wait_nonblocking() returned false on signaled semaphore\n");
        return false;
    }
    printf("    Got signal\n");

    // Now should block/timeout since signal consumed
    printf("  Testing wait(1000) after signal consumed...\n");
    result = sem_signaled.wait(1000);  // 1ms timeout
    if (result) {
        printf("  FAIL: wait() should have timed out\n");
        return false;
    }
    printf("    Correctly timed out\n");

    // Signal and wait
    printf("  Testing signal() then wait_nonblocking()...\n");
    sem_signaled.signal();
    result = sem_signaled.wait_nonblocking();
    if (!result) {
        printf("  FAIL: wait_nonblocking() after signal() failed\n");
        return false;
    }
    printf("    Signal received\n");

    printf("  PASS: Binary semaphore working\n");
    return true;
}

static bool test_util() {
    printf("\n--- Test: Util Functions ---\n");

    // Test memory info
    uint32_t free_mem = hal.util.available_memory();
    uint32_t total_mem = hal.util.total_memory();
    printf("  Available memory: %lu / %lu bytes\n",
           (unsigned long)free_mem, (unsigned long)total_mem);

    if (free_mem == 0 || free_mem > total_mem) {
        printf("  FAIL: Invalid memory values\n");
        return false;
    }

    // Test memory info string
    char mem_info[64];
    hal.util.mem_info(mem_info, sizeof(mem_info));
    printf("  Memory info: %s\n", mem_info);

    // Test system ID
    char sys_id[32];
    bool result = hal.util.get_system_id(sys_id, sizeof(sys_id));
    if (!result) {
        printf("  FAIL: get_system_id() returned false\n");
        return false;
    }
    printf("  System ID: %s\n", sys_id);

    // Test raw system ID
    uint8_t raw_id[8];
    uint8_t len = hal.util.get_system_id_unformatted(raw_id, sizeof(raw_id));
    printf("  Raw ID (%u bytes): ", len);
    for (uint8_t i = 0; i < len; i++) {
        printf("%02X", raw_id[i]);
    }
    printf("\n");

    // Test safety state (should be SAFETY_NONE on RocketChip)
    auto safety = hal.util.safety_switch_state();
    printf("  Safety state: %d (SAFETY_NONE=%d)\n",
           static_cast<int>(safety),
           static_cast<int>(RP2350::SafetyState::SAFETY_NONE));

    // Test soft arming
    printf("  Testing soft arm...\n");
    hal.util.set_soft_armed(true);
    if (!hal.util.get_soft_armed()) {
        printf("  FAIL: get_soft_armed() returned false after set_soft_armed(true)\n");
        return false;
    }
    printf("    Armed: %s\n", hal.util.get_soft_armed() ? "true" : "false");

    hal.util.set_soft_armed(false);
    if (hal.util.get_soft_armed()) {
        printf("  FAIL: get_soft_armed() returned true after set_soft_armed(false)\n");
        return false;
    }
    printf("    Armed: %s\n", hal.util.get_soft_armed() ? "true" : "false");

    printf("  PASS: Util functions working\n");
    return true;
}

static bool test_timer_callbacks() {
    printf("\n--- Test: Timer Callbacks ---\n");

    // Register timer callback
    g_timer_callback_count = 0;
    RP2350::MemberProc timer_proc(&g_callbacks, &TestCallbacks::timer_callback);
    hal.scheduler.register_timer_process(timer_proc);

    // Register I/O callback
    g_io_callback_count = 0;
    RP2350::MemberProc io_proc(&g_callbacks, &TestCallbacks::io_callback);
    hal.scheduler.register_io_process(io_proc);

    printf("  Registered timer and I/O callbacks\n");
    printf("  Waiting 1 second for callbacks to run...\n");

    // Wait for callbacks to run
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("  Timer callback count: %lu (expected ~1000 at 1kHz)\n",
           (unsigned long)g_timer_callback_count);
    printf("  I/O callback count: %lu (expected ~100 at 100Hz)\n",
           (unsigned long)g_io_callback_count);

    // Timer should run ~1000 times per second
    if (g_timer_callback_count < 800 || g_timer_callback_count > 1200) {
        printf("  FAIL: Timer callback count out of range\n");
        return false;
    }

    // I/O should run ~100 times per second
    if (g_io_callback_count < 80 || g_io_callback_count > 120) {
        printf("  FAIL: I/O callback count out of range\n");
        return false;
    }

    printf("  PASS: Timer callbacks working\n");
    return true;
}

static bool test_system_state() {
    printf("\n--- Test: System State ---\n");

    // Check initialization
    printf("  is_initialized(): %s\n",
           hal.is_initialized() ? "true" : "false");

    // Check system_initialized flag
    printf("  is_system_initialized(): %s\n",
           hal.scheduler.is_system_initialized() ? "true" : "false");

    // Set system initialized
    hal.scheduler.set_system_initialized();
    printf("  After set_system_initialized(): %s\n",
           hal.scheduler.is_system_initialized() ? "true" : "false");

    if (!hal.scheduler.is_system_initialized()) {
        printf("  FAIL: is_system_initialized() returned false after set\n");
        return false;
    }

    // Check main thread
    printf("  in_main_thread(): %s\n",
           hal.scheduler.in_main_thread() ? "true" : "false");

    printf("  PASS: System state working\n");
    return true;
}

// ============================================================================
// Main Test Task
// ============================================================================

static void test_task(void* params) {
    (void)params;

    printf("\n");
    printf("========================================\n");
    printf("AP_HAL_RP2350 Phase 1 Smoke Test\n");
    printf("========================================\n");

    // Initialize HAL
    printf("\nInitializing HAL...\n");
    hal.init();

    // Run tests
    int passed = 0;
    int failed = 0;

    if (test_timing()) passed++; else failed++;
    if (test_delay()) passed++; else failed++;
    if (test_semaphore()) passed++; else failed++;
    if (test_binary_semaphore()) passed++; else failed++;
    if (test_util()) passed++; else failed++;
    if (test_timer_callbacks()) passed++; else failed++;
    if (test_system_state()) passed++; else failed++;

    // Summary
    printf("\n========================================\n");
    printf("Test Results: %d passed, %d failed\n", passed, failed);
    printf("========================================\n");

    if (failed == 0) {
        printf("\nAll tests PASSED!\n");
        printf("AP_HAL_RP2350 Phase 1 components verified.\n");
    } else {
        printf("\nSome tests FAILED!\n");
        printf("Check output above for details.\n");
    }

    // Keep running to allow observation
    printf("\nTest complete. LED will blink to indicate status.\n");

    gpio_init(13);
    gpio_set_dir(13, GPIO_OUT);

    while (true) {
        if (failed == 0) {
            // Slow blink = success
            gpio_put(13, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(13, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // Fast blink = failure
            gpio_put(13, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(13, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main() {
    // Initialize stdio for USB output
    stdio_init_all();

    // Wait for USB connection
    sleep_ms(2000);

    // Create test task
    static StaticTask_t task_buffer;
    static StackType_t task_stack[2048];

    xTaskCreateStatic(
        test_task,
        "APHALTest",
        sizeof(task_stack) / sizeof(StackType_t),
        nullptr,
        tskIDLE_PRIORITY + 1,
        task_stack,
        &task_buffer
    );

    // Start scheduler
    vTaskStartScheduler();

    // Should never reach here
    while (true) {
        tight_loop_contents();
    }

    return 0;
}

// ============================================================================
// FreeRTOS Hooks (Required)
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
    (void)core;  // Same memory for any passive core
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
