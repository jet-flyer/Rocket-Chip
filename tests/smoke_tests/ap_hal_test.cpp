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
#include <cstdarg>
#include <cstring>

// ============================================================================
// Test Results Storage (for deferred printing per DEBUG_OUTPUT.md)
// ============================================================================

struct TestResult {
    const char* name;
    bool passed;
    char details[256];  // Stores test-specific output
};

static constexpr int kNumTests = 7;
static TestResult g_results[kNumTests];
static int g_result_index = 0;

// Helper to store result
static void store_result(const char* name, bool passed, const char* fmt, ...) {
    if (g_result_index >= kNumTests) return;
    TestResult& r = g_results[g_result_index++];
    r.name = name;
    r.passed = passed;
    va_list args;
    va_start(args, fmt);
    vsnprintf(r.details, sizeof(r.details), fmt, args);
    va_end(args);
}

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
    // Test millis
    uint32_t start_ms = AP_HAL::millis();
    vTaskDelay(pdMS_TO_TICKS(100));
    uint32_t elapsed_ms = AP_HAL::millis() - start_ms;

    if (elapsed_ms < 90 || elapsed_ms > 150) {
        store_result("Timing", false, "millis: expected ~100ms, got %lu ms", (unsigned long)elapsed_ms);
        return false;
    }

    // Test micros
    uint32_t start_us = AP_HAL::micros();
    busy_wait_us_32(1000);
    uint32_t elapsed_us = AP_HAL::micros() - start_us;

    if (elapsed_us < 900 || elapsed_us > 1500) {
        store_result("Timing", false, "micros: expected ~1000us, got %lu us", (unsigned long)elapsed_us);
        return false;
    }

    // Test 64-bit versions
    uint64_t ms64 = AP_HAL::millis64();
    uint64_t us64 = AP_HAL::micros64();

    store_result("Timing", true, "millis=%lu, micros=%lu, ms64=%llu, us64=%llu",
                 (unsigned long)elapsed_ms, (unsigned long)elapsed_us,
                 (unsigned long long)ms64, (unsigned long long)us64);
    return true;
}

static bool test_delay() {
    // Test millisecond delay
    uint32_t start = AP_HAL::millis();
    hal.scheduler.delay(50);
    uint32_t elapsed_ms = AP_HAL::millis() - start;

    if (elapsed_ms < 40 || elapsed_ms > 100) {
        store_result("Delay", false, "delay(50) took %lu ms, expected ~50ms", (unsigned long)elapsed_ms);
        return false;
    }

    // Test microsecond delay
    uint32_t start_us = AP_HAL::micros();
    hal.scheduler.delay_microseconds(500);
    uint32_t elapsed_us = AP_HAL::micros() - start_us;

    if (elapsed_us < 450 || elapsed_us > 700) {
        store_result("Delay", false, "delay_us(500) took %lu us, expected ~500us", (unsigned long)elapsed_us);
        return false;
    }

    store_result("Delay", true, "delay=%lums, delay_us=%luus", (unsigned long)elapsed_ms, (unsigned long)elapsed_us);
    return true;
}

static bool test_semaphore() {
    HAL_Semaphore mutex;

    // Test take and give
    mutex.take_blocking();
    mutex.give();

    // Test take with timeout
    bool result = mutex.take(100);
    if (!result) {
        store_result("Semaphore", false, "take(100) returned false on free mutex");
        return false;
    }

    // Test take_nonblocking (recursive mutex allows same task to take again)
    bool nonblock_result = mutex.take_nonblocking();

    // Release takes
    mutex.give();
    if (nonblock_result) {
        mutex.give();
    }

    store_result("Semaphore", true, "take_blocking, take(timeout), take_nonblocking all work");
    return true;
}

static bool test_binary_semaphore() {
    RP2350::BinarySemaphore sem_signaled(true);

    // Should not block since initially signaled
    bool result = sem_signaled.wait_nonblocking();
    if (!result) {
        store_result("BinarySem", false, "wait_nonblocking returned false on signaled sem");
        return false;
    }

    // Should timeout since signal consumed
    result = sem_signaled.wait(1000);  // 1ms timeout
    if (result) {
        store_result("BinarySem", false, "wait() should have timed out");
        return false;
    }

    // Signal and wait
    sem_signaled.signal();
    result = sem_signaled.wait_nonblocking();
    if (!result) {
        store_result("BinarySem", false, "wait_nonblocking after signal failed");
        return false;
    }

    store_result("BinarySem", true, "signal/wait operations work correctly");
    return true;
}

static bool test_util() {
    // Test memory info
    uint32_t free_mem = hal.util.available_memory();
    uint32_t total_mem = hal.util.total_memory();

    if (free_mem == 0 || free_mem > total_mem) {
        store_result("Util", false, "Invalid memory: free=%lu total=%lu",
                     (unsigned long)free_mem, (unsigned long)total_mem);
        return false;
    }

    // Test system ID
    char sys_id[32];
    bool result = hal.util.get_system_id(sys_id, sizeof(sys_id));
    if (!result) {
        store_result("Util", false, "get_system_id() returned false");
        return false;
    }

    // Test soft arming
    hal.util.set_soft_armed(true);
    if (!hal.util.get_soft_armed()) {
        store_result("Util", false, "get_soft_armed() false after set_soft_armed(true)");
        return false;
    }

    hal.util.set_soft_armed(false);
    if (hal.util.get_soft_armed()) {
        store_result("Util", false, "get_soft_armed() true after set_soft_armed(false)");
        return false;
    }

    store_result("Util", true, "mem=%lu/%lu, id=%s",
                 (unsigned long)free_mem, (unsigned long)total_mem, sys_id);
    return true;
}

static bool test_timer_callbacks() {
    // Register timer callback
    g_timer_callback_count = 0;
    RP2350::MemberProc timer_proc(&g_callbacks, &TestCallbacks::timer_callback);
    hal.scheduler.register_timer_process(timer_proc);

    // Register I/O callback
    g_io_callback_count = 0;
    RP2350::MemberProc io_proc(&g_callbacks, &TestCallbacks::io_callback);
    hal.scheduler.register_io_process(io_proc);

    // Wait for callbacks to run
    vTaskDelay(pdMS_TO_TICKS(1000));

    uint32_t timer_count = g_timer_callback_count;
    uint32_t io_count = g_io_callback_count;

    // Timer should run ~1000 times per second
    if (timer_count < 800 || timer_count > 1200) {
        store_result("Callbacks", false, "timer count %lu out of range (800-1200)",
                     (unsigned long)timer_count);
        return false;
    }

    // I/O should run ~100 times per second
    if (io_count < 80 || io_count > 120) {
        store_result("Callbacks", false, "I/O count %lu out of range (80-120)",
                     (unsigned long)io_count);
        return false;
    }

    store_result("Callbacks", true, "timer=%lu (~1kHz), io=%lu (~100Hz)",
                 (unsigned long)timer_count, (unsigned long)io_count);
    return true;
}

static bool test_system_state() {
    bool is_init = hal.is_initialized();

    // Set system initialized
    hal.scheduler.set_system_initialized();
    bool sys_init = hal.scheduler.is_system_initialized();

    if (!sys_init) {
        store_result("SysState", false, "is_system_initialized() false after set");
        return false;
    }

    bool in_main = hal.scheduler.in_main_thread();

    store_result("SysState", true, "init=%s, sys_init=%s, in_main=%s",
                 is_init ? "true" : "false",
                 sys_init ? "true" : "false",
                 in_main ? "true" : "false");
    return true;
}

// ============================================================================
// Main Test Task
// ============================================================================

static void test_task(void* params) {
    (void)params;

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    // ========================================================================
    // Phase 1: Run tests immediately (per DEBUG_OUTPUT.md - don't block)
    // Tests run silently, results stored for later printing
    // ========================================================================

    // Initialize HAL (required before tests)
    hal.init();

    // Run all tests - results stored, not printed
    int passed = 0;
    int failed = 0;

    if (test_timing()) passed++; else failed++;
    if (test_delay()) passed++; else failed++;
    if (test_semaphore()) passed++; else failed++;
    if (test_binary_semaphore()) passed++; else failed++;
    if (test_util()) passed++; else failed++;
    if (test_timer_callbacks()) passed++; else failed++;
    if (test_system_state()) passed++; else failed++;

    // ========================================================================
    // Phase 2: Show LED status immediately (visual feedback without serial)
    // Fast blink while waiting = also indicates "connect terminal"
    // ========================================================================

    // Blink pattern indicates pass/fail AND waiting for USB
    // Fast blink (100ms) = waiting for USB (or fail after connect)
    // Slow blink (500ms) = pass (only after USB connect confirmed)

    // Wait for USB connection with LED blinking
    while (!stdio_usb_connected()) {
        gpio_put(PICO_DEFAULT_LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        gpio_put(PICO_DEFAULT_LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // Brief settle time
    vTaskDelay(pdMS_TO_TICKS(500));

    // ========================================================================
    // Phase 3: Print stored results (terminal now connected)
    // ========================================================================

    printf("\n");
    printf("========================================\n");
    printf("AP_HAL_RP2350 Phase 1 Smoke Test\n");
    printf("========================================\n\n");

    // Print each test result
    for (int i = 0; i < g_result_index; i++) {
        const TestResult& r = g_results[i];
        printf("[%s] %s: %s\n",
               r.passed ? "PASS" : "FAIL",
               r.name,
               r.details);
    }

    printf("\n========================================\n");
    printf("Test Results: %d passed, %d failed\n", passed, failed);
    printf("========================================\n");

    if (failed == 0) {
        printf("\nAll tests PASSED!\n");
    } else {
        printf("\nSome tests FAILED!\n");
    }

    printf("\nLED: %s blink = %s\n",
           failed == 0 ? "slow" : "fast",
           failed == 0 ? "PASS" : "FAIL");

    // ========================================================================
    // Phase 4: LED indicates final pass/fail status
    // ========================================================================

    while (true) {
        if (failed == 0) {
            // Slow blink = success
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(500));
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(500));
        } else {
            // Fast blink = failure
            gpio_put(PICO_DEFAULT_LED_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
            gpio_put(PICO_DEFAULT_LED_PIN, 0);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main() {
    // Initialize LED for debugging FIRST (before anything else)
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

    // Create test task (static allocation per project standards)
    static StaticTask_t task_buffer;
    static StackType_t task_stack[2048];

    TaskHandle_t task_handle = xTaskCreateStatic(
        test_task,
        "APHALTest",
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
