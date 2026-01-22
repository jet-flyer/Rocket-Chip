/**
 * @file Scheduler.cpp
 * @brief AP_HAL Scheduler implementation using FreeRTOS
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#include "Scheduler.h"

#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/timer.h"
#include "hardware/watchdog.h"
#include "hardware/sync.h"

#include <cstdio>
#include <cstdarg>

namespace RP2350 {

// ============================================================================
// Constructor / Destructor
// ============================================================================

Scheduler::Scheduler()
    : m_num_timer_procs(0)
    , m_num_io_procs(0)
    , m_failsafe_proc(nullptr)
    , m_failsafe_period_us(0)
    , m_failsafe_last_call_us(0)
    , m_delay_callback(nullptr)
    , m_delay_callback_min_ms(0)
    , m_in_delay_callback(false)
    , m_initialized(false)
    , m_system_initialized(false)
    , m_expected_delay_end_ms(0)
    , m_main_task(nullptr)
    , m_timer_task(nullptr)
    , m_io_task(nullptr)
    , m_priority_boosted(false)
    , m_saved_priority(0)
{
    // Zero out callback arrays
    for (uint8_t i = 0; i < kMaxTimerProcs; i++) {
        m_timer_procs[i] = MemberProc();
    }
    for (uint8_t i = 0; i < kMaxIOProcs; i++) {
        m_io_procs[i] = MemberProc();
    }
}

Scheduler::~Scheduler() {
    // Tasks use static allocation, no cleanup needed
}

// ============================================================================
// Initialization
// ============================================================================

void Scheduler::init() {
    if (m_initialized) {
        return;
    }

    // Record main task handle (assumed to be the task calling init)
    m_main_task = xTaskGetCurrentTaskHandle();

    // Create timer task (high priority, runs callbacks at 1kHz)
    m_timer_task = xTaskCreateStatic(
        timer_task_entry,
        "AP_Timer",
        HAL_TIMER_STACK_SIZE,
        this,
        HAL_PRIORITY_TIMER,
        m_timer_stack,
        &m_timer_task_buffer
    );
    configASSERT(m_timer_task != nullptr);

    // Create I/O task (lower priority, for I/O callbacks)
    m_io_task = xTaskCreateStatic(
        io_task_entry,
        "AP_IO",
        HAL_IO_STACK_SIZE,
        this,
        HAL_PRIORITY_IO,
        m_io_stack,
        &m_io_task_buffer
    );
    configASSERT(m_io_task != nullptr);

    m_initialized = true;
}

// ============================================================================
// Timing Functions (Static)
// ============================================================================

uint32_t Scheduler::millis() {
    return static_cast<uint32_t>(time_us_64() / 1000ULL);
}

uint64_t Scheduler::millis64() {
    return time_us_64() / 1000ULL;
}

uint32_t Scheduler::micros() {
    return time_us_32();
}

uint64_t Scheduler::micros64() {
    return time_us_64();
}

// ============================================================================
// Delay Functions
// ============================================================================

void Scheduler::delay(uint16_t ms) {
    if (ms == 0) {
        return;
    }

    // If we have a delay callback and delay exceeds threshold, call it
    if (m_delay_callback != nullptr && ms >= m_delay_callback_min_ms) {
        uint32_t start = millis();
        while (millis() - start < ms) {
            call_delay_cb();

            // Yield between callback calls
            uint32_t remaining = ms - (millis() - start);
            if (remaining > 0) {
                uint32_t delay_chunk = (remaining > 10) ? 10 : remaining;
                vTaskDelay(pdMS_TO_TICKS(delay_chunk));
            }
        }
    } else {
        // Simple RTOS delay
        if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
            vTaskDelay(pdMS_TO_TICKS(ms));
        } else {
            // Scheduler not running, busy wait
            busy_wait_us_32(ms * 1000);
        }
    }
}

void Scheduler::delay_microseconds(uint16_t us) {
    // Use hardware timer for accurate microsecond delays
    busy_wait_us_32(us);
}

void Scheduler::delay_microseconds_boost(uint16_t us) {
    // Boost priority temporarily
    if (!m_priority_boosted && xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        m_saved_priority = uxTaskPriorityGet(nullptr);
        vTaskPrioritySet(nullptr, static_cast<UBaseType_t>(Priority::PRIORITY_BOOST));
        m_priority_boosted = true;
    }

    delay_microseconds(us);
}

void Scheduler::boost_end() {
    if (m_priority_boosted) {
        vTaskPrioritySet(nullptr, m_saved_priority);
        m_priority_boosted = false;
    }
}

// ============================================================================
// Callback Registration
// ============================================================================

void Scheduler::register_timer_process(MemberProc proc) {
    if (!proc.is_valid()) {
        return;
    }

    // Take semaphore to protect array modification
    m_timer_sem.take_blocking();

    if (m_num_timer_procs < kMaxTimerProcs) {
        m_timer_procs[m_num_timer_procs++] = proc;
    }

    m_timer_sem.give();
}

void Scheduler::register_io_process(MemberProc proc) {
    if (!proc.is_valid()) {
        return;
    }

    m_io_sem.take_blocking();

    if (m_num_io_procs < kMaxIOProcs) {
        m_io_procs[m_num_io_procs++] = proc;
    }

    m_io_sem.give();
}

void Scheduler::register_timer_failsafe(Proc failsafe, uint32_t period_us) {
    m_failsafe_proc = failsafe;
    m_failsafe_period_us = period_us;
    m_failsafe_last_call_us = micros();
}

void Scheduler::register_delay_callback(Proc proc, uint16_t min_time_ms) {
    m_delay_callback = proc;
    m_delay_callback_min_ms = min_time_ms;
}

// ============================================================================
// System State
// ============================================================================

void Scheduler::set_system_initialized() {
    m_system_initialized = true;
}

bool Scheduler::is_system_initialized() const {
    return m_system_initialized;
}

bool Scheduler::in_main_thread() const {
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {
        // Before scheduler starts, we're in "main thread"
        return true;
    }
    return xTaskGetCurrentTaskHandle() == m_main_task;
}

bool Scheduler::in_delay_callback() const {
    return m_in_delay_callback;
}

// ============================================================================
// Expected Delay
// ============================================================================

void Scheduler::expect_delay_ms(uint32_t ms) {
    m_expected_delay_end_ms = millis() + ms;
}

bool Scheduler::in_expected_delay() const {
    if (m_expected_delay_end_ms == 0) {
        return false;
    }
    return millis() < m_expected_delay_end_ms;
}

// ============================================================================
// Thread Creation
// ============================================================================

bool Scheduler::thread_create(MemberProc proc, const char* name,
                               uint32_t stack_size, Priority base, int8_t priority) {
    (void)proc;
    (void)name;
    (void)stack_size;
    (void)base;
    (void)priority;

    // Dynamic thread creation requires heap allocation.
    // For now, we only support statically allocated tasks.
    // TODO: Implement if needed using static task pools
    return false;
}

// ============================================================================
// System Control
// ============================================================================

[[noreturn]] void Scheduler::reboot(bool hold_in_bootloader) {
    if (hold_in_bootloader) {
        // Enter USB bootloader mode (UF2)
        reset_usb_boot(0, 0);
    } else {
        // Normal reboot via watchdog
        watchdog_reboot(0, 0, 0);
    }

    // Should never reach here
    while (true) {
        __wfi();
    }
}

// ============================================================================
// Interrupt Control
// ============================================================================

void* Scheduler::disable_interrupts_save() {
    return reinterpret_cast<void*>(save_and_disable_interrupts());
}

void Scheduler::restore_interrupts(void* state) {
    restore_interrupts_from_disabled(reinterpret_cast<uint32_t>(state));
}

// ============================================================================
// Internal - Task Entry Points
// ============================================================================

void Scheduler::timer_task_entry(void* param) {
    Scheduler* scheduler = static_cast<Scheduler*>(param);

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / HAL_TIMER_RATE_HZ);

    while (true) {
        // Wait for next period
        vTaskDelayUntil(&last_wake, period);

        // Run timer callbacks
        scheduler->run_timer_procs();
    }
}

void Scheduler::io_task_entry(void* param) {
    Scheduler* scheduler = static_cast<Scheduler*>(param);

    while (true) {
        // Run I/O callbacks at lower rate
        scheduler->run_io_procs();

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz I/O rate
    }
}

// ============================================================================
// Internal - Callback Execution
// ============================================================================

void Scheduler::run_timer_procs() {
    // Run registered timer processes
    m_timer_sem.take_blocking();
    uint8_t n = m_num_timer_procs;
    m_timer_sem.give();

    for (uint8_t i = 0; i < n; i++) {
        m_timer_procs[i].call();
    }

    // Run failsafe if registered and period elapsed
    if (m_failsafe_proc != nullptr && m_failsafe_period_us > 0) {
        uint32_t now = micros();
        if (now - m_failsafe_last_call_us >= m_failsafe_period_us) {
            m_failsafe_last_call_us = now;
            m_failsafe_proc();
        }
    }
}

void Scheduler::run_io_procs() {
    m_io_sem.take_blocking();
    uint8_t n = m_num_io_procs;
    m_io_sem.give();

    for (uint8_t i = 0; i < n; i++) {
        m_io_procs[i].call();
    }
}

void Scheduler::call_delay_cb() {
    if (m_delay_callback == nullptr || m_in_delay_callback) {
        return;
    }

    m_in_delay_callback = true;
    m_delay_callback();
    m_in_delay_callback = false;
}

}  // namespace RP2350

// ============================================================================
// AP_HAL Namespace - Panic Handler
// ============================================================================

namespace AP_HAL {

[[noreturn]] void panic(const char* fmt, ...) {
    // Disable interrupts to prevent further issues
    uint32_t saved = save_and_disable_interrupts();
    (void)saved;  // We won't restore

    // Print panic message
    printf("\n\n*** AP_HAL PANIC ***\n");
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    printf("\n");

    // Blink LED rapidly to indicate panic
    gpio_init(13);  // LED_BUILTIN
    gpio_set_dir(13, GPIO_OUT);

    while (true) {
        gpio_put(13, 1);
        busy_wait_us_32(100000);  // 100ms
        gpio_put(13, 0);
        busy_wait_us_32(100000);
    }
}

}  // namespace AP_HAL
