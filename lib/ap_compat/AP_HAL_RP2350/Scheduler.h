/**
 * @file Scheduler.h
 * @brief AP_HAL Scheduler implementation using FreeRTOS
 *
 * Provides timing functions, delays, and callback registration.
 * Maps ArduPilot's scheduler model to FreeRTOS tasks and timers.
 *
 * @note Part of AP_HAL_RP2350 - ArduPilot HAL for RocketChip
 */

#pragma once

// Mark that full timing implementation is provided by this file
// This prevents stub definitions in AP_HAL/AP_HAL.h
#define AP_HAL_TIMING_DEFINED 1

#include "hwdef.h"
#include "Semaphores.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include <cstdint>

// AP_HAL base class for inheritance
#include <AP_HAL/Scheduler.h>

namespace RP2350 {

// Use AP_HAL types for callbacks (required for AP_HAL::Scheduler inheritance)
using Proc = AP_HAL::Proc;
using MemberProc = AP_HAL::MemberProc;


/**
 * @brief Thread priority levels
 *
 * Maps to FreeRTOS priorities. Higher values = higher priority.
 */
enum class Priority : uint8_t {
    PRIORITY_NET       = 0,   // Lowest - networking
    PRIORITY_SCRIPTING = 0,
    PRIORITY_STORAGE   = 1,
    PRIORITY_UART      = 2,
    PRIORITY_IO        = 2,
    PRIORITY_RCIN      = 3,
    PRIORITY_LED       = 3,
    PRIORITY_RCOUT     = 4,
    PRIORITY_TIMER     = 5,
    PRIORITY_CAN       = 5,
    PRIORITY_I2C       = 5,
    PRIORITY_SPI       = 6,
    PRIORITY_MAIN      = 6,
    PRIORITY_BOOST     = 7,   // Highest
};


/**
 * @brief Scheduler singleton
 *
 * Manages timing, delays, and periodic callbacks. Creates FreeRTOS tasks
 * for timer and I/O processing.
 *
 * Inherits from AP_HAL::Scheduler for ArduPilot compatibility.
 */
class Scheduler : public AP_HAL::Scheduler {
public:
    Scheduler();
    ~Scheduler();

    // Prevent copying
    Scheduler(const Scheduler&) = delete;
    Scheduler& operator=(const Scheduler&) = delete;

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * @brief Initialize the scheduler
     *
     * Creates FreeRTOS tasks for timer and I/O callbacks.
     * Must be called before using any scheduler functions.
     */
    void init() override;

    // ========================================================================
    // Timing Functions
    // ========================================================================

    /**
     * @brief Get milliseconds since boot (32-bit)
     * @return Milliseconds, wraps at ~49 days
     */
    static uint32_t millis();

    /**
     * @brief Get milliseconds since boot (64-bit)
     * @return Milliseconds, effectively never wraps
     */
    static uint64_t millis64();

    /**
     * @brief Get microseconds since boot (32-bit)
     * @return Microseconds, wraps at ~71 minutes
     */
    static uint32_t micros();

    /**
     * @brief Get microseconds since boot (64-bit)
     * @return Microseconds, effectively never wraps
     */
    static uint64_t micros64();

    // ========================================================================
    // Delay Functions
    // ========================================================================

    /**
     * @brief Delay for milliseconds (RTOS-aware)
     *
     * Yields to FreeRTOS scheduler, allowing other tasks to run.
     * Calls registered delay callback if delay exceeds threshold.
     *
     * @param ms Milliseconds to delay
     */
    void delay(uint16_t ms) override;

    /**
     * @brief Delay for microseconds (busy-wait)
     *
     * Uses hardware timer for accuracy. Blocks the CPU.
     *
     * @param us Microseconds to delay (must be accurate)
     */
    void delay_microseconds(uint16_t us) override;

    /**
     * @brief Delay with priority boost for main thread
     *
     * Same as delay_microseconds but temporarily boosts priority.
     *
     * @param us Microseconds to delay
     */
    void delay_microseconds_boost(uint16_t us) override;

    /**
     * @brief End priority boost started by delay_microseconds_boost
     */
    void boost_end() override;

    // ========================================================================
    // Callback Registration
    // ========================================================================

    /**
     * @brief Register timer callback (high priority, ~1kHz)
     *
     * Callbacks run from the timer task at HAL_TIMER_RATE_HZ.
     * Maximum 8 timer processes.
     *
     * @param proc Member function to call
     */
    void register_timer_process(MemberProc proc) override;

    /**
     * @brief Register I/O callback (lower priority)
     *
     * Callbacks run from the I/O task.
     * Maximum 8 I/O processes.
     *
     * @param proc Member function to call
     */
    void register_io_process(MemberProc proc) override;

    /**
     * @brief Register failsafe callback
     *
     * Called at specified period regardless of main loop state.
     * Used for critical safety checks.
     *
     * @param failsafe Function to call
     * @param period_us Call period in microseconds
     */
    void register_timer_failsafe(Proc failsafe, uint32_t period_us) override;

    /**
     * @brief Register callback for long delays
     *
     * Called when delay() exceeds min_time_ms.
     * Allows background work during waits.
     *
     * @param proc Function to call
     * @param min_time_ms Minimum delay before callback triggers
     */
    void register_delay_callback(Proc proc, uint16_t min_time_ms) override;

    // ========================================================================
    // System State
    // ========================================================================

    /**
     * @brief Mark system as fully initialized
     *
     * Called after all subsystems are ready.
     */
    void set_system_initialized() override;

    /**
     * @brief Check if system is initialized
     * @return true if set_system_initialized() was called
     */
    bool is_system_initialized() override;

    /**
     * @brief Check if executing in main thread
     * @return true if current task is the main task
     */
    bool in_main_thread() const override;

    /**
     * @brief Check if inside delay callback
     * @return true if currently executing delay callback
     */
    bool in_delay_callback() const override;

    // ========================================================================
    // Expected Delay (Watchdog Management)
    // ========================================================================

    /**
     * @brief Notify scheduler of expected long operation
     *
     * Prevents watchdog timeouts during known long operations.
     *
     * @param ms Expected duration in milliseconds
     */
    void expect_delay_ms(uint32_t ms) override;

    /**
     * @brief Check if currently in expected delay
     * @return true if expect_delay_ms is active
     */
    bool in_expected_delay() const override;

    // ========================================================================
    // Thread Creation
    // ========================================================================

    /**
     * @brief Create a new thread
     *
     * @param proc Member function to run
     * @param name Thread name (for debugging)
     * @param stack_size Stack size in bytes
     * @param base Base priority level (uses AP_HAL::Scheduler::priority_base)
     * @param priority Priority offset from base
     * @return true on success
     */
    bool thread_create(MemberProc proc, const char* name,
                       uint32_t stack_size, priority_base base, int8_t priority) override;

    // ========================================================================
    // System Control
    // ========================================================================

    /**
     * @brief Reboot the system
     * @param hold_in_bootloader If true, enter bootloader mode
     */
    [[noreturn]] void reboot(bool hold_in_bootloader) override;

    // ========================================================================
    // Interrupt Control
    // ========================================================================

    /**
     * @brief Disable interrupts and save state
     * @return Opaque state to pass to restore_interrupts
     */
    void* disable_interrupts_save() override;

    /**
     * @brief Restore interrupt state
     * @param state Value returned by disable_interrupts_save
     */
    void restore_interrupts(void* state) override;

    // ========================================================================
    // Internal - called by FreeRTOS tasks
    // ========================================================================

    /** @brief Run timer callbacks (called by timer task) */
    void run_timer_procs();

    /** @brief Run I/O callbacks (called by I/O task) */
    void run_io_procs();

public:
    // Thread priorities for DeviceBus (per ESP32 pattern)
    static constexpr uint8_t kI2cThreadPriority = 5;  // I2C device polling
    static constexpr uint8_t kSpiThreadPriority = 6;  // SPI device polling (higher for IMU)

private:
    static constexpr uint8_t kMaxTimerProcs = 8;
    static constexpr uint8_t kMaxIOProcs = 8;

    // Callback arrays
    MemberProc m_timer_procs[kMaxTimerProcs];
    MemberProc m_io_procs[kMaxIOProcs];
    uint8_t m_num_timer_procs;
    uint8_t m_num_io_procs;

    // Failsafe
    Proc m_failsafe_proc;
    uint32_t m_failsafe_period_us;
    uint32_t m_failsafe_last_call_us;

    // Delay callback
    Proc m_delay_callback;
    uint16_t m_delay_callback_min_ms;
    bool m_in_delay_callback;

    // State flags
    bool m_initialized;
    bool m_system_initialized;
    uint32_t m_expected_delay_end_ms;

    // Main task handle (for in_main_thread check)
    TaskHandle_t m_main_task;

    // Task handles (buffers are file-scope in .cpp, like ArduPilot pattern)
    TaskHandle_t m_timer_task;
    TaskHandle_t m_io_task;
    TaskHandle_t m_storage_task;

    // Synchronization (pointers to file-scope objects)
    Semaphore* m_timer_sem;
    Semaphore* m_io_sem;

    // Priority boost state
    bool m_priority_boosted;
    UBaseType_t m_saved_priority;

    // Static task entry points
    static void timer_task_entry(void* param);
    static void io_task_entry(void* param);
    static void storage_task_entry(void* param);

    // Call delay callback if appropriate
    void call_delay_cb();
};

}  // namespace RP2350

// ============================================================================
// AP_HAL Namespace - Global timing functions
// ============================================================================

// Mark that full timing implementation is available
#define AP_HAL_TIMING_DEFINED 1

namespace AP_HAL {

// NOTE: Proc and MemberProc are defined in AP_HAL/AP_HAL_Namespace.h
// RP2350::Proc/MemberProc types must be compatible with AP_HAL::Proc/MemberProc

// Timing functions - declared here, defined in system.cpp
// These are required by ArduPilot libraries (e.g., AP_HAL/Util.cpp)
uint32_t millis();
uint64_t millis64();
uint32_t micros();
uint64_t micros64();

/**
 * @brief Panic handler - print message and halt
 */
[[noreturn]] void panic(const char* fmt, ...);

}  // namespace AP_HAL
