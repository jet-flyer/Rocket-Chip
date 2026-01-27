/**
 * @file Scheduler.h
 * @brief AP_HAL::Scheduler stub for RocketChip
 *
 * Provides minimal Scheduler interface for ArduPilot library compatibility.
 * Full implementation is in AP_HAL_RP2350/Scheduler.h.
 */
#pragma once

#include <cstdint>

namespace AP_HAL {

class Scheduler {
public:
    virtual ~Scheduler() = default;

    // Delay functions
    virtual void delay(uint16_t ms) { (void)ms; }
    virtual void delay_microseconds(uint16_t us) { (void)us; }

    // Registration for periodic callbacks
    typedef void (*proc_t)(void);
    virtual void register_timer_process(proc_t proc) { (void)proc; }
    virtual void register_io_process(proc_t proc) { (void)proc; }

    // Semaphore management
    virtual bool in_main_thread() const { return true; }
    virtual void suspend_timer_procs() {}
    virtual void resume_timer_procs() {}

    // System state
    virtual bool is_system_initialized() const { return true; }
    virtual void system_initialized() {}

    // Loop rate (used by AP_Param for rate limiting)
    virtual uint16_t get_loop_rate_hz() const { return 400; }
    virtual uint32_t get_loop_period_us() const { return 2500; }
    virtual float get_loop_period_s() const { return 0.0025f; }

    // Thread management (stubs)
    virtual void *thread_create(void (*fn)(void *), const char *name, uint32_t stack_size, void *ctx, int8_t priority) {
        (void)fn; (void)name; (void)stack_size; (void)ctx; (void)priority;
        return nullptr;
    }

    // Expect delay (used during calibration)
    virtual void expect_delay_ms(uint32_t ms) { (void)ms; }
};

}  // namespace AP_HAL
