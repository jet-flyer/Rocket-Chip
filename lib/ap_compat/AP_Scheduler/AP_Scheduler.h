/**
 * @file AP_Scheduler.h
 * @brief Stub AP_Scheduler for RocketChip
 *
 * RocketChip uses FreeRTOS tasks, not ArduPilot's scheduler.
 * Provides minimal interface for ArduPilot code compatibility.
 */
#pragma once

#include <cstdint>

class AP_Scheduler {
public:
    static AP_Scheduler* get_singleton() { return _singleton; }

    // Get main loop rate
    uint16_t get_loop_rate_hz() const { return _loop_rate_hz; }
    void set_loop_rate_hz(uint16_t hz) { _loop_rate_hz = hz; }

    // Loop timing info
    uint32_t get_loop_period_us() const {
        return _loop_rate_hz > 0 ? 1000000 / _loop_rate_hz : 0;
    }

    float get_loop_period_s() const {
        return _loop_rate_hz > 0 ? 1.0f / _loop_rate_hz : 0;
    }

    // Init
    void init() {}
    void loop() {}

    static AP_Scheduler* _singleton;

private:
    uint16_t _loop_rate_hz = 400;  // Default 400Hz
};

namespace AP {
    AP_Scheduler &scheduler();
}
