/**
 * @file AP_BattMonitor.h
 * @brief Minimal stub for AP_Compass_Backend motor compensation
 *
 * RocketChip doesn't use motor compass compensation, but AP_Compass_Backend.cpp
 * requires this header. Provides empty implementations.
 */

#pragma once

#include <cstdint>

class AP_BattMonitor {
public:
    // Motor compensation needs current reading
    bool current_amps(float &current) {
        current = 0.0f;
        return false;  // No battery monitor
    }

    // Singleton stub
    static AP_BattMonitor *get_singleton() { return nullptr; }
};

namespace AP {
    inline AP_BattMonitor &battery() {
        static AP_BattMonitor batt;
        return batt;
    }
}
