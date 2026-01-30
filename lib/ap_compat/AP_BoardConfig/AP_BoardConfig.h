/**
 * @file AP_BoardConfig.h
 * @brief Stub AP_BoardConfig for RocketChip
 *
 * Provides minimal interface to satisfy AP_Param dependencies.
 */
#pragma once

#include "AP_BoardConfig_config.h"
#include <cstdint>

class AP_BoardConfig {
public:
    static AP_BoardConfig* get_singleton() { return _singleton; }

    // Stubs for commonly called methods
    void init() {}

    // Board type enum - we return PX4_BOARD_AUTO (0) which doesn't match any special cases
    enum px4_board_type {
        PX4_BOARD_AUTO = 0,
        PX4_BOARD_PX4V1 = 1,
        PX4_BOARD_PIXHAWK = 2,
        PX4_BOARD_PIXHAWK2 = 3,
        PX4_BOARD_PIXRACER = 4,
        PX4_BOARD_PHMINI = 5,
        PX4_BOARD_AUAV21 = 20,
        PX4_BOARD_PH2SLIM = 21,
        PX4_BOARD_AEROFC = 98,
        PX4_BOARD_FMUV5 = 11,
        PX4_BOARD_FMUV6 = 14,
    };

    // Return AUTO - doesn't match any board-specific special cases in ArduPilot
    static px4_board_type get_board_type() { return PX4_BOARD_AUTO; }

    // Safety switch - always report armed/no switch
    bool safety_button_disabled() const { return true; }

    // PWM/Servo configuration stubs
    uint32_t get_default_safety_ignore_mask() const { return 0xFFFFFFFF; }

    // Parameter configuration (used by AP_Param)
    static bool allow_set_internal_parameters() { return true; }

    // Allocation error handler (panics on OOM)
    [[noreturn]] static void allocation_error(const char* reason) {
        (void)reason;
        while(1) {}  // Halt
    }

    // Configuration error handler (printf-style, used by AP_Param)
    static void config_error(const char* fmt, ...) {
        // Stub - ignore config errors on RocketChip
        // In real ArduPilot, this would trigger a pre-arm check failure
        (void)fmt;
    }

    // Config options (stubs)
    static bool io_enabled() { return false; }
    static bool rc_out_disabled() { return true; }

private:
    static AP_BoardConfig* _singleton;
};

namespace AP {
    AP_BoardConfig* boardConfig();
}
