/**
 * @file AP_Motors_Class.h
 * @brief Stub AP_Motors for RocketChip
 *
 * Motor output is disabled in calibration/sensor phases.
 */
#pragma once

#include <cstdint>

#ifndef AP_MOTORS_ENABLED
#define AP_MOTORS_ENABLED 0
#endif

class AP_Motors {
public:
    enum spool_up_down_desired {
        DESIRED_SHUT_DOWN = 0,
        DESIRED_GROUND_IDLE = 1,
        DESIRED_THROTTLE_UNLIMITED = 2,
    };

    static AP_Motors* get_singleton() { return nullptr; }

    spool_up_down_desired get_spool_state() const { return DESIRED_SHUT_DOWN; }
    bool armed() const { return false; }
};
