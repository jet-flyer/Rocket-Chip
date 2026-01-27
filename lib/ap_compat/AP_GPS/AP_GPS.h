/**
 * @file AP_GPS.h
 * @brief GPS stub for RocketChip
 *
 * Disables ArduPilot GPS support - RocketChip uses its own GPS driver.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

#include "AP_GPS_config.h"
#include <AP_Common/Location.h>

// GPS fix type enumeration
enum GPS_Status {
    NO_GPS = 0,
    NO_FIX = 1,
    GPS_OK_FIX_2D = 2,
    GPS_OK_FIX_3D = 3,
    GPS_OK_FIX_3D_DGPS = 4,
    GPS_OK_FIX_3D_RTK_FLOAT = 5,
    GPS_OK_FIX_3D_RTK_FIXED = 6
};

// Stub class for AP_GPS references
class AP_GPS {
public:
    static AP_GPS* get_singleton() { return nullptr; }

    // GPS status methods (all return "no GPS" since we have our own driver)
    GPS_Status status(uint8_t instance = 0) const { return NO_GPS; }
    uint8_t num_sensors() const { return 0; }

    // Location methods
    bool get_location(uint8_t instance, Location& loc) const { return false; }

    // State accessor
    bool is_healthy() const { return false; }
};

// Namespace accessor
namespace AP {
    inline AP_GPS* gps() { return AP_GPS::get_singleton(); }
}
