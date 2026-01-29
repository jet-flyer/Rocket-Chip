/**
 * @file AP_GPS.cpp
 * @brief GPS stub static definitions
 */

#include "AP_GPS.h"

// Singleton instance (lazy init to avoid static constructor issues)
static AP_GPS* s_gps_instance = nullptr;

AP_GPS* AP_GPS::get_singleton() {
    if (!s_gps_instance) {
        s_gps_instance = new AP_GPS();
    }
    return s_gps_instance;
}
