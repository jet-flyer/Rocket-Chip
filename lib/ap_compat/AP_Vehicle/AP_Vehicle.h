/**
 * @file AP_Vehicle.h
 * @brief Vehicle stub for RocketChip
 *
 * RocketChip doesn't use the full AP_Vehicle framework.
 * This stub satisfies include requirements from AP_Baro.
 */

#pragma once

#include "AP_Vehicle_Type.h"

// Stub class for compatibility
class AP_Vehicle {
public:
    static AP_Vehicle* get_singleton() { return nullptr; }
};
