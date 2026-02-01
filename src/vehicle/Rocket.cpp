/**
 * @file Rocket.cpp
 * @brief RocketChip vehicle implementation with AP_Param var_info table
 *
 * This file defines:
 * - AP_PARAM_VEHICLE_NAME macro (required by GOBJECTPTR)
 * - Parameters class with k_param enumeration
 * - var_info table linking sensors to parameter storage
 * - param_loader that registers the table with AP_Param
 */

// Define vehicle name BEFORE any ArduPilot includes
// This macro is used by GOBJECTPTR and related macros
#define AP_PARAM_VEHICLE_NAME rocket

#include "Rocket.h"

#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Param/AP_Param.h>

// Static singleton pointer
Rocket* Rocket::_singleton = nullptr;

// Global vehicle instance
Rocket rocket;

/**
 * @brief Parameter key enumeration
 *
 * These keys are stored in flash and map to specific objects.
 * DO NOT CHANGE existing values - that would break saved parameters.
 * Only add new values at the end.
 */
class Parameters {
public:
    enum {
        // Start at 1 (0 is reserved)
        k_param_ins = 1,
        k_param_compass = 2,
        k_param_baro = 3,

        // Add new parameters here, incrementing the number
        // k_param_gps = 4,  // Future
    };
};

/**
 * @brief AP_Param var_info table
 *
 * This table registers sensors with AP_Param for calibration persistence.
 * The table itself is static const (in .rodata), but the sensor pointers
 * it contains (_ins, _compass, _baro) are populated later in allocate_sensors().
 *
 * IMPORTANT: The AP_Param singleton must be created AFTER allocate_sensors()
 * but BEFORE AP_Param::setup(). We use a static pointer instead of a global
 * constructor to control initialization order.
 */
const AP_Param::Info Rocket::var_info[] = {
    { "INS",     (const void *)&rocket._ins,     {group_info: AP_InertialSensor::var_info}, AP_PARAM_FLAG_POINTER, Parameters::k_param_ins,     AP_PARAM_GROUP },
    { "COMPASS", (const void *)&rocket._compass, {group_info: Compass::var_info},           AP_PARAM_FLAG_POINTER, Parameters::k_param_compass, AP_PARAM_GROUP },
    { "BARO",    (const void *)&rocket._baro,    {group_info: AP_Baro::var_info},           AP_PARAM_FLAG_POINTER, Parameters::k_param_baro,    AP_PARAM_GROUP },
    AP_VAREND
};

// Static storage for param_loader - constructed on first use, not at global init
static AP_Param* s_param_loader = nullptr;

void Rocket::init_params() {
    if (s_param_loader == nullptr) {
        // Create AP_Param singleton with our var_info table
        // Using placement new into static storage to avoid heap allocation
        static uint8_t param_storage[sizeof(AP_Param)];
        s_param_loader = new (param_storage) AP_Param(var_info);
    }
}

// ============================================================================
// Rocket Implementation
// ============================================================================

Rocket::Rocket()
    : _ins(nullptr)
    , _compass(nullptr)
    , _baro(nullptr)
{
    if (_singleton) {
        // Panic or log error - only one instance allowed
        // For now just overwrite (shouldn't happen)
    }
    _singleton = this;
}

void Rocket::allocate_sensors()
{
    // Allocate sensors - must be called AFTER HAL init but BEFORE AP_Param::load_all()
    // The constructors may access AP_Param, which needs HAL to be initialized
    _ins = new AP_InertialSensor();
    _compass = new Compass();
    _baro = new AP_Baro();
}
