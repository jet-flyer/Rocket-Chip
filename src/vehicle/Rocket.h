/**
 * @file Rocket.h
 * @brief RocketChip vehicle class for ArduPilot integration
 *
 * Provides:
 * - Sensor object ownership (INS, Compass, Baro)
 * - AP_Param var_info table for calibration persistence
 * - Singleton pattern matching ArduPilot conventions
 *
 * This is a minimal vehicle class that enables proper AP_Param integration
 * without the full AP_Vehicle framework. When ChibiOS is stable, this can
 * be migrated to inherit from AP_Vehicle.
 */

#pragma once

#include <AP_Param/AP_Param.h>

// Forward declarations (avoid heavy includes in header)
class AP_InertialSensor;
class Compass;
class AP_Baro;

/**
 * @brief Minimal vehicle class for AP_Param integration
 *
 * Named "Rocket" for potential upstream ArduPilot acceptance.
 * Cleaner than "RocketChip" and appropriate for rocket applications.
 */
class Rocket {
public:
    Rocket();

    // Sensor accessors (allocated after HAL init)
    AP_InertialSensor* ins() { return _ins; }
    Compass* compass() { return _compass; }
    AP_Baro* baro() { return _baro; }

    // Sensor allocation (called after HAL init, before AP_Param::load_all)
    void allocate_sensors();

    // Initialize AP_Param with var_info table (called after allocate_sensors)
    void init_params();

    // Singleton access
    static Rocket* get_singleton() { return _singleton; }

    // AP_Param var_info table (defined in Rocket.cpp)
    static const AP_Param::Info var_info[];

    // Sensor pointers - public for var_info table access
    // These are allocated after HAL init (constructors access AP_Param)
    AP_InertialSensor* _ins;
    Compass* _compass;
    AP_Baro* _baro;

private:
    static Rocket* _singleton;
};

// Global rocket instance (constructed at startup, sensors allocated later)
extern Rocket rocket;
