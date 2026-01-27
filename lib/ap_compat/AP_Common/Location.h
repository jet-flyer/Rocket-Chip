/**
 * @file Location.h
 * @brief Location class stub for RocketChip
 *
 * Provides Location class matching ArduPilot's interface for GPS positions.
 * Used by CompassCalibrator's fix_radius() function.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

#include <cstdint>

/**
 * @brief Location structure for GPS position
 *
 * Minimal implementation matching ArduPilot's Location class interface.
 */
class Location {
public:
    Location() : lat(0), lng(0), alt(0), relative_alt(0), terrain_alt(0), origin_alt(0) {}

    Location(int32_t lat_e7, int32_t lng_e7, int32_t alt_cm)
        : lat(lat_e7), lng(lng_e7), alt(alt_cm), relative_alt(0), terrain_alt(0), origin_alt(0) {}

    int32_t lat;            // Latitude in 1E7 degrees
    int32_t lng;            // Longitude in 1E7 degrees
    int32_t alt;            // Altitude in cm

    uint8_t relative_alt : 1;
    uint8_t terrain_alt : 1;
    uint8_t origin_alt : 1;

    // Altitude frame (matches ArduPilot)
    enum class AltFrame {
        ABSOLUTE = 0,
        ABOVE_HOME = 1,
        ABOVE_ORIGIN = 2,
        ABOVE_TERRAIN = 3
    };

    void zero() {
        lat = 0;
        lng = 0;
        alt = 0;
        relative_alt = 0;
        terrain_alt = 0;
        origin_alt = 0;
    }

    // Get latitude in degrees
    float lat_deg() const {
        return lat * 1.0e-7f;
    }

    // Get longitude in degrees
    float lng_deg() const {
        return lng * 1.0e-7f;
    }
};
