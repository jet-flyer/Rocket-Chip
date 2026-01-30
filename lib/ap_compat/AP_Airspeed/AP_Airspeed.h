/**
 * @file AP_Airspeed.h
 * @brief Stub for ArduPilot airspeed sensor interface
 *
 * RocketChip doesn't currently use airspeed sensors. This stub satisfies include requirements.
 *
 * FUTURE CONSIDERATION: Airspeed sensing for rockets is more complex than aircraft:
 * - Standard pitot tubes don't work well at supersonic/transonic speeds
 * - Mach transitions create shockwaves affecting pressure readings
 * - Would need calibrated nose pressure sensors for high-speed regimes
 * - Could be useful for parachute descent phase (subsonic)
 * - Wind estimation could help with parachute drift prediction
 *
 * If implemented later, will need rocket-specific pressure models, not aircraft ones.
 */

#pragma once

// Airspeed is disabled - enable when we have appropriate sensors
#ifndef AP_AIRSPEED_ENABLED
#define AP_AIRSPEED_ENABLED 0
#endif

#ifndef AP_AIRSPEED_AUAV_ENABLED
#define AP_AIRSPEED_AUAV_ENABLED 0
#endif
