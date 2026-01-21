/**
 * @file AP_Vehicle_Type.h
 * @brief Vehicle type stub for RocketChip
 *
 * RocketChip is a rocket (type ROCKET), not a standard ArduPilot vehicle.
 */

#pragma once

// Vehicle type definitions
#define APM_BUILD_UNKNOWN              0
#define APM_BUILD_ArduPlane            1
#define APM_BUILD_ArduCopter           2
#define APM_BUILD_Rover                3
#define APM_BUILD_AntennaTracker       4
#define APM_BUILD_ArduSub              5
#define APM_BUILD_Replay               6
#define APM_BUILD_AP_Periph            7
#define APM_BUILD_Blimp                8
#define APM_BUILD_Heli                 9
#define APM_BUILD_UNKNOWN              0

// RocketChip is a custom vehicle type
#ifndef APM_BUILD_TYPE
#define APM_BUILD_TYPE(type) (APM_BUILD_UNKNOWN)
#endif

// Simple check macros
#ifndef APM_BUILD_COPTER_OR_HELI
#define APM_BUILD_COPTER_OR_HELI() (false)
#endif
