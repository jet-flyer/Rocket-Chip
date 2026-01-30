/**
 * @file compass_stubs.cpp
 * @brief Stubs for AP_Compass complex dependencies
 *
 * Provides minimal implementations for:
 * - Compass_PerMotor (motor compensation - not needed for rockets)
 * - Location helper functions
 * - SRV_Channel stubs (servo channel - not used)
 *
 * These stubs allow AP_Compass to compile without pulling in the full
 * ArduPilot motor/servo subsystem.
 */

#include <AP_Math/AP_Math.h>
#include <AP_Param/AP_Param.h>
#include <AP_Compass/Compass_PerMotor.h>

// ============================================================================
// Compass_PerMotor stubs
// Per-motor compensation is for multicopters, not rockets
// ============================================================================

const AP_Param::GroupInfo Compass_PerMotor::var_info[] = {
    AP_GROUPEND
};

Compass_PerMotor::Compass_PerMotor(Compass &_compass) :
    compass(_compass)
{
}

void Compass_PerMotor::compensate(Vector3f &offset)
{
    // No compensation - disabled for rockets
    (void)offset;
}

// These functions are only called during motor calibration
void Compass_PerMotor::calibration_start(void) { }
void Compass_PerMotor::calibration_update(void) { }
void Compass_PerMotor::calibration_end(void) { }

// ============================================================================
// Location stubs
// ============================================================================

#include <AP_Common/Location.h>

// Location::zero() - set location to origin
void Location::zero()
{
    lat = 0;
    lng = 0;
    alt = 0;
    relative_alt = 0;
    terrain_alt = 0;
    origin_alt = 0;
    loiter_ccw = 0;
    loiter_xtrack = 0;
}
