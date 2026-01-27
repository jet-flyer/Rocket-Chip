/**
 * @file AP_AHRS_View.h
 * @brief Stub AP_AHRS_View for RocketChip
 *
 * AP_AHRS_View provides rotated views into AHRS for sensors mounted
 * at different orientations. For RocketChip, we use identity rotation
 * and forward to the main AHRS.
 */
#pragma once

#include "AP_AHRS.h"

class AP_AHRS_View {
public:
    AP_AHRS_View(AP_AHRS &ahrs, enum Rotation rotation, float pitch_trim_deg = 0)
        : _ahrs(ahrs), _rotation(rotation), _pitch_trim_deg(pitch_trim_deg) {}

    // Forward to main AHRS - no rotation applied for stub
    const Matrix3f& get_rotation_body_to_ned() const {
        return _ahrs.get_DCM_rotation_body_to_ned();
    }

    float get_roll() const { return _ahrs.get_roll_rad(); }
    float get_pitch() const { return _ahrs.get_pitch_rad(); }
    float get_yaw() const { return _ahrs.get_yaw_rad(); }

    bool healthy() const { return _ahrs.healthy(); }

private:
    AP_AHRS& _ahrs;
    enum Rotation _rotation;
    float _pitch_trim_deg;
};
