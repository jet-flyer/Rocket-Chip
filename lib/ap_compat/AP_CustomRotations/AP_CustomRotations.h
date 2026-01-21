/**
 * @file AP_CustomRotations.h
 * @brief AP_CustomRotations stub for RocketChip
 *
 * Custom rotations are not used in RocketChip - we use standard orientations.
 */

#pragma once

#include <AP_Math/vector3.h>

// Feature flag to disable custom rotations
#ifndef AP_CUSTOMROTATIONS_ENABLED
#define AP_CUSTOMROTATIONS_ENABLED 0
#endif

#if AP_CUSTOMROTATIONS_ENABLED

class AP_CustomRotations {
public:
    void from_rotation(uint8_t r, Quaternion &q) const {
        // Not implemented - custom rotations disabled
        q.from_euler(0, 0, 0);
    }

    bool enabled(uint8_t r) const {
        return false;
    }

    void convert(uint8_t r, float roll, float pitch, float yaw,
                 float *roll_out, float *pitch_out, float *yaw_out) const {
        *roll_out = roll;
        *pitch_out = pitch;
        *yaw_out = yaw;
    }
};

namespace AP {
    inline AP_CustomRotations* custom_rotations() {
        return nullptr;
    }
}

#else

// When disabled, provide nullptr accessor
namespace AP {
    inline void* custom_rotations() {
        return nullptr;
    }
}

#endif // AP_CUSTOMROTATIONS_ENABLED
