/**
 * @file AP_ExternalAHRS.h
 * @brief External AHRS stub for RocketChip
 *
 * Disables external AHRS support - RocketChip uses internal AHRS.
 *
 * @note Part of RocketChip's ArduPilot compatibility layer
 */

#pragma once

#include "AP_ExternalAHRS_config.h"

// Stub class for AP_ExternalAHRS references
class AP_ExternalAHRS {
public:
    static AP_ExternalAHRS* get_singleton() { return nullptr; }
};
