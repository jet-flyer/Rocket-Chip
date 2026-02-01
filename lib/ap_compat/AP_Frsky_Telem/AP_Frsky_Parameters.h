/**
 * @file AP_Frsky_Parameters.h
 * @brief Stub for AP_Vehicle - feature disabled
 */
#pragma once

#ifndef HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#define HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL 0
#endif

#include <AP_Param/AP_Param.h>

// Stub class for parameter references
class AP_Frsky_Parameters {
public:
    static const struct AP_Param::GroupInfo var_info[];
};
