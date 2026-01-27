/**
 * @file AP_MSP.h
 * @brief Stub AP_MSP for RocketChip
 *
 * MSP (MultiWii Serial Protocol) is disabled - provides stub interface.
 */
#pragma once

#include "msp.h"

#if !HAL_MSP_SENSORS_ENABLED

class AP_MSP {
public:
    static AP_MSP* get_singleton() { return nullptr; }
    void init() {}
};

#endif // !HAL_MSP_SENSORS_ENABLED
