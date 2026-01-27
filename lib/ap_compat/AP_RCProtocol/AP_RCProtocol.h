/**
 * @file AP_RCProtocol.h
 * @brief Stub AP_RCProtocol for RocketChip
 *
 * RC Protocol parsing is disabled - provides stub interface.
 */
#pragma once

#include "AP_RCProtocol_config.h"

#if !AP_RCPROTOCOL_ENABLED

class AP_RCProtocol {
public:
    static AP_RCProtocol* get_singleton() { return nullptr; }
};

#endif // !AP_RCPROTOCOL_ENABLED
