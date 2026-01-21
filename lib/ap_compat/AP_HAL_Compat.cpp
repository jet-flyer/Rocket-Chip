/**
 * @file AP_HAL_Compat.cpp
 * @brief ArduPilot HAL compatibility implementation
 *
 * Provides global symbols required by ArduPilot libraries.
 */

#include "AP_HAL/AP_HAL.h"

// Global HAL instance (stub - not actually used but required for linkage)
static AP_HAL::HAL hal_instance;
const AP_HAL::HAL& hal = hal_instance;
