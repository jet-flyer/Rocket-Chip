/**
 * @file AP_HAL_Compat.cpp
 * @brief ArduPilot HAL compatibility implementation
 *
 * Provides global symbols required by ArduPilot libraries.
 */

#include "AP_HAL/AP_HAL.h"
#include "AP_InternalError/AP_InternalError.h"

// Global HAL instance (stub - not actually used but required for linkage)
static AP_HAL::HAL hal_instance;
const AP_HAL::HAL& hal = hal_instance;

// AP_InternalError static member definitions
uint32_t AP_InternalError::s_errorMask = 0;
uint32_t AP_InternalError::s_errorCount = 0;
