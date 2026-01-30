/**
 * @file AP_CANManager.h
 * @brief Stub for ArduPilot CAN manager
 *
 * RocketChip doesn't use CAN bus. This stub satisfies include requirements.
 */

#pragma once

// CAN is disabled
#ifndef HAL_NUM_CAN_IFACES
#define HAL_NUM_CAN_IFACES 0
#endif

#ifndef HAL_MAX_CAN_PROTOCOL_DRIVERS
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0
#endif

#ifndef HAL_ENABLE_DRONECAN_DRIVERS
#define HAL_ENABLE_DRONECAN_DRIVERS 0
#endif
