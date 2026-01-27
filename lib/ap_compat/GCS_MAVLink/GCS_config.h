/**
 * @file GCS_config.h
 * @brief GCS configuration for RocketChip
 *
 * Minimal GCS support - STATUSTEXT only via USB CDC.
 */
#pragma once

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 1  // Basic GCS enabled for status text
#endif

#ifndef HAL_NO_GCS
#define HAL_NO_GCS 0
#endif

#ifndef AP_MAVLINK_MSG_RELAY_STATUS_ENABLED
#define AP_MAVLINK_MSG_RELAY_STATUS_ENABLED 0
#endif

#ifndef AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED
#define AP_MAVLINK_MSG_MOUNT_CONTROL_ENABLED 0
#endif

#ifndef AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED
#define AP_MAVLINK_MSG_HIGHRES_IMU_ENABLED 0
#endif

#ifndef AP_MAVLINK_MSG_HIL_GPS_ENABLED
#define AP_MAVLINK_MSG_HIL_GPS_ENABLED 0
#endif

#ifndef AP_MAVLINK_BATTERY2_ENABLED
#define AP_MAVLINK_BATTERY2_ENABLED 0
#endif

#ifndef HAL_HIGH_LATENCY2_ENABLED
#define HAL_HIGH_LATENCY2_ENABLED 0
#endif

#ifndef HAL_MAVLINK_BINDINGS_ENABLED
#define HAL_MAVLINK_BINDINGS_ENABLED 0
#endif
