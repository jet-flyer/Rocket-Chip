/**
 * @file AP_HAL_Boards.h
 * @brief Board configuration shim for RocketChip
 *
 * This file intercepts ArduPilot's AP_HAL_Boards.h and provides
 * board configuration for our RP2350-based RocketChip.
 */

#pragma once

// Define our board type before anything else
#define HAL_BOARD_ROCKETCHIP 99
#define CONFIG_HAL_BOARD HAL_BOARD_ROCKETCHIP
#define CONFIG_HAL_BOARD_SUBTYPE 0

// Board name
#define HAL_BOARD_NAME "RocketChip-RP2350"

// Memory and flash limits
#define HAL_PROGRAM_SIZE_LIMIT_KB 4096
#define HAL_MEM_CLASS HAL_MEM_CLASS_500

// Feature flags - disable most ArduPilot features by default
// These can be overridden by hwdef.h if included first
#ifndef AP_PARAM_ENABLED
#define AP_PARAM_ENABLED 0
#endif

#ifndef HAL_GCS_ENABLED
#define HAL_GCS_ENABLED 0
#endif

#ifndef HAL_LOGGING_ENABLED
#define HAL_LOGGING_ENABLED 0
#endif

#ifndef HAL_HAVE_IMU_HEATER
#define HAL_HAVE_IMU_HEATER 0
#endif

#ifndef AP_AHRS_ENABLED
#define AP_AHRS_ENABLED 0
#endif

#ifndef AP_AHRS_DCM_ENABLED
#define AP_AHRS_DCM_ENABLED 0
#endif

#ifndef AP_GPS_ENABLED
#define AP_GPS_ENABLED 0
#endif

#ifndef HAL_NAVEKF2_AVAILABLE
#define HAL_NAVEKF2_AVAILABLE 0
#endif

#ifndef HAL_NAVEKF3_AVAILABLE
#define HAL_NAVEKF3_AVAILABLE 0
#endif

#ifndef AP_SCRIPTING_ENABLED
#define AP_SCRIPTING_ENABLED 0
#endif

#ifndef HAL_WITH_EKF_DOUBLE
#define HAL_WITH_EKF_DOUBLE 0
#endif

#ifndef AP_INERTIALSENSOR_ENABLED
#define AP_INERTIALSENSOR_ENABLED 0
#endif

#ifndef HAL_INS_ACCELCAL_ENABLED
#define HAL_INS_ACCELCAL_ENABLED 1
#endif

#ifndef AP_LOGGER_ENABLED
#define AP_LOGGER_ENABLED 0
#endif

#ifndef AP_COMPASS_ENABLED
#define AP_COMPASS_ENABLED 0
#endif

// Scheduler
#define HAL_SCHEDULER_ENABLED 0
#define HAL_STORAGE_SIZE 4096

// AP_FlashStorage type (2 = F4 style, can clear individual bits)
#define AP_FLASHSTORAGE_TYPE 2

// Required macros for ArduPilot headers
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif

// Serial ports
#define HAL_NUM_CAN_IFACES 0
#define BOARD_FLASH_SIZE 4096

// Required macros that would normally be set by board header
#define HAL_HAVE_BOARD_VOLTAGE 0
#define HAL_HAVE_SERVO_VOLTAGE 0
#define HAL_WITH_IO_MCU 0
#define HAL_WITH_DRONECAN 0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0
