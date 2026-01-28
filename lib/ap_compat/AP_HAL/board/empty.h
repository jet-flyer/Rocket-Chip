/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Board header for RP2350 - overrides ArduPilot's board/empty.h
 * This file is found first due to include path ordering.
 */

#pragma once

// ArduPilot build system macros - usually set by waf build
// __AP_LINE__ is used for error tracking with line numbers
#ifndef __AP_LINE__
#define __AP_LINE__ __LINE__
#endif

// WARN_IF_UNUSED must be defined before ArduPilot headers use it
// (AnalogIn.h uses it before AP_Common.h is included)
#ifndef WARN_IF_UNUSED
#define WARN_IF_UNUSED __attribute__((warn_unused_result))
#endif

// Board identification (use guards - hwdef.h may define these first)
#ifndef HAL_BOARD_NAME
#define HAL_BOARD_NAME "RP2350"
#endif
#ifndef HAL_CPU_CLASS
#define HAL_CPU_CLASS HAL_CPU_CLASS_150
#endif
#ifndef HAL_MEM_CLASS
#define HAL_MEM_CLASS HAL_MEM_CLASS_500  // RP2350 has 520KB SRAM + 8MB PSRAM
#endif

// Storage configuration
#ifndef HAL_STORAGE_SIZE
#define HAL_STORAGE_SIZE            16384
#endif
#define HAL_STORAGE_SIZE_AVAILABLE  HAL_STORAGE_SIZE

// Sensor defaults
#ifndef HAL_INS_DEFAULT
#define HAL_INS_DEFAULT HAL_INS_NONE
#endif
#ifndef CONFIG_HAL_BOARD_SUBTYPE
#define CONFIG_HAL_BOARD_SUBTYPE HAL_BOARD_SUBTYPE_NONE
#endif

// Program size
#ifndef HAL_PROGRAM_SIZE_LIMIT_KB
#define HAL_PROGRAM_SIZE_LIMIT_KB 8192  // 8MB flash
#endif

// Hardware features (use guards - hwdef.h may define these first)
#ifndef HAL_HAVE_BOARD_VOLTAGE
#define HAL_HAVE_BOARD_VOLTAGE 1
#endif
#ifndef HAL_HAVE_SERVO_VOLTAGE
#define HAL_HAVE_SERVO_VOLTAGE 0
#endif
#define HAL_HAVE_SAFETY_SWITCH 0
#ifndef HAL_WITH_IO_MCU
#define HAL_WITH_IO_MCU 0
#endif

// CAN - not supported on RP2350
#define HAL_NUM_CAN_IFACES 0
#define HAL_WITH_DRONECAN 0
#define HAL_WITH_UAVCAN 0
#define HAL_MAX_CAN_PROTOCOL_DRIVERS 0

// Disable some features not needed for RocketChip
#define AP_EXTERNAL_AHRS_ENABLED 0
#define HAL_GENERATOR_ENABLED 0
#define HAL_MOUNT_ENABLED 0
#define AP_CAMERA_ENABLED 0
#define HAL_SOARING_ENABLED 0
#define HAL_ADSB_ENABLED 0
#define HAL_BUTTON_ENABLED 0
#define AP_GRIPPER_ENABLED 0
#define AP_LANDINGGEAR_ENABLED 0
#define HAL_QUADPLANE_ENABLED 0
#define HAL_GYROFFT_ENABLED 0

// Compass backends - enable only what we use
#ifndef AP_COMPASS_BACKEND_DEFAULT_ENABLED
#define AP_COMPASS_BACKEND_DEFAULT_ENABLED 0
#endif

// Servo channels
#define NUM_SERVO_CHANNELS 8

// Disable temperature calibration
#define HAL_INS_TEMPERATURE_CAL_ENABLE 0

// Byte order
#define __LITTLE_ENDIAN  1234
#define __BYTE_ORDER     __LITTLE_ENDIAN

// RAM function attribute for RP2350
#define __RAMFUNC__ __attribute__((section(".time_critical")))

// Semaphore types - full definitions needed for member variables in ArduPilot headers
// Semaphores.h is designed to not require FreeRTOS.h (uses opaque storage)
#ifdef __cplusplus
#include <AP_HAL_RP2350/Semaphores.h>
#define HAL_Semaphore RP2350::Semaphore
#define HAL_BinarySemaphore RP2350::BinarySemaphore
#endif

// MAVLink types (MAV_RESULT, MAV_SEVERITY, etc.) - needed by many ArduPilot headers
// We provide minimal definitions here rather than including full MAVLink library
// because board/empty.h is included widely and not all targets have mavlink paths.
// Files that need full MAVLink (like GCS.h) include it directly with proper paths.
// Using same guards as MAVLink (HAVE_ENUM_*) to avoid conflicts when both are included.

#ifndef HAVE_ENUM_MAV_SEVERITY
#define HAVE_ENUM_MAV_SEVERITY
typedef enum MAV_SEVERITY {
    MAV_SEVERITY_EMERGENCY = 0,
    MAV_SEVERITY_ALERT = 1,
    MAV_SEVERITY_CRITICAL = 2,
    MAV_SEVERITY_ERROR = 3,
    MAV_SEVERITY_WARNING = 4,
    MAV_SEVERITY_NOTICE = 5,
    MAV_SEVERITY_INFO = 6,
    MAV_SEVERITY_DEBUG = 7
} MAV_SEVERITY;
#endif

#ifndef HAVE_ENUM_MAV_RESULT
#define HAVE_ENUM_MAV_RESULT
typedef enum MAV_RESULT {
    MAV_RESULT_ACCEPTED = 0,
    MAV_RESULT_TEMPORARILY_REJECTED = 1,
    MAV_RESULT_DENIED = 2,
    MAV_RESULT_UNSUPPORTED = 3,
    MAV_RESULT_FAILED = 4,
    MAV_RESULT_IN_PROGRESS = 5,
    MAV_RESULT_CANCELLED = 6
} MAV_RESULT;
#endif

// Scheduler overtime margin (microseconds) - be lenient during development
#define AP_SCHEDULER_OVERTIME_MARGIN_US 50000UL
