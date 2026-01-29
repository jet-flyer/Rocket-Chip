/**
 * @file hwdef.h
 * @brief Hardware definition for RocketChip (Adafruit Feather RP2350 HSTX)
 *
 * This file defines the sensor probes and board configuration for
 * ArduPilot libraries on RocketChip hardware.
 *
 * Hardware:
 * - MCU: RP2350A (dual Cortex-M33 @ 150MHz)
 * - IMU: ICM-20948 on I2C1 @ 0x69 (AD0 pulled HIGH, Adafruit default)
 * - Baro: DPS310 on I2C1 @ 0x77
 * - I2C1 pins: SDA=GPIO2, SCL=GPIO3 (STEMMA QT/Qwiic)
 */

#pragma once

// ============================================================================
// Board Identification
// ============================================================================

#define APJ_BOARD_ID 9999
#define HAL_BOARD_NAME "RocketChip-RP2350"

// ============================================================================
// IMU Configuration
// ============================================================================

// ICM-20948 on I2C bus 1 at address 0x69 (AD0=HIGH, Adafruit default)
// Bus 0 = Qwiic (I2C1 on Feather RP2350 hardware)
#define HAL_INS_DEFAULT INS_ROCKETCHIP_I2C
#define HAL_INS_INV2_I2C 1

// PROBE_IMU_I2C macro: Creates probe call for I2C IMUs
// driver: Driver class suffix (e.g., "Invensensev2" -> AP_InertialSensor_Invensensev2)
// bus: I2C bus number (0 = Qwiic on Feather)
// addr: 7-bit I2C address
// args: Additional arguments (e.g., rotation)
#define PROBE_IMU_I2C(driver, bus, addr, args ...) \
    ADD_BACKEND(AP_InertialSensor_ ## driver::probe(*this, GET_I2C_DEVICE(bus, addr), ##args))

// IMU probe list: ICM-20948 on bus 0 (Qwiic) at 0x69
// Note: Bus 0 in software maps to I2C1 hardware on Feather RP2350
// Adafruit ICM-20948 has AD0 pulled HIGH by default, so address is 0x69
// (bridging the SDO/ADR solder jumper changes it to 0x68)
#define HAL_INS_PROBE_LIST PROBE_IMU_I2C(Invensensev2, 0, 0x69, ROTATION_NONE)

// ============================================================================
// Barometer Configuration (future use)
// ============================================================================

// DPS310 on I2C bus 0 at address 0x77
// #define HAL_BARO_DEFAULT HAL_BARO_DPS280_I2C
// #define PROBE_BARO_I2C(driver, bus, addr) ...
// #define HAL_BARO_PROBE_LIST ...

// ============================================================================
// Magnetometer Configuration (future use)
// ============================================================================

// LIS3MDL on I2C bus 0 at address 0x1E
// ISM330DHCX+LIS3MDL FeatherWing uses different IMU but same mag
// #define HAL_MAG_DEFAULT HAL_MAG_LIS3MDL
// #define PROBE_MAG_I2C(driver, bus, addr, args...) ...
// #define HAL_MAG_PROBE_LIST ...

// ============================================================================
// I2C Configuration
// ============================================================================

// Feather RP2350 has Qwiic on I2C1 (GPIO2/3)
#define HAL_I2C_INTERNAL_MASK 0  // No internal I2C buses
#define HAL_I2C_EXTERNAL_MASK 1  // Bus 0 = Qwiic (external)

// ============================================================================
// Storage Configuration
// ============================================================================

// Flash storage at end of flash
#define HAL_STORAGE_SIZE 4096

// ============================================================================
// Serial Configuration
// ============================================================================

#define HAL_SERIAL0_BAUD_DEFAULT 115200
