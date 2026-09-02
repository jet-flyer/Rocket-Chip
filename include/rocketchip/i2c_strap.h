// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// 7-bit I2C strap map for this pack (Adafruit QT defaults).
// Board/job edge — not the generic master (WN-078 / WN-080).

#ifndef ROCKETCHIP_I2C_STRAP_H
#define ROCKETCHIP_I2C_STRAP_H

#include <stdint.h>

constexpr uint8_t kI2cAddrDps310      = 0x77;  // SDO high
constexpr uint8_t kI2cAddrDps310Alt   = 0x76;  // SDO low
constexpr uint8_t kI2cAddrIcm20948    = 0x69;  // AD0 high (Adafruit default)
constexpr uint8_t kI2cAddrIcm20948Alt = 0x68;  // AD0 low
constexpr uint8_t kI2cAddrAk09916     = 0x0C;  // Mag on ICM bypass
constexpr uint8_t kI2cAddrPa1010d     = 0x10;  // PA1010D I2C slave

#endif
