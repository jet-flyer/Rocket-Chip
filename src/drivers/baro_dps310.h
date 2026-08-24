// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// DPS310 Barometer wrapper using ruuvi.dps310.c library
// Provides Pico SDK integration for the proven ruuvi DPS310 driver.

#ifndef ROCKETCHIP_BARO_DPS310_H
#define ROCKETCHIP_BARO_DPS310_H

#include <stdint.h>
#include <stdbool.h>
#include "i2c_bus.h"

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kBaroDps310AddrDefault = kI2cAddrDps310;
constexpr uint8_t kBaroDps310AddrAlt     = kI2cAddrDps310Alt;

// Pressure OS/MR and temp OS/MR are the constexprs below.
// Datasheet OS / meas-time / RMS tables: Infineon DPS310 (not restated here).
constexpr uint8_t kBaroDps310PresOversampling = 8;
constexpr uint8_t kBaroDps310PresMeasRate     = 32;
constexpr uint8_t kBaroDps310TempOversampling = 1;
constexpr uint8_t kBaroDps310TempMeasRate     = 2;

// ============================================================================
// Types
// ============================================================================

typedef struct {
    float pressure_pa;      // Pressure in Pascals
    float temperature_c;    // Temperature in Celsius
    float altitude_m;       // Altitude in meters (calculated from sea level pressure)
    bool valid;             // Data validity flag
} baro_dps310_data_t;

// ============================================================================
// API
// ============================================================================

bool baro_dps310_init(uint8_t addr);

bool baro_dps310_ready(void);

bool baro_dps310_start_continuous(void);

bool baro_dps310_stop(void);

bool baro_dps310_read(baro_dps310_data_t* data);

void baro_dps310_set_sea_level(float pressurePa);

// Altitude in meters
float baro_dps310_pressure_to_altitude(float pressurePa, float seaLevelPa);

#endif // ROCKETCHIP_BARO_DPS310_H
