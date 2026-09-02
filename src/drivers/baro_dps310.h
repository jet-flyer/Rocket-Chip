// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Infineon DPS310. Prior Art: IFXDS_DPS310; Adafruit DPS310 init.

#ifndef ROCKETCHIP_BARO_DPS310_H
#define ROCKETCHIP_BARO_DPS310_H

#include <stdint.h>
#include <stdbool.h>
#include "rocketchip/i2c_strap.h"

constexpr uint8_t kBaroDps310AddrDefault = kI2cAddrDps310;
constexpr uint8_t kBaroDps310AddrAlt     = kI2cAddrDps310Alt;
constexpr uint8_t kBaroDps310PresOversampling = 8;
constexpr uint8_t kBaroDps310PresMeasRate     = 32;
constexpr uint8_t kBaroDps310TempOversampling = 1;
constexpr uint8_t kBaroDps310TempMeasRate     = 2;

typedef struct {
    float pressure_pa;
    float temperature_c;
    float altitude_m;
    bool valid;
} baro_dps310_data_t;

bool baro_dps310_init(uint8_t addr);
bool baro_dps310_ready(void);
bool baro_dps310_start_continuous(void);
bool baro_dps310_stop(void);
bool baro_dps310_read(baro_dps310_data_t* data);
void baro_dps310_set_sea_level(float pressure_pa);
float baro_dps310_pressure_to_altitude(float pressure_pa, float sea_level_pa);

#endif
