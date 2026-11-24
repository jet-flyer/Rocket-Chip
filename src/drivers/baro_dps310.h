// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Infineon DPS310. Prior Art: IFXDS_DPS310; Adafruit DPS310 init.

#ifndef ROCKETCHIP_BARO_DPS310_H
#define ROCKETCHIP_BARO_DPS310_H

#include <stdint.h>
#include <stdbool.h>
#include "rocketchip/i2c_strap.h"
#include "drivers/baro_sample.h"

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
// Unconditional re-read of the result registers (CLI/diagnostics). It
// returns the previous result again if no conversion finished since.
bool baro_dps310_read(baro_dps310_data_t* data);
// Sensor-loop read: one MEAS_CFG read, then TMP only if TMP_RDY, PSR only if
// PRS_RDY (gate: drivers/dps310_poll.h). kNewSample fills data; kNoNewSample
// leaves it untouched and is not a fault.
BaroSampleStatus baro_dps310_poll(baro_dps310_data_t* data);
// Longest a healthy part in continuous mode goes between new samples: two
// temperature periods. After (re)start a pressure can't be compensated until
// the first temperature (one period); after that pressure arrives every
// 1/kBaroDps310PresMeasRate s. Longer than this means conversions stopped.
constexpr uint32_t kBaroDps310GapTempPeriods = 2;
constexpr uint32_t kBaroDps310UsPerS = 1000000;
constexpr uint32_t kBaroDps310MaxSampleGapUs =
    kBaroDps310GapTempPeriods * kBaroDps310UsPerS / kBaroDps310TempMeasRate;
void baro_dps310_set_sea_level(float pressure_pa);
float baro_dps310_pressure_to_altitude(float pressure_pa, float sea_level_pa);

#endif
