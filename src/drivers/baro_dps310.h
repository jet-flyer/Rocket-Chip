// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file baro_dps310.h
 * @brief DPS310 Barometer wrapper using ruuvi.dps310.c library
 *
 * Provides Pico SDK integration for the proven ruuvi DPS310 driver.
 */

#ifndef ROCKETCHIP_BARO_DPS310_H
#define ROCKETCHIP_BARO_DPS310_H

#include <stdint.h>
#include <stdbool.h>

// ============================================================================
// Configuration
// ============================================================================

constexpr uint8_t kBaroDps310AddrDefault    = 0x77;
constexpr uint8_t kBaroDps310AddrAlt        = 0x76;

// DPS310 oversampling and measurement rate configuration.
// Change these to trade precision vs latency/power.
//
// DPS310 datasheet Table 16 (Infineon):
//   OS   Noise(Pa)  Alt(m)  MeasTime  Current@1Hz  MaxRate
//   1x    2.5       0.21     3.6ms     2.1uA       128Hz
//   2x    1.0       0.083    5.2ms     2.7uA        64Hz
//   4x    0.5       0.042    8.4ms     3.8uA        32Hz
//   8x    0.4       0.033   14.8ms     6.1uA        16Hz
//  16x    0.35      0.029   27.6ms    11  uA         8Hz  <-- ArduPilot default
//  32x    0.3       0.025   53.2ms    20  uA         4Hz
//  64x    0.2       0.017  104.4ms    38  uA         2Hz
// 128x   ~0.15      0.012  206.8ms    75  uA         1Hz
//
// Diminishing returns after 8x. ArduPilot uses 16x @ 32Hz.
// Flight: 8x-16x recommended (fast enough for boost/coast tracking).
// Ground testing: up to 64x is fine if latency isn't critical.
//
// Ruuvi driver enums: DPS310_OS_1 .. DPS310_OS_128, DPS310_MR_1 .. DPS310_MR_128
// Pressure and temperature are configured independently. Temperature compensation
// needs ~1-2 Hz (thermal time constant of sensor package is 5-15s). Pressure gets
// the full rate budget.
//
// Duty cycle model (CONT_BOTH mode):
//   total = P_rate × P_meas_time + T_rate × T_meas_time ≤ 1000ms
//   Current: 32×14.8ms + 2×3.6ms = 481ms (52% margin)
//
// Runtime reconfiguration is supported — DPS310 accepts new config
// mid-measurement. Future: phase-scheduled OS switching (Stage 10).
constexpr uint8_t kBaroDps310PresOversampling = 8;   // 8x: 0.4 Pa noise, 14.8ms meas time
constexpr uint8_t kBaroDps310PresMeasRate     = 32;  // 32 pressure readings/sec (proven rate)
constexpr uint8_t kBaroDps310TempOversampling = 1;   // 1x: minimal (compensation only)
constexpr uint8_t kBaroDps310TempMeasRate     = 2;   // 2 Hz (covers worst-case thermal transients)

// ============================================================================
// Types
// ============================================================================

/**
 * @brief Barometer data
 */
typedef struct {
    float pressure_pa;      ///< Pressure in Pascals
    float temperature_c;    ///< Temperature in Celsius
    float altitude_m;       ///< Altitude in meters (calculated from sea level pressure)
    bool valid;             ///< Data validity flag
} baro_dps310_data_t;

// ============================================================================
// API
// ============================================================================

/**
 * @brief Initialize the DPS310 barometer
 * @param addr I2C address (kBaroDps310AddrDefault or kBaroDps310AddrAlt)
 * @return true on success
 */
bool baro_dps310_init(uint8_t addr);

/**
 * @brief Check if barometer is initialized and ready
 * @return true if ready
 */
bool baro_dps310_ready(void);

/**
 * @brief Start continuous measurement mode
 * @return true on success
 */
bool baro_dps310_start_continuous(void);

/**
 * @brief Stop continuous measurement (enter standby)
 * @return true on success
 */
bool baro_dps310_stop(void);

/**
 * @brief Read latest pressure and temperature data
 * @param data Output data structure
 * @return true on success
 */
bool baro_dps310_read(baro_dps310_data_t* data);

/**
 * @brief Set sea level pressure for altitude calculation
 * @param pressure_pa Sea level pressure in Pascals (default 101325)
 */
void baro_dps310_set_sea_level(float pressurePa);

/**
 * @brief Calculate altitude from pressure
 * @param pressurePa Current pressure in Pascals
 * @param seaLevelPa Sea level pressure in Pascals
 * @return Altitude in meters
 */
float baro_dps310_pressure_to_altitude(float pressurePa, float seaLevelPa);

#endif // ROCKETCHIP_BARO_DPS310_H
