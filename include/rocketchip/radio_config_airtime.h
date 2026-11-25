// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
// Radio port query: time on air of one frame on one rate-table preset.
// Modulation-agnostic. The MAC session windows (starcom_adapt/byte_pump.cpp)
// use it and do not call a modem formula. The radio port implements it:
// LoRa in src/drivers/rfm95w_airtime.cpp (SX1276 4.1.1.6). An FSK port
// implements the same function in its own file, and the build links one
// port.
#ifndef ROCKETCHIP_RADIO_CONFIG_AIRTIME_H
#define ROCKETCHIP_RADIO_CONFIG_AIRTIME_H

#include <stdint.h>

#include "rocketchip/radio_config_table.h"

namespace rc {

// Microseconds on air for frame_octets modem payload octets on preset.
uint32_t radio_preset_airtime_us(const RadioConfigEntry& preset,
                                 uint8_t frame_octets);

}  // namespace rc

#endif  // ROCKETCHIP_RADIO_CONFIG_AIRTIME_H
