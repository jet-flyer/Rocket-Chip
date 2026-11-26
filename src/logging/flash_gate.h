// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Flash gate (FSK pre-plan item 2). Measurement only: the gate does not
// change when or how flash operations run.
//
// Each flash_safe_execute call site uses this pattern:
//   const uint64_t t0 = rc::flash_gate_begin();
//   const int r = flash_safe_execute(fn, &params, timeout_ms);
//   rc::flash_gate_end(rc::FlashOpType::kProgram, t0, r);
// The time includes the multicore lockout handshake and the IRQ-off
// window on Core 0, because the radio poll loop sees all of that time.
// The call site keeps its own SDK callback, so the gate adds no
// function-pointer parameter (P10 Rule 9).
//
// Core 0 only. Do not call from IRQ code. The dump uses rc_log.
#ifndef ROCKETCHIP_LOGGING_FLASH_GATE_H
#define ROCKETCHIP_LOGGING_FLASH_GATE_H

#include <stdint.h>

#include "diag/flash_stall_stats.h"

namespace rc {

uint64_t flash_gate_begin();
void flash_gate_end(FlashOpType type, uint64_t start_us, int sdk_result);

// Read the JEDEC ID (command 9Fh) one time inside flash_safe_execute,
// then print the decode. Call at boot, after Core 1 is a lockout victim.
// The caller does the Core 1 I2C pause and the I2C reset (LL 31).
void flash_gate_read_jedec();

// Poll-gap hooks. Call at the entry of the poll function.
void flash_gate_note_radio_poll();  // AO_Radio handle_rx_poll
void flash_gate_note_mac_poll();    // AO_Telemetry starcom_poll_mac_radio

// Debug 'w': print JEDEC decode, flash stats, and poll-gap stats.
// Values are cumulative since boot.
void flash_gate_dump();

} // namespace rc

#endif // ROCKETCHIP_LOGGING_FLASH_GATE_H
