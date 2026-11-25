// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// Debug bench for the RFM95 DIO1 and DIO2 jumpers, on both roles.
// Pins come from the board pack (board.h): Fruit Jam GPIO41 / 42,
// Feather GPIO26 / 27. Refuses on a pack without jumpers, and when the
// flight state is not IDLE. Station also refuses while the radio sends.
// On the vehicle, j/n arm a pending latch and wait up to 1 s of radio
// ticks for TX idle instead of refusing outright. Never Tx.
#ifndef ROCKETCHIP_DIAG_STATION_DIO_BENCH_H
#define ROCKETCHIP_DIAG_STATION_DIO_BENCH_H

#if defined(ROCKETCHIP_DEV_MODE)

void dio_bench_toggle_check();
void dio_bench_fifo_depth();

// Radio AO 100 Hz tick: run the armed bench once the radio is idle, or
// refuse after kIdleTimeoutTicks. No-op when the latch is clear.
void dio_bench_pending_tick(bool tx_active);

// True while a vehicle bench is armed, including while it runs.
bool dio_bench_blocks_tx();

#endif  // ROCKETCHIP_DEV_MODE

#endif
