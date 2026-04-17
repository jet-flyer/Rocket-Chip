// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//
// IVP-132a.4a — DIO0 bring-up test.
//
// Active test that the RP2350 can observe DIO0 edges from the SX1276.
// Required by JPL council ("don't bundle coverage gaps into soak tests"):
// this catches "init flag true + IRQ never fires" which passive observation
// alone cannot distinguish from "radio is healthy but idle."
//
// Runs ONCE at bring-up. Destructive to radio state (reconfigs DIO0
// mapping + forces RX mode); reinit radio afterwards.
//
// Bench binary only. GDB-callable: `call dio0_bringup_test()`.
#ifndef ROCKETCHIP_DEV_DIO0_BRINGUP_H
#define ROCKETCHIP_DEV_DIO0_BRINGUP_H

#include <stdint.h>

#ifdef BUILD_FOR_FLIGHT

static inline int dio0_bringup_test() { return 0; }

#else

extern "C" {
// Runs a forced-edge test on the DIO0 pin.
// Returns:
//   0  = PASS (edges observed, radio reinit'd OK)
//   1  = FAIL: could not drive DIO0 (GPIO conflict)
//   2  = FAIL: SX1276 unreachable (RegVersion != 0x12)
//   3  = FAIL: no edges observed after forcing
//   4  = FAIL: radio reinit failed after test
int dio0_bringup_test();

// Running counter of DIO0 edges observed. Readable between tests.
uint32_t dio0_bringup_edge_count();
}

#endif
#endif  // ROCKETCHIP_DEV_DIO0_BRINGUP_H
