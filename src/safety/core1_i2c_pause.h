// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
#ifndef ROCKETCHIP_SAFETY_CORE1_I2C_PAUSE_H
#define ROCKETCHIP_SAFETY_CORE1_I2C_PAUSE_H

// Cooperative pause/resume of Core 1's I2C sensor reads.
//
// Purpose: prevent the LL Entry 31 race class. When Core 0 runs a
// flash_safe_execute(), `multicore_lockout` halts Core 1's CPU but
// does NOT drain transactions the DW_apb_i2c peripheral was already
// driving on Core 1's behalf. An in-flight I2C transaction at the
// moment lockout fires gets abandoned (APB bridge 65,535-cycle
// timeout per RP2350 datasheet §2.1.4), leaving the peripheral in a
// corrupt state. R-15 added a post-flash i2c_bus_reset() to recover,
// but the in-flight transaction's sensor sample is lost.
//
// These primitives pre-empt that race by cooperatively pausing
// Core 1's sensor loop BEFORE Core 0 enters the flash window. Core 1
// finishes any in-flight transaction at its next loop boundary,
// acknowledges by setting `g_core1I2CPaused`, then waits in a
// busy-loop. Core 0 sees the acknowledgement and proceeds to
// flash_safe_execute(). After the flash op + reset, Core 0 calls
// core1_i2c_resume() and Core 1 returns to normal sensor reads.
//
// R-17 (2026-05-13 audit) wires these primitives around every
// reachable runtime flash_safe_execute() callsite. R-11's
// `xltl_no_i2c_during_flash` property is upgraded from documented-
// fail to hard-PASS once R-17 lands.
//
// Why a new module instead of using cal_hooks.cpp's mechanism?
//
//   cal_hooks.cpp's `cal_post_hook()` also sets `g_calReloadPending`
//   which signals Core 1 to reload calibration data. That's
//   calibration-specific; using it in non-cal flash paths (log flush,
//   flight-table erase) would falsely re-trigger a calibration reload.
//   `core1_i2c_pause()` / `core1_i2c_resume()` are the same I2C-pause
//   mechanism without the cal-reload tag-along.
//
//   Surfaced finding during R-17 prep (2026-05-13): the original
//   `cal_pre_hook()` in cal_hooks.cpp was DEAD CODE — defined but
//   never called from anywhere (the function-pointer table
//   `rc_os_cal_pre_hook` was assigned at main.cpp:315 but never
//   invoked). The calibration save path was running with the LL-31
//   race open. R-17 fixes this by wiring the new pause primitive at
//   `cal_save_to_flash()` directly; R-18 removes the now-fully-dead
//   `cal_pre_hook()` + function-pointer table.

namespace rc {

// Request Core 1 to pause its sensor loop and release I2C bus
// ownership. Blocks until Core 1 acknowledges (max ~100 ms per
// existing cal_hooks pattern); returns immediately if the sensor
// phase is not active (e.g., Core 1 idle on station/relay role).
//
// Safe to call from any handler context on Core 0 (cooperative-QV).
// Cost: up to ~100 ms wall time waiting for ack; budget accordingly
// in tick handlers (LL Entry 32 framing).
void core1_i2c_pause();

// Release the pause set by core1_i2c_pause(). Core 1's sensor loop
// resumes at its next tick. Idempotent; safe to call without a prior
// pause (no-op).
void core1_i2c_resume();

}  // namespace rc

#endif  // ROCKETCHIP_SAFETY_CORE1_I2C_PAUSE_H
