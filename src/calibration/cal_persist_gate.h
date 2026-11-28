// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// Calibration flash-persist gate (pure, host-testable).
//
// calibration_save() erases and reprograms a cal-storage sector through
// flash_safe_execute() (calibration_storage.cpp). For the whole erase the
// other core is locked out and this core runs with IRQs off. That stall is
// acceptable only on the ground while disarmed, so every calibration flash
// write that is not an explicit operator command should ask this gate first.
//
// Phase source: rc::flight_phase_observable_get() (flight_state.h). It is a
// lock-free, any-context read of the phase the FD last entered, and it reads
// kFault before the FD seeds it or if the pair is corrupt, which this gate
// refuses. Reading it adds no AO coupling (AO Commandments V, VII).
//============================================================================
#ifndef ROCKETCHIP_CAL_PERSIST_GATE_H
#define ROCKETCHIP_CAL_PERSIST_GATE_H

#include "flight_director/flight_state.h"

namespace rc {

// True only in kIdle: disarmed on the ground, left only by an ARM.
// kArmed..kMainDescent are armed or flying. kLanded and kAbort wait for an
// operator RESET (back to kIdle). kFault is degraded or an untrusted read.
constexpr bool cal_persist_allowed(FlightPhase phase) {
    return phase == FlightPhase::kIdle;
}

} // namespace rc

#endif // ROCKETCHIP_CAL_PERSIST_GATE_H
