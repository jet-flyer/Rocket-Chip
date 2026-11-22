// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
//============================================================================
// GPS-derived WMM position: apply now, persist only when disarmed.
//
// The first GPS 3D fix of a boot upgrades the WMM field source to GPS. The
// filter needs the new field at once, in flight or not. Writing the position
// to cal storage is a flash erase + program, which must not happen while
// armed or flying (cal_persist_gate.h). So the fix is accepted at once and
// the persist is held until the FD is in kIdle: immediately for a fix on the
// pad before ARM, or after DISARM / landing + RESET for a fix after ARM.
//
// One fix and at most one persist per boot. A failed write is not retried;
// the next boot re-acquires the fix.
//============================================================================
#ifndef ROCKETCHIP_WMM_PERSIST_H
#define ROCKETCHIP_WMM_PERSIST_H

#include "calibration/cal_persist_gate.h"

namespace rc {

struct WmmPosition {
    float lat_deg;
    float lon_deg;
};

class WmmGpsPersist {
public:
    // First GPS 3D fix of the boot. Returns true when this call accepted the
    // fix: the caller applies the field in RAM now, whatever the phase.
    // Returns false for every later fix.
    bool accept_fix(float lat_deg, float lon_deg) {
        if (fix_accepted_) {
            return false;
        }
        fix_accepted_ = true;
        persist_pending_ = true;
        position_ = WmmPosition{lat_deg, lon_deg};
        return true;
    }

    // True exactly once per boot: a fix is pending and the phase allows a
    // flash write. Copies the position to *out and clears the pending flag.
    bool take_persist(FlightPhase phase, WmmPosition* out) {
        if (!persist_pending_ || out == nullptr) {
            return false;
        }
        if (!cal_persist_allowed(phase)) {
            return false;
        }
        persist_pending_ = false;
        *out = position_;
        return true;
    }

    bool persist_pending() const { return persist_pending_; }

private:
    WmmPosition position_{0.0F, 0.0F};
    bool fix_accepted_ = false;
    bool persist_pending_ = false;
};

} // namespace rc

#endif // ROCKETCHIP_WMM_PERSIST_H
