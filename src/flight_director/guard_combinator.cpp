// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_combinator.cpp
 * @brief Guard combinator + safety lockout implementation
 */

#include "guard_combinator.h"
#include "flight_director.h"
#include "rocketchip/rc_log.h"
#include <cmath>

namespace rc {

// Phase bitmask helper
static constexpr uint8_t phase_bit(FlightPhase p) {
    return static_cast<uint8_t>(1U << static_cast<uint8_t>(p));
}

// Specification for one guard combinator (grouped to keep init within the
// JPL-25 parameter limit).
struct CombinatorSpec {
    CombinatorType type;
    const GuardId* ids;
    uint8_t n;
    uint16_t signal;
    uint32_t backup_ms;
    uint8_t phases;
};

static void init_combinator(GuardCombinator& c, const CombinatorSpec& spec) {
    c.type = spec.type;
    c.num_guards = spec.n;
    for (uint8_t i = 0; i < spec.n && i < 4; ++i) { c.guard_ids[i] = spec.ids[i]; }
    c.signal = spec.signal;
    c.backup_timeout_ms = spec.backup_ms;
    c.valid_phases = spec.phases;
    c.elapsed_ms = 0;
    c.fired = false;
}

void combinator_set_init(CombinatorSet* cs, const MissionProfile& profile) {
    cs->num_combinators = 0;
    cs->last_phase = FlightPhase::kIdle;

    // Apogee combinator: AND or OR based on profile
    {
        GuardId apogee_guards[] = { GuardId::kApogeeVelocity, GuardId::kBaroPeak };
        CombinatorType type = profile.apogee_require_both
            ? CombinatorType::kAnd : CombinatorType::kOr;
        init_combinator(cs->combinators[cs->num_combinators],
                        {type, apogee_guards, 2,
                         SIG_APOGEE, profile.coast_timeout_ms,
                         phase_bit(FlightPhase::kCoast)});
        ++cs->num_combinators;
    }

    // Main deploy combinator: single guard, lockout-gated, timer backup
    {
        GuardId main_guards[] = { GuardId::kMainDeploy };
        init_combinator(cs->combinators[cs->num_combinators],
                        {CombinatorType::kOr, main_guards, 1,
                         SIG_MAIN_DEPLOY, profile.main_backup_ms,
                         phase_bit(FlightPhase::kDrogueDescent)});
        ++cs->num_combinators;
    }
}

void combinator_set_reset(CombinatorSet* cs) {
    for (uint8_t i = 0; i < cs->num_combinators; ++i) {
        cs->combinators[i].elapsed_ms = 0;
        cs->combinators[i].fired = false;
    }
}

// Check if velocity lockout is active
static bool velocity_locked(const SafetyLockout& lo) {
    return lo.current_velocity_mps > lo.deploy_lockout_mps;
}

// Check if min-time lockout is active
static bool time_locked(const SafetyLockout& lo) {
    return lo.ms_since_launch < lo.apogee_lockout_ms;
}

// Evaluate a single combinator's sensor logic (Layer 2)
static bool evaluate_sensors(const GuardCombinator& c,
                              const GuardEvaluator* ev) {
    switch (c.type) {
        case CombinatorType::kAnd: {
            for (uint8_t i = 0; i < c.num_guards; ++i) {
                if (!guard_evaluator_is_sustained(ev, c.guard_ids[i])) {
                    return false;
                }
            }
            return true;
        }
        case CombinatorType::kOr:
        case CombinatorType::kPrimaryPlusTimeout: {
            for (uint8_t i = 0; i < c.num_guards; ++i) {
                if (guard_evaluator_is_sustained(ev, c.guard_ids[i])) {
                    return true;
                }
            }
            return false;
        }
    }
    return false;
}

// One combinator's evaluation this tick; returns its signal if it fires, else
// SIG_MAX. Split from combinator_set_evaluate for JSF-3 complexity (see CHANGELOG).
static uint16_t evaluate_one_combinator(GuardCombinator& c, uint8_t phase_mask,
                                        const GuardEvaluator* ev,
                                        const SafetyLockout& lockout,
                                        uint32_t tick_ms) {
    // Skip if not valid for current phase or already fired
    if ((c.valid_phases & phase_mask) == 0 || c.fired) {
        return SIG_MAX;
    }

    // Track elapsed time in phase
    c.elapsed_ms += tick_ms;

    // Layer 1: Lockout gates
    bool vel_locked = velocity_locked(lockout);
    bool t_locked = time_locked(lockout);

    // Confidence gate: when uncertain, block all pyro-bearing signals.
    // No fallback — when uncertain, the safest action is no action.
    if (!lockout.confident) {
        return SIG_MAX;
    }

    // Layer 2: Sensor detection (requires lockouts clear)
    if (!vel_locked && !t_locked) {
        if (evaluate_sensors(c, ev)) {
            c.fired = true;
            return c.signal;
        }
    }

    // Layer 3: Timer backup (requires lockouts clear, or ESKF-unhealthy
    // bypasses velocity lockout per Council A2)
    if (c.backup_timeout_ms > 0 && c.elapsed_ms >= c.backup_timeout_ms) {
        bool lockout_clear_for_timer = !t_locked &&
            (!vel_locked || !lockout.eskf_healthy);

        if (lockout_clear_for_timer) {
            c.fired = true;
            rc::rc_log("[FD] WARN: backup timer fired for signal %u\n",
                       static_cast<unsigned>(c.signal));
            return c.signal;
        }
    }

    return SIG_MAX;
}

uint16_t combinator_set_evaluate(CombinatorSet* cs,
                                  FlightPhase phase,
                                  const GuardEvaluator* ev,
                                  const SafetyLockout& lockout,
                                  uint32_t tick_ms) {
    // Phase transition: reset all combinator state
    if (phase != cs->last_phase) {
        combinator_set_reset(cs);
        cs->last_phase = phase;
    }

    uint8_t phase_mask = phase_bit(phase);

    for (uint8_t i = 0; i < cs->num_combinators; ++i) {
        uint16_t sig = evaluate_one_combinator(cs->combinators[i], phase_mask,
                                               ev, lockout, tick_ms);
        if (sig != SIG_MAX) {
            return sig;
        }
    }

    return SIG_MAX;
}

} // namespace rc
