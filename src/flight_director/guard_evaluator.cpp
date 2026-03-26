// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (c) 2025-2026 Rocket Chip Project
/**
 * @file guard_evaluator.cpp
 * @brief Guard sustain evaluator implementation (IVP-70)
 */

#include "guard_evaluator.h"
#include "guard_functions.h"
#include "flight_director.h"

namespace rc {

// Phase bitmask helper
static constexpr uint8_t phase_bit(FlightPhase p) {
    return static_cast<uint8_t>(1U << static_cast<uint8_t>(p));
}

static void init_guard(GuardState& g, uint32_t sustain_ms,
                        uint32_t tick_ms, float threshold,
                        uint16_t signal, uint8_t valid_phases) {
    g.sustain_count = 0;
    g.sustain_required = sustain_ms / tick_ms;
    g.threshold = threshold;
    g.signal = signal;
    g.valid_phases = valid_phases;
    g.fired = false;
}

void guard_evaluator_init(GuardEvaluator* ev,
                           const MissionProfile& profile,
                           uint32_t tick_period_ms) {
    if (tick_period_ms == 0) { tick_period_ms = 10; }

    auto* g = ev->guards;
    init_guard(g[static_cast<uint8_t>(GuardId::kLaunchAccel)],
               profile.launch_sustain_ms, tick_period_ms,
               profile.launch_accel_threshold, SIG_LAUNCH,
               phase_bit(FlightPhase::kArmed));

    init_guard(g[static_cast<uint8_t>(GuardId::kBurnoutAccel)],
               profile.burnout_sustain_ms, tick_period_ms,
               profile.burnout_accel_threshold, SIG_BURNOUT,
               phase_bit(FlightPhase::kBoost));

    init_guard(g[static_cast<uint8_t>(GuardId::kApogeeVelocity)],
               profile.apogee_sustain_ms, tick_period_ms,
               profile.apogee_velocity_threshold, SIG_APOGEE,
               phase_bit(FlightPhase::kCoast));

    init_guard(g[static_cast<uint8_t>(GuardId::kBaroPeak)],
               profile.baro_peak_sustain_ms, tick_period_ms,
               0.0f, SIG_APOGEE,
               phase_bit(FlightPhase::kCoast));

    init_guard(g[static_cast<uint8_t>(GuardId::kMainDeploy)],
               profile.main_deploy_sustain_ms, tick_period_ms,
               profile.main_deploy_altitude_m, SIG_MAIN_DEPLOY,
               phase_bit(FlightPhase::kDrogueDescent));

    init_guard(g[static_cast<uint8_t>(GuardId::kStationary)],
               profile.landing_sustain_ms, tick_period_ms,
               profile.landing_velocity_threshold, SIG_LANDING,
               phase_bit(FlightPhase::kDrogueDescent) |
               phase_bit(FlightPhase::kMainDescent));

    ev->last_phase = FlightPhase::kIdle;
}

void guard_evaluator_reset(GuardEvaluator* ev) {
    for (uint8_t i = 0; i < static_cast<uint8_t>(GuardId::kCount); ++i) {
        ev->guards[i].sustain_count = 0;
        ev->guards[i].fired = false;
        ev->guards[i].sustained = false;
    }
}

bool guard_evaluator_is_sustained(const GuardEvaluator* ev, GuardId id) {
    return ev->guards[static_cast<uint8_t>(id)].sustained;
}

uint16_t guard_evaluator_tick(GuardEvaluator* ev,
                                   FlightPhase phase,
                                   const FusedState& fused,
                                   float accel_z,
                                   float accel_mag) {
    // Phase transition: reset all counters and fired flags
    if (phase != ev->last_phase) {
        guard_evaluator_reset(ev);
        ev->last_phase = phase;
    }

    uint8_t phase_mask = phase_bit(phase);

    // Evaluate each guard condition using thresholds from MissionProfile
    bool conditions[static_cast<uint8_t>(GuardId::kCount)];

    const auto& g = ev->guards;
    conditions[static_cast<uint8_t>(GuardId::kLaunchAccel)] =
        guard_launch_accel(accel_z,
                           g[static_cast<uint8_t>(GuardId::kLaunchAccel)].threshold);
    conditions[static_cast<uint8_t>(GuardId::kBurnoutAccel)] =
        guard_burnout_accel(accel_mag,
                            g[static_cast<uint8_t>(GuardId::kBurnoutAccel)].threshold);
    conditions[static_cast<uint8_t>(GuardId::kApogeeVelocity)] =
        guard_apogee_velocity(fused.vel_d,
                              g[static_cast<uint8_t>(GuardId::kApogeeVelocity)].threshold);
    conditions[static_cast<uint8_t>(GuardId::kBaroPeak)] =
        guard_baro_peak(fused.baro_vvel);
    conditions[static_cast<uint8_t>(GuardId::kMainDeploy)] =
        guard_main_deploy_altitude(fused.baro_alt_agl,
                                   g[static_cast<uint8_t>(GuardId::kMainDeploy)].threshold);
    conditions[static_cast<uint8_t>(GuardId::kStationary)] =
        guard_stationary(fused.vel_n, fused.vel_e, fused.vel_d,
                         g[static_cast<uint8_t>(GuardId::kStationary)].threshold);

    // Process sustain counters
    for (uint8_t i = 0; i < static_cast<uint8_t>(GuardId::kCount); ++i) {
        GuardState& gs = ev->guards[i];

        // Skip if not valid for current phase or already fired
        if ((gs.valid_phases & phase_mask) == 0 || gs.fired) {
            gs.sustained = false;
            continue;
        }

        if (conditions[i]) {
            ++gs.sustain_count;
            if (gs.sustain_count >= gs.sustain_required) {
                gs.sustained = true;
                // Managed guards: don't auto-dispatch, combinator decides
                if (kGuardManaged[i]) {
                    continue;
                }
                // Unmanaged guards: auto-dispatch (IVP-70 behavior)
                gs.fired = true;
                return gs.signal;
            }
        } else {
            gs.sustain_count = 0;
            gs.sustained = false;
        }
    }

    return SIG_MAX;  // No unmanaged guard fired
}

} // namespace rc
