/* rocketchip_boot.pml — Cross-core boot handshake (R-12)
 *
 * Phase 8 Category 3 of the 2026-05-07 master standards audit. Closes
 * the formal-verification gap for R-1 / BM-2: the Core 0 / Core 1 boot
 * handshake via g_startSensorPhase + g_core1LockoutReady was bounded
 * by code (R-1) but had no formal proof. R-12 is the SPIN model.
 *
 * Firmware abstracted (src/core1/sensor_core1.cpp:482, src/main.cpp:340):
 *
 *   Core 0 path (after init):
 *     1. g_sensorPhaseActive = true
 *     2. g_startSensorPhase.store(true)
 *     3. wait until g_core1LockoutReady.load() == true
 *     4. proceed (flash ops, AO scheduling, etc.)
 *
 *   Core 1 path (core1_entry):
 *     1. mpu_setup_stack_guard()
 *     2. multicore_lockout_victim_init()
 *     3. g_core1LockoutReady.store(true)
 *     4. Vehicle: bounded wait (10s ceiling) for g_startSensorPhase;
 *        on timeout, crash_record_capture(kCrashReasonCore1BootWait)
 *        (NORETURN, fires NVIC_SystemReset).
 *     5. Station/Relay: unbounded wait — Holzmann scheduler exemption
 *        (Core 1 has no sensor work in these roles). Modeled here by
 *        a non-deterministic role flag and the "flag never set" branch.
 *     6. On flag set: core1_sensor_loop() (sensor read phase begins).
 *
 * Two LTL properties (council scoping 2026-05-07 Phase 6):
 *   P_NO_PREMATURE_SENSOR_READ: [](core1_in_sensor_loop ->
 *                                  was_g_startSensorPhase_set)
 *     Safety — Core 1 can only enter the sensor loop after Core 0 set
 *     the flag. Covers Vehicle and Station/Relay (vacuously true on
 *     Station/Relay since Core 1 never enters the loop there).
 *
 *   P_VEHICLE_CORE1_EVENTUALLY_PROCEEDS: <>(core1_in_sensor_loop ||
 *                                            core1_timeout_reset)
 *     Liveness (Vehicle path) — Core 1 either reaches the sensor loop
 *     OR fires the bounded-wait timeout (crash_record_capture + reset).
 *     Requires weak fairness (-f). On Station/Relay we don't claim
 *     liveness — Core 1 is intentionally idle by construction; the
 *     Holzmann inverted-rule exemption applies (statically prove the
 *     loop CANNOT terminate on these roles, which is what `kRole !=
 *     kVehicle` does in firmware).
 *
 * What's NOT modeled (intentional):
 *   - The MPU stack guard / multicore_lockout_victim_init internals.
 *     Those are SDK primitives; their correctness is assumed.
 *   - The post-handshake sensor loop body. R-12's job is the handshake
 *     itself; the sensor loop is covered by rocketchip_ao.pml.
 *   - The actual reset-vector flow after crash_record_capture. R-3's
 *     fault-handler design is covered by docs/decisions/
 *     FAULT_HANDLER_DESIGN.md; we model only the precondition that the
 *     timeout branch DOES fire when the flag stays unset.
 *
 * To run (from a Cygwin terminal at the repo root):
 *   cd tools/spin
 *   spin -a rocketchip_boot.pml
 *   gcc -O2 -o pan_boot pan.c
 *   ./pan_boot -a -N p_no_premature_sensor_read
 *   ./pan_boot -a -f -N p_vehicle_core1_eventually_proceeds
 *
 * Or via the master gate:
 *   bash tools/spin/run_stage_o_ao_spin.sh
 */

/* =============================================================
 * Shared state (atomics in firmware; bools here)
 * ============================================================= */

bool g_startSensorPhase   = false;   /* main.cpp:342 / sensor_core1.cpp:512 */
bool g_core1LockoutReady  = false;   /* sensor_core1.cpp:488 / main.cpp:347 */

/* Role selection: non-deterministic choice between Vehicle (sets flag)
 * and Station/Relay (never sets flag). Models both cases in one model. */
bool is_vehicle = false;

/* Model-only flags for LTL properties. */
bool core1_in_sensor_loop = false;
bool core1_timeout_reset  = false;
bool was_g_startSensorPhase_set = false;  /* latch: once set, stays set */

/* Bounded counter for the Vehicle wait loop. Mirrors
 * kCore1BootWaitMaxIters in sensor_core1.cpp:510 — the static bound
 * the firmware compiles to. Promela needs a finite range; the actual
 * value (1000) doesn't change the verification, so use a small number
 * for state-space economy. */
#define CORE1_WAIT_MAX_ITERS 5

/* =============================================================
 * Core 0 process
 * ============================================================= */

active proctype Core0() {
    /* Non-deterministic role choice at boot. */
    if
    :: is_vehicle = true;
    :: is_vehicle = false;
    fi;

    /* Init sequence runs in either role. (Abstracted away — not
     * relevant to handshake correctness.) */

    if
    :: is_vehicle ->
        /* Vehicle: set the flag, then wait for Core 1 lockout ready. */
        atomic {
            g_startSensorPhase = true;
            was_g_startSensorPhase_set = true;
        }
        /* Then: wait until Core 1 finished multicore_lockout_victim_init.
         * Bounded by Core 1's own forward progress. */
        do
        :: g_core1LockoutReady -> break;
        :: !g_core1LockoutReady -> skip;
        od;

    :: !is_vehicle ->
        /* Station/Relay: never set the flag. Core 1 idles forever
         * (Holzmann exemption). */
        skip;
    fi;
}

/* =============================================================
 * Core 1 process
 * ============================================================= */

active proctype Core1() {
    byte iter;

    /* Step 1: register as multicore_lockout victim. */
    g_core1LockoutReady = true;

    /* Step 2: wait for g_startSensorPhase. Two paths by role. */
    if
    :: is_vehicle ->
        /* Vehicle: bounded wait. CORE1_WAIT_MAX_ITERS abstracts the
         * 1000-iter limit in firmware. */
        iter = 0;
        do
        :: iter < CORE1_WAIT_MAX_ITERS && g_startSensorPhase ->
            /* Flag set within bound — break and enter sensor loop. */
            break;
        :: iter < CORE1_WAIT_MAX_ITERS && !g_startSensorPhase ->
            iter = iter + 1;
        :: iter >= CORE1_WAIT_MAX_ITERS && !g_startSensorPhase ->
            /* Timeout — fires crash_record_capture, NORETURN.
             * Model the reset as a terminal state. */
            core1_timeout_reset = true;
            goto core1_end;
        :: iter >= CORE1_WAIT_MAX_ITERS && g_startSensorPhase ->
            /* Edge case: flag set exactly on the timeout boundary.
             * Firmware re-checks the flag after the loop (line 517);
             * if set, the timeout branch doesn't fire. Model the same:
             * proceed to sensor loop. */
            break;
        od;
        /* Flag was set — enter sensor loop. */
        core1_in_sensor_loop = true;

    :: !is_vehicle ->
        /* Station/Relay: unbounded wait. Holzmann inverted-rule
         * exemption — by construction, Core 0 never sets the flag
         * on these roles, so this loop is intentionally non-
         * terminating. Model as a perpetual idle.
         *
         * Note: the SPIN verifier will accept this as a legitimate
         * non-progress cycle when role==Station. P_NO_PREMATURE_
         * SENSOR_READ is vacuously true (core1_in_sensor_loop never
         * gets set). P_VEHICLE_CORE1_EVENTUALLY_PROCEEDS is gated on
         * is_vehicle so it doesn't claim liveness here. */
        do
        :: g_startSensorPhase -> break;   /* won't happen */
        :: !g_startSensorPhase -> skip;
        od;
        /* Unreachable on Station/Relay by construction. */
        core1_in_sensor_loop = true;
    fi;

core1_end:
    skip;
}

/* =============================================================
 * LTL properties
 * ============================================================= */

/* P_NO_PREMATURE_SENSOR_READ:
 * Safety — Core 1 only enters the sensor loop after Core 0 has set
 * g_startSensorPhase. Globally true. */
ltl p_no_premature_sensor_read {
    [] (core1_in_sensor_loop -> was_g_startSensorPhase_set)
}

/* P_VEHICLE_CORE1_EVENTUALLY_PROCEEDS:
 * Liveness — on Vehicle role, Core 1 eventually either reaches the
 * sensor loop or fires the timeout branch. Requires weak fairness.
 * Gated on is_vehicle so the Station/Relay scenario (where Core 1
 * never proceeds, by design) does not falsify the property. */
ltl p_vehicle_core1_eventually_proceeds {
    [] (is_vehicle -> <> (core1_in_sensor_loop || core1_timeout_reset))
}
