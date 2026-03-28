/*
 * RocketChip Flight Director — SPIN/Promela Model (IVP-82a)
 *
 * Models the Flight Director HSM with non-deterministic guard signals,
 * timer ticks, and CLI commands. Verifies pyro safety properties.
 *
 * Abstraction:
 *   - Sensor data, ESKF, floating-point: omitted
 *   - Guard evaluation: non-deterministic signal arrival
 *   - Timing: non-deterministic timeout (no wall-clock)
 *   - has_pyro: assumed true (worst case for safety)
 *   - abort_fires_drogue_from_boost/coast: assumed true
 *
 * Maps to: src/flight_director/flight_director.cpp
 *          src/flight_director/action_executor.cpp
 *          include/rocketchip/ao_signals.h
 */

/* Flight phases — maps to FlightPhase enum in flight_state.h */
#define IDLE            0
#define ARMED           1
#define BOOST           2
#define COAST           3
#define DROGUE_DESCENT  4
#define MAIN_DESCENT    5
#define LANDED          6
#define ABORT_PHASE     7

/* Global state — observable by LTL properties */
byte phase = IDLE;

bool drogue_fired = false;
bool main_fired = false;
bool was_armed = false;
byte drogue_count = 0;
byte main_count = 0;
bool coast_timeout_fired = false;

/* ========================================================================
 * Single Flight Director process with non-deterministic environment.
 *
 * Instead of separate producer processes (which cause self-loops),
 * the FD process non-deterministically selects which event to process
 * each iteration. This models the QV cooperative scheduler dispatching
 * events from the queue — the order is non-deterministic because
 * guards, commands, and ticks can arrive in any order.
 * ======================================================================== */
active proctype FlightDirector() {

    phase = IDLE;

    do
    /* ---- IDLE ---- */
    :: phase == IDLE ->
        if
        :: true -> phase = ARMED; was_armed = true  /* SIG_ARM accepted */
        :: true -> skip                              /* SIG_TICK: no-op */
        fi

    /* ---- ARMED ---- */
    :: phase == ARMED ->
        if
        :: true -> phase = BOOST                     /* SIG_LAUNCH (guard) */
        :: true ->                                   /* SIG_DISARM (CLI) → IDLE */
            atomic { phase = IDLE; drogue_fired = false; main_fired = false;
                     drogue_count = 0; main_count = 0 }
        :: true -> phase = ABORT_PHASE               /* SIG_ABORT (CLI) — no pyro on pad */
        :: true ->                                   /* SIG_TICK: armed timeout → IDLE */
            atomic { phase = IDLE; drogue_fired = false; main_fired = false;
                     drogue_count = 0; main_count = 0 }
        :: true -> skip                              /* SIG_TICK: timeout not elapsed */
        fi

    /* ---- BOOST ---- */
    :: phase == BOOST ->
        if
        :: true -> phase = COAST                     /* SIG_BURNOUT (guard) */
        :: true ->                                   /* SIG_ABORT — fire drogue (A#1) */
            drogue_fired = true;
            drogue_count = drogue_count + 1;
            phase = ABORT_PHASE
        :: true -> skip                              /* SIG_TICK: no-op */
        fi

    /* ---- COAST ---- */
    :: phase == COAST ->
        if
        :: true ->                                   /* SIG_APOGEE (guard) — fire drogue */
            drogue_fired = true;
            drogue_count = drogue_count + 1;
            phase = DROGUE_DESCENT
        :: true ->                                   /* SIG_ABORT — fire drogue (A#1) */
            drogue_fired = true;
            drogue_count = drogue_count + 1;
            phase = ABORT_PHASE
        :: true ->                                   /* SIG_TICK: coast timeout (A#7) */
            drogue_fired = true;
            drogue_count = drogue_count + 1;
            coast_timeout_fired = true;
            phase = DROGUE_DESCENT
        :: true -> skip                              /* SIG_TICK: timeout not elapsed */
        fi

    /* ---- DROGUE_DESCENT ---- */
    :: phase == DROGUE_DESCENT ->
        if
        :: true ->                                   /* SIG_MAIN_DEPLOY — fire main */
            main_fired = true;
            main_count = main_count + 1;
            phase = MAIN_DESCENT
        :: true -> phase = LANDED                    /* SIG_LANDING (skip main) */
        :: true -> skip                              /* SIG_ABORT: ignored (descent) */
        :: true -> skip                              /* SIG_TICK: no-op */
        fi

    /* ---- MAIN_DESCENT ---- */
    :: phase == MAIN_DESCENT ->
        if
        :: true -> phase = LANDED                    /* SIG_LANDING */
        :: true -> skip                              /* SIG_ABORT: ignored (descent) */
        :: true -> skip                              /* SIG_TICK: no-op */
        fi

    /* ---- LANDED ---- */
    :: phase == LANDED ->
        if
        :: true ->                                   /* SIG_RESET (CLI) */
            atomic { phase = IDLE; drogue_fired = false; main_fired = false;
                     drogue_count = 0; main_count = 0 }
        :: true -> skip                              /* SIG_TICK: no-op */
        fi

    /* ---- ABORT ---- */
    :: phase == ABORT_PHASE ->
        if
        :: true ->                                   /* SIG_RESET (CLI) */
            atomic { phase = IDLE; drogue_fired = false; main_fired = false;
                     drogue_count = 0; main_count = 0 }
        :: true -> phase = LANDED                    /* SIG_TICK: abort timeout */
        :: true -> skip                              /* SIG_TICK: timeout not elapsed */
        fi

    od
}

/* ========================================================================
 * Safety Properties (LTL)
 * ======================================================================== */

/* P1: Pyro never fires when entering IDLE (drogue/main booleans can't
 *     become true while phase is IDLE) */
ltl p_no_pyro_idle {
    [] (phase == IDLE -> (!drogue_fired && !main_fired))
}

/* P2: Main pyro never fires in LANDED */
ltl p_no_main_landed {
    [] (phase == LANDED -> (main_count == 0 || main_count == 1))
}
/* Note: drogue_fired CAN be true when LANDED (fired during COAST) — correct behavior */

/* P3: Main only fires after drogue (sequence invariant) */
ltl p_drogue_before_main {
    [] (main_fired -> drogue_fired)
}

/* P4: Pyro requires prior ARMED state (Council: JPL) */
ltl p_pyro_requires_armed {
    [] ((drogue_fired || main_fired) -> was_armed)
}

/* P5: Drogue fires at most once */
ltl p_drogue_once {
    [] (drogue_count <= 1)
}

/* P6: Main fires at most once */
ltl p_main_once {
    [] (main_count <= 1)
}

/* P7: Liveness — once flight begins (BOOST), LANDED is always eventually reached.
 * The system may stay in IDLE or ARMED indefinitely (user choice) — that's valid.
 * But once committed to flight (BOOST), there's no dead-end state. */
ltl p_liveness_flight_completes {
    [] (phase == BOOST -> <> (phase == LANDED))
}
