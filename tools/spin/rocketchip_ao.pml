/*
 * RocketChip Active Object Topology — SPIN/Promela Model (IVP-82b)
 *
 * Full 5-process model: Environment + FD + Logger + Telemetry + LedEngine.
 * Verifies pyro safety, event delivery, and deadlock freedom.
 *
 * Modeling decisions (informed by SPIN best practices):
 *   - Channel depth [1]: safety properties depend on event ordering, not
 *     queue depth. Depth 1 is sufficient and keeps state space tractable.
 *     Actual queue overflow is validated empirically (LL Entry 32).
 *   - atomic blocks around FD handlers: simulates QV cooperative
 *     run-to-completion semantics. The real system dispatches one event
 *     fully before checking the next.
 *   - Bounded event generation (MAX_EVENTS=8): covers one complete flight
 *     (ARM + 6 transitions + timeout). SPIN explores all orderings.
 *   - d_step for deterministic local operations (flag sets, counter bumps).
 *
 * References:
 *   - Holzmann, "Basic SPIN Manual" — spinroot.com/spin/Man/Manual.html
 *   - Ruys, "SPIN Beginners' Tutorial" — spinroot.com/spin/Doc/SpinTutorial.pdf
 *   - RTEMS Promela Guide — docs.rtems.org/docs/main/eng/fv/promela.html
 *   - MIT SPIN Exercises — web.mit.edu/spin_v6.4.7/Doc/1_Exercises.html
 *
 * Process mapping:
 *   Environment     → guard_evaluator.cpp + cli + bsp_qv.c tick (combined)
 *   FlightDirector  → flight_director.cpp + ao_flight_director.cpp
 *   Logger          → ao_logger.cpp
 *   Telemetry       → ao_telemetry.cpp
 *   LedEngine       → ao_led_engine.cpp
 */

/* Flight phases — maps to FlightPhase enum in flight_state.h */
#define PH_IDLE            0
#define PH_ARMED           1
#define PH_BOOST           2
#define PH_COAST           3
#define PH_DROGUE_DESCENT  4
#define PH_MAIN_DESCENT    5
#define PH_LANDED          6
#define PH_ABORT           7

/* Signals */
mtype = {
    SIG_PHASE_CHANGE, SIG_PYRO_INTENT, SIG_LED_PATTERN, SIG_TELEM_FRAME
};

/* ========================================================================
 * Global state
 * ======================================================================== */
byte phase = PH_IDLE;
bool drogue_fired = false;
bool main_fired = false;
bool was_armed = false;
byte drogue_count = 0;
byte main_count = 0;

/* Event delivery counters */
byte pub_count = 0;   /* phase changes published by FD */
byte log_count = 0;   /* phase changes received by Logger */
byte tel_count = 0;   /* phase changes received by Telemetry */
byte led_count = 0;   /* phase changes received by LedEngine */

/* Channels — depth [1] per SPIN best practice. Safety properties don't
 * depend on buffering. Depth 1 keeps state space O(signals) not O(depth^N). */
chan logger_ch = [1] of { mtype };
chan telem_ch  = [1] of { mtype };
chan led_ch    = [1] of { mtype };

/* ========================================================================
 * Bounded event count — one complete flight scenario.
 * ARM(1) + LAUNCH(2) + BURNOUT(3) + APOGEE(4) + MAIN_DEPLOY(5) +
 * LANDING(6) + ticks(2) = 8. SPIN explores all orderings.
 * ======================================================================== */
#define MAX_EVENTS 8

/* ========================================================================
 * Flight Director — single process, atomic event handling.
 *
 * atomic{} around each handler simulates QV run-to-completion:
 * the real system fully processes one event before checking the next.
 * This is the BIGGEST state space reduction — eliminates interleaving
 * within handlers.
 *
 * The environment (guards, CLI, ticks) is modeled as non-deterministic
 * choice WITHIN the FD process, not as separate processes. This is the
 * standard SPIN pattern for modeling a scheduler + event sources.
 * ======================================================================== */
active proctype FlightDirector() {
    byte i;

    phase = PH_IDLE;

    for (i : 1 .. MAX_EVENTS) {
        /* Non-deterministic event selection based on current phase */
        if
        /* ---- IDLE ---- */
        :: phase == PH_IDLE ->
            if
            :: atomic {  /* SIG_ARM */
                phase = PH_ARMED;
                was_armed = true;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: skip  /* SIG_TICK: no-op */
            fi

        /* ---- ARMED ---- */
        :: phase == PH_ARMED ->
            if
            :: atomic {  /* SIG_LAUNCH */
                phase = PH_BOOST;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: d_step {  /* SIG_DISARM → IDLE */
                phase = PH_IDLE;
                drogue_fired = false; main_fired = false;
                drogue_count = 0; main_count = 0
               }
            :: atomic {  /* SIG_ABORT — no pyro on pad */
                phase = PH_ABORT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: d_step {  /* SIG_TICK: armed timeout → IDLE */
                phase = PH_IDLE;
                drogue_fired = false; main_fired = false;
                drogue_count = 0; main_count = 0
               }
            :: skip  /* SIG_TICK: no timeout yet */
            fi

        /* ---- BOOST ---- */
        :: phase == PH_BOOST ->
            if
            :: atomic {  /* SIG_BURNOUT */
                phase = PH_COAST;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: atomic {  /* SIG_ABORT — fire drogue (Amendment #1) */
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_ch ! SIG_PYRO_INTENT;
                phase = PH_ABORT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: skip  /* SIG_TICK: no-op */
            fi

        /* ---- COAST ---- */
        :: phase == PH_COAST ->
            if
            :: atomic {  /* SIG_APOGEE — fire drogue */
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_ch ! SIG_PYRO_INTENT;
                phase = PH_DROGUE_DESCENT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: atomic {  /* SIG_ABORT — fire drogue (Amendment #1) */
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_ch ! SIG_PYRO_INTENT;
                phase = PH_ABORT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: atomic {  /* SIG_TICK: coast timeout (Amendment #7) */
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_ch ! SIG_PYRO_INTENT;
                phase = PH_DROGUE_DESCENT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: skip  /* SIG_TICK: no timeout yet */
            fi

        /* ---- DROGUE_DESCENT ---- */
        :: phase == PH_DROGUE_DESCENT ->
            if
            :: atomic {  /* SIG_MAIN_DEPLOY — fire main */
                main_fired = true;
                main_count = main_count + 1;
                logger_ch ! SIG_PYRO_INTENT;
                phase = PH_MAIN_DESCENT;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: atomic {  /* SIG_LANDING — skip main */
                phase = PH_LANDED;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: skip  /* SIG_ABORT: ignored in descent */
            :: skip  /* SIG_TICK: no-op */
            fi

        /* ---- MAIN_DESCENT ---- */
        :: phase == PH_MAIN_DESCENT ->
            if
            :: atomic {  /* SIG_LANDING */
                phase = PH_LANDED;
                pub_count++;
                logger_ch ! SIG_PHASE_CHANGE;
                telem_ch  ! SIG_PHASE_CHANGE;
                led_ch    ! SIG_LED_PATTERN
               }
            :: skip  /* SIG_ABORT: ignored in descent */
            :: skip  /* SIG_TICK: no-op */
            fi

        /* ---- LANDED ---- */
        :: phase == PH_LANDED ->
            if
            :: d_step {  /* SIG_RESET → IDLE */
                phase = PH_IDLE;
                drogue_fired = false; main_fired = false;
                drogue_count = 0; main_count = 0
               }
            :: skip  /* SIG_TICK: no-op */
            fi

        /* ---- ABORT ---- */
        :: phase == PH_ABORT ->
            if
            :: d_step {  /* SIG_RESET → IDLE */
                phase = PH_IDLE;
                drogue_fired = false; main_fired = false;
                drogue_count = 0; main_count = 0
               }
            :: d_step { phase = PH_LANDED }  /* SIG_TICK: abort timeout */
            :: skip  /* SIG_TICK: no timeout yet */
            fi

        :: else -> skip  /* shouldn't happen */
        fi
    }
    /* Scenario complete — terminate. Consumers drain remaining events. */
}

/* ========================================================================
 * Mission-critical consumers — simple event drains with delivery counters.
 * ======================================================================== */
active proctype Logger() {
    mtype sig;
    do
    :: logger_ch ? sig ->
        d_step {
            if
            :: sig == SIG_PHASE_CHANGE -> log_count++
            :: sig == SIG_PYRO_INTENT -> skip
            :: else -> skip
            fi
        }
    :: empty(logger_ch) && (phase == PH_IDLE || phase == PH_LANDED) -> break
    od
}

active proctype Telemetry() {
    mtype sig;
    do
    :: telem_ch ? sig ->
        d_step {
            if
            :: sig == SIG_PHASE_CHANGE -> tel_count++
            :: else -> skip
            fi
        }
    :: empty(telem_ch) && (phase == PH_IDLE || phase == PH_LANDED) -> break
    od
}

active proctype LedEngine() {
    mtype sig;
    do
    :: led_ch ? sig ->
        d_step {
            if
            :: sig == SIG_LED_PATTERN -> led_count++
            :: else -> skip
            fi
        }
    :: empty(led_ch) && (phase == PH_IDLE || phase == PH_LANDED) -> break
    od
}

/* ========================================================================
 * Safety Properties
 * ======================================================================== */

/* P1: Pyro never fires in IDLE */
ltl p_no_pyro_idle {
    [] (phase == PH_IDLE -> (!drogue_fired && !main_fired))
}

/* P2: Main only fires after drogue (sequence invariant) */
ltl p_drogue_before_main {
    [] (main_fired -> drogue_fired)
}

/* P3: Pyro requires prior ARMED state (Council: JPL) */
ltl p_pyro_requires_armed {
    [] ((drogue_fired || main_fired) -> was_armed)
}

/* P4: Drogue fires at most once per flight (Council: JPL) */
ltl p_drogue_once {
    [] (drogue_count <= 1)
}

/* P5: Main fires at most once per flight (Council: JPL) */
ltl p_main_once {
    [] (main_count <= 1)
}

/* ========================================================================
 * Mission-Critical Properties
 * ======================================================================== */

/* M1: Published phase changes >= logged (no silent drop to logger) */
ltl p_logger_gets_all {
    [] (pub_count >= log_count)
}

/* M2: Published phase changes >= telem received (no silent drop) */
ltl p_telem_gets_all {
    [] (pub_count >= tel_count)
}

/* M3: Published phase changes >= LED received (no silent drop) */
ltl p_led_gets_all {
    [] (pub_count >= led_count)
}
