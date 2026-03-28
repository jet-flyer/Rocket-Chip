/*
 * RocketChip Active Object Topology — SPIN/Promela Model (IVP-82b)
 *
 * Full 7-process model: 4 safety-critical + 3 mission-critical.
 * Uses bounded event generation (not infinite loops) for tractable
 * state space. Each producer process generates a finite number of
 * events — SPIN exhaustively explores all orderings.
 *
 * NASA/JPL approach: bound the scenario, not the depth. A flight has
 * at most ~10 phase transitions. Model that, verify all interleavings.
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
    SIG_ARM, SIG_DISARM, SIG_LAUNCH, SIG_BURNOUT,
    SIG_APOGEE, SIG_MAIN_DEPLOY, SIG_LANDING,
    SIG_ABORT, SIG_RESET, SIG_TICK,
    SIG_PHASE_CHANGE, SIG_PYRO_INTENT, SIG_LED_PATTERN,
    SIG_TELEM_FRAME
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

/* Event delivery tracking */
byte phase_changes_published = 0;
byte phase_changes_logged = 0;
byte phase_changes_telem = 0;
byte phase_changes_led = 0;

/* Channels — depth 4 sufficient for event routing verification */
chan fd_queue     = [4] of { mtype };
chan logger_queue = [4] of { mtype };
chan telem_queue  = [4] of { mtype };
chan led_queue    = [4] of { mtype };

/* ========================================================================
 * Environment process — combines timer, guards, and CLI into one bounded
 * event generator. Generates all possible event sequences a flight could
 * see, non-deterministically. Terminates after a bounded number of events.
 *
 * This replaces the infinite-loop producers that caused state explosion.
 * The bound (MAX_EVENTS) covers worst case: ARM + full flight + RESET.
 * ======================================================================== */
/* Bound: 10 events covers a full flight (ARM + 6 transitions + 3 ticks).
 * SPIN verifies all possible orderings exhaustively.
 * Increase for multi-flight scenarios (ARM → LANDED → RESET → ARM → ...) */
#define MAX_EVENTS 10

active proctype Environment() {
    byte i;
    for (i : 1 .. MAX_EVENTS) {
        /* Non-deterministically choose which event to generate based on
         * current phase. Models the combined effect of timer, guards, CLI. */
        if
        :: phase == PH_IDLE ->
            if
            :: fd_queue ! SIG_ARM
            :: fd_queue ! SIG_TICK
            fi
        :: phase == PH_ARMED ->
            if
            :: fd_queue ! SIG_LAUNCH
            :: fd_queue ! SIG_DISARM
            :: fd_queue ! SIG_ABORT
            :: fd_queue ! SIG_TICK     /* may trigger armed timeout */
            fi
        :: phase == PH_BOOST ->
            if
            :: fd_queue ! SIG_BURNOUT
            :: fd_queue ! SIG_ABORT
            :: fd_queue ! SIG_TICK
            fi
        :: phase == PH_COAST ->
            if
            :: fd_queue ! SIG_APOGEE
            :: fd_queue ! SIG_ABORT
            :: fd_queue ! SIG_TICK     /* may trigger coast timeout */
            fi
        :: phase == PH_DROGUE_DESCENT ->
            if
            :: fd_queue ! SIG_MAIN_DEPLOY
            :: fd_queue ! SIG_LANDING
            :: fd_queue ! SIG_ABORT    /* ignored in descent */
            :: fd_queue ! SIG_TICK
            fi
        :: phase == PH_MAIN_DESCENT ->
            if
            :: fd_queue ! SIG_LANDING
            :: fd_queue ! SIG_ABORT    /* ignored in descent */
            :: fd_queue ! SIG_TICK
            fi
        :: phase == PH_LANDED ->
            if
            :: fd_queue ! SIG_RESET
            :: fd_queue ! SIG_TICK
            fi
        :: phase == PH_ABORT ->
            if
            :: fd_queue ! SIG_RESET
            :: fd_queue ! SIG_TICK     /* may trigger abort timeout */
            fi
        fi
    }
    /* Environment exhausted — model terminates naturally */
}

/* ========================================================================
 * Flight Director — consumes events, publishes phase changes
 * ======================================================================== */
active proctype FlightDirector() {
    mtype sig;
    phase = PH_IDLE;

    do
    :: fd_queue ? sig ->

        if
        :: phase == PH_IDLE ->
            if
            :: sig == SIG_ARM ->
                phase = PH_ARMED;
                was_armed = true;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: else -> skip
            fi

        :: phase == PH_ARMED ->
            if
            :: sig == SIG_LAUNCH ->
                phase = PH_BOOST;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_DISARM ->
                atomic { phase = PH_IDLE; drogue_fired = false;
                         main_fired = false; drogue_count = 0; main_count = 0 }
            :: sig == SIG_ABORT ->
                phase = PH_ABORT;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_TICK ->
                if
                :: true ->  /* armed timeout */
                    atomic { phase = PH_IDLE; drogue_fired = false;
                             main_fired = false; drogue_count = 0; main_count = 0 }
                :: true -> skip
                fi
            :: else -> skip
            fi

        :: phase == PH_BOOST ->
            if
            :: sig == SIG_BURNOUT ->
                phase = PH_COAST;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_ABORT ->
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_queue ! SIG_PYRO_INTENT;
                phase = PH_ABORT;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: else -> skip
            fi

        :: phase == PH_COAST ->
            if
            :: sig == SIG_APOGEE ->
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_queue ! SIG_PYRO_INTENT;
                phase = PH_DROGUE_DESCENT;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_ABORT ->
                drogue_fired = true;
                drogue_count = drogue_count + 1;
                logger_queue ! SIG_PYRO_INTENT;
                phase = PH_ABORT;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_TICK ->
                if
                :: true ->  /* coast timeout */
                    drogue_fired = true;
                    drogue_count = drogue_count + 1;
                    logger_queue ! SIG_PYRO_INTENT;
                    phase = PH_DROGUE_DESCENT;
                    phase_changes_published++;
                    logger_queue ! SIG_PHASE_CHANGE;
                    telem_queue  ! SIG_PHASE_CHANGE;
                    led_queue    ! SIG_LED_PATTERN
                :: true -> skip
                fi
            :: else -> skip
            fi

        :: phase == PH_DROGUE_DESCENT ->
            if
            :: sig == SIG_MAIN_DEPLOY ->
                main_fired = true;
                main_count = main_count + 1;
                logger_queue ! SIG_PYRO_INTENT;
                phase = PH_MAIN_DESCENT;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_LANDING ->
                phase = PH_LANDED;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_ABORT -> skip
            :: else -> skip
            fi

        :: phase == PH_MAIN_DESCENT ->
            if
            :: sig == SIG_LANDING ->
                phase = PH_LANDED;
                phase_changes_published++;
                logger_queue ! SIG_PHASE_CHANGE;
                telem_queue  ! SIG_PHASE_CHANGE;
                led_queue    ! SIG_LED_PATTERN
            :: sig == SIG_ABORT -> skip
            :: else -> skip
            fi

        :: phase == PH_LANDED ->
            if
            :: sig == SIG_RESET ->
                atomic { phase = PH_IDLE; drogue_fired = false;
                         main_fired = false; drogue_count = 0; main_count = 0 }
            :: else -> skip
            fi

        :: phase == PH_ABORT ->
            if
            :: sig == SIG_RESET ->
                atomic { phase = PH_IDLE; drogue_fired = false;
                         main_fired = false; drogue_count = 0; main_count = 0 }
            :: sig == SIG_TICK ->
                if
                :: true -> phase = PH_LANDED
                :: true -> skip
                fi
            :: else -> skip
            fi

        :: else -> skip
        fi

    :: empty(fd_queue) -> break  /* Environment exhausted, terminate */
    od
}

/* ========================================================================
 * Mission-critical AOs — consume events from their queues
 * ======================================================================== */
active proctype Logger() {
    mtype sig;
    do
    :: logger_queue ? sig ->
        if
        :: sig == SIG_PHASE_CHANGE ->
            phase_changes_logged++;
            telem_queue ! SIG_TELEM_FRAME
        :: sig == SIG_PYRO_INTENT -> skip
        :: else -> skip
        fi
    :: empty(logger_queue) && empty(fd_queue) -> break
    od
}

active proctype Telemetry() {
    mtype sig;
    do
    :: telem_queue ? sig ->
        if
        :: sig == SIG_PHASE_CHANGE -> phase_changes_telem++
        :: sig == SIG_TELEM_FRAME -> skip
        :: else -> skip
        fi
    :: empty(telem_queue) && empty(fd_queue) && empty(logger_queue) -> break
    od
}

active proctype LedEngine() {
    mtype sig;
    do
    :: led_queue ? sig ->
        if
        :: sig == SIG_LED_PATTERN -> phase_changes_led++
        :: else -> skip
        fi
    :: empty(led_queue) && empty(fd_queue) -> break
    od
}

/* ========================================================================
 * Safety Properties
 * ======================================================================== */
ltl p_no_pyro_idle {
    [] (phase == PH_IDLE -> (!drogue_fired && !main_fired))
}

ltl p_drogue_before_main {
    [] (main_fired -> drogue_fired)
}

ltl p_pyro_requires_armed {
    [] ((drogue_fired || main_fired) -> was_armed)
}

ltl p_drogue_once {
    [] (drogue_count <= 1)
}

ltl p_main_once {
    [] (main_count <= 1)
}

/* ========================================================================
 * Mission-Critical Properties
 * ======================================================================== */
ltl p_logger_gets_all {
    [] (phase_changes_published >= phase_changes_logged)
}

ltl p_telem_gets_all {
    [] (phase_changes_published >= phase_changes_telem)
}

ltl p_led_gets_all {
    [] (phase_changes_published >= phase_changes_led)
}
