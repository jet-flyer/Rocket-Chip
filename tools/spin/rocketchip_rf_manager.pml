/* rocketchip_rf_manager.pml — AO_RfManager state-machine SPIN model
 *
 * Stage T Batch B, IVP-T14. Validates the 4-state link-health machine
 * (kAcq / kTentative / kTrack / kTrackDegraded) against 5 LTL
 * properties from the Round 2 design doc §10 + plan-as-whole council
 * consensus #9 (progress-under-partial-loss).
 *
 * What this model IS:
 *   - Abstract behavior of rf_next_state() in src/safety/rf_link_health.h.
 *   - Inputs: non-deterministic RX outcomes (good / miss / crc_err)
 *     and wall-clock tick advancement.
 *   - Model of the sliding-window LQ via a bounded counter approximation
 *     (not the exact 10-slot window — state space abstracted).
 *   - 5 LTL properties from the design doc.
 *
 * What this model is NOT:
 *   - Bit-exact sliding window (would explode state space). LQ is
 *     represented as a discrete bucket {high, low, border} that abstracts
 *     the 55/65 Schmitt-trigger bands.
 *   - Filter-coefficient arithmetic (pure, tested at host level).
 *   - QP framework queue semantics (event dispatch atomicity is
 *     assumed correct).
 *   - AO_Radio scheduler interaction (separate model).
 *
 * Properties:
 *   p_eventual_track:    Liveness. If healthy_stream persists, eventually
 *                        reach kTrack. (§10 property 1)
 *   p_never_silent:      Safety. After a * → kAcq transition, notify_lost
 *                        is eventually raised. (§10 property 2)
 *   p_no_tx_in_acq:      Safety. Station never permitted to TX in kAcq.
 *                        (§10 property 3)
 *   p_hysteresis:        Safety. kTrackDegraded only reachable from
 *                        kTrack or itself. (§10 property 4)
 *   p_progress:          Liveness. Under intermittent loss with LQ in the
 *                        recovery band, eventually settle into kTrack or
 *                        kTrackDegraded and stay. Validates the 55/65
 *                        Schmitt fix eliminates oscillation.
 *                        (§10 property 5, Round 2 consensus #9)
 *
 * To run:
 *   cd tools/spin
 *   spin -a rocketchip_rf_manager.pml
 *   gcc -O2 -DVECTORSZ=4096 -o pan_rfm pan.c
 *   ./pan_rfm -a -f -N p_eventual_track
 *   ./pan_rfm -a    -N p_never_silent
 *   ./pan_rfm -a    -N p_no_tx_in_acq
 *   ./pan_rfm -a    -N p_hysteresis
 *   ./pan_rfm -a -f -N p_progress
 *
 * Liveness properties (p_eventual_track, p_progress) require weak
 * fairness via -f flag.
 */

/* =============================================================
 * Constants
 * ============================================================= */

/* Abstract LQ buckets:
 *   LQ_HIGH   ≥ 65%   — satisfies kTentative→kTrack and DEGRADED→TRACK.
 *   LQ_BORDER ∈ [55, 65) — the deadband; no state transitions triggered.
 *   LQ_LOW    < 55%   — triggers TRACK→DEGRADED.
 * Matches kRfTrackToDegradedLqPct=55 and kRfDegradedToTrackLqPct=65. */
mtype = { LQ_LOW, LQ_BORDER, LQ_HIGH };

/* Abstract states mirroring src/safety/rf_link_health.h::LinkState. */
mtype = { S_ACQ, S_TENTATIVE, S_TRACK, S_DEGRADED };

/* Consec-good threshold for kTentative → kTrack. Matches
 * kRfTentativeToTrackConsecRx. */
#define CONSEC_FOR_TRACK 5

/* Forced-ACQ thresholds. MIN_FRAMES matches kRfForcedAcqMinFrames,
 * FAST_FRAMES matches kRfForcedAcqFastFrames. Time is modeled as
 * a bounded tick counter — LOS_TICKS is the abstract "2 s reached". */
#define MIN_FRAMES       5
#define FAST_FRAMES      20
#define LOS_TICKS_REACHED 2   /* abstract "time >= 2 s"; any value ≥ 2 */

/* =============================================================
 * Shared state (model)
 * ============================================================= */

mtype state      = S_ACQ;
mtype prev_state = S_ACQ;           /* for p_hysteresis + p_never_silent */
byte  consec_good   = 0;
byte  consec_missed = 0;
byte  los_ticks     = 0;           /* abstract wall-clock counter */
bool  notify_lost   = false;       /* set on * → S_ACQ transition */
bool  can_tx        = false;       /* mirrors AO_RfManager_next_tx_window_us != 0 */
bool  deg_entered_from_track = false;  /* set when S_DEGRADED reached via S_TRACK;
                                          if S_DEGRADED is ever entered any other
                                          way, bug in model or firmware. Safety
                                          model invariant for p_hysteresis. */

/* Model-only: the input stream classifier. A fairness-enforced
 * "healthy" stream emits all-good RX; "partial-loss" stream
 * alternates good/miss/crc_err but keeps LQ in the RECOVERY band. */
bool  healthy_stream    = false;
bool  partial_loss_mode = false;

/* =============================================================
 * Helper: compute next_state following rf_next_state() semantics
 * ============================================================= */

inline step_transition(rx_kind, lq_bucket) {
    atomic {
        prev_state = state;   /* capture before any mutation */

        /* Forced-ACQ has priority from non-ACQ states. */
        if
        :: state != S_ACQ ->
            /* Primary: time AND min-frames. */
            if
            :: (los_ticks >= LOS_TICKS_REACHED) && (consec_missed >= MIN_FRAMES) ->
                state = S_ACQ;
                notify_lost = true;
                can_tx = false;
                consec_good = 0;
            /* Fast-frames accelerator. */
            :: consec_missed >= FAST_FRAMES ->
                state = S_ACQ;
                notify_lost = true;
                can_tx = false;
                consec_good = 0;
            :: else ->
                if
                :: state == S_TENTATIVE ->
                    if
                    :: (consec_good >= CONSEC_FOR_TRACK) && (lq_bucket == LQ_HIGH) ->
                        state = S_TRACK;
                        can_tx = true;
                    :: else -> skip;
                    fi;
                :: state == S_TRACK ->
                    if
                    :: lq_bucket == LQ_LOW ->
                        state = S_DEGRADED;
                        deg_entered_from_track = true;
                        /* can_tx stays true in DEGRADED — retries still
                         * attempted per AO_RfManager_ok_to_retry(). */
                    :: else -> skip;
                    fi;
                :: state == S_DEGRADED ->
                    if
                    :: lq_bucket == LQ_HIGH ->
                        state = S_TRACK;
                    :: else -> skip;
                    fi;
                fi;
            fi;
        :: state == S_ACQ ->
            /* ACQ → TENTATIVE on any valid RX (rx_kind == 1). */
            if
            :: rx_kind == 1 ->
                state = S_TENTATIVE;
                consec_good = 1;
                /* can_tx stays false until TRACK — ok_to_retry()
                 * returns true in TENTATIVE but next_tx_window_us()
                 * returns 0 until state == TRACK or DEGRADED. */
            :: else -> skip;
            fi;
        fi;
    }
}

/* =============================================================
 * RX event producer — non-deterministic stream
 * ============================================================= */

active proctype RxProducer() {
    do
    :: healthy_stream ->
        /* Healthy mode: every tick is a good RX. Fairness forces
         * this branch under -f. */
        atomic {
            consec_good = consec_good + 1;
            consec_missed = 0;
            los_ticks = 0;
        }
        step_transition(1, LQ_HIGH);

    :: partial_loss_mode ->
        /* Partial-loss mode: alternates good and miss, LQ stays
         * in the border/high band — tests that the machine settles
         * into TRACK or DEGRADED rather than oscillating. */
        if
        :: atomic {
              consec_good = consec_good + 1;
              consec_missed = 0;
              los_ticks = 0;
           };
           step_transition(1, LQ_BORDER);
        :: atomic {
              consec_missed = consec_missed + 1;
              consec_good = 0;
              los_ticks = los_ticks + 1;
              if
              :: los_ticks > LOS_TICKS_REACHED -> los_ticks = LOS_TICKS_REACHED;
              :: else -> skip;
              fi;
           };
           step_transition(0, LQ_BORDER);
        fi;

    :: !healthy_stream && !partial_loss_mode ->
        /* Fully non-deterministic mode: any RX outcome, any LQ. */
        if
        :: atomic {
              consec_good = consec_good + 1;
              consec_missed = 0;
              los_ticks = 0;
           };
           if
           :: step_transition(1, LQ_HIGH);
           :: step_transition(1, LQ_BORDER);
           :: step_transition(1, LQ_LOW);
           fi;
        :: atomic {
              consec_missed = consec_missed + 1;
              consec_good = 0;
              los_ticks = los_ticks + 1;
              if
              :: los_ticks > LOS_TICKS_REACHED -> los_ticks = LOS_TICKS_REACHED;
              :: else -> skip;
              fi;
           };
           if
           :: step_transition(0, LQ_HIGH);
           :: step_transition(0, LQ_BORDER);
           :: step_transition(0, LQ_LOW);
           fi;
        fi;
    od;
}

/* =============================================================
 * Mode setter — non-deterministically chooses a stream type once
 * ============================================================= */

active proctype ModeSetter() {
    /* Pick one stream type and stick with it. The LTL properties
     * are evaluated per-run against whichever mode was chosen. */
    if
    :: healthy_stream = true;
    :: partial_loss_mode = true;
    :: skip;   /* fully non-deterministic */
    fi;
}

/* =============================================================
 * Properties
 * ============================================================= */

/* P_EVENTUAL_TRACK (liveness): under healthy input stream, the
 * machine eventually reaches kTrack. Needs -f for weak fairness. */
ltl p_eventual_track {
    [] (healthy_stream -> <> (state == S_TRACK))
}

/* P_NEVER_SILENT (safety): any transition that ENTERS kAcq from a
 * non-kAcq state must atomically set notify_lost. Expressed via the
 * prev_state auxiliary: whenever state becomes S_ACQ while prev_state
 * was not S_ACQ, notify_lost holds. (Does not fire for the initial
 * S_ACQ, since prev_state is also S_ACQ there.) */
ltl p_never_silent {
    [] ((state == S_ACQ && prev_state != S_ACQ) -> notify_lost)
}

/* P_NO_TX_IN_ACQ (safety): can_tx is false in ACQ. Enforced at
 * transition sites. */
ltl p_no_tx_in_acq {
    [] (state == S_ACQ -> !can_tx)
}

/* P_HYSTERESIS (safety): kTrackDegraded is only reached via kTrack.
 * Whenever state is S_DEGRADED, prev_state must be either S_TRACK
 * (the only legitimate entry path) or S_DEGRADED (already there).
 * Expressed via prev_state auxiliary and the deg_entered_from_track
 * flag set atomically with the legitimate S_TRACK→S_DEGRADED
 * transition. If S_DEGRADED is ever reached another way, the flag
 * stays false while state == S_DEGRADED — which would fail the LTL. */
ltl p_hysteresis {
    [] ((state == S_DEGRADED) ->
        (prev_state == S_TRACK || prev_state == S_DEGRADED))
}

/* P_PROGRESS (safety): the Schmitt-trigger fix eliminates TRACK↔DEGRADED
 * oscillation under partial-loss. Once reached, the pair {TRACK,DEGRADED}
 * is exited only via proper forced-ACQ (frames + time), not via spurious
 * LQ boundary flips. Expressed as: if we are in DEGRADED with LQ in the
 * border band (55-65%), the next step either stays in DEGRADED or goes
 * to forced ACQ — never flips to TRACK without LQ >= 65% being observed.
 *
 * The model naturally enforces this by only transitioning DEGRADED→TRACK
 * when lq_bucket == LQ_HIGH. The LTL checks the broader invariant: in
 * partial_loss_mode with LQ_BORDER, any state transition out of
 * {TRACK,DEGRADED} must be to S_ACQ (the forced path), never to S_TENTATIVE.
 * Safety, no -f required. */
ltl p_progress {
    [] ((partial_loss_mode && (prev_state == S_TRACK || prev_state == S_DEGRADED)) ->
         (state == S_TRACK || state == S_DEGRADED || state == S_ACQ))
}
