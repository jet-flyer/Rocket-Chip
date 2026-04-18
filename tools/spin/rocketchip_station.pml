/* rocketchip_station.pml — Station radio RX / ACK / retry protocol
 *
 * Stage 16C IVP-147. Models the station-side command delivery channel
 * that vehicle `rocketchip_fd.pml` / `rocketchip_ao.pml` do not cover.
 *
 * Scope: station sends a tracked MAVLink COMMAND_LONG with a sequence
 * number, vehicle receives and replies with COMMAND_ACK carrying the
 * matching seq. Station's cmd_retry_tick retries at 3 s intervals, up
 * to 3 retries, then gives up. Radio channel is lossy (packet drop on
 * both directions). This models the behavior characterized by
 * IVP-132a.5 ACK stress test (6.7% first-try ACK rate — the
 * RadioScheduler-sync gap).
 *
 * Properties:
 *   P_TERMINATION: every pending command eventually terminates
 *                  (either acked or all retries exhausted). Liveness.
 *                  Verifies no deadlock in the retry ladder.
 *   P_NO_DOUBLE_CLEAR: pending bit cannot be cleared twice for the
 *                     same sequence number. Safety — catches the
 *                     race between ACK arrival and retry-exhaustion
 *                     clear that would drop a pending command twice.
 *
 * Scope (intentionally narrow for IVP-147 initial scaffolding):
 *   - Single command in flight with fixed seq=1.
 *   - Radio loss on both directions is nondeterministic.
 *   - ACK is validated by exact-seq rendezvous match; non-matching
 *     ACKs cannot be modelled here because the vehicle only ever
 *     emits ACK(1). Multi-command-in-flight + seq-mismatch ACK
 *     handling is a future SPIN extension when firmware actually
 *     supports multiple pending commands.
 *
 * What's NOT modeled (future SPIN extensions — add properties as
 * firmware grows):
 *   - MAVLink parser state (byte-by-byte decoding)
 *   - RadioScheduler TX window arbitration (the actual fix to the
 *     RadioScheduler-sync gap requires modelling the scheduler)
 *   - station_idle_tick GPS poll interleave with AO_Radio poll
 *   - Multiple simultaneous pending commands (firmware is single-pending
 *     today; when that changes, add seq-mismatch modelling).
 *
 * To run:
 *   cd tools/spin
 *   spin -a rocketchip_station.pml
 *   gcc -O2 -o pan_station pan.c
 *   ./pan_station -a -N p_termination
 *   ./pan_station -a -N p_no_double_clear
 *   ./pan_station -a -N p_seq_match
 *
 * Liveness (p_termination) requires weak fairness: add -f
 *   ./pan_station -a -f -N p_termination
 */

/* =============================================================
 * Constants
 * ============================================================= */

#define MAX_RETRIES 3

/* Radio message types */
mtype = { CMD, ACK };

/* =============================================================
 * Shared state
 * ============================================================= */

/* Pending-cmd tracking on station side, mirroring s_pending_cmd in
 * src/active_objects/ao_telemetry.cpp. */
bool     sta_pending      = false;
byte     sta_seq          = 0;    /* seq# of current pending command */
byte     sta_retries_left = 0;
byte     sta_terminated   = 0;    /* 0=in-flight, 1=acked, 2=exhausted */

/* Model-only: how many times have we cleared sta_pending for the
 * current seq? Must stay <= 1 to catch race bugs. */
byte     clear_count = 0;
byte     last_clear_seq = 255;    /* 255 = never cleared */

/* Lossy channels. Depth 2 allows the vehicle to reply while a new
 * retry is already in flight. */
chan s2v = [2] of { mtype, byte };   /* station -> vehicle */
chan v2s = [2] of { mtype, byte };   /* vehicle -> station (ACK) */

/* Bounded tick counters for liveness (Discrete-Time Promela pattern,
 * per IVP-121 rocketchip_fd.pml model). Each retry "tick" is one
 * unit; after 3 retries the counter saturates and forces termination. */

/* =============================================================
 * Station send-tracked-command process
 * ============================================================= */

active proctype Station() {
    byte send_seq;

    /* Kick off with a single tracked command (seq=1). Real firmware
     * cycles seq; modelling a single command is sufficient for the
     * termination/no-double-clear properties of interest here. */
    send_seq = 1;
    atomic {
        sta_pending      = true;
        sta_seq          = send_seq;
        sta_retries_left = MAX_RETRIES;
        sta_terminated   = 0;
        clear_count      = 0;
        last_clear_seq   = 255;
    }

    /* Lossy send: either the frame reaches the vehicle or it drops. */
    if
    :: s2v ! CMD(send_seq);
    :: skip;   /* packet dropped in air */
    fi;

    /* Retry loop + ACK handling interleaved. Each iteration is either
     * a "tick reached retry threshold" step or an "ACK arrived" step,
     * non-deterministic. Terminates when sta_terminated != 0. */
    do
    :: sta_pending && (sta_retries_left > 0) ->
        /* Retry path — models the 3s tick firing with pending set. */
        atomic {
            sta_retries_left = sta_retries_left - 1;
        }
        if
        :: s2v ! CMD(send_seq);   /* retry TX */
        :: skip;                   /* retry also dropped */
        fi;

    :: sta_pending && (sta_retries_left == 0) ->
        /* Exhausted path. Clear pending, mark terminated. */
        atomic {
            if
            :: sta_pending ->
                sta_pending      = false;
                clear_count      = clear_count + 1;
                last_clear_seq   = send_seq;
                sta_terminated   = 2;
            :: else -> skip;   /* already cleared by ACK race */
            fi;
        }

    :: v2s ? ACK(eval(send_seq)) ->
        /* ACK arrived with matching seq. */
        atomic {
            if
            :: sta_pending ->
                sta_pending      = false;
                clear_count      = clear_count + 1;
                last_clear_seq   = send_seq;
                sta_terminated   = 1;
            :: else -> skip;   /* retry-exhaust already cleared */
            fi;
        }

    :: (sta_terminated != 0) -> break;
    od;
}

/* =============================================================
 * Vehicle ACK-generator process
 * ============================================================= */

active proctype Vehicle() {
    byte rx_seq;

    do
    :: s2v ? CMD(rx_seq) ->
        /* Vehicle received a command. Respond with ACK carrying the
         * same seq — possibly dropped, modelling lossy downlink. */
        if
        :: v2s ! ACK(rx_seq);
        :: skip;
        fi;
    :: (sta_terminated != 0) -> break;
    od;
}

/* =============================================================
 * Properties (LTL)
 * ============================================================= */

/* P_TERMINATION: the pending command eventually reaches a terminal
 * state (either acked or exhausted). Liveness — requires -f flag. */
ltl p_termination { <> (sta_terminated != 0) }

/* P_NO_DOUBLE_CLEAR: pending cannot be cleared twice for the same
 * seq. Safety — catches races between ACK delivery and retry-exhaust
 * clear (defensive check on the atomic guard). */
ltl p_no_double_clear { [] (clear_count <= 1) }
