/* rocketchip_flash_protocol.pml — flash_safe_execute / Core 1 lockout /
 *                                  I2C reset protocol (R-11)
 *
 * Phase 8 Category 3 of the 2026-05-07 master standards audit. Highest-
 * ROI SPIN extension per Cubesat Startup Engineer council persona — the
 * informal protocol documented in LL Entries 28 + 31 is now formally
 * verified.
 *
 * History:
 *   LL Entry 28 (2026-02-10): i2c_bus_recover() corrupted DW_apb_i2c
 *     when it switched GPIO function from I2C to SIO without deinit-
 *     reinit of the peripheral. Fixed: peripheral deinit/reinit around
 *     the GPIO function switch.
 *   LL Entry 31 (2026-03-06): flash_safe_execute() corrupts I2C0 on
 *     RP2350B. The XIP cache disable + multicore lockout interacts
 *     with the AHB fabric in a way that leaves SDA/SCL driven LOW.
 *     Fix: (1) flash ops BEFORE i2c_bus_init() at boot, (2) call
 *     i2c_bus_reset() after every runtime flash_safe_execute().
 *
 * R-15 (committed 3bf760a, this audit cycle): closed the runtime-side
 * gap where cmd_flush_log + cli_do_erase_flights weren't calling
 * i2c_bus_reset() after their flash ops. With R-15 in, the firmware
 * now uniformly honors the LL-31 protocol at every reachable callsite.
 *
 * R-11 (this model) formally verifies that the corrected protocol
 * holds: every runtime flash_safe_execute() is followed by an
 * i2c_bus_reset(), Core 1 is locked out during flash, and no I2C
 * transaction overlaps a flash op.
 *
 * Three processes:
 *   FlashCaller     — Core 0 process that wraps flash_safe_execute().
 *                     Sets flash_busy, locks out Core 1, performs the
 *                     flash op, releases the lockout, resets I2C.
 *   Core1           — Core 1 process. When locked out, suspended; when
 *                     not, runs sensor reads via I2C transactions.
 *                     Modeled as a non-deterministic choice between
 *                     "do a transaction" and "skip" each step.
 *   I2CPeripheral   — Tracks the bus state machine (active / corrupt /
 *                     reset). Flash op corrupts; i2c_bus_reset() clears.
 *
 * Three LTL properties (council scoping 2026-05-07 Phase 6 — all
 * hard PASS after R-17 lands the cooperative-pause mechanism in this
 * same audit cycle):
 *
 *   P_CORE1_LOCKED_DURING_FLASH: [] (flash_busy -> core1_locked_out)
 *     Safety — Core 1 must be in multicore_lockout state while flash
 *     is running. SDK contract (pico/flash.h verbatim: "the other core
 *     is not executing/reading flash").
 *
 *   P_NO_I2C_DURING_FLASH: [] (flash_busy -> !i2c_txn_in_progress)
 *     Safety — no I2C transaction may be in flight at any point during
 *     a flash op. R-17's cooperative pause (`core1_i2c_pause()` /
 *     `core1_i2c_resume()` in `src/safety/core1_i2c_pause.{h,cpp}`)
 *     wraps every reachable flash_safe_execute callsite, so Core 1 is
 *     paused-and-acked before flash_busy raises and no in-flight
 *     transaction exists during the flash window.
 *
 *   P_I2C_RESET_AFTER_FLASH: [] (flash_just_finished ->
 *                                  <> i2c_was_reset_since)
 *     Liveness — every flash op is eventually followed by an i2c
 *     reset. Models the LL-31 prescribed protocol that R-15
 *     enforces; remains as belt-and-suspenders against any
 *     transaction that didn't drain in R-17's pause window.
 *
 * Investigation history (2026-05-13):
 *
 *   The Phase 6 council scoping specified P_NO_I2C_DURING_FLASH as a
 *   hard safety claim. During R-11 implementation, modeling it against
 *   THE-FIRMWARE-AT-START-OF-R-11 produced a counterexample that IS
 *   the LL-31 race in micro-detail: Core 1 has an I2C transaction in
 *   flight when Core 0 raises flash_busy; `multicore_lockout` halts
 *   Core 1's CPU but does NOT drain the in-flight transaction (which
 *   is being driven by the DW_apb_i2c peripheral over many APB cycles
 *   after the CPU has handed it off). The APB bridge eventually times
 *   out the stalled transfer (datasheet §2.1.4, 65,535-cycle ceiling),
 *   leaving the peripheral in an indeterminate state and discarding
 *   whatever sensor data was being read.
 *
 *   Surfaced bug (R-17): cal_pre_hook + cal_post_hook were DEFINED in
 *   cal_hooks.cpp with the cooperative-pause mechanism wired correctly
 *   (g_core1PauseI2C / g_core1I2CPaused atomics + Core 1's
 *   sensor_core1.cpp:365 honor loop) but cal_pre_hook had no callers.
 *   The function-pointer table rc_os_cal_pre_hook was assigned at
 *   main.cpp:315 but never invoked. Every reachable flash_safe_execute
 *   callsite was running the race open.
 *
 *   R-17 fix: extract general primitives `core1_i2c_pause()` /
 *   `core1_i2c_resume()` into `src/safety/core1_i2c_pause.{h,cpp}` and
 *   wrap every reachable runtime flash_safe_execute callsite:
 *     - ao_rcos.cpp cal_save_to_flash()
 *     - rc_os_commands.cpp cmd_flush_log()
 *     - rc_os_commands.cpp cli_do_erase_flights()
 *   With R-17 in, P_NO_I2C_DURING_FLASH is restored to hard PASS.
 *
 * What's NOT modeled (intentional simplifications):
 *   - The SDK's `multicore_lockout` mechanism itself. We assume it
 *     works (it's an SDK primitive); R-11 verifies the project's
 *     usage of it.
 *   - The actual XIP cache disable. The protocol invariant we care
 *     about is "Core 1 cannot run code OR touch I2C during the flash
 *     window," which we model by the lockout + the I2C peripheral
 *     state machine.
 *   - The detailed flash-op state machine (erase vs write, sector vs
 *     page). One model "flash op runs to completion atomically" is
 *     sufficient for the cross-actor protocol.
 *   - Boot-time init order (LL 31 fix #1). Boot is one-shot, not
 *     concurrent; static analysis suffices. R-11 verifies runtime
 *     ops only.
 *
 * Silicon-level rationale for the abstraction (RP2350 datasheet
 * 2026-05-13 intake):
 *
 *   §2.1 Bus fabric (Figure 5): I2C0/I2C1 sit on the APB Splitter
 *     downstream of the AHB→APB bridge; QMI+XIP cache sit on a
 *     separate downstream path. I2C and flash do NOT directly
 *     contend on the same bus segment.
 *   §2.1.4 APB bridge: "The APB bridge implements a fixed timeout
 *     for stalled downstream transfers. When an APB transfer exceeds
 *     65,535 cycles the APB bridge abandons the transfer and returns
 *     a bus fault." When Core 1 is mid-I2C-transaction at the moment
 *     multicore_lockout fires, the stalled APB transaction can hit
 *     this timeout, leaving the DW_apb_i2c peripheral in an
 *     indeterminate state.
 *   §4.4 XIP / §4.4.2 QMI: flash erase/program requires QMI to leave
 *     XIP mode; SDK `flash_safe_execute()` coordinates this. The
 *     `pico/flash.h` contract is "IRQs disabled + other core not
 *     executing/reading flash" — `multicore_lockout` enforces this.
 *
 * LL Entry 31 (2026-03-06) is the project's empirical discovery
 * BEYOND the SDK's documented contract: the lockout-mid-I2C ->
 * APB-bridge-timeout chain leaves the I2C peripheral corrupt, and
 * a peripheral deinit/reinit (i2c_bus_reset) is required to recover.
 * R-11 models the empirically-observed protocol that R-15 enforces
 * at every reachable callsite.
 *
 * To run (from a Cygwin terminal at the repo root):
 *   cd tools/spin
 *   spin -a rocketchip_flash_protocol.pml
 *   gcc -O2 -o pan pan.c
 *   ./pan -a -N p_no_i2c_during_flash
 *   ./pan -a -N p_core1_locked_during_flash
 *   ./pan -a -f -N p_i2c_reset_after_flash
 *
 * Or via the master gate:
 *   bash tools/spin/run_stage_o_ao_spin.sh
 */

/* =============================================================
 * Constants
 * ============================================================= */

#define MAX_FLASH_OPS 2   /* Bound the model — two flash ops is enough
                            to explore the interesting interleavings. */

/* I2C peripheral states. */
mtype = { I2C_OK, I2C_CORRUPT };

/* =============================================================
 * Shared state
 * ============================================================= */

bool flash_busy            = false;
bool core1_locked_out      = false;
bool i2c_txn_in_progress   = false;
mtype i2c_state            = I2C_OK;

/* R-17: cooperative pause atomics (g_core1PauseI2C / g_core1I2CPaused in
 * firmware). Core 0 requests pause via core1_pause_request; Core 1 acks
 * via core1_paused_ack after finishing any in-flight transaction. */
bool core1_pause_request   = false;
bool core1_paused_ack      = false;

/* Model-only flags for LTL property eval. */
bool flash_just_finished        = false;
bool i2c_was_reset_since        = false;

/* Bounded counter: how many flash ops have we performed? */
byte flash_ops_done = 0;

/* =============================================================
 * Flash-caller process (Core 0 wrapper around flash_safe_execute)
 *
 * Mirrors the R-15-corrected sequence:
 *   ao_rcos.cpp:332 cal_save_to_flash()   →  flash_safe_execute()
 *                                          →  i2c_bus_reset()
 *   rc_os_commands.cpp:1009 cmd_flush_log →  flush_ring_to_flash()
 *                                          →  i2c_bus_reset()  (R-15)
 *   rc_os_commands.cpp:1080 cli_do_erase  →  flight_log_erase_all()
 *                                          →  i2c_bus_reset()  (R-15)
 * ============================================================= */

active proctype FlashCaller() {
    do
    :: flash_ops_done < MAX_FLASH_OPS ->
        /* Step 0 (R-17): cooperative pause request — Core 0 calls
         * core1_i2c_pause() which sets pause_request and waits for ack. */
        core1_pause_request = true;
        /* Wait for Core 1 ack. (In firmware: 100ms timeout; for the
         * model, we wait unboundedly since fairness + Core 1's progress
         * guarantee an ack arrives.) */
        core1_paused_ack -> skip;

        /* Step 1: request multicore_lockout. SDK's flash_safe_execute()
         * does this internally; we model the precondition explicitly.
         * After R-17, Core 1 is already paused-and-acked at this point,
         * so there is no in-flight I2C transaction. */
        core1_locked_out = true;

        /* Step 2: flash op begins. flash_busy true throughout. */
        atomic {
            flash_busy = true;
            i2c_state  = I2C_CORRUPT;  /* LL 31: peripheral corruption */
            i2c_was_reset_since = false;  /* reset latch cleared on new flash */
        }

        /* Step 3: flash op completes. Lockout released. */
        atomic {
            flash_busy = false;
            core1_locked_out = false;
            flash_just_finished = true;
            flash_ops_done = flash_ops_done + 1;
        }

        /* Step 4: i2c_bus_reset() — the R-15 / LL-31 fix.
         * (Still required as belt-and-suspenders in case the pause
         * didn't take in time.) */
        atomic {
            i2c_state = I2C_OK;
            i2c_was_reset_since = true;
            flash_just_finished = false;
        }

        /* Step 5 (R-17): release the cooperative pause. core1_i2c_resume()
         * clears both flags atomically so a back-to-back pause call
         * doesn't observe a stale ack from this op. */
        atomic {
            core1_paused_ack = false;
            core1_pause_request = false;
        }

    :: flash_ops_done >= MAX_FLASH_OPS -> break;
    od;
}

/* =============================================================
 * Core 1 process (sensor reads via I2C)
 *
 * When locked out, blocked. Otherwise, may non-deterministically
 * attempt an I2C transaction. Transactions only succeed if the
 * peripheral is in I2C_OK state.
 * ============================================================= */

active proctype Core1() {
    do
    :: core1_pause_request && !core1_paused_ack ->
        /* R-17: Core 1 sees a pause request. In firmware, this branch
         * is at the top of Core 1's sensor loop, so any I2C transaction
         * from a previous iteration has already completed (the loop
         * runs transactions to completion before returning to the top).
         * Set the ack; Core 0 will then proceed to lockout + flash. */
        atomic {
            i2c_txn_in_progress = false;  /* loop boundary: no in-flight txn */
            core1_paused_ack = true;
        }

    :: !core1_locked_out && !core1_pause_request ->
        if
        :: i2c_state == I2C_OK ->
            /* Attempt an I2C transaction. CPU initiates; peripheral
             * drives it to completion over many APB cycles. */
            atomic { i2c_txn_in_progress = true; }
            atomic { i2c_txn_in_progress = false; }
        :: i2c_state == I2C_CORRUPT ->
            /* Peripheral corrupt — transaction either NACKs or hangs.
             * Model as a skip; firmware's consecutive-fail counter
             * handles the recovery. */
            skip;
        :: skip;  /* Or just don't initiate a transaction this step. */
        fi;

    :: core1_locked_out ->
        /* Locked out by multicore_lockout. Core 1 suspended.
         * With R-17's cooperative pause in place, we reach this state
         * only AFTER Core 1 has acked the pause, so no in-flight
         * transaction exists at this point. */
        skip;

    :: (flash_ops_done >= MAX_FLASH_OPS) && !flash_busy -> break;
    od;
}

/* =============================================================
 * LTL properties
 * ============================================================= */

/* P_CORE1_LOCKED_DURING_FLASH:
 * Safety — multicore_lockout must be active throughout the flash op.
 * Required by the SDK contract (pico/flash.h: "the other core is not
 * executing/reading flash"). */
ltl p_core1_locked_during_flash {
    [] (flash_busy -> core1_locked_out)
}

/* P_I2C_RESET_AFTER_FLASH:
 * Liveness — every flash op is eventually followed by an i2c reset.
 * Models the LL-31 prescribed protocol. R-15 implemented this at the
 * two reachable CLI handlers; ao_rcos.cpp:338 already did. With R-15
 * in, every flash-op exit path is followed by reset. Requires weak
 * fairness (-f). */
ltl p_i2c_reset_after_flash {
    [] (flash_just_finished -> <> i2c_was_reset_since)
}

/* P_NO_I2C_DURING_FLASH (council-original property, restored to hard
 * gate by R-17 which landed in this same audit cycle):
 * Safety — no I2C transaction may be in flight at any point during a
 * flash op. With R-17's cooperative pause wired into every reachable
 * flash_safe_execute callsite (ao_rcos.cpp cal_save_to_flash,
 * rc_os_commands.cpp cmd_flush_log + cli_do_erase_flights), Core 1 is
 * paused-and-acked BEFORE flash_busy raises, so no in-flight
 * transaction exists during the flash window. The post-flash
 * i2c_bus_reset (R-15) remains as belt-and-suspenders. */
ltl p_no_i2c_during_flash {
    [] (flash_busy -> !i2c_txn_in_progress)
}
