# Stage P7 + Stage 15 — Development Plan (SHELVED 2026-04-11)

## ⚠️ SHELVED — BLOCKED ON BENCH SIM DIAGNOSIS

**Status as of 2026-04-11:** Plan is shelved mid-execution. IVP-119 code changes are complete and locally verified via host tests + GDB inspection, but cannot be committed because the HW verification gate depends on `scripts/bench_flight_sim.py`, which has been silently broken since 2026-04-06 (commit 2254b16) and additionally exhibits a structural hang that I couldn't diagnose in-session without derailing Stage P7.

**Two latent issues uncovered during the attempt (both pre-existing, both unrelated to IVP-119):**

1. **`fused.baro_vvel` field name is misleading** — it's actually `g_eskf.v.z`, not a raw baro derivative. `guard_baro_peak()` reads it thinking it's independent of ESKF when it isn't. Stage P7 was designed to fix this as IVP-119's rename, then add a true raw-baro guard in IVP-120. The rename itself works (host tests 678/678, clean target build, GDB confirmed `fused.baro_pressure_pa` populated live with real DPS310 data). IVP-119 is **correct and ready to commit** once the HW gate can be met.

2. **`scripts/bench_flight_sim.py` is bit-rotten since 2026-04-06** — commit `2254b16` renamed the firmware log string from `[FD] PYRO INTENT: DROGUE` to `[FD] PYRO FIRED: DROGUE (primary)` but did NOT update the test script's `RE_PYRO` regex. Every commit since has claimed "9/9 bench sim PASS" as a gate without actually running the bench sim (honor-system gate never enforced). Tests 5, 6, 9 all fail on baseline firmware independent of IVP-119. Also: the script has a structural hang somewhere in `reset_to_idle()` after a failing test, and the in-progress `--max-runtime` deadline added during the shelved attempt only checks between tests, not inside `run_test()`, so a genuinely stuck run still runs indefinitely.

**Pivoting to a dedicated plan session** for the bench sim diagnostic + fix. Once bench sim is proven healthy on baseline firmware, this plan can be un-shelved and IVP-119 committed as originally scoped.

**Preserved state:**
- `stash@{0}` — partial bench_flight_sim.py fixes (PYRO regex, connect retry, incomplete timeout). Can be reused or dropped depending on the dedicated session's approach.
- `stash@{1}` — IVP-119 FusedState rename changes (12 files, WIP on be37b9d). Ready to unstash when the HW gate is available again.
- `docs/IVP.md` Stage P7 / Stage 15 insertion — committed at be37b9d, still accurate.

**Root-cause pattern to investigate in the dedicated session:** Both the latent `baro_vvel` misnomer and the bench sim rot share a root cause — **verification gates claimed but not performed**. Plans list "HW verify" as a gate, commits cite it in messages, but running the gate is honor-system and can be skipped silently. When skipped, downstream bit-rot accumulates until someone actually runs the gate. See feedback memory files `feedback_hw_verify.md` and `feedback_hw_verify_capability.md`.

---

## Context

Stages 1-14 are complete and the Notification Engine was the last architectural pass. Before we can safely field-test, a set of concrete gaps need closing. During plan development they split cleanly into two stages:

### Why two stages

The first planning pass bundled four items into a single "Stage 15: Pre-Flight Radio + Station": MAIN_DESCENT fix, half-duplex ACK, distance finish, station help cleanup. The first council round recommended a blind `main_descent_timeout_ms` wall-clock fallback for the MAIN_DESCENT problem. The user rejected that approach: *"I don't like automatically triggering any state purely based on time... that's a variable during missions that could change. An updraft could catch the parachute or a mission could last longer than expected."*

A second, deeper council round re-opened the question from first principles. It produced a completely different design — multi-channel voted landing detection with raw baro pressure derivative as a physically independent second channel. That design grew the scope by 5x (new guard, FusedState field rename, latent bug fix, ESKF-fault-confirmed-by-baro conjunction path, SPIN counter model, full host test matrix). It is also thematically disjoint from the station UX work — it's a safety/formal-verification pass, not field-readiness hardening.

Following the existing sub-stage convention used by `Stage M`, `Stage 3D`, and `Stage J`, this becomes:

- **Stage P7** — Multi-Channel Landing Detection & SPIN Liveness Fix (out-of-sequence, 3 IVPs)
- **Stage 15** — Pre-Flight Radio + Station (3 IVPs, renumbered from previous 4)

Stage P7 runs first. Stage 15 runs after, unchanged in thematic scope but with Item A lifted out.

### The gaps being closed

1. **SPIN property P7 (liveness) has been failing since Stage 9.** `state_main_descent` only exits via `SIG_LANDING`, which is fired by a stationary guard based on ESKF velocity. If the ESKF dies mid-descent, velocity estimate freezes, the guard never fires, the FD sits in MAIN_DESCENT until battery death. The fix is multi-channel voted landing detection. *(Stage P7 — IVP-119, 120, 121)*

2. **Latent bug in `fused.baro_vvel`.** Discovered during Stage P7 council research: the field name suggests a raw-baro-derived vertical velocity, but it is assigned directly from `g_eskf.v.z` at [src/active_objects/ao_logger.cpp:151](src/active_objects/ao_logger.cpp#L151). `guard_baro_peak()` is using it as if it were independent of ESKF when it isn't. Stage P7 fixes this as a side effect. *(Stage P7 — IVP-119)*

3. **Station ARM is single-key fire-and-forget.** Press `a`, station prints `[CMD] ARM sent` with no idea whether the vehicle received it. A typo glance-arms the rocket. No ACK path exists over LoRa — vehicle's existing `emit_command_ack()` only sends over MAVLink/USB to QGC. *(Stage 15 — IVP-122)*

4. **Distance-to-rocket is a stub.** [src/cli/rc_os_commands.cpp:1162-1173](src/cli/rc_os_commands.cpp#L1162) prints `"Vehicle distance: needs received position"`. Haversine helper already exists at line 1130, `AO_Telemetry_get_rx_state()` already caches received vehicle nav state. *(Stage 15 — IVP-123)*

5. **Station help menu is stale and AGENT_WHITEBOARD.md has a resolved item still listed as open.** Help lists `g-GPS d-Distance` but actual keys are `g d p a m t r`. Whiteboard line 78 says "IVP-103 Station GPS Push — Needs radio command path" but `cmd_station_gps_push()` has worked since Stage 7 Take 2. *(Stage 15 — IVP-124)*

Goal: **vehicle and station are ready for a first real ARM→LAUNCH→RECOVERY cycle**, with SPIN P7 passing, station ARM acknowledged end-to-end, distance-to-rocket functional, and documentation current.

### Hardware validation is mandatory — not optional

**Every IVP in this plan requires hardware verification on actual hardware before its gate is considered met.** No IVP ships on host-test passage alone. This rule applies even to documentation-adjacent IVPs (like IVP-124's help-menu refresh) — if it touches firmware, it gets flashed, it gets booted, and the claimed behavior is observed on real silicon before commit.

Why this rule exists as a blanket policy rather than a per-IVP checkbox:
- Multiple bugs in the lessons-learned journal (LL Entries 1, 13, 15, 28, 29, 35) were bugs that host tests did not and *could not* catch — they were bare-metal / dual-core / peripheral / QP-runtime issues.
- The cost of one HW verify session (flash via debug probe, 2-minute soak, eyeball CLI output) is small compared to the cost of discovering a silent regression after multiple IVPs have stacked on top.
- Host tests verify logic; HW verify is the only way to confirm the logic lands on actual hardware without being clobbered by some peripheral / memory / timing interaction.

**Per-IVP gates below include explicit `[HW VERIFY — REQUIRED BEFORE COMMIT]` sections.** Do not skip them. Do not batch them. Do not defer them to a post-stage soak and treat that as sufficient. Each IVP's commit message should include the line `HW verified: <date> <build-tag>` citing the build iteration tag from the running binary (LL Entry 2).

If HW verification is genuinely impossible for a specific IVP (e.g., hardware failure, probe broken), that is a blocker — escalate to the user, do not proceed with host-tests-only.

---

## Stage P7: Multi-Channel Landing Detection & SPIN Liveness Fix

**Purpose:** Close the SPIN P7 liveness gap by adding a physically independent secondary landing detection channel (raw baro pressure derivative), a health-degraded conjunction path (ESKF fault + baro stationary), and a last-resort backstop (`descent_max_duration_ms`) that fires only when all physical channels are silent. Also fixes a latent `fused.baro_vvel` misnomer bug discovered during design.

**Council-reviewed:** Four personas, Round 2 re-opened after user rejection of blind wall-clock timer. Consensus on multi-channel voted design, raw baro as primary fallback channel, time as last-resort-only backstop. See design principle at bottom of this stage.

**Prerequisites:** Stage 14 complete (Notification Engine).

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-119 | FusedState Baro Field Fix | Rename `baro_vvel` → `vert_vel_eskf`, add raw `baro_pressure_pa` field, update `guard_baro_peak()` comments. Small bug-fix commit; no behavioral change. |
| IVP-120 | Baro-Stationary Guard | New `guard_baro_stationary()` with 2-point Δpressure/Δt, sustain counter, MissionProfile parameters. Guard registration + phase bitmask. Guard unit tests. |
| IVP-121 | MAIN_DESCENT Liveness Fix | `state_main_descent` SIG_TICK logic: ESKF-fault-conjunction path + last-resort backstop with beacon. SPIN model counter transitions. Host test matrix. P7 resolution. |

---

### IVP-119: FusedState Baro Field Fix

**Prerequisites:** None (foundation for IVP-120)

**Implement:** Rename the misleading `baro_vvel` field in `FusedState` and add a true raw-baro field for downstream use. This is a bug fix — `fused.baro_vvel` is currently assigned directly from `g_eskf.v.z` at `ao_logger.cpp:151`, making the field name a trap. `guard_baro_peak()` uses it as if it were independent of ESKF when it isn't.

1. **Rename in `include/rocketchip/fused_state.h`:** `float baro_vvel;` → `float vert_vel_eskf;` with an inline comment clarifying: `// ESKF-propagated vertical velocity (NOT raw baro — see ao_logger.cpp)`.

2. **Add raw baro pressure field:** `float baro_pressure_pa;` with comment `// Raw DPS310 pressure, independent of ESKF`.

3. **Populate in `src/active_objects/ao_logger.cpp` `AO_Logger_populate_fused_state()`:**
   - `fused.vert_vel_eskf = g_eskf.v.z;` (was `fused.baro_vvel`)
   - `fused.baro_pressure_pa = snap.pressure_pa;` (new)

4. **Update every reader of the old field.** Grep for `baro_vvel` across `src/` and rename. Primary call site is `src/flight_director/guard_functions.cpp` in `guard_baro_peak()`.

5. **Update `guard_baro_peak()` comment block** to explicitly note: *"Uses ESKF-propagated vertical velocity (`vert_vel_eskf`). This is NOT independent of ESKF — if ESKF fails this guard will not fire correctly. For ESKF-independent descent detection, see `guard_baro_stationary()` (IVP-120)."*

6. **Update telemetry encoding** if `baro_vvel` is sent over the wire. Grep the telemetry encoder + Python decoder — if present, rename the symbol (wire format is unchanged, just the field name).

**[GATE]:**
- Clean target compile, zero warnings
- Clean host test compile, all existing tests pass (669/669 from Stage 14)
- `grep -r baro_vvel src/ include/ test/` returns zero matches
- No behavioral change in `guard_baro_peak()` — existing `CoastTimeoutForcedApogee` and related tests still pass

**[HW VERIFY — REQUIRED BEFORE COMMIT]:**
- Flash via debug probe to Feather RP2350 (not picotool — LL Entry 25)
- Boot cleanly, full boot banner, no ERR/FAIL markers
- CLI `s` output shows sensors online (IMU, baro, GPS all reporting)
- GDB break after `AO_Logger_populate_fused_state()` runs, inspect `fused.baro_pressure_pa` — must be nonzero, within ±10 Pa of raw DPS310 reading (cross-check via a separate CLI command if one exists, or a second GDB read at `snap.pressure_pa`)
- CLI `e` output shows ESKF velocity unchanged from pre-IVP baseline (field is renamed, value is identical)
- `scripts/bench_flight_sim.py` — 9/9 PASS (IVP-73 regression — confirms `guard_baro_peak()` still fires correctly through the rename)
- 2-minute idle soak on bench: no crash, no watchdog reset, IMU error count 0, baro error count 0

**[DIAG]:** If existing baro_peak tests fail after rename → check that the rename was mechanical (no accidental logic changes). If `baro_pressure_pa` shows zero → check seqlock read path in `AO_Logger_populate_fused_state()`; `snap.pressure_pa` is populated by the baro driver on Core 1. If bench flight sim fails at apogee → `guard_baro_peak()` is still using the renamed field but logic didn't follow; double-check the rename touched every reader.

---

### IVP-120: Baro-Stationary Guard

**Prerequisites:** IVP-119

**Implement:** Add a new guard, `guard_baro_stationary()`, that detects sustained near-zero baro altitude rate. Physically independent of ESKF. Critical design note: the rate computation (with its `prev_pressure_pa` / `prev_sample_ms` cache) lives in `AO_Logger_populate_fused_state()` and writes to a new `FusedState.baro_alt_rate_mps` field. The guard itself is stateless like `guard_baro_peak()` — it reads the precomputed rate and applies the sustain counter. This keeps the existing `GuardState` schema unchanged and matches how every other guard currently works.

1. **Add `GuardId::kBaroStationary`** to `src/flight_director/guard_evaluator.h` — append to the existing enum. This bumps `GuardId::kCount` by 1.

2. **Update `kGuardManaged[]` array** in `src/flight_director/guard_evaluator.cpp` — append one entry (`false` — unmanaged, same as other non-managed guards). The array is sized to `kCount` at compile time; adding an enum value without extending the array is a silent out-of-bounds read. This is a one-line addition but it MUST be in the same commit as step 1.

3. **Add `float baro_alt_rate_mps;` to `FusedState`** in `include/rocketchip/fused_state.h` — comment: `// Raw baro altitude rate (m/s), ESKF-independent. Populated in ao_logger.`

4. **Compute `baro_alt_rate_mps` in `AO_Logger_populate_fused_state()`** in `src/active_objects/ao_logger.cpp`:
   - Add file-scope cache: `static float s_prev_pressure_pa = 0.0f; static uint32_t s_prev_sample_ms = 0;`
   - On first call (`s_prev_sample_ms == 0`): cache current, write `fused.baro_alt_rate_mps = 0.0f`, return
   - Compute `dt_s = (now_ms - s_prev_sample_ms) * 0.001f`
   - Skip update if `dt_s < 0.05` (too short, noise-dominated) — keep previous rate value
   - Hydrostatic approximation: `dalt_dt = -(snap.pressure_pa - s_prev_pressure_pa) / (dt_s * kRhoAir * kGravity)` with `constexpr float kRhoAir = 1.225f; constexpr float kGravity = 9.81f;` declared near the top of the file. Alternatively use two calls to the DPS310 driver's existing `pressure_to_altitude()` helper and subtract — whichever is already present in the codebase. No magic numbers.
   - Write `fused.baro_alt_rate_mps = dalt_dt;`
   - Update cache: `s_prev_pressure_pa = snap.pressure_pa; s_prev_sample_ms = now_ms;`

5. **Implement `guard_baro_stationary(const FusedState& fused, const MissionProfile& profile, GuardState& state)`** in `src/flight_director/guard_functions.cpp` — stateless, identical pattern to `guard_baro_peak()`:
   ```cpp
   bool guard_baro_stationary(const FusedState& fused,
                              const MissionProfile& profile,
                              GuardState& state) {
       bool instant = fabsf(fused.baro_alt_rate_mps) < profile.baro_landing_rate_threshold_mps;
       return update_sustain(state, instant, profile.baro_landing_sustain_ms);
   }
   ```
   (where `update_sustain()` is the existing helper used by all sustain-counter guards — verify its name in the actual codebase, may be named differently).

6. **Register guard** in `src/flight_director/guard_evaluator.cpp`:
   - Add to guard dispatch table / switch
   - Valid phases bitmask: `(1 << FlightPhase::kDrogueDescent) | (1 << FlightPhase::kMainDescent)`
   - Fires signal: `SIG_LANDING` (same as `guard_stationary`)
   - Default `sustain_required` populated from `profile.baro_landing_sustain_ms`

7. **Add read-only accessor** `bool guard_evaluator_is_sustained(const GuardEvaluator* eval, GuardId id)` to `guard_evaluator.h/.cpp` — returns the `sustained` flag for the given guard. Needed by IVP-121's conjunction check. One-line wrapper.

8. **Add MissionProfile parameters** in `src/flight_director/mission_profile.h`:
   - `float baro_landing_rate_threshold_mps;` — alt rate threshold, default 0.3 m/s (above DPS310 noise floor ~0.2 m/s RMS at 10 Hz)
   - `uint32_t baro_landing_sustain_ms;` — sustain window, default 5000 ms (5 s)
   - `uint32_t descent_max_duration_ms;` — last-resort backstop used in IVP-121, default 900000 (15 min). Declared here so all Stage P7 fields land together.

9. **Update profile configs** `profiles/rocket.cfg` and `profiles/passive.cfg`:
   ```
   BARO_LANDING_RATE_THRESHOLD_MPS 0.3
   BARO_LANDING_SUSTAIN_MS 5000
   DESCENT_MAX_DURATION_MS 900000
   ```
   Skip `profiles/hab.cfg` — file does not exist yet. HAB profile creation deferred.

10. **Update generator** `scripts/generate_profile.py` to parse the three new fields and emit them in the MissionProfile struct initializer. Regenerate `src/flight_director/mission_profile_data.h`.

11. **Unit tests** in `test/test_guards.cpp` (or new file). Because the rate computation lives in `ao_logger`, the guard tests are simpler — they just vary `fused.baro_alt_rate_mps` directly:
    - `BaroStationary_RateBelowThresholdNotSustainedYet` — rate = 0.1 m/s, single tick, expect false
    - `BaroStationary_RateBelowThresholdSustainedFires` — rate = 0.1 m/s for `sustain_ms + tick`, expect true
    - `BaroStationary_RateAboveThresholdNeverFires` — rate = 0.5 m/s continuously, expect false
    - `BaroStationary_SustainResetsOnBreach` — rate = 0.1 m/s for half sustain, then 0.5, then back to 0.1, expect sustain restarts
    - `BaroStationary_ThresholdBoundary` — rate exactly at threshold → not below → false; rate just below → sustained after window

12. **Rate-computation tests** (new — tests `ao_logger` rate math): `test/test_ao_logger_baro_rate.cpp` (or add to existing logger test file):
    - `BaroRate_FirstSampleZero` — first call, expect baro_alt_rate_mps == 0
    - `BaroRate_DescendingPressureProducesPositiveAltRate` — pressure decreasing → altitude increasing → positive rate (sign convention check)
    - `BaroRate_StationaryPressureProducesZeroRate` — constant pressure → rate within ±noise
    - `BaroRate_SkipOnShortDt` — two samples within 50 ms → rate unchanged

**[GATE]:**
- `grep -r guard_baro_stationary src/` shows registration in evaluator
- `grep -rn "kBaroStationary" src/flight_director/guard_evaluator.cpp` shows `kGuardManaged` array entry added
- 5+5 unit tests pass (5 guard tests, 4 rate-computation tests)
- Target build compiles clean, zero warnings
- 680+ host tests pass (669 baseline + new tests from IVP-119 and IVP-120)

**[HW VERIFY — REQUIRED BEFORE COMMIT]:**
- Flash via debug probe
- Boot cleanly, CLI `s` output shows sensors online
- **Rate math live test:** lift device off desk smoothly, watch `fused.baro_alt_rate_mps` via GDB or debug print — value should become positive (altitude increasing) then negative (moving down) then return to near-zero (stationary on desk)
- **Guard firing test:** manually inject MAIN_DESCENT phase via GDB (`set var me->state.current_phase = rc::FlightPhase::kMainDescent`) or via a debug CLI key if one is exposed; place device on desk; confirm `guard_evaluator_is_sustained(&eval, kBaroStationary)` returns true within `baro_landing_sustain_ms + one tick` (via GDB read or a debug print)
- **Guard non-firing during simulated descent:** use bench flight sim or manually move device up-down at >0.5 m/s for 10 seconds; confirm guard sustain counter never reaches threshold
- `scripts/bench_flight_sim.py` — 9/9 PASS (no regression in existing guard behavior; new guard is additive and phase-bitmask-gated to descent only, so should not interfere with other phase transitions)
- 2-minute idle soak: no crash, no false LANDED transition in IDLE phase (guard bitmask must correctly exclude IDLE)

**[DIAG]:** Guard never fires → check `baro_landing_rate_threshold_mps` vs observed noise floor (print `fused.baro_alt_rate_mps` with `DBG_PRINT` temporarily). Guard fires immediately → check `sustain_ms` isn't zero; check `s_prev_sample_ms` initialization in ao_logger. Guard fires during nominal descent → rate threshold too low OR sustain window too short. Raise the threshold (e.g., 0.3 → 0.5 m/s) so nominal chute descent rates (typically 3-5 m/s) are well above it, or lengthen the sustain window so transient brief quiets during turbulence don't accumulate. Rate sign wrong → hydrostatic formula sign flipped; pressure DROPS as altitude RISES, so `dalt_dt = -(dP) / (ρg dt)`. Guard fires in IDLE phase → phase bitmask missing or incorrect; verify `(1 << FlightPhase::kDrogueDescent) | (1 << FlightPhase::kMainDescent)` is what's registered. Out-of-bounds read crash on boot → `kGuardManaged` array not extended for new `kCount` value.

---

### IVP-121: MAIN_DESCENT Liveness Fix

**Prerequisites:** IVP-119, IVP-120

**Implement:** Wire the baro-stationary guard into `state_main_descent` and add the last-resort backstop. Three new code paths, none of which is a blind timer as the nominal detector.

1. **Conjunction + backstop logic lives in `flight_director_evaluate_guards()`, NOT inside `state_main_descent()`.** Critical architectural point: `state_main_descent()` is a QHsm state handler with signature `(FlightDirector * const me, QEvt const * const e)` — it has no `fused` in scope. The existing pattern (verified at `flight_director.cpp:193-234`) is that guard evaluation happens in `flight_director_evaluate_guards(FlightDirector* me, const FusedState& fused, ...)`, which receives `fused` as a parameter and runs *before* signals are dispatched into the HSM. The Stage P7 additions belong in this function, gated on `phase == FlightPhase::kMainDescent`, and dispatch `SIG_LANDING` via the existing `flight_director_dispatch_signal(me, SIG_LANDING)` path that `state_main_descent` already handles.

   The beacon publish for the backstop path must use **static storage** — stack-local `QEvt` with `QACTIVE_PUBLISH` is the LL Entry 35 bug pattern.

   ```cpp
   // File-scope near top of flight_director.cpp (alongside other static events):
   static QEvt s_beacon_evt;  // Must be static — LL Entry 35

   // Inside flight_director_evaluate_guards(), AFTER guard_evaluator_tick() and
   // combinator_set_evaluate() return, BEFORE the function returns.
   // Only applies when phase == kMainDescent.
   if (phase == FlightPhase::kMainDescent) {
       uint32_t elapsed = me->tick_ms - me->state.phase_entry_ms;

       // Path 1: ESKF-fault-confirmed-by-baro
       // ESKF dead AND raw baro agrees stationary → close phase via SIG_LANDING.
       // Conjunction, not disjunction — ESKF fault alone is a health alert, not a landing.
       // NOTE: the normal baro-stationary guard (IVP-120) already dispatches SIG_LANDING
       // on its own. This conjunction path is the additional safety net for the
       // FM1 case where ESKF is confirmed dead — it fires even if the baro guard
       // hasn't quite met its full sustain window yet, provided baro is currently quiet.
       if (rc::health_eskf(fused.health_primary) == rc::kHealthFault &&
           guard_evaluator_is_sustained(&me->guard_eval, GuardId::kBaroStationary)) {
           printf("[FD] MAIN_DESCENT: ESKF fault + baro stationary → LANDED\n");
           flight_director_dispatch_signal(me, SIG_LANDING);
           return;  // Signal dispatched; exit the guard evaluator.
       }

       // Path 2: Last-resort backstop
       // Fires only when all physical channels have been silent for descent_max_duration_ms.
       // Distress state — publish beacon for recovery, then dispatch SIG_LANDING.
       if (elapsed >= me->profile->descent_max_duration_ms) {
           printf("[FD] MAIN_DESCENT: max duration elapsed → LANDED (distress)\n");
           s_beacon_evt.sig = rc::SIG_BEACON_ACTIVE;  // static — safe for QACTIVE_PUBLISH
           QACTIVE_PUBLISH(&s_beacon_evt, me);
           flight_director_dispatch_signal(me, SIG_LANDING);
           return;
       }
   }
   ```

   `state_main_descent()` itself needs NO changes — its existing `case SIG_LANDING` handler at `flight_director.cpp:455-456` handles all three paths (primary baro-stationary guard from IVP-120, the conjunction shortcut, and the backstop) uniformly. The backstop beacon is published *before* the SIG_LANDING dispatch so the AO_Notify beacon intent is active when the state transition runs.

2. **ESKF health source:** Read `fused.health_primary` directly from the `fused` parameter of `flight_director_evaluate_guards()`. This is the same pattern already used at line 228 for `lockout.eskf_healthy`. No new plumbing needed.

3. **`guard_evaluator_is_sustained()` accessor** must already be added in IVP-120 step 7. IVP-121 depends on it.

4. **SPIN model update** (content unchanged from prior draft; see Promela block below). SPIN's abstraction level doesn't care that the C++ logic moved from `state_main_descent` into `flight_director_evaluate_guards` — the model treats MAIN_DESCENT as a single state with a set of enabled transitions.

5. **SPIN model** `tools/spin/rocketchip_fd.pml:149-155` — replace current MAIN_DESCENT block:

   ```promela
   /* ---- MAIN_DESCENT ---- */
   :: phase == MAIN_DESCENT ->
       if
       :: true -> phase = LANDED                     /* SIG_LANDING (primary: ESKF stationary) */
       :: true -> phase = LANDED                     /* SIG_LANDING (secondary: raw baro stationary) */
       :: descent_ticks >= DESCENT_BACKSTOP_TICKS ->
           atomic { beacon_active = true; phase = LANDED }   /* last-resort backstop */
       :: true -> descent_ticks = descent_ticks + 1  /* SIG_TICK: increment counter */
       :: true -> skip                               /* SIG_ABORT: ignored */
       fi
   ```

   Add declarations at the top of the process:
   ```promela
   #define DESCENT_BACKSTOP_TICKS 10
   byte descent_ticks = 0;
   bool beacon_active = false;
   ```

   Reset `descent_ticks = 0` on phase entry — add to the existing phase-entry reset block or wrap MAIN_DESCENT entry in an `atomic` that clears it.

6. **Update P7 comment** at `rocketchip_fd.pml:227-241` — remove "KNOWN FAILING" language, mark resolved, cite IVP-121. Keep the failure mode description for historical context.

7. **Host tests** in `test/test_flight_director.cpp`:

   - **`MainDescentBaroStationaryFiresLanding`** — enter MAIN_DESCENT, inject baro-stationary guard sustained → expect `SIG_LANDING` dispatch → LANDED
   - **`MainDescentEskfFaultAloneStaysInDescent`** — enter MAIN_DESCENT, set ESKF health to fault, baro still moving → expect stay in MAIN_DESCENT (conjunction requires both)
   - **`MainDescentEskfFaultPlusBaroStationaryFiresLanding`** — both conditions → LANDED, verify NO beacon published (nominal landing, not distress)
   - **`MainDescentBackstopFiresLandingWithBeacon`** — no other signals, set fake time past `descent_max_duration_ms` → LANDED + `SIG_BEACON_ACTIVE` published
   - **`MainDescentBackstopDoesNotFireBeforeTimeout`** — same as above but elapsed < max_duration → stay in MAIN_DESCENT

   All tests modeled on existing `CoastTimeoutForcedApogee` pattern at `test/test_flight_director.cpp:357`.

8. **Whiteboard cleanup deferred to IVP-124.** The "MAIN_DESCENT Has No Timeout Fallback — SPIN P7 Fails" open flag at `AGENT_WHITEBOARD.md:43-49` will be removed in IVP-124's whiteboard cleanup pass alongside the other stale entries. Until then, the whiteboard entry becomes *technically stale but not yet edited* — acceptable for one IVP's worth of time. The IVP-121 commit message should cite "closes whiteboard MAIN_DESCENT P7 flag — removal in IVP-124" for traceability.

**[GATE]:**
- 5 new host tests pass
- SPIN re-run: all 11 existing properties + P7 pass under weak fairness (`-f` flag)
- Promela model compiles in `pan.exe` without syntax errors
- Bench flight sim regression: `scripts/bench_flight_sim.py` 9/9 PASS (IVP-73)
- HW verify 1 (nominal path): flash, run bench flight sim through to MAIN_DESCENT, place device on desk, verify transition to LANDED within ~5 s via baro-stationary (no beacon)
- HW verify 2 (ESKF fault conjunction): flash, attach debug probe, break in `flight_director_evaluate_guards` on MAIN_DESCENT entry. Inject `kHealthEskfFault` bit pattern into `fused.health_primary` via GDB `set var` (preferred) — OR grep for an existing CLI health-inject command first (`grep -rn "health_inject" src/`) and use that if present. Verify LANDED only transitions after `guard_evaluator_is_sustained(kBaroStationary)` also returns true. Alternatively, use the bench flight sim to drive MAIN_DESCENT and freeze ESKF updates programmatically.
- HW verify 3 (backstop): flash, enter MAIN_DESCENT, block the baro-stationary path somehow (or use a short test profile with `descent_max_duration_ms=60000`), verify backstop fires + beacon LED pattern activates after timeout
- SPIN property summary committed: `spin -a rocketchip_fd.pml && cc -o pan pan.c && ./pan -a -f` all properties PASS

**[DIAG]:** Backstop fires during nominal bench sim → `descent_max_duration_ms` too short, check profile. ESKF-fault path fires when it shouldn't → conjunction logic wrong, verify both conditions checked. SPIN P7 still fails → check Promela counter increments and reset on phase entry; run `./pan -a -f -N p_liveness_flight_completes` for targeted check. Nominal path fails → `guard_baro_stationary` not registered with `SIG_LANDING` in guard evaluator (IVP-120 regression).

---

### Stage P7 Design Principle (reference)

Landed detection is a voted multi-channel problem with physically independent signals. The current single-channel design (ESKF velocity via `guard_stationary`) has a single point of failure: when the ESKF dies, the only exit from MAIN_DESCENT disappears.

The correct class of solution: two or more *physically independent* landing signals (ESKF velocity, raw baro altitude derivative), with any single independent channel sufficient to trigger transition, plus a time-based backstop that fires only when all physical channels are silent. Physical channels are primary; time is last resort with explicit distress semantics (beacon publish).

**Reference prior art:** ArduPilot Copter's `land_complete_maybe()` votes vertical speed + climb rate + accel-near-1g + throttle. No single signal holds exclusive power.

**Failure modes closed by Stage P7:**
- **FM1 (ESKF dead, velocity frozen)** — closed by baro-stationary guard (IVP-120)
- **FM2 (ESKF alive but ZUPT-corrupted)** — partially closed; baro-stationary will fire once on the ground even if ESKF is slowly drifting
- **FM3 (IMU dead)** — closed by baro-stationary guard (IVP-120); baro is independent of IMU
- **FM4 (baro turbulence mid-descent)** — not a liveness issue; resolves on landing when turbulence source is removed
- **FM5 (HAB nominal drift)** — deferred; HAB profile doesn't exist yet
- **FM6 (all sensors dead)** — closed by backstop (IVP-121) with distress beacon

### Future Generalization — audit for other critical phase transitions (post-Stage 15)

Stage P7's pattern (**raw-sensor independent fallback between ESKF-primary and PIO-backup-timer layers**) generalizes to other flight-director transitions. The layered safety stack is:

1. **Primary:** ESKF-fused signal (current design for all guards)
2. **Middle (new in Stage P7 for landing):** Raw physical sensor, ESKF-independent
3. **Last resort:** PIO hardware backup timer (Stage 11)
4. **Recovery:** Distress beacon if middle + primary both fail (new in IVP-121)

Stage 11 already provides layer 3 for the pyro-deploy transitions — drogue and main deploy timers run in PIO and fire regardless of Core 0 / ESKF state. That's why apogee and main-deploy don't *currently* fail SPIN liveness. But they rely on hardcoded profile timers as the physical-independence fallback rather than actual sensor detection, which matches the same "time instead of physics" pattern we rejected for MAIN_DESCENT.

Candidates for a future multi-channel audit stage (post-first-flight):

| Transition | Current primary | Current last-resort | Middle-layer candidate |
|---|---|---|---|
| COAST→DROGUE (apogee) | `guard_apogee_velocity` + `guard_baro_peak` (both ESKF-fed via `vert_vel_eskf`) | PIO drogue timer | Raw baro derivative sign change |
| DROGUE→MAIN_DESCENT (main deploy alt) | `guard_main_deploy_altitude` (ESKF altitude) | PIO main timer | Raw baro altitude threshold |
| Launch/burnout detection | Raw IMU accel magnitude (already independent) | — | Already independent — no gap |

**Not Stage P7 scope** — Stage P7 closes a concrete SPIN property failure with a specific failure mode. Apogee/main-deploy are latent architectural improvements, not active bugs. They're acceptable risk for first flight because the PIO backup timers provide formal liveness. The audit should happen with real flight data from Stage 16C informing which middle-layer guards are actually needed.

**Tracked in:** IVP-124 whiteboard cleanup adds a forward-looking entry pointing to this section.

---

## Stage 15: Pre-Flight Radio + Station

**Purpose:** Harden station UX and radio command path for first real flight. Station ARM acknowledged end-to-end over LoRa, distance-to-rocket functional, help menu and whiteboard current.

**Prerequisites:** Stage P7 complete. Stage 15 runs on a repo where SPIN P7 passes and MAIN_DESCENT has multi-channel landing detection.

| Step | Title | Brief Description |
|------|-------|------------------|
| IVP-122 | Half-Duplex ACK + ARM Confirm UX | Dedicated CCSDS APID for command ACK (Option 2). Station ARM flow: `a` → type `ARM` caps + Enter → send → retry 3× wait ACK. Persistent ARM state indicator on dashboard. |
| IVP-123 | Distance-to-Rocket Finish | Fill `cmd_station_distance()` stub using cached RX nav + haversine. Add bearing. Freshness check. Dashboard row. Host test for haversine math. |
| IVP-124 | Station Help + Whiteboard Cleanup | Refresh station help text to match actual wired keys. Remove stale IVP-103 whiteboard entry. Audit + clean other stale station items. |

---

### IVP-122: Half-Duplex ACK + ARM Confirm UX

**Prerequisites:** Stage P7 complete, IVP-62c complete (station→vehicle ARM command path)

**Implement:** Add a dedicated CCSDS APID for command ACK, implement station-side pending-command state machine with wall-clock retry timer, implement multi-char ARM confirm UX on station (`a` → type `ARM` → Enter), and add a persistent ARM state indicator to the dashboard.

1. **New CCSDS APID for command ACK.** Pick next free APID (likely `0x003` if nav is `0x001`, diag was reserved at `0x002`). Declare in `include/rocketchip/telemetry_encoder.h`:
   ```cpp
   constexpr uint16_t kApidCommandAck = 0x003;
   ```

2. **New `CommandAckPacket` struct** in `include/rocketchip/telemetry_encoder.h`:
   ```cpp
   struct __attribute__((packed)) CommandAckPacket {
       uint8_t  cmd_seq;      // Sequence number of ack'd command
       uint16_t cmd_id;       // MAVLink command ID (e.g., kMavCmdArmDisarm)
       uint8_t  result;       // 0=accepted, 1=denied, 2=failed, 3=in_progress
       uint8_t  reserved;
   };  // 5 bytes — fits comfortably in smallest CCSDS frame
   ```

3. **Encoder/decoder** in `src/telemetry/telemetry_encoder.cpp`:
   - `encode_command_ack(const CommandAckPacket& ack, uint8_t* out, size_t& out_len)` — emits CCSDS primary header with APID 0x003, payload, CRC-16
   - `decode_command_ack(const uint8_t* buf, size_t len, CommandAckPacket& out)` — validates header, APID, CRC, extracts payload

4. **APID routing skeleton** — first second-APID consumer. In the station's telemetry RX path (likely in `ao_telemetry.cpp` `QS_ON_RX_BYTE` or equivalent), add a dispatch switch on APID:
   ```cpp
   switch (header.apid) {
       case kApidTelemetry:   handle_telemetry_packet(...); break;
       case kApidCommandAck:  handle_command_ack_packet(...); break;
       default: /* log unknown APID, drop */ break;
   }
   ```
   This satisfies the deferred "APID routing dispatch" item from `docs/RADIO_TELEMETRY_STATUS.md:118`.

5. **Vehicle command dispatch writes ACK** in `src/telemetry/mavlink_rx.cpp:167-228`:
   - After successful dispatch of `MAV_CMD_COMPONENT_ARM_DISARM` (or any command), build a `CommandAckPacket` with the received command seq (read from COMMAND_LONG `confirmation` field), command ID, and `result = 0` (accepted)
   - Enqueue to AO_Radio via new helper `AO_Telemetry_send_command_ack(seq, cmd_id, result)` (small wrapper around existing TX path)
   - Result codes: accepted on successful dispatch, denied if phase-invalid, failed if dispatch exception

6. **Station side command sequence tracking** in `src/active_objects/ao_telemetry.cpp`:
   - Add `static uint8_t s_cmd_seq = 0;` — monotonic counter
   - Add `static struct { bool pending; uint8_t seq; uint16_t cmd_id; uint32_t sent_ms; uint8_t retries_left; } s_pending_cmd;`
   - Modify `AO_Telemetry_send_command()` signature: add a parameter to indicate "tracked" (ARM) vs "fire-and-forget" (existing behavior). Tracked commands: increment `s_cmd_seq`, populate `s_pending_cmd`, pack seq into MAVLink `confirmation` field
   - New tick handler behavior: each tick, if `s_pending_cmd.pending`, check `(now_ms - s_sent_ms) >= 3000` (3-second retry window). If yes and `retries_left > 0`, re-send with same seq, decrement retries. If `retries_left == 0`, clear pending, publish a `SIG_CMD_FAILED` or equivalent so the UI can show "ARM no ACK".
   - New ACK handler: when `handle_command_ack_packet()` fires, if `ack.cmd_seq == s_pending_cmd.seq && ack.cmd_id == s_pending_cmd.cmd_id`, clear pending, publish `SIG_CMD_ACKED`.

7. **Station ARM confirm UX** in `src/cli/rc_os_commands.cpp` (case 'a' at line 1292) and `src/cli/rc_os.cpp` (input loop):
   - Replace the current case 'a' single-line implementation with a call to a new `cmd_station_arm_start()` helper
   - `cmd_station_arm_start()` sets a static flag `g_arm_confirm_active = true`, stores `g_arm_confirm_start_ms = now_ms`, clears `g_arm_confirm_buffer[4] = {0}`, calls `rc_os_dashboard_pause()`, prints `"Type ARM in caps then Enter to confirm (5s):"`
   - In `rc_os.cpp` input loop, if `g_arm_confirm_active`:
     - If `now_ms - g_arm_confirm_start_ms > 5000` → print `"ARM aborted (timeout)"`, clear flag, call `rc_os_dashboard_resume()`
     - Read character with `getchar_timeout_us(0)`
     - On `\r` or `\n`: if buffer == `"ARM"` → call `AO_Telemetry_send_command(kMavCmdArmDisarm, 1.0f, /*tracked=*/true)`, print `"[CMD] ARM sent, waiting for ACK..."`, clear flag. Dashboard stays paused until ACK/timeout signal comes back. Else → print `"ARM aborted (bad input)"`, clear flag, resume dashboard.
     - On printable char: append to buffer (max 3 chars); fourth char with no Enter is an abort
     - Skip normal single-key dispatch while flag is active
   - On `SIG_CMD_ACKED` from AO_Telemetry (subscribed via AO_RCOS or a direct callback): print `"[CMD] ARM ACK'd (seq=X)"`, resume dashboard
   - On `SIG_CMD_FAILED`: print `"[CMD] ARM no ACK (3 retries exhausted)"`, resume dashboard

8. **Dashboard pause/resume API** in `src/cli/rc_os_dashboard.cpp`:
   - `void rc_os_dashboard_pause()` — sets `g_dashboard_paused = true`. Render function checks flag at entry and returns immediately if paused.
   - `void rc_os_dashboard_resume()` — sets `g_dashboard_paused = false`, forces full redraw on next tick (clear screen, reprint frame)

9. **Persistent ARM state indicator on dashboard** in `src/cli/rc_os_dashboard.cpp`:
   - New top-of-dashboard field: `"ARM: [IDLE ]"` / `"ARM: [ARMED]"` / `"ARM: [  ?  ]"` rendered with color (green IDLE, red ARMED, yellow unknown)
   - Source data: `rc::TelemetryState::flight_state` from `AO_Telemetry_get_rx_state()`
   - Readable at a glance from several feet away — use 2-character padding and color codes, maybe double-width if ANSI supports it

10. **DISARM — gap discovered, must be addressed in this IVP.** On the vehicle, `d` → DISARM via `rc_os.cpp:336`. On the station, `d` is overloaded to `cmd_station_distance()` in the `kRadioModeRx` branch (`rc_os_commands.cpp:1265`). **There is currently no station-side key wired to send DISARM over LoRa.** This is not acceptable for Stage 15's "ready for first flight" bar — if ARM is acknowledged end-to-end but DISARM can't be sent, that's worse than the current state. Fix: pick an unambiguous station key for DISARM (proposal: `X` capital-only, since lowercase `x` is ABORT on vehicle and `x` is erase-flights on station in the TX branch). Wire it to send an unconfirmed single-key `AO_Telemetry_send_command(kMavCmdArmDisarm, 0.0f, /*tracked=*/true)`. Also tracked for ACK (same reasoning as ARM — operator needs to know it got through), but NO confirm prompt. Print `[CMD] DISARM sent, waiting for ACK...` → `DISARM ACK'd` or `DISARM no ACK`. Update station help text in IVP-124 to include the new key.

    **Council decision rationale:** ARM requires confirm because accidental arming is dangerous. DISARM has the opposite asymmetry — you want it to fire on the first try, under stress, with no modal gate. But it still needs to be acknowledged so the operator knows the command was received. Fire-and-forget DISARM is as bad as fire-and-forget ARM.

**Host tests:**
- `test/test_telemetry_encoder.cpp` — add `CommandAckPacketEncodeDecode`, `CommandAckPacketApidRouting`
- New `test/test_station_arm_ack.cpp`:
   - `ArmConfirm_ValidInputSendsCommand` — buffer = "ARM", Enter → send invoked with seq
   - `ArmConfirm_LowercaseRejected` — buffer = "arm", Enter → abort, no send
   - `ArmConfirm_PartialInputTimeout` — buffer = "AR", wait 5s → abort
   - `ArmConfirm_OverflowAborts` — 4 chars without Enter → abort
   - `PendingCmd_AckClearsPending` — mock send, inject ack with matching seq → pending cleared
   - `PendingCmd_WrongSeqIgnored` — mock send, inject ack with different seq → still pending
   - `PendingCmd_RetryOnTimeout` — mock send, advance clock 3s, verify re-send, seq unchanged
   - `PendingCmd_ExhaustedAfterThreeRetries` — advance clock 12s with no ack → SIG_CMD_FAILED published

**[GATE]:**
- 8+ host tests pass
- Target build compiles clean
- HW verify 1 (nominal): station + vehicle on bench, press `a`, type `ARM`, Enter → observe `ARM sent, waiting for ACK...` then `ARM ACK'd` within ~1 s (first retry window). Vehicle reports ARMED on MAVLink/USB cross-check. Dashboard resumes with red `[ARMED]` indicator.
- HW verify 2 (typo): press `a`, type `arm` lowercase, Enter → `ARM aborted`, dashboard resumes, vehicle still IDLE
- HW verify 3 (timeout): press `a`, wait 5 s with no input → `ARM aborted (timeout)`, dashboard resumes
- HW verify 4 (ACK loss): press `a`, type `ARM`, Enter, briefly remove vehicle antenna → observe 3 retries, then `ARM no ACK`, reattach antenna, re-ARM successfully on next attempt
- HW verify 5 (DISARM path): with vehicle in ARMED state, press `X` (capital) on station → immediate `DISARM sent` → `DISARM ACK'd` within ~1 s, vehicle transitions back to IDLE, dashboard shows green `[IDLE]` indicator
- HW verify 6 (DISARM ACK loss): arm vehicle, remove antenna briefly, press `X` → observe retries then `DISARM no ACK`, reattach, re-send DISARM successfully
- Dashboard ARM indicator visually readable from ~5 feet (bench test)

**[DIAG]:** ACK never arrives → check APID routing switch in station RX, verify encoder emits correct APID. Retries don't trigger → check wall-clock delta computation uses `now_ms`, not frame count. Dashboard corruption after resume → verify `rc_os_dashboard_resume()` forces full redraw, not partial update. Multi-char buffer corruption → verify `g_arm_confirm_active` flag gates normal single-key dispatch correctly.

---

### IVP-123: Distance-to-Rocket Finish

**Prerequisites:** None (independent of IVP-122, but slotted after per OoO)

**Implement:** Fill in the stubbed `cmd_station_distance()` at `src/cli/rc_os_commands.cpp:1162-1173` using the existing `haversine_m()` helper and the cached RX nav state from `AO_Telemetry_get_rx_state()`. Add bearing. Add dashboard row. Host test for the math.

1. **Add bearing helper** next to `haversine_m()` in `src/cli/rc_os_commands.cpp` (around line 1145):
   ```cpp
   [[maybe_unused]]
   static float bearing_deg(int32_t lat1_e7, int32_t lon1_e7,
                             int32_t lat2_e7, int32_t lon2_e7) {
       static constexpr float kDegToRad = 3.14159265f / 180.0f;
       static constexpr float kRadToDeg = 180.0f / 3.14159265f;
       static constexpr float kScale    = 1e-7f;

       float lat1 = static_cast<float>(lat1_e7) * kScale * kDegToRad;
       float lat2 = static_cast<float>(lat2_e7) * kScale * kDegToRad;
       float dlon = static_cast<float>(lon2_e7 - lon1_e7) * kScale * kDegToRad;

       float y = sinf(dlon) * cosf(lat2);
       float x = cosf(lat1) * sinf(lat2) - sinf(lat1) * cosf(lat2) * cosf(dlon);
       float bearing = atan2f(y, x) * kRadToDeg;
       return fmodf(bearing + 360.0f, 360.0f);  // normalize to 0-360
   }
   ```

2. **Fill in `cmd_station_distance()`:**
   ```cpp
   static void cmd_station_distance() {
       if constexpr (!kRadioModeRx) { return; }
       if (!g_gpsInitialized || g_bestGpsFix.fix_type < 2) {
           printf("Distance: station GPS has no fix\n");
           return;
       }
       const rc::RxTelemSnapshot* rx = AO_Telemetry_get_rx_state();
       if (!rx || !rx->valid) {
           printf("Distance: no vehicle telemetry received\n");
           return;
       }
       uint32_t age_ms = to_ms_since_boot(get_absolute_time()) - rx->met_ms;
       if (age_ms > 5000) {
           printf("Distance: telemetry stale (%lu ms)\n", age_ms);
           return;
       }
       if (rx->telem.lat_1e7 == 0 && rx->telem.lon_1e7 == 0) {
           printf("Distance: no vehicle GPS\n");
           return;
       }

       float dist = haversine_m(g_bestGpsFix.lat_1e7, g_bestGpsFix.lon_1e7,
                                rx->telem.lat_1e7, rx->telem.lon_1e7);
       float bearing = bearing_deg(g_bestGpsFix.lat_1e7, g_bestGpsFix.lon_1e7,
                                   rx->telem.lat_1e7, rx->telem.lon_1e7);
       printf("Distance: %.0f m  Bearing: %.0f°\n",
              static_cast<double>(dist), static_cast<double>(bearing));
   }
   ```

3. **Dashboard row** in `src/cli/rc_os_dashboard.cpp` — add a new rendered field below the existing Lat/Lon row:
   - `"Dist: 123m  @ 045°\n"` (nominal)
   - `"Dist: stale\n"` (freshness expired)
   - `"Dist: no vehicle GPS\n"` (vehicle lat/lon zero)
   - `"Dist: --\n"` (no station GPS or no RX yet)
   - Match existing dashboard style (cursor positioning, color codes)

4. **Host test** — new `test/test_haversine.cpp` (or add to existing test file):
   - `Haversine_SamePoint_ReturnsZero` — identical lat/lon → 0 m
   - `Haversine_KnownPair_MatchesExpected` — e.g., two points 1 km apart with known lat/lon diff → within ±10 m
   - `Haversine_LongDistance` — e.g., 100 km → within ±100 m
   - `Bearing_NorthOfPoint_Returns0` — point north → ~0° (within 1°)
   - `Bearing_EastOfPoint_Returns90` — point east → ~90°
   - `Bearing_Normalized_0To360` — wrapping cases

**[GATE]:**
- 6+ host tests pass
- Target build compiles clean
- HW verify 1 (bench): station + vehicle both with GPS fix on same bench → distance < 10 m, bearing reasonable
- HW verify 2 (stale): remove vehicle antenna → after 5 s, `"Dist: stale"` appears on dashboard + CLI
- HW verify 3 (no-GPS): power vehicle without GPS (indoor, no antenna) → `"Dist: no vehicle GPS"` appears
- HW verify 4 (walk): simulate vehicle movement by manually tweaking cached RX position in GDB, or walk the station 10-20 m away from a fixed vehicle → distance updates

**[DIAG]:** Distance is wildly wrong → check lat/lon scale (e7 vs e6), check input ordering (lat1, lon1, lat2, lon2). Bearing is 180° off → swap atan2 arguments. Stale never triggers → check `age_ms` calculation uses consistent time units.

---

### IVP-124: Station Help + Whiteboard Cleanup

**Prerequisites:** IVP-122 complete (so help can include the new ARM confirm flow)

**Implement:** Refresh station help text to match actual wired keys. Remove stale `IVP-103 Station GPS Push` item from `AGENT_WHITEBOARD.md`. Audit other station-related whiteboard items and close any that are now resolved.

1. **Update station help output** in `src/active_objects/ao_rcos.cpp:187-201`:
   ```cpp
   printf("Status:  h-Help  s-Sensor  b-Boot  p-Preflight\n");
   printf("Radio:   t-Status  r-Rate  m-Mode(ANSI/CSV/MAVLink)\n");
   printf("Station: g-GPS  d-Distance  p-GPS-Push\n");
   printf("Command: a-ARM(confirm)  X-DISARM\n");
   printf("Flight:  l-FlushLog  x-Erase\n");
   ```
   Notes: (1) `p` overloads — Preflight in main help, GPS-Push in station submenu; context-gated by `kRadioModeRx`. (2) Capital `X` is station DISARM (added in IVP-122), distinct from lowercase `x` which is Erase-flights in TX mode or ABORT on vehicle. Emphasize the case distinction in the help row if needed.

2. **Cross-reference with the dispatcher** at `src/cli/rc_os_commands.cpp:1260-1305` to ensure every wired key is documented. If any key is in the dispatcher but not in help, add it. If any is in help but not wired, remove it from help (or wire it, if that's the fix).

3. **AGENT_WHITEBOARD.md cleanup** (protected file — requires explicit commit-time approval). Close these stale/resolved entries:
   - Line 78: `- **IVP-103 Station GPS Push** — Needs radio command path.` — `cmd_station_gps_push()` has been implemented since Stage 7 Take 2
   - Lines 43-49: `MAIN_DESCENT Has No Timeout Fallback — SPIN P7 Fails` — **closed by IVP-121** (multi-channel landing detection + backstop)
   - "Stage 12A polish items" block around line 56: specifically `station help menu` (closed by this IVP) and `RSSI bar PIO contention` — audit current state, close if resolved, keep open if still relevant
   - Line 17 area: Stage 14 "IVP-119 Baro field rename + guard_baro_peak ESKF-dependency comment" note if it was added — close it now that IVP-119 is merged

4. **Add forward-looking whiteboard entry** — create new "Open Flags" item for post-Stage-15 consideration:
   ```
   ### Multi-Channel Phase Transition Audit (post-Stage 15)

   Stage P7 added raw-sensor independent fallback for MAIN_DESCENT→LANDED (baro
   stationarity between ESKF-primary and PIO-backup layers). The same layered
   pattern applies to COAST→DROGUE (apogee) and DROGUE→MAIN_DESCENT (main deploy
   altitude) transitions. Currently these rely on ESKF-fused guards with PIO
   timer backup (Stage 11). A middle layer of raw-baro physical detection would
   close the same class of failure mode (ESKF dead mid-coast → hardcoded timer
   fires pyro instead of physical apogee detection).

   Not urgent — PIO backup timers provide formal liveness. Audit should happen
   after Stage 16C field test data informs which transitions need hardening.
   See Stage P7 plan file ("Future Generalization") for design pattern and
   candidate transitions.
   ```

4. **Optional grep sweep:** `grep -rn "IVP-103" .` — remove or update any other stale references to IVP-103 as deferred work

**[GATE]:**
- Station help output on HW shows all actually-wired keys with correct RX/TX conditional sections
- `AGENT_WHITEBOARD.md` no longer contains the stale IVP-103 entry or the stale MAIN_DESCENT P7 entry
- `grep -rn "IVP-103.*Needs.*radio" .` returns zero matches
- Visual inspection: press `h` on station HW, confirm help output matches dispatcher
- Git diff shows only documentation + help string changes, no behavioral code changes

**[DIAG]:** Help text wraps oddly on narrow terminals → keep each line ≤ 70 chars. Dispatcher has a key not in help → find the case in `cli_handle_unhandled_key`, add to help.

---

## Stage Ordering

**Stage P7 runs first, then Stage 15.**

**Why Stage P7 first:**
- Closes a real liveness gap (SPIN P7). Flying with a known failing safety property is wrong on principle.
- Architecturally independent from Stage 15 (no shared files except possibly `mission_profile.h`, and P7 lands first so Stage 15 sees a clean header)
- Makes the repo's formal model consistent before Stage 15's bigger design decisions land
- Latent bug fix in IVP-119 (`baro_vvel` rename) is small and should land cleanly before any other work touches the FusedState

**Why Stage 15 second:**
- All three Stage 15 IVPs are on the station-side command/UX path, unrelated to flight director state machine work
- IVP-122 is the biggest station item with the most new code; it lands on a clean repo with P7 settled
- IVP-123 and IVP-124 are order-insensitive after IVP-122 but IVP-124 must follow IVP-122 so help can reference the new ARM confirm flow

**Within-stage ordering:**
- P7: IVP-119 → IVP-120 → IVP-121 (strict dependency chain — field must exist, then guard, then FD wiring)
- 15: IVP-122 → IVP-123 → IVP-124 (soft dependency — 124 after 122 for help content accuracy; 123 anywhere after 122)

---

## Verification Summary (all IVPs)

### Host tests
- `test/test_flight_director.cpp` — 5 new tests for MAIN_DESCENT paths (nominal baro, ESKF fault alone, ESKF fault + baro, backstop fires, backstop doesn't fire early)
- `test/test_guards.cpp` — 5 new tests for `guard_baro_stationary` mechanics
- `test/test_telemetry_encoder.cpp` — 2 new tests for CommandAckPacket encode/decode + APID routing
- `test/test_station_arm_ack.cpp` (new) — 8 new tests for ARM confirm state machine + pending command retry
- `test/test_haversine.cpp` (new or merged into existing) — 6 new tests for distance + bearing math

### Formal verification (SPIN)
- `spin -a rocketchip_fd.pml && cc -o pan pan.c && ./pan -a -f` — all 11 existing properties + P7 passing under weak fairness

### Bench regression
- `scripts/bench_flight_sim.py` — 9/9 PASS (IVP-73 regression)

### Hardware verification
- **IVP-119:** Flash, confirm no behavioral change, `baro_pressure_pa` readable via GDB
- **IVP-120:** Flash, bench test guard firing when device is stationary on desk
- **IVP-121:** Flash, bench flight sim to MAIN_DESCENT, verify nominal baro-stationary landing + ESKF fault conjunction + backstop distress path
- **IVP-122:** Full ARM cycle HW test (nominal, typo, timeout, ACK loss, DISARM unchanged)
- **IVP-123:** Distance on bench + stale + no-GPS + walk test
- **IVP-124:** Visual help inspection, whiteboard diff

### Soak
- 10-minute post-stage soak after final IVP lands: bench flight sim running in loop, watchdog reset tracking, full host test regression

### Git discipline
- One commit per IVP (6 commits total)
- Each commit message cites the IVP number and the 1-sentence gate summary
- `AGENT_WHITEBOARD.md` edits are user-approved at commit time (protected file)
- `docs/IVP.md` updated in a single commit alongside IVP-119 (the first P7 IVP) to reflect the new Stage P7 + renumbered Stage 15 entries
- `docs/PROJECT_STATUS.md` and `CHANGELOG.md` updated after final IVP lands (stage complete)

---

## IVP.md Amendments Required

This plan adds 3 new stage entries to `docs/IVP.md`. Changes needed (protected file — user approval required):

1. **Intro note (line 25-27 area):** Add `Stage P7` line describing the out-of-sequence safety pass between Stage 14 and Stage 15.

2. **Overview table (line 113-116 area):** Insert row for Stage P7 between Stage 14 and Stage 15:
   ```
   | **P7** | **MAIN_DESCENT Liveness Fix** | **Phase 9** | **IVP-119 — IVP-121** | **Full** | **(out-of-sequence)** |
   | 15 | Pre-Flight Radio + Station | Phase 9 | IVP-122 — IVP-124 | Full | |
   ```
   Update Stage 15 row to show the IVPs assigned. Stage 16 and Stage 17 rows remain unchanged (still placeholders).

3. **Stage body insertion:** Insert the full Stage P7 section (IVP-119 through IVP-121) after the Stage 14 "IVP-118 Final Audit" section (line 3208 area) and before the Stage 15 section header.

4. **Stage 15 body:** Replace the current placeholder Stage 15 table with the expanded version (IVP-122 / 123 / 124 with Brief Description rows, followed by the full IVP-122 / 123 / 124 sections copied from this plan file).

5. **Regression test matrix (line 3260+):** Add row for the new MAIN_DESCENT liveness detection chain:
   `| MAIN_DESCENT guard change | IVP-119 — IVP-121 | Baro stationary, backstop, SPIN P7 |`

All edits gated on user approval per PROTECTED_FILES.md rules.

---

## Critical Files (reference)

### Stage P7
- `include/rocketchip/fused_state.h` — FusedState struct
- `src/active_objects/ao_logger.cpp:151` — `AO_Logger_populate_fused_state()` (misnomer origin)
- `src/flight_director/guard_functions.h/.cpp` — guard implementations, `guard_baro_peak()` comment fix
- `src/flight_director/guard_evaluator.h/.cpp` — guard registration, phase bitmask
- `src/flight_director/mission_profile.h` — struct, existing `*_timeout_ms` fields
- `src/flight_director/flight_director.cpp:446-463` — `state_main_descent`
- `src/flight_director/flight_director.cpp:366-380` — COAST timeout pattern reference
- `profiles/rocket.cfg`, `profiles/passive.cfg` — profile parameters
- `scripts/generate_profile.py` — codegen
- `tools/spin/rocketchip_fd.pml:149-155` — MAIN_DESCENT Promela block
- `tools/spin/rocketchip_fd.pml:227-241` — P7 open-item comment
- `test/test_flight_director.cpp:357` — `CoastTimeoutForcedApogee` test pattern reference

### Stage 15
- `src/cli/rc_os_commands.cpp:1260-1305` — key dispatcher
- `src/cli/rc_os_commands.cpp:1292-1298` — current single-key ARM
- `src/cli/rc_os_commands.cpp:1130-1145` — `haversine_m()` helper
- `src/cli/rc_os_commands.cpp:1162-1173` — `cmd_station_distance()` stub
- `src/cli/rc_os_commands.cpp:1180-1192` — `cmd_station_gps_push()` (already implemented, referenced by D)
- `src/cli/rc_os.cpp` — single-key input loop (needs multi-char state)
- `src/cli/rc_os_dashboard.cpp` — ANSI dashboard render
- `src/active_objects/ao_rcos.cpp:187-201` — station help text
- `src/active_objects/ao_telemetry.cpp:431-458` — `AO_Telemetry_send_command()`
- `src/active_objects/ao_telemetry.cpp:84, 426` — `RxTelemSnapshot`, `get_rx_state()`
- `src/telemetry/mavlink_rx.cpp:167-228` — vehicle command dispatch
- `include/rocketchip/telemetry_encoder.h` — APID constants, new `CommandAckPacket`
- `include/rocketchip/telemetry_state.h` — TelemetryState wire format (unchanged by this plan)
- `include/rocketchip/radio_scheduler.h` — RadioPhase state machine (reference only)
- `docs/RADIO_TELEMETRY_STATUS.md:116-118` — APID routing deferred item (closed by IVP-122)
- `AGENT_WHITEBOARD.md:43-49, 78` — stale entries to remove in IVP-124
