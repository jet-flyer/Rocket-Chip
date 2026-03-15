# Stage 8: Flight Director Implementation Plan

## Context

Stage 8 adds a hierarchical state machine (UML statecharts / QEP) to the RocketChip flight computer. This is the core flight logic — state tracking from IDLE through LANDED, guard-based event detection, command validation, and action execution on transitions. Council-reviewed and unanimously approved (9 decisions). IVP-66 through IVP-75 (10 steps).

**Breakpoint:** Commit `504bb0e` on `main`. 358/358 host tests pass, target builds clean, pushed to remote. If anything goes wrong: `git reset --hard 504bb0e`.

**Branch strategy:** Feature branch `stage8/flight-director` from `504bb0e`. Merge to main after each IVP pair passes all gates.

---

## Part A: Foundation (IVP-66 through IVP-70)

### IVP-66: Watchdog Recovery Policy
**Complexity:** Medium | **New files:** 2 | **Modified:** 2 | **Host tests:** ~10

**Correction:** Bug fix in IVP text is already done (sentinel at main.cpp:1777-1786). Real work is the new policy infrastructure.

**Implement:**
- `src/watchdog/watchdog_recovery.h` + `.cpp` — scratch[5..7] layout (flight state, tick fn ID, reboot counter, magic validator), `watchdog_recovery_init()`, `watchdog_recovery_update_scratch()`, safe-mode detection (>3 rapid reboots), LAUNCH_ABORT flag, CLI acknowledge
- `main.cpp` — call recovery init in `init_hardware()`, update scratch in `watchdog_kick_tick()`, wire LAUNCH_ABORT flag
- ESKF failure backoff in `eskf_tick()` — consecutive init→diverge counter, disable after 5 cycles, CLI re-enable

**HW gate:** 5-min soak zero WDT fires; GDB verify scratch registers; forced WDT timeout increments counter; 3x rapid WDT triggers safe mode (NeoPixel solid red)

---

### IVP-67: Toolchain Validation (QP/C + STARS + QM + SPIN)
**Complexity:** Medium | **New files:** ~6 (vendored QEP) | **Host tests:** 0 (QEP host compilation verified in IVP-68's test library)

**Implement:**
- Vendor QEP into `lib/qep/`: `qep_hsm.c`, `qp.h`, `qp_pkg.h`, custom `qp_port.h` (Pico SDK critical sections, `Q_NASSERT`), `qp_config.h`
- License: GPL-3.0 (compatible with RocketChip GPL-3.0-or-later), add `lib/qep/LICENSE.txt`
- CMake: `add_library(qep STATIC ...)` with `extern "C"` handling
- Trivial 3-state HSM test on hardware (temporary, removed after gate)
- STARS/QM/SPIN evaluation (gated — document outcome in AGENT_WHITEBOARD)
- **Fallback if STARS fails:** Hand-authored QEP state handlers + manual Promela model for SPIN verification

**HW gate:** QEP compiles+links for ARM Cortex-M33; trivial HSM runs on device; dispatch <5µs

---

### IVP-68: QEP Integration + Phase Skeleton
**Complexity:** Large | **New files:** 4 | **Modified:** 3 | **Host tests:** ~18

**Implement:**
- `src/flight_director/flight_state.h` — `FlightPhase` enum (kIdle=0, kArmed, kBoost, kCoast, kDrogueDescent, kMainDescent, kLanded, kAbort), `FlightState` struct (phase, markers, counters)
- `src/flight_director/flight_director.h` + `.cpp` — QHsm subclass, 9 state handler functions (7 top-level + 2 DESCENT sub-phases), HSM hierarchy via Q_SUPER(), signal enum (SIG_TICK through SIG_RESET starting after Q_USER_SIG)
- `main.cpp` — `static FlightDirector g_director`, add `flight_director_tick()` at 100Hz between eskf and logging ticks, `populate_fused_state()` reads `g_director.state.current_phase`
- New CLI sub-menu `RC_OS_MENU_FLIGHT` accessed via `'f'` from main menu. Mnemonic keys inside: `'a'`=ARM, `'d'`=DISARM, `'l'`=LAUNCH, `'b'`=BURNOUT, `'p'`=APOGEE, `'m'`=MAIN_DEPLOY, `'n'`=LANDING, `'x'`=ABORT, `'r'`=RESET, `'s'`=status, ESC=back. Relocate existing `'f'`(flight list) and `'d'`(download) commands into a data sub-menu or reassign
- Recovery boot: `g_recoveryBoot` flag routes to correct initial state per IVP-66 policy
- `'s'` status output extended: show current phase + flight markers

**ABORT state specification (Council Amendment #1):**
- ABORT-from-BOOST: fire drogue pyro (safety — better to deploy chute under thrust than ballistic)
- ABORT-from-COAST: fire drogue pyro (same as normal apogee transition)
- ABORT-from-DESCENT: no-op (chutes already deployed, keep current state)
- ABORT has 5-min timeout → auto-transition to LANDED (prevents stuck ABORT state)
- Re-ARM from ABORT requires explicit RESET first (no direct ABORT→ARMED)

**MissionProfile struct defined here (Council Amendment #6):** `MissionProfile` struct + single interim `kDefaultRocketProfile` const instance created in IVP-68. This makes the Flight Director profile-driven from the start — guards, abort actions, and phase definitions all read from `const MissionProfile*`. The struct is final (serialization-ready, no pointers) but profile *values* are `⚠️ VALIDATE` starting points. IVP-74 adds the other two profiles (HAB, freeform), flash persistence, and CLI selection UI.

**Host tests:** FlightState init, phase enum contiguity, signal ordering, state transitions via signal sequence, abort from each flight phase (with source-specific behavior), reset paths, marker timestamps, ABORT timeout to LANDED

**HW gate:** All 9 phases reachable via CLI; transition log `[FD] IDLE -> ARMED`; QEP dispatch <5µs; 60s soak zero errors + ESKF stable; `'s'` shows phase

---

### IVP-69: Command Handler + Pre-Arm
**Complexity:** Medium | **New files:** 4 | **Modified:** 2 | **Host tests:** ~22

**Correction — battery voltage (Issue #2):** No ADC driver exists, no battery divider on Feather RP2350 HSTX. Move battery voltage from Tier 1 (immutable) to Tier 2 (profile-specific, warn severity). Check function returns `passed=true, reason="Battery: not monitored"`. Infrastructure ready when ADC hardware is added.

**Correction — flash storage meaning:** `check_flash_available()` = `g_flightTable.num_flights < kMaxFlights && g_flightTable.next_free_sector < kMaxFlashSectors`. Concrete, testable.

**Implement:**
- `src/flight_director/command_handler.h` + `.cpp` — `CommandType` enum, `command_handler_process()` validates command against current phase + pre-arm, returns QSignal or 0 (rejected)
- `src/flight_director/pre_arm_checks.h` + `.cpp` — Tier 1 (6 checks: IMU, baro, ESKF, flash, LAUNCH_ABORT, watchdog) + Tier 2 (profile-specific: GPS, mag, radio, battery stub). `PreArmResult` struct with pass/fail + reason string
- CLI: Flight Director sub-menu commands (`'a'`=ARM, `'d'`=DISARM, `'r'`=RESET) route through command handler; sensor event keys (`'l'`, `'b'`, `'p'`, `'m'`, `'n'`) bypass commands (simulating guards)
- Pre-arm failure output: `[PRE-ARM] 6/6 platform OK, 3/5 profile OK (GPS: NO LOCK, Battery: not monitored)`

**Armed timeout (Council Amendment #5):** Configurable auto-disarm timer, default 5 minutes. If ARMED state persists beyond timeout without SIG_LAUNCH, auto-transition to IDLE with `[FD] ARMED timeout — auto-disarm` log. Prevents forgotten armed state. Timeout value sourced from MissionProfile.

**Host tests:** CMD_ARM pass/fail per check, phase validation per command, LAUNCH_ABORT blocking, each individual pre-arm check with mock FusedState, battery stub always passes, armed timeout triggers auto-disarm

**HW gate:** ARM succeeds with sensors calibrated; ARM fails + prints reasons with sensors not ready; LAUNCH_ABORT blocks arming; full command cycle on device; 60s soak while ARMED; armed timeout fires after configured duration

---

### IVP-70: Core Guard Functions
**Complexity:** Medium | **New files:** 4 | **Modified:** 1 | **Host tests:** ~28

**Implement:**
- `src/flight_director/guard_functions.h` + `.cpp` — 6 guards: `guard_accel_body_z`, `guard_accel_magnitude`, `guard_velocity_zero_cross`, `guard_baro_peak`, `guard_altitude_agl`, `guard_stationary`. `CompareOp` enum, `compare_field()` helper
- `src/flight_director/guard_evaluator.h` + `.cpp` — `GuardEvaluator` class: per-guard sustain counters, phase-validity bitmask, returns signal when sustained condition met. Configured via `GuardConfig` struct (threshold, sustain_ms, compare_op, valid_phases)
- Wire into `flight_director.cpp`: phase SIG_TICK handlers call evaluator with current FusedState snapshot (one seqlock read per tick, not per guard)

**Guard sustain values with sources (Council Amendment #4):**
| Guard | Sustain | Source |
|-------|---------|--------|
| Launch (accel_body_z) | 50ms (5 samples @ 100Hz) | ArduPilot `AP_InertialNav`: 5-sample accel spike filter |
| Burnout (accel_magnitude) | 100ms (10 samples) | Motor burnout is gradual, not instantaneous |
| Apogee (velocity_zero_cross) | 30ms (3 samples) | Velocity sign change is clean; short sustain avoids delay |
| Baro peak | 200ms (20 samples) | Baro noise requires longer window for derivative |
| Main deploy (altitude_agl) | 50ms (5 samples) | Altitude is smooth; short sustain for timely deploy |
| Landing (stationary) | 2000ms (200 samples) | ArduPilot `LAND_SPEED` timeout pattern; prevents false landing on chute oscillation |
All values are `⚠️ VALIDATE` — starting points to be tuned with flight data.

**Host tests:** compare_field boundary tests; each guard with synthetic FusedState; sustain timing (N-1 doesn't fire, N fires); sustain reset on false; evaluator phase-validity; evaluator signal generation

**HW gate:** Hand-shake/toss triggers SIG_LAUNCH; no false landing during 5-min soak; guards <50µs total per 100Hz tick; guard results in CLI output

---

## Part B: Integration (IVP-71 through IVP-75)

### IVP-71: Multi-Method Detection Logic
**Complexity:** Medium | **New files:** 2 | **Modified:** 1 | **Host tests:** ~18

**Implement:**
- `src/flight_director/guard_combinator.h` + `.cpp` — `CombinatorType` enum (AND, OR, PRIMARY_PLUS_TIMEOUT), `GuardCombinator` struct, `combinator_evaluate()`. Edge detection (rising-edge only, reset on phase transition). Missed apogee fallback: PRIMARY_PLUS_TIMEOUT with coast timeout sourced from MissionProfile (Amendment #7), default 15s for rocket

**Host tests:** AND/OR/TIMEOUT logic; edge detection prevents re-fire; edge reset on phase change; timeout with simulated elapsed time; phase validity prevents wrong-context evaluation

**HW gate:** Toss triggers apogee via velocity zero-cross; with velocity guard disabled, timeout fires after 15s; no false transitions in 5-min soak

---

### IVP-72: Action Executor
**Complexity:** Medium | **New files:** 3 | **Modified:** 2 | **Host tests:** ~18

**Implement:**
- `src/flight_director/action_executor.h` + `.cpp` — `ActionType` enum (SET_LED, MARK_EVENT, REPORT_STATE, FIRE_PYRO, SET_BEACON), `action_execute()`, `action_execute_list()`
- `src/flight_director/flight_actions.h` — constexpr arrays of actions per phase (entry/exit/transition)
- Wire into QEP handlers: Q_ENTRY_SIG calls entry actions, Q_EXIT_SIG calls exit, transition actions between exit and entry
- NeoPixel: extend `g_calNeoPixelOverride` values for flight states (ARMED=amber, BOOST=red, COAST=yellow, DESCENT=red blink, LANDED=green slow blink, ABORT=red fast blink)
- Pyro actions: log intent only (no physical pyro in Stage 8)
- Logging: set `g_loggingActive` flag, write event markers via ring buffer

**Pyro negative tests (Council Amendment #2):** `static_assert` or compile-time check that no FIRE_PYRO action appears in any entry/exit action list — pyro ONLY in transition action lists. Host test verifies this programmatically across all constexpr action arrays.

**Host tests:** Action list execution order; SET_LED produces correct values; MARK_EVENT sets timestamps; FIRE_PYRO logs intent; abort actions differ by source phase; empty list no crash; **no FIRE_PYRO in any entry/exit list (safety negative test)**

**HW gate:** NeoPixel changes color on each phase transition; log markers written; pyro intent logged; transition action order verified via timestamps; 60s soak stable

---

### IVP-73: Bench Flight Simulation (SITL)
**Complexity:** Medium | **New files:** 1 (Python) | **Host tests:** 0

**Decision (Issue #3 — data injection):** Event-only injection via CLI keys. Tests the full pipeline (command → validation → event → state → actions) without needing FusedState overrides. Guards already validated in IVP-70/71.

**Implement:**
- `scripts/bench_flight_sim.py` — pyserial script sends CLI numeric key commands, parses `[FD]` transition logs, validates sequence + timing + actions
- Test sequences: happy path (IDLE→LANDED), abort from each flight phase, error injection (ARM with failed pre-arm, double ARM, ABORT from IDLE)

**HW gate:** Script runs unattended, produces pass/fail summary; all 9 phases visited in order; abort paths tested; invalid commands rejected; <60s total duration

---

### IVP-74: Mission Configuration
**Complexity:** Medium | **New files:** 3 | **Modified:** 2 | **Host tests:** ~12

**Decision (Issue #4 — profiles):** Rocket profile fully implemented. HAB and freeform use identical phase topology with different thresholds and no pyro actions. HAB phase topology refinement deferred to post-Stage 8.

**Implement:** (MissionProfile struct and `kDefaultRocketProfile` already exist from IVP-68)
- Add `kHabProfile` (no pyro actions, relaxed altitude guards) and `kFreeformProfile` (single flight phase, no guards) as additional `const MissionProfile` instances
- **Profile is boot-locked (safety).** All three profiles are compiled in, but the active profile is read from flash at boot and immutable for the session. CLI `'1'`-`'3'` keys change the *stored* selection (flash write), `'q'` shows active profile. Change takes effect on next power cycle. No runtime reconfiguration — eliminates risk of mid-session profile inconsistency. No recompile needed — all profiles are always present in the binary.

**Host tests:** Profile struct has no pointers (sizeof); rocket profile phase count + guard configs; HAB has no pyro actions; freeform has no guards; profile index bounds

**HW gate:** Three profiles selectable via CLI; selection persists across power cycle; bench sim passes with rocket profile; freeform: ARM succeeds, no auto transitions

---

### IVP-75: Active Object Migration Planning
**Complexity:** Small | **New files:** 1 (doc) + vendored QF/QV | **Host tests:** 0

**Implement:**
- Extend `lib/qep/` with QF+QV source files, write BSP (`QF_onStartup()`, `QV_onIdle()` with WFI)
- Compile gate only — does not replace superloop
- `docs/flight_director/ACTIVE_OBJECT_MIGRATION.md` — AO boundaries (FlightDirector, LedEngine, Logger, Telemetry, ErrorHandler), event catalog, pub-sub topology, dual-core constraints, Stage 9 step-by-step plan

**HW gate:** QF+QV compiles+links; existing Flight Director still works; no regressions (bench sim re-run)

---

## Cross-Cutting Concerns

### New Host Test Library
```cmake
add_library(rc_flight_director STATIC
    src/flight_director/flight_state.cpp
    src/flight_director/command_handler.cpp
    src/flight_director/pre_arm_checks.cpp
    src/flight_director/guard_functions.cpp
    src/flight_director/guard_evaluator.cpp
    src/flight_director/guard_combinator.cpp
    src/flight_director/action_executor.cpp
    src/flight_director/mission_profile.cpp
)
```
Pure C++ — no Pico SDK. **QEP included** — `qep_hsm.c` compiles for x86 host alongside rc_flight_director (Council Amendment #3). This enables host-side testing of full HSM dispatch, not just FlightState logic.

### CLI Key Allocation — Flight Director Sub-Menu
New third menu level: `RC_OS_MENU_FLIGHT` accessed via `'f'` key from main menu (currently `'f'` is in `handle_unhandled_key` for flight list — relocate to sub-menu). Within the Flight Director sub-menu:
- `'a'`/`'A'` = ARM, `'d'`/`'D'` = DISARM, `'x'`/`'X'` = ABORT, `'r'`/`'R'` = RESET (commands)
- `'l'`/`'L'` = inject LAUNCH, `'b'`/`'B'` = BURNOUT, `'p'`/`'P'` = APOGEE, `'m'`/`'M'` = MAIN_DEPLOY, `'n'`/`'N'` = LANDING (sensor event injection for bench testing)
- `'s'`/`'S'` = show flight status, `'q'`/`'Q'` = show profile, `'1'`-`'3'` = select profile
- ESC/`'z'` = return to main menu
Mnemonic keys within a dedicated context. No collisions with main or calibration menus.

### Estimated Host Test Growth
~108-126 new tests across 7 test files. Total: 358 + ~120 = ~480.

### Naming Clarification
- **`MissionProfile` struct** (Stage 8) = flight profile data (guards, thresholds, abort actions, pre-arm checks). Starts as compiled-in `const` data (FEMA approach). Will become user-definable in a future stage.
- **`mission.h` / `mission_vehicle.h` / `mission_station.h`** (existing, Stage 7) = compile-time device role selector (vehicle vs ground station). **Rename to `job.h` / `job_vehicle.h` / `job_station.h`** to distinguish from MissionProfile. Update `ROCKETCHIP_MISSION_STATION` → `ROCKETCHIP_JOB_STATION` CMake define. Small rename, do in IVP-68 alongside MissionProfile struct introduction.

### IVP Text Corrections Needed
1. **IVP-66:** Strike bug fix section (already done). Keep 5 policy items.
2. **IVP-69:** Move battery voltage to Tier 2. Define "flash available" concretely.
3. **IVP-68, 69, 71, 72:** Add explicit HW soak gates.
4. **All IVPs:** Note host test expectations.

### Files Modified (total across all IVPs)
- `CMakeLists.txt` — qep library, rc_flight_director library, 7 test executables
- `test/CMakeLists.txt` — test executable definitions
- `src/main.cpp` — flight_director_tick, CLI key wiring, populate_fused_state
- `src/cli/rc_os.h` + `.cpp` — minor (LAUNCH_ABORT ack if needed)
- `include/rocketchip/config.h` — flight director timing constants

### New Files (total ~25)
- `src/watchdog/watchdog_recovery.h` + `.cpp`
- `lib/qep/` (~6 vendored files)
- `src/flight_director/` (~14 .h/.cpp files)
- `scripts/bench_flight_sim.py`
- `docs/flight_director/ACTIVE_OBJECT_MIGRATION.md`
- `test/` (7 test files)

---

## Verification Strategy

1. **Per-IVP:** Host tests pass (`ctest`), target builds clean, HW gate items verified
2. **Per-merge:** Full test suite (358 + new), 60s HW soak, bench sim re-run on later IVPs
3. **End of Stage 8:** Full bench flight sim, all three profiles, 5-min soak, CHANGELOG + PROJECT_STATUS + WHITEBOARD updated

## Council Review

**Verdict: APPROVE with 7 amendments (all incorporated above)**

Reviewed by all 4 main personas (ArduPilot, NASA/JPL, Professor, Student). Unanimous on 5 points, majority on 2. Zero dissents.

**Amendments applied:**
1. ABORT state specification with source-phase-specific behavior → IVP-68
2. Pyro negative tests (`static_assert` no FIRE_PYRO in entry/exit) → IVP-72
3. QEP compiled for host tests (portable C99) → Host Test Library
4. Guard sustain values documented with sources → IVP-70
5. Armed timeout / auto-disarm (5 min default) → IVP-69
6. MissionProfile struct definition + single `kDefaultRocketProfile` instance in IVP-68; IVP-74 adds HAB/freeform profiles + flash persistence + CLI selection
7. Coast timeout sourced from MissionProfile, not hardcoded → IVP-71
