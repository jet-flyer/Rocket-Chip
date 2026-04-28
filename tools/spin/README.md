# RocketChip SPIN Formal Verification

Formal verification of the Active Object event topology using the SPIN model checker and Promela language. Exhaustively verifies pyro safety, event delivery, and deadlock freedom across all possible event orderings.

## Prerequisites

### SPIN Model Checker (6.5.2)
Prebuilt binary from the [SPIN GitHub repo](https://github.com/nimble-code/Spin). Requires Cygwin runtime (`cygwin1.dll`).

**Install:**
1. `choco install cygwin -y` (admin terminal)
2. Download `spin652_windows64.exe.gz` from `github.com/nimble-code/Spin/tree/master/Bin`
3. Extract and rename to `spin.exe`, place in `C:\Users\pow-w\bin\`
4. `copy C:\tools\cygwin\bin\cygwin1.dll C:\Users\pow-w\bin\`
5. Verify: `spin -V` → `Spin Version 6.5.2`

### MinGW-w64 GCC
SPIN generates C code; GCC compiles the verifier.

**Install:** `choco install mingw -y` (admin terminal)
**Verify:** `gcc --version` (may need shell restart for PATH)

### Cygwin GCC
The Cygwin-compiled `spin.exe` needs Cygwin's `gcc` for preprocessing. SPIN calls `gcc` as a subprocess and requires Cygwin DLLs (`cygintl-8.dll` etc.) to be loadable.

**Install:** `C:\tools\cygwin\cygwinsetup.exe --packages gcc-core --quiet-mode --site https://mirrors.kernel.org/sourceware/cygwin/`

**CRITICAL:** If SPIN reports "preprocessing failed," verify that the `gcc` in PATH is the Cygwin version (not MinGW) and that `/c/tools/cygwin/bin` is in PATH for the DLL dependencies. The Cygwin gcc at `~/bin/gcc` requires `cygintl-8.dll` from `/c/tools/cygwin/bin/`. See also: `PATH="/c/tools/cygwin/bin:$PATH" spin -a model.pml`

## Quick Start

**CRITICAL:** All SPIN commands MUST run via **Cygwin bash**, not Git Bash. SPIN is a Cygwin binary that calls `gcc` via the Cygwin environment. Git Bash cannot resolve the Cygwin gcc.

**Recommended (full `rocketchip_ao` regression):** from Cygwin bash, repo root:

```bash
PATH="/c/tools/cygwin/bin:/c/tools/cygwin/usr/bin:$PATH" \
  bash tools/spin/run_stage_o_ao_spin.sh
```

This generates `pan`, runs **11** LTLs on `rocketchip_ao.pml` (eight original safety + three fault/HealthMonitor claims; the last uses `./pan -a -f`), removes `.trail` files, and prints `SPIN_OK_ALL_11` if every `pan` run reports `errors: 0` (~tens of seconds; ~1.0 s per property on a typical dev PC). The script file must use **LF** line endings; CRLF breaks Cygwin `bash` (`cd: $'\r': No such file or directory`).

**Minimal (8 safety properties, ~8 s):** from `tools/spin/` via Cygwin bash:

```bash
# Generate verifier, compile, run all safety properties
/c/tools/cygwin/bin/bash.exe -lc '
cd /cygdrive/c/Users/pow-w/Documents/Rocket-Chip/tools/spin
spin -a rocketchip_ao.pml
gcc -O2 -o pan pan.c
for prop in p_no_pyro_idle p_drogue_before_main p_pyro_requires_armed \
            p_drogue_once p_main_once p_logger_gets_all p_telem_gets_all \
            p_led_gets_all; do
    echo "=== $prop ==="
    ./pan -a -N $prop
    rm -f *.trail
done
'
```

Expected: `errors: 0` for all 8 properties. Runtime: ~8 s total on a typical dev PC (≈1 s per property; model explores ~1.8M stored states per run).

## Models

### `rocketchip_fd.pml` (IVP-82a, updated IVP-121)
Flight Director HSM only. Single process, non-deterministic environment. Verifies 7 safety properties + 1 liveness property (P7). Use for quick pyro safety checks and flight-completes liveness proof.

**Bounded tick counters (IVP-121):** The model uses the Discrete-Time Promela pattern (Tripakis & Courcoubetis, TACAS 1996) to model physical time progression. Each flight phase has a bounded tick counter that increments unconditionally on every iteration. When the counter reaches the phase's limit, the exit transition is forced with no `skip` option. This encodes environmental inevitability: motors burn out (finite propellant), PIO timers fire (hardware countdown), backstops expire (wall-clock). The bounded counter is the Promela encoding of these physical assumptions. NASA JPL used the same pattern for DS1 flight software verification (Gluck & Holzmann, 2001).

**Liveness verification requires weak fairness:** Run `./pan -a -f -N p_liveness_flight_completes`. The `-f` flag enables weak process fairness. Without bounded counters, weak fairness is insufficient for single-process liveness (it applies at process granularity, not branch granularity). With bounded counters, the counter increment is unconditional and the exit guard eventually becomes the only option, making liveness provable under weak fairness.

### `rocketchip_ao.pml` (IVP-82b, extended IVP-104+)
Full Active Object topology. 5 processes: FlightDirector + Logger + Telemetry + LedEngine (+ implicit Environment via non-deterministic choice). **11** never claims: eight legacy safety (pyro, ordering, one-shot, publish ordering) plus three HealthMonitor/fault LTLs (`p_fault_blocks_arm`, `p_fault_latch_holds`, `p_armed_fault_safe_mode` — the last with `-f`). Use **`run_stage_o_ao_spin.sh`** for a full run; exhaustive depth stores on the order of **~1.8M** states per claim on current models.

### `rocketchip_station.pml` (IVP-147, Stage 16C)
Station-side command delivery channel — the RX/ACK/retry protocol that neither vehicle model covers. Two processes: Station + Vehicle, with lossy bidirectional channels. Initial scaffolding scope: single command in flight (seq=1), no MAVLink parser modelling, no RadioScheduler TX-window arbitration. Verifies:

- **P_TERMINATION** (liveness, requires `-f`): every pending command eventually terminates (either acked or all 3 retries exhausted).
- **P_NO_DOUBLE_CLEAR** (safety): the pending bit cannot be cleared twice for the same sequence number — guards against a race between ACK arrival and retry-exhaustion clear.

Run:
```bash
/c/tools/cygwin/bin/bash.exe -lc '
cd /cygdrive/c/Users/pow-w/Documents/Rocket-Chip/tools/spin
spin -a rocketchip_station.pml
gcc -O2 -o pan_station pan.c
./pan_station -a -N p_no_double_clear
./pan_station -a -f -N p_termination
'
```

Expected: `errors: 0` for both. Runtime <1s total. Future extensions (multi-pending-in-flight, RadioScheduler TX-window, MAVLink-parser state) are queued as whiteboard follow-ups when firmware gains the supporting behaviors.

### `rocketchip_rf_manager.pml` (IVP-T14, Stage T Batch B)
AO_RfManager state-machine model. 4-state link-health machine with Schmitt-trigger hysteresis (55/65 LQ bands, 10 pp deadband) and forced-ACQ protection (time-primary + frame-minimum per 2026-04-21 user direction). Two processes: RxProducer (non-deterministic good/miss/crc_err stream) + ModeSetter (picks healthy / partial-loss / fully-nondeterministic stream once). Verifies 5 properties covering the Round 2 design §10 set + plan-as-whole consensus #9:

- **p_eventual_track** (liveness, requires `-f`): healthy input stream reaches `kTrack`.
- **p_never_silent** (safety): any * → kAcq transition atomically sets `notify_lost`.
- **p_no_tx_in_acq** (safety): station TX never permitted in `kAcq`.
- **p_hysteresis** (safety): `kTrackDegraded` reachable only via `kTrack` (never `kTentative`/`kAcq` directly).
- **p_progress** (safety): Schmitt-trigger fix eliminates `kTrack↔kTrackDegraded` oscillation under partial-loss — exits from that pair only via proper forced-ACQ, never spuriously back to `kTentative`.

Run:
```bash
/c/tools/cygwin/bin/bash.exe -lc '
cd /cygdrive/c/Users/pow-w/Documents/Rocket-Chip/tools/spin
spin -a rocketchip_rf_manager.pml
gcc -O2 -o pan_rfm pan.c
for p in p_no_tx_in_acq p_hysteresis p_never_silent p_progress; do
    echo "=== $p ==="
    ./pan_rfm -a -N $p
    rm -f *.trail
done
echo "=== p_eventual_track (liveness, -f) ==="
./pan_rfm -a -f -N p_eventual_track
'
```

Expected: `errors: 0` on all 5. Runtime <1 s total. Abstractions: LQ is bucketed (LOW/BORDER/HIGH) rather than the full 10-slot sliding window — state space tractable while still proving the Schmitt/livelock properties. Filter coefficient arithmetic is host-tested separately (test/test_rf_link_health.cpp 32 tests).

## What's Modeled

| Real System | Promela Model | Abstraction |
|-------------|---------------|-------------|
| FlightDirector 10-state HSM | 8 phase values + non-deterministic transitions | Exact state mapping |
| Guard evaluation (sensor-based) | Non-deterministic signal arrival per phase | Guards fire when phase-appropriate, no sensor math |
| Timer ticks (100Hz) | Non-deterministic "tick or not" choice | No wall-clock timing |
| Pyro fire actions | Boolean flags + counters | Tracks fire/not-fire and count |
| AO event queues (depth 32) | Channels depth [1] | Safety depends on ordering, not depth |
| CLI commands | Non-deterministic valid commands per phase | No input parsing |
| Core 1 sensor loop | Not modeled | Data values irrelevant to event topology |
| ESKF, USB, I2C, SPI | Not modeled | Hardware peripherals outside scope |

## Safety Properties

| ID | Property | LTL | What It Catches |
|----|----------|-----|-----------------|
| P1 | No pyro in IDLE | `[](phase==IDLE -> !drogue && !main)` | Logic error in guard/transition |
| P2 | Drogue before main | `[](main -> drogue)` | Sequence invariant violation |
| P3 | Pyro requires ARMED | `[]((drogue\|\|main) -> was_armed)` | Shortcut path bypassing ARM |
| P4 | Drogue fires once | `[](drogue_count <= 1)` | Duplicate fire (hardware damage) |
| P5 | Main fires once | `[](main_count <= 1)` | Duplicate fire |
| P7 | Flight completes (liveness) | `[](phase==BOOST -> <>(phase==LANDED \|\| phase==ABORT))` | Stuck flight phase — requires `-f` flag and bounded counters |
| P9 | No LANDED without launch | `[](phase==LANDED -> has_launched)` | Pad abort entering LANDED state |
| M1 | Logger gets all phases | `[](pub >= log)` | Silent event drop to logger |
| M2 | Telem gets all phases | `[](pub >= tel)` | Telemetry data loss |
| M3 | LED gets all phases | `[](pub >= led)` | LED not reflecting flight state |

## How to Run

### Safety check (exhaustive)
```bash
spin -a rocketchip_ao.pml
gcc -O2 -o pan pan.c
./pan -a -N p_no_pyro_idle
```

### If a property fails — trace the counterexample
```bash
./pan -a -N p_no_pyro_idle        # generates .trail file
spin -t -p rocketchip_ao.pml      # replay with step-by-step output
```

### Simulate before verifying (understand model behavior)
```bash
spin -u100 -p -l rocketchip_ao.pml
```

### FD-only quick check (safety)
```bash
spin -a rocketchip_fd.pml
gcc -O2 -o pan pan.c
./pan -a -N p_no_pyro_idle
```

### FD liveness check (P7 — requires weak fairness)
```bash
spin -a rocketchip_fd.pml
gcc -O2 -o pan pan.c -DMEMLIM=512
./pan -a -f -N p_liveness_flight_completes
```
Note: the `-f` flag enables weak fairness. Without it, P7 cannot be proven even with bounded counters.

## Interpreting Results

**"errors: 0"** — Property holds across ALL reachable states. Exhaustive proof.

**"errors: 1"** — Counterexample found. A `.trail` file is generated. Replay with `spin -t -p model.pml` to see the exact event sequence that violates the property.

**"Search not completed"** — State space too large or depth limit hit. Increase `-m` (depth) or use `-DBITSTATE` (approximate) or reduce `MAX_EVENTS`.

**State count:** 107,818 states for `rocketchip_ao.pml` is healthy. If changes push it above ~10M, review model for unnecessary branching.

## Model-to-Code Mapping

| Promela Construct | C++ Source | Notes |
|-------------------|-----------|-------|
| `phase` variable (byte 0-7) | `FlightState.current_phase` in [flight_state.h](../../src/flight_director/flight_state.h) | `FlightPhase` enum |
| FD non-deterministic choice | State handlers in [flight_director.cpp](../../src/flight_director/flight_director.cpp) | Each `if :: ... fi` maps to a `case` in a state handler |
| `drogue_fired`, `main_fired` | `run_transition_actions()` calling `log_pyro_cb` in [action_executor.cpp](../../src/flight_director/action_executor.cpp) | Pyro intent callbacks |
| `drogue_count`, `main_count` | Not explicitly tracked in code | Model-only for idempotency verification |
| `was_armed` | Not explicitly tracked in code | Model-only for ARMED-prerequisite verification |
| `pub_count` | Implicit in FD phase transition code | Each transition that publishes SIG_PHASE_CHANGE |
| `logger_ch ! SIG_PHASE_CHANGE` | `QACTIVE_POST()` to Logger AO queue | Maps to [ao_logger.cpp](../../src/active_objects/ao_logger.cpp) `l_loggerAoQueue[32]` |
| `telem_ch ! SIG_PHASE_CHANGE` | `QACTIVE_POST()` to Telemetry AO queue | Maps to [ao_telemetry.cpp](../../src/active_objects/ao_telemetry.cpp) `l_telemAoQueue[32]` |
| `led_ch ! SIG_LED_PATTERN` | `AO_LedEngine_post_pattern()` | Maps to [ao_led_engine.cpp](../../src/active_objects/ao_led_engine.cpp) |
| Channel depth [1] | Actual queue depth 32 | Depth abstracted — safety doesn't depend on buffering. Queue overflow validated empirically (LL Entry 32) |
| `MAX_EVENTS = 8` | One flight cycle | ARM + 6 transitions + ticks. Increase for multi-flight scenarios |
| `atomic{}` blocks | QV run-to-completion dispatch | Each event handler runs fully before next event |

## Updating the Model

**When to update:** After any change to:
- Flight Director state transitions (new phase, new guard, new signal)
- AO event routing (new subscriber, new signal type)
- Pyro action logic (new channel, changed fire conditions)

**How to update:**
1. Add/modify the relevant `if :: ... fi` branch in the FD process
2. If new signal types, add to `mtype` declaration
3. If new AO, add a new consumer process with channel
4. Re-run all properties: all must show `errors: 0`
5. Update this mapping table

**When NOT to update:** Changes to sensor math, ESKF tuning, CLI text, LED colors, timing constants, queue depths. These are abstracted away.

## Limitations

SPIN does **NOT** verify:
- **Exact timing** — no wall-clock. Phase durations are modeled as bounded tick counts (Discrete-Time Promela), not as "after N milliseconds." The model proves liveness (flight eventually completes) but not that specific timeouts fire at their configured millisecond values
- **Sensor data values** — guard evaluation is non-deterministic, not based on accel/baro/GPS thresholds
- **Queue overflow** — channels are depth [1]. Actual overflow behavior validated empirically (LL Entry 32, 10-min soak)
- **Hardware faults** — I2C corruption, USB CDC timing, flash failures, radio brownout
- **Core 1 behavior** — sensor loop is outside the model
- **Floating-point arithmetic** — ESKF, calibration, coordinate transforms

These are verified through other means: 552 host tests, bench flight sim (9/9 PASS), 10-minute hardware soak, and Lessons Learned entries.

## References

### Discrete-Time Promela and Liveness (IVP-121)
- [Extending Promela and Spin for Real Time (Tripakis & Courcoubetis, TACAS 1996)](https://link.springer.com/chapter/10.1007/3-540-61042-1_53) — the bounded-counter pattern we use for liveness
- [Using SPIN for Flight Software Verification (Gluck & Holzmann, DS1, 2001)](https://www.academia.edu/19258496/Using_SPIN_model_checking_for_flight_software_verification) — NASA JPL verified DS1 flight software liveness with bounded time constraints
- [Liveness Checking as Safety Checking (Biere et al., FMICS 2002)](https://fmv.jku.at/papers/BiereArthoSchuppan-FMICS02.pdf) — K-liveness: reduce liveness to bounded safety
- [Modeling and Validating Launch Vehicle Onboard Software (AIAA)](https://doi.org/10.2514/1.I010876) — launch vehicle SPIN verification

### General SPIN
- [Basic SPIN Manual (Holzmann)](https://spinroot.com/spin/Man/Manual.html)
- [SPIN Beginners' Tutorial (Ruys)](https://spinroot.com/spin/Doc/SpinTutorial.pdf)
- [RTEMS Promela Modeling Guide](https://docs.rtems.org/docs/main/eng/fv/promela.html)
- [MIT SPIN Exercises](http://web.mit.edu/spin_v6.4.7/Doc/1_Exercises.html)
- [Holzmann "Using SPIN"](https://www.cmi.ac.in/~madhavan/courses/verification-2011/holzmann-using-spin.pdf)
- [SWARM for large state spaces (Holzmann)](https://link.springer.com/chapter/10.1007/978-3-540-85114-1_11)
- [NASA SPIN launch vehicle verification](https://doi.org/10.2514/1.I010876)
- [Promela language reference](https://spinroot.com/spin/Man/promela.html)
