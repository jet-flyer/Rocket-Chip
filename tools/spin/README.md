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
The Cygwin-compiled `spin.exe` needs Cygwin's `gcc` for preprocessing.

**Install:** `C:\tools\cygwin\cygwinsetup.exe --packages gcc-core --quiet-mode --site https://mirrors.kernel.org/sourceware/cygwin/`

## Quick Start

All commands run from `tools/spin/` via Cygwin bash:

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

Expected: `errors: 0` for all 8 properties. Runtime: ~0.3 seconds total.

## Models

### `rocketchip_fd.pml` (IVP-82a)
Flight Director HSM only. Single process, non-deterministic environment. 73 reachable states. Verifies 6 safety properties. Use for quick pyro safety checks.

### `rocketchip_ao.pml` (IVP-82b)
Full Active Object topology. 5 processes: FlightDirector + Logger + Telemetry + LedEngine (+ implicit Environment via non-deterministic choice). 107,818 reachable states. Verifies 5 safety + 3 mission-critical properties.

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

### FD-only quick check
```bash
spin -a rocketchip_fd.pml
gcc -O2 -o pan pan.c
./pan -a -N p_no_pyro_idle
```

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
- **Timing** — no wall-clock. "Coast timeout" is modeled as non-deterministic, not as "after N milliseconds"
- **Sensor data values** — guard evaluation is non-deterministic, not based on accel/baro/GPS thresholds
- **Queue overflow** — channels are depth [1]. Actual overflow behavior validated empirically (LL Entry 32, 10-min soak)
- **Hardware faults** — I2C corruption, USB CDC timing, flash failures, radio brownout
- **Core 1 behavior** — sensor loop is outside the model
- **Floating-point arithmetic** — ESKF, calibration, coordinate transforms

These are verified through other means: 552 host tests, bench flight sim (9/9 PASS), 10-minute hardware soak, and Lessons Learned entries.

## References

- [Basic SPIN Manual (Holzmann)](https://spinroot.com/spin/Man/Manual.html)
- [SPIN Beginners' Tutorial (Ruys)](https://spinroot.com/spin/Doc/SpinTutorial.pdf)
- [RTEMS Promela Modeling Guide](https://docs.rtems.org/docs/main/eng/fv/promela.html)
- [MIT SPIN Exercises](http://web.mit.edu/spin_v6.4.7/Doc/1_Exercises.html)
- [Holzmann "Using SPIN"](https://www.cmi.ac.in/~madhavan/courses/verification-2011/holzmann-using-spin.pdf)
- [SWARM for large state spaces (Holzmann)](https://link.springer.com/chapter/10.1007/978-3-540-85114-1_11)
- [NASA SPIN launch vehicle verification](https://doi.org/10.2514/1.I010876)
- [Promela language reference](https://spinroot.com/spin/Man/promela.html)
