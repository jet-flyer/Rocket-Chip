# RocketChip Verification Overview

**Status:** Active reference document
**Last Updated:** 2026-03-27
**Scope:** Complete verification strategy — all methods, when to use each, how they complement each other

---

## Verification Layers

RocketChip uses five complementary verification layers. No single layer catches everything — correctness comes from their combination.

| Layer | What It Catches | Speed | Coverage |
|-------|----------------|-------|----------|
| **Formal Verification (SPIN)** | Deadlocks, pyro safety violations, event loss across ALL possible event orderings | ~37ms per property | Exhaustive for modeled properties |
| **Host Unit Tests** | Logic errors in individual modules (math, state transitions, guards, calibration) | ~3s for 552 tests | Code-path coverage |
| **Bench Flight Sim** | End-to-end state machine behavior (CLI → command → transition → action) | ~10s for 9 scenarios | Scenario coverage |
| **Hardware Soak Tests** | Runtime stability, sensor errors, I2C bus health, memory leaks, watchdog | 1-10 minutes | Time-duration coverage |
| **Dynamic Validation** | ESKF accuracy under real motion (rotation, pendulum, elevator, vehicle) | Minutes to hours | Physical accuracy |

### When to run each:

| Trigger | Run These |
|---------|-----------|
| **Any code change** | Host unit tests (`ctest`) |
| **Flight Director / state machine change** | SPIN formal verification + bench flight sim |
| **New AO signal or subscriber** | SPIN formal verification (update model) |
| **Sensor driver change** | Hardware soak (60s minimum) |
| **ESKF tuning change** | Host unit tests + hardware soak |
| **Pre-flight** | Bench sim + 10-min soak + CLI sensor status |
| **New hardware** | Full soak + dynamic validation |

---

## 1. Formal Verification (SPIN)

**Location:** `tools/spin/`
**Guide:** [tools/spin/README.md](../tools/spin/README.md)

Exhaustively verifies the Active Object event topology using the SPIN model checker. Proves properties across ALL possible event orderings — something no finite test suite can do.

**Properties verified (8 total):**

*Safety (5):*
- Pyro never fires in IDLE
- Drogue fires before main (sequence invariant)
- Pyro requires prior ARMED state
- Drogue fires at most once per flight
- Main fires at most once per flight

*Mission-critical (3):*
- All phase changes reach Logger (no silent drop)
- All phase changes reach Telemetry
- All phase changes reach LED engine

**Run time:** ~0.3 seconds for all 8 properties (107,818 states each)

**When to update the model:** After any change to flight phases, guard signals, AO event routing, or pyro logic. See `tools/spin/README.md` "Updating the Model" section.

**Limitations:** Does not verify timing, sensor data values, queue overflow (validated empirically), or hardware faults.

---

## 2. Host Unit Tests

**Location:** `test/`
**Run:** `cmake --build build_host/ && ctest --test-dir build_host/`

552 tests covering:
- ESKF propagation and measurement updates (replay harness)
- Flight Director state transitions (QEP dispatch)
- Guard functions and combinators
- Go/No-Go pre-arm checks
- Action executor (pyro intent, LED mapping)
- Mission profile configuration
- Math libraries (Vec3, Quat, Mat)
- PCM frame encoding/decoding

**Standards:** GoogleTest framework. Each IVP adds tests for the functionality it introduces.

---

## 3. Bench Flight Simulation

**Location:** `scripts/bench_flight_sim.py`
**Run:** `python scripts/bench_flight_sim.py`

Automated pyserial script that drives the Flight Director through 9 test scenarios via CLI commands over serial:
1. Rejection tests (ABORT/RESET/DISARM from IDLE)
2. Abort path tests (from ARMED, BOOST, COAST, DESCENT)
3. Happy path (full flight: IDLE → ARMED → BOOST → COAST → DROGUE → MAIN → LANDED)
4. Double ARM rejection

**Verifies:** Command validation pipeline, HSM transitions, pyro intent logging, NeoPixel phase indication.

**Requires:** Device connected via USB serial (COM7). Sensors must be healthy (probe flash, not picotool, to avoid I2C corruption per LL Entry 25).

---

## 4. Hardware Soak Tests

**Run:** Serial monitor for 60s-10min, check sensor status via `s` command.

**What to check:**
- IMU read count increasing at ~999/s
- IMU error count: 0 (after boot transient)
- Baro error count: 0
- ESKF: HEALTHY, predict timing ~600µs avg
- AO_Counter jitter: avg ~100,000µs (10Hz exact)
- No `[QP ASSERT]` messages
- USB CDC stable (no disconnect/reconnect)

**Gate criteria (IVP-81):**
- 10-minute soak: zero sensor errors, zero unexpected state transitions
- Core 1 sensor rate ~993-1000 Hz
- Bench flight sim 9/9 PASS

---

## 5. Dynamic Validation (Physical Tests)

**Location:** `docs/DYNAMIC_VALIDATION.md`

Physical motion tests with known truth references for ESKF accuracy:

| Method | What It Tests | Effort |
|--------|---------------|--------|
| Allan Variance | Sensor noise characterization (Q tuning) | Low |
| Turntable rotation | Gyro + attitude tracking | Low |
| Pendulum | Oscillation tracking, gravity alignment | Very low |
| Elevator | Baro altitude accuracy | Low |
| Data logging + replay | Regression testing (all above become repeatable) | Medium |
| Vehicle GPS-vs-INS | Full dynamic accuracy | Medium |

**Priority:** Allan variance first (foundational), then data logging infrastructure (multiplier for all other tests).

---

## Relationship Between Layers

```
  Formal Verification (SPIN)
  "Is the design correct?"
  ├── Proves: event topology, pyro safety, deadlock freedom
  └── Cannot prove: timing, sensor accuracy, hardware behavior
          │
  Host Unit Tests
  "Is the code correct?"
  ├── Proves: math, logic, state transitions, encoding
  └── Cannot prove: cross-module integration, hardware interaction
          │
  Bench Flight Sim
  "Does the pipeline work end-to-end?"
  ├── Proves: CLI → command → HSM → action → output
  └── Cannot prove: sensor accuracy, timing under load
          │
  Hardware Soak
  "Does it survive real-time operation?"
  ├── Proves: stability, bus health, memory, watchdog
  └── Cannot prove: dynamic accuracy under motion
          │
  Dynamic Validation
  "Does it track real physics?"
  └── Proves: ESKF accuracy under known motion
```

Each layer catches a class of bugs that the others miss. Skipping any layer leaves a gap.

---

## CLI Quick Reference

| Command | What It Tests |
|---------|--------------|
| `python scripts/bench_flight_sim.py` | Flight Director 9-scenario sim |
| `python scripts/cli_test.py all` | Non-destructive CLI tests |
| `python scripts/accel_cal_6pos.py` | 6-position accel calibration |
| `ctest --test-dir build_host/` | 552 host unit tests |
| See `tools/spin/README.md` | SPIN formal verification |

---

## References

- [SPIN User Guide](../tools/spin/README.md) — Formal verification setup, properties, interpretation
- [ESKF Testing Guide](ESKF_TESTING_GUIDE.md) — Host-side ESKF tests, replay harness, NIS diagnostics
- [Dynamic Validation Methods](DYNAMIC_VALIDATION.md) — Physical test methods for ESKF accuracy
- [IVP.md](IVP.md) — Integration gates (pass/fail criteria per feature)
- [DEBUG_OUTPUT.md](../standards/DEBUG_OUTPUT.md) — Serial testing via Python, CLI test scripts
- [Lessons Learned](../.claude/LESSONS_LEARNED.md) — Debugging journal with known failure modes
