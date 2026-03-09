# Plan: Computational Load Analysis + Runtime Behavior Map

## Context

Stage 7 (Radio & Telemetry) is complete. Before Stage 8 (Flight Director), two audit documents are needed to understand the current firmware's runtime characteristics. No comprehensive map exists of CPU utilization per core, and no behavior map documents all execution paths, state transitions, and error recovery flows. These audits will:
- Quantify headroom for Stage 8 additions (state machine, pyro monitoring, phase-scheduled Q/R)
- Catch dead paths, missing error handling, and undocumented state transitions before adding complexity
- Serve as ongoing reference documents

## Deliverables

1. **`docs/audits/COMPUTATIONAL_LOAD_ANALYSIS.md`** — CPU utilization profile with measured timing data per core
2. **`docs/audits/RUNTIME_BEHAVIOR_MAP.md`** — All execution paths, state machines, cross-core interactions, error recovery flows
3. **`scripts/cla_collect.py`** — Repeatable CLA data collection script (serial connect → soak → parse → markdown output)
4. **CLA instrumentation code** — gated behind `ROCKETCHIP_CLA_INSTRUMENTATION` CMake define (stays in codebase, compiled out by default)
5. **Cross-reference note in `docs/HARDWARE_BUDGETS.md`** — Point to CLA for current timing data (HARDWARE_BUDGETS has stale IVP-13 numbers)
6. **Graphviz `.dot` diagrams** — Extend existing `tools/state_to_dot.py` pattern for RBM state machines (renderable to SVG via `dot`)

## Order of Operations

**CLA first → RBM second.** The CLA produces measured timing data that the RBM references in its blocking path analysis and headroom sections.

---

## Part 1: Computational Load Analysis (CLA)

### Approach: Hybrid (consolidate existing benchmarks + instrument gaps)

**Already measured (no new instrumentation):**
- ESKF predict (codegen FPFT, SRAM): 111µs avg, 101µs min, 156µs max
- Bierman scalar update: 43µs avg
- Full ESKF predict+factorize+reconstruct: 561µs avg, 106µs min, 735µs max
- I2C ICM-20948 full read: from HARDWARE_BUDGETS.md
- I2C DPS310 read: from HARDWARE_BUDGETS.md
- Core 1 cycle target: 1000µs with busy_wait padding

**Gaps requiring instrumentation:**
- `eskf_tick()` total wall time (all measurement updates combined)
- `logging_tick()` wall time (decimator + PCM encode + ring push)
- `telemetry_radio_tick()` TX and RX paths
- `mavlink_direct_tick()` wall time
- `cli_update_tick()` wall time
- Core 1 per-cycle actual elapsed distribution (already computes `elapsed` but doesn't record min/max/sum)
- Seqlock: writer hold time (Core 1), reader retry count (Core 0)
- **Jitter:** inter-invocation period variance for `eskf_tick()` — record timestamp of each call, compute period delta std dev. Critical for filter stability (Liu & Layland scheduling theory applied to cooperative dispatch)
- **Stack high-water mark:** paint both core stacks with 0xDEADBEEF sentinel at boot, scan for high-water after soak. Essential given LL Entry 1 stack overflow history

### Instrumentation Method

Add `time_us_32()` probes around each tick function in main loop. Same pattern already used for ESKF predict timing (main.cpp ~line 2047). Accumulate min/max/sum/count into static variables. Add a CLI command or extend 's' output to print timing stats. ~20 lines per tick function.

```
Steps:
1. Consolidate existing benchmark data from:
   - docs/HARDWARE_BUDGETS.md (I2C timing)
   - docs/benchmarks/UD_BENCHMARK_RESULTS.md (ESKF/Bierman timing)
   - AGENT_WHITEBOARD resolved sections (codegen, SRAM benchmarks)

2. Add instrumentation to src/main.cpp:
   - Static struct per tick: { uint32_t min, max, sum, count }
   - time_us_32() before/after each tick function call
   - Core 1: track min/max/sum of existing `elapsed` variable
   - CLI print command for all timing stats

3. Build, flash via debug probe, run 60s soak test

4. Collect timing data from serial output

5. Gate instrumentation behind CMake define ROCKETCHIP_CLA_INSTRUMENTATION
   (stays in codebase permanently, compiled out by default)

6. Write CLA document with all data tagged by confidence:
   - [MEASURED-SOAK] — from this instrumentation run
   - [MEASURED-BENCH] — from prior benchmark (with date/source)
   - [CALCULATED] — derived from measured inputs (e.g., headroom = tick_period - WCET)
   - [ESTIMATED] — informed guess without direct measurement
   - [DATASHEET] — vendor specification

7. Confirm instrumentation overhead is negligible:
   - time_us_32() is a single SIO register read (~1 cycle)
   - Static accumulation (min/max/sum) is ~5 ALU ops
   - State explicitly in methodology section
```

### CLA Document Structure

```
1. Executive Summary — lead with headroom table:
   | Core | Current Duty% | Stage 8 Estimate | Margin |
   Per-core utilization at a glance. This is the headline number.
2. Platform Parameters — RP2350 specs, SRAM/flash layout, no-OS cooperative dispatch
3. Core 0 Tick Budget — table: function, rate(Hz), WCET(µs), avg(µs), duty(%), source
   - heartbeat_tick (1Hz)
   - watchdog_kick_tick (1kHz)
   - eskf_tick (200Hz) — with sub-breakdown: predict, baro, mag, ZUPT, GPS, Mahony
   - logging_tick (50Hz)
   - telemetry_radio_tick (2-10Hz)
   - mavlink_direct_tick (1-10Hz)
   - cli_update_tick (20Hz)
4. Core 1 Sensor Budget — table: component, rate, duration, duty%
   - IMU, baro, GPS, mag, seqlock, NeoPixel, cal feed, watchdog flag
   - Worst-case cycle (IMU + baro + GPS coincide)
4b. Worst-Case Coincidence Stack-Up (NASA SWE-139)
   Sum of WCET for all functions that can fire in the same tick:
   - Core 0: eskf_tick(WCET) + logging_tick(WCET) + telemetry_radio_tick(WCET) + USB ISR
   - Core 1: IMU + baro + GPS + seqlock + NeoPixel (all in one cycle)
   - Cross-core: flash_safe_execute lockout stalling Core 1 while Core 0 flushes
   Hyperperiod: LCM of all tick rates (1kHz, 200Hz, 50Hz, 20Hz, 10Hz) — compute explicitly.
   Every hyperperiod, ALL ticks coincide. Prove margin at this point.
4c. Jitter Analysis
   Inter-invocation period variance for eskf_tick() (200Hz target = 5000µs period).
   Report: mean period, std dev, min/max period, histogram if useful.
   Threshold: σ < 500µs is acceptable for ESKF stability (10% of period).
5. Cross-Core Synchronization Overhead
   - Seqlock: writer hold time (Core 1), reader retry count/max (Core 0)
   - Atomic flags: negligible but documented
   - Calibration pause handoff: max wait time
   - flash_safe_execute lockout: max Core 1 stall duration
6. ISR Budget — USB CDC (Core 0), UART RX GPS (interrupt-driven), PIO NeoPixel
   Measure worst-case ISR execution time. Document preemption behavior.
7. Memory Budget
   - text, BSS, .time_critical, PSRAM, stack per core
   - Stack high-water mark: 0xDEADBEEF sentinel paint → scan after soak
   - Report: stack allocated, stack used (high-water), margin
8. Blocking Operations — flash flush, calibration commands, GPS I2C read, USB settle
9. Headroom Assessment
   - Define headroom target numerically (e.g., no core exceeds 50% duty including worst-case)
   - Available margin per core
   - Stage 8 cost estimates (state machine, pyro monitoring, phase-scheduled Q/R)
   - Core migration recommendations
10. Data Sources Table — every measurement mapped to its source and confidence level
```

### Critical Files
- `src/main.cpp` — tick functions, Core 1 loop (instrumentation target)
- `CMakeLists.txt` — add `ROCKETCHIP_CLA_INSTRUMENTATION` option
- `docs/HARDWARE_BUDGETS.md` — existing I2C timing data to consolidate
- `docs/benchmarks/UD_BENCHMARK_RESULTS.md` — ESKF/Bierman timing data
- `tools/state_to_dot.py` — existing Graphviz pattern (reference, not modified)

---

## Part 2: Runtime Behavior Map (RBM)

### Approach: Mermaid + Graphviz diagrams for state machines, structured markdown for call trees

**Graphviz:** Existing `tools/state_to_dot.py` generates `.dot` files from state definitions → renderable to SVG via `dot` command. Extend this pattern for RBM state machines and error recovery flowcharts. Graphviz `.dot` files are text-based, diffable, and produce publication-quality diagrams. Store `.dot` sources in `docs/audits/dot/`, render SVGs alongside.

**Mermaid:** Use for simpler diagrams where GitHub inline rendering is sufficient (CLI state machine, NeoPixel hierarchy). Mermaid is lower-friction for readers who just want to see the diagram in the markdown.

**Optional Doxygen scaffold:** Try call graph generation first. If clean output without excessive setup, use as starting point. **Hard boundary: 30 minutes max, then skip.**

### Diagram Guidelines
- **Keep diagrams modular:** 5-8 focused diagrams, not 1-2 monolithic charts
- **Graphviz for complex state/flow diagrams** (error recovery, boot sequence, concurrency timeline)
- **Mermaid for simple state machines** (CLI menus, NeoPixel hierarchy)
- **Annotate with real CLA numbers** where applicable (not just structure)

### RBM Document Structure

```
1. Overview — purpose, conventions (Mermaid for states, indented markdown for call trees)

2. Happy-Path Timing Diagram (Graphviz or ASCII timeline)
   Single timeline showing one normal iteration on each core when nothing goes wrong:
   - Core 0: 5ms tick (200Hz ESKF epoch) — what happens in what order, with MEASURED durations
   - Core 1: 1ms tick (1kHz sensor cycle) — sequential breakdown, with MEASURED durations
   Annotated with real CLA numbers (e.g., "ESKF predict 111µs, idle 4844µs").
   This is the "architecture at a glance" diagram — baseline against which all error paths make sense.

2b. Concurrency Timeline (Graphviz — Gantt-style)
   Both cores across system lifecycle phases: boot → init → sensor-phase → armed → flight.
   Shows what runs where at each phase, with cross-core interaction points:
   - Seqlock boundaries
   - Atomic flag transitions (g_startSensorPhase, g_core1LockoutReady)
   - I2C ownership handoffs (calibration pause/resume)
   - flash_safe_execute lockout windows
   Makes cross-core interactions visually obvious in a way call trees cannot.

3. Boot Sequence (indented call tree)
   main() → init_hardware() → init_application() → main loop
   All branches: watchdog check, PSRAM, Core 1 launch, I2C, sensors, SPI, radio, USB, hooks

4. Core 0 Main Loop Dispatch (indented call trees per tick function)
   Each tick to full call depth, annotated with rate and CLA timing data

5. Core 1 Sensor Loop (indented call tree)
   IMU → baro → GPS → seqlock → watchdog → NeoPixel → sleep
   Divider branches, cal feed, pause/reload check

6. CLI State Machine (Mermaid stateDiagram)
   Disconnected ↔ Connected → MainMenu ↔ CalMenu / ESKFLive / MAVLinkMode
   All key mappings documented

7. NeoPixel Priority Hierarchy (Mermaid flowchart)
   Decision tree: cal override → RX overlay → timeout → ESKF init → GPS → fallback

8. Error Recovery Paths (Mermaid flowcharts per subsystem)
   Every path must terminate in one of three defined outcomes:
   (a) Recovered and resumed normal operation
   (b) Degraded mode with documented capability loss
   (c) Watchdog reset (unrecoverable)

   For known gaps, annotate: trigger condition, current behavior, correct behavior.

   For each degraded outcome, document what capability is lost (feeds Stage 8 Flight Director).

   8.1 IMU: I2C fail / zero-output → consecutive fail escalation → recover → reinit → (a)
   8.2 Baro: read fail only → (b) degraded: no altitude updates, ESKF baro-only alt lost,
       GPS+ZUPT fallback. [KNOWN GAP: no recovery mechanism, no re-init of continuous mode]
   8.3 GPS: parse fail → skip cycle → (b) degraded: dead-reckoning only, no position updates
   8.4 ESKF: 5 health sentinels → coast → reinit → (a) or (b) degraded: no nav solution,
       NeoPixel indicates UNHEALTHY, all measurement updates skipped
   8.5 Watchdog: 5s dual-core AND → reboot → sentinel check → (c)
   8.6 USB: connect/disconnect → banner/drain → (a)
   8.7 Flash: flash_safe_execute → lockout → i2c_bus_reset → (a)
   8.8 I2C Bus: SCL check → 9-clock → deinit/reinit peripheral → (a)

9. Cross-Core Communication Map (table)
   All seqlock, atomic flags, multicore_lockout — writer, reader, type, purpose

10. Sensor Detection Permutations (table)
    All hardware presence combinations → resulting firmware behavior

11. Blocking Code Paths (table with CLA timing references)
    All paths that stall the main loop, duration, watchdog impact

12. Identified Gaps and Dead Paths
    Dead/unreachable code, missing error handling, undocumented transitions
    Known gaps: baro freeze recovery, radio retry/timeout, ESKF active reset
```

### Critical Files
- `src/main.cpp` — boot sequence, tick dispatch, Core 1 loop, all cross-core primitives
- `src/cli/rc_os.cpp` — CLI state machine, menu handlers, USB handling
- `src/drivers/icm20948.cpp` — IMU error recovery, mag bypass
- `src/drivers/baro_dps310.cpp` — baro read logic (and lack of recovery)
- `src/drivers/gps_uart.cpp` — UART GPS, ring buffer, overflow handling
- `src/telemetry/telemetry_service.cpp` — TX/RX tick paths
- `src/fusion/eskf.cpp` — health checks, measurement gates, filter reset
- `src/drivers/ws2812_status.cpp` — NeoPixel modes and transitions

---

## Verification

- **CLA:** Timing data collected from actual 60s HW soak via debug probe. All values tagged with confidence level. Cross-check: Core 0 duty% + Core 1 duty% should sum to reasonable utilization (both <50% expected).
- **RBM:** Every Mermaid diagram validated by tracing the corresponding source code paths. Sensor detection permutation table verified against `init_sensors()` probe-first logic. Gap list cross-checked against AGENT_WHITEBOARD known issues.
- **Both:** Review for consistency — CLA timing values referenced in RBM blocking paths section should match.

## Scope Boundaries

- **Document current state as-is.** Note gaps and optimization opportunities but don't fix them.
- **No code changes** except temporary instrumentation for CLA (removed or `#ifdef`'d after data collection).
- **No protected file edits.** HARDWARE_BUDGETS.md gets a cross-reference note only (not a protected file).
- **Stage 8 headroom analysis is advisory** — recommendations, not implementation.

---

## Part 3: Repeatability (Living Document Infrastructure)

Both documents are living artifacts — they must be re-runnable as the codebase evolves.

### CLA Repeatability

**Instrumentation stays in the codebase** (compiled out by default):
```cmake
option(ROCKETCHIP_CLA_INSTRUMENTATION "Enable CLA timing probes" OFF)
if(ROCKETCHIP_CLA_INSTRUMENTATION)
    add_compile_definitions(CLA_INSTRUMENTATION=1)
endif()
```

**Collection script: `scripts/cla_collect.py`**
- Connects to serial (COM7), sends CLA dump command
- Collects 60s of timing data, parses output
- Outputs markdown table in CLA format
- Compares against previous baseline (flag WCET regressions >20%)
- Usage: `python scripts/cla_collect.py --port COM7 --duration 60 --baseline docs/audits/cla_previous.md`

**Re-run process (entire CLA update):**
```bash
cmake --build build/ -DROCKETCHIP_CLA_INSTRUMENTATION=ON
# Flash via debug probe
python scripts/cla_collect.py --port COM7 --duration 60 --output docs/benchmarks/cla_$(date +%F).md
# Compare against previous
python scripts/cla_collect.py --compare docs/benchmarks/cla_previous.md docs/benchmarks/cla_$(date +%F).md
```

**Dated snapshots:** Each run produces `docs/benchmarks/cla_YYYY-MM-DD.md`. The main CLA document references the latest snapshot. Historical snapshots enable regression tracking.

### RBM Repeatability

**Staleness check:** A simple grep-based script (`scripts/rbm_check.py`) that:
- Scans RBM for function names → verifies they still exist in source
- Scans main.cpp for tick functions → verifies they're all documented in RBM
- Flags new functions/states not yet in the behavior map
- Usage: `python scripts/rbm_check.py`

**Graphviz diagrams:** `.dot` source files in `docs/audits/dot/`. Render script:
```bash
for f in docs/audits/dot/*.dot; do dot -Tsvg "$f" -o "${f%.dot}.svg"; done
```

**When to re-run:**
- CLA: After any stage completion, or when adding new tick functions / changing rates
- RBM: After adding new state transitions, error recovery paths, or CLI commands
- Both: Before major milestones (pre-flight, design reviews)

### Hardware Configuration for Reproducibility

Document in CLA methodology section:
- Sensors connected (IMU, baro, GPS model, radio)
- Qwiic chain order
- Power source (USB vs LiPo)
- Flash method (debug probe, not picotool — per LL Entry 25)
- Soak conditions (device stationary, terminal connected, room temperature)

---

## Council Review

**Reviewers:** Retired NASA/JPL Avionics Lead, Senior Aerospace Student, Embedded Systems Professor
**Verdict:** Approve with modifications (all incorporated)

Round 1 (NASA/JPL Lead + Senior Student):
1. CLA: Added worst-case coincidence stack-up (Section 4b, NASA SWE-139)
2. CLA: Added CALCULATED confidence tier (distinct from ESTIMATED)
3. CLA: Headroom summary table leads the Executive Summary
4. CLA: Instrumentation overhead stated explicitly in methodology
5. RBM: Added happy-path timing diagram (Section 2) as "architecture at a glance"
6. RBM: Every error recovery path terminates in defined outcome (recovered/degraded/watchdog)
7. RBM: Diagrams kept modular (5-8 focused, not monolithic)
8. Doxygen: Hard 30-minute boundary, then skip

Round 2 (Embedded Systems Professor):
9. CLA: Added jitter measurement (Section 4c) — inter-invocation period variance for ESKF
10. CLA: Added stack high-water mark (Section 7) — sentinel paint + scan after soak
11. CLA: Added hyperperiod analysis to worst-case stack-up (LCM of all tick rates)
12. CLA: Added seqlock hold time + reader retry count profiling (Section 5)
13. CLA: Define headroom target numerically (Section 9)
14. RBM: Added concurrency timeline (Section 2b) — Gantt-style both cores across lifecycle
15. RBM: Added degraded-mode capability loss annotations to error recovery paths
16. Added Part 3: Repeatability — CMake flag, collection script, staleness check, dated snapshots
17. Graphviz `.dot` diagrams extending `tools/state_to_dot.py` pattern (separate tooling for RBM)
