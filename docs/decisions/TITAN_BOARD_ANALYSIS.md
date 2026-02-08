# Titan Board: Path & Market Value — Council Panel Analysis

**Date:** 2026-02-07
**Status:** Council Review Complete — Conditional GO
**Panel:** NASA/JPL Avionics Lead, STEM Educator / Space Camp Counselor, Embedded Systems Professor
**Coordinator:** Claude Code CLI
**Context:** Project at IVP Stage 3 complete, entering Stage 4

---

## Panel Composition

| Panelist | Focus Area | Guiding Question |
|----------|-----------|-----------------|
| Retired NASA/JPL Avionics Lead | Safety, FMEA, validation | "What's the failure mode and can we prove the code catches it?" |
| STEM Educator / Space Camp Counselor | Market, education, accessibility | "Can I teach with this, and will it survive 30 teenagers?" |
| Embedded Systems Professor | Architecture, theory, compute budgets | "Why this design? What's the theory?" |

---

## Panelist 1: Retired NASA/JPL Avionics Lead

### Technical Path

The path to Titan is **strictly sequential** in the safety-critical areas. Sensor fusion must be flight-validated on Core/Main before pyro work begins — state machine bugs that misidentify apogee are an annoyance on a logger but a **safety event** when connected to pyro channels.

**Parallelizable now:** ADXL375 driver, PIO servo PWM, CAN bus driver, Gemini PCB design (once interface spec locked).

**Not parallelizable:** Pyro firmware (needs proven state machine), TVC control loops (needs validated IMU latency), MMAE bank (needs flight-proven single ESKF).

**Validation demands:** 20+ dual-deploy flights on Main before Titan pyro firmware is declared ready. Real pyro HITL testing with actual igniters, not LED switching tests. Independent code review of all pyro-path code.

### Safety Assessment (Ranked Failure Modes)

1. **Inadvertent pyro firing.** Gemini hardware voting (discrete AND gates) is essential, not nice-to-have. A single processor going haywire must not actuate a pyro channel. The hardware voting logic should be discrete logic — AND gates on the MOSFET gate drivers — not firmware on a third MCU.

2. **TVC servo runaway.** A servo commanded to full deflection during boost produces an immediate departure from planned trajectory. TVC code needs hard limits enforced at the PWM generation level, in the PIO program itself, not just in the control loop software. PIO's isolated execution model should be exploited for defense-in-depth.

3. **State machine lockup during descent.** Watchdog must trigger backup pyro firing, not just reset the processor. A reset during descent may re-enter IDLE and never fire recovery charges. Recommends independent barometric backup timer for recovery deployment (like Altus Metrum's hardware approach).

4. **High-G sensor saturation.** ADXL375 at ±200g covers most high-power scenarios, but motor cato transients can exceed this. ESKF must clamp and flag saturated readings, not feed garbage into the filter.

### Market Position

Titan fills a unique gap: **open-source TVC + hardware-voted pyro safety + modular Booster Packs**. No competitor offers all three.

| Competitor | Strengths | Titan Advantage |
|-----------|-----------|-----------------|
| Altus Metrum TeleMega/EasyMega | Proven, reliable, excellent telemetry | Open-source, modular, TVC capability, lower cost |
| Fluctus (Silicdyne) | All-in-one, bundled GCS | Open-source, modular architecture |
| Eggtimer Quasar/Proton | Affordable, well-documented | Higher capability, safety features |
| AVA (BPS.space) | TVC demonstrated, landing capability | Available product, open-source, documented |

RP2350 is adequate; pin/DMA budget is the real constraint, not compute. Map the full Titan peripheral set on paper before committing to board layout.

### MVP Titan Recommendation

**Ship (v1):** ADXL375 + dual pyro + Gemini hardware voting + proven single ESKF + arm switch + independent backup timer.

**Cut from v1:** TVC, MMAE, CAN bus.

**Timeline estimate:** 12-18 months of disciplined development from current state.

> *"Build the safety case first. Prove the state machine on dozens of flights. The features will follow, and they will be credible because the foundation is solid. That is what separates a flight computer from a development board with ambitions."*

---

## Panelist 2: STEM Educator / Space Camp Counselor

### Educational Value of the Tier System

Core→Main→Titan is a **staircase, not a cliff**. Right now the jump from "Eggtimer beeped at apogee" to "ESKF with TVC" has no intermediate steps. RocketChip's tier system could be that staircase.

Open source is THE value proposition for education. When a student asks "how does the Kalman filter work?" you can point them at the actual code. With a TeleMega, the answer is "it's proprietary, trust the black box." That is the opposite of education.

**Titan needs a "Learning Path" curriculum:**
- Semester 1: Datalogger (Core)
- Semester 2: Add pyro, dual-deploy (Main + Pyro Pack)
- Semester 3: TVC, sensor fusion (Titan)
- Lab guides, not just API docs: "Lab 4: Implement complementary filter. Lab 5: Compare to ESKF."

### Market Reality

| Metric | Core | Main | Titan |
|--------|------|------|-------|
| Est. units/year | 20,000-50,000 | 5,000-15,000 | 2,000-5,000 |
| Buyers | Hobbyists, schools, makers | L1-L2 fliers, clubs | University teams, L2/L3 certified, engineering clubs |
| Price sensitivity | $20-50 | $75-150 | $150-250 |

**Critical insight:** Titan buyers are **evangelists** who drive Core sales through credibility. A university team flying Titan at a competition tells 500 spectators "this is RocketChip."

**Price sweet spot:** $150-250 assembled. Above $250 → teams build their own. Below $150 → suspicion about pyro safety.

Open source is a purchasing factor for competition teams — judges score "understood our avionics" higher than "bought a black box."

### Accessibility Requirements

- Booster Packs are **essential** — Titan must NOT ship as monolith. Incremental capability addition = curriculum mapping.
- Need "Titan Lite" config for schools where energetics aren't allowed (no pyro wing installed — configuration, not separate SKU).
- QGC/Mission Planner compatibility = resume material for students.
- The "Flipper Zero" tagline works for Core/Main; Titan may need its own identity: "open-source avionics for the next generation."

### Single Most Important Feature

**Pyro channel reliability with auditable safety logic.** Everything else is impressive but optional. Get pyro right, the rest sells itself.

> *"Titan is not where you make money. Titan is where you earn trust. Build it right, make it teachable, and Core sales will follow."*

---

## Panelist 3: Embedded Systems Professor

### Architectural Assessment

Bare-metal Pico SDK is the **correct starting point**. The theoretical argument for an RTOS at Titan's workload is strong on paper: priority-based preemption gives formal schedulability analysis via Rate Monotonic Analysis (RMA). However, RMA presupposes well-characterized WCETs, which do not yet exist. Introducing an RTOS now would be premature abstraction.

The dual-core AMP strategy is theoretically sound. Core 1 as a dedicated sensor-sampling core is essentially a **hardware-enforced priority ceiling** — no lower-priority task can ever preempt it because nothing else runs there. This is stronger than software priority inversion prevention.

**Core 0 superloop concern:** No formal admission control. A long-running fusion update blocks telemetry; a flash write blocks everything. The architecture needs a formal timing budget before Titan ships. Needs a timing monitor that flags overruns during development.

Seqlock pattern is correct and appropriate for the data rates (~1kHz writer / 200Hz reader). The key correctness requirement is proper atomic operations with acquire/release semantics.

### Sensor Fusion Architecture

ESKF is the correct theoretical choice (reference: Sola, "Quaternion kinematics for the error-state Kalman filter," 2017). Error-state formulation keeps the state vector small (15 states), operates in a nearly-linear regime, and avoids quaternion normalization issues.

The three-layer architecture (Mission Config → Hypothesis Library → MMAE → Confidence Gate → Mission Engine) is textbook clean. Separates estimation from model selection from decision-making.

Mahony AHRS over Madgwick is defensible: gyro bias estimation via PI correction term, better stability under high angular rates during boost and spin.

### Compute Budget (Theoretical Estimates)

| Subsystem | Rate | Est. Time/Invocation | CPU Load |
|-----------|------|---------------------|----------|
| ESKF (single) | 200Hz | 150us | 3.0% |
| MMAE (x3) | 200Hz | 450us | 9.0% |
| PID control | 500Hz | 20us | 1.0% |
| MAVLink TX | 10Hz | 200us | 0.2% |
| Data logging | 50Hz | 500us | 2.5% |
| GPS parse | 10Hz | 100us | 0.1% |
| Mission engine | 100Hz | 50us | 0.5% |
| **Total** | | | **~16.3%** |

Feasible but **all numbers are theoretical**. Flash writes are the wild card — PSRAM-buffered logging with deferred flash writes is essential for Titan. CMSIS-DSP's `arm_mat_mult_f32` on Cortex-M33 at 150MHz handles 15x15 matrices in 15-25 microseconds.

### Software Engineering Quality

JSF AV C++ for a hobbyist product is **pedagogically strategic, not overkill**. The standards enforce habits (fixed-width types, no dynamic allocation post-init, named constants) that prevent the class of bugs most common in embedded systems.

Compile-time tier flags create a **combinatorial testing problem**. Three tiers times multiple Booster Pack configs equals a testing matrix that could overwhelm a small team. The IVP should include build-matrix verification.

Virtual base classes for drivers provide a clean porting path to future MCUs (STM32H7, ESP32-S3, or future RP2450).

### Platform Growth Path

When (not if) RP2350 hits limits, the architecture supports porting. The constraint will be peripheral bandwidth and pin budget, not compute. Map the full Titan peripheral pin and DMA channel allocation on paper before board layout.

> *"Nothing embarrassing. The architecture is clean, the standards are serious, the documentation is thorough. My only caution is the classic one: do not let the architecture documents outpace the running code. The IVP exists to prevent that. Follow it."*

---

## Coordinator Synthesis

### Consensus Points (All Three Agree)

1. **Sequential progression is non-negotiable.** Core must be flight-proven before Titan pyro work begins. No shortcuts through the IVP gates.
2. **Pyro safety is the single highest-stakes feature.** Hardware arm switch, auditable logic, council review before any pyro code merges.
3. **Single ESKF before MMAE.** Flight data first, multi-model estimation later.
4. **RP2350 compute budget is feasible but unproven.** Need measured timing, not estimates.
5. **Booster Pack modularity is essential.** Titan must not ship as a monolith.
6. **Open source is a core differentiator** for both education and market trust.
7. **Cut TVC, MMAE, CAN from Titan v1.** Ship the safety-critical features first.

### Tensions/Disagreements

| Tension | Panelists | Resolution |
|---------|-----------|------------|
| Timeline optimism vs caution | NASA/JPL (12-18mo) vs Professor (warns docs outpace code) | Follow IVP gates strictly; timeline is outcome of discipline, not a deadline |
| Gemini voting: hardware-only vs firmware-assisted | NASA/JPL (discrete logic only) vs Professor (notes PCB complexity) | Needs resolution before Gemini PCB layout |
| "Titan Lite" for schools | Educator (wants pyro-free config) vs others (not addressed) | Booster Pack configuration, not separate SKU — no firmware scope creep |
| Superloop admission control | Professor raises; others don't address | Real gap — add timing monitor to IVP |

### Red Flags

1. **No measured timing budget exists.** All CPU estimates are theoretical. A single slow flash write or unexpectedly expensive ESKF iteration could break the real-time budget.
2. **Compile-time tier flags create combinatorial test burden.** Three tiers × N feature flags = testing matrix that could overwhelm a small team. Needs build-matrix CI early.
3. **Current project is at Stage 3 complete.** Substantial work remains on Core/Main before Titan is relevant. Risk of distraction from immediate IVP priorities.

### Standards Alignment Check

| Standard | Status | Notes |
|----------|--------|-------|
| JSF AV C++ compliance | Aligned | All panelists endorse; Professor calls it "pedagogically strategic" |
| IVP sequential gates | Aligned | NASA/JPL's "20+ flights" maps to IVP; Professor reinforces discipline |
| No arbitrary numbers | **Gap** | All compute budget numbers are theoretical; per standards, cannot be design commitments until measured |
| Prior art research | Aligned | ArduPilot calibration, Altus Metrum backup timers, PIO servo patterns referenced |
| Pyro council review | Aligned | Required by coding standards; all panelists agree |
| Simplicity first | Aligned | Cutting TVC/MMAE/CAN from v1 follows this principle |

---

## Final Verdict

### CONDITIONAL GO

The Titan path is architecturally sound, market-viable, and standards-compliant in design.

**Conditions:**

1. Core/Main sensors flight-tested before any Titan-specific firmware work begins
2. Empirical timing measurements replace all theoretical CPU budget estimates
3. Titan v1 MVP scope formally locked:
   - ADXL375 high-G accelerometer (SPI)
   - Dual pyro channels with Gemini hardware voting (discrete AND gates)
   - Proven single-model ESKF, validated against flight data
   - 500Hz main loop with characterized jitter
   - Physical arm switch input (hardware interlock)
   - Independent backup timer for recovery deployment
4. Every line of pyro code passes council review before merge, no exceptions

**Panel reached consensus. No tie-breaking required.**

### Ranked Recommendations

| Priority | Action | Rationale |
|----------|--------|-----------|
| 1 | Continue IVP Stage 4+ (GPS, fusion, state machine) | Foundation must be flight-proven |
| 2 | Measure actual WCET for ESKF, sensor read, flash write | Replace theoretical estimates |
| 3 | Add Core 0 superloop timing monitor | Flag overruns during development |
| 4 | Begin ADXL375 driver and PIO servo PWM in parallel | Safely parallelizable per panel |
| 5 | Formally define and council-review Titan v1 MVP scope | Lock scope before implementation |
| 6 | Plan build-matrix CI for tier flag combinations | Prevent combinatorial test gaps |
| 7 | Defer TVC, MMAE, CAN to Titan v2+ | Reduce v1 risk and validation burden |

---

## Addendum: F' (F Prime) Comprehensive Evaluation

**Date:** 2026-02-08
**Context:** Multi-session feasibility research into NASA's F' flight software framework for RocketChip
**Research scope:** F' architecture, platform support, coding standards alignment, multicore limitations, platform candidates (RP2350, STM32H7, Pi Zero 2 W), hybrid architecture options

---

### 1. What F' Is

F' (F Prime) is NASA JPL's open-source flight software framework. It flew on **Mars Ingenuity** (72 flights on Mars), **ISS-RapidScat** (2 years on ISS), and **ASTERIA CubeSat** (2017). It is the institutional flight software standard at JPL.

**Core architecture:**
- **Components** — modular units encapsulating behavior, communicating only through typed ports
- **Ports** — typed interfaces between components (sync, async, guarded)
- **Topologies** — graph of connected components defining the system
- **FPP** — modeling language that auto-generates C++ stubs, serialization, GDS dictionaries, and test harnesses
- **GDS** — browser-based ground data system with commanding, telemetry, events, log viewing, file transfer

**Component types:**
- **Passive** — no thread, executes in caller's context (suitable for drivers)
- **Active** — has its own thread and message queue (suitable for mission logic, fusion)
- **Queued** — has queue but no thread (rare)

**What F' provides "for free":**
- Command dispatch and sequencing
- Telemetry channel management
- Structured event logging with severity levels (DIAGNOSTIC through FATAL)
- Parameter persistence (runtime-configurable values)
- Health monitoring between components
- Unit test framework with auto-generated harnesses
- File uplink/downlink

### 2. F' Coding Standards Alignment

F' coding rules are a **superset** of RocketChip's existing JSF AV C++ standards:

| Rule | F' | RocketChip (JSF AV) | Match? |
|------|-----|---------------------|--------|
| No exceptions | `-fno-exceptions` | Same | Yes |
| No RTTI | Forbidden | Same | Yes |
| No dynamic alloc after init | Yes | Yes | Yes |
| No recursion | Yes | Yes | Yes |
| Fixed loop bounds | Yes | Yes | Yes |
| No STL | Yes (uses Fw types) | N/A (bare-metal) | Compatible |
| Explicit enum values | Yes | Yes | Yes |
| Assert-based validation | `FW_ASSERT` | Custom asserts | Similar |
| No GOTOs | Yes | Yes | Yes |
| Check all return values | Yes | Yes | Yes |
| Minimize preprocessor | Yes | Yes | Yes |
| C++ standard | C++11 | C++11/14 | Compatible |
| Formatting | ClangFormat (Chromium) | Not enforced yet | Adoptable |

F' adds JPL heritage rules: prefer `Fw` and `Os` framework types, compile without warnings or static analysis failures, avoid `Os::Task::delay` for synchronization.

**Key finding:** The standards gap between RocketChip and F' is small. The main things F' adds that we lack are **structured telemetry/events** (replacing ad-hoc `DBG_PRINT`) and **auto-generated test harnesses**.

### 3. F' Platform Support

#### Officially Supported Platforms (from fprime docs)

| Hardware | OS | Architecture | Notes |
|----------|-----|-------------|-------|
| x86 | Linux | x86_64 | Primary dev target |
| Raspberry Pi | Linux | ARMv8 | Full F' support |
| **Adafruit Feather RP2350 HSTX** | **fprime-baremetal** | **ARM** | **Our exact board** (added v3.5.0) |
| Pi Pico / Pico 2 | Zephyr | ARM/RISC-V | Zephyr path |
| Feather M4 | FreeRTOS | ARM | SAMD51 |
| Teensy 4.1 | Zephyr | ARMv7-M | Cortex-M7 |
| NUCLEO-H723ZG | Zephyr | ARM | STM32H7 |
| BeagleBone Black | VxWorks 7 | ARMv7 | |

#### fprime-arduino Board Support (community package)

Tested with "basic LedBlinker deployment" — not complex multi-sensor systems:
- Raspberry Pi Pico 2 (RP2350) — F' 4.0.0
- **Adafruit Feather RP2350 HSTX** — F' 3.5.0
- Adafruit Feather RP2040 — F' 3.5.0
- Teensy 4.1, 4.0, 3.2
- ESP32 Dev Module
- STM32 NUCLEO boards
- Various Adafruit boards (nRF52840, SAMD21/51)

### 4. The Multicore Problem (Critical Finding)

**F' has no multicore execution model of its own.** On every platform, it delegates threading to the OS.

#### How F' "Does" Multicore on Linux

On Linux, each Active Component gets a `pthread_create()`. The Linux kernel scheduler distributes threads across available cores. F' optionally supports `pthread_attr_setaffinity_np()` for manual core pinning (GNU/Linux only). This is not a framework feature — it's "Linux handles threads, and F' doesn't break."

The actual source (`Os/Posix/Task.cpp`) shows:
- Thread creation via `pthread_create()` with a wrapper
- Optional CPU affinity via `pthread_attr_setaffinity_np()` (GNU-only, graceful fallback)
- Priority via `SCHED_RR` with system range clamping
- No automatic load balancing or core assignment

The F' docs say: *"We have yet to see any issues running F' on multi-core systems."* This is passive compatibility, not active multicore support.

#### Why This Blocks Every MCU Path

| Path | Multicore Status |
|------|-----------------|
| **F' + Zephyr** (RP2350 or STM32H7) | **Blocked.** Zephyr has no Cortex-M SMP support. Issue #59826 open since June 2023, targeted for Zephyr 4.4 (April 2026) but no committed date. Blocker: Cortex-M0/M33 lack exclusive monitor for Zephyr's spinlock model. |
| **F' + fprime-baremetal** (RP2350) | **Single-threaded.** Docs say "avoid Active Components at all costs." Rate groups on one core only. Core 1 completely outside F'. |
| **F' + FreeRTOS** (STM32H7) | **Experimental.** GitHub discussion #1933 shows people attempting H7 + FreeRTOS OSAL — builds compile but don't run on real hardware. |
| **F' + Linux** (Pi Zero 2 W, Raspberry Pi) | **Works.** Full pthreads, OS scheduler distributes across cores. This is where F' is designed to run. |

**Bottom line:** F' multicore works on Linux because Linux does the work. On every MCU target, F' is effectively single-core.

### 5. Platform Comparison for Titan

#### Hardware Specifications

| Spec | Pi Zero 2 W | STM32H743 | RP2350 (current) |
|------|------------|-----------|-------------------|
| **CPU** | 4x Cortex-A53 @ 1GHz | Cortex-M7 @ 480MHz (+M4 co-proc) | 2x Cortex-M33 @ 150MHz |
| **RAM** | 512MB SDRAM | 1MB SRAM (192KB TCM + 864KB user) | 520KB SRAM + 8MB PSRAM |
| **Flash/Storage** | SD card (GB+) | 2MB internal flash | 8MB QSPI flash |
| **Weight** | 9g | ~2-5g (board dependent) | ~5g (Feather) |
| **Dimensions** | 65 x 30mm | varies (Pixhawk ~50x30mm) | 50.8 x 22.8mm (Feather) |
| **Power (idle)** | ~100mA / 500mW | ~50-100mA typical | ~25-40mA |
| **Power (loaded)** | ~460mA / 2.3W | ~150-200mA | ~80-100mA |
| **Price** | ~$15 | ~$10-15 chip / $50+ board | ~$10 chip / $14 Feather |
| **WiFi/BT** | Yes (built-in) | No | No |
| **Real-time** | No (Linux, ~6ms worst-case PREEMPT_RT) | Yes (hard real-time, sub-µs) | Yes (bare-metal, sub-µs) |
| **FPU** | Double-precision (NEON) | Double-precision | Single-precision |
| **F' support** | First-class (Linux POSIX) | Experimental (Zephyr) | fprime-baremetal (LedBlinker only) |

#### F' Compatibility Comparison

| | Pi Zero 2 W | STM32H7 | RP2350 |
|---|---|---|---|
| Active Components | Full threading (pthreads) | Not working yet | Not recommended |
| Multi-core in F' | Yes — 4 cores via OS scheduler | Single-core (Zephyr SMP blocked) | Single-core |
| GDS connection | WiFi + serial | Serial only | Serial only |
| Flight heritage | Ingenuity ran F' on Linux (Snapdragon) | None in F' | None |

#### Strengths and Weaknesses

**Pi Zero 2 W + F' (Linux)**
- Every F' feature works: active components, threading, full GDS
- 512MB RAM = comfortable for ESKF + MMAE + logging + telemetry
- 4 cores = genuine parallelism
- WiFi = GDS dashboard during ground testing without wiring
- SD card = unlimited flight log storage
- *"Same framework as Mars Ingenuity"* — genuine marketing claim
- BUT: **No hard real-time** (~6ms worst-case jitter with PREEMPT_RT)
- BUT: **2.3W power** under load (tight on 400mAh battery)
- BUT: **10-30 second boot time** (Linux)
- BUT: **SD card corruption risk** on power loss

**STM32H7 + F'/Zephyr**
- Hard real-time (480MHz M7, TCM for zero-wait-state)
- M4 co-processor for sensor pre-processing
- Proven flight controller platform (Pixhawk 6X, CubeOrange+)
- Massive ArduPilot driver ecosystem
- Low power (~150-200mA loaded)
- BUT: **F' not working on H7 yet** (compiles, doesn't run)
- BUT: **Zephyr SMP blocked** — single-core only
- BUT: **1MB RAM** workable but not generous
- BUT: **Re-enters RTOS-land** (previously fled from, see AP_FreeRTOS branch)

### 6. Hybrid Architecture Proposal

The architecture that best leverages F' strengths while preserving real-time guarantees:

```
┌──────────────────────────────────────┐
│     Pi Zero 2 W (Mission CPU)        │
│     Linux + F' Framework             │
│                                      │
│  - Mission Engine (F' component)     │
│  - ESKF Sensor Fusion                │
│  - MMAE Bank (Titan)                 │
│  - GDS + Telemetry                   │
│  - Data Logging (SD card, unlimited) │
│  - WiFi ground testing               │
│  - GPS processing                    │
│  - Confidence Gate                   │
└──────────────┬───────────────────────┘
               │ SPI or UART (MAVLink or F' framing)
               │ ~1MHz+, <1ms latency
┌──────────────┴───────────────────────┐
│     RP2350 (Sensor/Safety CPU)       │
│     Bare-metal Pico SDK              │
│                                      │
│  - 1kHz IMU sampling (Core 1)        │
│  - Baro sampling                     │
│  - Pyro control (safety-critical)    │
│  - Watchdog                          │
│  - Hardware arm/fire interlocks      │
│  - Hard real-time guarantee          │
│  - Seqlock cross-core data sharing   │
└──────────────────────────────────────┘
```

**This mirrors the Pixhawk architecture** (STM32 main + STM32 IOMCU) but with an F'-powered Linux brain and the existing RP2350 as the real-time safety layer. It maps to the existing Gemini concept (SAD Section 14) — instead of two identical Cores, it's a powerful mission CPU + a real-time safety CPU.

**Benefits:**
- F' runs where it's designed to run (Linux, full POSIX, real threads)
- Hard real-time stays where it's needed (RP2350 bare-metal, validated in IVP-19 through IVP-30)
- Pyro safety remains on dedicated hardware with hardware interlocks
- Entire Stage 1-3 codebase stays intact on the RP2350
- Core tier ships with RP2350 alone; Titan adds Pi Zero 2 W as a "brain upgrade" Booster Pack
- Total weight: ~14g for both boards
- WiFi GDS during ground testing is a significant development velocity gain

**Concerns:**
- Two codebases (F'/Linux + bare-metal Pico SDK) with a bridge protocol between them
- Pi Zero 2 W power budget (2.3W loaded) requires larger battery or power management
- Linux boot time (10-30s) means RP2350 must be independently safe during boot
- SD card reliability under power loss (mitigated by RP2350 handling safety-critical logging)
- Cross-board communication latency adds to sensor-to-action pipeline

### 7. F' Component Mapping to RocketChip SAD

If F' is adopted (on any platform), the SAD module decomposition maps naturally:

| RocketChip SAD Module | F' Component Type | Notes |
|---|---|---|
| SensorTask (IMU, Baro, GPS) | Passive | Drivers wrapped in F' components |
| FusionTask (ESKF) | Active | Own thread, processes sensor port data |
| MissionEngine + StateMachine | Active | Flight state management, command handling |
| LoggerTask | Passive | Storage driver, invoked by rate group |
| TelemetryTask | Active | Downlink scheduling, MAVLink or F' framing |
| UITask / RC_OS | Passive | CLI replaced by F' GDS (or kept as companion) |
| ActionExecutor | Passive | Pyro, servo, LED — invoked on state transitions |
| CalibrationManager | Passive | Becomes F' parameters with persistence |
| HealthMonitor | Framework-provided | Built-in health checking between components |

### 8. What RocketChip Already Has That F' Would Provide

| Capability | Current RocketChip | F' Equivalent | Gap? |
|---|---|---|---|
| Sensor drivers | Custom C (icm20948, baro_dps310, gps_pa1010d) | Wrapped in passive components | Small — logic survives |
| Cross-core data sharing | Seqlock with atomics (validated) | F' ports (Linux) / custom bridge (hybrid) | F' can't replicate seqlock |
| CLI | RC_OS (custom) | GDS (browser-based) | GDS is significantly more capable |
| Debug output | DBG_PRINT macros | Structured events with severity levels | F' events are better |
| Calibration persistence | calibration_storage (flash) | F' parameters | Similar capability |
| Telemetry protocol | MAVLink (planned) | F' framing (custom) or CCSDS | **Incompatible** — no MAVLink in F' |
| Unit tests | None (planned) | Auto-generated harnesses from FPP | **Major gap** — F' biggest win |
| State machine | Planned (Phase 5) | F' state machine support (new in recent versions) | Natural fit |
| Watchdog | Hardware WDT (validated) | F' health component | Both needed |

### 9. MAVLink Compatibility Issue

F' uses its **own binary protocol** (F Prime framing) with optional CCSDS (space standard). It does **not** support MAVLink. This means:

- No QGroundControl compatibility
- No Mission Planner compatibility
- F' GDS replaces these with a browser-based dashboard
- If MAVLink is required for Titan (competition teams, QGC compatibility), a custom MAVLink bridge F' component would need to be written
- Alternative: Core/Main uses MAVLink (as planned), Titan uses F' GDS — different ground tools per tier

### 10. Flight Heritage

| Mission | Year | Platform | F' Role |
|---------|------|----------|---------|
| ISS-RapidScat | 2014-2016 | ISS instrument | Flight software framework |
| ASTERIA CubeSat | 2017 | CubeSat | Full flight software |
| **Mars Ingenuity** | 2021-2024 | Qualcomm Snapdragon (Linux) | Flight software for 72 Mars flights |
| Lunar Flashlight | Planned | CubeSat | Surface ice detection |
| NEA Scout | Planned | CubeSat | Asteroid mapping |
| Ocean Worlds Life Surveyor | Planned | Instrument | Life detection |

Ingenuity is the strongest reference: F' on Linux, on a flying vehicle, with real-time sensor processing and autonomous decision-making. 72 flights on Mars is compelling heritage.

### 11. Updated Strategic Implications

**Core/Main stays on RP2350 + Pico SDK bare-metal.** No change. The dual-core AMP architecture with seqlock is validated and correct. F' cannot replicate this on any MCU target today.

**Three viable Titan paths exist:**

| Path | Pros | Cons | Risk |
|------|------|------|------|
| **A: STM32H7 + F'/Zephyr** | Hard real-time, low power, ArduPilot ecosystem | F' not running on H7 yet, Zephyr SMP blocked, separate codebase | High (unproven) |
| **B: Pi Zero 2 W + F' (Linux)** | Full F' support, 4-core, WiFi GDS, Ingenuity heritage marketing | No hard RT, 2.3W power, boot time, SD corruption risk | Medium (proven framework, new integration) |
| **C: Hybrid (Pi Zero 2 W + RP2350)** | Best of both — F' on Linux, hard RT on RP2350 | Two boards, bridge protocol, power budget, complexity | Medium (both halves proven, bridge is new) |

**Path C (Hybrid) is the strongest architecture** — it mirrors proven flight computer designs (Pixhawk main+IOMCU, GASPACS CubeSat ran a Pi Zero) and leverages validated work on both halves. The Pi Zero 2 W becomes a Titan Booster Pack.

### 12. Candidate Dev Boards

**For F'/Linux (Pi Zero 2 W path):**
- Raspberry Pi Zero 2 W (~$15) — quad-core A53, 512MB, WiFi, 9g, 65x30mm
- Direct F' support, no porting needed
- Pairs with existing RP2350 Feather via SPI/UART

**For F'/Zephyr (STM32H7 path):**
- **Primary:** NUCLEO-H723ZG (~$30-40) — F'-tested board, 550MHz M7, integrated debugger
- **Secondary:** Matek H743-SLIM-V4 (~$40-50) — 480MHz M7, 1MB SRAM, onboard dual IMU + baro, ArduPilot-compatible
- Matek caveats: not the F'-tested chip (H723 vs H743), drone-oriented layout, needs custom Zephyr board definition

**Recommendation:** If pursuing F', start with Pi Zero 2 W — it's the path of least resistance (F' on Linux just works). STM32H7 + F'/Zephyr requires solving unsolved problems.

### 13. "Cherry-Pick" Alternative (No Full F' Adoption)

If the full F' adoption is too disruptive, the key F' benefits can be implemented independently:

1. **FPP-style component isolation** — enforce port-based interfaces with C++ abstract classes (already in SAD driver interfaces)
2. **Structured events** — severity-leveled, timestamped, typed event system (replacing DBG_PRINT)
3. **Parameter persistence** — extend calibration_storage to a general parameter system
4. **Auto-generated tests** — lightweight code-gen for test harnesses (biggest missing piece)
5. **GDS-style dashboard** — Python web dashboard reading telemetry stream (independent of F')
6. **Health monitoring** — component heartbeat pattern (straightforward to implement)

This preserves the single-codebase advantage across tiers while adopting F' patterns without F' framework overhead.

### 14. Open Questions for Future Council Review

1. Is the separate-codebase trade-off acceptable, or does shared firmware across tiers outweigh F' benefits?
2. Does F' GDS replace the need for MAVLink/QGroundControl compatibility on Titan? (F' uses its own protocol, not MAVLink)
3. Should the OpenMCT ground station plan (Stage 8) target F' GDS instead of MAVLink bridge for Titan?
4. When Zephyr Cortex-M SMP eventually ships, does it change the RP2350 calculus?
5. H723 vs H743 for final Titan PCB — 550MHz/564KB vs 480MHz/1MB trade-off?
6. **NEW:** Is the Pi Zero 2 W hybrid architecture (Path C) worth prototyping before committing to STM32H7?
7. **NEW:** Does the 2.3W power draw of Pi Zero 2 W require a larger battery SKU for Titan, and does that affect the weight/cost target?
8. **NEW:** Should Core/Main keep MAVLink while Titan uses F' GDS, or must all tiers use the same ground protocol?
9. **NEW:** Can the RP2350 in the hybrid architecture serve as a watchdog/safety backup if the Pi Zero 2 W crashes or hangs during boot?

---

*This document is a council review output. See `COUNCIL_PROCESS.md` for panel definitions and review protocol.*
