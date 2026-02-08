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

*This document is a council review output. See `COUNCIL_PROCESS.md` for panel definitions and review protocol.*
