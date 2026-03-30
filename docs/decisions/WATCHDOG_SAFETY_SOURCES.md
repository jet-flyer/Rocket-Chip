# Watchdog Safety Architecture — Research Sources

**Date:** 2026-03-29
**Purpose:** Sources that informed the decision to replace the SDK hardware watchdog with a PIO-based safety architecture. Archived for traceability per council recommendation.

---

## Primary Sources

### Avionics & Flight Control
- **Dave Bodden, Lockheed Martin:** "Fault Tolerance, Fault Diagnostics, and Prognostics in Flight Control Systems" — Stanford EE392M lecture. JSF F-35 fault isolation philosophy: detect → isolate → reconfigure, never reboot mid-flight. Triplex redundancy with voting. URL: `web.stanford.edu/class/ee392m/Lecture8Bodden.pdf`
- **JSF AV C++ Coding Standards Rev C:** Rule 208 (no exceptions) — errors are data, not events requiring recovery. URL: `stroustrup.com/JSF-AV-rules.pdf`

### ArduPilot
- **ArduPilot watchdog.c:** `AP_HAL_ChibiOS/hwdef/common/watchdog.c` — persistent_data saved to STM32 RTC backup registers before reset. Always reboots on watchdog timeout.
- **ArduPilot EKF failsafe:** Degrades (lane switching, LAND mode) rather than rebooting for EKF failures. Separate from watchdog path.
- **In-flight watchdog crashes:** GitHub issues #14582, #11296, #31108 — real-world evidence that reboot-on-watchdog causes crashes in drones. Rockets cannot tolerate even the 1-2s boot gap that drones survive.

### NASA/JPL
- **Curiosity/Perseverance safe mode:** Dual redundant computers. Staged response: fault → safe mode (minimal systems, sun-pointing, radio) → ground diagnosis → uplinked recovery. Not applicable to simplex systems but philosophy (staged degradation) applies.
- **Ingenuity (Mars Helicopter) F' watchdog:** FPGA watchdog acted as a gate (prevented unsafe spin-up), not a reset mechanism. NASA F Prime framework.

### Hobby Rocket Flight Computers
- **Altus Metrum TeleMega/AltOS:** Forward-only state machine with timer-based backup deployment. No reboot path. Timer fires drogue at T+N regardless of software state. Industry standard dead man's switch.
- **Featherweight Blue Raven:** 2-of-3 sensor voting for apogee. TVAL1/TVAL2 timer backup per pyro channel. Multi-sensor redundancy within a single board.
- **PerfectFlite StratoLogger, Eggtimer:** Simple barometric state machines. So simple they essentially can't crash. No reboot mechanism.
- **Common pattern:** NO hobby rocket FC reboots mid-flight. Universal approach: simple forward-only state machine + timer backup.

### CubeSat & Automotive
- **Beningo CubeSat watchdog paper (Univ. Michigan):** 10 properties of a good watchdog. PIO meets most but fails Rule 10 (independent clock — PIO shares clk_sys).
- **Brazilian CubeSat OBC (arXiv 2412.17732):** Multi-layer watchdog: internal → supervisor MCU → EPS power cycle.
- **ISO 26262 (automotive ASIL-D):** Requires external watchdog IC with own oscillator for safety-critical systems. PIO is a "semi-independent" solution — better than software-only, weaker than external IC.

### RP2350 Hardware
- **RP2350 Datasheet Section 11.2 (PIO):** "From this point on, state machines are generally autonomous." PIO continues executing during ARM core hardfaults, deadlocks, and interrupt-disabled states.
- **RP2350 Datasheet Section 7.5 (WDSEL):** Selective watchdog reset domains — PIO can be excluded from watchdog reset, preserving backup timers across any software-triggered reset.
- **PIO clock dependency:** PIO runs from clk_sys (hardware-generated, unaffected by ARM software failures). Not truly clock-independent (fails CubeSat Rule 10) but sufficient for all RocketChip failure modes.

### Other
- **PX4 failsafe hierarchy:** Warning → Hold → RTL → Land → Terminate. Reboots on hardfault (same as ArduPilot) but failsafe system degrades gracefully for EKF/sensor failures.
- **ChibiOS hardfault handler:** Default behavior is halt-forever (`chSysHalt`). ArduPilot overrides to save state + let watchdog fire. Neither is appropriate for rockets.
