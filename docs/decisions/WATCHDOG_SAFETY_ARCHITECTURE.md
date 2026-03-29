# Watchdog & Flight Safety Architecture Decision

**Date:** 2026-03-29
**Status:** Council-reviewed, pending implementation
**Council:** JPL Avionics Lead, ArduPilot Core, Embedded Systems Prof, Hobbyist Rocketeer, Cubesat Engineer
**Verdict:** Unanimous — 8 amendments

---

## Problem

Current watchdog policy: always reboot on timeout, regardless of flight phase. During flight, this restarts the state machine in IDLE, kills PIO timers, loses all ESKF state, and prevents chute deployment. A recoverable software fault becomes a loss-of-vehicle event.

Research found: NO hobby rocket FC reboots mid-flight. ArduPilot's reboot approach works for drones (1-2s freefall is survivable) but is wrong for rockets (missed deployment window = lawn dart).

## Decision: Three-Layer Safety Architecture

### Layer 1: Smart Path (Normal Operation)
ESKF + state machine + action executor. Detects apogee via sensor fusion, deploys at optimal time. Can fail — that's what Layers 2 and 3 are for.

### Layer 2: Health Monitor (Fault Detection)
Watchdog (2s, fixed) + cross-core heartbeat. Detects Layer 1 failures. Response is **phase-dependent and profile-configurable:**

| Flight Phase | Default Action | Rationale |
|-------------|---------------|-----------|
| IDLE | Reboot | Safe on ground |
| ARMED | Degrade | Protects PIO timers from reboot-kill |
| BOOST | Degrade | PIO timer running, no action needed |
| COAST | Degrade | PIO timer running |
| DROGUE_DESCENT | Degrade | PIO main timer running |
| MAIN_DESCENT | Degrade | Both chutes out |
| LANDED | Reboot | Safe on ground |

"Degrade" = kick watchdog from minimal loop, continue logging. Do NOT make pyro decisions. PIO timer (Layer 3) handles deployment.

**Profile-configurable safe mode behavior:**
- `SAFE_MODE_ACTION=0` (default): Degrade only — kick WD, log, PIO timer handles deploy
- `SAFE_MODE_ACTION=1`: Safe mode + radio recovery — for HAB/passive platforms that can wait for ground commands (Mars rover pattern)
- `SAFE_MODE_ACTION=2`: Immediate deploy — fire drogue NOW (not recommended, legacy option)

### Layer 3: PIO Dead Man's Switch (Primary Safety Mechanism)
Two PIO state machines, loaded at ARM time:
- SM 0: Fire drogue at T + `drogue_timer_s`
- SM 1: Fire main at T + `main_timer_s`

Timer values from Mission Profile `.cfg`. Completely autonomous — runs on PIO hardware peripheral, independent of both ARM cores. Survives any software fault except MCU reset (which Layer 2 prevents by not rebooting during flight).

Cancelled on successful smart deployment (Layer 1). If Layer 1 crashes, timers fire on schedule.

**Relationship to Gemini discrete watchdog:** The PIO timer is the on-chip version of the discrete hardware fallback watchdog considered for the Gemini (dual-board redundancy) tier. Same concept, different implementation:
- Core/Middle: PIO timer (survives core crash, not MCU reset)
- Titan/Gemini: External timer IC (survives MCU reset, capacitor-backed survives power loss)

## Mission Profile Fields

```ini
# BACKUP DEPLOYMENT (PIO dead man's switch)
DROGUE_TIMER_S      15      # Fire drogue at T+15 if software hasn't
MAIN_TIMER_S        45      # Fire main at T+45 if software hasn't

# SAFE MODE BEHAVIOR
SAFE_MODE_ACTION    0       # 0=degrade, 1=safe+radio, 2=immediate deploy
```

Validation: `drogue_timer_s > estimated_burn_time`, `main_timer_s > drogue_timer_s + 5`.

## Watchdog Configuration
- Timeout: 2 seconds, fixed across all states
- Windowed pattern: Core 0 kicks HW WD only if Core 1 heartbeat flag is fresh
- Both cores must be healthy for kick to happen

## Hardware
- Pyro GPIO: diode-OR for dual-source firing (Layer 1 Action Executor + Layer 3 PIO)
- PIO program loaded into instruction memory at ARM time only
- Physical arm switch (if present) gates PIO loading

## Implementation IVPs (TBD)
- PIO backup timer implementation + test
- Phase-dependent watchdog policy refactor
- Cross-core heartbeat (windowed watchdog)
- Mission Profile schema update
- SPIN model update for degraded mode + PIO timer
- Pyro hardware schematic update

## Risk Residual
PIO timer does not survive MCU power loss. On battery-powered rockets, a battery disconnect kills all mechanisms. Mitigation: secure battery connection. Capacitor-backed pyro circuit is a Titan/Gemini tier feature. This residual risk matches every other hobby FC on the market.

## Sources
- Dave Bodden, Lockheed Martin: Fault Tolerance in Flight Control Systems (Stanford EE392M)
- ArduPilot watchdog.c, AP_InternalError, EKF failsafe documentation
- NASA JPL: Curiosity/Perseverance safe mode operations
- Altus Metrum AltOS backup deployment timer
- Featherweight Blue Raven multi-sensor voting + timer backup
- JSF AV C++ Rules 208 (no exceptions), fault handling philosophy
- DO-178C / ARP 4754A fault tolerance guidance
