# IVP-132a.4 Re-Evaluated Plan — Station Soak (Post-Council)

**Status:** Council approved 2026-04-16 with required additions (all incorporated below).
**Date:** 2026-04-16
**Council:** NASA/JPL Avionics Lead, Embedded Systems Professor, ArduPilot Core Contributor
**Verdict:** Unanimous conditional approval, no dissents. Additions listed in §"Council Additions" are non-negotiable prerequisites to re-running the soak.
**Context:** Original IVP-132a.4 claimed GATE PASS but was invalid — station
binary was compiled with the wrong `PICO_BOARD` (Feather instead of Fruit
Jam), radio was never initialized, the soak validated Core 0 stability but
not the station's actual RX path. Commit reverted in 97ca488.

## Root Causes

1. **Build configuration was not verified before flashing.** I created
   `build_station_flight/` with `cmake -DROCKETCHIP_JOB_STATION=1
   -DBUILD_FOR_FLIGHT=ON ..`, missing `-DPICO_BOARD=adafruit_fruit_jam`.
   Defaulted to Feather. The binary linked successfully because all pins
   are just uint8_t constants; there's nothing at link time that says
   "Feather pins are wrong for Fruit Jam."

2. **Soak test watched derivative signals, not primary signals.** The
   script checked `AO_Radio->eQueue.nMin` (queue watermark) as a health
   metric. Watermark stable at 32/33 *looks* identical whether:
   - (a) radio is healthy, nothing transmitting (environmental)
   - (b) radio silently failed to init (catastrophic)
   No check of `AO_Radio->state.initialized` — the direct truth.

3. **No precondition check at T=0.** Script started sampling at T=0 and
   used "no crashes" + "queue stable" as pass criteria. Never asked "is
   the peripheral I'm supposed to be validating actually live?"

4. **Post-hoc explanation instead of hypothesis testing.** When I saw
   `rx_count=0` at every snapshot, I explained it away as "vehicle
   probably wasn't transmitting" rather than asking "is rx_count=0
   consistent with the radio working at all?"

## Re-Designed Soak

### Required preconditions at T=0 (block 1, must all pass)

Hard-fail the soak if any of these read false at T=0. No proceeding.

**Build identity (NASA/JPL #1, #3):**
- `kBuildTag` string
- `kBoardName` string (new firmware constant — catches Frankenstein class)
- `kGitHash` string (already exists from IVP-127b)
- `kBuildForFlight` bool (new constant — distinguishes bench vs flight tier)
- `kJobRole` string ("vehicle" / "station" / "relay") (new — distinguishes role)

**Peripheral init flags (station):**
- `AO_Radio->state.initialized == true`
- `AO_Radio->state.radio.initialized == true`
- `AO_Radio->state.scheduler.phase == kRxContinuous` (for station role)
- `AO_Radio->state.radio.cs_pin`, `rst_pin`, `irq_pin` match Fruit Jam
  constants (10, 6, 5)
- `g_spiOk == true`
- `g_neopixelInitialized == true`
- `g_bestGpsValid` — permitted false at T=0 if indoors, but log GPS init
  flag separately

**Active register readback (NASA/JPL #4 — the truth layer):**
- New firmware helper `rfm95w_read_version()` exposed for GDB `call`.
  Returns the SX1276 RegVersion (0x42). Must equal `0x12` for SX1276.
  This is the single check that would have caught the original
  Frankenstein bug directly — if the wrong pins are wired, SPI reads
  garbage and this fails.

**Passive IRQ wiring evidence (NASA/JPL revised verdict 2026-04-16):**
The active DIO0 forced-edge test is NOT in this soak — JPL council
requires it as a separate bring-up IVP-132a.4a (split verdict:
"don't bundle coverage gaps into soak tests"). For this soak, strengthen
passive coverage by logging per growth window:
- `gpio_get(kRadioIrqPin)` — raw pin state
- `NVIC->ISPR[...]` — interrupt pending register for the radio IRQ line
These catch "IRQ disabled in NVIC" and "pin stuck low" without forcing
edges. Logged as raw values in the precondition blob.

### Liveness growth checks (block 2, must hold across intervals)

At each snapshot after T=0, verify something that's supposed to grow has
actually grown:

- Uptime (time_us_32 or equivalent) — catches frozen target
- Per-AO queue-activity check (Professor #1, script-only, zero firmware
  cost): at each snapshot record `eQueue.head` and `eQueue.tail` for
  every AO. Between snapshots, if *both* indices are unchanged for an AO,
  that AO's queue had no activity in the window. Full-freeze = all AOs
  flat. Single-AO starvation = one AO flat with a non-empty queue
  (`nUsed > 0` sustained) while others show activity. This catches the
  partial-liveness case Professor flagged without needing to modify
  vendored QP/C. A proper per-AO counter is deferred to the next pass
  if the ring-index proxy turns out to be insufficient.
- If vehicle is transmitting during soak (ArduPilot #1, tightened): 
  `AO_Radio->state.rx_count > 1000` by T=5min. At 5Hz vehicle TX, that's
  1500 packets expected × 0.67 margin for indoor loss. `rx_count > 0` is
  too lax — passes on a single ACK'd packet.

Flat metrics that should stay stable (still watched):
- MSP (stack usage)
- AO queue watermarks (nMin): tightened threshold (ArduPilot #3).
  `nMin / depth >= 0.75` throughout the 30 min. Drop below 75% free =
  leak or back-pressure, investigate.

**New counter to add (ArduPilot #4):** `g_spi_error_count` in `spi_bus.cpp`
hot path. Growth expected to be 0 indoors. Growth > 10/30min during
indoor soak = EMI or wiring issue.

### Exit-gate criteria (block 3)

All of the following for a pass:
- All T=0 preconditions met (including RegVersion==0x12)
- All per-AO `events_handled` counters grew > 0 across each snapshot interval
- No crashes, no watchdog resets
- MSP never drops more than 256 bytes from T=0 value (stable stack)
- AO queue `nMin / depth >= 0.75` throughout (ArduPilot #3)
- `g_spi_error_count` growth < 10 over 30min indoor (ArduPilot #4)
- **If Variant B (vehicle TX active):**
  - `rx_count > 1000` by T=5min (ArduPilot #1)
  - Rolling-window CRC error ratio in any 5-min window < 10% (ArduPilot #2).
    Cumulative ratio hides late bursts against clean early packets.
  - `rx_count` and `rx_crc_errors` semantics documented:
    - `rx_attempts = rx_count` (every RX with bytes)
    - `rx_valid = rx_count - rx_crc_errors`
    - Fail case "wrong SF / wrong sync word" manifests as rx_count growing
      with rx_crc_errors growing 1:1 — rx_valid stays flat (ArduPilot #5)

### What the soak does NOT claim to validate

- Real radio link quality (dBm, SNR trends over time) — that's a separate
  link characterization test, not a soak
- Post-decode correctness (CCSDS / MAVLink content) — that's IVP-132a.3
  replay harness territory
- Round-trip command ACKing — that's IVP-132a.5
- **Active DIO0 IRQ wiring** — split to IVP-132a.4a per JPL council
  ("don't bundle coverage gaps into soak tests"). Soak has passive
  evidence (NVIC state + pin readback) but not a forced-edge test.
- **DMA channel ownership** (NASA/JPL add) — if two subsystems claim
  the same DMA channel, first one wins and the other silently fails.
  Soak doesn't currently enumerate claims.
- **flash_safe_execute vs radio contention** (NASA/JPL add, ref LL 4/12/31)
  — radio SPI transaction during flash op would wedge. Soak doesn't
  exercise flash writes mid-run.
- **Soak duration sufficiency** (Professor add) — 30-min is the default;
  see §"Duration rationale" below for why, and where it falls short.

### Test variants

**Variant A — Station idle stability** (vehicle off, no TX): validates
station alone doesn't crash/leak/stall over 30 min with radio in
kRxContinuous listening at idle. Precondition: radio initialized. No
growth check for rx_count.

**Variant B — Station integration** (vehicle transmitting nominal rate):
validates station processes inbound telemetry without queue build-up or
decoder failure. Preconditions same as A plus: rx_count grows > 0 by
T=5min.

**Both variants run in this IVP.** Variant A first (simpler, catches
station-intrinsic issues), then Variant B (catches station-under-load
issues).

## Council Additions — Required Before Re-Running

### Firmware changes (small, specific)

1. **`kBoardName` string** (NASA/JPL #1). In `include/rocketchip/board.h`
   or equivalent, add `extern const char* const kBoardName` =
   `"adafruit_fruit_jam"` or `"adafruit_feather_rp2350"`. Readable by
   GDB. Boot banner prints it.

2. **`kBuildForFlight` bool + `kJobRole` string** (NASA/JPL #2). Two
   more constants for full build/role identity at T=0.

3. **`rfm95w_read_version()` public helper** (NASA/JPL #4). Exposes
   a GDB-callable getter for RegVersion that a soak script can invoke
   at T=0 to confirm the radio chip is physically reachable and
   responding with 0x12 (SX1276).

4. ~~Per-AO `events_handled` counter~~ — Replaced by a script-side
   check that reads `eQueue.head` + `eQueue.tail` per AO at each
   snapshot and detects AOs with unchanged ring indices while their
   queue shows `nUsed > 0` (starvation). Zero firmware cost. Proper
   counter deferred if this proxy proves insufficient.

5. **`g_spi_error_count`** (ArduPilot #4). Atomic counter in
   `spi_bus.cpp`. Increment when SPI HW transaction indicates error
   (timeout, NACK, etc). Hot path cost: one atomic add per failed
   transaction.

6. **Extend `diag_stats_dump()`** to include all the above. Single
   GDB `call diag_stats_dump()` replaces a wall of `print` commands.

### Script changes (procedural)

7. **Precondition blob** (Professor #2). Soak script prologue emits
   JSON lines with all init flags, register readbacks, build identity,
   pin assignments. SHA256 hash of blob included in commit. Allows
   future audits to verify the soak ran under the conditions claimed.

8. **Two variants explicit.** Variant A (station idle, vehicle off)
   and Variant B (station integration, vehicle transmitting). Run both.

9. **Threshold tightening** per ArduPilot #1–3:
   - `rx_count > 1000` instead of `> 0` for Variant B
   - `nMin/depth >= 0.75` instead of 50%
   - Rolling 5-min CRC ratio, not cumulative

### Out of scope (split to separate IVPs)

- **IVP-132a.4a (new)**: Active DIO0 IRQ bring-up test per JPL split
  verdict. Forced-edge test with proper teardown. Runs once at bring-up,
  not during soak. Destructive-to-radio-state (OK pre-arm), catches
  init-true + IRQ-dead class.
- **Stage 16C (planned)**: build-system guard forcing correct
  `PICO_BOARD` for station role. Makes the Frankenstein class impossible
  instead of catchable.

## Required Script Changes

Soak GDB script prologue (new, mandatory):
1. Halt, print build/board/role/init-flag block
2. Validate each flag with GDB `if` + explicit pass/fail log line
3. Only enter the timed-snapshot loop if all preconditions pass
4. If any precondition fails, quit with a clear "PRECONDITIONS FAILED" log
   line — do NOT run the timed phase

## Duration Rationale (Professor add)

30 min default is justified by:
- **Thermal equilibrium window**: RP2350 reaches steady-state temp in
  ~15 min of active work on battery per IVP-132 Phase 4 observations.
  A 30-min window covers warmup + 15 min of equilibrium behavior where
  thermal-sensitive issues would manifest (crystal drift, regulator
  sag under load).
- **Queue-creep observable**: AO queues that leak at 1 event/min are
  visible in a 30-min window (would show nMin drop of 30 out of 33).
  Leaks slower than that need longer soaks.
- **Stack drift observable**: any recursive stack growth at <200
  bytes/min is visible in MSP watermark.

**Where 30 min falls short:**
- Slow PSRAM memory-corruption (flight logger territory, not this soak)
- Daily thermal cycling (air-conditioning on/off, etc.) — multi-hour
- Accumulated RF desense over hours — not a firmware issue, RF test

If any metric hovers near its threshold at T=30min (e.g., nMin dropping
toward 75% or g_spi_error_count trending up), extend the soak to 2 hrs
before gating. Document the decision in the commit.

## Risks / Open Questions

- **Indoor vs outdoor GPS precondition.** Station GPS won't lock indoors.
  Should be warning, not fail. Need a mode flag or document explicitly.
- **Vehicle TX timing for Variant B.** Vehicle emits at 5Hz by default
  (post-IVP-132a.0). First packet should arrive within ~200ms of both
  boards being up. If no packet in 10s with known-good binary, something's
  wrong — document as pre-soak check, not soak failure.
- **QV dispatch hook for `events_handled`** — exact implementation point.
  Easiest spot is in `QV_onIdle` or before `QACTIVE_dispatch_` — need to
  confirm QP framework allows a lightweight hook without breaking timing.

## Not in Scope

- Real distance/RSSI characterization (needs outdoor + known baseline)
- Fault-injection combined with soak (separate IVP)
- Long-duration thermal drift capture — depends on Stage 16C MCU temp path
- Stage 16C build-system board guard (makes the Frankenstein class
  impossible) — documented as separate work, not a prerequisite for
  IVP-132a.4 redo

## Acceptance

Gate passes when:
1. **Variant A** runs 30 min with all T=0 preconditions met (including
   RegVersion=0x12, NVIC ISPR sanity, pin readback matches board), all
   per-AO `events_handled` counters grew, zero crashes/resets, MSP
   within 256 bytes of T=0, all AO queue nMin/depth >= 0.75,
   `g_spi_error_count` < 10.
2. **Variant B** runs 30 min with vehicle transmitting, all of (1) plus
   `rx_count > 1000` by T=5min, rolling 5-min CRC ratio < 10%,
   rx_valid vs rx_attempts split reported.
3. **JSON precondition blob** hashed; hash referenced in commit message.
4. Both results committed with the raw preconditions log, not just
   summary metrics.
