# Council Review: Data Logging Architecture & Stage Resequencing

**Date:** 2026-03-03
**Topic:** Stage 9 (Data Logging) scope, fidelity tiers, internal data format, resequencing before telemetry
**Reference:** SAD.md sections 8-9, IVP.md Stage 9 placeholder (IVP-63-67), prior telemetry council consensus
**Panel:** NASA/JPL Lead, Senior Aerospace Student, CubeSat Engineer (aux), Advanced Hobbyist Rocketeer (aux), Embedded Professor

---

## Background

The telemetry encoder (Stage 6/IVP-50 at time of review) needs a canonical data source to package for radio. The logging system produces that canonical data. Therefore logging must precede telemetry. Additionally, the project lead proposed 2-3 tiers of logging fidelity and noted that the current SAD assumes MAVLink .bin logging -- now outdated given the CCSDS pivot.

---

## Round 1

### Embedded Professor

Resequencing is a dependency violation fix. SAD pipeline: SensorTask to FusionTask to LoggerTask/TelemetryTask. Logger and encoder are siblings consuming FusedState, but encoder needs to know field layout defined by the logging data model. Clear data dependency.

Fidelity tiers map to rate + content selection (decimation model):

| Tier | Rate | Content | Storage | Use case |
|---|---|---|---|---|
| Economy | 1-2 Hz | Position + altitude + state | Flash (minimal wear) | HAB, long-duration, soak test |
| Standard | 50-100 Hz | Full TelemetryState | PSRAM then flash | Normal flight logging |
| Research | 200-500 Hz | FusedState + raw sensors | PSRAM only | Bench testing, anomaly investigation |

Key insight: not three systems but three decimation levels of the same pipeline. Fusion outputs at native rate, decimator selects tier. Mission Profile picks tier. PCM frame format serves all three.

### NASA/JPL Lead

Storage engineering on hardware tiers:

- **SRAM (520 KB):** Unlimited write speed/endurance, volatile. Active buffers and current FusedState.
- **PSRAM (8 MB QSPI):** 10-20 MB/s burst writes, volatile. In-flight ring buffer. Standard fidelity (100 Hz x 128B = 12.8 KB/s) lasts ~10 minutes.
- **Flash (6.5 MB):** 50-200 KB/s, nonvolatile, endurance-limited. Writes require multicore_lockout (~45 ms per 4 KB sector erase). Post-flight storage, not in-flight primary.

Architecture: PSRAM ring buffer in-flight, flash after landing. HAB: periodic condensed flush to flash since PSRAM is volatile.

MAVLink .bin format in SAD is dead. PCM fixed frames with decommutation table from Mission Profile.

SensorSnapshot must capture raw pre-calibration readings. Can reconstruct post-calibration from raw + coefficients but never reverse. Critical for post-flight recalibration.

### CubeSat Engineer

"High-speed camera snapshot" concept = **triggered burst recording.** High-rate PSRAM ring buffer runs continuously, data flagged for preservation when trigger fires. Triggers: state transition, anomaly detection, time-based, ground command. Buffer always rolling at max rate; trigger marks "don't overwrite." After landing, marked regions flushed alongside standard log.

PSRAM as two regions: standard ring buffer (full flight duration) + burst ring buffer (N seconds preserved). Implement standard first, burst as Titan/experimental. Partition decision needed now.

HAB periodic flush: pre-erase sectors during pre-launch idle, then periodic flushes only program (100 us per 256B page). Eliminates multicore lockout stutter.

### Advanced Hobbyist Rocketeer

The log file is the primary deliverable. Live telemetry is secondary. Every other flight computer gets this right.

Naming for users: **Economy** (intentional minimal), **Standard** (every flight, the default), **Research** (full dump, opt-in).

Post-landing priorities: 1) be findable (beeper), 2) don't corrupt log, 3) keep transmitting GPS.

### Senior Aerospace Student

Minimum viable: TelemetryState definition, PSRAM ring buffer at standard tier, PCM frame format, flash flush, USB download. Defer: triggered burst, LittleFS evaluation, SD card, export tools, overclocking.

---

## Round 2

### Embedded Professor

Three-struct hierarchy: FusedState (float32 ESKF-internal, written by FusionTask), TelemetryState (fixed-point wire-ready, converted from FusedState, consumed by logger + encoder), SensorSnapshot (raw pre-calibration, written by SensorTask, research-tier logger).

### NASA/JPL Lead

SensorSnapshot contents: raw accel/gyro/mag (int16x3 each), raw baro pressure/temp (int32), GPS fields, microsecond MET. All pre-calibration.

For PCM fixed frames, raw flash is simpler than LittleFS. Sequential stream needs no filesystem. Raw ring buffer on flash with flight log table in Tier 1 storage.

### CubeSat Engineer

PSRAM flush math: 2 MB = 512 sectors x 45 ms = ~23s of periodic Core 1 pauses. Fine post-landing. For HAB: pre-erased pool + program-only during flight.

### Hobbyist Rocketeer

User requirements: 1) USB download after flight, 2) covers entire flight + pre-launch, 3) opens in tool showing altitude/velocity/acceleration, 4) HAB survives power loss. Standard tier only initially.

---

## Round 3 -- Convergence

- **NASA/JPL Lead:** Consensus. Every frame needs MET timestamp in header. Non-negotiable.
- **Embedded Professor:** Consensus. Frame-type byte in header (Economy=0, Standard=1, Research=2). Zero cost.
- **CubeSat Engineer:** Consensus. Design PSRAM dual-region partition. PSRAM_BURST_RESERVE_KB defaulting to 0.
- **Hobbyist Rocketeer:** Consensus. Python CSV decoder sufficient for V1.
- **Senior Aerospace Student:** Consensus called.

---

## Consensus

### Resequencing
Approved. Data Logging moves before Radio & Telemetry. Encoder reads from data structures defined by logging architecture.

### Data Model
Three-struct hierarchy: FusedState (float32 ESKF-internal), TelemetryState (fixed-point wire-ready), SensorSnapshot (raw pre-calibration).

### Logging Tiers
- **Economy** -- position + altitude + state at 1-2 Hz. HAB/long-duration. Periodic flash flush. Design now, implement later.
- **Standard** -- full TelemetryState at 50 Hz default. PSRAM ring buffer, post-flight flash flush. Implement first.
- **Research** -- full state + raw sensors at max rate. Triggered burst. Design partition, implement as advanced/Titan.

### Storage
- In-flight: PSRAM ring buffer (volatile, fast)
- Post-flight: Raw flash sectors (sequential write, no filesystem)
- Flight log table in Tier 1 storage
- Pre-erase sectors during pre-launch for HAB periodic flush
- LittleFS deferred until file management needed

### Frame Format
PCM fixed frames: sync word, MET timestamp in header, frame-type byte, payload per Mission Profile decommutation table.

---

## Post-Council Clarifications

1. **Tiers are a slider, not buckets.** log_rate_hz parameter from 1-200 Hz. Economy/Standard/Research are preset labels on a continuous slider. Implementation is a decimation counter.

2. **Averaging decimation over simple decimation.** When decimating from 200 Hz to 50 Hz, average groups of 4 rather than keep every 4th. ~10 extra MAC ops per frame. Cleaner data by default.

3. **Raw sensor toggle independent of log rate.** log_raw_sensors boolean (or per-sensor bitmask for advanced config) is orthogonal to log_rate_hz. Research Mode gated.

4. **"Research Mode" umbrella concept.** Advanced/experimental features that may be unstable or risky: high-rate raw logging, I2C GPS, MCU overclocking, disabled safety interlocks, unlocked ESKF tuning, FSK continuous mode. Each individually toggleable, all gated behind Research Mode acknowledgment.

5. **RTC timestamps.** GPS (primary) or PCF8523 RTC on FeatherWing (secondary) for UTC epoch anchor. Fallback to boot-relative MET. Both recorded. Post-flight tools anchor MET to UTC.

6. **DLPF verification needed.** ICM-20948 DLPF and oversampling settings confirmed before logging stage. Pre-stage check.

7. **Stage structure preserved.** Don't split stages or create single-IVP stages. Pull Stage 9 forward as complete Stage 6. Renumber sequentially.

8. **Log is primary deliverable.** High-fidelity live telemetry is RocketChip's differentiator but secondary to logged data for most use cases.
