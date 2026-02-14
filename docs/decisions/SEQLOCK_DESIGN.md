# Seqlock Cross-Core Data Sharing Design

**Status:** Approved — council review unanimous (2026-02-06)
**Date:** 2026-02-06
**Applies to:** IVP-24 (Seqlock Single-Buffer) and all downstream cross-core data sharing
**Research basis:** Four parallel research agents covering struct layout, prior art (ArduPilot/Betaflight/PX4), RP2350 memory model, and notification mechanisms
**Council:** ArduPilot Core Contributor, Retired NASA/JPL Avionics Lead, Embedded Systems Professor — unanimous APPROVE with 7 required modifications (all incorporated below)

---

## 1. Decision Summary

| Choice | Decision | Alternative Considered |
|--------|----------|----------------------|
| Synchronization | **Seqlock** (lock-free reader) | Pico SDK spinlock (simpler but disables Core 0 IRQs ~100ns) |
| Notification | **Poll seqlock sequence counter** | Doorbell IRQ (saves ~5ms latency Core 0 can't use at 200Hz) |
| Struct layout | **Single combined struct** (~124 bytes) | Separate per-sensor seqlocks (unnecessary complexity) |
| Buffer count | **Single buffer** with sequence counter | Double-buffer (no safety benefit for single-writer seqlock) |
| Calibration | **Applied on Core 1** (writer side) | Core 0 applies (would duplicate work in 5+ consumers) |
| Memory location | **Static SRAM** | PSRAM (unsafe — XIP cache coherency + exclusive monitor issues) |
| GPS coordinates | **`int32_t * 1e7`** | `double` (requires software FPU emulation on Cortex-M33) |

---

## 2. Shared Data Struct

All values are calibration-applied, in body frame, SI units. Updated at IMU rate (~1kHz). Baro/GPS fields update at their respective rates; check timestamps and `read_count` for freshness.

```c
#include <stdint.h>
#include <stdbool.h>

/**
 * Shared sensor data published by Core 1 via seqlock.
 *
 * Size: 124 bytes. Copy time: ~0.5us at 150MHz.
 * Collision probability: 0.08% at 1kHz write / 200Hz read.
 */
typedef struct {
    // --- IMU (updated every Core 1 cycle, ~1kHz) ---
    float accel_x;              // m/s^2, calibrated, body frame
    float accel_y;
    float accel_z;
    float gyro_x;               // rad/s, calibrated, body frame
    float gyro_y;
    float gyro_z;
    float mag_x;                // uT (when mag_valid)
    float mag_y;
    float mag_z;
    float imu_temperature_c;    // ICM-20948 die temperature
    uint32_t imu_timestamp_us;  // time_us_32() at read
    uint32_t imu_read_count;    // Monotonic, for stale detection
    uint32_t mag_read_count;    // Monotonic, increments only on new mag data
    bool accel_valid;           // false if I2C read failed
    bool gyro_valid;
    bool mag_valid;             // false if no new mag data
    uint8_t _pad_imu;
    // 56 bytes

    // --- Barometer (updated ~50Hz) ---
    float pressure_pa;          // Pascals (raw, no altitude conversion)
    float baro_temperature_c;
    uint32_t baro_timestamp_us;
    uint32_t baro_read_count;
    bool baro_valid;
    uint8_t _pad_baro[3];
    // 20 bytes

    // --- GPS (implemented IVP-31) ---
    int32_t gps_lat_1e7;        // Latitude * 1e7 (ArduPilot convention)
    int32_t gps_lon_1e7;        // Longitude * 1e7
    float gps_alt_msl_m;        // Altitude MSL in meters
    float gps_ground_speed_mps;
    float gps_course_deg;       // Course over ground
    uint32_t gps_timestamp_us;
    uint32_t gps_read_count;
    uint8_t gps_fix_type;       // 0=none, 2=2D, 3=3D
    uint8_t gps_satellites;
    bool gps_valid;
    uint8_t _pad_gps;
    // 32 bytes

    // --- Health (cumulative, never reset during flight) ---
    uint32_t imu_error_count;
    uint32_t baro_error_count;
    uint32_t gps_error_count;
    uint32_t core1_loop_count;  // Monotonic, Core 1 main loop iterations
    // 16 bytes

} shared_sensor_data_t;
// Total: 124 bytes

_Static_assert(sizeof(shared_sensor_data_t) == 124,
               "shared_sensor_data_t size changed — update seqlock copy budget");
_Static_assert(sizeof(shared_sensor_data_t) % 4 == 0,
               "shared_sensor_data_t must be 4-byte aligned for memcpy correctness");
```

### Field Rationale

| Field | Why |
|-------|-----|
| Per-sensor timestamps | Sensors update at 1kHz/50Hz/10Hz — ESKF needs accurate per-sensor dt |
| Per-sensor `read_count` | Stale detection without Core 0 needing to know expected rates |
| `mag_read_count` | AK09916 runs at 100Hz internally; ~9/10 IMU cycles have stale mag data. ESKF needs to distinguish new vs repeated mag readings. `mag_valid` alone is boolean — `mag_read_count` provides monotonic counter that only increments on genuinely new mag data |
| `core1_loop_count` | Watchdog/health monitoring: Core 0 can detect Core 1 stalls by checking if this counter advances. Also useful for CLI diagnostics and telemetry |
| Per-sensor validity flags | I2C reads can fail; mag only updates at 100Hz internally |
| Error counters | CLI status, telemetry health, watchdog assessment |
| IMU temperature | Bias drift compensation; free from existing burst read |
| No `altitude_m` in baro | Requires sea-level reference from Core 0 calibration state |
| No separate mag timestamp | Mag arrives in same I2C burst as accel/gyro; `mag_read_count` tracks new-data freshness |
| GPS as `int32_t * 1e7` | Avoids soft-float double on Cortex-M33; ArduPilot convention; ~1.1cm precision |
| GPS fields reserved now | Avoids struct layout change at IVP-31 |
| Explicit padding | Prevents compiler-inserted padding from varying between TUs |
| `_Static_assert` on size | Catches accidental struct changes at compile time. No runtime version field needed — both cores compile from same source |
| `_Static_assert` on alignment | Ensures 4-byte alignment for correct `memcpy` behavior on ARM |

---

## 3. Seqlock Wrapper

Single-buffer seqlock. The sequence counter detects torn reads — double-buffering is unnecessary for a single-writer pattern and adds complexity.

```c
#include <stdatomic.h>
#include "hardware/sync.h"  // __dmb()

typedef struct {
    _Atomic uint32_t sequence;      // Odd = write in progress
    shared_sensor_data_t data;
} sensor_seqlock_t;

// File scope, static allocation (per CODING_STANDARDS.md, LL Entry 1)
static sensor_seqlock_t g_sensor_seqlock;
```

### Writer (Core 1)

```c
void seqlock_write(const shared_sensor_data_t *new_data) {
    uint32_t seq = atomic_load_explicit(&g_sensor_seqlock.sequence,
                                         memory_order_relaxed);
    // Signal write-in-progress (odd)
    atomic_store_explicit(&g_sensor_seqlock.sequence, seq + 1,
                          memory_order_release);
    // DMB: release on STL only orders prior stores relative to the
    // counter store itself. The subsequent non-atomic memcpy is NOT
    // ordered by STL — DMB ensures the odd counter is visible to the
    // reader before any data bytes are written.
    __dmb();

    g_sensor_seqlock.data = *new_data;

    // DMB: ensures all data stores from the memcpy complete before
    // the even counter store below. Without this, the CPU write buffer
    // could reorder data stores past the counter, and the reader would
    // see the "write complete" signal before data is actually committed.
    __dmb();
    // Signal write-complete (even)
    atomic_store_explicit(&g_sensor_seqlock.sequence, seq + 2,
                          memory_order_release);
}
```

### Reader (Core 0)

```c
/**
 * Read a consistent snapshot from the seqlock.
 * Returns true on success, false if no consistent read after max retries.
 * At 1kHz write / 200Hz read, collision probability is 0.08% per attempt.
 * 4 retries gives >99.9999% success probability.
 */
bool seqlock_read(shared_sensor_data_t *out) {
    for (uint32_t attempt = 0; attempt < 4; attempt++) {
        uint32_t seq1 = atomic_load_explicit(&g_sensor_seqlock.sequence,
                                              memory_order_acquire);
        if (seq1 & 1u) continue;  // Write in progress, retry

        // DMB: acquire on LDA orders subsequent loads after the counter
        // load, but the non-atomic memcpy may be speculatively reordered
        // by the CPU ahead of the counter read. DMB ensures the counter
        // value is committed before any data bytes are read.
        __dmb();
        *out = g_sensor_seqlock.data;
        // DMB: ensures all data loads from the memcpy complete before
        // re-reading the counter. Without this, the CPU could reorder
        // the second counter load ahead of late data loads, giving a
        // false "consistent" result with partially stale data.
        __dmb();

        uint32_t seq2 = atomic_load_explicit(&g_sensor_seqlock.sequence,
                                              memory_order_acquire);
        if (seq1 == seq2) return true;  // Consistent snapshot
    }
    return false;  // All retries collided — caller uses previous data
}
```

### Why Single Buffer (Not Double)

The SAD (Section 4.3) shows a double-buffer seqlock with `buffers[2]` and `((seq+1)>>1)&1` indexing. This is unnecessary:

- **Seqlock already detects torn reads** via the sequence counter. Double-buffering adds a second layer of protection that provides no additional safety for a single-writer pattern.
- **Single buffer halves memory** (~128 bytes vs ~252 bytes).
- **Simpler indexing** — no `((seq+1)>>1)&1` arithmetic.
- **Standard pattern** — the Linux kernel seqlock uses a single data field.

Double-buffering would matter if the reader needed to always succeed without retry (e.g., hard real-time with no retry budget). Our reader at 200Hz has ample retry budget (collision probability 0.08%).

---

## 4. Memory Barrier Requirements

| Location | Barrier | Why |
|----------|---------|-----|
| Writer: after odd counter store | `__dmb()` | Counter must be visible to reader before data writes begin |
| Writer: after data write | `__dmb()` | All data stores must complete before even counter is visible |
| Reader: after counter load | `__dmb()` | Counter must be read before data copy begins |
| Reader: after data copy | `__dmb()` | Data copy must complete before re-reading counter |
| Sequence counter ops | `memory_order_acquire` / `memory_order_release` | Emit ARM LDA/STL instructions for the atomic itself |

### What NOT to Use

| Wrong Approach | Why |
|---------------|-----|
| `__compiler_memory_barrier()` | Compiler-only fence — no hardware barrier emitted. CPU write buffer can reorder. |
| Plain `volatile` | Prevents compiler optimizations but no DMB — insufficient for cross-core. |
| `_Atomic uint64_t` for timestamps | Falls back to spinlock #13 on RP2350 (disables IRQs). Use `uint32_t`. |
| PSRAM for shared data | XIP cache coherency issues + global exclusive monitor doesn't cover PSRAM. |
| Hardware spinlock registers directly | Broken by RP2350-E2 (SIO register aliasing). Use SDK APIs. |

---

## 5. Memory Placement

**The seqlock MUST live in static SRAM.** Three reasons:

1. **No data cache.** RP2350 SRAM is accessed directly through the bus fabric — no cache coherency issues between cores.
2. **Global exclusive monitor covers SRAM.** The SDK sets `ACTLR.EXTEXCLALL` on both cores at startup, routing all exclusive accesses through the global monitor. C11 atomics work correctly cross-core on SRAM.
3. **Deterministic latency.** Single-cycle access at 150MHz (~6.7ns). No cache miss penalty.

**PSRAM is NOT safe for cross-core shared data:**
- XIP write-back cache can hold dirty data not yet visible to the other core
- Global exclusive monitor does not cover PSRAM addresses — `LDREX`/`STREX` use local (per-core) monitor only, silently failing to detect cross-core writes
- Variable latency: 6.7ns cache hit to 100-500ns+ cache miss

---

## 6. Calibration Boundary

**Core 1 applies calibration before publishing.** This matches all three reference projects (ArduPilot, Betaflight, PX4).

### How It Works

1. Core 1 holds a local copy of `calibration_store_t` (~180 bytes)
2. After I2C read, Core 1 calls `calibration_apply_accel()` and `calibration_apply_gyro()` with raw data and its local calibration copy
3. Calibrated values are written to the seqlock struct
4. When user runs calibration (via CLI on Core 0), Core 0 sets `cal_reload_pending` flag
5. Core 1 checks the flag each loop iteration and reloads calibration params when set

### Calibration Reload Signaling

```c
// Shared flag — Core 0 sets, Core 1 clears after reload
static _Atomic bool g_cal_reload_pending = false;

// Core 0 (after calibration completes):
atomic_store_explicit(&g_cal_reload_pending, true, memory_order_release);

// Core 1 (top of main loop):
if (atomic_load_explicit(&g_cal_reload_pending, memory_order_acquire)) {
    calibration_load(&core1_local_cal);  // Reload from flash/RAM
    atomic_store_explicit(&g_cal_reload_pending, false, memory_order_release);
}
```

Uses `_Atomic bool` (1 byte, lock-free on Cortex-M33) instead of FIFO (reserved) or doorbell (overkill for a flag).

### Calibration Function Refactoring (Council Decision)

**Decision: Refactor to accept pointer parameter (option a).** Re-run IVP-15/16/17 calibration gates after refactoring.

The current `calibration_apply_accel()` and `calibration_apply_gyro()` read from file-scope global `g_calibration` in `calibration_manager.c`. For Core 1 to use its own copy, refactor both functions to accept `const calibration_store_t*`:

```c
// Before (reads global):
void calibration_apply_accel(float *x, float *y, float *z);

// After (accepts pointer):
void calibration_apply_accel(const calibration_store_t *cal,
                             float *x, float *y, float *z);
```

This is a mechanical change (2 function signatures + call sites). Council unanimously agreed this is the correct approach — re-running the calibration gates is the right cost to pay for thread safety.

---

## 7. Notification Mechanism

**Core 0 polls the seqlock sequence counter.** No doorbell, no FIFO.

| Approach | Verdict |
|----------|---------|
| **Seqlock polling** | 5-7 cycles per check (0.003% of 200Hz loop budget). Zero jitter. Zero conflicts. |
| Doorbell IRQ | Works but saves ~5ms latency the 200Hz fusion loop can't use. Adds ISR complexity. |
| FIFO notification | **Unsuitable.** 4-deep on RP2350, AND reserved by `multicore_lockout`/`flash_safe_execute`. |

Doorbells become relevant if RocketChip moves to a sleep-based power architecture (WFI/WFE on Core 0 between ticks).

---

## 8. Prior Art Comparison

| Aspect | ArduPilot | Betaflight | PX4 |
|--------|-----------|------------|-----|
| Mechanism | Recursive mutex | None (single-core cooperative) | Interrupt-disable + generation counter |
| Calibration | Before shared boundary | Before shared boundary | Before shared boundary |
| Timestamps | Per-batch (accumulated dt) | Implicit (scheduler timing) | Per-sample `uint64_t` |
| Lock-free? | No | N/A | Pseudo (interrupt-disable is bounded) |
| Multi-core safe? | Yes | No | Yes |

**Key takeaway:** All three apply calibration on the writer side. PX4's generation counter is closest to our seqlock.

---

## 9. RP2350 Platform Constraints

### RP2350-E2: SIO Register Aliasing

SIO register writes above offset `+0x180` alias hardware spinlock registers, causing spurious lock releases. The SDK defaults to `PICO_USE_SW_SPIN_LOCKS=1` on RP2350, using `LDAEXB`/`STREXB` (exclusive access instructions) instead of SIO registers. This is transparent to all SDK APIs.

**Note:** Previous documentation referenced "E17" — this errata ID was not found anywhere in the Pico SDK source. E2 is the confirmed errata affecting spinlocks.

### FIFO Reservation

The RP2350 multicore FIFO is 4-deep (halved from RP2040's 8). `multicore_lockout_victim_init()` (called by `flash_safe_execute`) registers an exclusive handler on the FIFO IRQ. The FIFO cannot be used for application messaging if flash operations are needed (IVP-28).

### Atomics

| Size | Implementation | Lock-free? |
|------|---------------|------------|
| 1/2/4 bytes | Native `LDREX`/`STREX` | Yes |
| 8 bytes | Spinlock #13 fallback | No (disables IRQs) |

Use `_Atomic uint32_t` for the sequence counter. Avoid `_Atomic uint64_t`.

---

## 10. Council Review — Resolved Questions

Questions originally posed to the council, now resolved with unanimous answers.

1. **Calibration function refactoring:** Yes — refactor `calibration_apply_accel/gyro()` to accept `const calibration_store_t*`. **Re-run IVP-15/16/17 calibration gates** after refactoring. The change is mechanical but the gates must be re-verified. See Section 6.

2. **Separate `mag_read_count`:** Yes — added `uint32_t mag_read_count` to the IMU section. Increments only when AK09916 reports new data (DRDY bit set). The ESKF must distinguish "new mag measurement" from "repeated stale mag data" to avoid biasing the filter. `mag_valid` alone (boolean) loses the monotonic count needed for proper measurement gating.

3. **Struct versioning:** No runtime version field. `_Static_assert(sizeof(...) == 124)` is sufficient — both cores compile from the same source, so layout changes are caught at compile time. A runtime version field is over-engineering for a single-binary system.

---

## 11. Council Required Modifications (All Incorporated)

The council unanimously approved the design with 7 required modifications. All have been incorporated into the sections above.

| # | Modification | Section |
|---|-------------|---------|
| 1 | **Bounded retry loop** — reader must not spin unbounded. Max 4 attempts, return false on failure. Caller uses previous data. | Section 3 (Reader) |
| 2 | **Add `core1_loop_count`** — monotonic counter for Core 1 main loop iterations. Enables watchdog/stall detection from Core 0. | Section 2 (Health) |
| 3 | **Add `mag_read_count`** — monotonic counter that increments only on new mag data (DRDY). ESKF needs this to gate mag updates. | Section 2 (IMU) |
| 4 | **`_Static_assert` for struct size** — catches accidental layout changes at compile time. | Section 2 (after struct) |
| 5 | **`_Static_assert` for 4-byte alignment** — ensures correct `memcpy` behavior on ARM. | Section 2 (after struct) |
| 6 | **Comment every `__dmb()` with rationale** — explain why `memory_order_release/acquire` on the atomic alone is insufficient for the non-atomic memcpy. | Section 3 (Writer/Reader) |
| 7 | **Use `_Atomic bool cal_reload_pending`** for calibration reload signaling. Lock-free, avoids FIFO (reserved) and doorbell (overkill). | Section 6 |

---

## 12. SAD Code Audit

The SAD (Section 4.3) contains a seqlock code example that predates this research. Issues found:

| Issue | SAD Code | Research Finding |
|-------|----------|-----------------|
| Missing `__dmb()` between counter and data | Relies solely on `memory_order_release` | Need explicit `__dmb()` — release on the counter doesn't fence the non-atomic struct copy |
| Unnecessary double-buffer | `SensorData buffers[2]` | Single buffer sufficient — sequence counter detects torn reads |
| "E17" errata reference | "RP2350 errata E17 may force SW spinlocks" | Should be E2 |
| `Vector3f` type | Conceptual, doesn't exist in codebase | Use flat `float` fields |
| No per-sensor timestamps | Not shown in struct | Required for multi-rate ESKF |

The SAD is a protected file. Corrections deferred to a future session.

---

*Decision document for `docs/decisions/`. See IVP-24 for verification gates. Council review transcript available on request.*
