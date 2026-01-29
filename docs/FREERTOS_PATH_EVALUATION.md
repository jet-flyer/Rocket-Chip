# FreeRTOS Path Evaluation & Reset Plan

**Date:** 2026-01-28
**Purpose:** Complete evaluation of what to keep vs. rewrite for std::atomic fix path
**Critical:** Includes checkpoints to detect circular progress

---

## Current State Summary

### Branch Status
- **main branch**: ChibiOS pivot (Phase 0 - LED/UART working, USB/I2C blocked)
- **freertos branch**: Advanced ArduPilot integration (paused at PD12/PD13 issues)

### The Core Problem (PD12)
ArduPilot's `AP_InertialSensor` reads `_new_gyro_data`/`_new_accel_data` flags **without synchronization**:
- Flags are plain `bool` (not volatile, not atomic)
- Writes happen inside semaphore (protected)
- Reads happen outside semaphore (unprotected) → **race condition on dual-core**

### Chosen Fix: Option B (std::atomic)
~30 lines of changes when bringing AP_InertialSensor into codebase.

---

## KEEP vs. REWRITE Decision Matrix

### ✅ KEEP - Working and Valuable

| Component | Location | Reason to Keep |
|-----------|----------|----------------|
| **Scheduler** | `AP_HAL_RP2350/Scheduler.cpp` | Complete, has memory barriers, FreeRTOS integration solid |
| **Storage** | `AP_HAL_RP2350/Storage.cpp` | Working flash ops, XIP-aware, wear-leveling via AP_FlashStorage |
| **Semaphores** | `AP_HAL_RP2350/Semaphores.cpp` | Clean FreeRTOS mutex wrappers |
| **GPIO** | `AP_HAL_RP2350/GPIO.cpp` | Basic I/O working |
| **AnalogIn** | `AP_HAL_RP2350/AnalogIn.cpp` | ADC working with voltage scaling |
| **UARTDriver** | `AP_HAL_RP2350/UARTDriver.cpp` | USB CDC + UART working |
| **Util** | `AP_HAL_RP2350/Util.cpp` | System ID, memory info working |
| **hwdef.h** | `AP_HAL_RP2350/hwdef.h` | Pin definitions, feature flags |
| **HAL_RP2350_Class** | `AP_HAL_RP2350/HAL_RP2350_Class.cpp` | Init order correct per PD1 |
| **Documentation** | `docs/PD12_FIX_OPTIONS.md` | Fix options documented |
| **Documentation** | `docs/RP2350_FULL_AP_PORT.md` | Platform differences documented |

### ⚠️ KEEP WITH CAUTION - May Need Fixes

| Component | Location | Issue | Action |
|-----------|----------|-------|--------|
| **I2CDevice** | `AP_HAL_RP2350/I2CDevice.cpp` | Hardware I2C is stub, using software fallback | Keep for now, fix later |
| **SPIDevice** | `AP_HAL_RP2350/SPIDevice.cpp` | Polling-only (no DMA per PD8) | Keep, may limit 1kHz+ rates |
| **DeviceBus** | `AP_HAL_RP2350/DeviceBus.cpp` | ~~Had Core 0 pinning hack~~ | ✅ Removed - std::atomic fix works |

### 🔴 REWRITE / REMOVE - Problematic or Incomplete

| Component | Location | Issue | Action |
|-----------|----------|-------|--------|
| ~~**Core pinning hack**~~ | ~~Various~~ | ~~Was workaround, not solution~~ | ✅ REMOVED (Phase 2 complete) |
| **AP_InertialSensor (upstream)** | `lib/ardupilot/` | Has the race condition | Copy to ap_compat, apply std::atomic fix |

### ❓ NOT YET IMPLEMENTED - Needed for EKF Path

| Component | Status | Blocker |
|-----------|--------|---------|
| ~~AP_Param~~ | ✅ **WORKING** | AP_Param::setup() runs, storage backend working |
| AP_Baro | Not in sparse checkout | Add to sparse checkout |
| AP_GPS | Not in sparse checkout | Add to sparse checkout |
| AP_NavEKF3 | Not in sparse checkout | Requires above + PD12 fix |

---

## Phased Implementation Plan with Checkpoints

### Phase 1: Apply std::atomic Fix (IMMEDIATE)

**Goal:** Fix PD12 memory visibility issue

**Steps:**
1. Copy AP_InertialSensor to `lib/ap_compat/AP_InertialSensor/`
2. Apply std::atomic changes (~30 lines per PD12_FIX_OPTIONS.md)
3. Update CMakeLists.txt to use local copy instead of upstream
4. Build and verify compilation

**Checkpoint 1A - COMPILE ONLY (Max 2 hours):** ✅ PASS (2026-01-28)
- [x] AP_InertialSensor copied to ap_compat
- [x] std::atomic applied to flags
- [x] Project compiles without errors (rocketchip.uf2 built, 180KB)
- **NO HARDWARE REQUIRED** - this is a build verification only

**Checkpoint 1B - HARDWARE TEST (Max 4 hours from start):** ✅ PASS (2026-01-28)
- [x] Flash firmware to board (smoke_ins_probe.uf2)
- [x] `wait_for_sample()` returns within expected time (init completed in 1311ms)
- [x] Gyro/accel sample rates visible (1125Hz confirmed)
- [x] No hangs during sensor init
- **HARDWARE VALIDATED** - std::atomic fix working

**🚨 CIRCUIT BREAKER 1:** If wait_for_sample() still hangs after std::atomic fix:
- STOP and evaluate Option F (Custom EKF)
- Do NOT spend more than 1 day debugging ArduPilot internals
- Document exact failure mode before pivoting

---

### Phase 2: Remove Core Pinning (After Phase 1 Success)

**Goal:** Verify dual-core operation works

**Steps:**
1. Remove any `xTaskCreatePinnedToCore()` calls pinning to Core 0
2. Let FreeRTOS SMP schedule freely across both cores
3. Verify sensors still work

**Checkpoint 2 - HARDWARE TEST (Max 2 hours):** ✅ PASS (2026-01-28)
- [x] Core pinning removed (code change)
- [x] Reflash firmware
- [x] Sensor reads work on both cores
- [x] No hangs or race conditions observed
- **HARDWARE VALIDATED** - dual-core operation confirmed

**🚨 CIRCUIT BREAKER 2:** If removing core pinning causes hangs:
- std::atomic fix is insufficient
- Document which core combination fails
- Consider Option F (Custom EKF) to avoid ArduPilot threading assumptions

---

### Phase 3: Enable AP_Param (After Phase 2 Success)

**Goal:** Enable ArduPilot parameter system

**Steps:**
1. Change `AP_PARAM_ENABLED=0` to `AP_PARAM_ENABLED=1` in AP_HAL_Compat.h
2. Build and document any new missing dependencies
3. Test parameter save/load to flash

**Checkpoint 3 (Max 4 hours):** ✅ PASS (2026-01-28)
- [x] AP_PARAM_ENABLED=1 compiles (already enabled in AP_HAL_Compat.h and hwdef.h)
- [x] AP_Param::setup() added and compiles (ins_probe_test.cpp, 269KB UF2)
- [x] AP_Param::setup() runs successfully (hardware validated)
- [ ] Parameters persist across reboot (not yet tested - need calibration to write params)

**🚨 CIRCUIT BREAKER 3:** If AP_Param causes cascading dependency failures:
- List all missing dependencies
- Evaluate if adding them is feasible vs. custom calibration storage
- Do NOT create "simplified wrappers" without approval

---

### Phase 4: Add Missing Sensor Libraries

**Goal:** Complete sensor stack for EKF

**Steps:**
1. `git sparse-checkout add libraries/AP_Baro`
2. `git sparse-checkout add libraries/AP_GPS`
3. Integrate with existing HAL
4. Verify sensor detection

**Checkpoint 4 (Max 1 day):**
- [ ] AP_Baro compiles and detects DPS310
- [ ] AP_GPS compiles and detects PA1010D
- [ ] All sensors report valid data

---

### Phase 5: EKF3 or Custom EKF Decision

**Decision Point:** Based on Phase 1-4 outcomes

**If all phases passed:**
- Add AP_NavEKF3 to sparse checkout
- Integrate with sensor stack
- Full ArduPilot sensor fusion path

**If any circuit breaker triggered:**
- Implement Custom 13-state EKF using AP_Math
- ~1000 lines, 2-3 weeks development
- Full control, no ArduPilot threading assumptions

---

## Progress Tracking

### Session Log Format
```
[DATE] [PHASE] [CHECKPOINT] [STATUS] [TIME SPENT]
Example:
2026-01-28 Phase1 1A PASS 1.5h
2026-01-28 Phase1 1B FAIL 3h - wait_for_sample hung after 2s
```

### Session Log
```
2026-01-28 Phase1 1A PASS ~1h - rocketchip.uf2 built with std::atomic fix
2026-01-28 Phase1 1B PASS ~30min - wait_for_sample() working, 1125Hz sample rate confirmed
2026-01-28 Phase2 2  PASS ~30min - Core pinning removed, dual-core operation confirmed
2026-01-28 Phase3 3A PASS ~20min - AP_Param::setup() added, compiles (269KB UF2)
2026-01-28 Phase3 3B PASS ~5min - AP_Param::setup() runs on hardware, no errors
```

### Circular Progress Detection

**Warning Signs (STOP if you see these):**
1. Same error message appearing after "fix" applied
2. Trying same approach with minor variations
3. Adding workarounds to workarounds
4. Time on single checkpoint exceeds max by 2x
5. User asking "didn't we already try this?"

**If Detected:**
1. STOP current approach
2. Document exact state and failure
3. Evaluate next circuit breaker option
4. Ask user before continuing

---

## Files to Track

| File | Purpose | Watch For |
|------|---------|-----------|
| `lib/ap_compat/AP_InertialSensor/AP_InertialSensor.h` | Modified flags | std::atomic applied |
| `lib/ap_compat/AP_InertialSensor/AP_InertialSensor.cpp` | Modified reads | .load(acquire) used |
| `lib/ap_compat/AP_InertialSensor/AP_InertialSensor_Backend.cpp` | Modified writes | .store(release) used |
| `lib/ap_compat/AP_HAL_RP2350/DeviceBus.cpp` | Core pinning | ✅ Removed in Phase 2 |
| `lib/ap_compat/AP_HAL_Compat.h` | Feature flags | AP_PARAM_ENABLED |

---

## Success Criteria (End State)

**Minimum Viable:** ✅ ACHIEVED (2026-01-28)
- [x] wait_for_sample() works reliably
- [x] Dual-core operation (no core pinning)
- [x] Sensor data flowing at expected rates (1125Hz confirmed)

**Full Success:**
- [x] AP_Param enabled and working (2026-01-28)
- [ ] All sensors integrated
- [ ] EKF3 running (or custom EKF if circuit breakers triggered)

---

## Reference Documents

- [PD12_FIX_OPTIONS.md](PD12_FIX_OPTIONS.md) - All fix options
- [RP2350_FULL_AP_PORT.md](RP2350_FULL_AP_PORT.md) - Platform differences
- `.claude/LESSONS_LEARNED.md` - Debugging history
- `chibios/PROGRESS.md` - ChibiOS path status (backup option)
