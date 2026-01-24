# AP_HAL_RP2350 Rebuild Context

**Date:** 2026-01-24
**Status:** RESOLVED
**Reason:** Storage crash debugging revealed fundamental flash_safe_execute + FreeRTOS SMP incompatibility

---

## Resolution Summary (2026-01-24)

The fix that works:
1. **RAM-resident free functions** with `__not_in_flash_func()` (not class methods)
2. **Direct flash ops** (`flash_range_erase`, `flash_range_program`) with interrupt disable
3. **XIP cache invalidation** via `xip_cache_invalidate_all()` after flash ops
4. **Storage init before scheduler.init()** in HAL init sequence (but after FreeRTOS starts)

Key insight: Flash operations work fine with FreeRTOS SMP running as long as:
- Flash functions are in RAM (`__not_in_flash_func`)
- Interrupts are disabled during flash ops
- XIP cache is invalidated after flash ops

The issue was specifically `flash_safe_execute()` + `multicore_lockout`, not flash ops themselves.

---

## Critical Finding: flash_safe_execute DOES NOT WORK with FreeRTOS SMP

### What We Proved
1. `flash_safe_execute()` uses `multicore_lockout` internally
2. `multicore_lockout` conflicts with FreeRTOS SMP's dual-core scheduler
3. Calling `vTaskSuspendAll()` before `flash_safe_execute()` does NOT fix this
4. The crash happens inside `flash_safe_execute()`, not in our callback

### Debug Output Showing Crash Point
```
Scheduler initialized
  flash_read: sector=0 offset=0 len=4
  flash_read: addr=0x10084000
  flash_read: OK
  flash_erase: sector=0
  flash_erase: offset=0x00084000
  flash_erase: suspending scheduler...
  flash_erase: calling flash_safe_execute...
<CRASH - never returns>
```

### Root Cause
- `flash_safe_execute()` tries to signal Core 1 via SIO FIFO to enter a spin loop
- FreeRTOS SMP is managing Core 1's execution
- The lockout mechanism and FreeRTOS scheduler conflict, causing a hard fault

---

## Correct Solution: Scheduler-Aware Flash Operations

### Before Scheduler Starts (SAFE)
- Only Core 0 is running
- Can call `flash_range_erase()` / `flash_range_program()` directly
- Just need to disable interrupts: `save_and_disable_interrupts()`

### After Scheduler Starts (REQUIRES SPECIAL HANDLING)
Options (in order of preference):
1. **Initialize storage before scheduler** - Cleanest, avoids the problem entirely
2. **Pin task to Core 0 + ensure Core 1 idle** - More complex, needs FreeRTOS coordination
3. **Defer writes to safe window** - Queue writes, execute during known-safe periods

---

## Design Decisions for Rebuild

### Storage.cpp
1. Detect scheduler state with `xTaskGetSchedulerState()`
2. Pre-scheduler: direct flash ops with interrupt disable
3. Post-scheduler: either defer or use Core 0 pinning strategy
4. Remove debug printf statements after verification

### HAL Initialization Order
Consider splitting `hal.init()`:
- `hal.early_init()` - Called before scheduler, initializes storage
- `hal.init()` - Called after scheduler, initializes scheduler-dependent subsystems

### Flash Write Strategy
- Runtime writes should be queued and executed during safe windows
- Or: Pin storage task to Core 0, signal Core 1 tasks to pause

---

## Files to Rebuild

1. **Storage.cpp** - Complete rewrite with scheduler awareness
2. **Storage.h** - Add early_init() if splitting initialization
3. **HAL_RP2350_Class.cpp** - Update init sequence
4. **RP2350_FULL_AP_PORT.md** - Update PD1 status from "Resolved" to "Workaround"

---

## Test Verification

After rebuild, verify:
1. Storage init works when called before scheduler
2. Storage reads work anytime (XIP is always safe for reads)
3. Storage writes work (either pre-scheduler or with new safe strategy)
4. No crashes during normal operation
