# RP2350 Multi-Core Rules (FreeRTOS SMP)

**Status:** Reference document
**Last Updated:** 2026-02-03
**Source:** Pico SDK source (`stdio_usb.c`), FreeRTOS SMP documentation, RP2350 datasheet

---

## The Key Principle

FreeRTOS SMP load-balances unpinned tasks across cores automatically. **Don't over-constrain core assignments.** Only pin a task to a specific core when there's a concrete technical reason.

---

## What's Hardwired (Can't Change)

### USB IRQ Handlers → Core 0

The Pico SDK registers USB interrupt handlers on whichever core calls `stdio_init_all()`. Since `main()` runs on Core 0, USB IRQ handling is pinned there. This is SDK-level behavior.

The handler itself is lightweight — it calls `tud_task()` under a mutex, takes microseconds. It does NOT monopolize the core.

**Source:** `pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c` — `irq_add_shared_handler(USBCTRL_IRQ, usb_irq, ...)` called from `stdio_usb_init()`, which is called from `stdio_init_all()`, which runs in `main()` on Core 0.

---

## What Should Be Pinned

### SensorTask → Core 1

Pin SensorTask to Core 1 to avoid USB IRQ jitter affecting sensor timing. At 1kHz sampling, even microsecond-level jitter from USB IRQs during I2C transactions degrades sensor fusion quality.

```c
vTaskCoreAffinitySet(sensorTaskHandle, (1 << 1));  // Pin to Core 1
```

This is the ONLY task that needs hard pinning.

---

## What Should Float (Unpinned)

Let FreeRTOS SMP handle load balancing for everything else:

| Task | Why Unpinned |
|------|-------------|
| FusionTask | Compute-heavy but tolerates scheduling jitter |
| MissionTask | Event-driven, low duty cycle |
| LoggerTask | Burst I/O, naturally yields during flash ops |
| TelemetryTask | Low rate (10Hz), limited by radio speed |
| UITask | Best-effort, human-perceptible timescales |
| CLITask | Interactive, blocks on input |

**Load balancing benefit:** FreeRTOS SMP migrates unpinned tasks to whichever core has capacity. This automatically distributes ESKF computation, logging bursts, and telemetry without manual tuning.

---

## printf / USB I/O Across Cores

### Current Status: NEEDS VALIDATION

The SDK's USB mutex (`stdio_usb_mutex`) is designed to be cross-core safe. Tasks on either core should be able to call `printf()` without breaking USB.

**However:** Previous development saw USB/CLI breakage when printf was used, potentially due to:
- The old (wrong) FreeRTOS fork — now fixed
- Core assignment contradictions in the docs — now clarified
- BASEPRI elevation blocking USB IRQs — documented in LESSONS_LEARNED Entry 3

**Validation test after reboot:** Run UITask/CLITask unpinned and verify:
1. USB CDC enumeration succeeds
2. printf output appears correctly from both cores
3. CLI commands work without freezing
4. No USB disconnects during sensor sampling

**If validation fails:** Pin UITask and CLITask to Core 0:
```c
vTaskCoreAffinitySet(uiTaskHandle, (1 << 0));
vTaskCoreAffinitySet(cliTaskHandle, (1 << 0));
```

---

## Cross-Core Communication Rules

### Use FreeRTOS Primitives

Queues, mutexes, and semaphores include implicit memory barriers. This is the correct way to share data between tasks regardless of which core they run on.

```c
// Producer (any core)
xQueueSend(sensorQueue, &data, portMAX_DELAY);

// Consumer (any core)
xQueueReceive(sensorQueue, &data, portMAX_DELAY);
```

### Never Use Plain volatile for Cross-Core Sharing

`volatile` prevents compiler reordering but does NOT issue ARM hardware memory barriers. Data written on one core may not be visible on the other.

```c
// ❌ BROKEN for cross-core
volatile bool flag = false;

// ✅ Works cross-core
std::atomic<bool> flag{false};  // or use FreeRTOS primitives
```

*(LESSONS_LEARNED Entry 8)*

### Don't Busy-Wait with taskYIELD()

`taskYIELD()` only yields to tasks of equal or higher priority. Use `vTaskDelay(1)` to yield to ALL tasks.

*(LESSONS_LEARNED Entry 9)*

---

## Memory Layout Considerations

### Stack Allocation

Each task gets its own stack, allocated from FreeRTOS heap. Stack sizes are specified in words (multiply by 4 for bytes on 32-bit ARM).

| Task Category | Minimum Stack | Notes |
|--------------|--------------|-------|
| Simple I/O | 256 words (1KB) | LED, button polling |
| Moderate compute | 512 words (2KB) | Filtering, parsing |
| Matrix math | 1024 words (4KB) | ESKF, calibration fits |

**Large local variables (>1KB) cause immediate stack overflow.** Use `static` or heap allocation instead. The crash appears random because the compiler allocates all stack space at function entry. *(LESSONS_LEARNED Entries 1, 19)*

### Shared Data Structures

For lock-free data sharing between SensorTask and FusionTask, consider a double-buffer or triple-buffer pattern rather than mutex-protected shared structs. This avoids priority inversion and keeps SensorTask deterministic.

```
SensorTask writes → Buffer A (while FusionTask reads Buffer B)
Atomic swap pointer when SensorTask completes a sample set
FusionTask reads → Buffer B (while SensorTask writes Buffer A)
```

This is a design decision for the ESKF implementation phase — not something that needs to be decided now.

---

## Flash Access

`flash_range_erase()` and `flash_range_program()` make the ENTIRE flash chip inaccessible. This means:
- Code executing from flash on EITHER core will crash
- USB IRQ handlers (in flash) will fail

**Always use `flash_safe_execute()` from `pico/flash.h`.** This coordinates both cores to ensure safe flash access.

*(LESSONS_LEARNED Entries 4, 12)*

---

## PIO State Machines

The RP2350 has 3 PIO blocks (PIO0, PIO1, PIO2) with 4 state machines each. PIO programs are NOT core-specific — any core can interact with any PIO state machine. Currently used:

- WS2812/NeoPixel driver uses 1 PIO state machine

Future users: SPI DMA for high-rate sensor mode (if/when migrating from I2C).

---

## What NOT to Worry About

- **FreeRTOS heap:** Shared across cores, thread-safe via internal locking
- **I2C bus:** SDK I2C functions are not core-specific. Bus arbitration is at the peripheral level, not core level
- **GPIO:** Any core can read/write any GPIO. The SDK functions are safe
- **Timers/Alarms:** The SDK alarm pool is shared across cores with proper synchronization

---

*This document captures RP2350 multi-core behavior as understood from SDK source code and project experience. Update when new multi-core issues are discovered or validated.*
