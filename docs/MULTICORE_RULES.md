# RP2350 Platform Notes

**Status:** Reference document
**Last Updated:** 2026-02-03
**Source:** Pico SDK source (`stdio_usb.c`), RP2350 datasheet, `pico/multicore.h`

---

## Core Architecture

The RP2350 is a dual-core ARM Cortex-M33. In bare-metal Pico SDK mode, Core 0 runs `main()` and Core 1 is available for explicit use via `multicore_launch_core1()`.

---

## Core 0: Main Loop + USB

### USB IRQ Handlers are on Core 0

The Pico SDK registers USB interrupt handlers on whichever core calls `stdio_init_all()`. Since `main()` runs on Core 0, USB IRQ handling is pinned there. This is SDK-level behavior.

The handler itself is lightweight -- it calls `tud_task()` under a mutex, takes microseconds. It does NOT monopolize the core.

**Source:** `pico-sdk/src/rp2_common/pico_stdio_usb/stdio_usb.c` -- `irq_add_shared_handler(USBCTRL_IRQ, usb_irq, ...)` called from `stdio_usb_init()`, which is called from `stdio_init_all()`, which runs in `main()` on Core 0.

### Main Loop Runs Here

Core 0 handles the main application loop, CLI/serial I/O, and any logic that needs USB access. All `printf()` and `getchar()` calls should originate from Core 0 to avoid cross-core USB mutex contention.

---

## Core 1: Available for Future Use

Core 1 is idle by default. It can be launched for dedicated workloads using the SDK's multicore API:

```c
#include "pico/multicore.h"

void core1_entry() {
    // Runs on Core 1
    while (true) {
        // Sensor sampling, computation offload, etc.
    }
}

int main() {
    stdio_init_all();
    multicore_launch_core1(core1_entry);

    // Core 0 main loop
    while (true) {
        // Application logic, CLI, USB I/O
    }
}
```

**Potential Core 1 uses:**
- High-rate sensor sampling (ISR-driven or polled)
- Computation offload (ESKF, calibration fits)
- Keeping time-critical work isolated from USB IRQ jitter

---

## Cross-Core Communication

If Core 1 is used, the SDK provides primitives for cross-core data sharing.

### SDK Multicore Primitives

```c
#include "pico/multicore.h"

// FIFO (32-bit words, hardware-backed, blocking)
multicore_fifo_push_blocking(data);
uint32_t data = multicore_fifo_pop_blocking();

// Non-blocking variants
bool success = multicore_fifo_push_timeout_us(data, timeout_us);
bool success = multicore_fifo_pop_timeout_us(&data, timeout_us);
```

The hardware FIFO is small (8 entries per direction) -- suitable for signaling and small data, not bulk transfer.

### Spinlocks for Shared Data

For protecting shared data structures accessed from both cores:

```c
#include "hardware/sync.h"

static spin_lock_t *shared_lock;

// Initialization (call once from Core 0)
shared_lock = spin_lock_init(spin_lock_claim_unused(true));

// Critical section (either core)
uint32_t saved = spin_lock_blocking(shared_lock);
// ... access shared data ...
spin_unlock(shared_lock, saved);
```

Spinlocks disable interrupts on the calling core while held. Keep critical sections short.

### Never Use Plain volatile for Cross-Core Sharing

`volatile` prevents compiler reordering but does NOT issue ARM hardware memory barriers. Data written on one core may not be visible on the other.

```c
// BROKEN for cross-core
volatile bool flag = false;

// Works cross-core
#include <stdatomic.h>
atomic_bool flag = false;  // or use spinlocks / FIFO
```

*(LESSONS_LEARNED Entry 8)*

### Shared Data Patterns

For bulk data sharing between cores (e.g., sensor samples to computation), consider a double-buffer pattern with an atomic swap:

```
Core 1 writes -> Buffer A (while Core 0 reads Buffer B)
Atomic swap pointer when Core 1 completes a sample set
Core 0 reads  -> Buffer B (while Core 1 writes Buffer A)
```

This avoids spinlock contention on high-rate data paths.

---

## printf / USB I/O Across Cores

The SDK's USB mutex (`stdio_usb_mutex`) is designed to be cross-core safe. However, calling `printf()` from Core 1 contends with USB IRQ handlers on Core 0.

**Recommendation:** Keep all USB I/O on Core 0. If Core 1 needs to report data, pass it to Core 0 via FIFO or shared buffer, and let Core 0 handle the printing.

---

## Flash Access

`flash_range_erase()` and `flash_range_program()` make the ENTIRE flash chip inaccessible. This means:
- Code executing from flash on EITHER core will crash
- USB IRQ handlers (in flash) will fail

**Always use `flash_safe_execute()` from `pico/flash.h`.** This coordinates both cores to ensure safe flash access.

*(LESSONS_LEARNED Entries 4, 12)*

---

## Memory and Stack Considerations

### Stack Size Awareness

The default main thread stack on RP2350 is limited. Large local variables cause silent stack overflow -- the crash appears random because the compiler allocates all stack space at function entry.

| Variable Size | Allocation Strategy |
|--------------|-------------------|
| < 256 bytes | Local variable (stack) is fine |
| 256 bytes - 1KB | Consider `static` if function is called frequently |
| > 1KB | Must be `static` or heap-allocated |

**Large local variables (>1KB) cause immediate stack overflow.** Use `static` or heap allocation instead. *(LESSONS_LEARNED Entries 1, 19)*

```c
// CORRECT: Static allocation for large objects
static float matrix[6][6];  // 144 bytes, safe as static

void compute() {
    // Use matrix here -- persists between calls
}

// INCORRECT: Large local in a function with limited stack
void compute() {
    float matrix[64][64];  // 16KB on stack -- will overflow
}
```

### MPU Stack Guard and Fault Handling

**Single source of truth:** `src/safety/fault_protection.cpp` (and header). Contains `mpu_setup_stack_guard()`, `memmanage_fault_handler()`, shared MPU limit constant, and `Q_onError`.

Both cores **must** call the guard setup:
- Core 0: from `init_early_hw()`
- Core 1: from `core1_entry()` (before sensor loop)

Do not duplicate handler bodies or guard sizes. For diagnosis use `SCB->CFSR` (`0xE000ED28`). GDB/OpenOCD may report “Handler HardFault” while PC is inside `memmanage_fault_handler()` — the label reflects the vector, not separate C functions.

*(See `docs/FAULT_INJECTION.md` for testing and `docs/baselines/stage_o_hw_verification_2026-04-28.md` for Stage O validation procedure.)*

### Core 1 Stack

When launching Core 1 via `multicore_launch_core1()`, the SDK provides a default stack. For custom stack sizes, use `multicore_launch_core1_with_stack()`:

```c
static uint32_t core1_stack[1024];  // 4KB stack for Core 1

multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof(core1_stack));
```

Size the stack based on the workload -- matrix math and calibration fits need at least 1024 words (4KB).

---

## PIO State Machines

The RP2350 has 3 PIO blocks (PIO0, PIO1, PIO2) with 4 state machines each. PIO programs are NOT core-specific — any core can interact with any PIO state machine. Currently used:

- WS2812/NeoPixel driver — 1 SM on PIO2 SM0 (or wherever `pio_claim_free_sm_and_add_program_for_gpio_range` lands it)
- Heartbeat watchdog SM — 1 SM on PIO2
- Backup deployment timers (drogue + main) — 2 SMs on PIO2

### PIO program lifecycle rule

**`pio_add_program` / `pio_remove_program` pair with driver init/teardown, NOT with arm/disarm.**

The program lives in shared PIO instruction memory (32 slots per block). Add at init, remove at teardown (if ever). Within that lifetime, the state machines start/stop via `pio_sm_set_enabled(sm, true/false)` — the program memory is not touched.

Why this matters: `pio_remove_program` mutates the SDK's `_used_instruction_space` metadata bitmap. Calling it twice with the same offset trips an assertion (`__assert_func` → `_exit(1)` → `__breakpoint`, Core 0 wedged). The idiomatic `pico-examples/pio/hello_pio` pattern pairs `pio_claim_free_sm_and_add_program_*` at init with `pio_remove_program_and_unclaim_sm` at teardown — never mid-cycle.

See LL Entry 42 for the lived-experience case: `pio_backup_timer_disarm()` called `pio_remove_program` as "defense in depth" — the first ARM→RESET cycle worked; the second cycle's disarm tripped the SDK assertion, wedging Core 0 and frozen-pipe'ing USB CDC. Latent in tree since IVP-89 (2026-04). Fixed 2026-05-20.

### Pin function lifecycle rule (paired with the program rule)

If `pio_gpio_init(pin)` is called at init and `gpio_set_function(pin, GPIO_FUNC_SIO)` is called at disarm (e.g., as a "pin LOW for visible safety" gesture), then **arm must restore the PIO function on the pin before enabling the SM.** Otherwise the SM runs against a pin that's still under SIO control — `set pins, 1` in the PIO program is a silent no-op.

Easiest path: call the auto-generated `<program>_program_init(pio, sm, offset, pin)` in arm() as well as init(). It's idempotent — `pio_gpio_init` reduces to `gpio_set_function(pin, PIO_FUNCSEL)`, and `pio_sm_init` is documented to handle already-running SMs by disabling first.

### Resuming a halted SM cleanly

`pio_sm_set_enabled(false)` does NOT reset the SM's internal state. When re-enabled, the SM resumes exactly where it was halted (mid-instruction, with whatever shift register / FIFO state it had). For arm/disarm-style cycles where you want a known starting state on re-arm:

- `pio_sm_init(pio, sm, offset, &config)` — full reset to entry point with config re-applied (heavyweight but unambiguous).
- `pio_sm_restart(pio, sm)` — lighter: clears ISR, shift counters, clock divider counter, pin-write flags, delay counter, latched EXEC, IRQ wait. Keeps program/config.
- `pio_sm_clear_fifos(pio, sm)` — drain TX/RX FIFOs (e.g., stale countdown values from prior arm).

### Primary sources

- [RP2350 datasheet §11.7 (PIO)](https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf) — instruction memory, state machine architecture, claim semantics.
- [pico-sdk `hardware/pio.h`](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pio/include/hardware/pio.h) — `pio_add_program`, `pio_remove_program`, `pio_sm_init`, `pio_sm_restart`, `pio_sm_set_enabled`, `pio_sm_clear_fifos`, `pio_gpio_init`.
- [pico-sdk `hardware/pio.c`](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_pio/pio.c) — the assertion at `pio_remove_program` (line ~203) is the one we tripped; the body confirms `pio_remove_program` is metadata-only (clears the `_used_instruction_space` bitmap), it does NOT zero PIO instruction memory.
- [pico-examples `pio/hello_pio`](https://github.com/raspberrypi/pico-examples/tree/master/pio/hello_pio) — idiomatic `claim+add → run → remove+unclaim` lifecycle. Pair this with the project's arm/disarm-via-`pio_sm_set_enabled` cycling.

Future users: SPI DMA for high-rate sensor mode (if/when migrating from I2C).

---

## What NOT to Worry About

- **I2C bus:** SDK I2C functions are not core-specific. Bus arbitration is at the peripheral level, not core level
- **GPIO:** Any core can read/write any GPIO. The SDK functions are safe
- **Timers/Alarms:** The SDK alarm pool is shared across cores with proper synchronization
- **DMA:** DMA channels are shared resources, not core-specific. Claim channels to avoid conflicts

---

*This document captures RP2350 platform behavior as understood from SDK source code and project experience. Update when new multi-core issues are discovered or validated.*
