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

### Core 1 Stack

When launching Core 1 via `multicore_launch_core1()`, the SDK provides a default stack. For custom stack sizes, use `multicore_launch_core1_with_stack()`:

```c
static uint32_t core1_stack[1024];  // 4KB stack for Core 1

multicore_launch_core1_with_stack(core1_entry, core1_stack, sizeof(core1_stack));
```

Size the stack based on the workload -- matrix math and calibration fits need at least 1024 words (4KB).

---

## PIO State Machines

The RP2350 has 3 PIO blocks (PIO0, PIO1, PIO2) with 4 state machines each. PIO programs are NOT core-specific -- any core can interact with any PIO state machine. Currently used:

- WS2812/NeoPixel driver uses 1 PIO state machine

Future users: SPI DMA for high-rate sensor mode (if/when migrating from I2C).

---

## What NOT to Worry About

- **I2C bus:** SDK I2C functions are not core-specific. Bus arbitration is at the peripheral level, not core level
- **GPIO:** Any core can read/write any GPIO. The SDK functions are safe
- **Timers/Alarms:** The SDK alarm pool is shared across cores with proper synchronization
- **DMA:** DMA channels are shared resources, not core-specific. Claim channels to avoid conflicts

---

*This document captures RP2350 platform behavior as understood from SDK source code and project experience. Update when new multi-core issues are discovered or validated.*
