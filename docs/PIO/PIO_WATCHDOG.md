# PIO Hardware Watchdog

**Status:** Concept / Design Notes
**Last Updated:** 2026-02-03
**Decision:** Not yet committed to IVP. Low implementation cost makes this viable as a test-when-ready feature.

---

## Motivation

PIO state machines are independent hardware that continues executing even if both ARM cores lock up, hardfault, or deadlock. This makes them suitable as a last-resort safety monitor that sits between software watchdogs and discrete hardware safety circuits.

### Safety Layer Model

```
Discrete hardware (arm switches, voter circuits)   <- cannot be overridden by software
PIO watchdog / lockout                              <- runs if both cores are dead
SDK hardware watchdog (hw_watchdog)                 <- runs if main loop is stuck
Application state machine (arm/disarm, timeouts)    <- flight logic layer
```

PIO fills the gap between "software is still running" and "need discrete hardware to intervene."

---

## Concept 1: Heartbeat Watchdog

A PIO state machine monitors a heartbeat from one or both ARM cores. If the heartbeat stops, PIO drives a safety GPIO (pyro disable, error LED, safe-mode signal).

### How It Works

1. Application code writes a token to the PIO TX FIFO on a regular cadence
2. PIO SM attempts a `pull` with a timeout (implemented as a counter loop)
3. If the pull succeeds, reset the counter — core is alive
4. If the counter expires, drive the safety output pin

### PIO Assembly (Sketch)

```pio
; Heartbeat watchdog - ~8 instructions
; Drives `out_pin` HIGH if TX FIFO is not fed within timeout

.program heartbeat_watchdog

.wrap_target
    set x, 31              ; outer loop counter (adjust for timeout)
outer:
    set y, 31              ; inner loop counter
inner:
    pull noblock            ; non-blocking pull from TX FIFO
    mov osr, ~null          ; load comparison value (all 1s)
    ; If pull succeeded, FIFO had data -> restart
    jmp !osre got_heartbeat ; if OSR != empty, heartbeat received
    jmp y-- inner           ; inner countdown
    jmp x-- outer           ; outer countdown
    ; Timeout expired - no heartbeat
    set pins, 1             ; drive safety pin HIGH (trigger safe mode)
    pull block              ; halt here until core recovers and feeds FIFO
    set pins, 0             ; core recovered, clear safety signal
    jmp got_heartbeat
got_heartbeat:
    set pins, 0             ; ensure safety pin LOW
.wrap
```

> **Note:** This is a conceptual sketch. The actual FIFO-empty detection mechanism needs validation against PIO behavior — `pull noblock` loads a value from the FIFO or copies X to OSR if empty, so the detection logic may need adjustment. Test on hardware before relying on this.

### Timeout Calculation

PIO instruction timing: 1 cycle per instruction at the SM's clock rate.

| Clock Divider | Instruction Rate | Inner x Outer (31x31) | Approx Timeout |
|---------------|-----------------|----------------------|----------------|
| 1 (150 MHz) | 150 MHz | ~961 iterations x ~4 inst | ~25 us |
| 256 | ~586 kHz | ~961 x 4 | ~6.5 ms |
| 65535 | ~2.3 kHz | ~961 x 4 | ~1.7 s |

For longer timeouts, increase counter values or nest additional loops. A 32-bit counter (two nested 16-bit loops) provides timeouts up to minutes at moderate clock dividers.

### C SDK Integration (Sketch)

```c
#include "hardware/pio.h"
#include "heartbeat_watchdog.pio.h"  // generated from .pio file

#define WATCHDOG_SAFETY_PIN 10  // choose an appropriate GPIO
#define WATCHDOG_TIMEOUT_DIVIDER 65535  // ~2.3 kHz instruction rate

static PIO watchdog_pio;
static uint watchdog_sm;

bool pio_watchdog_init(void) {
    watchdog_pio = pio2;  // use PIO2, keep PIO0/1 for I/O protocols
    watchdog_sm = pio_claim_unused_sm(watchdog_pio, false);
    if (watchdog_sm < 0) return false;

    uint offset = pio_add_program(watchdog_pio, &heartbeat_watchdog_program);

    pio_sm_config c = heartbeat_watchdog_program_get_default_config(offset);
    sm_config_set_set_pins(&c, WATCHDOG_SAFETY_PIN, 1);
    sm_config_set_clkdiv(&c, WATCHDOG_TIMEOUT_DIVIDER);

    pio_gpio_init(watchdog_pio, WATCHDOG_SAFETY_PIN);
    pio_sm_set_consecutive_pindirs(watchdog_pio, watchdog_sm,
                                   WATCHDOG_SAFETY_PIN, 1, true);

    pio_sm_init(watchdog_pio, watchdog_sm, offset, &c);
    pio_sm_set_enabled(watchdog_pio, watchdog_sm, true);

    return true;
}

// Call this periodically from main loop to feed the watchdog
static inline void pio_watchdog_feed(void) {
    pio_sm_put(watchdog_pio, watchdog_sm, 0xDEADBEEF);  // any non-zero token
}
```

### Application Usage

```c
int main() {
    // ... init ...
    pio_watchdog_init();

    while (true) {
        // Main loop work
        read_sensors();
        update_state_machine();

        // Feed the watchdog - if this stops, PIO triggers safety pin
        pio_watchdog_feed();

        sleep_ms(10);
    }
}
```

---

## Concept 2: Dual-Core Cross-Check

Extends the heartbeat concept to monitor both cores independently using state machines within the same PIO block (shared IRQ flags enable coordination).

```
PIO0 (or PIO2)
┌────────┬────────┬────────┐
│  SM0   │  SM1   │  SM2   │
│ Core 0 │ Core 1 │Monitor │
│heartbt │heartbt │        │
└───┬────┴───┬────┴───┬────┘
    │        │        │
    └── IRQ flags ────┘  (shared within block)
                      │
                   Safety GPIO
```

- SM0 watches Core 0's FIFO, sets IRQ flag 0 on timeout
- SM1 watches Core 1's FIFO, sets IRQ flag 1 on timeout
- SM2 monitors both IRQ flags, drives safety GPIO if either fires

Cost: 3 state machines in one PIO block. Leaves 1 SM in that block and 8 SMs across the other two blocks.

---

## Concept 3: Pyro Channel Hardware Lockout

A PIO SM sits between the ARM core and the pyro channel MOSFET gate, requiring a specific bit sequence (not just a GPIO high) to enable firing.

**Why this matters:** A crashed CPU with corrupted GPIO registers could drive a random pin high. A PIO lockout requires a deliberate multi-word sequence in the TX FIFO — something a crashed CPU cannot accidentally produce.

### Sequence

1. Software writes an arm token (e.g., `0xARM1ARM1`) to PIO FIFO
2. PIO SM validates the token bit pattern
3. PIO SM enters armed state, waiting for fire command
4. Software writes fire token (e.g., `0xF1REF1RE`)
5. PIO SM drives pyro GPIO for a precise, timed pulse
6. PIO SM automatically returns to disarmed state

This ensures:
- No accidental firing from CPU crash
- Precise, deterministic firing pulse width (PIO cycle-accurate)
- Automatic return to safe state

> **Note:** This concept adds a layer of software-controlled safety but does NOT replace a physical arm switch. The discrete hardware arm switch (per CODING_STANDARDS.md pyro safety requirements) remains the primary lockout.

---

## Resource Budget

| PIO Block | Current Use | Watchdog Use | Remaining |
|-----------|-------------|-------------|-----------|
| PIO0 | WS2812 (1 SM) | - | 3 SMs |
| PIO1 | Available | Future I/O protocols | 4 SMs |
| PIO2 | Available | Watchdog concepts (2-3 SMs) | 1-2 SMs |

Dedicating PIO2 to safety functions keeps it isolated from I/O protocol work on PIO0/PIO1.

---

## PIO Limitations for Safety Use

- **No sensor access:** PIO cannot read I2C/SPI sensors for health checks. It can only monitor GPIO pins and FIFO inputs.
- **No logging:** 2 scratch registers and no memory access. PIO can signal a problem but cannot record what happened.
- **Simple logic only:** 9 instruction types, no arithmetic beyond shifts. Pattern matching is limited to bit tests.
- **Not a replacement for discrete hardware:** PIO is still on the same silicon as the ARM cores. A power glitch or silicon fault can take out everything. Discrete voter circuits and physical arm switches remain necessary for critical safety functions.

---

## Next Steps (When Ready)

1. Prototype the heartbeat watchdog PIO program on hardware
2. Validate FIFO-empty detection behavior (pull noblock semantics)
3. Measure actual timeout accuracy vs calculated
4. If viable, integrate with the SDK hardware watchdog as a complementary layer
5. Evaluate whether pyro lockout concept adds meaningful safety over software-only arm/disarm

---

*This document captures design concepts for potential PIO-based safety features. Implementation is not committed — prototyping will determine if the complexity-to-benefit tradeoff is worthwhile.*
