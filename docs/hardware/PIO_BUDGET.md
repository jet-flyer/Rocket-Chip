# RP2350 PIO budget (Rocket Chip)

**Status:** Living map as of 2026-09-05 research pass (Goddard). Firmware claim sites
are authoritative; update this when SMs move.

RP2350 has **3 PIO blocks × 4 state machines** (12 SMs) and **32 instructions of
shared program memory per block**. Instruction memory is usually the scarce resource,
not raw SM count.

## Locked partition

| Block | Role | Rule |
|-------|------|------|
| **PIO0** | Status / soft peripherals | **WS2812** lives here today. Do not put safety timers here. |
| **PIO1** | Next feature rent | Prefer new work here (last-gasp beacon first). |
| **PIO2** | Safety only | Heartbeat WDT + backup pyro timers. Do not rent for product features. |

**Parked / non-starter for now:** I²C-via-PIO (needs rewiring; HW I2C is fine).

## Live claim map

| Block | Program / owner | SMs | Sources |
|-------|-----------------|-----|---------|
| PIO0 | WS2812 status LED | 1 (claimed via `pio_claim_free_sm_and_add_program_for_gpio_range`) | `src/drivers/ws2812_status.cpp`, SDK `ws2812.pio` |
| PIO1 | *(empty / reserved)* | 0 / 4 free | Comment in `pio_watchdog.cpp`: PIO1 reserved |
| PIO2 | Heartbeat watchdog | 1 (`pio_claim_unused_sm`) | `pio/heartbeat_watchdog.pio`, `src/safety/pio_watchdog.*` |
| PIO2 | Backup drogue + main timers | 2 (same `backup_timer` program, two SMs) | `pio/backup_timer.pio`, `src/safety/pio_backup_timer.*` |
| PIO2 | *(free)* | **1 SM free** | — |

Watchdog + backup both load programs into **PIO2 instruction memory** (shared 32-slot pool).

## Next rent (agreed order)

1. **Last-gasp / fault beacon on PIO1** (desk smoke first). Aligns with WB *PIO beacon + SPI last-gasp* sitting; prefer PIO-SPI / DIO-timed chirp over sharing HW SPI in a fault path. FSK-assist waits for a Starcom sitting.
2. **PIO WDT role policy** — use of the *existing* PIO2 WDT SM (Go/No-Go vs ARM), not a new program.
3. **RF / FSK bitstream assist** — later; keep LoRa packet SPI on the HW SPI controller.

## Leave alone

- ESKF / fusion math, NeoPixel (done), HW SPI/I2C/UART workloads that already work.
- “Use PIO because we have spare SMs” without a timing or independence requirement.

## First-flight note (passive Estes)

No pyro on first flight. Backup timer SMs may still init but are not load-bearing for chute.
Do not casually free PIO2 SMs without a profile gate; do not use that as an excuse to crowd PIO2 with beacon work — beacon stays on **PIO1**.

## Related

- WB: Early-impl table *Fault beacon (last-gasp)*; research row *PIO beacon + SPI last-gasp*.
- `docs/audits/EARLY_IMPL_REWORK_2026-08-31.md` (PIO backup-timer KEEP).
