# RP2350 Silicon Errata — Compliance Matrix

**Status:** Initial sweep complete 2026-04-22. 28 errata documented.
**Last datasheet revision checked:** local `docs/hardware/datasheets/rp2350-datasheet.pdf` as of 2026-04-22.
**Last pico-sdk version checked:** 2.2.0.
**Last full-sweep date:** 2026-04-22.

**Scope:** All documented RP2350 silicon errata, their applicability to
our hardware (RP2350A on Adafruit Feather, RP2350B on Adafruit Fruit
Jam — both A2 stepping), and our handling status for each.

**Why this document exists:** the RP2350 is early-life silicon with an
active errata list. Relying on the SDK to transparently handle every
erratum is unsafe — some errata are acknowledged but not mitigated, and
some mitigations have trigger-path gaps. Without a tracked list, errata
slip through during onboarding and reappear months later as mysterious
deadlocks. See `AGENT_WHITEBOARD.md` 2026-04-22 and LL Entry 36 for
the rot pattern this document prevents.

---

## How to use this document

1. **Before adding any `PICO_…` or RP2350-specific compile flag to
   `CMakeLists.txt`**, scan this document for errata whose workaround
   involves that flag. If the flag is here, add an inline comment
   referencing this file. If not, verify you understand why the flag
   is needed and whether it has errata implications.

2. **When a peripheral misbehaves on RP2350 in a way that doesn't
   reproduce on a different chip**, grep this document by silicon
   block (SIO, QMI, XIP, DMA, …). The observable-symptom column is
   meant to match field behavior quickly.

3. **When a new datasheet revision or pico-sdk release ships**, run
   the maintenance procedure (see "Maintenance" section at the bottom)
   and update the headers at the top.

4. **When a new erratum is discovered by us** (not from the official
   datasheet), add it with ID prefix `R-` (for "RocketChip") and
   document as a candidate for upstream report.

---

## Schema

Each actively-tracked erratum is documented with the following fields.
The schema itself is load-bearing — if a field doesn't apply, write
"N/A" explicitly rather than omitting it, so readers know the question
was asked.

| Field | Purpose |
|---|---|
| **ID** | `E1`, `E2`, … for datasheet-documented; `R-N` for our discoveries |
| **Name** | One-line title from the datasheet (or our coinage for `R-` entries) |
| **Datasheet ref** | Section / page in the RP2350 datasheet |
| **Affected steppings** | A0 / A1 / A2 / A3 / A4 / "all" |
| **Our stepping** | Whether we're in the affected set (currently A2) |
| **Silicon block** | IP block or subsystem |
| **Description** | 2-3 sentences of what's broken and when |
| **Trigger conditions** | *All* mentioned trigger paths, not just the primary one |
| **Observable symptom** | How do I recognize this in the wild? |
| **SDK status** | `transparent` / `flag-gated` / `acknowledged-only` / `none` |
| **Our status** | `none-needed` / `SDK-handles` / `workaround-applied` / `gap` / `unverified` |
| **Workaround reference** | `file:line` in our tree, or SDK path, or "N/A" |
| **Fixed in stepping** | Silicon stepping fixed, if any — matters at chip upgrade |
| **Notes** | Incidents, dates, linked whiteboard/LL entries, upstream issues |

For errata that are `none-needed` (we don't use the affected silicon
block), we use the abbreviated **Not-Applicable Table** at the bottom
rather than a full row — keeps the active-risk portion of the document
readable.

---

## Silicon stepping reference

**Our hardware:**
- **Vehicle — Adafruit Feather RP2350 HSTX:** RP2350**A** package (QFN-60), stepping A2
- **Station — Adafruit Fruit Jam:** RP2350**B** package (QFN-80), stepping A2

**Stepping claim source:** product pages, whiteboard notes. **Open item:**
document a procedure for reading the chip's stepping register directly
to self-verify (deferred from this sweep — see "Open items").

**A4 silicon is now shipping on new Adafruit Feather HSTX boards** (as
of 2026-03-20 per Adafruit product page revision history). A4 "fixes
E9 and others." If we buy replacement boards, they will likely be A4 —
**the `Affected steppings` and `Our stepping` columns in this document
must be re-checked at that point**, because a non-trivial number of
errata rows move from "affected" to "fixed-in-A3" or "fixed-in-A4" on
that silicon. Fruit Jam remains A2-only as of 2026-04-22.

**Procurement note — prefer A4 silicon on future purchases.** On A4
our active-attention errata list shrinks from {E2, E12} down toward
{E2} alone (E9 fixed, E11 still SDK-handled same as A2, E12 at least
partially mitigated). The only remaining active risk on A4 would be
E2's picotool/SWD gap cases. For any multi-board order, specifically
request or verify A4 stepping before accepting. For mixed-stock (some
A2, some A4), label the boards physically so we know which chip has
which errata profile during debug.

**RP2350A vs RP2350B:** different *package* (QFN-60 vs QFN-80, different
pin-out), same silicon die and therefore same errata. An erratum in the
SIO block affects both; a pinout-specific note may differ. Only one
erratum currently differentiates by package: **E3** (QFN-60 only).

---

## Summary — Our applicable errata

Of 28 documented errata, three require active attention from our code,
one is fully handled by the SDK, and 24 are not applicable:

| ID | Silicon block | Our status | One-line |
|---|---|---|---|
| **E2** | SIO spinlocks | workaround-applied (gap cases) | Spinlock mirror writes — known tonight's boot-deadlock source |
| **E9** | GPIO | audited, low residual risk (see per-pin audit below) | Bank 0 pad leakage when input-enabled in undefined voltage region |
| **E11** | XIP cache | SDK-handles | Cache clean-by-set/way modifies dirty line tag |
| **E12** | USB | unverified | USB status signal synchronization (datasheet excerpt truncated — needs re-read) |

The remaining 24 errata are listed in the **Not-Applicable Table** at
the bottom, each with the one-line reason we're not at risk.

---

## Active Erratum Rows

### E2 — SIO SPINLOCK writes mirrored at +0x80 offset

| Field | Value |
|---|---|
| **ID** | E2 |
| **Name** | SIO SPINLOCK writes mirrored at +0x80 offset (writes to new SIO registers at offsets 0x128-0x17c alias the spinlock registers at 0x108-0x17c) |
| **Datasheet ref** | Appendix E, page 1373 |
| **Affected steppings** | A2, A3, A4 (all shipped silicon) |
| **Our stepping** | A2 — **affected** |
| **Silicon block** | SIO — spinlock hardware |
| **Description** | SIO address decoder detects writes to spinlocks by decoding bit 7 of the address. Writes in the range 0x128-0x17c (new RP2350 registers: Doorbells, PERL_NONSEC, RISC-V soft IRQ, RISC-V MTIME, TMDS encoder) are spuriously detected as writes to the corresponding spinlock address 128 bytes below (range 0x108-0x17c). Writing to these high-addressed SIO registers silently sets the corresponding lock to unclaimed. Only affects writes to spinlock registers; reads are correctly decoded. |
| **Trigger conditions** | <ul><li>**Primary:** any code writing to SIO registers at offsets 0x128-0x17c while hardware spinlocks are in use. The SDK's default on RP2350 uses *software* spinlocks (LDAEXB/STREXB on SRAM locations) instead of hardware spinlocks. But the SW-spinlock path itself can interact with boot-time fault paths in a way that deadlocks Core 0 in `spin_lock_blocking` on the timer spinlock (ID 10) before reaching `main()`.</li><li>**Second trigger path (2026-04-22 tonight, R-1):** `picotool info -f --ser <serial>` reboots via USB vendor command into BOOTSEL + back. E2 fired on next boot despite `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` applied. User had to replug to recover. Mechanism unclear — may be state surviving the BOOTSEL transition, or a reset path not going through normal SDK runtime_init. Candidate for upstream report.</li><li>**Third trigger path (R-2):** SWD `monitor halt` during an in-progress `spin_lock_blocking` wait appears to lose the STREXB exclusive-monitor reservation. On `monitor resume`, STREXB fails forever, spinlock never acquires, eventually HardFault with same PC/LR signature. Candidate for upstream report.</li></ul> |
| **Observable symptom** | Core 0 deadlock at `spin_lock_blocking`, eventual HardFault with **PC=0xeffffffe, LR=0xffffffe9** — canonical lockup signature. USB CDC never enumerates. Core 1 stuck at bootrom `0x000000da` (waiting on a Core-0 cross-core flag that never comes). LED patterns may or may not be visible depending on how early the deadlock fires. |
| **SDK status** | `flag-gated`: SDK 2.2.0 defaults to SW spinlocks on RP2350 (partial mitigation), but does NOT set `PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1` by default. This flag disables the EXTEXCLALL bit in `M33_ACTLR` per-core at runtime_init, closing the primary boot-time trigger path. `acknowledged-only` for the picotool/SWD trigger paths — not mitigated. |
| **Our status** | `workaround-applied` for primary boot-time path; `gap` for picotool and SWD paths. |
| **Workaround reference** | `CMakeLists.txt:274` — `add_compile_definitions(PICO_SW_SPIN_LOCKS_NO_EXTEXCLALL=1)` under `PICO_PLATFORM STREQUAL rp2350*` guard. For the SWD gap: `AGENT_WHITEBOARD.md` 2026-04-22 documents the workflow ("follow `monitor halt` with `monitor reset halt` + `load` + `monitor resume`, not bare `monitor resume`"). For the picotool gap: no workaround yet other than power-cycle on recurrence. |
| **Fixed in stepping** | Not fixed in any shipping silicon (A2, A3, A4 all affected). Per pico-sdk issues #2495 and #2706. |
| **Notes** | Discovered 2026-04-22 during RocketChip onboarding: a tiny code diff in `ao_rcos.cpp` shifted the ELF enough to deterministically trigger the boot-time path. Trigger paths R-1 (picotool) and R-2 (SWD halt) are our discoveries — candidates for upstream pico-sdk issue reports. |

---

### E9 — GPIO Bank 0 pad leakage with input-enable in undefined voltage region

| Field | Value |
|---|---|
| **ID** | E9 |
| **Name** | Increased leakage current on Bank 0 GPIO when pad input enabled (in undefined logic region between V_L and V_H) |
| **Datasheet ref** | Appendix E, page 1366-1367 |
| **Affected steppings** | A2 |
| **Our stepping** | A2 — **affected** |
| **Silicon block** | GPIO (bank 0 pads 0-47 only) |
| **Description** | When a Bank 0 GPIO pad is configured as input-enabled (IE=1), output-disabled, isolation-clear, and the pad voltage sits in the undefined logic region (~0.8V-2.0V), leakage current can latch the pad at ~2.2V. Peak current ~30µA. Only ≤8.2kΩ external pull-down or a low-impedance driver overcomes this. Adafruit's guidance on both Feather HSTX and Fruit Jam learn guides matches: if a pull-down is used, it must be ≤8.2kΩ. A4 silicon fixes this entirely; current Adafruit Feather HSTX stock as of 2026-03-20+ is A4 and unaffected. |
| **Trigger conditions** | Bank 0 pad with **IE=1**, no active driver, no ≤8.2kΩ external pull, idle voltage in the 0.8V-2.0V undefined region. The load-bearing condition is **IE=1** — at reset, `PADS_BANK0_GPIO*_IE_RESET = 0` (pico-sdk `hardware/regs/pads_bank0.h`). Pads we never configure stay at IE=0 and are **not E9-vulnerable regardless of external pulls**. `gpio_init()` and `gpio_set_function()` both explicitly set IE=1. |
| **Observable symptom** | Floating GPIO pad measures ~2.2V instead of 0V / rail voltage. Input read returns HIGH when expected LOW. ADC reads show incorrect values on an ADC-configured pin. A weak external pull-down (say 100kΩ) fails to hold the pad low. |
| **SDK status** | `acknowledged-only`. The SDK does not auto-clear IE on unused pads — `IE=0` comes from the silicon reset default, not SDK action. ADC channels clear IE per datasheet §12.4.3 during ADC operation (partial mitigation for ADC-active pins). The SDK's `gpio_init()` and `gpio_set_function()` affirmatively set IE=1. |
| **Our status** | `workaround-applied` by-construction — per-pin audit 2026-04-22 (see table below) finds zero pins with actual E9 exposure that affects functionality. Two pins (SPI MISO on each board) are technically exposed during CS-high idle periods but the SPI peripheral doesn't sample during those windows, so leakage-induced voltage has no functional impact. Every other pin is either (a) left at reset IE=0 by our firmware never touching it, (b) actively driven by us or an external device, or (c) externally pulled by Adafruit board-level resistors. |
| **Workaround reference** | Per-pin audit tables below. If a new pin is configured in the future as a floating input (IE=1, no driver, no pull), a new row must be added to the at-risk table and either (a) an external ≤8.2kΩ pull added, (b) the input disabled when not actively reading, or (c) the pin moved to an A4-silicon board. |
| **Fixed in stepping** | A3 |
| **Notes** | Adafruit product page for the Feather HSTX notes A4 now shipping (post-2026-03-20) which fixes E9. Fruit Jam is still A2. If we upgrade to A4 Feathers, this whole row becomes historical (move to Not-Applicable). Until then, the audit tables below are the authoritative check. |

**Per-pin E9 audit — Feather RP2350 HSTX (vehicle, RP2350A QFN60, GPIOs 0-29)**

Reset default `IE=0` holds for every pin our firmware doesn't touch.
Every pin below traces the "what sets IE=1" → "what protects the pad"
chain. Sources: our `src/`, schematic via Adafruit-Feather-RP2350-PCB
GitHub repo (R-designators R7/R15/R16/R17 verified via schematic fetch
2026-04-22).

| GPIO | Our config | IE after config | Idle driver / pull | E9 exposure | Notes |
|---|---|---|---|---|---|
| 0 | UART0 TX | 1 | RP2350 actively drives | None | — |
| 1 | UART0 RX | 1 | PA1010D GPS push-pull drives NMEA | None | GPS-module-attached is assumed; if GPS detached, see open items |
| 2 | I2C1 SDA | 1 | 10kΩ to 3V3 via R16 (Feather on-board) + STEMMA QT module pulls | None | Always pulled — E9-safe |
| 3 | I2C1 SCL | 1 | 10kΩ to 3V3 via R17 + STEMMA QT pulls | None | Always pulled |
| 4, 5 | Not configured | 0 (reset default) | — | **None by IE=0** | Unused by our firmware |
| 6 | Radio DIO0 input (pulls disabled) | 1 | SX1276 push-pull drives DIO0 in all modes per SX1276 datasheet | None | Radio-always-attached assumed |
| 7 | LED output | 1 (but OD=0) | We drive; off-state the 10kΩ R7 + red LED acts as weak pull to GND via ~1.8V forward drop | None | Pin is output-driven. If ever reconfigured as input, R7+LED ~10kΩ is above Adafruit's 8.2kΩ threshold — treat as at-risk in that case |
| 8 | PSRAM CS (XIP_CS1) | N/A — QSPI macro | 10kΩ to 3V3 via R15 | None | QSPI pads are datasheet-excepted from E9 |
| 9 | Not configured | 0 | — | **None** | — |
| 10 | Radio CS output | 1 (OUT) | We drive | None | — |
| 11 | Radio RST output | 1 (OUT) | We drive | None | — |
| 12 | Pyro drogue backup (PIO) | 1 (PIO OUT) | PIO drives LOW when disarmed | None | — |
| 13 | Pyro main backup (PIO) | 1 (PIO OUT) | PIO drives LOW | None | — |
| 14-19 | Not configured (HSTX pins, not used) | 0 | — | **None** | HSTX pads exist on silicon, not used by us |
| **20** | SPI0 MISO | 1 (FUNC_SPI) | SX1276 drives CS-low; tri-stated CS-high | **Exposed CS-high, harmless** | SPI peripheral gates sampling on CS. Leakage-latched voltage is never read by the peripheral. No mitigation needed. |
| 21 | NeoPixel (PIO OUT) | 1 (PIO OUT) | PIO drives | None | — |
| 22 | SPI0 SCK | 1 | We drive | None | — |
| 23 | SPI0 MOSI | 1 | We drive | None | — |
| 24-29 | Not configured | 0 | — | **None** | Unused ADC/digital pins |

**Feather verdict:** Zero functionally-exposed pins. Safe without action.

**Per-pin E9 audit — Fruit Jam (station, RP2350B QFN80, GPIOs 0-47)**

| GPIO | Our config | IE after config | Idle driver / pull | E9 exposure | Notes |
|---|---|---|---|---|---|
| 0 | Not configured (boot button) | 0 | — | **None** | Button has onboard pull-up regardless |
| 1, 2 | Not configured (USB host) | 0 | — | **None** | USB PHY macro |
| 3 | Not configured (ESP32-C6 ACK) | 0 | — | **None** | We don't use ESP32-C6 today |
| 4 | Not configured (Button2) | 0 | — | **None** | Onboard pull-up on button |
| 5 | Radio DIO0 input (shared with Button3) | 1 | Button3 pull-up + SX1276 push-pull drives | None | Dual protection (pull + driver) |
| 6 | Radio RST output | 1 (OUT) | We drive | None | — |
| 7-9 | Not configured | 0 | — | **None** | — |
| 10 | Radio CS output | 1 (OUT) | We drive | None | — |
| 11-19 | Not configured (USB host 5V, HSTX DVI pins) | 0 | — | **None** | HSTX unused by us |
| 20 | I2C0 SDA | 1 | STEMMA QT pull-up (value 2.2kΩ or 10kΩ — Adafruit docs ambiguous; either is ≤8.2kΩ safe) | None | Always pulled |
| 21 | I2C0 SCL | 1 | STEMMA QT pull-up | None | Always pulled |
| 22 | Peripheral RESET output | 1 (OUT) | We drive HIGH | None | Shared RESET for ESP32-C6 + DAC |
| 23-27 | Not configured (I2S DAC + DAC IRQ) | 0 | — | **None** | Audio DAC unused; see open items if we add audio |
| **28** | SPI1 MISO | 1 (FUNC_SPI) | Radio drives CS-low; tri-stated CS-high | **Exposed CS-high, harmless** | Same rationale as Feather GPIO 20 |
| 29 | LED output | 1 (OUT) | We drive | None | Active-low LED |
| 30 | SPI1 SCK | 1 | We drive | None | — |
| 31 | SPI1 MOSI | 1 | We drive | None | — |
| 32 | NeoPixel (PIO OUT) | 1 (PIO OUT) | PIO drives | None | 5-LED NeoPixel chain |
| 33 | Not configured (undocumented on Adafruit pinout page) | 0 | — | **None by IE=0** | Schematic check deferred — if routed to something we use later, re-assess |
| 34-39 | Not configured (SD card SPI + SDIO) | 0 | — | **None** | SD card unused |
| 40-45 | Not configured (ADC pins) | 0 | — | **None** | Unused ADC inputs |
| 46 | Not configured (ESP32-C6 CS) | 0 | — | **None** | Unused today |
| 47 | PSRAM CS (XIP_CS1) | N/A — QSPI macro | Pulled by QMI | None | QSPI macro E9-excepted |

**Fruit Jam verdict:** Zero functionally-exposed pins. Safe without action.

**Current ADC usage — internal temperature sensor only.** We call `adc_init()` + `adc_set_temp_sensor_enabled(true)` + `adc_select_input()` in `src/drivers/mcu_temp.cpp` against the die-internal temperature sensor channel (ADC input 4 on Feather/RP2350A, ADC input 8 on Fruit Jam/RP2350B). These channels are **not Bank 0 GPIO pads** — they're dedicated internal analog sources. No GPIO-level E9 concern from `mcu_temp`. The `adc_init()` call itself does not touch GPIO pad IE bits; only `adc_gpio_init()` (which we do not call) configures external ADC-capable pins.

**Audit triggers for re-assessment.** This audit must be re-run if any of the following change:

- We add a new `gpio_init()`, `gpio_set_function()`, or PIO pindir-input call for a previously unconfigured pin. New pin must be added to the at-risk table with driver/pull analysis.
- We start using **external-pin ADC** via `adc_gpio_init()` (battery monitor, external sensor — not yet implemented but planned per battery-monitoring note in repo). External ADC pins (Feather GPIO 26-29 / Fruit Jam GPIO 40-45) become IE=1 during ADC use; leakage could distort readings if the pin floats between reads. Per datasheet §12.4.3, the SDK clears IE during `adc_read()` on external ADC pins, but the between-reads window is not protected by SDK. When added, either (a) tie the unused-read period to an external pull, or (b) ensure the voltage divider / sensor always drives the pin low-impedance.
- We start driving HSTX (Fruit Jam DVI output) or I2S DAC. Those pads become IE=1 and need driver/tri-state analysis.
- A Fruit Jam firmware path starts using the ESP32-C6 (GPIO 3, 22, 46). GPIO 3 ACK specifically is a tri-state-when-C6-off case worth checking.
- We remove a board-level protection (e.g., lose STEMMA QT module → the STEMMA connector pulls go away, and only the on-board Feather R16/R17 pull-ups remain — Fruit Jam likely has on-board pull-ups too but schematic verification would be needed).

---

### E11 — XIP cache clean by set/way modifies tag of dirty lines

| Field | Value |
|---|---|
| **ID** | E11 |
| **Name** | XIP cache clean by set/way operation modifies tag of dirty lines |
| **Datasheet ref** | Appendix E, page 1374 |
| **Affected steppings** | A2, A3, A4 (all shipped silicon) |
| **Our stepping** | A2 — **affected** |
| **Silicon block** | XIP (execute-in-place cache) |
| **Description** | Cache clean-by-set/way maintenance, when applied to a dirty line, both writes the dirty data downstream (correct) AND erroneously updates the line's tag to the address bits 25-13 of the maintenance write that initiated the clean (incorrect). This breaks the cache's address-to-tag invariant. Subsequent reads from the originally cached address can hit the now-mistagged line and return stale data from the wrong downstream window (e.g., PSRAM data returned for a flash address, or vice versa). |
| **Trigger conditions** | XIP cache clean by set/way on dirty lines when multiple QMI windows are in use and an address in one window aliases in the cache to data previously cached from another window. Spurious cache hits possible after the clean operation. |
| **Observable symptom** | Incorrect data reads from addresses in QMI windows after a cache-clean operation. In our context: if flash_flush writes to PSRAM dirty lines and we've got cached PSRAM data at a window address that aliases. No observable symptom in practice with our current usage (single flash window). |
| **SDK status** | `transparent`: the SDK's `xip_cache_clean_all()` function implements the workaround by choosing a maintenance-write address that can't alias with cached QMI data (upper 16 kB of the maintenance space). Calling the SDK API gets the fix for free. |
| **Our status** | `SDK-handles` — we only call `xip_cache_clean_all()` from the SDK, never construct raw set/way operations ourselves. |
| **Workaround reference** | `src/logging/flash_flush.cpp:341` calls `xip_cache_clean_all()` (SDK function with workaround). |
| **Fixed in stepping** | Not fixed in any shipping silicon. SDK-only fix. |
| **Notes** | If we ever add lower-level cache maintenance (per-line clean, specific range invalidate), we must re-verify those paths against the erratum — only `xip_cache_clean_all()` is known-safe. |

---

### E12 — USB: Inadequate synchronisation of USB status signals

| Field | Value |
|---|---|
| **ID** | E12 |
| **Name** | USB: Inadequate synchronisation of USB status signals |
| **Datasheet ref** | Appendix E, page 1375 |
| **Affected steppings** | A2, A3 (mitigated on A3), A4 |
| **Our stepping** | A2 — **affected** |
| **Silicon block** | USB |
| **Description** | **INCOMPLETE.** The extraction pass truncated the datasheet description — needs a targeted re-read of pages 1375-1377 for full description, trigger conditions, observable symptom, and workaround. |
| **Trigger conditions** | *Unverified — needs datasheet re-read.* |
| **Observable symptom** | *Unverified.* Historical LL Entries 12, 15, 22 document USB CDC enumeration and disconnect symptoms that may or may not be related to E12. |
| **SDK status** | *Unverified — needs datasheet re-read to check for SDK references.* |
| **Our status** | `unverified` |
| **Workaround reference** | *Pending E12 re-read.* |
| **Fixed in stepping** | A3 partial mitigation, A4 presumed better. |
| **Notes** | We use USB CDC continuously on both boards. E12 is a real risk until characterized. **Open item:** targeted re-read of datasheet pages 1375-1377 to complete this row. Suspected related prior incidents: LL Entry 12 (USB CDC init order), LL Entry 22 (Core 1 IMU rate degradation after USB reconnect), LL Entry 15 (CLI stops on terminal connect). These may be explainable by our own init-order bugs, or there may be underlying E12 residue. |

---

## Not-Applicable Table — 24 errata

(E9 moved to active rows after the per-pin audit 2026-04-22 found one
low-residual-risk pin category worth tracking — see E9 row above.)

For each, the one-line reason we're not at risk. If any of these conditions
change (we start using the affected silicon block, switch steppings, change
security posture), the row must be re-promoted to the active matrix above.

| ID | Silicon block | Why not applicable to us |
|---|---|---|
| E1 | SIO interpolator | We don't use the interpolator. No `interp_*` calls in `src/`. |
| E3 | ACCESSCTRL (QFN-60) | Feather is QFN-60 (affected package), but we don't use Non-secure mode or PADS register access control. All our pad access is from Secure. |
| E4 | Hazard3 (RISC-V) | We build ARM only (`rp2350-arm-s`), never RISC-V. Hazard3-specific errata N/A. |
| E5 | DMA CHAIN_TO + ABORT | We don't use DMA. No `dma_channel_*` claims in `src/`. |
| E6 | Hazard3 PMPCFG | RISC-V only — N/A. |
| E7 | Hazard3 mstatus.mie | RISC-V only — N/A. |
| E8 | DMA zero-length CHAIN_TO | No DMA use — N/A. |
| E10 | Bootrom UF2 drag-drop with partition table | We don't use partition tables. Standard UF2 flashing. Handled by picotool regardless. |
| E13 | Bootrom invalid IMAGE_DEF before valid | We have a single valid IMAGE_DEF. SDK's IGNORED item handling is used. |
| E14 | Bootrom connect_internal_flash() CS1 pin | We don't call this function. Flash connected to pin 0 by default on Adafruit boards. |
| E15 | OTP otp_access() permissions | We don't program or read OTP. |
| E16 | OTP USB_OTP_VDD disruption | No OTP use. Requires physical attack. |
| E17 | OTP guarded ECC read on paired row | No OTP use. |
| E18 | Bootrom FLASH_PARTITION_SLOT_SIZE ECC | We don't program this OTP row. |
| E19 | Bootrom reboot hang with FRCE_OFF bits | We don't use WATCHDOG or POWMAN boot paths with non-default FRCE_OFF. Standard reboot only. |
| E20 | Bootrom reboot() glitch attack | Physical attack. We don't ship secure-boot firmware; no adversary model includes physical glitch attacks. |
| E21 | OTP glitch attack in BOOTSEL | Physical attack, same as E20. |
| E22 | Bootrom "lollipop" block loop | Block loops are SDK-generated; SDK doesn't produce lollipops. |
| E23 | PICOBOOT GET_INFO PACKAGE_SEL | We don't call GET_INFO from firmware. picotool already handles the workaround. |
| E24 | Bootrom signature-check bypass glitch attack | Physical attack on secure boot. Not our threat model. |
| E25 | LOAD_MAP non-word sizes | SDK uses word-sized, word-aligned linker sections. N/A. |
| E26 | RCP delay side-channel | We don't use the Redundancy Coprocessor. |
| E27 | BUS_PRIORITY wrong-wire | We don't set bus priorities (`BUSCTRL`). Default round-robin arbitration. |
| E28 | OTP lock-word key protection | No OTP use. |

---

## Maintenance

**When to re-check:**

- **Every pico-sdk release.** Read the release notes. Grep the changed
  files in `src/rp2_common/hardware_sync/`, `src/rp2_common/hardware_xip_cache/`,
  `src/rp2_common/hardware_dma/`, `src/rp2350/hardware_regs/`, and
  `src/rp2_common/pico_bootrom/` for new errata references. New errata
  mitigations frequently ship between SDK releases.
- **Every datasheet revision.** Raspberry Pi occasionally updates the
  RP2350 datasheet; the errata appendix grows. Download the latest from
  the RP2350 product page, diff against our referenced revision, and
  add rows for any new errata.
- **Every 6 months, unconditionally.** Calendar-driven check. Even if no
  SDK release or datasheet update has triggered an ad-hoc check, time-
  based review catches quiet slippage.

**What to record after each check:**
- Update the "Last datasheet revision checked" header at the top.
- Update the "Last pico-sdk version checked" header.
- For any new errata found, add a row with `unverified` status and
  flag in `AGENT_WHITEBOARD.md` for triage.

**What to watch upstream:**
- [pico-sdk releases](https://github.com/raspberrypi/pico-sdk/releases)
- [pico-sdk issues tagged "errata"](https://github.com/raspberrypi/pico-sdk/issues?q=errata) (if they tag consistently)
- Raspberry Pi RP2350 product page for datasheet revision bumps.

---

## Open items / deferred

Identified during the initial sweep (council review 2026-04-22):

1. **E12 datasheet re-read.** Pages 1375-1377 need targeted extraction
   to fully populate the E12 row. Until then, the USB erratum is
   `unverified` even though USB is a daily-used peripheral on both
   boards. The historical LL entries 12/15/22 (USB CDC quirks) may
   or may not be explained by E12 — worth knowing.

2. **Silicon-stepping read procedure.** Currently claim A2 based on
   Adafruit product pages. Adafruit now ships A4 on the Feather HSTX
   (post 2026-03-20) which fixes E9 and others, so self-verification
   matters more than it did a few weeks ago. Need a documented
   procedure for reading the chip's stepping register.

3. **R-1 and R-2 upstream reports.** The E2 picotool-reboot trigger
   path (R-1) and SWD halt-during-spinlock trigger path (R-2) are not
   in the datasheet or the pico-sdk issues we've seen. Candidates for
   filing as new issues citing the workaround's gap cases.

4. **Regression test per erratum (where feasible).** For errata with
   observable symptoms (E2 deadlock signature, E11 cache range), a
   smoke test or firmware assertion that fails loudly if the
   workaround's preconditions are violated. Converts "documented
   assumption" to "enforced assumption." Council-flagged as valuable.

5. **Pre-commit enforcement of doc update on CMake flag changes.**
   Any diff touching RP2350-specific `add_compile_definitions` should
   also touch this document. Mechanical gate instead of honor-system
   (per LL Entry 36). Tooling-cost vs value is a later scope decision.

---

## See also

- `AGENT_WHITEBOARD.md` 2026-04-22 errata-sweep entry — tracking item
- `CMakeLists.txt:274` — E2 primary-path workaround applied in-tree
- `src/logging/flash_flush.cpp:341` — E11 workaround via SDK API call
- `.claude/LESSONS_LEARNED.md` Entry 36 — "infrastructure vs artifact"
  discipline; this document is structured as infrastructure
- `standards/CODING_STANDARDS.md` RP2350 Platform Constraints section —
  one-line pointer back to this document
