# In-flight flash writes: what other flight software does

- **Date:** 2026-11-21
- **Author:** Goddard
- **Scope:** How hobby and high-power rocketry altimeters (Altus Metrum AltOS, Featherweight Blue Raven, Eggtimer Quantum), autopilots (ArduPilot, PX4), FPV flight controllers (Betaflight, INAV) and the RP2350 vendor SDK (pico-sdk) handle flash writes and erases while flying, plus the Tiny 2350 flash chip timing. Feeds the flash rule in plan §19.1. Formal standards (CCSDS, ECSS, NASA, ED-112A/DO-178C) are Duke's lane and are not covered. cFS and F Prime were not researched (scope cut); nothing is claimed about them.
- **Repo checked:** `origin/main` at `a6d5d4b9` (fetched 2026-11-21), for the Rocket Chip files cited in §5.
- **Sources checked:** Source code was read from the live repos (master/main branches) on 2026-11-21, and every quoted line below was re-checked that day. AltOS was read from the Debian tarball `altos_1.9.22.orig.tar.gz` because git.gag.com was returning errors.
- **Status:** Frozen once committed. A later report replaces this one.

Every finding is labeled. **[Fact]** means it was read from code or a cited source. **[Inference]** means it is Goddard's judgment or arithmetic and has not been measured on hardware.

## Summary

- **[Fact]** Every logger surveyed writes its log in flight. What they restrict is erase, and in PX4's case parameter saves to internal flash. None has a "no flash writes while armed" rule for a log.
- **[Inference]** The common pattern is: erase on the ground (manually, or at power-up on the pad), append-only writes into already-erased space in flight, stop when full rather than wrap over the current flight, and erase again only when the user chooses (AltOS: manual) or automatically at power-up (Blue Raven: oldest bank).
- **[Fact]** The closest analogue to Rocket Chip's XIP problem is ArduPilot's parameter storage in the STM32's own flash: log-structured writes are allowed while armed, sector erases only while disarmed, because "erase will lock the flash and prevent code execution".
- **[Fact]** PX4 is stricter for internal flash: it defers parameter saves until disarmed, because "writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)".

## 1. Hobby and high-power rocketry altimeters

### Altus Metrum AltOS (TeleMega, EasyMega, TeleMetrum)
Source: https://deb.debian.org/debian/pool/main/a/altos/altos_1.9.22.orig.tar.gz

- **[Fact]** Logging starts at boost detection: `src/kernel/ao_flight.c:242-244`, `/* start logging data */` then `ao_log_start();`.
- **[Fact]** Pre-launch data is kept: the log task first writes its sample ring (`src/kernel/ao_log_mega.c:68`, "Write the whole contents of the ring to the log").
- **[Fact]** Logging stops at landing: `src/kernel/ao_log_mega.c:168`, `if (ao_log_state == ao_flight_landed)` followed by `ao_log_stop()`.
- **[Fact]** Logging stops when the slot is full: `src/kernel/ao_log.c:162`, `if (ao_log_current_pos >= ao_log_end_pos && ao_log_running)` followed by `ao_log_stop()`.
- **[Fact]** Each record carries a checksum, set on write and checked on read: `src/kernel/ao_log.c:159-160` (`log->csum = ao_log_csum((uint8_t *) log);`) and `:178`.
- **[Fact]** Erase is manual, from the ground station. The ChangeLog has the commit "altos: Require manual flight erasing." The manual (https://altusmetrum.org/AltOS/doc/altusmetrum.html, v1.9.22.2) says: "With version 0.9 or newer firmware, you must erase flights in order for the space they consume to be reused by another flight. This prevents accidentally losing flight data if you neglect to download data before flying again. Note that if there is no more space available in the device, then no data will be recorded during the next flight."
- **[Fact]** Full storage does not affect the flight. The manual's beep table: "Storage Full ... This will not prevent the flight computer from safely controlling the flight or transmitting telemetry signals, but no record of the flight will be stored in on-board flash."
- **[Fact]** Torn erases are handled by erasing back to front: `src/kernel/ao_storage.c:129-132`, "Start at the end of the area to erase so that the last block cleared is the first block; this will ensure that partially erased flight logs still appear in the list and can be re-erased."
- **[Fact]** AltOS logs to a separate SPI flash chip (for example the M25 driver, `src/drivers/ao_m25.c`), not to XIP flash. One automatic erase exists, on the ground: at boot, `ao_log_scan` erases log slots if the configured flight-log size changed (`src/kernel/ao_log.c`, around lines 284-291).

### Featherweight Blue Raven
Source: User's Guide, May 12 2026, https://www.featherweightaltimeters.com/uploads/1/0/9/5/109510427/blue_raven_users_manual_2026may12.pdf

- **[Fact]** "There is flash memory space in the Blue Raven for two flights. If there is no free space available when the unit is powered up, the oldest flight data is erased."
- **[Fact]** "If both memory banks are full of flight data, the oldest one will be erased automatically to make room for the new flight. This can take up to a few minutes if a long flight is being erased."
- **[Fact]** The red LED "turns on when the on-board flash memory is active, for example right after power-up when a previous flight's data is being erased ... and during the flight when flight data is being written to flash memory."
- **[Inference]** So Blue Raven writes in flight and erases at power-up on the pad. It erases without checking for a download, which is weaker than an "erase only after a verified offload" rule.

### Eggtimer Quantum
Source: User's Guide rev 1.09G, https://eggtimerrocketry.com/wp-content/uploads/2024/02/Eggtimer-Quantum-1_09G.pdf

- **[Fact]** Session logic: "When you're sitting on the pad or launched and have not yet reached LDA, the sampling rotates through those memory locations, saving the last reading < 3' AGL as the "start of flight". Once you hit LDA, the recording continues until either 1) the End of Flight benchmark (your rocket hasn't moved for at least 5 seconds) is reached, or 2) the memory wraps around to the start of flight location. In either case, recording stops there (it won't overwrite valid ascent data), but this does NOT affect any pending deployment events".
- **[Inference, not verified]** The original notes say the Quantum logs to a CAT24C512 EEPROM (byte-writable, no erase step), which would make it a session-logic analogue only, not a flash analogue. The 1.09G guide does not name the memory part, and this was not confirmed from another source.

### Not checked
Featherweight Raven, Missile Works RRC3/RRC2, and TeleMetrum hardware documents beyond the AltOS manual.

## 2. ArduPilot and PX4

### ArduPilot block-flash logger (external SPI dataflash)
Source: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/AP_Logger_Block.cpp

- **[Fact]** A full chip erase is refused while armed: `EraseAll()` returns early if `hal.util->get_soft_armed()`, with the comment "do not want to do any filesystem operations while we are e.g. flying" (line 274).
- **[Fact]** But it erases ahead during logging: "when starting a new sector, erase it" (line 108), then `SectorErase(get_block(df_PageAdr));` (line 122). It wraps over old logs and stops only if it would erase its own current log's headers (`chip_full = true`, line 119).
- **[Inference]** This is safe for ArduPilot only because the dataflash is a separate SPI chip and code runs from the MCU's internal flash. It does not transfer to an XIP-only board.

### ArduPilot parameter storage in internal MCU flash (the XIP-style analogue)
Sources: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_FlashStorage/AP_FlashStorage.h , https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_FlashStorage/AP_FlashStorage.cpp , https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/Storage.cpp

- **[Fact]** `AP_FlashStorage.h` header: "The basic methodology is to use a log based storage system over two flash sectors", with design elements including "erase of sectors only called on init, as erase will lock the flash and prevent code execution", "write using log based system", and "assumes flash that erases to 0xFF and where writing can only clear bits".
- **[Fact]** `Storage.cpp`: `_flash_erase_ok()` returns `!hal.util->get_soft_armed()` with the comment "only allow erase while disarmed" (line 448).
- **[Fact]** `Storage.cpp` `_timer_tick()` writes dirty lines while armed: "write out the first dirty line. We don't write more than one to keep the latency of this call to a minimum" (from line 257).
- **[Fact]** `Storage.cpp`: "a sector erase stops the whole MCU so set up a long expected delay", then `EXPECT_DELAY_MS(1000);` (lines 424-425).
- **[Fact]** `AP_FlashStorage.cpp`: when a sector fills and switching needs an erase, `if (!flash_erase_ok())` makes the write fail and defer rather than erase while armed (lines 199 and 457).
- **[Inference]** So ArduPilot programs while armed, from one writer, in small chunks, and erases only when disarmed. That is the same shape as the proposed §19.1 rule.

### PX4
- **[Fact]** https://github.com/PX4/PX4-Autopilot/blob/main/src/lib/parameters/autosave.cpp lines 89-90: "In case we save to FLASH, defer param writes until disarmed, as writing to FLASH can stall the entire CPU (in rare cases around 300ms on STM32F7)". If armed, it reschedules 1 s later.
- **[Inference]** This is the closest thing to Rocket Chip's original "no flash writes while armed" rule, but it applies to parameter saves (which may erase), not to logs.
- **[Fact]** https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/logger/module.yaml: `SDLOG_MODE` default is "0: when armed until disarm (default)". PX4 logs in flight to an SD card through a file system, not to raw NOR or XIP flash.

## 3. Betaflight and INAV Blackbox to onboard SPI flash

- **[Fact]** Betaflight `flashfs` is append-only into erased flash (https://github.com/betaflight/betaflight/blob/master/src/main/io/flashfs.c, lines 24-28): "On statup, call flashfsInit() after initialising the flash chip in order to init the filesystem. This will result in the file pointer being pointed at the first free block found, or at the end of the device if the flash chip is full." and "Flash devices require an erase before bits can return to 1."
- **[Fact]** Logging starts on arm and stops when full (Betaflight `src/main/blackbox/blackbox.c`): in `case BLACKBOX_STATE_STOPPED:` (line 2147), `if (ARMING_FLAG(ARMED)) { blackboxOpen(); blackboxStart(); }`, and "Did we run out of room on the device? Stop!" (line 2307) leads to `BLACKBOX_STATE_STOPPED`.
- **[Fact]** Erase only when not logging: Betaflight's erase-switch check (`BOXBLACKBOXERASE`, line 2153) sits inside `case BLACKBOX_STATE_STOPPED`. INAV (https://github.com/iNavFlight/inav/blob/master/src/main/fc/fc_msp.c, line 3066): `case MSP_DATAFLASH_ERASE:` calls `flashfsEraseCompletely()` only `if (blackboxMayEditConfig())`, else returns `MSP_RESULT_ERROR`; `blackboxMayEditConfig()` returns `blackboxState <= BLACKBOX_STATE_STOPPED` (INAV `src/main/blackbox/blackbox.c:635`).
- **[Fact]** Betaflight's own `MSP_DATAFLASH_ERASE` handler (`src/main/msp/msp.c:4191-4193`) calls `blackboxEraseAll()` with no armed check on those lines. Whether MSP writes are blocked while armed elsewhere was not traced (unverified).
- **[Fact]** As with ArduPilot, this is a separate SPI chip; code runs from MCU internal flash.

## 4. RP2350 vendor guidance (pico-sdk)

- **[Fact]** https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_flash/include/pico/flash.h: "Flash cannot be erased or written to when in XIP mode. However the system cannot directly access memory in the flash address space when not in XIP mode. It is therefore critical that no code or data is being read from flash while flash is been written or erased."
- **[Fact]** Same file: `flash_safe_execute` will "Execute a function with IRQs disabled and with the other core also not executing/reading flash".
- **[Fact]** Same file, escape hatches: "Set PICO_FLASH_ASSUME_CORE0_SAFE=1 to explicitly say that core 0 is never using flash. Set PICO_FLASH_ASSUME_CORE1_SAFE=1 to explicitly say that core 1 is never using flash." The user can also supply a strong definition of `get_flash_safety_helper()`.
- **[Fact]** https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/pico_multicore/include/pico/multicore.h: "When "locked out" the victim core is paused (it is actually executing a tight loop with code in RAM) and has interrupts disabled."
- **[Fact]** https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2_common/hardware_flash/include/hardware/flash.h: erase takes 4096-byte aligned sectors; program takes 256-byte aligned pages; "The only way to change a zero bit back to one is to "erase" the whole sector that the page resides in."

**What this means for Rocket Chip**
- **[Fact]** The repo uses the default `flash_safe_execute` (`src/logging/flash_flush.cpp:49, 54`; `src/calibration/calibration_storage.cpp:97, 115`), and `docs/MULTICORE_RULES.md:146` says "Always use `flash_safe_execute()` from `pico/flash.h`."
- **[Inference]** With the default helper, Core 1 is paused during every page program even if its code is in SRAM. To get the benefit of moving Core 1 to SRAM, the writer must stop locking Core 1 out (`PICO_FLASH_ASSUME_CORE1_SAFE=1` or a custom safety helper), and Core 1 must then touch nothing in flash: no const tables or rodata, string literals, vtables, libc or soft-float helpers, or IRQ handlers placed in flash.
- **[Inference]** Meanwhile the writing core runs with IRQs disabled for the whole operation, so everything on that core (flight director, watchdog kick, deploy timing, USB) is blocked for each page program.

## 5. Tiny 2350 flash chip timing

- **[Fact]** The Pimoroni Tiny 2350 schematic lists "4MB W25Q32JVUUIQ" (https://cdn-shop.adafruit.com/product-files/6248/Pimoroni_Tiny_2350_Schematic.pdf).
- **[Fact]** Winbond W25Q32JV datasheet Rev G (2018-03-27), AC characteristics p. 64 (https://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf):

  | Operation | Typical | Max |
  |---|---|---|
  | Page Program (256 B) tPP | 0.4 ms | 3 ms |
  | Sector Erase 4 KB tSE | 45 ms | 400 ms |
  | Block Erase 32 KB tBE1 | 120 ms | 1,600 ms |
  | Block Erase 64 KB tBE2 | 150 ms | 2,000 ms |
  | Chip Erase tCE | 10 s | 50 s |

  Note 2 on these values: "Value guaranteed by design and/or characterization, not 100% tested in production."
- **[Fact]** The chip also supports Erase/Program Suspend (75h) and a VCC write-inhibit threshold (VWI) at power-down (same datasheet). The SDK's `hardware_flash/flash.c` contains no reference to suspend or 75h (checked 2026-11-21); the bootrom flash routines were not checked.
- **[Inference]** These are chip times only. The SDK path adds XIP exit and re-entry, and the XIP cache is cold afterwards. Not measured.

**Duty cycle of page programs (arithmetic, not a source)**
- **[Inference]** At the rate assumed in the original notes (4 MB filled in 11.6 min, about 5.7 KB/s or 22 pages/s), page programs keep the writing core IRQs-off about 9 ms per second (~1%) at tPP typical, and about 67 ms per second (~7%) at tPP max, with each blackout up to ~3 ms plus SDK overhead. At the 3.8 KB/s rate in the log storage study (`2026-11-21_goddard_log-storage-study.md`) it is about 15 pages/s, 6 ms/s typical and 45 ms/s worst case. A 4 KB sector erase, by contrast, takes up to 400 ms per sector, which is why erases are excluded in flight.

## 6. Does the proposed flash rule match practice?

**[Inference]** It matches: append-only programs in flight, no erases while armed, one writer, stop when full (no wrap), close the session after landing, and erase on the ground only after a verified offload (stricter than Blue Raven, same spirit as AltOS's manual erase).

Suggested additions, all **[Inference]**:
1. Budget against tPP max (3 ms) plus SDK overhead, not typical. The stall gate should record the worst case over many thousands of programs.
2. State explicitly that the writer will not lock out Core 1 (`PICO_FLASH_ASSUME_CORE1_SAFE` or a custom helper), and add a link-map check that Core 1's whole call graph and data are in SRAM; otherwise moving Core 1 to SRAM buys nothing. This changes the `docs/MULTICORE_RULES.md` "Always use `flash_safe_execute()`" rule for this one path.
3. Verify the log region is erased before arming (AltOS and Betaflight scan for erased space), and refuse or flag logging if it is not. A ground erase of ~4 MB can take tens of seconds (chip max 50 s), so do it in Idle with feedback, like Blue Raven's LED.
4. Keep a per-page CRC (AltOS uses per-record checksums), so a torn page is just a CRC failure at offload.

## 7. Later corrections and decisions (2026-11-21)

These came after the notes were first written.

- **Flash rule (plan §19.1), PROPOSED, not yet signed off by Nathan.** Program pre-erased pages while armed, from one writer. No erase while armed, or until a CRC-verified offload. Deadline-critical code runs from SRAM on the non-writing core, with the writer configured not to pause it; otherwise the measured worst-case stall is budgeted with at least 10% margin. Additions 1 and 2 in §6 correspond to the stall-budget and SRAM clauses of this wording.
- **Nathan's decisions that touch the rule:**
  - Recording stops 2 minutes after landing, as a config default. This is the "close the session after landing" step.
  - The pre-trigger and post-trigger windows go in the mission-profile vehicle config.
- **Offload verification.** CFDP (CCSDS File Delivery Protocol) Class 1 with transaction closure is planned for USB offload, as its own `cfdp` module in the Starcom library. The plan has the board erase a log only after a Finished PDU reporting no error, Data Complete, and file status "Delivered file retained in filestore successfully" (CCSDS 727.0-B-5 Table 5-7, Finished PDU contents). See `2026-11-21_goddard_cfdp-reuse-study.md`.
