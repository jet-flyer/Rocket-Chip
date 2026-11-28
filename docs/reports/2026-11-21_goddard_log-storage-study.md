# Flight log storage on small-flash boards (Tiny 2350)

- **Date:** 2026-11-21
- **Author:** Goddard
- **Scope:** How the main logger stores flights today, what the planned Tiny 2350 sensor rates cost in flash, which storage encodings fit a 4 MB XIP-flash board with no PSRAM, and what the W25Q32JV flash part requires for in-flight writing. Research only; no code was changed. Feeds the logging plan (§19) and the flash rule (§19.1).
- **Repo checked:** `origin/main` at `a6d5d4b9` (fetched 2026-11-21). The original study read `c542058b`; the only change between the two commits is `starcom/AGENT_WHITEBOARD.md`, so every file and line cited below was re-read at `a6d5d4b9`.
- **Status:** Frozen once committed. A later report replaces this one.

Every finding is labeled. **[Fact]** means it was read from code, measured, or taken from a cited source. **[Inference]** means it is Goddard's estimate or judgment and has not been measured on hardware.

Method notes:
- The Tiny bring-up logger (`src/logging/sensor_page_log.h` on the `grok/tiny2350-bringup` branch) is not on the remote (`git ls-remote --heads origin` lists AP_ChibiOS, AP_FreeRTOS, agent/whiteboard-pending, fprime, grok/sc-dev and main), so its format was not read.
- Compression figures come from a synthetic data script (`sim.py`, kept outside the repo), not from flight data. The script was re-run on 2026-11-21 and reproduced the figures in §3.

## 1. What the main logger does today

The pipeline is `ao_logger` -> `log_decimator` -> `pcm_frame` -> `ring_buffer` -> (CLI) `flash_flush` + `flight_table`.

**What is logged**
- **[Fact]** One fused-state frame of 61 bytes: sync `0xEB90` (2), MET ms (4), type (1), length (1), payload (51), CRC-16-CCITT (2). `include/rocketchip/pcm_frame.h:3-16, 80-89`.
- **[Fact]** The payload is `TelemetryState`, 51 bytes: quaternion as 4 x Q15, GPS lat/lon/alt, NED velocity in cm/s, baro altitude in mm, baro vertical speed, GPS speed, fix/sats, flight state, health bits, temperature in whole degC, battery mV, MET, flags, and UTC fields. `include/rocketchip/telemetry_state.h:21-55`. It has no accel, gyro, mag or pressure fields, so no raw sensor samples are kept.
- **[Fact]** Event frames are 15 bytes (pyro, abort, phase change and others). `include/rocketchip/pcm_frame.h:39-68`.

**Rate**
- **[Fact]** A 50 Hz tick feeds a box-car decimator (`src/active_objects/ao_logger.cpp:7, 60-62, 367, 386`). The ratio is 4 with PSRAM and 8 without (`kDecimationPsram = 4`, `kDecimationSram = 8`).
- **[Inference]** By code reading, that gives 12.5 Hz with PSRAM and 6.25 Hz without (Tiny). Not checked on hardware.
- **[Fact]** The CLI stamps 50 or 25 Hz into the log header (`src/cli/rc_os_commands.cpp:1128`), and `docs/HARDWARE_BUDGETS.md:99-100` says ~25 Hz (SRAM) and ~50 Hz (PSRAM). These disagree with the decimator reading above. A dump comparing frame count with MET span would settle it.

**Storage**
- **[Fact]** Frames go into a RAM ring that overwrites the oldest frame when full (`src/logging/ring_buffer.h:49`, `src/logging/ring_buffer.cpp:56-64`). With no PSRAM the ring is a 200 KB static SRAM buffer (`src/active_objects/ao_logger.cpp:57-58, 74, 260-285`).
- **[Fact]** 200 KB / 61 B = 3,357 frames. That is about 9 min at 6.25 Hz, or about 2.2 min at the documented 25 Hz (arithmetic). The docs give a Tiny "145s" (`docs/HARDWARE_BUDGETS.md:99, 103`).
- **[Fact]** The ring is written to flash only when a user runs the CLI flush: `cmd_flush_log` (`src/cli/rc_os_commands.cpp:1108`) is the only caller of `flush_ring_to_flash` (call at `:1143`). The flush erases and writes whole 4 KB sectors (`src/logging/flash_flush.cpp:224-258`), puts a 64-byte `FlightLogHeader` at the start of the first sector (`include/rocketchip/pcm_frame.h:129-145`), then adds an entry to a dual-sector flight table of up to 32 flights (`src/logging/flight_table.h:35`, `src/logging/flash_flush.cpp:299-357`).
- **[Fact]** The log area runs from 512 KB up to 7 sectors below the top of flash (`include/rocketchip/flash_layout.h:34-63`). On 4 MB that is 0x080000-0x3F9000 = 3,641,344 B = 889 sectors (arithmetic).

**What this means for a flight**
- **[Inference]** If power is lost before someone types the flush command, the flight is lost, because the ring is in RAM. Because raw IMU data is never kept, the filter cannot be re-run and vibration cannot be examined after a flight.

**Side findings (code reading, not tested)**
- **[Fact]** `AO_Logger_log_event()` pushes a 15-byte `PcmFrameEvent` stack object into a ring whose slots are 61 bytes (`src/active_objects/ao_logger.cpp:239-250`, ring initialized with `kPcmFrameStandardSize` at `:280-281`). `ring_push` copies `frame_size` = 61 bytes (`src/logging/ring_buffer.cpp:60`).
- **[Inference]** So each event push reads 46 bytes past the end of the stack object, and each event slot carries stack contents after its CRC. Flagged for Hamilton.
- **[Fact]** Stale comments say the frame is 55 bytes (`src/logging/ring_buffer.h:35`, `src/logging/flight_table.h:42`). It is 61.

## 2. Data budget at the planned Tiny rates

The rates below are the planned Tiny bring-up rates as given in the room. Only the DPS310 pressure rate is on `origin/main`.

| Stream | Rate | Raw bytes | Raw B/s |
|---|---|---|---|
| ICM-20948 accel + gyro, 6 x int16 | 225 Hz | 12 | 2,700 |
| AK09916 mag, 3 x int16 (+ ST2) | 20 Hz | 6-7 | 120-140 |
| DPS310 raw pressure + temp, 2 x 24-bit | 32 Hz | 6 | 192 |
| **Total raw payload** | | | **~3,030** |

- **[Fact]** `origin/main` sets DPS310 pressure at 32 Hz (`src/drivers/baro_dps310.h:15`) and temperature at 2 Hz (`:17`). The AK09916 20 Hz continuous mode is defined at `src/drivers/icm20948_legacy.h:50`. The 225 Hz IMU rate is not on `origin/main`.
- **[Inference]** The IMU is about 90% of the data, so only the IMU encoding changes the log length much. With main's 2 Hz baro temperature, the baro row is about 102 B/s instead of 192 B/s. That changes the total by about 3% and none of the conclusions.

## 3. Encoding options

All durations use the 3,641,344-byte log area. Mag and baro are added at about 410 B/s (raw, tagged, paged) or 806 B/s (float records). "Ratio" means against packed raw int16 (option 2).

**Where the compression numbers come from**
- **[Fact]** Measured on synthetic data only: 60 s of int16 samples at 225 Hz in three cases. (1) Bench still: 1 g on Z, noise only, ~4 LSB accel and ~2 LSB gyro. (2) Boost, moderate: 8 g thrust, 1 g rms vibration, 10 dps rms gyro vibration, low-passed at ~50 Hz. (3) Boost, harsh: 3 g rms and 40 dps rms.
- **[Inference]** These are estimates for real flights, not measurements. Bench dumps so far come from a board that sits still, so ratios measured on them are too optimistic (Hamilton's point). Delta coding halves the bench-still data but saves only about 3-18% in the vibration cases. With a ~50 Hz low-pass and 225 Hz sampling, adjacent samples are not very alike when the board shakes, so the deltas stay large.

| # | Option | IMU B/sample (incl. page overhead) | Ratio vs raw | RAM (encoder) | Torn page | Log length (3.64 MB) |
|---|---|---|---|---|---|---|
| 0 | Today: 61-B fused frame, no raw IMU | n/a (61 B per frame at 6.25 Hz) | n/a | 200 KB SRAM ring | Whole flight lost if power drops before CLI flush | Flash: ~2.6 h at 6.25 Hz. RAM ring: ~9 min, then it overwrites |
| 1 | Float records, ArduPilot `IMU` style (54 B) | 54 | 0.28x | ~0 | Good if paged | ~4.7 min |
| 1b | Compact float (tag + counter + 6 floats = 26 B) | 27.7 | 0.54x | ~0 | Good (1 page lost) | ~8.6 min |
| 2 | Packed raw int16 (tag + 8-bit counter + 12 B = 14 B), 256-B page, 12-B header, CRC-32 | 15.1 | 1.0x | ~0 | Good: lose ≤1 page (~70 ms) | ~16 min |
| 3a | 2 + per-axis delta, zig-zag varint (Blackbox "Signed variable byte"), first sample of each page absolute | 7.8 still / 13.3 boost / 14.6 harsh | 1.9x / 1.13x / 1.03x | < 100 B | Good (keyframe each page) | 28 / 17.8 / 16.4 min |
| 3b | 2 + delta, width tag + bit-pack (like Blackbox TAG8_4S16 / TAG2_3S32) | 6.8 still / 12.4 boost / 13.9 harsh | 2.2x / 1.2x / 1.09x | < 100 B | Good (keyframe each page) | 31 / 18.9 / 17.2 min |
| 4 | LZ4 on independent 4 KB blocks of raw records | 11.0 still / 13.1 boost | 1.4x / 1.15x | 16 KB hash (default) + 4 KB staging; hash can shrink to 1 KB | Fair: lose up to 1 block (~1.2 s) | 21 / 18 min |
| 4p | LZ4 per 256-B page (independent) | 13.4 still / 13.6 boost | ~1.1x | as above | Good | ~17.5 min |
| 5 | heatshrink (LZSS) w=8, l=4, 4 KB blocks | 8.7 still / 14.4 boost | 1.7x / ~1.05x (worse than raw in harsh) | ~1 KB | Fair (per block) | 25.6 / 16.6 min |
| 6 | Deflate, 4 KB blocks, reference only | 6.8 still / 11.6 boost | 2.2x / 1.3x | ~350 KB default (miniz); ~41 KB cut-down (user report) | Fair (per block) | 31 / 20 min |
| 7 | IMU FIFO compression (ISM330DHCX) | not measured | "two or three times" (ST) | in sensor | Same as host format | Needs an IMU change |

- **[Fact]** The table's B/sample figures for options 2-6 are the synthetic-data results; the log lengths and ratios are arithmetic from them. The deflate figures were produced with zlib level 6 as a stand-in for miniz; the 11.6 B boost figure is deflate on delta-coded records (deflate on raw records gave 12.0 B).

**Option 1 (floats, ArduPilot)**
- **[Fact]** ArduPilot packets start with a 3-byte header `0xA3 0x95 msgid` (`libraries/AP_Logger/LogStructure.h`, `HEAD_BYTE1`/`HEAD_BYTE2`). Each self-describing `FMT` message is 89 bytes: header, type, length, 4-char name, 16-char format, 64-char labels (`struct log_Format`). That cost is paid once per message type.
- **[Fact]** The `IMU` message (`libraries/AP_InertialSensor/LogStructure.h`, `struct log_IMU`) is 54 bytes: 8-byte time, instance, float gyro and accel, error counts, temperature, health and rates. The big cost is floats and a 64-bit time per sample, not the `FMT`.
- **[Fact]** For high-rate raw data ArduPilot uses batch messages. `ISBD` holds 32 int16 samples of one 3-axis sensor in 207 bytes (`struct log_ISBD`), about 6.5 B per axis triplet or ~13 B per 6-axis sample, close to option 2.
- **[Inference]** Worth copying: write format descriptors once into the log header so the decoder does not need firmware headers.

**Option 2 (packed raw)**
- **[Inference]** Store raw register counts and log the scale settings (FS_SEL, DLPF, divider, read back at boot) once in the page or session header; the decoder converts to SI units. This is the biggest win against floats (3.6x against option 1, 1.8x against 1b) and costs almost nothing to run.

**Option 3 (Blackbox-style)**
- **[Fact]** Betaflight/Cleanflight Blackbox uses I-frames ("like video keyframes") and P-frames (value minus a predictor). Predictors include "straight line" and "average of the two previously logged values", the latter used "when there is significant random noise involved in the field". Results are coded with variable-byte or tagged bit-packed encoders (TAG8_4S16, TAG2_3S32). "When one interframe fails to decode, all following interframes will be undecodable up until the next intraframe." (Blackbox internals doc.)
- **[Fact]** The internals doc gives a "typical" 30 state variables at "an average of 28 bytes per frame" at 900 Hz (multirotor data, not rocket boost). The Cleanflight user doc says a 2 MB chip holds "about 3 minutes" at the default 1/1 rate and 2400 µs looptime, and about 650 B/s and "around 50 minutes" at 1/32 rate.
- **[Inference]** A keyframe at the start of every flash page keeps each page decodable on its own. The synthetic numbers agree with the Blackbox trend: big gains at rest, small gains under vibration.

**Options 4-6 (general compressors)**
- **[Fact]** LZ4's default `LZ4_MEMORY_USAGE` is 14 (16 KB) and the minimum is 10 (1 KB) (`lib/lz4.h`). heatshrink can "run in under 100 bytes of memory"; its optional index "adds 2^(window size+1) bytes", and its README suggests a window of 8-10 for embedded use. A miniz user reports the default tdefl compressor "takes about 350kb" and a cut-down build at "only 41kb" (miniz issue #300). uzlib's compressor "doesn't support streaming operation".
- **[Inference]** These need history to find repeats, and the noisy low bits of sensor data do not repeat much, so LZ-type compressors do poorly on raw IMU data, especially on small blocks. Bigger blocks help the ratio but lose more on a torn block and need RAM staging. Deflate compresses best but needs too much RAM for the default build.

**Option 7 (sensor-side compression)**
- **[Fact]** The ST ISM330DHCX embeds a 3 KB FIFO, "up to 9 kbytes with the compression feature enabled", and ST describes the data as able to be "compressed two or three times" (ISM330DHCX datasheet DS13012, read through a search index of the datasheet on 2026-11-21; the PDF could not be downloaded directly). The ICM-20948 datasheet rev 1.3 in the repo (`docs/hardware/datasheets/ICM-20948-datasheet-v1.3.pdf`) has no FIFO compression feature.
- **[Inference]** This helps the sensor-to-MCU link and FIFO depth. It is not a storage format, and it needs a different IMU.

## 4. Flash facts that drive the design (Tiny 2350)

- **[Fact]** The flash part is a Winbond W25Q32JVUUIQ ("4MB W25Q32JVUUIQ" on the Pimoroni Tiny 2350 schematic).
- **[Fact]** W25Q32JV (datasheet Rev G, AC characteristics p. 64): page program tPP 0.4 ms typical, 3 ms max; 4 KB sector erase tSE 45 ms typical, 400 ms max; minimum 100K program/erase cycles per sector; more than 20-year retention. Programs are 1-256 bytes within a 256-byte page; the minimum erase is 4 KB.
- **[Inference]** Wear is not a concern. Stop-when-full erases each log sector about once per flight, so at 100K cycles the log area lasts about 100K full flights.
- **[Fact]** While the RP2350 programs or erases its own XIP flash, code cannot run from flash. The repo uses `flash_safe_execute()` (`src/logging/flash_flush.cpp:49, 54`; `src/calibration/calibration_storage.cpp:97, 115`), and `docs/agents/LESSONS_LEARNED.md` Entry 31 records I2C side effects on the RP2350 (status line: "mechanism KEEP").
- **[Inference]** At ~3.8 KB/s the logger programs ~15 pages/s. That is ~6 ms/s busy typical and up to ~45 ms/s worst case. The IMU must buffer samples during each stall, at 12 B x 225 Hz = 2.7 KB/s.
- **[Fact]** The ICM-20948 FIFO is 512 bytes in DS-000189 rev 1.3 and 4 KB in rev 1.5 and 1.6, in both cases "FIFO size will vary depending on DMP feature-set". Rev 1.3 was read from the repo copy; rev 1.5 and 1.6 were read through a search index of the TDK and distributor copies on 2026-11-21. Which size the fitted chip has should be checked by filling the FIFO and reading `FIFO_COUNT`.
- **[Inference]** 512 B holds ~190 ms at 2.7 KB/s. That covers a page program (3 ms max) but not a worst-case sector erase (400 ms). So the whole log area should be erased on the pad, verified blank, and never erased in flight.

**Power loss**
- **[Inference]** A torn page program leaves some bits in that page programmed and some not. A per-page CRC-32 finds it; the decoder skips that page and loses at most one page.
- **[Inference]** An interrupted erase can leave bits weakly erased: they may read as 0xFF but program badly later. This is general NOR flash behavior; no W25Q32JV statement about it was found. Safe rule: after any reboot, treat a sector that is not fully verified blank as used, never append into it, and re-erase only on the pad.
- **[Inference]** Any scheme that carries state across pages (delta chains, LZ windows, a running CRC) fails after a torn page. Every page must decode alone, with a keyframe at the start, its own header and its own CRC.

## 5. Recommendation (ranked by benefit against complexity)

The goal is one upgraded main logger for no-PSRAM boards, not a second format next to the PCM frame. All items are **[Inference]**.

1. **Stream to pre-erased flash in flight, in independent 256-B pages** with CRC-32 and a header carrying session, page sequence and first-sample time/counter. This is the biggest win, because today a no-PSRAM flight lives only in a 200 KB RAM ring until someone types the CLI flush. It needs a new page container in `flash_flush`/`flight_table`, a launch-trigger start, a 1.5 s RAM pre-trigger buffer (~5.7 KB at 3.8 KB/s), and a pad erase with blank check.
2. **Store packed raw int16 sensor records (option 2) as new record types in that container, and keep the 61-B PCM fused frame as one more record type at its low rate.** This gives ~16 min of full-rate raw IMU, mag and baro in 3.64 MB, 3.4x longer than ArduPilot-style float records, and allows filter replay and vibration analysis. It needs a driver change (FIFO or data-ready reads, a sample counter, raw counts kept, config read-back logged once; already Council item 1) and a decoder change. Write a self-describing format table once per session, so the Python decoder does not hard-code layouts.
3. **Optional later: per-page Blackbox-style delta + bit-pack (option 3b).** Low RAM and CPU, and torn-page safe with a keyframe per page, but on vibrating data it saves only about 10-20% (synthetic estimate). Do it only if real flight logs show the need, and measure it on a real or shaker log, not on bench-still dumps.
4. **Do not use a general compressor** (LZ4, heatshrink, deflate) for the IMU stream on this board. It gives little on noisy vibration data and adds RAM and block-loss risk. Deflate does not fit in RAM next to the ring.
5. If more log time is ever needed, a **log policy** helps more than compression: full rate from trigger through apogee + N s, then a lower IMU rate under parachute.

## 6. Addendum: CCSDS 121.0-B-3 as a candidate (from Duke)

- **[Fact]** CCSDS 121.0-B-3, Lossless Data Compression, Blue Book, Issue 3, August 2020 (https://ccsds.org/Pubs/121x0b3.pdf). Duke checked that it is the current issue. It specifies an adaptive Rice coder: a preprocessor (predictor and mapper) feeds an entropy coder that picks a code option for each block of J samples.
- **[Inference, Duke]** RAM use is very small; it was designed for spacecraft sensor data on small processors. It is block-adaptive, so it fits page-at-a-time writes, and reference samples (interval r, section 3) let decoding restart after corruption, which fits the torn-last-page case.
- **[Inference, Duke]** It works on fixed-width integer samples, one IMU channel at a time, not on mixed records.
- **[Inference]** Not measured. Bench it next to delta + bit-pack on a shake log. Its predictor is close to the delta coding above, so expect a similar gain under vibration (about 10-20%) until measured.

## 7. Later corrections and decisions (2026-11-21)

These came after the study was first written.

- **[Fact]** Main's I2C bus runs at 400 kHz: `src/drivers/i2c_master.h:24`, `kI2cMasterFreqHz = 400000`. A 100 kHz figure used in earlier discussion came only from a scratch test harness.
- **Nathan's decisions:**
  - The pre-trigger and post-trigger windows go in the mission-profile vehicle config.
  - Recording stops 2 minutes after landing, as a config default.
  - The log rate stays as it is until a full test has been compared with real flights.
- **Flash rule (plan §19.1), PROPOSED, not yet signed off:** program pre-erased pages while armed, from one writer; no erase while armed, or until a CRC-verified offload; deadline-critical code runs from SRAM on the non-writing core, with the writer configured not to pause it; otherwise the measured worst-case stall is budgeted with at least 10% margin. Recommendation 1 above depends on this rule.

## Sources

- Repo: files and lines above, `origin/main` `a6d5d4b9`.
- W25Q32JV datasheet Rev G (Winbond, 2018-03-27), AC characteristics p. 64: https://www.winbond.com/resource-files/w25q32jv%20revg%2003272018%20plus.pdf . Also cited in the original study, not re-checked (the host now returns a bot challenge): Rev I copy at https://devzone.nordicsemi.com/cfs-file/__key/communityserver-discussions-components-files/4/6864.W25Q32JV-RevI-05042021-Plus.pdf . Rev K timing summary (same figures): https://pcbwiki.com/parts/w25q32jv
- Tiny 2350 schematic: https://cdn-shop.adafruit.com/product-files/6248/Pimoroni_Tiny_2350_Schematic.pdf
- ICM-20948 DS-000189 rev 1.5 (FIFO 4 KB): https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000189-icm-20948-v1.5.pdf ; rev 1.3 (FIFO 512 B): repo copy `docs/hardware/datasheets/ICM-20948-datasheet-v1.3.pdf` and https://www.mouser.com/datasheet/2/400/DS-000189-ICM-20948-v1.3-1385562.pdf
- Cleanflight Blackbox user doc: https://github.com/cleanflight/cleanflight/blob/master/docs/Blackbox.md
- Blackbox internals: https://github.com/cleanflight/cleanflight/blob/master/docs/development/Blackbox%20Internals.md
- ArduPilot log header and `FMT`: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Logger/LogStructure.h ; `log_IMU`, `log_ISBD`: https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_InertialSensor/LogStructure.h
- heatshrink: https://github.com/atomicobject/heatshrink
- LZ4 `LZ4_MEMORY_USAGE`: https://github.com/lz4/lz4/blob/dev/lib/lz4.h
- miniz tdefl memory: https://github.com/richgel999/miniz/issues/300 , https://github.com/richgel999/miniz/blob/master/miniz_tdef.h
- uzlib: https://github.com/pfalcon/uzlib
- ISM330DHCX datasheet: https://www.st.com/resource/en/datasheet/ism330dhcx.pdf ; AN5398: https://www.st.com/resource/en/application_note/an5398-ism330dhcx-alwayson-6axis-imu-inertial-measurement-unit-with-embedded-machine-learning-core-and-digital-output-for-industrial-applications-stmicroelectronics.pdf (AN5398 not re-checked)
- CCSDS 121.0-B-3: https://ccsds.org/Pubs/121x0b3.pdf
