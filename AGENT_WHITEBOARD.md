# Agent Whiteboard

**Purpose:** Cross-context / cross-agent communication channel for **active work only**.

> **Treat this like an IRL whiteboard - not a record of completed things.**
> When an item is done, **erase the row**. Don't add a "Resolved" section,
> don't strike it through, don't leave a "closed" marker. The CHANGELOG is
> the project's permanent record of what was done; this whiteboard's only
> job is to surface what's *still active*. A row's continued presence is
> the signal that it still needs attention. Stale "done" notes dilute that
> signal and bury the rows that actually matter.
>
> Before adding a row: check whether the work is already done elsewhere
> (CHANGELOG, git log, the relevant doc). Before acting on a row: spot-
> check it's still real - agent memory of "this needs doing" is exactly
> the failure mode that drives stale-row accumulation. If you reject an
> item after consideration, log the rejection rationale in CHANGELOG and
> erase the row, don't move it to a "rejected" section.

## Handover 2026-10-21 (Hamilton)

- **State:** LINK counters and the three-run LoRa baseline are on `main` (`8a80734d`, `c71fcbcf`, `06c398aa`; `docs/reports/2026-10-20_buzz_lora_baseline.md`). TX power clamp `7abfea14` (2-17 dBm, +20 dBm opt-in) pushed to `main` 2026-10-22 (`c497b833..7abfea14`); push gate PASS 2026-10-21 (2 dBm link check only, note in `refs/notes/rc-bench`). FSK work has not started.
- **Plan:** `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` is WIP and unreviewed; its open questions are in its status block.
- **Next bench, in order:** (1) Standby-before-write patch (not written yet; see the RFM95/SX1276 driver row), then re-bench (Now: AO / QEP); (2) soak/baseline script fixes (baseline report, "Open follow-ups"); (3) station stuck at ABORT (seen again after the 2026-10-21 bench) and DISARM with no ack; (4) watch whether push 3's 15 station CRC errors come back; (5) DIO soldering; (6) H6 part 1 and P36 part 1 (Next).
- **Off-repo files:** `rc-bench-archive\baseline_lora_2026-10-20\` (bench_layout.jpg, T1-0_PROMPT.md, FLASH_PROMPT.md), `rc-bench-archive\plan_reorg_draft\CHANGES.md`, `rc-bench-archive\pushgate_8a80734d_2026-10-21\`, `rc-bench-archive\pushgate_41a304f5_2026-10-21\`.

## Now (this sitting, before the next push)

### Station-only wiring checks before soldering (2026-10-21)

Breadboard jumpers first; test before any soldering (Nathan). Owner: Buzz.
1. SWD: OpenOCD chip ID, plus a memory read while the firmware runs.
2. RFM95 SPI/CS: RegVersion = 0x12; Standby write and read-back of a config register.
3. DIO: RX-done/CRC flags while receiving; TX-done via an `r` preset step.
4. GPS I2C: `g` reads climbing, no errors.

Link-quality checks (RSSI, CRC rate, soak) still need the vehicle.

### AO / QEP order: settle before FSK work (2026-10-11, Council 2026-10-21)

Source: `docs/reports/2026-10-11_goddard_ao_qep_audit.md`. Suggestions only; Nathan decides the split before any FSK pivot code.

- **Before the pivot:** ESKF rate measured 2026-10-14 (R-33): about 116 Hz with USB connected and with USB gone. Council the same day: leave the filter in place through the FSK bench. The 200 Hz label is IMU-poll/5 from Stage 5, not a dynamics requirement, and the live ~4.6 ms body does not fit a 5 ms period. A still-board debug `s` measured predict at 246 µs average inside a 4685 µs full tick. New FSK FIFO service runs from the DIO interrupt.
- **Council outcome (2026-10-21, Nathan took both split picks):** order is Standby-before-write patch now -> measure flash stall -> driver rewrite + QEP states on LoRa -> byte_pump split in its own sitting -> FSK. Rewrite: read-back + error return on every register write, RegOcp OcpTrim 17 (140 mA) for +20 dBm, LowDataRateOptimize computed from SF and BW (symbol > 16 ms) on both ends, airtime formula fixed separately for CR other than 4/5. States: rx, tx, applying-with-revert-watch; COP-P stays in `copp.cpp`; persisting moves to the flash owner. Chart: PlantUML only with a test against the transition table. Flash: one owner, writes only while disarmed, nothing stalls ARMED to landing; open question is a RAM ring for the flight log. Config: two-phase commit (apply, revert unless ground confirms within T, persist after confirm). Reboot keeps the last confirmed config; the no-link-within-T fallback to a link-safe config runs only when the vehicle wakes in a ground state, never in flight; the station re-acquires by alternating between last-confirmed and link-safe. Link-safe config: a fixed pre-agreed channel/rate modeled on Prox-1 hailing (211.0-B-6 §6.2.4.15-16, Annex C), not book hailing, which needs carrier acquisition the phase 1 hardware (RFM95) cannot do; continuous FSK at zero deviation is only an approximate carrier, an FSK-pivot idea; in-session link loss re-hails without resetting FOP-P/FARM-P (E80/E82, §1.5.1.2), so COP-P counters survive a fallback. Keeping the last confirmed config across an in-flight reboot is our own rule; Prox-1 has no reboot clause (Duke, 2026-10-21). FSK: 64-byte FIFO (~10 ms at 50 kb/s) needs its RX/refill path in SRAM or a PIO/DMA ring; DIO1 ISR only reads IRQ flags, timestamps and posts. Host tests to add: COP-P PLCW repeat, nonzero sync timeout, command resend, and COP-P sequence numbers surviving apply/confirm/revert. Datasheet figures, not measured.
- **Anytime (not during a bench run):** QP 8.1.3 to 8.1.5 (re-vendored files).
- **After the pivot:** RCOS / calibration wizard split; station GPS poll to core 1; STARS re-evaluation; paywall re-check (below).

### Needs rework: legacy modules to rewrite (Nathan, 2026-10-21)

Modules to rewrite from scratch rather than patch. Add rows as they turn up.

- **RFM95/SX1276 driver** (`src/drivers/rfm95w.*`): writes config registers while in RX (datasheet Rev 4 p.38 needs Sleep/Standby/FSTX), airtime math assumes CR 4/5 with LDRO off, setters return nothing and never read back, validation spread over 3 layers, 3 packet-done paths, `RegOcp` never set. The link-loss revert (`tick_symmetric_revert`) only compiles with `ROCKETCHIP_RADIO_PERSIST`, which is off and broken (CW-B39-01), so it is not live. The +/-6 dB per-command power rule had no datasheet or FCC basis; removed 2026-10-21 on `grok/tx-power-clamp` (`7abfea14`; no host test yet proves a single 2-to-17 dBm SET is accepted; `dispatch_set_radio_config` is not in the host build). Until the rewrite, the 2-17/20 dBm range check is the only live power guard. Overlaps AO split option A (Radio mode flags to QEP states); sizing in the room 2026-10-21 (Buzz).
- **+20 dBm (from the old "+20 dBm limit cause" row):** since `7abfea14`, 18-19 dBm are rejected and +20 dBm is opt-in. `RegOcp` is never set, so +20 dBm on PA_BOOST would hit the default current limit; set OcpTrim 17 (~140 mA) in the rewrite, then bench-check output power (not measured). Datasheet limits: `docs/hardware/HARDWARE.md`, RFM95W / SX1276 PA limits. Boot applies `kDefaultRocketRadioConfig` at 2 dBm (frozen plan rev 2 still says 20 dBm). Open: is the 1% duty limit heating only, or can +20 dBm past it damage the chip another way? Source must be Semtech. Bench power stays 2 dBm until a field test.

### COP-P open checks from the LoRa baseline (unverified) (2026-10-21)

1. **PLCW repeat:** confirm the firmware re-sends the PLCW on a timer per PLCW_Repeat_Interval (CCSDS 211.0-B-6 Annex C). Lost ACKs depend on it.
2. **SYNCH_TIMER = 0** (`copp.hpp:32`, never expires): `copp_sync_loss` is unmeasured, and lost Sequence Controlled frames are resent with no backoff.
3. **Command resend untested:** the baseline sent only Expedited traffic, so `copp_rtx=0` does not prove resend works. Folds in: DISARM got no ack in the push-gate soaks, plus Duke's note on it (unchecked).
4. **"Lock" never goes stale (code-read, 2026-10-19):** every "lock" is `fop.plcw_heard && mac_mode == active` (`ao_telemetry.cpp` `peer_plcw`); `synch_timeout = 0` in `src/starcom_adapt/byte_pump.cpp`; vehicle `t` and RC_OS text (`rc_os_commands.cpp`, `ao_rcos.cpp`) print "lock" with no freshness check (only the pad RF Link row adds `rf_live`). Seen 2026-10-19: station USB died, vehicle TX froze, vehicle still said lock. To do: rename to "COP-P sync", nonzero `synch_timeout` (a multiple of station PLCW_Repeat_Interval), start the timer on silence too, drop the indicator on expiry. Prox-1 reading (Duke): `starcom/AGENT_WHITEBOARD.md`, "COP-P sync indication".

### Small firmware / script fixes (Buzz) (2026-10-19..21)

- **`b` beacon (vehicle CLI):** main-menu `b` turns the find-me beacon on; a second `b` does not turn it off (stays on until the next state change). Seen on the board 2026-10-21.
- **Debug HW line always prints `20dBm`** (`rc_os_commands.cpp` about line 813, re-checked 2026-10-21).
- **`pre_push_gate.py --help`** does not act as a help call (deferred from the 2026-10-21 wrap; details not written down, re-check first).
- **`pre_push_gate.py` role table** (about line 125) still marks the vehicle as the SWD board; the station is never offered Path 2 (SWD) flash steps. Harmless (code-read 2026-10-19).

### Power SET reject codes on hardware (2026-10-21)

Two-board soak of `7abfea14` done (baseline report, "Push-gate soaks (2026-10-21)"). The SET replies (2-17 accept, reject codes, +20 opt-in) cannot be benched on this firmware: the station only steps 2 dBm presets, and MAVLink returns UNSUPPORTED. Needs Nathan's pick: a station power SET command, or a test build with a non-2 dBm preset. `scripts/power_set_check.py` exists, uncommitted (dry run OK).

## Next (before FSK work / before first flight)

### Bench items waiting on Nathan (2026-10-08)

Detail: `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` (line refs may be stale after the 10-20 reorg).

- H6 part 1 (never ran): desk RSSI baseline, both boards at 2 dBm, fixed distance (plan l.336 check 2; desk power l.135). Part 2 (antenna in airframe) waits for the airframe. Needs Nathan's go.
- P36 part 1: Nathan holds KB2040 BOOT while the gusmanb v6.0.0.1 UF2 enumerates (plan l.145, l.313); needed before any capture.
- IVP 21 (stale): `783a994` reset-looped, no second flash. Buzz suggests closing unless Nathan wants that branch.
- Commit `scripts/radio_link_bench.py` (makes radio pushes blocking) after Nathan's bench run of it. Still untracked (2026-10-21).
- Fruit Jam full-station freeze: open PR, `docs/PROBLEM_REPORTS.md` R-34.

### clang-tidy config defects (PR-2 / PR-1b) (2026-10-29)

Both `.clang-tidy` files glue comments into `Checks:` (root: 10 checks off, Starcom: 13, plus 23 `clang-analyzer-core.*` silenced in both), and `HeaderFilterRegex` uses `/`, so Windows reports no header hits. Fix order: `starcom/docs/audits/STARCOM_AUDIT_2026-10-29.md`, Section 2 "Code-fix plan" (Grok Build on the PC; firmware tree, bench at push). The root fix (G2) changes only `.clang-tidy`. New RC findings go to the RC master audit. The old RC PASS lines stay as written (Nathan, 2026-10-30); the Starcom audit records which rested on checks that were off.

### Two JSF rules broken with no stated reason: braces and names (Nathan, 2026-10-30)

JSF 60/61 (braces on own lines) are mandatory "will" rules with no exception (JSF Rev C 4.2.1, AV Rule 5), but all code uses K&R, and `docs/audits/STANDARDS_AUDIT_2026-02-07.md:271-272` is a false PASS; the `standards/CODING_STANDARDS.md:63` camelBack override of JSF 45/51/52 gives no reason (`docs/audits/RULE_VERIFIABILITY_TRIAGE.md:466` says only "repo-owner authorized"); JPL C puts naming and formatting out of its scope and Power of 10 has no layout or naming rule (Goddard, checked against both texts), so JSF governs; MISRA not checked. After Starcom and `src/starcom_adapt/` change to JSF 60/61 (W2), Starcom and the rest of RC use different brace styles, and `src/starcom_adapt/` differs from the rest of `src/`, until the RC-wide change lands; this row tracks that known mismatch. `CODING_STANDARDS.md:56` ("accepted deviation FP-1"; FP-1 closed 2026-05-13) and `:97` ("`#pragma once` is a live deviation"; RC has none) are also out of date; note only for now, so `CODING_STANDARDS.md` needs a deeper review later. RC master audit item; Starcom W2 converts Starcom and `src/starcom_adapt/` now (Nathan, 2026-10-30) (`starcom/docs/audits/STARCOM_AUDIT_2026-10-29.md`, Section 2).

### Pre-commit hook positive controls + gaps (TODO) (2026-10-21)

Hook still works; no change now. Gaps found in a read-only inventory:
1. G3 ignores the `cmake --build build_host` result; ctest can pass on old binaries.
2. Silent skips: G2 (no clang-tidy or `build/compile_commands.json`), G3 (no `build_host`), G4 (python missing or classifier crash).
3. G1 matches only the literal `<stdio.h>` in `src/*.cpp`; G2 skips `src/cli/**` and all `.h` files.
4. The audit-cycle citation check (online) runs on every commit; candidate fix `ctest -LE audit-cycle`.
5. G3 and G4 build the working tree, not the staged snapshot.
6. No LF/BOM or CHANGELOG checks exist.

Plan: scratch `git worktree add` at HEAD in a temp folder, configure the build dirs there, then one known-bad change per check (host-only; no boards).

### Bench script fixes (2026-10-21)

`baseline_lora.py` / `soak_two_board.py` bugs: `docs/reports/2026-10-20_buzz_lora_baseline.md`, "Open follow-ups". Scripts still usable via the raw logs.

### Soldering to-do (can be done today) (2026-10-08)

Prerequisite: Now, "Station-only wiring checks before soldering" (breadboard first).

Soldering only. Each board test after it still needs Nathan's go. Pins: Buzz, 2026-10-08 (schematic-checked, not measured). Plan = `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md`.

- **Vehicle RFM95 DIO jumpers to the Feather RP2350.** DIO1 -> A0 (GPIO26). DIO2 -> A1 (GPIO27). DIO5: no wire (no plan pin; Manchester is not adopted, rev 1 8.1). S7 only clips the analyzer on the DIO5 pad. Sources: plan l.157 (pin row), plan l.234 and l.317 (H1), plan l.318 (H2 continuity check after the jumper). E9: the internal pull-up is chosen (Nathan, 2026-10-08), so no resistor (plan §7.2).
- **Station RFM95 DIO jumpers to the Fruit Jam.** DIO1 -> A1 (GPIO41). DIO2 -> A2 (GPIO42). DIO5: no wire. Fruit Jam A0 is not on the header (plan l.158, l.161). No E9 resistor: the Fruit Jam is A4 (plan l.201).
- **KB2040 logic analyzer.** Solder the header pins. Add the trigger jumper TX (GPIO0) -> RX (GPIO1) and the D2 PWM self-check jumper (P36 part 2). Channels 1-8 are D2-D9 (GPIO2-9). Sources: plan l.144-145, l.313 (P36). The plan does not give the other end of the D2 self-check jumper; Buzz to confirm before soldering.
- **Pico 2 W "probe" (spare board, plan l.134). Job: spare (Nathan, 2026-10-11/12; plan Appendix B.3.1, D11).** Not the logic analyzer (KB2040 is) and not a second SWD probe for now.
  - Logic analyzer: Buzz chose the KB2040, because the gusmanb README says E9 locks bare Pico 2 inputs under PIO (plan l.146, P32). For SWD, E9 does not matter.
  - Before soldering, Buzz reads the Pico 2 W chip revision with picotool. A4 has no E9 (plan l.220: A2 by ship date, not measured).
  - Repo docs give no wiring for a second SWD probe. The current probe is the Adafruit #5699 (`docs/hardware/HARDWARE.md` l.58); the Feather has SWD and the Fruit Jam has no SWD (`docs/FLASHING.md` l.58, l.158).

### Station pad: link numbers from Starcom state, not raw radio counters (2026-10-19)

**Station Pkts/RSSI line and LED bar (Buzz, code-read).** The RSSI, Pkts/CRC counts and bar length still come from the raw radio counters (`rx_count`, `last_rx_rssi`). Only the colour/mode reads Starcom state. Move them onto the Starcom and rate counters, like the RX window, so they can't disagree with the RF Link row. Earlier code for this: stash 2 (2026-09-02, not on main), archived at `Documents\rc-stash-archive\stash2.patch` (CHANGELOG 2026-10-13-001). Bench check (Buzz): station `status` count/RSSI/LED bar vs vehicle TX count and a direct radio RSSI read.

Lock freshness: Now, COP-P open checks item 4.

### Opening-shock detector parked on `wip/opening-shock` (2026-10-19)

Committed as WIP `3cd5dbd3` on `wip/opening-shock` (kept local on purpose, not on origin: the pre-push gate requires a bench for any firmware push and this WIP is not benched; off-machine copy is a git bundle, Nathan 2026-10-19). The 12 dirty `flight_director` / `config_wizard` / test files are no longer in the main checkout. Host-tested only. Held up by: never flashed or benched, thresholds are a first cut (not from a flight), open time is USB-only (needs a `LogEventId`), and the branch base `1c71ce7b` needs a rebase onto `main` and review. Room-test procedure: subsection below.

#### Room test: launch, apogee, landing (not yet run; 2026-09-24)

Untethered vehicle on battery. Live view is the station pad phase. Afterward, plug the vehicle and download the flight log (`g` list, `d` download). Do not USB-tether the toss. Do not open the station COM port if QGC has it.

**Image:** the opening-shock detector is not on `main` (WIP `3cd5dbd3`, local `wip/opening-shock`).

**Arm:** station pad `a`, type `ARM`, Enter. Wait for ACK and `State: ARMED`. Disarm is pad `D`. ARM starts the backup timers: drogue pin can go high at 15 s if apogee has not cancelled it, main pin at 45 s. No ematch on those pins. Disarm before 15 s if the pad has not shown apogee, and disarm again at the end.

**Motions, IMU Z out of the top of the chip:**
1. Launch: snap along that axis. Needs `|accel_z| > 20 m/s²` for 50 ms.
2. Burnout: a moment of freefall (`|a| < 5 m/s²` for 100 ms). Sitting still is about 1 g and will not leave boost.
3. Apogee is locked out for 3 s after launch. The top of a room toss will not be the mark. Catch it and hold still. After the lockout, a quiet board can enter drogue.
4. Landing runs only in drogue or main. About 2 s still (speed under 0.5 m/s) after that can land. The baro path wants 5 s under 0.3 m/s.

**Log after:** phase changes and pyro-fired events are in the flight log. The opening-shock time is not. It is a USB line only (`drogue_open_ms` / `main_open_ms` in `FlightMarkers`, printed as response time). Add a `LogEventId` before any untethered pass that needs that interval on disk. oMCT `chute_detected` is still synthetic.

**Opening-shock code (not on `main`, backup on local `wip/opening-shock`, host-tested, not flashed):** every profile, coast through main. Specific force at or above 19.62 m/s² (2 g, above a settled canopy) and 2 m/s of descent-speed lost, held 20 ms. Does not move the phase and does not treat apogee as an opening. Pyro command stamps moved onto the fire transition (`kTransitionFireDrogue` / `kTransitionFireMain`). Response time is command timestamp to shock timestamp. Thresholds are a first cut, not from a flight.

**Verified 2026-09-24:** host `FlightDirectorTest` opening-shock / apogee-crossing / second-opening, guard tests, action-list tests, `scripts_generated_profiles`. Not on a board. Feather was not on the bus (only Bluetooth COM3).

### GCS standalone app (2026-10-01)

**Open bug:** pinning the running app window to the taskbar saves plain Chrome, not the app. The old tk launcher pinned fine. Planned fix: give the Chrome window its own AppUserModelID matching the shortcut and relaunch details, or go back to a native pywebview window (pywebview 6.2.1 is installed and worked earlier), which pins like the old launcher. Not started. It needs launch/close tests on the PC, so wait until Nathan says he is done testing.

**Unverified by Hamilton:** live station mode in the standalone app; plots with the facsimile feed after the Real-Time default; Real-Time showing on screen; taskbar icon and title.

**CONFORMANCE gap cleanup:** the seq count is board-verified (2026-10-20, baseline report), so the `starcom/docs/CONFORMANCE.md` Known gaps entries for seq count / rx gap can be re-checked and removed (Nathan's full-compliance policy, 2026-10-01; prompt files in `docs/decisions/prompts/`).

**Parked:** grey no-data look for gauges/LAD; RF page; FDAI 8-ball; GPS-present flag; MAVLink toggle-only; custom Leaflet map spike; phone/responsive support; Yamcs (judged overkill for now).

### FSK constraints and byte_pump split (2026-09-29, 2026-10-01)

**FSK constraints (still hold):** both SX1276s stay in FSK for a session; no mode change inside a turn. LoRa stays the long-range preset. Convolutional and LDPC encoders stay off the LoRa payload. Do not retune N or vehicle Send_Duration. Simplex and a second radio stay deferred.

**byte_pump surgery (2026-10-01; own sitting between the driver rewrite and FSK per the 2026-10-21 Council, AO / QEP row):** `src/starcom_adapt/byte_pump.cpp` is the air adapter and it has turned into a bucket: MIB timers, the LoRa catalog and COMM_CHANGE, the nav/cmd/ack packers, the two sequence lanes, and the send/receive drain. Split that file in the FSK sitting. Where the pump reimplemented a Blue Book procedure that `starcom/src/ccsds/` already provides, use the book path. Known overlap: the three packers and the two sequence lanes versus 133.0 assembly and extraction (`space_packet_service` is host-only and is not called on the air; root `CHANGELOG.md` 2026-10-01-003). COP-P, the MAC, Version-3, and the PLTU codec are already the library. Keep the product pieces in the adapter: the SX1276 bearer, the nav/cmd/ack payloads, the product APIDs, `flight_mac_mib`, and the catalog. `rx_lost` stays on the pump. The service object is about 10 KiB and stays off the 4 KiB stack. Inventory any other home-grown book procedure in that file the same way before removing it. Starcom row: `starcom/AGENT_WHITEBOARD.md` FSK bitstream bearer.

FSK coding-option question (211.2-B-3): `starcom/AGENT_WHITEBOARD.md`, FSK bitstream bearer. Earlier hitch measurements: CHANGELOG 2026-09-29 entries.

### First-flight prod strip of test/inject (2026-08-23)

Current `build_flight` ELF **is still development firmware.** Approach A (inject/debug linked, `test_mode_active()` no-op) is acceptable until first flight.

**Before first flight:** a dedicated production image that **omits** test/inject TUs from the link - not “compiled in and gated,” not “ifdef in the tree but we pinky-swear DEBUG is off.” `fault_force_*`, debug mutators, station inject, and the arm gate must not be in that ELF (`nm` / `strings` check). Mechanism (prod CMake preset vs stripped tree) is picked in that sitting, not now.

Does not reopen sitting 11. Does not strip on `main` until that sitting.

**Concerns:** Probe residual power (E2) if the board looks dead after SWD.

**Debug `o` (2026-10-08):** `f142a4db` force-one-TX-timeout is inside `#if ROCKETCHIP_DEV_MODE`, but that option defaults ON, and the case does not call `inject_allowed()`, so it runs with runtime DEV MODE off (help only hides it). Compile it out of field images or refuse it when DEV MODE is off. Needs Nathan.

## Later / Wanted

### RC side of Space Packet as the core unit (2026-10-19)

Design and ground-path options (Yamcs, network forwarding, SLE, XTCE): `starcom/AGENT_WHITEBOARD.md`, "Space Packet as the core unit". RC files to sort in that job:

- Now: air is Starcom only (`sc_air.h`), but the station decodes and re-encodes to MAVLink for USB (`src/station/gcs_mavlink.cpp`, `kMavlink` in `station_output_mode.h`, CHANGELOG `2026-09-19-001`). Open MCT reads that MAVLink (`docs/gcs/openmct/realtime/stream_mavlink_station.py`).
- Sort in the same job: over-air MAVLink `COMMAND_LONG` path in `ao_telemetry.cpp`, MAVLink pass-through in `ao_radio.cpp`, `ROCKETCHIP_STAGE_T3_MAVLINK`, `EncoderType::kMavlink` / `MavlinkEncoder`, `mavlink_rx.*`, `lib/mavlink`, their tests, about 30 docs.

### Roles vs. bench targets split (deferred, Nathan 2026-10-08)

**Roles vs. bench targets split. Deferred until the Forgix or the pico-ice gets an RC role (Nathan, 2026-10-08).** A role is a firmware config (vehicle or station). A role is not a board. One board (for example the Feather) can run both roles. Today the gate benches the vehicle and the station separately only because the current boards have different hardware quirks. Idea: a table that maps each bench target to the role that it runs. Each bench target row gives the physical board, the USB serial, the flash path and the board quirks (chip package and pin count, RP2350 stepping and E9). The table stores the stepping per board serial, from a picotool read of that board, not from the board type. Today the `ROLES` table in `scripts/ci/pre_push_gate.py` maps each role to one setup. Nothing is decided.

### Tiny 2350 as a core RC analogue (Nathan 2026-10-08)

**Tiny 2350 as a core RC analogue (Nathan, 2026-10-08).** Use the Tiny 2350 as a smaller, lighter core RC analogue when work on internal logging and operation without telemetry starts. Board facts and pin-map conflicts: `docs/hardware/HARDWARE.md`, "Tiny 2350 (PIM721)".

### Re-check paywall / commercial-tool decisions (LATER) (2026-10-11)

- STARS was rejected as a commercial autocoder, but it is open source (Apache-2.0, docs/flight_director/TOOLCHAIN_EVALUATION.md). Re-check every other tool or standard that the docs rejected as paid, commercial or paywalled; confirm the licence and price from the source before the decision stands (Nathan, 2026-10-11).

### GCS glass: live oMCT PoC (NEXT) (2026-09-20)

**PoC proven (desk):** Master Dashboard QGC-style ring + facsimile OK; live Fruit Jam COM7 into Open MCT works via `stream_mavlink_station.py` (MAVLink USB) when station is in kMavlink. Facsimile: `feed_facsimile.py` + Play http://127.0.0.1:8092/. Serve from `docs/gcs/openmct/` (not hello-world alone) so `../plugins` resolve — URL http://localhost:5000/hello-world/.

**Still to work out (not done):** field mapping / which gauges light from station vs vehicle; RSSI/LQ/SNR on glass when only MAVLink USB (no RADIO_STATUS yet); ANSI `stream_station.py` path vs MAVLink path; layout tweaks Nathan noted.

**Parked firmware:** auto first-STX (0xFD/0xFE) lock into exclusive MAVLink CDC fights ANSI dash / oMCT scrape — make MAVLink toggle-only, off by default (boot stays ANSI). See `src/cli/rc_os.cpp` sniff + `StationOutputMode`. Not a license this wrap.

### GCS glass: LCARS skin (WANTED) (2026-09-05)

Desk Open MCT hello-world is on Espresso with stacked RSSI/SNR/Baro + radio overlay. **LCARS skin** (louh/lcars or frame-wrapper) is the next aesthetic pass after more field testing / layout tweaks. Do not start unprompted; themes after ingest was the prior lock and ingest is now desk-proven.

### RC_OS / station pad UX (WANTED) (2026-09-18)

Station pad is the Fruit Jam ANSI dashboard (`src/cli/rc_os_dashboard.cpp`); console after `x` is `cli_menus.h`. Not Open MCT.

**Dashboard configurator.** Host tool so the user can lay out the pad (which rows, order) without a firmware sitting each time. **Drag-and-drop belongs on the configurator**, not the USB ANSI pad; the pad just consumes a saved layout. Default **operator pad stays one layout.** Not a license this sitting.

**Named views** (configurator presets, not extra firmware products): flight-only, RF-only, IMU, and a **raw-data / workbench lab** dash (full counters, unscaled fields, RATE-class dumps). Lab view is for the bench, not the range pad.

**ASCII attitude indicator.** Wanted if USB CDC + redraw latency stay acceptable (pad is 1 Hz idle / per-RX). 80-col / ~18-row glass may be the limiter, not the quaternion. Gate on a desk latency/readability check before keeping it. Not a license this sitting.

### Hobby secondary link (BT / Wi‑Fi) (WANTED) (2026-09-18)

CCSDS-style split: high-rate TM stays on the Prox-1 LoRa air; a **second physical path** for low-latency ground commands (pad ARM/DISARM, settings) and other short important traffic. Hobby-grade Bluetooth or Wi‑Fi, not a second Starcom MAC. FTS/abort stays onboard. LoRa remains the book range link. Not a license this sitting; PHY/chip TBD.

### HD token N: user-facing preset (WANTED) (2026-09-18)

Table and default N=11 live in `src/starcom_adapt/README.md` (RC product). Book MIB/token is `starcom/docs/USER_GUIDE.md` + GLOSSARY. Later configurator or radio preset; do not silent-retune `flight_mac_mib()`. Other latency cut: hobby secondary link row.

### FPV-style scan / find (WANTED) (2026-09-02, restated 2026-09-03)

**Coupled with LoRa Hail** (Prox-1 §6 / Best-effort hail - `starcom/AGENT_WHITEBOARD.md`). Same first-impl slot: station has to acquire a vehicle that is already on some PHY. **Either/or for first implementation; FPV scan won.** Hail is not cancelled - it stays the book/MAC path after find, or if scan is not enough.

Stashed as `wip phy-scan leds docs` with both in one Starcom row - split: **scan/find is RC (this board)**; hail write-up is Starcom.

**Find axis = SF×BW at 915.0 / sync `0x12`.** Firmware-legal **6 SF × 3 BW = 18** cells (not hundreds; do not add US915 64-freq hop). CR/power/nav are not find axes. 18 is FPV-goggle sized (Fatshark ~40 ch). Lock/apply uses `radio_config_nav_fits_hz` (125/10 SF7 refused) — same gate as COMM_CHANGE / NAV_PRESET.

**Dual-use (list stays small):** station tool = (1) find *our* vehicle (valid PLTU) (2) show which cells are occupied (RSSI/CAD) like goggles. Prior art: Hertz-Hunter, MikyM0use OLED-scanner / JAFaR (RX5808 RSSI sweep + autoscan), PortaPack FPV Detect (40-ch AutoScan). Pattern: finite table, RSSI bar per cell, lock strongest / first match. Extra vs analog FPV: CRC’d PLTU vs raw energy. **RX-only while sweeping.**

WIP files are gone from the tree; the 2026-09-02 stashes were triaged and archived (CHANGELOG 2026-10-13-001). Not a license to implement this sitting.

### Class D outdoor BW rank (WANTED) (2026-09-03)

125 vs 250 vs 500 at +20 dBm, then step down. Only BW-for-range rank. Desk stays 2 dBm. Procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`. Report: `docs/RADIO_SOAK_PASS_AB_2026-09-03.md`. Not a license this sitting.

### Adaptive TX power (WANTED) (2026-09-03)

Owner-wanted after LoRa Pass A. Not hail. Boot is 2 dBm on both boards now; waits on a SET sender (Power SET row) and the driver rewrite (AO / QEP row).

### Fault beacon last-gasp (HELD)

Held until FPGA PHY work is done. Then **PIO beacon + SPI fallback in one session** — do not SPI-from-fault-handler alone. PIO2 = safety; PIO0 = WS2812; **PIO1 empty on purpose** until this sitting (or a later FSK timing need). `docs/hardware/PIO_BUDGET.md`. Council B.5 / round 3: `docs/decisions/FAULT_RECOVERY_2026-05-14.md`. Not a license this sitting.

### Skills to add (OPEN)

Wanted skills - not written yet. Not a license to author them until scheduled.

- **Session-checklist skill** - grant add-only cadence writes (`CHANGELOG.md`, `PROJECT_STATUS.md`) only when that checklist scope is actually running (commit / push / wrap). Stops jumping the gun on a changelog because the list was *read*. If the protected-file hook is revived on Claude Code, this skill drives that gating. (Moved here from the graphify/hook note.)
- **Council skill** - when the user says “council review” / “panel check,” load `COUNCIL_PROCESS.md` (panel, stop conditions, incomplete-review offer) instead of relying on the agent to remember the file.

### `rp400` git remote = Pi 400 keyboard clone (DEFER) (2026-08-20)

Not a radio chip and not WSL. Git remote `rp400` (`npow@192.168.1.233:~/Rocket-Chip.git`) is an early clone onto the **Raspberry Pi 400** keyboard computer (CYBERDECK HAT/Bonnet on hand - `docs/hardware/HARDWARE.md` Ground Station). Host was off/unreachable 2026-08-20. Local tracking of `claude/tender-banach` was dropped; that branch may still exist on the Pi (Feb 2026 SAD/ESKF, already an ancestor of `main`). **Do not chase it now.** Next time that machine is used - likely Stage 12B Yamcs / OpenMCT / advanced GCS - if the clone has not been fully redone, delete leftover branches there (at least `claude/tender-banach`). CHANGELOG `2026-08-20-004` is the land-time note.

### Safety/ops criticality inventory (OPEN) (landed from walk WB W-15)

Optional project-wide map of **things the system does** (Go/No-Go, launch abort, pyro intent, confidence gate, ESKF healthy, FD HSM, …) -> owning files/APIs. Review priority / gate scope / doc SSOT. Not a C++ or build tier.

**WN tie is weak.** **WN-182**’s real claim is Go/No-Go SSOT; the inventory is an explicit owner tangent. **WN-184** is a load-bearing comment-vs-type contract and only points at W-15 as safety-adjacent. **Do not block** disposing those WNs on creating this list. Seeds if/when built: WN-182, WN-142, WN-172, WN-176, ESKF brake, fault recovery.

### Local-LLM try-later shortlist (OPEN)

Not adopted. Detail: `docs/tools/LOCAL_LLM_COMPANION_RESEARCH.md` §5. WSL `.wslconfig` is **48GB** (Cookbook ~47 GB after refresh).

Cookbook scan-row Download for Devstral hits **official** `mistralai/Devstral-Small-2-24B-Instruct-2512` (no GGUF). Use Direct Download: `unsloth/Devstral-Small-2-24B-Instruct-2512-GGUF`.

- **Devstral Small 2 24B** - owner trying **Q8_0** first (`…-Q8_0.gguf`, ~23 GB). Q4 later for A/B. Do not pull the whole Unsloth repo.
- Qwen3.6-35B-A3B
- Gemma 4 31B QAT-Q4_0 (`google/gemma-4-31B-it-qat-q4_0-gguf`, not Cookbook’s Q4_K_M row)
- Nemotron 3.5 Lightning 30B

### IEEE 1028 review-level -> decision-table mapping (PROPOSED / DEFERRED) (2026-07-04, Claude/Opus)

IEEE Std 1028-2008 recorded as a **review/audit-process** reference in `standards/AUDIT_GUIDANCE.md` Appendix B.5, kept deliberately distinct from the JSF/P10/JPL **coding** standards (how we review ≠ how we write). **Provisional, not a sole standard:** useful but broad, lightly vetted so far - complementary review standards may join it later; this is a starting point, not a settled adoption. **Open rework:** the "When to Do What" decision table in AUDIT_GUIDANCE.md sets review *scope* per trigger but leaves review *depth* implicit. IEEE 1028 names five review levels - management review, technical review, **inspection**, walk-through, audit. Map those onto the 7-tier procedure so each trigger states which depth applies (e.g. the L2-P5 manual walk = an **inspection** per Appendix B.4 "Phase 9"; a small change = a walk-through). Deferred - do when the audit procedure is next reworked. Owner decision on priority.

### ASD-STE100 for documents (LATER) (2026-10-04)

Owner will make ASD-STE100 the wording standard for Rocket-Chip documents and Starcom documents. Not this sitting. Do not start a wide rewrite from this row.

**Already written, do not rewrite:** `starcom/docs/USER_GUIDE.md`. That guide helps a reader who does not know CCSDS. The Version-3 / Version-4 frame choice is one example, not the only added help. That file uses ASD-STE100. The other documents do not.

**When this row is scheduled:** apply the same wording to the remaining Rocket-Chip docs and Starcom docs. Keep technical names and book cites. Do not drop operational numbers to make a sentence shorter. Erase this row when that standard is in force. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

### Differential analog FPV video (Telestar FPV) (LOOK) (2026-10-13)

Nathan, 2026-10-13. For the Telestar FPV board: send analog composite video as a differential signal over a twisted pair (like CCTV video baluns) to reject common-mode noise, with no added latency and no FPGA. Candidate parts: AD8132 driver + AD8130 receiver, both distributor-listed active (ADI pages not reachable; Octopart status ~4 yr old). Pointer only, nothing decided. Background: `starcom/spacewire/README.md`.

### Twiggs Space Lab as a hardware reference (LOOK) (2026-10-25)

Nathan, 2026-10-25: add Twiggs Space Lab as a hardware reference goal; STEM-focused small-sat hardware (supplier for the Aldrin Family Foundation S-Cubed program, per Goddard). Links (Goddard): [FlatSat kit](https://twiggs-space-lab.myshopify.com/products/tsl-flatsat-kit) (add-on connector for LoRa/GPS/SD), [TSLPB payload board](https://github.com/VCSFA-MARS/TSLPB/wiki) (Arduino Pro Mini); Mini Ground Station takes RF98 LoRa modules (SX127x family, but RFM98 is usually the 433 MHz part vs our 902-928 MHz RFM95; band unconfirmed, not plug-in compatible until their spec says so). Current FlatSat MCU and design-file openness not found. Pointer only, nothing decided; no outreach.

## Look (Starcom pointers)

Starcom-only LOOK/OPEN rows live on `starcom/AGENT_WHITEBOARD.md` (same titles): Pseudo-simplex main mode / Ingenuity link; Waveforms stay separate; Half-duplex carrier-only at turnaround; E2d asymmetric option; Hardware map for E2d-style path; Licence / bands; E2d history; UniClOGS / OreSat station reference; Re-evaluate Prox-1 vs USLP; APID-to-path assignment; Space Packet as the core unit; COP-P sync indication.

## Use Cases

1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **Deferred items** - Active intent kept visible until acted on (and then erased - see header rule)

## Medium (session-scale, 4-12 hours)

Pre-dates the FSK pivot; pre-flight hardening items, not scheduled.

- **QP/C naming-convention divergence - TRACKED (2026-06-24, Claude).** QP/C-vs-QP/C++ eval closed: stay on QP/C (`docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`). The L2-P5 naming pass renamed the project's AO/QP code from Samek/QP house conventions to the project's JSF house standard: QP **`l_`** module-static prefix -> `g_` (JSF-209/CODING_STANDARDS:469 static convention); QP state-handler **`Xxx_initial`/`Xxx_running`** CamelCase -> `lower_case` (JSF AV Rule 51); QP **`s_evt`/`tx_evt`** event statics -> `g_`-prefixed. **Why this is safe (researched 2026-06-24, primary sources):** (1) QP/C consumes these as *function pointers* (`Q_STATE_CAST(&FdAo_initial)`) and *variable identifiers* (`&l_fdAo.super`) - names are arbitrary to the framework, only signatures + registration matter; (2) the AO code is **hand-written** (no `.qm` model, no QM-generated banners) so nothing regenerates QP names back; (3) Quantum Leaps publishes their own coding style (QL-C/C++:2022) as **editable markdown explicitly meant to be forked/customized** to a project's house standard. So QP naming is *guidance, not a hard line*, and JSF governs (no accepted-deviation needed). **Watch-items down the line (the reason this is tracked):** (a) QP forum/book examples + `docs/decisions/AO_COMMANDMENTS.md` use Samek naming - a QP-veteran reading our AO code sees house naming instead; (b) **if QM (the QP modeling tool) is ever adopted**, its generated code reimposes QP conventions -> the divergence would resurface as a generated-vs-house conflict (revisit then); (c) stay-on-QP/C is recorded in `docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`. **Formalization TODO:** record this as an "Identifier naming (QP/Samek vs JSF)" bullet in `CODING_STANDARDS.md` -> "Worked consolidation decisions" (alongside the function-pointer P10-vs-JSF and nullptr-vs-JSF-175 resolutions) - that file is PROTECTED, so needs repo-owner to name it for editing.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). RadioScheduler is gone (always-on COP-P). Extend when firmware lands: multi-pending-in-flight, MAVLink parser state, `station_idle_tick` GPS poll interleave.

## Large (multi-session, architectural)

Pre-dates the FSK pivot; each needs a Council review or plan doc first.

- **Real-World Accuracy Tests plan.** Bench-side ground-truth validation - IMU known-angle tilts, baro altitude vs reference, GPS stationary/moving baseline characterization, ESKF replay vs synthetic truth, Allan variance for gyro/accel. Doesn't need launch window or airframe. Needs dedicated plan doc with prior-art research (ArduPilot EKF tuning, PX4 calibration) and equipment assessment.
- **Launch procedure audit items.** Six future safety items from NASA/SpaceX/NAR procedure comparison, all requiring Mission Profile or hardware support:
  1. Angle-rate abort guard (BOOST bank threshold -> ABORT; needs IMU attitude in BOOST)
  2. No-pyro-after-impact guard (landing guard before apogee guard -> suppress pyro)
  3. Hung-fire / ignition timeout (track time since ARM, station-side exclusion timer)
  4. Igniter continuity check (station-side pre-arm check)
  5. Air-dropped vehicle profile (altitude-aware abort, no "stay on ground")
  6. Multi-engine / staging support (partial engine light, inter-stage hold, TRA 13-9)

## Research / Deferred

No code this sitting. Unique leftovers only — last-gasp is **Fault beacon last-gasp (HELD)**; FSK/SDLS/SET OTA are the Starcom board.

- **Datasheet RAG.** Index the PDFs we already use (RP2350, SX1276, Pico SDK, CCSDS books) so register lookups are a query. Pointer not authority (LL 37/38). Independent of local vs cloud LLM (see Local-LLM shortlist). Starcom wire codecs already landed; remaining is datasheet/register RAG, not framing.
- **ELRS on RP2350.** Native ExpressLRS / PIO hop vs Telstar CRSF module. Future radio protocol, not this PHY.
- **PIO SM halt is undetectable** (IVP-130 Scenario 5). Accepted Core/Titan gap; mitigation is a second MCU. Gemini-tier.
- **AON-timer prior-uptime** — stubbed to 0 in the anomalous-boot confidence gate. POWMAN already covers brownout; wire `pico_aon_timer` only if auto-zero-baro false-positives need a second corroborator. `docs/decisions/FAULT_RECOVERY_2026-05-14.md`.
- **Battery ADC.** Hardware not wired. ADC pin + driver + telemetry field.

## Far-future

Mission Profile OTA, F' evaluation (branch `fprime`, CHANGELOG 2026-10-14-001), u-blox GPS, OTA drivers, GPS-free 3D reconstruction, MATLAB export - all tracked in `docs/PROJECT_STATUS.md` future features. FSK bitstream is on the Starcom board.

## Upcoming Stages

IVP stages are done. Current work is pre-flight hardening (link, driver, FSK) to make flight more reliable; the preflight stage has not started. The old Stage 15-17 lists are in git history.
