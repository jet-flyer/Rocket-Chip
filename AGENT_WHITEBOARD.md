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

## Pseudo-simplex main mode / Ingenuity link (LOOK) (2026-10-05)

Owner, this sitting. Two flight modes. Do not start the work from this row.

Passive rocket mode uses Proximity-1 simplex (211.0-B-6 §6.4.4). No hail. One direction.

The main mode may be pseudo-simplex. That name is ours. The pattern is the IRIG 106 rocket pattern: one-way telemetry in flight, two-way on the ground, one fixed pair, settings already known. This row does not select an IRIG 106 waveform. It does not replace the Proximity-1 frames. The half-duplex hitch row still says simplex stays deferred for that sitting. This row is the later mode split.

Look next at the Ingenuity–Perseverance link. It is a fixed pair with that on/off pattern. It is not Proximity-1 and not the rover's Electra relay.

Known so far. Not a design:

- Two identical COTS IEEE 802.15.4 radios, SiFlex 02 (LS Research), about 900 MHz (reported near 914 MHz). One is on the helicopter. One is the Mars Helicopter Base Station on Perseverance. The computers talk to the radios over UART.
- Balaram et al., AIAA 2018-0023, section F: one-way data during the short flights, at 20 kbit/s or 250 kbit/s, design range about 1 km. A secure two-way mode when landed. NTRS 20190026832. DOI 10.2514/6.2018-0023.
- N5BF, "Mars Helicopter Telecom," EME 2024 slides (`https://www.ok2kkw.com/eme/Trenton/papers/N5BF-Mars_Helicopter_Telecom_EME2024.pdf`): the protocol was modified from Zigbee and adapted to a two-point high-throughput link. Packet or ack loss triggers retries. Flight software driver name on those slides is ISF. Helicopter-side telecom mass about 13 g. TX under 3 W. RX under 0.2 W.
- Chahat et al., IEEE Antennas and Propagation Magazine, Dec 2020, "The Mars Helicopter Telecommunication Link." Antennas and the link budget. Not the protocol.

Still unread. Open these before any mode text is written:

- The actual frame, ack, and retry rules of the modified 802.15.4 stack.
- Whether flight "one-way" is the radio receiving off, or the same radio with data in one direction.
- Who starts a contact, and whether the radios are powered only at scheduled times.
- What of that is useful for a fixed pad-to-rocket pair, and what is Zigbee-only.

Do not adopt this radio or this protocol from this row. Do not write it into `starcom/docs/DESIGN.md`. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## Waveforms stay separate (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Four waveforms stay separate. Same power-saving motive does not merge them. Not a design decision. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## Half-duplex carrier-only at turnaround (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Under 211.0-B-6 §6 (and Goddard draft 235.1-R-1 Table 5-9), half duplex radiates Carrier Only (S51) at each turnaround. OPEN / not a product decision. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## E2d asymmetric option (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Books name Category E2d (211.1-B-4 / 211.1-P-4.2 Table 3-1). Working assumption IF chosen later; books give no FSK profile. Not a verified Starcom claim. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## Hardware map for E2d-style path (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Working assumption only, untested. Rocket residual-PSK TX needs SDR/I/Q; RFM95/96 and CC1101 cannot make that TX waveform. Module versus die note (SX1276 / RFM95W / RFM96W). Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## Licence / bands (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Part 97 UHF forward 420–450 MHz; fixed return Ch0/Ch1 in meteorological-aids band. Open licence note; not legal advice. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## E2d history (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. E2d named for microprobes; Greg Kazz SLS-SLP Sep 2016 cite; Green Book 210.0-G-2 descoped FSK case. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.


## UniClOGS / OreSat station reference (LOOK) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Fielded USLP + COP-1 + SDLS over a non-CCSDS PHY (GFSK ~50 kb/s). Not a Prox-1 PHY sample. Yamcs / LimeSDR station reference. Licenses are reference only. LOOK only — nothing decided. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.


## Deviation survey of other projects (OPEN) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Learn from deviations that other open projects state themselves. Record only written deviations with quote and URL. OPEN — nothing decided. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## Re-evaluate Prox-1 vs USLP for Starcom (OPEN) (2026-10-05)

Same row as `starcom/AGENT_WHITEBOARD.md`. Start only after the deviation survey row is done. Prox-1 may or may not fit a rocket-to-pad link versus USLP. Outcome is Nathan's decision. OPEN — nothing decided. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## ASD-STE100 for documents (LATER) (2026-10-04)

Owner will make ASD-STE100 the wording standard for Rocket-Chip documents and Starcom documents. Not this sitting. Do not start a wide rewrite from this row.

**Already written, do not rewrite:** `starcom/docs/USER_GUIDE.md`. That guide helps a reader who does not know CCSDS. The Version-3 / Version-4 frame choice is one example, not the only added help. That file uses ASD-STE100. The other documents do not.

**When this row is scheduled:** apply the same wording to the remaining Rocket-Chip docs and Starcom docs. Keep technical names and book cites. Do not drop operational numbers to make a sentence shorter. Erase this row when that standard is in force. Starcom pointer: `starcom/AGENT_WHITEBOARD.md`, same title.

## GCS standalone app / Hamilton handover (NEXT) (2026-10-01)

**Done, committed locally, not pushed (push held until wrap):**
- `f5a0424` standalone app: `docs/gcs/openmct/launcher/rc_gcs_app.pyw`, `app_shell.html`, `rc_gcs_core.py`. Chrome app-mode window, control server on :8093, ports 5000/8091/8092/8093. The old `rc_gcs.pyw` web launcher is kept. Shortcuts "Rocket Chip GCS" and "Rocket Chip GCS (web launcher)" share one icon.
- `afd18b3` the dashboard defaults to Real-Time.
- `0da60ae` Real-Time window is now the last 5 min ending at now. The old +5 s end offset was arbitrary, not an Open MCT default.

**Open bug:** pinning the running app window to the taskbar saves plain Chrome, not the app. The old tk launcher pinned fine. Planned fix: give the Chrome window its own AppUserModelID matching the shortcut and relaunch details, or go back to a native pywebview window (pywebview 6.2.1 is installed and worked earlier), which pins like the old launcher. Not started. It needs launch/close tests on the PC, so wait until Nathan says he is done testing.

**Unverified by Hamilton:** live station mode in the standalone app; plots with the facsimile feed after the Real-Time default; Real-Time showing on screen; taskbar icon and title.

**CCSDS findings (from Duke, repo @3988ecf):** every on-air packet carries sequence count 0. `byte_pump.cpp` `pump_pack_nav_packet`/cmd/ack set only APID and type. Fix: a 14-bit sequence counter on each APID (APID 0x001 nav; APID 0x003 is sent by two different senders, command from the station and ACK from the vehicle, and how it is counted is a decision needed, see Open item 6 below), plus an optional receive-side gap counter. Until then the walk-test CSV `seq` column is not useful. The firmware change goes through the visible Grok Build TUI. Nathan asked Buzz to write a report for Grok Build and Duke to list easy-win gaps; both replies are awaited.
**Open:** (6) APID 0x003 two senders (decision needed, see below); item 9 below is Not started. Owner: Duke + Hamilton.

**Open item 6 (includes former item 8), APID 0x003 with two senders (133.0-B-2 4.1.3.4.3.3).** Facts: 4.1.3.4.3.3 "Packet Sequence Counts are unique and independent per each user application as identified by the APID and are not shared across multiple APIDs." 4.1.3.4.3.4: continuous (modulo-16384). 2.2.1 NOTE: "two separate managed data paths, one for each direction, should be used". 4.1.3.3.4.2: "The APID shall provide the naming mechanism for the managed data path." Green Book 130.3-G-1 (April 2023) 2.4.1, 2.4.2 and 2.5.3 describe commands and their acknowledgments on separate managed data paths. The text does not scope the count per (APID, direction) or per path, and does not address two distinct senders on one APID (absence in the text). The terms "APID Qualifier" and "Path ID" are not in 133.0-B-2 or 130.3-G-1; they are in Issue 1 (133.0-B-1) only. Quotes: `starcom/docs/CONFORMANCE.md`, APID allocation. No CCSDS interpretation on two senders sharing one APID was located (search not exhaustive). **Question, decision needed:** Does Starcom allow two senders on APID 0x003? Facts: 133.0-B-2 4.1.3.4.3.3 does not address two senders on one APID; no CCSDS interpretation located (search not exhaustive).

**Open item 9, Prox-1 shall-statement sweep (requested by Nathan, 2026-10-01).** Scope: every "shall" in 211.0-B-6 EC1, 211.1-B-4 EC1 and 211.2-B-3, plus 133.0-B-2 EC2 for the Space Packet service, plus other books only where `starcom/docs/CONFORMANCE.md` already cites them. Deliverable: `starcom/docs/SHALL_MATRIX.md`, one row per shall with book, clause number, quoted text, and a status of Implemented / Not yet touched / Incorrectly done, each status backed by a code or test reference; a header table gives each book's title and its one-line description from the ccsds.org publications table. Not started.
**Compliance policy (Nathan, 2026-10-01):** wants FULL Blue Book compliance on a section wherever there is no feasible reason not to. Grok Build task: the 133.0-B-2 Space Packet service layer (Packet Service primitives, Octet String Service, Packet Assembly/Transfer/Extraction/Reception); mandatory rows are listed in the task file. The `CONFORMANCE.md` Known gaps section (seq count always 0, idle flag, no rx gap detection) gets removed once the Grok Build fix is board-verified.
**Grok Build task files (2026-10-01):** `docs/decisions/prompts/SEQ_COUNT_FIX_PROMPT.md` (per-APID, per-direction sequence count, rx_lost, idle flag; Status: Not started, then board verify). `docs/decisions/prompts/SPACE_PACKET_SERVICE_PROMPT.md` (133.0-B-2 service layer; Status: Not started, waits for the sequence-count fix to be verified on the board).

**Parked:** grey no-data look for gauges/LAD; RF page; FDAI 8-ball; GPS-present flag; MAVLink toggle-only; custom Leaflet map spike; phone/responsive support; Yamcs (judged overkill for now).

---
## Half-duplex hitch (NEXT) (2026-09-29)

Table 6-10 rows are in `starcom/src/ccsds/mac.cpp` and `src/starcom_adapt/byte_pump.cpp`. E38 sets persistence and does not reload Send_Duration. A pass token that is the first frame in S62 takes carrier lock, then E49. E42 does not count a failed pass. E50 does. Vehicle receive is one station contact (143 ms). Station receive and the carrier-loss hold are one vehicle contact (1278 ms). Vehicle send stays 1200 ms (N=11). S51, S52, and S58 still put no octets on the air. That stays in the adapter.

**Pad, flight-549a0fe, 2026-09-29:** after a Feather watchdog reboot, 11.4 s of the station pad was 7.45 Hz, median gap 154 ms, with repeated holes of about 616–618 ms. Air sat in MAC A/s50 or s51 through each hole and returned to s60 on the next nav. The station was holding the token.

**Later the same day, commit 37acb21:** the station lets the owed PLCW and the pass token leave in one contact. CLI pad on that image: about 10.4 packets/s, median gap 93 ms, longest 279 ms, CRC flat. The periodic half-second hole was gone on that pad. QGC still showed freezes. Those frames were not soaked, so the cause is still open.

**Ordered reboot, same day:** station already up, then a Feather power-on reset. Passive COM7 pad, 180 s: packets 203→2010, 10.0 Hz, median gap 93 ms, p90 124 ms, longest 338 ms. 99 gaps at least 200 ms, 12 at least 300 ms, none at least 500 ms. CRC errors 0→12. Every redraw was COP-P lock with nav, phase Idle. The longer gaps were MAC A/s50, A/s51, or A/s52, then A/s60. The earlier MAVLink soak had no attitude because the vehicle console was silent.

**Operator after that pad:** a periodic hitch is still there. It is much better than the half-second hole. The remaining rhythm matches one yield per vehicle send window (1200 ms): mostly the low 200s ms, a few near 308 ms.

**Next conversation — FSK:** both SX1276s stay in FSK for a session. No mode change inside a turn. LoRa stays the long-range preset. Convolutional and LDPC encoders stay off the LoRa payload. Do not retune N or vehicle Send_Duration. Simplex and a second radio stay deferred. QGC freezes were not in this pad capture. Opening-shock stays unstaged and off the room-test image.

**byte_pump surgery, same sitting as the FSK pivot (2026-10-01):** `src/starcom_adapt/byte_pump.cpp` is the air adapter and it has turned into a bucket: MIB timers, the LoRa catalog and COMM_CHANGE, the nav/cmd/ack packers, the two sequence lanes, and the send/receive drain. Split that file in the FSK sitting. Where the pump reimplemented a Blue Book procedure that `starcom/src/ccsds/` already provides, use the book path. Known overlap: the three packers and the two sequence lanes versus 133.0 assembly and extraction (`space_packet_service` is host-only and is not called on the air; root `CHANGELOG.md` 2026-10-01-003). COP-P, the MAC, Version-3, and the PLTU codec are already the library. Keep the product pieces in the adapter: the SX1276 bearer, the nav/cmd/ack payloads, the product APIDs, `flight_mac_mib`, and the catalog. `rx_lost` stays on the pump. The service object is about 10 KiB and stays off the 4 KiB stack. Inventory any other home-grown book procedure in that file the same way before removing it. Starcom row: `starcom/AGENT_WHITEBOARD.md` FSK bitstream bearer.

**FSK pivot question, ask when FSK work starts (moved here from the open list 2026-10-01):** Which of the three 211.2-B-3 coding options (no coding, convolutional, LDPC) does Starcom claim for the FSK link? Facts: 211.2-B-3 3.4.2.2 allows exactly one; PICS items 3, 4, 5 are each O.1 (at least one must be supported); 3.4.4.4 "The LDPC Codewords shall be randomized according to 3.4.5" (page 3-9 defines the whole randomizer); there is no randomizer text for the no-coding or convolutional options. Existing FSK text does not choose among the three: `starcom/docs/DESIGN.md` note 2026-08-29 ("211.2 encode on RP/T8; decode on Pi"), `docs/hardware/FPGA/README.md` ("T8 211.2 encode and FSK continuous bitstream"; "Viterbi / LDPC decode (too large for T8)"), `starcom/docs/research/ccsds_domain_claude.md`. Clause quotes: `starcom/docs/CONFORMANCE.md`, 211.2-B-3 notes 2 and 3. Decide when FSK work starts.

**Recorded 2026-10-01, not this sitting’s work:** other physical layers and the sounding-rocket umbilical are written down. `starcom/docs/DESIGN.md` note 2026-10-01, `docs/hardware/HARDWARE.md` Regulatory Notes, `standards/starcom/README.md` URL-only rows. SatNOGS-COMMS and IRIG 106 are not Proximity-1. RCC 319 is a separate termination uplink. Do not start that hardware from this row.

**Chips at this handoff:** Jam `BEC71B8EDC6AEBD1` is running the cadence image. Its banner still reads `0488d60-dirty` because that image was linked before `37acb21`. Feather `02FBDDB8E1CA1281` power-cycled, recovered the radio, and locked. The clean `37acb21` ELF was not written. The last halt-write on record is the opening-shock dirty ELF. Do not flash either board until that image is identified. Local main is ahead of origin and was not pushed.

The procedure countdown is in `tools/spin/proximity1_hd.pml` only. Book, `-DSHORT_WINDOW`, and `-DCODE_E38`: five claims `errors: 0`. `-DCODE_MISS`: `p_no_dual_s50` `errors: 1`, the other four `errors: 0`.

---

## GCS glass: live oMCT PoC (NEXT) (2026-09-20)

**PoC proven (desk):** Master Dashboard QGC-style ring + facsimile OK; live Fruit Jam COM7 into Open MCT works via `stream_mavlink_station.py` (MAVLink USB) when station is in kMavlink. Facsimile: `feed_facsimile.py` + Play http://127.0.0.1:8092/. Serve from `docs/gcs/openmct/` (not hello-world alone) so `../plugins` resolve — URL http://localhost:5000/hello-world/.

**Still to work out (not done):** field mapping / which gauges light from station vs vehicle; RSSI/LQ/SNR on glass when only MAVLink USB (no RADIO_STATUS yet); ANSI `stream_station.py` path vs MAVLink path; layout tweaks Nathan noted.

**Parked firmware:** auto first-STX (0xFD/0xFE) lock into exclusive MAVLink CDC fights ANSI dash / oMCT scrape — make MAVLink toggle-only, off by default (boot stays ANSI). See `src/cli/rc_os.cpp` sniff + `StationOutputMode`. Not a license this wrap.

**Hitch (2026-09-29):** Same link on the ANSI pad and the QGC HUD. After `37acb21` and a station-then-vehicle reboot the pad ran 10.0 Hz for 180 s, median gap 93 ms, longest 338 ms, COP-P lock and nav on every redraw. Operator: a shorter periodic hitch is still visible. FSK is the next conversation. Do not retune LoRa or COP-P from this note.


---

## Room test tomorrow: launch, apogee, landing (NEXT) (2026-09-24)

Untethered vehicle on battery. Live view is the station pad phase. Afterward, plug the vehicle and download the flight log (`g` list, `d` download). Do not USB-tether the toss. Do not open the station COM port if QGC has it.

**Image:** do not flash the dirty tree for this pass. Launch, burnout, apogee, and landing are already on the chip. The opening-shock edit is uncommitted on `main` and is not in that image.

**Arm:** station pad `a`, type `ARM`, Enter. Wait for ACK and `State: ARMED`. Disarm is pad `D`. ARM starts the backup timers: drogue pin can go high at 15 s if apogee has not cancelled it, main pin at 45 s. No ematch on those pins. Disarm before 15 s if the pad has not shown apogee, and disarm again at the end.

**Motions, IMU Z out of the top of the chip:**
1. Launch: snap along that axis. Needs `|accel_z| > 20 m/s²` for 50 ms.
2. Burnout: a moment of freefall (`|a| < 5 m/s²` for 100 ms). Sitting still is about 1 g and will not leave boost.
3. Apogee is locked out for 3 s after launch. The top of a room toss will not be the mark. Catch it and hold still. After the lockout, a quiet board can enter drogue.
4. Landing runs only in drogue or main. About 2 s still (speed under 0.5 m/s) after that can land. The baro path wants 5 s under 0.3 m/s.

**Log after:** phase changes and pyro-fired events are in the flight log. The opening-shock time is not. It is a USB line only (`drogue_open_ms` / `main_open_ms` in `FlightMarkers`, printed as response time). Add a `LogEventId` before any untethered pass that needs that interval on disk. oMCT `chute_detected` is still synthetic.

**Opening-shock code (uncommitted, host-tested, not flashed):** every profile, coast through main. Specific force at or above 19.62 m/s² (2 g, above a settled canopy) and 2 m/s of descent-speed lost, held 20 ms. Does not move the phase and does not treat apogee as an opening. Pyro command stamps moved onto the fire transition (`kTransitionFireDrogue` / `kTransitionFireMain`). Response time is command timestamp to shock timestamp. Thresholds are a first cut, not from a flight.

**Verified 2026-09-24:** host `FlightDirectorTest` opening-shock / apogee-crossing / second-opening, guard tests, action-list tests, `scripts_generated_profiles`. Not on a board. Feather was not on the bus (only Bluetooth COM3).

**Dirty, left unstaged (2026-09-29):** opening-shock detector is still not in a commit and not on the chips. Against `3988ecf`: `scripts/config_wizard/core/{cfg_emitter,derivation}.py`, `src/flight_director/` (actions header, director, state, guards, evaluator), `test/test_{action_executor,flight_director,guards}.cpp`. Profiles, the generated header, and the GCS layout files are clean. Do not fold this into the half-duplex commit.

---

## GCS glass: LCARS skin (WANTED) (2026-09-05)

Desk Open MCT hello-world is on Espresso with stacked RSSI/SNR/Baro + radio overlay. **LCARS skin** (louh/lcars or frame-wrapper) is the next aesthetic pass after more field testing / layout tweaks. Do not start unprompted; themes after ingest was the prior lock and ingest is now desk-proven.

---

## RC_OS / station pad UX (WANTED) (2026-09-18)

Station pad is the Fruit Jam ANSI dashboard (`src/cli/rc_os_dashboard.cpp`); console after `x` is `cli_menus.h`. Not Open MCT.

**Dashboard configurator.** Host tool so the user can lay out the pad (which rows, order) without a firmware sitting each time. **Drag-and-drop belongs on the configurator**, not the USB ANSI pad; the pad just consumes a saved layout. Default **operator pad stays one layout.** Not a license this sitting.

**Named views** (configurator presets, not extra firmware products): flight-only, RF-only, IMU, and a **raw-data / workbench lab** dash (full counters, unscaled fields, RATE-class dumps). Lab view is for the bench, not the range pad.

**ASCII attitude indicator.** Wanted if USB CDC + redraw latency stay acceptable (pad is 1 Hz idle / per-RX). 80-col / ~18-row glass may be the limiter, not the quaternion. Gate on a desk latency/readability check before keeping it. Not a license this sitting.

---

## Hobby secondary link (BT / Wi‑Fi) (WANTED) (2026-09-18)

CCSDS-style split: high-rate TM stays on the Prox-1 LoRa air; a **second physical path** for low-latency ground commands (pad ARM/DISARM, settings) and other short important traffic. Hobby-grade Bluetooth or Wi‑Fi, not a second Starcom MAC. FTS/abort stays onboard. LoRa remains the book range link. Not a license this sitting; PHY/chip TBD.

---

## HD token N: user-facing preset (WANTED) (2026-09-18)

Table and default N=11 live in `src/starcom_adapt/README.md` (RC product). Book MIB/token is `starcom/docs/USER_GUIDE.md` + GLOSSARY. Later configurator or radio preset; do not silent-retune `flight_mac_mib()`. Other latency cut: hobby secondary link row.

---

## FPV-style scan / find (WANTED) (2026-09-02, restated 2026-09-03)

**Coupled with LoRa Hail** (Prox-1 §6 / Best-effort hail - `starcom/AGENT_WHITEBOARD.md`). Same first-impl slot: station has to acquire a vehicle that is already on some PHY. **Either/or for first implementation; FPV scan won.** Hail is not cancelled - it stays the book/MAC path after find, or if scan is not enough.

Stashed as `wip phy-scan leds docs` with both in one Starcom row - split: **scan/find is RC (this board)**; hail write-up is Starcom.

**Find axis = SF×BW at 915.0 / sync `0x12`.** Firmware-legal **6 SF × 3 BW = 18** cells (not hundreds; do not add US915 64-freq hop). CR/power/nav are not find axes. 18 is FPV-goggle sized (Fatshark ~40 ch). Lock/apply uses `radio_config_nav_fits_hz` (125/10 SF7 refused) — same gate as COMM_CHANGE / NAV_PRESET.

**Dual-use (list stays small):** station tool = (1) find *our* vehicle (valid PLTU) (2) show which cells are occupied (RSSI/CAD) like goggles. Prior art: Hertz-Hunter, MikyM0use OLED-scanner / JAFaR (RX5808 RSSI sweep + autoscan), PortaPack FPV Detect (40-ch AutoScan). Pattern: finite table, RSSI bar per cell, lock strongest / first match. Extra vs analog FPV: CRC’d PLTU vs raw energy. **RX-only while sweeping.**

WIP untracked: `src/safety/station_phy_scan.h`, `test/test_station_phy_scan.cpp` (stash also has scan-bar LED + `ao_radio` tick - do not `git stash pop` into a soak wrap). Not a license to implement this sitting.

---

## Class D outdoor BW rank (WANTED) (2026-09-03)

125 vs 250 vs 500 at +20 dBm, then step down. Only BW-for-range rank. Desk stays 2 dBm. Procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`. Report: `docs/RADIO_SOAK_PASS_AB_2026-09-03.md`. Not a license this sitting.

---

## Adaptive TX power (WANTED) (2026-09-03)

Owner-wanted after LoRa Pass A. Not hail. Sit after station 2 dBm ELF is on the Jam so SET is not a +20 dBm desk shot. FSK bearer lives on the Starcom board.

---

## Fault beacon last-gasp (HELD)

Held until FPGA PHY work is done. Then **PIO beacon + SPI fallback in one session** — do not SPI-from-fault-handler alone. PIO2 = safety; PIO0 = WS2812; **PIO1 empty on purpose** until this sitting (or a later FSK timing need). `docs/hardware/PIO_BUDGET.md`. Council B.5 / round 3: `docs/decisions/FAULT_RECOVERY_2026-05-14.md`. Not a license this sitting.

---

## Skills to add (OPEN)

Wanted skills - not written yet. Not a license to author them until scheduled.

- **Session-checklist skill** - grant add-only cadence writes (`CHANGELOG.md`, `PROJECT_STATUS.md`) only when that checklist scope is actually running (commit / push / wrap). Stops jumping the gun on a changelog because the list was *read*. If the protected-file hook is revived on Claude Code, this skill drives that gating. (Moved here from the graphify/hook note.)
- **Council skill** - when the user says “council review” / “panel check,” load `COUNCIL_PROCESS.md` (panel, stop conditions, incomplete-review offer) instead of relying on the agent to remember the file.

---

## `rp400` git remote = Pi 400 keyboard clone (DEFER) (2026-08-20)

Not a radio chip and not WSL. Git remote `rp400` (`npow@192.168.1.233:~/Rocket-Chip.git`) is an early clone onto the **Raspberry Pi 400** keyboard computer (CYBERDECK HAT/Bonnet on hand - `docs/hardware/HARDWARE.md` Ground Station). Host was off/unreachable 2026-08-20. Local tracking of `claude/tender-banach` was dropped; that branch may still exist on the Pi (Feb 2026 SAD/ESKF, already an ancestor of `main`). **Do not chase it now.** Next time that machine is used - likely Stage 12B Yamcs / OpenMCT / advanced GCS - if the clone has not been fully redone, delete leftover branches there (at least `claude/tender-banach`). CHANGELOG `2026-08-20-004` is the land-time note.

---

## First-flight prod strip of test/inject (OPEN) (2026-08-23)

Current `build_flight` ELF **is still development firmware.** Approach A (inject/debug linked, `test_mode_active()` no-op) is acceptable until first flight.

**Before first flight:** a dedicated production image that **omits** test/inject TUs from the link - not “compiled in and gated,” not “ifdef in the tree but we pinky-swear DEBUG is off.” `fault_force_*`, debug mutators, station inject, and the arm gate must not be in that ELF (`nm` / `strings` check). Mechanism (prod CMake preset vs stripped tree) is picked in that sitting, not now.

Does not reopen sitting 11. Does not strip on `main` until that sitting.

**Concerns:** Probe residual power (E2) if the board looks dead after SWD.

---

## Safety/ops criticality inventory (OPEN) (landed from walk WB W-15)

Optional project-wide map of **things the system does** (Go/No-Go, launch abort, pyro intent, confidence gate, ESKF healthy, FD HSM, …) -> owning files/APIs. Review priority / gate scope / doc SSOT. Not a C++ or build tier.

**WN tie is weak.** **WN-182**’s real claim is Go/No-Go SSOT; the inventory is an explicit owner tangent. **WN-184** is a load-bearing comment-vs-type contract and only points at W-15 as safety-adjacent. **Do not block** disposing those WNs on creating this list. Seeds if/when built: WN-182, WN-142, WN-172, WN-176, ESKF brake, fault recovery.

---


## Local-LLM try-later shortlist (OPEN)

Not adopted. Detail: `docs/tools/LOCAL_LLM_COMPANION_RESEARCH.md` §5. WSL `.wslconfig` is **48GB** (Cookbook ~47 GB after refresh).

Cookbook scan-row Download for Devstral hits **official** `mistralai/Devstral-Small-2-24B-Instruct-2512` (no GGUF). Use Direct Download: `unsloth/Devstral-Small-2-24B-Instruct-2512-GGUF`.

- **Devstral Small 2 24B** - owner trying **Q8_0** first (`…-Q8_0.gguf`, ~23 GB). Q4 later for A/B. Do not pull the whole Unsloth repo.
- Qwen3.6-35B-A3B
- Gemma 4 31B QAT-Q4_0 (`google/gemma-4-31B-it-qat-q4_0-gguf`, not Cookbook’s Q4_K_M row)
- Nemotron 3.5 Lightning 30B

---

## IEEE 1028 review-level -> decision-table mapping (PROPOSED / DEFERRED) (2026-07-04, Claude/Opus)

IEEE Std 1028-2008 recorded as a **review/audit-process** reference in `standards/AUDIT_GUIDANCE.md` Appendix B.5, kept deliberately distinct from the JSF/P10/JPL **coding** standards (how we review ≠ how we write). **Provisional, not a sole standard:** useful but broad, lightly vetted so far - complementary review standards may join it later; this is a starting point, not a settled adoption. **Open rework:** the "When to Do What" decision table in AUDIT_GUIDANCE.md sets review *scope* per trigger but leaves review *depth* implicit. IEEE 1028 names five review levels - management review, technical review, **inspection**, walk-through, audit. Map those onto the 7-tier procedure so each trigger states which depth applies (e.g. the L2-P5 manual walk = an **inspection** per Appendix B.4 "Phase 9"; a small change = a walk-through). Deferred - do when the audit procedure is next reworked. Owner decision on priority.

---

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **Deferred items** - Active intent kept visible until acted on (and then erased - see header rule)

---

## Medium (session-scale, 4-12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **QP/C naming-convention divergence - TRACKED (2026-06-24, Claude).** QP/C-vs-QP/C++ eval closed: stay on QP/C (`docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`). The L2-P5 naming pass renamed the project's AO/QP code from Samek/QP house conventions to the project's JSF house standard: QP **`l_`** module-static prefix -> `g_` (JSF-209/CODING_STANDARDS:469 static convention); QP state-handler **`Xxx_initial`/`Xxx_running`** CamelCase -> `lower_case` (JSF AV Rule 51); QP **`s_evt`/`tx_evt`** event statics -> `g_`-prefixed. **Why this is safe (researched 2026-06-24, primary sources):** (1) QP/C consumes these as *function pointers* (`Q_STATE_CAST(&FdAo_initial)`) and *variable identifiers* (`&l_fdAo.super`) - names are arbitrary to the framework, only signatures + registration matter; (2) the AO code is **hand-written** (no `.qm` model, no QM-generated banners) so nothing regenerates QP names back; (3) Quantum Leaps publishes their own coding style (QL-C/C++:2022) as **editable markdown explicitly meant to be forked/customized** to a project's house standard. So QP naming is *guidance, not a hard line*, and JSF governs (no accepted-deviation needed). **Watch-items down the line (the reason this is tracked):** (a) QP forum/book examples + `docs/decisions/AO_COMMANDMENTS.md` use Samek naming - a QP-veteran reading our AO code sees house naming instead; (b) **if QM (the QP modeling tool) is ever adopted**, its generated code reimposes QP conventions -> the divergence would resurface as a generated-vs-house conflict (revisit then); (c) stay-on-QP/C is recorded in `docs/decisions/QP_C_VS_QP_CPP_2026-09-07.md`. **Formalization TODO:** record this as an "Identifier naming (QP/Samek vs JSF)" bullet in `CODING_STANDARDS.md` -> "Worked consolidation decisions" (alongside the function-pointer P10-vs-JSF and nullptr-vs-JSF-175 resolutions) - that file is PROTECTED, so needs repo-owner to name it for editing.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). RadioScheduler is gone (always-on COP-P). Extend when firmware lands: multi-pending-in-flight, MAVLink parser state, `station_idle_tick` GPS poll interleave.

## Large (multi-session, architectural)

Needs council review or planning doc before starting.

- **Real-World Accuracy Tests plan.** Bench-side ground-truth validation - IMU known-angle tilts, baro altitude vs reference, GPS stationary/moving baseline characterization, ESKF replay vs synthetic truth, Allan variance for gyro/accel. Doesn't need launch window or airframe. Complements Stage 18 field tuning. Needs dedicated plan doc with prior-art research (ArduPilot EKF tuning, PX4 calibration) and equipment assessment.
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

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, MATLAB export - all tracked in `docs/PROJECT_STATUS.md` future features. FSK bitstream is on the Starcom board.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** - AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 - evaluate justified vs. bloat).

**Stage 16: Field Tuning** - All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** - IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.

## Exact state (2026-09-20 wrap)
`main` tip after this wrap commit: oMCT Master Dashboard PoC (facsimile + live MAVLink bridge script) + prior station USB MAVLink/QGC work on origin. Live glass = PoC; STX toggle-only + field/RSSI specifics still NEXT (WB). Serve oMCT from `docs/gcs/openmct/`.
