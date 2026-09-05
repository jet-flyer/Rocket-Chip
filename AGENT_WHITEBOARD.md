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

## Exact state (2026-09-05 wrap)

Desk oMCT **Master Dashboard MVP** on main (push with this wrap). Single Flexible Layout home: phase LAD (named ARMED/BOOST/...), traj/dyn, dual-axis RSSI|SNR radio, gauges (telem source fixed), bottom Master Caution Display Layout strip. Facsimile on-demand Play :8092 / WS :8091; Reset Unicode crash fixed. Tip includes gauge/caution fix 89918d\ plus this wrap. **MVP not finished polish.** **Next:** verify live board USB/m (or flight) into the same glass path - facsimile path only so far. Pi Zero 2W: static nginx only, not Master-in-browser (~121MB serve + ~34MB feeder host; Chrome renderers 200-600MB). LCARS + passive Estes chute still WANTED below. Untracked local build_* / starcom/build-* dirs only (not for commit).

---

## Next after Pass A/B soak (OPEN) (2026-09-03; R-32 rate closed 2026-09-04)

Scored. Report: [`docs/RADIO_SOAK_PASS_AB_2026-09-03.md`](docs/RADIO_SOAK_PASS_AB_2026-09-03.md). Procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`. **Product boot on `main`:** 250 kHz / 10 Hz SF7 expedited + sparse PLCW (R-32 closed; CHANGELOG `2026-09-04-002`). 125/10 does not fit paper ToA.

Owner: Prefer MET + GPS on both ends; desk oMCT Master Dashboard is **MVP** on facsimile - live board pipe next; LCARS still open.

**Next to address (fresh sitting):**

1. Command AD / `V(S)=0` / SET hop - OTA radio settings still fiction.
2. Class D outdoor - 125 vs 250 vs 500 at +20 dBm, then step down. Only BW-for-range rank.
3. FSK (below).
4. **FPV-style scan / find** (below) - first impl vs LoRa Hail; FPV scan won. Hail still coupled.
5. Adaptive TX power.

## GCS glass: live board -> Master Dashboard (NEXT) (2026-09-05)

Desk **Master Dashboard MVP** is up on facsimile (Flexible Layout + on-demand feeder). **Not verified:** piping a live board (station USB/`m` or flight) into the same glass path. That is the next glass sitting - confirm RSSI/baro/phase with real RF, not only Big Daddy CSV.

Deferred polish (not blocking board-val): Condition Set + alphanumeric caution tiles (digits + color), LCARS, Zero static-nginx smoke.


---

## Passive Estes first flight: chute detect (WANTED) (2026-09-05)

Nathan (2026-09-05): **first flight is a passive motor Estes-style rocket** (single chute, **no pyro**). FD today is dual-deploy pyro (drogue/main fire on phase transitions). Need a **passive chute-detect / recovery-phase path** before that flight - post-apogee drag jump / |Vvel| collapse (baro + fusion), not pyro events. oMCT glance CSV uses layout-only synthetic chute_detected until this lands.

Not a license to implement this sitting. Owner: FD / fusion sitting when scheduled.

---

## GCS glass: LCARS skin (WANTED) (2026-09-05)

Desk Open MCT hello-world is on Espresso with stacked RSSI/SNR/Baro + radio overlay. **LCARS skin** (louh/lcars or frame-wrapper) is the next aesthetic pass after more field testing / layout tweaks. Do not start unprompted; themes after ingest was the prior lock and ingest is now desk-proven.

---

## FPV-style scan / find (WANTED) (2026-09-02, restated 2026-09-03)

**Coupled with LoRa Hail** (Prox-1 §6 / Best-effort hail - `starcom/AGENT_WHITEBOARD.md`). Same first-impl slot: station has to acquire a vehicle that is already on some PHY. **Either/or for first implementation; FPV scan won.** Hail is not cancelled - it stays the book/MAC path after find, or if scan is not enough.

Stashed as `wip phy-scan leds docs` with both in one Starcom row - split: **scan/find is RC (this board)**; hail write-up is Starcom.

**Find axis = SF×BW at 915.0 / sync `0x12`.** Firmware-legal **6 SF × 3 BW = 18** cells (not hundreds; do not add US915 64-freq hop). CR/power/nav are not find axes. 18 is FPV-goggle sized (Fatshark ~40 ch).

**Dual-use (list stays small):** station tool = (1) find *our* vehicle (valid PLTU) (2) show which cells are occupied (RSSI/CAD) like goggles. Prior art: Hertz-Hunter, MikyM0use OLED-scanner / JAFaR (RX5808 RSSI sweep + autoscan), PortaPack FPV Detect (40-ch AutoScan). Pattern: finite table, RSSI bar per cell, lock strongest / first match. Extra vs analog FPV: CRC’d PLTU vs raw energy. **RX-only while sweeping.**

WIP untracked: `src/safety/station_phy_scan.h`, `test/test_station_phy_scan.cpp` (stash also has scan-bar LED + `ao_radio` tick - do not `git stash pop` into a soak wrap). Not a license to implement this sitting.

---

## Station LED: COP-P lock vs RSSI (WANTED) (2026-09-03)

Owner: RSSI LEDs can be yellow/green (LoRa heard) while COP-P is **waiting peer PLCW**. Desk 2026-09-03: after Jam reset, RX climbing CRC=0 / yellow LEDs, no lock until vehicle USB replug. Wanted: keep current RSSI colour, **0.5 Hz on/off blink when not locked**. Not a soak step. Not a license to implement this sitting.

---

## FSK mode (WANTED) (2026-09-02)

Owner-wanted after LoRa Pass A. SX1276 FSK (packet and/or continuous bitstream - IVP-63 / ADVANCED_SETTINGS placeholder). Not tonight's LoRa SF/BW matrix. Not hail. Not a license to mint SF/BW. Sit after station 2 dBm ELF is on the Jam so SET is not a +20 dBm desk shot.

---

## Pre-commit vehicle bench_sim on station-only firmware (OPEN) (2026-08-27)

Hook treats any `src/drivers` / `src/main` touch as "run vehicle `bench_sim`". Station-job diffs still demand COM5. HEALTH-ring reclassify is no longer the reason (Core1 vitality + `passive_dump_needs_help`). Next: role-aware gate (`station_bench_sim` on COM7 for station-job diffs). Hook vs canary/recovery is documented in `standards/HW_GATE_DISCIPLINE.md` Rule 5.

---

## Audit all LESSONS_LEARNED entries for stale assumptions (OPEN) (2026-09-01)

LL entries get cited as current fact, but many are old and rest on assumptions the
tree has outgrown. Two hit this sitting: **LL 20** (2026-02) prescribes 32-byte
chunked PA1010D reads - but the MT3333 vendor says partial reads are "not
recommended", and Adafruit's 32 is just the Arduino `Wire.h` transfer limit, not a
reasoned departure. **LL 24** (2026-02) says never call `i2c_bus_recover()` in a hot
loop - **DISPROVED 2026-09-01 by measurement.** Removing the per-timeout
`i2c_bus_recover()` took IMU failures from 54% to **98.4%** (1101 reads / 68149 errs):
a timed-out transfer leaves DW_apb wedged until reinit, so recovery there is
load-bearing on the current bus layer. LL 24 predates the stretch-aware rewrite and
the 9-clock deletion. **LL 25** is already marked SUPERSEDED, which shows the rot is
real and unevenly tracked.

Pass should: date-check each entry against current code, add supersession headers in
place (LL 25 is the pattern), and flag any whose *mechanism* no longer exists. Do not
rewrite history. Append-only file - owner names it before editing.

---

## Skills to add (OPEN)

Wanted skills - not written yet. Not a license to author them until scheduled.

- **Session-checklist skill** - grant add-only cadence writes (`CHANGELOG.md`, `PROJECT_STATUS.md`) only when that checklist scope is actually running (commit / push / wrap). Stops jumping the gun on a changelog because the list was *read*. If the protected-file hook is revived on Claude Code, this skill drives that gating. (Moved here from the graphify/hook note.)
- **Council skill** - when the user says “council review” / “panel check,” load `COUNCIL_PROCESS.md` (panel, stop conditions, incomplete-review offer) instead of relying on the agent to remember the file.

---

## Starcom (OPEN) (2026-09-02)

Starcom-only flags live on [`starcom/AGENT_WHITEBOARD.md`](starcom/AGENT_WHITEBOARD.md). Sequence: [`starcom/docs/IVP.md`](starcom/docs/IVP.md). Product `starcom-v0.2.25`. RC consumer is **on `main`** (`ROCKETCHIP_USE_STARCOM` default OFF = STOP-GAP).

**Starcom library tree:** `C:\Users\pow-w\Documents\starcom_dev` (`grok/sc-dev`). One tree. Not a `Rocket-Chip-*` folder. `docs/starcom-sad-draft` and `grok/starcom-ivp23` are leftovers to drop after this sitting's port.

**Still RC:** two-board ON soak - station COP-P cmd SDU (ARM) after vehicle nav has given RfManager an anchor. Do not drain COP-P when the station TX window is 0 (Radio would drop the PLTU and FOP-P would never resend: synch_timeout=0). Next RC feature when scheduled: radio settings OTA on the ON path (`starcom/AGENT_WHITEBOARD.md`). FPGA PHY/decode held. Consumer guide: `starcom/docs/USER_GUIDE.md`.

---

## Station RSSI LEDs vs Starcom (OPEN) (2026-08-28)

Station/relay NeoPixel bar (`ao_radio.cpp` `handle_rssi_bar` -> `ws2812_set_rssi_bar`) is **last LoRa FIFO RSSI**, any payload. ON air is COP-P / PLTU. Rewire so the bar (and “no signal”) is gated on **decoded CCSDS/Starcom** - a COP-P lock or accepted nav SDU - not raw radio ticks. Dashboard `RSSI:` / `Pkts:` have the same leftover. Not this dashboard-counter sitting.

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

## Notify / LED system overhaul (OPEN) (2026-08-24)

AO_Notify + `led_patterns.h` + AO_LedEngine need a dedicated sitting, not more overlay nits.

**Known split to keep:** Stage L ARMED is **red solid** (APM2 LED A / traffic-light “motors live”). Pixhawk RGB standard is **solid green** with GPS 3D / **solid blue** without. Do not flip ARMED to green in overlay remediates.

Also in that sitting: USER_GUIDE Armed still says yellow; `kLedPhaseFault` dropped (`347f0a4`) - 28 is AP pre-arm yellow double-flash only; failsafe/EKF stay Notify `FaultIntent`.

---

## L2-P5 leftover sittings (OPEN) (2026-08-24)

Homes for rem WB rows that had no other project-WB row. Erase a bullet when that sitting lands.

- **HAB `EMERG_DEPLOY`** - unread flag removed. When HAB is scheduled: profile bit **and** combinator lockout skip (rocket still locked; HAB skip tested).
- **PIO WDT** - not Tier 1 Go/No-Go. Dedicated sitting: role vs ARM, CLI, whether a PIO WDT fault is pad-blocking.
- **Estes vs station ARM** - Go/No-Go is station pad control. Vehicle USB ARM is bring-up, not the Estes wire-arm procedure.
- **Tiny 2350 pin map (GWF-038 / GWF-039)** - I2C SCL and PSRAM CS both GPIO 21; `board_led_set` ignores polarity. Live hazard gated. Legal QMI CS1: 0 / 8 / 19 / 47. Do not invent a pin.

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

## Early-impl / rework-eval candidates (OPEN) (grouped 2026-08-05)

**What this is:** One place that lists systems which **work today** but were chosen early
(or sit on early HW/paths) and deserve a **deliberate re-evaluation** - not fire drills,
not mid-walk rewrites. Outcome of each eval is keep-with-written-*why*, or planned rework
(plan ± council before code).

**Rule:** Do **not** half-refactor live flight paths for “cleanup.” Detail lives in the
named sections / WNs / Research rows below - this header is the **index + grouping**, not
a replacement. If the set grows unwieldy, break out a dedicated doc later; **not now**.

**Shared constraint - PIO budget:** several candidates prefer or require PIO. Budget is
shared (watchdog + backup timers already on PIO2; beacon candidate wants PIO0/1). Eval
order for PIO-touching items should weigh **advantages vs remaining SM/instruction budget**
together, not in isolation.

KEEP closed 2026-08-31 (DW_apb I²C, Hamilton quat, seqlock, PCM, flash layout, PIO backup-timer design, RFM95W): `docs/audits/EARLY_IMPL_REWORK_2026-08-31.md`. This table is only sittings that remain.

| Candidate | Prefer / lean | Full detail |
|-----------|---------------|-------------|
| **Fault beacon (last-gasp)** | **Held until FPGA PHY work is done.** Then PIO beacon + SPI stop-gap in one session. | Research row *PIO beacon + SPI last-gasp* below |
| **RC_OS / CLI “pseudo-OS”** | Sitting in progress (`grok/rcos-rework`). | **§ RC_OS Rework** below |
| **Radio / telem surfaces** | Drivers KEEP. Remaining: Starcom ON two-board soak, radio-settings OTA. | Starcom WB |
| **CCSDS TC + COP-1** | KEEP defer post-Stage-17. Starcom library already has COP-P/COP-1. | Project status line |

**Not in this group:** pure process/tooling OPEN items (graphify re-pass, commit hygiene,
worktrees, IEEE 1028 mapping), accepted Gemini-tier PIO gaps, or active L2-P5 walk handoff.

**Add a member:** append a row here + keep/expand the detailed section or WN; do not scatter
new “maybe rework someday” bullets without listing them in this table.

---

## RC_OS Rework (OPEN) (2026-07-09, from CODE_TRIMMING §2)

**Group:** Early-impl / rework-eval candidates (see index above).

**Origin:** 2026-07-03 code-trimming / staleness survey noted CLI “morphed almost into a pseudo-OS” (`docs/audits/CODE_TRIMMING_AUDIT_2026-07-03.md` §2). Not a scheduled Stage/IVP yet.

**Intent:** Future workstream - structure RC_OS more like a proper UX/OS layer than accretion of single-key handlers: table-driven key->handler maps, clear UX vs domain ownership (AO_RCOS vs cal_manager vs FD), station/vehicle gating without copy-paste, host-testable dispatch. Entry docs: `docs/ROCKETCHIP_OS.md`, `docs/AO_ARCHITECTURE.md`, CODE_TRIMMING §2.

**Rule:** Do **not** half-refactor live menus for LOC first; proven-dead CLI symbols may still be deleted. Needs own plan + council before code.

---

## Graphify full re-pass (OPEN) - owner-gated

L2-P5 formal walk closed **2026-08-17**. Full /graphify remains **owner-gated** (billed). Cheap graphify update + curate already runs post-commit.

Still open if/when you schedule it: **doc->code connectivity pass** (standards/docs islands vs code symbols; rate-limit = batches of ~4). Not a sitting default.

---

## IEEE 1028 review-level -> decision-table mapping (PROPOSED / DEFERRED) (2026-07-04, Claude/Opus)

IEEE Std 1028-2008 recorded as a **review/audit-process** reference in `standards/AUDIT_GUIDANCE.md` Appendix B.5, kept deliberately distinct from the JSF/P10/JPL **coding** standards (how we review ≠ how we write). **Provisional, not a sole standard:** useful but broad, lightly vetted so far - complementary review standards may join it later; this is a starting point, not a settled adoption. **Open rework:** the "When to Do What" decision table in AUDIT_GUIDANCE.md sets review *scope* per trigger but leaves review *depth* implicit. IEEE 1028 names five review levels - management review, technical review, **inspection**, walk-through, audit. Map those onto the 7-tier procedure so each trigger states which depth applies (e.g. the L2-P5 manual walk = an **inspection** per Appendix B.4 "Phase 9"; a small change = a walk-through). Deferred - do when the audit procedure is next reworked. Owner decision on priority.

---

## L2-P5 §CM to-implement backlog - mechanical checks surfaced by the walk-guide trim (2026-07-01, Claude)

Trimming `L2P5_MANUAL_WALK_GUIDE.md` down to human-judgment-only lenses (on branch `claude/l2p5-walk-guide-c3757857`, recovered + committed 2026-07-01) surfaced a set of **mechanically-decidable** checks that had been riding inside the manual walk. Per the coverage≠mechanical principle, each belongs in a gate/grep, not the eyeball pass. **Repo-owner direction: wire these when patching §CM *after* the manual walk - likely as part of re-doing the audit**, not now.

**To implement (currently ungated / partial):**
1. **Commented-out code** (CERT MSC04-C / JSF 127) - greppable (a block of real C++ inside `//` or `/* */`) + the nested-`/*` delimiter hazard. Manual residual kept in Class 3: delete-vs-deliberate-`#if 0` disposition.
2. **Per-function assertion count / >10-line trigger** (P10-5 / JPL-16 density floor) - countable/greppable; function length is already measured by `readability-function-size`, the per-fn assertion count is not.
3. **`template<>` explicit function-specialization** (CCG T.144) - grep the banned token; presence = finding.
4. **static_assert-guard presence** on POD types meant to model a concept (CCG T.150) - greppable; `misc-static-assert` is enabled but doesn't cover "a guard exists for type X."
5. **Per-instantiation test coverage** (JSF 102 / T.102) - compiler emits the instantiation list; needs a coverage manifest to auto-check.
6. **lock-in-A / unlock-in-B cross-function pairing** (JPL 9 / CON51) - grep or a custom clang-tidy matcher (triage's stated conversion). Live surface: 1 `save_and_disable_interrupts`/`restore_interrupts` pair in `psram_init.cpp`, 0 spinlock pairs. Manual residual kept in Class 10: every abnormal exit releases the lock.
7. **JSF AV 166 - side-effects-in-`sizeof`** (canon; = the `sizeof` case of CERT EXP52) - greppable; `bugprone-sizeof-expression` is enabled but targets broader suspicious-`sizeof`, not side-effect-in-operand. **0 current violations in `src/`** (zero-risk gap). Surfaced 2026-07-01 by the I.4/EXP52 canon re-check.

**Already gated (no action - recorded for the split):** the other trimmed items already hit enabled gates - `bugprone-assert-side-effect`, `-Wnon-virtual-dtor`, `cppcoreguidelines-special-member-functions`, `google-explicit-constructor`, `readability-container-size-empty`, `-Wall`->`-Wsequence-point`, `cppcoreguidelines-init-variables`+`clang-analyzer`, `readability-function-size`/`cognitive-complexity`, `bugprone-branch-clone`, `clang-analyzer-deadcode`, `[[nodiscard]]`, **`readability-magic-numbers` (Class 13 demoted 2026-07-28; residual notes on `.clang-tidy` + CODING_STANDARDS)**. **Class-6 narrowing** (56 `-Wconversion`/`-Wsign-conversion` findings) is already tracked in the L2-P5 handoff above + `L2P5_WCONVERSION_FINDINGS_2026-06-25.md`.

**Class-14 do-not-"fix" guards (cut from the walk guide 2026-07-01; preserved here - they're gated-class cautions, not walk criteria):** while remediating or gating the mechanical surface, do **not** (a) convert mandated `uintN_t` / hardware-register / bitmask code to signed, or (b) add redundant arithmetic parentheses (JSF AV 213 is deliberately disabled - LL Entry 26). These guarded against a reviewer over-"fixing" the gated Class-14 (expressions / evaluation-order) surface; kept in case that surface is touched.

---

## Use Cases
1. **Cross-agent review** - Flag concerns about other agents' work (see `CROSS_AGENT_REVIEW.md`)
2. **Cross-context handoff** - Notes for future Claude sessions when context is lost
3. **Work-in-progress tracking** - Track incomplete tasks spanning multiple sessions
4. **Hardware decisions pending** - Flag items needing user input before code changes
5. **Deferred items** - Active intent kept visible until acted on (and then erased - see header rule)

---

## High priority

- **AO Commandments source-citation audit.** Investigating R-27 (RfManager Commandment XII observation) surfaced that Commandment XII's `Source:` line cites LL Entry 36, but LL 36 is about test-tool rot (bench_flight_sim.py going stale), not AO state-transition logging or runtime observability. A research agent walked the doc's stated sources (Samek PSiCC2 Ch. 11, state-machine.com Active Object/RTEF/QP/C SRS pages, NASA F´ Code Style + State Machines doc) and confirmed **no clean substitute citation exists in any of those** - the rule is project-internal invention generalized from folklore, not inherited from external authority. This is an [LL Entry 37](docs/agents/LESSONS_LEARNED.md)-class citation-rot finding. Per Entry 37 discipline ("if one citation was wrong, check the rest"), audit all 12 Commandment `Source:` lines in `docs/decisions/AO_COMMANDMENTS.md` against their cited sources; fix XII's citation (either reframe as project-internal "Rationale:" or cite PSiCC2 Ch. 11 honestly as topical-but-tool-framing); reassess R-27's disposition once the rule's authority is correctly understood. Est. ~1-2 hrs. Block on this is open per user direction 2026-05-22 - address before closing R-27.

- **Four-cycle plan - Cycle 4:** L2-P5 walk + remediates + CLA/RBM on `main`. **L2-P5 and L2-P10 closed 2026-08-24** in `PROBLEM_REPORTS.md`. Cycles 1-3 closed. Catchup doc never written as a separate `AUDIT_COVERAGE_CATCHUP_*.md` - evidence lives in the walk pack, CLA snapshot, and `docs/RBM/`.


- **IVP-T13 LQ-adaptive retry - deferred until after the CCSDS command-
  layer rework.** Original Stage T Batch C plan was to port the ELRS
  LQCALC pattern (retry aggressiveness scales U-shape with LQ: fewer
  retries when LQ is high, more when marginal, pause when LQ is near-
  dead) behind `ROCKETCHIP_LQ_ADAPTIVE_RETRY`, default OFF. Decision
  2026-04-22: don't polish parametric tuning on top of a retry
  architecture we've already marked as STOP-GAP pending proper CCSDS
  TC-Layer + COP-1. T13 reopens once the CCSDS command path lands -
  the adaptive algorithm likely maps onto whatever flow-control / FARM
  machinery the CCSDS layer provides, not onto naive retry counts.
  `AO_RfManager_ok_to_retry()` API already exists in-tree, ready for
  use when T13 returns.

- **Station->vehicle radio health channel - deferred to CCSDS rework batch.**
  Council A3 asked for condensing station readiness to a single bit the
  vehicle's GO/NO-GO consumes via radio. Current channel is command-only,
  no periodic telemetry-back. Moved into the CCSDS rework batch (with
  IVP-T13 + Stage T re-baseline) on 2026-05-21 - the telemetry-back
  direction wiring depends on what TC-Layer / TM-Layer split the CCSDS
  rework lands, so designing it now would commit to an interface that
  the CCSDS work will likely change.

- **Re-evaluate Stage T "95% first-try" gate with correct baseline.**
  User observation 2026-04-22: the Stage T diagnostics measured
  operator-burst ACK rate (10 Hz retry over ~300 ms), which treats "3rd
  retry succeeded at 300 ms" as a failure. But on a half-duplex LoRa
  link with sparse station TX (no heartbeat before IVP-T14d), a
  3rd-retry ACK latency of ~300 ms is within ABORT's 250 ms budget +
  reasonable operational margin. The 6.7% first-try number conflates
  "link broken" with "expected half-duplex latency for a burst into a
  sparse RX window." Once CCSDS station beacon lands, re-measure: (a)
  steady-state first-try success when station is continuously TXing;
  (b) actual ACK-latency distribution (p50 / p95 / p99); (c) whether
  the anchor-station-TX-to-vehicle-RxDone architectural fix is still
  needed or was over-engineered for our actual link conditions.

## Medium (session-scale, 4-12 hours)

Scope is clear but touches multiple files, needs verification, or has small design questions.

- **QP/C vs QP/C++ framework evaluation (2026-06-23, Claude).** **WN-052 lives here** (original claim: defer deep redesign of `ao_signals.h` signal catalog / event structs / `evt_cast` until QP/QF). Not a P10-9 pointer WN. FD/`action_executor` C-HSM callbacks may ride the same eval. The project uses **QP/C** (C framework) from C++ TUs - confirmed by first-member event composition (`struct PhaseChangeEvt { QEvt super; ... }`, `include/rocketchip/ao_signals.h`) + the "Allocated from QP/C dynamic event pool" comment + free `QActive_publish_()` API. The FreeRTOS->QP migration was council-reviewed, but **no record exists of the QP/C-vs-QP/C++ sub-choice being weighed or surfaced to the user** - it appears to have been made implicitly during the migration. The choice seems sound and works, but an seemingly-arbitrary undocumented decision of this size is a red flag (user direction 2026-06-23). **Do a full QP/C vs QP/C++ evaluation**: what each buys/costs for this codebase (event ergonomics, type-safety, the `(void *)0`-vs-`nullptr` and reinterpret_cast-downcast idioms QP/C forces, framework-core C++ runtime footprint, JSF/MISRA posture, migration cost if switching), whether the implicit choice was correct, and **record the rationale** (back-fill a decision doc either way). Surfaced during the L2-P5 standards walk while dispositioning the QP/C `(void *)0` sender idiom. **Concrete standards data point for the eval:** QP/C's first-member event model forces `reinterpret_cast<const DerivedEvt*>(e)` downcasts of `QEvt const*` (~10 AO sites, being centralized into one `evt_cast<E>` helper) - a standing **JSF-182** residual (no Exception covers it; well-defined only via standard-layout first-member equivalence). In **QP/C++** these become real inheritance downcasts (`static_cast`, JSF-178-compliant), **eliminating the JSF-182 residual outright**. So the C++ edition would retire a coding-standard exception - weigh against migration cost in the eval.

- **QP/C naming-convention divergence - TRACKED (2026-06-24, Claude).** The L2-P5 naming pass renamed the project's AO/QP code from Samek/QP house conventions to the project's JSF house standard: QP **`l_`** module-static prefix -> `g_` (JSF-209/CODING_STANDARDS:469 static convention); QP state-handler **`Xxx_initial`/`Xxx_running`** CamelCase -> `lower_case` (JSF AV Rule 51); QP **`s_evt`/`tx_evt`** event statics -> `g_`-prefixed. **Why this is safe (researched 2026-06-24, primary sources):** (1) QP/C consumes these as *function pointers* (`Q_STATE_CAST(&FdAo_initial)`) and *variable identifiers* (`&l_fdAo.super`) - names are arbitrary to the framework, only signatures + registration matter; (2) the AO code is **hand-written** (no `.qm` model, no QM-generated banners) so nothing regenerates QP names back; (3) Quantum Leaps publishes their own coding style (QL-C/C++:2022) as **editable markdown explicitly meant to be forked/customized** to a project's house standard. So QP naming is *guidance, not a hard line*, and JSF governs (no accepted-deviation needed). **Watch-items down the line (the reason this is tracked):** (a) QP forum/book examples + `docs/decisions/AO_COMMANDMENTS.md` use Samek naming - a QP-veteran reading our AO code sees house naming instead; (b) **if QM (the QP modeling tool) is ever adopted**, its generated code reimposes QP conventions -> the divergence would resurface as a generated-vs-house conflict (revisit then); (c) pairs with the QP/C-vs-QP/C++ eval above. **Formalization TODO:** record this as an "Identifier naming (QP/Samek vs JSF)" bullet in `CODING_STANDARDS.md` -> "Worked consolidation decisions" (alongside the function-pointer P10-vs-JSF and nullptr-vs-JSF-175 resolutions) - that file is PROTECTED, so needs repo-owner to name it for editing.

- **Station SPIN model extensions.** Scaffolding landed (IVP-147: P_TERMINATION + P_NO_DOUBLE_CLEAR, both PASS). Extend when corresponding firmware behavior lands: multi-pending-in-flight, RadioScheduler TX-window arbitration (needed for the sync-gap fix), MAVLink parser state, `station_idle_tick` GPS poll interleave.

- **`LESSONS_LEARNED.md` cleanup + update pass (2026-08-20).** Medium: the file is the process journal agents actually re-read, and it has aged in place. **Not** a rewrite of historical entries (append-only / frozen existing text). Pass should: (a) refresh the header (still says "so Claude can learn"; format claims every entry has time-spent, many don't); (b) kill or update the stale **Plan Files Reference** (`virtual-scribbling-finch.md` / AP_Vehicle, 2026-01-31 - almost certainly dead); (c) navigability - Entry 34 sits after 35; process/tooling lessons (36-45) sit in a HW-debug journal with no index; superseded Entry 25 is marked, check for other rot; (d) confirm FreeRTOS-archived numbers 7-10 / 14 / 17-19 still make sense as holes vs a short "see branch" pointer. Owner names the file when the sitting runs. Do not start this unprompted.

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

No code changes planned - kept as context for future decisions.

- **Datasheet RAG + spec-table->code transcription - evaluate (2026-07-28, Claude/Opus).** Two related ideas surfaced while scoping a local-LLM companion workflow; **both are worth visiting regardless of whether the model is local or cloud** - the value is in the pattern, not the hosting.
  1. **Datasheet RAG.** Stand up a retrieval index over the primary-source PDF set we already lean on (RP2350 datasheet, SX1276, Pico SDK docs, and for Starcom the CCSDS Blue Books) so register/bit/field lookups are a query instead of a page-hunt or a context-burning full-PDF read. Directly attacks the friction documented in [[l2p5-standards-walk-guide]]'s source-fetch method note (WebFetch's extractor fails on the compressed standards PDFs; pypdf + regex is the current workaround). Local embedding models are cheap enough (~0.6B, ~1.5 GB) that this is not a token-cost decision. **Caution:** RAG retrieval is a *pointer*, not an authority - per LL Entry 37 + 38, any rule/register value it surfaces still gets verified verbatim against the primary source before it lands in a doc or a citation. A summarizer already fabricated P10 quotes once.
  2. **Spec-table -> code, compiler-verified.** Transcribe tabular layout specs (CCSDS Blue Book bit layouts; RP2350/SX1276 register bitfields) into C++ structs + `static_assert` on `sizeof`/`offsetof`/field masks. The interest here is that the output is **machine-checkable** - the compiler proves the layout, so this is delegation with a real gate rather than trust. **Bounded claim (important):** `static_assert` proves *layout*, NOT *semantics* - a field transcribed at the correct offset under the wrong name or meaning still passes. So the gate covers the mechanical half only; field identity/meaning stays a human/primary-source check. Do not let "static_assert clean" become a Rule-7-style over-claim (see `RULE_VERIFIABILITY_TRIAGE.md` severity model: over-claiming tool-claims are critical findings).

  **Not scheduled, no code planned.** Most load-bearing for Starcom (strict Blue Book bit-layout fidelity is the whole point of the library) - evaluate before Starcom's data-link framing code locks in.

- **ELRS on RP2350 - research item.** Running ExpressLRS natively on RP2350 with PIO-assisted frequency hopping. Current RFM95W (bare SX1276 on SPI) may be compatible if packet format + hopping schedule can be implemented in firmware. Telstar Booster Pack already describes CRSF/UART to a dedicated ELRS module as the alternative path. Future radio protocol investigation.
- **PIO hardware failure gap - Gemini tier only.** IVP-130 Scenario 5 confirmed: external PIO SM halt is undetectable by firmware (PIO watchdog IRQ only fires from PIO program itself; ARM-side monitoring defeats the independence point). Correct mitigation = physical redundancy (second independent timer on separate MCU). Gemini-tier feature (dual-core carrier board). Accepted gap for Core/Titan.

- **Fault-recovery rework follow-ups (commits `ed7c569` + `8baa18a` landed 2026-05-14/15; see `docs/decisions/FAULT_RECOVERY_2026-05-14.md` for the design):**
  - **PIO beacon + SPI last-gasp combined session** - see dedicated row below.
  - **AON-timer prior-uptime signal** - stubbed to 0 in the anomalous-boot confidence gate. Wiring it requires adding `pico_aon_timer` to target_link_libraries + explicit timer-start at boot. Marginal value (POWMAN reset register already carries the high-confidence signal for brownout; AON timer would corroborate for the watchdog-RSM / hazard-DP / glitch-detect / SWcore-PD reset classes only). Worth picking up if auto-zero-baro suppression false-positive rate during bench testing needs an extra corroborator. Otherwise deferred.

- **PIO beacon + SPI last-gasp beacon (B.5) - combined dedicated future session.** **Group:** Early-impl / rework-eval candidates (index above; shares **PIO budget** with I²C-backend eval). Council round 3 (NASA/JPL + Cubesat, 2026-05-15) unanimously deferred the SPI-based last-gasp beacon (commit (c) of the rework was scoped for this and *not* implemented). User direction 2026-05-15: "merge with the future PIO beacon" - the two questions evaluate together rather than pre-committing to an interface (compile-time `ROCKETCHIP_LAST_GASP_BEACON` + `radio_init_confirmed` semantics) that would constrain the PIO design choice. Reasons for deferral, fully captured in plan B.5 + council-round-3 transcript at `C:\Users\pow-w\.claude\plans\parsed-soaring-popcorn-agent-a355e8caee0717e0b.md`:
  - **SPI peripheral state corruption** if fault occurred mid-byte/mid-burst - recovery is NOT bounded-cost (FIFO drain + CS deassert via GPIO function override + SX1276 hardware reset pulse + full cold re-init; each step has its own hang potential).
  - **`#ifdef`-scaffolding-rot pattern** per LL Entry 36 - code-shaped-but-never-exercised artifact creates false-confidence for future contributors.
  - **JPL precedent: always architect beacons as independent silicon** (Cassini LGA+USO, SMAP transponder). Cubesats that share the radio rely on modem-level autonomous beacon modes (e.g., FSK Beacon Mode); SX1276 LoRa lacks this in long-range mode.
  - **PIO beacon is the architecturally-correct answer** (Pico SDK has working PIO-SPI in 2-3 instructions per primary-source verification 2026-05-14; DIO0 wired to GPIO 6 on Adafruit Feather RP2350; DIO5 not currently routed but solder-jumperable on the RFM95W FeatherWing #3231).
  - **In-flight ARM-dead beacon-coverage gap is the trade** - accepted as a known gap for Core/Titan single-MCU pending this session.

  Scope of the combined session: (1) evaluate whether SPI-from-fault-handler is ever the right stop-gap given the failure-mode inventory; (2) design the PIO-driven beacon program (target: PIO0 or PIO1 - PIO2 already shared between watchdog SM0 + backup-timer SM1-3); (3) decide whether the design requires soldering the DIO5 jumper on the FeatherWing; (4) bench-verify on a known-faulted chip state if any stop-gap is in scope. Likely outputs a dedicated decision doc under `docs/decisions/`. User direction will determine sequencing relative to other open work.
- **RP2350B/Fruit Jam persistent bus-corruption hypothesis.** User hunch 2026-04-17: one boot during the Fruit Jam GPS debug had a transition not fully explained by the cable theory alone. Investigate whether RP2350B exhibits bus-corruption state that survives power cycles. Low priority - may be a dead end, keep passive.

- **I²C bus backend rework-eval (prefer PIO if advantages + budget) + Flipper prior art.** **Group:** Early-impl / rework-eval candidates (index above). **Owner lean (2026-08-05):** PIO master is **preferable** when advantages are real and **PIO budget** remains (coordinate with PIO beacon / watchdog allocation - do not eval in isolation). Current `i2c_bus` thin façade over DW_apb stays intentional either way; backend swap is the rework question, not deleting the project API. **Trigger / residual context:** Fruit Jam GPS cold-boot intermittency and related bus pain (`docs/plans/CYCLE_RESIDUALS_AFTER_R5.md`) - not a mandate to switch mid-walk. **Prior art:** Flipper One MCU firmware (public 2026-05-21) has a working RP2350 PIO I²C master at `lib/drivers/i2c_master_pio/pio_i2c.c` in https://github.com/flipperdevices/flipperone-mcu-firmware - claim/init + unclaim/deinit (LL-42 pattern), mid-cycle recover via drain + jump-to-wrap + IRQ clear (no program-memory touch), acquire/release pad-mux (LL-28) in `furi_hal_i2c_config.c`. Same SDK/toolchain family as us. **License check required before any code import.** Eval session outputs: advantages vs HW I²C, SM budget map, keep/reaffirm DW_apb vs plan PIO backend under `i2c_bus_*`.

## Deferred (near-term, post-Stage 15)

- **Battery ADC monitoring.** Hardware not wired. ADC pin + driver + telemetry field.
- **CCSDS SDLS command authentication.** Telecommand auth for Rocket profile.

## Far-future

Mission Profile OTA, F' evaluation, u-blox GPS, OTA drivers, GPS-free 3D reconstruction, FSK bitstream, MATLAB export - all tracked in `docs/PROJECT_STATUS.md` future features.

## Upcoming Stages

**Stage 15: Pre-Flight Polish** - AO responsibility audit (Stage 13 Core1 gap), Audio Output (I2S DAC, ~10-12 IVPs, fills Stage 14 audio backend stub), User Guide, Runtime Behavior Map update for AO architecture, defense-in-depth evaluation (Core1 stall checked in 3 places post-Stage-14 - evaluate justified vs. bloat).

**Stage 16: Field Tuning** - All VALIDATE parameters. Needs flight data.

**Stage 17: Field Testing** - IVP-135, 136, 137, 138. Airframe integration, ground test, flight test, exit gate. Needs hardware access and weather. IVP-134 (pre-flight checklist) already committed.
