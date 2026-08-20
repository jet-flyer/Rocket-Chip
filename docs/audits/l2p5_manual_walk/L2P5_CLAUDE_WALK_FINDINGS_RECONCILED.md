# L2-P5 Claude Agent Walk — Findings (CW), reconciled edition

> **Read this one for the three-way join.** Same 358 findings as the frozen
> `L2P5_CLAUDE_WALK_FINDINGS.md`, with an added Reconciliation section that collapses this
> pack's internal duplicates and settles its one self-contradiction, so it enters the join
> as **one vote / 336 distinct propositions**. No finding text differs between the two
> files — this edition only *adds* the reconciliation section and per-row markers.
> The original stays frozen as the as-walked record.

**This is not the owner walk.** It is an independent, blind agent re-walk of the same
186-file L2-P5 semantic pass, run to sit beside the owner findings (`WN-`) as the
redundancy layer the walk design called for. Finding IDs here are **`CW-`** and
correspond to nothing in the `WN-` set. No cross-reference has been performed.

| | |
|---|---|
| Base commit | `2989939` (worktree `RC-agent-walk`, branch `claude/l2p5-agent-walk`) |
| Files walked | **186 / 186** in 44 batches |
| Cross-cutting lanes | 5 / 5 |
| Findings raised | **358** |
| CONFIRMED | **270** |
| RESHAPED (narrowed, still real) | 66 |
| REFUTED (kept visible, not deleted) | 22 |

## Method

Each batch was walked by an agent that read the project field manual verbatim — the
spine plus exactly the lenses `L2P5_MANUAL_WALK_GUIDE.md`'s Class index assigns to that
subsystem — then read every assigned file whole, pairing each `.cpp` with its `.h`. A
second agent then tried to **refute** every finding, with instructions to default to
REFUTED when unsure and to kill anything a mechanical gate already covers.

**Blindness.** The tree was checked out at the last commit before the owner walk began,
so `L2P5_WALK_FINDINGS.md` does not exist in it and no `WN-` identifier appears anywhere
in the worktree. Agents were barred from reading the main checkout, the walk whiteboard
(which carried two pre-walk observations) and the superseded guide archive.

## Limitations — read before using these numbers

1. **Selectivity comes mostly from the walk stage, not the verifier.** Walk agents were
   told an over-claim is worse than a miss, so they self-censored; the refutation pass
   then cleared most of what survived. Do not read the confirm rate as two independent
   filters agreeing.
2. **Lens assignment bounds what was findable.** Each agent saw only the lenses the Class
   index assigns to its subsystem, so a defect visible only through an unassigned lens
   could not be found here, by construction.
3. **Claims against sources outside the tree are unverified.** Where a finding rests on a
   vendor datasheet or SDK source the agent could not open, the verdict says so. Those
   need a positive control before being cited as fact.
4. **Not repeatable.** A second run would produce a different set. "Every file was read"
   is provable from the coverage sections; "every defect was found" is not claimed.
5. **Findings are candidates, not dispositions.** Nothing here has been actioned, and no
   code was modified by any agent in this pass.

## Index

| Batch | Scope | Files | Findings | C | R | X |
|---|---|---|---|---|---|---|
| `B01` | public headers: cross-core contract + logging + config | 3 | 7 | 5 | 1 | 1 |
| `B02` | public headers: board HAL pack | 6 | 8 | 6 | 1 | 1 |
| `B03` | public headers: job/role pack | 5 | 3 | 2 | 1 | 0 |
| `B04` | public headers: notify + radio config/scheduler | 5 | 8 | 6 | 1 | 1 |
| `B05` | public headers: sensor snapshot/seqlock + telemetry surface | 5 | 14 | 10 | 4 | 0 |
| `B06` | public headers: remaining small contract headers | 9 | 9 | 9 | 0 | 0 |
| `B07` | math/ | 5 | 6 | 5 | 1 | 0 |
| `B08` | drivers: i2c_bus + spi_bus | 4 | 6 | 6 | 0 | 0 |
| `B09` | drivers: GPS (i2c + uart + shared iface) | 6 | 9 | 7 | 2 | 0 |
| `B10` | drivers: IMU + baro | 4 | 6 | 5 | 1 | 0 |
| `B11` | drivers: rfm95w radio | 2 | 7 | 5 | 1 | 1 |
| `B12` | drivers: mcu_temp + ws2812_status | 4 | 8 | 8 | 0 | 0 |
| `B13` | fusion: eskf core + ud_factor + state | 5 | 10 | 6 | 3 | 1 |
| `B14` | fusion: eskf_runner + brake + phase_qr | 4 | 7 | 7 | 0 | 0 |
| `B15` | fusion: confidence_gate + innovation_monitor | 4 | 4 | 3 | 1 | 0 |
| `B16` | fusion: mahony + generated tables/codegen (light+exempt) | 6 | 8 | 7 | 1 | 0 |
| `B17` | calibration: data + storage | 4 | 7 | 3 | 4 | 0 |
| `B18` | calibration: manager + hooks | 4 | 6 | 5 | 0 | 1 |
| `B19` | calibration: lm_solver (templates) | 2 | 6 | 5 | 0 | 1 |
| `B20` | flight_director: HSM core + state/actions types | 4 | 9 | 6 | 1 | 2 |
| `B21` | flight_director: command_handler + action_executor | 4 | 5 | 4 | 0 | 1 |
| `B22` | flight_director: go_nogo + guard_evaluator | 4 | 4 | 1 | 2 | 1 |
| `B23` | flight_director: guard_combinator + guard_functions | 4 | 5 | 3 | 2 | 0 |
| `B24` | flight_director: mission profile + generated profile data | 2 | 5 | 4 | 1 | 0 |
| `B25` | logging: rc_log sink + ring_buffer | 3 | 6 | 4 | 2 | 0 |
| `B26` | logging: flash_flush + psram_init | 4 | 7 | 4 | 1 | 2 |
| `B27` | logging: flight_table + log_decimator + data_convert | 6 | 6 | 5 | 1 | 0 |
| `B28` | logging: pcm_frame + radio_config_storage + CRC headers | 5 | 6 | 5 | 1 | 0 |
| `B29` | diag + notify backends | 5 | 6 | 2 | 3 | 1 |
| `B30` | telemetry + station | 4 | 8 | 5 | 2 | 1 |
| `B31` | safety: fault_protection + anomalous_boot | 4 | 8 | 7 | 1 | 0 |
| `B32` | safety: flight_in_progress + crash_record + health_monitor | 5 | 6 | 5 | 1 | 0 |
| `B33` | safety: fault injection + test_mode | 6 | 6 | 4 | 2 | 0 |
| `B34` | safety: core1_i2c_pause + pyro_edge_logger + rf_link_health | 5 | 8 | 6 | 2 | 0 |
| `B35` | safety: PIO backup timer + PIO watchdog | 4 | 7 | 6 | 0 | 1 |
| `B36` | core1: sensor loop (Core0<->Core1 boundary) | 2 | 4 | 2 | 2 | 0 |
| `B37` | AOs: flight_director + health_monitor | 4 | 7 | 5 | 1 | 1 |
| `B38` | AOs: rcos + logger | 4 | 8 | 7 | 1 | 0 |
| `B39` | AOs: radio + rf_manager | 4 | 6 | 5 | 1 | 0 |
| `B40` | AOs: telemetry + notify + led_engine | 6 | 9 | 7 | 2 | 0 |
| `B41` | top-level: main + shared_state | 2 | 8 | 6 | 2 | 0 |
| `B42` | cli: rc_os core | 2 | 9 | 8 | 0 | 1 |
| `B43` | cli: rc_os_commands | 2 | 10 | 8 | 2 | 0 |
| `B44` | cli: dashboard + debug | 4 | 9 | 7 | 1 | 1 |
| `X1` | cross-cutting lane | - | 8 | 4 | 1 | 3 |
| `X2` | cross-cutting lane | - | 8 | 6 | 2 | 0 |
| `X3` | cross-cutting lane | - | 10 | 8 | 2 | 0 |
| `X4` | cross-cutting lane | - | 12 | 10 | 2 | 0 |
| `X5` | cross-cutting lane | - | 9 | 6 | 3 | 0 |


---

## Reconciliation (2026-08-20) — one proposition, one Claude answer

This pack was produced on two axes (44 file batches + 5 cross-cutting lanes) that never
cross-referenced each other, so it shipped restating itself and, in one cluster, contradicting
itself. That is a defect of this pack, not a second opinion. Resolved here **without editing any
finding's text**: rows are marked, never deleted or renumbered.

**For a three-way join, count 336 distinct Claude propositions, not 358.**

### §1 — The UART / GPS-staleness cluster (the contradiction)

`CW-X1` concluded that the recovery branch in `core1_gps_staleness_check()`
(`src/core1/sensor_core1.cpp:253-278`) is dead code, and **refuted three of its own rows** on
that basis, while `CW-B09-03`, `CW-B36-01` and `CW-X5-02` treat the same branch as live.

**The branch is live on the flight vehicle.** Evidence, re-read for this reconciliation:

- `src/main.cpp:128-135` — `init_gps()` calls `bind_gps_uart_backend()` when
  `board::kUartGpsAvailable` is set and `gps_uart_init()` succeeds.
- `src/main.cpp:101-105` — that setter assigns `g_gpsTransport = GPS_TRANSPORT_UART`, the exact
  value the branch's guard at `sensor_core1.cpp:261` requires.
- `include/rocketchip/board_feather_rp2350.h:65` — `kUartGpsAvailable = true` on the **vehicle**
  board (also `board_pico2.h:71`; false only on `board_fruit_jam.h:87` and
  `board_tiny_2350_common.h:74`).
- The remaining guard, `g_lastValidGpsUs == 0`, only defers the branch until the first valid
  parse; it does not disable it.

So the dead-code premise holds *only* on the station/relay boards, and X1 generalised it to the
tree. **Claude's single answer: the branch is reachable on the vehicle, and the findings that
depend on its reachability stand.** The X1 rows built on the false premise are marked in place;
their withdrawal is a correction to this pack, not a new finding.

### §2 — Lane rows that restate a batch row (collapsed)

22 lane rows assert a proposition a batch row already made. Each is marked *duplicate of* its
batch row and should be counted **once**. The batch row is canonical because it is the
file-scoped original; the lane row is kept in place for its cross-file framing.

| Lane row | Duplicate of | Proposition |
|---|---|---|
| `CW-X1-01`, `CW-X5-02` | `CW-B09-03` | `gps_uart_reinit()` enables the UART IRQ on the wrong core's NVIC |
| `CW-X1-03` | `CW-B18-01` | calibration state shared across cores with no barrier |
| `CW-X1-04` | `CW-B09-04` | UART RX ring documented as needing no barrier |
| `CW-X1-05` | `CW-B01-01` | cross-core liveness flags are plain `bool` |
| `CW-X1-06` | `CW-B36-01` | Core 1 reads Core 0 ESKF state unsynchronized |
| `CW-X2-06` | `CW-B10-06` | barometric altitude model exists in two modules |
| `CW-X3-01` | `CW-B34-01` | "every reachable runtime `flash_safe_execute()` callsite" is false |
| `CW-X3-02`, `CW-X4-02` | `CW-B01-03` | `rc_log.h` LOCKED contract says drop-newest; sink drops oldest |
| `CW-X3-03`, `CW-X4-03` | `CW-B31-01` / `CW-B25-01` | `rc_log.h` forbids fault-handler logging; `Q_onError` logs |
| `CW-X3-06` | `CW-B06-03` | flash-layout region map omits a region |
| `CW-X3-09` | `CW-B05-03` | APID comment names a value the encoder never emits |
| `CW-X3-10` | `CW-B10-05` | DPS310 datasheet table transcription contradicts itself |
| `CW-X4-01` | `CW-B06-01` | signal catalog says QP events may be stack-allocated |
| `CW-X4-04` | `CW-B21-01` | LED pattern code space duplicated and diverged |
| `CW-X4-06` | `CW-B04-03` | radio-config whitelist header is not the `SET_RADIO_CONFIG` gate |
| `CW-X4-08` | `CW-B01-05` | `RC_ASSERT` documents a watchdog recovery this tree cannot perform |
| `CW-X4-11` | `CW-B02-02` | `board_pico2.h` omits a member `config.h` requires |
| `CW-X5-01` | `CW-B37-01` | PIO backup pyro timers disarmed only on the CLI path |
| `CW-X5-06` | `CW-B35-03` | disarm hands pyro pins to SIO without an output direction |

**Not collapsed** (share a file, assert different things, both stand): `CW-X1-02`, `CW-X2-03`,
`CW-X4-09`, `CW-X4-12`, `CW-X5-04`, `CW-X5-07`, `CW-X5-08`.

### §3 — What this pack is worth as a vote

One walk. The refute pass is this pack checking itself, and the lanes are a second read of the
same tree by the same method — neither is an independent walk, and neither should be counted as
corroboration in a three-way join.

---


## Tier 1 — Foundations (public headers, math, drivers)


### B01 -- public headers: cross-core contract + logging + config

#### Coverage

- `C:/Users/pow-w/Documents/RC-agent-walk/include/rocketchip/config.h` -- PARTIAL -- read whole; pin/I2C/timing maps delegate cleanly to `board::`/`job::` and the `if constexpr` DBG_* front-end is sound, but two documented mechanisms in the file (RC_ASSERT, the tier/feature-flag block) have no consumer anywhere in the tree.
- `C:/Users/pow-w/Documents/RC-agent-walk/include/rocketchip/rc_log.h` -- PARTIAL -- read whole as a Kind C contract surface; the promise set is unusually complete and the `void` signature honestly matches "caller cannot detect truncation", but the LOCKED contract's sink/overflow description disagrees with `src/log/rc_log.cpp` and with this header's own observability note.
- `C:/Users/pow-w/Documents/RC-agent-walk/include/rocketchip/shared_state.h` -- FAIL -- read whole as the canonical Kind A ownership map; the map is half-annotated, and for two flags the stated rules are contradicted by a real Core-1 write to a non-atomic bool that Core 0 reads on safety-visible paths.

#### Findings

### CW-B01-01 -- Two cross-core init flags are plain `bool` with two writers and no barrier
- Site: include/rocketchip/shared_state.h:35,37
- Lens: Concurrency & shared-data ownership -- JPL-C Rule 8 ("Data objects in shared memory should have a single owning task. Only the owner of a data object should be able to modify the object"), CCG CP.8 / CP.2; contract-surface helper Kind A
- Claim: `g_baroInitialized` and `g_gpsInitialized` are non-atomic `bool`s written by Core 0 during init and again by Core 1 at runtime, then read by Core 0 consumers, with no single owner and no synchronization named anywhere in the header.
- Why: Core 1 writes `g_baroInitialized = false` at src/core1/sensor_core1.cpp:221 ("Declare baro dead") and `g_gpsInitialized = false` at :276, after Core 0 wrote them at src/main.cpp:179 and :131/:147/:224. Core 0 then reads them on decision paths that matter -- health gating at src/safety/health_monitor.cpp:178, the preflight sensor verdict at src/cli/rc_os_commands.cpp:817 and :832-833, and notification state at src/active_objects/ao_notify.cpp:130. Because the objects are neither `std::atomic` nor `volatile`, nothing bounds when a Core-0 read must reload them; the compiler is free to keep the value in a register across a Core-0 loop, so the "baro is dead" transition can be invisible to the reader that is supposed to act on it, and there is no owner to say which core's value wins. The six flags four lines below (:67-72) are `std::atomic<bool>` for exactly this reason, so the file itself establishes the convention these two escape. The line comment "Core 1 reads/writes" is the only trace of the second writer, and the file banner asserts the opposite framing ("Core 0 owns initialization ... makes ownership clear").
- Confidence: high
- Direction: give these two the same `std::atomic<bool>` treatment as the neighbouring cross-core flags, or keep a single owner and let Core 1 publish "sensor dead" through the existing seqlock/atomic hand-off; either way correct the per-line ownership annotation to name both writers and the Core-0 readers.
- Verdict: CONFIRMED -- verified line-by-line: both flags are non-atomic `bool`, written by Core 1 at src/core1/sensor_core1.cpp:221 and :276 and by Core 0 at src/main.cpp:131/147/179/224, and read by Core 0 at src/safety/health_monitor.cpp:178, src/cli/rc_os_commands.cpp:817/832-833 and src/active_objects/ao_notify.cpp:130, with no owner, no synchronisation, no gate covering it, and docs/MULTICORE_RULES.md explicitly forbidding even `volatile` for cross-core sharing.

### CW-B01-02 -- Half the ownership map carries no owner annotation at all
- Site: include/rocketchip/shared_state.h:40-72
- Lens: Concurrency & shared-data ownership (CP.3, minimize and name the shared writable surface) + Comments & documentation quality (JSF AV 134, document a function's/interface's assumptions); helper Kind A question 4 -- ambiguity is itself the finding
- Claim: the init-attempted flags (:42-44), PSRAM state (:47-49), calibration-storage flag (:52), GPS transport (:55) and all six cross-core atomics (:67-72) have no owner/reader annotation, in a file whose stated purpose is to make ownership clear.
- Why: this header is the only place the dual-core rules are written down, so an absent rule is an invitation. Nothing here contradicts a future Core-1 write to `g_psramSelfTestPassed`, and for the pause handshake the atomic type shows that *a* barrier exists but not the protocol: which side sets `g_core1PauseI2C`, which side sets and which side clears `g_core1I2CPaused`, and what `g_core1LockoutReady` gates are recoverable only by reading src/safety/core1_i2c_pause.cpp. That is the map failing at the one job it has, and it is invisible to every gate because the declarations compile perfectly.
- Confidence: high
- Direction: extend the existing one-line owner/reader comment style to every `extern` in the file, and for the three pause/lockout atomics state the handshake direction (who sets, who clears) in one line each.
- Verdict: REFUTED -- the finding's premise fails: the pause/lockout handshake it calls "recoverable only by reading src/safety/core1_i2c_pause.cpp" is documented at length in that module's own header (src/safety/core1_i2c_pause.h:6-63 -- who sets, who acks, who clears, the 100 ms budget), and the project's dual-core rules are written down in docs/MULTICORE_RULES.md rather than only here, so what remains is a preference for more per-line annotation.

### CW-B01-03 -- The LOCKED contract says drop-newest; the implementation is drop-oldest
- Site: include/rocketchip/rc_log.h:24-27
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"), JSF AV 134
- Claim: the frozen CONTRACT block states that when the ring is full "the message is dropped on the floor", but `src/log/rc_log.cpp` keeps the new message and evicts the oldest bytes instead.
- Why: src/log/rc_log.cpp:430-433 states the implementation plainly ("if the ring is full, the OLDEST bytes are evicted to make room (drop-oldest semantics)") and :510-515 is the code that advances the tail past the evicted bytes. The two policies fail in opposite directions, so a caller who reads the contract draws exactly the wrong conclusion: it believes already-emitted output is safely queued and only the newest message is at risk, when in fact a burst destroys the earliest output. That is not hypothetical -- it is the failure the implementation comment records (a ~1.3 KB `diag_stats` dump evicting the T=0 preconditions block). This header is also self-contradictory: :100-101 correctly says bytes are "evicted by drop-oldest". Same bullet, second error: the ring is not "drained by tud_task"; it is drained by `rc_log_drain_to_cdc()`, declared at :95 of this file.
- Confidence: high
- Direction: rewrite the sink bullet to state drop-oldest eviction, point at `rc_log_dropped_bytes()` as the way to detect it, and name `rc_log_drain_to_cdc()` as the drainer rather than `tud_task`.
- Verdict: CONFIRMED -- citations exact: the LOCKED bullet at rc_log.h:24-27 says "dropped on the floor" and names `tud_task` as the drainer, while src/log/rc_log.cpp:430-433 states drop-oldest, the eviction code at :510-515 advances the tail, and this same header contradicts itself at :100-101 and declares the real drainer `rc_log_drain_to_cdc()` at :95.

### CW-B01-04 -- Ring-health guidance names a constant that does not exist
- Site: include/rocketchip/rc_log.h:102-104
- Lens: Comments & documentation quality -- CCG NL.3 / JSF AV 134 (a stated limit must be checkable); spine block B, non-self-contained symbol
- Claim: the `high_water` note tells the reader to judge the value by whether it "approaches `kRcLogRingBytes`", but `kRcLogRingBytes` is defined nowhere in the repository and the header never states the ring size.
- Why: a repo-wide search for the identifier returns this comment and nothing else; the real capacity is `kRingBytes = 8192U` at src/log/rc_log.cpp:480, private to an anonymous namespace in the `.cpp`. So the only check the contract prescribes for its own observability counter cannot be performed by any caller, CLI consumer, or soak script -- they must either guess the threshold or read the implementation, which defeats the point of a locked public contract. The size has already moved once (1024 to 8192, per the council note at src/log/rc_log.cpp:459-467), so a guessed constant would silently drift.
- Confidence: high
- Direction: export the ring capacity as a named `constexpr` alongside `kRcLogBufferBytes`, or delete the symbol name and describe the threshold in terms the header actually provides.
- Verdict: CONFIRMED -- a repo-wide grep returns `kRcLogRingBytes` only in this comment; the real capacity is `kRingBytes = 8192U` at src/log/rc_log.cpp:480, private to an anonymous namespace, so the threshold the public contract prescribes for its own counter is not available to any consumer.

### CW-B01-05 -- RC_ASSERT has no call sites, and its documented failure behavior cannot occur
- Site: include/rocketchip/config.h:20-46
- Lens: Comments & documentation quality -- CERT MSC12-C (doc-comment describing code that never executes) + CCG NL.2 (comment/code disagreement)
- Claim: the 27-line RC_ASSERT contract block documents a macro that is invoked nowhere in the tree, and the behavior it promises on failure -- spin "until watchdog resets the device (reboot-cause flag preserved)" -- is not achievable in this build.
- Why: two separate defects in one block. (a) A repo-wide search for `RC_ASSERT` finds only this definition plus audit/CHANGELOG prose -- no `src/` call site -- so the usage examples, the "side-effect safety" warning, and the debug/release split all document a mechanism the system never exercises, while the section banner ("Runtime Assertions (P10-5 / LOC-3.1 / JSF AV Env 15)") reads as a satisfied requirement. (b) No SDK watchdog API is called anywhere in `src/` (no `watchdog_enable`, `watchdog_caused_reboot`, or `watchdog_reboot`), and the surviving watchdog is the PIO heartbeat, which per src/safety/pio_watchdog.h:34-36 only raises PIO IRQ flag 0 for ARM code to poll -- it has no reset authority. So on the one path this macro exists for, a DEBUG build would hang the core in `while (true) { __asm volatile("nop"); }` indefinitely, with no reset and no reboot-cause flag for the post-boot health monitor to find.
- Confidence: high on both halves
- Direction: decide whether RC_ASSERT is meant to be the project's assertion surface; if yes, correct the failure-behavior text to what the current watchdog architecture actually does, and if no, delete the macro with its block rather than leaving a documented mechanism with no terrain behind it.
- Verdict: CONFIRMED -- both halves verified: `RC_ASSERT` appears nowhere outside config.h, and no SDK watchdog API (`watchdog_enable` / `watchdog_reboot` / `watchdog_caused_reboot`) is called anywhere in src/, so the documented "spin until watchdog resets the device" outcome cannot occur -- the surviving PIO watchdog only raises an IRQ flag for ARM to poll (src/safety/pio_watchdog.h:34-36), and the spinning core would never poll it.

### CW-B01-06 -- Tier / feature-flag block is a configuration map with no consumers
- Site: include/rocketchip/config.h:61-71
- Lens: Comments & documentation quality -- CERT MSC12-C; contract-surface helper Kind E (single-source configuration map)
- Claim: `ROCKETCHIP_TIER_MAIN` (with `_CORE`/`_TITAN` commented out) and the five `ROCKETCHIP_FEATURE_*` macros are referenced nowhere in the repository outside this file, so the block presents a build-configuration mechanism that configures nothing.
- Why: a repo-wide search for all eight identifiers returns only these definition lines. The comment "Tier selection (only one should be enabled)" and the per-feature toggles read as live switches on the project's primary configuration header, so an engineer who sets `ROCKETCHIP_FEATURE_RADIO` to 0 to bring up a board without a radio gets a firmware image that still initializes the radio, with no compile error to correct the misunderstanding. The real role/board selection happens immediately below at :73-78 through `job.h`/`board.h` and the CMake `ROCKETCHIP_JOB_STATION` define, which makes the dead block actively misleading about where the single source of truth lives. Note standards/ACCEPTED_STANDARDS_DEVIATIONS.md PP-1 still lists "feature flags (`ROCKETCHIP_TIER_*`)" among the retained `#define`s, so the stale map is cited outside the header too.
- Confidence: high
- Direction: delete the block and let `job.h`/`board.h` be the only configuration map, or, if a tier concept is still wanted, wire it to a consumer and say in the comment what reads it.
- Verdict: CONFIRMED -- grep over src/, include/, test/ and scripts/ returns all eight identifiers only at their definitions in config.h:62-71, and no gate covers this: scripts/audit/find_dead_code.py explicitly lists dead-global detection as out of scope, and clang-tidy's deadcode checks do not see unused header constants.

### CW-B01-07 -- Three function pointers on a public contract surface with the P10-9 deviation register marked fully resolved
- Site: include/rocketchip/shared_state.h:56-58
- Lens: Concurrency & shared-data ownership (the cross-core hand-off half) with the rule text from Power of Ten Rule 9 ("Limit pointer use to a single dereference, and do not use function pointers"); the pointer-rule half properly belongs to the pointers/casts class, not this batch
- Claim: `g_gpsFnUpdate`, `g_gpsFnGetData` and `g_gpsFnHasFix` are global function pointers -- a construct the project's own precedence ruling bans outright (standards/CODING_STANDARDS.md:56, "most-restrictive -> P10-9 governs") -- and no active deviation row covers them: the register's P10 Rule 9 section at standards/ACCEPTED_STANDARDS_DEVIATIONS.md:56 reads "All entries resolved as of 2026-05-13", while CODING_STANDARDS.md:56 still names FP-1 as the covering deviation even though FP-1 sits in the Resolved (historical) section.
- Why: verified at the sites -- declared here at :56-58, defined at src/shared_state.cpp:31-33, bound on Core 0 at src/main.cpp:102-104 and :109-111, called from Core 1's sensor loop. The site is not special, and that is what bounds the claim: src/cli/rc_os.h:112/149/158, src/flight_director/flight_director.h:85-89, src/flight_director/action_executor.h:115/118, src/fusion/eskf_runner.h:61, src/logging/flash_flush.h:76/105 and src/safety/test_mode.h:90 carry the same construct with the same absent-deviation status. So the surviving statement is documentary: the tree carries an unlogged P10-9 surface of which this header holds three, and two standards docs point at a retired row for cover. The cross-core half does NOT survive verification -- Core 1 is launched at src/main.cpp:284 but idles until `g_startSensorPhase` is published with a release store at src/main.cpp:346-347, after every bind site has run (init_gps_early at :219-227, init_sensors at :299), so an unsynchronised rebind under a concurrent Core-1 call is not reachable on any present path.
- Confidence: medium -- the fact is verified, but the rule half is mechanically greppable and may already be tracked by the pointers/casts class or the walk's own instrumentation, so treat this as a surfaced fact rather than a new defect claim
- Direction: either log an active P10-9 deviation row for the GPS backend dispatch with the "set once before Core 1 launch" rationale, or retire the pointers the way FP-1 was retired (a small backend struct or compile-time selection), and in either case name the publication point that makes the Core-0-to-Core-1 hand-off safe.
- Verdict: RESHAPED -- the cited fact is exact (ACCEPTED_STANDARDS_DEVIATIONS.md:56 reads "All entries resolved as of 2026-05-13") but the cross-core hazard is unreachable and the site is not special, so the claim is narrowed to the unlogged P10-9 surface across the tree plus the stale FP-1 pointer at CODING_STANDARDS.md:56.

### B02 -- public headers: board HAL pack

Walked as contract surfaces (helper Kind E — layout / identity / configuration map, with a Kind C
API-contract element for the three inline `board_*` functions). Surface spine applied per helper §9
(name test, one altitude, duplication, distrust confident comments) plus the Class-index lenses for
`include/rocketchip/` public headers: comments & documentation quality, declaration scope & object
lifetime, class & interface design. Claim-vs-truth checks were run against the real consumers
(`include/rocketchip/config.h`, `src/main.cpp`, `src/drivers/ws2812_status.*`,
`src/drivers/gps_uart.cpp`, `src/cli/rc_os_commands.cpp`) rather than deferred.

#### Coverage

include/rocketchip/board.h -- PARTIAL -- Selector read whole; include-chain rationale is a real intent comment, but the `#else` arm silently maps an unrecognised board onto the Feather pin map (CW-B02-06).
include/rocketchip/board_feather_rp2350.h -- PARTIAL -- Full map read and cross-checked against config.h/main.cpp; pin constants all reach a consumer, but the capability block, `kNeoPixelGpioBase` and `kLedActiveHigh` do not (CW-B02-01, CW-B02-04, CW-B02-07).
include/rocketchip/board_fruit_jam.h -- PARTIAL -- Full map read; [M1]/[M2]/[M3]/[N1] hazard notes are good JSF 134 preamble work, but `board_release_peripheral_reset()` hides its own preconditions (CW-B02-05) and the pack-wide dead-constant issues apply here too.
include/rocketchip/board_pico2.h -- FAIL -- Read whole; the map omits `kPsramCsPin`, which `config.h:90` consumes unconditionally, so this board cannot compile the moment its bring-up gate is lifted (CW-B02-02).
include/rocketchip/board_tiny_2350_common.h -- FAIL -- Read whole; `kPsramCsPin` is assigned the same GPIO as `kI2cSclPin` on a live consumer path (CW-B02-03), and the preamble points at a file and a contract member that do not exist (CW-B02-08).
include/rocketchip/board_tiny_2350_plus.h -- PASS -- Two-constant variant override, correctly placed and guarded; no defect of its own (note: its `kPsramAvailable = true` is what makes CW-B02-03's collision reachable).

#### Findings

### CW-B02-01 -- Capability flags are declared as a consumed contract but nothing reads them
- Site: include/rocketchip/board_feather_rp2350.h:69-77 (identical block at board_fruit_jam.h:91-99, board_pico2.h:75-79, board_tiny_2350_common.h:78-84)
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") + CERT MSC12-C (documentation must describe code that actually runs); spine block B (confabulation, NIST AI 600-1 — a confident rationale the body does not implement).
- Claim: The block states in the present tense that these flags are "consumed by role-agnostic shared code (health monitor, CLI, telemetry)" and make future board ports "drop-in without touching shared code", but `kPsramAvailable`, `kDvmAvailable`, `kSdCardAvailable` and `kI2cStemmaAvailable` have no reader anywhere in the tree — only these four definitions.
- Why: A reader of the board contract concludes that declaring `kPsramAvailable = false` is sufficient to keep PSRAM init off a board that has none. It is not: `src/main.cpp:276` calls `rc::psram_init(rocketchip::pins::kPsramCs)` unconditionally, branching on nothing. On the Pico 2 (`kPsramAvailable = false`) and on either Tiny variant, the bring-up IVP that lifts the `#error` gate gets an unconditional PSRAM probe against a CS pin that is wrong or absent, and the flag that was supposed to prevent it is inert. The named consumers (health monitor, CLI, telemetry) branch on board identity or on nothing at all.
- Confidence: high
- Direction: Either wire the flags into the paths that need them (make `psram_init` and the health/CLI reporting sites branch on `board::kPsramAvailable`), or downgrade the comment to what is true today — flags reserved for future use, not yet consumed. Do not leave the present-tense claim standing over an unread constant.
- Verdict: CONFIRMED — grep over src/, include/, test/ and scripts/ returns the four flags only at their definitions in the four board maps; the present-tense "consumed by role-agnostic shared code" claim is false, and src/main.cpp:276 does call `rc::psram_init(rocketchip::pins::kPsramCs)` with no capability branch.

### CW-B02-02 -- board_pico2.h omits kPsramCsPin, which config.h consumes unconditionally
- Site: include/rocketchip/board_pico2.h:75-79 (constant absent from the whole file; consumer at include/rocketchip/config.h:90)
- Lens: Class & interface design -- CCG C.3 (the public section must present a coherent, *complete* contract), supporting JSF AV 87/88 (interface-based design). The board pack is an implicit interface with no mechanical completeness check; helper Kind E q1/q3.
- Claim: Every other board map defines `kPsramCsPin` (Feather 8, Fruit Jam 47, Tiny 21) and `config.h:90` does `constexpr uint8_t kPsramCs = board::kPsramCsPin;` at namespace scope with no `#if`, so the Pico 2 map is missing a required member of the board contract.
- Why: The moment the bring-up IVP defines `PICO2_BRINGUP_OK` — the one action the header tells a future engineer to take — every translation unit that includes `config.h` fails to compile on `board::kPsramCsPin`. The failure is latent today only because the `#error` at :20-22 hides it, so the "scaffolding is ready, just verify the pins" contract this file advertises is not true. It also directly contradicts the drop-in claim in CW-B02-01: this port cannot avoid touching shared code.
- Confidence: high
- Direction: Define `kPsramCsPin` in board_pico2.h with an explicit "no PSRAM — value unused, guarded by kPsramAvailable" note, or make `config.h`'s alias conditional on `board::kPsramAvailable`. Better still, give the pack one static_assert-able required-member list so a missing constant fails loudly at the map rather than deep in a consumer.
- Verdict: CONFIRMED — `kPsramCsPin` is defined in feather (:61), fruit jam (:82) and tiny common (:88) but is absent from board_pico2.h, while include/rocketchip/config.h:90 aliases `board::kPsramCsPin` at namespace scope with no `#if`; the only thing hiding it is the `#error` at board_pico2.h:20-22, which the file itself tells a future engineer to remove.

### CW-B02-03 -- Tiny 2350 map assigns GPIO 21 to both I2C SCL and PSRAM CS
- Site: include/rocketchip/board_tiny_2350_common.h:88 (collides with :33)
- Lens: The spine, block C -- HW-register / peripheral-map fidelity (verify pin assignments against the datasheet; "functionally-correct-but-safety-blind hardware code"); helper Kind E q3 (two constants in the map silently disagree).
- Claim: `kI2cSclPin = 21` (:33) and `kPsramCsPin = 21` (:88) name the same GPIO on the same board, and both are live: `config.h:93-94` and `config.h:90` alias them, and `main.cpp:276` runs `psram_init(pins::kPsramCs)` unconditionally at boot.
- Why: On either Tiny variant after `TINY_2350_BRINGUP_OK` is defined, `psram_init(21)` reconfigures the pin the I2C bus needs for SCL into a QMI chip-select. The Plus variant makes this reachable and expected (`kPsramAvailable = true`, board_tiny_2350_plus.h:30). The symptom would be exactly the class of I2C failure this project has burned days on before — bus dead from boot with every sensor reporting absent — and the cause is visible in the map without any hardware. The `TODO(Tiny_2350+): confirm PSRAM CS pin` note at :87 flags the value as unverified but does not catch that it is unverified *and* already taken.
- Confidence: high
- Direction: Resolve the collision in the map before bring-up rather than on the bench — pick a free GPIO for PSRAM CS from the Pimoroni schematic, and add a compile-time uniqueness check over the board pin set so a future map cannot double-assign a pin silently.
- Verdict: CONFIRMED — board_tiny_2350_common.h:33 (`kI2cSclPin = 21`) and :88 (`kPsramCsPin = 21`) are the same GPIO, both aliased live at config.h:90 and :94, and board_tiny_2350_plus.h:30 sets `kPsramAvailable = true`; the `TODO(Tiny_2350+)` at :87 flags the value as unverified but not as already taken.

### CW-B02-04 -- kNeoPixelGpioBase states a hardware requirement that no code consumes
- Site: include/rocketchip/board_fruit_jam.h:51 (also feather:40, pico2:53, tiny common:54)
- Lens: Comments & documentation quality -- CERT MSC12-C (remove documentation that maps to code that never runs) + CCG NL.2 (comment/code disagreement).
- Claim: `kNeoPixelGpioBase = 16;  // Required for GPIO 32+` reads as a load-bearing hardware parameter, but it has no reader in the tree: `ws2812_status_init(PIO pio, uint pin, uint8_t num_leds)` (src/drivers/ws2812_status.h:59) takes no gpiobase, and `main.cpp:240-241` passes only pin and count.
- Why: The requirement it names is real for RP2350B GPIO 32+, but it is satisfied by a different mechanism — `pio_claim_free_sm_and_add_program_for_gpio_range(...)` at src/drivers/ws2812_status.cpp:249-250 — so the map documents a contract the consumer never honours. A reader porting a new board copies the constant believing it is wired up; conversely, if the driver ever moves off the auto-range helper to a manual `pio_add_program`, nothing points back at this value and the Fruit Jam's five-LED chain silently fails to drive. Four copies of an unread constant is also four places to keep in sync for nothing.
- Confidence: high
- Direction: Either pass `board::kNeoPixelGpioBase` into WS2812 init so the constant is the source of truth, or delete it from all four maps and put the one-line reason ("gpiobase handled by pio_claim_free_sm_and_add_program_for_gpio_range") where the driver claims the SM.
- 
- Verdict: CONFIRMED — grep returns `kNeoPixelGpioBase` only at its four definitions; `ws2812_status_init(PIO, uint pin, uint8_t num_leds)` (src/drivers/ws2812_status.h:59) has no gpiobase parameter, main.cpp:240-241 passes pio/pin/count, and the requirement is met instead by `pio_claim_free_sm_and_add_program_for_gpio_range` at src/drivers/ws2812_status.cpp:249-250.

### CW-B02-05 -- board_release_peripheral_reset() hides a blocking delay and a momentary reset assertion
- Site: include/rocketchip/board_fruit_jam.h:63-77 (contrast the "call unconditionally" note at board_feather_rp2350.h:51-58)
- Lens: Comments & documentation quality -- JSF AV 134 ("Assumptions (limitations) made by functions should be documented in the function's preamble"); spine block C (peripheral init sequence / lifecycle, blocking work).
- Claim: The preamble documents only that the function releases the shared active-low RESET, and says nothing about its two call-time constraints: it blocks for 50 ms, and (because `gpio_init` clears the output value before `gpio_set_dir(..., GPIO_OUT)` at :73-74) it briefly drives the shared line LOW, asserting reset on both the ESP32-C6 and the TLV320DAC3100 before releasing it at :75.
- Why: The sibling boards' version of this API is documented as callable "unconditionally" by role-agnostic init code (feather:53-55), which invites a call from anywhere; on the Fruit Jam that call resets two peripherals and stalls the caller for 50 ms. Called once from `main.cpp:236` during early init that is harmless, but nothing in the contract says early-init-only, so a later caller (a CLI recovery command, a re-init path) would knock the WiFi coprocessor and audio DAC over with no diagnostic. The 50 ms itself carries no source, which the project's own no-arbitrary-numbers rule and NL.3 (point to the spec) both want: BOARD_COMPARISON.md is cited for the reset line at :67 but not for the timing.
- Confidence: medium — the momentary-LOW mechanism depends on `gpio_init` clearing the output latch, which I could not verify against the SDK inside the working root; the documentation gap stands regardless of the glitch width.
- Direction: State the preconditions in the preamble — early-init only, blocks ~50 ms, momentarily asserts RESET on the DAC and ESP32-C6 — and cite the datasheet section behind the 50 ms. If the momentary assert is unwanted, set the output level before switching direction.
- Verdict: REFUTED — nothing is hidden: the function is an `inline` in this header with `sleep_ms(50)` and its rationale comment eight lines below the preamble (:76), the sole caller is early init (src/main.cpp:236), a "later caller" is hypothetical, and the momentary-LOW half is inconsequential because RP2350 pads leave reset as inputs with pull-down, so the active-low shared RESET is already asserted before this call releases it.

### CW-B02-06 -- Unrecognised PICO_BOARD silently compiles the Feather pin map
- Site: include/rocketchip/board.h:39-42 (recipe it undercuts at :14-18)
- Lens: The spine, block C -- functionally-correct-but-safety-blind hardware code (a build that compiles clean while leaving the hardware map wrong); comments/JSF AV 134 on the contract this file states for itself.
- Claim: The `#else` arm maps any board the chain does not recognise onto the Feather RP2350 pin map with no diagnostic, while the two unverified boards in the same pack are protected by hard `#error` gates (board_pico2.h:20-22, board_tiny_2350_plus.h:23-25) on the principle that an unverified pin map must not build.
- Why: CMakeLists.txt:212 documents that `PICO_BOARD` can be overridden explicitly, so an override to any other SDK board — or a new board added per this file's own recipe where step 2 (add the `#elif`) is forgotten, or where the SDK's detection macro differs from the guess — produces a firmware that drives radio CS, PSRAM CS, LED and I2C on Feather pins. That is the same hazard the `#error` gates exist to prevent, reached by a path that emits nothing. The default itself may well be deliberate; the missing signal for the *unrecognised* case is the defect.
- Confidence: medium
- Direction: Keep the default if it is wanted, but make it explicit rather than implicit — e.g. require an opt-in define for the fallback, or emit a `#warning`/`#error` naming the unrecognised board so the silent case cannot ship.
- Verdict: CONFIRMED — board.h:39-42 does map any unrecognised board onto the Feather map with no diagnostic, CMakeLists.txt:212 does document `-DPICO_BOARD=...` override as supported, and the same pack `#error`-gates its two unverified maps (board_pico2.h:20-22, board_tiny_2350_plus.h:23-25) on the opposite principle; the defect claimed is the missing signal, not the default itself.

### CW-B02-07 -- LED polarity is stated twice: a constant nothing drives, and a hardcoded body
- Site: include/rocketchip/board_feather_rp2350.h:44-49 (same shape at board_fruit_jam.h:56-61, board_pico2.h:57-61, board_tiny_2350_common.h:59-63)
- Lens: The spine, block A -- CCG ES.3 (don't repeat yourself; the same knowledge expressed in two places), Fowler "Duplicated Code".
- Claim: Each board states its LED polarity twice — once as `kLedActiveHigh` and again as a hardcoded expression inside `board_led_set` — with no compile-time link between the two; all four maps currently agree, and the constant's only reader is a diagnostic print at src/cli/rc_os_commands.cpp:745.
- Why: There is no present mismatch: feather (`true` / `gpio_put(kLedPin, on)`), fruit jam (`false` / `!on`), pico2 (`true` / `on`) and tiny common (`true` / `on`) are each internally consistent, so this is a duplication and divergence-risk finding, not a live bug. The compiler cannot keep the two statements in sync, so a future map — most plausibly one of the three unverified scaffolding maps, where board_tiny_2350_common.h:57 still carries `verify active-high/low` as an open TODO — can set the flag one way and write the body the other way and still build clean, after which the CLI diagnostic would report a polarity the hardware does not have.
- Confidence: medium
- Direction: Derive the body from the constant — one shared inline of the form `gpio_put(kLedPin, kLedActiveHigh ? on : !on)` — so the polarity fact exists once per board and the diagnostic cannot disagree with the drive.
- Verdict: RESHAPED — the duplication is real and every site is exact, but all four maps currently agree (feather true/`on`, fruit jam false/`!on`, pico2 true/`on`, tiny true/`on`), so this is a divergence risk on the unverified scaffolding maps rather than the "live risk" the original asserted.

### CW-B02-08 -- Tiny common preamble points at a header and a contract member that do not exist
- Site: include/rocketchip/board_tiny_2350_common.h:7-9
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation describing something the tree cannot produce) + CCG NL.2 (comment/code disagreement).
- Claim: The preamble states that variant overrides "live in board_tiny_2350.h and board_tiny_2350_plus.h" and names three of them — `kPsramAvailable`, `kBoardName`, `flash size` — but `board_tiny_2350.h` does not exist in the tree, and no board in the pack defines any flash-size constant.
- Why: Two headers in the same pack disagree about the same fact: board.h:31-33 records the base variant as deferred, while this file describes it as present. An engineer taking up the base-variant port follows the preamble to a nonexistent file and expects a flash-size member of the board contract that no board implements, which is exactly the kind of false map a contract surface exists to prevent. The same sentence is also the only place the base variant's `kPsramAvailable = false` is promised, and nothing enforces it.
- Confidence: high
- Direction: Reword to the deferred state that board.h already records ("the base variant header is not yet in tree; when added it must define ..."), and drop `flash size` or add it as a real constant to every map.
- Verdict: CONFIRMED — `board_tiny_2350.h` does not exist in include/rocketchip/, no board header defines any flash-size constant (only prose in the file banners), and board.h:31-33 records the base variant as deferred, so the preamble at :7-9 is a false map. (One minor over-claim left unreshaped: :79-81 also promises the base variant's `kPsramAvailable = false`.)

### B03 -- public headers: job/role pack

Batch scope: the compile-time device-role pack. All five files are contract surfaces
(helper Kind E "layout / identity / configuration map", with Kind D vocabulary in job.h),
so they were walked with the surface spine of the helper section 9 plus the comments,
scope/lifetime and class-design lenses. There are no function bodies in this batch --
every declaration is an `inline constexpr` or an enum, so the spine block A questions
were run on the file as a unit (name test, one altitude, duplication, confident-comment
distrust) rather than per function. Block B applies as claim-versus-truth: every
"Consumers:" and per-role behavioral claim in these headers was checked against the code
it names. Block C (embedded) has no volatile, MMIO, init-sequence or blocking surface here.

#### Coverage

include/rocketchip/job.h -- PARTIAL -- Read whole; enum, role table and CMake-usage
preamble are accurate and the `#if`/`#elif` selector matches the documented default
(vehicle), but the file never states the contract a role header must satisfy (CW-B03-03).

include/rocketchip/job_capabilities.h -- PARTIAL -- Read whole; the file's stated purpose
and its "do not add per-peripheral flags here, those live in board_*.h" pointer both check
out (board.h and board_feather_rp2350.h / board_fruit_jam.h exist and carry no capability
bools yet, so the "IVP-143 will generalize" forward claim is not stale). Two of the three
predicates verified true against their named consumers in src/safety/health_monitor.cpp;
the third has no consumer at all (CW-B03-01).

include/rocketchip/job_relay.h -- PARTIAL -- Read whole; kRole and kRadioModeRx are
correct and live, the Council 3 [C3-R2] link-layer-only claim is consistent with the relay
exclusions at src/main.cpp:500/506; kDefaultMavlinkOutput is dead (CW-B03-02) and the
include-order contract is unstated (CW-B03-03).

include/rocketchip/job_station.h -- PARTIAL -- Read whole; kRadioModeRx = true is live and
widely consumed; the default-output comment documents a policy that the constant it sits
on does not actually establish (CW-B03-02), and the include-order contract is unstated
(CW-B03-03).

include/rocketchip/job_vehicle.h -- PARTIAL -- Read whole; kRole and kRadioModeRx = false
are correct and live; same two shared findings as the other role headers (CW-B03-02,
CW-B03-03).

Checked and found sound (recorded so the coverage is honest, not just the defect list):
job_capabilities.h:28-35 claims station/relay never advance core1_loop_count because Core 1
"idles on g_startSensorPhase forever" -- src/main.cpp:284 launches Core 1 unconditionally
and src/main.cpp:345-361 only sets g_startSensorPhase on the vehicle role, so the claim is
true. Its consumer claim is true at src/safety/health_monitor.cpp:361. The kRoleRunsLogger
zero-init rationale at job_capabilities.h:37-45 is true at health_monitor.cpp:396-403. The
three predicates being textually identical expressions is not a duplication finding -- they
are distinct concepts that coincide today, and separating them is what stops a future role
change from silently moving three unrelated behaviors at once.

#### Findings

### CW-B03-01 -- kRoleHasFullGoNogo is documented as a capability gate but is never consumed
- Site: include/rocketchip/job_capabilities.h:47-54
- Lens: Comments & documentation quality -- CERT MSC12-C (doc-comment describing code that
  is never executed) with CCG NL.2 ("if the comment and the code disagree, both are likely
  to be wrong"); contract-surface helper Kind E claim-versus-truth.
- Claim: The predicate carries a six-line per-role rationale describing how station and
  relay opt out of the Go/No-Go matrix, but nothing in the tree reads it, so the masking it
  documents does not happen.
- Why: A repo-wide search for the identifier returns only its definition at line 54 (plus
  one CHANGELOG mention). Its two siblings each carry an explicit "Consumers:" line that I
  verified -- kRoleSamplesCore1 at src/safety/health_monitor.cpp:361 and :407,
  kRoleRunsLogger at :396 -- and this one carries no such line and no call site. The path
  it describes, rc::go_nogo_evaluate() invoked from src/cli/rc_os_commands.cpp:1481, runs
  ungated by role. The substitute it asserts for station -- "a single condensed bit (radio
  + watchdog + flash_if_present + mcu_not_critical) that plugs into the vehicle's Go/No-Go
  as one item over the radio" -- is not implemented either (AGENT_WHITEBOARD carries the
  station-to-vehicle health channel as deferred to the CCSDS batch). The concrete cost: a
  maintainer adding a vehicle-only check to go_nogo_checks.cpp reads this header, believes
  the role masking is already in place, and ships a check that a station build evaluates
  against absent hardware.
- Confidence: high
- Direction: Either consume the predicate where the matrix is assembled or evaluated and
  give it the "Consumers:" line the file's own convention uses, or remove the constant and
  its rationale and re-introduce both with the station-readiness work. If it is
  deliberately staged ahead of its consumer, the comment must say that rather than
  describing the masking in the present tense.
- Verdict: CONFIRMED -- a repo-wide grep returns `kRoleHasFullGoNogo` only at job_capabilities.h:54 plus one CHANGELOG line, while both siblings carry a `Consumers:` line that checks out (src/safety/health_monitor.cpp:361 and :396), and `rc::go_nogo_evaluate()` at src/cli/rc_os_commands.cpp:1481 does run ungated by role.

### CW-B03-02 -- kDefaultMavlinkOutput is a dead knob: identical in all three roles, read by nothing
- Site: include/rocketchip/job_station.h:24-27 (same shape at include/rocketchip/job_vehicle.h:23-24
  and include/rocketchip/job_relay.h:22-23; re-exported at include/rocketchip/config.h:78)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation attached to a
  declaration nothing reaches) and CCG NL.2; contract-surface helper Kind E ("is this the
  single place this knowledge lives?").
- Claim: The constant is false in all three role headers and has no reader anywhere, while
  the output-default policy it documents is actually decided elsewhere from a different
  constant.
- Why: A repo-wide search returns only the three definitions and the
  `using job::kDefaultMavlinkOutput;` re-export at config.h:78 -- no consumer. The live
  default is src/active_objects/ao_rcos.cpp:63, which derives the station-versus-vehicle
  output mode from kRadioModeRx. So setting job_station.h:27 to true -- the obvious move
  for someone acting on the MAVLink-for-QGC comment two lines above it -- changes no
  behavior at all, and nothing local tells the editor that. Three different per-role
  rationales attached to one identical value make it read as a live per-role
  differentiator; it is the only constant in the pack that differentiates nothing.
- Confidence: high
- Direction: Delete the constant from the three role headers and the config.h re-export,
  and move the station default-output rationale to the site that actually decides it
  (ao_rcos.cpp). If the knob is wanted, make that site read it instead of kRadioModeRx.
- Verdict: CONFIRMED -- grep returns the constant only at its three definitions and the `using` re-export at config.h:78, and the live default really is derived from `kRadioModeRx` at src/active_objects/ao_rcos.cpp:63, so three per-role rationales sit on a knob that differentiates nothing.

### CW-B03-03 -- The role-header contract is unwritten: neither the include-order requirement nor the required constant set is stated
- Site: include/rocketchip/job.h:41-48 and include/rocketchip/job_vehicle.h:16-24 (same
  shape at include/rocketchip/job_station.h:17-27 and include/rocketchip/job_relay.h:15-23)
- Lens: Comments & documentation quality -- JSF AV 134 ("Assumptions (limitations) made by
  functions should be documented in the function's preamble", applied to the file preamble
  of a declaration-only contract surface); contract-surface helper Kind E, "does the file
  claim to be the only place this knowledge lives?".
- Claim: Every role header uses `job::DeviceRole` without declaring or including it, so each is valid only in the
  position job.h:41-48 places it, and neither job.h's preamble nor any role header says so or names the
  constants a role header must define.
- Why: A direct `#include "rocketchip/job_station.h"` is a compile error the file itself does not explain, and
  job.h's preamble documents the roles, the CMake defines and the default but stops short of the contract.
  Both halves fail closed at compile time -- a missing or misplaced constant breaks the station or relay build
  rather than shipping silently -- so the surviving cost is bounded and documentary: whoever hits the error has
  to reverse-engineer the include contract from job.h, and because the push-time station/vehicle parity rebuild
  in docs/agents/SESSION_CHECKLIST.md item 6 is not triggered by include/ or by src/safety/ changes, that break
  lands on whoever next builds the non-default role rather than on the author.
- Confidence: medium
- Direction: Add one preamble line to job.h naming the constants every role header must
  define and stating that role headers are included only from here, and a one-line
  "included via job.h" note at the top of each role header. Documentation only, no code
  change.
- Verdict: RESHAPED -- the include-order half is true and exactly cited, but both halves fail closed at compile time, so the claim is narrowed to a contract-surface documentation gap rather than the latent-breakage framing of the original.

### B04 -- public headers: notify + radio config/scheduler

#### Coverage

- include/rocketchip/notify_backend.h -- FAIL -- Kind C contract surface; both backend declarations read, and the banner's dispatch contract checked against AO_Notify's tick handler and the LED backend, where it does not hold.
- include/rocketchip/notify_intents.h -- PASS -- Kind D vocabulary; all five intent enums, the NotifyState field set and every banner claim (priority order, zero-init default, beacon clear rule, vehicle_lost latch) checked against ao_notify.cpp and notify_backend_led.cpp and found true.
- include/rocketchip/radio_config.h -- PARTIAL -- Kind E map; the generate_profile.py sibling-generation claim verified true, but the per-field range comments are a second copy of the range map that has drifted from the executable validator.
- include/rocketchip/radio_config_table.h -- FAIL -- Kind E/C hybrid; whitelist, both validators and every banner design rule read, with the banner's gate claim, the whitelist's call-site claim and the nav-rate cap rationale all contradicted by the tree.
- include/rocketchip/radio_scheduler.h -- PARTIAL -- real inline code, spine run on all five member functions; behavior of each is correct in isolation, but the type's documented state machine is unenforced and its TX-deadline half is unread.

#### Findings

### CW-B04-01 -- notify_backend.h claims a priority resolution that has not happened yet
- Site: include/rocketchip/notify_backend.h:11-12, 18
- Lens: Comments & documentation quality (JSF AV 131/134, CCG NL.2 "if the comment and the code disagree, both are likely to be wrong"); spine block B confabulation (a confident contract the body does not implement).
- Claim: The header states each backend "receives the resolved NotifyState" and is "Called from AO_Notify's 33Hz tick handler after priority resolution", but AO_Notify performs no priority resolution -- it passes the raw multi-category state, and each backend resolves priority itself.
- Why: ao_notify.cpp:236-237 calls notify_backend_led_update(me->state) and notify_backend_audio_update(me->state) with the whole NotifyState, in which every category slot may be non-kNone at once; the Fault > Cal > Flight > Radio > Sensor > Idle resolver is notify_backend_led.cpp:136-144, inside the LED backend. So the category-priority order is not shared state that arrives resolved -- it is knowledge each backend must re-implement. The header explicitly invites more implementors ("Future OLED and other backends follow the same pattern") and the audio backend is a live stub at notify_backend_audio.cpp:33-35, so the next author reads this contract, trusts that resolution is already done, and acts on whichever field they look at first. Concrete divergence path: with fault = kCore1Stall and cal = kGyro both set, the LED shows the fault while a backend written to this header's contract may act on the calibration slot -- two outputs disagreeing about system state, with no test exercising the combination.
- Confidence: high
- Direction: Either state the truth in the banner (backends receive the unresolved per-category snapshot and must apply the documented priority order themselves) or make the claim true by hoisting the resolver out of the LED backend into a shared step AO_Notify runs before dispatch, so the order is expressed once.
- Verdict: CONFIRMED -- citations exact: notify_backend.h:11-12 and :18 promise a resolved state after priority resolution, src/active_objects/ao_notify.cpp:236-237 passes the whole `NotifyState`, and the Fault > Cal > Flight > Radio > Sensor > Idle resolver lives inside the LED backend at src/notify/notify_backend_led.cpp:136-144, with a live second implementor stubbed at src/notify/notify_backend_audio.cpp:33-35.

### CW-B04-02 -- backend extension point does not state its run-to-completion budget
- Site: include/rocketchip/notify_backend.h:18, 30, 34
- Lens: Comments & documentation quality (JSF AV 134, "assumptions (limitations) made by functions should be documented in the function's preamble"); spine block C blocking-in-cooperative-scheduler.
- Claim: The two backend declarations carry no statement of the timing contract they inherit from being called inside a QV active-object tick handler.
- Why: The header names the call context (a 33Hz tick) but not the obligation that follows from it -- under QV cooperative scheduling a handler must return within roughly one tick period, and a backend that blocks starves every other AO's queue. This is not hypothetical on this codebase: LESSONS_LEARNED Entry 32 records a queue-overflow crash caused by a blocking peripheral call reached from a handler. The backend most likely to be written next is audio, whose stub at notify_backend_audio.cpp:6-7 is slated to drive an I2S codec -- exactly the slow-peripheral case. A future implementor reading only this header sees no budget and has no reason to split a poll-to-completion driver call.
- Confidence: medium
- Direction: Add one preamble line to the header stating the backend contract -- must return promptly, no blocking peripheral waits, called from AO_Notify handler context on Core 0 -- and point at the AO/QV blocking rule rather than restating it.
- Verdict: REFUTED -- the run-to-completion budget the finding wants stated here is already a written, project-wide system invariant that the finding missed: docs/AO_ARCHITECTURE.md:14 ("AO handlers must complete within one tick period. No blocking calls inside handlers.") and docs/decisions/AO_COMMANDMENTS.md:44-45 (~1 ms per handler, start/poll split, LL Entry 32); asking every AO-called declaration to repeat it is a documentation preference.

### CW-B04-03 -- table banner names itself the SET_RADIO_CONFIG gate; it is not
- Site: include/rocketchip/radio_config_table.h:5-8, 14-18 (contradicted in-file at :58-65)
- Lens: Comments & documentation quality (JSF AV 131/134, CCG NL.2); contract-surface helper Step 4 verify-now, comment self-contradiction inside the file.
- Claim: The banner states this table is the canonical list and that "The SET_RADIO_CONFIG dispatcher rejects any incoming config not in this table with denied-ACK", while the same file at :61-62 says the opposite and the dispatcher confirms it.
- Why: ao_telemetry.cpp:254 gates SET_RADIO_CONFIG with radio_config_sx1276_legal, not with the table; radio_config_storage.cpp:117 validates persisted configs the same way. The banner's design rules then attach a strong safety promise to the wrong object -- "Every tuple must be a combination the firmware is tested to operate at" and "Entries that are merely legal on the SX1276 datasheet but untested in this project must NOT appear here" describe a tested-only discipline that the actual runtime gate deliberately does not apply. A reviewer or operator reasoning about what a ground SET command can put the radio into reads the top of this file and concludes the answer is the six rows below, when the reachable set is the whole datasheet-legal space. The two halves of the file cannot both be right, which is the NL.2 case where both are suspect.
- Confidence: high
- Direction: Rewrite the banner to describe the actual two-tier arrangement -- the table is the tested preset list used by the digit-key path and the future scanner, radio_config_sx1276_legal is the runtime SET gate -- and keep the tested-only design rule scoped to the table, with an explicit sentence on why the SET path is intentionally broader.
- Verdict: CONFIRMED -- the banner at :5-8 and the in-file text at :61-62 state opposite things about the same object, and the tree sides with :61-62: src/active_objects/ao_telemetry.cpp:254 and src/logging/radio_config_storage.cpp:117 both gate on `radio_config_sx1276_legal`, not on the table.

### CW-B04-04 -- the 50 Hz nav-rate cap does not prevent the wedge its comment cites
- Site: include/rocketchip/radio_config_table.h:91-94, 104
- Lens: Comments & documentation quality (CCG NL.2 comment/code disagreement); spine block B spec-noncompliance and passes-tests-yet-wrong (unexercised boundary inputs).
- Claim: The comment justifies the nav-rate cap by the interval-shorter-than-airtime failure ("the TX interval check in AO_Telemetry would silently wedge if interval < airtime"), but the cap is a fixed 50 Hz while airtime varies with spreading factor and bandwidth, so the validator still admits configs whose interval is far below their airtime.
- Why: The validator's other clauses accept sf up to 12 (:101) and bw down to 125 kHz (:100), and this project's own airtime figures make the gap large -- ao_radio.cpp:300-304 records roughly 270 ms at SF7/BW125 for a 128-byte payload, and generate_profile.py:297-300 records roughly 100 ms at SF7/BW125 for a 54-byte CCSDS packet. Every step of SF above 7 roughly doubles that again. So the accepted config {bw 125, nav 10, sf 12, cr 8, power 20} passes radio_config_sx1276_legal with a 100 ms interval against an airtime measured in seconds, and even the cap's own boundary case, nav 50 at bw 125, is a 20 ms interval against roughly 100 ms of airtime. This is reachable from the ground: MAV_CMD_USER_2 routes to dispatch_set_radio_config (ao_telemetry.cpp:239-256), whose only RF-parameter gate is this function, and the same function is the validator for the persisted boot config (radio_config_storage.cpp:117), so a bad tuple survives a reboot. The stated failure mode is silent, which is why no test surfaces it.
- Confidence: high on the claim that the cap does not cover the cited failure; medium on how far the resulting behavior degrades in practice.
- Direction: Make the check compute the airtime for the candidate {sf, bw, cr, payload} and refuse when it does not fit the nav period with margin, rather than capping the rate alone; if that is deferred, correct the comment so it no longer claims coverage the code does not provide.
- Verdict: CONFIRMED -- the comment at :92-94 claims the 50 Hz cap covers interval-below-airtime, but :100-104 accept bw 125 with sf up to 12, and the project's own airtime figures (src/active_objects/ao_radio.cpp:300-304, scripts/generate_profile.py:297-300) put SF7/BW125 at 100-270 ms, so `{125, 10, 12, 8, 20}` passes; the path is ground-reachable via MAV_CMD_USER_2 (ao_telemetry.cpp:295-297 into :254) and survives reboot through radio_config_storage.cpp:117.

### CW-B04-05 -- radio_config_in_whitelist documents three call sites and has none
- Site: include/rocketchip/radio_config_table.h:58-65, 66-79
- Lens: Comments & documentation quality (CERT MSC12-C, documentation must describe code that actually runs; JSF AV 131).
- Claim: The function's preamble states it is "Used by the debug-menu digit-key path (q<digit>z), channel-find scanner, and boot seed" and that its name is "retained ... for existing call-site compatibility", but the function has no callers anywhere in the tree.
- Why: A search across all C/C++ sources returns only the definition at :66. The three named consumers do something else: rc_os_debug.cpp:164-169 and rc_os_commands.cpp:95-96 index kRadioConfigTable directly rather than testing membership, the channel-find scanner is still deferred per the banner at :7-8, and the boot seed path (ao_radio.cpp:509-523) validates through radio_config_storage, which uses radio_config_sx1276_legal. So the preamble is a false map of the system in the exact place a reader goes to learn which validator governs which path, and the compatibility rationale defends a constraint that no longer exists -- which also means a future rename or removal will be evaluated against reasons that are not real.
- Confidence: high
- Direction: Decide the function's fate rather than its wording -- if the deferred scanner is the intended consumer, say so as a stated future use and drop the false present tense; if not, remove it and let the table stand alone.
- Verdict: CONFIRMED -- grep across src/, test/ and scripts/ finds no call of `radio_config_in_whitelist`, and each of the three named consumers does something else exactly as described (src/cli/rc_os_debug.cpp:164-169 and src/cli/rc_os_commands.cpp:95-96 index `kRadioConfigTable` directly; the boot seed at src/active_objects/ao_radio.cpp:509-523 goes through radio_config_storage).

### CW-B04-06 -- RadioConfig field-range comments are a stale second copy of the validator
- Site: include/rocketchip/radio_config.h:32-34
- Lens: Comments & documentation quality (JSF AV 131 / CCG NL.2); contract-surface helper Kind E, duplication of the map rather than of a constant.
- Claim: The per-field range comments disagree with the executable validator that actually admits values into these fields -- spreading_factor is annotated "Derived: 6-12" where SF6 is rejected, and nav_rate_hz is annotated "2, 5, or 10" where the accepted set is far wider.
- Why: radio_config_table.h:101 rejects sf below 7, and that same file at :90 states plainly that SF6 is implicit-header only and not supported by this firmware, so the "6" in this header advertises a value the firmware refuses; generate_profile.py:317 only ever emits 7. For nav_rate_hz, the generator accepts 1 through 10 (generate_profile.py:101) and the runtime SET path accepts 1 through 50 (radio_config_table.h:104), so "2, 5, or 10" describes neither. These fields are written by SET_RADIO_CONFIG at ao_telemetry.cpp:265-269, so a reader who trusts this header's ranges when reasoning about a received config -- or who adds a bound check derived from them -- is working from the wrong domain. This is the map-duplication case the helper warns about: the ranges live in two places and only one of them is executable.
- Confidence: medium
- Direction: Correct the spreading_factor bound to 7-12 and replace the nav-rate list with the real accepted range, or better, drop the numeric ranges here and leave a one-line pointer to radio_config_sx1276_legal as the single authority so the second copy cannot drift again.
- Verdict: CONFIRMED -- radio_config.h:34 does say "Derived: 6-12" while radio_config_table.h:101 rejects `sf < 7` and :89 states SF6 is unsupported by this firmware, and :32's "2, 5, or 10" matches neither the generator's 1-10 (scripts/generate_profile.py:101) nor the runtime SET path's 1-50 (radio_config_table.h:104); these fields are written from a received command at ao_telemetry.cpp:265-269.

### CW-B04-07 -- RadioScheduler's documented state machine is enforceable only by convention
- Site: include/rocketchip/radio_scheduler.h:27-32 (with :21 and :31)
- Lens: Class & interface design (JSF AV 67 public data in structs not classes, AV 72 the invariant is part of every constructor's postcondition; CCG C.2 class-vs-struct by invariant).
- Claim: `RadioPhase::kIdle`, documented at :21 as a state of the machine, is produced by no member function -- it is reached only because AO_Radio assigns `s.scheduler.phase` directly at src/active_objects/ao_radio.cpp:571, on the radio-not-detected branch where `init()` (:560) is never called.
- Why: init() sets kRxContinuous or kRxWindow, on_tx_start sets kTxActive, on_tx_complete sets one of the RX phases; nothing in the type yields kIdle. So the documented transition set is incomplete, and the one path that does reach kIdle leaves `tx_interval_ms` and `next_tx_deadline_ms` at static zero rather than at any value the type chose. The trailing underscore on `rx_continuous_` at :31, alone among the four members, shows an encapsulation that was intended and never applied. The broader hazard the original claimed -- that any future caller could drive `phase` to kTxActive without the paired deadline update -- is hypothetical: AO_Radio is the sole owner in the tree, and whether a single-owner public aggregate should be closed is a design preference rather than a defect.
- Confidence: medium
- Direction: Either make the invariant real -- private phase plus a named transition for the init-failure case, so kIdle is reachable through the machine -- or state in the header that this is a deliberately open aggregate whose transitions the owning AO enforces, and name AO_Radio as that sole owner.
- Verdict: RESHAPED -- the concrete half is verified exactly (src/active_objects/ao_radio.cpp:571 is the only producer of kIdle and sits on the branch where `init()` at :560 is skipped), but the "any future caller" hazard is hypothetical and closing a single-owner public aggregate is a design preference, so the claim is narrowed to the state that is unreachable through the machine.

### CW-B04-08 -- the scheduler's TX-deadline half is written but never read
- Site: include/rocketchip/radio_scheduler.h:9, 28-29, 41-45, 68-71
- Lens: Spine block A (CCG ES.3 don't repeat yourself, one piece of knowledge in one place; Fowler Duplicated Code) with CERT MSC12-C on the banner that describes it.
- Claim: tx_slot_open is the only reader of next_tx_deadline_ms and tx_interval_ms and it has no callers, so set_rate and the deadline arithmetic maintain state nothing consumes -- while the banner attributes the TX cadence to this type.
- Why: A search across all C/C++ sources finds no call of tx_slot_open; its sibling rx_active is live at ao_radio.cpp:726, so the absence is specific to the scheduling half. The work still happens: ao_radio.cpp:289 calls s.scheduler.set_rate(rc.nav_rate_hz) and on_tx_complete recomputes next_tx_deadline_ms on every TX, feeding a predicate no one asks. The real cadence gate is elsewhere, and the comment at ao_radio.cpp:290-297 records what that split already cost -- nav_rate_hz had to be wired separately into AO_Telemetry because "without this call, SET_RADIO_CONFIG only updated the RadioConfig struct + dashboard string, not the on-wire cadence", a bug that "made every prior test's nav_rate_hz cosmetic". The rate now lives in two places, and the header's opening claim at :9 that the vehicle "TX at scheduled rate" points a reader at the copy that does not drive the radio. The next person tuning the rate through this type will repeat the same class of bug.
- Confidence: medium
- Direction: Pick one owner of TX cadence. Either route the vehicle TX decision through tx_slot_open so this type earns its deadline fields, or remove the deadline half and set_rate and correct the banner to describe what remains -- a TX/RX phase tracker whose rate is owned by AO_Telemetry.
- Verdict: CONFIRMED -- grep finds no caller of `tx_slot_open` while its sibling `rx_active` is live at src/active_objects/ao_radio.cpp:726, yet `set_rate` (:289) and `on_tx_complete` keep the deadline fields current; the real cadence owner is AO_Telemetry, and the comment at ao_radio.cpp:290-296 records what that split already cost.

### B05 -- public headers: sensor snapshot/seqlock + telemetry surface

#### Coverage

- include/rocketchip/mavlink_rx.h -- PARTIAL -- Read whole as a Kind C API/behavioral contract; the SAFETY CONTRACT's "does not execute state transitions" claim was verified true against src/telemetry/mavlink_rx.cpp, but three load-bearing contract terms (opaque parser buffer sizing, response-buffer accumulation/overflow, ACCEPTED-without-effect on ARM) are not stated; also noted, not filed: the documented parameter now_ms is unused by the implementation (mavlink_rx.cpp:303).
- include/rocketchip/sensor_seqlock.h -- FAIL -- Read whole as Kind B shared protocol plus Kind A ownership map; the protocol body is sound and the barriers are conservatively correct, but the failure contract of seqlock_read, the writer-ownership claim, and one byte-count comment are wrong.
- include/rocketchip/sensor_snapshot.h -- PARTIAL -- Read whole; 40-byte packed layout is internally consistent and static_assert-guarded (verified field-by-field), but the file claims a present-tense use that no firmware code performs.
- include/rocketchip/telemetry_encoder.h -- FAIL -- Read whole as the CCSDS/MAVLink wire contract and cross-checked against src/telemetry/telemetry_encoder.cpp; the packet-length constants and static_asserts are correct and self-consistent, but two wire-format comments state values and a struct member the code does not have.
- include/rocketchip/telemetry_state.h -- PARTIAL -- Read whole; the 45-byte TelemetryState layout is correct and asserted (verified field-by-field), but FlightMetadata's stated size is wrong and unasserted, and two deprecated constants now decode the wrong subsystem.

#### Findings

### CW-B05-01 -- seqlock_read's failure contract cannot be honored by its signature
- Site: include/rocketchip/sensor_seqlock.h:130-145 (contract comment at :144)
- Lens: Comments & documentation quality (JSF AV 134, CCG NL.2) + spine block B unchecked-return residue (P10 Rule 7)
- Claim: The comment "All retries collided - caller uses previous data" promises the caller still holds its previous snapshot on failure, but every attempt that gets past the odd-counter check has already memcpy'd into dst, so on a false return dst holds either a torn snapshot or its prior (possibly never-initialized) contents, and nothing forces the caller to check.
- Why: On the retry-exhausted path the last loop iteration executes memcpy(dst, &sl->data, ...) at :137 and then returns false at :144, so "previous data" no longer exists -- dst has been overwritten with a read the function itself declared inconsistent. The header states neither that dst is clobbered on failure nor that the return must be checked, and seqlock_read is not [[nodiscard]], so the compiler gate cannot help. Six call sites currently discard the return (src/diag/diag_stats.cpp:220, src/cli/rc_os_commands.cpp:899/1428/1446, src/safety/health_monitor.cpp:604/742); diag_stats.cpp:219-231 declares an uninitialized shared_sensor_data_t snap, ignores the result, and prints snap.imu_temperature_c and the counters unconditionally -- so the all-collide path there reads a never-written struct, which is exactly the state the header's comment says cannot happen.
- Confidence: high
- Direction: State the real post-condition in the preamble (dst is written on every attempt; on false its contents are unspecified), and mark seqlock_read [[nodiscard]] so the existing -Werror contract gate covers the callers. If the "previous data" semantic is the one actually wanted, the copy has to land in a scratch buffer that is only published to dst after seq1 == seq2.
- Verdict: CONFIRMED -- verified at sensor_seqlock.h:137/144: every attempt past the odd-counter check memcpy's into dst before the false return, seqlock_read carries no [[nodiscard]], and diag_stats.cpp:219-220 declares `shared_sensor_data_t snap;` uninitialized, discards the result and prints the fields unconditionally; no enabled check covers it (bugprone-unused-return-value.CheckedFunctions holds only flash_safe_execute).

### CW-B05-02 -- nav payload contract derives 42 bytes from a TelemetryState member that no longer exists
- Site: include/rocketchip/telemetry_encoder.h:110 (repeated at :305)
- Lens: Comments & documentation quality (JSF AV 131/134, CCG NL.2 "if the comment and the code disagree, both are likely to be wrong")
- Claim: kNavPayloadLen is documented as the "TelemetryState subset (no _reserved, no met_ms)", but TelemetryState has no _reserved member, its size is 45, and 45 minus met_ms(4) is 41 -- so the stated derivation of 42 does not close and names a field that was removed.
- Why: The real payload, per src/telemetry/telemetry_encoder.cpp:96-108, is the first 40 bytes of TelemetryState plus two explicitly zeroed pad bytes; the struct byte the encoder drops is not "_reserved" but the live flags byte at offset 44 (telemetry_state.h:56, whose bit 0 is kFlagsZuptActive). A reader of this header therefore cannot determine that zupt_active is absent from every CCSDS nav packet, and the decoder doc at :305 repeats the same phantom field, so the station side inherits the same wrong mental model. The .cpp carries the same stale name at telemetry_encoder.cpp:102, so the error is consistent across the pair and will survive a diff-only review.
- Confidence: high
- Direction: Restate the payload as "first 40 bytes of TelemetryState (q_w..battery_mv) + 2 pad bytes; met_ms moves to the secondary header; the flags byte is not transmitted", and decide separately whether dropping flags/zupt_active from the downlink was intended.
- Verdict: CONFIRMED -- TelemetryState (telemetry_state.h:32-57) has no `_reserved` member, and the encoder's own comment at telemetry_encoder.cpp:96-102 shows the payload is the first 40 bytes plus two zeroed pads with the dropped struct byte being `flags`, so the header's stated derivation names a field that does not exist and does not yield 42.

### CW-B05-03 -- APID comment states an on-the-wire value the encoder never emits
- Site: include/rocketchip/telemetry_encoder.h:58-62
- Lens: Comments & documentation quality (CCG NL.2 comment/code disagreement) + spine block B confabulation (NIST AI 600-1: confidently-presented justification the body does not implement)
- Claim: The comment says "On new firmware we always emit 0x101; decoder falls back to 0x001 path if seen", but the constant it annotates is kApidNavWithConfig = 0x004 and that is the value actually placed in the primary header.
- Why: src/telemetry/telemetry_encoder.cpp:149 calls build_primary_header(p, ccsds::kApidNavWithConfig, ...) and :380 matches on the same constant; 0x101 appears nowhere in src/. This is the APID a ground station must filter on, on a project whose stated purpose is standards-fidelity wire formats -- an implementer writing a decoder (or the future Starcom library) from this header would select 0x101 and silently receive nothing, with a CRC-clean packet stream that simply never matches. The surrounding rationale ("old stations that know only 0x001 drop the packet cleanly") reads as authoritative and is what disarms the reader.
- Confidence: high
- Direction: Correct the sentence to name 0x004 (or, if 0x101 was the intended allocation, treat the constant as the defect and change it deliberately with the decoder). Either way the number should appear once, next to the constant, not restated in prose.
- Verdict: CONFIRMED -- `0x101` occurs exactly once in the entire tree (this comment); the constant it annotates is 0x004 and that is the value build_primary_header receives at telemetry_encoder.cpp:149 and that the decoder matches on.

### CW-B05-04 -- seqlock ownership map is wrong for the station role and silent for the six signalling flags
- Site: include/rocketchip/sensor_seqlock.h:33 (and the extern block at :151-159)
- Lens: Concurrency & shared-data ownership (JPL-C Rule 8 single-owner; contract-surface helper Kind A -- "if you cannot name owner, mutator and barrier, the ambiguity is the finding")
- Claim: The header states unconditionally that the shared struct is "Written by Core 1, read by Core 0", but in the ROCKETCHIP_JOB_STATION build the writer is Core 0's idle bridge, so the single stated owner is role-dependent and neither this header nor shared_state.h:63, which repeats the same line, says so.
- Why: src/core1/sensor_core1.cpp:454 is the vehicle writer (Core 1), but src/station/station_idle_tick.cpp:99 calls seqlock_write from the station idle-bridge tick, which is Core 0 work in the ROCKETCHIP_JOB_STATION binary -- still one writer per binary, but not the same core, and the ownership map that exists to answer exactly that question states only the vehicle case. The knowledge is not missing from the tree: station_idle_tick.cpp:16-18 records "same-core writer/reader on station -- memory-order correct without additional sync". It is missing from the header that is supposed to hold it, and shared_state.h:63 inherits the same wrong line. Explicitly not claimed here: the six cross-core atomic flags declared at :151-159 -- the g_core1PauseI2C / g_core1I2CPaused request/acknowledge direction is documented at src/safety/core1_i2c_pause.h:20 and visible at the store sites, so the header's silence there is a missing cross-reference, not an ownership ambiguity.
- Confidence: high on the station-writer discrepancy; medium on the flag block (ambiguity, not a demonstrated dual write)
- Direction: Qualify the writer line per role (vehicle Core 1 sensor loop / station Core 0 idle tick, one writer per binary) and add a one-line writer/reader annotation to each of the six externs, following the shape shared_state.h already uses.
- Verdict: RESHAPED -- the station-writer discrepancy is verified (station_idle_tick.cpp:99 calls seqlock_write from the Core 0 idle bridge), but the six-flag half is weaker than stated: the request/acknowledge direction is documented at src/safety/core1_i2c_pause.h:20 and at the store sites, which the finding missed.

### CW-B05-05 -- GPS section byte-count comment is 12 bytes short of the block it labels
- Site: include/rocketchip/sensor_seqlock.h:67
- Lens: Comments & documentation quality (JSF AV 131 / CCG NL.2)
- Claim: The "// GPS (32 bytes)" section header labels a block that is actually 44 bytes.
- Why: Counting the block at :68-84 gives 28 bytes of lat/lon/alt/speed/course/timestamp/read_count, 8 bytes of fix/sats/valid/gga/gsa/rmc plus the 2-byte pad, and 8 bytes of hdop/vdop = 44. The four section comments sum to 68 + 20 + 32 + 16 + 8 = 144, while the static_assert at :100 (correctly) requires 156 -- so the only in-file explanation of the magic 156 contradicts it by exactly the diagnostic fields and DOP pair that were appended later. A maintainer reconciling this struct against SEQLOCK_DESIGN.md, which the static_assert message points at, is handed the wrong arithmetic.
- Confidence: high
- Direction: Update the GPS section comment to 44 bytes; the same edit should confirm SEQLOCK_DESIGN.md's table matches, since the assert message makes that doc the authority.
- Verdict: CONFIRMED -- the block at :68-84 counts 28 + 8 + 8 = 44 bytes, and the five section comments sum to 144 against the static_assert's 156 at :100.

### CW-B05-06 -- opaque parser_buf size claim is hand-computed and enforced nowhere
- Site: include/rocketchip/mavlink_rx.h:55
- Lens: Class & interface design (CCG C.2/C.3 -- the type must guarantee its own invariant) + spine block C HW/SDK-symbol confabulation (the silent, resolving half)
- Claim: parser_buf[320] carries the comment "sizeof(mavlink_message_t) + sizeof(mavlink_status_t) + padding", but the header deliberately forward-declares those types so no compiler check of that claim is possible here, and the implementation adds none.
- Why: src/telemetry/mavlink_rx.cpp:306-309 reinterpret_casts state->parser_buf to mavlink_message_t and places mavlink_status_t at parser_buf + sizeof(mavlink_message_t); there is no static_assert anywhere in that translation unit. If the vendored c_library_v2 grows a field or the dialect changes MAVLINK_MAX_PAYLOAD_LEN, mavlink_parse_char writes past 320 bytes into the adjacent MavlinkRxState members (encoder pointer, gcs_sysid) with no diagnostic -- a silent memory-corruption path on the untrusted-byte-stream sink the OWASP LLM05 residue in the spine explicitly calls out. The buffer is also only 4-byte aligned (uint8_t array in a struct whose alignment comes from the pointer member) while mavlink_message_t carries 64-bit payload storage.
- Confidence: high on the unenforced size claim; medium on the alignment half (Cortex-M33 tolerates the accesses the compiler currently emits)
- Direction: Put the static_assert in the .cpp where both types are complete (sizeof(mavlink_message_t) + sizeof(mavlink_status_t) <= sizeof(MavlinkRxState::parser_buf)) and give the buffer an explicit alignas; the header comment then becomes a pointer to the assert rather than the only statement of the rule.
- Verdict: CONFIRMED -- mavlink_message_t and mavlink_status_t are forward-declared at :28-31 so no check is possible in the header, and src/telemetry/mavlink_rx.cpp contains no static_assert at all while reinterpret_cast-ing parser_buf to both types at :306-309.

### CW-B05-07 -- MavlinkRxResult contract omits who resets len and what happens when it fills
- Site: include/rocketchip/mavlink_rx.h:66-69 (with the feed_byte preamble at :83-99)
- Lens: Comments & documentation quality (JSF AV 134 -- assumptions and limitations belong in the preamble) + spine block B happy-path-only error handling
- Claim: The header describes result as a caller-owned buffer that feed_byte "writes response frames into", but never states that len is only ever accumulated (never reset by the callee) and that frames which do not fit are discarded with no indication to the caller.
- Why: src/telemetry/mavlink_rx.cpp:67-72 appends at result->len and returns silently when the frame would exceed sizeof(buf); mavlink_rx_feed_byte still returns true, whose documented meaning ("a complete message was parsed") is unaffected by the drop, so a caller has no way to learn a GCS response was lost. The "15 x ~45B" sizing rationale in the comment is not tied to anything: kParamCount is derived from the table in the .cpp with no static_assert against sizeof(buf), so adding a few parameters makes PARAM_REQUEST_LIST truncate mid-burst and the GCS parameter download never completes, with nothing logged. The 770-byte size also matters to callers on this stack budget and is not called out; the one firmware caller zero-initializes a fresh MavlinkRxResult per received byte (src/active_objects/ao_telemetry.cpp:349).
- Confidence: medium
- Direction: State in the preamble that len accumulates until the caller zeroes it, and that responses exceeding buf are dropped; then either add a dropped/overflow flag to MavlinkRxResult or a static_assert tying the worst-case param burst to sizeof(buf) so the sizing rationale is machine-checked.
- Verdict: CONFIRMED -- append_frame (mavlink_rx.cpp:67-72) returns silently when the frame would exceed sizeof(buf) and mavlink_rx_feed_byte still returns true; nothing in the callee resets result->len, and no static_assert ties kParamCount (mavlink_rx.cpp:60-61) to sizeof(buf).

### CW-B05-08 -- header's safety contract does not say ARM and mode commands are ACKed as ACCEPTED while doing nothing
- Site: include/rocketchip/mavlink_rx.h:12-17
- Lens: Comments & documentation quality (JSF AV 134 undocumented limitation) + contract-surface helper section 6 (FAIL shape: an API that promises more than it can keep)
- Claim: The SAFETY CONTRACT block states only what the handler will not do (no ARM, pyro or mode transitions); it does not state what it does instead, which is to answer MAV_CMD_COMPONENT_ARM_DISARM with MAV_RESULT_ACCEPTED.
- Why: src/telemetry/mavlink_rx.cpp:210-213 returns MAV_RESULT_ACCEPTED for ARM/DISARM with the inline note "Pre-Flight Director: ACK but no-op. IVP-67 wires to real ARM", and :201-208 accepts DO_SET_MODE whenever the phase is kIdle -- also without effect. A GCS that receives ACCEPTED displays the vehicle as armed / mode-changed, so the operator-visible state and the vehicle state disagree on a safety-critical command, and the header (the only place a GCS integrator reads) says nothing about it. The header cites IVP-67 as the gate that must own these transitions, which reads as though the wiring exists, while the implementation comment says it does not yet.
- Confidence: medium (the facts are verified; whether the stub ACK is an accepted interim state is the owner's call)
- Direction: Add the limitation to the preamble in one line -- these commands are acknowledged at protocol level only and have no effect on vehicle state -- or change the stub to MAV_RESULT_UNSUPPORTED/DENIED so the acknowledgment cannot be read as an arm confirmation.
- Verdict: CONFIRMED -- mavlink_rx.cpp:210-213 returns MAV_RESULT_ACCEPTED for ARM/DISARM under an explicit "ACK but no-op. IVP-67 wires to real ARM" comment and :201-208 accepts DO_SET_MODE whenever the phase is kIdle; the header's SAFETY CONTRACT states only the negative and never this limitation.

### CW-B05-09 -- encoder-selection comment contradicts the MAVLink size stated twice elsewhere in the same file
- Site: include/rocketchip/telemetry_encoder.h:42
- Lens: Comments & documentation quality (CCG NL.2 / JSF AV 131); contract-surface helper "verify-now" check for in-file self-contradiction
- Claim: EncoderType::kMavlink is annotated "MAVLink v2 3-message set -- ~105 bytes", while the file banner at :17-20 says four messages totalling ~144 bytes and MavlinkEncoder::max_packet_size() at :269 returns 144.
- Why: Three numbers for one thing inside one header, and the encoder really does emit four frames (encode_heartbeat, encode_sys_status, encode_attitude, encode_global_pos, all declared at :228-256 and concatenated by encode_nav at :264). Anyone sizing a TX buffer, an airtime budget, or a duty-cycle estimate from the enum comment underestimates by ~40 bytes per tick; the enum line is the one a caller reads when choosing the encoder, so it is the copy most likely to be trusted.
- Confidence: high
- Direction: Delete the size from the enum comment and let max_packet_size() be the single statement of it, or correct it to four messages / ~144 bytes to match the banner.
- Verdict: CONFIRMED -- three figures for one thing in one file: :42 "~105 bytes", :17-20 "= ~144 bytes total (4 messages per tick)", :269 `return 144`, with four encode_* frames declared at :228-256 and concatenated by encode_nav.

### CW-B05-10 -- stated reason for the explicit barriers misdescribes memory_order_release
- Site: include/rocketchip/sensor_seqlock.h:10-12 (barriers at :124 and :126)
- Lens: Comments & documentation quality (CCG NL.2 "state intent"; the comment is the file's only rationale) + Control-flow discipline / concurrency reasoning (CCG CP.8 as supporting elaboration)
- Claim: The file preamble justifies both __dmb() calls with "memory_order_release only orders the atomic store itself, not the non-atomic memcpy data", which misstates release semantics -- a release store orders everything sequenced before it and nothing sequenced after it.
- Why: The barrier that is genuinely load-bearing is the one at :124: the release store of the odd counter does not stop the following memcpy from being reordered ahead of it, which is what would let a reader see a matching even/even pair around torn data. The barrier at :126 is the redundant one, because the release store of the even counter at :127 already publishes the preceding memcpy. The preamble's single sentence is therefore wrong in both directions at once. Scope of the claim: the per-barrier statements are not wrong -- the inline comment at :124 ("Ensure odd counter visible before data writes") is correct, and docs/decisions/SEQLOCK_DESIGN.md section 4 tabulates both writer barriers with correct per-location reasons -- so the defect is confined to the generalising sentence at :10-12, which is the sentence a reader meets first and the one that design doc's Commandment 6 asks every __dmb() comment to restate. The code as written is conservative and correct; only the justification is wrong, which is why no test or gate will ever flag it.
- Confidence: medium
- Direction: Rewrite the rationale to say that release orders prior accesses only, so an explicit barrier is required after the odd-counter store to keep the data writes from being hoisted above it; note the second __dmb() as belt-and-braces if it is being kept deliberately, and keep the detailed derivation in SEQLOCK_DESIGN.md with a pointer rather than a paraphrase here.
- Verdict: RESHAPED -- the memory-model correction is right (a release store orders prior accesses, so :127 already publishes the memcpy and :124 is the load-bearing barrier), but the "steers a maintainer to delete :124" consequence is overstated: the inline comment at :124 already gives the correct per-barrier reason and SEQLOCK_DESIGN.md section 4 tabulates both barriers correctly.

### CW-B05-11 -- FlightMetadata's stated 14-byte size is 16, and nothing asserts it
- Site: include/rocketchip/telemetry_state.h:87
- Lens: Class & interface design (CCG C.2 -- the type does not enforce the invariant it claims) + Comments (CCG NL.2)
- Claim: The trailing _pad[2] is commented "Align to 14 bytes", but FlightMetadata is not packed and its uint32_t first member gives it 4-byte alignment, so sizeof is 16.
- Why: The members occupy 14 bytes, then the language rounds the size up to a multiple of the alignment. Unlike its siblings in this batch (TelemetryState, SensorSnapshot, CommandAckPayload), this struct has neither __attribute__((packed)) nor a static_assert, so the claim is unchecked -- and it is not a local claim: src/logging/flight_table.h:64 embeds it in the packed, flash-persisted FlightLogEntry with the comment "UTC epoch anchor (14B)", and that entry is CRC-32'd and written to a flash sector whose capacity is reasoned about by hand. Any tool or host-side decoder built from the documented 14 bytes will mis-parse every field after the anchor.
- Confidence: high
- Direction: Add a static_assert(sizeof(FlightMetadata) == 16) (and correct both comments), or pack the struct if 14 was the intended wire/flash size -- but that is a flash-schema change, so it needs the layout decision, not a comment edit.
- Verdict: CONFIRMED -- FlightMetadata's members occupy 14 bytes but its uint32_t first member forces 4-byte alignment and the struct is not packed, so sizeof is 16; there is no static_assert here and none anywhere in src/logging/flight_table.h, whose :64 comment repeats "14B".

### CW-B05-12 -- deprecated health aliases now decode a different subsystem and have no consumers left
- Site: include/rocketchip/telemetry_state.h:66-68
- Lens: Comments & documentation quality (CERT MSC12-C -- remove disabled/obsolete code and the comments mapping it)
- Claim: kHealthEskfHealthy and kHealthZuptActive are kept "until all consumers migrated", but a tree-wide search finds no user outside this header, while the health byte they index has since been redefined as four 2-bit subsystem fields.
- Why: Per the current encoding at :52 and :60-61, bit 0 belongs to IMU's 2-bit field and bit 1 is IMU's high bit, so kHealthEskfHealthy now reads part of IMU health and kHealthZuptActive reads the other part -- while the real zupt flag moved to the flags byte (kFlagsZuptActive at :64). The stated removal condition (all consumers migrated) is already satisfied, so what remains is a pair of live, compilable constants whose names promise one meaning and whose values select another; the DEPRECATED marker does not prevent a new caller from picking the familiar name.
- Confidence: high
- Direction: Delete both constants (git holds them) and drop the IVP-107 note with them; if a station-side or host-tool decoder still needs the legacy bit meanings, that belongs in the decoder, not in the vehicle's wire-format header.
- Verdict: CONFIRMED -- a tree-wide grep finds both constants only at telemetry_state.h:67-68, and per the :52 encoding bits 0 and 1 are IMU's 2-bit field while the live zupt flag is kFlagsZuptActive at :64.

### CW-B05-13 -- SensorSnapshot's stated use does not exist anywhere in the firmware
- Site: include/rocketchip/sensor_snapshot.h:5-8
- Lens: Comments & documentation quality (CERT MSC12-C -- documentation must describe code that actually runs); contract-surface helper claim-vs-truth
- Claim: The file states in the present tense that this layout is "Used for raw sensor logging" although no firmware source reads or writes any SensorSnapshot field, and it contradicts its own brief on units -- baro_pressure_raw is annotated "DPS310 Pa * 100" (:22) in a file whose brief says it holds "ADC counts and raw values before calibration offset/scale application" (:7).
- Why: A search across the tree finds the struct named only here, in test/test_data_model.cpp (which re-asserts the size), and in planning docs. So the 40-byte layout is a reservation whose field semantics no writer has ever fixed -- which is precisely why the units question is still open inside the file: "ADC counts" and "Pa * 100" cannot both be right, and nothing in the tree resolves which one the first implementer inherits. Explicitly not claimed: that the deferral is undocumented. It is -- docs/IVP.md:1640 lists IVP-55 raw sensor logging as deferred and docs/ADVANCED_SETTINGS.md:44 marks it Deferred. The residue is that the one file an implementer of IVP-55 would open states a live use and two incompatible unit conventions. This file is contrast-marked by the repo's own better practice: telemetry_encoder.h:53-56 keeps a reserved APID as a live constant with an explicit "no encoder yet ... Not dead code" note.
- Confidence: medium (I am unsure whether the milestone dead-code inventory script already surfaces header-only orphans of this shape; keeping the finding per the guide's instruction)
- Direction: Change the brief to say the layout is reserved for the deferred IVP-55 raw-logging tier and carries no writer yet, following the kApidDiag annotation style, and resolve the ADC-counts-versus-Pa*100 wording at the same time so the first implementer inherits one answer.
- Verdict: RESHAPED -- the no-writer fact and the in-file unit contradiction are verified, but the deferral itself is documented at docs/IVP.md:1640 and docs/ADVANCED_SETTINGS.md:44, so the claim narrows to this header's own present-tense wording and its two incompatible unit conventions.

### CW-B05-14 -- parked wire-format code is left commented out rather than reserved the way the same file reserves elsewhere
- Site: include/rocketchip/telemetry_encoder.h:72 (and :197-198)
- Lens: Comments & documentation quality (CERT MSC04-C, judging table: commented-out code -- delete, or #if 0 with a reason)
- Claim: kApidStationBeacon reserves APID 0x005 in a comment at :72, so -- unlike kApidDiag at :56, which the same file deliberately keeps as a live constant precisely so the number cannot be reallocated by accident -- nothing stops a later APID allocation from taking 0x005.
- Why: The file solves the identical "park a number for later" problem two ways twenty lines apart, and only one of the two protects the number. kApidDiag at :53-56 is a real constant with an explicit "Not dead code" note, so it is compiler-visible; kApidStationBeacon at :64-72 is prose, invisible to any future allocation. The next person adding an APID reads the live constants, not the parked comment block. Explicitly not claimed as a defect: the commented-out member declaration at :197-198 -- it carries the same parked rationale, it reserves no number that could collide, and delete-versus-comment there is a disposition call for the file's owner.
- Confidence: medium
- Direction: Pick one convention for the file -- either promote kApidStationBeacon to a live reserved constant like kApidDiag, or delete both blocks and keep the parking rationale in the decision record; the struct member declaration should go regardless, since git holds it.
- Verdict: RESHAPED -- the substantive half is the APID reservation that is not compiler-visible; the parked member declaration at :197-198 carries its own stated rationale, reserves no number that could collide, and delete-versus-comment there is a disposition preference rather than a defect.

### B06 -- public headers: remaining small contract headers

#### Coverage
include/rocketchip/ao_signals.h -- FAIL -- Kind D signal catalog + event-struct definitions read whole; catalog numbering re-derived enumerator-by-enumerator and matches every inline value comment, but the event-storage contract stated above the structs is wrong in two places.
include/rocketchip/flash_layout.h -- FAIL -- Kind E layout map read whole and all six region constants recomputed by hand from PICO_FLASH_SIZE_BYTES; the arithmetic is self-consistent, the banner map that documents it is not.
include/rocketchip/fused_state.h -- PARTIAL -- Kind E/B data-layout aggregate, every field's unit/frame documented and consistent; the preamble names a populating function that does not exist in the tree.
include/rocketchip/led_patterns.h -- PARTIAL -- Kind D pattern-code vocabulary; all documented value ranges match the constants, but two pieces of usage prose describing how the codes are consumed are stale.
include/rocketchip/linker_symbols.h -- PASS -- Kind F boundary header; reference-only claim, TP-2 rationale, and single-site suppression all hold; no project definition of either symbol exists.
include/rocketchip/pcm_frame.h -- PARTIAL -- Kind E wire-format contract; standard-frame layout, CRC span and triple-gate prose all verified true against src/logging/pcm_frame.cpp, but the Event frame this same header defines is missing from the format contract.
include/rocketchip/prearm_fail_ticks.h -- PASS -- Kind C pure-helper contract; the 33 Hz / 99-tick / reset-on-repost claims all check out against ao_notify.cpp:230-231 and :262, and the body matches the stated semantics on every path.
include/rocketchip/station_output_mode.h -- FAIL -- Kind A ownership header; the stated single-writer ownership is contradicted by the reader module in the tree today.
include/rocketchip/version.h -- PARTIAL -- Kind E identity map; values are consistent and the one downstream alias derives from it, but the single-source rule it states points at an API that does not exist.

#### Findings

### CW-B06-01 -- Header tells authors QP events may be stack-allocated; every real post site uses static, and LL-35 documents the crash
- Site: include/rocketchip/ao_signals.h:121-122
- Lens: Comments & documentation quality (JSF AV 134 preamble assumptions / CCG NL.2 "if the comment and the code disagree, both are likely to be wrong"); spine block B confabulation (NIST AI 600-1) and Declaration scope & object lifetime (CCG P.8, the QP/C stack-local-event case named verbatim in the lens).
- Claim: The comment introducing every event struct states "QV cooperative scheduling guarantees run-to-completion, so all events can be stack-allocated", which is false for posted events and is the exact reasoning that produced a shipped use-after-free.
- Why: QACTIVE_POST stores the raw QEvt pointer; the receiver dispatches after the poster's frame is gone. docs/agents/LESSONS_LEARNED.md Entry 35 records this as a live crash (Q_onError qf_dyn id=750, event read at stack address 0x20081eec) caused by a stack-local LedPatternEvt, and says the misleading QP comment pattern "should be corrected everywhere it appears". Every post site in the tree already complies with the opposite rule -- ao_telemetry.cpp:185, :225, :854, :897, :1016 and ao_radio.cpp:489 all use static RadioTxEvt/RadioRxEvt storage -- so the header is the single place still asserting the disproven rule, and it is the first thing an author adding a new event struct reads. The claim is only true for the synchronous-dispatch case (SIG_CLI_COMMAND, line 89); the qualifier that would make it safe is absent and the word "all" removes it.
- Confidence: high
- Direction: Replace the sentence with the rule the code actually follows -- posted events must have static or pool-allocated storage because QP/C does not copy them -- and keep the run-to-completion note only as the reason a single static buffer per post site is sufficient. Point at LESSONS_LEARNED Entry 35 rather than restating it.
- Verdict: CONFIRMED -- ao_signals.h:121-122 says verbatim that "all events can be stack-allocated"; LESSONS_LEARNED Entry 35 records the shipped use-after-free this exact reasoning produced and says the pattern should be corrected wherever it appears, and every post site in the tree already uses static storage (ao_telemetry.cpp:185/225/854/897/1016, ao_radio.cpp:489).

### CW-B06-02 -- Two event structs are annotated "allocated from QP/C dynamic event pool"; no pool exists and the same header says so
- Site: include/rocketchip/ao_signals.h:145 and 157
- Lens: Comments & documentation quality (CCG NL.2 comment/code disagreement; CERT MSC12-C, doc-comment describing a mechanism that is never executed).
- Claim: RadioTxEvt and RadioRxEvt are documented as pool-allocated, but there is no event pool in the build and both types are in fact used as file-scope statics.
- Why: A tree-wide search finds no QF_poolInit and no Q_NEW anywhere in src/; the only near-hit is a comment at ao_rf_manager.cpp:297. The actual allocation sites are static rc::RadioTxEvt at ao_telemetry.cpp:185/225/854/897/1016 and static rc::RadioRxEvt at ao_radio.cpp:489. The annotation also contradicts line 122 of this same file ("pool not allocated") twelve lines below it, so a reader cannot determine from the header which of the two claims to believe. Concretely, an author trusting line 145 would write Q_NEW(RadioTxEvt, ...) or add a QF_gc() path for an event that was never pool-allocated.
- Confidence: high
- Direction: Delete both "[C3-A1]" pool annotations, or reduce them to a historical note that the C3-A1 design called for a pool that was never wired. Whichever survives must agree with the storage rule fixed in CW-B06-01.
- Verdict: CONFIRMED -- a tree-wide grep finds no QF_poolInit and no Q_NEW in src/ (the only near-hit is a comment at ao_rf_manager.cpp:297), both types are used as function-scope statics, and line 122 of the same header says the pool is not allocated.

### CW-B06-03 -- Flash-layout banner omits the radio-config region and puts the flash-safe test sector 8 KB off
- Site: include/rocketchip/flash_layout.h:9-14
- Lens: Comments & documentation quality (CCG NL.2 / JSF AV 131-134) applied to a Kind E layout map; contract-surface helper section 4 Kind E "does the file claim to be the only place this knowledge lives".
- Claim: The top-of-file layout table -- the human-readable map this header exists to be -- has not been updated since the Stage T radio-config region was inserted, so it names the wrong address for one region and omits another entirely.
- Why: Recomputing the constants from the code: cal A/B occupy FLASH_SIZE-8KB..FLASH_SIZE (lines 48-51), flight table FLASH_SIZE-16KB..-8KB (55-58), radio config FLASH_SIZE-24KB..-16KB (62-65), flash-safe test FLASH_SIZE-28KB..-24KB (68-69). The banner at line 12 says the flash-safe test sector is at FLASH_SIZE-20KB (off by 8 KB, an address that is now inside the radio-config B sector), and no line of the banner mentions radio config at all. Line 13's "[512KB .. table-20KB] Flight log data" is wrong by the same 8 KB. Anyone reasoning about erase safety or adding a seventh region from this map -- which is exactly what the map is for, since the constants themselves are chained subtractions that are tedious to evaluate by eye -- will place it on top of live radio-config storage.
- Confidence: high
- Direction: Regenerate the banner from the constants below it (six regions, top-down, with the radio-config pair), and state the anchor rule once so the next inserted region forces the same edit.
- Verdict: CONFIRMED -- recomputing from :48-69 puts the flash-safe test sector at FLASH_SIZE-28KB against the banner's FLASH_SIZE-20KB (which now falls inside radio-config sector B), and the banner names no radio-config region at all.

### CW-B06-04 -- flash_layout_valid() documents a binary_end parameter and a boot-time check; it has no parameters and checks only a hardcoded 512 KB reserve
- Site: include/rocketchip/flash_layout.h:81-84 (claim also at line 16)
- Lens: Comments & documentation quality (JSF AV 134, function preamble must state real assumptions/limitations; CCG NL.2); spine block B confabulation -- a confident rationale ("Council C-A4: boot validation ensures regions don't overlap firmware") the body does not implement.
- Claim: The preamble describes a runtime validator taking the firmware's end address from a linker symbol, but the function takes no arguments, runs only at compile time, and compares against the fixed kFlashFirmwareReserve constant rather than the actual binary.
- Why: The declaration on line 84 is `static constexpr bool flash_layout_valid()` -- the documented `binary_end` argument does not exist, and "call from init" is not how it is used: line 99 already forces it via static_assert, so there is nothing for init to call. The substantive gap is that the promised property is not the property checked. kFlashLogStart is pinned to 512 KB (line 45/72) and nothing in the tree compares that to the real image size -- a search for binary_end / __flash_binary_end across all sources and linker scripts returns only this comment line. So a firmware image that grows past 512 KB compiles clean, passes every static_assert here, and the first flight-log write erases the tail of the running program. The header is the one place a reader would look to learn whether that case is covered, and it states that it is.
- Confidence: high
- Direction: Either implement the documented check (compare the linker-provided image end against kFlashLogStart at boot and fail loudly) or rewrite the preamble to say plainly that the guarantee is compile-time and rests on the 512 KB reserve being respected, and record the un-checked overflow case as a known limitation.
- Verdict: CONFIRMED -- the declaration at :84 is `static constexpr bool flash_layout_valid()` with no parameters, :99 already forces it via static_assert so there is nothing for init to call, and `binary_end` / `__flash_binary_end` appear nowhere in the tree except the comment at :82.

### CW-B06-05 -- fused_state.h names eskf_to_fused_state() as its populator; no such function exists
- Site: include/rocketchip/fused_state.h:8
- Lens: Comments & documentation quality (JSF AV 134 / CCG NL.2); spine block B non-self-contained-symbol / confabulation.
- Claim: The file-level contract states "Populated by eskf_to_fused_state() on Core 0 after each propagation", but that symbol does not exist anywhere in the tree.
- Why: A tree-wide search for the name returns exactly one hit -- this comment. The real populator is AO_Logger_populate_fused_state (declared src/active_objects/ao_logger.h:67, defined ao_logger.cpp:185) and it has two callers, ao_logger.cpp:302 and ao_flight_director.cpp:125. This header is the only documentation of who owns the struct, and line 59 of the same file already names the correct function for one field, so the file contradicts itself; a reader trying to find where a field gets its value follows a dead name, and the two-caller fact (the Flight Director also populates it) is what actually matters and is unstated.
- Confidence: high
- Direction: Name AO_Logger_populate_fused_state() in the preamble and state that both AO_Logger and AO_FlightDirector call it, so the "who fills this in" question is answered once at the top.
- Verdict: CONFIRMED -- a tree-wide grep for eskf_to_fused_state returns exactly one hit, this comment; the real populator is AO_Logger_populate_fused_state (ao_logger.h:67, ao_logger.cpp:185) with callers at ao_logger.cpp:302 and ao_flight_director.cpp:125, and line 59 of the same header already names it.

### CW-B06-06 -- station_output_mode.h declares AO_RCOS the sole writer; AO_Telemetry writes it through the exported setter
- Site: include/rocketchip/station_output_mode.h:8 (surface at 26-28)
- Lens: Declaration scope & object lifetime / concurrency-adjacent ownership (contract-surface helper Kind A: owner, mutator, barrier -- "ambiguity itself is a finding", dual writers with no hand-off story); Class & interface design (CCG C.3, the public section should be the contract).
- Claim: The header's stated ownership rule -- "AO_RCOS owns the write side, AO_Telemetry reads it" -- is contradicted by AO_Telemetry, which mutates the mode in the tree today.
- Why: AO_Telemetry_toggle_mavlink() at src/active_objects/ao_telemetry.cpp:791-797 calls AO_RCOS_set_output_mode() on both branches, and ao_telemetry.cpp:1077 sets it again; src/cli/rc_os.cpp:378 and :392 write it as well. The interface offers no mechanism supporting the claim either -- set and cycle are exported globally alongside the getter (lines 27-28), so the ownership rule is enforceable only by convention while the header presents it as fact. The consequence is the classic contract-surface failure: a mode change made by the telemetry path is invisible to anyone reasoning from this header, and the next module to need a write has documented precedent for doing it silently.
- Confidence: high
- Direction: Decide which statement is true and make the file say it -- either restrict the setter to AO_RCOS internals and route AO_Telemetry's toggle through an RCOS-owned command, or rewrite the ownership line to "any module may set; AO_RCOS holds the storage" and list the current writers.
- Verdict: CONFIRMED -- AO_Telemetry_toggle_mavlink calls AO_RCOS_set_output_mode on both branches (ao_telemetry.cpp:793/795) and again at :1077, with rc_os.cpp:378/392 writing it too, against the header's "AO_RCOS owns the write side, AO_Telemetry reads it" at :8; the setter and cycler are exported globally at :27-28 with no mechanism backing the rule.

### CW-B06-07 -- version.h's single-source rule mandates version_string(), which does not exist; the version is also carried twice with nothing linking the two forms
- Site: include/rocketchip/version.h:5 (second half at 12-15)
- Lens: Comments & documentation quality (CCG NL.2 / JSF AV 134) plus contract-surface helper Kind E "does the file claim to be the only place this knowledge lives"; CCG ES.3 (one piece of knowledge, one place) for the duplicated representation.
- Claim: The header states a project-wide rule -- "All print sites must use version_string() or the constants below" -- naming an accessor that is not declared here or defined anywhere in the project, and it stores the version both as a numeric triple and as a literal string with no derivation or assertion tying them.
- Why: A tree-wide search for version_string finds only this comment and an unrelated ETL member (EXTERNAL/etl-20.47.1/etl/version.h). No print site can follow the stated rule as written, and a reader cannot tell whether the accessor is missing, renamed, or planned. Separately, kVersionMajor/Minor/Patch (lines 12-14) have zero consumers in the tree while kFirmwareVersion "0.16.0" has four (rc_os_commands.cpp:741/849, ao_rcos.cpp:195, diag_stats.cpp:37, plus config.h:55 which correctly derives kVersionString from it) -- so the two representations can be bumped independently with nothing, not even a static_assert, to catch the divergence in the file whose entire job is to be the single source. Good news on the rest: config.h:55 derives rather than duplicates, so no live drift exists today.
- Confidence: medium
- Direction: Drop the version_string() clause (or add the accessor if it is genuinely wanted), and either delete the unused numeric triple or derive/assert it against kFirmwareVersion so one edit cannot leave the two forms disagreeing.
- Verdict: CONFIRMED -- version_string exists nowhere in the project (only EXTERNAL/etl-20.47.1/etl/version.h and one line of docs/IVP.md), and kVersionMajor/Minor/Patch have zero consumers while kFirmwareVersion has four plus the derived config.h:55 alias.

### CW-B06-08 -- led_patterns.h usage prose is stale in two places: a removed event type and a migration rationale naming files that no longer use the aliases
- Site: include/rocketchip/led_patterns.h:9 and 121-122
- Lens: Comments & documentation quality (CERT MSC12-C, documentation must describe code that actually runs; CCG NL.2).
- Claim: The header says its codes are posted as "LedPatternEvt or LedOverrideEvt" -- the second type does not exist -- and the 45-entry backward-compatibility alias block justifies itself by naming main.cpp and rc_os.cpp as its users, neither of which uses any alias.
- Why: LedOverrideEvt appears nowhere in the tree; its signal SIG_LED_OVERRIDE is itself marked LEGACY at ao_signals.h:88 and ao_notify.cpp:168 records the subscription being removed at IVP-116, so the header advertises a second live posting path that has not existed for some time. For the alias block, all 79 alias uses in src/ are in ao_led_engine.cpp (43), ao_rcos.cpp (35) and action_executor.h (1) -- main.cpp and rc_os.cpp have none. The stated exit condition for the shim ("until those files are migrated") is therefore already satisfied for the files it names, which means the duplicate non-namespaced vocabulary now has no recorded reason to exist and no owner for retiring it; every constant keeps two spellings indefinitely.
- Confidence: medium
- Direction: Delete the LedOverrideEvt reference, and re-point the alias block's rationale at its actual remaining users (ao_led_engine.cpp, ao_rcos.cpp, action_executor.h) with a concrete retirement condition -- or retire the block, since only three files stand between it and deletion.
- Verdict: CONFIRMED -- LedOverrideEvt appears nowhere in the tree but this comment (its signal is marked LEGACY at ao_signals.h:88), and neither main.cpp nor rc_os.cpp -- the two files the alias block names as its reason to exist -- uses any alias, while ao_led_engine.cpp and ao_rcos.cpp use them heavily.

### CW-B06-09 -- pcm_frame.h's format contract documents only the 55-byte standard frame, while the same header defines a 15-byte Event frame the encoder emits
- Site: include/rocketchip/pcm_frame.h:7-15 and 146-155 (types at 40, 63-74)
- Lens: Comments & documentation quality (JSF AV 134, undocumented limitation of a declared function; CCG NL.2) on a Kind E wire-format contract surface.
- Claim: The file's frame-format block lists frame types 0/1/2 and one 55-byte layout, but kPcmFrameTypeEvent (3) and the 15-byte PcmFrameEvent are also part of the emitted stream, and pcm_find_sync's doc promises "the next valid frame" while the implementation matches standard frames only.
- Why: Everything the header does document is true -- I checked the CRC span (8-byte header + 45-byte payload = bytes 0..52, src/logging/pcm_frame.cpp:59-60) and the triple gate (pcm_frame.cpp:63-85, :87-114) and both match the prose exactly. The gap is what is missing. pcm_encode_event (pcm_frame.cpp:120-134) writes frame_type 3 into the same stream, and pcm_find_sync rejects any candidate whose frame_type is not kPcmFrameTypeStandard (pcm_frame.cpp:100-101), so a ground decoder written from this header's contract silently walks past every event record. The event records are precisely the post-flight triage markers -- kPyroFiredDrogue, kPyroFiredMain, kAbortTriggered, kSafeModeEntry (lines 51-55) -- so the un-decoded case is the one a reviewer most wants after an anomaly. The @return line at 151 gives no hint that the scanner is standard-frame-only.
- Confidence: medium
- Direction: Extend the format block to a two-frame contract (type 3, 15 bytes, event payload) and state on pcm_find_sync that it anchors on standard frames only, with the intended rule for how a decoder walks past or picks up event frames.
- Verdict: CONFIRMED -- kPcmFrameTypeEvent and PcmFrameEvent are defined at :40 and :63-74 and written into the same stream (pcm_frame.cpp:120-134, called from ao_logger.cpp:240), while the format block at :7-15 documents types 0/1/2 and one 55-byte layout and pcm_find_sync rejects any non-standard frame_type at pcm_frame.cpp:100-101 without saying so at :146-155.

### B07 -- math/

#### Coverage

src/math/mat.h -- PARTIAL -- Header-only Mat<R,C> template plus the "ESKF-specific free functions" block; spine + templates + assertions + scope run on all 17 members and 6 free functions; three findings (a documented-but-unused duplicate dense FPFT, unchecked 3x3 block offsets, and the whole file sitting outside the clang-tidy header filter).

src/math/quat.cpp -- PARTIAL -- All 11 definitions read whole; Hamilton product, the rotate() expansion, the DCM and the ZYX Euler forms hand-verified against the Sola (2017) forms they cite; two findings (undeclared unit-norm precondition, near-parallel dead-band in from_two_vectors).

src/math/quat.h -- PARTIAL -- Contract surface (helper Kind C): convention, frame and source citations are present and genuinely load-bearing, but the unit-norm precondition that three of the declared conversions depend on is not stated anywhere in the contract.

src/math/vec3.cpp -- PARTIAL -- Three definitions, all arithmetically correct (cross product signs checked); the near-zero fallback in normalized() is implemented but its consequence is never stated.

src/math/vec3.h -- PARTIAL -- Thin value-type contract surface (helper Kind C); struct-of-public-data shape is correct per CCG C.2, but the two non-total operations it declares carry no contract prose.

#### Findings

### CW-B07-01 -- Quaternion-to-rotation conversions silently require a unit quaternion; nothing declares it

- Site: src/math/quat.cpp:64-75 (also 77-100 and 102-128); declarations at src/math/quat.h:45-54
- Lens: Assertions (JPL-C Rule 16 -- parameter-validity sanity checks; CCG P.5 compile-time-vs-runtime) with Comments (JSF AV 134 -- assumptions and limitations documented in the preamble; CCG NL.2 -- comment and code disagree)
- Claim: rotate(), to_rotation_matrix() and to_euler() implement forms that are only valid for a unit quaternion, and neither the header contract, the bodies, nor an assertion states that precondition -- while the comment at quat.cpp:65-66 asserts the expansion equals q * [0,v] * q*, which it does not for a non-unit q.
- Why: the expansion at quat.cpp:67-74 computes v + 2w(u x v) + 2u x (u x v) = (1 - 2|u|^2)v + 2(u.v)u + 2w(u x v), whereas q * [0,v] * q* = (w^2 - |u|^2)v + 2(u.v)u + 2w(u x v). The two agree only when w^2 + |u|^2 = 1, so on any quaternion that has drifted since its last normalize() the result is neither the stated product nor a rotation -- it is a rotation composed with a silent scale. The same unit hypothesis is baked into m[0] = 1 - 2(yy+zz) at quat.cpp:89 and cosr_cosp = 1 - 2(x*x+y*y) at quat.cpp:108. That the file is not uniformly unit-only is what makes the omission a trap: Quat::inverse() at quat.cpp:28-35 explicitly handles the non-unit case and quat.h:32 comments on it. Callers must therefore each re-derive the rule for themselves -- eskf.cpp:878 and :970, and mahony_ahrs.cpp:54, :81 and :86 (the last rotating q.conjugate(), whose norm is q's) all call rotate() at points whose distance from the nearest normalize() (eskf.cpp:122, :197, :481, :1000) a reader of quat.h cannot see.
- Confidence: high
- Direction: state the unit-norm precondition in quat.h on the three conversions (JSF 134 preamble), and either add an entry assertion on norm_sq() near 1 in the debug path or normalize defensively inside; at minimum correct the quat.cpp:65-66 comment so it does not claim equality with q * [0,v] * q* unconditionally.
- Verdict: CONFIRMED -- the expansion at quat.cpp:67-74 evaluates to (1-2|u|^2)v + 2(u.v)u + 2w(u x v) against q*[0,v]*q* = (w^2-|u|^2)v + 2(u.v)u + 2w(u x v), equal only when w^2+|u|^2 = 1; quat.h:45-54 declares all three conversions with no precondition, no body asserts one, and inverse() at quat.cpp:28-35 handles the non-unit case explicitly, so the file is not uniformly unit-only.

### CW-B07-02 -- fpft_dense claims a verification role that a second, separate implementation actually fills

- Site: src/math/mat.h:224-231 (section banner at 220-222; the file's own contrary rationale at 131)
- Lens: Comments (CERT MSC12-C -- documentation must describe code that actually runs; CCG NL.2) with the spine block A (CCG ES.3 / Fowler Duplicated Code)
- Claim: the comment says fpft_dense is "used as verification path against sparse version", but the dense reference the sparse path is actually checked against is dense_fpft_add in eskf.cpp:306, and fpft_dense has no caller in src/ at all.
- Why: eskf.cpp:360 calls the sparse codegen_fpft; its verification counterpart is dense_fpft_add (eskf.cpp:306-335), whose own comment at :304 states that role correctly. fpft_dense's only callers are test/test_mat.cpp:170, :193 and :356, which check it against a hand-built 3x3, a symmetry property, and 15x15 diagonal positivity -- never against codegen_fpft. So the same covariance-propagation knowledge is written twice in two shapes, and a maintainer who edits or trusts the mat.h copy changes nothing about what verifies flight math. The two shapes are not interchangeable either: dense_fpft_add uses static Mat24 temporaries citing LL Entry 1, which is the same rationale mat.h:131 gives for its own in-place mutators, and is exactly why fpft_dense's return-by-value form (an N x N local at :229 plus the transposed() temporary at :230) cannot be used at N=24 on this target.
- Confidence: high
- Direction: pick one owner -- either delete fpft_dense (or move it to test support) and leave eskf.cpp's as the reference, or give mat.h an in-place N x N form usable at N=24 and have dense_fpft_add call it. Correct the mat.h:224 comment either way so it names what the function is really for.
- Verdict: CONFIRMED -- fpft_dense has no caller in src/ (grep: only test/test_mat.cpp:170/193/356), while the reference the sparse codegen_fpft path (eskf.cpp:360) is actually checked against is dense_fpft_add at eskf.cpp:306, whose own comment at :304 states that role correctly.

### CW-B07-03 -- from_two_vectors discards rotations below ~0.81 degrees, on the path that sets initial attitude

- Site: src/math/quat.cpp:160-163 (threshold constant at src/math/quat.cpp:12)
- Lens: The spine, block B (passes-tests-yet-wrong: unstated boundary assumption on an input the tests do not exercise)
- Claim: the near-parallel branch returns exact identity for any dot product above kDotParallel = 0.9999, i.e. for any true rotation under arccos(0.9999) = 0.0141 rad = 0.81 degrees, although the half-way formula it bypasses is perfectly well-conditioned there.
- Why: the same constant is reused for the antiparallel branch at :165, where it is genuinely required (as d approaches -1 both 1+d and the cross product vanish). On the parallel side there is no such degeneracy -- at d = 0.9999 the returned quaternion at :178 is (1.9999, c) with |c| = 0.0141, which normalizes cleanly -- so the cut-off buys no numerical safety and only deletes signal. The consumers are the attitude-initialisation paths: eskf.cpp:119-121 and mahony_ahrs.cpp:20-22 both call from_two_vectors(body_down, ned_down), so a vehicle sitting within 0.81 degrees of level on the pad is initialised as exactly level and the measured tilt is dropped. The unit tests do not see it: test/test_quat.cpp:266 passes two identical vectors, which is deep inside the dead-band and returns identity correctly either way.
- Confidence: medium
- Direction: give the parallel branch its own, much tighter threshold (or drop it entirely and let the half-way formula run, guarding only the antiparallel case), and if a dead-band is deliberate, say so in the comment with the angle it corresponds to rather than presenting it as a numerical guard.
- Verdict: CONFIRMED -- kDotParallel = 0.9999F (quat.cpp:12) gates the parallel branch at :160-163, arccos(0.9999) = 0.81 deg, the half-way formula at :178 is well conditioned there (|c| = 0.0141 against a scalar of 1.9999), and both attitude-init callers pass body_down/ned_down (eskf.cpp:119-121, mahony_ahrs.cpp:20-22), so the dead-band sits on a live path.

### CW-B07-04 -- vec3.h declares two non-total operations with no statement of their degenerate behaviour

- Site: src/math/vec3.h:23 and src/math/vec3.h:33; implementation at src/math/vec3.cpp:23-29
- Lens: Comments (JSF AV 134 -- assumptions and limitations documented in the function preamble; CCG NL.2)
- Claim: normalized() silently returns the zero vector for any input shorter than 1e-12 and operator/(float) has no guard on its divisor, and the header that other modules read declares both with no comment at all.
- Why: a caller reading vec3.h has no way to learn that a function named normalized() can hand back something that is not a unit vector, so the check has to be re-invented at each call site or omitted. Both in-file consumers omit it: Quat::from_axis_angle (quat.cpp:150-151) and Quat::from_two_vectors (quat.cpp:156-157) use the result as a unit vector directly. With a zero axis, from_axis_angle leaves cosf(half_angle) as the only non-zero component, so the .normalized() at :151 rescales it to plus or minus identity -- a requested rotation is silently converted into no rotation, with no diagnostic. To be precise about severity: the flight callers in tree are guarded upstream (eskf.cpp:104-109 rejects a non-1g accelerometer magnitude before body_down is built), so this is a contract gap on a shared primitive rather than a live fault today -- but the header is the only thing the next caller will read.
- Confidence: high (that the contract is unstated); the live-fault half is explicitly not claimed
- Direction: state the degenerate behaviour on both declarations in vec3.h -- normalized() returns the zero vector below the 1e-12 norm floor, operator/ does not check its divisor -- or make the failure visible to the caller instead of substituting a plausible value.
- Verdict: CONFIRMED -- vec3.h:23 and :33 carry no comment at all, vec3.cpp:23-28 returns the zero vector below the 1e-12 norm floor, quat.cpp:150-151 and :156-157 use the result as a unit vector directly, and the finding already scopes the live-fault half out.

### CW-B07-05 -- The 3x3 block helpers take runtime offsets with an in-bounds precondition that is only prose

- Site: src/math/mat.h:189-218 (precondition stated at src/math/mat.h:185)
- Lens: Assertions (JPL-C Rule 16 -- sanity checks on parameter validity, in-bounds index) with CCG P.5 (prefer compile-time checking to run-time checking)
- Claim: block3, set_block3 and add_block3 index m.data[rb + r][cb + c] for r and c in [0,3) with nothing verifying that rb + 2 and cb + 2 are inside N; the only statement of the requirement is the prose comment at :185, and all three helpers currently have zero callers, so this is an unenforceable precondition on an API that is never instantiated.
- Why: The runtime-offset signature makes the requirement uncheckable by the compiler and nothing substitutes for it: the inner loops are constant-bound so the analyzer sees no out-of-range path, there is no assertion, and there is no compile-time check even though every intended caller would pass the constexpr kIdx* values (eskf_state.h:23-30) that the :185 comment names -- exactly the case CCG P.5 says should fail at compile time. Stated plainly, and this is the limit of the claim: as of this walk a tree-wide grep finds no use of any of the three in src/ or test/, so nothing instantiates them and no out-of-bounds write exists on any path today. The defect is a contract the type cannot enforce on a currently-dead API, which makes the prior question whether three uncalled helpers belong in the header at all.
- Confidence: medium
- Direction: promote rb and cb to non-type template parameters with a static_assert that Rb + 3 <= N and Cb + 3 <= N -- callers already have the offsets as constexpr, so the call sites do not change -- or, if runtime offsets are genuinely wanted, add the entry-point assertion the prose currently substitutes for.
- Verdict: RESHAPED -- the missing bounds check is real, but the out-of-bounds write is not reachable: a tree-wide grep finds no caller of block3/set_block3/add_block3 in src/ or test/, so the templates are never instantiated and the concrete Mat24 failure scenario is hypothetical.

### CW-B07-06 -- These three headers are outside the clang-tidy header filter, so the checks this walk defers to do not run on them

- Site: .clang-tidy:242
- Lens: The guide's mechanical-gating premise (CORE.md Class index note and the spine's section-CM triage -- magic numbers, function size, cognitive complexity, naming and const-correctness are deferred to clang-tidy rather than eyeballed)
- Claim: HeaderFilterRegex is '.*(src|include)/rocketchip/.*', which matches include/rocketchip/*.h but cannot match src/math/mat.h, src/math/quat.h or src/math/vec3.h, so clang-tidy reports no diagnostic located inside any of the three files in this batch.
- Why: mat.h is entirely header-resident code -- every operator, the block helpers, fpft_dense, scalar_update and cholesky live in the header and are only ever instantiated from another translation unit, so every diagnostic they would raise is located in mat.h and filtered away. That inverts the instruction this walk runs under: for these files the "already gated, do not re-hunt" set is not actually gated, and a clean full-tree run says nothing about them either way. I am reporting rather than dropping this per the walk's own instruction to keep a finding when the gating is uncertain; I read the config and confirmed no --header-filter override exists anywhere under scripts/, but I did not run the gate.
- Confidence: medium
- Direction: widen the regex (or pass an explicit --header-filter in the audit script) so that src/**/*.h is covered, then disposition whatever it surfaces in these three files; the change belongs in .clang-tidy, not in the math sources.
- Verdict: CONFIRMED -- HeaderFilterRegex at .clang-tidy:242 is '.*(src|include)/rocketchip/.*', which cannot match src/math/*.h, and a grep across scripts/ and the hooks finds no --header-filter override (the only hit is a cosmetic `grep -v "^Use -header-filter"` at run_clang_tidy.sh:97).

### B08 -- drivers: i2c_bus + spi_bus

#### Coverage
src/drivers/i2c_bus.cpp -- PARTIAL -- All 11 functions read whole and run through spine A/B/C; shape, guard clauses, scope and lifetime are clean (no early-exit leaves the bus half-configured), but the scan carries an unreachable identify-branch and the bit-bang recovery idiom is open-coded twice.
src/drivers/i2c_bus.h -- FAIL -- Contract surface (helper Kind C/E) inventoried in full: the file banner asserts a fixed instance and pin pair that the board-abstracted config immediately below contradicts, and the recovery/scan preambles state none of the cross-core preconditions the call sites actually enforce.
src/drivers/spi_bus.cpp -- PARTIAL -- All 5 functions plus the file-scope atomic read whole; CS is deasserted on every path and the atomic counter's memory order is defensible, but the error-detection helper cannot fire against the blocking SDK API it wraps and `spi_init`'s return is dropped.
src/drivers/spi_bus.h -- FAIL -- Contract surface (Kind C) inventoried: the init preamble names an instance and three GPIO numbers that are true only for one of four supported boards, and the counter block promises a diagnostic the implementation cannot produce.

#### Findings

### CW-B08-01 -- SPI error counter cannot increment, yet a soak gate reads it as evidence
- Site: src/drivers/spi_bus.cpp:26-50 (claim at spi_bus.h:101-104)
- Lens: Comments & documentation quality (JSF AV 131/134 companion NL.2 -- "if the comment and the code disagree, both are likely to be wrong"); spine block C "functionally-correct-but-safety-blind hardware code" and block B confabulation (NIST AI 600-1).
- Claim: `count_error_if()` only increments when a Pico SDK *blocking* SPI call returns fewer bytes than requested, but `spi_write_blocking` / `spi_read_blocking` / `spi_write_read_blocking` have no timeout and no short-return path -- they return `(int)len` unconditionally -- so `g_spi_error_count` is structurally pinned at 0 while the comment claims it counts "timeout / DMA error / etc".
- Why: The counter is not decoration; it is an acceptance criterion. `scripts/soak_station_30min.gdb` prints `g_spi_error_count._M_i` at seven checkpoints and `docs/plans/IVP-132a.4_reeval.md:123,285` gates on "growth < 10 over 30min". A radio soak with genuinely bad wiring or EMI would report a clean 0 at every checkpoint, because no code path can raise it -- the gate reads a constant. This is the LL Entry 43 shape ("clean from a static gate is negative evidence") on a runtime counter: no positive control exists, so 0 proves nothing. `diag_stats.cpp:61` prints the same constant to the operator.
- Confidence: medium -- high on the code as written and on the SDK's documented "@return Number of bytes written" blocking contract, but the SDK source (`pico-sdk/src/rp2_common/hardware_spi/spi.c`) sits outside this walk's tree so I could not read the return statement myself. Verify that first; if any of the three calls can short-return, this finding collapses.
- Direction: Either drive the transfers through an API that can actually report failure (the `spi_*_timeout` / DMA variants, or an explicit MISO-plausibility check on read-back), or delete the counter and its acceptance criterion and say plainly in the header that this driver has no failure channel. A counter that cannot increment is worse than no counter, because a gate is spending trust on it. Whichever way it goes, plant a forced short-count once to positive-control the path before the number is cited as evidence again.
- Verdict: CONFIRMED -- the in-tree half is fully verified: count_error_if fires only on a short return from the three untimed SDK blocking calls, scripts/soak_station_30min.gdb reads the counter at seven checkpoints (:23/56/72/88/104/120/138) and docs/plans/IVP-132a.4_reeval.md:123 and :285 gate on its growth; like the finding, I could not read the SDK source, so the "cannot increment" half still rests on the documented "@return Number of bytes written" blocking contract and should be positive-controlled before the number is cited as evidence again.

### CW-B08-02 -- Recovery/scan entry points document no assumptions despite hard cross-core preconditions
- Site: src/drivers/i2c_bus.h:60-63, 129-147
- Lens: Comments & documentation quality -- JSF AV 134 verbatim: "Assumptions (limitations) made by functions should be documented in the function's preamble."
- Claim: `i2c_bus_scan`, `i2c_bus_recover` and `i2c_bus_reset` are declared with no statement of who may call them or when, although each has a load-bearing precondition that the codebase enforces only at the call sites.
- Why: The rules exist and are written down everywhere except the contract surface. `i2c_bus_scan` must not run while Core 1 owns the bus -- the callers gate it behind `rc_os_i2c_scan_allowed` (rc_os_debug.cpp:109, rc_os_commands.cpp:567, set/cleared at main.cpp:349,356 with the note "LL Entry 23"), and LL Entry 23 records the bus corruption that follows if it runs unguarded. `i2c_bus_recover` / `i2c_bus_reset` deinit and reinit the peripheral out from under any in-flight transaction, and both cores call into them (sensor_core1.cpp:104,226 on Core 1; ao_rcos.cpp:347,1295 and rc_os_commands.cpp:1055,1118 on Core 0) with the pause protocol described only in comments at the call sites. A fourth caller reading only i2c_bus.h has no way to learn any of this and will write the exact call that LL Entry 23 and LL Entry 31 were written to prevent.
- Confidence: high
- Direction: Add one preamble line per entry point stating the precondition and its source -- scan: "caller must hold bus ownership; do not call while Core 1 is in the sensor phase (LL Entry 23)"; recover/reset: "deinits the peripheral -- the other core must be paused (see safety/core1_i2c_pause) and any concurrent transaction is destroyed." Point at the lesson, do not re-narrate it.
- Verdict: CONFIRMED -- i2c_bus.h:60-63 and :129-147 state no caller precondition, while the tree enforces them only at call sites: rc_os_i2c_scan_allowed is checked at rc_os_debug.cpp:109 and rc_os_commands.cpp:567 and set at main.cpp:349/356 with an explicit "LL Entry 23" note, and recover/reset are called from both cores (sensor_core1.cpp:104/226; ao_rcos.cpp:347/1295, rc_os_commands.cpp:1055/1118).

### CW-B08-03 -- Both driver headers document one board's instance and pins for board-abstracted code
- Site: src/drivers/i2c_bus.h:8; src/drivers/spi_bus.h:41-44
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 (comment and code disagree).
- Claim: `i2c_bus.h:8` states "Uses I2C1 on GPIO 2 (SDA) and GPIO 3 (SCL)" and `spi_bus.h:41-44` states "Initialize SPI0 bus / Configures SPI0 at 5 MHz ... Sets up GPIO 20/22/23 for SPI function", but both files select instance and pins through `board.h`, and both contradict themselves twenty lines later with a "Configuration (board-abstracted -- see board.h)" banner.
- Why: The claims are false on a board that ships. `board_fruit_jam.h:32-40` gives I2C0 on GPIO 20/21 and SPI1 on GPIO 28/30/31 -- that is the station role, built from this same source tree. A reader debugging the station scope-probes GPIO 2/3 and 20/22/23 and finds nothing; worse, LL Entry 31 and LL Entry 41 are both Fruit-Jam I2C0/GPIO-20-21 incidents, so the header actively misdirects on the exact hardware that has burned this project twice. `board_pico2.h` and `board_tiny_2350_common.h` disagree with the comment too.
- Confidence: high
- Direction: Replace the hardcoded instance and pin numbers with a pointer to the board header (for example "instance and pins per board::kI2c* / BOARD_I2C_INSTANCE -- see include/rocketchip/board.h"), and leave the Feather values in `board_feather_rp2350.h` where they are already correct. Same edit on both files.
- Verdict: CONFIRMED -- i2c_bus.h:8 names I2C1/GPIO 2,3 and spi_bus.h:40-43 names SPI0/GPIO 20,22,23 while both resolve BOARD_*_INSTANCE and board::kI2c*/kSpi* pins, and board_fruit_jam.h:32-39 gives I2C GPIO 20/21 and SPI GPIO 28/30/31 for the station role built from this same tree.

### CW-B08-04 -- Bus-recovery idiom open-coded twice and already drifted
- Site: src/drivers/i2c_bus.cpp:294-314 (duplicate of 203-232 + 246-272)
- Lens: The spine block A -- CCG ES.3 "Don't repeat yourself" / Fowler Duplicated Code; CCG F.1 (nameable action left inline).
- Claim: `i2c_bus_imu_recovery` re-implements the whole deinit -> SIO -> pull-up -> clock -> STOP -> restore-I2C -> reinit sequence that `i2c_bus_recover` and `clock_pulse_recovery` already express, rather than calling them, and the copy has already diverged from the original.
- Why: The knowledge is the same but the two copies now differ in three ways: the copy omits the SCL-stuck-low precheck that `clock_pulse_recovery:206-215` documents as the condition under which clocking cannot possibly work, omits the per-pulse SDA early exit, and omits the pull-up restore after switching back to `GPIO_FUNC_I2C` that both `i2c_bus_recover:266-267` and `i2c_bus_init:60-61` perform. The pull-up omission happens to be benign today only because `gpio_set_function` does not clear the pad pull bits set at init -- that is luck, not design. The concrete cost: this driver has absorbed three hard-won corrections (LL 28 deinit-before-GPIO-switch, LL 41 pad de-isolation, the Linux SCL-stuck pattern) and a fourth one will have to be applied in two places by whoever reads only the first.
- Confidence: high on the duplication and the divergence; medium on present-day harm (the current divergence is latent, not active).
- Direction: Factor the teardown/restore bracket and the clock loop into one named helper parameterised by pulse count and by whether the SDA early exit applies, and have both `i2c_bus_recover` and `i2c_bus_imu_recovery` call it. Keep the ICM-specific blind register writes in the IMU function -- that part is genuinely its own thing.
- Verdict: CONFIRMED -- i2c_bus_imu_recovery (:294-327) repeats the deinit / SIO / pull-up / clock / STOP / restore / reinit bracket of i2c_bus_recover (:246-272) plus clock_pulse_recovery (:203-232), and all three stated divergences check out: no SCL-stuck precheck, no per-pulse SDA early exit, and no pull-up restore after the GPIO_FUNC_I2C switch at :312-313.

### CW-B08-05 -- Unreachable GPS identify-branch in the bus scan
- Site: src/drivers/i2c_bus.cpp:111-113
- Lens: Comments & documentation quality -- CERT MSC12-C (comment/branch describing never-executed code); spine block A whole-function read.
- Claim: `case kI2cAddrPa1010d:` prints " (PA1010D GPS)" but can never execute, because the loop guard at line 97 excludes that exact address from probing: `if ((addr != kI2cAddrPa1010d) && i2c_bus_probe(addr))`.
- Why: The branch is a false map of the tool's behaviour. A reader of the switch concludes the scanner identifies the GPS, and an operator who runs the scan with the PA1010D connected sees it absent and can reasonably infer the module is dead -- when the scan deliberately never asked (LL Entry 20, correctly explained in the skip comment at lines 95-96). One place says "we skip the GPS," another says "here is how we report the GPS." Note: I am not certain whether the path-sensitive analyzer already flags this; `clang-analyzer`'s unreachable-code checker is not on by default, so I am keeping the finding rather than assuming it is gated.
- Confidence: high
- Direction: Delete the dead case, and if operators need to know the GPS was skipped rather than missing, print one explicit "0x10 skipped (see LL Entry 20)" line after the loop instead of leaving a label that implies it was scanned.
- Verdict: CONFIRMED -- the loop guard at :97 is `(addr != kI2cAddrPa1010d) && i2c_bus_probe(addr)`, so `case kI2cAddrPa1010d:` at :111-113 cannot execute; only clang-analyzer-deadcode.DeadStores is enabled, which does not cover unreachable switch labels, so the finding is not gated.

### CW-B08-06 -- spi_bus_init drops spi_init's return and cannot report failure, yet its result gates radio start
- Site: src/drivers/spi_bus.cpp:31-44 (contract at spi_bus.h:47-48)
- Lens: The spine block B -- Power of Ten Rule 7 (return value of non-void functions must be checked by each calling function) plus JSF AV 134 / NL.2 on the `@return true on success` claim.
- Claim: `spi_init` returns the baud rate actually achieved and that value is discarded at line 33; `spi_bus_init` then returns a literal `true` at line 43, so a function documented as "@return true on success" has no failure path at all.
- Why: The boolean is consumed as a health signal, not ignored: `main.cpp:248` stores it in `g_spiInitialized`, which gates `AO_Radio_start(8U, g_spiInitialized)` at main.cpp:491 and two CLI radio-diagnostic branches (rc_os_commands.cpp:778, 834). Those all read a compile-time-constant true. The asymmetry is visible inside this same batch: `i2c_bus_init:53-56` checks its peripheral-init return and propagates false, so the two sibling drivers make different promises about the same class of failure. A misconfigured peripheral clock (achieved baud 0) would leave the radio AO starting against a bus that never came up, reported as initialized.
- Confidence: medium -- the failure this would catch is rare on a fixed clock tree, so the consequence is a dishonest contract and a dead gate rather than a live bug.
- Direction: Mirror `i2c_bus_init` -- capture `spi_init`'s achieved baud, return false if it is 0 (or outside an acceptable band for a 5 MHz request), and let `g_spiInitialized` mean something. If the project would rather declare this unfailable, change the signature to `void` and say so in the preamble, so no caller keeps branching on a constant.
- Verdict: CONFIRMED -- spi_init's return is dropped at spi_bus.cpp:33 and :43 returns a literal true against "@return true on success" at spi_bus.h:46, while the sibling i2c_bus_init checks its own i2c_init return at i2c_bus.cpp:53-56; the boolean is consumed as a health signal at main.cpp:248/491 and rc_os_commands.cpp:778/834.

### B09 -- drivers: GPS (i2c + uart + shared iface)

#### Coverage
src/drivers/gps.h -- FAIL -- Transport-neutral type + enum contract surface (helper Kind D) walked in full; every field claim inventoried, and the `valid` field's documented rule is contradicted by one of the two backends that produce it.
src/drivers/gps_pa1010d.h -- PASS -- Declaration-only contract surface (helper Kind C) walked in full; `[[nodiscard]]` is applied consistently, every function has a preamble, and the prose matches the `.cpp` bodies (call-rate assumption, buffer-lifetime caveat, and the "debug status survives pre-USB init" rationale all check out).
src/drivers/gps_pa1010d.cpp -- FAIL -- All five public functions plus both statics walked with the spine; init sequence, PMTK const-array construction, and the padding filter verified against their comments; three findings (fix-type derivation, PMTK314 sentence-set claim, `read_nmea_data` parameter contract).
src/drivers/gps_uart.h -- FAIL -- Contract surface walked in full; the API mirrors gps_pa1010d.h cleanly, but two preambles state boot/blocking behaviour the implementation does not have and neither names the core-affinity precondition the IRQ path requires.
src/drivers/gps_uart.cpp -- FAIL -- All public functions, the ISR, and all five statics walked with the spine plus the block-C embedded ADDs (volatile-as-cross-core-barrier, peripheral init/lifecycle); SPSC index arithmetic verified correct, but the concurrency claim, the reinit IRQ lifecycle, the baud comments, and the duplication rationale all fail against the code.
src/drivers/lwgps_opts.h -- PARTIAL -- Vendored-config contract surface (helper Kind E) walked in full; the substantive settings (double precision, CRC on, sentence set, GSV-detail off) are consistent with what the drivers read, but two of its claim lines are wrong. Note: `lib/lwgps/` is an uninitialised submodule in this checkout, so the `LWGPS_CFG_*` names could not be checked against lwGPS's own defaults -- verdict rests on in-file consistency and driver usage only.

#### Findings

### CW-B09-01 -- gps.h documents a `valid` rule the UART backend deliberately does not implement
- Site: src/drivers/gps.h:68 (with src/drivers/gps_uart.cpp:253-257 and src/drivers/gps_pa1010d.cpp:188-189)
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions documented in the preamble) and CCG NL.2 ("if the comment and the code disagree, both are likely to be wrong"); contract-surface helper Kind D (shared vocabulary -- is there one definition?)
- Claim: `gps.h` defines `bool valid` as "Position is valid (RMC active AND GGA fix)", but the UART backend sets it from GGA alone and states in its own comment that it "Deliberately does NOT require RMC=A", so the single transport-neutral field carries two different meanings depending on which backend is bound.
- Why: `gps_data_t` is the whole point of this file -- it is the one contract every consumer depends on, and consumers cannot see which backend produced it. `sensor_core1.cpp:308` gates its position copy on `d.valid` and sets `local_data->gps_valid`, which `eskf_runner.cpp:371` uses (with `gps_fix_type >= 3`) to gate GPS position/velocity injection and the one-shot `set_origin()` call, and `rc_os_commands.cpp:1384` uses for the operator's fix display. Withdrawn on verification: `health_monitor.cpp` never reads `gps_valid` at all -- its GPS health term (`:274`) and GO/NO-GO term (`:764`) are built from `gps_fix_type` and `gps_satellites` only, so the earlier GO/NO-GO consequence does not stand. On the I2C path the surviving gates are backed by an RMC-active check; on the UART path they are not. A reader of `gps.h` -- the only file that claims to define the term -- gets the stricter of the two rules and cannot tell that the preferred vehicle transport (UART is tried first, `main.cpp:130`) uses the looser one. Related shape worth noting in the disposition: because the `valid`/`fix` relationship has no single owner, each consumer re-derives its own threshold (`>= 2` at sensor_core1.cpp:233 and health_monitor.cpp:274, `>= 3` at eskf_runner.cpp:371) -- the invariant is reconstructed by every caller rather than stated once.
- Confidence: high
- Direction: Decide which rule is the contract, then make `gps.h` state it and make both backends implement it -- or, if the two transports genuinely warrant different validity rules, say so in `gps.h` and expose which rule produced the flag so consumers can reason about it. The UART comment's justification (RMC lags GGA on first acquisition) is a real argument; it just needs to win in the header rather than only in one `.cpp`.
- Verdict: RESHAPED -- gps.h:68 vs gps_uart.cpp:253-257 vs gps_pa1010d.cpp:189 all read as claimed and the two backends genuinely disagree, but health_monitor consumes gps_fix_type/gps_satellites and never gps_valid, so the GO/NO-GO consequence is withdrawn and the claim narrowed to the ESKF and CLI consumers.

### CW-B09-02 -- GSA fixMode of "no fix" or "not yet received" is published as a 3D fix
- Site: src/drivers/gps_uart.cpp:228-238 (constant at :53) and src/drivers/gps_pa1010d.cpp:159-173 (constant at :32)
- Lens: The spine, block B (spec-noncompliance / passes-tests-yet-wrong -- "silently drops an explicit semantic requirement... while looking complete") plus Comments & documentation quality (CERT MSC12-C -- documentation describing a rule the code never executes)
- Claim: Both backends map GSA `fix_mode` with `if (fix_mode == kGsaFixMode2d) -> 2D else -> 3D`, so `fix_mode == 1` ("no fix" per the encoding table stated three lines above) is published as `GPS_FIX_3D` -- a case no comment covers -- and the constant written for the ">= 3" test, `kGsaFixMode3d = 3` with the comment "GSA fixMode >= 3 = 3D fix", is defined in both files and used in neither. (The `fix_mode == 0` "not yet received" case is NOT part of this claim: both files explicitly document that default as deliberate.)
- Why: gps_pa1010d.cpp:164-165 and gps_uart.cpp:228-229 both state "GSA fixMode==2 is 2D; anything else (3, or not yet updated) -> 3D", so the optimistic default on a not-yet-received GSA is a documented decision, and the original framing of the GGA-without-GSA window as the live hazard is withdrawn. What survives is narrower and still checkable: (a) `fix_mode == 1`, which the in-file table at gps_uart.cpp:229 / gps_pa1010d.cpp:162 defines as "none", falls into the same `else` and is published as a 3D fix, and no comment covers that case; (b) `kGsaFixMode3d` is dead in both translation units while its comment states a test ("fixMode >= 3 = 3D fix") the code never performs -- the CERT MSC12-C shape. Case (a) is not cosmetic: the value reaches `sensor_core1.cpp:327` -> `gps_fix_type` -> `eskf_runner.cpp:371`, whose `>= 3` gate admits the sample and on the first such sample calls the one-shot `set_origin(...)`, and `rc_os_commands.cpp:1384` shows the operator a 3D lock at the same moment.
- Confidence: high
- Direction: Test against the intent the dead constant already encodes -- 3D only when `fix_mode >= kGsaFixMode3d`, 2D when `== kGsaFixMode2d`, and something conservative (2D, or none) when GSA has not been seen. If defaulting optimistically is deliberate, the reason belongs in the comment, and the now-misleading encoding table and unused constant should go with it.
- Verdict: RESHAPED -- the dead kGsaFixMode3d constant and the uncovered fix_mode==1 case survive, but the adjacent comment in both files ("anything else (3, or not yet updated) -> 3D") documents the fix_mode==0 default as deliberate, so the GGA-without-GSA hazard that carried the original finding is withdrawn.

### CW-B09-03 -- gps_uart_reinit() enables the UART IRQ on the wrong core's NVIC
- Site: src/drivers/gps_uart.cpp:453-454 and :477-478 (against the registration at :376-379); contract at src/drivers/gps_uart.h:103-112
- Lens: The spine, block C (embedded ADD: "Peripheral init SEQUENCE / lifecycle -- a resource added at init but never paired with teardown... walk init->use->teardown as a sequence") plus Declaration scope & object lifetime (CCG CP.2 data-race lifetime -- "who else touches this, and when does it stop being valid?") and JSF AV 134
- Claim: `gps_uart_init()` registers and enables the UART0 IRQ on Core 0 (its own comment at :376 says so), but `gps_uart_reinit()` calls `irq_set_enabled()` -- documented by the vendored SDK as acting "on the executing core" only (pico-sdk 2.2.0 `src/rp2_common/hardware_irq/include/hardware/irq.h:246-252`) -- and its only caller runs on Core 1, so the disable is a no-op for the Core-0 enable and the re-enable adds a second core to the same interrupt.
- Why: The caller is `core1_gps_staleness_check()` -> `gps_uart_reinit()` at `sensor_core1.cpp:275`, reached from `core1_read_gps()` inside the Core 1 sensor loop (`sensor_core1.cpp:422`). After one successful staleness recovery, UART0 is enabled in Core 0's NVIC (never cleared, because the clear ran on Core 1) and in Core 1's NVIC (newly set). `PICO_VTABLE_PER_CORE` defaults to 0 in this SDK (`hardware/irq.h:22-23`), so both cores dispatch the same `gps_uart_rx_isr` -- which destroys the single-producer premise the ring buffer's entire thread-safety argument rests on (gps_uart.cpp:132-139): two cores concurrently doing the non-atomic read-modify-write of `g_rxHead` and writing `g_rxBuf[head]`, and on Core 1 the ISR now preempts the very loop that is draining the ring as its consumer. The reinit window itself is safe (`uart_set_irqs_enabled` masks at the shared peripheral, not the NVIC) -- the defect is the state left behind. `gps_uart.h:103-112` states no core-affinity precondition at all, which is why nothing flagged the mismatch; note also that `core1_read_gps()` is shared with the station idle bridge on Core 0 (`station_idle_tick.cpp:86`), so a second core would execute this path if the station were ever bound to the UART transport (today it is gated off by the `g_gpsTransport != GPS_TRANSPORT_UART` check).
- Confidence: high
- Direction: Make the IRQ enable/disable happen on the core that owns the handler -- either keep all NVIC manipulation on Core 0 (hand the reinit request across cores rather than executing it in place), or drop the `irq_set_enabled` pair and rely on `uart_set_irqs_enabled`, which is peripheral-level and core-independent. Either way, state the core-affinity precondition in the `gps_uart.h` preamble.
- Verdict: CONFIRMED -- gps_uart.cpp:377-378 enables UART0 on the init core (Core 0) while gps_uart_reinit():453/477 runs from sensor_core1.cpp:275 inside the Core 1 loop, and irq_set_enabled is per-executing-core, so a successful staleness recovery leaves the ISR enabled on both cores and breaks the SPSC premise the ring buffer's own comment rests on; no gate covers it and gps_uart.h:103-112 states no core-affinity precondition.

### CW-B09-04 -- cross-core ring declares volatile sufficient and no barrier needed, against the project's own rule
- Site: src/drivers/gps_uart.cpp:141-150 (with the accesses at :194-202 and :395-415)
- Lens: The spine, block C (embedded ADD: "Volatile-as-cross-core-barrier / weak-memory ordering -- volatile used as if it were a cross-core synchronization barrier"; also NIST AI 600-1 confabulation -- a confident justification the mechanism does not support) plus CCG CP.2
- Claim: The thread-safety block asserts "volatile is sufficient, no __dmb() needed" because "RP2350 SRAM is cache-coherent across cores", but coherence is not ordering, and `g_rxBuf` is a plain non-volatile array while only `g_rxHead`/`g_rxTail` are volatile -- so not even the compiler-level ordering the comment relies on ("ISR writes buffer entry BEFORE advancing head") is guaranteed.
- Why: C++ orders volatile accesses only with respect to each other; a non-volatile store such as `g_rxBuf[head] = byte` (:201) may be sunk past the volatile store `g_rxHead = next` (:202), and symmetrically the consumer's non-volatile reads of `g_rxBuf[tail]` (:406-411) may be hoisted above its volatile read of `g_rxHead` (:395). The result is the classic torn handoff: Core 1 feeds `lwgps_process()` bytes whose index has been published but whose payload has not landed -- corrupt NMEA the parser silently discards, presenting as unexplained missing fixes rather than a crash. The reasoning also contradicts `docs/MULTICORE_RULES.md` ("Never Use Plain volatile for Cross-Core Sharing... volatile prevents compiler reordering but does NOT issue ARM hardware memory barriers. Data written on one core may not be visible on the other"), a protected state-of-system doc -- so one of the two is wrong, and the driver comment is the one with no cited source behind it. The confident, technically-flavoured justification is exactly the shape block C says to probe rather than rubber-stamp. Caveat: I was given the comments / scope-lifetime / class-design lens files, not the volatile or concurrency lens, so the formal disposition may belong in that lens's findings table rather than this one.
- Confidence: high
- Direction: Express the handoff with the ordering it needs -- `std::atomic<uint32_t>` head/tail with release on publish and acquire on consume, or the SDK's barrier primitives around the existing volatile pair -- and either delete the "no __dmb() needed" rationale or reconcile it explicitly with `docs/MULTICORE_RULES.md` so the two documents stop disagreeing.
- Verdict: CONFIRMED -- gps_uart.cpp:141-150 states verbatim that volatile is sufficient and no __dmb() is needed, g_rxBuf at :163 is a plain non-volatile array, and docs/MULTICORE_RULES.md ("Never Use Plain volatile for Cross-Core Sharing") is a protected project doc saying the opposite; the two cannot both be right.

### CW-B09-05 -- init/reinit preambles state a 9600-baud, 2-second boot path the code no longer takes
- Site: src/drivers/gps_uart.h:46-48 and :107-108; src/drivers/gps_uart.cpp:19, :22, :187
- Lens: Comments & documentation quality -- CCG NL.2 (comment and code disagree) and JSF AV 134 (limitations stated in the preamble)
- Claim: Five comments describe a single-baud 9600 startup with a 2-second detection window and derive an ISR-rate budget from it, but `acquire_at_target_baud()` (:339-348) tries 57600 first and 9600 second -- so init and reinit can block for up to two full `kInitTimeoutUs` windows (~4 s), and the steady-state link is always 57600.
- Why: The reinit claim is the load-bearing one. `gps_uart.h:107` promises "Blocks for up to 2s during presence detection", and the call site repeats that budget -- but the call runs inside the Core 1 sensor loop (`sensor_core1.cpp:275`), so on the no-GPS-detected path the 1 kHz sampling loop stalls for roughly twice the documented budget before `gps_uart_reinit()` returns false. The `gps_uart.h:46-48` init claim ("Configures UART0 at 9600 baud") is contradicted by the file's own constants block at :31-33, which correctly describes the sticky-baud dual path -- so the header contradicts itself. The ISR comment at gps_uart.cpp:187 ("At 9600 baud, fires at most ~240 times/sec... Total Core 0 CPU impact: <0.1%") computes a Core 0 budget from a rate the driver never operates at; at the real 57600 the same arithmetic gives roughly 6x that, and the file's own ring-sizing comment 35 lines earlier (:152) already uses 57600. Anyone sizing Core 0 headroom, or the Core 1 stall budget, is handed numbers derived from a superseded configuration.
- Confidence: high
- Direction: Restate both preambles against `acquire_at_target_baud()` -- try-57600-then-9600, worst case two detection windows -- and recompute the ISR-rate and CPU-impact note at the operating baud, or point it at the ring-sizing block that already carries the right number.
- Verdict: CONFIRMED -- acquire_at_target_baud() (:339-348) tries 57600 then 9600 and always leaves the link at 57600, so gps_uart.h:46-48 ("Configures UART0 at 9600 baud"), gps_uart.h:107 ("Blocks for up to 2s", real worst case ~2x kInitTimeoutUs) and gps_uart.cpp:187-188's 9600-derived ISR-rate/CPU budget are each contradicted by the code, and the header contradicts its own constants block at :31-33.

### CW-B09-06 -- the comment that authorises the duplicated backend copy is no longer true
- Site: src/drivers/gps_uart.cpp:214-217 (the duplicate body at :219-264 against src/drivers/gps_pa1010d.cpp:150-197)
- Lens: The spine, block A -- CCG ES.3 (don't repeat yourself) / Fowler "Duplicated Code" ("duplicates individually correct and tested, until one requirement change must be made in N places"), with CCG NL.2 on the justification comment itself
- Claim: The doc block justifies keeping two copies of `update_data_from_lwgps()` on the grounds that "both backends produce identical `gps_data_t` from the same lwGPS state" and that the body is "~40 lines of trivial field copies" -- but the two copies have already diverged on the single most consequential field, and the comment that would have stopped a maintainer from noticing still asserts they are identical.
- Why: The bodies differ at exactly one line, and it is the validity rule: gps_pa1010d.cpp:189 computes `lwgps_is_valid(&g_gps) && fix >= 2D`, while gps_uart.cpp:257 computes `(g_gps.fix >= 1) && fix >= 2D` with a comment saying the RMC requirement was dropped on purpose. That is the divergence behind CW-B09-01. Any future edit to either copy -- for instance the fix-mode mapping in CW-B09-02, which is duplicated verbatim in both -- has to be made twice, and the standing comment tells the editor the copies are interchangeable, which is now false. The duplication decision itself is defensible and documented; the finding is that its stated premise has expired without the comment being updated.
- Confidence: high
- Direction: Either restore the premise (make the two bodies genuinely identical by settling the `valid` rule per CW-B09-01) or rewrite the justification to name the deliberate difference, so the next editor is told what must stay in sync and what must not.
- Verdict: CONFIRMED -- the justification at gps_uart.cpp:214-217 asserts the two bodies are identical while gps_pa1010d.cpp:189 and gps_uart.cpp:257 compute the valid flag differently on purpose, so the comment's stated premise has expired.

### CW-B09-07 -- PMTK314 comment claims GSV is enabled; the sentence literal disables it
- Site: src/drivers/gps_pa1010d.cpp:57 (literal at :59) and src/drivers/gps_uart.cpp:106 (literal at :110), with src/drivers/lwgps_opts.h:21
- Lens: Comments & documentation quality -- CCG NL.3 ("point to the spec; do not paraphrase it... on a standards-fidelity / sensor-driver codebase the paraphrase can be subtly wrong") and CERT MSC12-C
- Claim: Both drivers comment the PMTK314 body as "enable RMC + GGA + GSA + GSV sentences (rest disabled)", but in the PMTK314 field order (GLL, RMC, VTG, GGA, GSA, GSV, ...) the literal `0,1,0,1,1,0,...` sets the GSV field to 0, so the module is configured to emit RMC + GGA + GSA only -- and `lwgps_opts.h:21` enables the GSV parser for a sentence the module is told not to send.
- Why: The 19-digit literal is opaque; the comment is the only map a reader has, and it is the same wrong map in two files. Anyone extending the driver to use satellites-in-view (`sats_in_use` comes from GGA/GSA today, so nothing is currently broken) would read the comment, believe GSV is on the wire, and debug the wrong end of the chain. The mirrored effect in `lwgps_opts.h` is dead configuration: parser support compiled in for a sentence stream that never arrives, with no note explaining why. This is precisely the doc-paraphrase-instead-of-doc-pointer failure the comments lens flags as doubly dangerous on a driver, since a subtly wrong paraphrase of a vendor command reads exactly like a correct one.
- Confidence: high -- the finding rests on the standard PMTK314 field order (field 6 = GSV); the GlobalTop command spec is not in this checkout, so treat that field mapping as the one assumption to re-confirm against the primary source.
- Direction: Correct both comments to the sentence set the literal actually selects, and replace the paraphrase with a one-line pointer to the PMTK command reference (section + field numbers) so the next reader checks the spec rather than the prose. Then decide whether `LWGPS_CFG_STATEMENT_GPGSV` should be turned off to match, or GSV genuinely enabled in the PMTK314 body.
- Verdict: CONFIRMED -- in the PMTK314 field order (GLL,RMC,VTG,GGA,GSA,GSV) the literal 0,1,0,1,1,0,... sets GSV to 0, so both comments (gps_pa1010d.cpp:57, gps_uart.cpp:106) name a sentence the command disables, and lwgps_opts.h:21 compiles in a parser for a stream the module is told not to send.

### CW-B09-08 -- read_nmea_data()'s `max_len` bounds a different buffer than its name implies, and is unchecked
- Site: src/drivers/gps_pa1010d.cpp:119-122 (doc block at :104-118, call site at :306)
- Lens: Comments & documentation quality -- JSF AV 134 (a function with a precondition, or a range it does not validate, must say so in its preamble); The spine, block B (P10 Rule 7 residual -- "parameter validity must be checked inside each function") and CCG P.3 (express intent)
- Claim: `read_nmea_data(uint8_t* buffer, size_t max_len)` uses `max_len` as the byte count read into the fixed 255-byte function-local `g_raw`, not as the capacity of the caller's `buffer` its name suggests, and neither bound is documented or checked -- the 15-line preamble explains the padding-filter rationale but says nothing about either parameter.
- Why: The function carries two unstated preconditions: `max_len <= kGpsMaxRead` (or `i2c_bus_read` overruns `g_raw`) and `capacity(buffer) >= max_len` (or the filter loop overruns the caller's array, since `out` can reach `ret`). The single call site passes `kGpsMaxRead` with the 256-byte `g_buffer`, so nothing is live today -- this is a latent contract on a no-MMU, no-sanitizer target where a second caller, or a change to the read size, gets no compiler or runtime signal. A reader who takes `max_len` at face value as "capacity of `buffer`" and passes a smaller array with the default read size writes past it silently.
- Confidence: medium
- Direction: State both bounds in the preamble and pin the internal read to the constant it actually depends on (or assert `max_len <= kGpsMaxRead`); renaming the parameter to say which buffer it bounds would remove the ambiguity without changing behaviour.
- Verdict: CONFIRMED -- read_nmea_data's 15-line preamble (:104-118) documents only the padding filter; max_len bounds the function-local static g_raw[kGpsMaxRead] rather than the caller's buffer, and neither bound is stated or checked, which is exactly the JSF AV 134 preamble gap the finding files (the overflow itself is latent, as the finding says).

### CW-B09-09 -- lwgps_opts.h labels a disabled option "Enable"
- Site: src/drivers/lwgps_opts.h:14-15
- Lens: Comments & documentation quality -- CCG NL.2 / JSF AV 131 (comment and code disagree)
- Claim: `// Enable status callback` sits directly above `#define LWGPS_CFG_STATUS 0`, which disables it -- and the file's own convention elsewhere is to write "Disable ..." over a 0 (lines 23-24 and 26-28).
- Why: This file is a contract surface whose entire content is claims about configuration; there is no code to fall back on. Every other line's comment can be checked against its value and agrees, so a reader reasonably trusts the annotations, and this one asserts a feature is on that is off. Consequence is bounded -- nothing in the drivers registers a status callback, so no behaviour depends on it -- but it is the one line in the file that will mislead someone auditing which lwGPS features this build carries.
- Confidence: high
- Direction: Reword to match the value ("Status callback disabled -- unused"), in the same shape as the neighbouring disable comments.
- Verdict: CONFIRMED -- lwgps_opts.h:14-15 reads "// Enable status callback" directly above "#define LWGPS_CFG_STATUS 0", and the file's own convention (lines 23, 26) writes "Disable ..." over a 0; consequence is nil, which the finding states.

### B10 -- drivers: IMU + baro

#### Coverage

src/drivers/baro_dps310.h -- PARTIAL -- Read whole; contract surface (constants + 7-function C API + one POD data struct) is coherent, but the oversampling/rate decision block attributes a fabricated column to the DPS310 datasheet and contradicts the values it governs (CW-B10-05).

src/drivers/baro_dps310.cpp -- PARTIAL -- Read whole; all seven bodies walked plus the three ruuvi callbacks. Callback error mapping and the two-phase init/start_continuous sequence are sound; the barometric-altitude half is a second, unused, already-divergent copy of the project's altitude model (CW-B10-06). Note: the vendored `lib/ruuvi.dps310.c` source is not present in this checkout, so the third-party half of the contract (`DPS310_READY` bit semantics, `dps310_get_last_result` staleness behaviour) could not be verified against source -- claims about it are not asserted either way.

src/drivers/icm20948.h -- PARTIAL -- Read whole; register/scale contract and the two typedef'd POD structs are honest aggregates (no invariant, correctly struct-shaped, no encapsulation theatre). Two of the twelve declared functions carry a preamble that omits a limitation the rest of the tree treats as load-bearing (CW-B10-04). Eight of the twelve have no call site anywhere in `src/` or `test/` -- recorded as an observation, not filed.

src/drivers/icm20948.cpp -- FAIL -- Read whole; every function walked. Register map, FS_SEL bit positions, accel/gyro sensitivities, AK09916 scale and the temperature formula all verified correct against `docs/hardware/datasheets/ICM-20948-datasheet-v1.3.pdf` (pp. 10-11, 13, 33, 44, 58, 63, 78). Three defects: a null-guard that dereferences the null (CW-B10-01), an unbounded blocking device re-init inside the read path (CW-B10-02), and hidden cross-core phase state outside the device handle (CW-B10-03).

#### Findings

### CW-B10-01 -- Null guard in icm20948_read dereferences the null it just detected

- Site: src/drivers/icm20948.cpp:539-542
- Lens: The spine, block B (defensive-looking error branch that is itself wrong) + Power of Ten Rule 7 (parameter validity must be checked inside each function)
- Claim: The guard `if (dev == nullptr || data == nullptr || !dev->initialized)` calls `memset(data, 0, sizeof(icm20948_data_t))` inside its own body, so the branch that exists to handle `data == nullptr` would write `sizeof(icm20948_data_t)` (44 bytes) through that null pointer -- a self-defeating guard. No caller in the tree can reach it today, so this is a latent contradiction in the guard, not a live UB path.
- Why: Input: any caller that passes `data == nullptr`. The condition can be true precisely because `data` is null, and the very next statement passes that null as memset's destination -- undefined behaviour, and on RP2350 a write into the bootrom/reserved region at address 0. That path is unreachable in the current tree, so the defect is that the one branch written to survive a null output pointer is the one branch that cannot survive it. The branch reads as hardened (it is the only read function in the file that zero-fills on failure) which is exactly why it survives review; the two later error paths in the same function (lines 546-548, 555-557) clear only the validity flags and never touch memory, so the hardening is also inconsistent with itself. No current caller passes null (`sensor_core1.cpp:150`, `cal_hooks.cpp:44`, `rc_os_commands.cpp:401` all pass stack addresses), so nothing exercises it today -- the classic passes-tests-yet-wrong shape. I believe this is ungated: `clang-analyzer-core.*` is enabled in `.clang-tidy` but the checker that models a null argument to `memset` is `unix.cstring.NullArg`, and no `clang-analyzer-unix.*` checker is enabled; newlib's `memset` declaration does not carry `__nonnull`, so `core.NonNullParamChecker` has nothing to fire on. Flagging that I am not fully certain of the gate coverage rather than dropping it.
- Confidence: high
- Direction: Move the zero-fill after the null checks so the null case returns without touching memory, or split the guard so only the dev-null / not-initialized cases zero-fill a known-good `data`. Whichever is chosen, make the three error paths in this function agree on whether the output buffer is cleared, and say so in the header preamble.
- Verdict: RESHAPED -- the guard at icm20948.cpp:539-542 does memset through the pointer it just tested for null and no enabled checker covers it (unix.cstring.NullArg is not in .clang-tidy's list), but the byte count was wrong (sizeof(icm20948_data_t) is 44, not 28) and no caller in the tree can reach the null case, so the claim is narrowed to a self-defeating guard rather than a live UB path.

### CW-B10-02 -- Read path performs an unbounded, blocking magnetometer re-initialisation

- Site: src/drivers/icm20948.cpp:522-529 (branch), 240-274 and 214-237 (the blocking body it calls), 345-354 (the bounded retry it contrasts with)
- Lens: The spine, block C -- Blocking-in-cooperative-scheduler / ISR timing, and Peripheral init SEQUENCE / lifecycle; plus CppCoreGuidelines F.2 (a function should perform a single logical operation)
- Claim: `read_mag_bypass()` -- reached on every `icm20948_read()` call -- re-runs full magnetometer initialisation (bypass reconfiguration plus up to ~210 ms of `sleep_ms`) whenever `dev->mag_initialized` is false, with no attempt cap and no backoff.
- Why: Path: `icm20948_init()` at lines 345-354 bounds its magnetometer retries to `kMagInitRetries` = 3, and then sets `dev->initialized = true` and returns true even when all three failed, leaving `mag_initialized == false`. From that state, every tenth call to `icm20948_read()` takes the lazy-re-init branch and calls `init_magnetometer()`, which costs `sleep_ms(kInitStepDelayMs)` = 10 ms in `enable_bypass_mode()` plus, if the AK09916 answers at all, `sleep_ms(kResetSettleMs)` = 100 ms and up to ten `sleep_ms(10)` WHO_AM_I retries in `configure_magnetometer()` -- the ~220 ms the comment at line 524 itself states. Core 1's sensor loop calls `icm20948_read()` at roughly 1 kHz (`src/core1/sensor_core1.cpp:150`), so a magnetometer that is present-but-unhealthy drives a ~210 ms stall every ten samples, permanently: the IMU sample rate collapses to roughly 48 Hz, the seqlock publish and the ESKF dt step by 210 ms, and `imu_ok` stays true throughout, so `core1_imu_error_recovery()` never runs and no error counter moves. Nothing reports the degradation. Secondarily this is two operations wearing one name -- "read the magnetometer" and "reconfigure the device" -- so a caller cannot tell that a read may write USER_CTRL and INT_PIN_CFG.
- Confidence: high
- Direction: Take recovery out of the read path -- have the read report `mag_valid == false` and let the owning loop (Core 1) decide when to attempt re-init, the way it already schedules `baro_dps310_start_continuous()` retries under a bounded counter in `sensor_core1.cpp:216-222`. If a lazy re-init must stay, give it an attempt cap and a time budget, and state the worst-case blocking time in the `icm20948_read` preamble.
- Verdict: CONFIRMED -- read_mag_bypass's else-if branch (icm20948.cpp:522-529) calls init_magnetometer() every kMagReadDivider=10 calls with no cap; enable_bypass_mode plus configure_magnetometer sum to ~210 ms on the answers-but-fails-WHO_AM_I path, icm20948_init returns true with mag_initialized false, and the ~500 ms Core-1 stall thresholds (ao_led_engine.cpp:55, health_monitor.cpp:346) do not fire on a 210 ms stall, so the degradation really is unreported.

### CW-B10-03 -- Magnetometer read phase is hidden static state outside the device handle, advanced by both cores

- Site: src/drivers/icm20948.cpp:497 (declaration), 496-534 (use)
- Lens: Declaration scope & object lifetime -- Data-race lifetime on shared state (CCG CP.2) and ownership (CCG P.8); supported by Class & interface design (CG C.3, the type is the contract)
- Claim: `read_mag_bypass()` keeps its divider phase in a function-local `static uint8_t g_magDivCount`, so a piece of per-device state lives outside `icm20948_t` and is read-modify-written by every caller on either core with no stated owner and no barrier.
- Why: `icm20948_t` is documented as the "Device handle" (icm20948.h:100-117) and carries every other piece of device state -- address, scale factors, both init flags -- so a reader concludes the driver is handle-scoped and reentrant per handle. It is not. `icm20948_read()` is called from Core 1's sensor loop (`sensor_core1.cpp:150`) and from Core 0 (`rc_os_commands.cpp:401` in `print_direct_sensors`, which takes no I2C pause, and `cal_hooks.cpp:44`). Each Core 0 call advances the same counter, so a Core 0 status read consumes the divider slot that would have been Core 1's magnetometer sample and the mag result lands in Core 0's discarded local buffer -- the published `mag_read_count` silently drops without any error path firing. The same shared counter also gates the CW-B10-02 re-init branch, so a Core 0 read can trigger bypass reconfiguration writes to the ICM while Core 1 is mid-burst; `sensor_core1.cpp:171` states the opposite rule ("Core 0 must NOT do concurrent icm20948_read()") but the driver itself states no ownership rule at all, so a caller cannot learn the constraint from the file it is calling. The ambiguity is the finding: nothing in this file answers who may advance the phase, from which core, under what protection.
- Confidence: medium
- Direction: Move the divider (and any future read-phase state) into `icm20948_t` so it follows the handle, and state the driver's context rule in the header -- single owner core, or the pause discipline callers must hold. If a shared counter is genuinely intended, say so and give it the barrier the cross-core use requires.
- Verdict: CONFIRMED -- g_magDivCount is a function-local static outside icm20948_t, and rc_os_commands.cpp:401 (print_direct_sensors, no core1_i2c_pause) plus cal_hooks.cpp:44 advance it from Core 0 while sensor_core1.cpp:150 advances it from Core 1; the only statement of the rule lives in the caller (sensor_core1.cpp:171), not in the driver.

### CW-B10-04 -- read_accel / read_gyro preambles omit the data-ready limitation the tree relies on

- Site: src/drivers/icm20948.h:193-207 (preambles), src/drivers/icm20948.cpp:567-621 (bodies); cross-reference src/calibration/cal_hooks.cpp:39-42
- Lens: Comments & documentation quality -- JSF AV Rule 134 ("Assumptions (limitations) made by functions should be documented in the function's preamble") and CCG NL.2 (state intent)
- Claim: `icm20948_read_accel()` and `icm20948_read_gyro()` are documented only as "@return true on success", while the tree records that repeated use of the accel-only path wedges the sensor -- and that warning lives in a caller's comment in another module, not in the driver's contract.
- Why: `cal_hooks.cpp:39-42` states verbatim that the accel-only read "does NOT read through TEMP_OUT_L, so the data-ready flag is never cleared. After ~200 reads the output registers stop updating (all zeros)", and for that reason `cal_read_accel()` deliberately calls the full `icm20948_read()` instead. The register spans confirm the mechanism is at least structurally plausible: `read_accel` covers 0x2D-0x32 and `read_gyro` covers 0x33-0x38, neither reaching TEMP_OUT_L at 0x3A, while `icm20948_read()` spans all fourteen bytes through 0x3A. The consequence is that the next author who opens `icm20948.h`, sees a documented "read accelerometer only" entry point, and uses it in a sampling loop reproduces a silent all-zeros fault -- the same failure class the project already had to build a runtime sentinel for (`sensor_core1.cpp:156-166`, accel-magnitude floor). A limitation that one module knows and the owning header does not state is exactly the JSF 134 gap; it is also load-bearing enough that deleting the cal_hooks comment would lose the only record.
- Confidence: high
- Direction: Move the limitation into the `icm20948_read_accel` / `icm20948_read_gyro` preambles in `icm20948.h` as a stated precondition (single-shot use only, or read through TEMP_OUT_L to clear data-ready), leaving `cal_hooks.cpp` a one-line pointer rather than the authority. If the limitation makes the two entry points unsafe for their only plausible use, the honest alternative is to retire them.
- Verdict: CONFIRMED -- icm20948.h:193-207 documents both entry points with "@return true on success" and nothing else, while the register spans (0x2D-0x32 and 0x33-0x38 vs the 14-byte read through 0x3A) and cal_hooks.cpp:39-42's verbatim warning leave the load-bearing limitation in a caller instead of the owning header.

### CW-B10-05 -- DPS310 configuration block attributes a fabricated rate column to datasheet Table 16 and contradicts the values it governs

- Site: src/drivers/baro_dps310.h:26-35 (the table), 37 (the ArduPilot line), 52-55 (the configured values)
- Lens: Comments & documentation quality -- CCG NL.3 (point to the spec, do not paraphrase it) and NL.2 ("If the comment and the code disagree, both are likely to be wrong"); The spine, block B -- confabulation (NIST AI 600-1: confident content carrying fabricated justifying citation)
- Claim: The block presents an eight-row table headed "DPS310 datasheet Table 16 (Infineon)" whose "MaxRate" column does not exist in that table and is wrong, and whose values contradict both the prose beneath it and the constants it sits above.
- Why: I checked the block against the repo's own copy of the primary source, `docs/hardware/datasheets/DPS310-datasheet.pdf` (v1.2, 2020-10-15), p. 29. Table 16 is titled "Precision (Pa RMS) and pressure measurement time (ms) versus oversampling rate" and has exactly two data rows -- measurement time and precision. The comment's MeasTime and Noise columns match it exactly; the "Alt(m)" column is a derived quantity that is not in the datasheet, the "Current@1Hz" column is Table 17 not Table 16, and the "MaxRate" column exists in neither. That fabricated column is also numerically wrong: it reproduces 128/OS, whereas the real limit comes from the duty-cycle constraint the datasheet states on p. 11 and p. 29 (Rate_T x Time_T + Rate_P x Time_P < 1 second), whose n.a. grid in Table 17 gives 8x -> 64 Hz and 16x -> 32 Hz, not the 16 Hz and 8 Hz the comment claims. The contradiction is visible without the datasheet at all: line 31 says 8x maxes at 16 Hz while lines 52-53 configure 8x at 32 Hz, and line 37 says "ArduPilot uses 16x @ 32Hz" while line 32 says 16x maxes at 8 Hz. A reader tuning these constants -- which the block at lines 23-24 and 50-51 explicitly invites -- cannot determine whether the shipped configuration is in spec or whether the table is. (The shipped configuration is in spec: 32 x 14.8 ms + 2 x 3.6 ms = 481 ms, and the duty-cycle arithmetic in lines 46-48 is correct.) The same paraphrase also drops a real constraint it should have carried: the datasheet's PRS_CFG note on p. 28 marks oversampling 16x and above with "Note: Use in combination with a bit shift. See Interrupt and FIFO configuration (CFG_REG) register", which covers most of the range lines 38-39 recommend, and the block is silent on it. Whether the vendored ruuvi driver sets P_SHIFT could not be checked -- `lib/ruuvi.dps310.c` is empty in this checkout -- so I am not asserting a functional defect there, only that the paraphrase omits a precondition on the range it advertises.
- Confidence: high
- Direction: Replace the reproduced table with a one-line pointer to DPS310 datasheet Table 16 / Table 17 and the p. 11 duty-cycle inequality, and keep only what the datasheet cannot say: the project's chosen operating point, the duty-cycle arithmetic for it, and why 8x/32 Hz was picked. If a rate ceiling is genuinely wanted inline, derive it from the duty-cycle inequality rather than restating a column, and note the CFG_REG bit-shift precondition for oversampling above 8x.
- Verdict: CONFIRMED -- the internal contradictions are decisive without the datasheet (line 31 says 8x maxes at 16 Hz while lines 52-53 configure 8x at 32 Hz; line 37 says ArduPilot uses 16x @ 32 Hz while line 32 says 16x maxes at 8 Hz) and the MaxRate column is a 128/OS invention; the CFG_REG bit-shift precondition for 16x and above is confirmed verbatim in the repo's DPS310 PDF (PRS_CFG note, p.29 -- the finding cites p.28, a one-page slip in supporting detail).

### CW-B10-06 -- Second copy of the barometric altitude model, already divergent and with no consumer

- Site: src/drivers/baro_dps310.cpp:25-27 and 194-197, src/drivers/baro_dps310.h:67 and 107-119; duplicate at src/calibration/calibration_manager.cpp:83-86 and 1175-1183
- Lens: The spine, block A -- CCG ES.3 (don't repeat yourself) / Fowler "Duplicated Code"; plus Comments & documentation quality, CERT MSC12-C (documentation must describe code that actually runs)
- Claim: The same barometric-altitude knowledge -- the formula, its comment, and its three constants -- is expressed twice in two translation units, and the driver's copy has already drifted from the calibration copy while having no consumer to expose the drift.
- Why: `kStdAtmPressurePa`, `kHypsometricScale` and `kHypsometricExponent` are defined at baro_dps310.cpp:25-27 and again, with identical values and identical trailing comments, at calibration_manager.cpp:83-86; the bodies at baro_dps310.cpp:196 and calibration_manager.cpp:1182 are the same expression under the same "Barometric formula: h = 44330 * (1 - (P/P0)^0.1903)" comment. They are no longer equivalent: `calibration_get_altitude_agl()` sanity-floors its reference pressure (if p0 < kMinValidPressurePa then p0 = kStdAtmPressurePa) and the driver's copy does not, so the driver would return an infinite or NaN altitude for a reference pressure the calibration path rejects. That divergence is invisible because nothing reads the driver's output: `baro_dps310_data_t::altitude_m` has no consumer in `src/` or `test/` -- both call sites (`sensor_core1.cpp:196-201`, `rc_os_commands.cpp:441-445`) take `pressure_pa` and `temperature_c` only, and the CLI computes altitude via `calibration_get_altitude_agl()` instead. Its reference pressure is likewise frozen: `baro_dps310_set_sea_level()` -- the only writer of `g_seaLevelPa`, documented at baro_dps310.h:107-111 as the configurability hook -- has no call site, so `g_seaLevelPa` is permanently `kStdAtmPressurePa`. `baro_dps310_read()` therefore runs a `powf()` on every Core 1 baro sample to fill a field nobody reads, against a reference nobody can set, and the header's "Altitude in meters (calculated from sea level pressure)" documents a value that never reaches the system.
- Confidence: high
- Direction: Pick one owner for the barometric model -- the calibration path is the one the flight code actually consumes -- and have the other reference it rather than restate it, so the sanity floor and the constants exist once. Then decide the driver's `altitude_m` field and `set_sea_level` / `pressure_to_altitude` entry points explicitly: either wire them to a real consumer or retire them with the doc-comments that describe them.
- Verdict: CONFIRMED -- kStdAtmPressurePa/kHypsometricScale/kHypsometricExponent and the formula are duplicated at baro_dps310.cpp:25-27/196 and calibration_manager.cpp:84-86/1182, only the calibration copy carries the kMinValidPressurePa floor, and grep finds no reader of baro_dps310_data_t::altitude_m and no caller of baro_dps310_set_sea_level anywhere in src/ or test/.

### B11 -- drivers: rfm95w radio

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/drivers/rfm95w.cpp -- PARTIAL -- Read whole; spine A/B/C applied to all 16 functions and to the file-scope constant block, with register writes cross-checked against the file's own datasheet-derived constants and against the driver's only caller (src/active_objects/ao_radio.cpp); five findings.
- C:/Users/pow-w/Documents/RC-agent-walk/src/drivers/rfm95w.h -- PARTIAL -- Read whole and evaluated as a contract surface (helper Kind C API/behavioural contract, secondary Kind E register/layout map); prose-vs-signature and prose-vs-body checks on every doc block produced three findings, two of them shared with the .cpp.

#### Findings

### CW-B11-01 -- Header doc block still describes the superseded DIO0-polling design and a 100 ms timeout
- Site: src/drivers/rfm95w.h:200-201, 265
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); contract-surface helper Kind C, "does the signature match the prose".
- Claim: The public doc-comments for rfm95w_send and rfm95w_available both state the function polls DIO0, and rfm95w_send's states a 100 ms timeout, while the bodies read the latched RegIrqFlags register and use a 150 ms / caller-configurable threshold.
- Why: rfm95w_send_poll (rfm95w.cpp:241) carries an explicit "latched -- Council C3-R3: not GPIO DIO0" note, and rfm95w_available (rfm95w.cpp:337-341) carries a five-line rationale for why DIO0 polling is unreliable on Fruit Jam (GPIO5 shared with Button3 may clamp the line). The header a caller reads first tells that caller the opposite. A caller acting on the header would wire and depend on DIO0 as the completion signal on a board where the driver deliberately stopped trusting it. The 100 ms figure is wrong twice over: rfm95w.h:82-84 defines kTxTimeoutUs as 150000 us, and rfm95w_send_poll uses dev->tx_timeout_us when non-zero, so the real bound is whatever ao_radio installed via rfm95w_set_tx_timeout_us. The .cpp banner at rfm95w.cpp:13 repeats "100ms TX timeout" and attributes it to Council amendment #1, while rfm95w.h:83 attributes the 150 ms value to Council Amendment #2 -- the amendment record disagrees with itself across the pair.
- Confidence: high
- Direction: Rewrite both doc blocks to state the actual completion mechanism (RegIrqFlags read) and to reference kTxTimeoutUs / rfm95w_set_tx_timeout_us rather than a hardcoded millisecond figure; reconcile the two council-amendment lists so one of them is not silently stale.
- Verdict: CONFIRMED -- rfm95w.h:200-201 and :265 both say the driver polls DIO0 while rfm95w_send_poll (:242) and rfm95w_available (:342) read RegIrqFlags with explicit comments saying why, and the 100 ms figure contradicts kTxTimeoutUs=150000 at rfm95w.h:84; the amendment numbering also disagrees between rfm95w.cpp:13 and rfm95w.h:83.

### CW-B11-02 -- rfm95w_send is a caller-less blocking busy-wait still exported as the primary send API
- Site: src/drivers/rfm95w.cpp:271-286
- Lens: The spine, block C (blocking-in-cooperative-scheduler / ISR timing) plus Comments & documentation quality -- CERT MSC12-C (doc-comment describing code that is never executed).
- Claim: rfm95w_send has no caller anywhere in src/, include/ or test/, yet it is the first send function presented in the header and its doc block carries no prohibition against calling it from an active-object handler.
- Why: The body is an unbounded for(;;) that hammers SPI with back-to-back RegIrqFlags reads until send_poll returns kDone or kTimeout -- i.e. it blocks the caller for up to the full TX airtime budget (150 ms by default). The header's own non-blocking section at rfm95w.h:212-214 states that this is exactly what caused QP/C queue overflow (LL Entry 32), and the split API exists to replace it. The hazard is not the current build, where the function is unreferenced; it is that a future caller reading the header top-down meets the blocking version first, fully documented and with no forbidden-context note, and uses it inside a run-to-completion handler. Because the function is never called, its doc-comment currently documents behaviour the system does not produce.
- Confidence: high
- Direction: Either delete the wrapper (git retains it, and the start/poll pair is the supported path), or keep it and add an explicit "not for AO/handler context -- see LL Entry 32" prohibition to its preamble alongside the corrected mechanism/timeout text from CW-B11-01.
- Verdict: CONFIRMED -- grep finds no caller of rfm95w_send in src/, include/ or test/, the body at :276-285 is an unbounded busy-wait, and its own doc block carries no context prohibition (the LL-32 warning sits in the section banner for the next API, rfm95w.h:210-223, not in this function's preamble).

### CW-B11-03 -- SF/BW setters can leave the modem outside the validity window the file's own constant comment states, with no companion-register write
- Site: src/drivers/rfm95w.cpp:399-408
- Lens: The spine, block C (peripheral init sequence / lifecycle, and functionally-correct-but-safety-blind hardware code) plus JSF AV 134 (undocumented limitation).
- Claim: rfm95w_set_spreading_factor accepts SF 6-12 and rfm95w_set_bandwidth accepts any bandwidth code, but neither touches RegModemConfig3, whose default the file itself annotates as valid only for SF<=10 at BW>=125.
- Why: rfm95w.cpp:58-60 states "LowDataRateOptimize[3]=0 (off for SF<=10 at BW>=125)" and writes kModemCfg3Default = 0x04 once, in configure_modem. Nothing else in the file or the tree ever writes RegModemConfig3 again (two uses in the .cpp: one write in configure_modem, one read in rfm95w_read_audit). Setting SF11 or SF12 at BW125 therefore leaves LDRO off in a configuration the file declares invalid, and the modem's symbol timing and the station's decode assumptions diverge with no error path. This is reachable from the link, not just from a compile-time profile: rc::RadioConfig documents spreading_factor as 6-12 (include/rocketchip/radio_config.h:34) and src/active_objects/ao_telemetry.cpp:450 assigns it from lroundf(p3) of a received command parameter with no range check before the driver's clamp. Two corroborating internal inconsistencies point at the same gap: rfm95w_airtime_us clamps SF to 7-12 (rfm95w.cpp:455) while the setter clamps to 6-12, so two functions in this file disagree on the legal SF domain; and the boot audit's expected value kAuditModemCfg3Expected = 0x04 (rfm95w.h:113) hardcodes LDRO-off, so a correctly LDRO-enabled configuration would be reported as an audit mismatch. The SF6 end has the same shape -- the setter accepts 6 while kModemCfg1Default (rfm95w.cpp:47-48) hardcodes explicit-header mode, which SF6 on this part does not use, and no detection-optimize registers are touched.
- Confidence: medium (high that the setters can leave the modem outside the bound the file's own comment states and that nothing rewrites RegModemConfig3; medium on the precise hardware consequence, which I did not re-verify against the SX1276 datasheet in this pass)
- Direction: Either recompute and write RegModemConfig3's LDRO bit inside the SF/BW setters from the resulting symbol time, or narrow the setters' accepted range to the window the constant comment declares valid and say so in the header preamble; whichever is chosen, make rfm95w_airtime_us and kAuditModemCfg3Expected agree with it.
- Verdict: CONFIRMED -- RegModemConfig3 is written exactly once (rfm95w.cpp:138) and read only by the audit (:165), the setters accept SF 6-12 and any bw against the file's own "off for SF<=10 at BW>=125" annotation, SF reaches the setter from a link command via ao_telemetry.cpp:450 unchecked, and rfm95w_airtime_us (:455) disagrees with the setter on the legal SF domain.

### CW-B11-04 -- rfm95w_init leaves two struct fields unwritten, and the field comment's stated default never holds
- Site: src/drivers/rfm95w.cpp:168-176
- Lens: Declaration scope & object lifetime -- JSF AV 143 / CCG ES.22 (value-not-yet-meaningful, two-phase init); Comments & documentation quality -- CCG NL.2.
- Claim: rfm95w_init explicitly assigns seven of rfm95w_t's nine fields but never assigns tx_start_us or tx_timeout_us, and the header's comment on tx_timeout_us claims a default the field is never given.
- Why: rfm95w.h:138-140 says tx_timeout_us is the "Abort threshold for TX polling ... Default kTxTimeoutUs until configured". No code ever stores kTxTimeoutUs into that field; the default is implemented instead as a non-zero test inside rfm95w_send_poll (rfm95w.cpp:254-255). Anyone inspecting the handle -- in GDB, in a CLI diagnostic, or in a caller computing an ACK window from the installed timeout -- reads 0, not 150000, and the comment says otherwise. The correctness of the zero itself depends entirely on the header's "caller-owned, zero-initialized" note at rfm95w.h:156, which is a real precondition the function does not enforce and does not re-establish on the reinit-recovery path (ao_radio.cpp:248 re-calls rfm95w_init on an already-used handle during TX-fail recovery -- the earlier citation of ao_radio.cpp:540 was wrong: that site is the one-shot `radio_ao_initial` transition). Because every other field is written explicitly, a reader cannot tell whether the two omissions are deliberate carry-over across reinit or an oversight.
- Confidence: medium
- Direction: Assign both fields in rfm95w_init (tx_start_us = 0, tx_timeout_us = rfm95w::kTxTimeoutUs) so the handle is fully initialized by the function that owns it, or state in the preamble that these two survive reinit by design; either way make the field comment and the code agree on where the default lives.
- Verdict: RESHAPED -- rfm95w_init (:169-175) really does leave tx_start_us and tx_timeout_us unassigned while rfm95w.h:138-140 promises a kTxTimeoutUs default, but the re-init-on-a-used-handle path is ao_radio.cpp:248 (TX-fail recovery), not :540, which is the one-shot radio_ao_initial transition.

### CW-B11-05 -- send_start's re-entry precondition is neither enforced nor documented
- Site: src/drivers/rfm95w.cpp:203-234
- Lens: Comments & documentation quality -- JSF AV 134 ("assumptions (limitations) made by functions should be documented in the function's preamble"); contract-surface helper Kind C, "what is forbidden".
- Claim: rfm95w_send_start silently aborts an in-flight transmission and resets the timeout clock when called while dev->mode is kTx, and neither the body nor the header preamble states that this is a precondition violation.
- Why: The function's first hardware action is set_mode(kStandby) (rfm95w.cpp:209), which drops a packet the radio is mid-air with; it then rewrites the FIFO and resets dev->tx_start_us (rfm95w.cpp:228). A pending rfm95w_send_poll from the previous packet then observes the new packet's TxDone and reports kDone for a transmission that never completed -- the caller's TX success counter and any ACK-matching logic keyed off it are wrong, and nothing in the driver signals it. The driver holds dev->mode and could test it, but does not; the only guard in the tree is in the caller, at ao_radio.cpp:456, which checks the scheduler phase before calling. The header's usage example at rfm95w.h:216-222 shows the start/poll pattern with no mention of the constraint, and the documented return contract at rfm95w.h:234 lists only "not initialized or len invalid" as false cases.
- Confidence: medium
- Direction: Either return false when dev->mode == kTx so the start/poll state machine enforces its own precondition, or document the abort-in-progress behaviour and the caller's obligation in the preamble and in the header's usage example.
- Verdict: CONFIRMED -- rfm95w_send_start's first action is set_mode(kStandby) (:209) and it resets tx_start_us (:228) with no dev->mode test, while rfm95w.h:234 documents only "not initialized or len invalid" as false cases and the usage example at :216-222 states no constraint; the only enforcement is the caller's phase check at ao_radio.cpp:452.

### CW-B11-06 -- rfm95w_set_bandwidth does not validate its parameter while both sibling setters clamp theirs
- Site: src/drivers/rfm95w.cpp:390-397
- Lens: The spine, block B (parameter validity) -- Power of Ten Rule 7, second half: "parameter validity must be checked inside each function".
- Claim: rfm95w_set_bandwidth shifts the caller's bw value into RegModemConfig1[7:4] with no range check, whereas rfm95w_set_spreading_factor and rfm95w_set_coding_rate, the two functions immediately below it, both clamp.
- Why: With bw > 0x0F the expression (cfg1 & kBwLowerNibbleMask) | (bw << 4) promotes to int, and the static_cast<uint8_t> back-conversion discards the high bits, so the bandwidth field is written with a value the caller never asked for while the function returns normally. The result is a silent RF misconfiguration -- the link simply does not come up, with no error at any layer, which is the worst diagnostic shape for a radio. The header documents the intended domain as "rfm95w::kBw125, kBw250, or kBw500" (rfm95w.h:317) but the code enforces nothing. The current sole caller (ao_radio.cpp:281-284) always passes one of the three named constants, so this is not a live defect today; the finding is that the public setter's contract is unenforced and inconsistent with its two immediate neighbours.
- Confidence: medium
- Direction: Clamp or reject out-of-range bw the way the SF and CR setters do, so all three members of the modem-parameter trio have the same validation posture.
- Verdict: CONFIRMED -- rfm95w_set_bandwidth (:390-397) writes (bw << 4) into RegModemConfig1 with no range check while set_spreading_factor (:401-402) and set_coding_rate (:413-414) both clamp, so the documented domain at rfm95w.h:317 is unenforced; the finding already scopes the consequence as latent given the single caller.

### CW-B11-07 -- The TX-finish register sequence is duplicated across the two exit branches of send_poll
- Site: src/drivers/rfm95w.cpp:244-262
- Lens: The spine, block A -- CCG F.1 (a nameable action left inline; the guide names "a register-poke sequence appearing twice" as the finding shape) and CCG ES.3 / Fowler "Duplicated Code".
- Claim: The three-write "clear IRQ flags, restore DIO0 mapping, return to Standby" sequence appears verbatim in the TxDone branch (lines 246-248) and again in the timeout branch (lines 258-260), differing only in the returned enumerator.
- Why: This is the driver's TX teardown -- the step that leaves the SX1276 in a state the next send_start can rely on. It is one named action ("finish_tx"), expressed as knowledge in two places. The failure mode is drift, not present incorrectness: a later change that adds a step to one exit path (an extra flag clear, a mode read-back) and not the other leaves DIO0 still mapped to TxDone, or the radio still in TX, on whichever path was missed -- and the path most likely to be missed is the timeout path, the one no happy-path test exercises. Because these are two separate if statements rather than an if/else chain, bugprone-branch-clone does not cover them, so this is manual residue rather than a re-hunt of a gated check.
- Confidence: medium
- Direction: Extract the three writes into one named static helper (for example finish_tx(dev)) called from both branches, leaving each branch as the helper call plus its return value.
- Verdict: REFUTED -- three adjacent register writes repeated in two arms of one short function, with no present incorrectness and drift as the only stated failure mode, is a maintainability preference rather than a defect; the project's own clone gate (bugprone-branch-clone) deliberately targets a different shape.

### B12 -- drivers: mcu_temp + ws2812_status

#### Coverage

- src/drivers/mcu_temp.h -- FAIL -- Read whole as a Kind-C contract surface; the datasheet pointer and the ADC-consumer single-owner rule are good, but the stated ADC channel is wrong for RP2350B and the init() success contract is not implementable by the body.
- src/drivers/mcu_temp.cpp -- PARTIAL -- Read whole; conversion, idempotent init, and the sourced/bench-measured stuck-threshold rationale are sound; module statics are mutated on Core 1 and read on Core 0 with no stated ownership rule or barrier.
- src/drivers/ws2812_status.h -- FAIL -- Read whole as the public contract; three separate doxygen claims (init PIO parameter, RSSI-bar mapping, sweep-bar cadence) disagree with the implementation they document.
- src/drivers/ws2812_status.cpp -- FAIL -- Read whole; PIO program/SM lifecycle is correctly paired init-to-deinit and the mode engine is edge-triggered, but init performs no validation of num_leds against either the board sentinel or the fixed pixels[8] buffer, and update_blink omits the zero-period guard its sibling carries.

#### Findings

### CW-B12-01 -- ws2812_status_init discards the caller's PIO instance while the header documents it as the caller's choice
- Site: src/drivers/ws2812_status.h:54 (contract) vs src/drivers/ws2812_status.cpp:249-257 (body)
- Lens: Comments & documentation quality -- JSF AV 131/134, CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine block C, the silent-and-resolving half of SDK-symbol confabulation.
- Claim: The header documents "@param pio PIO instance (pio0, pio1, or pio2)" but the body passes the parameter by address to pio_claim_free_sm_and_add_program_for_gpio_range, which selects a block and overwrites it, so the caller's argument has no effect on which PIO is used.
- Why: The body's own shape proves it -- line 250 passes &pio and line 257 reads the local back into g_state.pio precisely because the SDK replaced it. main.cpp:240 passes pio0, so a reader of the header believes the NeoPixel is pinned to PIO0. It is not: docs/MULTICORE_RULES.md records the NeoPixel as "1 SM on PIO2 SM0 (or wherever pio_claim_free_sm_and_add_program_for_gpio_range lands it)", and PIO2 is the block that also carries the heartbeat watchdog SM and the two backup deployment-timer SMs, whose instruction-slot budget someone will eventually size against this header's promise.
- Confidence: high
- Direction: Document the parameter as ignored on input / selected by the SDK (or drop it and return the chosen instance), and have init report the block and SM it actually claimed so the PIO2 budget can be audited from the log rather than from the header.
- Verdict: CONFIRMED -- ws2812_status.h:54 documents pio as the caller's choice while ws2812_status.cpp:249-250 passes &pio to pio_claim_free_sm_and_add_program_for_gpio_range (which overwrites it) and :257 reads the SDK's selection back, and docs/MULTICORE_RULES.md independently records the NeoPixel as landing wherever the SDK puts it.

### CW-B12-02 -- ws2812_status_init accepts any num_leds: the board's documented "no NeoPixel" sentinel does not skip init, and the pixels[8] bound is unenforced
- Site: src/drivers/ws2812_status.cpp:261 (buffer declared at src/drivers/ws2812_status.cpp:59); contract claim at include/rocketchip/board_pico2.h:49-52
- Lens: Class & interface design -- CCG C.2 / JSF AV 67 (a real cross-member constraint, numLeds bounded by the pixels array, left enforceable only by convention); plus Power of Ten Rule 7, second half ("parameter validity must be checked inside each function").
- Claim: The line "g_state.numLeds = (num_leds > 0) ? num_leds : 1;" converts a declared chain length of 0 into 1 and accepts any value up to 255 into a fixed 8-entry buffer, while the only statement of the bound is the comment "Max 8 LEDs per chain".
- Why: Two concrete consequences. (a) board_pico2.h:50 states that 0 is a neutral "no NeoPixel chain" sentinel so WS2812 init skips -- nothing skips. PICO_BOARD=pico2 is an explicitly supported override (CMakeLists.txt:212 "Users can still override explicitly"), main.cpp:240 calls init unconditionally, the driver coerces the count to 1, claims a PIO state machine plus program slot, and drives kNeoPixelPin (GPIO 0 on that board) as a PIO output on hardware with no NeoPixel -- the pin-function / PIO-claim failure class already recorded in LESSONS_LEARNED Entries 33 and 42. (b) Any num_leds above 8 makes ws2812_show (line 140), ws2812_set_rssi_bar (line 159) and ws2812_set_sweep_bar (line 215) index past pixels[8]; the latter two write. No board currently declares more than 5, so (b) is latent while (a) is live for the pico2 configuration.
- Confidence: high
- Direction: Return early (false, or a documented no-op success) when num_leds is 0 so the board sentinel means what board_pico2.h says it means, and reject or clamp above the array bound; give the bound a named constant used by both the array declaration and the guard.
- Verdict: CONFIRMED -- board_pico2.h:49-52 states 0 is a "no NeoPixel chain" sentinel so WS2812 init skips, ws2812_status.cpp:261 coerces it to 1 and proceeds to claim a PIO SM and drive GPIO 0, main.cpp:240 calls init unconditionally, and the pixels[8] bound at :59 exists only as a comment.

### CW-B12-03 -- RSSI-bar header contract disagrees with the implementation on thresholds, colors, and no-signal behavior
- Site: src/drivers/ws2812_status.h:112-121 vs src/drivers/ws2812_status.cpp:163-195
- Lens: Comments & documentation quality -- JSF AV 131 / 134, CCG NL.2 (comment and code disagree).
- Claim: The doxygen block states thresholds and a color legend that the body does not implement, and describes a no-signal animation that does not exist.
- Why: Three checkable disagreements. (1) The doc says "Strong (>-70): green. Marginal (-70 to -90): yellow. Weak (<-90): red"; the body branches on -60 / -70 / -80 / -95 (lines 174-184). (2) The body paints the lower half of the lit pixels green unconditionally (line 188), so at -85 dBm two pixels light as green plus red -- the doc's "marginal" band never produces the all-yellow reading it promises. (3) The doc says "No signal: all off with slow red pulse"; lines 163-168 set pixel 0 to a static dim red and return, and no pulse or animation for that state exists anywhere in the driver. The station's link-quality legend is read off this header, so an operator interpreting the bar during a launch window is working from a legend the firmware does not honor.
- Confidence: high
- Direction: Rewrite the block to the implemented thresholds and color rule, and make an explicit call on the "slow red pulse" -- either implement it or delete the promise; it currently reads as a specified behavior that was never built.
- Verdict: CONFIRMED -- all three disagreements check out line by line: doc thresholds -70/-90 vs body branches at -60/-70/-80/-95 (:174-184), the unconditional lower-half green at :188 (at -85 dBm the bar reads green plus red, never all-yellow), and the promised slow red pulse against the static dim red at :163-168.

### CW-B12-04 -- mcu_temp.h states ADC input 4 while the implementation selects input 8 on RP2350B
- Site: src/drivers/mcu_temp.h:6-8 vs src/drivers/mcu_temp.cpp:25-32
- Lens: Comments & documentation quality -- JSF AV 131 / 134, CCG NL.2; header evaluated as a contract surface (helper Kind C/E, "single map" question).
- Claim: The header's opening line asserts "On-die sensor on ADC input 4. Available on every RP2350 variant (A/B)", but the .cpp selects input 4 only under PICO_RP2350A and input 8 otherwise.
- Why: The station role builds for the Fruit Jam, which is RP2350B (include/rocketchip/board_fruit_jam.h), so on that build the header names the wrong channel. This is not a cosmetic detail in this file: the same header (lines 14-17) instructs future readers to serialize adc_select_input against any additional ADC consumer, and a reader planning that battery-ADC channel allocation from the header would reason about the wrong reserved input on the station board. The .cpp is correct and even cites the SDK header it derived the split from; only the contract surface is wrong.
- Confidence: high
- Direction: Replace the flat "input 4" with the per-variant statement (or a one-line pointer to the kTempAdcInput constant), keeping the "no extra hardware on either variant" claim which is true.
- Verdict: CONFIRMED -- mcu_temp.h:6 asserts ADC input 4 on every RP2350 variant (A/B) while mcu_temp.cpp:28-32 selects input 4 only under PICO_RP2350A and input 8 otherwise, which is the station (Fruit Jam / RP2350B) build.

### CW-B12-05 -- mcu_temp_init is documented as returning success but can only return true, and its only caller discards it
- Site: src/drivers/mcu_temp.h:27-30, src/drivers/mcu_temp.cpp:48-56, src/main.cpp:242
- Lens: Comments (NL.2 / JSF 134 -- comment asserts what the body does not do) plus spine block B, unchecked returns: Power of Ten Rule 7's manual residual, "confirm a deliberately dropped-and-(void)-cast return was a justified ignore, with a reason".
- Claim: The header says "Returns true on success (ADC block came up)", but the body calls two void SDK functions and unconditionally returns true, so the bool carries no information; main.cpp:242 casts it to (void) with the trailing comment "Stage 16C IVP-142a", which is a ticket reference, not a justification.
- Why: mcu_temp_available() therefore means "init was called", not "the ADC works". That value is load-bearing downstream: health_monitor.cpp:235 uses it to decide whether die-temp health is Absent, and health_monitor.cpp:487 gates the 105 deg C critical flag on it. On a unit where the ADC block did not come up, both gates pass and the only remaining defense is the stuck detector, which needs 60 samples (about a minute at 1 Hz) to assert. Any future caller that writes an if (!mcu_temp_init()) branch gets dead code.
- Confidence: high
- Direction: Either make the return honest -- take one sample at init and range-check it so available() means verified -- or change the signature to void and say so in the header; if the drop at the call site stays, replace the ticket reference with the reason the result cannot fail.
- Verdict: CONFIRMED -- mcu_temp_init (:48-56) calls two void SDK functions and returns true unconditionally against mcu_temp.h:28's "Returns true on success (ADC block came up)", main.cpp:242 drops it with a ticket reference, and health_monitor.cpp:235 and :487 both gate real decisions on mcu_temp_available().

### CW-B12-06 -- ws2812_set_sweep_bar documentation contradicts itself on cadence inside one comment block
- Site: src/drivers/ws2812_status.h:126-133 (restated at src/drivers/ws2812_status.cpp:200-202)
- Lens: Comments & documentation quality -- CCG NL.2, JSF AV 134 (assumptions/limitations belong in the preamble, and must be consistent).
- Claim: The first paragraph promises the pixel "walks back and forth across the strip at a fixed cadence, independent of call frequency"; the third paragraph says "caller should throttle to ~20 Hz (every 50 ms) or similar for smooth motion", and the body advances exactly one step per call.
- Why: The two halves of the same contract assign the cadence responsibility to opposite sides. The lead sentence is the one a caller reads first, and it says the driver owns cadence -- a second caller written against it and driven from the 100 Hz AO tick produces a strobe, not a sweep. The one existing caller (src/active_objects/ao_radio.cpp:610-617) divides its tick by 5 by hand, so the current behavior is correct because of the caller's arithmetic, not because of anything the driver enforces.
- Confidence: high
- Direction: Delete the "at a fixed cadence, independent of call frequency" clause and state once that the function advances one step per call and the caller sets the rate; if driver-owned cadence was actually intended, gate the advance on elapsed time instead.
- Verdict: CONFIRMED -- ws2812_status.h:126-133 promises a fixed cadence independent of call frequency and then tells the caller to throttle to ~20 Hz, the body advances one step per call (:218-222), and the current cadence comes from ao_radio.cpp:612-614's hand-rolled divide-by-5.

### CW-B12-07 -- mcu_temp stuck-detector state is written on Core 1 and read on Core 0 with no barrier and no stated owner, while its sibling datum uses the seqlock
- Site: src/drivers/mcu_temp.cpp:44-46 (mutated at src/drivers/mcu_temp.cpp:74-81); written from src/core1/sensor_core1.cpp:428, read from src/safety/health_monitor.cpp:243 and src/diag/diag_stats.cpp:234-235
- Lens: Declaration scope & object lifetime -- CCG CP.2 ("an object's effective lifetime spans two cores, and one context reads while another writes"); with JSF AV 134 for the missing preamble assumption.
- Claim: g_mcuTempInitialized, g_lastRawSample and g_consecIdentical are plain non-atomic file-scope statics; in the vehicle build the mutating call runs on Core 1 while the 10 Hz health-monitor AO and the CLI diag path read them on Core 0, and the header's only ownership statement is the ADC-consumer rule, which is about channel serialization, not about who owns this state.
- Why: The asymmetry inside one decision is the concrete tell -- evaluate_mcu_temp (health_monitor.cpp:234-247) takes mcu_die_temp_c and mcu_temp_read_count from the sensor seqlock snapshot, the project's sanctioned cross-core channel, and then reads mcu_temp_is_stuck() straight out of the driver's unsynchronized statics for the same verdict. docs/MULTICORE_RULES.md states plainly that plain cross-core sharing is broken and to use atomics or the SDK primitives; nothing here records why this one is exempt. Practical corruption risk is low (single writer, word-aligned, monotone counter compared against a threshold), so the defect I am filing is the unstated ownership rule on a contract surface that otherwise documents its assumptions carefully, not a claim of observed tearing.
- Confidence: medium
- Direction: Add the owner/reader rule to the header preamble (Core 1 writes on the vehicle, Core 0 on the station, Core 0 reads the stuck flag), and either make the counter atomic or publish the stuck flag through the seqlock next to mcu_temp_read_count so one health verdict draws all its inputs from one channel.
- Verdict: CONFIRMED -- g_mcuTempInitialized/g_lastRawSample/g_consecIdentical (mcu_temp.cpp:44-46) are plain statics mutated at :74-81 from sensor_core1.cpp:428 on Core 1 and read from health_monitor.cpp:243 and diag_stats.cpp:234-235 on Core 0, and health_monitor.cpp:239 vs :243 really does draw one verdict from the seqlock and the other from the unsynchronized statics; the finding files the unstated ownership rule, not observed tearing.

### CW-B12-08 -- update_blink lacks the zero-period guard its sibling update_alternate carries, and two public timing setters accept 0 unvalidated
- Site: src/drivers/ws2812_status.cpp:381-382 (guard present at src/drivers/ws2812_status.cpp:406-407); setters at src/drivers/ws2812_status.cpp:355-361
- Lens: Spine block B, happy-path-only input handling -- Power of Ten Rule 7, second half ("parameter validity must be checked inside each function"); CCG F.3 ("how would you know if all possible alternatives have been correctly handled?").
- Claim: update_blink computes period = on_ms + off_ms and then elapsed % period with no zero check, while update_alternate guards the identical shape with an explicit early return on period == 0, and ws2812_set_mode_alternate:348 validates its own half-period argument -- the same author guarded two of the three sites and not the third.
- Why: ws2812_set_blink_timing(0, 0) followed by mode BLINK divides by zero (undefined behavior; on the Cortex-M33 UDIV it silently yields 0 unless DIV_0_TRP is set, so the failure is a wrong pattern rather than a fault), and ws2812_set_breathe_period(0) drives update_breathe:373 to inf, then sinf to NaN, then a float-to-uint8_t cast of NaN in apply_brightness. This is latent, not live: neither setter has any caller in src/, so only the defaults (500/500 and 2000) are ever in play, and clang-analyzer cannot reach it because no in-translation-unit caller supplies the zero. I am unsure whether any enabled check catches this across translation units, so per the guide I am keeping the finding and saying so rather than dropping it.
- Confidence: medium
- Direction: Apply the pattern already used at line 348 -- substitute the default when the argument is 0 -- in ws2812_set_blink_timing and ws2812_set_breathe_period, or add the same early return update_alternate has to update_blink and update_breathe.
- Verdict: CONFIRMED -- update_blink (:381-382) computes elapsed % (on_ms + off_ms) with no zero guard while update_alternate (:406-407) early-returns on period == 0 and ws2812_set_mode_alternate (:348) substitutes a default, and the two timing setters (:355-361) validate nothing; the finding already states both setters are caller-less so the div-by-zero is latent and clang-analyzer-core.DivideZero cannot reach it.

---

## Tier 2 — Domain logic & infrastructure


### B13 -- fusion: eskf core + ud_factor + state

#### Coverage
- src/fusion/eskf.cpp -- PARTIAL -- Read whole (1826 lines); spine run on every function; comment/assertion/scope lenses applied; five findings sited here (stale Joseph/Mat15 preambles, 3-axis mag Jacobian, ZUPT duplication, predict_dense divergence, healthy() covariance arm).
- src/fusion/eskf.h -- PARTIAL -- Read whole (625 lines) as the declared contract for eskf.cpp; constants carry sourced citations and units throughout; findings on the sync_dense_covariance precondition, the Joseph claims, and the healthy() "P bounds" claim.
- src/fusion/eskf_state.h -- PARTIAL -- Read whole (53 lines); evaluated as a Kind-D contract surface (shared index vocabulary) per the helper; layout is internally consistent and matches its block comment, but one named index is defined and never used while its consumers re-derive it.
- src/fusion/ud_factor.cpp -- PARTIAL -- Read whole (177 lines); spine run on all five functions; findings on the file-header block and on the module-static workspace / unchecked index.
- src/fusion/ud_factor.h -- PARTIAL -- Read whole (55 lines) as a Kind-C contract surface (prototypes plus prose); the prose documents parameter meanings but not the functions' assumptions or limitations, and the state dimension is hardcoded with nothing tying it to eskf_state.h.

#### Findings

### CW-B13-01 -- ud_factor.cpp file header documents code that is not in the file
- Site: src/fusion/ud_factor.cpp:3-12
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation must describe code that actually runs) and CCG NL.2 (if the comment and the code disagree, both are likely to be wrong); JSF AV 131.
- Claim: The file-level block states the file contains a "Thornton WMGS temporal update" with "Three Thornton precision variants (f32, mixed f32/f64, f64 accum)" and that "All hot functions" are placed in the .time_critical SRAM section, none of which is true of the file as it stands.
- Why: There is no Thornton function anywhere in src/ -- the only surviving occurrences of the word are these two comment lines plus one back-reference in ud_factor.h:13. And only bierman_scalar_update carries the section attribute (ud_factor.cpp:156); ud_to_dense (:32) and ud_factorize (:50) sit in flash, yet both run on the hot path -- ensure_dense() calls ud_to_dense on every predict() tick once the covariance has been UD-factored, and ensure_ud() calls ud_factorize on the first measurement update after each predict (eskf.cpp:545, :586). A reader trusting this header believes the LL Entry 30 SRAM rationale (2KB XIP cache) has been applied to the whole hot path when it has been applied to one function, and goes looking for three precision variants that were deleted.
- Confidence: high
- Direction: Rewrite the block to describe what the file is now -- ud_to_dense, ud_factorize, Bierman scalar update -- and state SRAM placement per function rather than as a blanket claim. If ud_factorize / ud_to_dense are meant to be in SRAM, that is a separate build change, not a comment change.
- Verdict: CONFIRMED -- ud_factor.cpp:7-11 does claim Thornton WMGS, three precision variants and blanket .time_critical placement; no Thornton symbol exists anywhere in src/ and only bierman_scalar_update carries the section attribute (:156) while ud_to_dense (:32) and ud_factorize (:50) run on the hot path via ensure_dense (eskf.cpp:545) and ensure_ud (eskf.cpp:586).

### CW-B13-02 -- Joseph-form and Mat15 claims survive the removal of the Joseph path
- Site: src/fusion/eskf.cpp:715-716 (also :819; src/fusion/eskf.h:365 and :380)
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") and CERT MSC12-C; JSF AV 131.
- Claim: The update_baro and update_mag_heading preambles still assert "Joseph form P update for numerical stability" and "Static locals for Mat15 temporaries (LL Entry 1, ~2.9KB BSS)", while the same file records at :537 that Joseph was removed and Bierman is the sole measurement path, and neither function declares any static temporary.
- Why: The contradiction sits inside one work product: eskf.h:31 says "Historical Joseph scalar path removed 2026-07", eskf.cpp:537 says "sole measurement path; Joseph removed", and eskf.h:365/:380 -- the public contract a caller reads -- still cite Joseph form. Mat15 is worse than stale: the filter is 24-state (eskf_state.h:35) and Mat15 is a 15x15 type (math/mat.h:178), so the comment describes the pre-24-state implementation. A maintainer debugging covariance conditioning will hunt for a Joseph update and a ~2.9KB static block that do not exist, and will not be pointed at the machinery that actually does the conditioning (the D floor at ud_factor.cpp:128 and the negative-diagonal floor at eskf.cpp:557-567).
- Confidence: high
- Direction: Delete the Joseph and Mat15 sentences at all four sites; where the numerical-stability rationale is still wanted, point at the Bierman UD path and its floors instead of re-describing a removed algorithm.
- Verdict: CONFIRMED -- eskf.cpp:715-716 and :819 still assert Joseph form and Mat15 statics while eskf.h:31 and eskf.cpp:537 record Joseph's removal, Mat15 is Mat<15,15> (math/mat.h:178) in a 24-state filter, neither function declares a static temporary, and the public contract at eskf.h:365/:380 repeats the Joseph citation.

### CW-B13-03 -- documented sync_dense_covariance() precondition is honored only by tests
- Site: src/fusion/eskf.h:443-447
- Lens: Comments & documentation quality / contract surface -- JSF AV 134 (assumptions and limitations documented in the preamble), plus the spine block-B passes-tests-yet-wrong type.
- Claim: The header states sync_dense_covariance() is "Required before reading P(i,j) after measurement updates (lazy dense)" and to "call at inspect sites", but no firmware call site calls it -- every caller in the tree is under test/.
- Why: The dense P is left stale by design once a measurement update factorizes into UD (eskf.cpp:652-660 update the UD factors only; nothing writes P.data). The firmware inspect sites read P(i,j) straight through the public member -- ao_logger.cpp:123-135, rc_os_commands.cpp:221/267/904, eskf_runner.cpp:484 -- so the covariance reaching the flight log, the CLI and the ESKF confidence report is the post-predict, pre-measurement matrix, systematically missing the shrink contributed by the most recent measurement epoch. Either the stated precondition is real and the flight readers are wrong, or the precondition is over-stated and the header is wrong; a green host suite cannot distinguish the two because the tests are exactly the callers that comply. Nothing prevents the stale read either: P is a public member, so the invariant the method exists to protect is unenforceable at the type.
- Confidence: high on the claim-versus-truth mismatch; which side is the defect is an owner call
- Direction: Decide which side is authoritative. Either route the firmware inspect sites through an accessor that syncs (or through a snapshot the runner refreshes once per tick), or weaken the header to say the dense P is a one-epoch-stale view and name who may read it raw.
- Verdict: CONFIRMED -- eskf.h:443-447 states the precondition, and a tree-wide grep shows every sync_dense_covariance() caller is under test/ while the firmware inspect sites (ao_logger.cpp:123-135, eskf_runner.cpp:484) read the public P member directly.

### CW-B13-04 -- 3-axis mag fusion applies an identity Jacobian to states whose innovation came from the rotated model
- Site: src/fusion/eskf.cpp:924-947 (innovation built at :970-977; comment at :919; flight call site eskf_runner.cpp:332)
- Lens: The spine, block B -- spec-noncompliance / passes-tests-yet-wrong; with CCG NL.2 on the accompanying comment.
- Claim: fuse_mag_axes computes each axis innovation from the full nonlinear model z_pred = R(q)*earth_mag + body_mag_bias, then fuses it with H set to a single +1 entry at earth_mag[axis], which is the correct Jacobian only when the body-to-NED rotation is the identity.
- Why: dh_i/d(earth_mag) is row i of R(q), not the unit vector e_i, and dh_i/d(delta_theta) is not zero at all -- so at any attitude away from NED alignment the earth-field correction lands on the wrong NED axes, and the attitude coupling that is the point of 3-axis mag fusion is never applied. A vertical rocket on the pad is roughly 90 degrees from level, so the nominal pre-flight attitude is near the worst case. The file's own comment at :919 says the earth_mag updates are "rotated to body", and eskf.cpp:811-812 spells out the correct H ("with H at attitude(0-2), earth_mag(15-17), body_mag_bias(18-20)") -- the code does neither. This is a live path: eskf_runner.cpp:332 calls update_mag_3axis whenever g_mag3dEnabled, and eskf_runner.cpp:305 clears the mag inhibit. The host tests (test/test_eskf_mag_3axis.cpp) exercise convergence and magnitude gating, not Jacobian correctness under rotation.
- Confidence: medium
- Direction: Either implement the rotated H -- which the single-nonzero-entry signature of bierman_scalar_update cannot express as-is, so it needs a vector-H Bierman variant or a per-axis pre-rotation -- or, if the identity-H form is a deliberate small-rotation approximation, state that and its validity envelope in the preamble and fix the :919 comment that currently claims the opposite.
- Verdict: CONFIRMED -- fuse_mag_axes (:924-936) fuses body-frame innovations built from R(q)*earth_mag + body_mag_bias (:970-977) against h_idx = kIdxEarthMag+axis with h_value 1.0, i.e. an identity Jacobian valid only at R(q)=I, on a live path (eskf_runner.cpp:330-332) that no test exercises for Jacobian correctness under rotation.

### CW-B13-05 -- ud_factor.h preamble gives parameter meanings but none of the function's assumptions or limitations
- Site: src/fusion/ud_factor.h:39-51 (implementation at src/fusion/ud_factor.cpp:96-111)
- Lens: Assertions -- JPL-C Rule 16 and P10 Rule 7 second clause (parameter validity checked inside each function); Comments -- JSF AV 134 (assumptions/limitations in the preamble).
- Claim: bierman_scalar_update is a public API in namespace rc whose preamble documents what each argument means but never states that hIdx must lie in [0,24) or that the function is non-reentrant, and the body checks neither.
- Why: Two limitations are invisible to anyone reading the header. First, bierman_compute_fg indexes ud.U[h_idx][i] (ud_factor.cpp:108) with no bound check, so an out-of-range hIdx is an out-of-bounds read on a no-MMU Cortex-M33 -- and the sibling routine ESKF::scalar_innovation_s (eskf.cpp:682) does range-check the same index and return early, so the hazard was recognised on one path and not the other; ESKF::bierman_kalman_update (eskf.cpp:650) passes h_idx straight through unchecked. Second, the working set is four module-static arrays (ud_factor.cpp:97-100), so two interleaved calls corrupt each other; eskf.h:572-573 documents exactly that constraint on the ESKF wrapper ("SAFETY: single-threaded, Core 0 only, never called from ISR") while the module that actually owns the shared state says nothing. Today every caller passes a compile-time constant or a loop-bounded index, so there is no live defect -- the contract is simply checkable only by reading the .cpp. Note this file has no runtime assertions at all.
- Confidence: medium-high
- Direction: Add both assumptions to the ud_factor.h preamble (valid hIdx range; single-threaded, non-reentrant, not ISR-safe) and add the entry-point index assertion inside bierman_scalar_update so an out-of-contract call trips the project fault path instead of reading past the row.
- Verdict: CONFIRMED as a contract-surface defect -- ud_factor.h:39-51 documents argument meanings only, the body indexes ud.U[h_idx][i] unchecked (ud_factor.cpp:108) and works out of four module statics (:97-100), and the sibling ESKF::scalar_innovation_s range-checks the same index (eskf.cpp:682); the finding itself states there is no live defect today, so it stands on the documentation/assertion gap alone.

### CW-B13-06 -- the two update_zupt overloads duplicate the whole scalar-update loop
- Site: src/fusion/eskf.cpp:1053-1078 and src/fusion/eskf.cpp:1101-1122
- Lens: The spine, block A -- CCG ES.3 (do not repeat yourself) and CCG F.1 (package meaningful operations as named functions); Fowler's Duplicated Code smell.
- Claim: The on-pad overload re-emits the non-pad overload's three-axis scalar-update loop verbatim -- component array, index, innovation, scalar_innovation_s, degenerate-S guard, NIS tracking, gate, Bierman call, last_zupt_nis_, accept counter -- differing only in which R constant is passed.
- Why: This is the same knowledge in two places, and it is the conceptual duplicate a clone check does not see (bugprone-branch-clone works within a function, not across overloads). Any future change to ZUPT gating -- the degenerate-S threshold, the gate multiplier, the NIS bookkeeping, the accept counter -- has to be made twice and will drift silently otherwise. The divergent reject accounting is NOT evidence of drift and is withdrawn from this finding: the non-pad path increments zupt_total_rejects_ on its stationarity check (:1047), and the on-pad overload deliberately has no stationarity check to reject, which its own preamble states at :1082-1084.
- Confidence: high
- Direction: Extract the loop into one private helper parameterised by R (for example fuse_zupt_axes(float r)) so each overload reduces to its own admission check plus that call; the extraction also forces an explicit decision on the divergent reject counting.
- Verdict: RESHAPED -- the two loops (:1057-1074 and :1104-1119) are verbatim duplicates apart from the R constant, but the divergent reject accounting is not accidental drift: the on-pad overload deliberately has no stationarity check to reject (:1082-1084), so that half of the rationale is withdrawn.

### CW-B13-07 -- predict_dense is documented as equivalent to predict() but omits two of its steps
- Site: src/fusion/eskf.cpp:380-399 (claim at :378 and src/fusion/eskf.h:357-359)
- Lens: Comments & documentation quality -- CCG NL.2; with the spine block-B passes-tests-yet-wrong type.
- Claim: predict_dense is described as producing the "Same result as predict()" and as the verification reference for the production path, but it neither calls ensure_dense() before overwriting P nor applies the phase-Q delta that predict() applies.
- Why: Two silent divergences. If the covariance is UD-factored -- which it is after any measurement update (eskf.cpp:652) -- predict_dense writes into the stale dense P without reconstructing it, so it propagates a matrix that is not the filter's current covariance; predict() guards exactly this at :355. And predict() adds the phase-aware Q delta at :367-369, which predict_dense never does, so once a PhaseQRTable is installed the two paths cannot agree by construction. The verification tests (test/test_eskf_propagation.cpp:275-292) compare two freshly-initialised filters driven only by predicts with no phase table -- precisely the configuration in which neither omission can show up -- so a green Test 8 does not establish that the reference validates what flies.
- Confidence: medium
- Direction: Either bring predict_dense to parity (ensure_dense plus the phase-Q delta) so the equivalence claim holds, or narrow the comment to the exact configuration under which the two paths are comparable and have the test assert that configuration.
- Verdict: CONFIRMED -- predict_dense (:380-399) calls neither ensure_dense() nor apply_phase_q_delta() while predict() does both (:355, :367-369), the equivalence is asserted at :378 and eskf.h:357-359, and Test 8 (test/test_eskf_propagation.cpp:278-292) drives two fresh filters with predicts only and no phase table.

### CW-B13-08 -- healthy() does not check the "P bounds" its contract claims, and its UD branch is effectively unfalsifiable
- Site: src/fusion/eskf.h:436-441 with src/fusion/eskf.cpp:1496-1541
- Lens: Assertions -- the meaningful-versus-vacuous half of P10 Rule 5 / JPL-C Rule 16 (a check must be able to detect an anomalous condition); Comments -- CCG NL.2 on the header claim.
- Claim: The public contract at eskf.h:436 says healthy() performs "NaN detection, P bounds, quaternion norm", but covariance_diagonals_healthy() tests only finiteness and strict positivity of the enabled diagonals -- there is no upper-bound test anywhere in it -- and the only accurate description of what the function checks lives in the .cpp preamble at eskf.cpp:1465-1476, which a caller reading the header never sees.
- Why: Upper-bound containment is done elsewhere -- clamp_covariance() caps the diagonals and check_p_growth() catches slow growth -- so a reader of eskf.h who takes "P bounds" at face value believes the LL Entry 29 divergence trip the runner acts on (eskf_runner.cpp:199, eskf_runner.h:125) includes a magnitude test that it does not, and will not go looking for the machinery that does. The lower-bound arm is also weaker than it looks in the UD representation: bierman_forward_pass floors every updated D element to kMinDFloat and replaces non-finite values (ud_factor.cpp:128-131, :149-152), and factorize_from_dense repairs non-positive core diagonals before factorizing (eskf.cpp:576-594, :613-628), so for indices 0-14 the D vector inspected at eskf.cpp:1500 is positive and finite in all but the pathological cases those repairs cannot fix -- rarely able to fire, rather than structurally impossible.
- Confidence: medium
- Direction: Correct the header to describe what is actually checked, and add the missing anomaly test where it can still fail -- for example a D-versus-clamp-ceiling comparison in the UD branch, plus a floor-hit indication so a D pinned at kMinDFloat is reported as degeneracy rather than read as healthy.
- Verdict: RESHAPED -- the eskf.h:436 'P bounds' over-claim is real, but the .cpp preamble at eskf.cpp:1465-1476 already describes the checks accurately and upper-bound containment is clamp_covariance()'s job, so the claim is narrowed to the header over-claim plus a UD lower-bound arm that is rarely able to fire rather than structurally unfalsifiable.

### CW-B13-09 -- kIdxYaw is defined once and used nowhere; its two consumers re-derive it under two local names
- Site: src/fusion/eskf_state.h:47 with src/fusion/eskf.cpp:885 and src/fusion/eskf.cpp:996
- Lens: The spine, block A -- CCG ES.3 (each piece of knowledge in one place) and CCG P.1 (express ideas directly in code); contract-surface Kind D (is there one place that defines these names?).
- Claim: eskf_state.h:47 defines kIdxYaw as part of the shared index vocabulary, but it has zero references anywhere in src/ or test/ -- it is dead -- while the two sites that need the yaw index open-code kIdxAttitude + 2 into fresh local constants spelled kHIdx (eskf.cpp:885) and kYawIdx (eskf.cpp:996).
- Why: A named constant with no consumers is dead code the project's own dead-code discipline asks to be dispositioned, and nothing surfaces it -- neither the compiler nor .clang-tidy flags an unused namespace-scope constexpr in a header, so a reader grepping kIdxYaw to locate the yaw consumers finds nothing. The duplication is benign today: both open-coded derivations are the identical expression eskf::kIdxAttitude + 2, so no off-by-one has been introduced and the header's stated purpose has not actually been defeated. What remains is readability and dead vocabulary -- the two different local spellings make the mag-heading update (:885) and the heading reset (:996) read as though they index different quantities when they do not, and a future change to the attitude block ordering must find two open-coded derivations the shared header was supposed to own.
- Confidence: medium
- Direction: Use eskf::kIdxYaw at both sites and drop the local constants; or, if the local names are preferred for readability, delete kIdxYaw so the header stops advertising a vocabulary entry nothing speaks.
- Verdict: RESHAPED -- kIdxYaw genuinely has zero references in src/ or test/ and both consumers open-code kIdxAttitude + 2, but the two derivations are the identical expression so no off-by-one hazard exists; narrowed to a dead-constant plus readability claim.

### CW-B13-10 -- ud_factor hardcodes the state dimension with nothing tying it to eskf::kStateSize
- Site: src/fusion/ud_factor.h:24-27 and src/fusion/ud_factor.cpp:22
- Lens: Assertions -- CCG P.5 (prefer compile-time checking to run-time checking); the statically-knowable-layout-assumption case of JPL-C Rule 16.
- Claim: The dimension 24 appears independently in UD24's array declarations, in the ud_to_dense / ud_factorize / bierman_scalar_update signatures, and again as kN in the .cpp, with no static_assert relating any of them to rc::eskf::kStateSize.
- Why: The coupling is real and crosses modules: eskf.cpp hands P.data (a Mat24 buffer) to float p[24][24] parameters while iterating its own loops to eskf::kStateSize. If kStateSize were changed without touching ud_factor -- and this filter has already been resized once, which is why eskf.cpp:716 still speaks of Mat15 -- the eskf-side loops would run past the ud-side buffers and the mismatch would compile cleanly, because array parameters decay and nothing cross-checks the two constants. The project already uses this guard idiom elsewhere (eskf_runner.h:41 static_asserts the snapshot size), so the missing check reads as an omission rather than a house-style choice.
- Confidence: medium
- Direction: Add a static_assert tying the UD dimension to rc::eskf::kStateSize -- in ud_factor.h if the dependency on eskf_state.h is acceptable, otherwise at the point in eskf.cpp where the two modules meet -- so a future state-count change fails the build instead of running off the end of U and D.
- Verdict: REFUTED -- speculative, and the stated mechanism is wrong: `float p[24][24]` keeps its inner extent in the parameter type (`float (*)[24]`), so a resized matrix would be a compile error rather than a silent overrun, and ESKF declares `Mat24 P` (eskf.h:68) not Mat<kStateSize,kStateSize>, so the cross-module coupling the static_assert would guard does not exist on the path described.

### B14 -- fusion: eskf_runner + brake + phase_qr

#### Coverage
- src/fusion/eskf_brake.cpp -- PARTIAL -- Three <=5-line functions read whole; the saturating counter and BSS-scoped statics are correct, but the file banner's "Cleared by eskf_reenable() — CLI-callable subsystem reset" understates the automatic clearing path (CW-B14-03).
- src/fusion/eskf_runner.cpp -- FAIL -- All 688 lines read; spine A/B/C run on every static and public function; five findings (CW-B14-01, -02, -04, -05, -07).
- src/fusion/eskf_runner.h -- PARTIAL -- Read whole as the declared contract for the .cpp; the snapshot layout contract is exemplary (the 68-byte size claim is backed by a static_assert at :41), but the runaway-brake contract block at :122-131 disagrees with :145-150 (CW-B14-03) and the init precondition is unstated (CW-B14-07).
- src/fusion/phase_qr.h -- PARTIAL -- Read whole as a Kind D/E contract surface per the helper; the Q-delta and R-absolute claims were checked against eskf.cpp and hold, and the ">= 1.0 / > 0.0 enforced by generate_profile.py" claim was checked against the generator and holds, but the phase-count claim is false and unenforced (CW-B14-06).

#### Findings

### CW-B14-01 -- Blocking flash write issued from the 200 Hz fusion tick, without the project's mandatory Core-1 I2C pause/reset
- Site: src/fusion/eskf_runner.cpp:269-277
- Lens: The spine, block C (embedded ADD — "Blocking-in-cooperative-scheduler / ISR timing" and "Peripheral init SEQUENCE / lifecycle"), plus the P10 Rule 7 manual residual (dropped return with no justification).
- Claim: save_wmm_position() calls calibration_save() — which reaches flash_safe_execute() and blocks the caller while halting the other core — from inside eskf_runner_tick(), and omits both the core1_i2c_pause() prologue and the i2c_bus_reset() epilogue that every other flash callsite in the tree performs.
- Why: eskf_runner_tick() runs from qv_idle_bridge() (src/main.cpp:448) under the QV cooperative scheduler. The project's own note at src/active_objects/ao_rcos.cpp:1301-1303 states calibration_save() "blocks ~100-500ms" and that the 320 ms of AO-queue headroom at depth 32 is "tight but sufficient" — that budget was sized for an operator-initiated, ground-only save. Here the same call is autonomous: it fires from eskf_tick_mag (:324) or try_enable_mag_3axis (:292) on the first GPS 3D fix, which is not necessarily on the pad — if the vehicle launches before lock, the block lands in BOOST or COAST and can overrun the AO queues (LL Entry 32, qf_actq id=130). Separately, calibration_storage.cpp performs no I2C protection of its own (cal_hooks.cpp:99-101 states the pause "is now invoked directly by every flash_safe_execute callsite"); ao_rcos.cpp:343/347 and rc_os_commands.cpp:1042/1055 and :1098/1118 all do so, and this callsite does neither — so Core 1's ICM-20948/DPS310 I2C traffic is interrupted mid-transaction with no reset afterwards, the exact corruption of LL Entry 31. Finally the cal_result_t return is discarded at :276 while g_wmmSource was already advanced to kGps at :291/:323, so a failed save is silent and never retried.
- Confidence: high
- Direction: Do not write flash from the fusion tick. Set a "WMM position dirty" flag here and let a ground/IDLE-phase owner (the AO that already owns cal saves) perform the write with the established core1_i2c_pause() / calibration_save() / i2c_bus_reset() sequence, checking the result before advancing g_wmmSource.
- Verdict: CONFIRMED -- save_wmm_position (:270-277) calls calibration_save() (which reaches flash_safe_execute via calibration_storage.cpp:107) from eskf_runner_tick, itself driven by qv_idle_bridge (main.cpp:448), with no core1_i2c_pause() and no i2c_bus_reset() -- exactly what cal_hooks.cpp:99-103 says every flash_safe_execute callsite must do -- and the cal_result_t return is discarded at :276.

### CW-B14-02 -- g_mag3dEnabled latch survives filter re-init, permanently disabling magnetometer fusion while the accessor still reports it active
- Site: src/fusion/eskf_runner.cpp:280-307
- Lens: The spine, block B ("Passes-tests-yet-wrong" — the unexercised state path) and Declaration scope & object lifetime (two-phase-init: set_inhibit_mag(false) is the second phase and is never re-run).
- Claim: g_mag3dEnabled is module state that is never cleared on any filter re-init path, so after a re-init the guard at :281 short-circuits and ESKF::set_inhibit_mag(false) / g_eskf.earth_mag = g_wmmFieldNed (:304-305) are never re-applied.
- Why: Every re-init route funnels through eskf_try_init() to rc::ESKF::init(), which unconditionally sets inhibit_mag_states_ = true and zeroes earth_mag (src/fusion/eskf.cpp:131-138). The re-init routes are real and reachable: CR-1 divergence at :199-202, the P-growth reset at :524-527, and eskf_runner_request_reinit() at :628-632, which the Flight Director fires on every non-startup entry to IDLE (src/flight_director/flight_director.cpp:353-360). After any of those, try_enable_mag_3axis() returns at :281 because the latch is still true, and eskf_tick_mag then takes the g_mag3dEnabled branch at :330 and calls update_mag_3axis(), which returns false immediately on inhibit_mag_states_ (src/fusion/eskf.cpp:960). The heading-only fallback at :333-345 is also skipped because the latch says 3-axis is active. Net: after one operator RESET from ABORT — the routine pre-re-ARM action — the filter has no magnetometer aiding for the rest of the session, and eskf_runner_mag_3d_active() (:646-648) reports true to CLI and telemetry, so the loss is invisible. Nothing in the test suite exercises re-init-then-mag, which is why it passes green.
- Confidence: high
- Direction: Clear g_mag3dEnabled (and decide explicitly whether g_wmmSource / g_wmmFieldNed should persist) wherever g_eskfInitialized is cleared — ideally by routing all three reset sites through one module-state reset helper, so the ESKF's inhibit state and the runner's latch cannot drift apart.
- Verdict: CONFIRMED -- g_mag3dEnabled (:259) is cleared nowhere, ESKF::init unconditionally sets inhibit_mag_states_ = true and zeroes earth_mag (eskf.cpp:131-138), eskf_runner_request_reinit (:628-632) clears only the initialized flags, and the FD fires it on every non-startup IDLE entry (flight_director.cpp:353-361 via ao_flight_director.cpp:249-251), after which :281 short-circuits and update_mag_3axis returns false at eskf.cpp:959 while eskf_runner_mag_3d_active() still reports true.

### CW-B14-03 -- Runaway-restart brake is documented as CLI-cleared but is auto-cleared by an automatic Flight Director transition
- Site: src/fusion/eskf_runner.h:128-130
- Lens: Comments & documentation quality (JSF AV 134 — assumptions and limitations documented in the preamble; CCG NL.2 — "If the comment and the code disagree, both are likely to be wrong").
- Claim: The brake's contract block states the filter "stays disabled until eskf_reenable() is called from CLI", while eskf_runner_request_reinit() calls eskf_reenable() (:631) from a non-CLI, automatic path documented forty lines lower in the same header.
- Why: The same header at :145-150 documents that eskf_runner_request_reinit() is called "from FD state_idle Q_ENTRY when re-entering IDLE from a flight phase (operator RESET from ABORT/LANDED, or pad-abort auto-IDLE timeout)", and that wiring is real (src/active_objects/ao_flight_director.cpp:249-250 into src/flight_director/flight_director.cpp:353-360, fired on any non-startup IDLE entry). The eskf_brake.cpp banner at :8-9 carries the same CLI-only implication. A reader of either preamble concludes the 5-strike cap is a session-scoped backstop requiring operator acknowledgement; it is not. Concretely, with a persistent underlying fault the brake cannot latch: an LL Entry 29 silent-zero IMU diverges the filter five times, the brake disables it, the pad-abort auto-IDLE timeout re-enters IDLE with no operator action, eskf_reenable() zeroes g_eskfFailCount, and the cycle repeats indefinitely — which is the runaway the brake exists to stop. The FD's clearing is deliberate and justified for a different reason (its comment at flight_director.cpp:355-358 says re-init is needed so re-ARM can succeed); the defect is that the two intents are conflated in one call and only one of them is written down.
- Confidence: high
- Direction: Separate "clear the divergence latch so re-ARM can succeed" from "reset the runaway fail counter" — have the FD path re-init the filter without zeroing g_eskfFailCount — and correct both preambles (eskf_runner.h:122-131 and eskf_brake.cpp:6-13) to state every clearing path.
- Verdict: CONFIRMED -- eskf_runner.h:128-130 says the filter stays disabled until eskf_reenable() is called from CLI, while eskf_runner_request_reinit() calls eskf_reenable() (eskf_runner.cpp:631) from the automatic FD path the same header documents at :145-150, and eskf_reenable() zeroes g_eskfFailCount (eskf_brake.cpp:29-32).

### CW-B14-04 -- eskf_run_predict() aborts by returning from a void function; the fusion cycle cannot see it and publishes anyway
- Site: src/fusion/eskf_runner.cpp:184-203
- Lens: The spine, block B ("Unchecked returns / happy-path-only error handling" — walk every exit path; the [[nodiscard]] gate cannot reach a void function) and Control-flow discipline.
- Claim: eskf_run_predict() has two abort paths — dt outside [kEskfMinDtUs, kEskfMaxDtUs] (:185-187) and !g_eskf.healthy() (:199-203) — both signalled only by returning from a void function, so eskf_runner_fusion_cycle() (:516-557) continues into the full measurement and publish sequence regardless.
- Why: On the divergence path the function has already set g_eskfInitialized = false and charged the runaway brake; the filter is, by its own health check, invalid and scheduled for teardown. The caller then runs eskf_tick_baro, eskf_tick_mag, eskf_tick_zupt, eskf_tick_gps and eskf_tick_mahony (:530-534) against that state, increments g_eskfEpoch (:548) and publishes SIG_SENSOR_DATA (:551-556). The downstream AOs named in the header preamble (Logger, Telemetry, LED) therefore consume and persist one full cycle of fusion output derived from a state the filter itself rejected — and everything computed in that cycle is discarded on the next tick when eskf_try_init() runs. The dt-out-of-range path is the same shape with a milder effect: measurement updates are applied to a state that was not time-propagated at all. Reading eskf_runner_fusion_cycle() alone, there is no way to tell that eskf_run_predict can fail — nothing in its name or signature says so.
- Confidence: high
- Direction: Give eskf_run_predict() a [[nodiscard]] bool (or an explicit outcome enum distinguishing "skipped, dt out of range" from "diverged") and have eskf_runner_fusion_cycle() return early on the diverged case before any measurement update or publish.
- Verdict: CONFIRMED -- eskf_run_predict is void with silent returns at :186 and :202-203, and eskf_runner_fusion_cycle (:516-557) has an early return for check_p_growth (:524-528) but none for the predict aborts, so the measurement ticks (:530-534), the epoch increment (:548) and the SIG_SENSOR_DATA publish (:551-556) all run on a state the filter just rejected.

### CW-B14-05 -- g_eskf and g_eskfInitialized are read by Core 1 while Core 0 writes them at 200 Hz, with no barrier and no stated tolerance
- Site: src/fusion/eskf_runner.cpp:70-74
- Lens: Control-flow discipline (CCG CP.8 — a plain object provides no atomicity, no ordering, and no cross-core visibility; JSF AV 205 pairing) and Declaration scope & object lifetime (CCG CP.2 — data-race lifetime on shared state).
- Claim: The comment at :71 exports g_eskf cross-core for Core 1's benefit and names the reader, but neither g_eskf (a ~970-byte struct) nor the plain bool g_eskfInitialized carries any synchronization, and the comment states no barrier, no ownership rule, and no tolerance rationale.
- Why: Core 0 writes g_eskf.v inside predict() every 5 ms and flips g_eskfInitialized on the init and divergence paths (:163, :200, :525, :629). Core 1 reads both, unguarded, at src/core1/sensor_core1.cpp:269-270 to compute probably_flying, and that boolean gates whether Core 1 calls gps_uart_reinit() — a peripheral re-initialization. g_eskfInitialized is neither volatile nor std::atomic, so nothing forces Core 1 to observe Core 0's write at all; docs/MULTICORE_RULES.md states this case explicitly ("volatile prevents compiler reordering but does NOT issue ARM hardware memory barriers... BROKEN for cross-core") and prescribes std::atomic, a spinlock, or the FIFO. The concrete failure is Core 1 evaluating a stale or torn snapshot as "not flying" and re-initializing the GPS UART during flight. The hazard is bounded — the gate is a heuristic and the read only occurs after a GPS staleness timeout on the UART transport — but the contract is silent, so no reader can tell whether the staleness is tolerated by design.
- Confidence: high on the defect (the unsynchronized cross-core read-while-write is directly visible); medium on how often it bites.
- Direction: Either publish the two values Core 1 needs through the existing synchronized channel (an atomic velocity-magnitude and initialized pair, or the seqlock), or, if a stale read is genuinely acceptable for this heuristic, say so in the declaration comment with the staleness bound and the reason it is safe.
- Verdict: CONFIRMED -- g_eskf and the plain bool g_eskfInitialized (:73-74) carry no synchronization, Core 1 reads both unguarded at sensor_core1.cpp:269-270 to gate gps_uart_reinit(), and docs/MULTICORE_RULES.md explicitly rules out plain/volatile sharing across cores; the declaration comment states no barrier, ownership rule or staleness tolerance.

### CW-B14-06 -- kPhaseCount is documented as matching FlightPhase::kCount; it does not, and nothing enforces it
- Site: src/fusion/phase_qr.h:48-51
- Lens: Comments & documentation quality (CCG NL.2 — comment and code disagree) and Assertions (CCG P.5 — prefer compile-time checking where the constraint is statically decidable).
- Claim: The comment says "Number of flight phases (matches FlightPhase::kCount in flight_state.h)" over kPhaseCount = 8, but FlightPhase::kCount is 9 (src/flight_director/flight_state.h:48-59, kFault = 8), and no static_assert ties the two together.
- Why: The index list at :49-50 quietly stops at ABORT=7 and omits kFault, so the table is deliberately one entry short — but the prose asserts equality, so a maintainer adding or renumbering a phase will trust a claim that is already false. The live consequence is silent: eskf_runner.cpp:459-469 forwards every phase change to ESKF::notify_phase_change(), which bounds-checks new_phase >= kPhaseCount and returns without doing anything (src/fusion/eskf.cpp:1737-1740). So entering FAULT leaves the filter running on the previous phase's Q/R tuning, with prev_phase_ and current_phase_ bookkeeping unchanged, and no diagnostic anywhere. The same false claim is duplicated in the generator (scripts/generate_profile.py:105, "NUM_PHASES = 8  # matches FlightPhase::kCount"), so the map is wrong in two places. The bounds check does prevent an out-of-range table read, so this is silent degradation, not a memory-safety defect.
- Confidence: high
- Direction: Either state plainly that the table covers phases 0-7 and that FAULT intentionally retains the prior phase's Q/R, or size the table from the enum; in both cases add a static_assert binding kPhaseCount to FlightPhase so the relationship is compiler-enforced rather than asserted in prose.
- Verdict: CONFIRMED -- phase_qr.h:48 claims kPhaseCount matches FlightPhase::kCount, which is 9 (flight_state.h:48-59) against kPhaseCount = 8, with no static_assert; ESKF::notify_phase_change silently returns for kFault (eskf.cpp:1737-1740) and the same false claim is repeated at scripts/generate_profile.py:105.

### CW-B14-07 -- The mission-profile pointer is null-guarded in one place and dereferenced unguarded in two others, with no stated precondition
- Site: src/fusion/eskf_runner.cpp:168 and 295, 338-339
- Lens: Assertions (JPL-C Rule 16 / P10 Rule 7 — parameter validity checked inside each function; the missing load-bearing entry precondition) and Comments & documentation quality (JSF AV 134 — document the function's assumptions).
- Claim: eskf_try_init() guards on g_profile != nullptr before use, while try_enable_mag_3axis() (:295-297) and eskf_tick_mag() (:338-339) dereference g_profile unconditionally; eskf_runner_init() (:598-602) neither asserts nor documents that profile must be non-null.
- Why: The two treatments cannot both be right. If nullptr is a legal argument — which is what the guard at :168 tells a reader — then g_profile->has_default_location at :295 and g_profile->default_lat_deg at :338 are null dereferences on the first mag sample after cal. If nullptr is illegal, the guard at :168 is dead defensive code and the real precondition is written nowhere, so nothing stops a future second caller (a host test, the station role, a HAB profile variant) from passing null. Today the single call site passes &rc::kDefaultRocketProfile (src/main.cpp:400) inside init_application(), which runs before QF_run() (src/main.cpp:532-543), so the null path is unreachable in the shipped build — this is a contract defect, not a live crash.
- Confidence: medium
- Direction: Pick one contract. State "profile must be non-null and outlive the runner" in the eskf_runner_init preamble in eskf_runner.h, assert it at :600, and drop the inconsistent guard at :168 — or keep null legal and add the missing guards at both dereference sites.
- Verdict: CONFIRMED -- eskf_try_init guards g_profile at :168 while try_enable_mag_3axis (:295) and eskf_tick_mag (:338-339) dereference it unguarded, and eskf_runner_init (:598-602) neither asserts nor documents the precondition; the finding correctly states the single caller (main.cpp:400) makes this a contract defect rather than a live crash.

### B15 -- fusion: confidence_gate + innovation_monitor

#### Coverage
- src/fusion/confidence_gate.h -- PARTIAL -- Contract surface read whole; the threshold block, struct field comments and hysteresis claims all match the body, but the input contract never states what a caller passes when the AHRS cross-check is unavailable, and the fail-open initial state carries no rationale (CW-B15-01, CW-B15-04).
- src/fusion/confidence_gate.cpp -- PARTIAL -- Both functions read whole and spine-walked; the hysteresis machine and time-tracking behaviour were verified line-by-line against the header's claims, but the 55-line safety evaluator carries zero entry checks on its two load-bearing input assumptions (CW-B15-02, CW-B15-04).
- src/fusion/innovation_monitor.h -- PARTIAL -- Preamble claims verified against the code that implements them (the q_scale min/cap rule matches the body, and the "caller freezes adaptation during phase transition ramps" claim holds at src/fusion/eskf.cpp:1785); the push declaration omits the silent-rejection limitation (CW-B15-03).
- src/fusion/innovation_monitor.cpp -- PARTIAL -- All three functions read whole; window/sum/alpha bookkeeping is self-consistent and the cap logic matches the header, but the anomalous-input path is silent and undocumented (CW-B15-03). The lowercase float suffix at line 21 is deliberately not reported -- it is mechanically gated (hicpp-uppercase-literal-suffix and readability-uppercase-literal-suffix are both enabled in .clang-tidy).

#### Findings

### CW-B15-01 -- ConfidenceInput cannot express "AHRS cross-check unavailable", so an absent cross-check reads as a perfect one
- Site: src/fusion/confidence_gate.h:28 (claim at :9-10); caller src/fusion/eskf_runner.cpp:474-476
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions/limitations documented in the function preamble); CCG NL.2 (comment and code disagree)
- Claim: The file preamble states the gate evaluates an ESKF-vs-Mahony cross-check, but ConfidenceInput carries no validity flag for mahony_div_deg and the preamble states no value for "cross-check not available", so the absent case is indistinguishable from perfect agreement.
- Why: With no stated convention the production caller resolves it in the permissive direction -- eskf_runner.cpp:474-476 substitutes 0.0F whenever g_mahonyInitialized is false or g_mahony.healthy() is false. 0.0F is below confidence::kAhrsDivMaxDeg, so that AND-term in confidence_gate.cpp:23 passes unconditionally and the gate can publish confident = true on the strength of a cross-check that never ran. The flag is not advisory: it reaches SafetyLockout via ao_logger.cpp:212 and flight_director.cpp:258, and guard_combinator.cpp:131 uses it to lock or unlock irreversible actions. Named state: Mahony uninitialised or unhealthy on any tick.
- Confidence: high
- Direction: State the precondition in the header preamble (what a caller must supply when the cross-check is unavailable) and give ConfidenceInput an explicit validity flag, so the gate can treat "no cross-check" as not-passing rather than as passing.
- Verdict: CONFIRMED -- ConfidenceInput (:27-34) has no validity flag for mahony_div_deg, the caller substitutes 0.0F whenever Mahony is uninitialised or unhealthy (eskf_runner.cpp:474-476), and 0.0F passes the kAhrsDivMaxDeg term at confidence_gate.cpp:23, so the AHRS-agreement condition IVP-84 requires is satisfied by a cross-check that never ran.

### CW-B15-02 -- confidence_gate_evaluate has two unstated, unchecked preconditions on now_ms
- Site: src/fusion/confidence_gate.cpp:17-59 (specifically :33-37 and :48-52); sentinel documented at src/fusion/confidence_gate.h:45-46
- Lens: Assertions -- P10 Rule 5 / JPL-C Rule 16 (a >10-line load-bearing function with no assertion; name the missing precondition), P10 Rule 7 (parameter validity checked inside each function)
- Claim: The 55-line safety evaluator asserts nothing about its inputs, while its arithmetic depends on two assumptions the file never states: that now_ms never decreases between calls, and that now_ms == 0 is not a real timestamp (0 is reserved as the "no period in progress" sentinel for bad_since_ms and good_since_ms).
- Why: Both assumptions are load-bearing and unwritten. (a) If input.now_ms is ever less than cs->good_since_ms, the unsigned subtraction at :36 wraps to roughly 4.29e9, which passes the >= kRecoveryDebounceMs test at :37, so the gate sets confident = true on the very next tick and skips the 2000 ms recovery debounce the header promises at :18-19 -- the fail-dangerous direction, since confident unlocks irreversible actions at guard_combinator.cpp:131 (the mirror case at :51-52 fails safe). On the shipped firmware now_ms is to_ms_since_boot(get_absolute_time()) truncated to uint32_t (eskf_runner.cpp:493), so reaching that state needs the ~49.7-day millisecond wrap and is not a flight-session concern -- the defect is the unstated, unchecked precondition, not a live regression. (b) A frozen clock makes every duration compute to 0, so no transition can ever fire and the gate stays in whatever state confidence_gate_init left it (confident). The host-test build does exactly that -- ci.now_ms = 0 at eskf_runner.cpp:495, under the ROCKETCHIP_HOST_TEST branch of :492-496 -- so the hysteresis machine is inert in every host test that drives the runner, and nothing in the gate reports it.
- Confidence: high that both preconditions are unstated and unchecked; medium on how reachable a clock regression is on the target clock source.
- Direction: State both assumptions in the header preamble and add entry checks (non-null cs; now_ms not earlier than the period start it will be subtracted from). Alternatively remove the ambiguity at its source by carrying explicit bad_active / good_active booleans, so a zero timestamp is no longer overloaded.
- Verdict: RESHAPED -- both preconditions are genuinely unstated and unchecked, but the frozen-clock path is the ROCKETCHIP_HOST_TEST branch at eskf_runner.cpp:492-496 (not a production build) and the clock regression needs the ~49.7-day uint32 millisecond wrap, so the reachability claims are narrowed.

### CW-B15-03 -- innovation_channel_push silently discards anomalous samples; the limitation is undocumented and unobservable
- Site: src/fusion/innovation_monitor.cpp:20-23; declaration comment at src/fusion/innovation_monitor.h:38-40
- Lens: Comments & documentation quality -- JSF AV 134 (document the function's limitations in its preamble); secondary Assertions -- JPL-C Rule 16 (an anomalous condition that should never happen is detected, then dropped rather than surfaced)
- Claim: push rejects non-finite and negative NIS values with a bare return, and neither the declaration comment nor the file preamble says a sample can be dropped or that alpha then holds its previous value.
- Why: The rejection is the right defensive move, but it is invisible. When the innovation covariance S degenerates and a caller hands the channel a NaN or Inf NIS (the push sites are eskf.cpp:767, :902, :1326 and :1384), the window and sum are left untouched and alpha stays frozen at its last healthy mean. That stale alpha is exactly what eskf_runner.cpp:478-482 folds into ci.max_innov_ratio for the confidence gate, so a filter in a degenerate state feeds the gate the last good innovation ratio instead of an out-of-range one -- the channel whose job is to detect inconsistency reports health. The function returns void and the struct carries no drop counter, so no caller and no diagnostic can distinguish "no bad samples" from "every sample rejected".
- Confidence: medium
- Direction: Document the rejection and its consequence (alpha retains its previous value) in the header preamble, and add a per-channel drop counter so a persistent non-finite source is observable rather than silent.
- Verdict: CONFIRMED -- innovation_channel_push rejects non-finite and negative NIS with a bare return (:21-23), the declaration comment at innovation_monitor.h:38-40 and the file preamble say nothing about dropping samples or about alpha holding its previous value, and the struct carries no drop counter, so a persistently degenerate source is indistinguishable from a healthy one at eskf_runner.cpp:478-482.

### CW-B15-04 -- The gate initialises to the trusting state with no recorded rationale, against the file's own stated safety principle
- Site: src/fusion/confidence_gate.cpp:8 (function :7-15); header preamble at src/fusion/confidence_gate.h:15-16, declaration comment at :61
- Lens: Comments & documentation quality -- JSF AV 134 (load-bearing assumption undocumented in the preamble); CCG NL.2 (stated intent and code behaviour disagree)
- Claim: The header states this layer's safety principle as "when uncertain, the safest action is no action", yet confidence_gate_init publishes confident = true before a single input has been evaluated, and nothing in either file says why the permissive initial value is the correct one.
- Why: From confidence_gate_init (called at eskf_runner.cpp:173) until the first sustained failure clears the 500 ms kLossDebounceMs window, cs->confident reads true on zero evaluated evidence and propagates unchanged to SafetyLockout (ao_logger.cpp:212, flight_director.cpp:258), where guard_combinator.cpp:131 reads it as "safe to execute irreversible actions". A reader of these two files cannot determine whether that is a deliberate warm-up allowance backed by a separate interlock (arming), or an oversight -- which is exactly the determination the preamble of a platform-safety layer should let them make without leaving the file.
- Confidence: medium
- Direction: Record the rationale for the initial value in the header preamble, naming the interlock that covers the warm-up window. If the trusting initial state is not deliberate, initialise confident = false and let the existing recovery debounce earn it.
- Verdict: CONFIRMED -- confidence_gate_init sets confident = true (:8) with no recorded rationale, against the header's own stated principle at :15-16, and the value propagates unevaluated to SafetyLockout (flight_director.cpp:258) and guard_combinator.cpp:131; IVP-84/85 specify the conditions and hysteresis but never the initial value, so nothing in the tree supplies the missing rationale.

### B16 -- fusion: mahony + generated tables/codegen (light+exempt)

#### Coverage
src/fusion/eskf_codegen.cpp -- PARTIAL -- Read whole under the CG-1 exemption (size/comment/design lenses N/A): banner, preamble, signature and mirror block read line by line, body verified structurally against its generator (300 P snapshots, 202 CSE intermediates, 300 upper-triangle assignments, 276 mirror assignments, and zero raw P reads on any right-hand side outside the snapshot block, which is the generator's stated in-place-update invariant); the body is faithful to the generator, the first two lines are not.
src/fusion/eskf_codegen.h -- PARTIAL -- Read whole as a contract surface (helper Kind C/E: the codegen_fpft call contract plus the Q_d sync constants that eskf.cpp static_asserts against); declarations and constants are consistent with the generator and with eskf.h, but the file carries a post-generation prepend and a non-UTF-8 byte.
src/fusion/mahony_ahrs.cpp -- FAIL -- Read whole, spine run on all six functions; the shape is clean (single-purpose functions, guard clauses, const locals declared at first use) but two attitude-reference defects and one state-reset gap are recorded below.
src/fusion/mahony_ahrs.h -- FAIL -- Read whole as the declared contract for the .cpp; parameter units and gate constants are well sourced, but the preamble carries a stale ESKF state count, the startup flag's comment does not match the code, and the two real preconditions are unstated.
src/fusion/wmm_tables.cpp -- PARTIAL -- Read whole; per the itinerary this is a generated data table walked light, so the three literal tables were scanned for shape and wrap behaviour rather than value-checked, and interp() plus the three public functions were walked line by line.
src/fusion/wmm_tables.h -- PARTIAL -- Read whole as a contract surface (helper Kind C/E); the three-function API, units and sign conventions are stated, the accepted input domain and the grid's polar limitation are not.

#### Findings

### CW-B16-01 -- generated codegen files carry a post-generation prepend the generator does not emit
- Site: src/fusion/eskf_codegen.cpp:1-3 and src/fusion/eskf_codegen.h:1-3 (compare scripts/generate_fpft.py:307-308 and :397-398)
- Lens: Comments & documentation quality -- JSF AV 131/134 comment-truth and CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); this is also the itinerary's CG-1 "confirm untouched" check and the CODING_STANDARDS "Auto-Generated Code" rule (edit the generator, never the output).
- Claim: Both files begin with two lines (SPDX-License-Identifier and Copyright) that scripts/generate_fpft.py never emits, so the "AUTO-GENERATED ... DO NOT EDIT" banner on line 3 is already untrue and regenerating today would delete the licence header.
- Why: The generator builds each output as a list whose first element is the AUTO-GENERATED banner -- for the .cpp at :307-308 and for the .h at :397-398 -- and writes it with no preamble hook. The committed files put that banner on line 3 instead; from line 3 onward the .h matches the generator's hlines list end to end (guard, blank, declaration comment, four-line declaration, blank, R3 comment, namespace, eight inline constexpr constants, close, blank, endif) and the .cpp's structure matches its emission loops exactly, so the delta is precisely those two lines per file. Running the documented regeneration step silently strips the SPDX identifier and copyright from two flight-path translation units, and a genuine future regeneration arrives as a diff carrying that loss on top of a 1130-line float blob where a reviewer will not see it. Same class as the mission_profile_data.h codegen drift already open on the whiteboard.
- Confidence: high
- Direction: Move the two licence lines into the generator's .cpp and .h preamble lists so regeneration is lossless, rather than re-applying them by hand afterwards. Note while doing it that the sibling generated pair (wmm_tables.cpp/.h) carries no licence header at all, so the licence policy for generated artifacts wants deciding once rather than per file.
- Verdict: CONFIRMED -- scripts/generate_fpft.py builds both outputs with the AUTO-GENERATED banner as the first list element (:307-308 for the .cpp, :396-397 for the .h) and writes them with no preamble hook (:392, :422), while the committed files carry SPDX and Copyright on lines 1-2 ahead of that banner, so regeneration would drop them.

### CW-B16-02 -- generated header contains a CP1252 byte because the generator writes with the locale codec
- Site: src/fusion/eskf_codegen.h:13 (root cause scripts/generate_fpft.py:392 and :422, source text at :408)
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.3 (a comment has to be readable to carry its content); in scope for this batch as the CG-1 "confirm untouched / codegen handled properly" check.
- Claim: The em dash in the "R3: Sync constants" comment is stored as the single byte 0x97 (CP1252), not as UTF-8, because generate_fpft.py opens both output files without encoding='utf-8'.
- Why: A byte dump of line 13 shows octal 227 (0x97) standing alone between "constants" and "baked"; the generator source at :408 contains a genuine U+2014 em dash, and both writes (:392 for the .cpp, :422 for the .h) call open with mode 'w' and newline set, but no encoding argument, so on this Windows host Python encodes it with the locale codec. Two concrete consequences: the header is not valid UTF-8, so any tool that decodes strictly shows a replacement character exactly where the rationale for the sync constants lives; and regenerating under the project's WSL/Linux build path emits three different bytes for the same logical text, producing a spurious diff on a file whose entire value is byte-stable regeneration. The sibling generator generate_wmm_table.py already passes encoding='utf-8' at :541 and :547 -- that fix was applied there and missed here.
- Confidence: high
- Direction: Add the explicit utf-8 encoding to both open calls in generate_fpft.py and re-emit; sequence it with CW-B16-01 so the re-emit does not drop the licence lines.
- Verdict: CONFIRMED -- an octal dump of eskf_codegen.h:13 shows the bare byte 0227 (0x97) where the generator source has a real U+2014 (generate_fpft.py:408), both open() calls omit encoding (:392, :422), and the sibling generate_wmm_table.py passes encoding='utf-8' at :541 and :547.

### CW-B16-03 -- Mahony converges to magnetic north while the ESKF it is checked against is corrected to true north
- Site: src/fusion/mahony_ahrs.cpp:77-88 (compare src/fusion/eskf.cpp:877-882 and src/fusion/wmm_tables.cpp:147-157)
- Lens: The spine, block B (spec-noncompliance / passes-tests-yet-wrong on inputs no test exercises), plus Comments & documentation quality (CCG NL.2) on the comments at mahony_ahrs.cpp:25 and :78-79
- Claim: The Mahony reference vector deliberately removes declination, so the filter locks to magnetic north, while the ESKF's mag update adds declination and locks to true north -- which puts a standing offset equal to the local declination into the only number by which the two estimators are compared.
- Why: compute_mag_error builds the reference b_ned = [sqrt(hx^2+hy^2), 0, hz] at :80-84 and drives the estimate until the measured field's East component is zero; its own comment at :78-79 states this "removes declination dependence". The ESKF does the opposite on both of its paths: the heading-only update adds declination explicitly (eskf.cpp:881, "+ declination_rad", sourced from wmm_get_declination at eskf_runner.cpp:344), and the 3-axis path consumes the WMM NED vector, which carries declination in its horizontal split (wmm_tables.cpp:155-157, fed at eskf_runner.cpp:263). The single consumer of the pair is the confidence gate: eskf_runner.cpp:474-475 computes MahonyAHRS::divergence_rad(g_eskf.q, g_mahony.q) and confidence_gate.cpp:23 requires it below kAhrsDivMaxDeg = 15.0 degrees (confidence_gate.h:53). So at a site with declination D the two estimators disagree by about the magnitude of D in steady state with nothing actually wrong -- roughly 3 to 12 degrees across the continental US per the lat +30 and +40 rows of wmm_tables.cpp:35-36, and 20 to 50 degrees in the lat +70 and +80 rows at :39-40, where the cross-check would deny the gate permanently. Nothing in the host suite sees it: test_mahony.cpp uses a synthetic field with zero declination, so measured and true north coincide there.
- Confidence: medium
- Direction: Put both estimators on one heading reference -- either rotate the Mahony reference vector by the WMM declination before the cross product, or subtract declination when forming the divergence -- and state in mahony_ahrs.h which north divergence_rad is measured against, since today the file cannot be read to find out.
- Verdict: CONFIRMED -- compute_mag_error builds b_ned = [h_xy, 0, hz] and states it removes declination dependence (mahony_ahrs.cpp:78-84), while update_mag_heading adds declination_rad (eskf.cpp:881-882) and the 3-axis path consumes the WMM NED vector whose horizontal split carries it (wmm_tables.cpp:154-156), so the divergence metric the confidence gate tests against kAhrsDivMaxDeg = 15 deg carries a standing offset of the local declination; the cited table rows check out.

### CW-B16-04 -- init() discards the tilt quaternion's own yaw when installing the magnetometer yaw
- Site: src/fusion/mahony_ahrs.cpp:30-35
- Lens: The spine, block A (CCG P.1 / P.3 -- the operation is expressed so that correctness cannot be judged from the code's shape) and block B (passes tests, wrong on inputs the suite never uses); Comments & documentation quality (CCG NL.2) on the rationale at :25 and :31
- Claim: The mag yaw is derived by rotating the field with the full attitude quaternion and is then written in as the absolute yaw, discarding the yaw the gravity-only quaternion already carries, so the initialised yaw is wrong by exactly that amount.
- Why: q at :22 comes from Quat::from_two_vectors(body_down, ned_down), whose rotation axis has no z component (quat.cpp:175 -- the cross product against [0,0,1] is (fn.y, -fn.x, 0)), but whose ZYX yaw is nonzero whenever roll and pitch are both nonzero, since to_euler reads yaw as atan2(2xy, 1-2yy) at quat.cpp:123-125. Worked case: true attitude roll 30 deg, pitch 30 deg, yaw 0 gives body_down = (-0.5, 0.433, 0.75) and q = (0.9354, 0.2315, 0.2673, 0), whose Euler triple is (30, 30, 8.2) degrees. Rotating the field with that q at :30 puts the horizontal component at 8.2 degrees, so :31 yields -8.2 and :34 installs it as the absolute yaw, where the correct value is euler.z - 8.2 = 0. The error is identically zero for a level board and for pure-roll or pure-pitch tilt -- which is exactly what test_mahony.cpp exercises, since it initialises from kAccelLevel -- so the suite cannot see it; it grows with combined tilt and lands directly in the divergence metric of CW-B16-03. The in-tree correct pattern is eskf.cpp:877, which rotates with from_euler(roll, pitch, 0) precisely to separate tilt compensation from heading. Two comments make the intent mismatch visible: :25 claims this matches ESKF init, but ESKF init (eskf.cpp:119-122) derives attitude from gravity alone and never touches the magnetometer; and :31's stated reason, "East-positive heading -> negate y", describes a heading convention, whereas the quantity being negated is a yaw residual, so the comment justifies the line with something the line is not doing.
- Confidence: high
- Direction: Rotate the field with the zero-yaw quaternion the way eskf.cpp:877 does, or add the residual to euler.z instead of replacing it, and correct the two comments to say what the step is for. Add a combined roll-and-pitch init case to test_mahony.cpp so the level-only blind spot closes.
- Verdict: CONFIRMED -- from_two_vectors returns an axis with zero z (quat.cpp:175) but a nonzero ZYX yaw whenever roll and pitch are both nonzero (quat.cpp:123-125), and the worked case reproduces exactly (body_down = (-0.5, 0.433, 0.75), q = (0.9354, 0.2315, 0.2673, 0), Euler yaw 8.2 deg), so :34 installs a residual as an absolute yaw; every init in test/test_mahony.cpp uses kAccelLevel, so the suite cannot see it.

### CW-B16-05 -- init() does not reset the startup-boost flag, and the flag's comment describes a transition the code never performs
- Site: src/fusion/mahony_ahrs.cpp:38-41 and src/fusion/mahony_ahrs.h:39
- Lens: Declaration scope & object lifetime (JSF AV 142/143 -- every member holds a meaningful value at its initialisation point; the manual residual here is re-init completeness on a long-lived file-static object), plus Comments & documentation quality (CCG NL.2)
- Claim: init() clears integral_error, elapsed_s and initialized_ but leaves startup_ended_ set, so a filter re-initialised after any ARM silently runs without the startup gain, and the member's comment claims an elapsed-time transition that nothing in the code carries out.
- Why: startup_ended_ is written in exactly one place, force_end_startup() at mahony_ahrs.h:100, reached from eskf_runner_end_mahony_startup() (eskf_runner.cpp:642-644) on SIG_ARM (ao_flight_director.cpp:317). Both re-init paths -- the health failure at eskf_runner.cpp:451-453 and reset_subsystems_cb at ao_flight_director.cpp:249 via eskf_runner_request_reinit() at eskf_runner.cpp:628-632 -- only clear g_mahonyInitialized and let eskf_runner.cpp:436 call init() again on the same file-static object, so elapsed_s returns to 0 while startup_ended_ stays true and update():102-103 therefore selects the nominal Kp. Suppressing the 10x boost in flight is deliberate, so the in-flight re-init is defensible on its own terms: mahony_ahrs.h:98-99 says force_end_startup exists to "avoid aggressive corrections in flight". The case that rationale does not cover is the Flight Director's return-to-IDLE re-init, which puts the filter back on the pad -- the power-on situation the BetaFlight-derived boost at mahony_ahrs.h:51-53 was added for -- with the boost permanently off for the rest of the session. Separately, the comment at :39 ("True after ARM or elapsed > kStartupDurationS") is false in its second clause -- no code path sets the flag from elapsed_s; the timeout is only ever evaluated inline at :102, so a reader or future caller trusting the flag gets a wrong answer.
- Confidence: high
- Direction: Reset startup_ended_ in init() alongside the other members, and reword the comment to describe what the flag actually is -- an ARM-forced early end of the startup boost -- leaving the timeout described where it is enforced.
- Verdict: RESHAPED -- the reset gap and the false second clause of the mahony_ahrs.h:39 comment are both real, but suppressing the 10x boost in flight is the documented intent of force_end_startup (mahony_ahrs.h:98-99), so the consequence is narrowed to the FD's return-to-IDLE re-init, where the on-pad rationale for the boost applies again.

### CW-B16-06 -- header preamble states the wrong ESKF state count
- Site: src/fusion/mahony_ahrs.h:9
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 (comment and code disagree; correct the disagreement)
- Claim: The preamble says the filter "runs alongside the 15-state ESKF"; the ESKF is 24-state.
- Why: eskf.h:6 states "ESKF: 24-state Error-State Kalman Filter", eskf_codegen.h:8 propagates P[24][24], and eskf.cpp:432 and :533 reason explicitly about all 24 states and about states 15-23. This line is the only place in mahony_ahrs.h that describes the system the file exists to cross-check, so a reader deciding which states the divergence metric actually covers is handed a number that has been stale since the 15-to-24 extension. Low severity on its own, but it is a plain factual disagreement in a file preamble, which the comment lens treats as a correction rather than a keep.
- Confidence: high
- Direction: State the current count, or drop the number entirely and say "the ESKF" so the sentence cannot rot again the next time the state vector grows.
- Verdict: CONFIRMED -- mahony_ahrs.h:9 says 15-state ESKF while eskf.h:6 and eskf_state.h:35 give 24 states; a plain factual disagreement in a file preamble.

### CW-B16-07 -- no assertions in the file, and the two preconditions the bodies rely on are neither asserted nor documented
- Site: src/fusion/mahony_ahrs.cpp:94-130 and :135-146 (declared contract at src/fusion/mahony_ahrs.h:81-93)
- Lens: Assertions -- P10 Rule 5 (density floor, and the meaning half: an assertion must check an anomalous condition that should never happen) and JPL-C Rule 16 (parameter-validity sanity checks at entry); JSF AV 134 for the preamble half
- Claim: The translation unit contains no assertions at all, and the two preconditions the bodies actually depend on -- a small positive dt in update(), and unit-norm quaternions in divergence_rad() -- are enforced only by the current caller and stated nowhere.
- Why: update() is 36 lines of load-bearing flight math that integrates through Quat::from_small_angle (quat.cpp:181-189), a first-order approximation valid only for a small rotation vector, and that scales the Ki accumulation by dt directly at :115; the only thing keeping dt sane is the caller's window check at eskf_runner.cpp:443, which is invisible from the class and would not survive a second call site or a replay harness. divergence_rad() clamps w to the range -1 to 1 at :140-144 with the comment "for numeric safety", but that same clamp is what makes a non-unit quaternion return 0 rad -- the health cross-check reports perfect agreement exactly when its input is malformed, which is the fail-unsafe direction for a value that confidence_gate.cpp:23 tests against a maximum. Neither the header preamble at :81-93 nor the bodies state that the inputs must be unit quaternions, or that dt is expected to be a single IMU tick.
- Confidence: medium
- Direction: Assert both contracts at entry (dt within a named positive range; both quaternion norms near unity in divergence_rad) and record the same limits in the header preamble per JSF 134. If the norm check is judged too costly on the 200 Hz path, state the precondition in the preamble at minimum, so the zero-return case reads as a documented contract violation rather than a silent pass.
- Verdict: CONFIRMED -- mahony_ahrs.cpp contains no assertion of any kind, update() scales the Ki accumulation by dt (:115) and integrates through the small-angle approximation (quat.cpp:181-189) with dt bounded only by the caller (eskf_runner.cpp:443), and divergence_rad's clamp (:139-145) returns 0 rad for a non-unit input -- the fail-unsafe direction for a value tested against a maximum at confidence_gate.cpp:23; neither precondition appears in the header preamble.

### CW-B16-08 -- declination is bilinearly averaged across the 180-degree branch cut, and the accepted input domain is undocumented
- Site: src/fusion/wmm_tables.cpp:98-129 (contract at src/fusion/wmm_tables.h:30-37)
- Lens: The spine, block B (passes-tests-yet-wrong on unexercised inputs), plus Comments & documentation quality / JSF AV 134 (a function's assumptions and limitations belong in its preamble)
- Claim: interp() blends the declination table as if it were an ordinary scalar field, so any query whose surrounding grid cell straddles the 180-degree wrap returns a value that is not merely imprecise but about 180 degrees wrong, and neither the header nor the body states any domain limitation.
- Why: kDeclination holds an angle in degrees and therefore wraps inside the table: at lat -90 the adjacent columns for lon 140 and 150 hold -171.91 and 178.09 (wmm_tables.cpp:23), at lat -80 they hold -174.21 and 170.70 (:24), at lat -70 -152.74 and 173.21 (:25), and the lat +90 row alternates 180.00 and 0.00 (:41). A query at lat -85, lon 145 therefore averages -171.91 with 178.09 and returns roughly +3 degrees where the true declination is near 177. wmm_get_earth_field_ned at :147-157 builds the earth-field NED vector straight from that angle, and eskf_runner.cpp:263 and :344 hand it to the ESKF mag update as the yaw reference, so the failure lands as a reversed heading reference rather than a degraded one. The same function also silently clamps out-of-range latitude at :109-110 and wraps longitude at :112-113 with no statement anywhere that this is the contract, so a caller cannot distinguish a real answer from a clamped one. The project's own launch sites sit far from the affected cells, which is why nothing has surfaced -- but the API carries no such restriction, and the header advertises it as a general lat/lon lookup.
- Confidence: medium
- Direction: Fix in scripts/generate_wmm_table.py rather than in the generated file: interpolate declination through its sine and cosine (or unwrap the four corner values relative to v00 before blending), and emit a short header comment stating the clamp and wrap behaviour plus the polar limitation of a 10-degree grid near the magnetic poles.
- Verdict: CONFIRMED -- interp() (:98-129) blends kDeclination as an ordinary scalar and the cited wrap-straddling cells check out (lat -90 lon 140/150 = -171.91/178.09 at :23; lat -80 = -174.21/170.70 at :24; the lat +90 row alternates 180.00/0.00 at :41), and the accepted domain is stated nowhere; note the clamp and wrap are at :101-105, not the :109-113 given in the Why.

### B17 -- calibration: data + storage

#### Coverage

- src/calibration/calibration_data.h -- FAIL -- Contract surface (Kind E, persisted-record layout) walked whole: layout, status vocabulary, size asserts and the four declared functions; the comments lens fails on a stale sentinel claim (:161) and on a page-fit claim (:139) that does not account for the on-flash headers the storage module prepends.
- src/calibration/calibration_data.cpp -- PARTIAL -- All four functions walked with the spine; bodies are short, guard-claused and single-purpose, but the CRC-region definition is expressed twice as an inline idiom (:105-110, :121-126); the file banner also names itself "calibration_data.c" (:4), a harmless doc slip left unfiled.
- src/calibration/calibration_storage.h -- PARTIAL -- Kind C API/behavioral contract walked claim-by-claim (init / read / write / erase); the prose is accurate as far as it goes but omits the blocking + timeout limitation and promises a success return the implementation cannot fail.
- src/calibration/calibration_storage.cpp -- FAIL -- Every function walked (flash wrappers, sector scan, dual-sector write, public API); the dual-sector power-safety and sequence logic hold up on every exit path I traced, but the file banner states 8MB-only absolute addresses while the code derives them per board, and the single-page write budget is unguarded. Note: kStorageSize / kStorageOffset (:26-27) have zero references tree-wide -- left to the mechanical dead-code inventory rather than filed here.

#### Findings

### CW-B17-01 -- Single-page write budget is unguarded; the guard that exists is stated against the wrong bound
- Site: src/calibration/calibration_storage.cpp:216-243 (buffer at :218, memcpy at :238-239), against src/calibration/calibration_data.h:139 and :166-169
- Lens: The spine, block B (passes-tests-yet-wrong / spec-noncompliance) plus block C (functionally-correct-but-safety-blind hardware code); comments lens NL.2 (comment and code disagree)
- Claim: The only stated page-fit bound for the record -- calibration_data.h:139 ("Total size should be < 256 bytes to fit in a single flash page") and the assert at :166 -- ignores the 24 bytes of sector + entry header that write_to_sector prepends inside the same 256-byte page buffer, so the documented budget overstates the real one (232 bytes) and nothing binds the actual on-flash layout.
- Why: write_to_sector places the record at g_pageBuffer + sizeof(sector_header_t) (16) + sizeof(entry_header_t) (8) and memcpys sizeof(calibration_store_t) there (:238-239), so the record's real budget inside the page is 232 bytes, not the 256 the header states. Today the record is 180 bytes and nothing overflows on any path -- the present-tense defect is that the stated bound is wrong and that no assert in calibration_storage.cpp binds sizeof(sector_header_t) + sizeof(entry_header_t) + sizeof(calibration_store_t) to FLASH_PAGE_SIZE (entry_header_t is not size-asserted at all, unlike sector_header_t at :45). A future record grown past 232 bytes would satisfy both existing asserts and still overrun, but that is a latent hazard rather than a live one.
- Confidence: high
- Direction: Put the budget assert where the layout actually lives -- a static_assert in calibration_storage.cpp binding the three sizes to FLASH_PAGE_SIZE -- and reword the calibration_data.h page-fit comment so it stops implying the record owns the whole page. Size-assert entry_header_t for symmetry with sector_header_t.
- Verdict: RESHAPED -- the 24-byte header offset and the wrong stated bound are exactly as cited, but the overrun needs a hypothetical future struct growth (180 bytes of a real 232-byte budget), so the live defect is the wrong documented bound plus the missing layout assert.

### CW-B17-02 -- The definition of "the CRC region" is written twice as an inline idiom
- Site: src/calibration/calibration_data.cpp:105-110 and :121-126
- Lens: The spine, block A -- CppCoreGuidelines F.1 ("package meaningful operations as carefully named functions"; the guide's own example of the smell is "the same several-line idiom (a CRC step ...) appears twice") and ES.3 (Fowler, Duplicated Code)
- Claim: calibration_validate and calibration_update_crc each open-code the same four-line CRC-region computation (start at &cal->accel, end at cal + sizeof(*cal), length, crc16_ccitt call) plus a duplicated two-line rationale comment, so one piece of knowledge -- which bytes the CRC covers -- is written in two places instead of named once.
- Why: the two copies are byte-for-byte identical today and the module is correct as it stands; the finding is the duplication itself, which CCG F.1 names explicitly (its own example of the smell is a repeated CRC step) and ES.3 / Fowler call Duplicated Code. Because writer and validator must agree exactly, a future edit that moved the region in one copy and not the other would make every record written by that firmware fail validation on the next boot, and nothing mechanical would see it -- the copies are not branches, so bugprone-branch-clone does not apply, and both halves compile and pass any test that round-trips through the same build. No such divergence exists now.
- Confidence: high
- Direction: Extract one small named helper (for example a function returning the covered span, or a crc_of_record(const calibration_store_t*) used by both) so the region is defined once, and keep a single copy of the JSF AV 18 / no-offsetof rationale comment on it.
- Verdict: RESHAPED -- the verbatim duplication is exactly as cited, but the "silently rejects its own writes" consequence requires a hypothetical divergent future edit, so the claim narrows to the duplication itself.

### CW-B17-03 -- calibration_storage_init discards the only signal it has and reports unconditional success
- Site: src/calibration/calibration_storage.cpp:249-257 (return dropped at :254), contract at src/calibration/calibration_storage.h:16-24
- Lens: The spine, block B -- Power of Ten Rule 7 (return values of non-void functions must be checked by each calling function), the manual residual left after the [[nodiscard]] gate; comments lens JSF AV 134 (document a function's assumptions/limitations)
- Claim: find_active_sector() returns whether a valid stored record was recovered, calibration_storage_init drops that bool with no check and no (void) cast, then returns a hardcoded true that the header advertises as "@return true on success".
- Why: after this call a caller cannot distinguish "restored the saved calibration" from "found nothing valid and fell back to calibration_init_defaults" (the :176-182 branch) -- yet that is exactly the state a boot-time consumer would want to report. main.cpp:252 stores the result in g_calStorageInitialized, and that flag is never read anywhere in src/ or include/ (only declared at shared_state.h:52 and defined at shared_state.cpp:28), so the health signal is discarded twice over. The drop is also unmarked: the same file's project convention marks deliberate ignores explicitly (main.cpp:242 uses "(void)rc::mcu_temp_init();"), and find_active_sector is a file-static, so the [[nodiscard]] + -Werror gate cannot see it. A defaults-only boot is not an error, but it is not "success" either, and today nothing can tell them apart.
- Confidence: high
- Direction: Either return find_active_sector()'s result and correct the header to say what true means (valid stored calibration recovered vs. defaults in effect), or keep the current shape and mark the ignore with (void) plus a one-line reason; if the header keeps promising "success", the function needs a path that can return false.
- Verdict: CONFIRMED -- find_active_sector()'s bool is dropped unmarked at :254 and calibration_storage_init returns a hardcoded true against a header that promises "@return true on success"; nothing gates it (bugprone-unused-return-value.CheckedFunctions lists only flash_safe_execute, and the callee is file-static).

### CW-B17-04 -- Storage contract does not state that its calls block, or for how long
- Site: src/calibration/calibration_storage.h:34-43 and :45-49 (mechanism at src/calibration/calibration_storage.cpp:56, :107, :125)
- Lens: Comments lens item 4 -- JSF AV 134 ("assumptions (limitations) made by functions should be documented in the function's preamble"); the spine, block C (blocking-in-cooperative-scheduler)
- Claim: the write and erase preambles document dual-core safety ("Uses flash_safe_execute for dual-core safety") and wear leveling but never state that the call blocks the caller through a 4KB sector erase plus a page program, bounded only by kFlashSafeTimeoutMs = 1000 ms.
- Why: this is the load-bearing limitation on a QP/QV run-to-completion codebase, and it is the one the header omits. A live caller already sits in exactly the wrong context: eskf_runner.cpp:276 (save_wmm_position) calls calibration_save -> calibration_storage_write from qv_idle_bridge (main.cpp:448), so a first GPS 3D fix performs a flash erase inside the idle path with no budget stated anywhere in the API it is calling. One caller has independently discovered the hazard and written it down locally (ao_rcos.cpp:1301, "calibration_save() calls flash_safe_execute() which blocks"), which is the tell that the knowledge belongs on the contract surface instead of being rediscovered per call site -- and LESSONS_LEARNED Entry 32 records what this project already paid for an undocumented blocking driver under this scheduler. calibration_storage_init has the same gap in the other direction: it documents the ordering precondition ("before stdio_init_all() per LL Entry 4/12") but not that it reads flash directly through XIP.
- Confidence: medium
- Direction: Add the limitation to the write/erase preambles -- blocks for an erase plus program, up to the flash_safe_execute timeout, not to be called from a handler with a tighter run-to-completion budget -- and name the timeout constant so callers can size against it.
- Verdict: CONFIRMED -- the write/erase preambles document dual-core safety and wear levelling but never that the call blocks through an erase plus program, and a live caller (eskf_runner.cpp:276 save_wmm_position -> calibration_save, reached from qv_idle_bridge at main.cpp:448) sits on the QV idle path with no budget stated anywhere in the API it calls.

### CW-B17-05 -- Comment still documents the 0-latitude sentinel that CAL_STATUS_WMM_SET replaced
- Site: src/calibration/calibration_data.h:161 (against :35)
- Lens: Comments lens -- CCG NL.2 / JSF AV 131 ("if the comment and the code disagree, both are likely to be wrong") and CERT MSC12-C (a comment describing a mechanism the code no longer uses)
- Claim: wmm_lat_deg is annotated "(0 = not set)" while the flag comment eleven lines earlier states that the explicit CAL_STATUS_WMM_SET flag "replaces 0,0 sentinel", so the header documents two mutually exclusive validity mechanisms for the same field.
- Why: the file contradicts itself and the surviving copy is the wrong one -- the only consumer, eskf_runner.cpp:293-294, gates on (cal->cal_flags & CAL_STATUS_WMM_SET) and never inspects the value, so the documented sentinel is not the rule the system runs on. The concrete hazard is a future reader taking the field comment at face value and writing "if (cal->wmm_lat_deg == 0.0F)": that reintroduces the float-equality sentinel this codebase deliberately removed and misclassifies a real equatorial position (latitude 0 is a valid latitude) as "not set". Deleting the parenthetical loses no information the flag does not already carry.
- Confidence: high
- Direction: Drop "(0 = not set)" from :161 and, if a validity note is wanted on the field, point at CAL_STATUS_WMM_SET as the single validity source.
- Verdict: CONFIRMED -- :161 still reads "(0 = not set)" while :35 states CAL_STATUS_WMM_SET "replaces 0,0 sentinel", and the only consumer (eskf_runner.cpp:293) gates on the flag and never inspects the value.

### CW-B17-06 -- File banner pins absolute flash addresses that the code derives per board
- Site: src/calibration/calibration_storage.cpp:7-9 (against :26-30 and include/rocketchip/flash_layout.h:44-52)
- Lens: Comments lens item 3 -- CCG NL.3 (point to the spec, do not paraphrase it) and NL.2 (comment/code disagreement)
- Claim: the banner states a fixed map ("Flash layout (at end of 8MB flash): Sector A: 0x7FE000 ... Sector B: 0x7FF000") while the code takes its offsets from rc::kFlashCalSectorA/B, which flash_layout.h anchors to PICO_FLASH_SIZE_BYTES -- so the banner is a second copy of a map whose single source is flash_layout.h, and it is wrong for the non-8MB board selections this file compiles under.
- Why: flash_layout.h:47-52 derives both sectors top-down from PICO_FLASH_SIZE_BYTES. board.h selects among four board headers: board_pico2.h is a 4MB part and board_fruit_jam.h (the station role) is 16MB, so on those builds the sectors are not at 0x7FE000/0x7FF000 and anyone debugging a flash dump against this comment reads the wrong addresses. (board_feather_rp2350.h and board_tiny_2350_plus.h are both 8MB parts, so the parenthetical is true of the default flight build -- the banner is not wrong everywhere, only wherever the part is not 8MB.) The top-down anchoring that makes the code portable is precisely what the comment hides.
- Confidence: medium
- Direction: Replace the three address lines with a one-line pointer to include/rocketchip/flash_layout.h (regions anchored top-down from PICO_FLASH_SIZE_BYTES), keeping the dual-sector / sequence-number explanation, which is genuine why and belongs here.
- Verdict: RESHAPED -- the duplicated absolute map is real, but the original named the wrong boards (board_tiny_2350_plus.h IS an 8MB part); the true narrower claim is that the banner is wrong for the 4MB Pico 2 and the 16MB Fruit Jam selections.

### CW-B17-07 -- Per-sensor status fields duplicate cal_flags and are never read
- Site: src/calibration/calibration_data.h:64, :80, :94, :114 (against :154)
- Lens: The spine, block A -- CCG ES.3 (don't repeat yourself; Fowler's Duplicated Code smell, the same knowledge expressed in two places); comments lens JSF AV 131 (doc-comments asserting a meaning the code does not use)
- Claim: the per-sensor accel/gyro/baro/mag .status fields are written at every calibration-completion site and read nowhere in the tree, while the same "which calibrations have been performed" knowledge is also carried in cal_flags -- the only copy any consumer reads.
- Why: every completion site writes both copies (calibration_manager.cpp:258/260, :296/298, :395/397, :616/618, :1020/1022) but a tree-wide search finds no read of any per-sensor .status field: the readers all test cal_flags (for example eskf_runner.cpp:284, :293). Because the fields sit inside the persisted flash record, the unread duplicate costs bytes and a format-version obligation forever, and the header's own doc-comments ("CAL_STATUS_LEVEL or CAL_STATUS_ACCEL_6POS") invite a future reader to treat a field nothing consumes as authoritative. (The per-sensor fields are assigned rather than OR'd, so they do track the latest calibration -- they cannot silently drift from cal_flags the way a supersession argument would suggest.)
- Confidence: medium
- Direction: Decide which copy is authoritative. Either drop the per-sensor status fields at the next version bump (cal_flags is the read path) or, if per-sensor state is wanted for a future consumer, say so in the doc-comments and make the supersession logic maintain both.
- Verdict: RESHAPED -- the fields are indeed written-and-never-read, but the supersession argument was wrong (the per-sensor field is assigned, not OR'd, so it tracks the latest cal by construction); narrowed to unread duplicated state inside a persisted record.

### B18 -- calibration: manager + hooks

#### Coverage
src/calibration/cal_hooks.cpp -- FAIL -- All four callbacks and both file-scope state groups read whole; cal_read_accel() is registered but has no call site anywhere in the tree, and three of five mag-diagnostic statics are write-only.
src/calibration/cal_hooks.h -- PARTIAL -- Thin Kind-C contract surface (four prototypes); every assumption a caller needs -- I2C ownership, the 10 ms block, "false means stale, not failed", raw-not-corrected -- is absent from the header.
src/calibration/calibration_manager.cpp -- FAIL -- All ~40 functions walked; the module's whole state block is plain non-atomic file-scope statics written from both cores, and one helper carries an unchecked count precondition.
src/calibration/calibration_manager.h -- FAIL -- Full API surface walked as the module's contract; it names the cross-core protocol in one doc-comment but never states ownership, a barrier, or the mutability of the store its accessor hands out.

#### Findings

### CW-B18-01 -- Calibration collection state is written from both cores with no barrier and no stated owner
- Site: src/calibration/calibration_manager.cpp:92-128 (state block; dual writes at 247-248, 348-354 vs 205, 309, 376, 493, 530)
- Lens: Declaration scope & object lifetime -- CCG CP.2 "Avoid data races" (lens judging table: object's lifetime spans two cores with unsynchronized read-while-write = FAIL); JSF AV 134 for the missing preamble assumption.
- Claim: g_calState, g_sampleAcc, the g_6posAsync* trio and g_calibration are plain non-atomic, non-volatile file-scope statics that Core 1 and Core 0 both write, with no atomic, spinlock, or documented hand-off anywhere in the module.
- Why: Truth checked against the callers. Core 1's sensor loop calls calibration_manager_get_state() and calibration_feed_gyro/accel/baro() every iteration (src/core1/sensor_core1.cpp:173-181, 206-209); on the completion tick Core 1 itself writes g_calState = CAL_STATE_COMPLETE/FAILED and writes g_calibration.gyro.bias plus the CRC (calibration_manager.cpp:247-264, 348-354). Core 0's AO drives the same variables from ao_rcos.cpp:489 (calibration_start_6pos_position writes g_calState), :522 (calibration_6pos_position_done reads g_6posAsyncCount), :540 (calibration_finalize_6pos_position reads g_6posAsyncSum[]), and polls calibration_is_active() at 20 Hz while Core 1 feeds at roughly 1 kHz. So g_calState is a dual-writer variable, and the 6-pos sums are published Core 1 to Core 0 with nothing ordering the three g_6posAsyncSum[] stores before the g_6posAsyncCount++ that releases them (feed_accel_6pos, :319-325). docs/MULTICORE_RULES.md states for this chip that even volatile is not a cross-core barrier, and this state is not even volatile. The module's own neighbours do it correctly -- g_calReloadPending is std::atomic<bool> with an explicit release/acquire pair (cal_hooks.cpp:108 and sensor_core1.cpp:351) -- so the established in-tree pattern simply was not applied here. A reader of calibration_manager.h cannot determine which core owns any of it: the header names the protocol exactly once ("Core 1 feeds samples via calibration_feed_accel()", :152-153) and is otherwise silent, while the apply-path doc two hundred lines later does discuss cross-core use (:283-284), which makes the silence read as "not shared" rather than "undocumented".
- Confidence: high
- Direction: Decide and write down the ownership rule for each item in the :92-128 block (which core writes, which reads, at what point), then make g_calState the published hand-off -- an atomic with release on the writer and acquire on the reader, matching g_calReloadPending -- so the sample buffers it gates are ordered by it. Put the resulting rule in the header preamble, not only in the .cpp.
- Verdict: CONFIRMED -- the :92-128 block is plain non-atomic statics; Core 1 writes g_calState, g_calibration and the g_6posAsync* trio through calibration_feed_* (sensor_core1.cpp:173-181, 206-209) while Core 0 reads and writes the same objects from ao_rcos.cpp:489/522/540, and the file's own neighbour g_calReloadPending shows the correct release/acquire pattern.

### CW-B18-02 -- cal_read_accel is dead code, still registered into a live function pointer, and its comment documents a path nothing runs
- Site: src/calibration/cal_hooks.cpp:37-52 (declared cal_hooks.h:24)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation must describe code that actually runs); spine block C, blocking-in-cooperative-scheduler.
- Claim: cal_read_accel() has no invocation anywhere in the tree, yet it is still installed into rc_os_read_accel and still carries a four-line rationale comment describing behaviour the system never executes.
- Why: Searched the whole source tree for read_accel. The only hits are the definition here, the declaration at cal_hooks.h:24, the typedef and extern at cli/rc_os.h:112-113, the nullptr definition at cli/rc_os.cpp:61, and the assignment rc_os_read_accel = cal_read_accel at main.cpp:317. The pointer is written and never called -- unlike its two siblings, which are invoked at ao_rcos.cpp:667 and :620. That matches the history: 6-position calibration became async and now takes its samples from Core 1 (deviation AP-3, "Resolved per RC_OS consolidation"), which is exactly what removed the need for a Core-0 direct accel read. Two consequences follow. The map is false: cal_hooks.h:6-7 tells a reader these callbacks are "wired into rc_os for calibration wizards", and the :39-42 comment is the only place in the tree recording the ICM-20948 data-ready quirk, so it reads as live, load-bearing knowledge attached to something unreachable. And it is a loaded gun: the pointer is non-null at runtime, and the body does sleep_ms(10) plus icm20948_read(&g_imu, ...) from Core 0 -- both forbidden on that path today, the first because it blocks a run-to-completion AO handler for 10 ms, the second because src/core1/sensor_core1.cpp:344-345 states the invariant "Core 0 must NOT call icm20948_*() unless g_core1I2CPaused == true". Anyone who wires up rc_os_read_accel inherits both violations with no warning in sight.
- Confidence: high
- Direction: Retire cal_read_accel(), its declaration, the rc_os_read_accel pointer and the main.cpp:317 assignment together, relocating the data-ready quirk note to the driver header that owns it. If it is instead meant to come back, keep it but state the two preconditions (Core 1 I2C paused; not callable from an AO handler) in the header preamble and leave the pointer null until a caller exists.
- Verdict: CONFIRMED -- rc_os_read_accel is assigned at main.cpp:317 and invoked nowhere in the tree, so cal_read_accel and its ICM-20948 data-ready rationale document a path nothing runs while the pointer stays non-null with a 10 ms sleep plus Core-0 I2C body; the file-level dead-code inventory cannot see a function-level orphan like this.

### CW-B18-03 -- cal_hooks.h declares four callbacks and states none of their assumptions or limitations
- Site: src/calibration/cal_hooks.h:23-27
- Lens: Comments & documentation quality -- JSF AV 134 ("assumptions (limitations) made by functions should be documented in the function's preamble"); contract-surface helper Kind C (API / behavioural contract).
- Claim: This header is the module's entire contract surface, and for all four functions it offers only signatures plus the line "Callback signatures matching rc_os.h function pointer types" -- none of the preconditions or return semantics a caller must know.
- Why: Each of the four carries a non-obvious contract that only the .cpp reveals. cal_read_mag() returns false for three different reasons -- seqlock read failed (:69), mag_valid false (:74), and sample unchanged since the previous call (:83) -- so false means "no new sample this tick", not "read failed". The single caller happens to treat it that way (ao_rcos.cpp:667), but a caller reading the header alone would reasonably error-handle or abort a wizard on the stale case, which fires constantly by design because Core 0 polls faster than Core 1 publishes. It also returns deliberately uncorrected data (:89, "Return RAW mag data -- ellipsoid solver needs uncorrected samples"), exactly the kind of unit/meaning assumption JSF 134 exists for; a caller who applied calibration on top would silently corrupt the fit. cal_post_hook() is a no-op unless g_sensorPhaseActive (:107), so a caller expecting a guaranteed reload signal does not get one. cal_read_accel() blocks 10 ms and touches I2C from the calling core. And cal_reset_mag_staleness() is not mentioned at all in the file preamble's list of what the module provides (:6-9), which enumerates only three of the four.
- Confidence: high
- Direction: Give each prototype a short preamble stating the one thing the code cannot say: for cal_read_mag, that false means "no fresh sample" and that the data is raw; for cal_post_hook, that it is a no-op outside the sensor phase. Refresh the file-preamble list so it matches the declarations.
- Verdict: CONFIRMED -- the header carries only four signatures and one line of prose; the false-means-stale, raw-not-corrected, no-op-outside-sensor-phase and blocking-I2C contracts exist only in the .cpp, and cal_reset_mag_staleness is absent from the preamble list at :6-9.

### CW-B18-04 -- The header's const accessor is the only handle to the live store, so a consumer casts the const away to write it
- Site: src/calibration/calibration_manager.h:55-58 (implementation calibration_manager.cpp:187-189)
- Lens: Comments & documentation quality -- JSF AV 134 (undocumented limitation on a published contract), with spine CCG P.3 (express intent) and the ownership half of Declaration scope & object lifetime (CCG P.8).
- Claim: calibration_manager_get() is documented as "Get the current calibration data" and typed const calibration_store_t*, but it returns the address of the module's mutable singleton and there is no supported mutator, so an in-tree consumer defeats the const to write through it.
- Why: The body returns &g_calibration (:188) -- the same object Core 1 writes during gyro and level calibration. src/fusion/eskf_runner.cpp:270-277 does const_cast<calibration_store_t*>(calibration_manager_get()), writes wmm_lat_deg, wmm_lon_deg and cal_flags, then calls calibration_save(): a real shipped write path that the header neither offers nor forbids. The const on the return type therefore states a guarantee the module does not provide, and a reader cannot determine from the header whether the pointer aliases live state, how long it stays valid, or who may write it. That matters beyond style here, because those writes land on the same singleton covered by CW-B18-01.
- Confidence: high
- Direction: Either add the named mutator the WMM path actually needs (a calibration_set_wmm_position() that updates the store and saves), or state in the accessor's preamble that the returned pointer is the live singleton, name who may write it, and drop the const the tree does not honour.
- Verdict: CONFIRMED -- calibration_manager_get returns &g_calibration (:188) typed const with no supported mutator, and eskf_runner.cpp:272 is the tree's one const_cast: a shipped path that writes three fields through it and then calls calibration_save().

### CW-B18-05 -- Three of the five mag-staleness statics are written and never read
- Site: src/calibration/cal_hooks.cpp:27-31 (writes at 61, 63-64, 70, 73, 84)
- Lens: Declaration scope & object lifetime -- CCG ES.5 (keep scopes small) applied to dead module state; supporting CERT MSC12-C on the banner that presents them as diagnostics.
- Claim: g_magDiagSeqlockFail, g_magDiagStale and g_magDiagLastSeenCount are incremented or assigned but never read anywhere; they are static to this file, so no other translation unit can reach them either.
- Why: Traced every use in the file. g_lastMagReadCount is read at :79 and :83 and g_magDiagNotValid at :76, so those two are live. The other three are write-only -- the seqlock-failure count and the stale-sample count, the two failure modes an operator would actually want when a compass calibration stalls, are accumulated and then discarded, while the only case that ever prints is mag_valid == false. The banner at :23-25 ("Mag read staleness tracking") plus a public cal_reset_mag_staleness() presents this as a working diagnostic surface, so a reader chasing a stalled wizard reasonably assumes the counters surface somewhere and loses time discovering they do not. I do not believe this is mechanically gated: the unused-variable warnings and the dead-store analyzer target locals, and these statics are read-modify-written, so nothing flags them.
- Confidence: high
- Direction: Either report the three counters where the mag-collection UI reports its other statistics, or delete them along with the parts of cal_reset_mag_staleness() that clear them, so the remaining state matches what the module actually observes.
- Verdict: CONFIRMED -- g_magDiagSeqlockFail, g_magDiagStale and g_magDiagLastSeenCount are written at :61/:70, :63/:84 and :64/:73 and read nowhere; they are file-static, and no enabled check flags read-modify-written file statics.

### CW-B18-06 -- mag_thin_samples has an unchecked count precondition; count == 0 underflows into a 65535-iteration out-of-bounds shuffle
- Site: src/calibration/calibration_manager.cpp:900-914
- Lens: The spine, block B (unstated boundary assumptions) -- Power of Ten Rule 7, "parameter validity must be checked inside each function"; JSF AV 134 for the unstated precondition.
- Claim: The Fisher-Yates loop opens with for (uint16_t i = count - 1; i > 0; i--), so a count of 0 wraps to 65535 and the body indexes g_magSamples[i] far past the 300-entry array, and neither the function nor its comment states that count >= 1 is required.
- Why: g_magSamples is float[300][3], so index 65535 is roughly 786 KB past the base; the memcpy at :907-909 writes outside the array entirely -- an MPU fault at best, silent corruption of whatever it lands on at worst, repeated tens of thousands of times. Today the path is not reachable: the only caller guards g_magSampleCount < kMagMinSamplesForFit (50) and returns CAL_RESULT_NO_DATA at :979-981. But that guard sits 78 lines away, is about fit quality rather than about this loop's precondition, and is stated nowhere near the helper -- so the memory safety of a static helper rests on an unrelated tuning constant staying non-zero. Lower kMagMinSamplesForFit, or add a second caller (the two-step fit already thins between its steps), and the underflow becomes live with no signal from any gate.
- Confidence: medium (the defect and the site are exact; currently unreachable through the single existing caller)
- Direction: Add the cheap guard at the top of the helper -- return count unchanged when it is below 2 -- and state the precondition in its comment so the next caller inherits it rather than inheriting the current caller's unrelated 50-sample threshold.
- Verdict: REFUTED -- the underflow arithmetic is exact but unreachable: mag_thin_samples has exactly one caller (:994), guarded at :979 by the kMagMinSamplesForFit (50) early return, so count == 0 cannot occur on any path and the consequence is hypothetical.

### B19 -- calibration: lm_solver (templates)

#### Coverage
C:/Users/pow-w/Documents/RC-agent-walk/src/calibration/lm_solver.h -- PARTIAL -- read whole (107 lines): module banner, LM constants, the two primitive declarations and all three function templates walked with the spine plus the comments, scope/lifetime and templates lenses; the algorithm is correct and readable, but the contract surface over-claims and leaves its template-argument requirements unstated.
C:/Users/pow-w/Documents/RC-agent-walk/src/calibration/lm_solver.cpp -- PARTIAL -- read whole (102 lines): forward_eliminate, back_substitute, mat_inverse and lm_compute_step each walked; the pivoting, singularity threshold and NaN/inf rejection hold up (back_substitute's divides are provably safe because forward_eliminate rejects any pivot below 1e-10 first), and the only finding sited here is the undocumented static working buffer.

#### Findings

### CW-B19-01 -- "no pointer-to-function in scope" is not what the templates do
- Site: src/calibration/lm_solver.h:13-14 (the claim), src/calibration/lm_solver.h:75-80 (the deduction site)
- Lens: Templates -- JSF AV 101 (review the template "with respect to the template in isolation considering assumptions or requirements placed on its arguments"); Comments -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"). The standards record it underwrites is P10 Rule 9.
- Claim: The header states that template dispatch replaced function-pointer dispatch with "no pointer-to-function in scope", but every in-tree call deduces ResFn and JacFn as raw function-pointer types, so the P10 Rule 9 construct is still present -- only unnamed.
- Why: ResFn and JacFn are by-value template parameters (lines 39, 54, 80). Every call site binds a plain function name -- calibration_manager.cpp:836-839 passes calc_sphere_residual / calc_sphere_jacobian, calibration_manager.cpp:932-935 passes calc_residual_mag / calc_jacobian_mag, test_calibration_lm.cpp:199-202 does the same -- so function-to-pointer decay deduces ResFn = float(*)(const float*, const float*) and JacFn = void(*)(const float*, const float*, float*). Both mag fits deduce identical types, so sphere-fit and ellipsoid-fit share ONE lm_solve specialization and which residual actually runs is chosen by the pointer VALUE passed at run time: that is function-pointer dispatch expressed through a deduced parameter. The "same machine code" half of the claim may well hold once the optimizer inlines lm_solve into each caller, but the "no pointer-to-function in scope" half is not what the source says -- and standards/ACCEPTED_STANDARDS_DEVIATIONS.md retired deviation FP-1 on exactly this reasoning, so anyone auditing P10 Rule 9 compliance from that record reaches the wrong conclusion.
- Confidence: high
- Direction: Either make the dispatch genuinely compile-time by passing the residual and jacobian as distinct stateless callable types (captureless lambdas or tag structs with operator()), so each fit gets its own instantiation and no pointer value exists, or keep the design and correct the header comment and the FP-1 record to describe the construct honestly.
- Verdict: CONFIRMED -- ResFn/JacFn are by-value template parameters that deduce to float(*)(const float*, const float*) and void(*)(const float*, const float*, float*) at every call site (calibration_manager.cpp:834/839, :930/935), so a pointer-to-function object genuinely is in scope inside lm_solve and both mag fits share one specialization; the header's "no pointer-to-function in scope" is not what the source says.

### CW-B19-02 -- template arguments carry no constraint, only prose
- Site: src/calibration/lm_solver.h:37-39, src/calibration/lm_solver.h:50-54, src/calibration/lm_solver.h:73-80
- Lens: Templates -- JSF AV 103 ("Constraint checks should be applied to template arguments"), under the AV 101 review mandate; CCG T.10 as the C++20 technique.
- Claim: All three function templates leave their entire requirement on ResFn and JacFn in a comment (lines 73-74), with no concept, requires-clause or static_assert, although the project builds at C++20 (CMakeLists.txt:21).
- Why: The load-bearing obligation -- that jacobianFn writes exactly numParams floats into the caller's buffer, which is what makes the fixed jacob[kLmMaxParams] at line 55 safe -- is stated nowhere, in code or comment. lm_mean_sq_residuals (37-39) and lm_accumulate_jtj (50-54) carry no signature documentation at all, so a reader of those two must reverse-engineer the callable contract from the bodies. A caller that transposes the last two arguments, or supplies a jacobian of different arity, fails deep inside lm_accumulate_jtj at line 60 rather than at its own call site; a residual returning double is silently narrowed at lines 42 and 59; and a callable that overruns the jacobian buffer is not diagnosed at all.
- Confidence: high that the constraint is absent; medium on severity, since only three instantiations exist in tree and all satisfy the intended contract.
- Direction: Express the requirement where the compiler can see it -- a small local concept (invocable with (const float(&)[3], const float*) returning float; jacobian returning void with a float* out-buffer) on both parameters, or at minimum a static_assert on std::is_invocable_r_v at the top of each template. The "writes exactly numParams entries" obligation cannot be expressed as a concept, so it belongs in the preamble alongside it.
- Verdict: CONFIRMED -- no concept, requires-clause or static_assert constrains ResFn/JacFn although the project builds at C++20, the "writes exactly numParams floats" obligation that makes jacob[kLmMaxParams] safe is stated nowhere, and JSF AV 103 is the walk's own governing template rule (L2P5_MANUAL_WALK_GUIDE.md:417).

### CW-B19-03 -- numParams indexes fixed 9-element buffers unchecked; the only dimension guard is downstream
- Site: src/calibration/lm_solver.h:55, 62, 82, 92 with src/calibration/lm_solver.cpp:67
- Lens: The spine, block B (unchecked / optimistic error handling) -- Power of Ten Rule 7, the parameter-validity half ("parameter validity must be checked inside each function"); the [[nodiscard]] half is gated and not at issue here.
- Claim: numParams is an unvalidated runtime uint8_t that sizes the writes into three fixed kLmMaxParams stack buffers, while the module's only dimension check sits in mat_inverse, where it runs after those writes.
- Why: lm_accumulate_jtj hands jacob[kLmMaxParams] (line 55) to jacobianFn (line 60) and indexes jtfi[row] for row < numParams (line 62); lm_solve declares jtfi[kLmMaxParams] (82) and newParams[kLmMaxParams] (92). Call lm_solve with numParams = 12 -- which the header invites by advertising the module as independently host-testable and by documenting kLmMaxParams as "largest param vector across all callers" (lines 26-28) rather than enforcing it -- and the stack is corrupted at lines 60 and 62 on the first iteration, before line 90 ever reaches the n > kMaxMatDim rejection at lm_solver.cpp:67. That existing guard therefore reads as dimension safety the module does not actually provide. Nothing is broken today: the three in-tree callers pass 4 and 9, and the mag path is additionally guarded on sample count (kMagMinSamplesForFit = 50), so the divide at line 45 is likewise unreachable with a zero denominator.
- Confidence: medium
- Direction: Make the bound structural rather than documented -- a non-type template parameter for the parameter count would size the buffers correctly and turn any mismatch into a compile error. If it must stay a runtime argument, reject numParams > kLmMaxParams at the entry of lm_solve and lm_accumulate_jtj, where the buffers actually live.
- Verdict: REFUTED -- unreachable: every in-tree call passes kMagSphereParams (4) or kMagEllipsoidParams (9), both <= kLmMaxParams (9), so no path writes past the fixed buffers; the finding describes a caller that does not exist.

### CW-B19-04 -- bestParams / bestFitness are in-out, but the preamble describes them as outputs
- Site: src/calibration/lm_solver.h:70-72 with src/calibration/lm_solver.h:96-99
- Lens: Comments & documentation quality -- JSF AV 134 ("Assumptions (limitations) made by functions should be documented in the function's preamble"); CCG NL.2.
- Claim: The lm_solve preamble says only that "On return, bestParams holds the best fit and *bestFitness holds RMS^2", but the body reads *bestFitness before it ever writes it and can return without writing bestParams at all.
- Why: Line 96 evaluates fitness < *bestFitness on the very first iteration, so the caller must pre-seed *bestFitness with a real fitness value; both mag fits do (calibration_manager.cpp:833-835 and :929-931 call lm_mean_sq_residuals first) and so does the test (test_calibration_lm.cpp:194), which is why nothing has ever failed. A caller who follows the preamble literally and passes an uninitialized float performs an indeterminate read at line 96, and -- because the memcpy into bestParams at line 98 sits inside the improvement branch -- if no iteration improves on that seed, or if the loop breaks early at line 90 or 93, bestParams is returned holding exactly whatever the caller left in it. The one sentence that describes these two parameters tells the reader the opposite of the obligation they carry.
- Confidence: high
- Direction: State the in-out contract in the preamble -- caller initializes bestParams to the starting parameters and *bestFitness to their fitness -- or remove the trap entirely by having lm_solve compute the initial fitness itself so that both really are pure outputs.
- Verdict: CONFIRMED -- :96 reads *bestFitness before anything writes it and the memcpy into bestParams at :98 sits inside the improvement branch, so both are in-out parameters while the preamble at :72 describes them purely as outputs.

### CW-B19-05 -- mat_inverse holds hidden static scratch that the "pure-function module" banner denies
- Site: src/calibration/lm_solver.cpp:71 with src/calibration/lm_solver.h:7-9 and src/calibration/lm_solver.h:30-32
- Lens: Comments & documentation quality -- JSF AV 134; CCG NL.2. (Declaration scope & object lifetime: the wide scope itself is the mandated-static case and is explicitly NOT the finding.)
- Claim: mat_inverse computes into a 648-byte function-local static augmented matrix, which neither the module banner nor the public declaration gives a reader any way to know.
- Why: g_aug (lm_solver.cpp:71) is the [A|I] working matrix the inverse is actually produced in; it is not one of the three buffers the banner enumerates at lines 7-9 ("all working state (samples, JtJ buffer, inverse buffer) is passed in by the caller"), and it is not passed in. The consequence is a reentrancy limitation on a function the header deliberately publishes for reuse ("Public so both the solver templates and the host tests can reach them", lines 30-31): two overlapping mat_inverse calls -- a second core, a diagnostic path, or a future concurrent fit, and docs/MULTICORE_RULES.md names "calibration fits" as a candidate Core-1 offload -- silently share one buffer and both return a wrong inverse with a true status. Holding it static is the right call on this chip (648 bytes is far past the >256-byte stack guidance in MULTICORE_RULES.md), so the defect is the silence, not the storage class.
- Confidence: medium
- Direction: Add one line at the mat_inverse declaration stating that it uses a shared static working buffer and is therefore not reentrant, and narrow the banner so that "no file-scope globals" cannot be read as "no hidden state".
- Verdict: CONFIRMED -- mat_inverse computes in the function-local static g_aug (lm_solver.cpp:71, 9x18 floats = 648 bytes), which is neither among the three caller-passed buffers the banner enumerates nor mentioned at the declaration the header publishes for reuse.

### CW-B19-06 -- the header half of the module is outside every routine gate, and it shows
- Site: src/calibration/lm_solver.h:33-34 and 76-80, against src/calibration/lm_solver.cpp:91-92
- Lens: The spine, block A -- CCG P.3 (express intent: on a contract surface the names are the contract) -- plus the project's own naming standard, JSF AV 51 as configured in .clang-tidy (ParameterCase / LocalVariableCase = lower_case). Reported with the coverage question attached rather than dropped as "gated", per the walk rule on uncertainty.
- Claim: The same function is declared with camelBack parameter names in the header and defined with lower_case names in the .cpp, and lm_solve carries 11 parameters against the project's own threshold of 6 -- none of which any routine gate reports, because no configured run reaches this header.
- Why: lm_compute_step is declared as (..., float* newParams, const float* jtjInv, const float* jtfi, uint8_t numParams) at lm_solver.h:33-34 and defined as (..., float* new_params, const float* jtj_inv, ..., uint8_t num_params) at lm_solver.cpp:91-92, so a reader of the contract surface cannot tell which spelling is authoritative -- and it is the .cpp side that matches the convention the 2026-06-24/25 identifier remediation applied across the rest of the tree. Every parameter and local in the header is camelBack (numSamples, bestParams, bestFitness, numParams, maxIter, jtjInv, residualFn, jacobianFn, and newParams at line 92); lm_solve's signature at lines 76-80 takes 11 parameters against readability-function-size.ParameterThreshold = 6. Why it survived: .clang-tidy sets HeaderFilterRegex to a pattern requiring src/rocketchip/ or include/rocketchip/ in the path, which src/calibration/lm_solver.h does not match, and scripts/audit/full_tree_clang_tidy.sh iterates the tracked src/*.cpp list without overriding that filter -- so diagnostics located in this header are suppressed in the pre-commit gate and the milestone sweep alike. This is the LL Entry 43 shape (a clean static gate is negative evidence), applied to a header rather than to a compile flag.
- Confidence: high on the declaration/definition divergence and on the parameter count; medium on the coverage conclusion, which is inferred from the config and the sweep script rather than from an observed run -- if some invocation I did not see passes a wide header filter, only the coverage half of this finding falls away.
- Direction: Bring the header's parameter and local names to lower_case so that declaration and definition agree, and settle the coverage question separately -- either widen HeaderFilterRegex to cover authored headers outside include/rocketchip/, or record the exclusion deliberately so it is a decision rather than an accident.
- Verdict: CONFIRMED -- lm_solver.h:33-34 declares newParams/jtjInv/numParams while lm_solver.cpp:91-92 defines new_params/jtj_inv/num_params, lm_solve takes 11 parameters against ParameterThreshold=6, and neither is reported: full_tree_clang_tidy.sh's GATED list is only the four size/complexity/return-value/reserved-id checks, and HeaderFilterRegex '.*(src|include)/rocketchip/.*' excludes this header from the size check as well.

### B20 -- flight_director: HSM core + state/actions types

#### Coverage
src/flight_director/flight_actions.h -- PARTIAL -- Read whole as a contract surface (Kind D/E, shared vocabulary + per-phase action map); all nine entry lists, both transition lists, the exit table and the two index tables inventoried against their consumers in flight_director.cpp.
src/flight_director/flight_director.cpp -- PARTIAL -- Read whole (687 lines); spine blocks A, B and C run on all 31 functions and state handlers, with the abort, coast-timeout, reset and fault paths hand-walked against the shipped mission profile.
src/flight_director/flight_director.h -- PARTIAL -- Read whole as the .cpp's declared contract; the HSM hierarchy, the signal list and the usage block all verified accurate against flight_director.cpp and include/rocketchip/ao_signals.h, but the constructor's precondition on profile is undeclared (CW-B20-06).
src/flight_director/flight_state.h -- PARTIAL -- Read whole; phase enum, fault-observable phase contract, FlightMarkers and FlightState reviewed claim-vs-truth against flight_director.cpp, src/safety/fault_protection.cpp and the SPIN models.

#### Findings

### CW-B20-01 -- File banner states eight phases; the enum defines nine
- Site: src/flight_director/flight_state.h:8
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong")
- Claim: The banner states "FlightPhase enum defines the 8 flight phases (7 nominal + ABORT)" while the enum at lines 48-60 defines nine values (kIdle through kFault) with kCount = 9.
- Why: The same file at lines 26-27 tells the reader the values are "contiguous starting from 0 for array indexing and compact wire encoding in PCM frames and telemetry packets (uint8_t)", so the phase count is load-bearing rather than prose. The file also contradicts itself: the topology block at lines 29-46 does list Fault. A reader who sizes a per-phase table or a wire field from the banner's count of eight drops kFault (value 8) -- the exact phase the fault path writes through flight_phase_observable_set from src/safety/fault_protection.cpp:73.
- Confidence: high
- Direction: Update the banner to nine phases (7 nominal + ABORT + FAULT), or delete the count from the banner so kCount is the single statement of arity.
- Verdict: CONFIRMED -- :8 says "8 flight phases (7 nominal + ABORT)" while the enum at :48-60 defines nine values through kFault with kCount = 9, and the same file's own topology block lists Fault.

### CW-B20-02 -- Both banners assert ABORT fires drogue; the shipped profile disables it
- Site: src/flight_director/flight_director.cpp:10-13 (paired claim at src/flight_director/flight_actions.h:117-121)
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2; spine block B (a confident comment the body does not implement)
- Claim: Both file banners state as unconditional behaviour that ABORT-from-BOOST and ABORT-from-COAST fire drogue pyro, but the implementation at flight_director.cpp:610-622 fires only when profile->abort_fires_drogue_from_boost / _from_coast is true, and the only shipped production profile sets both to false (src/flight_director/mission_profile_data.h:41-42).
- Why: A reviewer or maintainer reading either banner concludes that an operator ABORT during BOOST deploys the drogue as a safety action. On the shipped Rocket profile it deploys nothing, and recovery falls entirely to the independent PIO backup timers. The file disagrees with itself as well: the per-state comments at :417 and :444 do carry the "(fires drogue if profile flag set)" qualifier, so only the two banners -- the first thing a reader sees in each file -- carry the unqualified form, and flight_actions.h is the file a reader opens specifically to learn what the abort transition does.
- Confidence: high
- Direction: Qualify both banners with the governing profile flag (naming abort_fires_drogue_from_boost / _from_coast), or state the shipped default explicitly so the reader is not left with the opposite of the as-built behaviour.
- Verdict: CONFIRMED -- both banners state the drogue fire unconditionally while :610-622 gates it on profile->abort_fires_drogue_from_boost/_from_coast, and mission_profile_data.h:41-42 sets both false in the shipped profile.

### CW-B20-03 -- Pyro-fired latch comment describes a RESET clear that does not exist, and names consumers that do not exist
- Site: src/flight_director/flight_state.h:132-136
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 plus CERT MSC12-C (documentation must describe code that actually runs)
- Claim: The comment says the flags are "latched on fire, never cleared until RESET" and are "Queried by CLI, telemetry, and SPIN model (!drogue_fired guard)", but the only writer of false is FlightState::init() (line 138), which runs only from flight_director_ctor at boot (flight_director.cpp:198, called once from src/active_objects/ao_flight_director.cpp:258); no SIG_RESET path clears them, and a repository search finds no read of state.drogue_fired or state.main_fired in src/ at all.
- Why: After a flight in which the drogue fired, SIG_RESET dispatches to state_idle (flight_director.cpp:579 and :626) whose entry handler at :347-364 clears nothing, so the vehicle sits in kIdle with drogue_fired still true until a power cycle. The verified SPIN model does the opposite: it clears both flags on every transition to IDLE (tools/spin/rocketchip_ao.pml:135, 140, 153, 261, 272) and proves the safety property ltl p_no_pyro_idle at :376-378, which asserts that IDLE implies neither flag is set. The firmware falsifies that property on the second arm cycle of a single power cycle, so the comment records the model's semantics rather than the firmware's, and a future consumer that adopts the documented "!drogue_fired guard" would suppress deployment on the second flight.
- Confidence: high
- Direction: Either clear the pyro-fired flags on entry to kIdle so firmware and the verified model agree, or correct the comment to say the flags clear only at construction and log the model-versus-firmware divergence so the SPIN property is not read as covering the firmware.
- Verdict: CONFIRMED -- the only writer of false is FlightState::init() (:141-145), called once from flight_director_ctor:198; state_idle's entry handler clears nothing on SIG_RESET, and no read of drogue_fired/main_fired exists in src/ at all, so both halves of the comment are false.

### CW-B20-04 -- ABORT timeout branches on flight markers that survive RESET
- Site: src/flight_director/flight_director.cpp:628-646
- Lens: The spine, block B -- passes-tests-yet-wrong ("exercise corner/edge/unexercised inputs by hand; probe unstated ordering assumptions"); same root cause as CW-B20-03 seen at a behavioural site
- Claim: The ABORT timeout separates pad abort from in-flight abort with markers.launch_ms == 0 and uses markers.landing_ms == 0 as the beacon one-shot, but FlightMarkers::clear() is reached only from FlightState::init() at construction, so on any arm cycle after the first launched flight both tests read the previous flight's timestamps.
- Why: Concrete path, single power cycle: flight 1 reaches BOOST (kBoostEntry sets launch_ms) and LANDED (kLandedEntry sets landing_ms); operator issues RESET to IDLE, re-ARMs, then aborts on the pad. launch_ms is non-zero, so the pad-abort branch at :631-635 is skipped and the behaviour documented at :593 ("Pad abort (from ARMED): timeout -> IDLE (auto-safe, never launched)") never happens. landing_ms is also non-zero, so the in-flight branch at :639-645 is skipped as well and no beacon is raised. The vehicle stays latched in ABORT with neither documented outcome. Host tests and the bench sim construct a fresh director per case, so this second-cycle path is never exercised.
- Confidence: high
- Direction: Clear the markers (and the pyro-fired flags) on entry to kIdle -- the handler already distinguishes a non-startup entry with the transition_count > 1 test at :353 -- or carry an explicit per-flight epoch instead of overloading markers as one-shot flags.
- Verdict: CONFIRMED -- markers.clear() is reached only from FlightState::init() at construction (no other caller in src/), so on any arm cycle after a launched flight both :631 (launch_ms == 0) and :639 (landing_ms == 0) read the previous flight's timestamps and neither documented ABORT-timeout outcome occurs.

### CW-B20-05 -- FAULT entry actions are unreachable, and their comment documents an operator indication that cannot occur
- Site: src/flight_director/flight_actions.h:132-145 (index row at :169)
- Lens: Comments & documentation quality -- CERT MSC12-C (comments describing dead or unreachable code); spine block A one-page gestalt
- Claim: The FAULT block documents an entry action list for a phase the HSM has no state handler for. run_entry_actions (flight_director.cpp:97-105) indexes kPhaseEntryActions by the phase it is passed, and it is only ever called with the eight non-fault phases from the eight state handlers; me->state.current_phase is written only by enter_phase (:56), which is likewise never called with kFault.
- Why: kLedPhaseFault exists solely to produce an operator-visible distinction that action_executor.h:57-60 documents as "distinct from Abort (red) so operator can tell 'ARM/safety state lost integrity' apart from 'operator-commanded abort'". The only code that reaches kFault is fault_protection.cpp:73, which writes the fault-observable phase pair and then busy-loops forever without touching the FD HSM, and that handler's own visible signal (fault_emit_visible_signal, fault_protection.cpp:44-48) is a documented no-op placeholder. So the documented magenta fault indication is produced by nothing, while the comment block reads as a description of live behaviour.
- Confidence: high
- Direction: Either drive the fault indication from the path that actually reaches kFault, or remove kFaultEntry with its comment and record in the fault-recovery decision doc that FAULT currently has no operator-visible LED state.
- Verdict: CONFIRMED -- run_entry_actions is reached only from the state handlers and there is no handler for kFault; the fault path (fault_protection.cpp:73) writes only the observable pair and busy-loops, fault_emit_visible_signal is a documented no-op, and FAULT_RECOVERY_2026-05-14.md B.8's claim that enter_phase(kFault) sets the LED is not implemented -- so kFaultEntry and its magenta indication are dead.

### CW-B20-06 -- Constructor's non-null profile precondition is neither declared nor asserted, in the one module that declares assertion infrastructure
- Site: src/flight_director/flight_director.cpp:196-215 (module declaration at :33; header declaration at src/flight_director/flight_director.h:95)
- Lens: Assertions -- JPL-C Rule 16 (basic sanity checks / parameter validity at entry) and Power of Ten Rule 5; JSF AV 134 for the missing preamble statement
- Claim: flight_director_ctor takes a raw const MissionProfile* and dereferences it at :208 and :210 with no entry check, and neither the definition nor the header declaration states that profile must be non-null and must outlive the director; the whole 687-line translation unit contains no runtime assertion of any kind, even though it carries the only Q_DEFINE_THIS_MODULE in src/.
- Why: the stored pointer is dereferenced unguarded on the 100 Hz flight path at :255-256, :285-286, :401, :458, :469, :472, :525 and :611, so the non-null-and-outlives assumption is load-bearing -- yet it is written down nowhere and a caller has nothing to check itself against. Against JPL-C Rule 16 / Power of Ten Rule 5 the module's assertion density is zero, while the bare Q_DEFINE_THIS_MODULE at :33 makes the file read as assertion-instrumented when it contributes only a module-name string (Q_onError is a real implemented fault path at src/safety/fault_protection.cpp:174, so an assertion here would be a genuine runtime safety event rather than a debug no-op). No null or dangling profile is reachable today: the single caller (src/active_objects/ao_flight_director.cpp:258) passes &rc::kDefaultRocketProfile, a static that outlives the director -- so the finding is the unstated contract plus the missing entry check, not a live undefined-behaviour path.
- Confidence: high
- Direction: Add the entry-point sanity check on profile (and me) in the constructor, and state the non-null-plus-outlives assumption in the flight_director.h preamble per JSF AV 134; drop Q_DEFINE_THIS_MODULE if no assertion is intended in this module.
- Verdict: RESHAPED -- the unstated precondition and the zero-assertion/Q_DEFINE_THIS_MODULE mismatch are exactly as cited, but the null-or-dangling-profile UB is unreachable (the sole caller, ao_flight_director.cpp:258, passes &rc::kDefaultRocketProfile, a static), so the claim narrows to the documentation and assertion-density gap.

### CW-B20-07 -- Exit-action machinery is documented as executed but is never dispatched, and kEmptyActions is dead
- Site: src/flight_director/flight_actions.h:155-157 (claims at :6-7 and table at :172-183)
- Lens: Comments & documentation quality -- CERT MSC12-C plus JSF AV 131 / CCG NL.2
- Claim: flight_actions.h:6-7 states that the entry, exit and transition lists "are executed by the QEP state handlers via action_execute_list()", but no state handler in flight_director.cpp has a Q_EXIT_SIG case, no run_exit_actions exists anywhere in src/, kPhaseExitActions is read only by a host test, and kEmptyActions is referenced nowhere in the repository.
- Why: A maintainer adding a per-phase exit action would populate kPhaseExitActions and reasonably expect it to run on state exit; nothing dispatches it, so the action would silently never execute -- and on this state machine an exit action is exactly the kind of place a safety cleanup would be added. kEmptyActions carries the comment "Placeholder -- never executed with count=0" describing a role it does not have, since no table pairs it with a count. flight_director.cpp:7 repeats the same claim at the top of the implementation ("Each handler processes QEP signals (Q_ENTRY_SIG, Q_EXIT_SIG, user signals)") while no handler processes Q_EXIT_SIG.
- Confidence: high
- Direction: Delete kEmptyActions, and correct both banners so they describe the entry-plus-transition mechanism that actually runs; if exit lists are intended as a reserved slot, say so explicitly at :172 rather than describing them as executed.
- Verdict: CONFIRMED -- flight_actions.h:6-7 and flight_director.cpp:7 both claim exit lists / Q_EXIT_SIG are processed; Q_EXIT_SIG appears nowhere in src/ except that comment, kPhaseExitActions is read only by test_action_executor.cpp:293, and kEmptyActions has no reference anywhere in the repository.

### CW-B20-08 -- Fire-drogue-and-enter-descent idiom written inline twice in state_coast
- Site: src/flight_director/flight_director.cpp:456-463 and :466-478
- Lens: The spine, block A -- CCG F.1 ("Package meaningful operations as carefully named functions") and ES.3 (Fowler's Duplicated Code)
- Claim: The nameable action "fire the drogue if the profile has pyro, then transition to DESCENT" is written inline twice inside state_coast, once for SIG_APOGEE and once for the coast-timeout fallback, and the second copy's own comment concedes it at :471 ("same as SIG_APOGEE").
- Why: The two copies encode the same safety knowledge in two places -- the has_pyro gate, the kTransitionFireDrogue array, and the from/to phase pair handed to the action context. A later change to that path (adding a deploy lockout check, a second channel, or a different from-phase for the timeout case) has to land in both, and the two copies are not equally exercised: the timeout copy is precisely the one that runs when primary apogee detection has already failed, so a divergence would surface first on the degraded path. The bodies differ by the log line and the elapsed test, so the identical-branch gate does not see them.
- Confidence: medium
- Direction: Extract one named helper used by both cases (for example fire_drogue_then_descend(me)); the file already has run_transition_actions, so this is one further named step at the same altitude rather than a new abstraction.
- Verdict: REFUTED -- style/refactor preference: the safety knowledge is already centralised in the single named kTransitionFireDrogue array plus run_transition_actions, the two sites differ in guard and log line, and no divergence exists on any path -- a repeated four-line dispatch is not a defect.

### CW-B20-09 -- Cross-core assumption and second writer of the fault-observable phase pair are undocumented
- Site: src/flight_director/flight_director.cpp:147-167 (declared contract at src/flight_director/flight_state.h:66-92)
- Lens: Declaration scope & object lifetime -- CCG CP.2 (avoid data races on shared state) and JSF AV 134 (document a function's assumptions and limitations); the spine's embedded ADD on volatile-as-cross-core-barrier
- Claim: g_phaseObservablePair is a plain volatile uint32_t written by enter_phase on Core 0 and also written from either core's fault handler (src/safety/fault_protection.cpp:73) and read from either core's fault handler (:149 and :215), yet the contract block at flight_state.h:70-80 enumerates three constraints -- no allocation, no locks, single-byte corruption detection -- and names neither the cross-core visibility question nor the existence of a second writer.
- Why: docs/MULTICORE_RULES.md states as a project rule that plain volatile is not a cross-core mechanism, so this object specifically needs a recorded reason why that rule does not bite here, and the reader has no way to tell whether the reasoning was done. I am not claiming a demonstrated race: the access is a single naturally-aligned 32-bit load and store and the RP2350 Cortex-M33 has no data cache, so staleness is plausibly bounded and the design may well be sound. The undocumented second writer is the sharper gap -- a Core-1 fault sets kFault and spins forever while Core 0 keeps running and overwrites the pair on its next transition, so the "we already degraded" signal is silently lost, and nothing in either file tells a reader that is expected.
- Confidence: medium
- Direction: Extend the flight_state.h preamble to name the writers (FD on Core 0 plus either core's fault handler), the readers, and why a single aligned word needs no barrier on this part; or declare the object std::atomic<uint32_t> with relaxed ordering so the type itself carries the claim.
- Verdict: REFUTED -- the sharper half is contradicted by the cited file itself: flight_state.h:85-88 states the setter must be reachable from anywhere the phase changes, naming the fault handler as a direct kFault writer, and FAULT_RECOVERY_2026-05-14.md B.3/B.8 records the same second writer; the remaining cross-core half is explicitly not a demonstrated race.

### B21 -- flight_director: command_handler + action_executor

#### Coverage

- src/flight_director/action_executor.h -- FAIL -- Read whole; contract surface (helper Kind D vocabulary + Kind C API contract) evaluated claim-by-claim against led_patterns.h, ao_led_engine.cpp and the wired callbacks in ao_flight_director.cpp; three claims do not survive that check.
- src/flight_director/action_executor.cpp -- PARTIAL -- Read whole; the three functions are correctly shaped and single-purpose, but the two public entry points carry no parameter-validity checks and no documented preconditions.
- src/flight_director/command_handler.h -- PARTIAL -- Read whole; per-command contract prose is accurate against the body except that the kArm section omits one of the two blocking gates the implementation applies.
- src/flight_director/command_handler.cpp -- PASS -- Read whole; one flat validate-and-return switch, every branch reachable and eyeball-verifiable, rejection strings bounded and explicitly NUL-terminated, and the header's sensor-signal-bypass claim confirmed against ao_flight_director.cpp:287-289.

#### Findings

### CW-B21-01 -- LedPhaseValue is a second copy of the LED pattern map and has already drifted from it
- Site: src/flight_director/action_executor.h:47-61 (against include/rocketchip/led_patterns.h:4,18-19,75-89)
- Lens: The spine, block A -- CCG ES.3 "Don't repeat yourself, avoid redundant code" (Fowler's Duplicated Code smell); reinforced by the contract-surface helper Kind D/E question "is there ONE place that defines these names / could two copies of the map silently disagree?"
- Claim: The LedPhaseValue enum restates the flight-phase pattern codes that led_patterns.h declares itself the "Single Source of Truth" for, and the two copies now disagree at code 28.
- Why: led_patterns.h:18 asserts "20-27 = flight phase overlays (match LedPhaseValue in action_executor.h)" and :75 repeats "Values match LedPhaseValue enum in action_executor.h", but LedPhaseValue extends past that range to kLedPhaseFault = 28, and 28 is already taken by rc::led::kFdPreArmFail (led_patterns.h:89), which ao_led_engine.cpp:158 renders as a yellow double-flash. The pattern code emitted for the FAULT phase therefore means "pre-arm fail" to the only component that renders pattern codes. The colour comments have drifted too: action_executor.h:49 says kLedPhaseArmed is "Amber solid" while led_patterns.h:77 and ao_led_engine.cpp:148 both say solid red ("AP parity, Stage L -- was amber"). Today this cannot reach the LED engine because the wired set_led callback (ao_flight_director.cpp:222-229) discards every value except kLedPhaseBeacon, so the defect is latent rather than live -- but it is exactly the drift the two "values match" comments promise cannot happen.
- Confidence: high
- Direction: Have action_executor.h consume rc::led::kFd* directly (or alias LedPhaseValue members to them) so one table owns the codes, and resolve the 28 conflict by giving the FAULT phase its own code inside led_patterns.h's numbering.
- Verdict: CONFIRMED -- Verified in file: action_executor.h:57 defines kLedPhaseFault = 28 while led_patterns.h:89 already owns 28 as kFdPreArmFail (rendered yellow double-flash at ao_led_engine.cpp:158), the two "values match" comments at led_patterns.h:18/:75 are therefore false, flight_actions.h:143 really does emit that value, and the amber-vs-red drift at action_executor.h:49 against led_patterns.h:77 / ao_led_engine.cpp:148 is a live documentation defect today.

### CW-B21-02 -- Header points readers at main.cpp for two things that are no longer there, and describes a SET_LED path the system no longer takes
- Site: src/flight_director/action_executor.h:11, 44, 113-114
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 (comment and code disagree) and CERT MSC12-C (doc-comment describing a path the code can no longer take).
- Claim: Three location/behaviour claims in the header banner and the ActionContext block are stale.
- Why: (a) :44 says the values "extend the kCalNeo* / kRxNeo* overlay scheme in main.cpp" -- those constants live in include/rocketchip/led_patterns.h:124-136, whose own banner records that it "Replaces duplicate definitions in main.cpp and rc_os.cpp". (b) :113-114 says "main.cpp provides implementation" of the set_led callback -- the only production implementation is the lambda installed at ao_flight_director.cpp:222-229. (c) :11 says "SET_LED -- Set NeoPixel mode/color for current phase", but that lambda ignores every value except kLedPhaseBeacon; phase LED routing now runs through AO_Notify / notify_backend_led.cpp:63-72, which maps PhaseIntent to rc::led::kFd* without consulting these action lists. A maintainer following the header to main.cpp finds nothing, and one who assumes the kSetLed entries still drive the pixel will mis-diagnose any phase-LED behaviour.
- Confidence: high
- Direction: Repoint the two main.cpp references at led_patterns.h and ao_flight_director.cpp, and restate the SET_LED line as what it now is -- an action-list marker whose value is delivered to whatever callback the owner installs (currently beacon-only), with phase LED routing owned by AO_Notify.
- Verdict: CONFIRMED -- All three location claims check out with the files open: main.cpp contains zero kCalNeo*/kRxNeo* references (they live at led_patterns.h:124-136) and no set_led implementation at all (the only production one is the lambda at ao_flight_director.cpp:222-229, which acts only on kLedPhaseBeacon), and phase LED routing really does run through notify_backend_led.cpp's phase_to_pattern.

### CW-B21-03 -- FIRE_PYRO is documented as a log-only action; the wired callback also cancels the independent backup deployment timer
- Site: src/flight_director/action_executor.h:14, 117-118
- Lens: Comments & documentation quality -- JSF AV 134 ("Assumptions (limitations) made by functions should be documented in the function's preamble") and CCG NL.2; spine block B, the confident comment the body does not match.
- Claim: The header describes FIRE_PYRO as "Log pyro intent (no physical pyro in Stage 8)" and its callback as "log pyro intent (printf on target, stub on host)", but the production callback performs safety-relevant state changes, not logging.
- Why: fd_on_pyro_fired (ao_flight_director.cpp:178-197) latches director.state.drogue_fired / main_fired, writes a kPyroFiredDrogue/kPyroFiredMain flight-log event, publishes SIG_PYRO_FIRED to all AO subscribers, and calls pio_backup_timer_cancel(). Per pio_backup_timer.h:6-12 that timer is the ARM-core-independent path that "fire[s] pyro GPIOs on expiry" and is "cancelled on successful smart deployment" -- so executing a FIRE_PYRO action suppresses the redundant physical deployment. On the path where a maintainer trusts the header and treats a FIRE_PYRO entry as free of physical consequence -- reordering it, making it conditional, or short-circuiting it in a bench build -- they silently change whether the backup channel still fires.
- Confidence: high
- Direction: Rewrite the FIRE_PYRO line and the log_pyro field comment to state the real contract -- the callback is the primary-deployment notification and its owner is expected to latch state and cancel the backup timer -- and drop the "no physical pyro in Stage 8" clause, which no longer describes the system.
- Verdict: CONFIRMED -- fd_on_pyro_fired (ao_flight_director.cpp:178-197) latches drogue_fired/main_fired, logs the event, publishes SIG_PYRO_FIRED and calls pio_backup_timer_cancel(), while the header still calls the action and its callback "log pyro intent" with "no physical pyro in Stage 8"; pio_backup_timer.h:6-12 confirms the cancelled timer is the autonomous GPIO-firing channel, so the comment understates a safety-relevant side effect.

### CW-B21-04 -- Both public entry points dereference their pointer parameters unchecked while defensively null-checking that pointer's members
- Site: src/flight_director/action_executor.cpp:30-34, 39-40, 50-51, 66-69 (declarations at action_executor.h:125-130)
- Lens: Assertions -- Power of Ten Rule 7 ("parameter validity must be checked inside each function") / JPL-C Rule 16 (entry-point sanity checks); secondary JSF AV 134 for the missing preamble.
- Claim: action_execute and action_execute_list assume a non-null ctx, a non-null actions array holding at least count entries, and an in-range param-to-enum cast, and verify none of them.
- Why: The body establishes a nullability contract for the callbacks -- ctx->set_led (:33), ctx->markers (:39) and ctx->log_pyro (:50) are each guarded -- yet ctx itself is dereferenced five times with no check, and action_execute_list indexes actions[i] (:69) with no check that actions is non-null when count is greater than zero. Nothing states the contract either: the two declarations carry only "Execute a single action" / "Execute an action list". The enum casts are the same gap -- static_cast<MarkerId>(action.param) at :40 and static_cast<PyroChannel>(action.param) at :51 accept any uint8_t, and an out-of-range MarkerId falls through set_marker's fully-enumerated switch writing nothing at all, so a mis-authored action entry loses an event timestamp silently instead of tripping anything. Recorded honestly: no file under src/flight_director/ carries a runtime assertion today, so the absence is the subsystem's habit rather than a lapse unique to this file -- but this file is the one that makes the asymmetry visible by guarding the members and not the container.
- Confidence: medium
- Direction: Add entry assertions for ctx (and for actions when count is non-zero) plus a bound check before the enum casts, and state those preconditions in the two declaration preambles; where the caller is a constexpr action list the MarkerId/PyroChannel bound is a candidate for a compile-time check instead.
- Verdict: REFUTED -- Both entry points are module-internal: the only callers are flight_director.cpp:102 and :113, which pass the address of a stack-local ActionContext and constexpr ActionEntry arrays, so neither the null ctx, the null actions array, nor an out-of-range enum cast is reachable from any in-tree path, and the finding itself records that no file under src/flight_director/ carries a runtime assertion -- making this a subsystem-wide defensive-coding preference rather than a defect of this file.

### CW-B21-05 -- Declared kArm contract omits the test-mode blocking gate the implementation applies
- Site: src/flight_director/command_handler.h:39-51 (against src/flight_director/command_handler.cpp:45-52)
- Lens: Comments & documentation quality -- JSF AV 134 (document a function's assumptions and limitations in its preamble); contract-surface helper Kind C, "what may callers assume / what is forbidden".
- Claim: The header enumerates the kArm gates as "runs Go/No-Go poll, prints result, blocks if Tier 1 NO-GO" but the body rejects on a second, independent condition -- test mode being armed -- read from global state rather than from a parameter.
- Why: command_handler.cpp:50 returns rejected("Test mode active") from rc::test_mode_active(), a gate added by R-25-exec council amendment #2 and explained only in the .cpp. A caller reading the header alone -- the whole point of a contract surface -- concludes that an IDLE vehicle passing Tier 1 Go/No-Go will accept ARM, and cannot account for the rejection when it happens. The omission also hides that the function is not a pure function of its three parameters, which matters because the sibling contract it composes with advertises the opposite (go_nogo_checks.h:13: "All checks are pure functions of a GoNoGoInput snapshot -- no globals").
- Confidence: medium
- Direction: Add the test-mode gate to the kArm bullet in the header contract and note in the preamble that the function consults global test-mode state, so the declared contract lists every way an ARM can be refused.
- Verdict: CONFIRMED -- command_handler.cpp:50-52 really does reject on rc::test_mode_active(), a gate read from global state that the header's kArm bullet at :41 never mentions, and go_nogo_checks.h:13 does advertise the opposite property ("no globals") for the sibling contract it composes with.

### B22 -- flight_director: go_nogo + guard_evaluator

#### Coverage
src/flight_director/go_nogo_checks.h -- PASS -- Contract surface (2 structs + 2 free functions) read whole; every field carries a units/meaning comment, the kGoNoGoMaxChecks bump and the rf_link_state 0..3 encoding are both documented at the declaration, and the Tier-1-blocks / Tier-2-warns contract is stated in the preamble.
src/flight_director/go_nogo_checks.cpp -- PARTIAL -- All four functions walked; two findings (CW-B22-01 fail-open station overflow, CW-B22-02 GO-worded reason on a NO-GO station); the rest of the file is clean and the strncpy / etl::string lifetime reasoning at :47-50 checks out against the buffer sizes.
src/flight_director/guard_evaluator.h -- PARTIAL -- Contract surface read whole; the guard/state model and phase-validity behaviour are well documented, but guard_evaluator_is_sustained states no precondition on its GuardId argument, the "Initialize the evaluator" contract at :76-77 is not fully met by the implementation (CW-B22-03), and the uint8_t width of GuardState::valid_phases is not stated as a bound on FlightPhase (CW-B22-04).
src/flight_director/guard_evaluator.cpp -- FAIL -- All six functions walked; the init path leaves a GuardState field with no meaningful value (CW-B22-03) and the phase bitmask cannot represent the whole FlightPhase enum (CW-B22-04).

#### Findings

### CW-B22-01 -- Go/No-Go station overflow fails open on the ARM gate
- Site: src/flight_director/go_nogo_checks.cpp:19 (with :29-36 and :116)
- Lens: Assertions -- JPL-C Rule 16 ("assertions ... check for anomalous conditions that should never happen"), plus the P10 Rule 7 manual residual on error-path shape and spine block B (passes-tests-yet-wrong).
- Claim: add_station silently drops a station when the array is full, and neither tier1_total nor tier1_go records the drop, so GoNoGoResult has no way to represent a truncated poll and go_nogo_print reports a full-looking tally.
- Why: all_go at :116 is (tier1_go == tier1_total); both counters are incremented only inside add_station, after the overflow guard at :19, so a dropped station appears on neither side of the comparison and the printed tally at :121-127 shows no sign of the truncation. The Tier-1 fail-open escalation does not hold today: 13 stations occupy 16 slots, and go_nogo_evaluate adds all eight Tier-1 stations (:84-102) before any Tier-2 one (:105-114), so the first station an overflow could drop is the trailing Tier-2 "Battery" entry -- a Tier-1 gate cannot be the victim without also reordering the block. What survives is the missing truncation signal in GoNoGoResult; go_nogo_checks.h:25-27 records that the 12 -> 16 bump was reasoned about only as "without truncating Tier-2 stations", which is the same blind spot.
- Confidence: high that the aggregate is computed over the truncated set; medium that the overflow recurs.
- Direction: Make overflow fail closed rather than silent -- record a truncation flag in GoNoGoResult that forces all_go false and gets printed, and/or bind the expected station count to kGoNoGoMaxChecks at compile time so the array cannot be outgrown without a build failure. The static-check idiom is already in house use next door at mission_profile_data.h:82-90.
- Verdict: RESHAPED -- The silent-drop mechanism at :19 and the counter blindness at :116 are exactly as described, but the Tier-1 fail-open escalation is not reachable: go_nogo_evaluate adds all eight Tier-1 stations (:84-102) before any Tier-2 one (:105-114), so the first victim of an overflow is the trailing Tier-2 entry, not a Tier-1 gate.

### CW-B22-02 -- Terminal else emits a "GO ..." reason for a NO-GO RF Link station
- Site: src/flight_director/go_nogo_checks.cpp:72-77 (ladder :55-77; consumed at :130-134)
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") and JSF AV 134 (undocumented/unenforced function assumption); with the Assertions JPL-16 entry-sanity-check angle on the unvalidated parameter range.
- Claim: The reason-string ladder treats "not 0, not 1, not 3" as "state == kTrack", so any unrecognised rf_link_state with rf_lq_pct >= 65 produces the operator-facing text "GO LQ nn%" on a station whose go flag is false.
- Why: link_go at :44-46 requires rf_link_state == 2 exactly, but the final else at :72 is reached for every value that is not 0, 1 or 3 -- including 4..255. go_nogo_print prints only stations with !c.go (:132), so the operator sees the NO-GO detail line "  RF Link: GO LQ 90%" during a pre-arm poll: the one place the text is displayed is the one place it contradicts the verdict. The input is a uint8_t flattening of rc::LinkState (go_nogo_checks.h:60-61), so 0..3 is a caller contract that nothing in this file checks. The shape that hides the gap is the four-way ladder at :59-76, which repeats the same assign / to_string / append("%") idiom four times (CCG ES.3, Fowler Duplicated Code) and therefore needs a catch-all else instead of an explicit kTrack branch.
- Confidence: medium -- the mislabel is certain, but reaching it requires an out-of-contract rf_link_state that the current caller does not produce.
- Direction: Test rf_link_state == 2 explicitly for the GO wording and give the unrecognised-state case its own NO-GO text (or check the 0..3 contract at entry). Hoisting the shared percentage-append step out of the four branches leaves a single prefix selection and removes the catch-all.
- Verdict: REFUTED -- rf_link_state is populated only at health_monitor.cpp:777 by casting rc::LinkState, a closed four-value enum (rf_link_health.h:26-31), so the terminal else at :72 is reachable only with state == 2, where link_go is true, "GO LQ nn%" is the correct wording, and go_nogo_print (:132) does not display the line at all; the claimed mislabel requires an input the only producer cannot emit.

### CW-B22-03 -- guard_evaluator_init leaves GuardState::sustained with no value
- Site: src/flight_director/guard_evaluator.cpp:19-28 (contract at guard_evaluator.h:76-77, field at guard_evaluator.h:67)
- Lens: Declaration scope & object lifetime -- JSF AV 142 ("All variables shall be initialized before use") and JSF AV 143 (introduced only when a meaningful value exists); this is the two-phase-init / value-not-yet-meaningful residual the lens marks as the human half, not the gate-covered used-before-read half.
- Claim: init_guard writes six of GuardState's seven fields and never writes sustained, so guard_evaluator_init does not fully initialise the object its header contract at :76-77 says it initialises, and nothing in the header records the dependence on zero-initialised storage.
- Why: guard_evaluator.cpp:22-27 assigns sustain_count, sustain_required, threshold, signal, valid_phases and fired; sustained (guard_evaluator.h:67) is left holding whatever the caller's storage held, and guard_evaluator_init does not call guard_evaluator_reset either. The stale-flag consequence does not survive scrutiny: init sets ev->last_phase = kIdle (:73), flight_director_evaluate_guards returns early for kIdle (flight_director.cpp:304-307), so the first guard_evaluator_tick in any guarded phase always sees phase != last_phase and runs guard_evaluator_reset (:124-127), which clears sustained for every guard before guard_combinator.cpp:92/101 can read it. What survives is the initialisation-completeness gap itself (JSF AV 142/143) on the one field the combinator layer reads through guard_evaluator_is_sustained (:84-86) for apogee and main-deploy dispatch, plus the header's silence about it.
- Confidence: high that the field is unwritten; medium on live impact, since today's static allocation masks it.
- Direction: Have the init path clear sustained. guard_evaluator_reset already clears exactly the three runtime fields, so invoking it at the end of guard_evaluator_init closes the gap without introducing a new code path or a second place to keep in sync.
- Verdict: RESHAPED -- The unwritten sustained field is real and verifiable, but the stale-flag consequence is not reachable: init sets last_phase = kIdle (:73), flight_director_evaluate_guards returns early for kIdle (flight_director.cpp:304-307), so the first tick in any guarded phase always takes the phase-transition branch at :124-127 and clears sustained for every guard before any combinator read.

### CW-B22-04 -- Phase bitmask is one bit too narrow for FlightPhase
- Site: src/flight_director/guard_evaluator.cpp:15-17 (consumers :129 and :139; field guard_evaluator.h:65)
- Lens: Assertions -- CCG P.5 ("Prefer compile-time checking to run-time checking"): a statically decidable layout constraint with no static_assert. Spine block C, the silently-wrong-on-an-unexercised-path type.
- Claim: phase_bit narrows 1U << phase to uint8_t while FlightPhase runs 0..8 (kFault = 8, flight_state.h:57), so phase_bit(FlightPhase::kFault) evaluates to 0 and GuardState::valid_phases cannot represent that phase at all.
- Why: 0 is also the value meaning "this guard is valid in no phase", so the two cases are indistinguishable at the test site (:139). Two consequences follow, neither of which raises a diagnostic. A guard declared valid in kFault would be permanently inactive with a clean build and green tests -- test/test_guards.cpp exercises only kIdle..kMainDescent, so nothing covers the boundary. And if guard_evaluator_tick is reached with phase == kFault, phase_mask is 0 and every guard goes inert for that tick; flight_director_evaluate_guards filters kIdle, kLanded and kAbort but not kFault (flight_director.cpp:302-307), and kFault is a real degraded-in-place phase with its own entry actions (flight_actions.h:142-144). That the intended kFault behaviour (PIO backup timers handle pyro autonomously, flight_actions.h:135-140) coincides with the truncated result is a coincidence, not a decision recorded here. The narrowing is an explicit static_cast, so no compiler or clang-tidy signal exists.
- Confidence: high on the truncation and the enum width; medium on whether kFault currently reaches this tick path.
- Direction: Add a compile-time guard that the phase count fits the bitmask width (static_assert relating FlightPhase::kCount to the bit width of valid_phases) so a tenth phase, or a guard declared valid in kFault, fails the build instead of going quiet; and state the 8-phase bound in the header beside valid_phases. If guards are meant to be inert in kFault, say so at the phase filter rather than relying on the narrowing.
- Verdict: CONFIRMED -- FlightPhase::kFault = 8 with kCount = 9 (flight_state.h:57-59), so the static_cast<uint8_t> at :16 makes phase_bit(kFault) == 0, indistinguishable from "valid in no phase" at the test site :139; kFault reaches this tick path because flight_director.cpp:302-307 filters only kIdle/kLanded/kAbort, and no compiler or clang-tidy check fires on an explicit narrowing cast.

### B23 -- flight_director: guard_combinator + guard_functions

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/flight_director/guard_combinator.h -- FAIL -- Read whole as one work product with the .cpp; it is the contract surface for the three-layer deployment architecture, and both its architecture claim (dual-channel redundancy) and its layer contract (Layer 1/2/3 plus Council A2) overstate what the implementation delivers.
- C:/Users/pow-w/Documents/RC-agent-walk/src/flight_director/guard_combinator.cpp -- PARTIAL -- Read whole; the functions are single-purpose and the shipped-profile control flow is eyeball-verifiable, but the init path asserts none of its preconditions, the phase bitmask cannot represent one of the nine flight phases, and the Council-A2 timer bypass is superseded by the confidence gate placed ahead of it.
- C:/Users/pow-w/Documents/RC-agent-walk/src/flight_director/guard_functions.h -- FAIL -- Read whole; six of the seven guard contracts state units, frames and thresholds precisely, but guard_baro_peak documents a sign convention that its only caller does not supply.
- C:/Users/pow-w/Documents/RC-agent-walk/src/flight_director/guard_functions.cpp -- FAIL -- Read whole with the header; pure, single-expression, side-effect-free predicates that match their header text, except guard_baro_peak whose polarity is inverted relative to the NED-down value actually passed to it.

#### Findings

### CW-B23-01 -- guard_baro_peak is true while ascending, not while descending
- Site: src/flight_director/guard_functions.h:50-57, src/flight_director/guard_functions.cpp:30-33
- Lens: Comments & documentation quality -- CCG NL.2 / JSF AV 134 ("if the comment and the code disagree, both are likely to be wrong"; assumptions documented in the function preamble), reinforced by spine block B (passes-tests-yet-wrong).
- Claim: The header declares "@param vert_vel Vertical velocity (m/s), positive = ascending" and "If vert_vel <= 0 for sustained period, rocket is descending", but the guard's only caller passes FusedState::vert_vel_eskf, which is NED down-positive, so the predicate vert_vel <= 0.0F is true while the vehicle is ascending and false while it is descending.
- Why: The producer is ao_logger.cpp:150 (fused.vert_vel_eskf = g_eskf.v.z, commented "NED down = positive descent"), eskf.h:45 declares v as NED velocity, and ao_logger.cpp:106-108 assigns the identical g_eskf.v.z to fused.vel_d. The call site is guard_evaluator.cpp:105-106. Concretely, in kCoast with the shipped rocket profile (mission_profile_data.h:39, apogee_require_both = true) kBaroPeak sustains about 200 ms after coast entry while the rocket is still climbing, and goes false the moment vel_d crosses zero at apogee -- so the apogee AND can only fire in the pre-apogee band vel_d in (-0.5, 0] and can never fire once the vehicle is descending. If that roughly 50 ms band is missed (noise resetting the 3-tick kApogeeVelocity sustain, or coast entered late), apogee detection is lost for the rest of the flight and deployment falls back to the 15 s coast_timeout_ms timer. With apogee_require_both = false (a supported profile option -- generate_profile.py APOGEE_BOTH; the HAB profile at test/test_hab_profile_data.h:34 uses it) the OR combinator instead fires SIG_APOGEE during ascent as soon as the velocity and time lockouts clear. The unit tests cannot catch this because they encode the same inverted convention: test/test_guards.cpp:76-81 asserts guard_baro_peak(-0.5f) is true as "descending", and test/test_guards.cpp:243 sets vert_vel_eskf = +10 while vel_d = -10, a state the single-source data path cannot produce.
- Confidence: high
- Direction: Decide which side is authoritative -- the producer's NED-down convention is the one the rest of FusedState uses, so the predicate almost certainly needs to test the descending sense of that field, and the header's "positive = ascending" line needs to be replaced with the actual frame statement. Either way the guard tests and the combinator tests encode the old convention and must be re-derived from the producer, not from the header.
- Verdict: CONFIRMED -- The producer chain is single-source and inverted relative to the header: ao_logger.cpp:150 assigns g_eskf.v.z (eskf.h:45, NED velocity) to vert_vel_eskf with the comment "NED down = positive descent", the same value ao_logger.cpp:108 assigns to vel_d, and guard_evaluator.cpp:105-106 hands it to guard_baro_peak, whose vert_vel <= 0.0F is therefore true while ascending -- and test_guards.cpp:243 sets vert_vel_eskf = +10 alongside vel_d = -10, a state the single-source path cannot produce.

### CW-B23-02 -- Apogee "dual-channel" claim rests on two guards reading one float
- Site: src/flight_director/guard_combinator.h:7-13, src/flight_director/guard_combinator.cpp:48-50
- Lens: Comments & documentation quality -- CCG NL.2 / JSF AV 131, plus spine block B confabulation (NIST AI 600-1: a confident justification, here an external-practice citation, that the body does not implement).
- Claim: The combinator header never discloses that the two guards it pairs for apogee, kApogeeVelocity and kBaroPeak, are both computed from the same ESKF field, so a reader of this contract surface alone can read profile.apogee_require_both as buying channel redundancy.
- Why: guard_combinator.cpp:48 puts GuardId::kApogeeVelocity and GuardId::kBaroPeak in one combinator and :49-50 selects AND when profile.apogee_require_both is set. guard_evaluator.cpp:103-106 feeds fused.vel_d to one leg and fused.vert_vel_eskf to the other, and ao_logger.cpp:108 and :150 assign g_eskf.v.z to both, so a single bad ESKF velocity corrupts both legs identically and AND mode is a band-pass on one signal rather than a two-channel vote. guard_functions.h:52-54 discloses this for kBaroPeak ("NOT independent of ESKF ... planned but not yet implemented"); the combinator header carries no equivalent caveat and no cross-reference to it. The stronger reading does not survive: the banner line at :12-13 is a list of three industry precedents ("Altus Metrum velocity lockout + apogee lockout, Featherweight dual-channel, NASA timer-primary") offered for the three-layer architecture as a whole, not an assertion that this combinator is dual-channel.
- Confidence: high
- Direction: Qualify the banner so the header states what redundancy the shipped combinator actually provides (both legs derive from ESKF velocity; the independent baro channel is pending), and cross-reference the guard_functions.h:52-54 caveat from the combinator header so the two contract surfaces stop disagreeing.
- Verdict: RESHAPED -- The single-source dependence of both apogee legs is real and undisclosed in this header, but the banner reading is overstated: :12-13 lists three industry precedents for the three-layer architecture as a whole and is not itself a dual-channel claim about the apogee combinator.

### CW-B23-03 -- Confidence gate precedes Layer 3, so the Council-A2 timer bypass is unreachable in the case it exists for
- Site: src/flight_director/guard_combinator.cpp:129-133 and :143-155; contract text at src/flight_director/guard_combinator.h:75-78
- Lens: Comments & documentation quality -- JSF AV 134 (documented assumptions and limitations) and CERT MSC12-C (documentation describing a path the code effectively cannot take).
- Claim: The header's layer contract never mentions the confidence gate, and the Council-A2 comment says an unhealthy ESKF bypasses the velocity lockout for the timer backup, but the unconditional early return on !lockout.confident at :131-133 sits ahead of Layer 3, and an unhealthy ESKF is itself one of the confidence gate's own inputs.
- Why: flight_director.cpp:257 sets lockout.eskf_healthy from health_eskf(...) >= kHealthDegraded, and health_monitor.cpp:199-207 maps !ESKF::healthy() to kHealthFault; confidence_gate.cpp:22 makes input.eskf_healthy a conjunct of the gate's good condition, with kLossDebounceMs = 500 and kRecoveryDebounceMs = 2000 (confidence_gate.h:58-59). So on any path where !lockout.eskf_healthy holds, lockout.confident goes false after about 500 ms and stays false for at least 2 s after recovery -- meaning the "(!vel_locked || !lockout.eskf_healthy)" disjunct at :146-147 can only be exercised inside that 500 ms debounce window, and the timer backup is disabled for the remainder of a sustained ESKF failure. A reader of guard_combinator.h:75-78 cannot determine this: the header describes Layer 3 as firing "if sensors don't" and names only the lockouts and the A2 bypass as its gates.
- Confidence: high
- Direction: Resolve the two council decisions explicitly in the header -- either state that the confidence gate supersedes Council A2 and that Layer 3 is intentionally inert while uncertain (naming the PIO backup timers as the surviving channel), or, if A2 is meant to survive, scope the confidence-gate block to the sensor layer and say so. Either way the header's layer list needs the confidence gate in it.
- Verdict: CONFIRMED -- The whole chain verifies: flight_director.cpp:257 sets eskf_healthy from health_eskf >= kHealthDegraded (kHealthFault = 0b01 < kHealthDegraded = 0b10, health_monitor.h:28-31), evaluate_eskf returns kHealthFault on !ESKF::healthy(), eskf_runner.cpp:473 feeds that same ESKF::healthy() into confidence_gate.cpp:22 with kLossDebounceMs = 500 / kRecoveryDebounceMs = 2000 (confidence_gate.h:57-58), and the unconditional early return at :131-133 sits ahead of the Council-A2 disjunct at :146-147 that exists for exactly that condition -- none of which the header's layer list at :75-78 mentions.

### CW-B23-04 -- Phase bitmask cannot represent FlightPhase::kFault, and the truncation is silent
- Site: src/flight_director/guard_combinator.cpp:16-18 (used at :54, :64, :171)
- Lens: Assertions -- CCG P.5 ("Prefer compile-time checking to run-time checking"); spine block A ES.3 for the duplicated copy.
- Claim: phase_bit() narrows 1U shifted by the phase value into a uint8_t, but FlightPhase runs to kFault = 8 with kCount = 9 (flight_state.h:57-59), so phase_bit(kFault) evaluates to 0 with no diagnostic, and nothing ties the mask width to the phase count.
- Why: An 8-bit valid_phases field (guard_combinator.h:51) can encode only phases 0-7. Any future combinator declared valid in kFault would receive valid_phases == 0 and be silently inactive forever, and combinator_set_evaluate() called with phase == kFault computes phase_mask == 0 so every combinator is skipped -- today the conservative behavior one would want, but reached by accidental truncation rather than by a stated rule, so a later widening of FlightPhase past 8 breaks the encoding with no build-time signal. The static_cast makes the narrowing explicit, so no compiler or tidy check flags it, and the identical helper is duplicated verbatim at guard_evaluator.cpp:15-16, so the same defect has to be found twice.
- Confidence: high on the mechanism; the consequence today is latent
- Direction: Add a compile-time guard tying the mask width to FlightPhase::kCount so a ninth phase fails the build rather than silently disappearing, and hoist the single phase_bit definition to one header so the combinator and the evaluator share it.
- Verdict: CONFIRMED -- phase_bit at :16-18 narrows 1U << 8 to 0 for kFault (flight_state.h:57-59), valid_phases is uint8_t (guard_combinator.h:51), nothing ties the mask width to kCount, and the identical helper really is duplicated at guard_evaluator.cpp:15-17 so the same defect has to be found twice.

### CW-B23-05 -- init_combinator clamps the copy but not the count, with no entry preconditions anywhere in the file
- Site: src/flight_director/guard_combinator.cpp:31-40 (specifically :33-34) and :42-67 (:51, :61)
- Lens: Assertions -- JPL-C Rule 16 (sanity checks / parameter validity) and Power of Ten Rule 7 (parameter validity must be checked inside each function).
- Claim: init_combinator applies the four-element array bound to the id-copy loop at :34 but not to the c.num_guards assignment at :33, so the recorded guard count and the number of ids actually copied can disagree.
- Why: The author wrote the bound into the loop condition ("i < spec.n && i < 4") and not into "c.num_guards = spec.n" one line above, so a spec with n greater than 4 would yield num_guards greater than 4 with only four ids copied, and evaluate_sensors (:91, :100) would iterate past the four-element c.guard_ids (guard_combinator.h:47). The escalation to a live out-of-bounds read feeding a pyro-bearing decision does not hold: both in-tree call sites construct the CombinatorSpec inline at :52 and :62 with n = 2 and n = 1, and combinator_set_init writes exactly two hard-coded combinators against kMaxCombinators = 4, so neither bound is reachable from any input. What survives is the internal inconsistency inside one function -- the bound is written where it is enforced and not where it is recorded.
- Confidence: medium (real defect shape, currently unreachable from the two in-tree call sites)
- Direction: Make the count and the copy share one bound -- clamp or reject spec.n against the array extent rather than a bare literal -- and add the entry sanity checks the file lacks (combinator index below kMaxCombinators, guard count within the array, non-null state pointers), preferring a compile-time check where the spec is a constant.
- Verdict: RESHAPED -- The count/copy inconsistency at :33-34 is a genuine internal defect, but the out-of-bounds-read and missing-precondition escalation is unreachable: both call sites build the spec inline at :52 and :62 with n = 2 and n = 1, and two hard-coded combinators cannot reach kMaxCombinators = 4.

### B24 -- flight_director: mission profile + generated profile data

#### Coverage
src/flight_director/mission_profile.h -- FAIL -- Read whole as a contract surface (helper Kind E/C): units, PRELIMINARY warnings and per-field intent are good, but the header's central claim about how the active profile is selected does not match the firmware, and three cfg-documented options have no field.
src/flight_director/mission_profile_data.h -- FAIL -- Read whole; every generated field, the include guard and the GENERATED_FROM sha256 were checked against scripts/generate_profile.py and profiles/rocket.cfg and match, but the file also carries hand-edits its own banner forbids, and its compile-time validation block omits the two autonomous pyro timers.

#### Findings

### CW-B24-01 -- Profile-selection contract described in the header does not exist
- Site: src/flight_director/mission_profile.h:26 (also :39 and :131)
- Lens: Comments & documentation quality -- JSF AV 131/134, CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine block B (confabulation -- a confident claim the code does not implement).
- Claim: The header states the profile ID is "stored in flash, selects active profile at boot" and that the profile is "read from flash at boot, immutable for session", but nothing stores, reads or selects a profile ID, and the active profile is a compile-time constant pulled in unconditionally at line 131.
- Why: The MissionProfile.id field is read nowhere in src/ (only test/test_mission_profile.cpp:24 and test/test_flight_director.cpp:541); both firmware consumers hard-reference the single symbol (src/active_objects/ao_flight_director.cpp:258, src/main.cpp:400); there is no profile region in flash and no CLI selector -- the CLI only prints the name (src/cli/rc_os_commands.cpp:851-852). Meanwhile ProfileId offers kRocket/kHab/kFreeform with kCount = 3, and the sibling radio config really does have flash persistence (src/logging/radio_config_storage.h), so the assumption the comment invites is a plausible one. A reader or operator who believes it will expect a HAB profile to be selectable by writing a flash value; the firmware will always fly the rocket profile's lockouts, backup timers and pyro settings, silently.
- Confidence: high
- Direction: State the truth -- the active profile is fixed at compile time by the include at line 131 -- and either mark ProfileId / the id field as reserved-for-future-selection or name what actually consumes it.
- Verdict: CONFIRMED -- MissionProfile::id is assigned once at mission_profile_data.h:15 and read nowhere in src/; both firmware consumers hard-reference kDefaultRocketProfile (ao_flight_director.cpp:258, main.cpp:400), the CLI only prints .name (rc_os_commands.cpp:851-852), and the active profile arrives through the unconditional include at mission_profile.h:131 -- so the flash-selection contract at :26 and :39 describes machinery that does not exist.

### CW-B24-02 -- "Do not edit / re-run the generator" banner sits over hand-edited content; following it silently drops the MAVLink build option
- Site: src/flight_director/mission_profile_data.h:1-5, 95-99, 105-106
- Lens: Comments & documentation quality -- JSF AV 131/134, CCG NL.2 (comment/code disagreement); spine block B (spec-noncompliance). Project rule: CODING_STANDARDS "Auto-Generated Code".
- Claim: The banner declares the file generator output that must not be edited, but lines 95-99 and 105-106 cannot be produced by scripts/generate_profile.py, so the header is a hand-edited artifact whose own stated maintenance procedure deletes firmware behaviour.
- Why: The generator emits .protocol unconditionally (scripts/generate_profile.py:391 and :400-408) and emits nothing between .coding_rate and the closing brace; the committed file adds an ifdef ROCKETCHIP_STAGE_T3_MAVLINK protocol switch plus a Stage-T sweep comment. That ifdef is the only consumer of the CMake option at CMakeLists.txt:820-822, so anyone following the banner (edit the .cfg, re-run the generator, rebuild) removes it: a build configured with -DROCKETCHIP_STAGE_T3_MAVLINK=ON still prints "Stage T3 protocol: MAVLink" (CMakeLists.txt:823) while the vehicle transmits CCSDS -- vehicle and station fall out of protocol agreement with no diagnostic. Nothing detects this: the generator is not wired into the build (no reference to generate_profile or mission_profile_data in CMakeLists.txt or cmake/), and the GENERATED_FROM sha256 on line 2 hashes the input cfg, which I recomputed and it still matches (e1c22265fc444258) -- so the traceability line reads green while the output has drifted from it.
- Confidence: high
- Direction: Make the two hand-patches reproducible -- carry the protocol switch into the .cfg plus generator, or move the compile-time switch out of the generated file into a hand-owned header -- and either wire regeneration into the build or add an output-side hash check so drift is detectable rather than latent.
- Verdict: CONFIRMED -- generate_profile.py:391 computes protocol_enum unconditionally and :400-408 emits .mode through .coding_rate with nothing between it and the closing brace, so the #ifdef at mission_profile_data.h:95-99 and the sweep comment at :105-106 cannot be generator output; that #ifdef is the sole consumer of the CMake option (CMakeLists.txt:820-822), and no CMake rule regenerates the header, so following the banner silently deletes the MAVLink build path.

### CW-B24-03 -- The two autonomous pyro backup timers are the one profile group bounded at no layer
- Site: src/flight_director/mission_profile_data.h:47-48 and :82-91; declared contract at src/flight_director/mission_profile.h:110-112
- Lens: Assertions -- P10 Rule 5 / JPL-C Rule 16 (meaning clause: check the anomalous condition that should never happen) and CCG P.5 (prefer compile-time checking to run-time checking).
- Claim: drogue_timer_s and main_timer_s feed a hardware pyro path with no range check in the generator and no static_assert in the generated header, while five less consequential fields carry both.
- Why: In the generator's FIELDS table the bounds for DROGUE_TIMER_S and MAIN_TIMER_S are None (scripts/generate_profile.py:72-73), and the emitted validation block (mission_profile_data.h:82-91) covers launch accel, main altitude, armed timeout, landing sustain and ramp steps but neither timer. These values are handed to pio_backup_timer_arm() and fire pyro GPIOs autonomously from PIO with no ARM core in the loop, so no runtime software gate can reject a bad one. The .cfg the user is told to edit warns for exactly this input -- "Setting too low fires drogue during boost which will shred the chute" (profiles/rocket.cfg:194-197) -- and nothing enforces it: a mistyped DROGUE_TIMER_S of 1.5, or a swapped pair where main_timer_s is less than drogue_timer_s (main deployed before drogue, at speed), compiles clean and arms the hardware. Both conditions are statically decidable here, next to a static_assert block that already exists for weaker facts.
- Confidence: medium
- Direction: Add static_asserts alongside the existing block -- both timers non-negative, and when both are non-zero main_timer_s greater than drogue_timer_s -- and give the two fields real bounds in the generator's FIELDS table so a bad .cfg fails before the build.
- Verdict: CONFIRMED -- DROGUE_TIMER_S and MAIN_TIMER_S carry bounds None at generate_profile.py:72-73 while the five fields covered by the static_assert block at mission_profile_data.h:82-91 all carry generator ranges too (:33, :37, :44, :47, plus the QR_RAMP_STEPS check at :214-223); pio_backup_timer_arm (pio_backup_timer.cpp:66-86) performs no range validation, and profiles/rocket.cfg:194-196 warns about exactly the failure that nothing enforces.

### CW-B24-04 -- Three cfg options documented as profile settings have no field in the profile contract
- Site: src/flight_director/mission_profile.h:110-112 (and the struct as a whole, :41-125)
- Lens: Comments & documentation quality -- JSF AV 134 (document assumptions and limitations); contract-surface helper Kind E (is this the single, complete map?).
- Claim: DROGUE_TIMER_ACTION, MAIN_TIMER_ACTION and SAFE_MODE_ACTION are documented user-facing keys in profiles/rocket.cfg, but MissionProfile carries no corresponding field and neither the struct nor its comments say they are unimplemented.
- Why: profiles/rocket.cfg:199-204 and :216-224 present all three as configurable, and src/safety/pio_backup_timer.h:13-16 asserts "Timer actions are profile-configurable". The generator's FIELDS table (scripts/generate_profile.py:28-77) has no entry for any of them, so they fall into the unknown-field branch (:247-249) and are dropped with a stdout warning from a manual step nobody is required to run. The config wizard actively emits SAFE_MODE_ACTION (scripts/config_wizard/core/cfg_emitter.py:78), so a user who picks fault behaviour 1 or 2 -- radio recovery, or auto-reset, which the cfg itself calls out as dangerous -- gets no behavioural change and no error. Nothing in the contract header tells a reader which cfg keys are load-bearing and which are inert.
- Confidence: medium
- Direction: Either add the fields (plus their generator entries) or state in the profile header and the .cfg which keys are reserved-not-implemented; separately, make the generator fail rather than warn on an unknown key.
- Verdict: CONFIRMED -- profiles/rocket.cfg:202, :204 and :224 present all three keys as configurable and pio_backup_timer.h:13-16 calls timer actions "profile-configurable", but none appears in FIELDS/QR_FIELDS/RADIO_FIELDS, so all three hit the ignore-with-warning branch at generate_profile.py:247-249, and cfg_emitter.py:78 plus derivation.py:296 mean the wizard actively writes a key that has no effect.

### CW-B24-05 -- Council citations on the two safety-parameter groups do not resolve in the repo
- Site: src/flight_director/mission_profile.h:77 and :84
- Lens: Comments & documentation quality -- CCG NL.3 (point to the spec rather than paraphrase; a pointer has to resolve) and JSF AV 134; project citation discipline, LESSONS_LEARNED Entry 37.
- Claim: "Safety lockouts (Council A1)" and "Timer backups (Council A6)" name amendments of a flight-director council whose decisions no in-repo document records, so neither PRELIMINARY value can be traced to a rationale.
- Why: The labels belong to the council that guard_combinator.h:15 names -- "Council review 2026-03-25: 6 amendments incorporated (A1-A6)" -- but nothing under docs/ records it: a grep for 2026-03-25 across docs/ returns no file, and docs/plans/STAGE8_FLIGHT_DIRECTOR.md, which does hold the Amendment #5 and #7 rationale correctly cited from this same header at :46 and :48, has no lockout or timer-backup content. The other in-repo A1/A6 mentions (docs/AO_ARCHITECTURE.md:266 and :272, docs/decisions/AO_COMMANDMENTS.md:53 and :79, docs/decisions/NOTIFY_CONTRACT.md:209, docs/plans/STAGE_T_T14_DESIGN.md:292) belong to a different AO-council numbering -- ESKF-as-a-module, Core-1 vitality fallback, read-only accessor pattern -- and none of them sources these parameters either. Consequence: a reviewer asked to validate deploy_lockout_mps = 80 m/s or main_backup_ms = 120000, both flagged PRELIMINARY at :80 and :86, has no rationale to trace; the only quantitative record found (docs/decisions/OPENROCKET_INTEGRATION_EVALUATION_2026-05-22.md:221-222) checks the numbers against a sim rather than sourcing them.
- Confidence: medium
- Direction: Replace the bare labels with a resolvable pointer (decision or plan document plus section), or demote them to plain rationale prose stating why the lockout and the backup timer exist.
- Verdict: RESHAPED -- No in-repo document records the flight-director council these labels belong to (a grep for 2026-03-25 across docs/ returns nothing), so the substance holds, but the finding missed guard_combinator.h:15, which does date that council and state it carried six amendments (A1-A6), and its "the only Council A1 / A6 records" enumeration is incomplete (NOTIFY_CONTRACT.md:209 and STAGE_T_T14_DESIGN.md:292 also carry A1/A6 labels, from the unrelated AO-council numbering).

### B25 -- logging: rc_log sink + ring_buffer

#### Coverage
src/log/rc_log.cpp -- FAIL -- Read whole (732 lines); spine A/B/C run on all 18 functions, plus the comments, control-flow/volatile and concurrency lenses on the target_sink ring (5 volatile objects, the itinerary hot-spot cue); the paired contract include/rocketchip/rc_log.h was read for comment-truth only (it is its own itinerary row, not counted here); three findings.
src/logging/ring_buffer.cpp -- FAIL -- Read whole (169 lines); spine on all 9 functions; control-flow/ordering lens on the seqlock header write and scope/lifetime lens on the init path; two findings.
src/logging/ring_buffer.h -- PARTIAL -- Read whole (153 lines); evaluated as a contract surface (helper Kind B shared protocol plus Kind C API prose): the seqlock protocol claim and the documented init-then-recover sequence are both unsupported by the implementation (CW-B25-04, CW-B25-06), and the "Single-writer design -- no locking required" banner names neither the owning context nor the reader story (verified true today: ring_push from AO_Logger and ring_read/ring_stored_count from the CLI are both Core 0 QV-cooperative -- claim thin, not wrong).

#### Findings

### CW-B25-01 -- rc_log ring's single-context safety argument is contradicted by an in-tree fault-handler producer
- Site: src/log/rc_log.cpp:446-455 (the claim), 481-490 and 500-525 (the unprotected state), contradicted by src/safety/fault_protection.cpp:212
- Lens: Concurrency & shared-data ownership (CCG CP.8, JPL-C Rule 8) with Control-flow discipline (JSF AV 205); comment-truth NL.2
- Claim: rc_log.h:44-46 explicitly PROHIBITS rc_log from "ISR / fault-handler context", and rc_log.cpp:446-455 builds its no-barrier, volatile-only ring argument on exactly that contract ("rc_log is called from Core 0 cooperative context only ... never from ISR, never from Core 1 ... Producer and consumer never run concurrently") -- but Q_onError(), the QP assertion handler, calls rc::rc_log() at fault_protection.cpp:212, and Q_onError is reachable from the 100 Hz QF tick ISR (bsp_qv.c:40-42 -> QTIMEEVT_TICK_X -> QActive_post_ -> Q_ASSERT_INCRIT(130, ...) at qf_actq.c:76; LL Entry 32 records exactly this firing).
- Why: The invariant is false as stated, and it is the entire justification for protecting g_ring / g_head / g_tail / g_droppedBytes with plain volatile instead of any barrier: the tick ISR can preempt Core 0 anywhere, including between g_ring[h] = buf[i] (line 518) and g_head = (h + 1U) % kRingBytes (line 519), or inside rc_log_drain_to_cdc() after t = g_tail (line 689). The damage is bounded, though, and narrower than a general data race: Q_onError masks interrupts on entry (fault_protection.cpp:178) and is Q_NORETURN, so the preempted producer or consumer never resumes -- there is no interleaved-resume corruption of the indices and no accumulating loss of drop counts. What actually results is a partially written log line left in the ring immediately ahead of the "[QP ASSERT]" message, plus the loss of the interrupted producer's own g_droppedBytes increment if it was mid-eviction. The finding that survives is comment-truth plus contract-violation: the file's stated reason for choosing volatile over atomics or a critical section is not satisfied, the header's own PROHIBITED list is violated by an in-tree caller, and the violating caller is the fault path -- the moment the ring's contents matter most. The volatile qualifiers also remain unjustified under JSF AV 205 (this is not MMIO), which is what forces the const_cast/reinterpret_cast at lines 696-697.
- Confidence: high
- Direction: Either enforce the stated contract (no rc_log from Q_onError -- capture module/id into the CrashRecord that handler already writes at fault_protection.cpp:191-203), or accept a preemptive producer and give the ring a real mechanism: single-producer/single-consumer indices as std::atomic<size_t> with documented acquire/release, or a short interrupts-disabled critical section around the index updates. Under either fix the volatile qualifiers become unnecessary (they are not MMIO, so JSF AV 205 is unsatisfied today), and dropping them also removes the const_cast/reinterpret_cast at lines 696-697 that currently reads a volatile-defined array through a non-volatile lvalue.
- Verdict: RESHAPED -- The contract violation is real and verified (rc_log.h:44-46 forbids fault-handler logging; Q_onError calls rc_log at fault_protection.cpp:212 and is reached from the 100 Hz QF tick ISR), but the data-corruption consequences are overstated because Q_onError is Q_NORETURN and masks interrupts on entry, so the preempted producer/consumer never resumes.

### CW-B25-02 -- truncation marker is silently omitted when a conversion exactly fills the buffer
- Site: src/log/rc_log.cpp:555-561 (post-conversion check), contrast src/log/rc_log.cpp:390-408 (buffer_append); reached via src/log/rc_log.cpp:169 and 173
- Lens: Comments & documentation quality (JSF AV 131/134, CCG NL.2 -- comment and code disagree) with spine block B spec-noncompliance
- Claim: A conversion that fills the output string to capacity leaves no room for the "...\n" marker, so the message is truncated with no marker at all -- contradicting the locked contract at rc_log.h:20-23 that truncation always appends an explicit marker "within the 128-byte budget".
- Why: buffer_append (used for literal format text) reserves kTruncMarkerLen before appending, so literal-text truncation is always marked. The conversion helpers do not: format_string calls out.append(s, slen) directly (lines 169 and 173), and ETL's fixed-capacity string clamps to capacity and sets its own truncated flag (EXTERNAL/etl-20.47.1/etl/basic_string.h:496 and 811) rather than overflowing. So for rc_log("%s\n", s) with an s of 128 or more characters, out.size() reaches 128 == capacity; the guard at line 555 fires, line 556 computes avail = 0, line 557's avail >= kTruncMarkerLen is false, and the function returns having emitted a truncated line indistinguishable from a complete one. The same holds for any %f or %d given a large width. rc_log.h:22-23 states callers "do NOT detect their own truncation" -- the marker is the only signal that truncation occurred, and this path deletes it. rc_log.cpp never consults the sink's own out.is_truncated(), which would have detected it.
- Confidence: high
- Direction: Reserve the marker budget once, before formatting -- format into a working string whose usable capacity is capacity minus kTruncMarkerLen so every path leaves room -- or replace the hand-rolled capacity arithmetic at lines 555-561 with a single post-format check on out.is_truncated() that overwrites the last four bytes with the marker.
- Verdict: CONFIRMED -- Verified line by line: ETL clamps an over-long conversion to exactly capacity, and handle_percent_token's guard at rc_log.cpp:555-557 then computes avail == 0 and skips the marker, so any conversion that overflows the 128-byte buffer truncates unmarked -- contradicting the locked contract at rc_log.h:20-23, and rc_os_commands.cpp:917's 128-character format string is a live overflow candidate.

### CW-B25-03 -- format_float has no magnitude domain guard; large finite values convert out of range
- Site: src/log/rc_log.cpp:266-273, with the limitation absent from the preamble at src/log/rc_log.cpp:177-195
- Lens: Comments & documentation quality (JSF AV 134 -- undocumented function assumption/limitation) with spine block B (passes-tests-yet-wrong: unexercised boundary input)
- Claim: format_float scales the value by 10^precision and converts nearbyint() of the product to unsigned long long (rc_log.cpp:268-269) with no check that the product is representable, so any |value| * 10^prec at or above 2^64 is an out-of-range floating-to-integer conversion -- undefined behaviour in C++ -- and the 18-line preamble at rc_log.cpp:177-195 enumerates the special cases the function does handle (NaN, Inf, negative zero, round-half-to-even) without ever stating the representable magnitude domain.
- Why: Line 268 computes nearbyint(abs_v * scale) and line 269 does static_cast<unsigned long long>(rounded). A conversion whose truncated value is not representable in the destination integer type is undefined behaviour in C++; on the ARM target VCVT saturates, so the practical failure mode would be a silently plausible wrong number rather than a diagnosable fault. The break-even is roughly 1.8e13 for %.6f, 1.8e15 for %.4f and 1.8e12 for %.7f. No in-tree callsite is shown to reach those magnitudes: the %.7f sites are GPS lat/lon (rc_os_commands.cpp:325, 349, 353), bounded by +/-180, and the %.6f sites are the Pab/Pgb covariance diagonals (rc_os_commands.cpp:222), which the filter's own std::isfinite and positive-definiteness guards (eskf.cpp:614, 633) and its P-growth reset bound in practice. The claim that stands unaided is the documentation one: the preamble sets a byte-for-byte-libc acceptance criterion and explicitly enumerates the special cases it handles, which makes the omission of the magnitude domain read as covered when it is not. denom at line 271 overflows the same way for a precision above 19, though no in-tree format string uses one.
- Confidence: medium
- Direction: Add an explicit domain guard before line 269 -- if rounded exceeds the largest exactly-representable unsigned long long, emit a documented out-of-range rendering rather than converting. At minimum state the representable domain as a limitation in the preamble alongside the NaN/Inf cases, so callers know which values the formatter is not contracted for.
- Verdict: RESHAPED -- The missing representability guard and the undocumented magnitude domain are real, but no in-tree callsite is shown to reach the ~1.8e13 threshold (the %.7f sites are bounded lat/lon and the %.6f covariance sites are bounded by the ESKF's own isfinite/P-growth guards), so the defensible finding is the undocumented limitation, not a demonstrated wrong number.

### CW-B25-04 -- the crash-recovery seqlock is described but not implemented; the odd-seq store is a removable dead store
- Site: src/logging/ring_buffer.cpp:21-35 (implementation), claim at src/logging/ring_buffer.h:41-48 and 13-21
- Lens: Control-flow discipline (JSF AV 205 / CCG CP.8 -- ordering is not provided by plain stores) with Comments & documentation quality (CCG NL.2 -- "if the comment and the code disagree, both are likely to be wrong")
- Claim: sync_header writes hdr->seq odd, then the payload, then hdr->seq even through a plain RingHeader*, with no volatile, no atomic and no barrier -- nothing makes that ordering observable, and the first store to hdr->seq is dead-store-eliminable.
- Why: The header states the protocol as a correctness mechanism ("1. Write seq = odd (mark inconsistent) 2. Write head_offset and frame_count 3. Write seq = even") and ring_recover rejects odd seq at ring_buffer.cpp:147 on the strength of it. But lines 27-34 are five ordinary stores to members of one non-volatile object with no intervening read of hdr->seq and no call or barrier between them, so the compiler may legally drop line 27 and may sink or reorder lines 30-32 relative to it. If line 27 is elided, a crash between lines 30 and 34 leaves hdr->seq holding the previous sync's even value beside a half-updated head_offset/frame_count, and ring_recover accepts the torn header as consistent -- the exact outcome the seqlock exists to prevent. RingHeader is also declared packed (ring_buffer.h:50), so the compiler cannot assume alignment and may lower each 32-bit member store to byte stores, allowing seq itself to tear. The tree already contains the correct pattern: memmanage_fault_handler writes its crash-record payload, issues a dsb memory barrier, writes the magic last, then another dsb (fault_protection.cpp:141-144).
- Confidence: high
- Direction: Make the ordering real -- write the header through a volatile RingHeader* (this one is a genuine ordering-bearing memory view, not a software convenience) or place explicit dsb barriers after the odd-seq store and after the payload stores, mirroring fault_protection.cpp:141-144. While there, state in the header whether the caller-supplied pointer must already be the uncached PSRAM alias that "Council req. #1" (ring_buffer.h:23-26) requires, since the .cpp neither checks nor asserts it.
- Verdict: CONFIRMED -- sync_header (ring_buffer.cpp:21-35) issues five plain stores through a non-volatile RingHeader* with no barrier and no intervening observable access, so the odd-seq store at :26 is dead-store-eliminable and the protocol the header states at :41-48 -- and that ring_recover relies on at :147 -- is not implemented.

### CW-B25-05 -- a failed ring_init leaves the object marked initialized, so ring_push overruns the caller's memory
- Site: src/logging/ring_buffer.cpp:51-53, exploited at src/logging/ring_buffer.cpp:60-70
- Lens: Declaration scope & object lifetime (JSF AV 143, CCG P.8 -- two-phase object operable after a failed second phase) with spine block B (passes-tests-yet-wrong)
- Claim: ring_init sets rb->initialized = true at line 51 and only then returns false for max_frames == 0 at line 53, so the rejected object still passes every !rb->initialized guard in the API.
- Why: For ring_init(rb, mem, 16 + frame_size - 1, frame_size, div) -- the case test/test_ring_buffer.cpp:78-84 ("InitRejectsTooSmall") already exercises -- capacity is frame_size - 1 and max_frames is 0, the function returns false, and rb.initialized is left true. A caller that drops the return then calls ring_push: the guard at line 61 passes, line 64 memcpys frame_size bytes into a data region of frame_size - 1 bytes, overrunning the caller-supplied buffer, and line 68's rb->head >= rb->max_frames * rb->frame_size reduces to head >= 0, so head resets to 0 and the overrun repeats at the same address on every push. The existing test asserts only the return value; the success case at line 59 asserts EXPECT_TRUE(rb.initialized) but no test asserts the failure case leaves it false, so the state bug is green today. The production caller (ao_logger.cpp:270-274) does check the return, so this is a latent contract defect, not a live overrun.
- Confidence: high
- Direction: Move the max_frames == 0 check up with the other argument guards at lines 39-42, before any field of rb is written, so a rejected init leaves the object untouched and initialized false. Mark the five bool-returning entry points in ring_buffer.h [[nodiscard]] -- the project's P10 Rule 7 gate only fires where the attribute is present and none of them carry it -- and extend InitRejectsTooSmall to assert rb.initialized == false.
- Verdict: CONFIRMED -- ring_init sets rb->initialized = true at :51 before the max_frames == 0 rejection at :53, so a rejected object still passes every !initialized guard; InitRejectsTooSmall asserts only the return value, and no [[nodiscard]] or bugprone-unused-return-value.CheckedFunctions entry gates the discarded return.

### CW-B25-06 -- the documented init-then-recover sequence destroys the header ring_recover reads, and recover then reports success
- Site: src/logging/ring_buffer.h:85 and 137-141 (documented sequence), src/logging/ring_buffer.cpp:55-57 and 138-159 (implementation)
- Lens: Comments & documentation quality (CERT MSC12-C -- doc describing a path the system cannot take; CCG NL.2)
- Claim: ring_recover's preamble instructs "Call after ring_init() to attempt recovery", but ring_init writes a fresh zeroed header at line 56 before returning, so by the time ring_recover runs the prior header is gone -- and recover then validates the freshly written header, returns true, and restores head = 0, frame_count = 0.
- Why: A caller following the header verbatim gets a false positive rather than a failure: magic matches (just written by sync_header), seq is even (just written), and head_offset == 0 passes both the bounds check at line 151 and the alignment check at line 154, so line 159 returns true having recovered nothing. The caller cannot distinguish "resumed N frames after a watchdog reset" from "fresh boot", which is the one distinction the whole RingHeader mechanism exists to make. The test suite confirms the sequence does not work: test/test_ring_buffer.cpp:283-292 hand-populates all eight RingBuffer fields with the explicit comment "Manually set struct fields without overwriting memory header" instead of calling ring_init, i.e. the only passing recovery test deliberately bypasses the documented API. No firmware caller invokes ring_recover at all -- ao_logger.cpp:270 calls only ring_init -- so the recovery path described across ring_buffer.h:13-21, 41-48 and 132-141 has never run on target, while sync_header still costs a header write every kHeaderSyncDiv frames on the logging path to maintain it.
- Confidence: high
- Direction: Split the two responsibilities -- give ring_init a mode (or add a ring_attach) that populates the runtime struct without writing the header, so recover can read what a prior run left, and have ring_recover distinguish "restored prior state" from "header was fresh". Then either wire recovery into AO_Logger's PSRAM path or mark the mechanism explicitly unused in the header so the next reader does not trust a path nothing exercises.
- Verdict: CONFIRMED -- ring_init writes a fresh header via sync_header at :56, so a caller following ring_buffer.h:137 gets magic/even-seq/offset-0 all valid and ring_recover returns true having restored nothing; the only passing recovery test hand-populates the struct instead of calling ring_init, and no firmware caller invokes ring_recover at all (ao_logger.cpp:270 calls only ring_init).

### B26 -- logging: flash_flush + psram_init

#### Coverage
- C:/Users/pow-w/Documents/RC-agent-walk/src/logging/flash_flush.cpp -- FAIL -- Every function read whole and spine-walked (flash trampolines, dual-sector table I/O, sector flush engine, flush_ring_to_flash); capacity math, sector indexing and header size verified against flight_table.h / flash_layout.h / pcm_frame.h; three findings.
- C:/Users/pow-w/Documents/RC-agent-walk/src/logging/flash_flush.h -- PASS -- Contract surface (Kind C) walked per the helper: the load-bearing claims check out (5 s watchdog matches main.cpp:90, xip_cache_clean_all present at flash_flush.cpp:349, 64-byte header static_asserted at pcm_frame.h:180, every FlushResult member is actually returned somewhere in the body).
- C:/Users/pow-w/Documents/RC-agent-walk/src/logging/psram_init.cpp -- FAIL -- Every function read whole and spine-walked (detect, timing calc, QMI config, self-test, accessors, flash-safe test); the register pokes were judged as an init sequence and an interrupt/XIP window, not line by line; four findings.
- C:/Users/pow-w/Documents/RC-agent-walk/src/logging/psram_init.h -- FAIL -- Contract surface (Kind C/E) walked per the helper: the memory-map constants and accessor promises are sound, but two documented preconditions/promises do not match the code or the only caller.

#### Findings

### CW-B26-01 -- First two flight-table saves both target sector A, defeating the dual-sector guarantee
- Site: src/logging/flash_flush.cpp:178-191
- Lens: The spine, block B (spec-noncompliance / passes-tests-yet-wrong); Comments lens CCG NL.2 -- "if the comment and the code disagree, both are likely to be wrong"
- Claim: The active_sequence == 0 override at lines 185-187 sends the first save to sector A, and the parity rule at lines 180-182 then sends the second save to sector A as well, so the only valid copy of the flight table is erased while no other valid copy exists.
- Why: Trace a factory-fresh device -- flight_table_load fails, flight_table_init sets active_sequence = 0 and loaded = true (flight_table.cpp:25-27). Save 1: active_sequence is 0, parity picks B, the override forces A; A now holds sequence 1 and B is blank. Save 2: active_sequence is 1, so 1 % 2 == 0 is false and the target is A again -- write_table_sector erases A (flash_flush.cpp:100) while B has never been written. A reset, brownout or watchdog bite inside that erase leaves neither sector valid; read_table_sector rejects both on state/magic/CRC, flight_table_load returns false, and the entire flight index -- every FlightLogEntry for every flight already flushed -- is lost, while the raw log sectors stay occupied. That is precisely the failure the A/B pattern named on line 178 exists to prevent. From save 3 onward alternation is correct, so the defect is confined to, and always present in, the first rewrite after a fresh table (including after a failed load).
- Confidence: high
- Direction: Delete the seq==0 special case -- with it removed, active_sequence 0 already selects B and the alternation is correct from the first save onward -- or derive the target from which sector currently holds the active copy rather than from sequence parity.
- Verdict: CONFIRMED -- flash_flush.cpp:180-187 verified: with active_sequence == 0 the parity picks B and the override forces A, then at active_sequence == 1 the parity picks A again, so saves 1 and 2 both erase and rewrite sector A while B has never been written (flight_table_init sets active_sequence = 0 at flight_table.cpp:26).

### CW-B26-02 -- psram_init's documented call-order precondition is violated by its only caller
- Site: src/logging/psram_init.h:43
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions/limitations stated in the function preamble) and CCG NL.2
- Claim: The preamble states "Must be called BEFORE stdio_init_all() and flash operations", but the single caller runs psram_init after both.
- Why: main.cpp:531 calls init_hardware, which calls init_early_hw first (main.cpp:268); init_early_hw runs radio_config_storage_init and ends with init_usb, which is stdio_init_all (main.cpp:193); psram_init is only reached afterwards at main.cpp:276. Both halves of the stated precondition are false at the actual call site, and main.cpp:270-274 gives a different ordering rationale entirely ("MUST be before Core 1 launch"). A reader cannot determine which statement is authoritative: either the header records a real hardware constraint that the boot path silently breaks (the flash-before-USB family the project already learned the hard way), or the constraint is stale and future callers will be steered by a false rule. On a file whose whole job is QMI/XIP reconfiguration, a wrong ordering claim is not cosmetic.
- Confidence: high
- Direction: Decide which is true and make the two agree -- either restate the preamble as the constraint the code actually depends on (before Core 1 launch, before any flash_safe_execute is needed) or move the call so the documented order holds.
- Verdict: REFUTED -- The call graph is misread -- init_early_hw (main.cpp:229-242) does not call init_usb; stdio_init_all and the flash-storage inits live in init_peripherals, which main.cpp:304 runs at the END of init_hardware, i.e. AFTER psram_init at main.cpp:276, so the documented precondition is in fact honoured.

### CW-B26-03 -- The "hard gate" flash-safe test claims erase+program but only erases
- Site: src/logging/psram_init.cpp:298-344
- Lens: The spine, block B (confabulation -- a confident justification the body does not implement, NIST AI 600-1); Comments lens CCG NL.2 / JSF AV 134
- Claim: Both the header (psram_init.h:79-81) and the in-file rationale (lines 298-301) describe the test as a "flash_safe_execute() erase+program" cycle, but the callback at lines 313-316 performs only flash_range_erase and no flash_range_program appears anywhere in the function.
- Why: The stated purpose is to validate the SDK's QMI M1 save/restore for the CS1-not-in-FLASH_DEVINFO case, and the same comment names the asymmetry being checked ("timing/rcmd/rfmt restored, wfmt/wcmd untouched"). The program cycle is the half of flash_safe_execute that exercises the write path; an erase alone never drives it. So the gate whose result main.cpp:395 stores as g_psramFlashSafePassed reports a pass for a path it never ran -- the gate-is-green-but-the-check-did-not-run shape. A reader trusting the header will believe PSRAM survivability across flash programming has been demonstrated when only erase has been, and this is labelled a council hard gate.
- Confidence: high
- Direction: Either add the program half so the test matches its own description (erase, then program a page inside the same guarded window, then verify PSRAM), or correct both comments to say erase-only and state explicitly that the program path is unverified.
- Verdict: CONFIRMED -- psram_init.cpp contains flash_range_erase at :315 and no flash_range_program anywhere in the file, while psram_init.h:79-81 and the in-file rationale both describe the gate as an 'erase+program' cycle whose result main.cpp:395 stores as g_psramFlashSafePassed.

### CW-B26-04 -- psram_self_test reads back through the cached alias, so the XIP cache can satisfy every check
- Site: src/logging/psram_init.cpp:242-266
- Lens: The spine, block C (functionally-correct-but-safety-blind hardware code) plus block B (passes-tests-yet-wrong)
- Claim: The self-test writes and reads its three patterns through kPsramCachedBase, so each readback can be served from the XIP cache without any PSRAM transaction, which is not the addressing check the header promises.
- Why: Each pattern is written at lines 257-259 and read back at lines 262-266 with at most two intervening accesses. This project's own code states the cache is write-back over PSRAM: flash_flush.h:11-12 and flash_flush.cpp:349 exist specifically to write back "dirty PSRAM cache lines", and the sibling test in this same file deliberately uses kPsramUncachedBase for exactly this reason (lines 323-325). A pass therefore does not establish that the M1 read/write format is correct or that the top of the 8 MB range addresses cleanly. The result is load-bearing: main.cpp:278 stores it and ao_logger.cpp:258-261 uses it to commit the flight ring to PSRAM instead of the SRAM fallback, so a false pass means a whole flight is logged into a device that is not actually working, discovered only at readback. I did not verify the XIP cache geometry against the datasheet in this pass, hence medium rather than high.
- Confidence: medium
- Direction: Perform the readback through kPsramUncachedBase as psram_flash_safe_test already does, or invalidate the cache between the write pass and the read pass, so the verification actually crosses the QMI boundary.
- Verdict: CONFIRMED -- psram_self_test writes and reads its three patterns through kPsramCachedBase (psram_init.cpp:239-266) with at most two intervening accesses, while the sibling test in the same file deliberately uses kPsramUncachedBase and flash_flush.cpp:349's xip_cache_clean_all confirms the alias is write-back cached -- so a pass does not prove a PSRAM transaction occurred, and main.cpp:278 / ao_logger.cpp:259 make the result load-bearing.

### CW-B26-05 -- QMI direct-mode window in psram_configure_qmi runs with interrupts enabled, unlike the identical window in psram_detect
- Site: src/logging/psram_init.cpp:180-220
- Lens: The spine, block C (peripheral init SEQUENCE / lifecycle; HW-register / MMIO discipline)
- Claim: psram_configure_qmi enables QMI direct mode at line 182, issues the QPI-enable command, rewrites M1 timing/rfmt/rcmd/wfmt/wcmd and only clears direct mode at line 216, all without the interrupt guard that psram_detect wraps around the same class of window at lines 79 and 133.
- Why: The file's own rationale for the SRAM placement (lines 13-15) is that these functions "manipulate QMI registers that control XIP flash execution" -- but that placement only covers this function's own instructions. Anything else executing inside the window is unprotected: an interrupt taken between lines 182 and 216 vectors into a handler that, unless it is itself SRAM-resident, fetches through the XIP path direct mode has taken over, and does so while M1's read format is mid-rewrite. psram_detect's guard is the in-file evidence that the author judged this hazard real, and psram_configure_qmi -- reached from the same psram_init call at line 232 -- is the longer of the two windows. The exposure is timing-dependent, so it will not reproduce on demand and no test will show it.
- Confidence: medium
- Direction: Wrap the direct-mode window in save_and_disable_interrupts / restore_interrupts the way psram_detect does, with a single exit so the restore cannot be skipped, or state in the preamble why this window is exempt.
- Verdict: REFUTED -- The stated exposure requires an interrupt inside the window, but psram_configure_qmi is reached only from psram_init at main.cpp:276, which runs before stdio_init_all/USB, before the QF tick timer and before Core 1 launch -- none of the pre-psram init steps (fault handlers, GPIO, i2c_bus_init, ws2812, mcu_temp_init) arm an interrupt source, so no handler can vector during the window on the real boot path.

### CW-B26-06 -- A failed ring read is swallowed, and the flush still reports kOk before destroying the source data
- Site: src/logging/flash_flush.cpp:247-253
- Lens: The spine, block B (unchecked / happy-path-only error handling; Power of Ten Rule 7 manual residual -- walk every exit path, and confirm a dropped failure was a justified ignore)
- Claim: When ring_read_sequential fails, the break abandons only the current sector while the outer loop keeps running, every remaining sector is erased and programmed as all-0xFF filler, and flush_sectors still returns FlushResult::kOk at line 266 -- a detected read error converted into a reported success. The branch is, however, unreachable on the current call path.
- Why: frame_idx is not advanced on failure, so each later iteration of the sector loop would retry the same failing index, fail immediately, and write a 4 KB sector of 0xFF; flush_ring_to_flash would then treat that as success -- save_flight_entry records frame_count = stored and sector_count = sectors_needed (lines 357-359), and ring_reset at line 364 discards the only surviving copy. But ring_read_sequential (ring_buffer.cpp:106-118) returns false only for a null or uninitialised rb, a null frame_out, or abs_index >= ring_stored_count(rb) -- and the inner loop guard at line 247 is frame_idx < stored, where stored is that same ring_stored_count value, while an uninitialised rb makes stored zero so the loop is never entered. No live path therefore reaches the break. What remains is an error-handling-contract defect: a detected failure is silently upgraded to FlushResult::kOk, and the enum already carries kEraseError / kWriteError shapes that would have expressed it.
- Confidence: medium
- Direction: Return a distinct failure result on the read failure so flush_ring_to_flash aborts before it touches the flight table and before ring_reset, leaving the ring contents recoverable.
- Verdict: RESHAPED -- The swallowed-error shape is real (break, outer loop continues, kOk returned at :266), but the branch is unreachable on the current call path -- ring_read_sequential fails only for null/uninitialised rb or abs_index >= ring_stored_count, and the loop guard at :247 uses that same bound -- so it is an error-handling-contract defect, not a live data-loss path.

### CW-B26-07 -- The flash_safe_execute trampoline is duplicated, including a same-named non-static type in two translation units
- Site: src/logging/psram_init.cpp:303-316
- Lens: The spine, block A -- CCG ES.3 "Don't repeat yourself, avoid redundant code" (Fowler, Duplicated Code); the ODR consequence is invisible to every gate on this build
- Claim: kFlashSafeTimeoutMs, struct flash_erase_params and do_flash_erase are defined here and again in flash_flush.cpp:27-48, and flash_flush.cpp:24 records that the same idiom exists a third time in calibration_storage.cpp.
- Why: The helper function and the constant have internal linkage, so their duplication is only a maintenance cost -- but struct flash_erase_params is not: it is a non-static type declared in namespace rc in both translation units. Today the two definitions are token-identical so the program is well formed, but the moment either copy gains a field, changes a member type or reorders members (say psram_init needs a flag), the two definitions of rc::flash_erase_params disagree, the program becomes ill-formed with no diagnostic required, and the linker silently keeps one layout for both call sites -- a flash offset or length read out of the wrong slot, in code that erases flash. Nothing in the build catches it: the compiler sees one translation unit at a time and there is no cross-TU ODR check in the gate set.
- Confidence: medium
- Direction: Hoist the trampoline (params struct, callback, timeout constant) into one shared internal header used by all three sites; at minimum put each local copy in an anonymous namespace so the types cannot collide across translation units.
- Verdict: CONFIRMED -- Verified: kFlashSafeTimeoutMs / struct flash_erase_params / do_flash_erase are defined at flash_flush.cpp:26-47 and again at psram_init.cpp:306-316, both inside namespace rc and neither in an anonymous namespace, so the params struct is a same-named external-linkage type in two TUs (calibration_storage.cpp:77 uses a differently-named typedef, matching the 'third instance of the idiom' note at flash_flush.cpp:24).

### B27 -- logging: flight_table + log_decimator + data_convert

#### Coverage

- src/logging/data_convert.h -- PARTIAL -- Read whole; the quantization block is the project's declared ICD text (docs/IVP.md:1655 "the struct IS the ICD") and states error bounds without the saturation limits the .cpp implements.
- src/logging/data_convert.cpp -- PARTIAL -- Read whole; both conversion functions and all four clamp helpers walked field-by-field against FusedState/TelemetryState; field mapping and Q15/scale factors are correct, guard totality is not.
- src/logging/flight_table.h -- FAIL -- Read whole as a Kind E layout/identity contract surface; the file-banner flash map contradicts the derived layout in flash_layout.h, and one nested-struct size claim is wrong.
- src/logging/flight_table.cpp -- PARTIAL -- Read whole; all eleven functions walked, null/loaded/full guards are consistent, but the add-entry contract has an unstated and unchecked precondition.
- src/logging/log_decimator.h -- PARTIAL -- Read whole as the decimator's declared contract; its field-treatment list is presented as exhaustive and is not.
- src/logging/log_decimator.cpp -- PARTIAL -- Read whole; accumulate/average field lists cross-checked one-for-one against fused_state.h, quaternion antipodal handling and normalization guard are sound; three live FusedState fields are silently unhandled.

#### Findings

### CW-B27-01 -- flight_table.h banner flash map contradicts the derived layout it claims to describe
- Site: src/logging/flight_table.h:7-16
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") and NL.3 (point to the spec, do not paraphrase it); Contract-Surface Helper Kind E ("Could two boards/roles silently disagree if someone duplicates a constant elsewhere? Finding: duplication of the map").
- Claim: The banner re-states the whole flash map as hard-coded absolute addresses that no longer match include/rocketchip/flash_layout.h, overstating the flight-log region by three sectors and omitting two regions entirely.
- Why: flash_layout.h derives every boundary from PICO_FLASH_SIZE_BYTES top-down. For the 8MB part that yields calibration at 0x7FE000, flight table at 0x7FC000, radio-config storage at 0x7FA000-0x7FBFFF (flash_layout.h:62-65, added by Stage T IVP-T5.5), and the flash-safe test sector at 0x7F9000 (flash_layout.h:68-69), so kFlashLogEnd is 0x7F9000. The banner instead says flight logs run "0x080000-0x7FBFFF" and lists neither the radio-config nor the flash-safe-test region -- it describes the layout as it stood before those two were carved out. The three sectors 0x7F9000, 0x7FA000 and 0x7FB000 are claimed by the banner as log space while flash_layout.h assigns them to other owners. The banner's "1912 sectors" also disagrees with the value the code actually produces (test/test_flight_table.cpp:96 asserts kFlightLogSectors == 1913). Separately, flash_layout.h:4-7 states the layout is derived precisely so it "adapts to any flash size (8MB Feather, 4MB Tiny2350)" -- absolute hex addresses are true for one part only, so on a 4MB board every address in this banner is wrong. A reader sizing a new region, or hand-checking a flash dump, is reading a map that points at storage owned by radio config.
- Confidence: high
- Direction: Replace the address table with a one-line pointer to include/rocketchip/flash_layout.h as the single source, keeping only the prose flash_layout.h does not carry (the dual-sector A/B alternate-write plus higher-sequence-wins rule, and the "identical to calibration_storage.cpp" note).
- Verdict: CONFIRMED -- Computed from flash_layout.h for the 8 MB part: radio config occupies 0x7FA000-0x7FBFFF and the flash-safe test sector 0x7F9000, so kFlashLogEnd is 0x7F9000 and kFlashLogSectors is 1913 (asserted at test/test_flight_table.cpp:96), while the banner at flight_table.h:8 still claims 0x080000-0x7FBFFF and 1912 sectors and lists neither newer region.

### CW-B27-02 -- Decimator silently freezes the confidence-gate fields at the first sample of each window
- Site: src/logging/log_decimator.cpp:82-91 (omission's effect at :98-100 and :124)
- Lens: The spine, block B -- spec-noncompliance ("silently drops an explicit semantic requirement ... while looking complete and passing normal-usage tests") and passes-tests-yet-wrong; Comments & documentation quality -- JSF AV 134 (document the function's assumptions/limitations).
- Claim: FusedState::confident, ::confidence_div_deg and ::uncertain_ms are neither accumulated, nor averaged, nor refreshed by copy_discrete_fields, so the decimated output carries whatever those fields held on the first sample of the window.
- Why: decimator_push copies the whole struct on the first sample (:99) and thereafter touches only the quaternion, the 24 fields in accumulate_floats, and the 8 fields in copy_discrete_fields. The confidence triple (fused_state.h:83-85) is in none of the three sets, so out.confident and its peers are up to ratio-1 samples stale -- at the 200Hz-to-50Hz ratio the header advertises, up to 15 ms behind, and further at the PSRAM ratio. These are live data, not dead fields: ao_logger.cpp:212-214 populates all three from the confidence gate on every tick before the push. The omission is also internally inconsistent -- copy_discrete_fields already forwards the peer bool zupt_active (:88) and the peer uint32 met_ms (:90), so a reader cannot tell whether confident and uncertain_ms were excluded deliberately or missed. The header's field-treatment list (log_decimator.h:7-14) reads as exhaustive and names none of them, so nothing records which rule was intended. Today no consumer reads them off the decimated struct -- fused_to_telemetry ignores them -- so the defect is latent rather than live; it becomes a wrong logged value the moment the log-schema extension already queued for Stage 17 reads a confidence field out of a decimated frame.
- Confidence: medium
- Direction: Decide the rule for each of the three (latest-sample for confident/uncertain_ms matches the other discretes; mean or worst-case for confidence_div_deg) and make the header's field-treatment list exhaustive so a future FusedState field cannot land in the same silent gap.
- Verdict: CONFIRMED -- fused_state.h:83-85 (confident, confidence_div_deg, uncertain_ms) appear in none of accumulate_floats, copy_discrete_fields (log_decimator.cpp:81-91) or the quaternion block, so they retain the first-sample values copied at :99, while their peers zupt_active and met_ms are forwarded -- and ao_logger.cpp:212-214 populates all three every tick.

### CW-B27-03 -- Quantization bounds are published as the ICD without the saturation limit the code enforces
- Site: src/logging/data_convert.h:23-27
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions and limitations documented in the preamble) and CCG NL.2 (state intent; a comment/code disagreement is a latent bug).
- Claim: The preamble gives a per-field max quantization error with no statement of the representable range, but the implementation clamps rather than reporting, so the stated bound is false for any velocity outside +/-327.67 m/s.
- Why: docs/IVP.md:1655 records the project decision that "Fixed-point scaling documented as code comments -- the struct IS the ICD", which makes this block authoritative interface text (it is duplicated near-verbatim at telemetry_state.h:26-30, so both copies carry the same gap and must move together). The .cpp meets those bounds only in range: clamp_round_i16 (data_convert.cpp:31-35) saturates at +/-32767, so vel_n_cms / vel_e_cms / vel_d_cms (:75-77) and baro_vvel_cms (:83) cap at +/-327.67 m/s. A high-power flight through Mach 1 is roughly 343 m/s, so a boost-phase vertical velocity of 400 m/s is written to both the flight log and the radio snapshot as 327.67 m/s -- a 72 m/s error against a preamble promising +/-0.005 m/s. The clamp helpers exist precisely because the author anticipated saturation; the ICD text does not mention it, so a consumer decoding the log has no documented reason to distrust a pegged value. The bounds are also inconsistently derived (the code rounds, so half-LSB is the true bound: quaternion and altitude are quoted at a full LSB, velocity and temperature at a half), which is the tell that the block was written by inspection rather than from the conversion it describes.
- Confidence: high
- Direction: State the representable range per field alongside the error bound, and say that out-of-range inputs saturate silently rather than being flagged; if a saturation indicator is wanted, TelemetryState::flags has seven reserved bits (telemetry_state.h:56).
- Verdict: CONFIRMED -- data_convert.h:23-27 states per-field max quantization error with no representable range, while clamp_round_i16 (data_convert.cpp:31-35) saturates at +/-32767 so vel_*_cms (:75-77) and baro_vvel_cms (:83) peg at +/-327.67 m/s; docs/IVP.md:1655's 'the struct IS the ICD' decision makes this block the interface contract, and the same text is duplicated at telemetry_state.h:26-30.

### CW-B27-04 -- Clamp helpers are shaped as total guards but pass NaN through to the float-to-integer cast
- Site: src/logging/data_convert.cpp:31-56
- Lens: The spine, block B -- passes-tests-yet-wrong ("exercise corner/edge/null/unexercised inputs by hand"); Power of Ten Rule 7 (parameter validity must be checked inside each function).
- Claim: All four clamp_round_* helpers guard only the two finite extremes with >= and <=, so a NaN input fails every comparison and falls through to a static_cast on std::roundf(NaN), an out-of-range floating-to-integer conversion and therefore undefined behaviour -- the helpers are shaped as total guards but are not total, contrary to Power of Ten Rule 7's in-function parameter check.
- Why: The bodies read as total -- high bound, low bound, else convert -- but NaN compares false against both bounds, so the fall-through case is the conversion, not a guard, and there is no non-finite screen anywhere between the filter and this boundary in ao_logger or fused_to_telemetry. Reachability is not demonstrated, however: the ESKF screens every measurement input and every covariance element with std::isfinite (eskf.cpp:614, 633, 725, 861-862, 1034-1036, 1279-1280), so no in-tree path is shown to deliver a non-finite value into FusedState. The claim that survives on its own is the guard-totality one, which P10 Rule 7 states as an in-function obligation independent of caller behaviour -- and the stakes are why it matters: on the Cortex-M33 the hardware vcvt saturates rather than trapping, so a non-finite input that did arrive would become a silently plausible extreme value in a permanent flash-log frame and a radio frame, and this conversion is the last point before that happens.
- Confidence: medium
- Direction: Add an explicit non-finite branch returning a defined value (0, or the saturated bound) before the cast, and state in the preamble what a non-finite input maps to.
- Verdict: RESHAPED -- The guard-totality defect and the out-of-range float-to-int conversion are real and verified, but the 'reachable input is a diverged filter' framing is overstated -- the ESKF screens every measurement input and covariance element with std::isfinite, so no in-tree path is shown to deliver a non-finite value to this boundary.

### CW-B27-05 -- flight_table_add_entry lets the caller's sector fields define next_free_sector, precondition unstated and unchecked
- Site: src/logging/flight_table.cpp:46-63 (specifically :58); contract at src/logging/flight_table.h:126-127
- Lens: Comments & documentation quality -- JSF AV 134 (a function with an ordering requirement, or a range it does not validate, must say so); Power of Ten Rule 7 (parameter validity checked inside each function).
- Claim: The function validates null-ness, loaded-ness and table fullness but not the entry's sector fields, then overwrites the table's allocation cursor with entry->start_sector + entry->sector_count, while its doc comment names table-full as the only failure mode.
- Why: next_free_sector is the table's allocation pointer -- flash_flush.cpp:334 reads it to choose where the next flight is erased and written. Setting it from unvalidated caller data means an entry whose start_sector is stale or below the current cursor silently rewinds the pointer, and the next flush erases sectors holding a previous flight; an entry whose start_sector + sector_count runs past kFlightLogEnd/kFlashSectorSize moves the cursor into the flash-safe-test and radio-config regions. The sole production caller is safe today -- flush_ring_to_flash checks start_sector + sectors_needed against kFlightLogEnd (flash_flush.cpp:335) before building the entry -- so this is a contract gap, not a live defect. But the header presents add_entry as a standalone host-testable API whose only stated failure is "Returns false if table is full", so a second caller (a table-repair path, a recovered-entry reinsertion) has nothing telling it that monotonic, contiguous, in-region sectors are required. Every host test feeds well-formed contiguous entries, so no test exercises the assumption.
- Confidence: medium
- Direction: Either state the precondition in the header doc comment (entry sectors must be contiguous with the current next_free_sector and within the log region) or enforce it in the body and return false -- the existing bool return already accommodates a second failure mode.
- Verdict: CONFIRMED -- flight_table.cpp:58 sets next_free_sector directly from entry->start_sector + entry->sector_count with no validation, while the header doc at flight_table.h:126 names table-full as the only failure mode; the sole production caller bounds-checks first (flash_flush.cpp:334-337), so this is a real undocumented precondition on the cursor flash_flush.cpp:334 reads to choose where to erase.

### CW-B27-06 -- FlightLogEntry's metadata size comment understates the field by the nested struct's implicit padding
- Site: src/logging/flight_table.h:64
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 (comment and code disagree); Contract-Surface Helper Kind E (a layout map's numbers are the contract).
- Claim: The comment annotates FlightMetadata as "(14B)" but the type is not packed and carries 4-byte alignment, so it occupies 16 bytes inside the packed entry.
- Why: FlightMetadata (telemetry_state.h:78-88) is a plain struct: uint32 at 0, uint16 at 4, six uint8 at 6-11, _pad[2] at 12-13 -- 14 bytes used, alignment 4 from the leading uint32, so sizeof rounds to 16 with two bytes of implicit trailing padding. The packed attribute on the enclosing FlightLogEntry removes padding between that struct's own members but does not repack a nested type, so those two bytes are real bytes of every on-flash entry and are covered by flight_entry_compute_crc (flight_table.cpp:35-36), which CRCs sizeof(FlightLogEntry) - 4. The neighbouring "(36B)" annotation for FlightSummary is correct, so the pair reads as an accurate table rather than an approximation, and anyone hand-computing the on-flash entry layout or the maximum entry count from these numbers is off by two bytes per entry. The explicit _pad[2] member also implies its author believed padding had been eliminated, which it has not.
- Confidence: medium -- the alignment reasoning is deterministic, but no build was run to confirm sizeof, per the no-build constraint on this pass.
- Direction: Correct the annotation to 16B, or add a static_assert on sizeof(FlightMetadata) alongside the existing one at telemetry_state.h:58 so the number is enforced rather than asserted in prose.
- Verdict: CONFIRMED -- FlightMetadata (telemetry_state.h) is unpacked with a leading uint32_t, so its alignment is 4 and its 14 used bytes round to sizeof 16; the packed attribute on FlightLogEntry does not repack a nested type, so the '(14B)' annotation at flight_table.h:64 understates the field by two bytes (the neighbouring '(36B)' for FlightSummary is correct at 9 x 4, and no static_assert covers either).

### B28 -- logging: pcm_frame + radio_config_storage + CRC headers

#### Coverage
src/logging/crc16_ccitt.h -- PASS -- Contract surface (helper Kind C/E, header-only algorithm): every documented parameter checked against the body -- poly 0x1021 (:28), init 0xFFFF (:66), no final XOR (:71), MSB-first table generation (:30-40), 256 x uint16 = 512 B constexpr table with no runtime init (:42-52); the JSF AV-182 Exception-1 note at :63-64 accurately describes the one void*->T* conversion.
src/logging/crc32.h -- PASS -- Same check: reflected poly 0xEDB88320 (:26), init 0xFFFFFFFF (:63), final XOR (:68), 256 x uint32 = 1024 B constexpr table; crc32_update's stated contract "caller manages init and final XOR" (:73) matches the body exactly (:78-86).
src/logging/pcm_frame.cpp -- FAIL -- Decom table offsets verified field-by-field against the packed TelemetryState layout (all 20 rows correct), but the flight-log build stamp is a frozen literal, the table's own unit rule is contradicted by several rows, and the event-frame CRC span is a bare literal where the standard-frame sites derive it.
src/logging/radio_config_storage.cpp -- FAIL -- Dual-sector sequence/wear logic walked on every path (both-invalid, one-valid, both-valid tie, erase-then-write, failed-write) and is sound; the defects are a byte-comparison of a struct that contains padding and the missing half of the LL-31 flash/I2C protocol.
src/logging/radio_config_storage.h -- PARTIAL -- Contract surface (helper Kind C): four declarations whose prose is usable but incomplete -- init()'s documented failure/ordering contract is not what the body does, and the write/erase preamble cites LL Entry 31 without stating the caller obligation that citation carries.

#### Findings

### CW-B28-01 -- Flight log header stamps a build tag frozen at IVP-74
- Site: src/logging/pcm_frame.cpp:158
- Lens: The spine A -- CCG ES.3 (don't repeat yourself: one piece of knowledge, one place) + Fowler "Duplicated Code"; comments lens MSC12-C (documentation describing something the system no longer produces).
- Claim: flight_log_header_fill() writes the string literal "ivp74-profile-1" into FlightLogHeader::build_tag, while the project's maintained build identity lives in include/rocketchip/version.h:49 as kBuildIterationTag (currently "16B-init").
- Why: every flight log written by this firmware carries a build stamp that is roughly ten stages stale, so post-flight analysis cannot tell which binary produced a log -- the exact provenance question the 64-byte header exists to answer (pcm_frame.h:158-167, "magic number and version for forward compatibility"), and the exact failure LL Entry 2 was written about. The sibling field on the line above (:157) does read a live constant (kVersionString), which is what makes the frozen literal read as an oversight rather than a decision. Nothing catches it: the field is never compared to anything, and no host test covers flight_log_header_fill (test/test_pcm_frame.cpp has no case for it).
- Confidence: high
- Direction: stamp kBuildIterationTag (version.h is already reachable here via config.h) instead of the literal, and update the "e.g." comment at pcm_frame.h:176 so the header no longer advertises the stale value as the example.
- Verdict: CONFIRMED -- pcm_frame.cpp:158 writes the literal "ivp74-profile-1" into build_tag while the maintained tag is version.h:49 kBuildIterationTag = "16B-init", and the line above at :157 does read the live kVersionString; nothing compares the field and test_pcm_frame.cpp has no flight_log_header_fill case.

### CW-B28-02 -- Decom table's stated unit rule is contradicted by several of its own rows
- Site: src/logging/pcm_frame.cpp:18-20, 26-27, 38-39 (rule declared at include/rocketchip/pcm_frame.h:100 and :108)
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") + JSF AV 134 (document the assumptions and limitations a consumer must honor).
- Claim: the table's contract says "multiply raw integer by scale to get SI units", but lat_1e7/lon_1e7 scale to degrees (:26-27), temperature_c to Celsius (:38), and battery_mv to millivolts with scale 1.0F (:39) -- none of which are SI, and the millivolt row leaves the value unconverted while every other length/velocity/time row does convert (mm->m 0.001F, cm/s->m/s 0.01F, ms->s 0.001F).
- Why: DecomField carries no unit string, so the stated rule is the only unit information a ground decoder has. A decoder that implements the documented rule labels battery_mv * 1.0F = 7400 as volts (1000x wrong) and labels temperature as kelvin; a decoder that instead infers the unit from the name suffix ignores the scale column and gets baro_alt_mm in millimetres. The two readings disagree on 4 of 20 rows and nothing in the build resolves which is authoritative. The existing tests only check offset coverage and range (test/test_pcm_frame.cpp DecomTableCoversAllBytes / DecomTableOffsetsInRange) -- no test looks at scale, so the table stays green while its contract is ambiguous.
- Confidence: medium
- Direction: state the convention the table actually follows (e.g. "scale converts to the field's engineering unit; 0 = no scaling") rather than "SI units", or add an explicit unit string to DecomField so the rule stops having to be inferred.
- Verdict: CONFIRMED -- The rule at pcm_frame.cpp:20 and pcm_frame.h:108 says the scale converts to SI units, but lat_1e7/lon_1e7 (:26-27) scale to degrees, temperature_c (:38) to Celsius and battery_mv (:39) to millivolts with scale 1.0F -- 4 of 20 rows -- and DecomField carries no unit string to disambiguate.

### CW-B28-03 -- Wear-avoidance skip compares a struct that contains padding bytes
- Site: src/logging/radio_config_storage.cpp:229-233
- Lens: The spine B (passes-tests-yet-wrong, unexercised path) -- SEI CERT EXP42-C, "Do not compare padding data".
- Claim: radio_config_storage_write() decides whether a flash write is needed with memcmp(&g_cached, cfg, sizeof(*cfg)) at src/logging/radio_config_storage.cpp:231 over rc::RadioConfig, which is not packed and carries two padding bytes (include/rocketchip/radio_config.h:29-37 -- five uint8_t-sized members, EncoderType being uint8_t-backed, then a uint16_t forces one pad byte before bandwidth_khz and one trailing pad byte), so two logically identical configs can compare unequal.
- Why: the two operands acquire their padding from different places. g_cached is filled either from a flash-read Entry (:159, :166, :174, :178 -- padding is whatever write_to_sector's 0xFF memset at :190 left there) or from a caller's struct (:241), while cfg comes from AO_Radio's runtime_config. Padding is not required to be copied by struct assignment, so two logically identical configs can compare unequal, and nothing in the enabled gate set catches it (bugprone-suspicious-memory-comparison / cert-exp42-c is not in .clang-tidy's Checks list). The failure direction is safe -- an unnecessary write, not a wrong one -- but it defeats the stated purpose of the skip, and each miss costs a 4 KB erase plus a page program under flash_safe_execute. The consequence is latent rather than live, however: the only caller of radio_config_storage_write() is src/active_objects/ao_radio.cpp:706, inside the ROCKETCHIP_RADIO_PERSIST-defined branch of ao_radio.cpp:688-712, and ROCKETCHIP_RADIO_PERSIST is defined nowhere in src/, test/ or CMakeLists.txt. So the comparison never executes on the shipped build; the defect matters the moment the persist gate is turned on -- the same gate CW-B28-04 flags.
- Confidence: medium
- Direction: compare the fields that define the config rather than its bytes (a field-wise equality helper next to the struct), or compare the CRC that compute_crc() already produces.
- Verdict: RESHAPED -- The padding-comparison defect is real (RadioConfig carries one interior and one trailing pad byte given the uint8_t-backed EncoderType and the uint16_t bandwidth_khz), but the wear consequence is latent, not live: the only caller of radio_config_storage_write is ao_radio.cpp:706, inside a ROCKETCHIP_RADIO_PERSIST-gated region, and that macro is defined nowhere in the tree.

### CW-B28-04 -- Write/erase contract cites LL Entry 31 but omits the obligation that citation carries
- Site: src/logging/radio_config_storage.h:29-35 (body at src/logging/radio_config_storage.cpp:79-91)
- Lens: The spine C ADD -- "functionally-correct-but-safety-blind hardware code" (blind to the peripheral's failure mode) + comments lens JSF AV 134 (a function with an ordering precondition must state it in its preamble).
- Claim: the preamble tells the caller "Uses flash_safe_execute() per LL Entry 31" but does not state LL Entry 31's actual requirement -- pause Core 1's I2C before the flash window and call i2c_bus_reset() after it -- and neither safe_write()/safe_erase() nor the module's only writer performs either step.
- Why: every other runtime flash_safe_execute() path in the tree does both. cal_save_to_flash() calls rc::core1_i2c_pause() then i2c_bus_reset() (src/active_objects/ao_rcos.cpp:335-355) and the CLI flush path does the same (src/cli/rc_os_commands.cpp:1047-1055, 1113-1118); src/safety/core1_i2c_pause.h records R-17 as having wired those primitives "around every reachable runtime flash_safe_execute() callsite". The radio persist writer (src/active_objects/ao_radio.cpp:706) does neither, so with the feature enabled a debounced persist erases and programs flash from an AO tick while Core 1 is mid-I2C-transaction -- the DW_apb_i2c corruption class of LL Entry 31 -- with no post-op reset to recover from it. This is latent today, not live: ROCKETCHIP_RADIO_PERSIST is not defined anywhere in the tree, which is also why an "every reachable callsite" sweep did not see it. The contract surface is where a reader would look for the rule, and it currently points at the lesson without carrying it.
- Confidence: medium
- Direction: either state the caller obligation in the header preamble the way ao_rcos.cpp:335-343 states it at its call site, or move core1_i2c_pause()/i2c_bus_reset() inside safe_write()/safe_erase() so a future caller cannot omit it; either way, flag that the persist gate needs this settled before it is turned on.
- Verdict: CONFIRMED -- safe_write/safe_erase (radio_config_storage.cpp:79-91) call flash_safe_execute with no core1_i2c_pause/i2c_bus_reset, ao_radio.cpp:706 adds neither, and the contrast case cal_save_to_flash (ao_rcos.cpp:335-355) does both with the R-17 rationale spelled out at the call site -- while the header at :30 cites LL Entry 31 without carrying its obligation; the finding already scopes itself as latent behind ROCKETCHIP_RADIO_PERSIST.

### CW-B28-05 -- init()'s documented contract does not match its body
- Site: src/logging/radio_config_storage.h:20-22 (body at src/logging/radio_config_storage.cpp:210-215)
- Lens: Contract-surface helper Kind C ("does the signature match the prose?") -- JSF AV 134; P10 Rule 7 manual residual (a discarded return with no stated reason).
- Claim: the header documents radio_config_storage_init() as having a boot-ordering precondition and a success/failure return, but the body has no failure path (it returns true unconditionally) and performs no flash erase or program -- only XIP reads via flash_read() at :93-96.
- Why: two things a reader cannot determine from the contract. First, what a false would mean: find_active_sector() cannot report "both sectors present but corrupt" -- that case falls through to the same state as a blank device (:150-155, activeSector=A, sequence=1, cachedValid=false) -- and all four call sites discard the value anyway (:219, :227, :247 and src/main.cpp:259) with no (void) cast and no reason, so if init ever gains a real failure path they will silently continue past it. Second, the "BEFORE stdio_init_all() (LL Entry 4/12)" precondition cites a lesson about erase/program making flash inaccessible to the USB IRQ handlers; this function does neither, so the cited rationale does not govern it and a reader cannot tell whether the ordering is a hard requirement or inherited boilerplate.
- Confidence: medium
- Direction: make the declaration honest -- either give init() a real failure return and check it at the call sites, or declare it void and drop the "returns true on success" line -- and either drop the LL Entry 4/12 precondition or restate it as the read-only ordering preference it actually is.
- Verdict: CONFIRMED -- radio_config_storage.cpp:210-215 returns true unconditionally with no failure path and touches flash only through the XIP read at :93-96, while the header at :20-22 documents a success/failure return and a stdio_init_all ordering precondition citing LL Entry 4/12 (which is about erase/program); all four call sites (:219, :227, :247, main.cpp:259) discard the value with no (void) cast.

### CW-B28-06 -- Event-frame CRC span is a bare literal where the standard-frame sites derive it
- Site: src/logging/pcm_frame.cpp:131-133 (compare :59, :77, :103)
- Lens: The spine A -- CCG ES.3 (the same knowledge expressed in more than one place) + CCG P.1 (express the idea directly: the span is "every byte before crc16").
- Claim: pcm_encode_event() CRCs a hardcoded 13 bytes while the three standard-frame sites express the identical idea as sizeof(PcmFrameHeader) + sizeof(TelemetryState), so one of the four expressions of "the bytes preceding the CRC field" does not track the struct it describes.
- Why: PcmFrameEvent (include/rocketchip/pcm_frame.h:63-72) is edited as a unit -- adding or widening a context field changes the CRC span, and only the total-size static_assert at :73 pushes back; a developer who updates kPcmFrameEventSize to satisfy that assert has nothing telling them to update the 13. The result is a frame CRC'd over a short span that the encoder and any decoder disagree about, silently, on the first event written in flight. Nothing exercises it: test/test_pcm_frame.cpp has no pcm_encode_event case at all, and there is no pcm_decode_event to cross-check the encoder against.
- Confidence: medium
- Direction: express the span the way the standard-frame sites do -- offsetof(PcmFrameEvent, crc16), or the same sizeof-sum shape -- so the constant follows the struct; one shared "bytes before the CRC" helper would collapse all four sites.
- Verdict: CONFIRMED -- pcm_frame.cpp:132 CRCs a bare literal 13 while the three standard-frame sites (:59, :77, :103) derive the identical idea from sizeof(PcmFrameHeader) + sizeof(TelemetryState); the value is correct for PcmFrameEvent as declared at pcm_frame.h:63-72 and only the total-size static_assert at :73 constrains a future edit, so this is a verified ES.3 duplication with a conditional consequence -- which is how the finding states it.

### B29 -- diag + notify backends

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/diag/diag_stats.cpp -- FAIL -- Walked all 6 functions whole; the AO-queue snapshot reads three QP getters outside the critical section the vendor API documents as the caller's job, and the T=0 SPI probe can increment the very error counter the same block prints.
- C:/Users/pow-w/Documents/RC-agent-walk/src/diag/diag_stats.h -- PARTIAL -- Contract surface (Kind C) walked as one work product with the .cpp; the three declarations are clear but the "pure read-only / no state mutation / no risk / safe to run from any phase" prose over-states what the body does.
- C:/Users/pow-w/Documents/RC-agent-walk/src/notify/notify_backend_audio.cpp -- FAIL -- Whole file read; the no-op stub itself is correct and honestly labelled, but the five tone constants are unreachable dead data carrying a rationale the code cannot satisfy.
- C:/Users/pow-w/Documents/RC-agent-walk/src/notify/notify_backend_led.cpp -- FAIL -- All 8 functions walked; the resolver logic and beacon overlay are correct and match led_patterns.h, but two comment blocks assert a wiring state that ao_notify.cpp contradicts and one asserts 0 is not a valid pattern code when it is.
- C:/Users/pow-w/Documents/RC-agent-walk/src/notify/notify_resolver.h -- PARTIAL -- Contract surface (Kind C primary, D secondary) evaluated per the helper; decode_health_faults verified correct against the notify_intents.h priority ordering, but the resolve_led_pattern contract prose is stale with respect to the Stage L beacon overlay.

#### Findings

### CW-B29-01 -- LED backend documents itself as not yet wired, but AO_Notify calls it every tick
- Site: src/notify/notify_backend_led.cpp:13-14, 152-153
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation must describe code that actually runs) and CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine block B, confabulated justification.
- Claim: Both comments state that notify_backend_led_update() is compiled but not called from the AO_Notify tick and that IVP-116 will wire it up, while src/active_objects/ao_notify.cpp:236 already calls it on every tick.
- Why: The file banner tells a reader that changing this function is inert and that the "old direct-caller paths" still drive the LED, so an edit here looks consequence-free when it in fact changes the live flight status indicator on every AO_Notify tick. The staleness also hides that Stage L features (beacon overlay, kFdPreArmFail, kFdBootInit) were added on top of a supposedly-unwired path, so a reader cannot tell which of the file's claims survived the intervening stages.
- Confidence: high
- Direction: Delete both IVP-115/IVP-116 wiring notes and replace the banner sentence with the current fact -- called from AO_Notify's tick handler, one post per tick to AO_LedEngine.
- Verdict: CONFIRMED -- lines 13-14 and 152-153 state verbatim that the function is not called from the AO_Notify tick, while src/active_objects/ao_notify.cpp:236 calls notify_backend_led_update(me->state) inside the tick handler; no gate covers comment staleness.

### CW-B29-02 -- Audio tone table is unreachable dead data with a rationale the linkage forbids
- Site: src/notify/notify_backend_audio.cpp:9-10, 24-28
- Lens: Comments & documentation quality -- CERT MSC12-C (remove doc-comments describing code that is never executed) and CCG NL.2; spine block B, non-self-contained / confabulated claim.
- Claim: The banner says the tone constants are "defined here as data so the notification engine can reference them now", but a tree-wide grep over src/, include/ and test/ finds no reference to any of the five outside their own definitions at :24-28 -- they are unreferenced dead data, marked [[maybe_unused]] so the build stays quiet about it.
- Why: Nothing anywhere consumes the table, so the banner documents a consumption that does not exist: a future audio-stage author reads it, assumes the data half of the contract is already published and consumed, and wires a parser against constants no caller has ever exercised. Meanwhile the strings sit in the flight image as an unverified transcription of an external format whose only check is the format pointer at line 22. Narrowed from the original: the constants being file-static does mean no *other* translation unit can name them, but "the notification engine" plausibly denotes this same TU's deferred parser, so the linkage shows the stated purpose is unfulfilled -- not that it is impossible as written.
- Confidence: high
- Direction: Either delete the table until the audio stage needs it (git holds it) or move it behind the public notify surface with external linkage so the stated rationale becomes true; correct the banner either way.
- Verdict: RESHAPED -- the five constants are genuinely unreferenced anywhere in the tree and the banner's stated purpose is unfulfilled, but the "internal linkage forbids the rationale" framing over-reads it, since the intended consumer is plausibly this same TU's deferred parser.

### CW-B29-03 -- AO queue snapshot reads QP getters without the critical section their contract requires
- Site: src/diag/diag_stats.cpp:120-125
- Lens: Declaration scope & object lifetime -- CCG CP.2 "Avoid data races" (unsynchronized read while another context writes); spine block B, unchecked API contract / passes-tests-yet-wrong.
- Claim: dump_ao_queue() calls QEQueue_getFree(), QEQueue_getMin() and QEQueue_getUse() as three separate unprotected reads of a live AO queue, but lib/qep/qf_qeq.c:274-296 documents each getter as not applying a critical section, and QP's own wrapper QActive_getQueueUse() (lib/qep/qf_actq.c:315-341) brackets the identical call in QF_CRIT_ENTRY/QF_CRIT_EXIT.
- Why: diag_stats_dump() runs from the CLI handler (src/cli/rc_os_debug.cpp:138) while the QF tick ISR posts time events into these same queues, so nFree can change between the three reads and the printed line is not a coherent sample. The error is bounded, though: nMin is by construction <= nFree at every instant, so depth (free_slots + use_now) is always >= min_free and the uint16_t subtraction at line 125 cannot underflow -- the high=65535 outcome is not reachable. What a torn read actually produces is depth and high_water off by about one slot in either direction (a drain landing between getFree and getUse under-reports; a post over-reports). Small, but these are the numbers the soak procedure reads back as queue-pressure evidence, and the skew is avoidable by bracketing the three getters the way QP's own QActive_getQueueUse() does.
- Confidence: high
- Direction: Bracket the three getter calls in one QF_CRIT_ENTRY/QF_CRIT_EXIT pair so the sample is coherent, or take the total depth from the AO's configured queue length instead of deriving it from two live counters.
- Verdict: RESHAPED -- the unsynchronized triple read and both QP contract citations check out verbatim (qf_qeq.c:274-296 documents no crit-section; qf_actq.c:315-341 brackets the same call), but the claimed uint16_t underflow to high=65535 is impossible because nMin <= nFree always; the real consequence is a roughly one-slot skew.

### CW-B29-04 -- Resolver contract omits the beacon overlay that can override its documented result
- Site: src/notify/notify_resolver.h:6-11, 26-28
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions and limitations stated in the preamble) and CCG NL.2; contract-surface helper Kind C, "does the signature match the prose".
- Claim: The header states that resolve_led_pattern returns the rc::led::k* code for the first non-kNone category and that the idle fallback returns kSensorNoGps, but src/notify/notify_backend_led.cpp:136-143 passes every result through apply_beacon_overlay(), which can replace the returned code entirely -- the header prose was never updated for the Stage L overlay.
- Why: With NotifyState.beacon_manual set, a kImuFail fault resolves to kFdBeacon (pure white), not the fault code the header describes, and under beacon_auto the documented kSensorNoGps idle fallback takes the overlay's default arm and also becomes kFdBeacon -- so two of the header's three behavioural sentences are conditional on state the header never mentions. Narrowed from the original: this header declares itself internal at :13-14, and both of its stated consumers already carry the overlay -- notify_backend_led.cpp documents it at :103-110 and :131-135, and test/test_notify.cpp:271-370 tests both beacon arms -- so the exposure is to a future reader or a future second backend, not to a live mis-reading by any code that exists today.
- Confidence: high
- Direction: Add one sentence to the header contract naming beacon_manual and beacon_auto as post-resolution overrides and pointing at the overlay table in notify_backend_led.cpp, rather than restating the mapping.
- Verdict: RESHAPED -- the header prose is genuinely stale against the Stage L overlay, but the "host tests read this contract and are misled" framing does not hold: the header declares itself internal and both of its consumers already document or test the overlay.

### CW-B29-05 -- "Pure read-only, no state mutation, no risk" is contradicted by the T=0 SPI probe
- Site: src/diag/diag_stats.h:9-14, 30-31 (body at src/diag/diag_stats.cpp:44-45, 60-61)
- Lens: Comments & documentation quality -- CCG NL.2 (comment and code disagree) and JSF AV 134 (undocumented limitation); spine block C, functionally-correct-but-safety-blind hardware code.
- Claim: The header justifies running diag_stats_dump() unconditionally on the flight binary because it is "a pure read-only snapshot of AO queue depth, MSP high-water, radio counters, health, and sensor state -- no state mutation, no risk" and "safe to run from any phase", but the dump's first act is diag_stats_t0_preconditions(), which drives an SPI transaction to the radio via rfm95w_read_version() into spi_bus_read_reg(), and that path calls count_error_if(), which does g_spi_error_count.fetch_add(1) on a short transfer (src/drivers/spi_bus.cpp:46-59).
- Why: The enumerated justification lists only software-state reads, so the hardware transaction is invisible to a reader deciding whether pressing the debug 'd' key is safe in BOOST. Two concrete consequences: the dump asserts the radio chip-select line and clocks the SPI bus the flight radio owns, and on any failed transfer the probe increments the same g_spi_error_count that lines 60-61 print two statements later as evidence of hot-path SPI health -- the measurement perturbs the metric it reports, so a rising SPI error count can be an artifact of repeated dumping rather than a hot-path fault. The sibling 'l' key at src/cli/rc_os_debug.cpp:141-144 is gated precisely because it is "state-mutating", which shows the project applies that test and that this function was judged against an inaccurate description of itself.
- Confidence: medium
- Direction: Correct the header to say the dump performs one SPI register read on the radio and can increment the SPI error counter, and state the phase assumption explicitly instead of claiming "any phase"; alternatively split the SPI probe out of the always-on read-only path.
- Verdict: CONFIRMED -- diag_stats_dump() at :248 calls diag_stats_t0_preconditions(), whose first act is rfm95w_read_version() -> spi_bus_read_reg() -> count_error_if() (spi_bus.cpp:46-50, 52-62), so the header's "no state mutation, no risk" is false and the SPI error counter printed at :60-61 can be incremented by the probe itself; the cited decision doc calls the dump "observational" but never mentions the SPI transaction.

### CW-B29-06 -- Zero sentinel is documented as not a valid pattern code, but kOff is zero
- Site: src/notify/notify_backend_led.cpp:30 (sentinel returns at 41, 43, 56, 58, 73-76, 84, 86, 98, 100)
- Lens: Comments & documentation quality -- CCG NL.2 (comment and code disagree); contract-surface helper Kind D, single-vocabulary check against include/rocketchip/led_patterns.h.
- Claim: The comment states that 0 is "not a valid pattern code" and is therefore safe as the per-category absent sentinel, but include/rocketchip/led_patterns.h:37 defines rc::led::kOff = 0 as a real pattern code meaning "Normal NeoPixel behavior".
- Why: The sentinel and a legitimate member of the pattern vocabulary occupy the same value, so the five per-category helpers cannot express "this intent maps to kOff" -- such an intent would be silently read as absent and the resolver would fall through to the next category. Nothing maps to kOff today so there is no live bug, but the comment tells a future author the collision cannot exist, which is the condition under which it gets introduced. It also leaves the pattern-code vocabulary two-valued at 0 across two files meant to share one catalog.
- Confidence: medium
- Direction: State the sentinel honestly -- 0 means "no intent in this category" here and deliberately aliases kOff, which no intent may map to -- or return a distinct sentinel outside the pattern range.
- Verdict: REFUTED -- 0/kOff is already the project's designated "no overlay / normal" value system-wide (ao_rcos.cpp:325-326 maps kCalNeoOff straight to CalIntent::kNone), so the aliasing is deliberate rather than an unnoticed collision, resolve_led_pattern can never return 0 (line 143 falls back to kSensorNoGps), and the finding itself concedes there is no live bug on any path.

### B30 -- telemetry + station

#### Coverage

- src/station/station_idle_tick.h -- PARTIAL -- Thin contract surface (helper Kind C: two prototypes plus prose); walked as a contract, and the banner's stated behaviour no longer matches the implementation it fronts (CW-B30-01).
- src/station/station_idle_tick.cpp -- PARTIAL -- Two functions read whole; spine A/B and scope-lifetime clean (the file-scope statics are the mandated persistent-snapshot kind), but the tick fuses two operations behind one precondition (CW-B30-02), and the line-53 comment still names the pre-rename s_lastGpsReadUs for what is now g_lastGpsReadUs.
- src/telemetry/mavlink_rx.cpp -- FAIL -- All 13 functions walked against include/rocketchip/mavlink_rx.h as one work product; the documented response contract is not delivered by the only production caller (CW-B30-03), placeholder ACK semantics and their comments are stale (CW-B30-04), the opaque parser buffer is unasserted (CW-B30-05), and a self-include test artifact ships in flight source (CW-B30-06).
- src/telemetry/telemetry_encoder.cpp -- PARTIAL -- All 12 encode/decode functions walked; header/CRC construction is consistent and the packet-length claims are static_assert-backed in the header, but the 40-byte payload offset claim is comment-only (CW-B30-07) and the nav decoder validates its length parameter after dereferencing the buffer (CW-B30-08); also kCrcOffset / kCrcLoIdx at :361-362 are defined and never used anywhere in src/, include/ or test/, and their "CRC starts at byte 52" comment holds only for the 54-byte APID 0x001 packet, not the 58-byte APID 0x004 packet this same decoder now handles.

#### Findings

### CW-B30-01 -- Header banner still describes the IVP-140 no-op scaffold
- Site: src/station/station_idle_tick.h:10 (banner 3-11; declarations 18-21)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation must describe code that actually runs) plus JSF AV 134 (assumptions/limitations belong in the preamble); contract-surface helper Kind C.
- Claim: The header states "IVP-140: scaffolding only. Tick body is a no-op. GPS poll lands in IVP-141," but the .cpp it declares is the IVP-141 implementation that polls GPS, samples the MCU die temperature and publishes a seqlock snapshot.
- Why: This header is the whole contract a caller sees. main.cpp:455 calls rc::station_idle_tick() from qv_idle_bridge() on every idle pass; a reader of the header concludes the call is free and side-effect-free, when it can in fact perform a blocking I2C GPS read that the .cpp banner itself budgets at ~6 ms worst case on Fruit Jam, and mutates g_sensorSeqlock. The one limitation the header does record ("Rate-limited internally") is the mild one; the cost and the side effect -- the two facts a caller needs before adding a second call site -- are absent, and the text that is present actively denies them.
- Confidence: high
- Direction: Replace the IVP-140 scaffold banner with the current contract: what the tick does, its worst-case blocking cost, that it writes g_sensorSeqlock, and its precondition on GPS init. The .cpp banner already holds that material; the header should carry the caller-facing half rather than a superseded status note.
- Verdict: CONFIRMED -- station_idle_tick.h:10 says verbatim "IVP-140: scaffolding only. Tick body is a no-op. GPS poll lands in IVP-141," while the .cpp it fronts is the IVP-141 body that calls core1_read_gps(), samples MCU temp and calls seqlock_write(); main.cpp:455 is the live caller.

### CW-B30-02 -- GPS-init guard also suppresses MCU-temp capture and the seqlock publish
- Site: src/station/station_idle_tick.cpp:71-99 (guard at :72, temp block at :88-94, publish at :99)
- Lens: The spine, block A -- CppCoreGuidelines F.2 (a function should perform a single logical operation); reinforced by Comments (JSF AV 134 / CCG NL.2, against the file's own claim at :58-59).
- Claim: station_idle_tick() returns at :72 when !g_gpsInitialized, so the MCU die-temperature capture and the seqlock publish -- neither of which depends on GPS -- never run on a station whose GPS failed to initialise.
- Why: The comment at :58-59 asserts the on-die temperature is "captured on both roles," and the vehicle peer path shows that is the intent: in src/core1/sensor_core1.cpp:418-430 the MCU-temp block is a sibling of the GPS block, gated only by mcu_temp_available(), so a vehicle with a dead GPS still reports die temperature. On a station where main.cpp never sets g_gpsInitialized (GPS absent or init failed), this tick is the only writer of g_sensorSeqlock -- Core 1 stays idle for the station role (main.cpp:344-347), and the only two seqlock_write() call sites in the tree are sensor_core1.cpp:454 and this file's :99 -- so the seqlock is never written at all. On that path: health_monitor.cpp:235 short-circuits on mcu_temp_read_count == 0 and returns kHealthAbsent, so MCU temperature is never classified and the over-temperature critical check at health_monitor.cpp:487-489 (which also requires mcu_temp_read_count > 0) can never fire; and the -999.0F sentinel that station_idle_tick_init() writes at :68 with the comment "Sentinel so seqlock readers don't see 0.0 C before first capture" is never published, so diag_stats.cpp:233 takes the > -100.0F branch on the zero-initialised snapshot and prints "MCU temp=0.00C" -- exactly the outcome that sentinel comment promises to prevent. Narrowed from the original: the preflight CLI is NOT affected -- rc_os_commands.cpp:1447 also tests hs->mcu == kHealthAbsent, which is what evaluate_mcu_temp returns on this path, so it correctly prints "--- (sensor not ready)".
- Confidence: high
- Direction: Move the temperature capture and the seqlock publish out from behind the GPS guard so the tick reads as peer steps (poll GPS if initialised; sample temperature if available; publish), matching the vehicle Core 1 structure. Publishing the sentinel once from station_idle_tick_init() would additionally make the comment at :67 true on every path.
- Verdict: RESHAPED -- the fused guard, the never-written seqlock and the dead MCU-temp classification all check out, but one named consequence is wrong: rc_os_commands.cpp:1447 short-circuits on hs->mcu == kHealthAbsent and prints "--- (sensor not ready)", so only diag_stats.cpp:233 shows the fabricated 0.00 C.

### CW-B30-03 -- Documented response path is not delivered by the only production caller
- Site: include/rocketchip/mavlink_rx.h:83-99 (contract) and src/telemetry/mavlink_rx.cpp:302-317 (implementation); response builders at :67-106 and :123-255
- Lens: Comments & documentation quality -- CERT MSC12-C (doc-comment describing a path the system cannot take) plus CCG NL.2; spine block B (green tests on an unreachable path -- the machinery is exercised only by test/test_mavlink_rx.cpp).
- Claim: The header documents that mavlink_rx_feed_byte writes response frames into result->buf and that "Caller is responsible for writing result->buf to stdout/USB," but the sole production caller allocates result as a discarded stack local and never transmits it, so no PARAM_VALUE, COMMAND_ACK, AUTOPILOT_VERSION or MISSION_COUNT frame this file builds can reach a GCS.
- Why: src/active_objects/ao_telemetry.cpp:342-355 is the only call site in src/ (grep for mavlink_rx_feed_byte and MavlinkRxResult returns this file plus the declaration). It declares rc::MavlinkRxResult result = {} inside the loop, passes it, then lets it go out of scope unread. Reachability is worse than the discard: mavlink_rx_feed_byte is invoked only inside the if (mavlink_parse_char(MAVLINK_COMM_2, ...)) body, so the parser on kRxChannel (MAVLINK_COMM_1, :36) is fed only the final byte of each already-completed frame and will essentially never assemble a message -- meaning dispatch_message at :261 is unreachable in flight even before the result is dropped. The QGC-compatibility surface the header advertises (param list, param read/set, mission count, capability ACK) is therefore documented, unit-tested and non-functional on hardware, which is precisely the shape a green host-test run conceals.
- Confidence: high
- Direction: Decide the module's status and make the documents match it -- either wire the call site to feed every RX byte to this parser and write result->buf back out to the CDC/radio path, or state plainly in the header that this is a host-tested protocol library with no live consumer. Leaving a full doxygen contract on an unreachable path is the defect.
- Verdict: CONFIRMED -- grep confirms the only production caller is ao_telemetry.cpp:349-352, which declares MavlinkRxResult as a loop-local, passes it and never reads it; the call sits inside the MAVLINK_COMM_2 parse-success branch so the COMM_1 parser at mavlink_rx.cpp:36 is fed one byte per completed frame, and the header at :87-88 still tells the caller to transmit result->buf.

### CW-B30-04 -- Pre-Flight-Director placeholders ACK safety commands as ACCEPTED
- Site: src/telemetry/mavlink_rx.cpp:201-213 (DO_SET_MODE at :201-208, ARM_DISARM at :210-213); same decision duplicated at :230-239
- Lens: Comments & documentation quality -- CERT MSC12-C / CCG NL.2 (stale comment describing a superseded system state); spine block B (spec-noncompliance -- a status return that does not describe what was done).
- Claim: ARM_DISARM returns MAV_RESULT_ACCEPTED unconditionally at :212 and DO_SET_MODE returns it whenever the phase is kIdle (:203-206), for commands this dispatcher does not execute, and the comments justifying that ("Pre-Flight Director: accept only if currently IDLE", "Pre-Flight Director: ACK but no-op. IVP-67 wires to real ARM.") describe a pre-Flight-Director system state that no longer exists.
- Why: The Flight Director has landed -- src/flight_director/ holds flight_director.cpp, command_handler.cpp and action_executor.cpp, and this very file includes flight_director/flight_state.h and reads rc::FlightPhase::kIdle at :203 -- while real MAV_CMD_COMPONENT_ARM_DISARM handling now lives in ao_telemetry.cpp:278-284, which dispatches SIG_ARM/SIG_DISARM to the Flight Director. So the promised IVP-67 wiring is not pending; it was done elsewhere and this branch was left behind as a second, silently diverging ARM implementation. The comments are the only thing telling a reader that ACCEPTED here means "nothing happened" -- delete them and the code claims a successful arm. Narrowed on two points: "IVP-67" does still appear elsewhere, in this module's own header at include/rocketchip/mavlink_rx.h:16; and per CW-B30-03 no production path reaches this dispatcher, so the "a GCS is told the vehicle armed" exposure is conditional on wiring that does not exist. The live defect is the stale comment set plus a duplicate ARM branch left in the tree.
- Confidence: medium
- Direction: Return MAV_RESULT_UNSUPPORTED (or DENIED) for the commands this dispatcher deliberately does not execute, and replace the "Pre-Flight Director / IVP-67" comments with a pointer to the authoritative handler in ao_telemetry.cpp. If the branch is genuinely superseded, delete it rather than leaving a second ARM path in the tree.
- Verdict: RESHAPED -- the ACCEPTED returns and the stale "Pre-Flight Director" comments are verbatim as cited and the authoritative ARM handler really is ao_telemetry.cpp:278, but "IVP-67 appears nowhere else" is wrong (mavlink_rx.h:16) and the wrong-ACK-to-a-GCS consequence is unreachable while this dispatcher has no live caller.

### CW-B30-05 -- Opaque parser buffer is hand-sized in a comment with no compile-time guard
- Site: include/rocketchip/mavlink_rx.h:52-60 (buffer at :55) and src/telemetry/mavlink_rx.cpp:305-309
- Lens: Comments & documentation quality -- JSF AV 134 (a load-bearing assumption stated only in prose) and CCG NL.2; spine block B (unverified layout claim on a vendored API). Note: static_assert-guard presence is listed as currently ungated in the project's own mechanical backlog, so I am treating this as manual residue, though I am not certain no other gate covers it.
- Claim: parser_buf[320] is sized by a trailing comment -- "sizeof(mavlink_message_t) + sizeof(mavlink_status_t) + padding" -- and the .cpp reinterpret_casts that raw byte array into a mavlink_message_t* at offset 0 and a mavlink_status_t* at offset sizeof(mavlink_message_t), with no static_assert anywhere that 320 is actually large enough.
- Why: The header forward-declares both MAVLink types (mavlink_rx.h:28-31) precisely so it cannot see their sizes, so the number 320 cannot be checked where it is written; the .cpp, which does include common/mavlink.h and therefore could check it, does not. lib/mavlink is an unpopulated submodule in this tree, so the remaining margin cannot be inspected at all right now. If a MAVLink library bump grows either struct past the hand-computed budget, the second cast at :308-309 hands mavlink_parse_char a pointer to an object extending past parser_buf into the adjacent MavlinkRxState members (encoder, gcs_sysid, gcs_compid, gcs_seen) -- silent corruption of a borrowed encoder pointer, with no compiler diagnostic and no test that would notice, because the sizes are correct today.
- Confidence: medium
- Direction: Add a static_assert in the .cpp, where both types are complete, asserting sizeof(MavlinkRxState::parser_buf) >= sizeof(mavlink_message_t) + sizeof(mavlink_status_t), so a library bump fails the build instead of the field.
- Verdict: CONFIRMED -- parser_buf[320] is sized only by the trailing comment at mavlink_rx.h:55, the .cpp reinterpret_casts it into mavlink_message_t* at offset 0 and mavlink_status_t* at offset sizeof(mavlink_message_t) (:306-309), and grep finds no static_assert in either file; the .clang-tidy misc-static-assert check does not test for the presence of a guard, and lib/mavlink is empty in this tree so the remaining margin cannot even be inspected.

### CW-B30-06 -- Self-include test artifact left in flight source
- Site: src/telemetry/mavlink_rx.cpp:14-15
- Lens: Comments & documentation quality -- CERT MSC04-C (no developer scaffolding parked in place of removal) and JSF AV 131 / CCG NL.1.
- Claim: The translation unit includes its own header twice, with the second occurrence annotated "// double-include guard test" -- a one-off developer probe committed into flight source.
- Why: The line does nothing at build time (the guard at mavlink_rx.h:21-22 absorbs it), which is exactly why it survives every gate and every pass that reads for correctness. Its cost falls on the reader: it labels an ordinary include as a test, so the next person editing the include block has to work out whether deleting it breaks something, and it sets the precedent that ad-hoc probes may live in src/. There is no build variant or platform for which the duplicate is needed, so the "#if 0 with a stated reason" escape does not apply.
- Confidence: high
- Direction: Delete line 15. If double-inclusion is worth asserting at all, it belongs in the header test suite, not in a flight translation unit.
- Verdict: REFUTED -- mechanically gated: `readability-duplicate-include` is enabled in .clang-tidy, src/telemetry/mavlink_rx.cpp is in ROCKETCHIP_SOURCES (CMakeLists.txt:115/442/587) and is not on the full_tree_clang_tidy.sh exemption list (src/cli/**, eskf_codegen.cpp), so the duplicate include at :15 is exactly what that check reports.

### CW-B30-07 -- The 40-byte wire payload boundary is asserted nowhere
- Site: src/telemetry/telemetry_encoder.cpp:58-61 and :96-109 (encoder), :409 (decoder)
- Lens: Comments & documentation quality -- JSF AV 134 / CCG NL.2 (a load-bearing contract carried only by prose); spine block B (spec-noncompliance -- a requirement the code depends on that no gate checks).
- Claim: kTelemPayloadBytes = 40 and the layout comment at :99-102 ("bytes 0-39: q_w through battery_mv, bytes 40-43: met_ms, byte 44: _reserved") are the only statement that met_ms begins at offset 40 of TelemetryState; telemetry_state.h:58 asserts only sizeof(TelemetryState) == 45, not any field offset.
- Why: write_nav_payload_42 at :103-109 memcpy's the first 40 raw bytes of the struct straight onto the wire, and the decoder at :409 memcpy's them straight back. A future edit that reorders or resizes fields inside TelemetryState while keeping the total at 45 bytes -- swapping two same-width members, or moving met_ms earlier and _reserved later -- keeps the size assert green, keeps the packet-length asserts in telemetry_encoder.h green, and keeps every host round-trip test in test_telemetry_encoder.cpp green, because encoder and decoder shift together. What breaks is the wire format: a ground station on any other build decodes the CCSDS APID 0x001 / 0x004 payload with the old field meanings, so latitude, altitude or battery voltage land in the wrong columns behind a valid CRC. This is the layout-claim-in-a-comment hazard on a standards-fidelity path.
- Confidence: medium
- Direction: Add offsetof-based static_asserts beside the constants -- at minimum that met_ms sits at kTelemPayloadBytes -- so the comment's claim is compiler-enforced rather than reader-enforced.
- Verdict: CONFIRMED -- grep over telemetry_state.h, telemetry_encoder.h and telemetry_encoder.cpp finds only sizeof/packet-length static_asserts (telemetry_state.h:58; telemetry_encoder.h:105/115/130/143) and no offsetof guard, while write_nav_payload_42 (:103-108) and the decoder (:409) both memcpy the first 40 raw struct bytes, so a size-preserving field reorder passes every gate and silently changes the wire format.

### CW-B30-08 -- ccsds_decode_nav dereferences the buffer before validating its length
- Site: src/telemetry/telemetry_encoder.cpp:368-389 (reads at :372 and :376-377; length check at :385-389)
- Lens: The spine, block B -- Power of Ten Rule 7 (parameter validity must be checked inside each function), with the OWASP LLM05 residual the manual keeps for byte-stream parsers on untrusted input.
- Claim: The function reads buf[0] and buf[1] to validate the version field and extract the APID before it consults len, so a caller passing a buffer shorter than two bytes gets an out-of-bounds read instead of a clean false.
- Why: The input is radio-received bytes -- the live caller is ao_telemetry.cpp:613 on rx_evt->buf / rx_evt->len -- the least trustworthy surface in the system. The current call site happens not to fault, because RadioRxEvt::buf is a fixed 256-byte array (ao_signals.h:158-164), so even a len == 0 event reads in-bounds stale bytes; the host test at test_telemetry_encoder.cpp:565 passes len = 0 against a full 54-byte buffer for the same reason, so the test that asserts short packets are rejected never exercises a genuinely short allocation. The defect is therefore latent rather than live today, and it is a contract defect: the function accepts (buf, len) and does not honour len on its first two reads. The same file shows the right shape a hundred lines later -- ccsds_decode_cmd_ack at :467-479 checks len != kExpectedLen before touching buf[0] -- so the two sibling decoders disagree about their own precondition.
- Confidence: medium
- Direction: Move a minimum-length check (at least ccsds::kPrimaryHeaderLen) to the top of ccsds_decode_nav, ahead of the version and APID reads, matching ccsds_decode_cmd_ack; the existing exact-length check can stay where it is once the APID has selected the expected size.
- Verdict: CONFIRMED -- buf[0] is read at :372 and buf[1] at :377, and len is not consulted until :387; the sibling ccsds_decode_cmd_ack checks len != kExpectedLen at :472 before touching buf[0] at :477, and the live caller (ao_telemetry.cpp:613) passes the fixed 256-byte RadioRxEvt::buf (ao_signals.h:158-164), which is exactly why the defect is latent rather than live.

---

## Tier 3 — Integrators (safety, core1, active objects, top level)


### B31 -- safety: fault_protection + anomalous_boot

#### Coverage
src/safety/anomalous_boot.h -- PARTIAL -- Contract surface (Kind C) read whole; the preamble states a "veto + 2-of-N corroborator" classification and an accessor validity precondition that the implementation does not deliver or enforce.
src/safety/anomalous_boot.cpp -- PARTIAL -- All four public functions, both static helpers and both file-scope mask constants walked with spine A/B/C; the sentinel path is sound but the corroborator half is inert and the accessors carry no precondition check.
src/safety/fault_protection.h -- PARTIAL -- Declaration/contract surface read whole; the documented visible-signal behaviour and the "registered for both cores" claim are not observable in the tree.
src/safety/fault_protection.cpp -- FAIL -- Both handlers, all three static helpers and mpu_setup_stack_guard walked; several load-bearing comments describe behaviour the bodies do not perform, and the MPU guard setup has no precondition and no verification.

Assertion-lens note recorded rather than filed: the absence of assertions inside memmanage_fault_handler and Q_onError is correct by design here (an assertion tripping inside Q_onError would recurse through the fault path), so JPL-16 density is not a finding for those two bodies. mpu_setup_stack_guard is the one non-handler body where the rule bites -- see CW-B31-06.

#### Findings

### CW-B31-01 -- Q_onError logs from the context rc_log.h forbids, and the diagnostic it claims to emit cannot reach anyone
- Site: src/safety/fault_protection.cpp:205-212 (contract at include/rocketchip/rc_log.h "PROHIBITED" block)
- Lens: Comments & documentation quality -- JSF AV 134 / CCG NL.2 (comment and code disagree); contract-surface helper Kind C (prose vs use); spine B (unearned confidence in a defensive-looking branch)
- Claim: Q_onError calls rc::rc_log() from fault-handler context, which the rc_log.h PROHIBITED section explicitly excludes, and the adjacent comment's assertion that the module string and id are "printed live to serial below" cannot hold on either dispatch path.
- Why: rc_log.h states the sink is "a non-blocking ring buffer drained by tud_task on Core 0's main loop" and lists "ISR / fault-handler context -- handlers do not log" as prohibited. Line 178 executes cpsid i before the call, so tud_task cannot run to drain the ring; on the kIdle path fault_reset_with_visible_signal() then resets the chip roughly 50 ms later, discarding it. Concrete case: a QP state-machine assertion on the pad writes reason = kCrashReasonNone at line 198, which is the same value as "no reason recorded", so after the reset the operator gets a prior-hardfault latch with no module, no id and no reason category -- exactly the identifying information the comment at 188-192 says survives. rc_log also formats through a 128-byte per-call stack buffer, which is the kind of C-level call crash_record.h:66-71 says the handler deliberately avoids ("fault-on-fault -> lockup risk ... when the failing stack is barely above the MPU guard").
- Confidence: high
- Direction: Reconcile the two documents -- either amend the rc_log.h PROHIBITED clause to carve out an explicitly best-effort fault-handler use that may be lost, or drop the call and carry the identity in the record instead, adding a QP-assert CrashReason plus module/id fields so the assertion is identifiable after the reset.
- Verdict: CONFIRMED -- rc_log.h:45-46 lists "ISR / fault-handler context -- handlers do not log" as PROHIBITED and Q_onError calls rc::rc_log at fault_protection.cpp:212 after cpsid i at :178; neither dispatch tail can ever drain the ring (kIdle resets ~50 ms later, any flight phase busy-loops in fault_degrade_in_place with interrupts masked), and rec->reason is written kCrashReasonNone (= 0 = "no reason recorded") at :198.

### CW-B31-02 -- fault_reset_with_visible_signal emits no visible signal and waits 50 ms for a drain that cannot occur
- Site: src/safety/fault_protection.cpp:44-48 and 84-97 (claims also at src/safety/fault_protection.h:36-40 and 51-53)
- Lens: Comments & documentation quality -- CERT MSC12-C (comment describing a path the code does not take) and CCG NL.2; spine B (confabulated justification)
- Claim: The function named for emitting a visible signal calls an empty placeholder, and its 50 ms busy-wait is justified by a USB CDC drain that the interrupts-disabled handler context makes impossible.
- Why: fault_emit_visible_signal() at 44-48 has an empty body with a TODO; its comment says "Serial banner via printf below is the visible signal", but memmanage_fault_handler contains no print of any kind -- it writes the crash record and dispatches. So on the pad path the operator observes a spontaneous reboot with nothing on the wire and no LED, while header lines 51-53 restate the emission as fact. The 50 ms busy_wait_us at line 95 is justified as letting "in-progress USB CDC transmission of the serial banner ... drain", but cpsid i at line 107 (and at 178) masks the USB interrupt for the whole handler, so TinyUSB cannot move a byte during that window on either entry point.
- Confidence: high
- Direction: Downgrade the emission claims in the header and the two comment blocks to an explicit not-yet-implemented note, and either drop the 50 ms wait or re-justify it against a drain mechanism that actually runs with interrupts masked -- a directly polled CDC flush, or the raw-GPIO LED toggle the TODO already names.
- Verdict: CONFIRMED -- fault_emit_visible_signal (:44-48) is an empty body whose comment names a "serial banner via printf below" that memmanage_fault_handler (:106-158) never emits, while fault_protection.h:36-38 and :51-53 restate the banner as fact; the 50 ms busy_wait at :95 is justified as a USB CDC drain window, but cpsid i at :107 and :178 masks the USB interrupt for the whole handler.

### CW-B31-03 -- The one-shot flag rationale contradicts the crash-record design it sits next to
- Site: src/safety/fault_protection.cpp:29-33
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine B (confabulation: a confident justifying mechanism that is not the real one)
- Claim: The comment justifies never clearing g_inFaultHandler by asserting that the AIRCR reset "wipes SRAM-to-the-power-domain (the flag's storage)", which contradicts the SRAM-survives-reset premise the same handler depends on.
- Why: The reset the comment refers to is the AIRCR.SYSRESETREQ write in fault_trigger_reset() at 52-63, and crash_record.h:51 states the record lives in .uninitialized_data and "survives NVIC_SystemReset()" -- the same reset. The capture-then-reset design at lines 136-145 only works because SRAM is not wiped, so both statements cannot be true. The flag is in fact false after reset because it is an initialized static that C runtime startup re-zeroes, which is a different mechanism entirely. A reader who takes the comment at face value would conclude post-AIRCR SRAM contents are unavailable, and could either duplicate the crash record into flash or distrust a valid record on the next boot.
- Confidence: high
- Direction: Replace the SRAM-wipe rationale with the real one (statically initialized, re-zeroed by startup); if the one-shot property is load-bearing, state it as an invariant rather than deriving it from a storage claim.
- Verdict: CONFIRMED -- the comment at :29-33 derives the one-shot property from an AIRCR reset that "wipes SRAM-to-the-power-domain", which directly contradicts crash_record.h:51 ("survives NVIC_SystemReset()") and crash_record.cpp:12-15, both of which the capture-then-reset design at :136-145 depends on; the flag is in fact false after reset because it is a zero-initialised static re-zeroed by C startup.

### CW-B31-04 -- One non-atomic flag serves as a reentrance guard for two entry points and, per the header, two cores
- Site: src/safety/fault_protection.cpp:34, 110-115, 181-186 (registration claim at src/safety/fault_protection.h:48-49)
- Lens: Spine C -- volatile-as-cross-core-barrier ADD (detailed criteria live in the Concurrency lens, which this batch does not carry); project rule docs/MULTICORE_RULES.md "Never Use Plain volatile for Cross-Core Sharing"; CCG CP.2; comment claim per CCG NL.2
- Claim: g_inFaultHandler is a plain volatile bool with a non-atomic test-then-set shared by both fault entry points and -- per the header's "Registered for both cores" claim -- by both cores, which docs/MULTICORE_RULES.md forbids for cross-core sharing, while the comment at 24-27 describes it as a same-context reentrance guard when it is in fact a global one-shot.
- Why: The comment at 24-27 scopes the guard to "a second fault fires from inside the handler", but the flag is global and never cleared. The cross-core case is the one that genuinely bites: if Core 1 faults while Core 0 has already latched the flag, Core 1's entry at line 110 sees it set and drops straight to WFE with no crash record and no phase-aware dispatch; and the read-modify-write at 110/115 and 181/186 is not atomic and volatile issues no barrier, so two contexts can both pass the test and both write rc::g_crash_record, producing a mixed record that the magic-written-last discipline still accepts. Whether the cross-core case is live cannot be settled from this module: fault_protection.h:48-49 states the handler is "Registered for both cores", but the only registration in the tree is the single Core-0 pair at src/main.cpp:209-210, while Core 1 (src/core1/sensor_core1.cpp:472) installs only the MPU guard. Narrowed from the original: the "genuinely independent second fault on the same core is swallowed" path is withdrawn -- both dispatch tails end in an interrupts-masked WFE loop or an AIRCR reset, so any same-core second fault is by construction a fault raised from inside the handler, which is exactly what the guard is for.
- Confidence: medium (high on the comment-versus-behaviour gap; medium on the cross-core half, because the per-core vector-table question is not answerable from inside this module)
- Direction: State in the header which cores actually reach the handler, then make the guard say what it means -- a per-core flag if true reentrance is the intent, or an atomic test-and-set if it is meant as a system-wide first-fault-wins arbiter, with the swallow behaviour documented as deliberate.
- Verdict: RESHAPED -- the plain-volatile non-atomic global, the MULTICORE_RULES.md conflict and the unverifiable "registered for both cores" header claim all hold, but the named same-core second-fault path does not: after the flag is set, both tails halt with interrupts masked or reset, so any further same-core fault is a fault-from-inside-the-handler, which is the guard's stated purpose.

### CW-B31-05 -- The guard / capture / dispatch sequence is written out twice
- Site: src/safety/fault_protection.cpp:109-157 and 180-219 (crash-record write at 117-145 versus 193-203)
- Lens: Spine A -- CCG ES.3 "Don't repeat yourself" and CCG F.1 (a nameable action left inline at each call site); Fowler Duplicated Code
- Claim: memmanage_fault_handler and Q_onError each inline the same three-part idiom -- reentrance check, full CrashRecord field write with the dsb / magic / dsb ordering, phase-aware dispatch -- differing only in the fault-status values and the reason code.
- Why: Every field of CrashRecord is stored explicitly in both bodies, including reserved[0] and reserved[1]. The struct at crash_record.h:56-64 is documented as "pad to 32 bytes for future expansion", so the first added field must be written in both places or one path silently ships a partially-stale record while still writing the magic that makes the consumer trust it. The same exposure applies to the torn-write ordering: the dsb-then-magic-then-dsb discipline is safety-relevant and is currently maintained by hand in two locations. Because these are two functions rather than two branches, bugprone-branch-clone does not see it.
- Confidence: medium
- Direction: Extract one always-inline capture helper taking the reason plus the four status words, and one dispatch tail, so both entry points read as guard then capture then dispatch; keep both inline-only so the no-call constraint crash_record.h:66-71 describes for the handler still holds.
- Verdict: CONFIRMED -- the reentrance guard (:110-115 vs :181-186), the full seven-field CrashRecord write with dsb/magic/dsb (:136-145 vs :193-203) and the phase-aware dispatch tail (:149-157 vs :215-219) are byte-for-byte the same idiom in both bodies, so the "pad to 32 bytes for future expansion" reserved[] at crash_record.h:63 and the torn-write ordering are each maintained by hand in two places; bugprone-branch-clone does not see cross-function duplication.

### CW-B31-06 -- The stack guard is programmed with no precondition, no read-back and no failure path, yet a crash reason exists for its failure
- Site: src/safety/fault_protection.cpp:255-292 (declaration at src/safety/fault_protection.h:84; orphan reason code at src/safety/crash_record.h:46)
- Lens: Assertions -- P10 Rule 5 / JPL-C Rule 16 (a load-bearing function with no sanity check) and CCG P.5 (prefer compile-time checking); spine C "functionally-correct-but-safety-blind hardware code"
- Claim: mpu_setup_stack_guard consumes stack_bottom assuming 32-byte alignment, never checks it, never reads back the programmed region and returns void, so a guard that did not take effect is indistinguishable from one that did.
- Why: Line 267 masks the base down with and-not-0x1F and line 273 masks the limit down the same way. If a caller passes a symbol that is not 32-byte aligned -- the two call sites pass linker-provided addresses, __StackBottom at src/main.cpp:211 and __StackOneBottom at src/core1/sensor_core1.cpp:472, whose alignment is a property of the linker script and not of anything this function can see -- the region silently starts below the stack bottom and spans something other than the 64 bytes kMpuGuardSizeBytes documents, marking non-stack memory read-only. Nothing asserts that. The verification gap is the higher-consequence half: the comment block at 231-249 in this same file records that this guard was already silently ineffective once, through a wrong AP encoding that survived until R-3; and crash_record.h:46 defines kCrashReasonMpuConfigFail as "mpu_setup_stack_guard() couldn't configure" -- a reason src/safety/health_monitor.cpp:314 can print but nothing in the tree can ever emit, because this function has no failure detection and no return value.
- Confidence: medium-high (high that no check or verification exists and that the reason code has no producer; medium on the practical alignment exposure, since the linker script was not in this batch)
- Direction: Read back rnr / rbar / rlar / ctrl after programming and confirm the region matches what was requested; give the function a status result, or an explicit crash_record_capture(kCrashReasonMpuConfigFail, ...) on mismatch so the existing reason code has a producer, and check the incoming address alignment rather than silently masking it.
- Verdict: CONFIRMED -- mpu_setup_stack_guard (:255-293) returns void, masks stack_bottom down at :267 and :273 with no alignment check and no read-back, and grep shows kCrashReasonMpuConfigFail has exactly one occurrence outside its own definition: the print at health_monitor.cpp:314, with no producer anywhere; the in-file R-3 note at :231-249 records that this same guard was already silently ineffective once.

### CW-B31-07 -- Accessors do not enforce their documented init precondition, and the uninitialized default is the unsafe verdict
- Site: src/safety/anomalous_boot.cpp:119-129 and 22-23 (precondition stated at src/safety/anomalous_boot.h:64-72)
- Lens: Assertions -- JPL-C Rule 16 (missing entry-point sanity check on a documented precondition, with the checkable state already present); Comments & documentation quality -- JSF AV 134 (the stated assumption is nowhere enforced)
- Claim: The header states the accessors are "Valid after anomalous_boot_init() runs" and that init "MUST be called exactly once, very early in main()", but none of the four accessors consults g_initialized, and the zero-initialized snapshot they return decodes to the module's own worst-case answer.
- Why: g_signals is a file-scope static, so before init it reads all-zero: verdict decodes to kProbablyOnPad because that is enumerator 0 at anomalous_boot.h:42, and had_bor reads false. The header's stated design bias at lines 17-20 is to "refuse to act as fresh pad on ambiguous evidence" precisely because a false negative is mission loss; a pre-init read delivers exactly the opposite, silently. The ordering holds today only by construction -- src/main.cpp:206 runs init before the consumers at src/main.cpp:368-382 -- but src/safety/health_monitor.cpp:330 calls anomalous_boot_brownout_detected() from a different module, so a future reorder that pulls health_monitor_init ahead of anomalous_boot_init returns "no brownout, probably on pad" with nothing to signal that the gate never ran. g_initialized already exists and is written at line 116; nothing ever reads it.
- Confidence: medium
- Direction: Assert g_initialized at the top of each accessor so a pre-init read trips the project fault path instead of returning a fabricated clean-pad answer, and consider giving the unsafe-side verdict a non-zero enumerator so a zero-initialized snapshot cannot decode as on-pad.
- Verdict: CONFIRMED -- all four accessors (:119-137) read g_signals with no reference to g_initialized, which is written at :116 and never read; kProbablyOnPad is enumerator 0 (anomalous_boot.h:42) so a zero-initialised snapshot decodes to the answer the header's own :17-20 bias exists to avoid, and health_monitor.cpp:330 calls one accessor from a different module.

### CW-B31-08 -- The documented 2-of-N corroborator path cannot fire; the verdict reduces to the sentinel alone
- Site: src/safety/anomalous_boot.cpp:62-64 and 72-86 (claims at src/safety/anomalous_boot.h:16-17 and 43)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation describing a path the code can never take) and CCG NL.2; spine B (a confident description the body does not implement)
- Claim: compute_verdict requires two corroborators, but only two corroborator slots exist and one is hardwired to zero by read_prior_uptime_ms(), so the non-sentinel branch always returns kProbablyOnPad.
- Why: read_prior_uptime_ms() returns 0U unconditionally at 62-64, so the test at line 79 against the 10000 ms threshold is never true, corroborators can never exceed 1, and the branch at line 82 is unreachable. The header sells the design as "veto + 2-of-N corroborator logic" and the enum comment at line 43 says kProbablyMidFlight fires on "Sentinel and/or 2+ corroborating signals", so a reader auditing this safety gate believes two independent evidence paths exist where there is one. Named path where that bites: crash_record.h:115-119 states a BOR reset clears .uninitialized_data, which is where the flight-in-progress sentinel lives, so in that case sentinel_was_set is false while had_bor and had_any_non_por are both true, corroborators reaches exactly 1, and the gate answers kProbablyOnPad. The in-file comment at 72-74 acknowledges the deferral but does not say the threshold has become unreachable, and had_run_low / had_any_non_por are captured at 102-105 for a decision they cannot influence.
- Confidence: high
- Direction: State plainly in the header and at compute_verdict that until the AON-timer corroborator lands the verdict is sentinel-only, or lower the threshold to match the number of live signals; if the reduced coverage is accepted, record it where the fault-recovery design is dispositioned rather than leaving the header describing the intended design as the shipped one.
- Verdict: CONFIRMED -- read_prior_uptime_ms() returns 0U unconditionally (:62-64), so the threshold test at :79 is never true, corroborators cannot exceed 1 and the branch at :82-84 is unreachable; the header sells "veto + 2-of-N corroborator logic" (:16-17) and the enum comment claims "2+ corroborating signals" (:43), and the in-file note at :58-59 even asserts the reset-cause signals are "load-bearing on their own" when one alone can never reach the threshold.

### B32 -- safety: flight_in_progress + crash_record + health_monitor

#### Coverage
src/safety/crash_record.cpp -- FAIL -- Both functions read whole; the capture path's stated torn-write protection is not established by the code (CW-B32-01).
src/safety/crash_record.h -- PARTIAL -- Contract surface (Kind C) walked in full: crash-record claims verified against crash_record.cpp and both writers in fault_protection.cpp; the flight-sentinel declaration block carries a predicate-named destructive read (CW-B32-05).
src/safety/flight_in_progress.cpp -- PARTIAL -- All three functions read whole; the #if-split declaration and the volatile/barrier story are sound and single-context (FD enter_phase on Core 0, boot consumer in anomalous_boot_init), but flight_in_progress_was_set() hides a state change behind a query name (CW-B32-05).
src/safety/health_monitor.cpp -- FAIL -- All 787 lines and every function read; four findings (CW-B32-02/03/04/06), the heaviest being an ignored seqlock return feeding every health decision and a comment set that denies an auto-action the code performs.
src/safety/health_monitor.h -- FAIL -- Contract surface walked in full: encodings, thresholds and the two inline pure helpers verified against their .cpp uses; the HealthCritical preamble states a no-auto-trigger rule the implementation breaks (CW-B32-04) and kHealthWatchdogOk is declared with no comment and a name the body does not honour (CW-B32-03).

#### Findings

### CW-B32-01 -- crash_record_capture() does not enforce the "magic last" ordering its comment relies on
- Site: src/safety/crash_record.cpp:29-42
- Lens: Comments & documentation quality (JSF 131/134; CCG NL.2 "if the comment and the code disagree, both are likely to be wrong"); spine block B confabulation (a confident rationale the body does not implement)
- Claim: The comment at lines 36-38 asserts that writing magic last makes a torn write reject on the next boot, but the six payload stores and the magic store are plain stores to a non-volatile struct with no barrier between them, so nothing in the code fixes their order.
- Why: g_crash_record is a plain (non-volatile) object, so the compiler may reorder or merge the stores at lines 29-39 freely; the only barrier is the dsb/memory clobber at line 42, which sits after the magic store and therefore orders nothing inside the sequence. If an asynchronous reset (brownout, RUN pin, external reset) lands mid-sequence -- the exact scenario the comment names -- the next boot can find magic == kCrashRecordMagic with stale cfsr/hfsr/stacked_pc from a previous capture, and health_monitor_init() (health_monitor.cpp:308-324) will latch kHealthCriticalPriorHardfault and print a fabricated fault address. The codebase itself shows the intended pattern: both other writers of this same record -- memmanage_fault_handler (fault_protection.cpp:136-145) and Q_onError (fault_protection.cpp:194-203) -- place a dsb plus memory clobber between the payload stores and the magic store, with the comment "magic last so torn writes reject on consume". Only crash_record_capture() omits it.
- Confidence: high
- Direction: Insert the same dsb plus memory clobber between the payload stores and the magic store so all three writers of this record use one pattern; alternatively declare the record volatile as flight_in_progress.cpp does for its sibling sentinel, and state in the preamble which mechanism the ordering rests on.
- Verdict: CONFIRMED -- crash_record.cpp:29-39 is seven plain stores to the non-volatile g_crash_record with the magic last and the only dsb at :42, after the magic; both sibling writers (fault_protection.cpp:136-145 and :193-203) place a dsb + memory clobber between the payload and the magic, so the inconsistency and the unbacked comment at :36-38 are exactly as claimed.

### CW-B32-02 -- Every health decision is computed from an unchecked seqlock_read()
- Site: src/safety/health_monitor.cpp:603-604, src/safety/health_monitor.cpp:741-742
- Lens: Spine block B unchecked returns / Power of Ten Rule 7 ("the return value of non-void functions must be checked by each calling function"). Manual residual, not gated: seqlock_read (include/rocketchip/sensor_seqlock.h:130) is not [[nodiscard]] and .clang-tidy CheckedFunctions is flash_safe_execute only.
- Claim: health_monitor_tick() and health_monitor_fill_go_nogo() discard the boolean that says whether the sensor snapshot is consistent, then evaluate every subsystem from that snapshot as if the read had succeeded.
- Why: seqlock_read returns false after kSeqlockMaxRetries collisions, and on that path dst holds either the caller's zero-initialised struct or a snapshot already memcpy-ed from an inconsistent generation (the memcpy at sensor_seqlock.h:137 runs before the seq2 check at :139). Core 1 writes this seqlock at roughly 1 kHz while the health tick reads it at 10 Hz, so the collision path is reachable. On failure at line 604 the whole tick proceeds on a false snapshot: accel_valid/baro_valid read as 0 and are pushed into both sliding windows (lines 159, 182), so a few such ticks drive invalid_count past kBaroDegradeThreshold (3/10) and report Baro DEGRADED with no sensor fault present; evaluate_critical() (line 480) reads mcu_die_temp_c from the same possibly-torn snapshot and can set kHealthCriticalMcu, which auto-DISARMs and latches launch-abort (see CW-B32-04). On failure at line 742, gng->gps_has_lock (lines 763-765) goes false and the operator gets a spurious GPS NO-GO at the moment they press ARM. The correct shape already exists in this same file: check_core1_vitality() (lines 366-368) checks the identical return and documents the fallback ("Can't read -- assume alive").
- Confidence: high
- Direction: Check the return at both sites and take a defined fallback -- skip the tick (leaving prev_* and the windows untouched) or reuse the last good snapshot -- rather than evaluating health from an unvalidated struct; state that fallback in the tick preamble.
- Verdict: CONFIRMED -- seqlock_read is declared without [[nodiscard]] at sensor_seqlock.h:130 and .clang-tidy's CheckedFunctions is 'flash_safe_execute' only, so nothing gates it; the returns really are dropped at health_monitor.cpp:604 and :742, the memcpy at sensor_seqlock.h:137 does precede the seq2 test at :139, snap.accel_valid / snap.baro_valid are pushed into the sliding windows at :159 and :182 regardless, and check_core1_vitality at :366-368 checks the identical return and documents its fallback.

### CW-B32-03 -- kHealthWatchdogOk reports ESKF state, not any watchdog
- Site: src/safety/health_monitor.h:57, src/safety/health_monitor.cpp:404-412
- Lens: Comments & documentation quality (JSF 131/134; CCG NL.2 state intent, CCG P.3 express intent -- the name is the only documentation this bit has)
- Claim: The bit named kHealthWatchdogOk is set solely from !eskf_is_disabled(), and is hard-coded true on any role that does not sample Core 1, yet it is surfaced to operators under the label "Watchdog" in four places.
- Why: The declaration at health_monitor.h:57 carries no comment at all (its siblings do), so the name is the contract. The body sets the bit from ESKF runaway-restart state only, and on station/relay roles the if constexpr (job::kRoleSamplesCore1) guard at line 407 leaves eskf_healthy at its initialised true, so the bit is unconditionally OK there. Everything downstream then reports a watchdog: log_secondary_transitions prints "HEALTH: Watchdog -> FAULT" (names array at line 458), the CLI prints "Watchdog: %s" (rc_os_commands.cpp:1439), and the preflight table adds a "Watchdog" row that reads "NO-GO SAFE MODE" (go_nogo_checks.cpp:94-95) via gng->watchdog_ok (line 754). No watchdog is consulted on any of those paths -- the PIO heartbeat watchdog has its own separate bit (kHealthPioOk, line 413) and the SDK hardware watchdog was removed from the tree at IVP-90 (recorded at fault_protection.cpp:168-172). An RSO reading "Watchdog: FAULT" during a pad check is being told the wrong thing: the actual condition is that the ESKF was runaway-restart-disabled.
- Confidence: high
- Direction: Rename the bit and its four operator-facing labels to what is computed (ESKF / safe-mode state, matching the FaultIntent::kSafeMode mapping at notify_resolver.h:53-55), or give the declaration a comment stating the real meaning; the GoNoGoInput field comment "No safe-mode, no ESKF disabled" (go_nogo_checks.h:42) is the closest existing statement of intent and only half of it is implemented.
- Verdict: CONFIRMED -- health_monitor.cpp:406-412 sets kHealthWatchdogOk from !eskf_is_disabled() alone and leaves eskf_healthy at its initialised true when job::kRoleSamplesCore1 is false, and the bit is surfaced as "Watchdog" at health_monitor.cpp:458, rc_os_commands.cpp:1439 and go_nogo_checks.cpp:94-95 ("NO-GO SAFE MODE"); the only inaccuracy is minor -- three of the four sibling enumerators at health_monitor.h:55-59 are also uncommented, not just this one.

### CW-B32-04 -- The critical byte auto-triggers a power-cycle-only launch-abort latch that three comments say it cannot trigger
- Site: src/safety/health_monitor.h:76-80, src/safety/health_monitor.cpp:703-719
- Lens: Comments & documentation quality (CCG NL.2 comment/code disagreement; JSF 134 documented assumptions); spine block B passes-tests-yet-wrong (an unexercised single-tick path)
- Claim: health_monitor_critical_fault() returns true whenever any HealthCritical bit is set, and its only caller uses that to auto-DISARM and latch launch-abort, while the header preamble, the evaluate_critical() preamble and the function's own comment all state that these bits do not trigger automatic state transitions.
- Why: health_monitor.h:76-80 states "Consumers do NOT auto-trigger state transitions from these bits -- bits exist so humans ... can abort manually"; health_monitor.cpp:470-477 repeats it; lines 706-716 repeat it a third time ("those are visibility-only and do not auto-trigger state transitions"). Line 717 then returns true on g_health.critical != 0, and ao_flight_director.cpp:138-143 turns that into SIG_DISARM plus flight_director_set_launch_abort() -- described there as "level 3 in the safety state model -- power-cycle-only clear". The reachable input is a single tick of mcu_die_temp_c >= kMcuTempSafeModeC while ARMED (evaluate_critical, lines 487-491): unlike the primary-byte faults it is not persistence-gated, so one sample -- including a torn sample per CW-B32-02, or a stuck sensor that evaluate_mcu_temp clamps to DEGRADED but evaluate_critical does not consult -- permanently aborts the launch. That is precisely the failure the file's own council rationale (health_monitor.h:154-163) introduced kCriticalFaultPersistTicks to prevent, and the comment at lines 713-716 waves it off as "threshold-bound and don't benefit from persistence smoothing" without accounting for a bad sample.
- Confidence: medium
- Direction: Decide which half is right -- either drop the critical byte out of the auto-action return and leave it on the visibility paths (LED / telemetry / preflight / log) as all three comments promise, or keep the auto-action, correct the three comments, and give the MCU bit the same consecutive-tick gate the primary-byte faults get.
- Verdict: CONFIRMED -- health_monitor.h:76-80, health_monitor.cpp:474-477 and :707-716 all state the critical bits are visibility-only and do not auto-trigger transitions, while :717-719 returns true on g_health.critical != 0 and ao_flight_director.cpp:138-143 turns that into SIG_DISARM plus flight_director_set_launch_abort(); evaluate_critical at :487-491 sets kHealthCriticalMcu from a single unsmoothed sample, and the same comment block even contradicts itself at :706 ("a fault that demands auto-action").

### CW-B32-05 -- flight_in_progress_was_set() is a predicate name for a destructive read
- Site: src/safety/flight_in_progress.cpp:40-45, src/safety/crash_record.h:125
- Lens: Spine block A (CCG F.2 single logical operation -- the honest name needs "and"; CCG P.3 express intent)
- Claim: A function named like a query clears the sentinel it reports on, so its name covers only the first of the two things it does and the second is discoverable only from a trailing header comment.
- Why: The body reads the magic, zeroes it, and returns the prior value; the sole documentation is the end-of-line comment "Called once at boot. Clears on read." at crash_record.h:125. The sentinel is the primary PROBABLY_MID_FLIGHT evidence for the anomalous-boot confidence gate (crash_record.h:110-113) and functions there as a standalone veto -- anomalous_boot.cpp:69-71 returns kProbablyMidFlight on the sentinel alone -- so any second reader (a diagnostic dump, a future safe-mode consumer, a re-init path) would silently consume it and leave the next reader's verdict to fall through to the corroborator arithmetic at :75-85. The same header names its other consume-on-read function honestly (crash_record_consume_prior, line 101), so the convention exists and this declaration departs from it. The single current caller (anomalous_boot.cpp:109) is correct and its module is guarded by g_initialized, so this is a latent command-query-separation trap rather than a live defect.
- Confidence: medium
- Direction: Rename to match the sibling (flight_in_progress_consume / _take) so the clear is in the name rather than in a comment, and move the "read exactly once at boot" restriction into the declaration preamble per JSF 134.
- Verdict: RESHAPED -- the destructive read behind a predicate name (flight_in_progress.cpp:40-45) and the naming inconsistency with crash_record_consume_prior are exactly as cited, but the Why mis-stated the mechanism: the sentinel is the standalone veto at anomalous_boot.cpp:69, not one of the corroborators counted at :75-85.

### CW-B32-06 -- The "usable GPS fix" rule is written out three times
- Site: src/safety/health_monitor.cpp:274, src/safety/health_monitor.cpp:763-765
- Lens: Spine block A (CCG ES.3 don't repeat yourself; Fowler Duplicated Code -- the conceptual duplicate bugprone-branch-clone cannot see)
- Claim: The definition of a usable GPS fix (fix_type >= 2 and satellites >= 4) is expressed independently in evaluate_gps() and again in health_monitor_fill_go_nogo(), plus a third time as prose in the consumer's header.
- Why: evaluate_gps() returns kHealthDegraded on snap.gps_fix_type < 2 || snap.gps_satellites < 4; roughly 490 lines later fill_go_nogo() computes gng->gps_has_lock from snap.gps_fix_type >= 2 && snap.gps_satellites >= 4; and go_nogo_checks.h:53 documents the same rule a third time in a comment. Unlike IMU/baro/ESKF -- which fill_go_nogo derives from the already-computed health byte at lines 745-751 -- GPS re-derives the rule from a second snapshot, so the two copies can disagree within one tick and will silently drift the moment the fix criterion is revised (for example to require 5 satellites): the health byte would keep reporting GPS OK while the go/no-go row said NO-GO, or vice versa.
- Confidence: medium
- Direction: Hoist the rule into one named predicate (a gps_fix_usable(snap) helper, or named constants for the two thresholds) and have both evaluate_gps() and fill_go_nogo() call it, leaving the header comment as a pointer rather than a third copy.
- Verdict: CONFIRMED -- evaluate_gps tests snap.gps_fix_type < 2 || snap.gps_satellites < 4 at :274, fill_go_nogo independently recomputes gps_fix_type >= 2 && gps_satellites >= 4 at :763-765, and go_nogo_checks.h:53 states the rule a third time in prose, while the sibling IMU/baro/ESKF rows at :745-751 are derived from the already-computed health byte rather than re-derived.

### B33 -- safety: fault injection + test_mode

#### Coverage
src/safety/fault_inject.cpp -- PARTIAL -- All 10 hooks walked with the spine; gate helper, static posted event, --undefined retention and the __StackBottom/64-byte MPU-guard claim all verified true, but the force_hardfault preamble claims a verification path the test-mode gate cannot reach (CW-B33-04).
src/safety/fault_inject.h -- PARTIAL -- Contract surface (helper Kind C) walked: declarations, the two extern volatile flags and the doc-pointer to FAULT_INJECTION.md are sound, but the banner's blanket "every fault_force_* entry checks test_mode_active()" is contradicted by its own line-24 exception marker (context for CW-B33-05).
src/safety/station_fault_inject.cpp -- FAIL -- Four hooks walked; the ungated fault_force_station_gps_restore() writes the same value as fault_force_station_gps_loss() while a nine-line comment justifies it as a recovery action (CW-B33-01).
src/safety/station_fault_inject.h -- PARTIAL -- Contract surface walked; declarations and the two extern volatile counters match their consumers in ao_telemetry.cpp, but the header states a gating invariant the file's own restore hook breaks with no marker (CW-B33-05).
src/safety/test_mode.cpp -- PARTIAL -- All six functions walked; the three-condition AND gate, the fail-closed ordering and the single-use magic clear are correct as written, but three comment blocks describe state, mechanism and host-test machinery the bodies do not have (CW-B33-03, CW-B33-06).
src/safety/test_mode.h -- PARTIAL -- Contract surface (helper Kind C) walked claim-by-claim; the arming/clearing design prose checks out against the bodies and against ao_flight_director.cpp:232 and command_handler.cpp:50, but two accessor contracts overstate what the module delivers (CW-B33-02, CW-B33-03).

Lens notes for coverage completeness. Assertions (P10-5 / JPL-16): zero assertions across all six files, but every precondition in this batch is already expressed as an explicit fail-closed branch (test_mode_evaluate's null-accessor, non-kIdle and window checks; the nine fi_test_mode_gate call sites, all of which do check the returned bool), so no load-bearing precondition is left unverified -- no finding manufactured for density alone. Declaration scope and object lifetime: the posted event at fault_inject.cpp:120 uses static const storage (correct per the QP no-copy rule), the .uninitialized_data magic word's deliberately-uninitialized lifetime matches the documented crash_record pattern, and all file-scope statics here are single-owner Core 0 -- PASS. Concurrency was not walked as a lens (not assigned to this batch), but the itinerary's three-question cue was answered far enough to rule out a scope/lifetime CP.2 case: every reader of the five volatiles is on Core 0 (main.cpp:439-443, ao_telemetry.cpp:506/603, and the test_mode_active() call sites), so none of them is a cross-core mutable.

#### Findings

### CW-B33-01 -- station GPS "restore" hook re-injects the fault it documents as recovery
- Site: src/safety/station_fault_inject.cpp:64-79
- Lens: Comments & documentation quality (JSF AV 131/134, CCG NL.2 "If the comment and the code disagree, both are likely to be wrong") plus The spine block B, confabulation (NIST AI 600-1) -- a confident justification the body does not implement.
- Claim: fault_force_station_gps_restore() executes g_bestGpsValid.store(false), byte-for-byte the same write as fault_force_station_gps_loss() at line 60, so the hook re-asserts the injected fault instead of ending it.
- Why: The nine-line comment at 70-75 justifies the store(false) by asserting the call "drops the currently-injected sticky state" so GDB users can "explicitly end the fault-injected state". There is no separate sticky state: the only injected station-GPS state IS that atomic (declared sensor_core1.h:44, set true by Core 1 at sensor_core1.cpp:246 on a better fix). Concrete path: operator runs the gps-loss scenario, Core 1 re-acquires and stores true, operator calls fault_force_station_gps_restore() expecting recovery -- the flag goes false again and the station GPS status print (rc_os_commands.cpp:709) and diag_stats.cpp:179 keep reporting no fix. Worse, this hook is deliberately ungated (lines 66-68), so unlike every other injector it can clear a live GPS-valid flag at any point in the session, after test mode has cleared.
- Confidence: high
- Direction: Make the hook actually recover, or delete it. If the intent is only "stop suppressing", there is nothing to suppress and the honest form is to remove the function together with its --undefined entry (CMakeLists.txt:738) and its FAULT_INJECTION.md row; if a real suppression latch is wanted, add one that gps_loss sets and restore clears, and keep only the clear ungated.
- Verdict: CONFIRMED -- fault_force_station_gps_restore() at station_fault_inject.cpp:77 writes g_bestGpsValid.store(false), byte-identical to the loss hook at :60, and a tree-wide grep confirms that atomic is the only injected station-GPS state, so the comment's "drops the currently-injected sticky state" justification is contradicted by its own body.

### CW-B33-02 -- test_mode_status_string() documented as the boot-banner / preflight NO-GO surface has no production caller
- Site: src/safety/test_mode.h:124-130
- Lens: Comments & documentation quality (CERT MSC12-C, doc-comment describing code that is never executed; JSF AV 134 for the unstated limitation)
- Claim: The header presents test_mode_status_string() as the "Status for boot banner / preflight VERDICT (test-mode-active surfaces as forced NO-GO)", but the function is called from nowhere in src/ -- only from test/test_test_mode.cpp.
- Why: The preflight verdict is printed at rc_os_commands.cpp:1492 as VERDICT: GO/NO-GO derived solely from hs->go_nogo_ready, with no test-mode input, and no boot banner emits the string either. So on a boot where the operator armed test mode, preflight can print GO with no indication anywhere that a fault-injection gate is open. The actual safety interlock exists elsewhere (command_handler.cpp:50 rejects kArm with "Test mode active"), so this is an operator-visibility gap rather than an arming hole -- but the header's promise is what a reader relies on when asking "will preflight tell me test mode is live?", and today the answer is no.
- Confidence: high
- Direction: Either wire the string into the preflight/banner path the comment names, or reword the comment to describe the function as it is (a status accessor with no current consumer) and note that the forced-NO-GO behaviour is implemented as an ARM rejection in command_handler, not as a VERDICT input.
- Verdict: CONFIRMED -- test_mode_status_string() has no caller anywhere outside test/test_test_mode.cpp, and the VERDICT line at rc_os_commands.cpp:1492 derives solely from hs->go_nogo_ready, so the header's "boot banner / preflight VERDICT" framing describes a wiring that does not exist.

### CW-B33-03 -- magic-observed-at-boot is documented as surviving the whole boot session; idle-exit clears it
- Site: src/safety/test_mode.h:113-121
- Lens: Comments & documentation quality (CCG NL.2 / JSF AV 131 -- comment and code disagree)
- Claim: The header states the flag behind test_mode_magic_observed_at_boot() "Stays true for the whole boot session even after test_mode_clear_on_idle_exit() flips g_test_mode_enabled false", but test_mode.cpp:116 sets g_magicObservedAtBoot = false inside exactly that function.
- Why: The comment names its own use case -- callers that want to know "did the operator arm this boot?" even after the gate clears -- and that is precisely the case that returns the wrong answer. Any caller added after the first non-kIdle transition (ao_flight_director.cpp:232 fires the clear on every phase change away from kIdle) sees false for a boot that WAS armed. The one current consumer, AO_RCOS_start (ao_rcos.cpp:1083), happens to run before any transition, so this half is latent -- but the same variable makes a second documented behaviour unreachable now: test_mode.h:128-129 says "stale-arm" covers "boot-window expired or non-IDLE state", yet after a non-IDLE transition g_magicObservedAtBoot is false and test_mode_status_string() returns "off", so the non-IDLE half of that documented status can never be observed. The .cpp side compounds it: the rationale at test_mode.cpp:112-115 attributes the fail-closed property to "s_magic_observed_at_boot was cleared on the single-use init read", which is wrong twice over -- no symbol of that name exists anywhere in src/, and what init cleared was the SRAM magic word, not the boolean, which is cleared two lines above the comment.
- Confidence: high
- Direction: Decide which behaviour is intended -- session-sticky (drop line 116 and let the clear touch only g_test_mode_enabled) or fail-closed-on-idle-exit (keep line 116 and correct both the header contract and the "stale-arm" status description). Whichever is chosen, rewrite the test_mode.cpp:112-115 rationale to name the live variable and the real clearing point; the dead name also appears in the batch's host-test comments and should be swept with it.
- Verdict: CONFIRMED -- test_mode.h:114-117 promises the flag "stays true for the whole boot session even after test_mode_clear_on_idle_exit()" and test_mode.cpp:116 clears it inside exactly that function; the name s_magic_observed_at_boot cited at :113 exists nowhere in src/.

### CW-B33-04 -- force_hardfault preamble claims a flight-phase verification path the gate makes unreachable
- Site: src/safety/fault_inject.cpp:133-141
- Lens: Comments & documentation quality (CCG NL.2) plus The spine block B, confabulation -- a confident verification claim the code cannot deliver
- Claim: The preamble says "Use this from GDB to verify either dispatch path end-to-end depending on the current flight phase", but the documented arming flow can only leave the test-mode gate open in kIdle, so an operator following that flow can exercise only the kIdle-reset branch of memmanage_fault_handler.
- Why: fault_force_hardfault() enters through fi_test_mode_gate (line 148) -> rc::test_mode_active() -> g_test_mode_enabled, which test_mode_evaluate() sets true only when the phase accessor returns FlightPhase::kIdle (test_mode.cpp:93-96); and the first transition out of kIdle calls test_mode_clear_on_idle_exit(), which clears g_magicObservedAtBoot so the gate cannot re-open for the rest of the boot. The branch the comment promises to exercise is real and safety-critical -- fault_protection.cpp:150-157 dispatches kIdle to a reset and any flight phase (or a corrupted phase byte) to the in-place kFault degrade. It is not strictly unreachable: the hook is GDB-only, and the same GDB session could write rc::g_test_mode_enabled directly in any phase -- but that bypasses the three-condition gate the module exists to enforce, so it is not a procedure this comment can be read as describing. The effect is that a reviewer following the documented arming flow in docs/FAULT_INJECTION.md believes the in-flight degrade path has an end-to-end verification hook when only the kIdle branch is reachable that way.
- Confidence: high
- Direction: Narrow the comment to state that this hook exercises the kIdle/reset dispatch only, because the test-mode gate is kIdle-only by design, and say how (or whether) the flight-phase degrade branch is verified instead -- separately from any decision about whether such a hook should exist.
- Verdict: RESHAPED -- the gate/branch mechanics check out (test_mode.cpp:93-96 plus the clear-on-idle-exit at ao_flight_director.cpp:232, against the phase dispatch at fault_protection.cpp:150-157), but "unreachable" overstates it: the hook is GDB-only and the same GDB session can set rc::g_test_mode_enabled directly in any phase, so the true claim is that the documented arming flow reaches only the kIdle branch.

### CW-B33-05 -- station header states a gating invariant its own restore hook breaks, with no marker
- Site: src/safety/station_fault_inject.h:9-12
- Lens: Comments & documentation quality (JSF AV 134 -- document the assumptions/limitations in the preamble; CCG NL.2), applied as a contract surface per the thin-file helper, Kind C
- Claim: The header banner states "every fault_force_station_* entry checks rc::test_mode_active() and returns early if not armed", but fault_force_station_gps_restore (declared line 26) is deliberately ungated at station_fault_inject.cpp:64-68, and line 26's inline comment ("allow GPS fix to repopulate") carries no NOT-gated marker.
- Why: This is the contract surface a caller or auditor reads to answer "is all station fault injection behind the physical-presence gate?", and the answer it gives is wrong for one of its four entries -- the one that writes a safety-relevant flag (see CW-B33-01). The vehicle header shows the compliant form for exactly this case: fault_inject.h:24 annotates its ungated hook "Recovery action -- NOT gated" (though its own banner at 9-10 carries the same blanket over-claim). The mismatch also undercuts the audit invariant asserted at fault_inject.cpp:38-41, which advertises the gating as "mechanically verifiable (grep for fi_test_mode_gate)"; that grep returns 9 of the 10 vehicle entries declared in fault_inject.h and 3 of 4 station hooks, so the stated check does not confirm the stated invariant.
- Confidence: high
- Direction: Reword both banners to say that every entry which injects a fault is gated and that the recovery-only entries listed below are intentionally ungated; add the NOT-gated marker to station_fault_inject.h:26 to match fault_inject.h:24; and restate the fault_inject.cpp:38-41 invariant so the grep and the sentence agree (the exemption list already exists at FAULT_INJECTION.md:244 and can be pointed to rather than re-paraphrased).
- Verdict: RESHAPED -- the banner over-claim and the missing NOT-gated marker at station_fault_inject.h:26 are real and verified, but the vehicle grep count was wrong (fault_inject.h declares 10 entries, 9 of which call fi_test_mode_gate -- not 8 of 9); corrected in the Why. FAULT_INJECTION.md:244 already records both exemptions, so the gap is the header banners, not the design.

### CW-B33-06 -- host-build comment describes a boot-window counter and test fixture that do not exist
- Site: src/safety/test_mode.cpp:12-18
- Lens: Comments & documentation quality (CCG NL.2 / CERT MSC12-C) plus The spine block B, non-self-contained / invented mechanism
- Claim: The comment says the host build's boot-time-window check "uses a static counter that defaults to 0 ms (tests can advance via the test fixture if needed)", but the host implementation is a hard-coded return of 0U at line 17 -- there is no counter and no fixture hook.
- Why: Condition (c) of the three-condition arming gate is therefore not merely defaulted on host, it is unreachable: now_ms is always 0, so the >= kTestModeArmWindowMs branch at test_mode.cpp:100-103 can never be taken in any host test, and test/test_test_mode.cpp contains no window coverage (its only mention of the condition is the file-header comment). A test author who reads this comment will go looking for the fixture hook and find nothing, and a reviewer counting host coverage of the arming gate would credit a branch that is dead in that build -- the boot-window defense is verified nowhere.
- Confidence: high
- Direction: Either replace the sentence with what the code does (host builds pin boot time at 0, so the window condition is always satisfied and is not host-testable), or add the settable static the comment already promises so the expiry branch gains a host test.
- Verdict: CONFIRMED -- test_mode.cpp:17 is a bare "return 0U;" with no counter and no fixture hook, and test/test_test_mode.cpp contains no boot-window coverage (its only mention of condition (c) is the file-header comment at :11).

### B34 -- safety: core1_i2c_pause + pyro_edge_logger + rf_link_health

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/safety/core1_i2c_pause.h -- FAIL -- Read whole; contract surface (Kind C, API/behavioural contract) walked per the helper: the preamble's central safety claim (every reachable runtime flash_safe_execute callsite is wrapped) is false against the current tree, and the blocking-call preamble omits the un-acked-return limitation.
- C:/Users/pow-w/Documents/RC-agent-walk/src/safety/core1_i2c_pause.cpp -- PARTIAL -- Read whole with its header as one work product; both functions pass the spine name/altitude test and the atomics carry justified acquire/release, but the pause/resume pair does not nest, the ack flag is written by the non-owning core, and the timeout path is silent.
- C:/Users/pow-w/Documents/RC-agent-walk/src/safety/pyro_edge_logger.h -- FAIL -- Read whole; the file-banner contract calls the storage a "static ring buffer", which the implementation is not.
- C:/Users/pow-w/Documents/RC-agent-walk/src/safety/pyro_edge_logger.cpp -- PARTIAL -- Read whole; concurrency 3-question test on the one volatile counter comes out clean (single-core ISR writer, Core 0 thread readers, no cross-core surface), but the drop-newest-on-full policy is undocumented at the site and unobservable to callers.
- C:/Users/pow-w/Documents/RC-agent-walk/src/safety/rf_link_health.h -- FAIL -- Read whole; contract surface (Kind B/C) for the RF link state machine. The logic itself is sound, self-consistent and host-tested, but three separate load-bearing comments describe a parameter, a duplicate declaration and a function that do not exist.

#### Findings

### CW-B34-01 -- Header claims every runtime flash_safe_execute callsite is pause-wrapped; three are not
- Site: src/safety/core1_i2c_pause.h:25-28
- Lens: Comments & documentation quality (JSF AV 134 preamble assumptions, CCG NL.2 "if the comment and the code disagree, both are likely to be wrong"); spine block B confabulation (NIST AI 600-1 -- confidently-presented claim with fabricated justifying logic).
- Claim: The preamble states "R-17 (2026-05-13 audit) wires these primitives around every reachable runtime flash_safe_execute() callsite" and on that basis upgrades R-11's xltl_no_i2c_during_flash SPIN property "from documented-fail to hard-PASS", but three reachable runtime callsites in the current tree call flash_safe_execute with no core1_i2c_pause().
- Why: Only three call sites invoke the primitive (ao_rcos.cpp:343 cal_save_to_flash, rc_os_commands.cpp:1042 flush, rc_os_commands.cpp:1098 erase). Unwrapped: (a) AO_RCOS_start_cal_save() at ao_rcos.cpp:1292 calls calibration_save() -> flash_safe_execute (calibration_storage.cpp:107/125) with only the post-hoc i2c_bus_reset(), i.e. the R-15 recovery without the R-17 prevention; (b) save_wmm_position() at eskf_runner.cpp:276 calls calibration_save() with neither the pause nor an i2c_bus_reset -- and it is reached autonomously from try_enable_mag_3axis() (eskf_runner.cpp:328) on the first 3D GPS fix when mag cal is present, running on Core 0 via eskf_runner_tick() at main.cpp:448 while Core 1 owns the I2C bus, so this is the LL-31 race firing unattended rather than under operator command; (c) tick_persist_debounce() at ao_radio.cpp:706 calls radio_config_storage_write() -> flash_safe_execute (radio_config_storage.cpp:83/90) with neither pause nor reset -- currently build-gated off because ROCKETCHIP_RADIO_PERSIST is defined nowhere in CMakeLists.txt or cmake/, so it is latent rather than live. A reader of this header, or anyone auditing the SPIN property, is told the class of race is closed when path (b) leaves it open on an automatic trigger.
- Confidence: high
- Direction: Correct the preamble to state which callsites are wrapped rather than asserting universal coverage, and separately decide whether save_wmm_position() and AO_RCOS_start_cal_save() get the pause (and, for the former, the post-flash i2c_bus_reset). The xltl_no_i2c_during_flash PASS claim needs re-derivation from the actual callsite set before it can stand.
- Verdict: CONFIRMED -- save_wmm_position() (eskf_runner.cpp:270-277) calls calibration_save() -> flash_safe_execute with neither core1_i2c_pause() nor a post-hoc i2c_bus_reset(), and it is reached autonomously from eskf_tick_mag / try_enable_mag_3axis (eskf_runner.cpp:328) on the first 3D fix, so the preamble's "every reachable runtime flash_safe_execute() callsite" is false on a live vehicle path.

### CW-B34-02 -- pause/resume do not nest, though the code's own comment names a nested use case
- Site: src/safety/core1_i2c_pause.cpp:20-22, 43-44
- Lens: Concurrency & shared-data ownership (CCG CP.20 / JPL-C Rule 9 -- the release must correspond to the acquire on every path); Comments (CCG NL.2 comment/code disagreement).
- Claim: core1_i2c_pause() silently no-ops when a pause is already outstanding ("Already paused (e.g., calibration wizard nested under this)"), but core1_i2c_resume() unconditionally clears both flags, so an inner resume releases an outer caller's pause.
- Why: In the exact scenario the comment at :21 names -- an inner pause/flash/resume sequence running under an outer pause -- the inner core1_i2c_resume() stores false into g_core1PauseI2C at :44, Core 1 exits its wait loop at sensor_core1.cpp:360 and resumes I2C sensor reads, and the outer caller then completes its own flash_safe_execute window believing Core 1 is still parked. That is precisely the LL-31 in-flight-transaction corruption this module exists to prevent, reintroduced by the module's own release path. I checked every caller: the three current callsites (ao_rcos.cpp:343, rc_os_commands.cpp:1042 and :1098) are mutually exclusive operator-driven paths and none nests today, so this is a latent composability defect rather than a live one -- but the comment advertises the unsupported shape as supported.
- Confidence: high
- Direction: Either make the pair genuinely re-entrant (a depth counter released to zero before clearing g_core1PauseI2C) or delete the nesting language and state in the header that pause/resume must not be nested, so a future caller does not take the comment at face value.
- Verdict: CONFIRMED -- core1_i2c_pause.cpp:20-22 no-ops on an outstanding pause while :43-44 clears both flags unconditionally, and the :21 comment names the nested calibration-wizard case the pair cannot support; the finding already states that no current caller nests, so the claim is not overstated.

### CW-B34-03 -- pause() can return without an acknowledgement, and nothing anywhere says so
- Site: src/safety/core1_i2c_pause.cpp:24-33 (contract at src/safety/core1_i2c_pause.h:50-53)
- Lens: Assertions (JPL-C Rule 16 -- assertions check anomalous conditions that should never happen; P10 Rule 5); Comments (JSF AV 134 -- assumptions and limitations belong in the preamble).
- Claim: The ack loop falls through after 100 iterations into a bare comment and returns normally, but the header preamble at core1_i2c_pause.h:50-57 says only "Blocks until Core 1 acknowledges (max ~100 ms ...)" -- the limitation that it may return with the pause not in effect is stated nowhere on the contract surface a caller reads.
- Why: The fall-through at core1_i2c_pause.cpp:29-33 is real, and it is the specific condition under which the caller's assumption is void. cal_save_to_flash() (ao_rcos.cpp:343-344) and cli_do_flush() (rc_os_commands.cpp:1042-1044) proceed straight into flash_safe_execute() on a void return with no way to distinguish "Core 1 parked" from "Core 1 stalled". The behaviour itself is not undecided: the .cpp comment at :30-32 documents the timeout and rationalises it (the post-flash i2c_bus_reset() per LL Entry 31 / R-15 is the belt-and-suspenders recovery). The defect is that this limitation never reaches the caller-facing preamble (JSF AV 134), so a caller reading only the header believes the pause is always in effect on return.
- Confidence: high
- Direction: State the un-acked-return limitation in the header preamble, and give the timeout a signal -- a bool return the caller can check, or at minimum an rc_log line plus the project assertion so a Core 1 stall is visible rather than swallowed.
- Verdict: RESHAPED -- the header omission is real, but the original Why over-reached: the silent-timeout behaviour is deliberately documented and rationalised at core1_i2c_pause.cpp:30-32, and the eskf_runner.cpp:276 path it invoked never calls core1_i2c_pause() at all, so that path says nothing about this contract.

### CW-B34-04 -- Core 0 writes g_core1I2CPaused, the acknowledgement flag Core 1 owns
- Site: src/safety/core1_i2c_pause.cpp:43
- Lens: Concurrency & shared-data ownership (JPL-C Rule 8 -- "Data objects in shared memory should have a single owning task. Only the owner of a data object should be able to modify the object.").
- Claim: g_core1I2CPaused is Core 1's acknowledgement (set at sensor_core1.cpp:357, cleared at :363); core1_i2c_resume() writes it from Core 0, making it a two-writer object with no hand-off protocol.
- Why: Answering the lens's three questions for this object gives two mutators and no stated hand-off: Core 1 sets and clears it around its wait loop, Core 0 also clears it. The concrete manifestation is a lost acknowledgement -- after resume() clears both flags at :43-44, Core 1 may still be inside its sleep_ms(1) poll at sensor_core1.cpp:360-361; if a second core1_i2c_pause() sets g_core1PauseI2C back to true within that window, Core 1 re-reads the request as still-true, stays in the loop, and never re-executes the g_core1I2CPaused.store(true) at sensor_core1.cpp:357 -- so the second pause() sees no ack, spins the full 100 ms and exits down the silent timeout path of CW-B34-03. Core 1 does happen to remain parked, so the outcome is a 100 ms stall plus a false timeout rather than corruption; the window is narrow (under ~1 ms) and no current caller issues back-to-back pauses, which is why I rate this medium rather than high. The comment at :39-42 explains why the double clear was added but does not address the ownership inversion it creates.
- Confidence: medium
- Direction: Keep the ack single-owner -- have resume() clear only g_core1PauseI2C and wait for Core 1 to clear its own ack, or replace the two-boolean handshake with a request/ack generation counter so a stale ack is distinguishable from a fresh one without Core 0 reaching into Core 1's flag.
- Verdict: CONFIRMED -- g_core1I2CPaused is set and cleared by Core 1 at sensor_core1.cpp:357/363 and also cleared by Core 0 at core1_i2c_pause.cpp:43; the adjacent :39-42 comment justifies the double clear without addressing the two-writer ownership it creates, and the finding already rates the back-to-back manifestation as latent.

### CW-B34-05 -- Banner calls the storage a "ring buffer"; it is a one-shot saturating buffer that drops the newest edges
- Site: src/safety/pyro_edge_logger.h:7 (implementation at src/safety/pyro_edge_logger.cpp:19, 30)
- Lens: Comments & documentation quality (JSF AV 131 / CCG NL.2 -- the comment and the code disagree, and NL.2's warning applies: the disagreement is itself a latent defect).
- Claim: The header contract says "static ring buffer", but gpio_edge_callback() at pyro_edge_logger.cpp:19 returns early once g_count reaches kPyroEdgeBufferSize and g_count is reset only inside pyro_edge_logger_init() at :30 -- so the buffer never wraps and never recycles; it fills once per power cycle and then discards every subsequent edge.
- Why: "Ring buffer" means the newest N events survive; this keeps the oldest 64 of the whole power cycle and silently discards everything after. pyro_edge_logger_init() is called exactly once, at main.cpp:340, so the 64 slots are a boot-to-power-off budget rather than a rolling window, and the edges that get dropped are the last ones recorded -- the opposite of what the banner promises for data the header itself calls "forensic data for post-flight analysis". Nothing surfaces the loss: pyro_edge_logger_count() at :45 returns 64 for both a full log and a truncated one, and pyro_edge_logger_dump_cli() at :54 prints "Pyro log: 64 events" with no overflow marker, so an analyst reading the dump cannot tell whether later edges are missing or never happened. Scope note: the logger is bound to kPioDroguePin/kPioMainPin = GPIO 12/13 (main.cpp:334-340), which main.cpp:335-336 records as PIO backup-timer bench pins "not connected to pyro hardware yet", so no primary deployment edge reaches this buffer in the current tree -- the defect is the false contract plus the unobservable truncation, not a present loss of flight forensics.
- Confidence: high
- Direction: Make the banner match the behaviour (state that it captures the first 64 edges and stops), or make the behaviour match the banner by wrapping the index. Either way add a dropped-edge indicator to count()/dump_cli() so a truncated log is distinguishable from a complete one.
- Verdict: RESHAPED -- the "static ring buffer" banner versus the saturating implementation (pyro_edge_logger.cpp:19, g_count reset only in init at :30) and the missing overflow indicator are exactly as described, but the forensic-loss framing overstated it: the logger is wired to GPIO 12/13, the PIO backup-timer bench pins that main.cpp:335-336 records as "not connected to pyro hardware yet".

### CW-B34-06 -- rf_lq_window_push preamble documents an in-out count parameter the signature does not have
- Site: src/safety/rf_link_health.h:83-85
- Lens: Comments & documentation quality (JSF AV 131 / 134, CCG NL.2 -- the preamble states a contract the signature contradicts).
- Claim: The comment says "Returns new window. count is incremented (caller passes in-out)", but rf_lq_window_push(uint16_t window, uint8_t good_bit) takes no count parameter and increments nothing.
- Why: Maintaining lq_window_count is entirely the caller's job -- ao_rf_manager.cpp:102-105 does it by hand immediately after the call -- and that count is load-bearing twice over: it is the divisor in rf_lq_compute_pct() at :91-99, and it gates both Schmitt transitions in rf_next_state() via the lq_window_count >= kRfLqWindowSize tests at :151 and :159. A second caller that trusts this preamble and does not increment the count leaves lq_pct pinned at 0 (the count == 0 early return at :92) while the window fills, so TENTATIVE never promotes to TRACK (the lq_pct >= 65 floor at :142 can never be met) and TRACK never drops to DEGRADED. The link-health machine would sit in TENTATIVE forever with a green build and no runtime error.
- Confidence: high
- Direction: Delete the count clause from the preamble and state explicitly that the caller owns the count, or change the signature to take the count by pointer/reference so the documented behaviour is the actual behaviour.
- Verdict: CONFIRMED -- rf_link_health.h:85 takes (uint16_t window, uint8_t good_bit) and increments nothing, while the :84 preamble promises an in-out count; ao_rf_manager.cpp:102-105 does the increment by hand, and that count gates both Schmitt tests at :151 and :159.

### CW-B34-07 -- Comment instructs maintainers to keep a duplicate LinkState declaration in sync; there is no duplicate
- Site: src/safety/rf_link_health.h:22-24
- Lens: Comments & documentation quality (CCG NL.2 / CERT MSC12-C -- documentation describing something the code no longer contains); contract-surface helper Kind E single-source check.
- Claim: The comment says "(Also declared in ao_rf_manager.h; the enum itself is portable -- keep both declarations syntactically identical.)", but ao_rf_manager.h contains no LinkState declaration; it includes this header (ao_rf_manager.h:32) and says so at :29.
- Why: LinkState is a contract-surface enum shared between the AO, this pure-logic module and the host tests, and this is the one place a reader looks to learn whether it is single-sourced. The comment tells them it is not, and hands them a manual synchronisation duty for a declaration that does not exist. A maintainer acting on it -- re-adding the "other" declaration to ao_rf_manager.h to restore what the comment says should be there -- either breaks the build on redefinition (both headers land in the same translation unit via ao_rf_manager.cpp:17 and :32) or, if guarded apart, reintroduces exactly the silent value drift the single definition currently prevents.
- Confidence: high
- Direction: Delete the parenthetical, or replace it with a one-line statement that this header is the single definition and ao_rf_manager.h includes it.
- Verdict: CONFIRMED -- ao_rf_manager.h contains no LinkState declaration; :29 states the enum is defined in safety/rf_link_health.h and :32 includes it, so the "keep both declarations syntactically identical" instruction at rf_link_health.h:23-24 refers to a duplicate that does not exist.

### CW-B34-08 -- kAcq case points the reader at rf_on_valid_rx, a function that does not exist
- Site: src/safety/rf_link_health.h:136-138
- Lens: Comments (CCG NL.2 / JSF AV 134); spine block B non-self-contained-reference / confabulation (NIST AI 600-1 -- a confidently-presented pointer to something that is not there).
- Claim: The kAcq branch comments "Promotion happens on RX, handled at call site (see rf_on_valid_rx)" and returns kAcq unchanged, but no symbol named rf_on_valid_rx exists anywhere in src/, include/ or test/.
- Why: This is the one transition rf_next_state deliberately does not implement, so the pointer is the only thing standing between a reviewer and the missing half of the state machine. The promotion actually lives inline and unnamed in the RX handler at ao_rf_manager.cpp:199-201 (if state == kAcq then state = kTentative), which is not reachable from the given name by grep or by graph lookup. Anyone auditing the four-state machine from this header -- the file that advertises itself as owning "the state transition predicates" -- is sent to a dead reference and can reasonably conclude the ACQ promotion is unimplemented. The same test file that covers every other transition (test_rf_link_health.cpp:96-98) can only assert that the promotion is absent here, so no gate contradicts the comment either.
- Confidence: high
- Direction: Point the comment at the real site (the RX handler in ao_rf_manager.cpp), or extract that two-line promotion into a named helper in this header so the reference becomes true and the whole machine is testable from one place.
- Verdict: CONFIRMED -- a tree-wide grep for rf_on_valid_rx returns exactly one hit, the comment itself at rf_link_health.h:137; the real promotion is the unnamed inline block at ao_rf_manager.cpp:199-201.

### B35 -- safety: PIO backup timer + PIO watchdog

#### Coverage

- src/safety/pio_backup_timer.h -- PARTIAL -- Read whole as a Kind-C contract surface (API prose + declarations); every declared function judged against its implementation, and two preamble claims do not match the module.
- src/safety/pio_backup_timer.cpp -- PARTIAL -- Read whole, both the target branch and the ROCKETCHIP_HOST_TEST stub branch; spine run on all six target functions plus the six stubs; init/arm/cancel/disarm walked as an init-use-teardown sequence.
- src/safety/pio_watchdog.h -- PARTIAL -- Read whole as a contract surface; the kPioWatchdogCountdown arithmetic (666666 x 3 cycles = ~2.0 s at 1 MHz) was checked against the PIO program and is correct; the fault-query contract is not complete.
- src/safety/pio_watchdog.cpp -- PARTIAL -- Read whole, both branches; init/feed/fault/deinit walked as a lifecycle (pio_add_program correctly paired with init/deinit, not feed -- the LL-42 hazard is not present here).

Verification reads outside the batch, used only to check claims made inside it: pio/backup_timer.pio, pio/heartbeat_watchdog.pio, src/active_objects/ao_flight_director.cpp (the only consumer of fired/cancel/arm/disarm), src/main.cpp init_pio_safety, src/safety/health_monitor.cpp, src/safety/pyro_edge_logger.cpp, src/drivers/ws2812_status.cpp, src/flight_director/mission_profile.h.

Considered and deliberately not filed: the two-channel state is carried as six parallel scalars (g_drogueSm/g_mainSm, g_droguePin/g_mainPin, g_drogueArmed/g_mainArmed) with the channel-select ternary repeated in four functions and the arm sequence written twice. For exactly two fixed channels this is a defensible shape, not a defect -- ES.3 pressure worth watching only if a third channel is ever added.

#### Findings

### CW-B35-01 -- Backup pyro fire pulse is ~5 ms, not the ~200 ms the design claims
- Site: pio/backup_timer.pio:39-52 (out-of-batch site; surfaced verifying the pio_backup_timer.h:9 "fire pyro GPIOs on expiry" contract and routed here because it is what makes CW-B35-02 bite)
- Lens: The spine, block C -- functionally-correct-but-safety-blind hardware code; and block B confabulation (a confident inline rationale the body does not implement). Comments lens JSF 131/134, CCG NL.2 ("if the comment and the code disagree, both are likely to be wrong").
- Claim: The e-match hold loop produces roughly 5.2 ms of pin-HIGH, not the ~180-200 ms its own comments and the program header assert.
- Why: The nested loop is set y,31 then per outer pass mov x,osr (1 cycle) + set x,31 (1) + 32 executions of jmp x-- pulse_inner [4] (5 cycles each = 160) + jmp y-- (1) = 163 cycles; 32 outer passes = ~5216 cycles = ~5.2 ms at the 1 MHz SM clock. The comment at :45-46 computes 31 x 31 x 6 and labels that figure "per outer", then multiplies by 31 again -- it counts the whole loop nest once as an inner-loop cost. The structure as written cannot reach 200 ms at all: with two nested SET-immediate loops the ceiling is 32 x 32 x 32 = 32768 cycles (~33 ms) even at the maximum [31] delay. Concrete consequence: on the ARM-dead path this module exists for, the backup channel delivers a ~5 ms gate pulse to an e-match instead of the intended ~200 ms. GPIO 12/13 are bench pins today (src/main.cpp:335 "not connected to pyro hardware yet"), so this is latent, not live -- it becomes live the moment the pins are wired.
- Confidence: high (on the arithmetic and the comment/code disagreement); medium on the ignition consequence, which depends on the e-match chosen.
- Direction: Recompute the hold from the required pulse energy and restructure the delay (third nesting level, or a larger clkdiv for the hold phase), then state the achieved pulse width in cycles in the comment so the next reader can check it. Do not simply edit the comment down to ~5 ms without confirming that is the intended e-match pulse.
- Verdict: CONFIRMED -- the cycle count re-derives independently: 32 inner passes at 5 cycles each (jmp x-- [4]) = 160, plus mov/set/jmp overhead = 163 per outer pass, times 32 outer passes = 5216 cycles = ~5.2 ms at 1 MHz, against the ~180-200 ms the program header at :16 and the inline comment at :45-46 assert.

### CW-B35-02 -- pio_backup_timer_fired() samples a transient pin level while the latched PIO fire flag goes unread
- Site: src/safety/pio_backup_timer.cpp:153-162
- Lens: The spine, block B (happy-path-only / passes-tests-yet-wrong: the miss is on an input no test exercises) and block C (peripheral lifecycle). Comments lens JSF 134 -- the function's stated limitation is absent and its stated purpose is wrong.
- Claim: The fire query is a level read of a GPIO that is HIGH only for the duration of the pulse, so a fire can be missed entirely, even though the PIO program latches the same event in IRQ flag 1.
- Why: The only consumer, fd_check_pio_backup in src/active_objects/ao_flight_director.cpp:79-92, polls this at the FD's 100 Hz tick (ao_flight_director.cpp:39, 10 ms period). Against the ~5.2 ms HIGH window measured in CW-B35-01 that is roughly a coin flip per fire; and that same file's comment at :59 records that a blocking LoRa send in QV_onIdle stalls dispatch for 50-150 ms, during which the window closes completely. On a miss, pio_drogue_reported stays false, director.state.drogue_fired is never set, and SIG_PYRO_FIRED (source=1) is never published -- the flight state machine and the event log never learn that the backup layer fired. After the pulse the PIO drives the pin LOW and stalls forever, so the query then returns false permanently: it is an instantaneous level, not the latch its one-shot consumer treats it as. pio/backup_timer.pio:54 already sets IRQ flag 1 with the stated purpose "signal timer fired to ARM code", and no code in the tree reads it (the only pio_interrupt_get call is pio_watchdog.cpp:59, on flag 0). Two further notes: the header at :48-49 labels this "for diagnostics" while the FD uses it as the authoritative backup-fire detector; and no host test can reach the true branch, because the host stub at :199-201 returns g_drogue_fired / g_main_fired, which nothing ever writes.
- Confidence: high
- Direction: Read the latched PIO IRQ flag instead of the pin level, and give each channel its own flag (the two backup SMs both execute an absolute irq set 1, so the current flag cannot distinguish drogue from main -- a relative irq set is the usual way). Clear the flag when the fire is consumed, and correct the header to say this reports a latched fire event consumed by the FD, not a diagnostic level.
- Verdict: CONFIRMED -- pio_backup_timer.cpp:161 returns gpio_get(pin), an instantaneous level, while backup_timer.pio:54 latches the same event in IRQ flag 1 and the only pio_interrupt_get call in the whole tree is pio_watchdog.cpp:59 on flag 0; the one consumer at ao_flight_director.cpp:79 polls it at the 100 Hz tick against a ~5 ms pulse.

### CW-B35-03 -- "Drive LOW" on disarm/cancel never enables the pad output; the pyro line is released high-Z
- Site: src/safety/pio_backup_timer.cpp:131-141 (same pattern at :108-113)
- Lens: The spine, block C -- functionally-correct-but-safety-blind hardware code. Comments lens CCG NL.2 / JSF 131 -- comment and code disagree on a safety property.
- Claim: The comment asserts the pin is driven LOW after disarm so a stale SM cannot drive it HIGH, but the code only selects SIO function and writes the SIO output register; nothing ever sets the pad's output enable, so the pad stays an input.
- Why: gpio_set_function selects the function and sets input-enable; it does not touch the SIO output-enable bit, and gpio_put writes only the output level register. Nothing in this module or anywhere else in the tree sets a direction for GPIO 12/13 -- grepping gpio_set_dir / gpio_init across src/ and include/ returns only i2c_bus, rfm95w, main.cpp's LED, and board_fruit_jam. The project's own peer driver shows the required idiom (src/drivers/rfm95w.cpp:101-102 does gpio_init then gpio_set_dir(GPIO_OUT) before driving). So after every disarm and every cancel the pyro line is a high-impedance input relying on the pad's reset pull-down, not an actively driven LOW. On a real pyro FET gate that is the difference between a hard 0 and a floating node -- exactly the inadvertent-ignition case the comment claims to have closed, and the claim is what a future reviewer will trust instead of re-deriving it.
- Confidence: high on the mechanism (no direction is ever set); medium on the field consequence, which depends on the pyro driver circuit that is not yet wired.
- Direction: Set the pin as an SIO output before writing the level in both cancel and disarm, so the claimed LOW is actually driven; if releasing to a pulled input is in fact the intended behaviour, say that in the comment instead and name what holds the line down.
- Verdict: CONFIRMED -- gpio_set_function sets FUNCSEL and the pad input-enable but never the SIO output-enable, gpio_put writes only the level register, and a tree-wide grep shows gpio_set_dir / gpio_init are called only for I2C, RFM95W, the LED and the Fruit Jam reset pin -- never for GPIO 12/13, so the "drive LOW" comment describes a pad that is left high-Z.

### CW-B35-04 -- Header contract describes two behaviours the module does not implement
- Site: src/safety/pio_backup_timer.h:13-16
- Lens: Comments lens -- CERT MSC12-C (documentation describing behaviour the code cannot produce) and JSF 131 / CCG NL.2; contract-surface helper Kind C (the prose plus the signatures are the thing under review).
- Claim: The header preamble documents a profile-configurable timer-action code (0 = disabled, 1 = fire drogue GPIO, 2 = fire main GPIO) that no function accepts and no profile field carries, and the disarm declaration at :44-46 says the call clears PIO instruction memory, which the implementation deliberately does not do.
- Why: No declaration in this header takes an action parameter -- init takes two pins, arm takes two timeouts -- and src/flight_director/mission_profile.h:110-112 carries only drogue_timer_s and main_timer_s, with no action field anywhere in the struct. A reader of the header would believe the channel-to-GPIO binding is selectable per timer at profile level; it is actually fixed at init by the pin arguments, and the real "disabled" mechanisms are timeout == 0 and pin == 0xFF. The disarm claim is worse than stale: pio_backup_timer.cpp:143-148 records that removing the program at disarm is precisely the LL Entry 42 defect (SDK metadata assertion on the second disarm, Core 0 wedged), so the header currently instructs the next maintainer to restore a known Tier-1 safety regression.
- Confidence: high
- Direction: Delete the action-code block or replace it with the real contract (pins fixed at init; per-channel disable is timeout == 0 or pin == 0xFF), and rewrite the disarm line to say it stops both SMs and releases the pins while the program stays resident, with the LL-42 pointer already present in the .cpp.
- Verdict: CONFIRMED -- no declaration in the header takes an action code and mission_profile.h has no action field at all (grep for "action" returns nothing), while the disarm declaration at :44 promises exactly the pio_remove_program behaviour that pio_backup_timer.cpp:143-148 records as the LL-42 regression.

### CW-B35-05 -- PIO allocation comment contradicts the project's own PIO rules
- Site: src/safety/pio_watchdog.cpp:22
- Lens: Comments lens JSF 131 / CCG NL.2 (comment and truth disagree) plus NL.3 (a claim about a resource map that should point at the map). The map is docs/MULTICORE_RULES.md "PIO State Machines".
- Claim: The comment states PIO2 is "dedicated to safety (PIO0 = WS2812, PIO1 = reserved)", but the WS2812 driver does not pin itself to PIO0 and the project's own rules document says it lands on PIO2.
- Why: src/main.cpp:240 passes pio0 to ws2812_status_init, but src/drivers/ws2812_status.cpp:248-250 passes that instance by address to pio_claim_free_sm_and_add_program_for_gpio_range, which overwrites it with whichever block it lands on; docs/MULTICORE_RULES.md records "WS2812/NeoPixel driver -- 1 SM on PIO2 SM0 (or wherever pio_claim_free_sm_and_add_program_for_gpio_range lands it)", and LESSONS_LEARNED Entry 33 records the same search-order surprise as a lived incident. WS2812 init runs before init_pio_safety, so the realistic allocation is WS2812 + watchdog + two backup timers = 4 of PIO2's 4 state machines, with zero headroom -- the opposite of the "dedicated" claim. The failure mode is silent: pio_claim_unused_sm returns -1, pio_backup_timer_init returns false, and src/main.cpp:337-339 only emits a DBG_ERROR and continues booting with Layer 3 absent, so the comment's false reassurance is the only thing standing between a future PIO consumer and a missing backup-deployment layer.
- Confidence: medium-high (the contradiction with MULTICORE_RULES.md and LL 33 is certain; the exact runtime landing depends on SDK claim order, which is why the comment should not assert it at all).
- Direction: Replace the parenthetical with a pointer to docs/MULTICORE_RULES.md "PIO State Machines" and state the real invariant this module needs (a free SM on PIO2 at init time), rather than asserting an allocation the code does not enforce.
- Verdict: CONFIRMED -- ws2812_status.cpp:249-251 passes the PIO instance by address to pio_claim_free_sm_and_add_program_for_gpio_range, which overwrites the pio0 hint from main.cpp:240, and docs/MULTICORE_RULES.md "PIO State Machines" records WS2812 as landing on PIO2 -- the opposite of the comment's "PIO0 = WS2812" allocation claim.

### CW-B35-06 -- Watchdog fault flag is self-clearing on the next feed, and the contract does not say so
- Site: src/safety/pio_watchdog.h:34-36 (implementation at src/safety/pio_watchdog.cpp:55-60)
- Lens: Assertions/contract side of JSF 134 -- "assumptions (limitations) made by functions should be documented in the function's preamble"; the spine block B (a status query written to the happy path).
- Claim: The header presents the fault query as a state read, but the flag is erased by this module's own feed, so a false return cannot be distinguished from "a fault occurred and our next feed wiped it".
- Why: In pio/heartbeat_watchdog.pio:35-38 the timeout path does irq set 0, then pull block, then irq clear 0 -- so the first pio_watchdog_feed after a stall clears the fault before anyone can observe it. main.cpp's watchdog_kick_tick feeds unconditionally on every main-loop pass (src/main.cpp:418) while the only consumer, src/safety/health_monitor.cpp:413, samples it to set kHealthPioOk at the health cadence. The exact case the watchdog exists to catch -- ARM stalled longer than ~2 s and then recovered -- therefore usually leaves kHealthPioOk set, and nothing in the module latches or counts the event, so no post-flight record survives either. Neither the header nor the .cpp states this ordering dependency, so a caller has no way to know that reading after feeding is meaningless.
- Confidence: medium-high (the PIO clear-on-recovery mechanism is certain; how often the health sample loses the race depends on tick ordering).
- Direction: Either latch a sticky fault (a module-level counter or flag set whenever the query observes IRQ 0, cleared only by an explicit call) so the recovery event survives for health and logging, or state plainly in the header that the flag self-clears on the next feed and that callers must sample before feeding.
- Verdict: CONFIRMED -- heartbeat_watchdog.pio:35-38 sets IRQ 0, blocks on pull, then clears IRQ 0 on the next feed, and watchdog_kick_tick() runs at the top of qv_idle_bridge ahead of any AO dispatch, so health_monitor.cpp:413 samples a flag the recovery feed has already erased; neither pio_watchdog.h nor the .cpp states that ordering dependency.

### CW-B35-07 -- No assertions in either safety translation unit; init accepts any pin value
- Site: src/safety/pio_backup_timer.cpp:25-33 (same absence in pio_watchdog.cpp:17-45)
- Lens: Assertions -- Power of Ten Rule 5 (density) and JPL-C Rule 16 ("all functions of more than 10 lines should have at least one assertion... parameter validity must be checked inside each function").
- Claim: Neither file contains a single assertion, and the load-bearing entry-point check that is actually missing is validation of the pin parameters in pio_backup_timer_init.
- Why: pio_backup_timer_init (40 lines), pio_backup_timer_arm (33), pio_backup_timer_disarm (30) and pio_watchdog_init (29) are all well over the JPL-16 ten-line trigger, in a subsystem where a tripped assertion routes to the project fault path. Concretely: drogue_pin and main_pin are consumed as-is -- any value other than the 0xFF sentinel flows straight into pio_gpio_init, pio_sm_set_consecutive_pindirs, gpio_set_function, gpio_put and gpio_get, with no check that it is a valid bank-0 GPIO. Today the only caller passes two constexpr constants (src/main.cpp:334-337), so the contract is satisfied by luck of the call site rather than by anything in the function; a second caller, a board variant, or a profile-driven pin would fault or corrupt an unrelated pad with no diagnostic. The 0xFF disabled-sentinel is the only input rule the module enforces, and it is enforced by silent skip rather than by a stated contract.
- Confidence: medium
- Direction: Add an entry assertion in pio_backup_timer_init that each pin is either the 0xFF sentinel or a valid GPIO index, and consider lifting it to a compile-time check at the call site (CCG P.5) since the pins are constexpr there; the same treatment applies to the g_pio / g_offset invariant the arm and cancel paths rely on.
- Verdict: REFUTED -- the consequence is hypothetical: pio_backup_timer_init has exactly one caller (main.cpp:337) passing two constexpr constants, the 0xFF sentinel is the only input rule the module needs today, and the tree as a whole carries a single runtime assert() in all of src/ (eskf.cpp:562), expressing preconditions as fail-closed branches instead -- so this is an assertion-density claim, not a defect on any reachable path.

### B36 -- core1: sensor loop (Core0<->Core1 boundary)

#### Coverage
- C:/Users/pow-w/Documents/RC-agent-walk/src/core1/sensor_core1.cpp -- FAIL -- Read whole (524 lines); spine A/B/C run on all 11 functions and the comments, scope/lifetime, control-flow and concurrency lenses applied; four findings sited here, all on the cross-core boundary rather than on any single line's logic.
- C:/Users/pow-w/Documents/RC-agent-walk/src/core1/sensor_core1.h -- PARTIAL -- Read whole (66 lines); evaluated as a contract surface per the helper (Kind A shared-object catalog + Kind B shared protocol + Kind C API prose). The g_bestGpsFix ownership claims and its "benign for diagnostics" licence were checked against every consumer in the tree and hold; the core1_read_gps preamble understates the helper's side effects (CW-B36-03).

#### Findings

### CW-B36-01 -- Core 1 reads Core 0's live ESKF state with no barrier, and gates a recovery action on it
- Site: src/core1/sensor_core1.cpp:269-270 (declaration site src/core1/sensor_core1.h:61-64)
- Lens: Concurrency & shared-data ownership -- JPL-C Rule 8 (single owning task, owner-only mutation), CCG CP.2 (avoid data races); spine block C (cross-core / weak-memory barrier reasoning)
- Claim: g_eskfInitialized (plain bool) and g_eskf.v (three non-atomic floats) are written by Core 0 and read by Core 1 with no barrier, no seqlock and no snapshot, and the header at sensor_core1.h:61-64 calls the read "GPS staleness and related diagnostics" when it is in fact the gate on a multi-second blocking recovery action.
- Why: eskf_runner_tick() runs on Core 0 inside qv_idle_bridge (src/main.cpp:448) and mutates g_eskf.v in predict(); Core 1 reads v.x/v.y/v.z here to compute a norm. Nothing orders those three component loads against Core 0's stores, and docs/MULTICORE_RULES.md is explicit that plain (not even volatile) cross-core sharing without atomics or a spinlock does not carry the required barriers on this part. Applying the lens's three-question test to g_eskf gives owner = Core 0, mutator = Core 0, barrier = none, and that missing third answer is the finding. It matters because the read is load-bearing rather than diagnostic: it is the sole "probably flying" gate suppressing the multi-second blocking gps_uart_reinit() at line 275, yet the extern that publishes it describes it as diagnostics. What the missing barrier does not establish is a specific in-flight failure -- a stale velocity is still a flight-magnitude velocity, so the realistic way this gate opens in flight is an uninitialised or CR-1-reset filter (g_eskfInitialized false, v zeroed), which is a design dependency on ESKF health rather than a torn read.
- Confidence: high
- Direction: Publish the flight-state signal Core 1 actually needs as a single atomic -- a speed scalar or a flight-phase enum stored release by Core 0's ESKF tick and loaded acquire by Core 1 -- instead of reaching into the filter object across cores, and state the owner and barrier on the extern in sensor_core1.h.
- Verdict: RESHAPED -- the unsynchronised cross-core read and the header's "diagnostics" framing are verified against docs/MULTICORE_RULES.md, but the concrete "composed norm below 5 m/s lets the reinit run mid-flight" outcome is speculative (a stale velocity is still a flight-magnitude velocity); narrowed to the ownership/barrier violation plus the mischaracterised contract.

### CW-B36-02 -- Core 1 mutates the Core-0-owned calibration module's state with no handshake barrier
- Site: src/core1/sensor_core1.cpp:172-182 (same pattern at 206-209)
- Lens: Concurrency & shared-data ownership -- JPL-C Rule 8 (only the owner may modify; ownership passed explicitly), CCG CP.3 (minimize the shared writable surface), CCG CP.2; spine block B (a confident comment that answers a different question than the one that matters)
- Claim: Core 1 reads calibration_manager_get_state() and calls calibration_feed_gyro/accel/baro, which mutate Core 0's private module state (g_calState, g_sampleAcc, g_calibration and its CRC) through plain non-atomic globals with no release/acquire pairing on either side.
- Why: calibration_start_gyro() runs on Core 0 (src/active_objects/ao_rcos.cpp:382). It calls reset_accumulator() and then writes g_calState = CAL_STATE_GYRO_SAMPLING with plain stores (src/calibration/calibration_manager.cpp:204-205). Core 1's plain read at line 173 can observe the new state before the accumulator reset becomes visible, so Core 1 accumulates into a stale accumulator whose count is already at target_count; calibration_feed_gyro then immediately takes its completion branch and writes g_calibration.gyro.bias plus calibration_update_crc() from Core 1 (calibration_manager.cpp:246-264). The result is a CRC-valid calibration derived from a previous run's sums. The mirror direction is equally reachable: Core 0 setting CAL_STATE_IDLE while Core 1 is already past the line-212 state check lets Core 1 drive an aborted calibration to CAL_STATE_COMPLETE. The adjacent comment at lines 170-171 asserts safety, but it reasons only about I2C bus contention -- it never addresses the shared accumulator, which is the state that actually crosses cores. The same function shows the correct pattern 240 lines later: rc_os_mag_cal_active is a std::atomic<bool> with release/acquire for precisely this Core-0-arms / Core-1-observes handshake (line 420), so the discipline is inconsistent within one file.
- Confidence: high
- Direction: Give the sampling handshake the same shape as the mag-cal gate -- an atomic cal-sampling state stored release by Core 0 after the accumulator reset and loaded acquire by Core 1 -- or move the feed off Core 1 entirely by having Core 0 consume raw samples from the seqlock, so the accumulator has one owner.
- Verdict: CONFIRMED -- g_calState is a plain static (calibration_manager.cpp:93) written by Core 0 at :204-205 and read by Core 1 at sensor_core1.cpp:173, with calibration_feed_gyro mutating g_sampleAcc, g_calibration and its CRC from Core 1 at :246-264; the adjacent :170-171 comment reasons only about I2C bus contention, and the correct atomic handshake sits 240 lines later in the same file at :420.

### CW-B36-03 -- Shared GPS reader can block for seconds; both its comment and its published contract understate it
- Site: src/core1/sensor_core1.cpp:251-252 and :275 (contract at src/core1/sensor_core1.h:52-59)
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions and limitations documented in the function preamble), CCG NL.2 ("if the comment and the code disagree, both are likely to be wrong"); spine block C (blocking work inside a cooperative-scheduler context)
- Claim: core1_gps_staleness_check() calls gps_uart_reinit(), whose worst case is about 4-4.25 s rather than the "Blocks up to 2s" the comment at :251-252 states, and the public preamble for core1_read_gps() (sensor_core1.h:52-59) says the helper "only updates the local struct and invokes update_best_gps_fix", omitting both the blocking reinit and the g_gpsInitialized mutation at line 276.
- Why: gps_uart_reinit() calls acquire_at_target_baud(), which calls try_baud() twice; each try_baud() calls detect_gps_presence(), a busy poll bounded by kInitTimeoutUs = 2 000 000 us, and a successful fallback adds busy_wait_ms(250) (src/drivers/gps_uart.cpp:311-348, 468). Worst case is therefore about 4.25 s, and because this path only runs after 10 s with no valid NMEA parse, the no-GPS-present worst case is the expected case, not a corner. The live consequence is on the vehicle: Core 1's 1 kHz loop stops publishing core1_loop_count for that whole window, far past health_monitor's stall threshold of 6 ticks at 10 Hz (src/safety/health_monitor.cpp:344-376), so a GPS antenna fault is annunciated as a Core 1 stall. The station role shares the same helper on Core 0 from qv_idle_bridge (src/station/station_idle_tick.cpp:86) under a header budget of "worst-case GPS I2C cost ... ~6 ms ... bounded by ... the 5 s watchdog (800x margin)" (station_idle_tick.cpp:25-28), but that exposure is NOT live on the shipping station: the Fruit Jam sets kUartGpsAvailable = false (include/rocketchip/board_fruit_jam.h:87), so g_gpsTransport is GPS_TRANSPORT_I2C and core1_gps_staleness_check returns at sensor_core1.cpp:261 before reaching gps_uart_reinit(). It becomes reachable only if a station is ever built on a UART-GPS board.
- Confidence: high
- Direction: Correct the duration in the comment and add the side effects (may block for seconds, may clear g_gpsInitialized) to the core1_read_gps preamble in the header; then decide whether a watchdog-bearing idle path should carry a blocking recovery at all -- a start/poll split, or restricting the staleness reinit to the vehicle Core 1 path, would keep the station idle bridge inside its stated budget.
- Verdict: RESHAPED -- the timing derivation (two kInitTimeoutUs windows plus the 250 ms negotiate delay) and the header omission hold, and the vehicle consequence is live, but the station half is not reachable: the station is the Fruit Jam (board_fruit_jam.h:87 kUartGpsAvailable = false), so g_gpsTransport is GPS_TRANSPORT_I2C and core1_gps_staleness_check returns at sensor_core1.cpp:261 before ever calling gps_uart_reinit().

### CW-B36-04 -- Core 1's pause indicator overwrites the LED compositor's state without invalidating its change-detection cache
- Site: src/core1/sensor_core1.cpp:356-366 (LED calls at 358-359 and 364-365)
- Lens: Concurrency & shared-data ownership -- JPL-C Rule 8 (single owner), CCG CP.3 (shared writable surface); Comments & documentation quality -- CCG NL.2 (the comment disagrees with the code it describes)
- Claim: Core 1 calls ws2812_set_mode() and ws2812_update() on a driver whose declared sole owner is Core 0's AO_LedEngine, and unconditionally restores SOLID/blue on resume, leaving the compositor's cached last_mode/last_color out of sync with the hardware.
- Why: led_set_if_changed() re-issues ws2812_set_mode() only when the newly composited pattern differs from the AO's own cached last_mode/last_color (src/active_objects/ao_led_engine.cpp:93-103); it never reads the driver's actual state. When Core 1 forces WS2812_MODE_SOLID with kColorBlue at line 364 while the compositor's cache says it is showing, for example, the fault BLINK_FAST or the pre-arm-fail DOUBLE_FLASH, the AO believes the hardware already matches and never re-issues, and ws2812_update() is a no-op for MODE_SOLID (src/drivers/ws2812_status.cpp:454-457). The LED therefore stays solid blue until the composited layer next changes, so a fault or pre-arm-fail annunciation is silently replaced by a normal-looking indication. That path is deterministic, not a rare interleaving. Separately the two cores can genuinely overlap here: Core 0 returns from core1_i2c_resume() into QV and ticks AO_LedEngine while Core 1 is still executing lines 363-365, and both paths write the non-atomic g_state struct and push words into the same PIO TX FIFO through send_pixel(). The ownership comment at src/active_objects/ao_led_engine.h:13-14 grants this exception as "debug-only", which the tree contradicts: core1_i2c_pause() is invoked from the production flight-log flush and erase paths and from the calibration save (src/cli/rc_os_commands.cpp:1042, 1098; src/active_objects/ao_rcos.cpp:343).
- Confidence: high for the cache-desync clobber; medium for the concurrent-write window
- Direction: Route the pause indication through the owner rather than the driver -- Core 1 sets an atomic pause flag that AO_LedEngine composites as its own layer -- so the driver keeps one writer and the compositor's cache stays authoritative; then correct or delete the "debug-only" carve-out in ao_led_engine.h.
- Verdict: CONFIRMED -- led_set_if_changed (ao_led_engine.cpp:93-103) compares only against the AO's own cache and never reads the driver, ws2812_update() is a no-op for MODE_SOLID (ws2812_status.cpp:454-457), and the "debug-only" carve-out at ao_led_engine.h:13-14 is contradicted by the production callers at rc_os_commands.cpp:1042 / 1098 and ao_rcos.cpp:343.

### B37 -- AOs: flight_director + health_monitor

#### Coverage

- src/active_objects/ao_flight_director.cpp -- PARTIAL -- Read whole; spine A/B/C run on all 11 functions and lambdas, concurrency 3-question test answered (single Core-0 QV owner, run-to-completion barrier, g_sensorSeqlock read through a checked seqlock_read); findings CW-B37-01/02/04/06/07.
- src/active_objects/ao_flight_director.h -- PARTIAL -- Read whole as a Kind C contract surface per the helper; six declarations inventoried, the prose is migration history rather than assumptions and limitations; finding CW-B37-03.
- src/active_objects/ao_health_monitor.cpp -- PARTIAL -- Read whole; four functions walked, the subscribe/publish story verified against every SIG_HEALTH_STATUS and SIG_PHASE_CHANGE endpoint in the tree; finding CW-B37-05.
- src/active_objects/ao_health_monitor.h -- PASS -- Thin Kind C/D contract surface; every claim checked and true (10Hz tick, queue depth 8, subscribes SIG_PHASE_CHANGE, priority 6 sits between FD 9 and Logger 4, AO_HealthMonitor consumed by diag_stats.cpp:141). One imprecision noted but not filed: the banner names "LED" as a consumer, but AO_LedEngine does not subscribe to SIG_HEALTH_STATUS -- the LED is driven indirectly through AO_Notify.

#### Findings

### CW-B37-01 -- Auto-DISARM on critical fault leaves the PIO backup pyro timers armed
- Site: src/active_objects/ao_flight_director.cpp:138-143 (paired hooks at 310-325)
- Lens: The spine, block C -- "Peripheral init SEQUENCE / lifecycle ... a resource added at init but never paired with teardown"; supported by the comments lens JSF AV 134 / NL.2 against the stated contract in src/safety/pio_backup_timer.h:44-46 ("Disarm both timers ... Called on SIG_DISARM or SIG_RESET").
- Claim: pio_backup_timer_arm() and pio_backup_timer_disarm() are wired only inside AO_FlightDirector_process_command, so the auto-DISARM that fd_tick issues on a critical sensor fault reaches IDLE with both autonomous backup deployment countdowns still running.
- Why: On the concrete path "vehicle ARMED on the pad, IMU or ESKF critical fault", line 141 calls rc::flight_director_dispatch_signal(&me->director, rc::SIG_DISARM) directly and line 142 latches launch abort. That call never passes through AO_FlightDirector_process_command, and line 323 is the only call to pio_backup_timer_disarm() in the entire tree. The backup timers are documented as "Completely independent of ARM cores -- survives crashes" (pio_backup_timer.h:11), so nothing else stops them: they will drive the drogue GPIO at drogue_timer_s and the main GPIO at main_timer_s. docs/USER_GUIDE.md:145-160 then instructs the operator to approach the disarmed vehicle and physically inspect the igniter wiring. The same gap sits on two further live paths -- the HSM's own ARMED-timeout auto-disarm (src/flight_director/flight_director.cpp:400-407, a bare Q_TRAN to state_idle) and the MAVLink DISARM at src/active_objects/ao_telemetry.cpp:282. The mirror case has the same root cause: a MAVLink ARM at ao_telemetry.cpp:282 reaches ARMED without ever calling pio_backup_timer_arm(), so a radio-armed flight silently has no backup deployment at all.
- Confidence: high
- Direction: Bind the arm/disarm hooks to the phase transition rather than to one command entry point -- the phase_change_cb already wired at line 230 observes every entry to and exit from ARMED regardless of which path caused it. Failing that, funnel every DISARM/RESET source through a single function that owns the teardown.
- Verdict: CONFIRMED -- pio_backup_timer_disarm() is called only at ao_flight_director.cpp:323 inside process_command, so the fd_tick auto-DISARM (:141), the HSM ARMED-timeout Q_TRAN to state_idle (flight_director.cpp:400-407) and the MAVLink DISARM (ao_telemetry.cpp:282) all reach IDLE with the timers running, and MAVLink ARM never calls the arm hook.

### CW-B37-02 -- Five raw function-pointer callbacks wired here, with no active P10 Rule 9 deviation on record
- Site: src/active_objects/ao_flight_director.cpp:221-252 (declarations at src/flight_director/flight_director.h:85-89)
- Lens: Class and interface design (dispatch mechanism) plus the spine's spec-noncompliance criterion; sourced to Power of Ten Rule 9 -- "Limit pointer use to a single dereference, and do not use function pointers" -- with JSF AV 176 ("A typedef will be used to simplify program syntax when declaring function pointers") as the secondary.
- Claim: fd_wire_callbacks assigns five raw function pointers (set_led_cb, phase_change_cb, log_pyro_cb, beacon_cb, reset_subsystems_cb) in flight-critical code, while standards/ACCEPTED_STANDARDS_DEVIATIONS.md records no active function-pointer deviation.
- Why: The register's section "Function Pointer Usage (P10 Rule 9)" (ACCEPTED_STANDARDS_DEVIATIONS.md:54-56) contains only the line "(All entries resolved as of 2026-05-13. See Resolved section below for FP-1.)", FP-1 being the lm_solve case retired by the lm_solver template refactor. A reader consulting the register to answer "does this codebase use function pointers?" is told no; these five are the live counter-example, and they carry the pyro-fired logger, the SIG_PHASE_CHANGE publisher, the IVP-121 distress-beacon backstop and the RESET-to-IDLE ESKF re-init. They are also declared as bare void (*name)(args) members with no typedef, which is the JSF 176 half. Per LESSONS_LEARNED Entry 37 the register is exactly the artifact that goes stale this way -- the same "resolved" shape was previously false for the naming rules.
- Confidence: high on the facts (the callbacks exist; the register lists no active entry). The disposition -- accept with rationale versus remediate -- is the owner's.
- Direction: Either log a deviation row naming these five with rationale and a remediation path (the C HSM / C++ AO seam is the same argument the notify backend used to avoid a vtable), or remove the indirection by having the HSM call a small fixed set of named AO entry points directly.
- Verdict: CONFIRMED -- flight_director.h:85-89 declares five bare function-pointer members with no typedef, CODING_STANDARDS.md:56 records P10-9 (outright ban) as governing, and ACCEPTED_STANDARDS_DEVIATIONS.md:54-56 carries no active entry in its function-pointer section.

### CW-B37-03 -- Header does not state that the two public dispatch paths differ in validation
- Site: src/active_objects/ao_flight_director.h:33-39
- Lens: Comments and documentation quality -- JSF AV 134 ("Assumptions (limitations) made by functions should be documented in the function's preamble"); secondary Power of Ten Rule 7, parameter-validity half ("parameter validity must be checked inside each function"), which is the manual residual since only the return-check half is gated by [[nodiscard]].
- Claim: The preamble for AO_FlightDirector_dispatch_signal(int) does not state that it bypasses the Go/No-Go validation and the PIO backup-timer arm/disarm hooks that AO_FlightDirector_process_command applies, does not state what values the int parameter may take, and describes the path as CLI-only although MAVLink and fault-injection callers also use it.
- Why: process_command's preamble does carry contract -- "Process a validated flight command (Go/No-Go + dispatch)" -- so the gap is one-sided, on the dispatch_signal preamble, which says only "Dispatch a flight signal from CLI (ARM, DISARM, ABORT, RESET, etc.). Replaces cli_dispatch_flight_signal()." In the implementation, process_command runs rc::command_handler_validate (line 304) and the PIO hooks (311-325), while dispatch_signal applies static_cast<rc::FlightSignal> to an unchecked int and hands it straight to the HSM (287-288). Real non-CLI callers take the unvalidated path for ARM, DISARM and ABORT: src/active_objects/ao_telemetry.cpp:282-286 and :1092-1095 (MAVLink COMMAND_LONG from the ground station) and src/safety/fault_inject.cpp:109-112. A caller reading only this header cannot determine that the radio ARM path skips the Go/No-Go gate the CLI path applies at src/cli/rc_os.cpp:286, nor that an out-of-range int becomes an out-of-range enum value that the HSM silently drops through Q_SUPER(&QHsm_top).
- Confidence: high on the documentation gap; medium on whether the validation asymmetry itself is intended.
- Direction: State in each preamble which gate the path applies and which it does not, and take the typed rc::FlightSignal / rc::CommandType instead of int (or range-check before the casts at lines 288 and 297) so the contract lives in the signature rather than in prose.
- Verdict: RESHAPED -- the validation asymmetry and the undocumented int domain are real, but process_command's preamble does state "(Go/No-Go + dispatch)", so "the preambles say only 'Replaces ...'" and "documents neither" were overstated; narrowed to the dispatch_signal side.

### CW-B37-04 -- Queue-depth-32 rationale cites a blocking call that no longer exists
- Site: src/active_objects/ao_flight_director.cpp:58-61
- Lens: Comments and documentation quality -- CERT MSC12-C (documentation must describe code that actually runs) and NL.2 (comment and code disagree).
- Claim: The justification for the load-bearing depth-32 constant describes telemetry_radio_tick() blocking 50-150 ms inside QV_onIdle; that function does not exist anywhere in the tree and the blocking send it refers to was replaced by a non-blocking split.
- Why: A tree-wide search for telemetry_radio_tick finds only this comment and a matching stale reference at src/active_objects/ao_led_engine.cpp:313 -- no definition, no call. qv_idle_bridge (src/main.cpp:433-470) now contains the watchdog kick, eskf_runner_tick, the station retry tick, diag stats and the rc_log drain, and no radio transmit; the radio moved to AO_Radio (main.cpp:491), which uses rfm95w_send_start / rfm95w_send_poll (ao_radio.cpp:207 and 216) -- the start/poll API declared at src/drivers/rfm95w.h:210-219 precisely to remove that blocking. The consequence is that the current worst-case backlog for this 100 Hz queue is undocumented: anyone asked to justify, raise or trim depth 32 has a rationale pointing at a deleted mechanism, and the trailing "Real fix: non-blocking LoRa driver" note reads as outstanding work that has in fact already landed.
- Confidence: high
- Direction: Replace the paragraph with the present-day bound -- what actually holds off FD dispatch now, and how many 10 ms ticks that is -- or say plainly that the depth is a margin choice with no current blocking source.
- Verdict: CONFIRMED -- telemetry_radio_tick exists nowhere in the tree, qv_idle_bridge (main.cpp:433-470) contains no radio transmit, and the radio moved to AO_Radio's rfm95w_send_start/rfm95w_send_poll pair.

### CW-B37-05 -- Startup health publish reaches zero subscribers
- Site: src/active_objects/ao_health_monitor.cpp:80-81
- Lens: Comments and documentation quality (NL.2, comment and code disagree) plus the spine block B "passes-tests-yet-wrong" criterion.
- Claim: The comment "Publish initial state so consumers get health on startup" describes a publish that no consumer can receive, because every SIG_HEALTH_STATUS subscriber starts after AO_HealthMonitor and subscribes inside its own initial transition.
- Why: hm_publish is called from hm_initial, that is during AO_HealthMonitor_start, which main.cpp:501 runs at priority 6. The three subscribers are AO_Telemetry (subscribe at ao_telemetry.cpp:743, started main.cpp:516), AO_Logger (ao_logger.cpp:346, started main.cpp:513) and AO_Notify (ao_notify.cpp:165, started main.cpp:512) -- all later, so the subscriber list for SIG_HEALTH_STATUS is empty at the moment of this publish. Consumers therefore hold no health state until the first change or the 1 Hz forced re-publish at lines 101-104, up to a second later. That re-publish carries its own comment calling it the guard against "startup-order bugs"; a maintainer trusting the line-80 comment could delete it as redundant and leave the LED, log and telemetry health bytes undefined for an unbounded window.
- Confidence: high
- Direction: Either drop the initial publish and let the 1 Hz re-publish own startup (correcting the comment), or state in the comment that this publish is deliberately best-effort and that the 1 Hz re-publish is the actual startup guarantee.
- Verdict: CONFIRMED -- QActive_start runs the top-most initial transition inline (lib/qep/qv.c:238), so hm_publish fires at priority-6 start time while Notify/Logger/Telemetry subscribe later (main.cpp:512-516).

### CW-B37-06 -- Pyro-fired latch is written on four paths and read by nothing
- Site: src/active_objects/ao_flight_director.cpp:83, 94, 182, 186
- Lens: Comments and documentation quality -- CERT MSC12-C / NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine block A whole-file gestalt (dead state).
- Claim: These four assignments to director.state.drogue_fired and main_fired are the only accesses to those fields in the firmware; nothing reads them, while the fields' own comment asserts three readers.
- Why: A tree-wide search over .c/.cpp/.h finds exactly these four writes, the declarations and the init() reset at src/flight_director/flight_state.h:135-145, and nothing else. The field comment at flight_state.h:134 states "Queried by CLI, telemetry, and SPIN model (!drogue_fired guard)", but the CLI status printer in this same file (lines 337-374) prints phase, previous phase, in-phase time, transition count and the eight markers and never prints either flag; no telemetry field carries them; and the SPIN drogue_fired is a Promela model variable declared at tools/spin/rocketchip_ao.pml:53, not this field. So the model's safety property at rocketchip_ao.pml:382 has no firmware counterpart being consulted, and a reviewer asking "did a pyro fire, and was the sequence legal" finds a latch documented as persistent and authoritative that in fact influences nothing.
- Confidence: medium
- Direction: Decide which it is -- give the latch its documented reader (print it in the status block and/or carry it in the telemetry health byte), or delete the two fields with the four writes and correct the SPIN correspondence table at tools/spin/README.md:200.
- Verdict: CONFIRMED -- a tree-wide grep finds only the four writes plus the declarations and the init() reset; flight_state.h:134 claims three readers and the status printer at ao_flight_director.cpp:331-372 prints neither flag.

### CW-B37-07 -- Beacon-publish body duplicated verbatim across two callbacks
- Site: src/active_objects/ao_flight_director.cpp:222-228 and 242-248
- Lens: The spine, block A -- CppCoreGuidelines F.1 ("Package meaningful operations as carefully named functions"; its agent-tendency note is copy-pasted non-trivial lambdas across call sites) and ES.3 / Fowler "Duplicated Code".
- Claim: The set_led_cb and beacon_cb lambdas contain the same nameable action -- publish SIG_BEACON_ACTIVE -- written out twice, each with its own function-local static QEvt g_beaconEvt.
- Why: The bodies are identical apart from the trailing rc_log in beacon_cb: declare a static QEvt, assign rc::SIG_BEACON_ACTIVE, publish with &g_fdAo.super and g_fdAo.super.prio. Any change to how the beacon event is raised -- adding a payload field, changing the publisher identity, adding a dedup guard -- must be made in both places or the two beacon sources drift apart. The two identically-named but distinct statics also make it non-obvious on a quick read that one signal is backed by two event objects.
- Confidence: high on the duplication; the consequence is maintenance drift rather than a runtime defect.
- Direction: Extract a named fd_publish_beacon_active() and call it from both lambdas, keeping the rc_log at the distress-beacon call site.
- Verdict: REFUTED -- a four-line static-QEvt publish is the project's established idiom (LL Entry 35) and the finding itself concedes no runtime consequence, so extracting a helper for two four-line lambdas is a style preference, one AK_GUIDELINES 2/3 argues against.

### B38 -- AOs: rcos + logger

#### Coverage
- src/active_objects/ao_logger.cpp -- FAIL -- read whole; spine run on all nine functions plus both state handlers; findings on the baro-rate builder (comment vs body, shared differentiator cache), a misplaced function banner, and a subscribed-but-unhandled signal.
- src/active_objects/ao_logger.h -- PASS -- contract surface (helper Kind C): every prototype carries its promise, its "must be called after" precondition, and its caller set; the banner's "read-only accessors (Council A6)" framing sits oddly beside the two `_mut` accessors, but each is declared with its reason on the adjacent line, so no reader is misled.
- src/active_objects/ao_rcos.cpp -- FAIL -- read whole; spine run on the cal-UI state machine, the dispatch helpers and the public trigger API; two live functional defects (USB input starvation in MAVLink mode, discarded state-transition returns) plus comment debris and a blocking flash op inside the handler.
- src/active_objects/ao_rcos.h -- PARTIAL -- contract surface (helper Kind C): the trigger API is complete and each entry states "returns immediately", but the two claims that bound the AO's run-to-completion budget ("All calibration wizards are now non-blocking"; "synchronous but fast <500ms") are the ones the implementation does not keep -- see CW-B38-05.

#### Findings

### CW-B38-01 -- MAVLink output mode routes USB input into the dashboard key-eater, starving the only MAVLink RX path
- Site: src/active_objects/ao_rcos.cpp:278-284 (with 255-274 and 1036-1042)
- Lens: The spine, block B -- spec-noncompliance (a required step silently dropped on a path normal use does not exercise); the mechanism is stated by this file's own note at ao_rcos.cpp:62 ("kAnsi causes poll_dashboard_keys() to eat all input").
- Claim: `cli_dispatch()` sends `StationOutputMode::kMavlink` into `poll_dashboard_keys()`, which drains the entire USB CDC input buffer in a `while (getchar_timeout_us(0) != PICO_ERROR_TIMEOUT)` loop and discards every byte except x/X (and a/D on station), so no byte survives to reach the MAVLink parser.
- Why: On the vehicle the mode flips to kMavlink the moment a GCS HEARTBEAT is parsed (ao_telemetry.cpp:1077). From the next AO tick on, `cli_dispatch()` runs at ao_rcos.cpp:1036 -- before `rc_os_update()` at :1042 -- and empties the buffer; `rc_os_update()`'s single `getchar_timeout_us(0)` (rc_os.cpp:488) then returns PICO_ERROR_TIMEOUT, so `AO_Telemetry_feed_usb_byte()` (rc_os.cpp:490) is never called again. That call is the only USB MAVLink RX path on the vehicle -- `mavlink_direct_tick()` is TX-only (ao_telemetry.cpp:663-693) -- so after the first heartbeat, GCS heartbeats stop being detected and COMMAND_LONG (ARM/DISARM/ABORT from the GCS) is never dispatched: the same mode that enables MAVLink TX disables MAVLink RX. Separately, any 0x78/0x58 byte occurring inside a binary MAVLink frame (payload, seq, sysid, CRC) matches the `ch == 'x' || ch == 'X'` test at :260 and calls `enter_cli_menu()`, which forces kMenu and prints the station-labelled banner ("Station RX", GPS/Distance/GPS-Push, :191-205) into the vehicle's MAVLink stream.
- Confidence: high
- Direction: restrict `poll_dashboard_keys()` to kAnsi in `cli_dispatch()`, and give kMavlink a path that either forwards bytes to `AO_Telemetry_feed_usb_byte()` or leaves the read to `rc_os_update()`; an in-band escape in MAVLink mode needs a framing-aware trigger such as the existing ESC lockout at rc_os.cpp:377, not a raw byte match.
- Verdict: CONFIRMED -- cli_dispatch (:281-283) routes kMavlink into poll_dashboard_keys, whose while-loop at :259 drains the CDC buffer before rc_os_update's single getchar (rc_os.cpp:488), and AO_Telemetry_feed_usb_byte (rc_os.cpp:491) is the only USB MAVLink RX path; enter_cli_menu (:191-205) is ungated by role.

### CW-B38-02 -- `cal_ui_next_or_idle()` return value discarded at three sites, so skip/cancel never leaves the prompt state
- Site: src/active_objects/ao_rcos.cpp:396, 409, 635
- Lens: The spine, block B -- unchecked returns (P10 Rule 7: "the return value of non-void functions must be checked by each calling function"). The guide notes the must-check half is gated by `[[nodiscard]]` on project APIs; this file-static helper carries no attribute and the `bugprone-unused-return-value` CheckedFunctions list is `flash_safe_execute` only, so this is manual residue.
- Claim: `cal_ui_next_or_idle(RcosAo*)` computes and returns the next state but has no side effect; at :396, :409 and :635 the result is dropped, so `me->cal_ui_state` is left unchanged -- unlike the nine correct sites (:467, :479, :496, :513, :561, :606, :653, :693, :748) which assign it.
- Why: Pressing 'x' or ESC at the prompt this file itself advertises ("ENTER to start, 'x' to skip." -- :787, :801, :835, :1108, :1124, :1140) prints "Skipped." at :408 and then re-enters `cal_ui_handle_async_prompt()` on the next 20 Hz tick, still in kAsyncPrompt. That handler has no USB-disconnect escape (contrast :433 and :482) and no timeout, and `AO_RCOS_cal_active()` returns true (:1236), which makes `rc_os_update()` return early at rc_os.cpp:485 -- so the whole CLI stays unresponsive until the operator presses ENTER and runs the calibration they were trying to skip. Same shape at :635 for the compass prompt. At :396 a failed `calibration_start_*` prints the error and silently stays in kAsyncPrompt instead of advancing the wizard. In wizard mode the documented per-step skip is therefore unavailable for gyro, level, baro and mag.
- Confidence: high
- Direction: assign the result at all three sites, or have the helper set `me->cal_ui_state` itself so no call site can drop it; marking the helper `[[nodiscard]]` would turn any recurrence into a build error.
- Verdict: CONFIRMED -- cal_ui_next_or_idle is a pure function (:368-370) whose result is assigned at nine sites and dropped at :396, :409 and :635, leaving cal_ui_state non-kIdle so AO_RCOS_cal_active() (:1235-1237) keeps rc_os_update returning early at rc_os.cpp:486; neither CheckedFunctions nor [[nodiscard]] covers it.

### CW-B38-03 -- "keep previous rate" comment is not what the body does; the baro rate is published as zero on most calls
- Site: src/active_objects/ao_logger.cpp:171-182 (comment at :182)
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong"); spine block B, confabulation (NIST AI 600-1) -- a confident rationale the body does not implement.
- Claim: On the `dt_s < 0.05F` path `populate_baro_fields()` never writes `fused.baro_alt_rate_mps`, so the field keeps the caller's initial value -- zero -- rather than the previous rate the comment claims.
- Why: Both callers hand in a freshly value-initialized struct (`rc::FusedState fused = {}` at ao_logger.cpp:301, `rc::FusedState fused{}` at ao_flight_director.cpp:124), so "keep previous rate" is unreachable -- there is no previous value in that struct to keep. The shared cache advances only once per 50 ms while the two callers together invoke the builder at roughly 150 Hz, so most evaluations publish 0.0. Concretely: `guard_baro_stationary()` (guard_functions.cpp:45) tests `fabsf(rate) < threshold` with threshold 0.3 m/s (mission_profile_data.h:32), so on those ticks the landing guard reads "stationary" while the vehicle is descending -- and that guard is unmanaged, auto-dispatching SIG_LANDING once sustained (guard_evaluator.cpp:150-155, valid in DROGUE_DESCENT and MAIN_DESCENT). What holds it off today is the 5000 ms sustain window (mission_profile_data.h:33) plus the timing accident that the flight director still catches some real samples; a change to either AO's tick period narrows that margin. The logged value is wrong as well: the decimator averages 4 (PSRAM) or 8 (SRAM) samples of which about one is real (log_decimator.cpp:45,73), so recorded `baro_alt_rate_mps` under-reads by that factor for post-flight analysis and guard tuning.
- Confidence: high
- Direction: write the field on every call -- retain the last computed rate in the same cache and assign it on the sub-interval path -- or expose the rate through a getter the callers read, so a caller cannot silently receive zero; then make the comment state the retained-rate contract it is trying to describe.
- Verdict: CONFIRMED -- the dt_s < 0.05F path never writes fused.baro_alt_rate_mps, both callers pass a value-initialised struct (ao_logger.cpp:301, ao_flight_director.cpp:124), and the field feeds guard_baro_stationary (guard_evaluator.cpp:114) and the decimator average (log_decimator.cpp:45,73).

### CW-B38-04 -- the two-point baro differentiator cache has two writers at different rates and no named owner
- Site: src/active_objects/ao_logger.cpp:155-157 (used at :163-181)
- Lens: Concurrency & shared-data ownership -- JPL-C Rule 8 ("Data objects in shared memory should have a single owning task. Only the owner of a data object should be able to modify the object"); supporting CCG CP.3 (minimize the shared writable surface). Also Declaration scope & object lifetime, JSF AV 136.
- Claim: `g_prevPressurePa` / `g_prevSampleMs` are function-local statics inside a function reached from two different active objects -- AO_Logger's 50 Hz `logging_tick()` (:302) and AO_FlightDirector's 100 Hz `fd_tick()` (ao_flight_director.cpp:125) -- with no statement of which AO owns them and no hand-off, and the comment at :155 misstates their storage as file scope.
- Why: The derivative itself is arithmetically correct over whatever interval it spans, so this is not a second numerical error; the defect is ownership. The one update produced per 50 ms window is consumed by whichever AO reaches the function first -- decided by QP dispatch order rather than by design -- which is what compounds CW-B38-03. There is no data race (both AOs run on Core 0 under cooperative QV), so nothing mechanical will ever flag it; an unowned shared mutable in flight-critical code is itself the JPL-C Rule 8 finding. The comment at :155 ("Cache lives at file scope so it persists across calls") is also wrong: the storage is function-local static, and persistence across *callers* is the load-bearing fact it omits.
- Confidence: medium
- Direction: give the differentiator one owner -- compute it once in AO_Logger and expose the result through an accessor the flight director reads, or pass the cache in as caller-owned state -- and name that owner in the preamble. The comment at :155 ("Cache lives at file scope so it persists across calls") also needs correcting: the storage is function-local static, and persistence across callers is the load-bearing fact it omits.
- Verdict: RESHAPED -- the unowned two-writer cache and the wrong "file scope" comment are real, but the derivative is arithmetically correct over the interval it spans, so the "neither AO's rate corresponds to its own sampling interval" framing implied a numerical error that does not exist.

### CW-B38-05 -- flash save blocks inside the AO handler for longer than the file's own headroom arithmetic allows
- Site: src/active_objects/ao_rcos.cpp:1289-1304 (comment at :1301-1303); same call at :335-356; header claims at src/active_objects/ao_rcos.h:9-10 and :48-49
- Lens: The spine, block C -- blocking-in-cooperative-scheduler (under run-to-completion scheduling a handler that blocks past its budget starves every other object; blocking is itself the defect). Supporting: JSF AV 134 (assumptions and limitations documented in the preamble).
- Claim: `AO_RCOS_start_cal_save()` runs `calibration_save()` -- which the adjacent comment says blocks 100-500 ms via `flash_safe_execute()` -- inside the AO_RCOS tick handler, and the comment's own justification ("320 ms headroom ... tight but sufficient") does not cover the 500 ms worst case it states two lines earlier.
- Why: The chain is `rcos_ao_running` -> `rc_os_update()` (:1042) -> `handle_calibration_menu` -> `AO_RCOS_start_cal_save()` (rc_os.cpp:241), so the block happens inside a QV run-to-completion handler. AO_FlightDirector arms its time event at 1U/1U = 100 Hz (ao_flight_director.cpp:155) against a 32-deep queue (:62); 500 ms is 50 time events into 32 slots -- the exact `qf_actq id=130` overflow the project already lived through (LESSONS_LEARNED Entry 32, whose own sizing rule, depth >= (block_ms / tick_ms) x 2, would demand 100 here). The same blocking call sits at :344 inside `cal_save_to_flash()`, reached from the kResult and kAsyncWaiting states of the cal UI with no budget note at all, while ao_rcos.h:9-10 asserts "All calibration wizards are now non-blocking".
- Confidence: medium
- Direction: either establish and state the real bound (measure worst-case flash write time, size the tightest AO queue to it, and correct the two header claims), or move the save off the handler -- pause the AO tick across it, or split the flash write into a start/poll pair as the radio driver already does per LESSONS_LEARNED Entry 32.
- Verdict: CONFIRMED -- AO_RCOS_start_cal_save (:1289-1304) runs calibration_save() inside the AO tick chain (rc_os.cpp:241 via rc_os_update at :1042), the adjacent comment's 320 ms headroom does not cover the 500 ms it states two lines earlier, and ao_rcos.h:9-10/:48-49 assert non-blocking / <500 ms.

### CW-B38-06 -- abandoned design deliberation left as comments, over a dead store with a false comment
- Site: src/active_objects/ao_rcos.cpp:1215-1231
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.1 (do not restate mechanics), CCG NL.2 (comment and code disagree), CERT MSC04-C (superseded or alternative implementations retained in comment form).
- Claim: `g_rcosAo.cal_wizard_step = 0;` at :1215 is dead (overwritten at :1231) and its comment "Will be incremented to kWizardGyro in kWizardNext" is false -- 0 increments to 1, kWizardLevel -- and :1220-1230 retain ten lines of unresolved first-person deliberation ("Hmm ... Let me rethink ... Better: ... Actually simplest: ... No wait ...") proposing four designs, three of which were not taken.
- Why: A reader cannot determine from the block which scheme is live, and the retained text asserts a hazard that does not exist ("step >= kWizardDone (4) would fire immediately on 255" -- it cannot, because `cal_ui_handle_wizard_next()` increments before the comparison at :852-854). The one fact that does need recording -- that `UINT8_MAX` is a deliberate unsigned wrap so the shared increment-then-dispatch path lands on kWizardGyro -- is buried among the discarded alternatives.
- Confidence: high
- Direction: delete the dead assignment at :1215 and the deliberation at :1220-1230, keeping the single intent line already present at :1231.
- Verdict: CONFIRMED -- :1215 is overwritten at :1231, its comment is contradicted by the block's own line :1229, and :1220-1230 is retained first-person deliberation; clang-analyzer deadcode does not cover stores to a file-scope struct member.

### CW-B38-07 -- function banner says "Non-static ... Public via ao_logger.h" over a static function
- Site: src/active_objects/ao_logger.cpp:142-147
- Lens: Comments & documentation quality -- JSF AV 131 / CCG NL.2 (comment and code disagree).
- Claim: The banner at :142-143 states "Non-static: shared between AO_FlightDirector (guard evaluation) and logging_tick (PCM frame encoding). Public via ao_logger.h", but the function it heads is `static void populate_baro_fields(...)` at :147; the description belongs to `AO_Logger_populate_fused_state()` at :185, which is the function actually declared in the header.
- Why: The comment was left behind when the body was split out -- its own last line says "Extracted from AO_Logger_populate_fused_state" -- so the file now tells a reader that a translation-unit-local helper is part of the public surface shared with the flight director. That is backwards for anyone deciding whether they may call or change it, and the genuinely public function at :185 is left with no preamble at all, so the linkage claim is wrong in one place and absent in the other.
- Confidence: high
- Direction: move the "shared between AO_FlightDirector and logging_tick, declared in ao_logger.h" sentence onto `AO_Logger_populate_fused_state()` at :185, and leave `populate_baro_fields()` a one-line note of what it computes.
- Verdict: CONFIRMED -- the banner at :142-143 claims non-static/public linkage over `static void populate_baro_fields` at :147, while the genuinely header-declared AO_Logger_populate_fused_state at :185 carries no preamble.

### CW-B38-08 -- AO_Logger subscribes to SIG_HEALTH_STATUS but has no handler for it
- Site: src/active_objects/ao_logger.cpp:346 (state handler at :352-380)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation must describe code that actually runs; a doc-comment mapping a path the code cannot take is a false map).
- Claim: `QActive_subscribe(&me->super, rc::SIG_HEALTH_STATUS);` is annotated "IVP-105: health in FusedState", but `logger_ao_running()` has cases only for SIG_LOG_TICK, SIG_PHASE_CHANGE and SIG_PYRO_FIRED; SIG_HEALTH_STATUS falls through `default:` to `Q_SUPER(&QHsm_top)` and is discarded.
- Why: The health byte actually reaches FusedState by a direct read at :138 (`rc::health_monitor_get_state()->primary`), so the subscription contributes nothing but queue traffic -- AO_HealthMonitor publishes on change plus a 1 Hz re-publish (ao_health_monitor.cpp:7,58). A reader tracing how health gets into the log is pointed at an event path that does not exist, and the queue-depth rationale at :334-335 accounts only for tick events ("At 50Hz, 150ms = ~8 events"), not for the subscribed publications the AO actually receives during that same blocking window. AO_Telemetry carries the identical unhandled subscription (ao_telemetry.cpp:743), which suggests the pair was copied rather than each being wired.
- Confidence: medium
- Direction: drop the subscription and the IVP-105 comment if the direct read is the intended source, or add the case that consumes the event; either way, state in one line where the health byte enters FusedState.
- Verdict: CONFIRMED -- the subscription at :346 has no matching case in logger_ao_running (:352-380), and the health byte reaches FusedState by the direct read at :138.

### B39 -- AOs: radio + rf_manager

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/active_objects/ao_radio.cpp -- FAIL -- Read whole (807 lines); spine A/B/C run on every function, and the concurrency three-question test run on every file-scope mutable (g_spiOk, g_pendingRadioConfig/Valid, g_pendingApplyBackstopCount, g_configJustChanged, g_persistRequested, g_persistDebounceCount, g_lastRelaySeq, g_radioAo.state, the two posted static events, the four function-local static dividers) -- all single-owner in Core-0 handler context, no volatile, no locks, no cross-core reach, so the ownership half is clean; three findings sit here.
- C:/Users/pow-w/Documents/RC-agent-walk/src/active_objects/ao_radio.h -- PARTIAL -- Read whole (91 lines); evaluated as a contract surface (helper Kind C plus A) field by field -- the cooperative-dispatch access rule at line 26 is stated, the T5.5 apply/revert fields carry real intent comments, and AO_Radio_set_pending_config states its caller-validates precondition in proper JSF 134 form; the one defect is the rx_count claim at line 36.
- C:/Users/pow-w/Documents/RC-agent-walk/src/active_objects/ao_rf_manager.cpp -- FAIL -- Read whole (382 lines); spine A/B/C run on every function, concurrency three-question test run on g_rf and g_rfQueue (single owner, Core-0 cooperative dispatch, documented invariant, no volatile, no locks, no cross-core reach); three findings sit here.
- C:/Users/pow-w/Documents/RC-agent-walk/src/active_objects/ao_rf_manager.h -- PARTIAL -- Read whole (106 lines); evaluated as a contract surface (helper Kind C) member by member -- the AO Commandment V cooperative-dispatch invariant at lines 41-46 is exactly the kind of claim this lens wants stated, and the test-hook preamble at lines 95-102 documents its own gating honestly; three member/API claims are contradicted by the .cpp.

#### Findings

### CW-B39-01 -- ROCKETCHIP_RADIO_PERSIST branches reference four identifiers that no longer exist
- Site: src/active_objects/ao_radio.cpp:381, 388-389, 697-705
- Lens: The spine, block B -- "Non-self-contained / hallucinated symbols" (undefined helpers or variables; code that looks complete but is not). Same class as the residual named in LESSONS_LEARNED Entry 44.
- Claim: Both bodies guarded by ROCKETCHIP_RADIO_PERSIST assign and test s_config_just_changed, s_persist_requested, s_persist_debounce_count and s_pending_radio_config_valid, none of which is declared anywhere in the tree.
- Why: The file's actual statics are g_configJustChanged (line 85), g_persistRequested (line 95), g_persistDebounceCount (line 94) and g_pendingRadioConfigValid (line 74). A grep across src/ and include/ finds the s_-prefixed spellings only at these nine lines. ROCKETCHIP_RADIO_PERSIST is defined in no CMakeLists.txt and nothing under cmake/, so the branches never compile and the build stays green -- but the moment persistence is enabled, which the file's own comments at lines 372-375 and 645-649 describe as a deliberate, expected build mode for non-sweep operation, the translation unit fails to compile. The feature is therefore not merely off, it is unbuildable, and nothing in the file or its header says so; a reader sees two fully-written functions and a documented gate.
- Confidence: high
- Direction: Rename the four references to the current g_-prefixed statics, then prove the fix by compiling the persist configuration at least once, since the whole point of the finding is that no gate in the build currently reaches this code.
- Verdict: CONFIRMED -- the s_-prefixed names appear only at :381, :388-389 and :697-705, the live statics are g_-prefixed (:74, :85, :94, :95), and ROCKETCHIP_RADIO_PERSIST is defined in no CMakeLists.txt or cmake/ file, so the persist build cannot compile.

### CW-B39-02 -- Missed-frame counters advance per 10 Hz tick, not per nav frame, so forced-ACQ fires early below 10 Hz
- Site: src/active_objects/ao_rf_manager.cpp:214-254 (increments at 229-232)
- Lens: Comments and documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") with JSF AV 134; reinforced by the spine block B passes-tests-yet-wrong type.
- Claim: handle_tick pushes a miss slot and increments consec_missed_rx and packets_missed on every 10 Hz tick for which since_rx_ms exceeds 1.5 nav periods, so the counters measure elapsed 100 ms ticks rather than the telemetry frames the consuming thresholds are specified in.
- Why: kRfForcedAcqMinFrames = 5 and kRfForcedAcqFastFrames = 20 are specified as frame counts. src/safety/rf_link_health.h:60-74 states the intended mapping outright ("10 Hz: 5 drops = 500 ms ... 2 Hz: 5 drops = 2500 ms ... 1 Hz: 5 drops = 5000 ms") and docs/plans/STAGE_T_T14_DESIGN.md:339 gives the rationale: the frame minimum exists to protect low-rate configs from single-drop false LOS. A fixed 10 Hz increment collapses that protection. Concrete live case, the 2 Hz row of the whitelist at include/rocketchip/radio_config_table.h:51 -- silence is detected at 750 ms, consec_missed_rx reaches 5 at about 1.25 s, the 2000 ms time gate closes at 2.0 s, so rf_next_state declares forced-ACQ at 2.0 s after 4 actually-missed frames where the design specifies 2.5 s and 5 frames. The consequence is not cosmetic: transition_to_acq posts VEHICLE NOT HEARD (line 127), and once the state is kAcq, AO_RfManager_next_tx_window_us returns 0 (lines 329-331), which makes handle_tx_event drop every station TX (src/active_objects/ao_radio.cpp:197-205) -- an early LOS verdict also stops the station transmitting commands. The same defect makes the 20-frame accelerator unreachable at every nav rate (it now needs a fixed 2.0 s of ticks after detection, always later than the 2.0 s primary gate) despite rf_link_health.h:69-71 documenting it as the high-nav-rate path, and it makes packets_missed report ten drops per second of outage regardless of nav rate.
- Confidence: high
- Direction: Derive the slot advance from elapsed nav periods rather than from tick arrivals -- track the last slot boundary and push one miss per nav_period_ms crossed -- so consec_missed_rx and packets_missed are denominated in frames, as both their thresholds and their header comment assume.
- Verdict: CONFIRMED -- handle_tick increments consec_missed_rx/packets_missed once per 10 Hz tick (:229-232) while rf_link_health.h:60-71 specifies the thresholds in nav frames; the 2 Hz whitelist row (radio_config_table.h:51) reaches the 5-frame gate at ~1.2 s instead of 2.5 s, and the 20-frame accelerator can never precede the 2 s time gate.

### CW-B39-03 -- The CRC-error branch in handle_valid_rx is unreachable and packets_crc_err is structurally always zero
- Site: src/active_objects/ao_rf_manager.cpp:172, 202-206 (claim at src/active_objects/ao_rf_manager.h:55)
- Lens: Comments and documentation quality -- CERT MSC12-C, "Detect and remove code that has no effect or is never executed"; documentation must describe code that actually runs.
- Claim: The predicate good = (rx->len > 0) can never be false for a delivered RadioRxEvt, so the else arm that increments packets_crc_err, pushes a zero LQ slot and clears consec_good_rx is dead, while the header documents packets_crc_err as "Cumulative CRC failures".
- Why: src/active_objects/ao_radio.cpp:459-498 is the only producer of SIG_RADIO_RX in the tree (verified by grep over src/ and include/), and it returns early on len == 0 at line 465 and on a failed CRC at line 476 before ever populating the event, so every published event has len > 0. The comment at lines 167-171 is aware of half of this ("AO_Radio already validated for CCSDS packets") yet leaves both the branch and the counter in place, so any consumer reading RfManagerState sees a CRC-failure metric pinned at zero while the real failures are counted only in RadioAoState.rx_crc_errors. The LQ window inherits the same distortion: because CRC failures are filtered upstream, the window can only ever be degraded by the tick path, never by a corrupt packet, which is a different definition of link quality from the one the member names imply.
- Confidence: high
- Direction: Either carry a crc_ok flag on RadioRxEvt so the branch becomes live and the counter means what the header says, or delete the dead arm and the packets_crc_err member and point consumers at RadioAoState.rx_crc_errors.
- Verdict: CONFIRMED -- handle_rx_poll returns at ao_radio.cpp:465 on len==0 and at :476 on CRC failure before the only SIG_RADIO_RX publish at :497, so `good` is always true and packets_crc_err (documented at ao_rf_manager.h:55) stays zero.

### CW-B39-04 -- rx_count is documented as valid packets but is incremented before CRC validation
- Site: src/active_objects/ao_radio.h:36 against src/active_objects/ao_radio.cpp:427-443
- Lens: Comments and documentation quality -- JSF AV 131 and 134 with CCG NL.2 (comment and code disagree; treat the disagreement itself as a latent bug).
- Claim: The member comment "Total valid packets received" is contradicted by validate_rx_packet, which performs s.rx_count++ at line 431 and only then runs the CCSDS CRC check that returns false at lines 437-440.
- Why: rx_count therefore counts every received frame including CRC failures, making rx_crc_errors a subset of it rather than a disjoint sibling -- while src/cli/rc_os_commands.cpp:995 and src/cli/rc_os_dashboard.cpp:477 print the two side by side as though they were disjoint. The load-bearing consumer is the loss estimate: src/cli/rc_os_dashboard.cpp:184-186 and src/cli/rc_os_commands.cpp:952-961 compute lost = expected - rx_count from the received sequence span, so every CRC-failed packet is scored as successfully received and the operator-visible loss figure is understated by exactly rx_crc_errors -- the number an operator uses to judge whether the link is good enough to launch on. s.last_rx_seq is likewise taken from the unvalidated packet at line 436 before the CRC test, so a corrupt sequence counter can also feed expected. The naming is the root cause: validate_rx_packet is not a validator, it records metadata and validates, which is why the ordering inside it went unnoticed (spine A, CCG F.2 -- the honest name needs an "and").
- Confidence: high
- Direction: Move the metadata commit after the CRC verdict, or split the recording step from the validation step and let the caller decide what to count; either way state in the header which of the two counters contains the other.
- Verdict: CONFIRMED -- validate_rx_packet increments s.rx_count at :431 and sets last_rx_seq at :436 before the CRC verdict at :437-440, while ao_radio.h:36 calls it "Total valid packets received" and rc_os_dashboard.cpp:184-186 / rc_os_commands.cpp:952-954 derive lost = expected - rx_count from it.

### CW-B39-05 -- The TX-window contract claims RxDone anchoring that the body does not implement
- Site: src/active_objects/ao_rf_manager.cpp:325-351 against src/active_objects/ao_rf_manager.h:81-84
- Lens: Comments and documentation quality -- JSF AV 134 (a function preamble must document its assumptions and limitations) with CCG NL.2; contract-surface helper Kind C, does the prose match the signature and the body.
- Claim: The skeleton status of the TX-window arithmetic is recorded only in the .cpp (in-body comment at 343-349: "Skeleton ... Full window arithmetic per design section 6 lands in a follow-up commit"); the header preamble at ao_rf_manager.h:81-84 reads as a finished contract, and the sole caller uses the return purely as a zero / non-zero gate.
- Why: The honest limitation is recorded where a caller will not read it. The single caller, src/active_objects/ao_radio.cpp:197-205, treats the result purely as a zero / non-zero liveness gate, and its own comment at lines 190-196 tells the next reader that "station-side TX is anchored to vehicle RxDone via AO_RfManager" -- so both the contract surface and the call site read as half-duplex anchoring while the delivered behaviour is only "TX allowed unless the link is in kAcq or the deadman fired". Anyone extending this from the header would use the return as a schedule-at timestamp, where the body returns s.last_rx_us + 2000U -- the opening of the window just past, not a future slot. Secondary: 0 is both the no-window sentinel and a legal (if rare) value of last_rx_us + 2000, so the sentinel is not disjoint from the value domain.
- Confidence: medium
- Direction: State the skeleton status and the boolean-gate semantics in the header preamble until the section 6 arithmetic lands, and correct the ao_radio.cpp call-site comment to describe the gate it actually performs; if the return stays a timestamp, give no-window a sentinel outside the value domain.
- Verdict: RESHAPED -- the header/.cpp documentation split and the overstated call-site anchoring comment are real, but the body returns the opening of the current safe window rather than violating the stated contract outright, so "a timestamp already in the past" over-framed it.

### CW-B39-06 -- The TX-fail reinit recovery ignores the rfm95w_init success return
- Site: src/active_objects/ao_radio.cpp:248-255
- Lens: The spine, block B -- Power of Ten Rule 7 (the return value of non-void functions must be checked by each calling function). Manual residual rather than gate-covered: src/drivers/rfm95w.h carries no [[nodiscard]] on any declaration, so the project's nodiscard-under-Werror gate does not see this call. Flagging it even though I am not certain the gate is silent here, per the keep-if-unsure instruction.
- Claim: After three consecutive TX timeouts the handler reinitialises the radio but discards the bool that rfm95w_init returns to report whether the device was detected and configured, with no (void) cast and no stated justification.
- Why: The same function is return-checked on the boot path at line 540, so the file already treats the return as meaningful. On the recovery path a failure -- exactly what three consecutive TX timeouts suggest, an absent or wedged SX1276 -- is invisible: control falls straight into ao_radio_apply_runtime_config(s) at line 255, which writes bandwidth, spreading-factor, coding-rate and power setters into a device that may not be responding; s.initialized stays true, since no post-boot path in the file ever clears it; and handle_radio_tick keeps polling as though the radio were healthy. tx_consec_fail then climbs to the five-fail threshold at line 242 and only emits a log line, so the "error flag set" its message announces does not exist anywhere. The same dropped return appears at line 453, where handle_relay_forward calls rfm95w_send_start and then unconditionally advances the scheduler to kTxActive and increments relay_count, whereas the sibling call at line 207 correctly guards on the result -- the file writes the same idiom two ways.
- Confidence: medium
- Direction: Branch on the reinit result -- on failure clear s.initialized, skip the setter replay, and surface the state so the dashboard and the pre-arm aggregator can see a dead radio; guard the relay send_start the way line 207 does, and consider [[nodiscard]] on the bool-returning rfm95w declarations so this class stops being a hand-walk.
- Verdict: CONFIRMED -- rfm95w_init's bool is dropped at ao_radio.cpp:248 with no (void) cast while the boot call at :540 checks it, rfm95w.h carries no [[nodiscard]], and CheckedFunctions lists only flash_safe_execute; the same idiom split appears between :207 and :453.

### B40 -- AOs: telemetry + notify + led_engine

#### Coverage
src/active_objects/ao_led_engine.cpp -- PARTIAL -- Whole file read (341 lines, every helper and both state handlers spined); the dedup cache, the Core-1 vitality fallback and the dev override each carry a finding.
src/active_objects/ao_led_engine.h -- PARTIAL -- Contract surface (Kind C): the banner's "sole owner of the NeoPixel hardware" claim does not hold against the tree, and the dev-helper preamble misdescribes the layer it writes.
src/active_objects/ao_notify.cpp -- PARTIAL -- Whole file read (390 lines); intent mapping, tick handler and all four post functions walked, static-event posts verified against LL Entry 35; sensor-timeout latch and a rule mis-citation found.
src/active_objects/ao_notify.h -- PASS -- Contract surface (Kind C/D): each declared post function's prose claim (counter reset-to-full, latch-until-cleared, station-only no-op) checked against the body and matches.
src/active_objects/ao_telemetry.cpp -- FAIL -- Whole file read (1133 lines, every function spined); an undefined identifier survives only because a macro discards it, the tracked-command dedupe and safety-class logic is inert, and an RX helper does not do what its comment claims.
src/active_objects/ao_telemetry.h -- PARTIAL -- Contract surface (Kind C/E, 6 structs + 17 declarations); prose is otherwise sound but the retry-stat field comment states a retry budget the implementation no longer uses.

#### Findings

### CW-B40-01 -- Tracked-command dedupe branch is identical to the path it guards, so the safety-class carve-out does nothing
- Site: src/active_objects/ao_telemetry.cpp:945-959
- Lens: The spine, block A (CCG ES.3 "Don't repeat yourself, avoid redundant code" / Fowler Duplicated Code) and block B (NIST AI 600-1 confabulation -- a confident rationale the body does not implement). Not gate-caught: bugprone-branch-clone matches if/else and switch-case clones, not an if-body-with-return against the fall-through that follows it; flagged as unsure-if-gated per the walk rule.
- Claim: The "newest-wins dedupe" branch executes exactly the same four statements as the fresh-send path that follows it (allocate seq from g_cmdSeq, build MavCmdParams, populate_pending, tx_tracked_command_wire), so neither the dedupe nor its documented safety-class exemption changes behaviour.
- Why: The comment at 937-944 states that safety-class commands "bypass dedupe -- every deliberate press is preserved as its own tracked command with its own ACK window". There is exactly one pending slot (g_pendingCmd, 361-372) and populate_pending overwrites it wholesale, so a second ARM press while the first ARM is still pending destroys the first command's seq and ACK window -- identical to the dedupe path it is supposed to bypass. The late ACK for the first ARM is then dropped by the seq/cmd_id match at 512-516, and is_tracked_command_safety_class (876-884) is computed for no effect. Nothing exercises this: the tracked-command path is inside #ifndef ROCKETCHIP_HOST_TEST (886, 936) and no host test references AO_Telemetry at all, so the T14a council decision is both untested and currently unimplemented.
- Confidence: high
- Direction: Either implement what the comment claims (per-class or queued pending slots, so a safety-class command cannot evict an in-flight one), or collapse the branch and correct the comment to say all tracked commands are newest-wins on a single slot -- then re-decide whether the safety-class distinction is still wanted.
- Verdict: CONFIRMED -- lines 945-953 execute the identical four statements as the fall-through at 955-959, so is_tracked_command_safety_class (:876-884) changes nothing and populate_pending still overwrites the single g_pendingCmd slot.

### CW-B40-02 -- try_mavlink_rx feeds a second full parser one byte per frame and discards its 770-byte result, under a comment saying it does not re-parse
- Site: src/active_objects/ao_telemetry.cpp:342-356
- Lens: Comments and documentation quality (JSF AV 131 / CCG NL.2 -- comment and code disagree), with The spine block B (dead computation, unchecked return) and Declaration scope and object lifetime (CCG ES.5 -- oversized short-lived object).
- Claim: The comment "Feed parser bookkeeping path -- doesn't re-parse (COMM_2 consumed)" is false: mavlink_rx_feed_byte runs its own mavlink_parse_char on MAVLINK_COMM_1 and dispatches complete frames, but it is handed only the final byte of each COMM_2 frame, so it can never assemble one and its output is never read.
- Why: src/telemetry/mavlink_rx.cpp:36 sets kRxChannel = MAVLINK_COMM_1 and :311 parses on it, then dispatch_message writes response frames into result->buf. Because the call at 350 fires only when the COMM_2 parser has already completed a frame, the COMM_1 parser gets a one-byte-per-frame trickle, never reaches a frame boundary, and neither dispatches nor responds; result is destroyed unread each iteration. Meanwhile MavlinkRxResult is 770 bytes (include/rocketchip/mavlink_rx.h: buf[768] + len) zero-initialised on the Core 0 handler stack on every completed frame -- against the project rule that treats 256B-1KB locals as "consider static" (docs/MULTICORE_RULES.md). The bool return is ignored. Delete the comment and a reader correctly concludes the call is dead; keep it and the reader believes an RX bookkeeping path exists that does not.
- Confidence: high
- Direction: Decide what the LoRa-side MAVLink RX is meant to do -- either drop the mavlink_rx_feed_byte call and its result buffer (COMM_2 plus handle_parsed_mavlink already do the work), or feed every received byte to it and actually transmit result->buf -- then rewrite the comment to match.
- Verdict: CONFIRMED -- mavlink_rx_feed_byte parses on MAVLINK_COMM_1 (mavlink_rx.cpp:36, :311) and dispatches at :315, but is fed one byte per completed COMM_2 frame at ao_telemetry.cpp:350, and both the 770-byte MavlinkRxResult and the bool return are discarded.

### CW-B40-03 -- Three QACTIVE_POST call sites name an identifier, l_telemAo, that exists nowhere in the tree
- Site: src/active_objects/ao_telemetry.cpp:860
- Lens: The spine, block B (non-self-contained code / undefined symbols -- code that looks complete but is not), with Comments (CERT MSC12-C: a reference to something the system no longer has).
- Claim: Lines 860, 903 and 1022 pass &l_telemAo.super as the QACTIVE_POST sender argument, but the AO instance is g_telemAo (:124) and l_telemAo is defined in no file; the file compiles only because the non-Q_SPY form of the macro discards that argument.
- Why: lib/qep/qp.h:772-773 defines QACTIVE_POST(me_, e_, dummy) as ((void)QActive_post_((me_), (e_), QF_NO_MARGIN, (void *)0)) -- the third argument is never expanded, so the undefined name is never compiled. Q_SPY appears nowhere in CMakeLists.txt, cmake/, lib/qep/qp_port.h or src/, so today the defect is invisible; the first build that defines Q_SPY (the documented way to get QP tracing) fails to compile this file with three undefined-identifier errors, and the sender attribution the trace exists to record is wrong in intent as well. This is residue of the l_ to g_ AO-static rename (LESSONS_LEARNED Entry 44) that no compiler pass could catch; every other post site in this batch passes nullptr or me.
- Confidence: high
- Direction: Replace the three occurrences with the real instance (&g_telemAo.super), or with nullptr to match the sibling AOs, and note macro-discarded arguments as a blind spot for future symbol sweeps.
- Verdict: CONFIRMED -- l_telemAo appears only at ao_telemetry.cpp:860, :903 and :1022, the instance is g_telemAo (:124), and lib/qep/qp.h:772-773 discards the third macro argument when Q_SPY is undefined, which it is tree-wide.

### CW-B40-04 -- The "sensor phase" 5-minute timeout is an unconditional uptime latch, so a healthy vehicle shows the timeout pattern from T+5:00 onward
- Site: src/active_objects/ao_notify.cpp:126-127
- Lens: The spine, block B (passes-tests-yet-wrong on an unexercised path; spec-noncompliance), with Comments (CCG NL.2 -- the name and comment say "sensor phase timeout", the code implements "time since AO start").
- Claim: notify_evaluate_sensor_status tests only elapsed time since a stamp taken once at AO init and never re-based, ahead of every health branch, so state.sensor latches to SensorIntent::kTimeout five minutes after boot regardless of ESKF or GPS state and never clears.
- Why: sensor_phase_start_ms is written once at :158 and nowhere else; kSensorPhaseTimeoutMs is 300000 (:120); the timeout branch is the first arm of the chain, so from T+5:00 it pre-empts the eskf-init, GPS-fix and no-GPS branches for the rest of the boot. src/notify/notify_backend_led.cpp:73 returns 0 for PhaseIntent::kIdle, so before ARM the sensor category is what actually drives the LED -- a vehicle with a 3D fix sitting on the pad more than five minutes (routine on a launch day) switches from the GPS-3D pattern to kSensorTimeout and stays there until an ARM raises a phase intent. src/active_objects/ao_led_engine.cpp:169 renders that as solid magenta, the same rendering as kFaultNeoCore1Stall at :176, so what the operator sees is indistinguishable from the highest-priority fault. test/test_notify.cpp exercises only resolve_led_pattern, never this evaluator, so the suite stays green.
- Confidence: high
- Direction: Decide what the window is actually timing (most plausibly "sensors never became ready"), then gate the branch on that condition -- report kTimeout only while ESKF is still uninitialised, or re-base sensor_phase_start_ms when the sensors do come up -- and state the intended meaning in the comment.
- Verdict: CONFIRMED -- sensor_phase_start_ms is written only at :158 and the elapsed test at :126 is the first arm of the chain, so state.sensor latches to kTimeout at T+5:00 and renders as the same solid magenta as kFaultNeoCore1Stall (ao_led_engine.cpp:169 vs :176).

### CW-B40-05 -- Seven extraction comments cite "JSF AV Rule 1" for a 60-line function cap that Rule 1 does not impose
- Site: src/active_objects/ao_notify.cpp:241
- Lens: Comments and documentation quality (JSF AV 134 / CCG NL.2 -- a stated rationale that is false), against the project's own rule-citation discipline (LESSONS_LEARNED Entry 37: verify the wording, and verify it is the governing standard).
- Claim: Two extraction comments attribute a 60-line function cap to JSF AV Rule 1 -- ao_notify.cpp:241 ("the JSF AV Rule 1 60-line cap") and ao_telemetry.cpp:473 ("the 60-line JSF AV Rule 1 cap") -- but Rule 1 is the 200 L-SLOC rule and the 60 comes from P10 Rule 4 / the readability-function-size gate; five further comments in these two files cite Rule 1 generically for extractions the 60-line gate actually drove.
- Why: docs/audits/RULE_VERIFIABILITY_TRIAGE.md:129 states rule 1 as "function size (200 L-SLOC)" with "P10-4/JPL 25 govern (pointer-row)", and .clang-tidy sources the 60 from "P10 target: 60 lines per function" while giving JSF the 200-statement threshold; neither yields a 60-line JSF cap. The two explicit sites are ao_notify.cpp:241 and ao_telemetry.cpp:473; the generic ones are ao_notify.cpp:177, :199 and ao_telemetry.cpp:238, :442, :553-554 (the same generic string also appears at ao_logger.cpp:146, ao_flight_director.cpp:203 and :212, flight_director.cpp:241 and :267, guard_evaluator.cpp:89, sensor_core1.cpp:388). A reader deciding whether an extraction is still required, or auditing whether the codebase honours JSF 1, reasons from a rule that says something else -- the LL-37 failure mode.
- Confidence: high
- Direction: Re-cite to the governing source (P10 Rule 4, or the readability-function-size gate) or drop the citation and keep the plain readability rationale; grep the rest of src/ for the same string while fixing.
- Verdict: RESHAPED -- only ao_notify.cpp:241 and ao_telemetry.cpp:473 actually attribute the 60-line cap to JSF AV Rule 1; the other five cite Rule 1 generically without naming a line count, so "seven comments cite Rule 1 for a 60-line cap" was overstated.

### CW-B40-06 -- Retry-accounting comments still describe a 3-retry budget after the constant moved to 8, and the retry-timeout seed comment says 500 ms where the code seeds 250 ms
- Site: src/active_objects/ao_telemetry.cpp:407-410
- Lens: Comments and documentation quality (JSF AV 131 / CCG NL.2 -- comment and code disagree, with NL.2's warning that both are then suspect).
- Claim: The RetryStats field comments define first_try_ack_count as "ACK with retries_left == 3" and total_retries_used as "sum of (3 - retries_left)", and ao_telemetry.h:51 defines failed as "all 3 retries exhausted", while kAckMaxRetries is 8 (:95) and the code computes retries_used as kAckMaxRetries - retries_left (:487, :976); separately :827 says the ACK-retry timeout "Seeds at 500 ms until first apply" where :83 seeds 250 ms.
- Why: These counters exist to drive the Stage T Batch C retry decision from field data, and the diag surface that exposes them (AO_Telemetry_get_retry_stats, :986) is read by a human against exactly these definitions. Anyone reducing a captured first_try or total_retries_used column with the documented "3" derives a wrong retries-used figure and a wrong first-try rate -- for the very measurement the whiteboard says will re-baseline the 95% first-try gate. The 500-vs-250 comment also contradicts the same file's own history block at 68-83, which records the 250 ms seed correctly, so the file disagrees with itself about a safety-command retry window.
- Confidence: high
- Direction: Express these counts in terms of kAckMaxRetries rather than a literal (here and in ao_telemetry.h:51), and correct :827 to reference the single seed definition at :83.
- Verdict: CONFIRMED -- :407 and :410 define the counters against a literal 3 while kAckMaxRetries is 8 (:95) and the code computes kAckMaxRetries - retries_left (:487), ao_telemetry.h:51 repeats "all 3 retries", and :827 says 500 ms where :83 seeds 250 ms.

### CW-B40-07 -- The LED dedup cache assumes AO_LedEngine is the only writer of the WS2812 state, but the header names a second writer and a third exists
- Site: src/active_objects/ao_led_engine.cpp:94-104
- Lens: Concurrency and shared-data ownership (JPL-C Rule 8 -- single owning task, owner-only mutation; CCG CP.3 -- minimise the shared writable surface), read together with the contract surface at ao_led_engine.h:13-14.
- Claim: led_set_if_changed suppresses the hardware call whenever the requested mode and colour equal its shadow copy (last_mode / last_color), so any write to the WS2812 driver from outside this AO desynchronises that cache and can leave the LED stuck on the foreign pattern -- and the header's "No other module calls ws2812_set_mode() or ws2812_update() directly (except Core 1 pause indicator ..., debug-only)" is both an admission of the second writer and incomplete.
- Why: src/core1/sensor_core1.cpp:358-365 sets SOLID orange on entry to the Core 1 I2C pause and SOLID blue on exit, from Core 1, while AO_LedEngine runs on Core 0. That pause is triggered by ordinary operations -- src/active_objects/ao_rcos.cpp:343 and src/cli/rc_os_commands.cpp:1042 and :1098 around flash writes -- not by a debug-only path as the header states. After the pause the hardware is SOLID blue while the AO's cache still holds the last pattern it applied; on the next tick the resolved pattern is unchanged, led_set_if_changed compares equal and returns without calling ws2812_set_mode, so the status LED stays solid blue until the composited pattern happens to change -- a stale display of vehicle state after every calibration save. src/main.cpp:291 is a third writer (launch-abort solid red) the header does not mention at all, and it is silently overwritten by the first LedEngine tick once QF_run starts.
- Confidence: medium
- Direction: Either route the foreign writers through the AO (post a pattern or an override) or let the AO invalidate its cache -- for instance re-issue the current mode unconditionally at a low cadence, or expose a cache-dirty hook the pause path sets. Either way, correct the header so it lists every real writer and its context.
- Verdict: CONFIRMED -- led_set_if_changed (:94-104) suppresses the driver call on an unchanged shadow while sensor_core1.cpp:358-365 writes the driver from Core 1 on a pause triggered by ordinary flash saves (ao_rcos.cpp:343, rc_os_commands.cpp:1042, :1098), and main.cpp:291 is a third writer the header omits.

### CW-B40-08 -- The Core-1 vitality fallback is skipped on exactly the failure it exists to catch
- Site: src/active_objects/ao_led_engine.cpp:258-261
- Lens: The spine, block B (happy-path-only handling -- the failed-read exit is unhandled), with Concurrency and shared-data ownership (the cross-core read protocol's failure mode).
- Claim: led_check_core1_vitality runs only when seqlock_read succeeds, so a Core 1 that stops inside the seqlock write window leaves the sequence permanently odd, every subsequent read fails, the stall counter never advances, and layers[kLayerFault] is never set to kFaultNeoCore1Stall.
- Why: include/rocketchip/sensor_seqlock.h:130-145 returns false after four attempts that all observe an odd sequence, and seqlock_write at :120-128 publishes odd, memcpys 156 bytes, then publishes even -- a Core 1 hard fault, lockup or debugger halt between those two stores leaves the counter odd forever. In that state the tick handler at 258-261 never enters the stall path at 191-205, core1_stall_ticks freezes at its last value, and the compositor keeps showing the Notify or idle layer: a normal-looking LED while Core 1 is dead. The comment at 182-189 states this check exists precisely as the local net for when AO_Notify or AO_HealthMonitor have themselves crashed, so on that path nothing else covers it. AO_Notify has the same shape at ao_notify.cpp:202-205, where a failed read silently retains the previous sensor intent.
- Confidence: medium
- Direction: Treat a persistently failing seqlock_read as a stall signal in its own right -- count consecutive read failures alongside the loop-count comparison and raise the fault layer at the same threshold -- rather than skipping the check entirely.
- Verdict: CONFIRMED -- ao_led_engine.cpp:259-261 calls the vitality check only when seqlock_read succeeds, and sensor_seqlock.h:130-145 returns false on every subsequent call once the sequence is left odd by a Core 1 stop inside seqlock_write (:120-128).

### CW-B40-09 -- The dev-only override is documented as forcing the Fault layer, but it outranks the Fault layer
- Site: src/active_objects/ao_led_engine.h:37-41
- Lens: Comments and documentation quality (JSF AV 131 / CCG NL.2 -- comment and code disagree), with CCG P.3 (the name does not express what the function does).
- Claim: The header's stated rationale for AO_LedEngine_dev_force_fault_layer -- "force a pattern into the Fault layer so it wins over AO_Notify's continuous re-publishes" (ao_led_engine.h:37-41) -- misdescribes the mechanism: the function writes g_devOverridePattern (cpp:339-341), which the compositor tests before any layer (cpp:212-216), so the forced pattern outranks the Fault layer rather than occupying it, and the layer array a reader would inspect is not the state driving the display.
- Why: The .cpp comment at 336-338 does record the mechanism ("Writes a single static variable that the compositor checks first"), so the header is the surface that is wrong; neither the function name nor the header says that a genuine Core 1 stall raised at cpp:201-202 is masked while an override is held, because the compositor returns at :215 before the layer loop runs. Scope is bounded -- the helper is dev-CLI-only (dev_cli.cpp, "Not for flight code") and the only exit is a second call with 0 -- so the practical consequence is confined to an LED-test session, but a reader of the header can derive neither fact.
- Confidence: medium
- Direction: Rename to reflect what it is (an override above all layers) and say in both comments that it masks the Core-1 stall fault while set; alternatively make it write layers[kLayerFault] so the documented priority actually holds.
- Verdict: RESHAPED -- the header's "into the Fault layer" wording is genuinely wrong, but the .cpp comment at :336-338 does say the compositor checks the variable first and the helper is dev-CLI-only, so the "neither comment tells a reader" framing was overstated.

### B41 -- top-level: main + shared_state

#### Coverage
- C:/Users/pow-w/Documents/RC-agent-walk/src/main.cpp -- FAIL -- Read whole (544 lines); spine run on all 16 functions plus the idle bridge and main(); boot-order, GPS bring-up and cross-core launch claims verified against the files they cite.
- C:/Users/pow-w/Documents/RC-agent-walk/src/shared_state.cpp -- FAIL -- Read whole (46 lines); every definition traced to its declaration and to its real readers/writers across src/; the defect set is contract-level and shared with the header.
- C:/Users/pow-w/Documents/RC-agent-walk/include/rocketchip/shared_state.h -- FAIL -- Read whole (77 lines) as the paired contract surface per the contract-surface helper; each declaration and each ownership annotation judged against actual access.

#### Findings

### CW-B41-01 -- Boot hangs forever in baro auto-zero on any non-vehicle role that has a barometer
- Site: src/main.cpp:366-376 (loop at :371-373)
- Lens: The spine, block C (peripheral init SEQUENCE / lifecycle; passes-tests-yet-wrong on an unexercised path) + Power of Ten Rule 2 (every loop must have a fixed upper bound)
- Claim: init_baro_auto_zero() waits on calibration_is_active() with no bound and no timeout, and its only exit is fed by a producer that the role gate immediately above it may never start.
- Why: calibration_start_baro() sets CAL_STATE_BARO_SAMPLING, and that state is left only when calibration_feed_baro() accumulates kBaroCalSamples (50) samples (src/calibration/calibration_manager.cpp:369-400, 1043-1047 -- no timeout, no failure exit). The sole caller of calibration_feed_baro() in the tree is src/core1/sensor_core1.cpp:207, inside Core 1's sensor loop. init_core1_role() (main.cpp:345-361) starts that loop only for job::DeviceRole::kVehicle; on station/relay Core 1 stays idle (the station_idle_tick.cpp header comment confirms "IMU/baro/health remain zero on station since Core 1 is idle here"). But init_sensors() (main.cpp:177-183) probes and starts the DPS310 with no role gate, so g_baroContinuous can be true on a station. Concrete path: station or relay build, DPS310 present on the I2C bus, anomalous_boot_verdict() == kProbablyOnPad -- Core 0 enters the loop at :371 and never leaves. This is before QF_run() and before init_pio_safety(), so there is no AO, no CLI banner (AO_RCOS never starts), and no watchdog to observe it; USB CDC is already up, so the board enumerates and then does nothing. The vehicle role has the same unbounded shape on a narrower path: if the baro fails during the ~1 s sampling window, sensor_core1.cpp:214-221 escalates to g_baroInitialized = false, which gates off further baro reads at sensor_core1.cpp:413, so the feed stops and the same loop never terminates. The codebase already has the bounded form of exactly this wait -- core1_i2c_pause() polls a Core 1 ack for kPauseAckMaxMs and documents the timeout path (src/safety/core1_i2c_pause.cpp:24-33) -- so the shape is available and was not used here.
- Confidence: high
- Direction: Gate init_baro_auto_zero() on the same role condition that starts the Core 1 sensor loop (or on g_sensorPhaseActive), and bound the wait with a sample timeout that calls calibration_reset_state() and logs the skip, mirroring the kPauseAckMaxMs pattern.
- Verdict: CONFIRMED -- verified at main.cpp:366-376: the wait has no bound and its only producer is calibration_feed_baro() at sensor_core1.cpp:207, which Core 1 stops calling once it sets g_baroInitialized=false (:221), and which never runs at all on a non-vehicle role since init_core1_role() gates the sensor loop on kVehicle while init_sensors() starts the baro with no role gate.

### CW-B41-02 -- Early GPS bring-up runs the exact sequence that init_sensors() documents as forbidden
- Site: src/main.cpp:217-227 (called at :238) vs src/main.cpp:124-127 and :156-161
- Lens: Comments and documentation quality (NL.2 -- "if the comment and the code disagree, both are likely to be wrong"; JSF AV 134 -- function assumptions and limitations) + The spine block C (peripheral init SEQUENCE)
- Claim: init_gps_early() performs the full PA1010D I2C bring-up before the IMU is initialized, contradicting the GPS-last ordering that init_sensors() states as a hardware requirement in the same file and never reconciles; and where the early bring-up succeeds, the transport preference documented in init_gps() is never exercised.
- Why: init_sensors() states as a hardware requirement: "Init order matters: IMU + baro FIRST, GPS LAST ... Probing the GPS (0x10) triggers NMEA streaming which can corrupt AK09916 init transactions. Defer GPS probe until after IMU bypass mode is fully established." init_gps_early() runs from init_early_hw() (main.cpp:238), i.e. before init_sensors() is reached at :299, and calls gps_pa1010d_init(), which is not a probe but a full bring-up: i2c_bus_recover(), three blind PMTK writes and eight buffered reads at 0x10 (src/drivers/gps_pa1010d.cpp:203-243). The early path is not undocumented -- its own comment gives the reason (main.cpp:217-218: the MT3333 has a brief I2C slave window after cold boot that init_sensors() misses by hundreds of ms) -- so the defect is narrower than "which ordering is real": init_sensors()'s comment was never updated, so it still presents GPS-last as a hardware constraint that the boot path in the same file does not honour, and a reader has no way to tell which of the two is current. Second, narrower consequence on the same path: init_gps() documents "UART first, I2C fallback ... UART GPS has no I2C bus contention (LL Entry 24), preferred for production", but init_sensors() calls init_gps() only when !g_gpsInitialized (main.cpp:185), so on a board where board::kUartGpsAvailable is true (Feather RP2350, Pico 2) and a PA1010D also answers on I2C, the early path binds GPS_TRANSPORT_I2C at :225 and the preferred UART transport is never attempted. That combination is reachable but is not the documented vehicle build, so this half is a latent override rather than an observed one.
- Confidence: high (on the contradiction and the transport pre-emption); the hardware consequence of the ordering cannot be judged from the source
- Direction: Decide which ordering is authoritative and rewrite or delete the losing comment; if the early cold-boot window is genuinely required, say so in init_sensors() and make the early path explicitly conditional (board or role, or only where no UART GPS exists) so it cannot silently override the documented transport preference.
- Verdict: RESHAPED -- the early path does carry its own documented rationale (main.cpp:217-218, the cold-boot MT3333 window), which the original finding did not credit; what survives is that init_sensors() still states GPS-last as a hardware requirement with no reconciliation, and that a successful early I2C bind skips the documented UART-first preference.

### CW-B41-03 -- Unconditional g_gpsInitAttempted defeats the attempted-vs-not-installed distinction it feeds
- Site: src/main.cpp:222
- Lens: The spine, block B (spec-noncompliance -- a documented semantic requirement silently dropped) + Comments (JSF AV 134 -- the flag's documented contract)
- Claim: init_gps_early() latches g_gpsInitAttempted = true before any evidence a GPS exists, on every board and every role, so a board with no GPS installed reports a GPS failure at boot.
- Why: shared_state.h:40-41 defines the flag's contract: "Init-attempted flags (IVP-142c). Distinguishes 'attempted and failed' from 'not present on this role'." The consumers implement exactly that: check_sensor() returns without counting when attempted is false (src/cli/rc_os_commands.cpp:811-818) and print_hw_failures() prints "[FAIL] GPS" only when attempted-and-not-initialized (rc_os_commands.cpp:833; same pattern at :635). The other two write sites honour the contract -- main.cpp:141 sets it only after i2c_bus_probe(kGpsPa1010dAddr) succeeds, main.cpp:129 only when the board declares kUartGpsAvailable. Line 222 sets it with no probe and no board condition. Concrete input: any board with no GPS on the bus (LL Entry 20 documents the PA1010D being physically removed from the vehicle Qwiic chain) now counts one hardware FAIL in the boot summary and prints "[FAIL] GPS" on terminal connect -- precisely the false alarm IVP-142c added the flag to remove.
- Confidence: high
- Direction: Set g_gpsInitAttempted in init_gps_early() only on the path where the blind PMTK sequence produced evidence of a device (alongside the g_gpsInitialized = true assignment), or make it conditional on a board/role capability constant the way line 129 does.
- Verdict: CONFIRMED -- main.cpp:222 sets the flag with no probe and no board guard, while the contract at shared_state.h:40-41 and both consumers (rc_os_commands.cpp:811-818 count_hw_checks, :833 print_hw_failures) treat attempted-and-not-initialized as a hardware FAIL; the other two write sites (main.cpp:129, :141) are guarded.

### CW-B41-04 -- Two init flags are written from both cores as plain bools while their six siblings are atomics
- Site: include/rocketchip/shared_state.h:35,37 ; src/shared_state.cpp:15,17
- Lens: Concurrency and shared-data ownership (JPL-C Rule 8 -- single owning task, owner-only mutation; CP.8 -- no non-atomic cross-context sharing) + Declaration scope and object lifetime (CCG CP.2 -- avoid data races)
- Claim: g_baroInitialized and g_gpsInitialized are plain bool objects written by Core 0 and by Core 1 and read by both, with no atomic type, no volatile, and no barrier.
- Why: Core 0 writes them during init (main.cpp:179, and :131/:147/:224). Core 1 writes them at runtime to retire a dead device -- sensor_core1.cpp:221 "Declare baro dead" and sensor_core1.cpp:276 "GPS dead -- stop polling". Core 0 then reads them from the health pipeline and CLI (health_monitor.cpp:178, :256, :763 gps_has_lock; rc_os_commands.cpp:617-635, :817-818, :832-833; ao_notify.cpp:130), and Core 1 reads them in its own loop (sensor_core1.cpp:413, :419). None of these accesses is synchronized, so this is the data race the concurrency lens's three-question test exists to catch, and the consequence is safety-facing: Core 1's "dead device" retirement is a plain store with no release and Core 0's health-monitor read is a plain load the compiler may hoist or keep in a register, so GO/NO-GO and the boot summary can keep reporting a sensor healthy after Core 1 has declared it dead. The same header declares six genuinely cross-core flags as std::atomic<bool> (shared_state.h:67-72) and docs/MULTICORE_RULES.md states that even volatile is insufficient across cores -- the correct mechanism is established in this very file and these two objects are the exceptions.
- Confidence: high
- Direction: Promote both to std::atomic<bool> with release stores on the retirement writes and acquire loads on the health reads, matching the six sibling flags; alternatively give each a single owning core and hand the retirement across via one of the existing atomics.
- Verdict: CONFIRMED -- both are plain bool (shared_state.cpp:15,17), written by Core 1 at sensor_core1.cpp:221 and :276 and read by Core 0 at health_monitor.cpp:178/:256/:763, rc_os_commands.cpp:617-833 and ao_notify.cpp:130, with no atomic, volatile or barrier, against six std::atomic<bool> siblings in the same header and MULTICORE_RULES.md's explicit rule.

### CW-B41-05 -- Ownership annotations on the contract surface name readers that do not exist
- Site: include/rocketchip/shared_state.h:32,36,75
- Lens: Comments and documentation quality (NL.2 -- state intent, plus the comment/code disagreement warning; JSF AV 134 -- documented assumptions) applied to a contract surface
- Claim: Three of the header's ownership annotations describe Core 1 access that no Core 1 code performs.
- Why: These annotations are the only ownership documentation the file carries, and the concurrency lens's "who owns it, who mutates it, what barrier" question is answered from them. Line 32 annotates g_neopixelInitialized "Core 1 reads" -- its only readers are main.cpp:290 and rc_os_commands.cpp:747/804/829, all Core 0. Line 36 annotates g_baroContinuous "Core 1 reads" -- its only readers are main.cpp:316/367, rc_os_commands.cpp:439 and eskf_runner.cpp:234, all Core 0; src/core1/ never mentions it. Line 75 annotates g_sensorPhaseActive "Core 0 write, Core 0/Core 1 read for gating" -- its readers are eskf_runner.cpp:560, cal_hooks.cpp:107, rc_os_commands.cpp:501 and core1_i2c_pause.cpp:17/36, all Core 0 contexts. The effect is that a reviewer auditing the cross-core surface is pointed at three plain bools as cross-core (they are not) while the two that genuinely are cross-core carry weaker annotations (CW-B41-04); the map is wrong in both directions.
- Confidence: high
- Direction: Re-derive each annotation from actual access and state the barrier next to the reader, e.g. "Core 0 write / Core 0 read only -- no cross-core access", so the annotation set can be checked against a grep on the next edit.
- Verdict: CONFIRMED -- a tree-wide grep finds no src/core1/ reader of g_neopixelInitialized or g_baroContinuous, and every g_sensorPhaseActive reader (cal_hooks.cpp:107, rc_os_commands.cpp:501, eskf_runner.cpp:560, core1_i2c_pause.cpp:17/36) is a Core 0 context, so all three annotations name access that does not occur.

### CW-B41-06 -- The seqlock header re-declares the whole cross-core atomic set, contradicting the "one place" claim
- Site: include/rocketchip/sensor_seqlock.h:151-160 vs include/rocketchip/shared_state.h:64-72
- Lens: The spine (CCG ES.3 -- don't repeat yourself; Fowler's Duplicated Code smell) + Comments (NL.2 -- comment/code disagreement)
- Claim: g_sensorSeqlock and all six cross-core atomics are declared in two headers, so the cross-core extern set has two declaration points and any change to it has to be made in both.
- Why: shared_state.h:3-13 says the file "Consolidates all init flags, GPS function pointers, seqlock, atomics, and device handles from main.cpp ... this makes ownership clear", and shared_state.cpp:7 says "Single translation unit keeps cross-core state in one place for review and linking." Both of those claims hold -- the definitions are in one TU, and they did move out of main.cpp. What does not hold is a single declaration point: sensor_seqlock.h:151-160 carries an identical extern set. It compiles because shared_state.h:25 includes sensor_seqlock.h and identical extern redeclarations are legal, so no gate can see it. The consequence is bounded rather than silent: because the two headers are always pulled in together, a re-type in only one of them is a compile error, not drift. What it actually costs is the two-place edit and the split map -- adding, removing or re-typing a cross-core flag (for example the retype proposed in CW-B41-04) has to be done in two headers, and a reviewer grepping for the cross-core contract lands on whichever header they hit first.
- Confidence: high
- Direction: Delete the duplicated extern block from sensor_seqlock.h and have it include shared_state.h (or the reverse), leaving exactly one declaration point so the file-header claim becomes true.
- Verdict: RESHAPED -- the duplicate extern set at sensor_seqlock.h:151-160 is real, but the 'one place' claims the finding cited are about the single TU and the move out of main.cpp, both of which hold, and because shared_state.h:25 always pulls sensor_seqlock.h in, a re-type in one header is a compile error rather than silent drift.

### CW-B41-07 -- Dead entries on the shared-state contract: one atomic with no user, one flag written and never read
- Site: src/shared_state.cpp:28,40 ; include/rocketchip/shared_state.h:52,68
- Lens: Comments and documentation quality (CERT MSC12-C -- detect and remove code that has no effect or is never executed, and the documentation that maps it) applied per the contract-surface helper
- Claim: g_sensorPhaseDone is declared twice, defined, and never read or written anywhere in the tree; g_calStorageInitialized is assigned once and never read.
- Why: A repository-wide search over .cpp/.h/.c/.py finds g_sensorPhaseDone only at sensor_seqlock.h:155, shared_state.h:68 and its definition at shared_state.cpp:40 -- no producer and no consumer, yet it sits inside the block commented "Cross-core synchronization atomics", so a reviewer auditing the cross-core handshake counts a signalling flag that participates in nothing. g_calStorageInitialized has exactly one appearance outside its declaration and definition: the assignment at main.cpp:252; nothing consumes it, so a calibration_storage_init() failure is recorded into a global that no status display, health check or boot summary ever inspects -- the result looks captured and is not. Neither is visible to a gate: these are externally-linked objects, so -Wunused-variable does not apply, and no unused-variable check is enabled in .clang-tidy.
- Confidence: high
- Direction: Delete g_sensorPhaseDone (both declarations and the definition) unless a pending consumer is intended, in which case say so in the comment; either consume g_calStorageInitialized in the boot summary alongside the other init flags, or drop it and check calibration_storage_init()'s result at the call site.
- Verdict: CONFIRMED -- a repo-wide grep finds g_sensorPhaseDone only at its two declarations and its definition (shared_state.cpp:40) with no producer or consumer, and g_calStorageInitialized only at main.cpp:252 plus its declaration and definition; no enabled clang-tidy check or -W flag covers unused externally-linked objects.

### CW-B41-08 -- Stale watchdog-timeout constant in main.cpp, with a comment that documents its own obsolescence
- Site: src/main.cpp:89-90
- Lens: Comments and documentation quality (CERT MSC12-C -- remove the stale doc-comment along with the dead code it maps; NL.1)
- Claim: kWatchdogTimeoutMs is unused in main.cpp and its comment records that the watchdog it named moved elsewhere.
- Why: The only other occurrence of the identifier in src/ is a separate constant in rc_os_commands.cpp:795; nothing in main.cpp reads line 90's copy. Its comment, "Watchdog (moved to pio_watchdog; see OPT-IVP-01 for fault protection extraction)", states the very reason it is dead. The live watchdog described at main.cpp:414-416 is the PIO heartbeat, which "never resets the chip" and carries its timeout elsewhere -- so a reader looking up the watchdog budget in main.cpp finds a 5000 ms figure that governs nothing, sitting next to a comment asserting the mechanism has moved. Ungated: -Wunused-const-variable is not enabled by -Wall/-Wextra for C++, and .clang-tidy enables no unused-variable check (only misc-unused-parameters, misc-unused-using-decls, bugprone-unused-return-value). Low severity, recorded for completeness.
- Confidence: high
- Direction: Delete the constant and its comment; if the PIO heartbeat's timeout is a figure a reader of main.cpp needs, leave a one-line pointer to safety/pio_watchdog next to the watchdog_kick_tick() comment instead.
- Verdict: CONFIRMED -- kWatchdogTimeoutMs occurs exactly once in main.cpp (line 90) and is never read there; -Wunused-const-variable is not implied by -Wall/-Wextra for C++ and .clang-tidy enables no unused-variable check, so nothing mechanical reports it.

---

## Tier 4 — CLI


### B42 -- cli: rc_os core

#### Coverage

src/cli/rc_os.cpp -- FAIL -- Read whole; spine run on all 20 functions; the mechanism-level checks pass (cross-core atomic correctly ordered, dashboard pause/resume paired on every exit, no stack-local QP events, arm buffer in bounds), but one function is unreachable and several comments assert things the body does not do.
src/cli/rc_os.h -- FAIL -- Read whole as the module's contract surface (helper Kind C primary, Kind A secondary for the atomic flag); the owner/mutator/barrier story for rc_os_mag_cal_active is sound, but four of the header's claims are contradicted by the implementation or have no consumer.

#### Findings

### CW-B42-01 -- MAVLink input handler is unreachable; its documented 'm' exit can never run
- Site: src/cli/rc_os.cpp:388-399 (caller at 493-494)
- Lens: The spine block B (never-executed path behind a confident comment) + Comments lens item 6 (CERT MSC12-C -- documentation describing code that is never executed)
- Claim: handle_mavlink_input() can never take any action, because rc_os_update() returns one line earlier on exactly the condition the function requires in order to proceed.
- Why: handle_mavlink_lockout() returns true whenever AO_RCOS_get_output_mode() == StationOutputMode::kMavlink (the third term of the disjunction at 373-375), and rc_os_update line 493 returns immediately when it does. handle_mavlink_input's first statement (389) returns false unless the mode is kMavlink. The two conditions are complementary, so lines 390-398 are dead on every path: the "MAVLink mode off. CLI active." escape does not exist at runtime -- only the ESC path at 376-381 leaves MAVLink mode. Corroborating independently: in that same mode AO_RCOS's cli_dispatch() / poll_dashboard_keys() (src/active_objects/ao_rcos.cpp:277-283 and 255-273) drains the whole USB input queue one call earlier in the same 20 Hz tick, so rc_os_update's getchar at 488 normally sees nothing at all when the mode is kMavlink.
- Confidence: high
- Direction: Decide which exit is intended and delete the other. If 'm' is meant to toggle MAVLink mode off, that test has to run before the lockout check (and the byte-stealing dashboard drain in kMavlink mode has to be resolved first); if ESC is the only exit, delete handle_mavlink_input together with its comment.
- Verdict: CONFIRMED -- handle_mavlink_lockout()'s third disjunct (rc_os.cpp:375) is exactly handle_mavlink_input()'s entry precondition (:389), and rc_os_update() returns at :493 whenever the lockout is true, so lines 390-398 are unreachable on every path.

### CW-B42-02 -- rc_os_update()'s header contract states three things the body does not do
- Site: src/cli/rc_os.h:50-64 (body src/cli/rc_os.cpp:472-520)
- Lens: Comments lens items 2 and 4 (CCG NL.2 -- "if the comment and the code disagree, both are likely to be wrong"; JSF AV 134 -- preamble assumptions) + contract-surface helper Kind C
- Claim: The preamble says the function "Runs calibration state machines", is called "from main loop", and returns "true if a command was processed"; the body does none of the three.
- Why: (a) Line 486 does the opposite -- it returns false as soon as AO_RCOS_cal_active() is true, and the calibration UI is driven by cal_ui_tick(), a separate sibling call in the AO_RCOS tick (src/active_objects/ao_rcos.cpp:1042-1045); the .cpp's own note at 135-136 already says the UI moved. (b) The only caller is that AO tick handler at 20 Hz, not the main loop. (c) Line 519 returns true unconditionally, including when handled is false -- an unrecognised key in the calibration or flight menu (272, 328) -- and on the lockout path at 493 where no command ran; conversely line 479 returns false after dev_eskf_live_poll() consumed a key. No caller reads the value today (ao_rcos.cpp:1042 discards it), so the first caller that believes the documented meaning inherits a wrong answer on those paths.
- Confidence: high
- Direction: Rewrite the preamble to the behaviour that exists (USB connect/settle/banner, arm-confirm, single-key dispatch, and that it yields while a cal UI sequence runs), and either make the return value mean what it says or redocument it as "input byte consumed".
- Verdict: CONFIRMED -- all three preamble claims fail against the body: :486 yields instead of running the cal state machine (cal_ui_tick is a sibling call at ao_rcos.cpp:1045), the sole caller is the AO tick at ao_rcos.cpp:1042 rather than the main loop, and :519 returns true unconditionally including on the lockout path.

### CW-B42-03 -- MAVLink-lockout rationale cites the wrong key and the wrong flash operation
- Site: src/cli/rc_os.cpp:344-346
- Lens: Comments lens item 2 (NL.2 comment/code disagreement) + spine block B confabulation (NIST AI 600-1 -- a confident, specific justification the code does not support)
- Claim: The comment justifies the lockout with "0x4C='L' triggers flash erase, crashing AO scheduler", but 'L' does not erase flash.
- Why: 0x4C is 'L', and the main-menu fallthrough dispatcher maps 'l'/'L' to cmd_flush_log() (src/cli/rc_os_commands.cpp:1497); flight-log erase is lowercase 'x' = 0x78 (rc_os_commands.cpp:1498, where uppercase 'X' is deliberately routed elsewhere). The lockout is well-motivated -- both handlers do flash work under the AO scheduler -- but the stated byte-to-consequence mapping is wrong, so anyone auditing "which stray bytes are dangerous" from this comment guards the wrong byte and mis-scopes the hazard.
- Confidence: high
- Direction: State the hazard by class ("a stray printable byte can reach the log-flush and erase handlers, which do flash work under the AO scheduler") or correct the byte/handler pair.
- Verdict: CONFIRMED -- 0x4C='L' dispatches to cmd_flush_log() at rc_os_commands.cpp:1497; flight-log erase is 'x' at :1498, so the comment's byte-to-consequence mapping is wrong even though the lockout itself is well-motivated.

### CW-B42-04 -- rc_os_read_accel is an exported callback contract that nothing ever invokes
- Site: src/cli/rc_os.h:105-113 (definition src/cli/rc_os.cpp:61)
- Lens: Comments lens item 6 (MSC12-C -- doc-comment on code that never runs) + contract-surface helper Kind C (a promise with no consumer)
- Claim: The rc_os_read_accel function pointer is assigned once and never called anywhere in the tree.
- Why: The only reference outside this declaration/definition pair is the assignment at src/main.cpp:317; a tree-wide search for read_accel turns up no call through the pointer. The 6-position calibration named in the doc-comment now acquires samples through calibration_start_6pos_position() / calibration_6pos_position_sample_count() in src/active_objects/ao_rcos.cpp:472-560. This is the identical defect the file's own R-17/R-18 note at rc_os.cpp:62-67 records as fixed ("both pointers were assigned at main.cpp:315 but never invoked anywhere") -- that audit removed two of the three pointers and left this one, still carrying a preamble that asserts a live blocking contract ("Should block until a fresh sample is available (~10ms at 100Hz)").
- Confidence: high
- Direction: Confirm the 6-pos path no longer needs a caller-supplied accel read; if so, retire the typedef, the extern, the definition and the main.cpp assignment in one change, and check whether cal_read_accel (src/calibration/cal_hooks.cpp:37) is left orphaned by it.
- Verdict: CONFIRMED -- a tree-wide grep finds rc_os_read_accel only at its typedef and extern (rc_os.h:112-113), its definition (rc_os.cpp:61) and the assignment at main.cpp:317; there is no call through the pointer, and cal_read_accel is likewise referenced only by that assignment.

### CW-B42-05 -- rc_os_is_calibrating() promises more than it reports
- Site: src/cli/rc_os.h:71-74 (body src/cli/rc_os.cpp:526-528)
- Lens: Comments lens item 4 (JSF AV 134 -- undocumented limitation on an exported predicate) + contract-surface helper Kind C
- Claim: The header promises "Check if a calibration is currently in progress", but the body reports only the low-level sampling state, not the calibration sequence the rest of the module actually gates on.
- Why: The body returns calibration_is_active(), which is false in CAL_STATE_IDLE / COMPLETE / FAILED (src/calibration/calibration_manager.cpp:1043-1047). A 6-position run spends most of its wall-clock time in the UI's prompt, validating and computing states with the manager idle -- where AO_RCOS_cal_active() (ao_rcos.cpp:1235-1237) is true and this predicate is false. This file's own guards use the AO predicate, not this one (rc_os.cpp:204, 486), so the exported accessor contradicts the module's internal notion of "calibrating". It has zero callers tree-wide (as do rc_os_get_menu() and rc_os_is_connected()), so nothing is broken today; the exposure is the first caller -- a preflight gate or a station UI -- that believes the sentence and lets an operation start mid-calibration.
- Confidence: high
- Direction: Either narrow the doc to "low-level calibration sampling in progress" or make the body report the union with the AO cal-UI state; if the accessor is genuinely unused, retire it alongside the other two under the project's dead-code discipline.
- Verdict: CONFIRMED -- calibration_is_active() is false in IDLE/COMPLETE/FAILED (calibration_manager.cpp:1043-1047), and a 6-pos run enters CAL_STATE_ACCEL_6POS_SAMPLING only per position (:493), so the manager is idle across the prompt and rotate intervals where AO_RCOS_cal_active() -- the predicate this file's own guards at :204 and :486 use -- is true.

### CW-B42-06 -- Ownership comment on rc_os_mag_cal_active names a setter that does not exist
- Site: src/cli/rc_os.h:128-136
- Lens: Comments lens item 2 (NL.2) + contract-surface helper Kind A (owner / mutator / barrier) -- the itinerary row's concurrency check for this file
- Claim: The doc says the flag is "Set to true by cmd_mag_cal()"; no cmd_mag_cal() exists anywhere in the tree.
- Why: Every write is in src/active_objects/ao_rcos.cpp (622 sets it; 603, 650, 658, 689 and 742 clear it), all with memory_order_release; the single reader is src/core1/sensor_core1.cpp:420 with memory_order_acquire. The mechanism passes the three-question test -- one writing context (AO_RCOS on Core 0), one reading context (Core 1), an explicit release/acquire pair -- so this is not a barrier finding. What fails is the map: a reader asking "who is allowed to set this, and on which paths does it get cleared" is sent to a symbol that does not exist and will not find the one set and five distinct clear paths, one of which is the abort path that must run or Core 1 stops reading GPS for the remainder of the session.
- Confidence: high
- Direction: Replace the function name with the owning context and the rule -- set and cleared by AO_RCOS's mag-cal sequence on Core 0, read by Core 1 before core1_read_gps(), release/acquire -- so the claim survives the next rename.
- Verdict: CONFIRMED -- no cmd_mag_cal() exists anywhere in the tree; every write is in ao_rcos.cpp (one set at :622, five clears at :603/:650/:658/:689/:742) and the single reader is sensor_core1.cpp:420.

### CW-B42-07 -- Five file-scope constants are unused, and three name a mechanism the next comment says is gone
- Site: src/cli/rc_os.cpp:35-43 (with line 52)
- Lens: Comments lens item 6 (MSC12-C -- documentation of code that no longer exists) + Declaration scope & object lifetime (JSF AV 143 / CCG ES.21 -- a name introduced before, or without, any use)
- Claim: kRcOsPollMs, kResetConfirmTimeoutUs, kResetConfirmBufSize, kResetConfirmMaxIdx and kUsbSettleMs each occur exactly once in the tree -- their own definitions.
- Why: The three kResetConfirm* constants describe a buffered 10-second "type YES" reset confirmation, while line 52, nine lines below, states "(no extra state needed -- reset confirmation is blocking)"; they are the residue of a removed mechanism sitting in the first thing a reader of this file sees. kUsbSettleMs = 200 is worse than merely unused: the settle that actually runs is a five-tick counter in handle_usb_connect (402-427) driven by the 20 Hz AO tick, so editing the 200 changes nothing and the constant misstates the real settle window. On gating -- the build applies -Wall -Wextra -Wpedantic -Wshadow -Wfloat-equal -Wnon-virtual-dtor (CMakeLists.txt:303-304, 607) with no -Wunused-const-variable (not implied by -Wall for C++), and src/cli/** is exempt from the clang-tidy script (scripts/audit/full_tree_clang_tidy.sh:37, 53), so nothing mechanical reports these.
- Confidence: high
- Direction: Delete the four dead constants. For the settle time, either derive the tick count from kUsbSettleMs inside handle_usb_connect or replace it with a named tick-count constant, so the number a reader edits is the number the code uses.
- Verdict: CONFIRMED -- each of the five constants occurs exactly once in the tree (its own definition at rc_os.cpp:35/39/40/41/43), the real settle is the five-tick counter in handle_usb_connect (:404-427), and src/cli/** is exempt from the clang-tidy gate (scripts/audit/full_tree_clang_tidy.sh:37,53) with no -Wunused-const-variable for C++.

### CW-B42-08 -- Orphaned NOLINTNEXTLINE attaches a size-exemption rationale to a comment line
- Site: src/cli/rc_os.cpp:343-347
- Lens: Comments lens item 2 (NL.2 -- comment and code disagree) + spine block B (a confidence marker that has not been earned)
- Claim: The NOLINTNEXTLINE(readability-function-size) at 343 suppresses nothing, and its rationale ("USB state machine, splitting breaks state tracking") describes no function at that location.
- Why: NOLINTNEXTLINE applies to the following line only; line 344 is a comment and line 347 is the g_mavlinkDetected declaration -- there is no function here at all. The three sibling suppressions (142, 201, 295) do sit on their functions, so a reader takes this one as an equally reviewed exemption and attaches its "USB state machine" reason to the MAVLink lockout block it now heads, while the function it was presumably written for (handle_usb_connect at 404, or rc_os_update at 472) carries no marker. The consequence is confined to the reader today -- clang-tidy does not run on src/cli/** at all -- so the defect is the misdirected rationale, not a live suppression hole.
- Confidence: high
- Direction: Move the suppression onto the function it was written for, or delete it if that function no longer needs one.
- Verdict: CONFIRMED -- line 343's NOLINTNEXTLINE is followed by a comment (344-346) and then the g_mavlinkDetected declaration at 347, with no function at that location, while the three sibling suppressions (142, 201, 295) do sit on their functions.

### CW-B42-09 -- The arm-confirm teardown sequence is written out three times
- Site: src/cli/rc_os.cpp:438-441, 455-457, 464-467
- Lens: The spine block A (CCG ES.3 "don't repeat yourself" / Fowler Duplicated Code; CCG F.1 -- a nameable action left inline and unnamed)
- Claim: The identical three-statement "end the ARM confirm session" idiom -- clear g_armConfirmActive, rc_os_dashboard_resume(), print_prompt() -- is repeated at all three exits of handle_arm_confirm() instead of being named once.
- Why: The copies have already drifted from their counterpart: rc_os_start_arm_confirm() (360-368) resets g_armBufPos on entry, and none of the three teardowns clear it, so the index survives a session end and is saved only by the next start resetting it. Any later addition to the teardown -- clearing the buffer, logging the outcome, recording why the dashboard resumed -- must be made in three places on the ARM path, which is the drift this rule exists to prevent. The resource half is clean: one pause at 365 and a resume on every one of the three exits, so this is not a leak-on-unhappy-path finding.
- Confidence: medium
- Direction: Extract one end_arm_confirm() helper called from all three exits, and settle there whether g_armBufPos should be cleared on exit as well as on entry.
- Verdict: REFUTED -- a three-statement teardown repeated at three exits of one 38-line function, with no live consequence (g_armBufPos is reset on entry at :363, which the finding itself concedes), is a DRY preference rather than a defect.

### B43 -- cli: rc_os_commands

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_commands.cpp -- FAIL -- Read whole (1590 lines, ~40 functions); spine blocks A/B/C run on every function; defects found on the station-distance, flight-download, preflight and boot-summary paths, plus a cross-TU constant duplicate whose comment cites a file that does not define it.
- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_commands.h -- PARTIAL -- Declaration-only contract surface (Kind C, API/behavioral contract) evaluated per the helper: all 9 declared prototypes resolve to real definitions with matching signatures, but the contract prose is false in two places and the header under-declares the module's public surface.

Notes carried with the coverage verdict, not filed as findings:

- Itinerary hot-spot cue ("3 volatile T2 command handoff -> concurrency 3-question test"), answered for g_t2_pending / g_t2_cmd / g_t2_p1 (rc_os_commands.cpp:53-55). Owner: stage_t2_queue_command (line 57), reached only from cli_handle_unhandled_key (line 1545) -- Core 0, QV handler context. Mutator/reader: stage_t2_fire_pending_if_any (line 67), reached only from handle_rx_packet in src/active_objects/ao_telemetry.cpp:642-643 -- also Core 0, also QV handler context, also inside the same ROCKETCHIP_STAGE_T2_CHEAT guard. Barrier: none needed -- both contexts are run-to-completion handlers on one core, so there is no cross-core surface and no ISR preemption. The volatile qualifiers therefore buy nothing, but they are not standing in for a missing barrier either. No concurrency finding; the block is build-flag-gated throwaway code and its comments (lines 45-51, 64-66) match what the code does.
- The ESKF globals this file reads directly (g_eskf, lines 262-307 and 894-929) are updated by eskf_runner_tick(), which runs in qv_idle_bridge (src/main.cpp:433-448) -- Core 0 idle. CLI display runs in Core 0 handler context. Under QV cooperative scheduling idle cannot run while a handler runs, so these unsynchronized reads are not a CP.2 race. No finding.
- cmd_findme_beacon uses a function-scope static QEvt (line 1585) before QActive_publish_ -- correct per the scope/lifetime lens' canonical QP case (LL Entry 35). PASS.
- The -Wshadow hit previously measured at this file's line 1332 is remediated; the surviving comment records the decision.
- Relevant to how much weight the manual pass carries here: src/cli/** is exempted from the clang-tidy gate (scripts/audit/full_tree_clang_tidy.sh:37 and :53), so none of this file is covered by readability-function-size, bugprone-infinite-loop, bugprone-integer-division, or any other tidy check. Where a finding below might normally be argued as gated, it is not gated for this file.

#### Findings

### CW-B43-01 -- Station distance staleness gate compares two unrelated clocks
- Site: src/cli/rc_os_commands.cpp:1356-1362
- Lens: The spine, block B (passes-tests-yet-wrong / spec-noncompliance) plus CCG ES.3 (the same "is telemetry stale" knowledge expressed twice, one of them wrong)
- Claim: age_ms is computed as station uptime minus the vehicle's mission-elapsed time (to_ms_since_boot(get_absolute_time()) - rx->met_ms), which subtracts two clocks with different origins, so the 5-second staleness gate does not measure telemetry age.
- Why: met_ms is documented as "MET from CCSDS secondary header" (src/active_objects/ao_telemetry.h:61) -- a vehicle-side counter. The station-local receive timestamp is rs->last_rx_ms (src/active_objects/ao_radio.h:35), and the same file uses it correctly 400 lines earlier at line 950 to compute packet age. Concrete path: the station has been powered for 10 minutes on the pad (uptime ~600000 ms) while the vehicle reports a small MET, so age_ms is ~600000, the age_ms > 5000 test is true, and the d key answers "Distance: telemetry stale" on every press even though packets are arriving normally. In the opposite ordering (vehicle booted long before the station) the subtraction underflows uint32_t to a near-4-billion value -- the same wrong verdict. If the two clocks happen to be close, the gate passes and never fires when telemetry really is stale. The check is additionally wrapped in an ifndef ROCKETCHIP_HOST_TEST, so no host test exercises it.
- Confidence: high
- Direction: Compute the age against the station-local receive timestamp that print_station_rx_fields already uses, and consider hoisting that one age computation into a named helper so the two call sites cannot diverge again.
- Verdict: CONFIRMED -- line 1357 subtracts rx->met_ms (vehicle MET from the CCSDS secondary header, ao_telemetry.h:61) from station uptime, while the same file computes packet age correctly from rs->last_rx_ms at :950; the two clocks have different origins, so the 5-second gate does not measure telemetry age in either direction.

### CW-B43-02 -- Download loop divides by an unvalidated flash-resident frame size and can stop making progress while feeding the watchdog
- Site: src/cli/rc_os_commands.cpp:1179-1207 (entered from :1213-1236)
- Lens: The spine, Power of Ten Rule 7 second half ("parameter validity must be checked inside each function") plus block B (unchecked / optimistic use of returned data)
- Claim: stream_flight_binary uses entry.frame_size, read straight out of the flash flight table, as a divisor and as the loop's progress term with no local zero or range check, contrary to P10 Rule 7's second half and to the sibling guard that exists for the same parameter.
- Why: cli_do_download_flight validates only the operator-supplied flight number (line 1216) and then passes the entry through; flight_table_get_entry (src/logging/flight_table.cpp:66-73) is a bare bounds-checked memcpy that does not validate the entry, and the per-entry helper flight_entry_validate_crc (same file, line 39) is never called on this path. Nothing in stream_flight_binary checks frame_size before dividing kFlashSectorSize by it at 1181 and multiplying by it at 1193 and 1199, even though the sibling module rejects exactly this value for exactly this parameter (ring_init returns false on frame_size == 0, src/logging/ring_buffer.cpp:40). The integrity that makes this safe today lives entirely outside the function: frame_size originates from rb->frame_size, which ring_init already rejected if zero, and the whole flight table -- entries included -- is CRC-validated on load (flight_table_validate_crc at src/logging/flash_flush.cpp:87, called from flight_table_load at :127, which falls back to flight_table_init when validation fails). So no input an operator can supply reaches this function with frame_size == 0, and this is a defensive-check gap rather than a reachable hang. The failure shape is still worth recording in case the invariant is ever broken by a logger change: frames_per_sector at 1181 would be a divide-by-zero, batch and batch_bytes at 1197-1199 would be 0, frames_sent would never advance, and the while at 1189 would spin while calling rc::pio_watchdog_feed() at 1206.
- Confidence: high
- Direction: Validate the entry before streaming -- non-zero frame_size that divides sensibly into a sector, plus the existing per-entry CRC check -- and bail with a message rather than entering the loop; independently, make the loop's progress unconditional so a zero-length batch cannot spin.
- Verdict: RESHAPED -- the missing local check is real (nothing validates entry.frame_size before the divide at :1181), but the wedge it predicted needs frame_size == 0, and that value is blocked at its source (ring_buffer.cpp:40) and covered end-to-end by the table CRC checked at load (flash_flush.cpp:87 via :127), so this is a defensive-check gap rather than a reachable hang.

### CW-B43-03 -- Blocking CDC writer checks the connection once at entry, then waits without bound
- Site: src/cli/rc_os_commands.cpp:1164-1177
- Lens: The spine, block C ADD (blocking-in-cooperative-scheduler) plus block B (defensive-looking branch whose unhappy path is not walked)
- Claim: cdc_write_blocking tests tud_cdc_connected() only before the loop and then busy-waits on tud_cdc_write_available() with no timeout, no re-check of the connection, and a watchdog feed on the stalled branch.
- Why: The comment above it states the deliberate design ("waits for CDC backpressure rather than dropping. Operator wants complete data"), and blocking for backpressure is a defensible choice; the defect is that the wait is unbounded on a path the operator can trivially trigger. If the host disconnects, closes the terminal, or simply stops draining part-way through a multi-megabyte flight download, avail stays 0, the else branch never runs, written never advances, and line 1170 feeds pio_watchdog_feed() on every pass -- so the loop never exits and the watchdog cannot end it. Because this runs in a Core 0 QV handler (reached from AO_RCOS's download completion callback, src/active_objects/ao_rcos.cpp:966), a run-to-completion handler never returns and every other active object is starved. This is the same failure shape as LL Entry 32's blocking-handler rule, with the added twist that the recovery mechanism is being fed. Secondary and weaker: the return of tud_cdc_write is dropped at line 1173 and written += chunk assumes the full chunk landed.
- Confidence: high
- Direction: Bound the stall -- re-test tud_cdc_connected() inside the loop and give the wait a deadline after which the download aborts with a marker the host-side tool can see -- rather than feeding the watchdog while making no progress.
- Verdict: CONFIRMED -- tud_cdc_connected() is tested once at :1165 and never re-tested; the loop at :1167-1176 has no deadline and feeds rc::pio_watchdog_feed() at :1170 on the no-progress branch, so a host that stops draining mid-download parks a Core 0 QV handler (reached from ao_rcos.cpp:966) indefinitely with the recovery mechanism suppressed.

### CW-B43-04 -- Dropped seqlock_read results print a zero-filled snapshot as live data, including on the preflight go/no-go screen
- Site: src/cli/rc_os_commands.cpp:1445-1452 (same defect at :899 and :1427-1432)
- Lens: The spine, Power of Ten Rule 7 (unchecked return used as if it always succeeds) plus the declaration-scope / object-lifetime lens' JSF 143 half (value not yet meaningful when read)
- Claim: Three call sites ignore seqlock_read's boolean result and then format the zero-initialized destination struct as if it held a real sample.
- Why: seqlock_read (include/rocketchip/sensor_seqlock.h:130-146) returns false after kSeqlockMaxRetries collisions with the Core 1 writer, leaving the destination untouched; it is not marked nodiscard, so nothing gates the drop. The same file checks the result correctly at line 503 and prints "Seqlock read failed" -- so the failure is known to be real. At line 1445 the destination is a value-initialized shared_sensor_data_t (all zeros) and the guard on line 1447 tests hs->mcu == kHealthAbsent or snap.mcu_die_temp_c < -100.0F. That sentinel is documented as -999.0 for "sensor not initialized" (include/rocketchip/sensor_seqlock.h:92-96), so a failed read yields 0.0F, sails past the guard, and PREFLIGHT prints "MCU temp: GO 0.0C" -- a fabricated reading on the operator's go/no-go screen, indistinguishable from a real one. At line 1428 the same pattern renders "fix=0 sats=0" on the GPS non-GO line, which reads as "no satellites" rather than "no sample". At line 899 the effect is limited to a bogus B=0 frame count.
- Confidence: high
- Direction: Check the return at all three sites and render an explicit "sample unavailable" for the affected fields, the way lines 503-506 already do; the preflight site is the one that matters most because it feeds an arm / no-arm decision.
- Verdict: CONFIRMED -- seqlock_read (sensor_seqlock.h:130-146) returns false without touching the destination and is not [[nodiscard]]; the result is dropped at :899, :1428 and :1446, and the preflight guard at :1447 tests against the -999.0 sentinel documented at sensor_seqlock.h:92-96, so a zero-filled snapshot prints as a real MCU temperature while the same file checks the result correctly at :503.

### CW-B43-05 -- Boot summary's FAIL count and FAIL list are computed from different criteria and can disagree
- Site: src/cli/rc_os_commands.cpp:798-837
- Lens: The spine, CCG ES.3 / Fowler Duplicated Code (one checklist, two divergent encodings) plus block B (passes-tests-yet-wrong)
- Claim: count_hw_checks and print_hw_failures each re-encode the same pass/fail checklist by hand, and their criteria have already drifted apart in both directions.
- Why: Two concrete divergences. First, PSRAM is counted as an unconditional pass (line 820) while print_psram_status (lines 653-673) renders a real FAIL when PSRAM is present but its self-test failed -- so the summary line reports "Hardware: N/N OK" for a board whose detail view says the PSRAM self-test failed, and print_hw_failures lists nothing. Logging is the same shape at line 821 versus the "Logging: not initialized" branch at line 687. Second, in the other direction, the radio is counted as a fail whenever it is not initialized (line 819) but is only listed as a fail when g_spiInitialized is additionally true (line 834). On the SPI-bus-init-failure path -- g_spiInitialized is assigned from spi_bus_init() at src/main.cpp:248 and passed into AO_Radio_start at :491, so a failed SPI bus guarantees an uninitialized radio -- the operator sees "Hardware: 13/14 OK (1 FAIL)" followed by an empty failure list, and cli_print_hw_status prints nothing about the radio at all because neither branch at lines 764/778 is taken. The file demonstrates it knows the right idiom: the check_sensor helper at lines 811-818 exists precisely to keep "not installed" out of the fail count, and the radio does not use it.
- Confidence: high
- Direction: Derive the summary count and the failure list from one shared per-check table (name, predicate, applicability flag) so the count and the list cannot state different things, and decide explicitly whether PSRAM and Logging are "not applicable" or genuinely checkable.
- Verdict: CONFIRMED -- count_hw_checks counts PSRAM and Logging as unconditional passes (:820, :821) while print_psram_status renders [FAIL] on a failed self-test (:653-674), and the radio is counted as a fail at :819 whenever uninitialized but listed only when g_spiInitialized at :834, so an SPI-bus failure (main.cpp:248 feeding AO_Radio_start at :491) yields a FAIL count with an empty failure list and no radio line at all (:764/:778).

### CW-B43-06 -- ESKF buffer capacity is a local copy of another module's private constant, and its comment names the wrong file
- Site: src/cli/rc_os_commands.cpp:161-162 (consumed at :306-307)
- Lens: Comments & documentation quality (JSF 131/134, CCG NL.2 -- comment and code disagree) plus the spine, CCG ES.3 and block B confabulation (a confidently-stated cross-reference that does not exist)
- Claim: The comment asserts "must match main.cpp kEskfBufferSamples", but main.cpp defines no such symbol -- the real owner is src/fusion/eskf_runner.cpp:58 -- and the duplicated value silently desynchronizes the ratio printed at lines 306-307.
- Why: Line 307 prints "buf: N/M samples" with the numerator taken from the exported eskf_runner_get_buffer_count() and the denominator taken from this file's own copy of 1000. src/fusion/eskf_runner.cpp:58 holds the authoritative constant and it is file-static, so nothing links the two. Change the buffer to 2000 in eskf_runner.cpp and this CLI starts printing "buf: 1500/1000 samples": no build break, no test failure, and the reader who follows the comment goes to main.cpp and finds nothing to reconcile against. The same duplication pattern is already dead one block later -- kWatchdogTimeoutMs at lines 794-795 carries the same "must match main.cpp" claim, duplicates src/main.cpp:90, and is never used anywhere in this file.
- Confidence: high
- Direction: Delete the local copy and take the capacity from the runner (export an accessor beside eskf_runner_get_buffer_count() if needed) so the printed ratio has one source; delete the unused kWatchdogTimeoutMs duplicate rather than leaving a hand-maintained cross-file invariant with no consumer.
- Verdict: CONFIRMED -- the comment at :161 names main.cpp, which defines no such symbol; the authoritative constant is the file-static kEskfBufferSamples at eskf_runner.cpp:58, and the local copy is the denominator of the ratio printed at :306-307. The unused kWatchdogTimeoutMs duplicate at :794-795 is confirmed too.

### CW-B43-07 -- Header contract prose is false: the module does own state, and cmd_findme_beacon's documented behaviour is only the vehicle branch
- Site: src/cli/rc_os_commands.h:6-8 and :37-41
- Lens: Comments & documentation quality (JSF 134 assumptions/limitations, CCG NL.2) applied to a Kind C contract surface per the contract-surface helper
- Claim: The header's two load-bearing prose claims -- "owns no state" and the description of cmd_findme_beacon -- are both contradicted by the implementation.
- Why: On the first claim, the module holds three pieces of mutable state: the function-static cycle index at rc_os_commands.cpp:94, which is not a display cache but the variable that decides which radio configuration the station commands the vehicle onto on each r press; the three volatile Stage-T2 handoff variables at lines 53-55; and the static event at line 1585. A reader who trusts "reads state from AO public APIs and sensor seqlock, owns no state" would reasonably assume r is idempotent or that the target config lives in an AO -- it does not, it lives here and advances on every press with no way to query or reset it. On the second claim, the header documents cmd_findme_beacon as publishing SIG_BEACON_MANUAL so AO_Notify flips beacon_manual, but the body (lines 1577-1589) branches on kRadioModeRx: on a station build it publishes nothing locally and instead sends MAV_CMD_USER_1 over the radio and waits for an ACK. Half the builds do not do what the contract says. The .cpp has the correct role-split explanation at lines 1572-1576; only the header -- the file a caller actually reads -- is wrong.
- Confidence: high
- Direction: Correct the preamble to name the state the module owns (the radio-config cycle index in particular), and move the role-split sentence from the .cpp comment into the header declaration so the contract matches both builds. While there, state the role gating for cli_handle_unhandled_key, several of whose keys are silent no-ops on the vehicle role.
- Verdict: CONFIRMED -- the header's 'owns no state' (line 7) is contradicted by the function-static radio-config cycle index at :94, the Stage-T2 volatiles at :53-55 and the static event at :1585; and cmd_findme_beacon's documented local publish is only the vehicle branch, with the station branch (:1578-1582) sending MAV_CMD_USER_1 instead.

### CW-B43-08 -- Header under-declares the module's public surface; consumers hand-roll their own extern declarations
- Site: src/cli/rc_os_commands.h:13-41
- Lens: Contract-surface helper, Kind C ("does the file state the whole public surface?") plus CCG ES.3
- Claim: Two external-linkage entry points defined in this translation unit -- cli_do_erase_flights (cpp:1091) and cli_do_download_flight (cpp:1213) -- are absent from the header, and a third, stage_t2_fire_pending_if_any (cpp:67), has no header presence at all.
- Why: Because the header does not declare them, the consumer re-declares them itself: src/active_objects/ao_rcos.cpp:37-38 carries its own file-scope prototypes for both cli_do_ functions, and src/active_objects/ao_telemetry.cpp:642 declares stage_t2_fire_pending_if_any with a function-local extern inside the body of handle_rx_packet. The consequence is not a link failure (C++ mangling makes a signature change fail closed) but a map that lies: the header presents itself as the module's contract while a third of the callable surface is discoverable only by grepping the .cpp, and the two flash-mutating operations -- erase-all-flights and download -- are exactly the entries a reader most needs to find from the contract. It also means those declarations are maintained in three places instead of one.
- Confidence: medium
- Direction: Declare all externally-linked entry points of this TU in its own header and have ao_rcos.cpp and ao_telemetry.cpp include it instead of restating prototypes; make anything that is genuinely internal static.
- Verdict: CONFIRMED -- cli_do_erase_flights (:1091) and cli_do_download_flight (:1213) are absent from the header and re-declared at ao_rcos.cpp:37-38, and stage_t2_fire_pending_if_any (:67) is declared with a function-local extern at ao_telemetry.cpp:642.

### CW-B43-09 -- Three doc comments describe code that does not exist or names things wrongly
- Site: src/cli/rc_os_commands.cpp:735-736 (also :4 and :878)
- Lens: Comments & documentation quality (CERT MSC12-C -- comments describing removed code; CCG NL.2 -- comment and code disagree)
- Claim: Section headers and file prose in this file assert facts that the file contradicts.
- Why: Lines 735-736 are an orphan two-line header ("Build tag constant -- must match main.cpp" / "Build tag from version.h (single source of truth)") sitting above no constant at all -- the constant it introduced was removed, the header was not, and the two lines contradict each other about where the single source of truth lives; a reader looking for the build-tag constant this file supposedly defines finds nothing. Line 4 is a Doxygen file tag naming cli_commands.cpp on a file named rc_os_commands.cpp, so generated documentation and any tooling keyed on that tag names a file that does not exist. Line 878 says "Full boot status -- called by cli_print_boot_status() and 'b' key" on the definition of cli_print_boot_status itself, telling the reader the function is called by itself. Each is individually trivial; together they are the accumulation the comments lens exists to catch, in the file whose comments are the only navigation aid for 1590 lines.
- Confidence: high
- Direction: Delete the orphan build-tag header, fix the file tag to the real filename, and correct line 878 to name the actual caller.
- Verdict: CONFIRMED -- line 4 tags the file cli_commands.cpp, lines 735-736 are an orphan two-line build-tag header above no constant, and line 878 says cli_print_boot_status() is called by cli_print_boot_status().

### CW-B43-10 -- Function definitions sit inside the include block, and two symbols are resolved only transitively
- Site: src/cli/rc_os_commands.cpp:37-146 (consequence visible at :61 and :1486)
- Lens: The spine, block A one-page gestalt (Power of Ten Rule 4's rationale -- a unit a reader can hold and verify) plus CCG ES.5 on where declarations live
- Claim: Roughly 85 lines of constants, a build-flag-gated block and two function definitions are wedged between two halves of the include list, so the file's opening cannot be read as one dependency block.
- Why: Includes run at lines 11-35, then constants and the Stage-T2 block and cmd_radio_config_cycle occupy lines 37-120, then includes resume at lines 121-146 -- a reader scanning for "what does this TU depend on" has to notice that the list restarts after 85 lines of unrelated code, in the file whose opening is the only orientation for 1590 lines. A visible side effect is that symbols are used before their headers appear: rc::rc_log is called at lines 61, 73, 98 and 114 while the include of rocketchip/rc_log.h does not appear until line 145 (it resolves transitively through include/rocketchip/config.h at line 15), and strcmp is called at 1486 with no string header anywhere in the translation unit. I am not filing that half as the defect: the project has deliberately declined include-hygiene enforcement (.clang-tidy carries "SKIPPED: misc-include-cleaner -- too noisy with SDK cross-includes"), and a transitive path that breaks fails loudly at compile time rather than silently. The defect is the split block itself.
- Confidence: medium
- Direction: Move the constants and the two function definitions below a single contiguous include block, and include rc_log.h and the string header directly rather than relying on transitive reach.
- Verdict: RESHAPED -- the split include block is real (includes at 11-35 and 121-146 around 85 lines of constants and two function definitions), but the transitive-include half is a documented project posture (.clang-tidy explicitly skips misc-include-cleaner as too noisy) and would fail loudly at compile time, so only the layout claim survives.

### B44 -- cli: dashboard + debug

#### Coverage

- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_dashboard.cpp -- FAIL -- Read whole (482 lines); spine run on all 10 functions; each operator-facing cell was checked against the telemetry field that actually reaches it, and four disagree.
- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_dashboard.h -- PARTIAL -- Thin API contract surface (helper Kind C): the render/pause/resume promises are clear, but the newest parameter carries an undocumented contract and the doxygen block binds to the wrong declaration.
- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_debug.cpp -- FAIL -- Read whole (230 lines); spine run on all 7 functions; dispatcher control flow and test-mode gating are sound, but the operator-facing menu text duplicates and paraphrases authority that lives elsewhere.
- C:/Users/pow-w/Documents/RC-agent-walk/src/cli/rc_os_debug.h -- FAIL -- Thin API contract surface (helper Kind C): the migration rationale and gating story are well stated, but the documented return contract of the main dispatch entry point contradicts both its body and its only caller.

#### Findings

### CW-B44-01 -- Dashboard "Temp:" cell renders a literal 0 while a transmitted temperature is discarded
- Site: src/cli/rc_os_dashboard.cpp:397-404 (literal at :402)
- Lens: The spine, block B -- spec-noncompliance (a transmitted field silently dropped while the display looks complete); Comments & documentation quality (CCG NL.2 -- the presented claim and the behaviour disagree)
- Claim: The "Temp: %dC" cell is fed the constant static_cast<int>(0) rather than the received TelemetryState::temperature_c, so the ground station always displays "Temp: 0C".
- Why: src/logging/data_convert.cpp:100 populates t.temperature_c from the DPS310 baro temperature, and the field sits inside the 40-byte nav payload (src/telemetry/telemetry_encoder.cpp:58,100), so a real reading arrives at the station on every packet and is thrown away. An operator watching the dashboard on a cold pad reads "Temp: 0C" as a measurement, not as an absent one, and there is no marker (dash, question mark, colour) separating the two. The neighbouring cell has the same shape: d.batt_v at :164 renders battery_mv, which telemetry_state.h:54 documents as "0 = not measured" and which data_convert.cpp:103 hardcodes to 0, so "Batt: 0.00V" is likewise a sentinel rendered as a reading.
- Confidence: high
- Direction: Feed the cell from t.temperature_c; for cells whose source really is unmeasured, render the sentinel distinguishably rather than as a numeric value, so absent data cannot be read as measured data.
- Verdict: CONFIRMED -- the cell at :402 is fed static_cast<int>(0) while TelemetryState::temperature_c is populated at data_convert.cpp:100 and rides in the 40-byte nav payload (telemetry_encoder.cpp:58,100); the battery_mv sentinel documented at telemetry_state.h:54 and hardcoded to 0 at data_convert.cpp:103 renders the same way.

### CW-B44-02 -- "Alt:" and "Baro:" rows print the same variable, giving false two-source corroboration
- Site: src/cli/rc_os_dashboard.cpp:367-374 (source at :155)
- Lens: The spine, block A -- CCG ES.3 / Fowler "Duplicated Code" (one piece of knowledge rendered as two); block B -- spec-noncompliance
- Claim: Both the "Alt:" row and the "Baro:" row are formatted from d.alt_m, which decode_telem_fields computes solely from t.baro_alt_mm, so the two rows always display byte-identical numbers.
- Why: The layout puts "Alt" and "Baro" on separate lines with "Max" and "GPS" fields between them, which reads as two independently sourced altitudes agreeing with each other. During a flight where the baro is the failing sensor (LL Entry 34's turbulence case, or a blocked port), the operator gets two rows corroborating one another from one bad source and no dissent anywhere on screen. Meanwhile the transmitted TelemetryState::alt_mm -- populated from GPS MSL altitude at src/logging/data_convert.cpp:71 -- is never decoded by this file, so the one genuinely independent altitude in the packet is not on the dashboard at all.
- Confidence: high
- Direction: Either drive the "Baro:" row from a genuinely different source (the transmitted alt_mm, relabelled to say which datum it is) or delete the duplicate row; do not display one value under two sensor labels.
- Verdict: CONFIRMED -- decode_telem_fields sets d.alt_m solely from t.baro_alt_mm at :155 and build_frame formats it twice (:371 for the 'Alt:' row and :374 for the 'Baro:' row), while t.alt_mm (GPS MSL, data_convert.cpp:71) appears nowhere in this file.

### CW-B44-03 -- ESKF cell reports green "OK" for a degraded filter and red "FAIL" for an absent one
- Site: src/cli/rc_os_dashboard.cpp:163 (rendered at :403)
- Lens: Comments & documentation quality -- JSF AV 134 (a load-bearing threshold with no stated assumption); The spine, block B -- spec-noncompliance (a boundary silently moved)
- Claim: The expression d.eskf_ok = (rc::health_eskf(t.health) >= rc::kHealthDegraded) collapses the packet's four-level health encoding onto a two-state display in which kHealthDegraded is shown as green "OK", with no comment stating that mapping.
- Why: src/safety/health_monitor.h:28-31 defines an ordered encoding -- kHealthAbsent = 0b00, kHealthFault = 0b01, kHealthDegraded = 0b10 ("Working but reduced quality"), kHealthOk = 0b11. With the comparison sitting at the Degraded boundary, a filter running at reduced quality is indistinguishable on screen from a fully operational one, and the variable name eskf_ok plus the literal string "OK" reinforce the stronger reading. At the other end the same expression maps kHealthAbsent ("Not present / not initialized") onto the same red "FAIL" as kHealthFault, so "ESKF never started" and "ESKF diverged" look identical. This is the operator's only ESKF indicator during the pre-arm window, where the flight director's own checks do distinguish the levels.
- Confidence: high
- Direction: Render the outcomes the encoding already carries (OK / DEGRADED / FAIL, with ABSENT separated from FAIL), or, if a binary indicator is deliberate, state the chosen threshold and its rationale in a comment at the decode site.
- Verdict: REFUTED -- >= kHealthDegraded is the project's established GO boundary, not a silently moved one: rc_os_commands.cpp:1410 states it outright ('OK or degraded = GO'), health_monitor.cpp:749-751 uses it for every go/no-go flag, and flight_director.cpp:257 collapses eskf_healthy at exactly the same boundary, so the dashboard agrees with the flight director rather than contradicting it; the remaining Absent-vs-Fault display granularity is a preference.

### CW-B44-04 -- "Lost:" packet-loss counter pegs to zero for the rest of the session once the sequence span is exceeded
- Site: src/cli/rc_os_dashboard.cpp:179-187
- Lens: The spine, block B -- passes-tests-yet-wrong (correct on short runs, wrong on an unexercised input); CCG ES.3 (knowledge re-derived where an authority already exists)
- Claim: The value expected is masked to 14 bits (& 0x3FFF) while rs->rx_count is a free-running uint32_t, so once rx_count exceeds 16383 the condition expected > rs->rx_count can never hold and "Lost:" displays 0 permanently.
- Why: The masking correctly handles one wrap of the CCSDS 14-bit counter, but it also caps expected at 16383 forever, while RadioAoState::rx_count (src/active_objects/ao_radio.h:36, cumulative uint32_t) keeps climbing. At the whitelisted nav rates of 5 and 10 Hz (include/rocketchip/radio_config_table.h:43-52) that boundary is reached in roughly 55 and 27 minutes -- well inside a pad-and-launch session -- after which a link dropping packets still shows "Lost: 0". Nothing in the code or a comment states the bound. Separately, rc::RfManagerState::packets_missed (src/active_objects/ao_rf_manager.h:56) already tracks cumulative drops via inter-arrival gap detection and is reachable from this file, which calls AO_RfManager_get_state() at :246 -- so this hand-rolled figure is a second, weaker implementation of an existing counter.
- Confidence: high
- Direction: Display RfManagerState::packets_missed instead of re-deriving loss here; if a locally derived figure is still wanted, carry the wrap in a 32-bit accumulator rather than masking the difference, and state the counter's span in a comment.
- Verdict: CONFIRMED -- expected is masked to 14 bits at :185 while RadioAoState::rx_count (ao_radio.h:36) is a free-running uint32_t, so the > test at :186 can never hold once rx_count passes 16383 (about 27 min at the 10 Hz whitelist entries, radio_config_table.h:46-48), and RfManagerState::packets_missed (ao_rf_manager.h:56) already tracks the same quantity through a state pointer this file fetches at :246.

### CW-B44-05 -- dev_debug_menu_dispatch's documented return contract contradicts both its body and its only caller
- Site: src/cli/rc_os_debug.h:27-29 (body at src/cli/rc_os_debug.cpp:195-201)
- Lens: Comments & documentation quality -- CCG NL.2 ("If the comment and the code disagree, both are likely to be wrong") / JSF AV 134 (the function preamble states the contract); contract-surface helper Kind C
- Claim: The header states "Returns true if the key was handled", but the body returns true from its default case for keys it does not handle and returns false only for the menu-exit keys, and the caller reads false as "leave the debug menu".
- Why: src/cli/rc_os.cpp:509 is the only call site -- else if (!dev_debug_menu_dispatch(c)) { g_menu = RC_OS_MENU_MAIN; } -- and it then sets handled = true unconditionally. So the real contract is "return false to pop the submenu", a different predicate from the documented one, and it disagrees in both directions: pressing 'z', 'Z' or ESC is fully handled yet returns false, while pressing an unmapped key is not handled yet returns true. A second caller written against the header -- the natural next step for the RC_OS rework tracked on the whiteboard -- would treat 'z' as an unrecognised key and never leave the menu, while treating genuine typos as accepted.
- Confidence: high
- Direction: Reword the header to state what the value actually means (stay in versus exit the submenu), or return an explicit two-value result so "handled" and "stay resident" are not conflated in one bool.
- Verdict: CONFIRMED -- the body returns true from its default case (:198-201) and false only for z/Z/ESC (:195-197), and the sole caller at rc_os.cpp:509 reads false as 'leave the submenu' then sets handled=true regardless, so the documented predicate is wrong in both directions.

### CW-B44-06 -- The radio-config whitelist is paraphrased twice inside rc_os_debug.cpp, and its size is hardcoded as case labels
- Site: src/cli/rc_os_debug.cpp:151, :160-162, :192-193
- Lens: Comments & documentation quality -- CCG NL.3 / JSF AV 131 (point to the spec, do not paraphrase it); CCG ES.3 (the same knowledge in more than one place)
- Claim: The contents of rc::kRadioConfigTable are transcribed by hand twice in this file -- once in the comment at :160-162 and once in the operator-facing help text at :192-193 -- and the digit range is hardcoded as case '0' through case '5' at :151 rather than derived from rc::kRadioConfigTableSize.
- Why: include/rocketchip/radio_config_table.h:11-12 declares itself authoritative for exactly this reason ("Code is authoritative; docs/RADIO_TELEMETRY_STATUS.md references THIS header (not vice versa) -- prevents doc-vs-code drift"), and the two transcriptions here are the drift it warns about, one of them shown to an operator who is about to reconfigure a live radio link. The table currently holds six entries, so the '0' through '5' labels happen to match today; adding a seventh entry, which that header invites by targeting up to ten, leaves index 6 unreachable from the menu with no compile error while the help text keeps naming the old mapping. The reordering case is worse: the table's own comment says its order is meant to track a future channel-find sweep order, so a reorder silently re-points every digit the help text advertises. The runtime bound check at :164 protects memory safety but not the advertised mapping.
- Confidence: high
- Direction: Generate the help line by iterating kRadioConfigTable up to kRadioConfigTableSize and drop the hand-written comment paraphrase, leaving a one-line pointer to the header; range-check the digit against the table size instead of enumerating case labels.
- Verdict: CONFIRMED -- the table's six entries are transcribed by hand at :160-162 and again in operator-facing help at :192-193, and the digit range is enumerated as case labels at :151 rather than derived from rc::kRadioConfigTableSize, against the header's own anti-drift claim at radio_config_table.h:11-12.

### CW-B44-07 -- Both debug-menu banners advertise a "Replay" command that was retired, and the two banners have drifted apart
- Site: src/cli/rc_os_debug.cpp:42-45 and :188-194 (retired handler at :128-136)
- Lens: Comments & documentation quality -- CERT MSC12-C (documentation describing code that no longer runs); CCG ES.3 (the same menu vocabulary maintained in two places)
- Claim: The entry banner offers "r-Replay" and the help banner offers "r-Replay inject", but the 'r' handler only prints a retirement notice, and the two banners have separately drifted so neither lists the same key set.
- Why: R-25-exec steps 5 and 6 deleted the vehicle and station replay injectors (documented at :16-18, :129-134 and :221-229), yet both operator-facing menus still present replay as an available command; an operator following the menu presses 'r' expecting an injection facility and gets a redirect to a host script. The drift between the two copies is already visible: the entry banner lists "h-Help" and the help banner does not, the help banner lists the 0 through 5 radio-config keys and the entry banner does not, and the same command is named "Replay" in one and "Replay inject" in the other. Because the key set is maintained as two independent string literals, every future key addition has to be made twice and silently need not be.
- Confidence: high
- Direction: Print one shared banner function from both entry points and remove the replay line from it, leaving the retirement redirect on the key itself for operators who remember the old binding.
- Verdict: CONFIRMED -- both banners advertise replay (:44 'r-Replay', :191 'r-Replay inject') while the handler at :128-136 only prints a retirement redirect, and the two literal key lists have already diverged on h-Help and on the 0-5 radio-config keys.

### CW-B44-08 -- Dashboard render doxygen omits the rx parameter and binds to the forward declaration rather than the function
- Site: src/cli/rc_os_dashboard.h:23-41
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions and limitations documented in the function preamble); contract-surface helper Kind C (promise / forbidden / signature-matches-prose)
- Claim: The doxygen block documents five of ansi_dashboard_render's six parameters and omits rx, and it is separated from the function by the forward declaration struct RxTelemSnapshot; at :36, so the block binds to that declaration instead of to the function it describes.
- Why: rx is the parameter whose contract is least guessable: it defaults to nullptr, and the implementation at :437 treats both a null rx and rx->echo_bw_khz == 0 as "vehicle config unknown", which silently degrades the Radio row to "Vehicle: ?" rather than failing. A caller that omits the argument gets a dashboard that looks complete and quietly stops reporting station-versus-vehicle radio-config mismatch, the very condition the row exists to surface (:443-446). The misplacement also means generated documentation attributes the whole block, parameter list included, to an empty struct declaration. Related comment drift in the pair: the implementation's @file tag at rc_os_dashboard.cpp:4 still names ansi_dashboard.cpp, a file that does not exist in the tree.
- Confidence: medium
- Direction: Move the forward declaration above the doc block, add an @param rx line stating the null and zero-echo semantics and what the caller loses by omitting it, and correct the stale @file tag in the implementation.
- Verdict: CONFIRMED -- the doc block at :23-34 documents five parameters and omits rx, and the forward declaration struct RxTelemSnapshot; at :36 sits between it and the function at :38-41; rx defaults to nullptr and a null rx silently degrades the config row via :437 and :443-446, and the implementation's @file tag still names ansi_dashboard.cpp.

### CW-B44-09 -- rssi_bar accepts a buffer-size parameter and explicitly discards it
- Site: src/cli/rc_os_dashboard.cpp:95-108 (call at :206)
- Lens: The spine, block B -- Power of Ten Rule 7 ("parameter validity must be checked inside each function")
- Claim: rssi_bar(char* buf, int max, int16_t rssi) advertises a bounded write in its signature, then discards the bound with a void cast at :106 and writes a fixed 13 bytes; the length it computes and returns is dropped by its only caller.
- Why: The body writes '[' plus ten cells plus ']' plus a NUL -- 13 bytes -- unconditionally, then explicitly discards max at :106, so the parameter documents a check the function does not perform. There is no overflow and no reachable defect today: rssi_bar is file-static with exactly one call site, which passes sizeof(d.bar) on a 16-byte array (:141, call at :206), so the overflow scenario needs a caller that does not exist. What is verifiable now is that both halves of the interface are dead -- the bound going in is void-cast away, and the length coming out is dropped at the call site without even a (void) marker. On a no-MMU, no-sanitizer target a signature that promises a bound it does not honour is worth naming as a dead-parameter defect; it is not a live bounds hazard.
- Confidence: medium
- Direction: Either honour the bound (return early or truncate when max is smaller than the fixed frame) or delete the parameter and the unused return so the signature stops promising a check the body does not perform.
- Verdict: RESHAPED -- the dead bound and the dropped return are verifiable, but the overflow needs a caller that does not exist: rssi_bar is file-static with one call site passing sizeof(d.bar) (16) for a fixed 13-byte write, so the true defect is a signature promising a check the body does not perform, not a live bounds hazard.

---

## Cross-cutting lanes (whole-system, not visible file-by-file)


### X1 -- cross-core and ISR shared-data ownership

Lane scope: every object that crosses the Core 0 / Core 1 boundary or an ISR / consumer
boundary in src/ and include/. The enumeration below was built from declarations, not from the
itinerary rows: a whole-tree grep of volatile, std::atomic, spin_lock, multicore_*,
save_and_disable_interrupts, irq_set_*, and every extern non-const global in a header, then
each candidate was traced to its writers and readers to answer the field manual's three
questions (who owns it, who mutates it, what barrier protects it).

#### Coverage

Enumerated cross-boundary objects (the working inventory):

- Seqlock: g_sensorSeqlock (shared_sensor_data_t, 156 B) -- one writer per role, many Core 0 readers.
- Cross-core atomics: g_startSensorPhase, g_sensorPhaseDone, g_calReloadPending, g_core1PauseI2C, g_core1I2CPaused, g_core1LockoutReady, g_bestGpsValid, rc_os_mag_cal_active, g_spi_error_count.
- Plain (non-atomic) cross-core objects: g_imu, g_gpsTransport plus the three GPS function pointers, g_imuInitialized, g_baroInitialized, g_baroContinuous, g_gpsInitialized, g_sensorPhaseActive, g_bestGpsFix, g_eskf, g_eskfInitialized, the calibration_manager module statics, the ws2812_status driver state.
- ISR-shared objects: g_rxBuf / g_rxHead / g_rxTail / g_rxOverflow (UART0 RX ISR), g_buffer / g_count (GPIO edge ISR), g_phaseObservablePair and g_crash_record (fault handler), g_inFaultHandler.
- Core 0-only shared objects confirmed as such and dispositioned by reference: the rc_log ring (g_ring / g_head / g_tail), the PSRAM RingBuffer, the test_mode / fault_inject / station_fault_inject volatiles, and all QP/C AO state and posted static events.
- Cross-core hardware: I2C1 (Core 1 owns during the sensor phase), UART0 and its NVIC line, PIO2 and the WS2812 state machine, the ADC (die temp), flash via multicore_lockout.

Per-file verdicts for every file read in this lane:

- include/rocketchip/shared_state.h -- FAIL -- read whole; its ownership prose and its mix of atomics and plain bools is the contract CW-X1-05 judges.
- src/shared_state.cpp -- PARTIAL -- read whole; definitions match the header but inherit its type mismatch.
- include/rocketchip/sensor_seqlock.h -- PASS -- read whole; single-writer seqlock with explicit __dmb() on both sides, retry-bounded reader, struct size static_asserted; correct, and the model the rest of the tree should follow.
- src/core1/sensor_core1.cpp -- FAIL -- read whole; the Core 1 side of every finding except CW-X1-07 and CW-X1-08.
- src/core1/sensor_core1.h -- FAIL -- read whole; declares g_bestGpsFix, g_eskf and g_eskfInitialized as cross-core surface (CW-X1-06).
- src/drivers/gps_uart.cpp -- FAIL -- read whole; ISR/consumer ring and reinit path (CW-X1-01, CW-X1-04).
- src/drivers/gps_uart.h -- PASS -- read whole; declaration only, no ownership claims.
- src/main.cpp -- PARTIAL -- read whole; boot ordering, Core 1 launch and the release-store publication of init state are correct, but it hosts the cal-hook binding of CW-X1-08 and the boot baro auto-zero of CW-X1-03.
- src/station/station_idle_tick.cpp -- PASS -- read whole; station only, same-core writer and reader, gated by kRadioModeRx so it can never become a second seqlock writer on the vehicle.
- src/safety/core1_i2c_pause.cpp and .h -- PASS -- read whole; the pause/ack/resume handshake is acquire/release correct and bounded, and Core 0 also writing the ack flag at resume is a documented deliberate choice whose only failure mode is a spurious 100 ms ack timeout, not a lost pause.
- src/calibration/calibration_manager.cpp -- FAIL -- state machine, accumulators and the feed/start/cancel API read (CW-X1-03).
- src/calibration/cal_hooks.cpp -- FAIL -- read whole (CW-X1-08); cal_read_mag correctly uses the seqlock instead of touching I2C.
- src/drivers/ws2812_status.cpp -- FAIL -- driver state and send_pixel read (CW-X1-07).
- src/active_objects/ao_led_engine.cpp -- PARTIAL -- LED-driving and Core 1 vitality sections read; Core 0 only otherwise.
- src/fusion/eskf_runner.cpp -- PARTIAL -- ownership of g_eskf and g_eskfInitialized and the seqlock read path traced (CW-X1-06).
- src/log/rc_log.cpp -- PASS -- ring and drain read whole; producer and consumer are both Core 0 cooperative and the "never from Core 1, never from ISR" contract holds, verified by grepping every rc_log call site reachable from core1_entry (none).
- src/logging/ring_buffer.cpp and .h -- PASS -- single-writer PSRAM ring; all writers and readers are Core 0 (AO_Logger, CLI, flash_flush).
- src/logging/psram_init.cpp -- PASS -- the one save_and_disable_interrupts / restore_interrupts region is paired inside one function and runs on Core 0 before multicore_launch_core1.
- src/safety/fault_protection.cpp -- PASS -- fault path read whole; the phase observable is read with a complement check and g_crash_record is written magic-last, so single-fault ordering is sound.
- src/flight_director/flight_director.cpp -- PASS -- g_phaseObservablePair is a single aligned word with a self-checking complement, written by Core 0 and read from fault context; a correct use of volatile for a handler-observable.
- src/safety/test_mode.cpp -- PASS -- volatile is for probe and SRAM persistence across reset, not cross-core; all callers are Core 0.
- src/safety/fault_inject.cpp and station_fault_inject.cpp -- PASS -- probe-invoked, Core 0, gated by test_mode_active().
- src/safety/pyro_edge_logger.cpp -- PASS -- the GPIO edge ISR and its reader are both on Core 0, and the count is published after the payload write.
- src/safety/flight_in_progress.cpp -- PASS -- Core 0 only; the dsb around the sentinel is for reset persistence, not cross-core ordering.
- src/safety/health_monitor.cpp -- PARTIAL -- Core 1 vitality check and the sensor / GO-NO-GO consumers of the shared flags read (CW-X1-02, CW-X1-05).
- src/cli/rc_os_commands.cpp -- PARTIAL -- every shared-state consumer read (g_bestGpsFix, the init flags, the two flash paths that pause Core 1); the CLI's own state is Core 0.
- src/cli/rc_os.cpp and rc_os.h -- PASS -- rc_os_mag_cal_active is a proper std::atomic<bool> with a Core 0 writer and a Core 1 acquire-reader.
- src/drivers/spi_bus.cpp and .h -- PASS -- g_spi_error_count is atomic and the SPI bus is touched only from Core 0 AOs.
- src/drivers/mcu_temp.cpp -- PASS -- the ADC has exactly one consumer per role and the driver statics stay on the consuming core.
- src/drivers/i2c_bus.cpp -- PARTIAL -- ownership traced (Core 1 owns during the sensor phase, Core 0 only inside a pause window); i2c_bus_scan is correctly gated by rc_os_i2c_scan_allowed.
- src/active_objects/ao_flight_director.cpp, ao_logger.cpp, ao_notify.cpp, ao_radio.cpp, ao_health_monitor.cpp, ao_rcos.cpp, ao_rf_manager.cpp -- PASS -- posted events all use static storage per LL 35, and every AO handler, the QV idle bridge and all AO-visible state are Core 0, so those statics are serialized by the cooperative scheduler.
- src/diag/diag_stats.cpp -- PASS -- Core 0 reader; the NVIC ISPR access is genuine MMIO and correctly volatile-qualified.

Directories swept for further cross-boundary declarations with no additional candidates found:
src/math/, src/notify/, src/telemetry/, src/flight_director/ (beyond the observable above),
src/fusion/ (beyond eskf_runner), and the remaining include/rocketchip/ headers, which are
layout, enum and contract only. lib/, EXTERNAL/ and the Pico SDK were out of scope.

Reviewed and deliberately not filed: g_bestGpsFix is written field by field by Core 1 and read
field by field by Core 0 with only the first valid transition release-published, but
src/core1/sensor_core1.h:31-33 already states and disposes of that ("Atomic flag guards
visibility (not struct consistency -- benign for diagnostics, not flight-critical)"), and the
navigation consumers of it (rc_os_commands.cpp:1311-1374) are kRadioModeRx-gated to the
station, where the same core both writes and reads it. Worth noting only that on the vehicle
two readers disagree on the memory order used (rc_os_commands.cpp:346 relaxed vs :709 acquire).

#### Findings

### CW-X1-01 -- Core 1 tears down and re-enables a UART interrupt that Core 0 owns
- Site: src/core1/sensor_core1.cpp:275 with src/drivers/gps_uart.cpp:376-379 and src/drivers/gps_uart.cpp:448-478
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 8 (single owning task, ownership passed explicitly), JPL-C Rule 6 (no task directly executes code or accesses data belonging to another task); docs/MULTICORE_RULES.md "Cross-Core Communication"
- Claim: gps_uart_reinit() performs per-core NVIC operations on the UART0 interrupt line but is called from Core 1, while the handler was registered and enabled on Core 0, so the final irq_set_enabled(..., true) at gps_uart.cpp:477 enables the interrupt on Core 1's NVIC without ever clearing Core 0's, leaving the RX ISR live on both cores.
- Why: gps_uart_init() runs on Core 0 (main.cpp:130 via init_sensors) and its own comment at gps_uart.cpp:376 records the affinity -- "IRQ registers on Core 0 (the core running this init). Drained by Core 1." The SDK's irq_set_enabled acts on the executing core's NVIC only. The teardown at line 453 is therefore a no-op on the owning core (the peripheral-level mask at line 454 is what actually stops the ISR, so the teardown survives by accident), but the re-arm at line 477 lands on Core 1 and line 478 re-asserts the UART IRQ line for both. From that moment gps_uart_rx_isr() runs concurrently on Core 0 and Core 1 against a ring the file documents as "SPSC (single-producer single-consumer)" (gps_uart.cpp:132-133): two producers perform an unsynchronized read-modify-write of g_rxHead (lines 194-202), so bytes are lost or duplicated and head and tail can cross. Core 1 also begins taking a roughly 240 Hz interrupt inside its 1 kHz sensor loop, a cost the timing note at gps_uart.cpp:187-188 budgets for Core 0 only. The trigger is ordinary: 10 s without a valid NMEA parse on a UART GPS with the vehicle not moving (sensor_core1.cpp:253-279).
- Confidence: high
- Direction: keep NVIC ownership on the core that registered the handler. Either move the reinit into a Core 0 context (a request flag Core 1 sets and an AO or the idle bridge services), or reduce gps_uart_reinit() to peripheral-level operations only (uart_set_irqs_enabled, uart_deinit / uart_init) and leave irq_set_enabled exclusively in gps_uart_init() on Core 0.
- Verdict: REFUTED -- the Core 1 call site is unreachable: `gps_uart_update()` (the only function ever bound to `g_gpsFnUpdate` for UART transport, main.cpp:103) returns false only when `!g_initialized`, and `g_initialized` is cleared only inside `gps_uart_reinit()` itself, so `parsed` is always true and `core1_gps_staleness_check()` always returns at sensor_core1.cpp:255 before reaching line 275; `gps_uart_reinit()` has no other caller in the tree, making the cross-core NVIC hazard latent (dead code) rather than the live, ordinarily-triggered defect claimed.
- **Reconciled 2026-08-20:** duplicate of **CW-B09-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.
- **Reconciled 2026-08-20:** this row's verdict rests on the premise that `core1_gps_staleness_check()`'s recovery branch is dead code. **That premise is false** — see Reconciliation §1. The premise is withdrawn; the batch row stands.

### CW-X1-02 -- Core 1's GPS recovery blocks the seqlock publisher past Core 0's stall detectors
- Site: src/core1/sensor_core1.cpp:251-279 and src/core1/sensor_core1.cpp:445-463, against src/safety/health_monitor.cpp:346 and src/active_objects/ao_led_engine.cpp:54-55
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 7 (task synchronization shall not be performed through task delays) and the field manual's blocking-in-cooperative-scheduler / ISR-timing criterion; CCG CP.3 (minimize the shared writable surface, here the liveness signal itself)
- Claim: core1_gps_staleness_check() runs a blocking UART reacquisition inside the Core 1 loop body, between two seqlock_write() publishes, for longer than the interval Core 0 uses to declare Core 1 dead.
- Why: the reinit path calls acquire_at_target_baud(), which is up to two try_baud() attempts each bounded by detect_gps_presence()'s kInitTimeoutUs = 2 s (gps_uart.cpp:68 and 311-334), plus kGpsBaudNegotiateDelayMs = 250 ms -- about 2 s typically and up to roughly 4.25 s worst case. local_data.core1_loop_count is published only at sensor_core1.cpp:453-454, after core1_sensor_pass() returns, so the counter freezes for that whole window. Core 0 reads that same counter as its Core 1 liveness signal and latches a stall at 6 ticks of 10 Hz = 600 ms (health_monitor.cpp:346, check_core1_vitality) and at 17 ticks of 33 Hz, about 515 ms (ao_led_engine.cpp:54-55). A single GPS dropout therefore deterministically trips the Core 1 stall fault -- clearing kHealthCore1Ok, driving the magenta stall fault intent through notify_resolver.h:57-59, and polluting the pre-arm health byte -- while the IMU sample stream the ESKF consumes stops for seconds. The comment at sensor_core1.cpp:251-252 states "Blocks up to 2s", but nothing at that site or in either detector connects the two numbers.
- Confidence: high
- Direction: make the reacquisition non-blocking (a start/poll split like the radio driver's, per LL Entry 32), or run it off Core 1 entirely. If it must stay, it has to keep publishing core1_loop_count while it waits, and the two stall thresholds need to be stated as a budget the recovery path is checked against.
- Verdict: REFUTED -- the blocking reacquisition never runs on Core 1 for the same reason as CW-X1-01 (`gps_uart_update()` cannot report a failed parse, so the 10 s staleness branch at sensor_core1.cpp:259-279 is dead), so no GPS dropout can freeze `core1_loop_count` past the 600 ms / 515 ms stall detectors; the threshold-vs-budget arithmetic is correct but describes a path that is never entered.
- **Reconciled 2026-08-20:** this row's verdict rests on the premise that `core1_gps_staleness_check()`'s recovery branch is dead code. **That premise is false** — see Reconciliation §1. The premise is withdrawn; the batch row stands.

### CW-X1-03 -- The calibration state machine is shared between both cores with no synchronization at all
- Site: src/calibration/calibration_manager.cpp:92-107 and 370-403, with src/core1/sensor_core1.cpp:172-182 and 206-209, src/main.cpp:366-376, and src/active_objects/ao_rcos.cpp:382-390, 423-434, 441-463
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 8 (single owner, only the owner modifies) and CCG CP.8 (no atomicity and no ordering without a barrier); the field manual's three-question test
- Claim: g_calState, g_sampleAcc and g_calibration are plain non-atomic statics that Core 0 writes (start, cancel, reset) while Core 1 also writes them from the sensor loop (feed and completion), with no atomic, no barrier and no pause on either side.
- Why: Core 0 starts a calibration by memsetting the accumulator and then storing the new state -- reset_accumulator(kBaroCalSamples) at calibration_manager.cpp:375 followed by g_calState = CAL_STATE_BARO_SAMPLING at :376 -- two plain stores with nothing sequencing them for the other core. Core 1 polls that same plain variable every 32nd loop pass (calibration_manager_get_state() == CAL_STATE_BARO_SAMPLING, sensor_core1.cpp:206) and on seeing it writes into g_sampleAcc and can drive the machine to CAL_STATE_COMPLETE itself (calibration_manager.cpp:395-401). The live path is boot, not a CLI corner: init_baro_auto_zero() (main.cpp:366-376) runs after init_core1_role() has already released Core 1 into the sensor loop, calls calibration_start_baro(), then spins on while (calibration_is_active()), a plain read of the state Core 1 is mutating. If Core 1 observes the state store before the accumulator reset lands, it feeds a stale accumulator whose count already satisfies target_count and the ground pressure reference is computed from a truncated sample set, silently, with g_calState going straight to COMPLETE so Core 0's wait returns immediately -- and that reference is what the barometric AGL used for main-deploy altitude is measured against. calibration_cancel() (:1033) and calibration_reset_state() (:1037) can likewise land mid-feed from the other core.
- Confidence: high
- Direction: give the module one owner. Either have Core 0 pause Core 1 (rc::core1_i2c_pause / core1_i2c_resume, already used for the flash paths) around every start, cancel and reset, or publish the state through a release/acquire atomic with the accumulator reset ordered before it; then state the ownership rule in calibration_manager.h the way shared_state.h does.
- Verdict: CONFIRMED -- `g_calState`/`g_sampleAcc`/`g_calibration` are plain statics (calibration_manager.cpp:92-106) with no atomic, barrier or pause on either side; `init_baro_auto_zero()` (main.cpp:366-376) genuinely runs after `init_core1_role()` has released Core 1 into the sensor loop (main.cpp:389-398), Core 1 polls the plain state and can drive it to COMPLETE itself (sensor_core1.cpp:206-209 -> calibration_manager.cpp:382-402), and `calibration_manager.h` states no ownership rule -- and because `g_sampleAcc` is zero-initialized at boot, `target_count == 0` makes the truncated-reference outcome reachable on the very first calibration.
- **Reconciled 2026-08-20:** duplicate of **CW-B18-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X1-04 -- The UART RX ring is a cross-core channel documented as needing no barrier
- Site: src/drivers/gps_uart.cpp:142-149 with 163-166, 190-205 and 388-414
- Lens: volatile / control-flow discipline and Concurrency -- JSF AV Rule 205 (volatile shall not be used unless directly interfacing with hardware), CCG CP.8 ("volatile does not provide atomicity, does not synchronize between threads, and does not prevent instruction reordering"); docs/MULTICORE_RULES.md "Never Use Plain volatile for Cross-Core Sharing"
- Claim: the ring is written by an ISR on Core 0 and drained on Core 1 using only volatile indices over a non-volatile payload buffer, and the file asserts in comments that this is sufficient.
- Why: the producer writes the payload at gps_uart.cpp:201 (g_rxBuf[head] = byte, a plain static uint8_t array declared at :163) and then publishes the index at :202 (g_rxHead = next); the consumer on Core 1 snapshots g_rxHead at :395 and then reads the payload at :406-412. Nothing orders the payload store before the index store, in the compiler or in the machine -- volatile on the index constrains neither the placement of the non-volatile array store nor the completion order of the Cortex-M33 write buffer as observed by the other core. The comment at :146-149 states the opposite as fact ("RP2350 SRAM is cache-coherent across cores ... volatile is sufficient, no __dmb() needed"), which is exactly the confident-justification pattern the field manual's confabulation criterion says to distrust: absence of a data cache is not absence of store reordering. The project's own protected rule doc says plainly that volatile "does NOT issue ARM hardware memory barriers. Data written on one core may not be visible on the other", and the project's own seqlock (include/rocketchip/sensor_seqlock.h:10-12 and 124-138) spends two explicit __dmb() calls on the identical publish-payload-then-index shape and documents why release ordering on the index alone is not enough. Two sites in the same tree give opposite answers to the same question. The failure mode is a parser fed one stale byte at the head of a burst -- an NMEA sentence that fails checksum, which presents as GPS staleness, which is the trigger for CW-X1-01 and CW-X1-02.
- Confidence: high
- Direction: make the indices std::atomic<uint32_t> with release on the producer's index store and acquire on the consumer's index load (or an explicit __dmb() on each side, matching the seqlock), and replace the comment block with the ordering argument rather than an assertion that none is needed.
- Verdict: CONFIRMED -- the producer is the Core 0 UART0 ISR (gps_uart.cpp:189-205) and the consumer runs on Core 1 via `g_gpsFnUpdate` -> `gps_uart_update()` -> `gps_uart_drain()` (gps_uart.cpp:388-414), the payload array `g_rxBuf` is non-volatile (gps_uart.cpp:163) while only the indices are volatile, no `__dmb()` appears anywhere in the file, and the in-file comment at :146-149 asserting sufficiency directly contradicts docs/MULTICORE_RULES.md and the tree's own seqlock, which spends two explicit barriers on the identical publish shape; no mechanical gate in .clang-tidy covers volatile-as-synchronization.
- **Reconciled 2026-08-20:** duplicate of **CW-B09-04**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X1-05 -- Sensor-liveness flags are plain bools mutated by Core 1 and consumed by Core 0 safety logic
- Site: include/rocketchip/shared_state.h:32-37 and 67-75, with src/core1/sensor_core1.cpp:221 and :276, against src/safety/health_monitor.cpp:154, 178, 256, 763
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 8 (single owning task, ownership passed explicitly) and CCG CP.8; also Comments and documentation (CCG NL.2 -- the header's ownership statement and the code disagree)
- Claim: g_baroInitialized and g_gpsInitialized are plain bools that Core 1 clears at runtime to mean "this sensor is dead", while the header declares Core 0 the owner of initialization state and Core 0 reads them as safety inputs, with no atomic and no barrier on a channel whose six siblings in the same header are std::atomic<bool>.
- Why: shared_state.h:9-13 states the model -- "Core 0 owns initialization. Core 1 reads most sensor flags" -- and then annotates two of them "Core 1 reads/writes" without changing their type or naming a barrier, so the header's own ownership claim is contradicted three lines below it. Core 1 writes g_baroInitialized = false after three failed re-init attempts (sensor_core1.cpp:216-222) and g_gpsInitialized = false when the UART reacquisition fails (sensor_core1.cpp:275-277). Core 0 reads exactly those flags to build the health byte (health_monitor.cpp:154, 178, 256) and the pre-arm GO/NO-GO input gng->gps_has_lock (:763), and again in the preflight report (rc_os_commands.cpp:815-833) and in AO_Notify (ao_notify.cpp:130). Because the object is neither atomic nor barrier-published, the transition has no defined visibility point and two reads inside one Core 0 evaluation can straddle the flip, so the health byte and the GO/NO-GO line can disagree about the same sensor in the same tick -- and a reader cannot tell from the declaration that the value can change under it at all. The correct primitive is already in the same file for the six handshake flags.
- Confidence: medium
- Direction: promote the flags Core 1 can clear to std::atomic<bool> with release on Core 1's clear and acquire on Core 0's reads, or leave them Core 0-owned and have Core 1 report sensor death through the seqlock snapshot it already publishes; then correct the ownership sentence at the top of shared_state.h to match.
- Verdict: RESHAPED -- narrowed to `g_baroInitialized` alone: Core 1 clears it at sensor_core1.cpp:221 (reachable: 50 consecutive baro read failures x 3 re-init attempts) while Core 0 reads it as a health input at health_monitor.cpp:178 and rc_os_commands.cpp:617/621/817/832, with no atomic and no barrier, and shared_state.h:35 annotates it "Core 1 reads/writes" while shared_state.h:9-13 declares Core 0 the owner of initialization. The `g_gpsInitialized` half is refuted: its only Core 1 write (sensor_core1.cpp:276) sits in the dead staleness branch shown unreachable under CW-X1-01, so on the vehicle that flag is written by Core 0 only and the GO/NO-GO input at health_monitor.cpp:763 cannot straddle a Core 1 flip.
- **Reconciled 2026-08-20:** duplicate of **CW-B01-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.
- **Reconciled 2026-08-20:** this row's verdict rests on the premise that `core1_gps_staleness_check()`'s recovery branch is dead code. **That premise is false** — see Reconciliation §1. The premise is withdrawn; the batch row stands.

### CW-X1-06 -- Core 1's mid-flight safety gate reads Core 0's filter state unsynchronized, and fails open
- Site: src/core1/sensor_core1.cpp:268-273 with src/core1/sensor_core1.h:61-64 and src/fusion/eskf_runner.cpp:71-74, 196-201
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 8, CCG CP.2 (avoid data races) and CP.8
- Claim: the "probably flying" guard that is supposed to prevent a mid-flight UART reacquisition is computed on Core 1 from g_eskfInitialized (a plain bool) and g_eskf.v (three floats inside a large object) that Core 0 mutates continuously, with no atomic, barrier or snapshot.
- Why: g_eskf is owned by the Core 0 fusion path -- eskf_runner_tick() calls g_eskf.predict(...) from the QV idle bridge (eskf_runner.cpp:196, main.cpp:448) -- and eskf_runner.cpp:71 records the sharing as deliberate: "Non-static: Core 1 reads g_eskf.v for GPS staleness heuristic". Core 1 evaluates g_eskfInitialized && g_eskf.v.norm() > kGpsFlyingVelocityThreshold (sensor_core1.cpp:269-270) as the only thing standing between a stale GPS and the multi-second blocking reinit of CW-X1-01 and CW-X1-02. The boolean half of the gate is set false by Core 0 the moment the filter is declared unhealthy (eskf_runner.cpp:199-200), so the guard opens precisely in the anomaly it exists to protect against: an unhealthy ESKF during descent lets Core 1 stall its own 1 kHz loop for seconds and reconfigure the UART from the wrong core. The vector half is a torn read of another core's working state, and the codebase already owns the primitive for publishing a consistent cross-core snapshot without using it here.
- Confidence: medium
- Direction: publish the small piece Core 1 actually needs (a flight-active boolean, or the speed) through the existing seqlock or an atomic Core 0 writes with release, and re-derive the gate so an unhealthy filter is treated as "possibly flying" rather than "not flying".
- Verdict: REFUTED -- the `probably_flying` gate at sensor_core1.cpp:268-273 is inside the same dead branch as CW-X1-01: `parsed` is always true for UART transport, so the guard is never evaluated and Core 1 never reads `g_eskfInitialized` or `g_eskf.v`; no unsynchronized cross-core read of the filter state actually occurs, and the fail-open concern has no live path to open.
- **Reconciled 2026-08-20:** duplicate of **CW-B36-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.
- **Reconciled 2026-08-20:** this row's verdict rests on the premise that `core1_gps_staleness_check()`'s recovery branch is dead code. **That premise is false** — see Reconciliation §1. The premise is withdrawn; the batch row stands.

### CW-X1-07 -- The NeoPixel driver is written from both cores on the pause-exit path
- Site: src/core1/sensor_core1.cpp:356-367 with src/drivers/ws2812_status.cpp:51-97 and 106-126, and src/active_objects/ao_led_engine.cpp:102, 119, 265
- Lens: Concurrency and shared-data ownership -- CCG CP.3 (minimize the shared writable surface) and JPL-C Rule 8 (single owner)
- Claim: ws2812_status's file-static g_state (mode, colors, animation timing, pixel buffer) and its PIO state-machine FIFO are mutated from Core 1 inside core1_check_pause_and_reload() while Core 0's AO_LedEngine owns the same driver, with no ownership rule and no guard.
- Why: on pause entry (sensor_core1.cpp:358-359) the write is in fact safe by construction, because Core 0 is blocked inside core1_i2c_pause() within a single AO run-to-completion step so no other Core 0 AO can be dispatched. The exit at :363-365 is not: Core 0 clears the pause flag in core1_i2c_resume() and returns from its handler, after which QV is free to dispatch AO_LedEngine at 33 Hz, and only then does Core 1 execute ws2812_set_mode(...) and ws2812_update(). Both cores then write the same non-atomic struct (ws2812_status.cpp:51-97) and push words into the same PIO TX FIFO through pio_sm_put_blocking (:124), so one core's mode or colour update can be half-overwritten by the other and the two pixel streams interleave at word granularity. The consequence is bounded -- a wrong or corrupted status colour, which on this vehicle is the operator-facing fault indication that CW-X1-02 also feeds -- but no file states who owns the LED, and docs/MULTICORE_RULES.md grants only that any core may touch a PIO block, not that two may do so concurrently.
- Confidence: medium
- Direction: give the LED a single owner on Core 0 -- have Core 1 signal its pause state through a flag Core 0 already has (g_core1I2CPaused) and let AO_LedEngine render it -- rather than having Core 1 drive the driver directly.
- Verdict: CONFIRMED -- `core1_i2c_resume()` (core1_i2c_pause.cpp:34-43) clears both flags and returns immediately with no wait for Core 1, so after Core 0's handler returns QV is free to dispatch AO_LedEngine (ao_led_engine.cpp:102/119/265) while Core 1 is still executing `ws2812_set_mode()` + `ws2812_update()` at sensor_core1.cpp:363-365; both mutate the same non-atomic `g_state` (ws2812_status.cpp:51-97) and push into the same PIO TX FIFO via `send_pixel()`, the pause path is live (ao_rcos.cpp:343/353, rc_os_commands.cpp:1042/1058/1098/1122), and no file states an owner for the LED.

### CW-X1-08 -- A Core 0 I2C sensor reader is bound into the CLI hook table that the Core 1 ownership invariant forbids
- Site: src/main.cpp:317 with src/calibration/cal_hooks.cpp:37-52 and src/core1/sensor_core1.cpp:10-12, 170-171, 344-345
- Lens: Concurrency and shared-data ownership -- JPL-C Rule 8 (only the owner accesses the object) and the field manual's rule that if you cannot name owner, mutator and barrier, the ambiguity is itself the finding
- Claim: rc_os_read_accel is wired to cal_read_accel, which performs a direct icm20948_read(&g_imu, ...) from Core 0 with no pause of Core 1, contradicting the invariant stated twice in sensor_core1.cpp that Core 0 must not call icm20948_*() unless g_core1I2CPaused is true.
- Why: sensor_core1.cpp:10-12 and again at :344-345 state the ownership rule as an INVARIANT, and :170-171 repeats it at the read site ("Core 1 owns I2C, so no bus contention. Core 0 must NOT do concurrent icm20948_read()"). cal_read_accel (cal_hooks.cpp:37-52) does exactly that, and main.cpp:317 installs it as the CLI's accel-read callback. The only thing preventing the violation today is that no caller of rc_os_read_accel exists anywhere in the tree -- rc_os_read_mag is invoked from ao_rcos.cpp:667, but the accel hook is assigned and never used. That is the same shape as the defect recorded in core1_i2c_pause.h:86-93: a calibration hook "defined but never called from anywhere" that left the LL-31 race open until someone traced it. Nothing at the binding site, in the function, or in its header marks the precondition, so the first caller added to the 6-position calibration UI reintroduces concurrent I2C access to a device Core 1 is reading at 1 kHz -- the LL Entry 21 and 23 failure class.
- Confidence: medium
- Direction: either delete the dead binding and the function with it, or wrap the body in rc::core1_i2c_pause() / core1_i2c_resume() and state the precondition in cal_hooks.h so it cannot be called from Core 0 without the pause.
- Verdict: CONFIRMED -- `cal_read_accel()` (cal_hooks.cpp:37-52) does a direct `icm20948_read(&g_imu, ...)` with no `core1_i2c_pause()`, contradicting the invariant stated at sensor_core1.cpp:10-12, 170-171 and 344-345; `main.cpp:317` installs it as `rc_os_read_accel`, and a whole-tree grep confirms the finding's own qualifier -- `rc_os_read_accel` is never invoked anywhere (only `rc_os_read_mag` is, at ao_rcos.cpp:667), so this is a correctly-scoped latent-violation/dead-binding finding, not an overstatement.

### X2 -- cross-module duplicated knowledge

Lane: whole-system cross-cutting X2. Target = the same piece of knowledge (a constant, a
conversion, a validation pattern, a protocol/layout assumption, a state-transition rule)
expressed in two or more places across DIFFERENT modules. Guide lens: The spine ES.3 /
Fowler "Duplicated Code" at system scale, plus the contract-surface helper Kind D/E
("is this the only map, or a second copy?"). Literal clones are gate-caught
(bugprone-branch-clone); everything below is a conceptual duplicate a single-file reader
cannot see.

#### Coverage

Enumeration built: every file under src/ and include/ (187 files; 186 source files plus
one .gitkeep). Vendored lib/, EXTERNAL/ and pico-sdk excluded per scope. Method: build the
per-directory enumeration, then run cross-tree greps for the duplication classes named in
the lane brief -- physical/atmospheric constants, unit and fixed-point conversions, CRC and
integrity algorithms, flash/wire layout constants, enum numbering re-encoded as literals,
validation thresholds, and storage/protocol patterns -- then read the candidate files whole
in pairs. 44 files read whole or in substantial part.

include/rocketchip (33 files) -- PARTIAL -- contract-surface headers swept for "is this the
only map"; telemetry_state.h, telemetry_encoder.h, notify_intents.h, config.h,
flash_layout.h, radio_config.h, radio_config_table.h, pcm_frame.h, sensor_snapshot.h,
fused_state.h read whole; board_*.h and job*.h checked only for constant duplication
against config.h.
src (2 files) -- PASS -- main.cpp and shared_state.cpp checked for duplicated init/layout
knowledge; nothing this lane owns.
src/active_objects (18 files) -- PARTIAL -- ao_notify, ao_logger, ao_telemetry, ao_radio,
ao_flight_director, ao_rf_manager read for re-encoded enum/threshold knowledge; the
remaining AOs swept by grep for literal re-encodings of shared vocabularies.
src/calibration (10 files) -- PARTIAL -- calibration_data.cpp and calibration_storage.cpp
read whole; calibration_manager.cpp read at the atmosphere/constants and storage sections;
lm_solver swept only for shared math constants.
src/cli (8 files) -- PARTIAL -- rc_os_dashboard.cpp and rc_os_commands.cpp read at every
telemetry-decode, threshold and constant site; rc_os.cpp / rc_os_debug.cpp swept by grep.
src/core1 (2 files) -- PASS -- sensor_core1.{cpp,h} checked for GPS scale and snapshot
layout duplication; the 1e7 scale is named by the field and has a single owner.
src/diag (2 files) -- PASS -- diag_stats swept for duplicated pin/threshold knowledge.
src/drivers (20 files) -- PARTIAL -- baro_dps310, icm20948, rfm95w, i2c_bus, gps_pa1010d,
spi_bus read at their constant/address/conversion tables; others grep-swept.
src/flight_director (18 files) -- PARTIAL -- flight_state.h, go_nogo_checks.{h,cpp} and
guard_functions.cpp read whole; flight_director.cpp, mission_profile.h,
mission_profile_data.h, flight_actions.h, guard_evaluator/guard_combinator read at the
phase-numbering and threshold sites.
src/fusion (20 files) -- PARTIAL -- eskf.h, eskf_runner.cpp, mahony_ahrs.h read at their
constants; eskf.cpp and eskf_codegen.cpp swept by grep only (generated / single-owner math,
no cross-module duplicate surfaced).
src/log (1 file) -- PASS -- rc_log.cpp swept; no duplicated knowledge in this lane.
src/logging (17 files) -- PARTIAL -- crc16_ccitt.h, crc32.h, data_convert.{h,cpp} and
radio_config_storage.cpp read whole; pcm_frame.{h,cpp}, flight_table, flash_flush read at
their layout tables.
src/math (5 files) -- PASS -- vec3/quat/mat checked for constants duplicated into callers;
single owner.
src/notify (3 files) -- PASS -- notify_resolver.h and notify_backend_led.cpp read; the
intent-to-pattern maps are single-owner.
src/safety (24 files) -- PARTIAL -- rf_link_health.h read whole; fault_protection,
pio_backup_timer, health_monitor, test_mode, flight_in_progress read at the
shared-vocabulary sites; remainder grep-swept.
src/station (2 files) -- PASS -- station_idle_tick swept.
src/telemetry (2 files) -- PARTIAL -- telemetry_encoder.cpp read at every layout, CRC and
enum-mapping site; mavlink_rx.cpp read at its phase comparisons.

#### Findings

### CW-X2-01 -- FlightPhase numbering re-encoded as bare integers in two other modules; the telemetry copy has already drifted
- Site: src/telemetry/telemetry_encoder.cpp:184-200 (and src/active_objects/ao_notify.cpp:84-98) vs src/flight_director/flight_state.h:46-58
- Lens: The spine ES.3 "Don't repeat yourself" / Fowler Duplicated Code; contract-surface helper Kind D (shared vocabulary -- "is there one place that defines these names?"); NL.2 (comment and code disagree).
- Claim: The canonical flight-phase numbering owned by rc::FlightPhase is re-expressed as untyped integer literals in the telemetry encoder and in AO_Notify, and the telemetry copy encodes a superseded enum in which 4=DESCENT, 5=LANDED, 6=ERROR.
- Why: flight_state travels as a bare uint8_t (src/active_objects/ao_logger.cpp:216-217 assigns static_cast<uint8_t>(current_phase); src/logging/data_convert.cpp:94 passes it through to TelemetryState.flight_state), so no compiler check ties the copies to the enum. Against the current enum (kDrogueDescent=4, kMainDescent=5, kLanded=6, kAbort=7, kFault=8), flight_state_to_mav_state maps a vehicle in MAIN_DESCENT to MAV_STATE_STANDBY and a vehicle that has LANDED to MAV_STATE_CRITICAL, and sends MAV_STATE_BOOT for ABORT and FAULT. This is live, not theoretical: the station's runtime-selectable MAVLink output mode calls it at src/active_objects/ao_telemetry.cpp:570 and :697-699 with the vehicle's real phase byte, so a MAVLink ground station reads "standby" while the rocket is descending under main and "critical" after a nominal landing. The AO_Notify copy is the same defect one step behind drift: its switch stops at case 7 and its header claims "Maps 1:1 to FlightPhase enum in flight_state.h" (include/rocketchip/notify_intents.h:28-31), which the addition of kFault=8 already falsified. Every other consumer (src/cli/rc_os_dashboard.cpp:196, src/cli/rc_os_commands.cpp:957-958) casts back to rc::FlightPhase and is correct -- which is why the two literal copies read as safe on a per-file pass.
- Confidence: high
- Direction: Make the phase byte's meaning owned in one place -- either give flight_state.h a named mapping/accessor both consumers call, or have the consumers switch over rc::FlightPhase (static_cast once at the event/wire boundary) so a new or reordered phase produces an unhandled-enum warning under -Werror instead of a silent mis-map. Re-derive the MAV_STATE table from the current enum as part of that change.
- Verdict: CONFIRMED -- flight_state.h:48-58 numbers kDrogueDescent=4/kMainDescent=5/kLanded=6/kAbort=7/kFault=8, ao_logger.cpp:216 static_casts that enum straight into the wire byte and data_convert.cpp:94 copies it unchanged, yet telemetry_encoder.cpp:189-199 (live via encode_heartbeat, called at ao_telemetry.cpp:570 and :697) still reads 5 as LANDED and 6 as ERROR, and ao_notify.cpp:84-98 stops at case 7 under a header comment claiming a 1:1 map -- same knowledge, three places, no barrier or gate, telemetry copy already drifted.


### CW-X2-02 -- TelemetryState byte 44 is "flags" in the struct and "_reserved" in the encoder module
- Site: src/telemetry/telemetry_encoder.cpp:9 and :99-104 (also include/rocketchip/telemetry_encoder.h:110 and :305) vs include/rocketchip/telemetry_state.h:56-62
- Lens: The spine ES.3 (a layout assumption expressed twice) with NL.2 ("if the comment and the code disagree, both are likely to be wrong"); contract-surface helper Kind B/E (the wire layout is the contract).
- Claim: The wire meaning of the last byte of TelemetryState is stated in two modules and they disagree -- telemetry_state.h defines it as flags with kFlagsZuptActive in bit 0, while the encoder module still describes byte 44 as "_reserved" and drops it from the CCSDS nav payload on that basis.
- Why: There is no _reserved member anywhere in the tree; the only occurrences are the four stale mentions inside the telemetry-encoder module. write_nav_payload_42 copies only the first 40 bytes (src/telemetry/telemetry_encoder.cpp:104-110) and ccsds_decode_nav memsets the destination and copies the same 40 bytes back (:407-409), so TelemetryState.flags is set on the vehicle by src/logging/data_convert.cpp:106-107, written into PCM log frames, and then silently zeroed on the radio path. A station operator can therefore never observe zupt_active, and the justification a reader finds for that ("_reserved ... dropped") describes a field that no longer exists. The same stale knowledge is repeated in the public header a caller reads (kNavPayloadLen "no _reserved, no met_ms").
- Confidence: high
- Direction: Decide once whether flags belongs on the wire and state that decision in one place. If it is deliberately not downlinked, say so using the real field name at the payload-length constant; if it should be downlinked, extend the payload and add a static_assert on offsetof/sizeof(TelemetryState) so the encoder's 40-byte prefix is tied to the struct and the next field addition cannot drift silently.
- Verdict: CONFIRMED -- grep shows no _reserved member exists anywhere in src/ or include/; telemetry_state.h:51-64 names byte 44 `flags` with kFlagsZuptActive, data_convert.cpp:106-107 sets it and the decom table logs it, while write_nav_payload_42 copies only kTelemPayloadBytes=40 and ccsds_decode_nav memsets+copies the same 40, with telemetry_encoder.cpp:9/:102 and telemetry_encoder.h:110/:305 justifying the drop by the vanished name -- the byte's meaning is stated in two modules and they disagree, with no static_assert tying the encoder's 40-byte prefix to the struct.


### CW-X2-03 -- CRC-16-CCITT implemented twice, in two modules, under the same name
- Site: src/calibration/calibration_data.cpp:12-39 vs src/logging/crc16_ccitt.h:24-72
- Lens: The spine ES.3 / Fowler Duplicated Code; CCG F.1 (a nameable action re-emitted instead of reused).
- Claim: The project has one shared, header-only, table-driven rc::crc16_ccitt used by several modules, and calibration_data.cpp carries a second private bitwise implementation of the identical algorithm -- with its own re-declared polynomial, init value and bit-width constants -- under the same function name.
- Why: The integrity algorithm for persisted calibration is now a piece of knowledge living in two places. The users of the shared routine (src/telemetry/telemetry_encoder.cpp:127, src/logging/pcm_frame.cpp:60, src/active_objects/ao_radio.cpp:413, src/logging/radio_config_storage.cpp:107) would all pick up a change to the CRC parameters; the calibration store would not, and its records would then validate against a different polynomial with no build error -- surfacing as calibration silently failing to load on the next boot. The shared name is itself a hazard: adding the include to calibration_data.cpp would compile, bind to the file-static, and give no warning that a reader's assumption about which implementation runs is wrong. The two also differ in cost on the same MCU (256-entry table versus a per-bit loop).
- Confidence: high
- Direction: Delete the private implementation and call rc::crc16_ccitt from crc16_ccitt.h so polynomial, init and bit order are stated once. If the calibration record's CRC must stay byte-compatible with already-flashed devices, confirm the two produce identical output first (they appear to) and record that check.
- Verdict: CONFIRMED -- calibration_data.cpp:26-38 is a file-static bitwise crc16_ccitt with its own kCrc16Init/kCrc16Poly/kCrc16HighBit, called unqualified at :110 and :126, while rc::crc16_ccitt (crc16_ccitt.h:62) is the shared table-driven implementation of the identical parameters used by telemetry_encoder, pcm_frame, ao_radio and radio_config_storage; calibration_data.cpp includes no CRC header, so the integrity algorithm genuinely lives in two modules and the shared name would silently keep resolving to the local copy.


### CW-X2-04 -- The dual-sector flash persistence protocol is implemented twice, cloned by hand
- Site: src/logging/radio_config_storage.cpp:26-200 vs src/calibration/calibration_storage.cpp:34-236
- Lens: The spine ES.3 / Fowler Duplicated Code at module scale; CCG F.1 (a multi-step idiom re-emitted at a second site rather than named once).
- Claim: Two modules independently implement the same A/B wear-levelled flash record protocol -- identical 16-byte sector header {state, sequence, reserved[2]}, identical in-use marker convention, identical page/sector alignment guards around flash_safe_execute, identical "higher sequence number wins" arbitration with the same seq_a >= seq_b tie-break, identical alternate-sector selection and erase-then-program-one-page write -- and the second file says so in its own comments ("Pattern cloned from calibration_storage", "mirrored from calibration_storage for consistency").
- Why: The on-flash record protocol is a single piece of knowledge that must now be changed in two files. Divergence has already started and is invisible per-file: radio_config_storage validates the decoded payload against radio_config_sx1276_legal and skips no-op writes to limit flash wear (:110-121 and :227-231); calibration_storage does neither. Any future correction to the protocol -- power-loss handling between the erase and the program, sequence-number wraparound at UINT32_MAX (neither file handles it), or a change of header size -- lands in one copy and leaves the other with the old behaviour on the same flash device, while the compiler and the host tests both stay green because each file is internally consistent.
- Confidence: high
- Direction: Extract the sector/sequence/alternation mechanism into one module parameterized by {sector A, sector B, payload size, validate callback}, and let calibration and radio-config supply only their record type and validator. At minimum make the sector-header type and the arbitration function shared, so the on-flash layout has a single owner.
- Verdict: CONFIRMED -- both files independently implement the same A/B protocol: a 16-byte {state, sequence, reserved[2]} header with static_assert==16, an in-use magic marker, page/sector-alignment guards wrapping flash_safe_execute, identical find_active_sector arbitration down to the seq_a >= seq_b tie-break, alternate-sector selection and erase-then-program-one-page write; radio_config_storage.cpp:11 and :56 admit the clone in comments, and the claimed divergence (whitelist validation at :110-121, no-op-write skip at :227-231, absent from the calibration copy) is present exactly as described.


### CW-X2-05 -- The "RF link is good enough" rule is expressed in three modules, twice with raw literals
- Site: src/flight_director/go_nogo_checks.cpp:41-47 and src/cli/rc_os_dashboard.cpp:241-262 vs src/safety/rf_link_health.h:29-41
- Lens: The spine ES.3 (a validation rule expressed as the same knowledge in more than one place); contract-surface helper Kind D (one vocabulary, one place).
- Claim: The predicate "LinkState is kTrack and LQ is at least 65 percent" is both the pre-arm RF gate and the dashboard green threshold, and both consumers re-encode it with bare literals (state == 2, lq >= 65U) instead of using LinkState::kTrack and the named thresholds in rf_link_health.h.
- Why: Each copy carries a comment asserting it matches the other ("matches the dashboard glance-indicator green threshold", go_nogo_checks.cpp:39-41; "matches the FD pre-arm threshold", rc_os_dashboard.cpp:241-243) -- the agreement is maintained by prose, not by the code. The GoNoGoInput field is declared uint8_t rf_link_state (src/flight_director/go_nogo_checks.h:60-63), so the enum type is erased at the module boundary and the literal 2 cannot be checked. Changing kRfDegradedToTrackLqPct or kRfTentativeLqFloorPct in rf_link_health.h -- the file that presents itself as owning these tunables and says flight test may revisit them -- moves the state machine's hysteresis while leaving the arm gate and the operator's green light at 65, so the indicator and the gate would then disagree with the link state they claim to report. Renumbering LinkState is worse: both literals would silently point at the wrong state.
- Confidence: high
- Direction: Give rf_link_health.h a named predicate (for example rf_link_is_prearm_go(state, lq)) built from LinkState and the existing constants, and have both the Go/No-Go station and the dashboard call it; carry LinkState rather than uint8_t across the GoNoGoInput boundary so the state comparison is type-checked.
- Verdict: RESHAPED -- the two-consumer duplication is real and unmitigated (go_nogo_checks.cpp:43-46 and rc_os_dashboard.cpp:256 both spell the predicate as state == 2 && lq >= 65U, each carrying a comment asserting it matches the other, with GoNoGoInput.rf_link_state declared bare uint8_t so the LinkState numbering is erased at the boundary); but the three-module framing overstates it -- rf_link_health.h declares no pre-arm or green-indicator threshold at all, only the state machine's Schmitt constants kRfTrackToDegradedLqPct=55 / kRfDegradedToTrackLqPct=65 / kRfTentativeLqFloorPct=65, so it is not a third expression of this rule and the narrower true claim is: the pre-arm/green predicate is duplicated between flight_director and cli, and both re-encode LinkState::kTrack as the literal 2.


### CW-X2-06 -- The barometric altitude formula and its coefficients exist in two modules
- Site: src/calibration/calibration_manager.cpp:84-86 and :1175-1183 vs src/drivers/baro_dps310.cpp:25-27 and :194-197
- Lens: The spine ES.3 / Fowler Duplicated Code (a conversion expressed twice); CCG F.1 (an existing named function re-implemented at a second call site).
- Claim: The hypsometric pressure-to-altitude conversion is implemented twice -- the same two coefficients (44330.0F scale, 0.1903F exponent) declared verbatim in both files with the same comments, and the same expression body -- once in the barometer driver and once in the calibration manager.
- Why: calibration_get_altitude_agl computes kHypsometricScale * (1 - powf(p/p0, kHypsometricExponent)), which is exactly what baro_dps310_pressure_to_altitude(pressure_pa, p0) already does and which is declared in a header the calibration module could include (src/drivers/baro_dps310.h:119). What is duplicated is not two magic numbers but the atmosphere model: if the project moves to the full ISA form, adds a temperature term, or corrects the exponent (0.1903 versus the exact 1/5.255 = 0.190295), one copy gets updated and the other does not, and the AGL altitude reported through the calibration path then disagrees with the altitude the driver publishes -- on a vehicle where AGL feeds the main-chute deploy decision. Neither file states which is authoritative, so a reader of either concludes it is.
- Confidence: high
- Direction: Have calibration_get_altitude_agl call baro_dps310_pressure_to_altitude with the stored ground pressure, or lift the conversion and its two coefficients into one shared header both include. Either way delete the second copy of the constants so the atmosphere model is stated once.
- Verdict: CONFIRMED -- kHypsometricScale=44330.0F and kHypsometricExponent=0.1903F (plus kStdAtmPressurePa=101325.0F) are declared verbatim with identical comments at baro_dps310.cpp:25-27 and calibration_manager.cpp:84-86, and calibration_get_altitude_agl (calibration_manager.cpp:1175-1182) restates the body of baro_dps310_pressure_to_altitude (baro_dps310.cpp:195-197, declared baro_dps310.h:119); calibration_manager.cpp includes only calibration/lm_solver headers, so nothing links the two copies of the atmosphere model.
- **Reconciled 2026-08-20:** duplicate of **CW-B10-06**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X2-07 -- The I2C device-address map is declared in five places, and the copy that presents itself as the map is dead
- Site: include/rocketchip/config.h:112-124 vs src/drivers/i2c_bus.h:36-41, src/drivers/i2c_bus.cpp:22-24, src/drivers/icm20948.h:25-27, src/drivers/baro_dps310.h:20-21, src/drivers/gps_pa1010d.h:21, src/cli/rc_os_commands.cpp:164-166
- Lens: The spine ES.3; contract-surface helper Kind E ("Could two boards/roles silently disagree if someone duplicates a constant elsewhere? Finding: duplication of the map, not just a magic number.").
- Claim: The bus addresses of the three I2C devices are declared independently in four modules under five different constant names, and the declaration that advertises itself as the map -- config.h's rocketchip::i2c namespace, headed "I2C Addresses (from HARDWARE.md)" -- has no callers anywhere in src/ or include/.
- Why: A reader who opens config.h finds what looks like the authoritative address map sitting next to the authoritative pin map, and it is inert: every actual transaction uses i2c_bus.h's kI2cAddr* names or a driver's own kIcm20948AddrDefault / kBaroDps310AddrDefault / kGpsPa1010dAddr, and rc_os_commands.cpp declares a third copy of the two alternate addresses. Changing a hardware strap (the exact hazard recorded in LESSONS_LEARNED Entry 13, where the ICM-20948 default was 0x69 and not 0x68) means finding and editing four or five files, the most visible of which has no effect at all. The alternate-address constants in particular are duplicated between i2c_bus.cpp and rc_os_commands.cpp with identical names and values, so the two can drift apart while both keep compiling. The same shape exists one level up for pins: some modules use the rocketchip::pins:: aliases from config.h while others use board:: directly, giving two names for the same pin knowledge.
- Confidence: high
- Direction: Pick one owner for the address map -- i2c_bus.h is the one the code actually uses -- and have the driver headers and the CLI reference it instead of redeclaring. Either delete config.h's rocketchip::i2c namespace or make it the single source the others alias; the "from HARDWARE.md" claim should sit only on the surviving copy.
- Verdict: CONFIRMED -- the three device addresses are declared independently at config.h:118-122, i2c_bus.h:38-41, icm20948.h:26-27, baro_dps310.h:20-21 and gps_pa1010d.h:21, with the two alternates duplicated verbatim between i2c_bus.cpp:23-24 and rc_os_commands.cpp:165-166, and a tree-wide grep for kIcm20948/kDps310/kPa1010d/i2c:: returns only the config.h declarations themselves -- the copy headed "from HARDWARE.md" has zero callers in src/, include/ or test/.


### CW-X2-08 -- The TelemetryState fixed-point decode is re-derived in three modules, and the decommutation table repeats the struct offsets by hand
- Site: src/cli/rc_os_dashboard.cpp:51-52 and :155-162, src/cli/rc_os_commands.cpp:939-947, src/logging/pcm_frame.cpp:19-41 vs src/logging/data_convert.cpp:57-152
- Lens: The spine ES.3 / Fowler Duplicated Code (a conversion and a layout assumption expressed in several places).
- Claim: The quantization contract of the wire struct -- millimetres to metres, centimetres per second to metres per second, the Q15 scale, and the [7:4]=fix / [3:0]=sats packing of gps_fix_sats -- is stated in data_convert.cpp, re-declared with local kMmToM / kCmsToMs constants and raw 0x0F masks in two CLI files, and repeated a fourth time as hand-written byte offsets and scale factors in kStandardDecomTable.
- Why: data_convert.cpp owns the encode direction (fused_to_telemetry) and a matching decode (telemetry_to_fused_approx), but neither display path uses it; each re-applies the same factors itself. The decommutation table -- the self-describing map handed to ground tools -- restates every field's offset and scale with no static_assert tying any entry to offsetof(TelemetryState, ...). The offsets are currently correct, which is exactly why nothing will complain when they stop being: inserting or resizing a field in telemetry_state.h shifts every later entry, and the compiler, the host tests and the frame CRC all stay green because the frame is still 45 bytes of whatever it now contains. A change of scale (moving altitude from millimetres to centimetres, say) has to be made in four places, two of them display code no test exercises.
- Confidence: medium
- Direction: Route the display paths through one decode function (telemetry_to_fused_approx, or a small named accessor per field) so the scale factors and the nibble packing are stated once, and anchor kStandardDecomTable to the struct with a static_assert per row comparing the entry's offset to offsetof, so a layout change fails the build rather than the ground decode.
- Verdict: RESHAPED -- the conversion half holds (data_convert.cpp owns the scales and the nibble packing; rc_os_dashboard.cpp:51-52/155-162 and rc_os_commands.cpp:939-947 each redeclare kMmToM/kCmsToMs and re-apply the [7:4]/[3:0] split, and telemetry_to_fused_approx has no caller outside test_data_model.cpp), but the decommutation-table half is overstated: test/test_pcm_frame.cpp:311-323 does anchor the table to the struct -- DecomTableCoversAllBytes compares the summed field sizes to sizeof(TelemetryState) and DecomTableOffsetsInRange bounds every entry -- so inserting or resizing a field without updating the table fails the host tests; the residual gap is narrower, namely reordering same-sized fields or a table edit that keeps the sizes right while getting an offset wrong.

### X3 -- comments versus their source of truth

Lane X3 is a whole-system cross-cutting pass, not a file-by-file walk. It reads the
comments in `src/` and `include/` that name an authority — a datasheet, a standard, a
project document, a deviation-register ID, an SDK or framework contract, a council
decision — and asks two questions: does the named authority exist and say what the
comment says it says, and does the body do what the comment claims. Findings are
weighted toward claims whose truth lives in a different file from the claim.

#### Coverage

The enumeration swept: all 186 `.c/.cpp/.h/.hpp` files under
`C:/Users/pow-w/Documents/RC-agent-walk/src` and
`C:/Users/pow-w/Documents/RC-agent-walk/include`. Vendored `lib/`, `EXTERNAL/` and
`pico-sdk` were excluded per scope. Each line below is a swept group, not a per-file
verdict; files named individually are the ones read whole.

- include/rocketchip/ (33 files) -- PARTIAL -- Contract-surface sweep of every header's claim prose; `rc_log.h`, `flash_layout.h`, `sensor_seqlock.h`, `telemetry_encoder.h`, `linker_symbols.h`, `version.h`, `config.h` read whole; findings CW-X3-02, -03, -05, -06, -07, -09.
- src/active_objects/ (18 files) -- PARTIAL -- All AO headers read for rate/priority/queue-depth claims and cross-checked against the `.cpp` definitions; `ao_rcos.{h,cpp}`, `ao_radio.cpp`, `ao_telemetry.{h,cpp}`, `ao_rf_manager.{h,cpp}`, `ao_flight_director.cpp`, `ao_health_monitor.cpp` read for claim-vs-body; findings CW-X3-01, -08, -09.
- src/safety/ (18 files) -- PARTIAL -- Every deviation-ID and design-doc citation resolved; `core1_i2c_pause.{h,cpp}`, `fault_protection.cpp`, `crash_record.h` read whole; findings CW-X3-01, -04.
- src/log/, src/logging/ (17 files) -- PARTIAL -- `rc_log.cpp` read whole against `rc_log.h`; `crc16_ccitt.h` and `crc32.h` verified line-by-line against their own stated polynomial/init/final-XOR claims (both PASS); `psram_init.cpp`, `radio_config_storage.{h,cpp}`, `flash_flush.{h,cpp}` swept for datasheet and flash-map claims; findings CW-X3-02, -03, -06.
- src/drivers/ (20 files) -- PARTIAL -- The datasheet-paraphrase concentration; every `datasheet`/`§`/`Section` citation enumerated; `baro_dps310.h`, `icm20948.cpp`, `rfm95w.{h,cpp}`, `i2c_bus.cpp`, `mcu_temp.{h,cpp}`, `gps_pa1010d.cpp`, `spi_bus.cpp` read for paraphrase-vs-pointer form; finding CW-X3-10. Register-offset and timing claims against vendor datasheets are UNVERIFIED — this pass has no datasheet access.
- src/fusion/ (16 files) -- PARTIAL -- Algorithm-provenance citations (Sola 2017 section/equation numbers, ArduPilot/PX4 parameter provenance, DPS310/AK09916/MT3333 noise figures) cross-checked against each other and against `docs/plans/PHASE5_ESKF_PLAN.md`; `eskf.h`, `mahony_ahrs.h`, `eskf_runner.h` read whole; finding CW-X3-05. `eskf.h:199`'s DPS310 noise/altitude derivation agrees with `baro_dps310.h`'s table -- PASS.
- src/flight_director/ (14 files) -- PASS -- All `docs/` pointers resolve; `mission_profile_data.h`'s auto-generated banner and `guard_functions.h`'s shelved-plan pointer both resolve to existing documents.
- src/cli/, src/diag/, src/station/, src/telemetry/, src/notify/, src/calibration/, src/core1/, src/math/, src/main.cpp, src/shared_state.cpp (30 files) -- PARTIAL -- Swept for doc pointers, LL-Entry citations and init-order claims; `main.cpp`'s init-order comments ("flash/I2C before USB per LL Entry 4/12") verified against the actual call order in `init_hardware()` / `init_peripherals()` -- PASS; findings CW-X3-01, -05.
- Cross-tree citation audit -- PARTIAL -- All 36 distinct `docs/`, `standards/` and `*.md` paths named in `src/`+`include/` comments were resolved against the working tree; 2 do not exist (CW-X3-05). All 40+ `LL Entry N` citations were checked against `docs/agents/LESSONS_LEARNED.md` for topic match; every one is correctly numbered and topically correct — a genuine PASS worth recording, since this is the class the project's own LL Entry 37 warns about.
- Cross-tree deviation-register audit -- PARTIAL -- Every deviation ID cited from code (`TP-2`, `CAST-1`, `CAST-2`, `FH-1`, `CG-1`) checked against `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`; 1 does not exist (CW-X3-04).

#### Findings

### CW-X3-01 -- "every reachable runtime flash_safe_execute() callsite" is not every callsite
- Site: src/safety/core1_i2c_pause.h:25-28 (claim); src/active_objects/ao_rcos.cpp:1289-1303 (unprotected path); src/calibration/cal_hooks.cpp:102 (same claim restated)
- Lens: Comments & documentation quality -- JSF AV 134 (assumptions/limitations in the preamble) + NL.2 comment/code disagreement; spine block B confabulation (NIST AI 600-1 -- confident justification the body does not implement)
- Claim: The `core1_i2c_pause` module header states that R-17 wired the I2C pause around every reachable runtime `flash_safe_execute()` callsite, and that this upgrades the R-11 SPIN property `xltl_no_i2c_during_flash` to hard-PASS; at least one reachable runtime callsite runs unpaused.
- Why: There are exactly three `core1_i2c_pause()` / `core1_i2c_resume()` pairs in the tree (`ao_rcos.cpp:343/353`, `rc_os_commands.cpp:1042/1058`, `rc_os_commands.cpp:1098/1122`). `AO_RCOS_start_cal_save()` (`ao_rcos.cpp:1289`) calls `calibration_save()` and reaches `calibration_storage.cpp:107 flash_safe_execute(...)` with no pause. It is reachable from the live CLI: pressing `v` in the calibration menu (`src/cli/rc_os.cpp:241`). So the exact LL Entry 31 race the header says was closed is still open on the operator-triggered save-calibration key while Core 1 is driving I2C in the sensor phase. The sibling path `cal_save_to_flash()` (`ao_rcos.cpp:335`) does pause and carries the R-17 rationale verbatim — two save paths, one protected, one not, with the header asserting universal coverage. A second unpaused runtime callsite sits behind `ROCKETCHIP_RADIO_PERSIST` (`ao_radio.cpp:706` -> `radio_config_storage.cpp:83`), currently compiled out (the define is absent from `CMakeLists.txt`) and live the moment the flag is enabled. The formal-verification sentence is the sharpest part: a reader is told a SPIN property passes because of a completeness claim that is false.
- Confidence: high
- Direction: Correct the header to state the callsites the pause actually covers, and remove or re-qualify the `xltl_no_i2c_during_flash` hard-PASS sentence until it is true. Separately raise the unpaused `AO_RCOS_start_cal_save()` path for the safety/concurrency lane — the comment fix alone does not close the race.
- Verdict: CONFIRMED -- opened every cited file: the header's "every reachable runtime flash_safe_execute() callsite" is false, since `AO_RCOS_start_cal_save()` (ao_rcos.cpp:1289) reaches `calibration_storage.cpp:107 flash_safe_execute()` with no `core1_i2c_pause()` and is reachable from the live CLI 'v' key (rc_os.cpp:239-241), while only three pause/resume pairs exist (ao_rcos.cpp:343/353, rc_os_commands.cpp:1042/1058 and 1098/1122), cal_hooks.cpp:102 restates the same universal claim, and `xltl_no_i2c_during_flash` appears nowhere in the tree at all -- so the hard-PASS sentence rests on an unverifiable property plus a false completeness claim.
- Verdict: CONFIRMED -- (independent adversarial re-verification) Opened every cited file: core1_i2c_pause.h:25-28 does assert R-17 wrapped "every reachable runtime flash_safe_execute() callsite" and on that basis upgrades `xltl_no_i2c_during_flash` to hard-PASS, yet exactly three pause/resume pairs exist (ao_rcos.cpp:343/353, rc_os_commands.cpp:1042/1058 and 1098/1122) while `AO_RCOS_start_cal_save()` (ao_rcos.cpp:1289) -- reachable from the live CLI 'v'/'V' key at rc_os.cpp:239-241 -- calls `calibration_save()` into calibration_storage.cpp:107 `flash_safe_execute()` with no pause and no compensating barrier, ownership rule or mechanical gate, and the SPIN property name appears nowhere in the tree.
- **Reconciled 2026-08-20:** duplicate of **CW-B34-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X3-02 -- rc_log.h's LOCKED contract states drop-newest; the implementation is drop-oldest
- Site: include/rocketchip/rc_log.h:24-28 (contract); src/log/rc_log.cpp:436-444, 499-527 (implementation)
- Lens: Comments & documentation quality -- NL.2 ("if the comment and the code disagree, both are likely to be wrong"); JSF AV 134
- Claim: The header's contract, explicitly marked "LOCKED at Unit B per council round 1+2", says that when the ring is full "the message is dropped on the floor"; `emit()` instead evicts already-queued bytes from the tail and always admits the new message.
- Why: `emit()` computes `evict = len - avail`, advances `g_tail` past that many bytes, then writes the new message unconditionally. The retention semantics are the inverse of the contract: a caller reading the header concludes that once `rc_log()` returns, earlier output is safe and only the newest message can be lost under backpressure. In reality a burst — the `diag_stats_dump()` case the `.cpp` names at line 465, or the boot banner before the host attaches — discards the oldest bytes, which is where the highest-value content lives (the `.cpp` comment cites losing the T=0 Preconditions block for exactly this reason). Any soak script or HW gate written against the header's semantics ("I saw the last line, therefore I saw all lines") is unsound, which is the LL Entry 36 hard-gate-to-soft-gate pattern the `.cpp` is otherwise guarding against. The `.cpp` documents drop-oldest and cites a council decision dated 2026-05-16 — after the header contract was declared locked — so this is a real supersession that never propagated to the contract surface.
- Confidence: high
- Direction: Update the header's Sink bullet to state drop-oldest ("newest wins") with the two observability counters as the detection mechanism, and note the supersession date so "LOCKED at Unit B" is not read as still binding.
- Verdict: CONFIRMED -- rc_log.h's LOCKED Sink bullet says a full ring means "the message is dropped on the floor", while `emit()` (rc_log.cpp:499-527) computes `evict = len - avail`, advances `g_tail`, and then writes the new message unconditionally; the .cpp's own header comment names this "drop-oldest" and cites a 2026-05-16 council decision post-dating the Unit-B lock, so the supersession is real and never reached the contract surface.
- Verdict: CONFIRMED -- (independent adversarial re-verification) The LOCKED Sink bullet at rc_log.h:24-28 states that on a full ring "the message is dropped on the floor", while `emit()` (rc_log.cpp:499-527) computes `evict = len - avail`, advances `g_tail` past that many already-queued bytes and then writes the new message unconditionally; the .cpp's own target-sink preamble names this "drop-oldest (not drop-newest)" under a council decision dated 2026-05-16 that post-dates the Unit-B lock, and no supersession note reaches the header, so the contract surface is affirmatively wrong rather than merely vague.
- **Reconciled 2026-08-20:** duplicate of **CW-B01-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X3-03 -- rc_log.h forbids fault-handler logging; Q_onError logs
- Site: include/rocketchip/rc_log.h:45-46 (PROHIBITED); src/safety/fault_protection.cpp:212 (violating caller); src/log/rc_log.cpp:446-452 (rationale resting on the claim)
- Lens: Comments & documentation quality -- NL.2 / JSF AV 134; spine block B (a confident contract the tree does not honor)
- Claim: The rc_log contract's PROHIBITED list says "ISR / fault-handler context — handlers do not log; the post-boot health_monitor logs prior-fault state", but `Q_onError()` — a `Q_NORETURN` fault handler that has already executed `cpsid i` — calls `rc::rc_log("[QP ASSERT] module=%s, id=%d\n", ...)`.
- Why: The claim is load-bearing twice over. `rc_log.cpp:446-452` justifies plain `volatile` head/tail rather than atomics on the explicit ground that "rc_log is called from Core 0 cooperative context only (per rc_log.h contract — never from ISR, never from Core 1) ... Producer and consumer never run concurrently; the volatile head/tail are belt-and-braces against compiler reordering, not against preemption." QP raises `Q_onError` from wherever the assertion fires, and the project's own LL Entry 32 documents the canonical case: `qf_actq id=130` queue overflow raised from the 100 Hz QF tick ISR posting into a full AO queue. On that path `Q_onError` preempts Core 0 mid-`emit()` (after `g_head` has advanced for some but not all bytes of the in-flight message) and appends its own bytes, so the "producer and consumer never run concurrently" premise is false and the ring can be left interleaved. The blast radius is bounded only by the accident that `Q_onError` never returns. A reviewer reading either file alone sees a coherent story; only reading both plus the caller shows the premise is unmet.
- Confidence: high on the contract violation; medium on the ISR-preemption consequence (depends which QP assertion fires)
- Direction: Decide which side is authoritative — either carve a documented exception into the rc_log contract for the one NORETURN fault-path caller and re-derive the `rc_log.cpp` concurrency rationale without the "never from ISR" premise, or move `Q_onError` off the ring to a direct emission path.
- Verdict: CONFIRMED -- rc_log.h:45-46 lists "ISR / fault-handler context" as PROHIBITED, yet `Q_onError` (fault_protection.cpp:212, `Q_NORETURN`, after `cpsid i`) calls `rc::rc_log("[QP ASSERT] ...")`, and rc_log.cpp:446-452 still derives its plain-`volatile` (no-atomics) concurrency rationale from the premise "never from ISR, never from Core 1"; the .cpp's local "best-effort live print" comment acknowledges the call but carves no exception into either the contract or the concurrency derivation, so both load-bearing claims remain unmet.
- Verdict: CONFIRMED -- (independent adversarial re-verification) rc_log.h:45-46 lists "ISR / fault-handler context" under PROHIBITED, `Q_onError` (fault_protection.cpp:174-212, `Q_NORETURN`, after `cpsid i`) calls `rc::rc_log("[QP ASSERT] ...")`, and rc_log.cpp's target-sink preamble still derives its plain-`volatile`/no-atomics concurrency argument from the premise "rc_log is called from Core 0 cooperative context only (per rc_log.h contract -- never from ISR, never from Core 1)"; the only mitigating text is a local "best-effort live print" comment at the callsite, which carves no exception into either the contract or the derivation.
- **Reconciled 2026-08-20:** duplicate of **CW-B31-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X3-04 -- deviation ID FH-1 is cited from code but has no row in the register
- Site: src/safety/fault_protection.cpp:101-103; standards/ACCEPTED_STANDARDS_DEVIATIONS.md (no FH-1 row); docs/decisions/FAULT_HANDLER_DESIGN.md:139 (same citation)
- Lens: Comments & documentation quality -- NL.3 (point to the spec) + spine block B confabulation (NIST AI 600-1 -- a cited reference that does not exist)
- Claim: `memmanage_fault_handler`'s preamble states "Function-size deviation logged in standards/ACCEPTED_STANDARDS_DEVIATIONS.md (FH-1)", but that register contains no FH-1 entry.
- Why: The register's Active section holds CG-1, FP-1, CAST-1 and CAST-2; the adopted-code register holds TP-1 and TP-2; the Resolved section holds FP-1, RC-1, BM-6, PP-1, GT-1, AP-1, AP-3. A full-file search for `FH-1` returns nothing. Two independent artifacts cite it (the source comment and the R-3 design record), so a reader gets no signal that the row is missing. The consequence is procedural and concrete: `docs/agents/SESSION_CHECKLIST.md` item 17 requires that any clang-tidy function-size / cognitive-complexity violation be fixed, decomposed, or logged as an accepted deviation before a stage can close. For the tree's most safety-critical function the code says the deviation is logged, the register says nothing, and the milestone gate is satisfied by a comment rather than by the record. This is the LL Entry 37 citation-rot class the project already names.
- Confidence: high
- Direction: Either add the FH-1 row (rule, severity, difficulty, rationale, remediation path) to the register's Active section, or correct both citations to name whatever record actually accepted the deviation.
- Verdict: CONFIRMED -- `grep -n FH-1` over `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` returns nothing (Active section holds CG-1/CAST-1/CAST-2, adopted-code holds TP-1/TP-2, FP-1 sits in Resolved), while fault_protection.cpp:103 and FAULT_HANDLER_DESIGN.md:78/139 both cite it; no NOLINT for `readability-function-size` exists on `memmanage_fault_handler`, so no mechanical gate absorbs the missing row either.
- Verdict: CONFIRMED -- (independent adversarial re-verification) A full-file search of `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` returns no `FH-1` row in any section, while fault_protection.cpp:103 and docs/decisions/FAULT_HANDLER_DESIGN.md:78 and :139 both cite it as logged there, and `memmanage_fault_handler` carries no `readability-function-size` NOLINT (its only NOLINT pair at :261/:292 is `readability-magic-numbers` inside the MPU setup), so no mechanical gate or documented decision absorbs the missing record.


### CW-X3-05 -- flight-tuning constants cite a plan document that does not exist, and the plan that does exist gives different values
- Site: src/fusion/mahony_ahrs.h:16 and :40-67; docs/plans/PHASE5_ESKF_PLAN.md:239 (the only extant authority); same class at src/active_objects/ao_telemetry.cpp:80 and include/rocketchip/version.h:5
- Lens: Comments & documentation quality -- NL.3 (a pointer beats a paraphrase, but the pointer must resolve); spine block B confabulation / non-self-contained symbol
- Claim: `mahony_ahrs.h` attributes its council-approved gains to `PHASE5_MAHONY_PLAN.md`, which exists nowhere in the tree, and the only surviving plan that specifies Mahony parameters states different numbers than the code uses.
- Why: A repo-wide search finds no `PHASE5_MAHONY_PLAN.md`; the only match is `docs/plans/PHASE5_ESKF_PLAN.md`. Its IVP-45 section specifies `kMahonyKp = 2.0f`, `kMahonyKi = 0.005f`, accel gate `[0.8, 1.2] g`. The header ships `kKp = 0.2F`, `kKi = 0.0087F`, gate `[0.9, 1.1] g` — an order of magnitude apart on Kp. So a reviewer trying to validate the attitude cross-check that feeds the confidence gate follows a dead pointer, falls back to the one live plan, and finds numbers that contradict the code, with nothing in the tree recording which supersedes which. The header's second attribution ("arXiv:0811.4303 + 3-stack consensus", line 41) and the per-constant ArduPilot / PX4 / BetaFlight attributions are UNVERIFIED here — they may well be right, but they are paraphrases of external projects rather than pointers, so nothing in-tree can settle the disagreement. Two further dead pointers of the same class: `ao_telemetry.cpp:80` sends the reader to `docs/decisions/COP1_NOT_PURSUED.md` for the rationale behind the STOP-GAP retry tuning — that file was planned (`docs/plans/STAGE_T_T14_DESIGN.md:607` says "written in this batch") but never written; and `version.h:5` mandates "All print sites must use version_string()", a function that exists nowhere in the tree nor in the audit document it cites.
- Confidence: high (both the non-existence and the numeric divergence are verified in-tree)
- Direction: Re-home the Mahony parameter rationale into a document that exists — add a Mahony section to `PHASE5_ESKF_PLAN.md` carrying the shipped values and why they superseded IVP-45's, or state the per-constant sources inline as the authority and delete the dead pointer. Fix or remove the `COP1_NOT_PURSUED.md` and `version_string()` references the same way.
- Verdict: RESHAPED -- the dead pointers are all verified (`PHASE5_MAHONY_PLAN.md`, `docs/decisions/COP1_NOT_PURSUED.md` and `version_string()` exist nowhere in the tree; STAGE_T_T14_DESIGN.md:607 does promise the COP-1 doc "in this batch"), but the IVP-45-vs-shipped numeric divergence is not itself a comment/code contradiction: `docs/plans/*` is classified historical-record and frozen-on-commit by SESSION_CHECKLIST, so a stage plan differing from shipped values is expected drift, and mahony_ahrs.h:41-67 does carry in-tree per-constant provenance (ArduPilot AP_AHRS_DCM Kp/Ki, PX4/BetaFlight gates). Narrowed claim: three cited authorities named in `src/`+`include/` comments do not exist, and the Mahony gains therefore have no resolvable pointer -- the IVP-45 numbers are corroborating evidence that the pointer must be re-homed, not an independent contradiction needing adjudication.
- Verdict: RESHAPED -- (independent adversarial re-verification) All three dead pointers verify (`PHASE5_MAHONY_PLAN.md`, `docs/decisions/COP1_NOT_PURSUED.md` -- promised at STAGE_T_T14_DESIGN.md:142/:607 -- and `version_string()` exist nowhere outside ETL), but `docs/plans/*` is classified historical-record and frozen-on-commit by SESSION_CHECKLIST's trigger map, so PHASE5_ESKF_PLAN.md:239's IVP-45 values (Kp 2.0f / Ki 0.005f / [0.8,1.2] g) differing from the shipped 0.2F / 0.0087F / [0.9,1.1] g is expected stage drift rather than an independent comment/code contradiction; the prior narrowed claim -- three named authorities do not exist and the Mahony gains therefore have no resolvable pointer, with the IVP-45 numbers as corroboration only -- is the claim that survives.


### CW-X3-06 -- flash_layout.h's region map omits an 8 KB region, and its validator does not do what its preamble says
- Site: include/rocketchip/flash_layout.h:9-16 (map) and :78-99 (validator); src/logging/radio_config_storage.cpp:23-24 (the omitted region's consumer)
- Lens: Comments & documentation quality -- JSF AV 134 + NL.2; contract-surface helper Kind E (layout map, "single map?"); spine block B (comment asserts behavior the body does not implement)
- Claim: The header banner — the file whose whole job is to be the one place flash regions are laid out — is 8 KB stale and omits the radio-config region entirely, and `flash_layout_valid()`'s doc-comment describes a runtime boot check taking a `binary_end` linker symbol that the body neither takes nor performs.
- Why: The banner lists "[FLASH_SIZE - 20KB] Flash-safe test sector (4KB)" and "[512KB .. table-20KB] Flight log data". The constants below tell a different story: `kFlashRadioCfgSectorA/B` (lines 61-66, added at Stage T IVP-T5.5) occupy FLASH_SIZE-24KB..FLASH_SIZE-16KB, so `kFlashSafeTestOffset` (line 68) is FLASH_SIZE-28KB. The address the banner names as the flash-safe test sector — the one region in the map that is deliberately erased — is in fact `kFlashRadioCfgSectorB`. Anyone placing a new region, or hand-computing an erase address, from the banner rather than from the constants collides with persisted radio config. Second half: lines 81-83 say "Call from init to verify layout doesn't overlap firmware binary. binary_end: address of last byte of firmware (from linker symbol). Returns true if layout is valid." The body is a nullary `constexpr` holding three `static_assert`s against `kFlashFirmwareReserve` (a fixed 512 KB estimate), reads no linker symbol, and is called from nowhere but its own `static_assert` on line 99 — a repo-wide search for `flash_layout_valid` and `binary_end` confirms both. So the banner's "Council C-A4: boot validation ensures regions don't overlap firmware" describes a boot-time check against the real firmware end that does not exist, and a firmware image that grows past 512 KB is caught by nothing.
- Confidence: high
- Direction: Regenerate the banner map from the constants (add the radio-config region, move the flash-safe test sector to -28KB, correct the log-region bound), and either implement the described runtime check against the linker's binary-end symbol or rewrite the preamble to describe the compile-time asserts it actually performs.
- Verdict: CONFIRMED -- worked the constants: kFlashCalSectorA = SIZE-8KB, kFlashTableSectorA = SIZE-16KB, so kFlashRadioCfgSectorA/B = SIZE-24KB/SIZE-20KB and kFlashSafeTestOffset = SIZE-28KB; the banner's "[FLASH_SIZE - 20KB] Flash-safe test sector" therefore names the persisted radio-config sector B, and the radio-config region is absent from the map entirely (its consumer is radio_config_storage.cpp:23-24). The validator half also holds: `flash_layout_valid()` is nullary `constexpr` with three `static_assert`s against the fixed 512 KB `kFlashFirmwareReserve`, and `grep` finds `binary_end` only inside its own doc-comment and `flash_layout_valid` only at its own line-99 `static_assert`.
- Verdict: CONFIRMED -- (independent adversarial re-verification) Working the constants from the header itself: kFlashCalSectorA = SIZE-8KB, kFlashTableSectorA = SIZE-16KB, so kFlashRadioCfgSectorA/B = SIZE-24KB/SIZE-20KB and kFlashSafeTestOffset = SIZE-28KB -- meaning the banner's "[FLASH_SIZE - 20KB] Flash-safe test sector (4KB)" names the persisted radio-config sector B and the radio-config region (consumed at radio_config_storage.cpp:23-24) is absent from the map entirely, while `flash_layout_valid()` is a nullary `constexpr` of three `static_assert`s against the fixed 512 KB `kFlashFirmwareReserve` and a repo search finds `binary_end` only inside its own doc-comment and `flash_layout_valid` only at its own line-99 assert.
- **Reconciled 2026-08-20:** duplicate of **CW-B06-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X3-07 -- the seqlock's named source of truth describes a different struct than the one that ships
- Site: include/rocketchip/sensor_seqlock.h:10, :32, :67, :100-101; docs/decisions/SEQLOCK_DESIGN.md:36, :81, :91-94
- Lens: Comments & documentation quality -- NL.3 (the pointer must resolve to something true) + NL.2; contract-surface helper Kind B (shared protocol / data layout is the contract)
- Claim: The header twice declares the shared cross-core struct to be "per SEQLOCK_DESIGN.md (council-approved)" and its `static_assert` message instructs maintainers to "update SEQLOCK_DESIGN.md", but that document specifies a 124-byte struct with a different field set, and the in-code group-size comment inherited from it is now wrong by 12 bytes.
- Why: The shipping struct is 156 bytes (asserted at line 100). `SEQLOCK_DESIGN.md` states "Size: 124 bytes", carries its own `_Static_assert(sizeof(shared_sensor_data_t) == 124)`, and describes GPS fields as "reserved now". A reader following the header's pointer to learn the cross-core data contract — the Core 1 writer / Core 0 reader interface that fusion, logging, telemetry and the CLI all consume — gets a layout missing `gps_hdop`, `gps_vdop`, the three GPS diagnostic bytes, and the entire MCU-die-temperature pair. The drift is demonstrable inside the header too: the document's per-group byte counts were copied in as section comments and only some were maintained. IMU was updated (56 -> 68, correct), but line 67 still reads "GPS (32 bytes)" while the fields sum to 44 (28 bytes of scalars + 3 status bytes + 3 diagnostic bytes + 2 pad + two 4-byte DOP floats); 68 + 20 + 44 + 16 + 8 = 156, matching the assert. Compounding it, `docs/decisions/*` is classified historical-record / frozen by `docs/agents/SESSION_CHECKLIST.md`, so the assert message institutionalizes editing a document the project's own rules say must not be amended forward — the pointer can never be made true by following its own instruction.
- Confidence: high
- Direction: Repoint the header (and the assert message) at a state-of-system home for the layout rather than the frozen decision record, or add a supersession note in `SEQLOCK_DESIGN.md` naming the header as authoritative; correct the "GPS (32 bytes)" group comment to 44 in the same pass.
- Verdict: CONFIRMED -- SEQLOCK_DESIGN.md:36/91/93 specifies a 124-byte struct (no mag_raw_*, no gps_hdop/gps_vdop, no GPS diagnostic bytes, no MCU-die-temperature pair) while the shipping struct asserts 156 at sensor_seqlock.h:100, the header points at that document twice plus in the assert message, the doc carries no supersession header, and the inherited group comment at :67 reads "GPS (32 bytes)" against fields that sum to 44 (68+20+44+16+8 = 156, matching the assert); SESSION_CHECKLIST does classify `docs/decisions/*` as frozen historical-record, so the assert's "update SEQLOCK_DESIGN.md" instruction points at a document the project's own rules bar from forward edits.
- Verdict: CONFIRMED -- (independent adversarial re-verification) SEQLOCK_DESIGN.md specifies "Size: 124 bytes" with its own `_Static_assert(... == 124)` and a field set lacking mag_raw_x/y/z, gps_hdop/gps_vdop, the three GPS diagnostic bytes and the MCU-die-temperature pair, carries no supersession header (Status still "Approved"), and is pointed at twice by sensor_seqlock.h plus in the assert message on a struct that asserts 156 at :100; the inherited group comment at :67 reads "GPS (32 bytes)" against fields summing to 44 (68+20+44+16+8 = 156), and SESSION_CHECKLIST does classify `docs/decisions/*` as frozen, so the assert's "update SEQLOCK_DESIGN.md" instruction is unfollowable as written.


### CW-X3-08 -- a blocking-flash safety justification computed against the wrong active object's parameters
- Site: src/active_objects/ao_rcos.cpp:1300-1302; contradicted by src/active_objects/ao_rcos.h:14, ao_rcos.cpp:146/179/1089, and ao_flight_director.cpp:58-62
- Lens: Comments & documentation quality -- NL.2; spine block B (confident quantitative rationale the code does not support); embedded ADD "blocking in a cooperative scheduler"
- Claim: The comment reassuring the reader that a ~100-500 ms blocking flash write is safe reasons from "100Hz tick rate with queue depth 32" and a "320ms headroom", but AO_RCOS runs at 20 Hz with queue depth 16, and even the comment's own worst case exceeds the headroom it cites.
- Why: `AO_RCOS_start_cal_save()` calls `calibration_save()`, which blocks in `flash_safe_execute()`. The comment's numbers belong to a different object: `g_rcosAoQueue[16]` (line 179) and `QTimeEvt_armX(&me->tick_timer, 5U, 5U)` on a 100 Hz base (line 1089) give AO_RCOS 20 Hz and depth 16, exactly as its own header states at `ao_rcos.h:14`. The 100 Hz / depth-32 / 320 ms figures describe AO_FlightDirector, AO_Radio and AO_Logger (`ao_flight_director.cpp:58-62` and siblings) — which is, in fairness, where the real hazard lives under LL Entry 32, since a blocking handler starves every other AO under QV run-to-completion. But as written the comment computes the budget for the wrong queue and then concludes "tight but sufficient for typical flash writes (~200ms)" from a sentence that has just said the block can reach 500 ms. On the 500 ms tail the 100 Hz depth-32 queues overflow at 320 ms — the `qf_actq` assertion of LL Entry 32, triggered by an operator keypress. A reader auditing whether the cal-save path is scheduler-safe is handed arithmetic that looks rigorous and is not about this object.
- Confidence: high on the parameter mismatch; medium on the overflow consequence (depends on the real distribution of `flash_safe_execute` durations)
- Direction: Restate the budget against the AOs whose queues actually absorb the stall (100 Hz, depth 32, 320 ms) and against the stated worst case rather than the typical case, or drop the reassurance and record the blocking call as a known deviation from the run-to-completion contract.
- Verdict: RESHAPED -- the parameter-mismatch framing is over-stated: under QV run-to-completion a blocking handler starves every AO, so the 100 Hz / depth-32 / 320 ms figures (ao_flight_director.cpp:58-62) are the queues that actually absorb the stall and are the right budget to reason about, even though AO_RCOS itself is 20 Hz / depth 16 (ao_rcos.h:14, `g_rcosAoQueue[16]` at :179, `QTimeEvt_armX(...,5U,5U)` at :1089). Narrowed claim: the comment never names which AO's queue it is budgeting -- inviting a reader to attribute 100 Hz / depth 32 to AO_RCOS, whose own header states otherwise -- and its conclusion "tight but sufficient for typical flash writes (~200ms)" silently drops the 500 ms worst case the same sentence states, which exceeds the 320 ms headroom it cites.
- Verdict: RESHAPED -- (independent adversarial re-verification) The parameter mismatch is real (ao_rcos.h:14 "20Hz tick rate, queue depth 16", `g_rcosAoQueue[16]` at :179, `QTimeEvt_armX(&me->tick_timer, 5U, 5U)` at :1089), but under QV run-to-completion the queues that actually absorb a blocking `flash_safe_execute()` are the 100 Hz depth-32 ones (`g_fdAoQueue[32]`, ao_flight_director.cpp:58-62), so the comment's 100 Hz / depth-32 / 320 ms figures are the correct budget to reason about and the "wrong object's parameters" framing overstates the defect; the surviving claim is the narrower one already written -- the comment never names whose queue it is budgeting, inviting attribution to AO_RCOS whose own header says otherwise, and its "tight but sufficient for typical flash writes (~200ms)" conclusion silently drops the 500 ms worst case stated in the same sentence, which exceeds the 320 ms headroom it cites.


### CW-X3-09 -- wire-format comment names an APID the encoder never emits
- Site: include/rocketchip/telemetry_encoder.h:57-62; src/telemetry/telemetry_encoder.cpp:149, :379-380
- Lens: Comments & documentation quality -- NL.2 (comment/code disagreement); NL.3 (ICD claims must be exact, not approximate)
- Claim: The comment annotating `kApidNavWithConfig` states "On new firmware we always emit 0x101; decoder falls back to 0x001 path if seen", while the constant it annotates is `0x004` and every emitter and decoder in the tree uses `0x004`.
- Why: `0x101` occurs exactly once across `src/` and `include/` — in this comment. `build_primary_header(p, ccsds::kApidNavWithConfig, seq_count, data_len)` at `telemetry_encoder.cpp:149` writes `0x004`; `ccsds_decode_nav` dispatches on the same constant at line 380; `ao_radio.cpp:401`, `ao_telemetry.h:64` and `telemetry_encoder.h:183/295` all say `0x004`. This is the one interoperability number in the file — the value a third-party or future ground-station decoder must match to accept the vehicle's nav stream — and the prose asserts a value 253 apart from the one on the wire. On a codebase whose stated purpose includes CCSDS standards fidelity, an ICD claim disagreeing with the constant beside it is the wrong thing to leave for a reader to reconcile.
- Confidence: high
- Direction: Correct the sentence to `0x004`, and check whether `0x101` was a real earlier allocation worth recording as history rather than silently deleting.
- Verdict: CONFIRMED -- `0x101` occurs exactly once across `src/` and `include/`, in the comment itself; the constant it annotates is `constexpr uint16_t kApidNavWithConfig = 0x004` (telemetry_encoder.h:62), and both the emitter (`build_primary_header(..., kApidNavWithConfig, ...)`, telemetry_encoder.cpp:149) and the decoder dispatch (:380) use that symbol, so the ICD sentence names a value nothing on the wire ever carries.
- Verdict: CONFIRMED -- (independent adversarial re-verification) `0x101` occurs exactly once across `src/` and `include/` -- in the comment itself at telemetry_encoder.h:61 -- while the constant it annotates is `constexpr uint16_t kApidNavWithConfig = 0x004` (:62), and both the emitter (`build_primary_header(p, ccsds::kApidNavWithConfig, ...)`, telemetry_encoder.cpp:149) and the decoder dispatch (:379-380, whose own comment reads "0x004 = nav-with-config") use that symbol, so the ICD sentence names a value nothing on the wire ever carries.
- **Reconciled 2026-08-20:** duplicate of **CW-B05-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X3-10 -- a transcribed datasheet table contradicts itself and the configuration it justifies
- Site: src/drivers/baro_dps310.h:26-53; related restatements at src/fusion/eskf.h:198-203 and src/core1/sensor_core1.cpp:45
- Lens: Comments & documentation quality -- NL.3 ("point to the spec; do not paraphrase it"), judging row "Comment paraphrases an external doc/datasheet/standard instead of pointing to it -- FAIL, replace with a one-line pointer"
- Claim: An eight-row transcription of Infineon DPS310 Table 16 is inlined in the driver header instead of a pointer, and the transcription's own MaxRate column contradicts both the shipped configuration and a second rendering of the same constraint four lines below it.
- Why: The table gives `8x -> MaxRate 16Hz` (line 31) and `16x -> MaxRate 8Hz ... <-- ArduPilot default` (line 32). Line 37 then says "ArduPilot uses 16x @ 32Hz" — four times the maximum the table just asserted for 16x. The shipped config sets `kBaroDps310PresOversampling = 8` with `kBaroDps310PresMeasRate = 32`, labelled "(proven rate)" at line 53 — twice the maximum the table asserts for 8x. Lines 48-51 then give a third, different rendering of the same silicon constraint as a duty-cycle budget (`P_rate x P_meas_time + T_rate x T_meas_time <= 1000ms`, computed as 481 ms), under which 8x @ 32 Hz is comfortably legal. So the file carries two mutually inconsistent paraphrases of one datasheet constraint, and the shipped configuration satisfies one and violates the other. A reader cannot determine from the tree which is the real limit, and the same numbers are relied on elsewhere: `eskf.h:199` derives `kSigmaBaro = 0.033` from this table's 8x row (that derivation is internally consistent and PASSes) and `sensor_core1.cpp:45` restates the 32 SPS figure. The underlying datasheet claims are UNVERIFIED — this pass has no access to IFXDS_DPS310_v1.1 — so the finding is that the inline paraphrase is self-contradictory and unresolvable in-tree, not that any particular number is wrong.
- Confidence: medium
- Direction: Replace the transcribed table with a one-line pointer (for example `// see Infineon DPS310 datasheet IFXDS_DPS310_v1.1 Table 16`) plus only the rows the configuration actually depends on, and reconcile the MaxRate-versus-duty-cycle reading against the datasheet so the "(proven rate)" annotation on line 53 carries a source.
- Verdict: CONFIRMED -- and correctly bounded to the in-tree contradiction rather than to the datasheet: baro_dps310.h:31-32 gives 8x -> 16 Hz and 16x -> 8 Hz MaxRate, line 37 asserts "ArduPilot uses 16x @ 32Hz", the duty-cycle rendering at :47-49 computes 32x14.8 + 2x3.6 = 481 ms as legal, and the shipped config at :52-53 is 8x @ 32 Hz labelled "(proven rate)" -- two mutually inconsistent paraphrases of one constraint, with the shipped configuration satisfying one and violating the other; eskf.h:198-199's 0.4 Pa x 0.083 m/Pa = 0.033 m derivation and sensor_core1.cpp:45's 32 SPS restatement both track the same table.
- Verdict: CONFIRMED -- (independent adversarial re-verification) And correctly bounded to the in-tree contradiction rather than to the unavailable datasheet: baro_dps310.h:31-32 gives 8x -> MaxRate 16Hz and 16x -> MaxRate 8Hz, :37 asserts "ArduPilot uses 16x @ 32Hz", the duty-cycle rendering at :47-49 computes 32x14.8ms + 2x3.6ms = 481ms as legal with 52% margin, and :52-53 ships 8x @ 32 Hz labelled "(proven rate)" -- two mutually inconsistent paraphrases of one silicon constraint with the shipped configuration satisfying one and violating the other, while eskf.h:198-199 and sensor_core1.cpp:45 both restate figures from the same table.
- **Reconciled 2026-08-20:** duplicate of **CW-B10-05**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### X4 -- public header contract coherence

Lane: whole-system cross-cutting read of include/rocketchip/ as ONE interface surface -- does each
header earn its file, is each declared contract honored by its implementation, are related
contracts split or duplicated across headers, and does a consumer get a coherent story. Primary
lens: field manual **Class & interface design** plus **Comments & documentation quality**, run
through the contract-surface helper (kinds A-F). Every header in the directory was read whole; the
implementation half of each contract was read or grepped wherever the claim was checkable.

#### Coverage

Public surface -- all 33 headers in C:/Users/pow-w/Documents/RC-agent-walk/include/rocketchip/:

- include/rocketchip/ao_signals.h -- FAIL -- Kind D signal catalog; vocabulary and numbering are sound, but the event-allocation contract contradicts itself and the tree (CW-X4-01), the LED payload points at the wrong header (CW-X4-04), and rc_signal_name is declared with no definition (CW-X4-07).
- include/rocketchip/board.h -- PARTIAL -- Kind E selector; clean dispatch chain, but the board:: interface it dispatches to is nowhere enumerated, so an incomplete board header is caught only by a build break (CW-X4-11).
- include/rocketchip/board_feather_rp2350.h -- PASS -- complete board:: member set; pins and capability flags coherent with the config.h consumers.
- include/rocketchip/board_fruit_jam.h -- PASS -- complete member set; the [M1] and [M3] hazard notes are real constraints, not restated code.
- include/rocketchip/board_pico2.h -- FAIL -- omits kPsramCsPin, which config.h references unconditionally (CW-X4-11).
- include/rocketchip/board_tiny_2350_common.h -- PASS -- complete member set including kPsramCsPin; TODO markers honestly scope the unverified pins.
- include/rocketchip/board_tiny_2350_plus.h -- PASS -- variant overrides only; the bring-up error gate matches the stated story.
- include/rocketchip/config.h -- PARTIAL -- overloaded hub (pins, I2C map, timing, feature flags, version alias, debug macros, assertions); the RC_ASSERT facility it advertises promises a recovery path this tree does not have (CW-X4-08).
- include/rocketchip/flash_layout.h -- PASS -- Kind E map; every derived region anchored from one symbol and pinned by static_assert; the single-source claim holds.
- include/rocketchip/fused_state.h -- PASS -- producer-owned snapshot struct, units documented per field, consumers read-only.
- include/rocketchip/job.h -- PASS -- Kind E role selector; enum plus include dispatch, one job per file.
- include/rocketchip/job_capabilities.h -- PARTIAL -- two of three predicates are consumed by health_monitor exactly as documented; kRoleHasFullGoNogo has zero consumers despite a detailed consumers rationale.
- include/rocketchip/job_relay.h -- PASS -- three role constants, matching the surface job.h implies.
- include/rocketchip/job_station.h -- PASS -- same surface as its siblings; the MAVLink-default comment states a real why.
- include/rocketchip/job_vehicle.h -- PASS -- same surface as its siblings.
- include/rocketchip/led_patterns.h -- FAIL -- claims Single Source of Truth for the LED code space while a divergent duplicate lives in action_executor.h (CW-X4-04).
- include/rocketchip/linker_symbols.h -- PASS -- Kind F boundary; reference-not-define rationale centralized exactly as the helper's Example 4 prescribes.
- include/rocketchip/mavlink_rx.h -- PASS -- API contract states ownership (caller-owned state, borrowed encoder, caller writes the result); signatures match the prose.
- include/rocketchip/notify_backend.h -- PASS -- two free functions, both defined and built; the audio-always-compiled claim is true (src/notify/notify_backend_audio.cpp appears in both CMake source lists).
- include/rocketchip/notify_intents.h -- PASS -- Kind D vocabulary; the stated priority order and the ascending-value-picks-max rule match src/notify/notify_resolver.h.
- include/rocketchip/pcm_frame.h -- PASS -- Kind B/E layout; byte-offset prose matches every static_assert; encode and decode pairs all defined.
- include/rocketchip/prearm_fail_ticks.h -- PASS -- pure helper; the semantics block matches the body line for line.
- include/rocketchip/radio_config.h -- FAIL -- publishes a kDefaultRadioConfig that no consumer uses and that disagrees with the real boot default (CW-X4-05).
- include/rocketchip/radio_config_table.h -- FAIL -- banner names the table as the SET_RADIO_CONFIG gate; it is not, and the membership test it exports is dead with a fabricated consumer list (CW-X4-06).
- include/rocketchip/radio_scheduler.h -- PASS -- header-only state machine; the two-phase init contract is satisfied at its single construction site (ao_radio.cpp:560).
- include/rocketchip/rc_log.h -- FAIL -- the LOCKED CONTRACT block states drop-newest while the sink implements drop-oldest (CW-X4-02), and the PROHIBITED clause is violated by the project's own fault handler (CW-X4-03).
- include/rocketchip/sensor_seqlock.h -- PARTIAL -- Kind B protocol is coherent and barrier-correct in itself, but it also re-declares the whole cross-core global set without the ownership annotations that live in shared_state.h (CW-X4-09).
- include/rocketchip/sensor_snapshot.h -- FAIL -- a 40-byte layout contract with zero producers and zero consumers in src/ (CW-X4-10).
- include/rocketchip/shared_state.h -- FAIL -- declares itself the centralized ownership map, but most cross-core consumers never include it (CW-X4-09).
- include/rocketchip/station_output_mode.h -- PARTIAL -- earns its file as a circular-include break, but carries three AO_RCOS_* function declarations that otherwise belong to ao_rcos.h, splitting that AO's API across two headers.
- include/rocketchip/telemetry_encoder.h -- PARTIAL -- the encoder/decoder surface and the CCSDS size constants are internally consistent and static_assert-pinned; the MAVLink frame count and size are stated three different ways (banner "4 messages ~144 B", line 42 "3-message set ~105 B", line 269 max_packet_size()==144), and the nav-with-config comment at line 60 names APID 0x101 where the constant at line 62 and the encoder both use 0x004.
- include/rocketchip/telemetry_state.h -- PASS -- wire format pinned at 45 bytes; DEPRECATED aliases are labelled and referenced nowhere load-bearing.
- include/rocketchip/version.h -- PARTIAL -- the single-source-of-identity claim is undermined by a second board-identity string (CW-X4-12), and line 5 directs all print sites to a version_string() that exists nowhere in the tree.

Implementation half read or grepped to judge the claims above (not my assigned files; opened only
as truth-checks): src/log/rc_log.cpp, src/safety/fault_protection.cpp, src/safety/pio_watchdog.h,
src/active_objects/{ao_telemetry.cpp, ao_radio.cpp, ao_rf_manager.cpp, ao_flight_director.cpp,
ao_rcos.h}, src/flight_director/{flight_director.cpp, action_executor.h, action_executor.cpp,
flight_actions.h, mission_profile_data.h}, src/notify/notify_resolver.h,
src/logging/{pcm_frame.cpp, radio_config_storage.cpp}, src/diag/diag_stats.cpp,
src/drivers/{ws2812_status.h, gps_uart.cpp}, src/main.cpp, src/shared_state.cpp, CMakeLists.txt,
test/test_data_model.cpp.

#### Findings

### CW-X4-01 -- Signal catalog states an event-allocation rule that no post site follows and that LL-35 proved fatal

- Site: include/rocketchip/ao_signals.h:121-123, :145, :157 (vs src/active_objects/ao_telemetry.cpp:225, src/active_objects/ao_radio.cpp:489)
- Lens: Class & interface design (JSF AV 72 -- the invariant is part of the construction contract; CCG C.3 interface-vs-implementation), plus spine block B confabulation (NIST AI 600-1) and NL.2 comment-code disagreement
- Claim: The one header every AO includes to learn the event vocabulary gives a consumer two mutually exclusive storage rules for the events it defines, and both are wrong -- "all events can be stack-allocated" in the section banner, and "Allocated from QP/C dynamic event pool [C3-A1]" on RadioTxEvt and RadioRxEvt.
- Why: There is no event pool -- Q_NEW and QF_poolInit appear nowhere in src/ or include/, and the banner itself concedes "pool not allocated". Every real post site instead uses function-static storage (static rc::RadioTxEvt g_txEvt at ao_telemetry.cpp:225, :854, :897, :1016; static rc::RadioRxEvt g_rxEvt at ao_radio.cpp:489), precisely because a stack-local posted event is a use-after-free: QP/C stores the raw pointer and does not copy. That defect already cost this project a boot-time qf_dyn assertion (LESSONS_LEARNED Entry 35). A consumer adding the next AO reads this header, writes RadioTxEvt evt on the stack per the banner, posts it, and reproduces Entry 35 exactly; a consumer trusting the pool comment writes Q_NEW and fails to link. The header is the map, and the map contradicts both the terrain and the project's own hard-won rule.
- Confidence: high
- Direction: Replace both claims with the single rule the tree enforces -- posted events use static storage (or pool storage once a pool exists), never stack-local -- and put the Entry 35 pointer next to the event struct definitions rather than in a lessons file no consumer of this header will open.
- Verdict: CONFIRMED -- both claims verified verbatim (ao_signals.h:122 banner "can be stack-allocated", :145/:157 "dynamic event pool"); Q_NEW/QF_poolInit appear nowhere in src/ or include/, every real post site (ao_telemetry.cpp:185/225/854/897/1016, ao_radio.cpp:489) uses function-static storage, and LL Entry 35 records the stack-local variant as a shipped use-after-free.
- **Reconciled 2026-08-20:** duplicate of **CW-B06-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-02 -- rc_log's LOCKED contract says drop-newest; the sink implements drop-oldest, and the same header says so 70 lines later

- Site: include/rocketchip/rc_log.h:24-27 and :97-104 (vs src/log/rc_log.cpp:429-435 and :500-515)
- Lens: Comments & documentation quality (JSF AV 134 -- assumptions and limitations belong in the preamble; CCG NL.2 -- "if the comment and the code disagree, both are likely to be wrong"), plus helper Kind C API contract
- Claim: The block labelled "CONTRACT (LOCKED at Unit B per council round 1+2)" states that on a full ring "the message is dropped on the floor", while the implementation always writes the new message in full and evicts the OLDEST queued bytes -- and the observability paragraph in the same header correctly calls it "evicted by drop-oldest".
- Why: The two policies have opposite consequences for a caller. Under the documented drop-newest rule, anything already logged is safe and only new output is at risk, so a caller can emit a positive-control line and trust it survives a later burst. Under the implemented drop-oldest rule, any later burst silently erases earlier bytes -- which is exactly the failure the ring-sizing comment at rc_log.cpp:462-470 records having already happened once (the T=0 Preconditions block vanishing mid-dump on the station, named there as the LL Entry 36 pattern). A reader who takes the LOCKED block at face value will place a load-bearing diagnostic early in a burst and believe it is safe. The same clause also names the wrong drain ("drained by tud_task"); the actual drain is rc_log_drain_to_cdc() called from the QV idle bridge, as the header's own line 91-95 states.
- Confidence: high
- Direction: Correct the CONTRACT block to state drop-oldest and to name rc_log_drain_to_cdc() as the drain. Since the block is marked LOCKED, record this as a correction of a mis-stated contract rather than a contract change.
- Verdict: CONFIRMED -- rc_log.h:24-27 states "dropped on the floor" / "drained by tud_task" while src/log/rc_log.cpp evicts g_tail forward on overflow (drop-oldest, message always written in full) and the drain is rc_log_drain_to_cdc() from the QV idle bridge; the same header at :98-100 already says "evicted by drop-oldest", so the file contradicts itself.
- **Reconciled 2026-08-20:** duplicate of **CW-B01-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-03 -- rc_log.h forbids fault-handler logging; Q_onError logs, and rc_log.cpp's own safety argument rests on that forbidden case not existing

- Site: include/rocketchip/rc_log.h:42-46 (vs src/safety/fault_protection.cpp:174-212 and src/log/rc_log.cpp:446-450)
- Lens: Comments & documentation quality (JSF AV 134), plus Concurrency & shared-data ownership (JPL-C Rule 8 single owner; CCG CP.3 -- who else touches this, in which context)
- Claim: The header's PROHIBITED section says "ISR / fault-handler context -- handlers do not log", but Q_onError -- the QP fault handler, entered from exception context with interrupts masked by cpsid i at :178 -- calls rc::rc_log at :212, and rc_log.cpp justifies its unsynchronized head/tail on the strength of that very prohibition.
- Why: rc_log.cpp:446-449 reasons that "rc_log is called from Core 0 cooperative context only (per rc_log.h contract -- never from ISR, never from Core 1). Producer and consumer never run concurrently; the volatile head/tail are belt-and-braces against compiler reordering, not against preemption." If a MemManage fault or QP assertion fires while Core 0 is inside rc_log_drain_to_cdc() -- which is where Core 0 spends its idle time, per the main.cpp idle bridge -- the handler's rc_log mutates g_head and g_tail mid-drain, and the no-preemption premise no longer holds. The damage is bounded today only because Q_onError is Q_NORETURN so the interrupted drain never resumes; the premise the implementation is written against is documented as guaranteed and is not. Separately, the project's two logging surfaces now disagree, so the next handler author has no honest rule to follow: the header and fault_protection.cpp:204-211 cannot both be right.
- Confidence: high
- Direction: Pick one and make both files say it -- either carve a narrow, explicit exception into the PROHIBITED clause for the terminal Q_NORETURN fault path and restate rc_log.cpp's concurrency note to cover preemption, or route the fault handler through the crash record only and drop the live print.
- Verdict: CONFIRMED -- rc_log.h:42-46 forbids fault-handler logging, Q_onError (fault_protection.cpp:174-212) masks interrupts at :178 and calls rc::rc_log at :212, and rc_log.cpp:446-449 rests its unsynchronized head/tail on "per rc_log.h contract -- never from ISR"; the ISR reachability is real, not hypothetical -- lib/qep/bsp_qv.c:40-43 runs QTIMEEVT_TICK_X from a 100 Hz hardware-timer IRQ and lib/qep/qf_actq.c:76 asserts Q_ASSERT_INCRIT(130) on queue-full from that post path (the qf_actq id=130 trip recorded in LL Entry 32), so Q_onError -> rc_log can preempt rc_log_drain_to_cdc; docs/decisions/FAULT_RECOVERY_2026-05-14.md carries no logging exception, and memmanage_fault_handler (the one true exception handler) does not log, so nothing reconciles the header with fault_protection.cpp.
- **Reconciled 2026-08-20:** duplicate of **CW-B25-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-04 -- The LED pattern code space is defined in one header, duplicated and diverged in a second, and pointed at a third that no longer holds it

- Site: include/rocketchip/led_patterns.h:4-7, :18, :75, :89 -- src/flight_director/action_executor.h:47-61 -- include/rocketchip/ao_signals.h:135
- Lens: Class & interface design / duplication (CCG ES.3 -- each piece of knowledge in exactly one place; Fowler "Duplicated Code"), plus helper Kind D single-vocabulary check
- Claim: led_patterns.h declares itself "Single Source of Truth" for the pattern codes and asserts twice that values 20-27 "match LedPhaseValue in action_executor.h", but LedPhaseValue is a second full copy of that code space which has already drifted -- code 28 is kFdPreArmFail (yellow double-flash) in the public header and kLedPhaseFault (magenta blink) in the private one, and code 20's comment there still reads "Amber solid" where the public header records the Stage L swap to red.
- Why: Two headers own the same knowledge, so the SSOT claim is false and one copy will always be stale. The concrete collision is live in the flight tables: flight_actions.h:143 emits {ActionType::kSetLed, kLedPhaseFault} (= 28) for FAULT entry, and ao_led_engine.cpp:158 maps 28 to WS2812_MODE_DOUBLE_FLASH yellow -- the pre-arm-fail visual, not the documented magenta fault blink. It does not misfire today only because fd_wire_callbacks (ao_flight_director.cpp:222-228) drops every set_led_cb value except kLedPhaseBeacon, which also means kFaultEntry's documented "magenta blink" never renders at all; the first time that callback is widened, FAULT lights up as pre-arm-fail. Compounding it, ao_signals.h:135 tells a consumer that LedPatternEvt.pattern carries "kCalNeo*, kRxNeo*, kFdNeo* values from ws2812_status.h" -- those names no longer exist in ws2812_status.h and live only in led_patterns.h, so the catalog sends the reader to the wrong file.
- Confidence: high
- Direction: Make LedPhaseValue derive from rc::led::k* rather than restate it (or delete it and use the public codes directly), fix the ao_signals.h pointer to name led_patterns.h, and reconcile code 28 in the same change.
- Verdict: CONFIRMED -- led_patterns.h claims SSOT and asserts 20-27 "match LedPhaseValue", but action_executor.h:47-61 is a full second copy that has diverged (28 = kLedPhaseFault magenta vs kFdPreArmFail yellow double-flash; 20 commented "Amber solid" vs the Stage L red swap); flight_actions.h:143 emits 28 and ao_led_engine.cpp:158 maps 28 to DOUBLE_FLASH yellow, and ws2812_status.h contains no kCalNeo*/kRxNeo*/kFdNeo* names, so ao_signals.h:135's pointer is wrong.
- **Reconciled 2026-08-20:** duplicate of **CW-B21-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-05 -- Two "default RadioConfig" constants disagree, and the public one has no consumers

- Site: include/rocketchip/radio_config.h:39-48 (vs src/flight_director/mission_profile_data.h:93-106 and src/active_objects/ao_radio.cpp:536)
- Lens: Class & interface design / duplication (CCG ES.3), plus helper Kind E single-map check
- Claim: The public header exports kDefaultRadioConfig described as "used when no profile [radio] section exists", but nothing in src/, include/ or test/ references it; the value the firmware actually boots on is kDefaultRocketRadioConfig in the generated private header src/flight_director/mission_profile_data.h, and the two disagree.
- Why: ao_radio.cpp:536 assigns s.runtime_config = rc::kDefaultRocketRadioConfig, and ao_telemetry.cpp:204 branches on that same constant's protocol -- the public one is never read. They differ on nav rate (public nav_rate_hz = 2; real = 5) and on protocol (the real one is switched by ROCKETCHIP_STAGE_T3_MAVLINK, the public one is unconditionally CCSDS). A maintainer opening include/rocketchip/radio_config.h to answer "what does this default to?" gets 2 Hz CCSDS, which is not what any built binary does. Worse, that header is the definitional home of the type and so is the natural place to change a default -- a change that would have no effect on any binary. The authoritative copy also lives in a file banner-marked auto-generated, which is the drift surface already flagged for the profile generator.
- Confidence: high
- Direction: Delete kDefaultRadioConfig, or make kDefaultRocketRadioConfig derive from it, so exactly one constant answers "what is the default radio config" and it is the one the public header shows.
- Verdict: CONFIRMED -- kDefaultRadioConfig's only occurrence tree-wide is its own definition at radio_config.h:40, while ao_radio.cpp:536 and ao_telemetry.cpp:204 use kDefaultRocketRadioConfig from mission_profile_data.h:93, which differs on nav_rate_hz (5 vs 2) and gates protocol on ROCKETCHIP_STAGE_T3_MAVLINK.

### CW-X4-06 -- The radio config whitelist header claims to be the SET_RADIO_CONFIG gate; it is not, and its membership test is dead with a fabricated consumer list

- Site: include/rocketchip/radio_config_table.h:5-9, :58-65, :66-79 (vs src/active_objects/ao_telemetry.cpp:254 and src/logging/radio_config_storage.cpp:117)
- Lens: Comments & documentation quality (JSF AV 134 / CCG NL.2), plus spine block B confabulation (NIST AI 600-1 -- a confident rationale the body does not implement)
- Claim: The banner states that "The SET_RADIO_CONFIG dispatcher rejects any incoming config not in this table with denied-ACK", and the design rules state that untested-but-datasheet-legal tuples "must NOT appear here" -- but the production dispatcher validates with radio_config_sx1276_legal(), which accepts any SX1276-legal tuple, and radio_config_in_whitelist() has zero call sites anywhere in src/.
- Why: Both halves of the header's stated purpose are false, and the file contradicts itself: the banner says the table is the gate, while the comment at :58-62 says "Not the production gate for runtime SET_RADIO_CONFIG -- that uses radio_config_sx1276_legal() below." The consequence is that the tested-tuples-only curation the table exists to enforce is enforced nowhere: ao_telemetry.cpp:254 and radio_config_storage.cpp:117 both take the broad validator, so an uplinked SF12 / CR8 config the firmware has never been operated at is accepted at runtime. Separately, radio_config_in_whitelist()'s own doc-comment names three consumers -- "the debug-menu digit-key path (q<digit>z), channel-find scanner, and boot seed" -- and none of them call it, so the header documents a wiring that does not exist.
- Confidence: high
- Direction: Rewrite the banner to state what is true (the table is a preset list and future scanner sweep order, not the accept gate), and either wire radio_config_in_whitelist() into a real caller or remove it together with its consumer list.
- Verdict: CONFIRMED -- the banner's "SET_RADIO_CONFIG dispatcher rejects any incoming config not in this table" is contradicted by :58-62 in the same file and by the only two validation call sites (ao_telemetry.cpp:254, radio_config_storage.cpp:117), both of which call radio_config_sx1276_legal(); radio_config_in_whitelist() has zero call sites in src/, include/ or test/, so its three named consumers do not exist.
- **Reconciled 2026-08-20:** duplicate of **CW-B04-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-07 -- ao_signals.h declares a signal-name lookup that is never defined, and the function it claims to extend covers a third of the catalog

- Site: include/rocketchip/ao_signals.h:196-197 (vs src/flight_director/flight_director.cpp:185-190)
- Lens: spine block B non-self-contained / undefined symbol, plus Class & interface design (CCG C.3 -- the public section is the contract)
- Claim: const char* rc_signal_name(uint16_t sig) is declared in the public signal catalog with the comment "extends flight_signal_name for system-wide signals", but no translation unit defines it and no call site exists.
- Why: Any consumer that takes the header at its word -- a diagnostic dump, a new AO's trace hook, a host test asserting signal names -- gets an undefined reference at link, not a compile error at the call, so the failure surfaces late and in someone else's build. The gap it advertises is real and still open: flight_signal_name() returns kSignalNames[sig - SIG_TICK] only for SIG_TICK <= sig < SIG_MAX (4..13) and "?" for everything else, so all nineteen system-wide signals (SIG_SENSOR_DATA 14 through SIG_RADIO_CONFIG_CHANGED 32) have no name lookup at all. The header promises the fix and ships the hole.
- Confidence: high
- Direction: Either implement rc_signal_name over the full RcSignal range (handling the sentinel gap at 18) or delete the declaration and its "extends" comment, so the catalog stops advertising a lookup the system does not have.
- Verdict: CONFIRMED -- rc_signal_name is declared at ao_signals.h:197 and appears nowhere else in src/, include/ or test/ (no definition, no caller), and flight_director.cpp:185-190 does return "?" for everything outside SIG_TICK..SIG_MAX, leaving signals 14-32 unnamed.

### CW-X4-08 -- config.h's RC_ASSERT documents a watchdog-reset recovery this tree cannot perform

- Site: include/rocketchip/config.h:19-46, especially :25-27 and :41 (vs src/safety/pio_watchdog.h:20-39, src/safety/fault_protection.cpp:165-172, src/main.cpp:414-418)
- Lens: Assertions (P10 Rule 5 / JPL-C Rule 16 -- on this firmware assertions are a live safety mechanism, not a debug no-op), plus Comments & documentation quality (JSF AV 134) and spine block B confabulation
- Claim: The most-included public header advertises RC_ASSERT(expr) with the contract "prints file:line + expression to USB, then spins until the watchdog resets the device (reboot-cause flag preserved)", but the body is while (true) { __asm volatile("nop"); } and there is no watchdog in this tree that resets the device.
- Why: The SDK hardware watchdog was removed at IVP-90; main.cpp:414-418 replaces it with pio_watchdog_feed(), and pio_watchdog.h:34-36 documents the PIO watchdog as raising an IRQ flag the ARM must poll via pio_watchdog_fault_detected(). A core spinning in RC_ASSERT never polls, so nothing fires and nothing resets -- fault_protection.cpp:168-172 records exactly this conclusion for a sibling path ("with no auto-reset path, the previous halt-forever code would leave the chip dead until manual power cycle"). A tripped RC_ASSERT therefore wedges the core permanently instead of rebooting, and the "reboot-cause flag preserved" promise is unreachable. The mirror half: without DEBUG the macro is ((void)0), so the whole facility disappears from the shipped binary -- the opposite of how this project treats assertions elsewhere, where Q_ASSERT routes to Q_onError and is live. Latent today (RC_ASSERT has zero call sites tree-wide), but it sits in the header fifteen modules include, presented as the project's assertion mechanism.
- Confidence: medium
- Direction: Either delete RC_ASSERT (nothing uses it, and Q_ASSERT / Q_onError is the live mechanism) or re-point its failure path at Q_onError and correct the preamble to describe what actually happens on this hardware.
- Verdict: RESHAPED -- the primary claim holds (RC_ASSERT's body is while(true){nop}, the SDK watchdog was removed at IVP-90, main.cpp only feeds the poll-only PIO watchdog, so "spins until the watchdog resets the device / reboot-cause flag preserved" is unreachable and a trip wedges the core); but the mirror half is hypothetical -- CMakeLists.txt:240-245 FORCEs CMAKE_BUILD_TYPE=Debug and defines DEBUG=1, so the facility never compiles out of any binary this tree builds. Narrowed claim: config.h documents a watchdog-reset recovery the tree cannot perform, in a macro with zero call sites.
- **Reconciled 2026-08-20:** duplicate of **CW-B01-05**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-09 -- The cross-core ownership map is declared twice, and the copy most consumers include carries no ownership rules

- Site: include/rocketchip/shared_state.h:3-14 and :63-75, versus include/rocketchip/sensor_seqlock.h:151-159 (definitions in src/shared_state.cpp:39-44)
- Lens: helper Kind A shared-object catalog, plus Concurrency & shared-data ownership (JPL-C Rule 8 -- a shared object has one owning task and ownership is stated) and CCG ES.3 duplication
- Claim: shared_state.h presents itself as the centralized map ("Consolidates all init flags, GPS function pointers, seqlock, atomics ... makes ownership clear. Core 0 owns initialization. Core 1 reads most sensor flags."), but sensor_seqlock.h independently re-declares g_sensorSeqlock and all six cross-core atomics, and the majority of cross-core consumers include only the latter.
- Why: Eleven translation units reach these globals through sensor_seqlock.h and never include shared_state.h -- ao_flight_director.cpp, ao_led_engine.cpp, ao_logger.cpp, ao_notify.cpp, sensor_core1.cpp, health_monitor.cpp, station_idle_tick.cpp, diag_stats.cpp, rc_os_commands.cpp, eskf_runner.cpp, main.cpp -- against seven that include shared_state.h. The ownership prose exists in exactly one of the two copies, so for most of the tree the contract surface a reader actually sees is nine bare extern lines under the comment "Cross-core signaling (atomic flags - FIFO reserved by multicore_lockout)", with no statement of who sets, who clears, or who may read. That is the helper's Kind A failure mode verbatim: the hub does not reach its consumers, so each caller invents its own story. The duplication is also a standing divergence risk, since the two declaration sets must be edited in lockstep and nothing enforces that.
- Confidence: high
- Direction: Have sensor_seqlock.h declare only the seqlock type, protocol and its own instance, and make shared_state.h the single declaration home for the cross-core atomics (or the reverse) -- one home, with the per-object owner / mutator / barrier annotations attached to the declarations every consumer actually sees.
- Note: the correctness of the ownership content itself (for example g_baroInitialized and g_gpsInitialized annotated "Core 1 reads/writes" on plain non-atomic bools) is a concurrency-lens question and is deliberately not judged here.
- Verdict: RESHAPED -- the duplication is real (sensor_seqlock.h:151-159 re-declares g_sensorSeqlock plus all six atomics with no ownership prose, shared_state.h:3-14/63-75 carries the ownership map), but the consumer census is overstated and inverted: main.cpp, rc_os_commands.cpp and eskf_runner.cpp include shared_state.h directly, and sensor_core1.cpp gets it via sensor_core1.h:15. Narrowed claim: seven TUs (ao_flight_director, ao_led_engine, ao_logger, ao_notify, diag_stats, health_monitor, station_idle_tick) reach these globals through the annotation-free copy only.

### CW-X4-10 -- sensor_snapshot.h is a public layout contract with no producer and no consumer

- Site: include/rocketchip/sensor_snapshot.h:3-9 and :18-29 (versus the whole of src/, and test/test_data_model.cpp:22-31)
- Lens: Class & interface design (does the header earn its file), plus Comments & documentation quality (CERT MSC12-C -- documentation must describe code that actually runs) and helper Kind B/E
- Claim: The header defines a 40-byte packed SensorSnapshot stating it holds "ADC counts and raw values before calibration offset/scale application. Used for raw sensor logging" -- but no file in src/ names the type or any of its fields, and its only reference tree-wide is a host test that asserts its size.
- Why: There is no raw-sensor-logging path. accel_raw, gyro_raw, mag_raw, baro_pressure_raw, met_us and the rest appear nowhere in src/; the logging pipeline runs on FusedState fed from shared_sensor_data_t, both of which carry calibrated values. So the public interface advertises a data product the system does not produce, and a consumer sizing a log region or writing a ground decoder against it is building for a stream that never exists. The size static_assert in test/test_data_model.cpp:31 makes this worse rather than better: it is the only coverage the type has, and it proves layout stability for a layout nothing writes -- a green test standing in for a feature that does not exist, which is the passes-tests-yet-wrong signal this walk exists to refuse. The contract-surface helper also lists this file as a canonical contract surface, so a reader is actively directed to treat it as load-bearing.
- Confidence: high
- Direction: Decide whether raw logging is still intended. If yes, state in the header that the layout is reserved and unimplemented; if no, delete the header and its size test rather than leaving a map with no terrain.
- Verdict: CONFIRMED -- SensorSnapshot and every one of its fields appear nowhere in src/; the type's only tree-wide reference is test/test_data_model.cpp:22/31, which asserts its size, so the 40-byte contract has no producer and no consumer.

### CW-X4-11 -- The board:: interface is implicit, and board_pico2.h is missing a member config.h requires

- Site: include/rocketchip/board_pico2.h:28-84 (no kPsramCsPin) versus include/rocketchip/config.h:90; contrast board_feather_rp2350.h:61, board_fruit_jam.h:82, board_tiny_2350_common.h:88
- Lens: Class & interface design (CCG I.25 / JSF AV 87-88 -- a family of implementations needs an explicit interface), plus helper Kind E
- Claim: config.h:90 unconditionally evaluates constexpr uint8_t kPsramCs = board::kPsramCsPin for every board, but board_pico2.h never defines kPsramCsPin, so a Pico 2 build fails to compile inside a shared header -- and nothing anywhere states what the board:: namespace must provide.
- Why: board.h is the dispatcher for four board headers, but the contract those headers implement exists only by convention: there is no interface header, no concept, and no static_assert enumerating the required board:: names. The three other boards happen to define kPsramCsPin; the Pico 2 has no PSRAM, so its author reasonably omitted the pin and instead set kPsramAvailable = false -- but the consumer does not branch on kPsramAvailable. The break is latent today because board_pico2.h is gated behind a PICO2_BRINGUP_OK #error, which means it will surface during hardware bring-up as a confusing failure inside config.h rather than in the board file, and it is not a pin-verification issue that bring-up is expecting to find. The same gap will bite every future board port, which is the stated purpose of the capability-flag design.
- Confidence: medium
- Direction: Give board:: an explicit required-member list -- a small interface header, or a static_assert block in board.h naming each required constant -- and either add a sentinel kPsramCsPin to board_pico2.h or make config.h's kPsramCs conditional on board::kPsramAvailable.
- Verdict: CONFIRMED -- board_pico2.h defines kPsramAvailable=false (:76) and no kPsramCsPin anywhere, while config.h:90 evaluates board::kPsramCsPin unconditionally for every board; board.h is a bare #if/#elif include chain with no interface header, concept or static_assert enumerating required board:: names, and the three other boards do define kPsramCsPin (feather :61, fruit_jam :82, tiny_2350_common :88). Both stated facts are present-tense true; only the resulting compile break is gated behind board_pico2.h:20-21 PICO2_BRINGUP_OK, which the finding itself already scopes as latent.
- **Reconciled 2026-08-20:** duplicate of **CW-B02-02**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X4-12 -- Two different board-identity strings, both spelled kBoardName; the T=0 identity gate and the flight log report different boards

- Site: include/rocketchip/version.h:33-39 versus include/rocketchip/board_feather_rp2350.h:80 (and board_fruit_jam.h:102, board_pico2.h:82, board_tiny_2350_plus.h:31); consumers src/diag/diag_stats.cpp:40, src/logging/pcm_frame.cpp:159, src/active_objects/ao_rcos.cpp:196, src/cli/rc_os_commands.cpp:850
- Lens: helper Kind E single-source-of-identity, plus CCG ES.3 duplication and the header's own single-source-of-truth claim
- Claim: version.h declares itself the "Single source of truth" for identity and exports a global kBoardName = PICO_BOARD, while each board header exports board::kBoardName as a human-readable name -- the same identifier spelling, different values, different namespaces, and both headers are pulled into the same translation units through config.h.
- Why: The two are not interchangeable and the tree uses both. diag_stats.cpp:40 prints the version.h one in the T=0 Preconditions block -- the block whose documented job (diag_stats.cpp:12-14) is to catch Frankenstein builds -- so it reports the CMake board slug, for example adafruit_feather_rp2350. Meanwhile pcm_frame.cpp:159 writes board::kBoardName into the flight log header's board_name[16] field and the CLI banners print board::kBoardName, so the artifact a ground tool parses says "Adafruit Feather RP2", truncated from the 28-character "Adafruit Feather RP2350 HSTX" into a 16-byte field. An operator comparing a soak transcript against a flight log sees two different board strings for one device, and because the only difference at a call site is a namespace qualifier, an unqualified kBoardName in new code silently picks the version.h one.
- Confidence: medium
- Direction: Pick one board-identity string as authoritative and have the other derive from or reference it; rename whichever survives so the two spellings cannot be confused at a call site, and check the chosen name's length against the 16-byte log field.
- Verdict: CONFIRMED -- version.h:39 exports global kBoardName = PICO_BOARD (a CMake slug) while each board header exports board::kBoardName as a display name, both reach the same TUs via config.h -> board.h, and the tree consumes both (diag_stats.cpp:40 unqualified vs pcm_frame.cpp:159 / ao_rcos.cpp:196 / rc_os_commands.cpp:850 qualified); pcm_frame.h:177's board_name[16] does truncate the 28-char Feather string, and version_string() exists nowhere in the tree.

### X5 -- peripheral init, sequence and lifecycle pairing

Lane scope: init -> use -> teardown as a sequence across module boundaries. Boot ordering
dependencies between subsystems, acquire/release pairing, arm/disarm symmetry, and resources set up
in one module and torn down (or not) in another. Field-manual anchor: spine block C, "Peripheral
init SEQUENCE / lifecycle" ADD criterion, supported by CCG P.8 (don't leak any resource; release on
every exit), JSF AV 143 / two-phase-init, JSF AV 134 (documented assumptions and ordering
requirements), CCG NL.2 (comment/code disagreement) and JPL-C Rule 8 (single owner).

Method: built a tree-wide enumeration of lifecycle-shaped symbols across src/ and include/
(`*_init`, `*_deinit`, `*_begin`, `*_start`, `*_stop`, `*_arm`, `*_disarm`, `*_cancel`, `*_claim`,
`*_unclaim`, `*_reset`, `*_recover`, plus the SDK primitives `irq_set_enabled`,
`irq_set_exclusive_handler`, `gpio_set_irq_enabled*`, `exception_set_exclusive_handler`,
`pio_add_program` / `pio_remove_program` / `pio_sm_claim` / `pio_sm_unclaim`,
`multicore_launch_core1` / `multicore_lockout_victim_init`, `flash_safe_execute`,
`save_and_disable_interrupts` / `restore_interrupts`, `uart_init` / `uart_deinit`,
`i2c_init` / `i2c_deinit`, `spi_init`) -- 264 call sites. Then walked each pairing across its
call graph rather than per file, reading the integrator files whole (main.cpp, sensor_core1.cpp)
and the owning drivers/AOs around each pair.

#### Coverage

Boot-sequence integrators (read whole):
src/main.cpp -- FAIL -- full boot chain walked: init_early_hw -> init_hardware -> init_application -> start_active_objects; three ordering/pairing defects anchored here (CW-X5-03, CW-X5-04, CW-X5-05).
src/core1/sensor_core1.cpp -- FAIL -- core1_entry lockout/start-phase handshake, sensor loop, and the Core-1-side GPS reinit call that crosses a core boundary (CW-X5-02).
src/shared_state.cpp -- PASS -- definitions only; init-order-relevant flags (g_i2cInitialized, g_psram*, g_gps*) all zero-init before main, no static ctor ordering hazard.
include/rocketchip/shared_state.h -- PARTIAL -- ownership map read as a contract surface; g_baroContinuous is set once at init and never cleared when Core 1 declares the baro dead (noted, not filed -- data-validity lane).

PIO / pyro arm-disarm chain:
src/safety/pio_backup_timer.cpp -- FAIL -- init/arm/cancel/disarm sequence walked against LL-42 discipline; program lifecycle now correct, but the pin-ownership half of the contract is unimplemented (CW-X5-06).
src/safety/pio_backup_timer.h -- PASS -- declares the arm/cancel/disarm triad clearly; no claim contradicted here.
src/safety/pio_watchdog.cpp -- PARTIAL -- claim/add/init/feed sequence is correct and symmetric; pio_watchdog_deinit() exists, is exported in the header, and has zero callers tree-wide (dead teardown, recorded not filed).
src/safety/pio_watchdog.h -- PARTIAL -- same: exports a teardown no one calls.
src/safety/pyro_edge_logger.cpp -- PASS -- GPIO edge IRQ registered once on Core 0 after the PIO pins exist; no teardown needed, none advertised.
src/active_objects/ao_flight_director.cpp -- FAIL -- the arm/disarm hooks and the backup-fire latch both walked; CW-X5-01 and CW-X5-07.
src/flight_director/flight_director.cpp -- FAIL -- enter_phase() sentinel pairing and the in-HSM ARMED-timeout auto-disarm are two of the bypass routes (CW-X5-01, CW-X5-08).
src/safety/flight_in_progress.cpp -- PARTIAL -- set/clear/consume primitives are correct in isolation; the asymmetry is at the call sites (CW-X5-08).
src/safety/crash_record.h -- PARTIAL -- documents "cleared on safe LANDED entry"; that is the whole clear-side contract and it under-covers the disarm paths (CW-X5-08).
src/safety/anomalous_boot.cpp -- PASS -- consume-once read of the sentinel is correct and singly-owned; it is the upstream setter that is unpaired.
src/active_objects/ao_telemetry.cpp -- FAIL -- both MAVLink ARM/DISARM entry points (radio at :278, USB at :1088) bypass the hardware pairing (CW-X5-01).
src/cli/rc_os.cpp -- PASS -- CLI routes ARM/DISARM/ABORT/RESET through the command wrapper that does maintain the pairing; dispatch_flight_signal is used only for sensor-event injection.
src/flight_director/command_handler.cpp / .h -- PASS -- validation only, no resource lifecycle.
src/safety/fault_inject.cpp -- PASS -- arms the FD via the raw signal path, but is test-mode gated by design and says so.

Flash / Core-1 pause protocol:
src/safety/core1_i2c_pause.h -- PARTIAL -- states the governing contract ("wires these primitives around every reachable runtime flash_safe_execute() callsite"); one reachable callsite is unwrapped (CW-X5-04).
src/safety/core1_i2c_pause.cpp -- PASS -- pause/ack/resume primitive is correct, bounded, and idempotent.
src/cli/rc_os_commands.cpp -- PASS (lifecycle) -- both flash chains (flush at :1042 and erase at :1098) correctly pause, flash, i2c_bus_reset, resume.
src/active_objects/ao_rcos.cpp -- PARTIAL -- cal_save_to_flash() at :335 is correctly wrapped; AO_RCOS_start_cal_save() at :1289 calls calibration_save() with the i2c_bus_reset but no pause/resume (single-file asymmetry, left to the file walk).
src/calibration/calibration_storage.cpp -- PASS -- init is an XIP read only; writes go through flash_safe_execute and are wrapped by their callers.
src/logging/radio_config_storage.cpp -- PARTIAL -- init is read-only; the write path is inside ROCKETCHIP_RADIO_PERSIST, which is never defined in the build, so the unwrapped flash write at ao_radio.cpp:706 is currently unreachable (recorded, not filed).
src/logging/flash_flush.cpp -- PASS -- flash primitives only; pairing is the caller's, and callers do it.
src/logging/psram_init.cpp -- FAIL -- detect/self-test/flash-safe-test sequence walked against Core 1 launch; save_and_disable_interrupts/restore_interrupts pair is single-exit and correct, but the flash-safe test's placement and its unused verdict are defects (CW-X5-04, CW-X5-05).
src/logging/psram_init.h -- FAIL -- documents psram_flash_safe_test() as a "hard gate"; nothing gates on it (CW-X5-05).
src/active_objects/ao_logger.cpp -- FAIL -- init_logging_ring() selects PSRAM on the self-test flag only, ignoring the flash-safe verdict (CW-X5-05).
src/logging/ring_buffer.cpp / .h, src/logging/flight_table.cpp, src/logging/log_decimator.cpp -- PASS -- pure init-into-caller-owned-memory; no acquire/release pair.

Bus and sensor drivers:
src/drivers/gps_uart.cpp -- FAIL -- init registers the RX IRQ on Core 0; reinit is reachable from Core 1 and re-enables it there (CW-X5-02).
src/drivers/gps_uart.h -- PARTIAL -- documents the Core-0-ISR / Core-1-consumer split but not that reinit must run on Core 0.
src/drivers/gps_pa1010d.cpp / .h -- PASS (lifecycle) -- blind-PMTK then probe sequence is internally ordered and documented; the defect is where it is called from (CW-X5-03).
src/drivers/i2c_bus.cpp / .h -- PASS -- init de-isolates pads before recover (LL-41 order), recover deinits the peripheral before the funcsel switch (LL-28 order), reset re-flags correctly around recover.
src/drivers/icm20948.cpp / .h -- PASS (lifecycle) -- bypass-mode enable sequence is ordered and self-documenting; device-reset-clears-BYPASS_EN is stated at the site.
src/drivers/baro_dps310.cpp / .h -- PASS (lifecycle) -- init then start_continuous is a genuine two-phase contract and both phases are satisfied on the one construction path.
src/drivers/spi_bus.cpp / .h -- PARTIAL -- initialises MISO/SCK/MOSI but not any CS pin; CS ownership lives in the radio driver, which is fine except for the documented pre-init read path (CW-X5-09).
src/drivers/rfm95w.cpp / .h -- PARTIAL -- init_gpio_and_reset is the only place a CS pin becomes an output; the header advertises a pre-init diagnostic that depends on it (CW-X5-09).
src/drivers/ws2812_status.cpp / .h -- PARTIAL -- init claims SM + program and deinit releases both symmetrically (correct LL-42 shape); ws2812_status_deinit() has zero callers tree-wide.
src/drivers/mcu_temp.cpp / .h -- PASS -- adc_init + sensor enable once, sole ADC user in the tree, guarded availability accessor.
src/station/station_idle_tick.cpp / .h -- PASS -- init and tick are gated by the same kRadioModeRx constant; reuses the vehicle GPS reader on the same core that owns it.

Swept with no lifecycle surface found (enumeration returned no init/acquire/release pair, or only
caller-owned struct initialisers): src/fusion/ (eskf, eskf_runner, eskf_codegen, eskf_brake,
confidence_gate, innovation_monitor, mahony_ahrs, ud_factor, wmm_tables), src/math/,
src/calibration/ (lm_solver, calibration_data, calibration_manager, cal_hooks),
src/flight_director/ (guard_evaluator, guard_combinator, guard_functions, go_nogo_checks,
action_executor, mission_profile*), src/logging/ (crc16/crc32, data_convert, pcm_frame),
src/log/rc_log.cpp, src/telemetry/, src/diag/diag_stats.cpp (read for CW-X5-09), src/notify/,
src/safety/ (health_monitor, fault_protection, test_mode, station_fault_inject, rf_link_health),
src/active_objects/ (ao_led_engine, ao_notify, ao_health_monitor, ao_rf_manager -- each an
AO_*_start + QActive_start pair with no hardware acquire), src/cli/rc_os_dashboard.cpp,
rc_os_debug.cpp, and all of include/rocketchip/ (board*.h, job*.h, flash_layout.h,
linker_symbols.h, version.h, ao_signals.h, sensor_seqlock.h, sensor_snapshot.h, rc_log.h,
notify_*.h, radio_config*.h, telemetry_*.h) read as contract surfaces for ordering claims.

#### Findings

### CW-X5-01 -- PIO backup pyro timers are disarmed only on the CLI command path; three other ARMED-exit routes leave them counting
- Site: src/active_objects/ao_flight_director.cpp:311-325 (the only arm/disarm pairing), bypassed at src/active_objects/ao_flight_director.cpp:141, src/flight_director/flight_director.cpp:400-404, src/active_objects/ao_telemetry.cpp:278-283 and :1088-1093
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (arm/disarm pairing, resource armed in one module and released in another) + CCG P.8 "don't leak any resource" on every exit path
- Claim: `pio_backup_timer_arm()` / `pio_backup_timer_disarm()` are called only from `AO_FlightDirector_process_command()`, but the FlightDirector HSM can enter and leave ARMED through three routes that never reach that function, so the hardware timers and the software phase can disagree in both directions.
- Why: The backup timers are autonomous PIO state machines that drive the drogue and main pyro GPIOs HIGH at `drogue_timer_s`=15 s and `main_timer_s`=45 s measured from ARM (src/flight_director/mission_profile_data.h:47-48), independent of the ARM cores. (a) Release direction: `fd_tick()` at ao_flight_director.cpp:138-143 dispatches `SIG_DISARM` straight into the HSM on a critical sensor fault while ARMED and latches LAUNCH ABORT; the timers are not disarmed, so a fault at T+2 s produces a vehicle that is IDLE with launch-abort latched and still fires both backup pyro channels at T+15 s and T+45 s on the pad. The in-HSM ARMED-timeout auto-disarm (`flight_director.cpp:400-404`, `Q_TRAN(&state_idle)`) has the same shape. `phase_change_cb` (ao_flight_director.cpp:230-240) does not disarm on kIdle entry either, so nothing downstream repairs it. (b) Acquire direction: a GCS or station ARM arrives as `MAV_CMD_COMPONENT_ARM_DISARM` and goes to `AO_FlightDirector_dispatch_signal()` (ao_telemetry.cpp:282 for the radio path, :1092 for the USB path), which has no PIO hooks -- the vehicle reports ARMED over the link with the backup deployment silently absent, and a subsequent MAVLink DISARM after a CLI ARM leaves the timers armed. Neither file shows this: ao_telemetry.cpp only sees a signal dispatch, ao_flight_director.cpp only sees its own command wrapper.
- Confidence: high
- Direction: move the arm/disarm calls out of the command wrapper and into the phase transition itself -- arm on kArmed entry, disarm on any exit from kArmed/kAbort into kIdle -- so every route through the state machine carries the hardware pairing, and `AO_FlightDirector_process_command` keeps only the operator logging.
- Verdict: CONFIRMED -- `pio_backup_timer_arm`/`_disarm` have exactly two callsites tree-wide (ao_flight_director.cpp:313 and :323, both inside `AO_FlightDirector_process_command`), while fd_tick's critical-fault auto-DISARM (:141), `state_armed`'s `SIG_DISARM` and 300 s-timeout `Q_TRAN(&state_idle)` (flight_director.cpp:394, :398-404) and both `AO_FlightDirector_dispatch_signal` MAVLink entry points (ao_telemetry.cpp:282 radio, :1091 USB) all reach the HSM without touching the driver, and neither `phase_change_cb` nor any kIdle entry action repairs it; only the present-day blast radius is narrower than written, since GPIO 12/13 are documented bench pins not wired to pyro hardware (main.cpp:332-334).
- **Reconciled 2026-08-20:** duplicate of **CW-B37-01**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X5-02 -- gps_uart_reinit() is reachable from Core 1 but manipulates a UART IRQ that was enabled on Core 0
- Site: src/core1/sensor_core1.cpp:251-279 (call site) against src/drivers/gps_uart.cpp:354-382 and :448-481
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (a resource acquired in one context torn down and re-acquired in another) + JPL-C Rule 8 single owner; the file's own SPSC contract at gps_uart.cpp:132-149
- Claim: `core1_gps_staleness_check()` runs on Core 1 and calls `gps_uart_reinit()`, whose `irq_set_enabled()` calls act on the executing core's NVIC, so the disable at gps_uart.cpp:453 does not disable the interrupt Core 0 actually owns and the enable at :477 arms it on a second core.
- Why: `gps_uart_init()` is called from Core 0 (main.cpp:130 via init_gps) and enables `UART_IRQ_NUM(uart0)` on Core 0's NVIC (gps_uart.cpp:378). The Pico SDK's `irq_set_enabled` operates on the executing core only. When Core 1 later reinitialises the UART after a 10 s NMEA staleness (sensor_core1.cpp:275, reached only for `GPS_TRANSPORT_UART`, i.e. the vehicle Feather), two things break that no single file shows. First, during reinit Core 0's ISR is still armed while Core 1 executes `uart_deinit()` / `uart_init()` and `detect_gps_presence()`'s blocking `uart_getc()` -- the Core 0 handler can drain the same RX FIFO and write `g_rxBuf`/`g_rxHead` while Core 1 is zeroing them at :458-459 and re-running `lwgps_init()` at :462, and it can also touch `uart_get_hw()->dr` on a peripheral that Core 1 has just reset. Second, after :477 the UART IRQ is enabled on both cores' NVICs, so `gps_uart_rx_isr` can be taken on either core -- permanently violating the "Producer: gps_uart_rx_isr() on Core 0 ... single-producer" contract the file states at :132-136, on which the `volatile`-is-sufficient reasoning at :141-147 rests. Secondary: sensor_core1.cpp:252 documents this call as "Blocks up to 2s", but `acquire_at_target_baud()` can spend 2 s at 57600 plus 2 s at 9600 plus the 250 ms PMTK251 settle -- ~4.25 s of Core 1 sensor blackout.
- Confidence: high
- Direction: keep the UART's IRQ ownership on the core that installed it -- have Core 1 raise a request flag that Core 0 services in `qv_idle_bridge`, or move the whole reinit to the Core 0 idle path; either way disable the IRQ on the owning core before `uart_deinit()` and correct the blocking-duration comment.
- Verdict: RESHAPED -- the durable half holds (`irq_set_enabled` acts on the calling core's NVIC and `multicore_launch_core1` gives Core 1 the same vector table, so the enable at gps_uart.cpp:477 executed from Core 1 leaves `gps_uart_rx_isr` armed on BOTH cores and permanently breaks the single-producer contract stated at gps_uart.cpp:132-136; and `acquire_at_target_baud()`'s 2 s + 2 s + 250 ms worst case (kInitTimeoutUs=2000000 twice, kGpsBaudNegotiateDelayMs=250) does contradict "Blocks for up to 2s" in gps_uart.h:107 and sensor_core1.cpp:252). Narrower surviving claim: the mid-reinit corruption half is refuted -- `uart_set_irqs_enabled(GPS_UART_INST, false, false)` at :454 masks RXIM/RTIM in the UART's own core-agnostic IMSC before `uart_deinit()`, so Core 0's ISR cannot drain the FIFO or race `g_rxBuf`/`g_rxHead` during `detect_gps_presence()` beyond at most one already-pending latched IRQ.
- **Reconciled 2026-08-20:** duplicate of **CW-B09-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X5-03 -- init_gps_early() inverts the documented GPS init ordering and the UART-first transport policy on every board
- Site: src/main.cpp:217-227 against src/main.cpp:123-150 and src/main.cpp:152-161
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (mis-ordered init across subsystems) + CCG NL.2 (comment and code disagree) / JSF AV 134 (ordering assumption stated but not honoured)
- Claim: `init_gps_early()` runs a full PA1010D I2C bring-up before the IMU exists and unconditionally on all boards, contradicting both ordering rules that `init_sensors()` and `init_gps()` state a few lines above it.
- Why: `init_sensors()` states the constraint at main.cpp:157-161 -- "Init order matters: IMU + baro FIRST, GPS LAST ... Probing the GPS (0x10) triggers NMEA streaming which can corrupt AK09916 init transactions. Defer GPS probe until after IMU bypass mode is fully established." But `init_early_hw()` calls `init_gps_early()` at main.cpp:238, well before `init_sensors()` at :299, and `gps_pa1010d_init()` (src/drivers/gps_pa1010d.cpp) issues an `i2c_bus_recover()`, three blind PMTK writes and up to eight 250-byte reads at 0x10 -- roughly 1.5 s of GPS traffic on the bus the AK09916 at 0x0C will later share through the ICM-20948's bypass bridge. The stated ordering constraint is therefore never satisfied on any board that has a PA1010D attached. Second, `init_gps()` documents "UART first ... I2C fallback ... UART GPS has no I2C bus contention (LL Entry 24), preferred for production" (main.cpp:124-127), but `init_gps_early()` has no `board::kUartGpsAvailable` guard, and `init_sensors()` only calls `init_gps()` when `!g_gpsInitialized` (main.cpp:185). On the flight Feather, where `kUartGpsAvailable` is true (include/rocketchip/board_feather_rp2350.h:65), a PA1010D present on the Qwiic chain wins the race and the preferred UART backend is never bound. The only file that records the intent -- src/station/station_idle_tick.cpp:9-11 -- describes this path as "station Fruit Jam: ultra-early in init_early_hw()", i.e. a Fruit-Jam-specific workaround that shipped ungated.
- Confidence: high
- Direction: gate `init_gps_early()` on `if constexpr (!board::kUartGpsAvailable)` so only the boards that have no UART GPS take the early I2C window, and either satisfy or retract the IMU-before-GPS ordering statement in `init_sensors()` now that a legitimate earlier GPS path exists.
- Verdict: CONFIRMED -- `init_gps_early()` (main.cpp:219-227) is called unconditionally from `init_early_hw()` at :238 with no `board::kUartGpsAvailable` guard, and `gps_pa1010d_init()` does issue `i2c_bus_recover()` (gps_pa1010d.cpp:211), three blind PMTK writes (:222-232) and up to eight 250-byte reads (:241-242) at 0x10 long before `init_sensors()` (:299) establishes IMU bypass mode, while `if (!g_gpsInitialized) init_gps();` at :185 makes a successful early I2C bind pre-empt the UART-first preference documented at :123-127 on the Feather (kUartGpsAvailable = true, board_feather_rp2350.h:65).

### CW-X5-04 -- the boot-time PSRAM flash-safe test runs an unprotected flash_safe_execute after Core 1's sensor loop is already driving I2C
- Site: src/main.cpp:389-398 against src/safety/core1_i2c_pause.h:6-28 and src/logging/psram_init.cpp:318-347
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (ordering dependency between subsystems at boot) + CCG P.8 / JPL-C Rule 8 on the documented pause-flash-reset protocol
- Claim: `init_application()` calls `psram_flash_safe_test()` immediately after `init_core1_role()` has released Core 1 into its sensor loop, without the `core1_i2c_pause()` / `i2c_bus_reset()` / `core1_i2c_resume()` sequence that core1_i2c_pause.h declares mandatory around every reachable flash_safe_execute callsite.
- Why: `init_core1_role()` (main.cpp:345-361) sets `g_sensorPhaseActive = true` and stores `g_startSensorPhase`, which releases Core 1 from its wait loop into `core1_sensor_loop()` (sensor_core1.cpp:495-523) where it begins ~1 kHz `icm20948_read()` traffic; it then only waits on `g_core1LockoutReady`, which Core 1 had already set before that wait (sensor_core1.cpp:476-477), so the wait returns essentially immediately. Core 0 then executes `psram_flash_safe_test()` at main.cpp:395, which calls `flash_safe_execute(do_flash_erase, ...)` at psram_init.cpp:344. That is exactly the LL-31 window core1_i2c_pause.h:8-15 describes: `multicore_lockout` halts Core 1's CPU but does not drain the DW_apb_i2c transaction already in flight, which is then abandoned at the APB bridge timeout and leaves the peripheral corrupt. Neither the preventive pause nor the recovery `i2c_bus_reset()` is present. The next statement, `init_baro_auto_zero()` (main.cpp:398), spins in `while (calibration_is_active())` waiting on baro samples that only Core 1's I2C reads can produce -- so a corrupted peripheral here stalls boot at the auto-zero rather than failing visibly. The three other reachable flash chains (rc_os_commands.cpp:1042 and :1098, ao_rcos.cpp:343) all do wrap; this one is the outlier, and it is invisible from psram_init.cpp, which cannot know when Core 1 was released.
- Confidence: high
- Direction: either move the flash-safe test above `init_core1_role()` (it only needs Core 1 registered as a lockout victim, which happens before the sensor phase starts) or wrap it in the same pause / reset / resume triad the other callsites use.
- Verdict: CONFIRMED -- `init_core1_role()` sets `g_startSensorPhase` (main.cpp:349) then waits only on `g_core1LockoutReady`, which Core 1 already stored at sensor_core1.cpp:476-477 before its own start-flag wait, so Core 0 falls straight through to `psram_flash_safe_test()` -> `flash_safe_execute(do_flash_erase, ...)` (psram_init.cpp:344) as Core 1 enters `core1_sensor_loop()`; the tree-wide grep shows no `core1_i2c_pause()` and -- independently of the race's timing -- no post-flash `i2c_bus_reset()` here, while every other reachable callsite (rc_os_commands.cpp:1042/:1098, ao_rcos.cpp:343) applies both, and the main.cpp:302-303 comment documents only why the test is deferred, not an exemption from the protocol.

### CW-X5-05 -- psram_flash_safe_test() is documented as a hard gate but its verdict gates nothing; the flight log ring is placed in PSRAM on the weaker self-test alone
- Site: src/logging/psram_init.h:76-84 and src/main.cpp:394-396 against src/active_objects/ao_logger.cpp:251-277 and src/active_objects/ao_logger.cpp:389-394
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (a resource brought up in one module and committed to by another) + NIST AI 600-1 confabulation / CCG NL.2 -- a confident stated contract the code does not implement
- Claim: `g_psramFlashSafePassed` is computed at main.cpp:395 and consumed nowhere except a CLI status print, so the "Council req. #2 (hard gate)" the header claims does not exist; `init_logging_ring()` commits the flight log ring to PSRAM on `g_psramSelfTestPassed` alone.
- Why: psram_init.h:79-81 states the test's purpose as a hard gate validating that PSRAM contents survive a `flash_safe_execute()` erase/program cycle -- precisely the operation `flush_ring_to_flash()` performs many times while the ring lives in PSRAM. But `main()` passes only `g_psramSize` and `g_psramSelfTestPassed` to `AO_Logger_start` (main.cpp:513), `init_logging_ring()` branches on those two at ao_logger.cpp:259, and the only reader of `g_psramFlashSafePassed` in the whole tree is the status line at src/cli/rc_os_commands.cpp:664. A board whose PSRAM passes the pattern self-test but fails the flash-safe test therefore still gets an 8 MB PSRAM-backed ring at 50 Hz decimation, and every flush corrupts the frames still buffered ahead of it -- silently, because the flush reports success. Additionally, because the test is only run when `g_psramSize > 0 && g_psramSelfTestPassed` (main.cpp:394), a false verdict is indistinguishable from "never run" in the CLI print at rc_os_commands.cpp:666-668. No single file shows this: psram_init.h states the gate, main.cpp computes it, ao_logger.cpp makes the decision without it.
- Confidence: high
- Direction: pass the flash-safe verdict into `AO_Logger_start` and require it in `init_logging_ring()`'s PSRAM branch (falling back to the SRAM ring when it fails), or downgrade the header's "hard gate" wording to "diagnostic" so the claim matches the code.
- Verdict: CONFIRMED -- tree-wide grep shows `g_psramFlashSafePassed` written only at main.cpp:395 and read only at rc_os_commands.cpp:664, `AO_Logger_start(4U, g_psramSize, g_psramSelfTestPassed)` at main.cpp:513 never carries the verdict, and `init_logging_ring()` branches on `g_loggerPsramSize > 0 && g_loggerPsramSelfTestPassed` alone (ao_logger.cpp:259), so psram_init.h:79-81's "Council req. #2 (hard gate)" gates nothing.

### CW-X5-06 -- backup-timer disarm hands the pyro pins to SIO but nothing ever gives SIO an output direction, so the pins float instead of being driven LOW
- Site: src/safety/pio_backup_timer.cpp:131-141 and src/safety/pio_backup_timer.cpp:108-113
- Lens: spine block C "Functionally-correct-but-safety-blind hardware code" plus "Peripheral init SEQUENCE / lifecycle" (a pin whose direction is owned by one peripheral and handed to another that was never configured); CCG NL.2 -- comment asserts a behaviour the body does not produce
- Claim: `pio_backup_timer_disarm()` and `pio_backup_timer_cancel()` switch the pyro pins to `GPIO_FUNC_SIO` and call `gpio_put(pin, 0)`, but no code in the tree ever calls `gpio_set_dir()` on those pins, so SIO's output enable stays at its reset value (input) and the pads are left high-impedance rather than actively driven low.
- Why: the pins' output direction is established only through the PIO path -- `backup_timer_program_init()` calls `pio_gpio_init()` and `pio_sm_set_consecutive_pindirs(..., true)`, which drives the pad OE from the PIO state machine. On RP2350 the pad's output enable is selected by the current funcsel, so once `gpio_set_function(pin, GPIO_FUNC_SIO)` runs at pio_backup_timer.cpp:135 and :139 the OE comes from SIO's `GPIO_OE` register, which nothing in src/ or include/ ever sets for GPIO 12/13 (`gpio_set_dir` appears only in i2c_bus.cpp, rfm95w.cpp, main.cpp:233 for the LED, and board_fruit_jam.h:74). `gpio_put()` writes the output data register but has no electrical effect while OE is 0. The comment at :131-133 states the safety intent -- "Return pins to SIO control, drive LOW. Visible safety signal: pin can't be driven HIGH by a stale SM after disarm" -- and the PIO program header states the same contract ("PIO owns the pin via FUNCSEL when armed; software owns when disarmed"). Only the first half holds: the SM genuinely cannot drive the pin, but neither does software, so a disarmed pyro gate line sits floating at whatever the pad's default pull gives it. `pio_backup_timer_fired()` at :153-162 reads that same floating pad through `gpio_get()`. The pins are currently bench-only per main.cpp:334, which bounds today's consequence but not the contract's.
- Confidence: medium
- Direction: give the two pins an explicit SIO output direction once (at `pio_backup_timer_init`, alongside the PIO setup) so the disarm/cancel path actually drives them low, and make `pio_backup_timer_fired()` state which owner it is sampling.
- Verdict: CONFIRMED -- the pins' output enable is established only through `pio_gpio_init()` + `pio_sm_set_consecutive_pindirs(..., true)` in `backup_timer_program_init()` (pio/backup_timer.pio:66-67), `pyro_edge_logger_init()` only enables edge IRQs and never touches direction, and `gpio_set_dir` appears tree-wide only in i2c_bus.cpp, rfm95w.cpp, main.cpp:233 and board_fruit_jam.h:74 -- never for GPIO 12/13 -- so after `gpio_set_function(pin, GPIO_FUNC_SIO)` the pad's OE comes from an SIO GPIO_OE bit nothing ever sets and `gpio_put(pin, 0)` is electrically inert; the finding's own hedge is right that the pad's reset pull-down, not the code, is what holds the line low.
- **Reconciled 2026-08-20:** duplicate of **CW-B35-03**, which states the same proposition from the file-by-file pass. Counts as **one** Claude row, not two — the lane restated it, it is not a second walk.

### CW-X5-07 -- the PIO backup-fire report latches are set once and never cleared on disarm or re-arm
- Site: src/active_objects/ao_flight_director.cpp:79-101 and src/active_objects/ao_flight_director.cpp:262-263, against src/safety/pio_backup_timer.cpp:66-98
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (arm/disarm symmetry across a module boundary) + CCG P.8 state-lifetime reasoning
- Claim: `pio_drogue_reported` / `pio_main_reported` are cleared only in `AO_FlightDirector_start()`, while the underlying timers are fully re-armable within one power cycle, so a backup deployment on the second and later arm cycles is never published or logged.
- Why: `pio_backup_timer_arm()` re-runs `backup_timer_program_init()` (pio_backup_timer.cpp:76-81), which resets the SM to its entry point and drives the pin low, then re-enables and re-loads the countdown -- so the timers are genuinely re-armable and LL Entry 42 records that multi-cycle ARM/RESET is an exercised bench sequence. But `fd_check_pio_backup()` guards each publish with `if (!me->pio_*_reported)` and sets the latch true on first detection; nothing clears it on `SIG_DISARM`/`SIG_RESET`, on kIdle entry, or in `pio_backup_timer_disarm()`. After a first cycle in which a backup timer fired, a second cycle's backup fire raises the pyro pin exactly as before, and the AO stays silent: no `SIG_PYRO_FIRED` is published, `AO_Logger` never records the event, `director.state.main_fired` is never set, and the flight log shows a deployment that the hardware performed. The defect is only visible by reading the AO's latch against the driver's re-arm semantics in the other file.
- Confidence: medium
- Direction: clear both latches wherever the timers are re-armed or disarmed -- simplest is to reset them in the same place `pio_backup_timer_arm()`/`pio_backup_timer_disarm()` are called, so latch lifetime is tied to the hardware's arm cycle rather than to AO construction.
- Verdict: CONFIRMED -- `pio_drogue_reported` / `pio_main_reported` are assigned false only at ao_flight_director.cpp:263-264 inside `AO_FlightDirector_start()` and set true in `fd_check_pio_backup()` (:82, :93), with no clear on SIG_DISARM/SIG_RESET, on kIdle entry, or in the driver, while `pio_backup_timer_arm()` genuinely re-arms (re-runs `backup_timer_program_init` to reset the SM to its entry point, then re-enables and re-loads the countdown, pio_backup_timer.cpp:76-97), so a second-cycle backup fire raises the pin with no publish, no log event and no `main_fired`.

### CW-X5-08 -- the flight-in-progress sentinel is set on ARM but cleared only on LANDED, so a disarm or abort leaves it set for the next boot's mid-flight verdict
- Site: src/flight_director/flight_director.cpp:54-67, consumed at src/safety/anomalous_boot.cpp:107-114 and src/main.cpp:366-387
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (acquire/release pairing across modules) + JSF AV 134 -- the header states a clear-side contract that does not cover every release path
- Claim: `enter_phase()` calls `flight_in_progress_set()` on kArmed entry and `flight_in_progress_clear()` only on kLanded entry, so every ARMED exit that is not a completed flight -- DISARM, the ARMED timeout, ABORT, RESET -- leaves the reset-surviving sentinel asserted.
- Why: the sentinel lives in `.uninitialized_data` and survives every non-power-cycling reset (watchdog, `NVIC_SystemReset` from `crash_record_capture`, a debug-probe reset, a BOOTSEL reflash). `anomalous_boot_init()` consumes it at anomalous_boot.cpp:109 and `compute_verdict()` treats it as sufficient on its own -- "Signal #1: sentinel was set. Alone triggers PROBABLY_MID_FLIGHT" (anomalous_boot.cpp:68-71). `init_baro_auto_zero()` (main.cpp:366-376) then skips `calibration_start_baro()` entirely on that verdict, so the vehicle boots with the previous ground reference instead of a fresh one, and main deploy is computed against a stale datum. The reachable sequence is ordinary bench and range practice: ARM on the pad, scrub, DISARM (or let the 300 s ARMED timeout fire, or ABORT then RESET), then any non-POR reset. Both `crash_record.h:123-124` and the code comment at flight_director.cpp:61-63 present LANDED as the clear point, which is true for a completed flight and silently incomplete for every scrubbed one. Neither file alone shows it: flight_director.cpp sets and clears without seeing the consumer, anomalous_boot.cpp reads a flag whose set/clear symmetry it cannot check.
- Confidence: medium
- Direction: clear the sentinel on entry to kIdle as well as kLanded (or on any transition out of the armed/airborne subtree that is not a crash), and update the crash_record.h contract text to name every clear point.
- Verdict: RESHAPED -- the finding missed a documented decision: docs/decisions/FAULT_RECOVERY_2026-05-14.md "kAbort invariant -- flight-in-progress sentinel" states the clear is deliberately LANDED-only and explicitly NOT done on kAbort/kFault, because an aborted flight still requires operator inspection, so the ABORT and RESET-from-abort half is a decision rather than a defect. Narrower surviving claim: the never-launched scrub exits (SIG_DISARM from ARMED, the armed-timeout `Q_TRAN(&state_idle)`) are outside that rationale, and the decision doc's stated operator escape ("manual CLI `r` in kIdle") clears nothing -- `flight_in_progress_clear()` has exactly one callsite, `enter_phase(kLanded)` (flight_director.cpp:65). The consequence is also narrower: `flight_in_progress_was_set()` clears on read (flight_in_progress.cpp:40-45), so the sentinel suppresses baro auto-zero for exactly one subsequent non-POR boot, with an explanatory DBG_PRINT at main.cpp:377-387.

### CW-X5-09 -- rfm95w_read_version() is documented as usable before rfm95w_init(), but the CS pin only becomes an output inside rfm95w_init()
- Site: src/drivers/rfm95w.h:165-180 against src/drivers/rfm95w.cpp:100-104, src/drivers/rfm95w.cpp:177 and src/drivers/spi_bus.cpp:52-62
- Lens: spine block C "Peripheral init SEQUENCE / lifecycle" (two-phase init contract, JSF AV 143) + CCG NL.2 -- a documented precondition the implementation cannot satisfy
- Claim: the header states `rfm95w_read_version()` "Does NOT require rfm95w_init() to have been called first -- uses the raw SPI bus", but chip-select assertion depends on GPIO configuration performed only by `rfm95w_init()`, so the documented pre-init use reads an unasserted bus.
- Why: `spi_bus_init()` configures MISO, SCK and MOSI only (spi_bus.cpp:39-41); no CS pin is touched anywhere else in the tree. `init_gpio_and_reset()` (rfm95w.cpp:100-104), reached only from `rfm95w_init()` at rfm95w.cpp:177, is the sole place the CS pin gets `gpio_init` + `gpio_set_dir(GPIO_OUT)` + idle-high. Until that runs the pad's funcsel is unassigned and its output enable is 0, so `gpio_put(cs_pin, 0)` in `spi_bus_read_reg()` (spi_bus.cpp:56) leaves CS floating and the SX1276 never sees it asserted -- the read returns 0x00/0xFF, which is exactly the value the header tells the reader to interpret as "SPI line is dead or no chip present". The header even scripts the failing use directly: "(gdb) call rfm95w_read_version(10)". The same trap is live in firmware whenever the radio was not initialised: `AO_Radio` skips `rfm95w_init()` when `g_spiOk` is false, and `diag_stats_t0_preconditions()` (src/diag/diag_stats.cpp:44-48) then prints a T=0 "is the radio physically reachable" verdict derived from an unasserted chip select. Nothing in rfm95w.h shows that CS setup is init-only; nothing in spi_bus.cpp shows that CS is somebody else's job.
- Confidence: medium
- Direction: either configure the radio CS pin in `spi_bus_init()` (or a small `rfm95w_prepare_cs(cs)` the diagnostic can call) so the raw-read path is genuinely init-independent, or correct the header to state that `rfm95w_init()` must have run and have the diagnostic report "not initialised" instead of a register value.
- Verdict: RESHAPED -- the header/implementation mismatch is real (`init_gpio_and_reset` at rfm95w.cpp:100-104, reached only from `rfm95w_init` at :177, is the sole place any CS pin becomes an output; `spi_bus_init()` configures MISO/SCK/MOSI only, spi_bus.cpp:39-41), so rfm95w.h:172-176's "Does NOT require rfm95w_init()" and its GDB recipe are unsatisfiable at T=0. Narrower surviving claim: the in-firmware extension is refuted -- `spi_bus_init()` returns true unconditionally (spi_bus.cpp:43), so `AO_Radio_start(8U, g_spiInitialized)` (main.cpp:491) always has `g_spiOk == true` and `radio_ao_initial` always calls `rfm95w_init()` (ao_radio.cpp:540), hence `init_gpio_and_reset()`, before any CLI-reachable `diag_stats_t0_preconditions()`; and the predicted 0x00/0xFF is not certain either, since the pad's reset pull-down can hold an active-low CS asserted.

---

## Completeness critic — what this pass missed

**What this is:** an independent blind read of the finished L2-P5 claude_walk result set (44 batch
files `B01`-`B44`, 5 cross-cut lanes `X1`-`X5`) against the field manual
`L2P5_MANUAL_WALK_GUIDE.md` and the route `L2P5_WALK_ITINERARY.md`. It asks one question only:
**what did this pass miss?** It is a critique of the review, not of the firmware. No source file was
re-reviewed and no new code finding is filed here.

**Corpus measured:** 311 batch findings + 47 lane findings = 358. Every finding carries a
`- Verdict:` line (236 CONFIRMED / 56 RESHAPED / 19 REFUTED in the batches; 32 CONFIRMED /
12 RESHAPED / 3 REFUTED in the lanes). 187 unique source paths appear in the Coverage sections,
matching the itinerary's in-scope set — **file coverage is complete; lens coverage is not.**

---

## 1. The headline: eight lenses were commissioned, one lens ran

Classifying every `- Lens:` line in the 44 batch files by the Class-index lens it names first:

| Lens (guide Class index) | Batch findings | Share |
|---|---|---|
| Comments & documentation quality | **168** | 54% |
| The spine — block B (AI-distrust types) | 33 | 11% |
| The spine — block A (function shape / altitude) | ~24 | 8% |
| Assertions | 20 | 6% |
| The spine — block C (embedded ADD) | ~19 | 6% |
| Concurrency & shared-data ownership | 13 | 4% |
| Declaration scope & object lifetime | 11 | 4% |
| Class & interface design | **6** | 2% |
| volatile / control-flow discipline | **2** | 0.6% |
| Templates | **2** | 0.6% |

The rule-ID histogram says the same thing more bluntly. Three IDs carry the corpus —
`JSF AV 134` (94 citations), `CERT MSC12-C` (55), `JSF AV 131` (46), with `CCG NL.2` cited 132
times. Everything else is a long tail.

This is a **documentation-truth audit wearing a semantic-walk label.** It is a good documentation
audit — the comment-vs-code findings are precise, line-cited, and mostly survive the verify pass.
But the guide commissioned a gestalt code review with comments as *one* of eight lenses, and the
result set cannot tell the owner whether the other seven were applied and came out clean or were
never pointed at the file.

**Structural cause: absence of a lens is indistinguishable from a lens PASS.** Only 8 of 44 batches
(`B05`, `B10`, `B17`, `B31`, `B33`, `B35`, `B37`, `B43`) record an explicit "this lens ran and was
clean" note. In the other 36, the only evidence a lens was applied is a finding filed under it. The
guide's completeness principle ("record every file, PASS included") was honored at the *file*
granularity and dropped at the *lens* granularity — which is where it mattered.

---

## 2. Lenses under-applied or absent, with the batches that owed them

### 2.1 Class & interface design — 6 findings, and the pattern the guide calls dominant produced zero

The guide states the dominant passing-but-improper pattern for this lens is **encapsulation
theater** (CCG C.131 / Fowler "Data Class"). Across all 49 result files:

- `C.131` — **0 citations.** `"Data Class"` — 0. `"encapsulation theater/theatre"` — 1, and it is a
  clean-pass aside in `B10`'s coverage note for `icm20948.h`, not a walked judgment.
- The lens's binding house standard is JSF AV 67/68/72/76/78/79/87/88/177. Actual citations:
  AV 67 x2 (`CW-B04-07`, `CW-B12-02`), AV 72 x2, AV 87/88 x1 (`CW-X4-11`). **AV 68, AV 76, AV 79 and
  AV 177 — zero.**
- `CERT OOP50-CPP` (virtual call from ctor/dtor), `OOP52` (virtual-dtor disposition),
  `OOP58` (const-correct copy) — **zero citations.**
- `CCG C.8` (keyword honesty), `C.21` (rule-of-five disposition), `C.46` (explicit-ctor intent),
  `I.25` (thin interfaces) — zero, except `I.25` once in `CW-X4-11`.

The guide names this lens primary for "the QP/C AOs in `active_objects/`, the driver classes in
`drivers/`, the calibration and logging types, the small value types in `math/`". Those are:

- **`B08`, `B09`, `B10`, `B11`, `B12`** — the eight driver modules. One class-design finding total
  (`CW-B12-02`). Nothing on `rfm95w`, `i2c_bus`, `spi_bus`, `gps_uart`, `baro_dps310` — each a
  resource-owning handle type where AV 76 / AV 79 / C.21 are the live questions.
- **`B37`, `B38`, `B39`, `B40`** — the eight AOs. One finding (`CW-B37-02`), and it is about a
  dispatch mechanism, not a type. `B38` and `B39` contain no class-design finding and no
  class-design clean note.
- **`B17`, `B18`, `B19`** — the calibration types. Zero.
- **`B25`-`B28`** — the logging types (`RingBuffer`, `FlightTable`, `LogDecimator`, `PcmFrame`).
  Zero, despite `RingBuffer` being a two-phase-init resource-owning class (`CW-B25-05` files it
  under scope/lifetime instead).
- **`B07`** — `math/vec3.h`, `quat.h`, `mat.h`. `B07`'s coverage note reaches the right verdict
  ("struct-of-public-data shape is correct per CCG C.2") but for one file only.

`X4` is the closest thing to a class-design lane, and its 12 findings are almost entirely
contract-prose and single-source-of-truth — the Comments lens again, at system scale.

### 2.2 volatile / control-flow discipline — one half of the lens never ran at all

Two findings in 44 batches: `CW-B14-05` and `CW-B25-04`. Both are on the `volatile`/ordering half.

The **evaluation-order half produced nothing anywhere and left no trace of having been considered**:
`JSF AV 204` / `204.1` — 0 citations. `JPL-C Rule 18` — 0. `CCG ES.43` — 1 incidental mention in
`B39`. `CCG ES.44` — 0. `CERT EXP50-CPP` — 0. The guide scopes this to "every `.cpp` carrying terse
arithmetic or multi-argument call sites (fusion math, drivers, `math/`)" — i.e. `B07`, `B08`-`B12`,
`B13`-`B16`. Not one of those nine batches records an eval-order judgment, clean or otherwise.

The `volatile` half fared better than the count suggests because much of it was absorbed into the
Concurrency lens and into `X1`. But the itinerary attached an explicit
`-> concurrency 3-question test` hot-spot cue to 14 rows, and **several of those batches contain no
volatile/atomic/barrier judgment at all**:

- **`B05`** walks `sensor_seqlock.h` and `sensor_snapshot.h` — the guide's canonical
  volatile/control-flow site and a named concurrency site — and **the word `volatile` does not
  appear in the batch file once.** The seqlock's barrier argument gets one clause in a coverage note
  ("the barriers are conservatively correct") and nothing else; the 11 of 14 findings that follow
  are Comments-lens findings about the seqlock's contract *prose* (`CW-B05-01`, `CW-B05-04`).
- **`B08`** (spi_bus, cue "1 atomic error counter"): no `volatile` mention; the atomic is disposed of
  in a single coverage clause.
- **`B42`** (rc_os, cue "1 atomic mag-cal-active flag"): no `volatile` mention; one coverage clause.
- **`B33`** (fault_inject + station_fault_inject + test_mode, cues totalling seven volatiles): all
  six findings are Comments-lens; the volatiles are named only as "two extern volatile flags" in a
  coverage line, never classified true-MMIO vs cross-context-sync.
- **`B20`** and **`B31`** did the work but filed it elsewhere — `CW-B20-09` under the scope lens,
  and `B31` under "Spine C" with the explicit admission *"detailed criteria live in the Concurrency
  lens, which this batch does not carry."*

That admission is the diagnosis: the batch kit distributed lenses per batch, while the Class index
distributes them per subsystem. Where the two disagreed, the batch kit won.

### 2.3 Templates — 2 findings, both from one batch; the second named site was not walked

`CW-B19-01` (JSF AV 101) and `CW-B19-02` (AV 103), both in `calibration/lm_solver`. The guide names
the template sites as "`calibration/lm_solver`, `math/`, header utilities."

- **`B07` (`math/mat.h`) produced no template finding and no template clean note**, despite `mat.h`
  being — per `B07`'s own `CW-B07-06` — an entirely header-resident templated matrix library.
  `CCG T.10/T.11/T.20/T.41/T.47/T.61/T.69/T.120/T.143` — zero citations tree-wide. `T.150` appears
  once, in `B11`, unrelated to a template.
- **`JSF AV 102`** (tests cover every actual instantiation) — zero. The guide explicitly instructs
  *"currently ungated, so flag it to wire/grep — don't eyeball."* Nobody flagged it. AV 105
  (instantiation-context dependence) and AV 106 (pointer specializations) also got no citation.

### 2.4 Assertions — ran on roughly a quarter of the tree

20 findings, of which **10 cite `CCG P.5`** — half the lens's output is "lift this to
`static_assert`", the compile-time-preference half, and only about half is the guide's stated
high-value catch (*"name the load-bearing precondition it is actually missing"*). Exactly one
finding (`CW-B13-08`, the meaningful-vs-vacuous half of P10-5 / JPL-16) judges an *existing*
assertion; the guide's "decorative density-padding" agent-tendency was never tested.

Six batches never contain the string `assert`: **`B04`, `B11`, `B18`, `B39`, `B40`, `B44`.** The
guide scopes this lens to "the longer, load-bearing functions of safety/, fusion/,
flight_director/, core1/, and **drivers/**." Consequently:

- **`B11`** — `rfm95w.cpp`, 16 functions of register sequencing and timing: zero assertion judgment.
- **`B39`** — `ao_radio.cpp` at 807 lines (the batch's own count): zero.
- **`B40`** — `ao_telemetry.cpp` at 1133 lines: zero.
- **`B36`** — `core1/sensor_core1.cpp`, named by the guide: zero.
- **`B25`-`B28`** (logging), **`B41`** (`main.cpp`), **`B08`**-**`B10`** (bus + IMU/baro drivers):
  all zero.

`B33` is the model the other 43 should have followed: *"Assertions (P10-5 / JPL-16): zero assertions
across all six files, but every precondition in this batch is already expressed as an explicit
fail-closed branch..."* One sentence turns a silence into a recorded verdict. It appears once.

### 2.5 Declaration scope & object lifetime — 11 findings, of which 5 are really concurrency

`CW-B10-03`, `CW-B12-07`, `CW-B18-01`, `CW-B20-09` and `CW-B29-03` all cite `CCG CP.2` "avoid data
races" — concurrency findings routed here because CP.2 appears in this lens's bullet list. The
scope/lifetime lens *proper* yielded about six findings tree-wide.

The guide names three canonical cases. None produced a recorded verdict:

- **Stack-local QP event posting (CCG P.8, the guide's "canonical case on a QP/C codebase").** The
  Class index names `active_objects/ao_led_engine`, and the itinerary row repeats it:
  *"(LL 35 stack-local event history — scope/lifetime lens)"*. **`AO_LedEngine_post_pattern` and
  `LedPatternEvt` are not named anywhere in the 49 result files.** `B40`'s coverage note for
  `ao_led_engine.cpp` lists three findings (dedup cache, vitality fallback, dev override) and never
  mentions the event-storage check — while the *same* batch records it for `ao_notify.cpp`
  ("static-event posts verified against LL Entry 35") and `B43` records it for `cmd_findme_beacon`.
  The reviewers knew the criterion; the one file the guide points at is the one with no record.
- **Leak-on-unhappy-path / acquire-then-return-early (CCG P.8).** Zero findings. `CCG CP.20` appears
  once (`CW-B34-02`); `JPL-C Rule 9` once; `CERT CON51` zero.
- **"Mandated wide scope is not a violation."** The guide's explicit instruction to *confirm* whether
  a large file-scope static's width is mandated. No batch records that confirmation for any of the
  tree's large statics.

Also absent: `ES.21`/`ES.22` declaration-distance (1 finding, `CW-B11-04`), `ES.6` (0), and — a
named spine-A criterion — **`CCG F.56` guard clauses / unnecessary nesting: zero mentions across 49
files and 186 source files.**

### 2.6 Spine block C — the untrusted-input residual

The guide's DROP-with-residual is explicit: *"any **CLI / UART / USB byte-stream parser** is an
untrusted-input sink — treat LLM-written parsing of a serial frame or command token with zero
trust"* (OWASP LLM05:2025). `OWASP` / `LLM05` / `untrusted` appear in exactly two files: **`B05`**
and **`B30`**. The ingress points that owed this lens and did not get it:

- **`B09`** — NMEA sentence parsing over I2C and UART, the archetypal untrusted byte stream.
- **`B42`, `B43`, `B44`** — the CLI command-token surface, named verbatim in the guide.
- **`B28`** — `pcm_frame` / `radio_config_storage` record decode from flash.
- **`B11`** — LoRa RX payload handling.

Similarly, the guide keeps package hallucination **HIGH for host-side Python tooling**. `B24` and
`B16` read `scripts/generate_profile.py` and `scripts/generate_fpft.py` as *reference artifacts* to
diff generated output against, finding real defects that way (`CW-B24-02`, `CW-B16-02`) — but
neither script got a coverage row or a lens applied. That is a defensible scope call (the itinerary
is `src/` + `include/`), but the guide's block C names the surface, so it owed an explicit
`N/A — out of itinerary scope` line rather than silence.

---

## 3. Where coverage reads thin

The batch coverage notes are, on the whole, not skimmed. Nearly every one names functions, line
numbers, consumers and the specific claim checked, and several report file line counts. The
thinness is not in the prose; it is in the **shape of the output**, which reads as budget-driven
rather than content-driven.

**Findings per batch cluster tightly at 3-10 (mean 7.1), and file size at 9-27 KB, regardless of
what the batch covers.** `B19` (2 files, `lm_solver.{cpp,h}`) yields 6 findings; `B43`
(`rc_os_commands`, the largest CLI module) yields 10; `B29` (10 files) yields 6; `B36`
(`sensor_core1`, the tree's highest-consequence concurrency file) yields 4. A defect-density-driven
pass does not produce a distribution that flat.

Batches where the note gives the skim away:

- **`B33`** (safety fault injection + `test_mode`, 6 files): all 6 findings Comments-lens. The
  itinerary attached three separate volatile hot-spot cues to these files (2 + 2 + 3 volatiles). The
  batch disposes of assertions honestly and says nothing about the seven volatiles it was routed
  here to classify.
- **`B03`** (job/role pack, 5 headers): 3 findings, all Comments-lens. To its credit it carries a
  "Checked and found sound (recorded so the coverage is honest, not just the defect list)" block —
  one of only two batches to do so — but the class-design lens the Class index assigns to
  `include/rocketchip/` headers is not among the things checked.
- **`B39`** (`ao_radio` 807 lines + `ao_rf_manager`, 9 files, 45 line citations): 2 spine-B + 4
  Comments. No concurrency finding, no class-design finding, no assertion judgment — on a pair of
  AOs that own a peripheral, a retry state machine and a TX window.
- **`B29`** (diag + notify backends): 27 line citations across 10 files, the lowest density in the
  corpus. Five of six findings are comment-truth. The `notify_backend` function-pointer dispatch
  surface — an `I.25` / `AV 87-88` question, and a P10-9 question the tree carries elsewhere
  (`CW-B01-07`) — is not judged.
- **`B44`** (`rc_os_dashboard` 482 lines + `rc_os_debug`, 11 files): 9 findings, none under
  assertions, class design, scope, control-flow or concurrency.
- **`B05`** — high yield (14 findings) but 11 Comments-lens, on the batch whose two headline files
  are the guide's named sites for *both* the control-flow and the concurrency lens (see 2.2).

---

## 4. Findings that rest on a claim nobody verified against the thing it references

### 4.1 Hardware claims verified against project code instead of the datasheet sitting in the repo

`docs/hardware/datasheets/` in the working root contains `rp2350-datasheet.pdf`,
`SX1276-datasheet.pdf`, `PA1010D-datasheet-v03.pdf`, `PA1010D-NMEA-over-I2C-appnote.pdf`,
`DPS310-datasheet.pdf` and `ICM-20948-datasheet-v1.3.pdf`.

**Exactly one batch opened any of them.** `B10` cites `DPS310-datasheet.pdf` p. 29 for `CW-B10-05`
and `ICM-20948-datasheet-v1.3.pdf` pp. 10-11/13/33/44/58/63/78 in its `icm20948.cpp` coverage note.
That is the guide's block-C "HW-register / MMIO fidelity" criterion executed properly, and it is the
only instance in 49 files. Elsewhere:

- **`CW-B09-07`** (PMTK314 field order: "the comment claims GSV is enabled; the literal disables
  it"). Its own Confidence line says *"the GlobalTop command spec is not in this checkout, so treat
  that field mapping as the one assumption to re-confirm against the primary source."* Two PA1010D
  documents **are** in this checkout. The Verdict is nonetheless `CONFIRMED`, restating the same
  unverified field ordering as its evidence. A self-declared assumption was promoted to a confirmed
  finding by the verify pass rather than resolved by it.
- **`CW-B11-03`** (RegModemConfig3 / low-data-rate-optimize bound): *"medium on the precise hardware
  consequence, which I did not re-verify against the SX1276 datasheet."* `SX1276-datasheet.pdf` is
  in the repo.
- **`CW-B26-04`** (PSRAM self-test reads back through the cached alias). The load-bearing premise —
  that the XIP alias over PSRAM is write-back and can satisfy a readback with no QMI transaction —
  is argued from *the existence of `xip_cache_clean_all` in a sibling file* and from a sibling test
  using the uncached alias. That is inference from project code about silicon behavior: the exact
  LL-38 "code shows current-state, primary sources show possibility-space" trap.
  `rp2350-datasheet.pdf` is in the repo; the verdict repeats the same proxy evidence rather than
  resolving it.
- **`CW-B02-05`** (`board_release_peripheral_reset` momentary-LOW): *"depends on `gpio_init` clearing
  the output latch, which I could not verify against the SDK inside the working root."* Honest, and
  the finding is correctly narrowed to the documentation gap. This one is handled well and is the
  model the three above should have followed.

### 4.2 The verify pass is not uniform across the corpus

`X3` carries **two independent verdict lines per finding** ("independent adversarial
re-verification"). No other lane and no batch does. The corpus therefore mixes single-pass and
double-pass verification with no marker distinguishing them — and 24% of batch findings (75 of 311)
did not survive even their single verify pass as filed. A reader cannot tell which of the 236
CONFIRMED verdicts were checked by reasoning independent of the reasoning that produced them.

---

## 5. Contradictions left standing between the two axes

**There are zero cross-references between the batch axis and the lane axis** — no `CW-X..` ID
appears in any `B..` file, and no `CW-B..` ID appears in any `X..` file. Nothing merged, deduplicated
or reconciled them. At least ten defects are filed two or three times: rc_log drop-oldest
(`CW-B01-03` / `CW-X3-02` / `CW-X4-02`); the fault-handler logging contract (`CW-B25-01` /
`CW-X3-03` / `CW-X4-03`); `board_pico2.h` missing `kPsramCsPin` (`CW-B02-02` / `CW-X4-11`);
RC_ASSERT (`CW-B01-05` / `CW-X4-08`); the LED cross-core write (`CW-B36-04` / `CW-X1-07`); the
calibration cross-core race (`CW-B36-02` / `CW-X1-03`); the UART RX ring (`CW-B09-04` / `CW-X1-04`);
the second barometric-altitude copy (`CW-B10-06` / `CW-X2-06`). Duplication across a two-axis design
is expected; leaving it unmarked pushes the merge onto the owner.

**Worse, one cluster is not duplicated but contradictory — and it is the flight-critical one.**

`X1` determined that `core1_gps_staleness_check()`'s recovery branch (`sensor_core1.cpp:259-279`) is
**dead code**: `gps_uart_update()` can return false only when `!g_initialized`, which is cleared
only inside `gps_uart_reinit()` itself, so `parsed` is always true and the function returns at
`:255`. On that basis `X1` **REFUTED three of its own findings** (`CW-X1-01`, `CW-X1-02`,
`CW-X1-06`) and refuted half of `CW-X1-05`.

The same branch is the entire basis of:

- **`CW-B36-01`** — filed RESHAPED, confidence **high**, describing an unsynchronized cross-core read
  of `g_eskf.v` as a live ownership violation on a load-bearing gate.
- **`CW-B36-03`** — filed RESHAPED with *"the vehicle consequence is live"*: a ~4.25 s Core-1
  blackout annunciated as a Core-1 stall.
- **`CW-X5-02`** — titled *"gps_uart_reinit() is reachable from Core 1"*, RESHAPED, with the
  reachability premise explicitly retained in the surviving claim.

Three result files hand the owner three mutually exclusive answers about whether the vehicle's GPS
recovery path executes at all — and the dispositions diverge completely (fix a flight-path race /
delete dead code / re-architect UART IRQ ownership). Nothing in the result set flags the
disagreement. This is the single most consequential defect in the pass.

**Secondary unpropagated finding.** `CW-B07-06` discovered that `.clang-tidy`'s `HeaderFilterRegex`
is `.*(src|include)/rocketchip/.*`, so **no header under `src/**` is covered by the gates this walk
defers to** — invalidating the "already gated, do not re-hunt" premise for roughly half the tree's
headers. Only `B07` and `B19` noticed. The other 42 batches and all 5 lanes continued deferring to
gates that do not run on the files they were reading, and no lane generalized the hole beyond
`src/math/*.h` and `src/calibration/lm_solver.h`. This is an LL-43-class finding the pass produced
and then failed to act on.

---

## 6. Whole-system questions the five lanes did not ask

The five lanes are well chosen and well executed (`X1` cross-core ownership, `X2` duplicated
knowledge, `X3` comments-vs-authority, `X4` header contract coherence, `X5` init/lifecycle pairing).
Note that three of the five (`X2`, `X3`, `X4`) are comments/duplication lanes, reinforcing the
corpus-wide tilt. The following whole-system questions have no lane and no batch home:

1. **The run-to-completion budget as a system property.** The guide's block-C ADD
   *"Blocking-in-cooperative-scheduler / ISR timing"* is the project's most-documented systemic
   failure mode (LL Entry 32: a 150 ms blocking send overflowed a depth-16 AO queue at 100 Hz).
   Individual blocking sites were found — `CW-B36-03` (~4.25 s), `CW-X5-02`, `CW-X1-02` — but
   **nobody summed worst-case handler time across all eight AOs plus `qv_idle_bridge` against the
   tick rate and queue depths.** That is a one-lane question with a numeric answer, and the answer is
   what tells the owner whether the depth-32 mitigation still holds. One batch finding was even
   REFUTED on the grounds that *"the run-to-completion budget ... is already a written, project-wide
   system invariant"* (`docs/AO_ARCHITECTURE.md:14`) — precisely the invariant a lane should have
   tested rather than deferred to.
2. **Fail-open vs fail-closed error propagation end-to-end.** P10 Rule 7 was applied per file (22
   citations), but no lane traces one sensor failure from detection -> `health_monitor` ->
   annunciation -> FD gating -> telemetry -> flight log to ask whether the system as a whole fails
   safe. `CW-X1-06`'s "fails open" observation is the shape of the question; nothing generalized it.
3. **Untrusted-input ingress as one surface.** No lane collects NMEA, MAVLink RX, LoRa RX payload,
   CLI keystroke dispatch and flash/PSRAM record decode and asks the same robustness question of
   each. Per 2.6 this was also missed at the batch level.
4. **The assertion / fault-path story.** Three result files independently establish pieces of it —
   `RC_ASSERT` has no call sites (`CW-B01-05`), its documented watchdog reset cannot occur
   (`CW-X4-08`), `Q_onError` logs through a sink that forbids fault-handler use (`CW-B25-01` /
   `CW-X3-03` / `CW-X4-03`). **Nobody assembled them into the system-level claim**: what actually
   happens when an assertion trips in flight, and whether this tree has a working runtime assertion
   mechanism at all. The finding the owner needs exists only in fragments.
5. **The role x board x `#ifdef` configuration matrix.** Station-vs-vehicle build parity is the
   project's own named hazard (`SESSION_CHECKLIST` item 6). Findings repeatedly brush it:
   `ROCKETCHIP_RADIO_PERSIST` code that has apparently never compiled (`CW-X3-01`, plus `CW-B39-02`,
   whose Direction says *"prove the fix by compiling the persist configuration at least
   once"*), `ROCKETCHIP_STAGE_T3_MAVLINK` (`CW-B24-02`), the `board_pico2` `#error` gate
   (`CW-X4-11`), and `CW-B36-03`'s station-vs-vehicle reachability reversal. **Inactive `#ifdef`
   branches are exactly the surface a per-file read cannot see**, and no lane owns it.
6. **Test-suite coherence.** The corpus's most-cited spine type is "passes-tests-yet-wrong", and the
   tree carries ~856 host tests — yet `test/` is outside the itinerary and no lane asks which of the
   358 findings the suite should have caught, or whether any test encodes a contract these findings
   show to be wrong. `JSF AV 102` is the guide's own instruction to flag this, and it was not
   flagged.
7. **Gate coverage as a lane.** Per section 5, `CW-B07-06` shows the walk's deferral premise is false
   for `src/**/*.h`. A short lane applying the LL-43 move (read `.clang-tidy` + `CMakeLists.txt`,
   prove which files each gate actually reaches) would have bounded what the walk was entitled to
   defer. It was not run.

---

## 7. Drift into gated / mechanical territory, and into preference

The pass is mostly disciplined here. The guide's exclusions were respected: magic numbers were not
hunted (the two literal-shaped findings, `CW-B28-06` and `CW-X2-05`, are genuine ES.3 duplication
findings), function length was not hand-counted, and one finding was correctly REFUTED with
*"mechanically gated: `readability-duplicate-include` is enabled..."*. The residual drift:

- **Dead-code / no-consumer reachability is ~10% of the corpus.** 31 of 358 finding titles are some
  form of "has no consumer / is never read / is dead": `CW-B01-06`, `CW-B02-01`, `CW-B03-02`,
  `CW-B04-08`, `CW-B05-12`, `CW-B10-06`, `CW-B17-07`, `CW-B18-02`, `CW-B18-05`, `CW-B20-07`,
  `CW-B25-04`, `CW-B29-02`, `CW-B38-06`, `CW-B41-07`, `CW-B42-07`, `CW-X2-07`, `CW-X4-05`,
  `CW-X4-06`, `CW-X4-10`, and more. Whether an identifier has a reader is *mechanically decidable by
  grep*, and the project owns the tool (`scripts/audit/find_dead_code.py`, `SESSION_CHECKLIST` 17b)
  plus `clang-analyzer-deadcode`. `CW-B01-06` argues the gate gap explicitly
  (*"`find_dead_code.py` lists dead-global detection as out of scope; clang-tidy's deadcode checks
  do not see unused header constants"*) — the right move, and only one or two findings make it. The
  other ~29 present a greppable reachability fact as an eyeball finding without saying why the
  mechanical path was unavailable. That is a section-CM inventory wearing a walk label, and it
  inflates the corpus.
- **`FAIL` no longer carries severity.** 70 of ~187 files are `FAIL`, 104 `PARTIAL`, only 13 `PASS`.
  Roughly 41 of the 70 `FAIL` rows are stated purely in comment / banner / contract-prose terms
  (`ws2812_status.h` FAIL for "three doxygen claims disagree"; `pyro_edge_logger.h` FAIL because a
  banner says "static ring buffer" and the implementation is not one). Each is rule-correct — the
  Comments judging table does say comment/code disagreement is `FAIL` — but the aggregate means
  guide step 5 would generate ~70 `R-NN` problem reports, most of them comment rewrites, with no
  signal separating them from `CW-X1-03` (a cross-core calibration race that can corrupt the
  main-deploy altitude reference). The vocabulary was applied faithfully and the result is
  undifferentiated.
- **Preference drift was caught, not prevented.** Of the 19 REFUTED batch findings, at least five
  were refuted *as preference*: *"a maintainability preference rather than a defect"*;
  *"style/refactor preference"*; *"a DRY preference rather than a defect"*; *"extracting a helper for
  two four-line lambdas is a style preference, one AK_GUIDELINES 2/3 argues against"*; and
  `CW-B01-02`'s *"what remains is a preference for more per-line annotation"*. The verify pass is
  doing real work. But five preference-shaped findings reaching the verify stage — concentrated in
  the ES.3 / duplication family (`CW-B37-07`, `CW-B42-09`, `CW-B11-07`) — suggests ES.3 was run as a
  clone hunt rather than the guide's "same *knowledge* in two places" test.
- **One meta-finding sits outside the deliverable's frame.** `CW-B07-06`'s Lens line is *"The guide's
  mechanical-gating premise"* and its Site is `.clang-tidy:242` — a finding about the audit
  apparatus, filed in a source-batch results file where no disposition path reaches it. It is correct
  and important (section 5); it is in the wrong container.

---

## 8. Deliverable-format gaps

Guide procedure steps 4 and 5 are unperformed:

- **`L2P5_WALK_ITINERARY.md`: 98 checkbox rows, 0 ticked.** The itinerary is the walk's progress
  tracker and coverage map, and it records no coverage.
- **All 7 per-class Findings tables in `L2P5_MANUAL_WALK_GUIDE.md` are still empty.** The guide says
  *"put any `FAIL`/`PARTIAL` in the relevant class's findings table."* Nothing was placed — and as a
  side effect there is no per-lens view of the corpus, which is exactly what would have made
  sections 1 and 2 of this critique visible to the walkers while they were still walking.
- **Nothing was dispositioned.** `docs/PROBLEM_REPORTS.md` contains zero `CW-` references; no row
  migrated to `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`. 70 `FAIL` verdicts carry no owner
  decision.
- **No rollup or index exists** under `claude_walk/` — 49 files, 358 findings, no severity ordering,
  no cross-batch dedup list, no "top N for the owner."

---

## 9. What a second pass should target, ranked

1. **Reconcile the `sensor_core1.cpp:259-279` reachability contradiction** (`CW-B36-01`,
   `CW-B36-03`, `CW-X1-01/02/05/06`, `CW-X5-02`). One answer, one disposition.
2. **Re-walk the four lens-silent surfaces with a recorded verdict per file, PASS included:** class
   & interface design across `drivers/` (`B08`-`B12`) and `active_objects/` (`B37`-`B40`);
   assertions across `B11`, `B25`-`B28`, `B36`, `B39`, `B40`, `B41`, `B44`; the eval-order half of
   control-flow across `B07`-`B16`; templates in `B07`; and the `ao_led_engine` stack-local-event
   check the guide names as canonical.
3. **Close `CW-B07-06` tree-wide** before anything else defers to a gate: the `HeaderFilterRegex`
   hole leaves the walk's "already gated" premise unproven for every `src/**/*.h`.
4. **Add the missing whole-system lanes** — run-to-completion budget, fail-open/fail-closed error
   propagation, untrusted-input ingress, the assertion/fault-path story, and the
   role x board x `#ifdef` matrix.
5. **Re-verify the three hardware claims against PDFs already in the repo** (`CW-B09-07` PA1010D,
   `CW-B11-03` SX1276, `CW-B26-04` RP2350 XIP/QMI), and mark any finding whose verdict rests on a
   proxy source.
6. **Re-triage the ~29 no-consumer findings** through `find_dead_code.py` or a grep inventory,
   keeping as walk findings only those that argue a specific gate gap.
7. **Produce the deliverable the guide specifies:** tick the itinerary, populate the seven per-class
   findings tables, dedupe batch-vs-lane, and rank for disposition.
