# L2-P5 three-walk compare (derived)

**Not a record.** Finding packs stay frozen. This overlay is **note trust**, not
disposition order.

Rebuilt 2026-08-20 against the **aligned** Claude pack
(`L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md`). The join helper
(`_gen_three_walk_join.py`) was rewritten for that file; the old
`### CW-B01-01 --` parser is gone.

| Pack | File | Vote |
|------|------|------|
| Owner | `L2P5_WALK_FINDINGS.md` | WN-001–327 · one vote |
| Grok | `L2P5_GROK_WALK_FINDINGS.md` | GWF-001–498 · one vote |
| Claude | `L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md` | one vote · live rows after dropping 22 lane duplicates and REFUTED |

Join key: **itinerary leaf + the proposition.** Silence (`nothing of note`, Grok
`PASS`, Claude empty / REFUTED) is not “this is fine.” It is “this sitting did
not assert P.” Unique finds are why extra walks were run.

Grok’s 2026-08-19 two-way vs owner is **not an input**.

Claude batch vs Claude lane is **not** a second walk. Canonical Claude answer
is the batch row; Reconciliation §1 says the UART-staleness branch is **live
on the vehicle**.

---

## Claude pack fitness (checked before this join)

Usable as one vote **if** the join honors the reconciliation table (done here).

Still messy, not blocking:

- Banner vs CHANGELOG vs parse: “405” / “407 entries” / **402** parsed IDs.
  Close; not exact.
- “What changed” still says 5 claims unverifiable; the residual paragraph says
  none remain. Believe the later paragraph.
- Some lens-rerun rows still carry two `Verdict:` lines plus a corrected claim.
  Join takes the last verdict.
- Completeness critic at the tail still lists “reconcile UART reachability” as
  to-do. That is historical; Reconciliation §1 already answered it.
- A few `CW-L` rows are glued to the previous entry (no blank line). They still
  parse.

---

## 1. 2-vs-1 across the three walks

One pack confidently asserts P; the other **two packs** assert not P. Same P.

**Harvest: none.**

Closest misses (so they are not smuggled back in):

**LED range map.** Owner **WN-055**: range table current *vs this file’s own
`k*`*. Problem filed: taxonomy lives only as a banner. Grok **GWF-087** and
Claude **CW-B21-01**: SSOT / “values match `LedPhaseValue`” fails at code 28.
**WN-177** flagged split homes, no numeric check. Different P.

**QMI configure IRQ window.** Grok **GWF-311** (high): `qmi_hw` fenced in
`psram_detect`, unfenced in `psram_configure_qmi`. Claude **CW-B26-05**
REFUTED: `psram_init` from `main` before USB / QF tick / Core 1; no IRQ armed.
Owner silent. **1-vs-1.** Do not land a fix from one agent alone.

**`fused_state` 1-sigma (`GWF-094`), `gps_uart` rates/PMTK, Sola cites.** One
pack found what the others did not claim.

---

## 2. Triples (high trust the issue is real)

All three packs independently named the same underlying issue. Not a queue
position. Claude duplicates collapsed.

| Issue | Owner | Grok | Claude |
|-------|-------|------|--------|
| `RC_ASSERT` defined, zero call sites; documented failure/recovery cannot occur | **WN-007** (unused); **WN-006/008** banner/policy | **GWF-016** | **CW-B01-05** |
| Unknown / unmatched board → silent Feather pin map | **WN-020** | **GWF-023** | **CW-B02-06** |
| Phantom `version_string()`; no such API | **WN-011** | **GWF-102** | **CW-B06-07** |
| ICM-20948 lazy mag re-init on the 1 kHz read path | **WN-089** | **GWF-146** | **CW-B10-02** |
| Deprecated health aliases live, no consumers, wrong byte | **WN-051** | **GWF-080** | **CW-B05-12** |
| `g_imu` Core 0 init / Core 1 use; handoff not on the header | **WN-001**, **WN-002** | **GWF-006** | (Claude’s live hit on this leaf is the init-flag bools, not this handle) |

Same-neighborhood (owner asked home/process; agents named a concrete lie):

| Surface | Owner asked | Agents named |
|---------|-------------|--------------|
| Flash-layout banner | **WN-060** map-as-SoT | **GWF-096**, **CW-B06-03** stale numbers / missing radio-config |
| `flash_layout_valid()` | folded into **WN-060/062** | **GWF-097/098**, **CW-B06-04** |
| DPS310 OS/rate table | **WN-090** belongs in a sensor doc | **GWF-147**, **CW-B10-05** fabricated MaxRate / vs 8×@32 Hz |
| Log overflow | **WN-003** / **WN-201** both facts | **GWF-009**, **GWF-284**, **CW-B01-03** header/body clash |
| LED split homes | **WN-177** | **GWF-087**, **CW-B21-01** collision at 28 |

`g_baroInitialized` / `g_gpsInitialized` as live two-writer plain `bool`s:
Grok **GWF-003** + Claude **CW-B01-01** (CONFIRMED; Reconciliation §1 keeps
the gps half live). Owner did not file that P (3-question deferred). Two-agree,
not a clash with `shared_state.h`’s `g_imu` WNs.

---

## 3. Two agree, third silent (extra walks working)

Not contradictions. Medium-high trust of the *claim*. Owner-only policy on
the same leaf does not cover these claims.

| Claim | Who has it | Silent |
|-------|------------|--------|
| `eskf_to_fused_state()` does not exist | GWF-093, CW-B06-05 | Owner |
| `kRcLogRingBytes` named, not declared | GWF-011, CW-B01-04 | Owner |
| Dashboard `Temp:` is literal `0` | GWF-487, CW-B44-01 | Owner |
| Dashboard Alt and Baro print the same AGL value | GWF-488, CW-B44-02 | Owner |
| Station distance “stale” uses MET vs station uptime | GWF-481, CW-B43-01 | Owner |
| Boot-summary FAIL count vs FAIL list disagree | GWF-485, CW-B43-05 | Owner |
| PSRAM “hard gate” is erase-only | GWF-309, CW-B26-03 | Owner |
| `psram_self_test` readback through cached alias | GWF-310, CW-B26-04 | Owner |
| `ring_init` clobbers the header `ring_recover` reads | GWF-288, CW-B25-06 | Owner |
| Failed `ring_init` still marks initialized | GWF-292, CW-B25-05 | Owner |
| `core1_i2c_pause` can return without ack / fail-open | GWF-383, CW-B34-03 | Owner (**WN-264** is earn-rent, different P) |
| Pause/resume do not nest | GWF-386, CW-B34-02 | Owner |
| Anomalous-boot 2-of-N cannot fire | GWF-349, CW-B31-08 | Owner |
| `FlightMetadata` 14B vs sizeof 16 | GWF-079, CW-B05-11 | Owner |
| PMTK314 comment lists GSV; literal disables it | GWF-132, CW-B09-07 | Owner |
| UART RX ring: `volatile` claimed sufficient | GWF-135, CW-B09-04 | Owner |
| `gps_uart_reinit()` wrong-core NVIC (live on vehicle) | GWF-135, CW-B09-03 | Owner |
| Calibration session objects, both cores, no barrier | GWF-222, CW-B18-01 / CW-B36-02 | Owner |
| Seqlock + six atomics declared in two homes | GWF-007, CW-X4-09 | Owner |

Owner-only policy (SPDX **WN-004**, Doxygen **WN-054/081**, earn-rent, NOLINT,
Go/No-Go **WN-179**, RF/legal **WN-100**, comment archaeology) is the inverse:
owner found what the agent lenses were not for.

---

## 4. How to use this

**No three-walk 2-vs-1.** Disposition planning is not blocked by an
owner-vs-both-agents fight.

**1-vs-1 QMI:** don’t land a fix from GWF-311 alone.

**When reading a note:** a triple (§2) is strong evidence the issue is real.
A two-agree (§3) is why the extra walks were run. A unique is a candidate.

Not a disposition order, not a merge of IDs, not a certification.
