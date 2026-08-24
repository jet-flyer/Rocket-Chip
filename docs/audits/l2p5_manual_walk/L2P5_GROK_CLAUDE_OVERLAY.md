# L2-P5 Grok + Claude combined overlay

**Not a merge of IDs. Not a record. Finding packs stay frozen.**
Owner walk (`WN-001–327`) stays a **separate** chunk — that work is
already labeled and the do-now remediates are on `main`. This overlay
is chunks 2+3 together, because the two agent packs named many of the
same propositions.

| Pack | File | Role |
|------|------|------|
| Owner | `L2P5_WALK_FINDINGS.md` | Already dispositioned. Used here only as the previous sitting. |
| Grok | `L2P5_GROK_WALK_FINDINGS.md` | `GWF-001–498`. One vote. |
| Claude | `L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md` | One vote. Skip lane duplicates and REFUTED (except the disagree bin). |

Join key: **itinerary leaf + the proposition** (same helper as
`L2P5_THREE_WALK_COMPARE.md`). Silence is not “this is fine.”

Generator: `_gen_grok_claude_overlay.py`. Re-run after rule tweaks.
Review **buckets**, not each row. Default for remaining live rows is
still REMEDIATE; do not auto-ACCEPT.

---

## How to read the four bins

Every live Grok row and every live Claude row is tagged on two axes,
then dropped into a bucket (the same idea as the owner 16).

| Axis | Bin | Meaning |
|------|-----|---------|
| vs owner disposition | **cleared** | Same P as a closed REMEDIATE WN, or a comment/NOLINT/Doxygen note on a leaf that sitting already rewrote. Skip. |
| vs owner disposition | **affected** | That leaf was remediates, but the agent P is a different claim (or the comment sitting may have only half-fixed it). Re-read before skip or fix. |
| vs owner disposition | **defer-home** | Same P (or same leaf, no closed remediates) as an owner DEFER. Park with that home. Do not reopen Starcom / RC_OS structure / early-impl. |
| vs owner disposition | **untouched** | Owner did not remediate this leaf, or did not name this P. Extra walks working. |
| vs the other agent | **agree** | Grok and Claude independently named the same underlying issue. |
| vs the other agent | **disagree** | True is/isn’t split. Stop. |
| vs the other agent | **unique** | Only one pack asserted P. Candidate, not a fight. |

A row can be **agree and cleared** (both found it; owner already fixed
it). The work queue is: not-cleared rows, by bucket, skip `defer-home`
the same way owner skipped those sittings.

---

## Counts

| | n |
|--|--:|
| Grok kept | 498 |
| Claude live (no lane-dup, no REFUTED) | 355 |
| Combined rows in this overlay | 853 |
| Grok+Claude ident pairs (score≥4) | 81 |
| Cleared | 59 |
| Affected | 574 |
| Defer-home | 188 |
| Untouched | 32 |
| Both agree | 159 |
| Disagree | 1 |
| Unique | 693 |
| Remaining work (not cleared, not defer-home) | 606 |

Owner log parsed: 259 closed REMEDIATE, 66 DEFER.

---

## Completely disagree

Across Grok and Claude there was **one** curated is/isn’t split.
**Settled 2026-08-23:** Grok. Fence configure the same way as detect.

**P:** (the claim) Is the unfenced QMI direct-mode window in `psram_configure_qmi` a live IRQ hazard?

| Pack | ID | Answer |
|------|----|--------|
| Grok | `GWF-311` | Yes (high). Same `qmi_hw` is IRQ-fenced in `psram_detect`, unfenced in configure. **Kept.** |
| Claude | `CW-B26-05` | REFUTED. Only caller is `psram_init` from `main` before USB / QF tick / Core 1; no IRQ armed. **Refute fails.** USB/QF/Core 1 are later; that is not “no IRQ.” `init_gps_early()` → `gps_pa1010d_init()` → `sleep_ms` runs **before** `psram_init` and arms the pico-sdk default alarm-pool TIMER IRQ (`pico_time`; handler always enabled; IVT in flash). `hardware_flash.h`: QMI/XIP windows are unsafe with flash-resident handlers unless IRQs are off. Datasheet 12.14.5: `DIRECT_CSR.EN` disconnects the AHB XIP window (same class as the `clock_get_hz` lockup already fixed). |
| Owner | — | Nearby WNs on this leaf (WN-215–217) are density / board-coupling / test-permanence. Not this P. |

**Land:** `psram_configure_qmi` takes the same `save_and_disable_interrupts` / `restore_interrupts` fence as `psram_detect` for the EN window. Frozen packs stay frozen.

`0` / `kOff` in the LED resolver is the same facts, different “is this
a finding?” call — not this bin.

No additional live-Grok vs REFUTED-Claude ident pairs at score≥6.

---

## Fully cleared by the owner disposition

Skip these in the agent chunk. Frozen packs still contain the original
text — “cleared” means the **work** is done, not that the walk file
was edited.

### Curated triples (from the three-walk compare)

| Issue | Owner | Owner close | Grok | Claude | Bin |
|-------|-------|-------------|------|--------|-----|
| `RC_ASSERT` defined, zero call sites | `WN-007` REMEDIATE closed | `RC_ASSERT` deleted with `config.h` (0 callers) | `GWF-016` | `CW-B01-05` | **cleared** |
| Unknown / unmatched board → silent Feather pin map | `WN-020` REMEDIATE closed | unknown `PICO_BOARD` fail-closed; host still comp… | `GWF-023` | `CW-B02-06` | **cleared** |
| Phantom `version_string()`; no such API | `WN-011` REMEDIATE closed | phantom `version_string()` dropped; callers use `… | `GWF-102` | `CW-B06-07` | **cleared** |
| ICM-20948 lazy mag re-init on the 1 kHz read path | `WN-089` DEFER labeled | WB early-impl — lazy mag re-init recreate/test | `GWF-146` | `CW-B10-02` | **defer-home** |
| Deprecated health aliases live, no consumers, wrong byte | `WN-051` REMEDIATE closed | deleted unused `kHealthEskfHealthy` / `kHealthZup… | `GWF-080` | `CW-B05-12` | **cleared** |
| `g_imu` Core 0 init / Core 1 use; handoff not on the header | `WN-002` REMEDIATE closed | `g_imu` contract in header; after handoff Core 0 … | `GWF-006` | — | **cleared** |
| | | Claude's live hit on this leaf is the init-flag bools, not this handle. | | | |

### Auto-cleared (same-P or comment/NOLINT/Doxygen policy sitting)

Conservative: ident overlap with a **closed REMEDIATE** WN, or a
comment-shaped title on a leaf that sitting 1/5/13 already rewrote.
If a later re-read shows the live contract is still wrong, move the
row to **affected** — do not treat this list as certification.

Auto + curated cleared: **59** rows.

- **Log ring init / recover / named sizes** (1): `GWF-280`

- **Cross-core publication / fail-open** (3): `CW-B30-01`, `GWF-006`, `GWF-385`

- **Phantom / missing symbols** (4): `CW-B06-07`, `CW-B33-02`, `CW-B38-08`, `GWF-102`

- **In-source NOLINT / suppressions** (1): `GWF-105`

- **Doxygen + header comment-density** (1): `GWF-219`

- **Process archaeology in comments** (16): `CW-B19-05`, `CW-B23-03`, `CW-B24-05`, `GWF-015`, `GWF-089`, `GWF-148`, `GWF-149`, `GWF-166`, `GWF-184`, `GWF-226`, `GWF-238`, `GWF-241`, `GWF-319`, `GWF-412`, `GWF-450`, `GWF-459`

- **Comment still disagrees with the body** (7): `CW-B10-06`, `CW-B21-02`, `GWF-043`, `GWF-080`, `GWF-083`, `GWF-230`, `GWF-401`

- **HW leakage vs domain code** (3): `CW-B02-06`, `GWF-023`, `GWF-033`

- **Starcom / radio-telem supersession** (6): `CW-B05-12`, `GWF-150`, `GWF-156`, `GWF-337`, `GWF-445`, `GWF-447`

- **Safety / ops SSOT (Go/No-Go, pyro, guards)** (7): `CW-B31-07`, `CW-B34-04`, `CW-L019`, `GWF-247`, `GWF-363`, `GWF-384`, `GWF-396`

- **Early-impl / design re-eval** (2): `CW-X3-07`, `GWF-145`

- **RC_OS / CLI structure** (2): `CW-B44-07`, `GWF-418`

- **Version / identity / config.h grab-bag** (3): `CW-B01-05`, `CW-B03-02`, `GWF-016`

- **Fusion / math / cal live invariants** (1): `GWF-172`

- **One-off / leftover** (2): `GWF-028`, `GWF-088`

---

## Affected by the owner disposition

The file (or a nearby WN) was remediates, but the agent claim is not
the same P — or the comment sitting may have deleted the essay and
left a live lie. **Re-read the current tree** before skip or fix.

### Curated same-neighborhood (owner asked home; agents named a lie)

| Issue | Owner | Close | Grok | Claude |
|-------|-------|-------|------|--------|
| Flash-layout banner / `flash_layout_valid()` stale numbers | `WN-060` REMEDIATE closed | constexprs are the layout; comment map gone … | `GWF-096`, `GWF-097`, `GWF-098` | `CW-B06-03`, `CW-B06-04` |
| | | Sitting 13 made constexprs the code SSOT. Re-read whether agent stale-number P is gone. | | |
| DPS310 OS/rate table / fabricated MaxRate | `WN-090` REMEDIATE closed | OS table gone; constexprs stay (`0613747`) | `GWF-147` | `CW-B10-05` |
| | | Owner asked home; agents named a numeric lie. Comment sitting trimmed the table — check the numbers. | | |
| Log overflow header/body clash (drop-oldest vs drop-newest) | `WN-003` REMEDIATE closed | live bounds/drain/no-stdio; drop plan path (… | `GWF-009`, `GWF-284` | `CW-B01-03` |
| | | WN-201 also. Archaeology sitting rewrote comments; the ring policy may still be a live P. | | |
| LED split homes / collision at 28 | `WN-177` REMEDIATE closed | LED phase codes point at `led_patterns.h`, n… | `GWF-087` | `CW-B21-01` |
| | | Owner closed pointer-at-led_patterns.h. Agents named a numeric collision. Different P — re-read. | | |

### Auto-affected

**574** rows. Compact IDs here; titles live in Combined remaining buckets.

- **Dashboard / station CLI display lies** (23): `CW-B38-01`, `CW-B39-04`, `CW-B43-01`, `CW-B43-05`, `CW-B44-01`, `CW-B44-02`, `CW-B44-04`, `CW-B44-08`, `CW-B44-09`, `CW-L022`, `CW-X2-01`, `CW-X2-05`, `CW-X2-08`, `GWF-269`, `GWF-481`, `GWF-485`, `GWF-486`, `GWF-487`, `GWF-488`, `GWF-489`, `GWF-490`, `GWF-491`, `GWF-492`

- **PSRAM init / QMI window** (4): `CW-B26-03`, `CW-B26-04`, `CW-X5-05`, `GWF-310`

- **Log ring init / recover / named sizes** (24): `CW-B01-03`, `CW-B25-01`, `CW-B25-02`, `CW-B25-03`, `CW-B25-04`, `CW-B25-05`, `CW-B25-06`, `CW-B43-02`, `CW-L040`, `GWF-009`, `GWF-014`, `GWF-018`, `GWF-281`, `GWF-282`, `GWF-283`, `GWF-284`, `GWF-285`, `GWF-286`, `GWF-287`, `GWF-288`, `GWF-289`, `GWF-290`, `GWF-291`, `GWF-292`

- **GPS UART / PMTK / wrong-core NVIC** (9): `CW-B09-07`, `CW-B14-05`, `CW-B36-01`, `CW-B36-03`, `CW-L041`, `GWF-008`, `GWF-125`, `GWF-402`, `GWF-405`

- **Cross-core publication / fail-open** (55): `CW-B01-01`, `CW-B10-03`, `CW-B12-07`, `CW-B18-01`, `CW-B18-03`, `CW-B22-01`, `CW-B30-02`, `CW-B31-04`, `CW-B32-01`, `CW-B34-02`, `CW-B34-03`, `CW-B36-02`, `CW-B41-01`, `CW-B41-03`, `CW-B41-04`, `CW-B41-05`, `CW-X1-07`, `CW-X1-08`, `CW-X4-09`, `CW-X5-03`, `CW-X5-04`, `GWF-001`, `GWF-002`, `GWF-003`, `GWF-004`, `GWF-005`, `GWF-007`, `GWF-013`, `GWF-031`, `GWF-123`, `GWF-168`, `GWF-171`, `GWF-183`, `GWF-208`, `GWF-222`, `GWF-234`, `GWF-340`, `GWF-341`, `GWF-342`, `GWF-343`, `GWF-361`, `GWF-368`, `GWF-381`, `GWF-386`, `GWF-404`, `GWF-417`, `GWF-425`, `GWF-454`, `GWF-460`, `GWF-462`, `GWF-463`, `GWF-469`, `GWF-470`, `GWF-482`, `GWF-498`

- **Phantom / missing symbols** (37): `CW-B01-04`, `CW-B02-08`, `CW-B03-03`, `CW-B05-02`, `CW-B05-13`, `CW-B06-04`, `CW-B06-08`, `CW-B10-05`, `CW-B12-03`, `CW-B20-03`, `CW-B24-01`, `CW-B29-02`, `CW-B30-04`, `CW-B37-04`, `CW-B38-06`, `CW-B39-06`, `CW-B42-01`, `CW-B42-06`, `CW-B43-06`, `CW-B43-09`, `CW-X3-04`, `CW-X3-05`, `CW-X4-10`, `GWF-011`, `GWF-021`, `GWF-036`, `GWF-037`, `GWF-045`, `GWF-050`, `GWF-074`, `GWF-086`, `GWF-258`, `GWF-339`, `GWF-376`, `GWF-409`, `GWF-466`, `GWF-496`

- **Class-design / published guts** (20): `CW-B01-07`, `CW-B02-02`, `CW-B05-06`, `CW-B07-06`, `CW-B12-02`, `CW-B37-02`, `CW-B39-01`, `CW-L021`, `CW-L023`, `CW-L024`, `CW-L026`, `CW-L027`, `CW-L028`, `CW-L029`, `CW-L030`, `CW-L031`, `CW-L032`, `CW-L033`, `CW-L036`, `CW-X4-07`

- **In-source NOLINT / suppressions** (1): `CW-B42-08`

- **Process archaeology in comments** (10): `CW-B06-03`, `CW-B07-02`, `CW-B12-05`, `CW-B20-07`, `CW-B29-04`, `CW-B41-08`, `GWF-096`, `GWF-202`, `GWF-321`, `GWF-461`

- **Comment still disagrees with the body** (213): `CW-B02-04`, `CW-B04-01`, `CW-B04-03`, `CW-B06-01`, `CW-B06-02`, `CW-B07-01`, `CW-B07-03`, `CW-B07-04`, `CW-B08-01`, `CW-B08-02`, `CW-B08-03`, `CW-B08-04`, `CW-B08-05`, `CW-B08-06`, `CW-B09-08`, `CW-B09-09`, `CW-B12-01`, `CW-B12-04`, `CW-B12-06`, `CW-B12-08`, `CW-B13-01`, `CW-B13-02`, `CW-B13-03`, `CW-B13-04`, `CW-B13-05`, `CW-B13-06`, `CW-B13-07`, `CW-B13-08`, `CW-B14-01`, `CW-B14-03`, `CW-B14-07`, `CW-B15-01`, `CW-B15-03`, `CW-B15-04`, `CW-B16-03`, `CW-B16-04`, `CW-B16-05`, `CW-B16-06`, `CW-B17-01`, `CW-B17-02`, `CW-B17-03`, `CW-B17-04`, `CW-B17-05`, `CW-B17-06`, `CW-B17-07`, `CW-B18-04`, `CW-B19-04`, `CW-B20-01`, `CW-B20-02`, `CW-B20-04`, `CW-B20-05`, `CW-B23-01`, `CW-B23-02`, `CW-B24-04`, `CW-B26-01`, `CW-B26-06`, `CW-B26-07`, `CW-B27-01`, `CW-B27-02`, `CW-B27-03`, `CW-B27-04`, `CW-B27-05`, `CW-B27-06`, `CW-B28-04`, `CW-B29-01`, `CW-B29-05`, `CW-B36-04`, `CW-B37-01`, `CW-B37-05`, `CW-B37-06`, `CW-B38-03`, `CW-B38-07`, `CW-B40-04`, `CW-B40-05`, `CW-B40-09`, `CW-B41-02`, `CW-B41-07`, `CW-X2-03`, `CW-X2-04`, `CW-X5-07`, `CW-X5-08`, `GWF-010`, `GWF-022`, `GWF-024`, `GWF-030`, `GWF-032`, `GWF-035`, `GWF-044`, `GWF-051`, `GWF-052`, `GWF-053`, `GWF-056`, `GWF-057`, `GWF-058`, `GWF-059`, `GWF-069`, `GWF-070`, `GWF-077`, `GWF-078`, `GWF-084`, `GWF-097`, `GWF-107`, `GWF-108`, `GWF-109`, `GWF-110`, `GWF-111`, `GWF-113`, `GWF-114`, `GWF-115`, `GWF-116`, `GWF-117`, `GWF-118`, `GWF-119`, `GWF-120`, `GWF-121`, `GWF-122`, `GWF-124`, `GWF-126`, `GWF-127`, `GWF-128`, `GWF-147`, `GWF-157`, `GWF-158`, `GWF-160`, `GWF-161`, `GWF-162`, `GWF-163`, `GWF-164`, `GWF-167`, `GWF-169`, `GWF-170`, `GWF-177`, `GWF-179`, `GWF-182`, `GWF-194`, `GWF-195`, `GWF-196`, `GWF-198`, `GWF-199`, `GWF-200`, `GWF-201`, `GWF-205`, `GWF-206`, `GWF-214`, `GWF-216`, `GWF-217`, `GWF-221`, `GWF-224`, `GWF-225`, `GWF-227`, `GWF-229`, `GWF-232`, `GWF-236`, `GWF-237`, `GWF-239`, `GWF-240`, `GWF-242`, `GWF-243`, `GWF-245`, `GWF-251`, `GWF-253`, `GWF-254`, `GWF-256`, `GWF-257`, `GWF-259`, `GWF-260`, `GWF-261`, `GWF-263`, `GWF-264`, `GWF-265`, `GWF-266`, `GWF-268`, `GWF-270`, `GWF-271`, `GWF-272`, `GWF-274`, `GWF-277`, `GWF-293`, `GWF-296`, `GWF-297`, `GWF-299`, `GWF-301`, `GWF-302`, `GWF-303`, `GWF-312`, `GWF-313`, `GWF-314`, `GWF-315`, `GWF-317`, `GWF-318`, `GWF-322`, `GWF-323`, `GWF-324`, `GWF-325`, `GWF-327`, `GWF-403`, `GWF-406`, `GWF-407`, `GWF-410`, `GWF-411`, `GWF-421`, `GWF-422`, `GWF-423`, `GWF-424`, `GWF-426`, `GWF-451`, `GWF-452`, `GWF-453`, `GWF-456`, `GWF-457`, `GWF-465`, `GWF-467`, `GWF-468`

- **HW leakage vs domain code** (3): `CW-X2-07`, `GWF-034`, `GWF-040`

- **Safety / ops SSOT (Go/No-Go, pyro, guards)** (74): `CW-B03-01`, `CW-B21-03`, `CW-B21-05`, `CW-B22-04`, `CW-B23-04`, `CW-B31-01`, `CW-B31-02`, `CW-B31-03`, `CW-B31-05`, `CW-B31-06`, `CW-B31-08`, `CW-B32-02`, `CW-B32-03`, `CW-B32-04`, `CW-B32-05`, `CW-B32-06`, `CW-B33-01`, `CW-B33-03`, `CW-B33-04`, `CW-B33-05`, `CW-B33-06`, `CW-B34-01`, `CW-B34-05`, `CW-B35-05`, `CW-B35-06`, `CW-B37-03`, `CW-B43-04`, `CW-L004`, `CW-L015`, `CW-L016`, `CW-L017`, `GWF-244`, `GWF-248`, `GWF-249`, `GWF-250`, `GWF-262`, `GWF-344`, `GWF-345`, `GWF-346`, `GWF-347`, `GWF-348`, `GWF-349`, `GWF-350`, `GWF-351`, `GWF-352`, `GWF-353`, `GWF-354`, `GWF-355`, `GWF-356`, `GWF-357`, `GWF-358`, `GWF-359`, `GWF-360`, `GWF-362`, `GWF-364`, `GWF-365`, `GWF-366`, `GWF-367`, `GWF-369`, `GWF-370`, `GWF-371`, `GWF-372`, `GWF-373`, `GWF-374`, `GWF-375`, `GWF-377`, `GWF-378`, `GWF-379`, `GWF-380`, `GWF-382`, `GWF-393`, `GWF-394`, `GWF-395`, `GWF-397`

- **Test / inject / debug in the flight tree** (3): `GWF-413`, `GWF-493`, `GWF-494`

- **File earn-rent / naming / packaging** (8): `CW-B05-11`, `CW-B21-01`, `GWF-079`, `GWF-087`, `GWF-176`, `GWF-220`, `GWF-273`, `GWF-326`

- **Version / identity / config.h grab-bag** (9): `CW-B01-06`, `CW-B02-03`, `CW-B28-03`, `CW-L046`, `GWF-017`, `GWF-019`, `GWF-104`, `GWF-276`, `GWF-316`

- **Fusion / math / cal live invariants** (34): `CW-B07-05`, `CW-B14-04`, `CW-B15-02`, `CW-B16-07`, `CW-B19-06`, `CW-L006`, `CW-L007`, `CW-L008`, `CW-L009`, `CW-L010`, `CW-L012`, `CW-L013`, `CW-L014`, `CW-L042`, `GWF-106`, `GWF-112`, `GWF-173`, `GWF-174`, `GWF-175`, `GWF-178`, `GWF-180`, `GWF-181`, `GWF-192`, `GWF-193`, `GWF-197`, `GWF-203`, `GWF-207`, `GWF-209`, `GWF-215`, `GWF-218`, `GWF-223`, `GWF-228`, `GWF-231`, `GWF-233`

- **P10-9 function pointers** (5): `CW-B18-02`, `CW-B19-01`, `CW-B42-04`, `GWF-235`, `GWF-294`

- **One-off / leftover** (42): `CW-B14-02`, `CW-B18-05`, `CW-B19-02`, `CW-B20-06`, `CW-B21-04`, `CW-B22-03`, `CW-B23-05`, `CW-B29-03`, `CW-B38-04`, `CW-B40-07`, `CW-B40-08`, `CW-L001`, `CW-L002`, `CW-L003`, `CW-L043`, `CW-L044`, `CW-L045`, `CW-L047`, `GWF-012`, `GWF-029`, `GWF-038`, `GWF-039`, `GWF-085`, `GWF-098`, `GWF-103`, `GWF-129`, `GWF-159`, `GWF-165`, `GWF-246`, `GWF-252`, `GWF-255`, `GWF-267`, `GWF-295`, `GWF-298`, `GWF-300`, `GWF-304`, `GWF-320`, `GWF-408`, `GWF-427`, `GWF-455`, `GWF-458`, `GWF-464`

---

## Covered by an owner DEFER (park, do not reopen)

Same homes as the owner chunk: Starcom, RC_OS structure, early-impl
rewrites, codegen audit, WN-100. Agent polish on those surfaces waits
with the owner DEFER. Dashboard **display lies** are *not* RC_OS
structure — they stay in the dashboard bucket.

**188** rows.

- **Starcom / radio-telem supersession** (97): `CW-B04-04`, `CW-B04-05`, `CW-B04-06`, `CW-B04-07`, `CW-B04-08`, `CW-B05-03`, `CW-B05-07`, `CW-B05-08`, `CW-B05-09`, `CW-B05-14`, `CW-B06-09`, `CW-B11-01`, `CW-B11-02`, `CW-B11-03`, `CW-B11-04`, `CW-B11-05`, `CW-B11-06`, `CW-B28-01`, `CW-B28-02`, `CW-B28-06`, `CW-B30-03`, `CW-B30-05`, `CW-B30-07`, `CW-B30-08`, `CW-B34-06`, `CW-B34-07`, `CW-B34-08`, `CW-B39-02`, `CW-B39-03`, `CW-B39-05`, `CW-B40-01`, `CW-B40-02`, `CW-B40-03`, `CW-B40-06`, `CW-L037`, `CW-L038`, `CW-X2-02`, `CW-X4-05`, `CW-X5-09`, `GWF-054`, `GWF-055`, `GWF-060`, `GWF-061`, `GWF-062`, `GWF-063`, `GWF-071`, `GWF-072`, `GWF-073`, `GWF-075`, `GWF-076`, `GWF-081`, `GWF-082`, `GWF-090`, `GWF-091`, `GWF-092`, `GWF-151`, `GWF-152`, `GWF-153`, `GWF-154`, `GWF-155`, `GWF-305`, `GWF-306`, `GWF-307`, `GWF-308`, `GWF-328`, `GWF-329`, `GWF-330`, `GWF-331`, `GWF-332`, `GWF-333`, `GWF-334`, `GWF-335`, `GWF-336`, `GWF-338`, `GWF-398`, `GWF-399`, `GWF-400`, `GWF-428`, `GWF-429`, `GWF-430`, `GWF-431`, `GWF-432`, `GWF-433`, `GWF-434`, `GWF-435`, `GWF-436`, `GWF-437`, `GWF-438`, `GWF-439`, `GWF-440`, `GWF-441`, `GWF-442`, `GWF-443`, `GWF-444`, `GWF-446`, `GWF-448`, `GWF-449`

- **Generated files / codegen hygiene** (22): `CW-B13-09`, `CW-B14-06`, `CW-B16-01`, `CW-B16-02`, `CW-B24-02`, `CW-B24-03`, `GWF-020`, `GWF-185`, `GWF-186`, `GWF-187`, `GWF-188`, `GWF-189`, `GWF-190`, `GWF-191`, `GWF-204`, `GWF-210`, `GWF-211`, `GWF-212`, `GWF-213`, `GWF-275`, `GWF-278`, `GWF-279`

- **Early-impl / design re-eval** (37): `CW-B05-01`, `CW-B05-04`, `CW-B05-05`, `CW-B05-10`, `CW-B06-06`, `CW-B09-01`, `CW-B10-01`, `CW-B10-02`, `CW-B10-04`, `CW-B35-01`, `CW-B35-02`, `CW-B35-03`, `CW-B35-04`, `CW-B41-06`, `CW-L018`, `GWF-064`, `GWF-065`, `GWF-066`, `GWF-067`, `GWF-068`, `GWF-099`, `GWF-100`, `GWF-101`, `GWF-138`, `GWF-139`, `GWF-140`, `GWF-141`, `GWF-142`, `GWF-143`, `GWF-144`, `GWF-146`, `GWF-387`, `GWF-388`, `GWF-389`, `GWF-390`, `GWF-391`, `GWF-392`

- **RC_OS / CLI structure** (32): `CW-B38-02`, `CW-B38-05`, `CW-B42-02`, `CW-B42-03`, `CW-B42-05`, `CW-B42-07`, `CW-B43-03`, `CW-B43-07`, `CW-B43-10`, `CW-B44-05`, `CW-B44-06`, `CW-L039`, `CW-X3-08`, `GWF-414`, `GWF-415`, `GWF-416`, `GWF-419`, `GWF-420`, `GWF-471`, `GWF-472`, `GWF-473`, `GWF-474`, `GWF-475`, `GWF-476`, `GWF-477`, `GWF-478`, `GWF-479`, `GWF-480`, `GWF-483`, `GWF-484`, `GWF-495`, `GWF-497`

---

## Both agree (and still live)

Grok and Claude named the same underlying issue. Owner was silent, or
the overlapping WN did not close this P. This is why the extra walks
were run. Medium-high trust of the *claim* — still not a rank.

### From the three-walk compare (owner silent)

| Claim | Grok | Claude | Bucket |
|-------|------|--------|--------|
| eskf_to_fused_state() does not exist | `GWF-093` (untouched) | `CW-B06-05` (untouched) | Phantom / missing symbols |
| kRcLogRingBytes named, not declared | `GWF-011` (affected) | `CW-B01-04` (affected) | Phantom / missing symbols |
| Dashboard Temp: is literal 0 | `GWF-487` (affected) | `CW-B44-01` (affected) | Dashboard / station CLI display lies |
| Dashboard Alt and Baro print the same AGL value | `GWF-488` (affected) | `CW-B44-02` (affected) | Dashboard / station CLI display lies |
| Station distance stale uses MET vs station uptime | `GWF-481` (affected) | `CW-B43-01` (affected) | Dashboard / station CLI display lies |
| Boot-summary FAIL count vs FAIL list disagree | `GWF-485` (affected) | `CW-B43-05` (affected) | Dashboard / station CLI display lies |
| PSRAM hard gate is erase-only | `GWF-309` (untouched) | `CW-B26-03` (affected) | PSRAM init / QMI window |
| psram_self_test readback through cached alias | `GWF-310` (affected) | `CW-B26-04` (affected) | PSRAM init / QMI window |
| ring_init clobbers the header ring_recover reads | `GWF-288` (affected) | `CW-B25-06` (affected) | Log ring init / recover / named sizes |
| Failed ring_init still marks initialized | `GWF-292` (affected) | `CW-B25-05` (affected) | Log ring init / recover / named sizes |
| core1_i2c_pause can return without ack / fail-open | `GWF-383` (untouched) | `CW-B34-03` (affected) | Cross-core publication / fail-open |
| Pause/resume do not nest | `GWF-386` (affected) | `CW-B34-02` (affected) | Cross-core publication / fail-open |
| Anomalous-boot 2-of-N cannot fire | `GWF-349` (affected) | `CW-B31-08` (affected) | Safety / ops SSOT (Go/No-Go, pyro, guards) |
| FlightMetadata 14B vs sizeof 16 | `GWF-079` (affected) | `CW-B05-11` (affected) | File earn-rent / naming / packaging |
| PMTK314 comment lists GSV; literal disables it | `GWF-132` (untouched) | `CW-B09-07` (affected) | GPS UART / PMTK / wrong-core NVIC |
| UART RX ring: volatile claimed sufficient | `GWF-135` (untouched) | `CW-B09-04` (untouched) | GPS UART / PMTK / wrong-core NVIC |
| gps_uart_reinit() wrong-core NVIC (live on vehicle) | `GWF-135` (untouched) | `CW-B09-03` (untouched) | GPS UART / PMTK / wrong-core NVIC |
| Calibration session objects, both cores, no barrier | `GWF-222` (affected) | `CW-B18-01` (affected) | Cross-core publication / fail-open |
| Seqlock + six atomics declared in two homes | `GWF-007` (affected) | `CW-X4-09` (affected) | Cross-core publication / fail-open |

GWF-135 is one Grok row covering two Claude UART claims (volatile ring
+ wrong-core NVIC). Do not mint a second GWF.

### Other ident pairs still not cleared / not defer-home

**63** pairs (a row may appear in more than one pair).

| Leaf | Grok | ax | Claude | ax | score |
|------|------|----|--------|----|------:|
| `include/rocketchip/fused_state.h` | `GWF-093` File-level owner/mutator/source… | untouched | `CW-B06-05` fused_state.h names eskf_to_fus… | untouched | 13 |
| `log/rc_log.cpp` | `GWF-287` Owner is asserted in this comme… | affected | `CW-B25-01` rc_log ring's single-context sa… | affected | 12 |
| `active_objects/ao_logger.{cpp,h}` | `GWF-426` Subscription comment claims an … | affected | `CW-B38-08` AO_Logger subscribes to SIG_HEA… | cleared | 12 |
| `active_objects/ao_radio.{cpp,h}` | `GWF-429` Persist-on revert/persist path … | defer-home | `CW-B39-01` ROCKETCHIP_RADIO_PERSIST branch… | affected | 12 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-458` Header name and comment promise… | affected | `CW-B40-09` The dev-only override is docume… | affected | 12 |
| `include/rocketchip/board_tiny_2350_…` | `GWF-038` Both roles are bound to GPIO 21… | affected | `CW-B02-03` Tiny 2350 map assigns GPIO 21 t… | affected | 9 |
| `fusion/eskf_runner.{cpp,h}` | `GWF-171` Cross-core shared object g_eskf… | affected | `CW-B14-05` g_eskf and g_eskfInitialized ar… | affected | 9 |
| `fusion/eskf_runner.{cpp,h}` | `GWF-171` Cross-core shared object g_eskf… | affected | `CW-L041` `g_eskf` is read from Core 1 wi… | affected | 9 |
| `calibration/calibration_manager.{cp…` | `GWF-223` Progress contract does not cove… | affected | `CW-B18-01` Calibration collection state is… | affected | 9 |
| `flight_director/flight_director.{cp…` | `GWF-237` Public field is written once to… | affected | `CW-X5-08` the flight-in-progress sentinel… | affected | 9 |
| `flight_director/flight_state.h` | `GWF-270` File banner is stale versus the… | affected | `CW-B20-01` File banner states eight phases… | affected | 9 |
| `flight_director/flight_actions.h` | `GWF-272` File banner overstates the surf… | affected | `CW-L002` B. Phase- and signal-indexed ta… | affected | 9 |
| `station/station_idle_tick.{cpp,h}` | `GWF-341` Comment attributes seqlock visi… | affected | `CW-B30-02` GPS-init guard also suppresses … | affected | 9 |
| `safety/test_mode.{cpp,h}` | `GWF-375` The public accessor contract (a… | affected | `CW-B33-03` magic-observed-at-boot is docum… | affected | 9 |
| `core1/sensor_core1.{cpp,h}` | `GWF-404` Owner is Core 0 fusion; mutator… | affected | `CW-B36-01` Core 1 reads Core 0's live ESKF… | affected | 9 |
| `active_objects/ao_logger.{cpp,h}` | `GWF-421` Header and body comments descri… | affected | `CW-B38-08` AO_Logger subscribes to SIG_HEA… | cleared | 9 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-456` Public file banner still descri… | affected | `CW-B40-08` The Core-1 vitality fallback is… | affected | 9 |
| `include/rocketchip/shared_state.h` | `GWF-001` File-level ownership story disa… | affected | `CW-B01-01` Two cross-core init flags are p… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-001` File-level ownership story disa… | affected | `CW-B41-04` Two init flags are written from… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-001` File-level ownership story disa… | affected | `CW-X4-09` The cross-core ownership map is… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-002` Explicitly cross-core gating fl… | affected | `CW-B01-01` Two cross-core init flags are p… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-003` Init flags that Core 1 may writ… | affected | `CW-B01-01` Two cross-core init flags are p… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-003` Init flags that Core 1 may writ… | affected | `CW-B41-04` Two init flags are written from… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-003` Init flags that Core 1 may writ… | affected | `CW-X4-09` The cross-core ownership map is… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-004` Contract surface leaves writer … | affected | `CW-B41-05` Ownership annotations on the co… | affected | 6 |
| `include/rocketchip/shared_state.h` | `GWF-007` Seqlock and signaling atomics h… | affected | `CW-X4-09` The cross-core ownership map is… | affected | 6 |
| `include/rocketchip/rc_log.h` | `GWF-011` kRcLogRingBytes is not declared… | affected | `CW-B01-04` Ring-health guidance names a co… | affected | 6 |
| `include/rocketchip/version.h` | `GWF-104` The SSOT header itself has two … | affected | `CW-B06-07` version.h's single-source rule … | cleared | 6 |
| `include/rocketchip/board_tiny_2350_…` | `GWF-036` board_tiny_2350.h is not presen… | affected | `CW-B02-08` Tiny common preamble points at … | affected | 6 |
| `fusion/eskf_runner.{cpp,h}` | `GWF-171` Cross-core shared object g_eskf… | affected | `CW-B14-02` g_mag3dEnabled latch survives f… | affected | 6 |
| `fusion/eskf_runner.{cpp,h}` | `GWF-171` Cross-core shared object g_eskf… | affected | `CW-B14-04` eskf_run_predict() aborts by re… | affected | 6 |
| `calibration/calibration_manager.{cp…` | `GWF-222` Documented cross-core session o… | affected | `CW-B18-01` Calibration collection state is… | affected | 6 |
| `flight_director/guard_evaluator.{cp…` | `GWF-255` The documented phase-bit encodi… | affected | `CW-B22-04` Phase bitmask is one bit too na… | affected | 6 |
| `flight_director/guard_evaluator.{cp…` | `GWF-255` The documented phase-bit encodi… | affected | `CW-L004` D. The phase bitmask is one bit… | affected | 6 |
| `flight_director/guard_combinator.{c…` | `GWF-262` uint8_t phase bitmask / phase_b… | affected | `CW-B23-04` Phase bitmask cannot represent … | affected | 6 |
| `flight_director/flight_actions.h` | `GWF-271` Public symbol named kEmptyActio… | affected | `CW-B20-07` Exit-action machinery is docume… | affected | 6 |
| `flight_director/flight_actions.h` | `GWF-272` File banner overstates the surf… | affected | `CW-B20-05` FAULT entry actions are unreach… | affected | 6 |
| `flight_director/flight_actions.h` | `GWF-273` Header color inventory paraphra… | affected | `CW-B20-05` FAULT entry actions are unreach… | affected | 6 |
| `log/rc_log.cpp` | `GWF-284` Public header promises drop-new… | affected | `CW-B25-01` rc_log ring's single-context sa… | affected | 6 |
| `logging/flight_table.{cpp,h}` | `GWF-296` File-level layout comment is st… | affected | `CW-B27-01` flight_table.h banner flash map… | affected | 6 |
| `logging/psram_init.{cpp,h}` | `GWF-310` Comment claims an addressing se… | affected | `CW-B26-04` psram_self_test reads back thro… | affected | 6 |
| `station/station_idle_tick.{cpp,h}` | `GWF-341` Comment attributes seqlock visi… | affected | `CW-B30-01` Header banner still describes t… | cleared | 6 |
| `station/station_idle_tick.{cpp,h}` | `GWF-342` Owner (include: Core 1) and mut… | affected | `CW-B30-01` Header banner still describes t… | cleared | 6 |
| `station/station_idle_tick.{cpp,h}` | `GWF-342` Owner (include: Core 1) and mut… | affected | `CW-B30-02` GPS-init guard also suppresses … | affected | 6 |
| `safety/fault_protection.{cpp,h}` | `GWF-348` Reset/SRAM wipe story contradic… | affected | `CW-B31-04` One non-atomic flag serves as a… | affected | 6 |
| `safety/anomalous_boot.{cpp,h}` | `GWF-349` Header design comment states a … | affected | `CW-B31-08` The documented 2-of-N corrobora… | affected | 6 |
| `safety/anomalous_boot.{cpp,h}` | `GWF-350` Enum comments overstate what ea… | affected | `CW-B31-08` The documented 2-of-N corrobora… | affected | 6 |
| `safety/test_mode.{cpp,h}` | `GWF-377` evaluate() never reads g_test_m… | affected | `CW-B33-03` magic-observed-at-boot is docum… | affected | 6 |
| `safety/test_mode.{cpp,h}` | `GWF-382` This leaf’s own comments put wr… | affected | `CW-B33-03` magic-observed-at-boot is docum… | affected | 6 |
| `safety/core1_i2c_pause.{cpp,h}` | `GWF-383` Public pause contract is succes… | untouched | `CW-B34-04` Core 0 writes g_core1I2CPaused,… | cleared | 6 |
| `safety/pyro_edge_logger.{cpp,h}` | `GWF-396` File banner calls the store a s… | cleared | `CW-B34-05` Banner calls the storage a "rin… | affected | 6 |
| `core1/sensor_core1.{cpp,h}` | `GWF-401` File banner understates the pub… | cleared | `CW-B36-01` Core 1 reads Core 0's live ESKF… | affected | 6 |
| `active_objects/ao_health_monitor.{c…` | `GWF-413` Header and AO_HealthMonitor_sta… | affected | `CW-B37-05` Startup health publish reaches … | affected | 6 |
| `active_objects/ao_logger.{cpp,h}` | `GWF-425` Two-AO writers of hidden rate s… | affected | `CW-B38-04` the two-point baro differentiat… | affected | 6 |
| `active_objects/ao_logger.{cpp,h}` | `GWF-427` Two public entry points for the… | affected | `CW-B38-08` AO_Logger subscribes to SIG_HEA… | cleared | 6 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-456` Public file banner still descri… | affected | `CW-B40-09` The dev-only override is docume… | affected | 6 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-457` File-level comment asserts per-… | affected | `CW-B40-08` The Core-1 vitality fallback is… | affected | 6 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-458` Header name and comment promise… | affected | `CW-B40-08` The Core-1 vitality fallback is… | affected | 6 |
| `active_objects/ao_led_engine.{cpp,h}` | `GWF-459` IVP-106 priority comment is lef… | cleared | `CW-B40-08` The Core-1 vitality fallback is… | affected | 6 |
| `main.cpp` | `GWF-464` Core 1 is launched on every rol… | affected | `CW-X5-04` the boot-time PSRAM flash-safe … | affected | 6 |
| `include/rocketchip/board_feather_rp…` | `GWF-027` Two polarity promises (flag vs … | untouched | `CW-B02-07` LED polarity is stated twice: a… | untouched | 6 |
| `drivers/gps_uart.{cpp,h}` | `GWF-135` reinit ownership and barrier ve… | untouched | `CW-B09-04` cross-core ring declares volati… | untouched | 6 |
| `include/rocketchip/fused_state.h` | `GWF-095` The group comment presents vert… | untouched | `CW-B06-05` fused_state.h names eskf_to_fus… | untouched | 4 |

---

## Unique rows that still group

One pack asserted P; the other sitting did not. Unique is why extra
walks were run. They still sit in the same buckets when the theme
matches — do not run them as 400 isolated nits.

**488** unique remaining (not cleared, not defer-home).
Titles live in Combined remaining buckets — do not treat unique as a
separate queue.

| n | Bucket |
|--:|--------|
| 15 | Dashboard / station CLI display lies |
| 1 | PSRAM init / QMI window |
| 15 | Log ring init / recover / named sizes |
| 6 | GPS UART / PMTK / wrong-core NVIC |
| 33 | Cross-core publication / fail-open |
| 33 | Phantom / missing symbols |
| 19 | Class-design / published guts |
| 1 | In-source NOLINT / suppressions |
| 7 | Process archaeology in comments |
| 209 | Comment still disagrees with the body |
| 6 | HW leakage vs domain code |
| 61 | Safety / ops SSOT (Go/No-Go, pyro, guards) |
| 2 | Test / inject / debug in the flight tree |
| 3 | File earn-rent / naming / packaging |
| 7 | Version / identity / config.h grab-bag |
| 32 | Fusion / math / cal live invariants |
| 5 | P10-9 function pointers |
| 33 | One-off / leftover |

---

## Combined remaining buckets

Same sitting style as the owner 16: one theme per sitting, related
notes in the same bucket even if they are not one bug. Do not open a
fourth taxonomy. New buckets below are **agent-only themes the owner
walk did not file** — that is the extra-walks harvest — not a new
process.

Inside a bucket: skip `cleared`, park `defer-home`, re-read `affected`,
do-now = `untouched` + surviving `affected`. Test in groups of 2–4
(R-10). **QMI disagree settled** (fence configure with detect).

**Per remaining item:** one-line summary of the claim + a suggested
label (REMEDIATE / previously resolved / park / DEFER-home) before
`src/` in that group. Do not auto-ACCEPT. Do not say skip for work
the tree already matches — say **previously resolved**. See plan
Phase 4.

**Graph first:** `graphify query` / `path` / `explain` before a
tree-wide grep or a speculative read of an overlay leaf. Open only
the files the graph names. After code edits, `graphify update .`.
If graph and tree disagree, the tree wins (graph can lag this
sitting). Stale nodes are not a skip. Plan Phase 4 has the full
rule.

| n remaining | Bucket | Later sitting |
|--:|--------|---------------|
| 1 | Grok vs Claude complete disagree | Stop. Owner settles before any code. |
| 23 | Dashboard / station CLI display lies | Owner-silent two-agree. Print/path bugs, not RC_OS menu structure. |
| 5 | PSRAM init / QMI window | Hard-gate, cached alias, QMI 1-vs-1. Do not silent-touch QMI. |
| 24 | Log ring init / recover / named sizes | ring_init vs recover, failed-init flag, kRcLogRingBytes. |
| 14 | GPS UART / PMTK / wrong-core NVIC | Live on the vehicle (Claude Reconciliation §1). |
| 56 | Cross-core publication / fail-open | Plain bools, missing barriers, pause fail-open, dual-home atomics. |
| 41 | Phantom / missing symbols | Named in comments or callers; not declared, or declared unused. |
| 20 | Class-design / published guts | Claude lens re-run: AO/driver headers publish the innards. Unique-heavy; still one sitting. |
| 1 | In-source NOLINT / suppressions | Same policy as owner bucket. Skip if already closed on that leaf. |
| 10 | Process archaeology in comments | Sitting 13 trimmed essays. Essay-only leftovers are cleared-policy, not a second archaeology sitting. |
| 228 | Comment still disagrees with the body | Re-read after sittings 5/13. Fix the live lie or delete the sentence. Not essay density. |
| 6 | HW leakage vs domain code | Sitting 4 closed most. Remaining = new agent P or DEFER riders. |
| 74 | Safety / ops SSOT (Go/No-Go, pyro, guards) | Live invariants, not comment style. |
| 3 | Test / inject / debug in the flight tree | Owner sitting closed. Remaining = new agent P. |
| 8 | File earn-rent / naming / packaging | Owner sitting closed KEEP/fold. Remaining = new agent P. |
| 9 | Version / identity / config.h grab-bag | config.h gone. R-9 version bump still owner-scheduled. |
| 34 | Fusion / math / cal live invariants | Numerical contracts, not comment style. |
| 5 | P10-9 function pointers | Owner GPS/watchdog remediates closed; QP-tied still DEFER. |
| 44 | One-off / leftover | Does not share a later sitting with a pile above. |

### ID lists (cite this in the plan)

**Grok vs Claude complete disagree:** `GWF-311` vs `CW-B26-05` (REFUTED, not in live Claude count)

Stop. Owner settles before any code.

**Dashboard / station CLI display lies** (23) — Owner-silent two-agree. Print/path bugs, not RC_OS menu structure.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B38-01` | affected | unique | `active_objects/ao_rcos.{cpp,h}` | MAVLink output mode routes USB input into the d… |
| `CW-B39-04` | affected | unique | `active_objects/ao_radio.{cpp,h}` | rx_count is documented as valid packets but is … |
| `CW-B43-01` | affected | agree | `cli/rc_os_commands.{cpp,h}` | Station distance staleness gate compares two un… |
| `CW-B43-05` | affected | agree | `cli/rc_os_commands.{cpp,h}` | Boot summary's FAIL count and FAIL list are com… |
| `CW-B44-01` | affected | agree | `cli/rc_os_dashboard.{cpp,h}` | Dashboard "Temp:" cell renders a literal 0 whil… |
| `CW-B44-02` | affected | agree | `cli/rc_os_dashboard.{cpp,h}` | "Alt:" and "Baro:" rows print the same variable… |
| `CW-B44-04` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | "Lost:" packet-loss counter pegs to zero for th… |
| `CW-B44-08` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | Dashboard render doxygen omits the rx parameter… |
| `CW-B44-09` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | rssi_bar accepts a buffer-size parameter and ex… |
| `CW-L022` | affected | unique | `(project-wide)` | F2 — AO_Radio's public header publishes the AO'… |
| `CW-X2-01` | affected | unique | `telemetry/telemetry_encoder.cpp` | FlightPhase numbering re-encoded as bare intege… |
| `CW-X2-05` | affected | unique | `flight_director/go_nogo_checks.…` | The "RF link is good enough" rule is expressed … |
| `CW-X2-08` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | The TelemetryState fixed-point decode is re-der… |
| `GWF-269` | affected | unique | `flight_director/guard_functions…` | Comment asserts descent through the altitude; t… |
| `GWF-481` | affected | agree | `cli/rc_os_commands.{cpp,h}` | cmd_station_distance claims a telemetry-stalene… |
| `GWF-485` | affected | agree | `cli/rc_os_commands.{cpp,h}` | Boot-summary FAIL count and FAIL list are not t… |
| `GWF-486` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | Phase-color comments and switch still document … |
| `GWF-487` | affected | agree | `cli/rc_os_dashboard.{cpp,h}` | Dashboard label claims a live temperature; the … |
| `GWF-488` | affected | agree | `cli/rc_os_dashboard.{cpp,h}` | Two labeled altitude fields are the same conver… |
| `GWF-489` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | Public render contract does not state the inher… |
| `GWF-490` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | Pause API is named as dashboard-wide; the other… |
| `GWF-491` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | File-level @file tag is a stale path from the o… |
| `GWF-492` | affected | unique | `cli/rc_os_dashboard.{cpp,h}` | Lead-in count disagrees with the documented and… |

**PSRAM init / QMI window** (5) — Hard-gate, cached alias, QMI 1-vs-1. Do not silent-touch QMI.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B26-03` | affected | agree | `logging/psram_init.{cpp,h}` | The "hard gate" flash-safe test claims erase+pr… |
| `CW-B26-04` | affected | agree | `logging/psram_init.{cpp,h}` | psram_self_test reads back through the cached a… |
| `CW-X5-05` | affected | unique | `logging/psram_init.{cpp,h}` | psram_flash_safe_test() is documented as a hard… |
| `GWF-309` | untouched | agree | `logging/psram_init.{cpp,h}` | Header and .cpp both promise an erase+program c… |
| `GWF-310` | affected | agree | `logging/psram_init.{cpp,h}` | Comment claims an addressing self-test of PSRAM… |

**Log ring init / recover / named sizes** (24) — ring_init vs recover, failed-init flag, kRcLogRingBytes.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B01-03` | affected | agree | `include/rocketchip/rc_log.h` | The LOCKED contract says drop-newest; the imple… |
| `CW-B25-01` | affected | agree | `log/rc_log.cpp` | rc_log ring's single-context safety argument is… |
| `CW-B25-02` | affected | unique | `log/rc_log.cpp` | truncation marker is silently omitted when a co… |
| `CW-B25-03` | affected | unique | `log/rc_log.cpp` | format_float has no magnitude domain guard; lar… |
| `CW-B25-04` | affected | unique | `logging/ring_buffer.{cpp,h}` | the crash-recovery seqlock is described but not… |
| `CW-B25-05` | affected | agree | `logging/ring_buffer.{cpp,h}` | a failed ring_init leaves the object marked ini… |
| `CW-B25-06` | affected | agree | `logging/ring_buffer.{cpp,h}` | the documented init-then-recover sequence destr… |
| `CW-B43-02` | affected | unique | `cli/rc_os_commands.{cpp,h}` | Download loop divides by an unvalidated flash-r… |
| `CW-L040` | affected | unique | `(project-wide)` | CF-1 — Crash-recovery header seqlock is written… |
| `GWF-009` | affected | agree | `include/rocketchip/rc_log.h` | Same header states opposite overflow policies: … |
| `GWF-014` | affected | unique | `include/rocketchip/rc_log.h` | Writer, allowed readers (any core vs Core 0 onl… |
| `GWF-018` | affected | unique | `include/rocketchip/config.h` | The DBG_* surface therefore does not provide on… |
| `GWF-281` | affected | unique | `log/rc_log.cpp` | The sink-overview comment was not updated when … |
| `GWF-282` | affected | unique | `log/rc_log.cpp` | Comment names tud_task as the drain agent; the … |
| `GWF-283` | affected | unique | `log/rc_log.cpp` | 'Raw spec' overstates what is echoed; a callsit… |
| `GWF-284` | affected | agree | `log/rc_log.cpp` | Public header promises drop-newest / drop-the-m… |
| `GWF-285` | affected | unique | `log/rc_log.cpp` | The frozen header still describes the rejected … |
| `GWF-286` | affected | unique | `log/rc_log.cpp` | Literal text honors the truncation-marker contr… |
| `GWF-287` | affected | agree | `log/rc_log.cpp` | Owner is asserted in this comment, not in the h… |
| `GWF-288` | affected | agree | `logging/ring_buffer.{cpp,h}` | Public init-then-recover protocol is unfulfilla… |
| `GWF-289` | affected | unique | `logging/ring_buffer.{cpp,h}` | Recover docs attribute a start-fresh side effec… |
| `GWF-290` | affected | unique | `logging/ring_buffer.{cpp,h}` | Documented seqlock owner/mutator is this module… |
| `GWF-291` | affected | unique | `logging/ring_buffer.{cpp,h}` | Struct comment disagrees with the accessor comm… |
| `GWF-292` | affected | agree | `logging/ring_buffer.{cpp,h}` | Failed init still publishes an initialized Ring… |

**GPS UART / PMTK / wrong-core NVIC** (14) — Live on the vehicle (Claude Reconciliation §1).

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B09-03` | untouched | agree | `drivers/gps_uart.{cpp,h}` | gps_uart_reinit() enables the UART IRQ on the w… |
| `CW-B09-04` | untouched | agree | `drivers/gps_uart.{cpp,h}` | cross-core ring declares volatile sufficient an… |
| `CW-B09-05` | untouched | unique | `drivers/gps_uart.{cpp,h}` | init/reinit preambles state a 9600-baud, 2-seco… |
| `CW-B09-07` | affected | agree | `drivers/gps_pa1010d.{cpp,h}` | PMTK314 comment claims GSV is enabled; the sent… |
| `CW-B14-05` | affected | agree | `fusion/eskf_runner.{cpp,h}` | g_eskf and g_eskfInitialized are read by Core 1… |
| `CW-B36-01` | affected | agree | `core1/sensor_core1.{cpp,h}` | Core 1 reads Core 0's live ESKF state with no b… |
| `CW-B36-03` | affected | unique | `core1/sensor_core1.{cpp,h}` | Shared GPS reader can block for seconds; both i… |
| `CW-L041` | affected | agree | `fusion/eskf_runner.{cpp,h}` | `g_eskf` is read from Core 1 with no synchroniz… |
| `GWF-008` | affected | unique | `include/rocketchip/shared_state…` | Cross-core function-pointer table is promised s… |
| `GWF-125` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | Comment lists GSV as enabled; the sentence body… |
| `GWF-132` | untouched | agree | `drivers/gps_uart.{cpp,h}` | Sentence comment lists GSV as enabled; the lite… |
| `GWF-135` | untouched | agree | `drivers/gps_uart.{cpp,h}` | reinit ownership and barrier versus the documen… |
| `GWF-402` | affected | unique | `core1/sensor_core1.{cpp,h}` | The contract comment hides control-plane side e… |
| `GWF-405` | affected | unique | `core1/sensor_core1.{cpp,h}` | Comment labels a control decision as diagnostic… |

**Cross-core publication / fail-open** (56) — Plain bools, missing barriers, pause fail-open, dual-home atomics.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B01-01` | affected | agree | `include/rocketchip/shared_state…` | Two cross-core init flags are plain `bool` with… |
| `CW-B10-03` | affected | unique | `drivers/icm20948.{cpp,h}` | Magnetometer read phase is hidden static state … |
| `CW-B12-07` | affected | unique | `drivers/mcu_temp.{cpp,h}` | mcu_temp stuck-detector state is written on Cor… |
| `CW-B18-01` | affected | agree | `calibration/calibration_manager…` | Calibration collection state is written from bo… |
| `CW-B18-03` | affected | unique | `calibration/cal_hooks.{cpp,h}` | cal_hooks.h declares four callbacks and states … |
| `CW-B22-01` | affected | unique | `flight_director/go_nogo_checks.…` | Go/No-Go station overflow fails open on the ARM… |
| `CW-B30-02` | affected | agree | `station/station_idle_tick.{cpp,…` | GPS-init guard also suppresses MCU-temp capture… |
| `CW-B31-04` | affected | agree | `safety/fault_protection.{cpp,h}` | One non-atomic flag serves as a reentrance guar… |
| `CW-B32-01` | affected | unique | `safety/crash_record.{cpp,h}` | crash_record_capture() does not enforce the "ma… |
| `CW-B34-02` | affected | agree | `safety/core1_i2c_pause.{cpp,h}` | pause/resume do not nest, though the code's own… |
| `CW-B34-03` | affected | agree | `safety/core1_i2c_pause.{cpp,h}` | pause() can return without an acknowledgement, … |
| `CW-B36-02` | affected | unique | `core1/sensor_core1.{cpp,h}` | Core 1 mutates the Core-0-owned calibration mod… |
| `CW-B41-01` | affected | unique | `main.cpp` | Boot hangs forever in baro auto-zero on any non… |
| `CW-B41-03` | affected | unique | `main.cpp` | Unconditional g_gpsInitAttempted defeats the at… |
| `CW-B41-04` | affected | agree | `include/rocketchip/shared_state…` | Two init flags are written from both cores as p… |
| `CW-B41-05` | affected | agree | `include/rocketchip/shared_state…` | Ownership annotations on the contract surface n… |
| `CW-X1-07` | affected | unique | `core1/sensor_core1.{cpp,h}` | The NeoPixel driver is written from both cores … |
| `CW-X1-08` | affected | unique | `main.cpp` | A Core 0 I2C sensor reader is bound into the CL… |
| `CW-X4-09` | affected | agree | `include/rocketchip/shared_state…` | The cross-core ownership map is declared twice,… |
| `CW-X5-03` | affected | unique | `main.cpp` | init_gps_early() inverts the documented GPS ini… |
| `CW-X5-04` | affected | agree | `main.cpp` | the boot-time PSRAM flash-safe test runs an unp… |
| `GWF-001` | affected | agree | `include/rocketchip/shared_state…` | File-level ownership story disagrees with the p… |
| `GWF-002` | affected | agree | `include/rocketchip/shared_state…` | Explicitly cross-core gating flag has owner and… |
| `GWF-003` | affected | agree | `include/rocketchip/shared_state…` | Init flags that Core 1 may write (and Core 0 is… |
| `GWF-004` | affected | agree | `include/rocketchip/shared_state…` | Contract surface leaves writer and visibility (… |
| `GWF-005` | affected | unique | `include/rocketchip/shared_state…` | Six shared atomics have a barrier and no stated… |
| `GWF-007` | affected | agree | `include/rocketchip/shared_state…` | Seqlock and signaling atomics have two declarat… |
| `GWF-013` | affected | unique | `include/rocketchip/rc_log.h` | No owner/mutator/barrier for the implied ring: … |
| `GWF-031` | affected | unique | `include/rocketchip/board_fruit_…` | Shared reset line has a public mutator with sid… |
| `GWF-123` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Shared init flag and bus hardware have ambiguou… |
| `GWF-168` | affected | unique | `drivers/ws2812_status.{cpp,h}` | Two writer families on one LED chain with no ex… |
| `GWF-171` | affected | agree | `fusion/eskf_runner.{cpp,h}` | Cross-core shared object g_eskf: owner is this … |
| `GWF-183` | affected | unique | `fusion/eskf.{cpp,h}` | File-static mutators have an instance-global al… |
| `GWF-208` | affected | unique | `fusion/ud_factor.{cpp,h}` | File-static g_bf, g_bg, g_bK, and g_balpha are … |
| `GWF-222` | affected | agree | `calibration/calibration_manager…` | Documented cross-core session objects: owner/mu… |
| `GWF-234` | affected | unique | `calibration/cal_hooks.{cpp,h}` | Comments present an always-on post-save signal.… |
| `GWF-340` | affected | unique | `station/station_idle_tick.{cpp,…` | Header banner still describes a no-op IVP-140 s… |
| `GWF-341` | affected | agree | `station/station_idle_tick.{cpp,…` | Comment attributes seqlock visibility to an unp… |
| `GWF-342` | affected | agree | `station/station_idle_tick.{cpp,…` | Owner (include: Core 1) and mutator (this Core … |
| `GWF-343` | affected | unique | `station/station_idle_tick.{cpp,…` | Comment uses a stale identifier that does not m… |
| `GWF-361` | affected | unique | `safety/health_monitor.{cpp,h}` | Cross-core init-flag reads have no barrier here… |
| `GWF-368` | affected | unique | `safety/fault_inject.{cpp,h}` | This leaf mutates a foreign plain bool via a fi… |
| `GWF-381` | affected | unique | `safety/test_mode.{cpp,h}` | g_phaseAccessor is a shared function pointer th… |
| `GWF-383` | untouched | agree | `safety/core1_i2c_pause.{cpp,h}` | Public pause contract is success-by-return; the… |
| `GWF-386` | affected | agree | `safety/core1_i2c_pause.{cpp,h}` | The nest example implies stacked pause sessions… |
| `GWF-404` | affected | agree | `core1/sensor_core1.{cpp,h}` | Owner is Core 0 fusion; mutator is not in this … |
| `GWF-417` | affected | unique | `active_objects/ao_rcos.{cpp,h}` | g_outputMode is a plain static enum. Mutators a… |
| `GWF-425` | affected | agree | `active_objects/ao_logger.{cpp,h}` | Two-AO writers of hidden rate state; comments m… |
| `GWF-454` | affected | unique | `active_objects/ao_notify.{cpp,h}` | Cross-core GPS-init gate is an unsynchronized p… |
| `GWF-460` | affected | unique | `active_objects/ao_led_engine.{c…` | Shared override has no barrier in the type and … |
| `GWF-462` | affected | unique | `main.cpp` | The same file's init_gps_early() calls gps_pa10… |
| `GWF-463` | affected | unique | `main.cpp` | That UART-first policy is not the actual bring-… |
| `GWF-469` | affected | unique | `shared_state.cpp` | Owner is contradictory (Core 0 init vs Core 1 w… |
| `GWF-470` | affected | unique | `shared_state.cpp` | A cross-core gate is a non-atomic bool. Owner a… |
| `GWF-482` | affected | unique | `cli/rc_os_commands.{cpp,h}` | Shared T2 command slot has ambiguous barrier: v… |
| `GWF-498` | affected | unique | `cli/rc_os_debug.{cpp,h}` | The public mutator promised as test-mode-gated … |

**Phantom / missing symbols** (41) — Named in comments or callers; not declared, or declared unused.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B01-04` | affected | agree | `include/rocketchip/rc_log.h` | Ring-health guidance names a constant that does… |
| `CW-B02-08` | affected | agree | `include/rocketchip/board_tiny_2…` | Tiny common preamble points at a header and a c… |
| `CW-B03-03` | affected | unique | `include/rocketchip/job.h` | The role-header contract is unwritten: neither … |
| `CW-B05-02` | affected | unique | `include/rocketchip/telemetry_en…` | nav payload contract derives 42 bytes from a Te… |
| `CW-B05-13` | affected | unique | `include/rocketchip/sensor_snaps…` | SensorSnapshot's stated use does not exist anyw… |
| `CW-B06-04` | affected | agree | `include/rocketchip/flash_layout…` | flash_layout_valid() documents a binary_end par… |
| `CW-B06-05` | untouched | agree | `include/rocketchip/fused_state.h` | fused_state.h names eskf_to_fused_state() as it… |
| `CW-B06-08` | affected | unique | `include/rocketchip/led_patterns…` | led_patterns.h usage prose is stale in two plac… |
| `CW-B10-05` | affected | agree | `drivers/baro_dps310.{cpp,h}` | DPS310 configuration block attributes a fabrica… |
| `CW-B12-03` | affected | unique | `drivers/ws2812_status.{cpp,h}` | RSSI-bar header contract disagrees with the imp… |
| `CW-B20-03` | affected | unique | `flight_director/flight_state.h` | Pyro-fired latch comment describes a RESET clea… |
| `CW-B24-01` | affected | unique | `flight_director/mission_profile…` | Profile-selection contract described in the hea… |
| `CW-B29-02` | affected | unique | `notify/notify_backend_audio.cpp` | Audio tone table is unreachable dead data with … |
| `CW-B30-04` | affected | unique | `telemetry/mavlink_rx.cpp` | Pre-Flight-Director placeholders ACK safety com… |
| `CW-B37-04` | affected | unique | `active_objects/ao_flight_direct…` | Queue-depth-32 rationale cites a blocking call … |
| `CW-B38-06` | affected | unique | `active_objects/ao_rcos.{cpp,h}` | abandoned design deliberation left as comments,… |
| `CW-B39-06` | affected | unique | `active_objects/ao_radio.{cpp,h}` | The TX-fail reinit recovery ignores the rfm95w_… |
| `CW-B42-01` | affected | unique | `cli/rc_os.{cpp,h}` | MAVLink input handler is unreachable; its docum… |
| `CW-B42-06` | affected | unique | `cli/rc_os.{cpp,h}` | Ownership comment on rc_os_mag_cal_active names… |
| `CW-B43-06` | affected | unique | `cli/rc_os_commands.{cpp,h}` | ESKF buffer capacity is a local copy of another… |
| `CW-B43-09` | affected | unique | `cli/rc_os_commands.{cpp,h}` | Three doc comments describe code that does not … |
| `CW-X3-04` | affected | unique | `safety/fault_protection.{cpp,h}` | deviation ID FH-1 is cited from code but has no… |
| `CW-X3-05` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | flight-tuning constants cite a plan document th… |
| `CW-X4-10` | affected | unique | `include/rocketchip/sensor_snaps…` | sensor_snapshot.h is a public layout contract w… |
| `GWF-011` | affected | agree | `include/rocketchip/rc_log.h` | kRcLogRingBytes is not declared on this surface… |
| `GWF-021` | affected | unique | `include/rocketchip/board.h` | The comment names PICO_BOARD as the selection k… |
| `GWF-036` | affected | agree | `include/rocketchip/board_tiny_2…` | board_tiny_2350.h is not present beside this he… |
| `GWF-037` | affected | unique | `include/rocketchip/board_tiny_2…` | "Overrides" and "keeps false" imply a false def… |
| `GWF-045` | affected | unique | `include/rocketchip/job_capabili…` | The station/relay commentary asserts an uncondi… |
| `GWF-046` | untouched | unique | `include/rocketchip/job_relay.h` | The file doc-comment publishes CRC validation, … |
| `GWF-049` | untouched | unique | `include/rocketchip/job_station.h` | The comment says station is the same binary as … |
| `GWF-050` | affected | unique | `include/rocketchip/job_vehicle.h` | Public contract names DeviceRole and DeviceRole… |
| `GWF-074` | affected | unique | `include/rocketchip/telemetry_en…` | Nav-payload comment names a field that does not… |
| `GWF-086` | affected | unique | `include/rocketchip/led_patterns…` | The posting-path comment names a type that does… |
| `GWF-093` | untouched | agree | `include/rocketchip/fused_state.h` | File-level owner/mutator/source contract names … |
| `GWF-258` | affected | unique | `flight_director/guard_evaluator…` | The combinator contract is named as an array th… |
| `GWF-339` | affected | unique | `telemetry/telemetry_encoder.cpp` | Comment cites a header encoding that does not e… |
| `GWF-376` | affected | unique | `safety/test_mode.{cpp,h}` | Comment names a symbol that does not exist and … |
| `GWF-409` | affected | unique | `active_objects/ao_flight_direct…` | Public header names a catalog time signal the b… |
| `GWF-466` | affected | unique | `main.cpp` | That main-loop contract does not exist after th… |
| `GWF-496` | affected | unique | `cli/rc_os_debug.{cpp,h}` | Load-bearing routing comment names a function t… |

**Class-design / published guts** (20) — Claude lens re-run: AO/driver headers publish the innards. Unique-heavy; still one sitting.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B01-07` | affected | unique | `include/rocketchip/shared_state…` | Three function pointers on a public contract su… |
| `CW-B02-02` | affected | unique | `include/rocketchip/board_pico2.h` | board_pico2.h omits kPsramCsPin, which config.h… |
| `CW-B05-06` | affected | unique | `include/rocketchip/telemetry_en…` | opaque parser_buf size claim is hand-computed a… |
| `CW-B07-06` | affected | unique | `(project-wide)` | These three headers are outside the clang-tidy … |
| `CW-B12-02` | affected | unique | `drivers/ws2812_status.{cpp,h}` | ws2812_status_init accepts any num_leds: the bo… |
| `CW-B37-02` | affected | unique | `active_objects/ao_flight_direct…` | Five raw function-pointer callbacks wired here,… |
| `CW-B39-01` | affected | agree | `active_objects/ao_radio.{cpp,h}` | ROCKETCHIP_RADIO_PERSIST branches reference fou… |
| `CW-L021` | affected | unique | `(project-wide)` | F1 — AO_Logger publishes unrestricted mutable p… |
| `CW-L023` | affected | unique | `(project-wide)` | F3 — AO_FlightDirector hands out the whole HSM … |
| `CW-L024` | affected | unique | `(project-wide)` | F4 — AO_Telemetry's public interface carries tw… |
| `CW-L026` | affected | unique | `(project-wide)` | `g_state.numLeds` can exceed the `pixels[8]` ca… |
| `CW-L027` | affected | unique | `(project-wide)` | `rfm95w_t`'s accessor surface is incoherent — a… |
| `CW-L028` | affected | unique | `(project-wide)` | `icm20948_t` is a public struct carrying a real… |
| `CW-L029` | affected | unique | `(project-wide)` | The WS2812 singleton's period setters admit zer… |
| `CW-L030` | affected | unique | `(project-wide)` | `icm20948_t` presents as a per-device handle, b… |
| `CW-L031` | affected | unique | `(project-wide)` | `gps_uart_init()` claims an exclusive NVIC hand… |
| `CW-L032` | affected | unique | `(project-wide)` | CD-1 — `RadioScheduler` publishes its state mac… |
| `CW-L033` | affected | unique | `(project-wide)` | CD-2 — `MavlinkRxState` publishes parser scratc… |
| `CW-L036` | affected | unique | `(project-wide)` | CD-5 — `sensor_seqlock_t` publishes the data it… |
| `CW-X4-07` | affected | unique | `include/rocketchip/ao_signals.h` | ao_signals.h declares a signal-name lookup that… |

**In-source NOLINT / suppressions** (1) — Same policy as owner bucket. Skip if already closed on that leaf.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B42-08` | affected | unique | `cli/rc_os.{cpp,h}` | Orphaned NOLINTNEXTLINE attaches a size-exempti… |

**Process archaeology in comments** (10) — Sitting 13 trimmed essays. Essay-only leftovers are cleared-policy, not a second archaeology sitting.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B06-03` | affected | agree | `include/rocketchip/flash_layout…` | Flash-layout banner omits the radio-config regi… |
| `CW-B07-02` | affected | unique | `math/mat.h` | fpft_dense claims a verification role that a se… |
| `CW-B12-05` | affected | unique | `drivers/mcu_temp.{cpp,h}` | mcu_temp_init is documented as returning succes… |
| `CW-B20-07` | affected | agree | `flight_director/flight_actions.h` | Exit-action machinery is documented as executed… |
| `CW-B29-04` | affected | unique | `notify/notify_resolver.h` | Resolver contract omits the beacon overlay that… |
| `CW-B41-08` | affected | unique | `main.cpp` | Stale watchdog-timeout constant in main.cpp, wi… |
| `GWF-096` | affected | agree | `include/rocketchip/flash_layout…` | File-level layout diagram is stale versus the d… |
| `GWF-202` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Documented health contract omits the initialize… |
| `GWF-321` | affected | unique | `diag/diag_stats.{cpp,h}` | Comments classify msp_tick with dump as a gated… |
| `GWF-461` | affected | unique | `main.cpp` | The comment is attached to a void init_hardware… |

**Comment still disagrees with the body** (228) — Re-read after sittings 5/13. Fix the live lie or delete the sentence. Not essay density.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B02-01` | untouched | unique | `include/rocketchip/board_feathe…` | Capability flags are declared as a consumed con… |
| `CW-B02-04` | affected | unique | `include/rocketchip/board_fruit_…` | kNeoPixelGpioBase states a hardware requirement… |
| `CW-B02-07` | untouched | agree | `include/rocketchip/board_feathe…` | LED polarity is stated twice: a constant nothin… |
| `CW-B04-01` | affected | unique | `include/rocketchip/notify_backe…` | notify_backend.h claims a priority resolution t… |
| `CW-B04-03` | affected | unique | `include/rocketchip/radio_config…` | table banner names itself the SET_RADIO_CONFIG … |
| `CW-B06-01` | affected | unique | `include/rocketchip/ao_signals.h` | Header tells authors QP events may be stack-all… |
| `CW-B06-02` | affected | unique | `include/rocketchip/ao_signals.h` | Two event structs are annotated "allocated from… |
| `CW-B07-01` | affected | unique | `math/quat.{cpp,h}` | Quaternion-to-rotation conversions silently req… |
| `CW-B07-03` | affected | unique | `math/quat.{cpp,h}` | from_two_vectors discards rotations below ~0.81… |
| `CW-B07-04` | affected | unique | `math/vec3.{cpp,h}` | vec3.h declares two non-total operations with n… |
| `CW-B08-01` | affected | unique | `drivers/spi_bus.{cpp,h}` | SPI error counter cannot increment, yet a soak … |
| `CW-B08-02` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Recovery/scan entry points document no assumpti… |
| `CW-B08-03` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Both driver headers document one board's instan… |
| `CW-B08-04` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Bus-recovery idiom open-coded twice and already… |
| `CW-B08-05` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Unreachable GPS identify-branch in the bus scan |
| `CW-B08-06` | affected | unique | `drivers/spi_bus.{cpp,h}` | spi_bus_init drops spi_init's return and cannot… |
| `CW-B09-02` | untouched | unique | `drivers/gps_uart.{cpp,h}` | GSA fixMode of "no fix" or "not yet received" i… |
| `CW-B09-06` | untouched | unique | `drivers/gps_uart.{cpp,h}` | the comment that authorises the duplicated back… |
| `CW-B09-08` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | read_nmea_data()'s `max_len` bounds a different… |
| `CW-B09-09` | affected | unique | `drivers/lwgps_opts.h` | lwgps_opts.h labels a disabled option "Enable" |
| `CW-B12-01` | affected | unique | `drivers/ws2812_status.{cpp,h}` | ws2812_status_init discards the caller's PIO in… |
| `CW-B12-04` | affected | unique | `drivers/mcu_temp.{cpp,h}` | mcu_temp.h states ADC input 4 while the impleme… |
| `CW-B12-06` | affected | unique | `drivers/ws2812_status.{cpp,h}` | ws2812_set_sweep_bar documentation contradicts … |
| `CW-B12-08` | affected | unique | `drivers/ws2812_status.{cpp,h}` | update_blink lacks the zero-period guard its si… |
| `CW-B13-01` | affected | unique | `fusion/ud_factor.{cpp,h}` | ud_factor.cpp file header documents code that i… |
| `CW-B13-02` | affected | unique | `fusion/eskf.{cpp,h}` | Joseph-form and Mat15 claims survive the remova… |
| `CW-B13-03` | affected | unique | `fusion/eskf.{cpp,h}` | documented sync_dense_covariance() precondition… |
| `CW-B13-04` | affected | unique | `fusion/eskf.{cpp,h}` | 3-axis mag fusion applies an identity Jacobian … |
| `CW-B13-05` | affected | unique | `fusion/ud_factor.{cpp,h}` | ud_factor.h preamble gives parameter meanings b… |
| `CW-B13-06` | affected | unique | `fusion/eskf.{cpp,h}` | the two update_zupt overloads duplicate the who… |
| `CW-B13-07` | affected | unique | `fusion/eskf.{cpp,h}` | predict_dense is documented as equivalent to pr… |
| `CW-B13-08` | affected | unique | `fusion/eskf.{cpp,h}` | healthy() does not check the "P bounds" its con… |
| `CW-B14-01` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Blocking flash write issued from the 200 Hz fus… |
| `CW-B14-03` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Runaway-restart brake is documented as CLI-clea… |
| `CW-B14-07` | affected | unique | `fusion/eskf_runner.{cpp,h}` | The mission-profile pointer is null-guarded in … |
| `CW-B15-01` | affected | unique | `fusion/confidence_gate.{cpp,h}` | ConfidenceInput cannot express "AHRS cross-chec… |
| `CW-B15-03` | affected | unique | `fusion/innovation_monitor.{cpp,…` | innovation_channel_push silently discards anoma… |
| `CW-B15-04` | affected | unique | `fusion/confidence_gate.{cpp,h}` | The gate initialises to the trusting state with… |
| `CW-B16-03` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Mahony converges to magnetic north while the ES… |
| `CW-B16-04` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | init() discards the tilt quaternion's own yaw w… |
| `CW-B16-05` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | init() does not reset the startup-boost flag, a… |
| `CW-B16-06` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | header preamble states the wrong ESKF state cou… |
| `CW-B16-08` | untouched | unique | `fusion/wmm_tables.{cpp,h}` | declination is bilinearly averaged across the 1… |
| `CW-B17-01` | affected | unique | `calibration/calibration_storage…` | Single-page write budget is unguarded; the guar… |
| `CW-B17-02` | affected | unique | `calibration/calibration_data.{c…` | The definition of "the CRC region" is written t… |
| `CW-B17-03` | affected | unique | `calibration/calibration_storage…` | calibration_storage_init discards the only sign… |
| `CW-B17-04` | affected | unique | `calibration/calibration_storage…` | Storage contract does not state that its calls … |
| `CW-B17-05` | affected | unique | `calibration/calibration_data.{c…` | Comment still documents the 0-latitude sentinel… |
| `CW-B17-06` | affected | unique | `calibration/calibration_storage…` | File banner pins absolute flash addresses that … |
| `CW-B17-07` | affected | unique | `calibration/calibration_data.{c…` | Per-sensor status fields duplicate cal_flags an… |
| `CW-B18-04` | affected | unique | `calibration/calibration_manager…` | The header's const accessor is the only handle … |
| `CW-B19-04` | affected | unique | `calibration/lm_solver.{cpp,h}` | bestParams / bestFitness are in-out, but the pr… |
| `CW-B20-01` | affected | agree | `flight_director/flight_state.h` | File banner states eight phases; the enum defin… |
| `CW-B20-02` | affected | unique | `flight_director/flight_director…` | Both banners assert ABORT fires drogue; the shi… |
| `CW-B20-04` | affected | unique | `flight_director/flight_director…` | ABORT timeout branches on flight markers that s… |
| `CW-B20-05` | affected | agree | `flight_director/flight_actions.h` | FAULT entry actions are unreachable, and their … |
| `CW-B23-01` | affected | unique | `flight_director/guard_functions…` | guard_baro_peak is true while ascending, not wh… |
| `CW-B23-02` | affected | unique | `flight_director/guard_combinato…` | Apogee "dual-channel" claim rests on two guards… |
| `CW-B24-04` | affected | unique | `flight_director/mission_profile…` | Three cfg options documented as profile setting… |
| `CW-B26-01` | affected | unique | `logging/flash_flush.{cpp,h}` | First two flight-table saves both target sector… |
| `CW-B26-06` | affected | unique | `logging/flash_flush.{cpp,h}` | A failed ring read is swallowed, and the flush … |
| `CW-B26-07` | affected | unique | `logging/psram_init.{cpp,h}` | The flash_safe_execute trampoline is duplicated… |
| `CW-B27-01` | affected | agree | `logging/flight_table.{cpp,h}` | flight_table.h banner flash map contradicts the… |
| `CW-B27-02` | affected | unique | `logging/log_decimator.{cpp,h}` | Decimator silently freezes the confidence-gate … |
| `CW-B27-03` | affected | unique | `logging/data_convert.{cpp,h}` | Quantization bounds are published as the ICD wi… |
| `CW-B27-04` | affected | unique | `logging/data_convert.{cpp,h}` | Clamp helpers are shaped as total guards but pa… |
| `CW-B27-05` | affected | unique | `logging/flight_table.{cpp,h}` | flight_table_add_entry lets the caller's sector… |
| `CW-B27-06` | affected | unique | `logging/flight_table.{cpp,h}` | FlightLogEntry's metadata size comment understa… |
| `CW-B28-04` | affected | unique | `logging/radio_config_storage.{c…` | Write/erase contract cites LL Entry 31 but omit… |
| `CW-B29-01` | affected | unique | `notify/notify_backend_led.cpp` | LED backend documents itself as not yet wired, … |
| `CW-B29-05` | affected | unique | `diag/diag_stats.{cpp,h}` | "Pure read-only, no state mutation, no risk" is… |
| `CW-B36-04` | affected | unique | `core1/sensor_core1.{cpp,h}` | Core 1's pause indicator overwrites the LED com… |
| `CW-B37-01` | affected | unique | `active_objects/ao_flight_direct…` | Auto-DISARM on critical fault leaves the PIO ba… |
| `CW-B37-05` | affected | agree | `active_objects/ao_health_monito…` | Startup health publish reaches zero subscribers |
| `CW-B37-06` | affected | unique | `active_objects/ao_flight_direct…` | Pyro-fired latch is written on four paths and r… |
| `CW-B38-03` | affected | unique | `active_objects/ao_logger.{cpp,h}` | "keep previous rate" comment is not what the bo… |
| `CW-B38-07` | affected | unique | `active_objects/ao_logger.{cpp,h}` | function banner says "Non-static ... Public via… |
| `CW-B40-04` | affected | unique | `active_objects/ao_notify.{cpp,h}` | The "sensor phase" 5-minute timeout is an uncon… |
| `CW-B40-05` | affected | unique | `active_objects/ao_notify.{cpp,h}` | Seven extraction comments cite "JSF AV Rule 1" … |
| `CW-B40-09` | affected | agree | `active_objects/ao_led_engine.{c…` | The dev-only override is documented as forcing … |
| `CW-B41-02` | affected | unique | `main.cpp` | Early GPS bring-up runs the exact sequence that… |
| `CW-B41-07` | affected | unique | `include/rocketchip/shared_state…` | Dead entries on the shared-state contract: one … |
| `CW-X2-03` | affected | unique | `calibration/calibration_data.{c…` | CRC-16-CCITT implemented twice, in two modules,… |
| `CW-X2-04` | affected | unique | `logging/radio_config_storage.{c…` | The dual-sector flash persistence protocol is i… |
| `CW-X5-07` | affected | unique | `active_objects/ao_flight_direct…` | the PIO backup-fire report latches are set once… |
| `CW-X5-08` | affected | agree | `flight_director/flight_director…` | the flight-in-progress sentinel is set on ARM b… |
| `GWF-010` | affected | unique | `include/rocketchip/rc_log.h` | Two different agents are named as the thing tha… |
| `GWF-022` | affected | unique | `include/rocketchip/board.h` | "same-binary builds" disagrees with a compile-t… |
| `GWF-024` | affected | unique | `include/rocketchip/board.h` | Load-bearing #include "pico/stdlib.h" has no co… |
| `GWF-026` | untouched | unique | `include/rocketchip/board_feathe…` | The LED setter promised by this header is not b… |
| `GWF-030` | affected | unique | `include/rocketchip/board_fruit_…` | Polarity is promised twice and not coupled. The… |
| `GWF-032` | affected | unique | `include/rocketchip/board_fruit_…` | The pin constants are labeled unused but alias … |
| `GWF-035` | affected | unique | `include/rocketchip/board_pico2.h` | GPIO 0 is dual-owned: NeoPixel sentinel versus … |
| `GWF-044` | affected | unique | `include/rocketchip/job.h` | The advertised interface does not match the pre… |
| `GWF-047` | untouched | unique | `include/rocketchip/job_relay.h` | The comment attaches receive-then-re-TX to kRad… |
| `GWF-051` | affected | unique | `include/rocketchip/notify_backe…` | Comments inside this header disagree on the aud… |
| `GWF-052` | affected | unique | `include/rocketchip/notify_inten…` | The 1:1 FlightPhase map is false on numeric val… |
| `GWF-053` | affected | unique | `include/rocketchip/notify_inten…` | The published priority ladder and the five-slot… |
| `GWF-056` | affected | unique | `include/rocketchip/radio_config…` | Banner contract and later comment/API disagree … |
| `GWF-057` | affected | unique | `include/rocketchip/radio_config…` | Struct field comments state a single supported … |
| `GWF-058` | affected | unique | `include/rocketchip/radio_config…` | Comment describes an airtime-headroom contract … |
| `GWF-059` | affected | unique | `include/rocketchip/radio_config…` | Comments disagree with each other on whether ch… |
| `GWF-069` | affected | unique | `include/rocketchip/sensor_snaps…` | File brief disagrees with the payload: GPS (and… |
| `GWF-070` | affected | unique | `include/rocketchip/sensor_snaps…` | Baro contract is self-contradictory: identifier… |
| `GWF-077` | affected | unique | `include/rocketchip/telemetry_st…` | File brief overstates the quantization story; t… |
| `GWF-078` | affected | unique | `include/rocketchip/telemetry_st…` | Decoder/type pointer is stale; health nibble se… |
| `GWF-084` | affected | unique | `include/rocketchip/ao_signals.h` | RadioTxEvt (line 145) and RadioRxEvt (line 157)… |
| `GWF-094` | untouched | unique | `include/rocketchip/fused_state.h` | Comment, field names, and unit annotations all … |
| `GWF-095` | untouched | agree | `include/rocketchip/fused_state.h` | The group comment presents vert_vel as barometr… |
| `GWF-097` | affected | agree | `include/rocketchip/flash_layout…` | Doc-comment describes a runtime boot API and a … |
| `GWF-107` | affected | unique | `math/quat.{cpp,h}` | The DCM implementation is attributed to Sola (2… |
| `GWF-108` | affected | unique | `math/quat.{cpp,h}` | The sandwich rotate expansion is attributed to … |
| `GWF-109` | affected | unique | `math/quat.{cpp,h}` | Euler extraction is attributed to Sola (2017) E… |
| `GWF-110` | affected | unique | `math/quat.{cpp,h}` | from_small_angle is attributed to Sola (2017) E… |
| `GWF-111` | affected | unique | `math/quat.{cpp,h}` | Adjacent public-API comments name opposite angl… |
| `GWF-113` | affected | unique | `math/quat.{cpp,h}` | The comment requires a unit axis. |
| `GWF-114` | affected | unique | `math/mat.h` | The return-value comment is stale: it names a 3… |
| `GWF-115` | affected | unique | `math/mat.h` | Name and lead comment present this as a measure… |
| `GWF-116` | affected | unique | `math/mat.h` | Inline comments state the textbook formulas and… |
| `GWF-117` | affected | unique | `math/mat.h` | Comments disagree with the actual return contra… |
| `GWF-118` | affected | unique | `drivers/i2c_bus.{cpp,h}` | File brief hard-codes Feather STEMMA QT wiring … |
| `GWF-119` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Public reset contract (sequence and success mea… |
| `GWF-120` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Recover's documented surface understates a full… |
| `GWF-121` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Probe is documented as an ACK-only presence che… |
| `GWF-122` | affected | unique | `drivers/i2c_bus.{cpp,h}` | Scan brief over-promises 'all' devices, and the… |
| `GWF-124` | affected | unique | `drivers/i2c_bus.{cpp,h}` | imu_recovery's success contract silently depend… |
| `GWF-126` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | Header parenthetical 'data received' disagrees … |
| `GWF-127` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | PA1010D-specific header treats this GPS as 10 H… |
| `GWF-128` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | Header attributes the capture to a function thi… |
| `GWF-130` | untouched | unique | `drivers/gps_uart.{cpp,h}` | File brief and gps_uart_init() docstring disagr… |
| `GWF-131` | untouched | unique | `drivers/gps_uart.{cpp,h}` | Public 10Hz contract and receive-path comments … |
| `GWF-133` | untouched | unique | `drivers/gps_uart.{cpp,h}` | ISR timing/CPU comments describe factory baud, … |
| `GWF-134` | untouched | unique | `drivers/gps_uart.{cpp,h}` | Header return contract overstates what the func… |
| `GWF-136` | untouched | unique | `drivers/gps_uart.{cpp,h}` | Loss-free claim disagrees with the overflow pat… |
| `GWF-137` | untouched | unique | `drivers/gps_uart.{cpp,h}` | Timeout comments understate the two-baud probe … |
| `GWF-147` | affected | agree | `drivers/baro_dps310.{cpp,h}` | Datasheet paraphrase is mis-cited and internall… |
| `GWF-157` | affected | unique | `drivers/spi_bus.{cpp,h}` | Function doc contradicts both the file-level bo… |
| `GWF-158` | affected | unique | `drivers/mcu_temp.{cpp,h}` | The public header states a single ADC input 4 f… |
| `GWF-160` | affected | unique | `drivers/mcu_temp.{cpp,h}` | The header’s 60-identical-read rule does not ma… |
| `GWF-161` | affected | unique | `drivers/mcu_temp.{cpp,h}` | The stated LSB step disagrees with the conversi… |
| `GWF-162` | affected | unique | `drivers/mcu_temp.{cpp,h}` | Name and comment say raw ADC storage; the value… |
| `GWF-163` | affected | unique | `drivers/ws2812_status.{cpp,h}` | Header RSSI-bar contract (range, color bands, n… |
| `GWF-164` | affected | unique | `drivers/ws2812_status.{cpp,h}` | Sweep API documents a self-timed cadence the bo… |
| `GWF-167` | affected | unique | `drivers/ws2812_status.{cpp,h}` | Alternate-mode timing comment disagrees with it… |
| `GWF-169` | affected | unique | `drivers/lwgps_opts.h` | Comment states the status callback is enabled, … |
| `GWF-170` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Banner comment disagrees with control flow: SIG… |
| `GWF-177` | affected | unique | `fusion/eskf.{cpp,h}` | Measurement-update comments disagree with the B… |
| `GWF-179` | affected | unique | `fusion/eskf.{cpp,h}` | Mag-heading measurement-model comment has the i… |
| `GWF-182` | affected | unique | `fusion/eskf.{cpp,h}` | Inhibit-enable comments describe an assert and … |
| `GWF-194` | affected | unique | `fusion/confidence_gate.{cpp,h}` | These comments only restate the next obvious st… |
| `GWF-195` | affected | unique | `fusion/innovation_monitor.{cpp,…` | Header treats high NIS as a definite Q-vs-senso… |
| `GWF-196` | affected | unique | `fusion/innovation_monitor.{cpp,…` | Comment names only the non-finite check; the bo… |
| `GWF-198` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Skipped-mag path does not force yaw=0; comments… |
| `GWF-199` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Tilt-comp comments disagree with the rotation u… |
| `GWF-200` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Member comment describes automatic time expiry … |
| `GWF-201` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Comments say gain decay; body is a hard step fr… |
| `GWF-205` | affected | unique | `fusion/ud_factor.{cpp,h}` | Comment-truth mismatch: nothing in this file ze… |
| `GWF-206` | affected | unique | `fusion/ud_factor.{cpp,h}` | Module comment overstates a representation prop… |
| `GWF-214` | affected | unique | `calibration/calibration_data.{c…` | WMM unset is defined twice and the two comments… |
| `GWF-216` | affected | unique | `calibration/calibration_data.{c…` | File-level SI-units promise contradicts the uni… |
| `GWF-217` | affected | unique | `calibration/calibration_data.{c…` | Comment labels 20 °C as standard-atmosphere gro… |
| `GWF-221` | affected | unique | `calibration/calibration_manager…` | Doc-comment disagrees with the body: Core-1-saf… |
| `GWF-224` | affected | unique | `calibration/calibration_manager…` | Header claims a 9-parameter store; the body fit… |
| `GWF-225` | affected | unique | `calibration/calibration_manager…` | Documented FIT_FAILED-on-diverge contract is no… |
| `GWF-227` | affected | unique | `calibration/calibration_storage…` | Comment overstates sector setup and asserts a b… |
| `GWF-229` | affected | unique | `calibration/calibration_storage…` | Comment requires page alignment of the RAM buff… |
| `GWF-232` | affected | unique | `calibration/cal_hooks.{cpp,h}` | Banner disagrees with the body. There is no pau… |
| `GWF-236` | affected | unique | `flight_director/flight_director…` | Only state_landed and state_abort handle SIG_RE… |
| `GWF-237` | affected | agree | `flight_director/flight_director…` | Public field is written once to false in flight… |
| `GWF-239` | affected | unique | `flight_director/flight_director…` | ABORT entry fires drogue only when profile->abo… |
| `GWF-240` | affected | unique | `flight_director/flight_director…` | state_idle ENTRY calls the callback on every no… |
| `GWF-242` | affected | unique | `flight_director/flight_director…` | state_coast SIG_TICK coast-timeout (466-477) fi… |
| `GWF-243` | affected | unique | `flight_director/command_handler…` | Header and .cpp comments state an ARMED/BOOST/C… |
| `GWF-245` | affected | unique | `flight_director/action_executor…` | Public action type is documented as logging a p… |
| `GWF-251` | affected | unique | `flight_director/guard_evaluator…` | The tick contract says all active guards are ev… |
| `GWF-253` | affected | unique | `flight_director/guard_evaluator…` | The field comment describes a count comparison;… |
| `GWF-254` | affected | unique | `flight_director/guard_evaluator…` | The file-level sustain rule is not true on the … |
| `GWF-256` | affected | unique | `flight_director/guard_evaluator…` | The threshold-from-profile comment is false for… |
| `GWF-257` | affected | unique | `flight_director/guard_evaluator…` | The field comment reads as a general edge-detec… |
| `GWF-259` | affected | unique | `flight_director/guard_combinato…` | Public CombinatorType documents a third mode th… |
| `GWF-260` | affected | unique | `flight_director/guard_combinato…` | The evaluate docstring's universal 'blocks laye… |
| `GWF-261` | affected | unique | `flight_director/guard_combinato…` | Load-bearing fail-closed confidence gate is omi… |
| `GWF-263` | affected | unique | `flight_director/guard_functions…` | File-level comment assigns a FusedState/ESKF re… |
| `GWF-264` | affected | unique | `flight_director/guard_functions…` | Comment describes a near-zero NED crossing (and… |
| `GWF-265` | affected | unique | `flight_director/guard_functions…` | Stale comment says guard_baro_stationary is uni… |
| `GWF-266` | affected | unique | `flight_director/guard_functions…` | The same block claims body-Z, /accel_z/ via seq… |
| `GWF-268` | affected | unique | `flight_director/guard_functions…` | The name claims a baro peak; the implementation… |
| `GWF-270` | affected | agree | `flight_director/flight_state.h` | File banner is stale versus the enum and the la… |
| `GWF-271` | affected | agree | `flight_director/flight_actions.h` | Public symbol named kEmptyActions is not empty … |
| `GWF-272` | affected | agree | `flight_director/flight_actions.h` | File banner overstates the surface: this is a c… |
| `GWF-274` | affected | unique | `flight_director/mission_profile…` | The struct-level unit contract lists ms as the … |
| `GWF-277` | affected | unique | `flight_director/mission_profile…` | Name and comment describe different mechanisms … |
| `GWF-293` | affected | unique | `logging/flash_flush.{cpp,h}` | Sector pick is (active_sequence even ? B : A), … |
| `GWF-296` | affected | agree | `logging/flight_table.{cpp,h}` | File-level layout comment is stale versus the h… |
| `GWF-297` | affected | unique | `logging/flight_table.{cpp,h}` | Header describes dual-sector flash persistence … |
| `GWF-299` | affected | unique | `logging/flight_table.{cpp,h}` | Per-entry size comment understates the nested m… |
| `GWF-301` | affected | unique | `logging/flight_table.{cpp,h}` | “Reset to empty” is not the same as init. erase… |
| `GWF-302` | affected | unique | `logging/log_decimator.{cpp,h}` | The header field-policy comment disagrees with … |
| `GWF-303` | affected | unique | `logging/data_convert.{cpp,h}` | Comment and TelemetryState field name claim bar… |
| `GWF-312` | affected | unique | `logging/psram_init.{cpp,h}` | The two ID-read comments disagree with each oth… |
| `GWF-313` | affected | unique | `logging/radio_config_storage.{c…` | Doc comment disagrees with the sector/entry lay… |
| `GWF-314` | affected | unique | `logging/radio_config_storage.{c…` | Read contract still says whitelist membership; … |
| `GWF-315` | affected | unique | `logging/radio_config_storage.{c…` | Public init contract overstates what the functi… |
| `GWF-317` | affected | unique | `logging/radio_config_storage.{c…` | API comments describe a flash hit and a ~100 ms… |
| `GWF-318` | affected | unique | `logging/crc16_ccitt.h` | The file brief presents the module as C++20 con… |
| `GWF-322` | affected | unique | `diag/diag_stats.{cpp,h}` | Comment names a CMSIS intrinsic and a header th… |
| `GWF-323` | affected | unique | `diag/diag_stats.{cpp,h}` | Comment attributes UINT32_MAX seeding to the fi… |
| `GWF-324` | affected | unique | `notify/notify_backend_audio.cpp` | The file header promises engine-visible tone da… |
| `GWF-325` | affected | unique | `notify/notify_backend_led.cpp` | Staging comments still describe the backend as … |
| `GWF-327` | affected | unique | `notify/notify_backend_led.cpp` | 0 is treated as a category-empty sentinel, but … |
| `GWF-403` | affected | unique | `core1/sensor_core1.{cpp,h}` | Who may write g_bestGpsFix is contradictory on … |
| `GWF-406` | affected | unique | `core1/sensor_core1.{cpp,h}` | Comment cites a static_assert that is not in th… |
| `GWF-407` | affected | unique | `core1/sensor_core1.{cpp,h}` | Load-bearing comment is physically false withou… |
| `GWF-410` | affected | unique | `active_objects/ao_flight_direct…` | Header safety contract for SET_RADIO_CONFIG is … |
| `GWF-411` | affected | unique | `active_objects/ao_flight_direct…` | The callback inventory comment drops a wired ho… |
| `GWF-421` | affected | agree | `active_objects/ao_logger.{cpp,h}` | Header and body comments describe a 200→50/25 H… |
| `GWF-422` | affected | unique | `active_objects/ao_logger.{cpp,h}` | Comment asserts an FD timestamp path the handle… |
| `GWF-423` | affected | unique | `active_objects/ao_logger.{cpp,h}` | Documented start precondition is stale relative… |
| `GWF-424` | affected | unique | `active_objects/ao_logger.{cpp,h}` | File-level A6/ownership promise is contradicted… |
| `GWF-426` | affected | agree | `active_objects/ao_logger.{cpp,h}` | Subscription comment claims an event path the s… |
| `GWF-451` | affected | unique | `active_objects/ao_notify.{cpp,h}` | Boot-init comment names the wrong function, dro… |
| `GWF-452` | affected | unique | `active_objects/ao_notify.{cpp,h}` | Comments claim this AO runs priority resolution… |
| `GWF-453` | affected | unique | `active_objects/ao_notify.{cpp,h}` | Phase-change comment invents a tick re-stamp pa… |
| `GWF-456` | affected | agree | `active_objects/ao_led_engine.{c…` | Public file banner still describes the pre-IVP-… |
| `GWF-457` | affected | agree | `active_objects/ao_led_engine.{c…` | File-level comment asserts per-tick GPS/ESKF ev… |
| `GWF-465` | affected | unique | `main.cpp` | The function is only reached from main() via in… |
| `GWF-467` | affected | unique | `main.cpp` | The start-call order is not highest-first: Radi… |
| `GWF-468` | affected | unique | `main.cpp` | The brief disagrees with this file and the Core… |

**HW leakage vs domain code** (6) — Sitting 4 closed most. Remaining = new agent P or DEFER riders.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-X2-07` | affected | unique | `include/rocketchip/config.h` | The I2C device-address map is declared in five … |
| `GWF-025` | untouched | unique | `include/rocketchip/board_feathe…` | The DVM-unavailable reason comment disagrees wi… |
| `GWF-034` | affected | unique | `include/rocketchip/board_pico2.h` | File banner and PICO2_BRINGUP_OK error treat th… |
| `GWF-040` | affected | unique | `include/rocketchip/board_tiny_2…` | This header itself never tests TINY_2350_BRINGU… |
| `GWF-041` | untouched | unique | `include/rocketchip/board_tiny_2…` | The docblock advertises 8 MB flash and says eve… |
| `GWF-042` | untouched | unique | `include/rocketchip/board_tiny_2…` | Enabling PSRAM here does not remap pins. The in… |

**Safety / ops SSOT (Go/No-Go, pyro, guards)** (74) — Live invariants, not comment style.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B03-01` | affected | unique | `include/rocketchip/job_capabili…` | kRoleHasFullGoNogo is documented as a capabilit… |
| `CW-B21-03` | affected | unique | `flight_director/action_executor…` | FIRE_PYRO is documented as a log-only action; t… |
| `CW-B21-05` | affected | unique | `flight_director/command_handler…` | Declared kArm contract omits the test-mode bloc… |
| `CW-B22-04` | affected | agree | `flight_director/guard_evaluator…` | Phase bitmask is one bit too narrow for FlightP… |
| `CW-B23-04` | affected | agree | `flight_director/guard_combinato…` | Phase bitmask cannot represent FlightPhase::kFa… |
| `CW-B31-01` | affected | unique | `safety/fault_protection.{cpp,h}` | Q_onError logs from the context rc_log.h forbid… |
| `CW-B31-02` | affected | unique | `safety/fault_protection.{cpp,h}` | fault_reset_with_visible_signal emits no visibl… |
| `CW-B31-03` | affected | unique | `safety/fault_protection.{cpp,h}` | The one-shot flag rationale contradicts the cra… |
| `CW-B31-05` | affected | unique | `safety/fault_protection.{cpp,h}` | The guard / capture / dispatch sequence is writ… |
| `CW-B31-06` | affected | unique | `safety/fault_protection.{cpp,h}` | The stack guard is programmed with no precondit… |
| `CW-B31-08` | affected | agree | `safety/anomalous_boot.{cpp,h}` | The documented 2-of-N corroborator path cannot … |
| `CW-B32-02` | affected | unique | `safety/health_monitor.{cpp,h}` | Every health decision is computed from an unche… |
| `CW-B32-03` | affected | unique | `safety/health_monitor.{cpp,h}` | kHealthWatchdogOk reports ESKF state, not any w… |
| `CW-B32-04` | affected | unique | `safety/health_monitor.{cpp,h}` | The critical byte auto-triggers a power-cycle-o… |
| `CW-B32-05` | affected | unique | `safety/flight_in_progress.cpp` | flight_in_progress_was_set() is a predicate nam… |
| `CW-B32-06` | affected | unique | `safety/health_monitor.{cpp,h}` | The "usable GPS fix" rule is written out three … |
| `CW-B33-01` | affected | unique | `safety/station_fault_inject.{cp…` | station GPS "restore" hook re-injects the fault… |
| `CW-B33-03` | affected | agree | `safety/test_mode.{cpp,h}` | magic-observed-at-boot is documented as survivi… |
| `CW-B33-04` | affected | unique | `safety/fault_inject.{cpp,h}` | force_hardfault preamble claims a flight-phase … |
| `CW-B33-05` | affected | unique | `safety/station_fault_inject.{cp…` | station header states a gating invariant its ow… |
| `CW-B33-06` | affected | unique | `safety/test_mode.{cpp,h}` | host-build comment describes a boot-window coun… |
| `CW-B34-01` | affected | unique | `safety/core1_i2c_pause.{cpp,h}` | Header claims every runtime flash_safe_execute … |
| `CW-B34-05` | affected | agree | `safety/pyro_edge_logger.{cpp,h}` | Banner calls the storage a "ring buffer"; it is… |
| `CW-B35-05` | affected | unique | `safety/pio_watchdog.{cpp,h}` | PIO allocation comment contradicts the project'… |
| `CW-B35-06` | affected | unique | `safety/pio_watchdog.{cpp,h}` | Watchdog fault flag is self-clearing on the nex… |
| `CW-B37-03` | affected | unique | `active_objects/ao_flight_direct…` | Header does not state that the two public dispa… |
| `CW-B43-04` | affected | unique | `cli/rc_os_commands.{cpp,h}` | Dropped seqlock_read results print a zero-fille… |
| `CW-L004` | affected | agree | `flight_director/guard_evaluator…` | D. The phase bitmask is one bit too narrow for … |
| `CW-L015` | affected | unique | `safety/fault_protection.{cpp,h}` | The project's assertion facility is implemented… |
| `CW-L016` | affected | unique | `safety/fault_protection.{cpp,h}` | `mpu_setup_stack_guard()` never verifies the gu… |
| `CW-L017` | affected | unique | `safety/health_monitor.{cpp,h}` | Health classification runs on an unverified sen… |
| `GWF-244` | affected | unique | `flight_director/command_handler…` | Public kArm contract (phase + Go/No-Go poll/pri… |
| `GWF-248` | affected | unique | `flight_director/go_nogo_checks.…` | Print-format contract disagrees with the body o… |
| `GWF-249` | affected | unique | `flight_director/go_nogo_checks.…` | Station comment states a GO predicate and a yel… |
| `GWF-250` | affected | unique | `flight_director/go_nogo_checks.…` | GoNoGoCheck.reason contract does not match the … |
| `GWF-262` | affected | agree | `flight_director/guard_combinato…` | uint8_t phase bitmask / phase_bit cannot encode… |
| `GWF-344` | affected | unique | `safety/fault_protection.{cpp,h}` | Header still advertises no-access after the R-3… |
| `GWF-345` | affected | unique | `safety/fault_protection.{cpp,h}` | Load-bearing no-stack contract is asserted in b… |
| `GWF-346` | affected | unique | `safety/fault_protection.{cpp,h}` | Comments and helper names claim a live serial/L… |
| `GWF-347` | affected | unique | `safety/fault_protection.{cpp,h}` | Public header and implementing file disagree on… |
| `GWF-348` | affected | agree | `safety/fault_protection.{cpp,h}` | Reset/SRAM wipe story contradicts the included … |
| `GWF-349` | affected | agree | `safety/anomalous_boot.{cpp,h}` | Header design comment states a veto + 2-of-N fa… |
| `GWF-350` | affected | agree | `safety/anomalous_boot.{cpp,h}` | Enum comments overstate what each verdict means… |
| `GWF-351` | affected | unique | `safety/anomalous_boot.{cpp,h}` | Public init contract and BootSignals field comm… |
| `GWF-352` | affected | unique | `safety/anomalous_boot.{cpp,h}` | Call-site comment disagrees with read_prior_upt… |
| `GWF-353` | affected | unique | `safety/flight_in_progress.cpp` | Comment disagrees with linkage: 'read/write dir… |
| `GWF-354` | affected | unique | `safety/health_monitor.{cpp,h}` | Banner and apply_fault_latch claim IDLE auto-re… |
| `GWF-355` | affected | unique | `safety/health_monitor.{cpp,h}` | Header LANDED persistence contract disagrees wi… |
| `GWF-356` | affected | unique | `safety/health_monitor.{cpp,h}` | Comment describes a GPS fault/persistence path … |
| `GWF-357` | affected | unique | `safety/health_monitor.{cpp,h}` | Published tick-changed contract omits the criti… |
| `GWF-358` | affected | unique | `safety/health_monitor.{cpp,h}` | go_nogo_ready does not implement the Tier-1 set… |
| `GWF-359` | affected | unique | `safety/health_monitor.{cpp,h}` | Visibility-only HealthCritical contract is wire… |
| `GWF-360` | affected | unique | `safety/health_monitor.{cpp,h}` | Comment attributes the 105 °C safe-mode check t… |
| `GWF-362` | affected | unique | `safety/crash_record.{cpp,h}` | g_crash_record is non-volatile/non-atomic with … |
| `GWF-364` | affected | unique | `safety/fault_inject.{cpp,h}` | The exported C prototype and name promise a per… |
| `GWF-365` | affected | unique | `safety/fault_inject.{cpp,h}` | ao_priority is part of the public unmangled ABI… |
| `GWF-366` | affected | unique | `safety/fault_inject.{cpp,h}` | The exported name says HardFault. The comment a… |
| `GWF-367` | affected | unique | `safety/fault_inject.{cpp,h}` | That one-line comment covers both definitions. … |
| `GWF-369` | affected | unique | `safety/station_fault_inject.{cp…` | File banner (mirrored at station_fault_inject.c… |
| `GWF-370` | affected | unique | `safety/station_fault_inject.{cp…` | Header contract comment describes restore as en… |
| `GWF-371` | affected | unique | `safety/station_fault_inject.{cp…` | The restore block comment disagrees with the bo… |
| `GWF-372` | affected | unique | `safety/station_fault_inject.{cp…` | The honest operation is clear-valid, not restor… |
| `GWF-373` | affected | unique | `safety/station_fault_inject.{cp…` | Thin header promises a restore operation and do… |
| `GWF-374` | affected | unique | `safety/station_fault_inject.{cp…` | Stated hook path does not match the write targe… |
| `GWF-375` | affected | agree | `safety/test_mode.{cpp,h}` | The public accessor contract (and the stated AO… |
| `GWF-377` | affected | agree | `safety/test_mode.{cpp,h}` | evaluate() never reads g_test_mode_arm_magic. I… |
| `GWF-378` | affected | unique | `safety/test_mode.{cpp,h}` | No timer is started. The window is to_ms_since_… |
| `GWF-379` | affected | unique | `safety/test_mode.{cpp,h}` | test_mode_boot_ms() is return 0U with no counte… |
| `GWF-380` | affected | unique | `safety/test_mode.{cpp,h}` | The object is a public extern volatile bool, so… |
| `GWF-382` | affected | agree | `safety/test_mode.{cpp,h}` | This leaf’s own comments put writes to g_test_m… |
| `GWF-393` | affected | unique | `safety/pio_watchdog.{cpp,h}` | The header presents the call as a read of PIO I… |
| `GWF-394` | affected | unique | `safety/pio_watchdog.{cpp,h}` | Fault-flag lifetime is unspecified. deinit is a… |
| `GWF-395` | affected | unique | `safety/pio_watchdog.{cpp,h}` | The host stub cannot fulfill the header fault c… |
| `GWF-397` | affected | unique | `safety/pyro_edge_logger.{cpp,h}` | Thin header does not say who may write the log … |

**Test / inject / debug in the flight tree** (3) — Owner sitting closed. Remaining = new agent P.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `GWF-413` | affected | agree | `active_objects/ao_health_monito…` | Header and AO_HealthMonitor_start() do not ment… |
| `GWF-493` | affected | unique | `cli/rc_os_debug.{cpp,h}` | Header still inventories a 'replay trigger' as … |
| `GWF-494` | affected | unique | `cli/rc_os_debug.{cpp,h}` | Documented bool contract is handled-or-not; imp… |

**File earn-rent / naming / packaging** (8) — Owner sitting closed KEEP/fold. Remaining = new agent P.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B05-11` | affected | agree | `include/rocketchip/telemetry_en…` | FlightMetadata's stated 14-byte size is 16, and… |
| `CW-B21-01` | affected | agree | `flight_director/action_executor…` | LedPhaseValue is a second copy of the LED patte… |
| `GWF-079` | affected | agree | `include/rocketchip/telemetry_st…` | Documented 14-byte FlightMetadata layout is not… |
| `GWF-087` | affected | agree | `include/rocketchip/led_patterns…` | Two headers claim authority over the same uint8… |
| `GWF-176` | affected | unique | `fusion/eskf.{cpp,h}` | Header predict/predict_dense comments still des… |
| `GWF-220` | affected | unique | `calibration/calibration_data.{c…` | Doc comment filename does not match the source … |
| `GWF-273` | affected | agree | `flight_director/flight_actions.h` | Header color inventory paraphrases the LED enum… |
| `GWF-326` | affected | unique | `notify/notify_backend_led.cpp` | Header and resolver comments disagree with each… |

**Version / identity / config.h grab-bag** (9) — config.h gone. R-9 version bump still owner-scheduled.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B01-06` | affected | unique | `include/rocketchip/config.h` | Tier / feature-flag block is a configuration ma… |
| `CW-B02-03` | affected | agree | `include/rocketchip/board_tiny_2…` | Tiny 2350 map assigns GPIO 21 to both I2C SCL a… |
| `CW-B28-03` | affected | unique | `logging/radio_config_storage.{c…` | Wear-avoidance skip compares a struct that cont… |
| `CW-L046` | affected | unique | `include/rocketchip/config.h` | `dbg_print` / `dbg_error` forward an unconstrai… |
| `GWF-017` | affected | unique | `include/rocketchip/config.h` | The published assert contract does not name own… |
| `GWF-019` | affected | unique | `include/rocketchip/config.h` | kLedRed promises a red LED the board layer does… |
| `GWF-104` | affected | agree | `include/rocketchip/version.h` | The SSOT header itself has two firmware-version… |
| `GWF-276` | affected | unique | `flight_director/mission_profile…` | The documented contract is a flight-configurati… |
| `GWF-316` | affected | unique | `logging/radio_config_storage.{c…` | Boot-override comment names a default that this… |

**Fusion / math / cal live invariants** (34) — Numerical contracts, not comment style.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B07-05` | affected | unique | `math/mat.h` | The 3x3 block helpers take runtime offsets with… |
| `CW-B14-04` | affected | agree | `fusion/eskf_runner.{cpp,h}` | eskf_run_predict() aborts by returning from a v… |
| `CW-B15-02` | affected | unique | `fusion/confidence_gate.{cpp,h}` | confidence_gate_evaluate has two unstated, unch… |
| `CW-B16-07` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | no assertions in the file, and the two precondi… |
| `CW-B19-06` | affected | unique | `calibration/lm_solver.{cpp,h}` | the header half of the module is outside every … |
| `CW-L006` | affected | unique | `fusion/eskf.{cpp,h}` | F1 — `ESKF::predict()` states no `dt` contract … |
| `CW-L007` | affected | unique | `fusion/ud_factor.{cpp,h}` | F2 — Bierman's `h_idx` indexes `U[24][24]` with… |
| `CW-L008` | affected | unique | `fusion/confidence_gate.{cpp,h}` | F3 — `confidence_gate_evaluate()` — the pyro-lo… |
| `CW-L009` | affected | unique | `fusion/eskf.{cpp,h}` | F4 — On-pad ZUPT guard checks parameters that p… |
| `CW-L010` | affected | unique | `fusion/eskf_runner.{cpp,h}` | F5 — `g_profile` non-null contract is asserted … |
| `CW-L012` | affected | unique | `math/quat.{cpp,h}` | Unit-norm precondition of `Quat` is load-bearin… |
| `CW-L013` | affected | unique | `math/mat.h` | `Mat::operator()` is the declared element acces… |
| `CW-L014` | affected | unique | `math/quat.{cpp,h}` | 9-element buffer contract on `to_rotation_matri… |
| `CW-L042` | affected | unique | `fusion/eskf.{cpp,h}` | `reset_covariance_attitude()` writes a dense `P… |
| `GWF-106` | affected | unique | `math/vec3.{cpp,h}` | Load-bearing near-zero policy for normalized() … |
| `GWF-112` | affected | unique | `math/quat.{cpp,h}` | Header inverse() states the algebraic formula o… |
| `GWF-173` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Confidence-gate P diagonals are taken from dens… |
| `GWF-174` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Fusion runner is an undeclared writer of the ca… |
| `GWF-175` | affected | unique | `fusion/eskf_runner.{cpp,h}` | Header allows a nullable profile pointer and do… |
| `GWF-178` | affected | unique | `fusion/eskf.{cpp,h}` | Public return/counter/gate contract overstates … |
| `GWF-180` | affected | unique | `fusion/eskf.{cpp,h}` | Header/cpp 3-axis contract describes a rotated … |
| `GWF-181` | affected | unique | `fusion/eskf.{cpp,h}` | P-growth API does not name or implement the aut… |
| `GWF-192` | affected | unique | `fusion/confidence_gate.{cpp,h}` | The type promised as published output also carr… |
| `GWF-193` | affected | unique | `fusion/confidence_gate.{cpp,h}` | A published safety-adjacent bool is part of the… |
| `GWF-197` | affected | unique | `fusion/innovation_monitor.{cpp,…` | Header push contract does not mention silent re… |
| `GWF-203` | affected | unique | `fusion/mahony_ahrs.{cpp,h}` | Init mag contract understates the 1 µT floor ac… |
| `GWF-207` | affected | unique | `fusion/ud_factor.{cpp,h}` | Failure contract for the inout UD24 is unspecif… |
| `GWF-209` | affected | unique | `fusion/ud_factor.{cpp,h}` | Thin-header contract is incomplete: no valid ra… |
| `GWF-215` | affected | unique | `calibration/calibration_data.{c…` | Two parallel status channels with no ownership … |
| `GWF-218` | affected | unique | `calibration/calibration_data.{c…` | Version field implies layout compatibility that… |
| `GWF-223` | affected | agree | `calibration/calibration_manager…` | Progress contract does not cover the 6-pos (or … |
| `GWF-228` | affected | unique | `calibration/calibration_storage…` | Public success contract is 'valid data was read… |
| `GWF-231` | affected | unique | `calibration/lm_solver.{cpp,h}` | bestParams and *bestFitness are implicit in-out… |
| `GWF-233` | affected | unique | `calibration/cal_hooks.{cpp,h}` | g_imu is a shared I2C handle (shared_state: ini… |

**P10-9 function pointers** (5) — Owner GPS/watchdog remediates closed; QP-tied still DEFER.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B18-02` | affected | unique | `calibration/cal_hooks.{cpp,h}` | cal_read_accel is dead code, still registered i… |
| `CW-B19-01` | affected | unique | `calibration/lm_solver.{cpp,h}` | "no pointer-to-function in scope" is not what t… |
| `CW-B42-04` | affected | unique | `cli/rc_os.{cpp,h}` | rc_os_read_accel is an exported callback contra… |
| `GWF-235` | affected | unique | `calibration/cal_hooks.{cpp,h}` | Header does not state units, frame, or raw-vs-c… |
| `GWF-294` | affected | unique | `logging/flash_flush.{cpp,h}` | The brief and the used-only paragraph disagree.… |

**One-off / leftover** (44) — Does not share a later sitting with a pile above.

| ID | vs owner | vs other | Leaf | Title |
|----|----------|----------|------|-------|
| `CW-B14-02` | affected | agree | `fusion/eskf_runner.{cpp,h}` | g_mag3dEnabled latch survives filter re-init, p… |
| `CW-B18-05` | affected | unique | `calibration/cal_hooks.{cpp,h}` | Three of the five mag-staleness statics are wri… |
| `CW-B19-02` | affected | unique | `calibration/lm_solver.{cpp,h}` | template arguments carry no constraint, only pr… |
| `CW-B20-06` | affected | unique | `flight_director/flight_director…` | Constructor's non-null profile precondition is … |
| `CW-B21-04` | affected | unique | `flight_director/action_executor…` | Both public entry points dereference their poin… |
| `CW-B22-03` | affected | unique | `flight_director/guard_evaluator…` | guard_evaluator_init leaves GuardState::sustain… |
| `CW-B23-05` | affected | unique | `flight_director/guard_combinato…` | init_combinator clamps the copy but not the cou… |
| `CW-B29-03` | affected | unique | `diag/diag_stats.{cpp,h}` | AO queue snapshot reads QP getters without the … |
| `CW-B38-04` | affected | agree | `active_objects/ao_logger.{cpp,h}` | the two-point baro differentiator cache has two… |
| `CW-B40-07` | affected | unique | `active_objects/ao_led_engine.{c…` | The LED dedup cache assumes AO_LedEngine is the… |
| `CW-B40-08` | affected | agree | `active_objects/ao_led_engine.{c…` | The Core-1 vitality fallback is skipped on exac… |
| `CW-L001` | affected | unique | `flight_director/flight_director…` | A. The Flight Director's whole configuration co… |
| `CW-L002` | affected | agree | `flight_director/flight_actions.h` | B. Phase- and signal-indexed tables are bounds-… |
| `CW-L003` | affected | unique | `flight_director/guard_combinato…` | C. `init_combinator`'s bound clamps the copy bu… |
| `CW-L043` | affected | unique | `calibration/lm_solver.{cpp,h}` | `lm_solve` / `lm_accumulate_jtj` state their ca… |
| `CW-L044` | affected | unique | `math/mat.h` | `block3` / `set_block3` / `add_block3` are neve… |
| `CW-L045` | affected | unique | `math/mat.h` | `scalar_update` and `cholesky` carry `N`-genera… |
| `CW-L047` | affected | unique | `include/rocketchip/ao_signals.h` | `evt_cast`'s `static_assert` pins standard-layo… |
| `GWF-012` | affected | unique | `include/rocketchip/rc_log.h` | Unspecified whether 256 or n wins, whether the … |
| `GWF-027` | untouched | agree | `include/rocketchip/board_feathe…` | Two polarity promises (flag vs setter body) wit… |
| `GWF-029` | affected | unique | `include/rocketchip/board_fruit_…` | The file declares a cross-device SPI1 share tha… |
| `GWF-038` | affected | agree | `include/rocketchip/board_tiny_2…` | Both roles are bound to GPIO 21 with no comment… |
| `GWF-039` | affected | unique | `include/rocketchip/board_tiny_2…` | The polarity flag and the setter are published … |
| `GWF-048` | untouched | unique | `include/rocketchip/job_relay.h` | The published object is only a default-off MAVL… |
| `GWF-085` | affected | unique | `include/rocketchip/ao_signals.h` | LedPatternEvt (line 132) documents the same pay… |
| `GWF-098` | affected | agree | `include/rocketchip/flash_layout…` | Header promises a boot-time no-overlap-with-fir… |
| `GWF-103` | affected | unique | `include/rocketchip/version.h` | Role identity is dual-encoded. The comment asse… |
| `GWF-129` | affected | unique | `drivers/gps_pa1010d.{cpp,h}` | Init/ready/last-raw ownership is one-shot in th… |
| `GWF-159` | affected | unique | `drivers/mcu_temp.{cpp,h}` | The declared success condition is not observed,… |
| `GWF-165` | affected | unique | `drivers/ws2812_status.{cpp,h}` | Public num_leds contract is unbounded; the only… |
| `GWF-246` | affected | unique | `flight_director/action_executor…` | Thin-header ActionContext promises two FlightPh… |
| `GWF-252` | affected | unique | `flight_director/guard_evaluator…` | After init, sustained is uninitialized. is_sust… |
| `GWF-255` | affected | agree | `flight_director/guard_evaluator…` | The documented phase-bit encoding cannot repres… |
| `GWF-267` | affected | unique | `flight_director/guard_functions…` | Header promises a direct FusedState/ao_logger r… |
| `GWF-295` | affected | unique | `logging/flash_flush.{cpp,h}` | If ring_read_sequential fails with frame_idx < … |
| `GWF-298` | affected | unique | `logging/flight_table.{cpp,h}` | Contract comment promises flash load/save on ta… |
| `GWF-300` | affected | unique | `logging/flight_table.{cpp,h}` | Allocation ownership is inverted and undocument… |
| `GWF-304` | affected | unique | `logging/data_convert.{cpp,h}` | Header contract for the test-only reverse attri… |
| `GWF-320` | affected | unique | `diag/diag_stats.{cpp,h}` | Contract surface says dump is safe from any pha… |
| `GWF-408` | affected | unique | `core1/sensor_core1.{cpp,h}` | Comments assign the LED to Core 0 while this le… |
| `GWF-427` | affected | agree | `active_objects/ao_logger.{cpp,h}` | Two public entry points for the same flight eve… |
| `GWF-455` | affected | unique | `active_objects/ao_notify.{cpp,h}` | The same file states overlapping static posts a… |
| `GWF-458` | affected | agree | `active_objects/ao_led_engine.{c…` | Header name and comment promise a Fault-layer w… |
| `GWF-464` | affected | agree | `main.cpp` | Core 1 is launched on every role (283-284), but… |

---

## Out of this document

- Actual REMEDIATE / ACCEPT / DEFER labels on GWF/CW rows (next sitting).
- Hardware gates for any code that later lands.
- Editing the frozen finding packs.
- Mixing owner WNs back into this queue.

When the owner opens the first agent-bucket sitting: work remaining
rows in that bucket, in groups of 2–4, on the disposition worktree.
Do not start those `src/` edits on `main`.

