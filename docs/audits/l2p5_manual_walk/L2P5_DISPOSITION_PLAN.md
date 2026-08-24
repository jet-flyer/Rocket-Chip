# L2-P5 disposition plan

Replace the prelim skeleton with a real plan. Finding packs stay frozen. Work lives on a dedicated worktree and is removed when this workstream is done.

**Progress (2026-08-23 owner-chunk land):** Phase 0–2 closed. Phase 3 owner do-now closed and on `main` (`2026-08-23-002`). Sitting 6 RF skipped (WN-100 DEFER). **Next: Phase 4 Grok `GWF-001–498`**, then Phase 5 Claude. Keep worktree `C:\Users\pow-w\Documents\Rocket-Chip-l2p5-disposition` · `grok/l2p5-disposition`. Do not start Phase 4 `src/` on `main`.

**Session unit:** the 16 WN buckets in `docs/audits/l2p5_manual_walk/L2P5_WN_CLUSTERS.md` (327 notes, generator `_gen_wn_clusters.py`). First disposition-prep step after walk-WB close. One theme per sitting: all comments together; other related notes in the same bucket even if they are not one bug. Do not split a bucket across weeks.

**Progress docs (whole disposition):** `AGENT_WHITEBOARD.md`, `L2P5_DISPOSITION_LOG.md` sitting rows, and this file’s **Progress** line update **when a bucket sitting closes**, not after each R-10 group. Code still commits in groups of 2–4 WNs. Do not restamp sitting progress mid-bucket. Unexpected blockers still go on the rem WB; that is not a progress restamp.

Appendix C (gate → foundations → behavior → cleanup) does **not** rank buckets. It was written for small mostly-automated audit PR lists. **Inside a bucket**, that order still applies when rows conflict.

This plan is owner/agent-chunk disposition, not Stage-17, Starcom execution, RC_OS rework, QP/C++, or L2-P10.

---

## Where the 16 buckets came from

After itinerary 121/121 and walk-WB drain (2026-08-17), PROJECT_STATUS / whiteboard said: WN cluster index, then disposition. `L2P5_WN_CLUSTERS.md` is that index — grouping overlay, not extractive truth, not yet labels. Owner reviews **buckets**, not 327 isolated rows. Every WN is landed before L2-P5 closes.

| n | Bucket | Code sitting in this plan? |
|--:|--------|----------------------------|
| 13 | In-source NOLINT / suppressions | Yes (gate honesty) |
| 2 | P10-9 function pointers (+ 18-site WB list) | Yes (gate honesty: empty deviation log is not evidence of absence). FD / `action_executor` callbacks may DEFER to the QP eval. GPS hooks and `kick_watchdog` do not wait on QP. `lm_solver` stays closed. |
| 19 | Doxygen + header comment-density policy | Yes (inventory → policy; not a mass delete) |
| 118 | Process archaeology in comments | Yes, **one sitting**, after Doxygen policy |
| 28 | HW leakage vs domain code | Yes (after HW-agnostic rule write) |
| 22 | Starcom / radio-telem supersession | **No — DEFER sitting only** |
| 5 | Generated files / codegen hygiene | Label; full start-from-scratch audit stays on the WB row unless a surgical REMEDIATE is obvious. No post-gen hand-edits; no silent regenerate of `mission_profile_data.h`. |
| 1 | SPDX / third-party license inventory | Yes |
| 1 | RF / regulatory config hazards (WN-100) | Yes — do **not** park with Starcom; freq/power/band can put a board out of legal operation |
| 10 | Safety / ops SSOT | Yes (optional ops inventory does not block WN-182/184). Do **not** smuggle agent-only two-agree bugs into this bucket. |
| 25 | Early-impl / design re-eval | **No — ACCEPT keep-with-why or DEFER to named rework.** Do not rewrite seqlock / I²C / ICM / PCM / quaternion in this pass. |
| 11 | RC_OS / CLI structure | **No — DEFER except proven-dead symbol deletes** |
| 10 | Test / inject / debug in the flight tree | Yes |
| 37 | File earn-rent / naming / packaging | Yes (after W-5) |
| 15 | Version / identity / config.h grab-bag | Yes |
| 10 | Fusion / math / cal live invariants | Yes |

---

## Already recorded (do not re-litigate)

- **Chunks:** owner WNs → Grok → Claude. Do not merge IDs. Do not use three-walk triples as a rank.
- Owner `nothing of note` ≠ “P is false.” Agent-only hits wait for chunks 2–3.
- Canonical Claude vote = aligned batch row; skip 22 lane duplicates; UART-staleness live on the vehicle.
- **QMI 1-vs-1:** `GWF-311` vs `CW-B26-05`. Chunk 1 ignores it. Chunk 2 stops and settles with `pico-sdk` present before any code. No fix from one agent.
- Each WN is a PR (open → analyzed → in progress → verified → closed). Default REMEDIATE. ACCEPT = signed deviation. DEFER without a safety-impact one-liner is invalid.
- Frozen packs stay frozen.

This session:

- Label **all 16 buckets**, then remediate **do-now buckets** as sittings.
- Owner first. Grok/Claude packs are not inputs to Phases 2–3; they wait for chunks 2–3.
- W-2 and W-5 before any owner bucket sitting.
- Starcom / RC_OS structure / early-impl rewrites cut from do-now.
- Dedicated worktree; LL-45 changelog copy before remove.
- Pre-remediation gates and LOC snapshot are after plan approval, before the first `src/` commit — not a reason to delay this paper plan.

**Fatigue fallback:** if labeling all 327 in one phase stalls, stop at a **bucket boundary**. Never a mixed-ID queue.

---

## Bucket-to-bucket edges (only these)

The 16 buckets are a classification overlay, not a dependency graph. Honor only:

1. Doxygen inventory → WN-054/081 policy → archaeology sitting
2. HW-agnostic rule write → HW leakage labels/code (`CODING_STANDARDS.md` and/or `SAD.md` — user names the file at that sitting)
3. W-5 include/consumer numbers → earn-rent sitting

Labeling “paper” still mutates protected standards on (2) and the Doxygen policy. Those are real edits; they are not comments.

---

## Phase 0 — After plan approval, before any WN sitting

First execution sitting. Not “starting disposition.”

**Worktree**

- Branch `grok/l2p5-disposition` in its own worktree.
- `git submodule update --init --recursive` — load-bearing; empty `pico-sdk` / `lib/mavlink` already produced false “unverifiable” walk rows.
- `cmake -B build_host` and `cmake -B build -G Ninja` so Gate 2/3 do not silent-skip.
- Stage explicit paths only. No `git add -A`.

**Tear-down** (workstream done): merge to `main`; **LL 45** — every branch `CHANGELOG.md` / `LESSONS_LEARNED.md` entry must exist on `main` before `worktree remove` / branch delete; delete local + remote branch.

**Walk-plan pre-remediation**

| # | Item | Status |
|---|------|--------|
| 0.1 | Worktree + submodules | After approval |
| 0.2 | Gates live (`compile_commands.json`, `build_host` CTest file) | After approval |
| 0.3 | Positive-control: a commit that actually prints clang-tidy + host ctest. Required before any `src/` commit (LL 43). Doc-only walk commits never exercised this. | After approval |
| 0.4 | Walk WB drained | **Done** 2026-08-17 |
| 0.5 | Deferred-venue routed | This plan |

**LOC / size baseline (before any disposition edits)**

`docs/baselines/l2p5_disposition_<date>/`. Improvement delta + break bisect, not a gate. Authored tree only — vendor LOC will drown the delta.

- LOC: `src/` + `include/rocketchip/` (exclude `EXTERNAL/`, `lib/`, `pico-sdk`, files banner-marked generated). Prefer an in-tree counter if one exists; else `cloc` with that exclude list. Record the exact command in the baseline `README`.
- Host: `ctest` pass count from `build_host`.
- Target (if the vehicle image builds): `.text` / `.data` via the same `size` / map method as `docs/baselines/build_audit_2026-04-23/`.
- Record HEAD SHA, date, commands. Re-run the same commands at close (`after` files).

**HW baseline (known-working board, before any disposition edits)**

It has been a while since the hardware actually ran. Capture a live positive-control snapshot of current `HEAD` on the bench **before** any `src/` edits. Store logs in the same baseline dir. This is not a 3-boot reseat of a firmware change; it is “does today’s image still boot and prove the buses.”

- Start Pico-SDK OpenOCD (`scripts/start_openocd_pico_sdk.ps1`); confirm `127.0.0.1:3333`.
- Flash the worktree vehicle image (current `HEAD`, no disposition diffs).
- Vehicle: `python scripts/bench_sim.py` — both tests PASS. Cite observed signals, not only the script name (Rule 1/3): at least `[FD] PYRO FIRED` on the happy path, plus whatever Hardware Status / banner tokens the run actually prints (DAC ACK, `RegVersion=0x12`, GPS `window_hit` if present).
- Three cold-restart repeats of that same positive-control (Rule 2): probe `monitor reset halt` + `resume`, or the bench_sim restart path, three times. All three must show the same signals.
- Station: `python scripts/station_bench_sim.py` if the Fruit Jam is on the bench and enumerates; if it is off/unplugged, record that and do not block Phase 0 on station.
- Save raw logs + a one-line `README` of the observed signals and COM ports.

If the board fails this snapshot, **stop**. Disposition does not start on a rotten or unpowered bench — fix or attribute the HW first (LL 36 / IVP-140 class).

---

## Phase 1 — W-5 then W-2 (before any owner bucket)

Read-only. Dated note under `docs/audits/l2p5_manual_walk/`. Then erase the WB rows.

- **W-5:** per contested header: direct includes, symbol consumers, indirect fan-in. Do not couple keep/fold across “same class” thin headers.
- **W-2:** owner / mutator / barrier for the listed 22 `volatile` + 9 `std::atomic`. Ambiguity is the finding.

---

## Phase 2 — Label all 16 buckets

`docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_LOG.md`: one row per WN, REMEDIATE / ACCEPT / DEFER, safety one-liner on DEFER.

Walk buckets in **cluster-doc table order** (conversation order only). Apply the three edges above. Starcom, RC_OS structure, early-impl: still label every WN (DEFER or ACCEPT keep-with-why); that is the sitting; there is no code queue.

P10-9 is labeled as **gate honesty**, not a comment nit.

Exit: 327/327 labeled, or a bucket-boundary pause under the fatigue fallback.

---

## Phase 3 — Owner remediations, one bucket sitting at a time

Only REMEDIATE rows. **Do not mix buckets.** Skip a bucket with zero REMEDIATE.

Do-now code sittings:

1. NOLINT
2. P10-9 (18 live sites; QP-tied subset may already be DEFER)
3. SPDX
4. HW-agnostic rule (if not already written) + HW leakage
5. Doxygen inventory/policy apply (not mass-delete of unmarked files)
6. RF / regulatory (WN-100)
7. Version / identity / config.h
8. Generated-file surgical REMEDIATEs only
9. Safety / ops SSOT
10. Fusion / math / cal
11. Test / inject / debug
12. Earn-rent / packaging
13. Process archaeology comments (all 118 in this sitting)

Inside a bucket, if rows conflict: gate-honesty → shared mechanism → behavior (flight-critical → support → ground) → comment/doc. Pattern-shared: one commit per file. A bug **revealed** by verifying PR-X rides with PR-X; coincident bugs split. Log/WB/PLAN progress line: sitting close only (see **Progress docs** above).

Verification: per-commit host ctest + the change’s own signal. **Flight-critical paths (FD, ESKF, pyro, fault, Go/No-Go): `bench_sim` with an observed positive-control.** “ctest was green” is not that signal. Do not run a six-hour audit suite per comment file.

**No code sitting:** Starcom (22), RC_OS structure (11, except dead-symbol deletes if Phase 2 marked them REMEDIATE), early-impl rewrites (25).

---

## Phase 4 — Grok chunk

`GWF-001–498`. No new WNs by default. Covered by a disposed WN → skip. Unique-and-real → label. Nit or Starcom/RC_OS/early-impl → skip/DEFER to the same home.

**QMI `GWF-311`:** stop; re-read boot IRQ with SDK; owner settles.

Remediate this chunk’s REMEDIATE set by the same bucket themes; do not open a fourth taxonomy.

---

## Phase 5 — Claude chunk

Aligned pack, one vote, skip lane duplicates. Same questions. Then remaining REMEDIATE.

---

## Phase 6 — Close

- Re-run Phase 0 LOC/size commands into the same baseline dir (`after` files).
- Plan-3 doc `docs/audits/AUDIT_COVERAGE_CATCHUP_YYYY-MM-DD.md` with NOT MECHANICALLY COVERED. L2-P5 in `PROBLEM_REPORTS.md` only when the owner names that file.
- L2-P10 is not this plan.
- Merge, LL-45, delete branch, remove worktree.
- Erase finished WB rows (W-2, W-5, P10-9 if decided, Doxygen order if inventory+policy landed, HW-agnostic if written). Leave deferred rework rows.

---

## Cut from do-now / routed out

| Pile | Home |
|------|------|
| Starcom / radio-telem (22) + agent polish on those surfaces | Starcom / CCSDS post–Stage-17 |
| RC_OS / CLI structure | WB RC_OS rework |
| Early-impl rewrites (seqlock, PCM, flash map, quaternion, I²C PIO, PIO beacon, ICM backend, …) | WB early-impl table |
| QP/C vs QP/C++ | WB medium (P10-9 FD subset waits) |
| 56 `-Wconversion`, §CM to-implement gates | §CM / re-audit |
| Graphify full re-pass | Owner-gated |
| L2-P10 CLA-RBM | After L2-P5 |
| CCSDS TC + COP-1, IVP-T13, station radio-health | Post–Stage-17 |

---

## Artifacts

| File | Role |
|------|------|
| `docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_PLAN.md` | This plan in-repo (prelim stays skeleton) |
| `docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_LOG.md` | Per-WN labels |
| Dated W-2/W-5 note | Prep |
| `docs/baselines/l2p5_disposition_<date>/` | Authored LOC / ctest / `.text` before and after |
| `docs/audits/AUDIT_COVERAGE_CATCHUP_YYYY-MM-DD.md` | Close-out |

Protected files only when named at that sitting.

---

## First sitting after approval

1. Worktree + submodules + `build` / `build_host`.
2. Positive-control the hook (must print clang-tidy + ctest).
3. LOC/size baseline at HEAD.
4. Land `L2P5_DISPOSITION_PLAN.md` (docs).
5. W-5 then W-2.
6. Stop until the owner opens Phase 2.
