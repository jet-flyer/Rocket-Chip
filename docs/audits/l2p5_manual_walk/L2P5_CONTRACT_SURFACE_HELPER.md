# L2-P5 Helper — Thin files & contract surfaces

**What this is:** a **pedagogical + practical** companion for the files that look
empty, boring, or “not real code” — and are therefore easy to skip — but that
actually **name, own, or wire** important pieces of the system together.

**What this is not:** a replacement for the field manual
(`L2P5_MANUAL_WALK_GUIDE.md`). Criteria, standards IDs, and findings tables still
live there. This helper teaches you *how to see* these files and *what to fill in*
when you hit one on the itinerary.

**Owner:** Nathan (the walk). **Status vocabulary:** same as the field manual —
`PASS` / `FAIL` / `PARTIAL` / `NOT CHECKED` / `N/A`.

**Use with:** itinerary (route) → this helper when a file *looks thin* → field
manual lens sections when you need full criteria or to record a finding.

---

## 1. The trap this helper exists to break

When a file has almost no executable body — only names, types, `extern`s,
prototypes, enums, constants, or a wall of includes — the natural reaction is:

> “There’s nothing here. Tick PASS and move on.”

That reaction is **exactly wrong** for a specific class of files.

Those files are often the **map**, not the terrain:

| Terrain (implementation) | Map (contract surface) |
|---|---|
| Who actually writes the flag this tick | Who is *allowed* to write it |
| The seqlock read loop in Core 0 | The shared struct + “Core 1 writes / Core 0 reads” rule |
| Every `rc_log(...)` call site | The promise “bounded, may drop, not for ISR” |
| Flash erase/write code | Where each region lives and must not overlap |

If the map is wrong, every “correct” implementation that trusts it is wrong
together. Tools and tests are bad at catching that: the map still *compiles*.

### Why this matters for you (learning note)

In safety-critical review, a large share of high-consequence defects are not
“this line does the wrong math” but **“two parts of the system disagree about
the rules.”** Thin files are where those rules are supposed to be written down
in one place. Skipping them is like reviewing every room of a building and never
checking the fire-exit plan on the wall.

The field manual’s one-liner is right: **“No code” is not “nothing to walk.”**
This helper turns that slogan into steps you can actually run.

---

## 2. Plain vocabulary (use these words out loud)

You do **not** need expert C++ fluency for the core of this check. You need four
ideas:

| Word | Plain meaning | Ask yourself |
|---|---|---|
| **Contract** | The promise this file makes to every other file that uses it | “If someone only read *this* file, what rules would they think apply?” |
| **Claim** | What comments, names, or types *say* the rule is | “What does the file *assert* is true?” |
| **Truth** | What the rest of the codebase *actually* does | “Who really writes/reads this, and under what protection?” |
| **Hub** | A file whose main job is connecting many modules, not computing | “If this file disappeared, would half the tree lose its shared language?” |

For anything shared across cores, ISRs, or active objects, add three more (from
the field manual’s concurrency lens — highest consequence):

| Word | Plain meaning | Fail if you cannot answer |
|---|---|---|
| **Owner** | The one context allowed to **write** (Core 0, Core 1, one AO, boot-only, …) | Ambiguity itself is a finding |
| **Mutator(s)** | Who actually changes it (should match owner, or hand-off is explicit) | Dual writers with no hand-off story |
| **Barrier** | What makes cross-context use safe (`std::atomic`, seqlock, message/event, “set once at boot then read-only”, …) | Plain `bool`/`int` shared across cores with no story |

### Why this matters for you (learning note)

**Comment-truth** is the gap between claim and truth. A green build only proves
the *syntax* lines up. It does not prove Core 1 still only reads a flag the
comment says “Core 1 reads.” Your job on thin hub files is often: **inventory
the claims**, then **spot-check the truth** (or defer a named re-check — see §6).

---

## 3. How to recognize a “thin but vital” file

Open the file. If **most** of these are true, use this helper:

1. Little or no function *bodies* (or only tiny inline helpers).
2. Mostly: `extern`, prototypes, `struct`/`enum` fields, `constexpr`/`#define`,
   typedefs, signal catalogs, layout constants, or includes that select a role.
3. Other modules **include** it to learn names/rules, not to run heavy logic.
4. You feel the urge to skip because “there’s nothing to read.”

**Examples already on the itinerary (not exhaustive):**

| File | Why it’s a hub / contract surface |
|---|---|
| `include/rocketchip/shared_state.h` | Central cross-core + CLI globals — pure `extern`s; ownership map for the dual-core split |
| `include/rocketchip/sensor_seqlock.h` | Shared sensor packet + lock-free publish/subscribe rules |
| `include/rocketchip/sensor_snapshot.h` | Packed raw layout — size and field meaning are the contract |
| `include/rocketchip/rc_log.h` | Logging API promises (bounded, drop-on-full, not ISR, …) |
| `include/rocketchip/ao_signals.h` | System-wide event vocabulary every AO must agree on |
| `include/rocketchip/job*.h` | Compile-time role (vehicle/station/relay) — what features exist |
| `include/rocketchip/board*.h` | Pin/peripheral map per board — hardware contract |
| `include/rocketchip/flash_layout.h` | Where flash regions live — must not collide |
| `include/rocketchip/linker_symbols.h` | Stack-guard symbols the linker owns; we only reference them |
| `include/rocketchip/version.h` | Single source of version/identity strings |

If the file is a normal `.cpp` with real control flow, **stop using this helper**
and go back to the spine + class lenses in the field manual. This is a *mode*,
not a whole-tree method.

---

## 4. Taxonomy — pick a kind, then ask that kind’s questions

Do **not** force every thin file through “class design.” Pick the **primary
kind**, then apply the matching questions. (Secondary kinds can still apply —
e.g. seqlock is both *shared protocol* and *data layout*.)

### Kind A — Shared-object catalog (ownership map)

**Looks like:** many `extern` globals, especially `g_*`; atomics; “Core 0 / Core 1 / CLI” comments.

**Canonical:** `shared_state.h`.

**Primary lens:** field manual → **Concurrency & shared-data ownership**.

**Questions (for each object):**

1. Who **owns** (writes)?
2. Who may **read**?
3. What **barrier** (or “boot-set-once”) protects it?
4. Does the **comment claim** match (1)–(3), or is it silent / contradictory?
5. Is dual-write claimed or possible? (JPL-8 single-owner pressure)

**Practical utility:** catches dual writers, missing barriers, and ownership
docs that lie — defects that tests almost never hit.

---

### Kind B — Shared protocol / mechanism

**Looks like:** a small type plus rules for how two contexts exchange data
(seqlock, ring, pause handshake).

**Canonical:** `sensor_seqlock.h` (and related snapshot/layout).

**Primary lens:** concurrency + (if types) class/interface design lightly.

**Questions:**

1. Who is writer, who is reader?
2. What makes a read **safe** (sequence, atomic, barrier, dmb, …)?
3. Is that rule stated in the file (or pointed at a design doc)?
4. Do field units / validity flags match what consumers will assume?

**Practical utility:** this is the Core 1 → Core 0 sensor path; a wrong protocol
contract corrupts fusion silently.

---

### Kind C — API / behavioral contract (prototypes + prose)

**Looks like:** function declarations plus a long “CONTRACT”, “USAGE”,
“PROHIBITED” block; few or no bodies in this file.

**Canonical:** `rc_log.h`.

**Primary lens:** field manual → **Comments & documentation quality** (JSF 134 /
intent).

**Questions:**

1. What may callers assume (success always? may drop? max size?)?
2. What is **forbidden** (ISR, hot path, huge messages)?
3. Does the signature match the prose (`void` vs “returns error”)?
4. Are load-bearing limits (128 bytes, drop-on-full) stated once and clearly?

**Practical utility:** stops call sites from treating diagnostics like reliable
telemetry, and keeps the public surface honest for future edits.

---

### Kind D — Shared vocabulary (enums, signals, modes)

**Looks like:** enum catalogs, signal lists, mode tables — little logic.

**Canonical:** `ao_signals.h`, pieces of job/notify headers.

**Primary lens:** comments + “is this the single vocabulary?”

**Questions:**

1. Is there **one** place that defines these names?
2. Are gaps, sentinels, and reserved ranges explained?
3. Would adding a signal here without updating subscribers be a foot-gun?
   (If yes, is that risk documented?)

**Practical utility:** mismatched signal numbers and “magic” event IDs are
classic integration bugs; the catalog is the agreement.

---

### Kind E — Layout / identity / configuration map

**Looks like:** addresses, sizes, version strings, board pins, role selects.

**Canonical:** `flash_layout.h`, `version.h`, `board*.h`, `job.h`.

**Primary lens:** comments + consistency with the “single source of truth” claim.

**Questions:**

1. Does the file claim to be the **only** place this knowledge lives?
2. Are units and origins clear (flash top-down? pins vs net names?)?
3. Could two boards/roles silently disagree if someone duplicates a constant
   elsewhere? (Finding: duplication of the map, not just a magic number.)

**Practical utility:** wrong flash map or board pin is a brick / wrong hardware
path; cheap to review in the map file, expensive later.

---

### Kind F — Boundary / adopted symbols

**Looks like:** `extern` of linker or SDK symbols; NOLINT + deviation notes.

**Canonical:** `linker_symbols.h`.

**Primary lens:** comments + accepted-deviations story (TP-2 etc.).

**Questions:**

1. Do we **define** or only **reference** the symbol?
2. Is the reserved-name / vendoring rationale in **one** place?
3. Is the use case narrow (stack guard) and stated?

**Practical utility:** keeps “weird but justified” symbols from spreading.

---

## 5. Procedure — five steps (copy this into your head)

When the itinerary lands you on a thin file:

### Step 1 — Name the kind

Say out loud: “This is Kind A / B / C / …” (from §4). If two kinds apply, pick
a **primary** and note the secondary.

### Step 2 — Inventory the surface (claims)

Skim once and list the **things the file introduces**: each major `extern`,
API, struct, enum group, or layout constant group. You are not judging yet —
you are making a checklist of claims.

Use the worksheet in §7. One row per object or per coherent group (e.g. “all
PSRAM flags” can be one row if they share the same owner story).

### Step 3 — Apply the kind’s questions

Fill owner / mutator / barrier **or** contract prose checks **or** single-source
checks — whatever the kind requires. Prefer the field manual’s judging tables
when you need a formal verdict:

| Kind | Field-manual home for FAIL/PARTIAL rows |
|---|---|
| A, B (shared) | **Concurrency** findings table (columns: Shared object / Owner / Barrier / Mutated by) |
| C, D, E, F (mostly docs/map) | **Comments** findings table; class-design only if a real type invariant is in play |

### Step 4 — Verify claim vs truth (lightly)

You do **not** need to reverse-engineer the whole tree on every row.

**Verify-now** (cheap, do it on this file’s session):

- Comment **self-contradiction** inside the file (two comments disagree).
- Type already implies a barrier (`std::atomic`) or its absence (plain `bool`
  with a dual-core comment).
- Dual-write **claimed in the comment** (“Core 1 reads/writes”) — that is
  already a JPL-8 pressure finding or a demand for an explicit hand-off story;
  you can file from the claim alone.
- `static_assert` size / layout claims visible in-file.

**Verify-later** (name it; don’t pretend PASS on truth):

- “Who really writes `g_foo`?” when writers are still ahead on the itinerary.
- Implementation of a protocol (seqlock body) if only the header is here.

Write a one-liner on the itinerary, e.g.:

`PASS claims inventory; truth re-check when walking shared_state.cpp + core1`

or put a walk-whiteboard row if the re-check must not be lost
(`L2P5_WALK_WHITEBOARD.md`).

Optional tools (itinerary already documents these):

```text
graphify explain "g_sensorSeqlock"     # neighbors / who touches
graphify path "shared_state.h" "sensor_core1.cpp"
```

Repo search for a symbol name is also enough when graphify is not handy.

### Step 5 — Record and move on

- Itinerary: tick + one-line note (PASS included).
- Any `FAIL`/`PARTIAL`: field-manual findings table for the right lens.
- Disposition later per field manual (fix → `PROBLEM_REPORTS`, accept → deviations).

**Completeness still rules:** a thin file is not exempt from a recorded verdict.

---

## 6. What “good enough” looks like (so you don’t over-walk)

| Outcome | Meaning |
|---|---|
| **PASS** | For every inventory row, claims are clear **and** either truth matches or a **named** later re-check is unnecessary because the claim is self-contained (e.g. pure constants with single-source prose). |
| **PARTIAL** | Contract usable but incomplete (missing owner on a shared flag, vague dual-core comment, API prose missing a real limitation). Cheap fix: add/clarify the claim. |
| **FAIL** | Claim is wrong or dangerous (shared mutable across cores with no barrier; comment says single-owner but dual-write is stated; API promises reliability it cannot keep). |
| **NOT CHECKED (truth)** | Only for the *implementation* half you deferred — **not** for “I skipped the file.” Inventory + claim review must still happen. |

### Why this matters for you (learning note)

The practical value of this check is **not** rewriting every thin file into a
novel. It is forcing **one honest map**:

- If ownership is unclear in the hub, every caller invents its own story.
- If the API contract is incomplete, every caller invents limits.
- If the layout is duplicated, the map and the terrain drift.

Your verdict can be “claims incomplete” without you yet knowing the perfect
owner — incomplete is already actionable (`PARTIAL`).

---

## 7. Worksheet (copy per file)

Duplicate this block into notes, a scratch buffer, or a walk log. One file per
block.

```text
FILE: _______________________________________________
KIND (A–F, primary): _____   secondary: _____
ONE-SENTENCE JOB OF THIS FILE:
_____________________________________________________

| # | Name / group | Claim (from comment/type) | Owner | Mutator | Barrier / rule | Claim OK? | Truth checked? | Verdict |
|---|--------------|---------------------------|-------|---------|----------------|-----------|----------------|---------|
| 1 |              |                           |       |         |                | Y/N/?     | now / later / n/a |         |
| 2 |              |                           |       |         |                |           |                |         |
| 3 |              |                           |       |         |                |           |                |         |

DEFERRED TRUTH RE-CHECKS (symbol → when/where):
- 

ITINERARY ONE-LINER:
- 

FINDINGS TO LOG (lens + short text):
-
```

For Kind C/E/F you may replace Owner/Mutator/Barrier columns with:

`Promise` | `Forbidden` | `Single-source?` | `Signature matches prose?`

---

## 8. Worked mini-examples (real tree, teaching only)

These illustrate the **method**. They are **not** pre-judged walk verdicts —
your walk still owns dispositions. Numbers/lines may drift; re-read the file.

### Example 1 — Kind A: `shared_state.h` (fragment)

| Name | Claim in file | What you should notice |
|---|---|---|
| File banner | Core 0 owns init; Core 1 reads most sensor flags; CLI reads status | Overall ownership story — good that it exists |
| `g_imuInitialized` | “Core 1 reads” | Owner of **writes** not named on the line (banner implies Core 0 init) — check consistency |
| `g_baroInitialized` | “Core 1 reads/writes” | Dual-write **claim** → JPL-8 pressure; either FAIL/PARTIAL for dual ownership, or demand an explicit hand-off/barrier story |
| `g_imu` | “initialized on Core 0, used on Core 1” | Cross-core **device handle** — barrier/protocol must exist somewhere; header must not be silent if use is concurrent |
| `g_sensorSeqlock` | “Core 1 writer, Core 0 reader” | Clear owner/mutator story — good claim shape |
| `g_startSensorPhase` etc. | `std::atomic<bool>` | Barrier type visible; still want **who** sets/clears |
| `g_sensorPhaseActive` | plain `bool`, “Core 0 write, Core 0/Core 1 read” | Cross-core **read** of non-atomic — claim invites a barrier question even if writer is single |

**Learning takeaway:** you can fill half the concurrency findings table from
this header alone, before opening any `.cpp`.

### Example 2 — Kind C: `rc_log.h` (fragment)

| Claim | Why it matters |
|---|---|
| Bounded 128-byte buffer; truncate with marker | Callers must not assume full message delivery |
| Drop on ring full; never blocks | Not a reliability channel |
| Not for ISR / fault-handler | Wrong context = latent hang or re-entrancy risk |
| `void rc_log(...)` | Signature matches “caller cannot detect truncation” |

**Learning takeaway:** here the “code” is the **prose + signature**. Comment
lens is primary; concurrency is secondary (Core 0 drain) only if you care about
the sink story.

### Example 3 — Kind E: `flash_layout.h` / `version.h`

| File | What “walking” means |
|---|---|
| `flash_layout.h` | Regions, anchors, non-overlap story stated? Single source? |
| `version.h` | Identity strings centralized? Role/board defines match how firmware is built? |

**Learning takeaway:** no loops, still load-bearing. Ask “single map?” not
“does-one-thing function shape.”

### Example 4 — Kind F: `linker_symbols.h`

| Claim | Check |
|---|---|
| Linker-defined; we only reference | No definitions in project that re-create the symbol |
| Reserved `__` names justified; centralized | Matches accepted-deviation / TP-2 story |

**Learning takeaway:** thin + weird is often **boundary**. Review the
justification, not invent algorithms that aren’t there.

---

## 9. Spine adaptation (when there are no functions)

Field manual Step 2 says run the spine on each **function**. On a thin hub file,
run a **surface spine** instead:

1. **Name test:** one short phrase for the file’s job, no “and” if possible
   (“cross-core global ownership map”, “AO signal catalog”). If you need three
   “ands,” the hub may be overloaded (note it — design smell, not automatic FAIL).
2. **One altitude:** does the file stay at “names and rules,” or does it smuggle
   half an implementation that belongs in a `.cpp`?
3. **Duplication:** is this the only map, or a second copy of pins/signals/globals?
4. **Distrust confident comments** (spine B): long ownership comments that never
   name a barrier are a confidence signal to probe, not to rubber-stamp.

Then apply §4–§5. Do not skip the file because the function-spine has nothing
to chew on.

---

## 10. Where this sits in the walk pack

| Artifact | Role |
|---|---|
| `L2P5_WALK_ITINERARY.md` | Which file next; tick coverage |
| **This helper** | How to evaluate thin / hub / contract-surface files |
| `L2P5_MANUAL_WALK_GUIDE.md` | Full lenses, criteria, findings tables |
| `L2P5_WALK_PLAN.md` | Prep/gates/close-out (not day-to-day eyeball method) |
| `L2P5_WALK_WHITEBOARD.md` | Deferred truth re-checks / cross-file items with no home yet |
| `L2P5_RP_SOURCES_2026-06-25.md` | Source quotes if a criterion’s wording is unclear |

**Field-manual pointer (kept short on purpose):** the guide’s
“Declaration-only headers are contracts, not skips” note is the *alert*. This
file is the *how*. Prefer expanding **this** helper over growing that paragraph.

---

## 11. Quick card (print / pin)

```text
THIN FILE? → name KIND (A–F)
→ inventory claims (worksheet)
→ owner / mutator / barrier  OR  promise / forbidden / single-source
→ claim OK in-file?  truth now or named later?
→ itinerary note + findings table if not PASS
→ “looks empty” is never a reason to skip
```
