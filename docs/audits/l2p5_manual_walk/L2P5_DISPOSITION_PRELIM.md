# L2-P5 disposition — preliminary planning

**Not the disposition.** No fix / accept / defer / merge of IDs here. This is
the skeleton the real plan will sit on.

Walks are closed as observations. Findings stay frozen:

| Chunk | Pack | File |
|-------|------|------|
| 1 | Owner | `L2P5_WALK_FINDINGS.md` (WN-001–327) |
| 2 | Grok | `L2P5_GROK_WALK_FINDINGS.md` (GWF-001–498) |
| 3 | Claude | `L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md` (one vote; skip lane duplicates) |

Trust overlay (not a queue): `L2P5_THREE_WALK_COMPARE.md`. Owner buckets for
chunk 1: `L2P5_WN_CLUSTERS.md`.

---

## Order of work

Three separate chunks, in this order:

1. **Owner WNs first.** These are the notes that will actually be dispositioned
   as `WN-*`. Use the 16 clusters as the sitting structure, not 327 isolated
   rows. Agent packs are not mixed into this chunk.
2. **Then Grok.** Second pass over GWF rows. Do not mint new `WN`s by default.
   Ask: already covered by a disposed WN? unique-and-looks-real? skip as
   comment-nit?
3. **Then Claude.** Same questions against the aligned pack. Canonical Claude
   answer is the batch row (Reconciliation §1: UART-staleness branch is live
   on the vehicle). Do not count refute-pass or lanes as a second walk.

Do **not** merge 327+498+358 into one list. Do **not** use walk-compare
triples as a disposition rank — they only say “this note is real.”

Owner `nothing of note` means that sitting did not find anything, not “there
is nothing there.” Agent-only hits wait for chunks 2–3.

---

## Standing note: one direct agent contradiction

Across Grok and Claude, **one** proposition is a true is/isn’t split. Owner
did not speak to it. It is **not** a 2-vs-1. Record it so chunk 2 does not
land a fix from Grok alone.

**P:** Is the unfenced QMI direct-mode window in `psram_configure_qmi`
(`src/logging/psram_init.cpp` ~180–220) a live IRQ hazard?

| Pack | Answer |
|------|--------|
| Grok **GWF-311** (high) | Yes. Same `qmi_hw` object is IRQ-fenced in `psram_detect` and unfenced in configure. |
| Claude **CW-B26-05** | **REFUTED.** Only caller is `psram_init` from `main` before USB, QF tick, and Core 1; no interrupt source is armed. |
| Owner | Silent. Nearby WNs on this leaf (**WN-215–217**) are comment density / board-coupling / test-permanence — not this P. |

**Rule for the real plan:** when chunk 2 reaches `GWF-311`, stop and settle P
(re-read boot IRQ state) before a code change. Do not treat Claude’s REFUTED
as automatically winning, and do not treat Grok’s high as automatically live.
Chunk 1 can ignore this row.

No other Grok-vs-Claude is/isn’t opposite was found. (`0`/`kOff` in the LED
resolver is the same facts, different “is this a finding?” call — not this
bucket.)

---

## Trust, for when a note comes up

Use while disposing, not as the order of sittings:

- **Triple (all three packs named the same issue):** strong evidence the
  issue is real. Still dispose in the chunk that owns that ID. Examples:
  dead `RC_ASSERT`, unknown board → Feather pins, phantom `version_string()`,
  ICM mag re-init stall, deprecated health aliases.
- **Two agents, owner silent:** extra walks working. Belongs in chunk 2
  and/or 3. Examples: dashboard Temp=0, PSRAM erase-only “hard gate,”
  `ring_init` clobbering recover.
- **Unique:** candidate, not a fight. Owner-only policy (SPDX, Doxygen,
  earn-rent, NOLINT, Go/No-Go SSOT) stays in chunk 1.

---

## Out of this document

- Actual dispositions (fix / accept / defer / deviation).
- Hardware gates for any code that later lands.
- W-5 include/consumer and W-2 concurrency 3-question (main whiteboard;
  not these semantic packs).

When planning proper starts: replace or extend this file; do not silently
edit the frozen finding packs.
