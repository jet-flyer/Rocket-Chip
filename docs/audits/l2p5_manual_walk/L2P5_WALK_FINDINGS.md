**Last edited:** 2026-08-01 · Grok · create walk findings log; WN-001/002; guidance compact

# L2-P5 Walk Findings

Owner-initiated observations from the manual file walk (may be drafted with an agent after discussion).
Same path strings as `L2P5_WALK_ITINERARY.md` (coverage checkboxes); add path sections as files are walked.
Tangents → `L2P5_WALK_WHITEBOARD.md`. Not PASS/FAIL, remediation, or disposition.

| Rule | Detail |
|------|--------|
| **IDs** | Global `WN-NNN`, fixed prefix, never renumber. Only noteworthy entries get IDs. `— nothing of note.` is unnumbered and does not advance the counter. **Next ID** = `max(WN-*)+1` in body (line below is convenience; body wins). Section IDs may look out of order after revisits — feature, not bug. |
| **Who** | `[Agent]` = who wrote the text (name only). Every entry is owner-initiated; agents do not open paths or invent `nothing of note` on their own. |
| **Form** | `**WN-NNN** — [Agent] · \`kind\` · **title**` then observation. Kind = context (`comment`, `invariant`, `ownership`, `cross-file`), not priority/fix-timing. Anchor with symbol and/or quote (lines optional/soft). |
| **Append-only** | Frozen once written; never edit prior text. Later find on same path → new global `WN`. |
| **Split vs merge** | One `WN` per claim that could disposition differently later. Same sitting ≠ merge. One `WN` only if bullets support a single claim. |
| **Content** | Observations only; no design close-outs. Stated claim on the declaring file; enforcement/evidence under the implementing path when walked (new `WN`, don't edit old). |
| **Itinerary sync** | Adding a later path while an earlier itinerary path is checked off but missing here → flag owner for `— nothing of note.` (do not invent it). |

**Next ID:** WN-003

---

## Tier 1 — Foundations

### include/rocketchip/

#### `include/rocketchip/shared_state.h`

**WN-001** — [Grok] · `comment` · **`g_imu` banner**  
Quote: `// IMU device handle (initialized on Core 0, used on Core 1)`. Reads as a closed
lifetime contract. Observed model is phase-based: Core 0 boot-fills under I2C ownership; after
`g_startSensorPhase` release, Core 1 does steady-state use and may re-`icm20948_init` on recovery.
Comment omits handoff, re-init, and post-handoff exclusivity.

**WN-002** — [Grok] · `invariant` · **`g_imu` shared handle**  
Handle is shared RAM across cores. Header does not state single-writer-after-handoff (or the
pause exception). Correctness of that invariant is not established from this file alone.
