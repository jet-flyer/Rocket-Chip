# Agent Behavioral Guidelines

**PRIORITY:** Consider these behavioral rules first. If any conflict arises with project-specific instructions, ask for clarification before proceeding.

Behavioral guidelines to reduce common LLM coding mistakes. Merge with project-specific instructions as needed.

**Tradeoff:** These guidelines bias toward caution over speed. For trivial tasks, use judgment.

## 1. Think Before Coding

**Don't assume. Don't hide confusion. Surface tradeoffs.**

Before implementing:
- State your assumptions explicitly. If uncertain, ask.
- If multiple interpretations exist, present them - don't pick silently.
- If a simpler approach exists, say so. Push back when warranted.
- If something is unclear, stop. Name what's confusing. Ask.

## 2. Simplicity First

**Minimum code that solves the problem. Nothing speculative.**

- No features beyond what was asked.
- No abstractions for single-use code.
- No "flexibility" or "configurability" that wasn't requested.
- No error handling for impossible scenarios.
- If you write 200 lines and it could be 50, rewrite it.

Ask yourself: "Would a senior engineer say this is overcomplicated?" If yes, simplify.

## 3. Surgical Changes

**Touch only what you must. Clean up only your own mess.**

When editing existing code:
- Don't "improve" adjacent code, comments, or formatting.
- Don't refactor things that aren't broken.
- Match existing style, even if you'd do it differently.
- If you notice unrelated dead code, mention it - don't delete it.

When your changes create orphans:
- Remove imports/variables/functions that YOUR changes made unused.
- Don't remove pre-existing dead code unless asked.

The test: Every changed line should trace directly to the user's request.

## 4. Goal-Driven Execution

**Define success criteria. Loop until verified.**

Transform tasks into verifiable goals:
- "Add validation" → "Write tests for invalid inputs, then make them pass"
- "Fix the bug" → "Write a test that reproduces it, then make it pass"
- "Refactor X" → "Ensure tests pass before and after"

For multi-step tasks, state a brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong success criteria let you loop independently. Weak criteria ("make it work") require constant clarification.

## 5. Document Rewrites — Read Before You Clobber

**Never replace large parts of a document (or empty a board/handoff) without reading what you are removing.**

Whenever you would rewrite a file, replace a large section, empty a whiteboard, or compress process state into a short summary:

1. **Read first** — the whole document, or every paragraph/row you are about to overwrite or delete.
2. **Inventory** — list live items, open decisions, and anything with a disposition target still open.
3. **Do not ignore** — each live item must stay, move to a **correct named home** (with owner OK if ambiguous), or be explicitly erased with a stated reason.
4. **Ask before bulk park** — do not relocate unfinished work to another whiteboard or a summary table unless the owner directed that move.
5. **Close gates are content-complete, not file-short** — “must be empty” means the work is finished, not that the file was truncated.

Prefer **edit in place** (erase only resolved rows) over full-file rewrite. If you cannot disposition an item, **leave it and report** — do not invent an empty board.

Origin: L2-P5 walk-WB premature empty (2026-08-09) — open W-rows bulk-parked without owner disposition.

---

**These guidelines are working if:** fewer unnecessary changes in diffs, fewer rewrites due to overcomplication, clarifying questions come before implementation rather than after mistakes, and process docs are not silently clobbered.

---

*Source: [forrestchang/andrej-karpathy-skills](https://github.com/forrestchang/andrej-karpathy-skills) (sections 1–4); section 5 is project-specific agent discipline.*
