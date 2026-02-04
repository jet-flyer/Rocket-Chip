# Session Checklist

**Purpose:** Prevent incomplete handoffs, lost work, and undocumented changes between agent sessions.

---

## Session Start

1. Read all files referenced in `CLAUDE.md` (or equivalent agent instructions)
2. Check `PROJECT_STATUS.md` for current phase and blockers
3. Check `CHANGELOG.md` for recent changes
4. Check `AGENT_WHITEBOARD.md` for flags from previous sessions
5. If resuming interrupted work: read whiteboard notes and verify repo state matches expectations

---

## During Session

- **No arbitrary numbers.** Any numerical value (sample rates, buffer sizes, thresholds, timeouts) must be justified by a source (datasheet, SDK docs, reference implementation, forum post confirming it works). If no source exists, flag it and ask before proceeding.
- **No assumptions on unspecified decisions.** If a task or design choice isn't covered by existing docs and internet research doesn't definitively answer it, ask before implementing. "Definitively answered" means confirmed working in a credible source, not a speculative forum post.
- **Research before implementing.** Before writing code that touches hardware interfaces or sensor drivers, check relevant documentation, datasheets, and especially recent forum posts for guidelines and known issues.
- **Log as you go.** Don't batch all documentation to the end. If you discover something unexpected, note it immediately.

---

## Session End (Normal Completion)

Before committing and pushing:

1. **Verify build compiles clean** — `cmake --build build/` with no errors
2. **Update CHANGELOG.md** — What changed, who (which agent), why
3. **Update PROJECT_STATUS.md** — Current phase, what's done, what's next, any new blockers
4. **No orphaned work** — Every file change should be committed. No half-finished edits left in the working tree
5. **No unintended deletions** — Run `git diff --stat` and verify the diff matches what you intended to change. If files were deleted, confirm that was intentional
6. **Commit with descriptive message** — Format: `[agent] brief description of what and why`

---

## Session End (Handoff to Another Session)

All of the above, plus:

7. **Update AGENT_WHITEBOARD.md** with:
   - What was in progress
   - What's blocked and why
   - Any concerns or open questions
   - Specific files that were being worked on
8. **Don't leave broken code on main** — If work is incomplete, either stash it or commit to a feature branch
9. **Note the exact state** — "Build compiles, sensor reads work, CLI untested" is better than "made progress"

---

## Session End (Milestone Completion)

All of the normal completion items, plus:

10. **Update PROJECT_STATUS.md** with milestone completion and next phase
11. **Review SCAFFOLDING.md** — If directory structure changed, update it
12. **Consider LESSONS_LEARNED.md** — If significant debugging occurred, document it

---

## Red Flags (Stop and Ask)

- You're about to delete files you didn't create
- A build that was working now fails after your changes
- You're changing more files than the task requires
- You're picking numerical values without a source
- You're "fixing" something adjacent to the actual task
- You notice a conflict between documentation and code — flag it, don't silently resolve it
- The task would require changes to a protected file

---

*This checklist is the minimum standard. Doing less risks losing work or introducing silent regressions.*
