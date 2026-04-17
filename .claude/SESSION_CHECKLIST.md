# Session Checklist

**Purpose:** Prevent incomplete handoffs, lost work, and undocumented changes between agent sessions.

---

## Session Start

1. Read all files referenced in `CLAUDE.md` (or equivalent agent instructions)
2. Check `PROJECT_STATUS.md` for current phase and blockers
3. Check `CHANGELOG.md` for recent changes
4. Check `AGENT_WHITEBOARD.md` for flags from previous sessions
5. If resuming interrupted work: read whiteboard notes and verify repo state matches expectations
6. **If this session will touch flight-critical paths** (`src/flight_director/`, `src/active_objects/ao_flight_director.cpp`, `src/active_objects/ao_logger.cpp`, `src/cli/rc_os.cpp`, or firmware `[FD]` log messages), run the bench sim canary before making any changes:

       python scripts/bench_sim.py

   If the canary fails on unchanged main, the tool has rotted — fix the tool first, not the firmware. See LESSONS_LEARNED.md Entry 36. The pre-commit hook runs the same test automatically when the probe is available and the staged diff touches these paths; the session-start canary catches rot introduced by previous sessions that committed via `--no-verify` when the probe was not available.

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
3. **Update `docs/PROJECT_STATUS.md`** — Current phase, what's done, what's next, any new blockers
4. **No orphaned work** — Every file change should be committed. No half-finished edits left in the working tree
5. **No unintended deletions** — Run `git diff --stat` and verify the diff matches what you intended to change. If files were deleted, confirm that was intentional
6. **Check AGENT_WHITEBOARD.md** — Update with current state, clear resolved items, add any new flags or open items discovered during the session
7. **Architecture doc drift check** — If this session changed any AO (added, removed, renamed, re-prioritized), signal (new, renamed, rerouted), or queue depth: run `grep -n "superloop\|core 0 cooperative" docs/SAD.md docs/SCAFFOLDING.md` and verify zero stale hits. Also verify the AO list in `docs/AO_ARCHITECTURE.md`, `docs/SAD.md` Section 5.1, and `docs/SCAFFOLDING.md` Module Responsibilities table all match `src/active_objects/`. Patch any drift before committing.
8. **Station/vehicle build parity check** — If this session modified CMakeLists.txt, any `src/active_objects/`, `src/cli/`, `src/telemetry/`, or `src/drivers/rfm95w.cpp`, rebuild BOTH tiers for BOTH roles to confirm no build is broken:
   ```
   cmake --build build/              # vehicle bench
   cmake --build build_flight/       # vehicle flight  (-DBUILD_FOR_FLIGHT=ON)
   cmake --build build_station/      # station bench   (-DROCKETCHIP_JOB_STATION=1)
   cmake --build build_station_flight/  # station flight (both flags)
   ```
   Station and vehicle share the same source tree gated by `ROCKETCHIP_JOB_STATION` / `kRadioModeRx` — a change that compiles on one role can silently break the other. If any build directory doesn't exist yet, create it with the appropriate `cmake -DROCKETCHIP_JOB_STATION=1 -DBUILD_FOR_FLIGHT=ON ..` flags (see `docs/BENCH_TEST_PROCEDURE.md`). When hardware-verifying, both the vehicle Feather and the Fruit Jam station should exercise the changed path before the session closes.
9. **Commit with descriptive message** — Format: `[agent] brief description of what and why`
10. **Push to remote** — `git push` so work is not stranded locally

---

## Session End (Handoff to Another Session)

All of the above, plus:

11. **Expand AGENT_WHITEBOARD.md** with handoff-specific details:
    - What was in progress
    - What's blocked and why
    - Any concerns or open questions
    - Specific files that were being worked on
12. **Don't leave broken code on main** — If work is incomplete, either stash it or commit to a feature branch
13. **Note the exact state** — "Build compiles, sensor reads work, CLI untested" is better than "made progress"

---

## Session End (Milestone Completion)

All of the normal completion items, plus:

14. **Update `docs/PROJECT_STATUS.md`** with milestone completion and next phase
15. **Review SCAFFOLDING.md** — If directory structure changed, update it
16. **Consider LESSONS_LEARNED.md** — If significant debugging occurred, document it

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
