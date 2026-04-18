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
17. **Full-tree clang-tidy sweep** — The pre-commit hook only gates staged files, so latent JSF AV Rule 1 violations (function size > 60 lines) can accumulate in files that aren't touched stage-by-stage. At milestone/stage close, run the same checks the hook runs against EVERY `src/**/*.cpp` not on the exemption list (exemptions: `src/cli/**`, `src/dev/**`, `eskf_codegen.cpp`). Zero warnings required for milestone closure — any new violations must be decomposed or logged as an accepted deviation in `standards/STANDARDS_DEVIATIONS.md` before the stage can close. Pattern (Git Bash, adjust toolchain path if needed):

        for f in $(git ls-files 'src/*.cpp' | grep -v 'src/cli/' | grep -v 'src/dev/' | grep -v eskf_codegen.cpp); do
          "C:/Program Files/LLVM/bin/clang-tidy.exe" "$f" -p build/ \
            --checks="-*,readability-function-size,readability-function-cognitive-complexity" \
            --extra-arg="--sysroot=C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi" \
            --extra-arg="--target=armv8m.main-none-eabi" \
            --extra-arg="-isystem" \
            --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1" \
            --extra-arg="-isystem" \
            --extra-arg="C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/arm-none-eabi/include/c++/14.2.1/arm-none-eabi/thumb/v8-m.main+fp/softfp" \
            --extra-arg="-Wno-format-security" \
            --extra-arg="-Wno-gnu-zero-variadic-macro-arguments" 2>&1 \
            | grep -E "warning:.*readability-function-(size|cognitive-complexity)"
        done

   Added 2026-04-18 after Stage L surfaced 5 latent violations (ao_logger, ao_telemetry, core1_sensor_loop, guard_evaluator_tick, flight_director_evaluate_guards) that had accumulated silently across earlier stages. See LL Entry 36 discipline — "gates that only check incremental change cannot catch pre-existing rot."

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
