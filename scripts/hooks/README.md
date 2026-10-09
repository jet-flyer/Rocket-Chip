# Tracked Git hooks (RocketChip)

## One-time setup (per clone)

Run the tracked installer from Git Bash:

```bash
bash scripts/hooks/install.sh
```

It sets `core.hooksPath = scripts/hooks` (Git then ignores `.git/hooks/`), checks Git 2.36+ and `python`, and runs a matrix self-check. Hooks are versioned here; no copy step. Worktrees share the clone's config.

## Contents

| File | Purpose |
|------|---------|
| `pre-commit` | Fast gates only: `<stdio.h>` ban, clang-tidy size/CC on staged `src/**/*.cpp` (excludes CLI/dev per policy), **`ctest`** when `build_host/` exists, and the **flight target cross-compile** for both roles (`build_flight`, `build_station_flight`) when a firmware path is staged (`TRIGGER_TARGET_BUILD` from **`scripts/ci/pre_commit_matrix.py`**), then the **list-completeness check** (`scripts/ci/check_elf_paths.py`: every repo file the two builds used must be in `[elf]`). No bench. |
| `pre-push` | **Hardware gate, once per push** (`scripts/ci/pre_push_gate.py`): benches the commit being pushed in the clean bench worktree (a firmware or gate change benches BOTH roles; a change where every firmware path is in `[station-only]` benches the station only), checks the build ID (firmware-tree hash), records each PASS as a git note (`refs/notes/rc-bench`, local only: do not push notes) on the bench key (firmware tree, gate files, compiler, picotool and pioasm versions, Pico SDK, CMake presets) with the USB serial of each benched board, reuses a note only when those boards are attached, warns on push shape, and enforces the radio-push rule once `scripts/radio_link_bench.py` exists. Does not flash: it prints the flash steps (Path 1, `picotool load -f` over USB, for both setups; then `--record-only`; then wait for LED + CDC) and the OpenOCD step with the main checkout folder (first line of `git worktree list`). Start OpenOCD from that folder, never from the bench worktree. Plan only: `python scripts/ci/pre_push_gate.py --plan`. Fresh mode (ignore all notes, after a hardware change on the bench): `RC_BENCH_FRESH=1 git push`, or `--plan --fresh`. UNTESTED ON HARDWARE (draft 2026-10-08). |
| `install.sh` | Tracked installer (above). |
| `post-commit` | Graphify AST rebuild (background) + project **curate** (semantic-cache re-align). Snapshot **verify is not automatic** — run `python scripts/graphify_verify.py` at milestones. |
| `graphify_pretool_bash.py` / `graphify_pretool_read.py` | Project PreToolUse (`.claude/settings.json`). **Two real bugs on Grok:** (1) Claude `hookSpecificOutput` is not Grok’s `{"decision":"allow"}` — malformed → UI fail-open exit 1; (2) `${CLAUDE_PROJECT_DIR}/...` often does not expand on Windows → Python “can’t open file” → exit 1/2. Command is workspace-relative `python scripts/hooks/...`. Grok → decision allow; Claude → optional additionalContext. Mandate also in `.grok/rules/graphify.md`. |

Clang-tidy and toolchain paths assume the Windows CMake default layout from `README`/Pico VS Code extension; edit the hook if your toolchain lives elsewhere.

**No bypass:** never use `--no-verify` (commit or push). If a hook fails, Git makes no commit or push. Fix the cause and run the same commit or push again. Fix an unpushed commit with `git commit --fixup=<sha>` + `git rebase -i --autosquash`.

**Paths:** the firmware path list is ONE tracked file, `scripts/ci/firmware_paths.txt`. The matrix, the identity gate and `cmake/rc_version.cmake` (`kFirmwareTreeId`) all read it.

## Legacy

Older clones may still have `.git/hooks/pre-commit`; remove or symlink after adopting `core.hooksPath` to avoid duplicate runs.
