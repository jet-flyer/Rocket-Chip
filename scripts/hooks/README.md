# Tracked Git hooks (RocketChip)

## One-time setup (per clone)

Point Git at this directory instead of `.git/hooks/`:

```bash
git config core.hooksPath scripts/hooks
```

Git 2.9+ required. Hooks are versioned here; no copy step.

## Contents

| File | Purpose |
|------|---------|
| `pre-commit` | Clang-tidy size/CC on staged `src/**/*.cpp` (excludes CLI/dev per policy), **`ctest`** when `build_host/` exists, then **Tier 6b** triggers from **`scripts/ci/pre_commit_matrix.py`** (`eval` safe `TRIGGER_*=0|1`). |
| `post-commit` | Graphify AST rebuild (background) + project **curate** (semantic-cache re-align). Snapshot **verify is not automatic** — run `python scripts/graphify_verify.py` at milestones. |
| `graphify_pretool_bash.py` / `graphify_pretool_read.py` | Project PreToolUse (`.claude/settings.json`; Grok loads Claude hooks). **Root cause of exit-code-1 UI noise:** stock Graphify one-liners printed Claude `hookSpecificOutput` JSON; Grok expects `{"decision":"allow"\|"deny"}` and records malformed/non-decision output as fail-open failure. Scripts emit Grok `decision:allow` under `GROK_*` env; Claude still gets `additionalContext` for search/read nudges. Mandate on Grok also lives in `.grok/rules/graphify.md`. |

Clang-tidy and toolchain paths assume the Windows CMake default layout from `README`/Pico VS Code extension; edit the hook if your toolchain lives elsewhere.

**Bypass:** `git commit --no-verify` — repo-owner approval only this session.

## Legacy

Older clones may still have `.git/hooks/pre-commit`; remove or symlink after adopting `core.hooksPath` to avoid duplicate runs.
