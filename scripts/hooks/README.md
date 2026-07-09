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
| `graphify_pretool_bash.py` / `graphify_pretool_read.py` | Wired from **project** `.claude/settings.json` PreToolUse (Grok also loads Claude settings). Advisory “use graphify first” only; **always exit 0** (Grok shows non-zero as UI fail even when fail-open). |

Clang-tidy and toolchain paths assume the Windows CMake default layout from `README`/Pico VS Code extension; edit the hook if your toolchain lives elsewhere.

**Bypass:** `git commit --no-verify` — repo-owner approval only this session.

## Legacy

Older clones may still have `.git/hooks/pre-commit`; remove or symlink after adopting `core.hooksPath` to avoid duplicate runs.
