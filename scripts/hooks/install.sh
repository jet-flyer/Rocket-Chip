#!/bin/bash
# RocketChip hook installer (tracked). Run once per clone, from Git Bash:
#     bash scripts/hooks/install.sh
# Worktrees share the clone's config, so one run covers them.
#
# Sets core.hooksPath so Git runs the TRACKED hooks in scripts/hooks/:
#   pre-commit   fast gates (stdio ban, clang-tidy, host ctest, flight
#                target cross-compile for both roles)
#   pre-push     hardware gate: bench the commit being pushed, once per push
#   post-commit  graphify rebuild
# Nothing here bypasses a hook. Never use --no-verify.
set -euo pipefail

root="$(git rev-parse --show-toplevel)"
cd "${root}"

for hook in pre-commit pre-push post-commit; do
    if [[ ! -f "scripts/hooks/${hook}" ]]; then
        echo "install: missing scripts/hooks/${hook}" >&2
        exit 1
    fi
done

git config core.hooksPath scripts/hooks

# The hooks call `python`, git worktree/notes, and `git rev-parse
# --path-format` (Git 2.31+). `git hook run` (dry run) needs Git 2.36+.
ver="$(git version | sed -E 's/^git version ([0-9]+)\.([0-9]+).*/\1 \2/')"
read -r major minor <<<"${ver}"
if (( major < 2 || (major == 2 && minor < 36) )); then
    echo "install: Git ${major}.${minor} is too old (need 2.36+)" >&2
    exit 1
fi
if ! python -c "import sys; sys.exit(0 if sys.version_info >= (3, 8) else 1)"; then
    echo "install: 'python' (3.8+) not on PATH" >&2
    exit 1
fi

# Self-check: the list file parses and the matrix runs.
python scripts/ci/pre_commit_matrix.py >/dev/null

# Bench key inputs (scripts/firmware_tree.py, rc-bench-key v2). A missing
# compiler or picotool does not stop the install, but the pre-push gate
# blocks a firmware push until the key can be made.
if ! python scripts/firmware_tree.py --key-inputs | sed 's/^/install: bench key: /'; then
    echo "install: WARNING - a bench key input is missing (tool or Pico SDK)."
    echo "         Set RC_ARM_GCC, RC_PICOTOOL, RC_PIOASM (full paths) or"
    echo "         RC_PICO_SDK (a git checkout), or install the pinned versions."
fi

if [[ -d .git/hooks ]] && ls .git/hooks | grep -v '\.sample$' >/dev/null 2>&1; then
    echo "install: note - .git/hooks has non-sample files; Git ignores them now"
    echo "         (core.hooksPath=scripts/hooks):"
    ls .git/hooks | grep -v '\.sample$' | sed 's/^/           /'
fi

echo "install: core.hooksPath = $(git config core.hooksPath)"
echo "install: pre-commit + pre-push + post-commit are active."
echo "install: dry-run a commit's gates with: git hook run pre-commit"
echo "install: plan a push (no build, no board): python scripts/ci/pre_push_gate.py --plan"
