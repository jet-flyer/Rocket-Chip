# Worktree & Project-Log Hygiene

**Purpose:** How to *work* when this sitting is not on `main` — every edit, every commit, every time a log entry is needed. Wrap/handoff only verify the standing rule was kept. They are not the moment the rule starts.

Lived failures: untracked work wiped by `git clean` (WB 2026-07-01); walk CHANGELOG entries committed on `grok/l2p5-agent-walk` and dropped at merge (LL Entry 45); wrap that pushed the feature branch and left sitting CLs off `main` (2026-08-21).

---

## Layout

| Role | Typical path | Branch | What happens here |
|---|---|---|---|
| **Primary tree** | `C:\Users\pow-w\Documents\Rocket-Chip` | `main` | Hot logs. Merges that land work. Push to `origin/main`. |
| **Linked worktree** | sibling folder, e.g. `...\Rocket-Chip-l2p5-disposition` | one feature branch | This sitting's work. May outlive the sitting. |

Git will not check the same branch out in two trees. That is why `main` stays in the primary tree.

**Branch is the floor. Worktree is extra isolation.** A per-session branch stops two agents fighting over `origin/main`. A worktree stops `git clean` / `reset --hard` / edits in one tree from touching the other. Use a branch even for docs. Add a worktree when another agent may also be in the repo, or when the workstream will outlive one sitting (the L2-P5 shape).

---

## Standing rules (all sitting long)

These apply the first time you touch a file, not at wrap.

### 1. Hot shared logs — edit on `main` only

One file, every sitting writes it, sequential or append-only. A worktree that also writes it is a second allocator. Merge then "take `main`" drops the branch copy (LL-45).

| File | Why it is hot |
|---|---|
| `CHANGELOG.md` | Sequential `YYYY-MM-DD-NNN`. Two sittings on the same day both want `…-001`. |
| `docs/agents/LESSONS_LEARNED.md` | Append-only journal. Same merge-drop class. |

**Never** stage, commit, or `git add -A` these paths in the worktree. Not "with the feature so history is atomic." Not "I'll copy it to main at wrap."

**When** is not this file's invention. `CHANGELOG.md` header: confirm with the user first. Typically wrap. "Clear up," "that's done," "get back to other work," or an agent-initiated commit/push does **not** mint a `YYYY-MM-DD-NNN`. A lessons-learned note may still be written on `main` when a lesson was actually earned (checklist "log as you go") — that is not a CHANGELOG ID.

The next CHANGELOG ID is **whatever `origin/main`'s `CHANGELOG.md` already shows, plus one**. Other agents do not need a reservation board, a claimed-ID whiteboard row, or changelog fragments. They read `main`, take the next NNN, commit to `main`. That is the coordination tool.

If two people write `CHANGELOG.md` on `main` at the same moment, Git conflicts. That is the correct conflict. Keep both bodies; give the later one the next free NNN.

### 2. Unique-path records — the worktree is fine

A *new* file with a unique path (this sitting's audit pack, a new decision doc, a new baseline directory) does not mint a shared ID. Commit it on the feature branch. Do not copy it onto `main` just to "sync records" while the work is still in flight.

### 3. Whiteboard — active work, from the tree you are in

`AGENT_WHITEBOARD.md` is not a log and not an ID allocator. Edit it where you are working. Two agents adding different rows: keep both.

### 4. Feature files — the worktree

Source, plans, tests, unique docs: commit on the feature branch, often. Durability is the commit, not the folder (untracked files lose to `git clean`).

### 5. Before every commit in the worktree

```powershell
git status --short
git diff --cached --stat
```

If `CHANGELOG.md` or `docs/agents/LESSONS_LEARNED.md` is staged, unstage it (`git restore --staged -- <path>`). Do not use `git add -A` / `git add .` in a worktree unless you have just checked those two paths are clean.

### 6. If a hot-log write *is* authorized

Only after the user confirmed a CHANGELOG (or started wrap), or an LL entry was actually earned. Still not in the worktree — on `main` in the primary tree:

```powershell
cd <primary>          # e.g. C:\Users\pow-w\Documents\Rocket-Chip
git fetch origin
git pull origin main
# CHANGELOG: next ID = last NNN on origin/main + 1 (only if authorized)
git add CHANGELOG.md   # and/or docs/agents/LESSONS_LEARNED.md
git commit -m "[agent] CHANGELOG YYYY-MM-DD-NNN: <one line>"
git push origin main
```

This file fixes **where**. **When** is `CHANGELOG.md`'s header + checklist item 8. Lived miss: `2026-08-21-002` was minted on `main` because a sub-task was committed and pushed without wrap or a user confirm.

Then, in the worktree, merge so your checkout of those files is a read-only mirror:

```powershell
git fetch origin
git merge origin/main
```

If that merge conflicts on a hot log, someone edited it on the branch. Stop. **Recovery**. Do not "take ours."

### 7. Keep `main` merged in

On a long-running worktree, merge `origin/main` into the feature when you start a sitting and after you (or anyone) pushes a log to `main`. You see other agents' entries. The hot files in the worktree stay a mirror you do not edit.

### 8. Pushing the feature is not the log

`git push -u origin HEAD` from the worktree is backup. Other agents still will not see a CHANGELOG entry until it is on `origin/main`.

---

## Why not a clash-reservation tool

| Approach | Verdict here |
|---|---|
| **Write hot logs only on `main`** (this file) | Existing format. One allocator. Other agents see the real next ID. |
| **Towncrier / Changesets fragments** | Usual multi-contributor fix. Changes format, adds a tool. Revisit only if several agents mint CLs every day and still collide on `main`. |
| **Whiteboard / lock file claiming the next NNN** | Another hot file. Claim/release gets forgotten. |
| **Per-agent ID suffix always** (`…-001G` / `…-001C`) | Splits agents, not two trees of the same agent. Still edits the same file from two branches. |

Do not install a reservation protocol unless the main-only rule is failing in practice.

---

## Create a worktree

From the primary tree, on `main`, clean index:

```powershell
git fetch origin
git pull origin main
git worktree add ..\Rocket-Chip-<short-name> -b <agent>/<task>
```

Existing branch: `git worktree add <path> <branch>`.

**Submodules do not come along.** Immediately in the new tree:

```powershell
git submodule update --init --recursive
```

Lived miss: Claude walk `2026-08-20-006` (no `pico-sdk/` / `lib/mavlink/`).

---

## Wrap and handoff (the net, not the rule)

The standing rules above already forbade committing hot logs on the branch. Wrap checks that, writes the sitting's CHANGELOG **on `main`** (unless skip), and leaves the feature unmerged. Handoff does the check and the whiteboard; it does not add a CHANGELOG unless asked.

Feature stays on the branch. Worktree stays. Incomplete code stays off `main`.

**Log-sync check** (worktree, after `git fetch origin`):

```powershell
git log origin/main..HEAD -- CHANGELOG.md docs/agents/LESSONS_LEARNED.md
```

Empty: the branch did not invent log commits. Non-empty: **Recovery**, then re-check. Wrap/handoff is not done.

Then: wrap entry on `main` if required; push `origin/main`; push the feature branch. Do **not** merge the feature as part of wrap or handoff.

---

## Land the feature (optional, not wrap)

When the *work* should ship, from the primary tree on `main`:

```powershell
git fetch origin
git pull origin main
git merge <feature-branch>    # FF is fine when main has not diverged
git push origin main
```

Hot logs were never committed on the branch, so this merge should not conflict on `CHANGELOG.md`. Unique-path files come along. Keep the worktree if the workstream continues.

If it *does* conflict on a hot log, the standing rule was broken. Keep both bodies; retitle colliding IDs; do not drop the branch block.

---

## Teardown (workstream done)

Only after the log-sync check is clean:

```powershell
git worktree remove <path>           # refuses if dirty; do not --force to discard work
git branch -d <feature-branch>
git push origin --delete <feature-branch>
```

Use `git worktree remove`, not `rm -rf`. If the folder was deleted by hand, `git worktree prune` from the primary tree. Git refuses `remove` while tracked files are dirty or untracked files exist — e.g. a leftover bench log.

Delete-after-merge is teardown of a *finished* workstream, not wrap.

---

## Recovery (a hot log was committed on the branch)

1. Read the branch-only entries (`git log origin/main..HEAD -- CHANGELOG.md` and the diff).
2. On `main`: add each missing entry with a **non-colliding** ID. Cite branch hashes if you retitle. Do not discard the body.
3. On the feature branch, make those paths match `main`:

   ```powershell
   git checkout origin/main -- CHANGELOG.md docs/agents/LESSONS_LEARNED.md
   git commit -m "[agent] Drop branch-local project log; canonical copy is on main"
   ```

4. Re-run the log-sync check — must be empty.

---

## Commands

```powershell
git worktree list
git fetch origin
git status --short
git diff --cached -- CHANGELOG.md docs/agents/LESSONS_LEARNED.md
git log origin/main..HEAD -- CHANGELOG.md docs/agents/LESSONS_LEARNED.md
git merge origin/main
git submodule update --init --recursive
git worktree remove <path>
git worktree prune
```

`git diff origin/main -- CHANGELOG.md` from a feature that has not merged `main` may show *main* is ahead. That is fine. Dangerous: commits on the branch that `main` does not have (`git log origin/main..HEAD --` those paths).

---

## Four events (do not collapse)

| Event | Meaning | Does **not** mean |
|---|---|---|
| **Push the feature branch** | Backup. | The project log is on `main`. |
| **Write a hot log on `main`** | Entry exists on `origin/main`. Only when authorized (wrap, or user confirmed). | The feature was merged. The worktree must be deleted. A sub-task finishing. |
| **Land the feature** | Merge because the work should ship. | Required at wrap. Required because the worktree stays alive. |
| **Teardown** | `worktree remove` + delete branch. | The only moment the log may reach `main`. |

---

## Checklist hooks

Landed on `SESSION_CHECKLIST.md` (items 7–8, wrap intro, 11, 12). Standing rules in this file remain the detail; the checklist is the reminder. **No Session-Start prompt.**

**During Session** — If this sitting is not on `main`: follow `docs/agents/WORKTREE.md` standing rules. Do not stage `CHANGELOG.md` / `LESSONS_LEARNED.md` in the worktree. Merge `origin/main` into the feature when you need the mirror updated.

**Item 7 (Push)** — If a CHANGELOG/LL write *was authorized*, it is committed on `main`, not in the same commit as feature work.

**Item 8** — *When:* confirm first; wrap unless skip; agent-initiated commit/push/"clear up" does not mint an ID. *Where:* on `main` only.

**Wrap / Handoff** — Log-sync check. Recovery before the scope is done. Do not merge the feature as part of wrap or handoff.

**Item 12** — Push the feature branch. If this sitting wrote a hot log, also push `main`.

---

## What this file is not

- Not a license to edit protected files.
- Not a requirement to merge or delete the worktree at wrap.
- Not GitHub-flow "delete the branch after every merge" while the workstream is still active.
- Not a second copy of `CHANGELOG.md` format rules.
