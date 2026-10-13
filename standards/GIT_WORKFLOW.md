# Git Workflow Standards

## Branch Management

### Branch Naming
- Feature branches: `claude/<description>-<session-id>`
- Main branch: `main` (or default branch)

### Branch Lifecycle

**IMPORTANT: Always clean up branches after merging PRs**

1. **Create branch** for new work
2. **Develop and commit** changes
3. **Create PR** when ready
4. **Merge PR** to main
5. **Delete branch immediately** after merge - both local and remote

### Cleanup After Merge

When a PR is merged, **immediately delete the associated branch**:

```bash
# Delete remote branch
git push origin --delete <branch-name>

# Delete local branch
git checkout main
git branch -D <branch-name>
```

### Why Clean Up Branches?

- Prevents accumulation of stale branches
- Keeps repository navigable
- Reduces confusion about active work
- Feature files that **survived the merge** are preserved in main branch history — this is **not** automatic for every path the branch touched

Delete-after-merge applies when the **workstream** is done. A long-running worktree (L2-P5 shape) may stay after a wrap or even after a checkpoint merge of some files. Do not treat wrap as teardown.

### Project log lives on `main` (LL Entry 45)

Standing procedure: `docs/agents/WORKTREE.md` — **during the sitting**, not only at wrap.

`CHANGELOG.md` and `docs/agents/LESSONS_LEARNED.md` are written on `main` in the primary tree. Do not stage them on a feature branch, not even "to copy at wrap." Wrap/handoff only check that this held. They do not merge the feature. A successful merge of findings/docs is not evidence the log landed.

Teardown (`git worktree remove` / branch delete) still runs the log-sync check in WORKTREE.md as a safety net — including Recovery if a sitting broke the rule.

### Checking for Stale Branches

Periodically audit branches:

```bash
# List all branches
git branch -a

# Check which remote branches are merged
git branch -r --merged origin/main | grep 'claude/'

# Clean up merged branches
git push origin --delete <branch-name>
```

## Commit Messages

- Use clear, descriptive messages
- Focus on "why" rather than "what"
- Format: Present tense, imperative mood
- Multi-line format for complex changes:
  ```
  Brief summary line (50 chars or less)

  Detailed explanation of changes, motivation, and context.
  Can span multiple lines.
  ```

## Pull Requests

- Title should clearly describe the change
- Include "Summary" section with bullet points
- Include "Test plan" section with verification steps
- Link to related issues if applicable

## Local history

- Commit at milestones. Push only at the end of the sitting.
- Before the push, fix unpushed commits in place:
  - the last commit: `git commit --amend`;
  - an older unpushed commit: `git commit --fixup=<sha>`, then
    `GIT_SEQUENCE_EDITOR=: git rebase -i --autosquash <remote>/<branch>`.
- You may amend, fix up or autosquash unpushed commits without asking the repo owner.
- Never amend, rebase or squash a commit that is on the remote. Never force-push.
- Never use `--no-verify`, on commit or on push. If a hook fails, fix the cause and run the same commit or push again.
- The bench runs once per push, in the pre-push hook (`standards/HW_GATE_DISCIPLINE.md` Rule 5). A bench failure is fixed in the unpushed commit, not in a new fix commit.
- The bench PASS notes (`refs/notes/rc-bench`) stay local. Push only from the bench PC; its worktrees share the notes. Do not push `refs/notes/*`. Notes are keyed on the firmware tree, the gate files, the compiler, picotool and pioasm versions, the Pico SDK and the CMake presets. An amend or autosquash that keeps all of these the same keeps the PASS. The gate reuses a PASS only when the same boards (USB serial) are attached. After a hardware change on the bench, push with `RC_BENCH_FRESH=1 git push`.

## Pushing Changes

- Always use `git push -u origin <branch-name>` for first push
- Retry on network errors (up to 4 times with exponential backoff)
- Never force-push.
