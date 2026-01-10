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
- All merged work is preserved in main branch history

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

## Pushing Changes

- Always use `git push -u origin <branch-name>` for first push
- Retry on network errors (up to 4 times with exponential backoff)
- Never force push to main/master without explicit approval
