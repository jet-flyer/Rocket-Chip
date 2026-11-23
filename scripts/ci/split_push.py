#!/usr/bin/env python3
"""EVALUATION ONLY, NOT YET PERMANENT (Nathan, 2026-11-23).

Print the push sequence that fits the gate's push-shape rules, so a push is
split the way the gate asks instead of tripping its shape warnings:
  * a pyro/deploy, guard, radio-adapter or turnaround commit ([isolate:*] in
    firmware_paths.txt) goes in a push with no other firmware commit;
  * at most MAX_FIRMWARE_COMMITS firmware commits per push.
Non-firmware commits (docs, etc.) ride along with the push they follow.

Read-only: it prints the git push commands and pushes nothing. Each printed
push still runs the real pre-push gate. Nathan is wary of the gate's script
count growing; keep or drop this after trying it on real pushes.

Usage (from the worktree of the branch to push):
    python scripts/ci/split_push.py [<remote>] [<branch>]
Defaults: remote origin, branch = the current branch's upstream branch name.
Exit codes: 0 = plan printed (or nothing to push), 2 = usage or git error.
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import List, Mapping, Sequence

sys.path.insert(0, str(Path(__file__).resolve().parent))
import firmware_tree as ft  # noqa: E402
import pre_commit_matrix as matrix  # noqa: E402
import pre_push_gate as pg  # noqa: E402


def split(commits: Sequence[str], firmware: Mapping[str, bool],
          isolate: Mapping[str, bool], max_fw: int) -> List[List[str]]:
    """Group commits (oldest first) into pushes. Pure, host-tested.
    A group closes before a firmware commit that would break a rule."""
    groups: List[List[str]] = []
    cur: List[str] = []
    fw_count = 0
    cur_isolated = False
    for c in commits:
        if firmware.get(c):
            iso = bool(isolate.get(c))
            if cur and fw_count and (iso or cur_isolated or fw_count >= max_fw):
                groups.append(cur)
                cur, fw_count, cur_isolated = [], 0, False
            fw_count += 1
            cur_isolated = cur_isolated or iso
        cur.append(c)
    if cur:
        groups.append(cur)
    return groups


def main(argv: List[str]) -> int:
    repo = Path(pg.git(Path.cwd(), 'rev-parse', '--show-toplevel'))
    remote = argv[1] if len(argv) > 1 else 'origin'
    if len(argv) > 2:
        branch = argv[2]
    else:
        up = pg.git(repo, 'rev-parse', '--symbolic-full-name', '@{u}', check=False)
        branch = pg.board_config.branch_of(up) if up else ''
        if not branch:
            print('split_push: no upstream; name the branch: '
                  'split_push.py <remote> <branch>')
            return 2
    local_sha = pg.git(repo, 'rev-parse', 'HEAD')
    remote_sha = pg.git(repo, 'rev-parse', f'{remote}/{branch}', check=False) or pg.ZERO
    sections = ft.load(repo, local_sha)
    commits = pg.commits_to_push(repo, remote, local_sha, remote_sha)
    if not commits:
        print('split_push: nothing to push.')
        return 0
    firmware, isolate = {}, {}
    for c in commits:
        fw = [p for p in pg.changed_paths(repo, c) if matrix.is_firmware(p, sections)]
        firmware[c] = bool(fw)
        isolate[c] = any(ft.classes(p, sections) for p in fw)
    groups = split(commits, firmware, isolate, pg.MAX_FIRMWARE_COMMITS)
    print(f'split_push (EVALUATION): {len(commits)} commit(s) to {remote}/{branch} '
          f'in {len(groups)} push(es). Run them in order; each runs the gate.')
    for i, g in enumerate(groups, 1):
        n_fw = sum(firmware[c] for c in g)
        tag = ', isolated' if any(isolate[c] for c in g) else ''
        print(f'\n  push {i}/{len(groups)}: {len(g)} commit(s), {n_fw} firmware{tag}')
        for c in g:
            print(f'    {pg.subject(repo, c)}')
        print(f'    git push {remote} {g[-1][:12]}:refs/heads/{branch}')
    return 0


if __name__ == '__main__':
    try:
        sys.exit(main(sys.argv))
    except pg.GateFail as exc:
        print(f'split_push: {exc}')
        sys.exit(2)
