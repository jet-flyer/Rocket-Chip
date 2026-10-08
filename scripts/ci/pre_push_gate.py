#!/usr/bin/env python3
"""Pre-push hardware gate: bench the commit being pushed, ONCE PER PUSH.

UNTESTED ON HARDWARE. Draft 2026-10-08 (council answer: bench once per
push; standards/HW_GATE_DISCIPLINE.md Rule 5). Called by
scripts/hooks/pre-push. Plan-only mode (no build, no board):

    python scripts/ci/pre_push_gate.py --plan [--fresh] [<remote>]

Fresh mode ignores every bench note and benches the push (a push with no
firmware or gate path still needs no bench). Use it after a hardware
change on the bench: radio board swap, jumper, solder or antenna change.
Git passes no options to a hook, so a real push takes an env var:

    RC_BENCH_FRESH=1 git push

`--fresh` and RC_BENCH_FRESH=1 do the same thing (also for --plan). The
release-tag check is not a bench, so fresh mode does not change it.

Per pushed ref (stdin: <local-ref> <local-sha> <remote-ref> <remote-sha>):

 1. Find the commits being pushed (not yet on the remote) and the paths
    they change. Classify with scripts/ci/firmware_paths.txt (the list as
    stored in the pushed commit).
 2. Shape rules (AGENT rules in SESSION_CHECKLIST 6b; the gate only WARNS):
    at most MAX_FIRMWARE_COMMITS firmware commits per push; a commit in an
    [isolate:*] class (pyro/deploy, guard, radio adapter, turnaround) goes
    in a push with no other firmware commit.
 3. Bench key (scripts/firmware_tree.py, format rc-bench-key v2) = blob
    id over the arm-none-eabi-gcc, picotool and pioasm version lines, the
    Pico SDK (path, describe --dirty, HEAD, submodule status), the role
    CMake presets, and [elf]+[gate] at the pushed sha. After each build
    the gate checks that the build used those same tools and that SDK. A git note
    on refs/notes/rc-bench is reused (skip build, flash and bench) only
    when ALL of these are true:
      - it says "result: PASS";
      - its roles: line covers the roles this push needs (+ radio-link);
      - its boards: line names a USB serial for each needed board, and
        each of those serials is attached now (USB enumeration only; no
        port is opened). A swapped MCU board gives no match;
      - fresh mode is off.
    The boards: line is a record. It is not a pre-flash board-ID check.
    The USB serial follows the MCU board, so it does NOT catch a radio
    board swap, a jumper, solder or antenna change: use fresh mode.
    Notes stay LOCAL: all pushes go from the bench PC, and its worktrees
    share refs/notes. The gate never pushes notes.
 4. Roles (scripts/ci/pre_commit_matrix.match): a firmware ([elf]) or
    [gate] change benches BOTH roles. When every firmware path is in
    [station-only], only the station bench runs. ROLES below is the one
    table that maps a role to its build dir, preset, bench script and
    flash path.
    Otherwise: check out the pushed sha in the bench worktree
    (<primary>-bench, detached, clean), build each needed role there
    unless its build already has the pushed firmware-tree hash, require a
    flash record for that exact ELF (the gate does NOT flash; it prints
    the flash steps: Path 1, picotool load -f over USB, for both setups;
    the OpenOCD step names the main worktree from git worktree list),
    then run bench_sim / station_bench_sim
    with --commit <sha>.
 5. Radio push ([radio] path changed). The radio rule is ACTIVE only
    when the pushed commit contains RADIO_LINK_BENCH. Then BOTH boards are
    benched and the two-board round trip must pass. Until that script
    lands, the gate only WARNS and benches the roles the matrix names
    (both roles, because a [radio] path is a firmware path). The
    script and the blocking rule land in the same commit (DRAFT.md).
 6. On PASS: write the git note and print the Rule 3 citation line.
 7. Release tags (refs/tags/vX.Y.Z exactly, the standards/VERSIONING.md
    "How to cut a release" form; starcom-v*, pre-*, archive/* and other
    tags are not release tags): the tagged commit must have a bench PASS
    note. Flight qualification (soak + flight
    checklist on that exact binary) is a separate gate (Rule 8).

Stdin cases (githooks pre-push): new branch (remote sha all zeroes:
commits = rev-list <local> --not --remotes=<remote>); delete (local sha
all zeroes: skipped); tag (annotated tag sha is the tag object: peeled
with <sha>^{commit}). A non-zero exit aborts the WHOLE push, every ref.

On any failure nothing is pushed. Fix unpushed commits with
`git commit --fixup=<sha>` + `git rebase -i --autosquash`. Never
--no-verify.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import os
import re
import subprocess
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Set, Tuple

_SCRIPTS = Path(__file__).resolve().parents[1]
for _p in (_SCRIPTS, _SCRIPTS / 'ci'):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))
import check_elf_paths as cep  # noqa: E402
import firmware_tree as ft  # noqa: E402
import pre_commit_matrix as matrix  # noqa: E402

ZERO = '0' * 40
NOTES_REF = 'refs/notes/rc-bench'
MAX_FIRMWARE_COMMITS = 5
# standards/VERSIONING.md: annotated vMAJOR.MINOR.PATCH on the release commit.
# Anchored and exact, so starcom-v0.2.25 and v1-scratch do not match.
RELEASE_TAG = re.compile(r'^refs/tags/v\d+\.\d+\.\d+$')
RADIO_LINK_BENCH = 'scripts/radio_link_bench.py'   # GAP: not built yet

# Flash paths (docs/FLASHING.md). FLASH_SWD: the debug probe is on this
# board, so the flash script (OpenOCD) is path 2 and picotool is path 1.
# FLASH_USB: no probe on this board; picotool over USB only.
FLASH_SWD = 'swd'
FLASH_USB = 'usb'
# The ONE role table: role -> (build dir, CMake preset, bench script, flash
# path). Roles are named by function. No other code maps a role to these.
ROLES: Dict[str, Tuple[str, str, str, str]] = {
    'vehicle': ('build_flight', 'vehicle-flight', 'bench_sim.py', FLASH_SWD),
    'station': ('build_station_flight', 'station-flight', 'station_bench_sim.py',
                FLASH_USB),
}
_RE_TREE = re.compile(r'constexpr const char\*\s+kFirmwareTreeId\s*=\s*"([^"]*)"')
_RE_HASH = re.compile(r'constexpr const char\*\s+kGitHash\s*=\s*"([^"]*)"')
_RE_RESULT = re.compile(r'RESULT:\s*(\d+)/(\d+)\s*PASS')
# Printed by bench_sim.py / station_bench_sim.py (_rc_test_common.board_usb_serial).
_RE_BOARD = re.compile(r'^board_usb_serial:\s*(\S+)\s*$', re.MULTILINE)
NOTE_FORMAT = 'rc-bench v2'
FRESH_ENV = 'RC_BENCH_FRESH'
UNKNOWN_SERIAL = 'unknown'
PRESETS: Dict[str, str] = {r: v[1] for r, v in ROLES.items()}


class GateFail(Exception):
    """Block the push with this message."""


def say(msg: str) -> None:
    print(f'pre-push: {msg}', flush=True)


def git(repo: Path, *args: str, check: bool = True) -> str:
    r = subprocess.run(['git', *args], cwd=str(repo), capture_output=True,
                       text=True, check=False)
    if check and r.returncode != 0:
        raise GateFail(f'git {" ".join(args)} failed:\n  {r.stderr.strip()}')
    return r.stdout.strip()


def run(cmd: List[str], cwd: Path) -> None:
    """Run with output on the console; block the push on failure."""
    say('$ ' + ' '.join(cmd))
    if subprocess.run(cmd, cwd=str(cwd), check=False).returncode != 0:
        raise GateFail(f'command failed: {" ".join(cmd)}')


# ---------------------------------------------------------------------------
# 1-2. What is being pushed
# ---------------------------------------------------------------------------

@dataclass
class PushPlan:
    local_ref: str
    local_sha: str
    commits: List[str] = field(default_factory=list)
    paths: Set[str] = field(default_factory=set)
    firmware_commits: List[str] = field(default_factory=list)
    isolate: Dict[str, List[str]] = field(default_factory=dict)
    vehicle: bool = False
    station: bool = False
    radio: bool = False

    @property
    def roles(self) -> List[str]:
        need = {'vehicle': self.vehicle, 'station': self.station}
        return [r for r in ROLES if need[r]]


def commits_to_push(repo: Path, remote: str, local_sha: str,
                    remote_sha: str) -> List[str]:
    args = ['rev-list', '--reverse', local_sha]
    if remote_sha != ZERO:
        if subprocess.run(['git', 'cat-file', '-e', f'{remote_sha}^{{commit}}'],
                          cwd=str(repo), capture_output=True).returncode != 0:
            raise GateFail(f'remote tip {remote_sha[:12]} is not in this clone; '
                           f'run git fetch {remote} first')
        args.append(f'^{remote_sha}')
    known = git(repo, 'remote').split()
    args += ['--not', f'--remotes={remote}' if remote in known else '--remotes']
    out = git(repo, *args)
    return [c for c in out.splitlines() if c]


def changed_paths(repo: Path, commit: str) -> List[str]:
    out = git(repo, 'diff-tree', '-r', '--root', '--no-commit-id',
              '--name-only', '-m', '--first-parent', commit)
    return [p for p in out.splitlines() if p]


def make_plan(repo: Path, remote: str, local_ref: str, local_sha: str,
              remote_sha: str) -> PushPlan:
    sections = ft.load(repo, local_sha)
    plan = PushPlan(local_ref=local_ref, local_sha=local_sha)
    plan.commits = commits_to_push(repo, remote, local_sha, remote_sha)
    for c in plan.commits:
        paths = changed_paths(repo, c)
        plan.paths.update(paths)
        fw = [p for p in paths if matrix.is_firmware(p, sections)]
        if not fw:
            continue
        plan.firmware_commits.append(c)
        cls = sorted({k for p in fw for k in ft.classes(p, sections)})
        if cls:
            plan.isolate[c] = cls
    plan.vehicle, plan.station = matrix.match(sorted(plan.paths), sections)
    plan.radio = matrix.radio(sorted(plan.paths), sections)
    return plan


def subject(repo: Path, c: str) -> str:
    return git(repo, 'log', '-1', '--format=%h %s', c)


def warn(msg: str) -> None:
    print(f'pre-push: WARNING: {msg}', flush=True)


def check_shape(repo: Path, plan: PushPlan) -> List[str]:
    """Push shape is an AGENT rule (SESSION_CHECKLIST 6b). Warn, never block."""
    out: List[str] = []
    fw = plan.firmware_commits
    if len(fw) > MAX_FIRMWARE_COMMITS:
        listing = '\n  '.join(subject(repo, c) for c in fw)
        out.append(
            f'{len(fw)} firmware commits in one push (agent rule: at most '
            f'{MAX_FIRMWARE_COMMITS}). One bench run covers them all, so a '
            f'failure is harder to trace. Next time split the push: '
            f'git push <remote> <sha>:<branch>.\n  {listing}')
    if plan.isolate and len(fw) > 1:
        lines = [f'{subject(repo, c)}  [{", ".join(cls)}]'
                 for c, cls in plan.isolate.items()]
        out.append(
            'agent rule: a pyro/deploy, guard, radio-adapter or turnaround '
            'commit goes in a push with no other firmware commit.\n  '
            + '\n  '.join(lines))
    for m in out:
        warn(m)
    return out


# ---------------------------------------------------------------------------
# 3. Bench notes
# ---------------------------------------------------------------------------

def find_note(repo: Path, key: str) -> Optional[str]:
    r = subprocess.run(['git', 'notes', '--ref', NOTES_REF, 'show', key],
                       cwd=str(repo), capture_output=True, text=True)
    return r.stdout if r.returncode == 0 else None


def fresh_mode(argv: List[str]) -> bool:
    """--fresh, or RC_BENCH_FRESH set to anything but '', 0, false, no, off."""
    val = os.environ.get(FRESH_ENV, '').strip().lower()
    return '--fresh' in argv or val not in ('', '0', 'false', 'no', 'off')


def bench_key(repo: Path, sha: str, write: bool) -> Tuple[str, Dict[str, str]]:
    """(bench key, tool version lines) for ``sha``. Fails closed (no tool)."""
    versions = ft.tool_versions(repo, sha)
    return ft.bench_key(repo, sha, PRESETS, write=write, versions=versions), versions


def attached_board_serials() -> Optional[Set[str]]:
    """USB serials of attached RocketChip CDC ports (VID:PID as in
    _rc_test_common). Enumeration only: no port is opened, nothing is sent.
    None when pyserial is missing (then no note is reused)."""
    try:
        import serial.tools.list_ports as lp  # type: ignore
        from _rc_test_common import ROCKETCHIP_USB_PID, ROCKETCHIP_USB_VID
    except Exception:  # noqa: BLE001 - any import failure = cannot read serials
        return None
    return {i.serial_number for i in lp.comports()
            if i.vid == ROCKETCHIP_USB_VID and i.pid == ROCKETCHIP_USB_PID
            and i.serial_number}


def note_boards(note: str) -> Dict[str, str]:
    m = re.search(r'^boards: (.*)$', note, re.MULTILINE)
    out: Dict[str, str] = {}
    for item in (m.group(1).split() if m else []):
        role, _, ser = item.partition('=')
        if role and ser:
            out[role] = ser
    return out


def note_covers(note: str, roles: List[str], radio: bool,
                attached: Optional[Set[str]]) -> Tuple[bool, str]:
    """(reuse?, reason). See the module docstring, step 3."""
    if 'result: PASS' not in note:
        return False, 'note is not a PASS'
    m = re.search(r'^roles: (.*)$', note, re.MULTILINE)
    have = set(m.group(1).split()) if m else set()
    need = set(roles) | ({'radio-link'} if radio else set())
    if not need <= have:
        return False, f'note covers {sorted(have)}, push needs {sorted(need)}'
    boards = set(roles) | (set(ROLES) if radio else set())
    recorded = note_boards(note)
    if attached is None:
        return False, 'cannot read USB serials (pyserial missing)'
    for b in sorted(boards):
        ser = recorded.get(b)
        if not ser or ser == UNKNOWN_SERIAL:
            return False, f'note has no USB serial for the {b} board'
        if ser not in attached:
            return False, (f'{b} board {ser} from the note is not attached '
                           f'(attached: {", ".join(sorted(attached)) or "none"})')
    return True, 'boards ' + ' '.join(f'{b}={recorded[b]}' for b in sorted(boards))


# ---------------------------------------------------------------------------
# 4. Bench worktree, build, flash record
# ---------------------------------------------------------------------------

def primary_tree(repo: Path) -> Path:
    common = git(repo, 'rev-parse', '--path-format=absolute', '--git-common-dir')
    return Path(common).parent


MAIN_FALLBACK = 'the main checkout (first line of git worktree list)'
OPENOCD_SCRIPT = 'scripts\\start_openocd_pico_sdk.ps1'


def main_worktree(repo: Path) -> Optional[Path]:
    """The main worktree: the first line of `git worktree list --porcelain`
    is its `worktree <path>` entry. None when the lookup fails (the caller
    then prints MAIN_FALLBACK; a print path never blocks the push)."""
    try:
        r = subprocess.run(['git', 'worktree', 'list', '--porcelain'], cwd=str(repo),
                           capture_output=True, text=True)
    except OSError:
        return None
    first = r.stdout.splitlines()[0] if r.returncode == 0 and r.stdout else ''
    if not first.startswith('worktree ') or not first[len('worktree '):].strip():
        return None
    return Path(first[len('worktree '):].strip())


def openocd_step(main: Optional[Path]) -> str:
    """How to start OpenOCD. Only for the OpenOCD check and the Path 2
    alternative; the gate never starts OpenOCD."""
    where = str(main) if main else MAIN_FALLBACK
    return (f'Start OpenOCD from {where}:\n'
            f'  powershell -ExecutionPolicy Bypass -File {OPENOCD_SCRIPT}\n'
            'Start it from that folder. Never start it from the bench worktree:\n'
            'the running OpenOCD process locks the folder it starts in.')


def bench_worktree(primary: Path) -> Path:
    return primary.parent / f'{primary.name}-bench'


def prepare_worktree(primary: Path, wt: Path, sha: str) -> None:
    if not (wt / '.git').exists():
        say(f'creating bench worktree {wt} (gate-owned; do not edit it)')
        git(primary, 'worktree', 'add', '--detach', str(wt), sha)
    else:
        dirty = git(wt, 'status', '--porcelain', '--untracked-files=no')
        if dirty:
            raise GateFail(f'bench worktree {wt} has edits. It is gate-owned. '
                           f'Inspect and clean it by hand, then push again.\n{dirty}')
        if git(wt, 'rev-parse', 'HEAD') != sha:
            git(wt, 'checkout', '--detach', '--quiet', sha)
    # docs/agents/WORKTREE.md: submodules do not come along.
    run(['git', 'submodule', 'update', '--init', '--recursive'], wt)


def header_value(header: Path, rx: 're.Pattern[str]') -> Optional[str]:
    if not header.is_file():
        return None
    m = rx.search(header.read_text(encoding='utf-8'))
    return m.group(1) if m else None


def build_role(wt: Path, role: str, want: str,
               keyed: Dict[str, str]) -> Tuple[Path, str]:
    bdir_name, preset = ROLES[role][:2]
    bdir = wt / bdir_name
    elf = bdir / 'rocketchip.elf'
    header = bdir / 'generated' / 'rocketchip' / 'version.h'
    if elf.is_file() and header_value(header, _RE_TREE) == want:
        say(f'{role}: reuse {elf} (firmware tree {want[:12]} unchanged)')
    else:
        if not (bdir / 'CMakeCache.txt').is_file():
            run(['cmake', '--preset', preset, '-G', 'Ninja'], wt)
        run(['cmake', '--build', str(bdir)], wt)
    got = header_value(header, _RE_TREE)
    if got != want:
        raise GateFail(f'{role}: build ID {got} != firmware tree {want} of the '
                       f'pushed commit (bench worktree must be clean)')
    check_build_tools(bdir, role, keyed, wt)
    check_list_complete(wt, bdir, role)
    return elf, header_value(header, _RE_HASH) or '?'


def check_list_complete(wt: Path, bdir: Path, role: str) -> None:
    """Every repo file this build used is in [elf] (scripts/ci/check_elf_paths.py)."""
    try:
        res = cep.check(wt, [bdir])
    except (cep.CheckError, ft.FirmwareTreeError) as exc:
        raise GateFail(f'{role}: list-completeness check failed: {exc}')
    if not res.ok:
        raise GateFail(f'{role}: the build used repo files that are not in [elf] of '
                       f'{ft.LIST_REL}, so the build ID and the bench key miss '
                       f'them:\n{cep.report(res)}')


def check_build_tools(bdir: Path, role: str, keyed: Dict[str, str],
                      wt: Optional[Path] = None) -> None:
    """The build must have used the compiler, picotool, pioasm and Pico SDK
    that the bench key names. Else the key would not describe the image."""
    if not ft.cache_value(bdir, 'CMAKE_C_COMPILER'):
        raise GateFail(f'{role}: CMAKE_C_COMPILER not in {bdir / "CMakeCache.txt"}')
    for tool in ft.TOOLS:
        exe = ft.tool_from_cache(tool, bdir)
        if exe is None:
            if tool == 'arm-none-eabi-gcc':
                raise GateFail(f'{role}: the compiler in {bdir / "CMakeCache.txt"} '
                               f'is not a file')
            continue   # not built yet (the SDK builds it on first use)
        try:
            got = ft.tool_version(tool, exe)
        except ft.FirmwareTreeError as exc:
            raise GateFail(f'{role}: {exc}')
        if got != keyed[tool]:
            env = ft.TOOLS[tool][0]
            raise GateFail(
                f'{role}: the build used {tool} "{got}" ({exe}), but the bench '
                f'key has "{keyed[tool]}". Set {env}={exe} so the key names '
                f'the tool the build uses, then push again.')
    sdk = ft.cache_value(bdir, 'PICO_SDK_PATH')
    if not sdk:
        raise GateFail(f'{role}: PICO_SDK_PATH not in {bdir / "CMakeCache.txt"}')
    try:
        used = ft.sdk_inputs(Path(sdk), wt or bdir.parent)
    except ft.FirmwareTreeError as exc:
        raise GateFail(f'{role}: {exc}')
    for k in ft.SDK_KEYS:
        if used[k] != keyed[k]:
            raise GateFail(
                f'{role}: the build used Pico SDK {k} "{used[k]}", but the bench '
                f'key has "{keyed[k]}". Set RC_PICO_SDK={sdk} (or reconfigure the '
                f'main checkout build), then push again.')



def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open('rb') as f:
        for chunk in iter(lambda: f.read(1 << 20), b''):
            h.update(chunk)
    return h.hexdigest()


def flash_recorded(elf: Path) -> bool:
    sidecar = elf.with_name(elf.name + '.flashed.json')
    try:
        return json.loads(sidecar.read_text(encoding='utf-8')).get('sha256') == sha256(elf)
    except (OSError, ValueError):
        return False


def flash_instructions(main: Optional[Path], wt: Path, role: str, elf: Path,
                       picotool: str = 'picotool') -> str:
    """Flash text for one role. Both setups flash by Path 1 (picotool over
    USB). ``main`` is main_worktree() (None -> MAIN_FALLBACK)."""
    uf2 = elf.with_suffix('.uf2')
    flash = wt / 'scripts' / 'flash_elf_halt_write.py'
    swd = ROLES[role][3] == FLASH_SWD
    setup = ('debug probe on this board' if swd else 'USB only, no debug probe')
    text = (
        f'  {role} setup ({setup}):\n'
        '    Path 1 (use this): flash over USB with picotool:\n'
        f'      {picotool} load {uf2} -f --bus <bus> --address <addr>\n'
        '      (-f returns to the app by itself; docs/FLASHING.md "Picotool")\n'
        '    Then record the flash:\n'
        f'      python {flash} --elf {elf} --record-only\n'
        '    Then wait for LED + CDC. If a fresh boot is needed: CLI k\n'
        '    (I2C-safe restart). No reset halt (docs/FLASHING.md rule 2).')
    if not swd:
        return text
    step = '\n'.join('      ' + ln for ln in openocd_step(main).splitlines())
    return text + (
        '\n    The debug probe does not flash in this procedure. It serves only\n'
        '    the OpenOCD check.\n'
        '    Path 2 (alternative, this setup only; docs/FLASHING.md\n'
        '    "Iterative flash"): probe halt-write, needs OpenOCD:\n'
        f'      python {flash} --elf {elf}\n'
        + step)


def needs_openocd(roles: List[str]) -> bool:
    """OpenOCD is needed when a role flashes over the debug probe (ROLES)."""
    return any(ROLES[r][3] == FLASH_SWD for r in roles)


def openocd_listening() -> bool:
    # netstat only. Do NOT open a socket to :3333 - a GDB connect can halt
    # the core (openocd_cmsis_dap.cfg halt-only gdb-attach).
    try:
        r = subprocess.run(['netstat', '-an'], capture_output=True, text=True)
    except OSError:
        return False  # no netstat: treat as not listening (fail closed)
    return bool(re.search(r'127\.0\.0\.1:3333\s+\S+\s+LISTENING', r.stdout))


def run_bench(wt: Path, script: str, sha: str, label: str) -> Tuple[str, str]:
    """(result text, USB serial of the benched board or UNKNOWN_SERIAL)."""
    cmd = [sys.executable, '-u', str(wt / 'scripts' / script), '--commit', sha]
    say(f'{label}: $ ' + ' '.join(cmd))
    r = subprocess.run(cmd, cwd=str(wt), capture_output=True, text=True)
    out = (r.stdout or '') + (r.stderr or '')
    print(out, flush=True)
    if r.returncode == 2:
        raise GateFail(f'{label}: exit 2 (board not found / tool self-check / '
                       f'watchdog). The push needs this bench: a skip is a fail.')
    if r.returncode != 0:
        raise GateFail(f'{label}: FAIL (exit {r.returncode})')
    m = _RE_RESULT.search(out)
    b = _RE_BOARD.findall(out)
    if not b:
        warn(f'{label}: no board_usb_serial line. The note records '
             f'"{UNKNOWN_SERIAL}", so it is never reused.')
    return (f'{m.group(1)}/{m.group(2)} PASS' if m else 'PASS'), (b[-1] if b else UNKNOWN_SERIAL)


RADIO_GAP = (
    'RADIO PUSH: this push changes radio/link code ([radio] in\n'
    'scripts/ci/firmware_paths.txt). The two-board round trip\n'
    f'({RADIO_LINK_BENCH}) is not in this commit, so the radio rule is not\n'
    'active and this push is NOT radio-benched. Agent rule (SESSION_CHECKLIST\n'
    '6a): run the two-board soak by hand and cite it.')


def radio_rule_active(repo: Path, sha: str) -> bool:
    """The blocking radio rule lands with the round-trip bench, not before."""
    return subprocess.run(['git', 'cat-file', '-e', f'{sha}:{RADIO_LINK_BENCH}'],
                          cwd=str(repo), capture_output=True).returncode == 0


# ---------------------------------------------------------------------------
# Per-ref handling
# ---------------------------------------------------------------------------

def handle_tag(repo: Path, local_ref: str, local_sha: str) -> None:
    if not RELEASE_TAG.match(local_ref):
        return
    # Annotated tag: local_sha is the tag object. Peel to the commit before
    # the note lookup. A tag on a tree/blob has no commit: nothing to gate.
    commit = git(repo, 'rev-parse', '--verify', '--quiet',
                 f'{local_sha}^{{commit}}', check=False)
    if not commit:
        say(f'{local_ref}: tag does not point at a commit; no bench rule.')
        return
    note = find_note(repo, bench_key(repo, commit, write=True)[0])
    if not note or 'result: PASS' not in note:
        raise GateFail(f'{local_ref}: release tag on {commit[:12]}, which has no '
                       f'bench PASS note. Only a bench-passed commit is tagged.')
    say(f'{local_ref}: bench PASS note found for {commit[:12]}.')
    say('A bench pass is not flight qualification: run the soak test and the '
        'flight checklist on this exact binary (HW_GATE_DISCIPLINE Rule 8).')


def handle_branch(repo: Path, remote: str, local_ref: str, local_sha: str,
                  remote_sha: str, plan_only: bool, fresh: bool = False) -> None:
    plan = make_plan(repo, remote, local_ref, local_sha, remote_sha)
    say(f'{local_ref} @ {local_sha[:12]}: {len(plan.commits)} commit(s), '
        f'{len(plan.firmware_commits)} firmware, roles={plan.roles or "none"}'
        f'{", RADIO PUSH" if plan.radio else ""}'
        f'{", isolate=" + str(plan.isolate) if plan.isolate else ""}')
    check_shape(repo, plan)
    if plan.radio and not radio_rule_active(repo, local_sha):
        warn(RADIO_GAP)
        plan.radio = False
    if not plan.roles:
        say('no firmware or gate path changed: no bench needed.')
        return
    key, versions = bench_key(repo, local_sha, write=not plan_only)
    say(f'bench key {key[:12]} ({ft.BENCH_KEY_FORMAT}): '
        + '; '.join(f'{t} "{v}"' for t, v in versions.items())
        + '; preset ' + ' '.join(f'{r}={p}' for r, p in sorted(PRESETS.items())))
    note = find_note(repo, key)
    if fresh:
        why = 'fresh mode: notes ignored'
    elif not note:
        why = f'no note for key {key[:12]}'
    else:
        ok, why = note_covers(note, plan.roles, plan.radio, attached_board_serials())
        if ok:
            say(f'bench note {key[:12]} covers this firmware tree, tools and '
                f'{why}: skip flash and bench.')
            return
        why = f'note {key[:12]} not reused: {why}'
    if plan_only:
        say(f'would bench {local_sha[:12]} ({", ".join(plan.roles)}'
            f'{" + radio link" if plan.radio else ""}); {why}.')
        return
    say(f'bench needed: {why}.')

    primary = primary_tree(repo)
    wt = bench_worktree(primary)
    main_wt = main_worktree(repo)
    if needs_openocd(plan.roles) and not openocd_listening():
        raise GateFail('OpenOCD is not on 127.0.0.1:3333 (the debug-probe setup '
                       'needs it for the OpenOCD check).\n' + openocd_step(main_wt))
    prepare_worktree(primary, wt, local_sha)
    want = ft.elf_tree_id(wt, local_sha)
    built: Dict[str, Tuple[Path, str]] = {r: build_role(wt, r, want, versions)
                                          for r in plan.roles}
    if plan.radio:
        # The link has two ends: both boards must run this firmware tree.
        for r in ROLES:
            if r not in built:
                built[r] = build_role(wt, r, want, versions)
    # Flash with the picotool whose version is in the bench key.
    picotool = str(ft.resolve_tool('picotool', repo, local_sha))
    missing = [flash_instructions(main_wt, wt, r, elf, picotool)
               for r, (elf, _) in built.items() if not flash_recorded(elf)]
    if missing:
        raise GateFail('flash the pushed build (Path 1, picotool over USB, for '
                       'each setup), record it, wait LED+CDC (docs/FLASHING.md), '
                       'then run git push again. The gate does not flash.\n'
                       + '\n'.join(missing))

    results: Dict[str, str] = {}
    boards: Dict[str, str] = {}
    for r in built:
        results[r], boards[r] = run_bench(wt, ROLES[r][2], local_sha, f'{r} bench')
    if plan.radio:
        results['radio-link'], _ = run_bench(wt, Path(RADIO_LINK_BENCH).name,
                                             local_sha, 'radio link')

    stamp = datetime.datetime.now().astimezone().isoformat(timespec='seconds')
    rng = f'{plan.commits[0][:12]}..{plan.commits[-1][:12]}' if plan.commits else '-'
    lines = [NOTE_FORMAT, 'result: PASS', f'date: {stamp}',
             f'commit: {local_sha}', f'firmware_tree: {want}',
             f'range: {rng} ({len(plan.commits)} commits)',
             f'roles: {" ".join(results)}',
             'boards: ' + ' '.join(f'{r}={s}' for r, s in boards.items())]
    lines += [f'{t}: {v}' for t, v in versions.items()]
    lines.append('preset: ' + ' '.join(f'{r}={p}' for r, p in sorted(PRESETS.items())))
    if fresh:
        lines.append('fresh: yes')
    lines += [f'{r}: {res} image flight-{built[r][1]}' if r in built
              else f'{r}: {res}' for r, res in results.items()]
    git(repo, 'notes', '--ref', NOTES_REF, 'add', '-f', '-m', '\n'.join(lines), key)
    cite = ', '.join(f'{r} {res}' + (f' (flight-{built[r][1]})' if r in built else '')
                     for r, res in results.items())
    say('PASS. Rule 3 citation for the last commit or CHANGELOG entry:')
    print(f'  Verified at push: {cite}; firmware tree {want[:12]}; range {rng}.',
          flush=True)


def main(argv: List[str]) -> int:
    plan_only = '--plan' in argv
    fresh = fresh_mode(argv)
    args = [a for a in argv[1:] if a not in ('--plan', '--fresh')]
    remote = args[0] if args else 'origin'
    # A hook gets GIT_DIR and others for this repo. Drop them so git calls
    # in the bench worktree and the Pico SDK see their own repo (cwd).
    keep = ft.clean_git_env()
    os.environ.clear()
    os.environ.update(keep)
    repo = Path(git(Path.cwd(), 'rev-parse', '--show-toplevel'))
    if plan_only:
        upstream = git(repo, 'rev-parse', '--symbolic-full-name', '@{u}', check=False)
        remote_sha = git(repo, 'rev-parse', '@{u}', check=False) or ZERO
        lines = [f'{git(repo, "symbolic-ref", "HEAD")} {git(repo, "rev-parse", "HEAD")} '
                 f'{upstream or "-"} {remote_sha}']
    else:
        lines = sys.stdin.read().splitlines()
    if fresh:
        say('FRESH MODE: every bench note is ignored; a push with a firmware '
            'or gate path benches.')
    try:
        for line in lines:
            parts = line.split()
            if len(parts) != 4:
                continue
            local_ref, local_sha, _remote_ref, remote_sha = parts
            if local_sha == ZERO or local_ref.startswith('refs/notes/'):
                continue
            if local_ref.startswith('refs/tags/'):
                handle_tag(repo, local_ref, local_sha)
                continue
            handle_branch(repo, remote, local_ref, local_sha, remote_sha,
                          plan_only, fresh)
    except (GateFail, ft.FirmwareTreeError) as exc:
        print(f'\nPRE-PUSH BLOCKED: {exc}\n', flush=True)
        print('Nothing was pushed. Fix an unpushed commit with\n'
              '  git commit --fixup=<sha>\n'
              '  GIT_SEQUENCE_EDITOR=: git rebase -i --autosquash <remote>/<branch>\n'
              'then push again. Do not use --no-verify.', flush=True)
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
