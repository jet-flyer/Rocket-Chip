#!/usr/bin/env python3
"""Firmware path list + firmware-tree hash (build ID) helpers.

UNTESTED ON HARDWARE. Draft 2026-10-08 (pre-push bench plan).

Single reader of ``scripts/ci/firmware_paths.txt`` for Python code. The
same file feeds ``cmake/rc_version.cmake`` (``kFirmwareTreeId``), so the
build ID and the gate agree on what "firmware" means.

Firmware-tree hash
------------------
``git ls-tree -r <rev> -- <[elf] entries>`` lists every firmware blob and
submodule commit at ``rev``. The git blob id of that listing
(``git hash-object --stdin``) is the firmware-tree hash. It changes when a
firmware byte changes, and it does NOT change for a docs-only commit. CMake
computes the same value at configure time (``--no-filters``, same git
options), so ``kFirmwareTreeId`` in version.h can be compared with it.

Bench key (format ``rc-bench-key v2``)
-------------------------------------
A blob id over this text, in this order:

    rc-bench-key v2
    [tool]
    arm-none-eabi-gcc: <first line of `arm-none-eabi-gcc --version`>
    picotool: <first line of `picotool version`>
    pioasm: <first line of `pioasm --version`>
    pico-sdk: <SDK path; '<repo>/...' when inside the checkout>
    pico-sdk-describe: <git -C <sdk> describe --always --dirty --tags>
    pico-sdk-head: <git -C <sdk> rev-parse HEAD>
    pico-sdk-submodules: <path>=<state>:<sha>; ...  (submodule status
                         --recursive, sorted; state ok|uninit|moved|conflict)
    preset: <role>=<CMake preset> ...   (sorted by role)
    [elf]
    <git ls-tree -r <rev> -- [elf] entries>
    [gate]
    <git ls-tree -r <rev> -- [gate] entries>

The pre-push gate stores each bench PASS as a git note
(``refs/notes/rc-bench``) on that blob. A push finds the note only when
the firmware, the gate files, the compiler, picotool, pioasm, the Pico SDK
(path, commit, dirty state, submodules) and the presets are all the same.

Tool lookup (``resolve_tool``), per tool, first hit wins:
  1. ``RC_ARM_GCC`` / ``RC_PICOTOOL`` / ``RC_PIOASM`` (full path);
  2. the tool that a configured flight build in this checkout uses
     (``build_flight`` / ``build_station_flight`` CMakeCache.txt:
     ``CMAKE_C_COMPILER``, ``picotool_DIR``, ``pioasm_DIR``; else the
     picotool / pioasm program that build.ninja runs; else the copy the SDK
     built in that build dir: ``_deps/picotool/picotool``,
     ``pioasm-install/pioasm/pioasm``);
  3. the Pico VS Code install that CMakeLists.txt pins at ``rev``
     (``~/.pico-sdk/toolchain/<toolchainVersion>/bin/arm-none-eabi-gcc``,
     ``~/.pico-sdk/picotool/<picotoolVersion>/picotool/picotool``,
     ``~/.pico-sdk/tools/<sdkVersion>/pioasm/pioasm``; ``pico-vscode.cmake``);
  4. ``PATH``.
SDK lookup (``sdk_path``), first hit wins:
  1. ``RC_PICO_SDK``;
  2. ``PICO_SDK_PATH`` in the flight build CMakeCache.txt files (they must
     agree);
  3. what CMake would pick: ``~/.pico-sdk/sdk/<sdkVersion>`` when
     ``~/.pico-sdk/cmake/pico-vscode.cmake`` exists, else the
     ``PICO_SDK_PATH`` env var, else the in-repo ``pico-sdk`` submodule.
The SDK must be the top of a git checkout. A missing tool, a failed run, an
empty version line, or an SDK that is not a git checkout is an error. The
key never hashes an empty string and never falls back to a version name
(fail closed). After the build, the pre-push gate checks that the build
used the same tools and SDK.
"""
from __future__ import annotations

import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Optional, Tuple

LIST_REL = 'scripts/ci/firmware_paths.txt'
_GIT_OPTS = ('-c', 'core.quotePath=true')
BENCH_KEY_FORMAT = 'rc-bench-key v2'
_EXE = '.exe' if os.name == 'nt' else ''
# tool name -> (override env var, version argument, CMakeLists.txt pin
# variable, CMakeCache.txt variable)
TOOLS: Dict[str, Tuple[str, str, str, str]] = {
    'arm-none-eabi-gcc': ('RC_ARM_GCC', '--version', 'toolchainVersion', 'CMAKE_C_COMPILER'),
    'picotool': ('RC_PICOTOOL', 'version', 'picotoolVersion', 'picotool_DIR'),
    'pioasm': ('RC_PIOASM', '--version', 'sdkVersion', 'pioasm_DIR'),
}
SDK_KEYS = ('pico-sdk', 'pico-sdk-describe', 'pico-sdk-head', 'pico-sdk-submodules')
# Flight build dirs in a checkout (same as the pre-push gate ROLES).
FLIGHT_BUILD_DIRS = ('build_flight', 'build_station_flight')


class FirmwareTreeError(RuntimeError):
    """git failed or the list file is malformed."""


def parse(text: str) -> Dict[str, List[str]]:
    """Parse the list file text into {section: [entries]}."""
    sections: Dict[str, List[str]] = {}
    current: Optional[str] = None
    for lineno, raw in enumerate(text.splitlines(), start=1):
        line = raw.strip()
        if not line or line.startswith('#'):
            continue
        if line.startswith('[') and line.endswith(']'):
            current = line[1:-1].strip()
            sections.setdefault(current, [])
            continue
        if current is None:
            raise FirmwareTreeError(f'{LIST_REL}:{lineno}: entry before any [section]')
        if '\\' in line:
            raise FirmwareTreeError(f'{LIST_REL}:{lineno}: use / not \\')
        if current == 'elf' and '*' in line:
            raise FirmwareTreeError(
                f'{LIST_REL}:{lineno}: [elf] entries must be git literal paths (no *)')
        sections[current].append(line)
    if not sections.get('elf'):
        raise FirmwareTreeError(f'{LIST_REL}: no [elf] entries')
    return sections


def _git(repo: Path, *args: str, stdin: Optional[bytes] = None) -> bytes:
    r = subprocess.run(
        ['git', *_GIT_OPTS, *args],
        cwd=str(repo), input=stdin,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
    )
    if r.returncode != 0:
        raise FirmwareTreeError(
            f'git {" ".join(args[:3])}... failed: '
            f'{r.stderr.decode("utf-8", "replace").strip()}')
    return r.stdout


def load(repo: Path, rev: Optional[str] = None) -> Dict[str, List[str]]:
    """Load the list from the working tree (rev=None) or from a commit."""
    if rev is None:
        path = Path(repo) / LIST_REL
        if not path.is_file():
            raise FirmwareTreeError(f'missing {LIST_REL}')
        return parse(path.read_text(encoding='utf-8'))
    return parse(_git(Path(repo), 'show', f'{rev}:{LIST_REL}').decode('utf-8'))


def entry_matches(path: str, entry: str) -> bool:
    """Match one repo-relative path against one list entry."""
    path = path.replace('\\', '/')
    if entry.endswith('*'):
        return path.startswith(entry[:-1])
    if entry.endswith('/'):
        return path.startswith(entry)
    return path == entry or path.startswith(entry + '/')


def in_section(path: str, sections: Dict[str, List[str]], name: str) -> bool:
    return any(entry_matches(path, e) for e in sections.get(name, ()))


def classes(path: str, sections: Dict[str, List[str]]) -> List[str]:
    """Isolation classes ([isolate:<name>]) that this path belongs to."""
    out = []
    for name, entries in sections.items():
        if name.startswith('isolate:') and any(entry_matches(path, e) for e in entries):
            out.append(name.split(':', 1)[1])
    return out


def _listing(repo: Path, rev: str, entries: Iterable[str]) -> bytes:
    return _git(repo, 'ls-tree', '-r', rev, '--', *entries)


def _hash(repo: Path, data: bytes, write: bool) -> str:
    args = ['hash-object', '--stdin']
    if write:
        args.insert(1, '-w')
    return _git(repo, *args, stdin=data).decode('ascii').strip()


def elf_tree_id(repo: Path, rev: str = 'HEAD',
                sections: Optional[Dict[str, List[str]]] = None) -> str:
    """Firmware-tree hash of ``rev`` (same value CMake writes to version.h).

    ``sections`` defaults to the working-tree list for HEAD (what CMake
    reads) and to the list stored in ``rev`` for any other revision.
    """
    repo = Path(repo)
    if sections is None:
        sections = load(repo, None if rev == 'HEAD' else rev)
    return _hash(repo, _listing(repo, rev, sections['elf']), write=False)


def _pins(repo: Path, rev: str) -> Dict[str, str]:
    """``set(<name> <value>)`` tool pins from CMakeLists.txt at ``rev``."""
    r = subprocess.run(['git', *_GIT_OPTS, 'show', f'{rev}:CMakeLists.txt'],
                       cwd=str(repo), capture_output=True, check=False)
    if r.returncode != 0:
        return {}
    text = r.stdout.decode('utf-8', 'replace')
    return dict(re.findall(
        r'^\s*set\s*\(\s*(toolchainVersion|picotoolVersion|sdkVersion)\s+([^\s)]+)\s*\)',
        text, re.MULTILINE))


def _home() -> Path:
    if os.name == 'nt' and os.environ.get('USERPROFILE'):
        return Path(os.environ['USERPROFILE'])
    return Path(os.environ.get('HOME') or os.environ.get('USERPROFILE') or '~').expanduser()


def _pinned_path(tool: str, pin: Optional[str]) -> Optional[Path]:
    if not pin:
        return None
    sdk = _home() / '.pico-sdk'
    if tool == 'arm-none-eabi-gcc':
        return sdk / 'toolchain' / pin / 'bin' / f'arm-none-eabi-gcc{_EXE}'
    if tool == 'picotool':
        return sdk / 'picotool' / pin / 'picotool' / f'picotool{_EXE}'
    return sdk / 'tools' / pin / 'pioasm' / f'pioasm{_EXE}'


def cache_value(bdir: Path, name: str) -> Optional[str]:
    """``name`` from ``bdir``/CMakeCache.txt; None if absent, empty or NOTFOUND."""
    try:
        text = (Path(bdir) / 'CMakeCache.txt').read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None
    m = re.search(rf'^{re.escape(name)}:[A-Z]+=(.*)$', text, re.MULTILINE)
    val = m.group(1).strip() if m else ''
    return val if val and not val.endswith('-NOTFOUND') else None


def tool_from_cache(tool: str, bdir: Path) -> Optional[Path]:
    """Program path that a configured build uses, from its CMakeCache.txt."""
    bdir = Path(bdir)
    val = cache_value(bdir, TOOLS[tool][3])
    if val:
        p = Path(val)
        if tool != 'arm-none-eabi-gcc':     # *_DIR -> program in that dir
            p = p / f'{tool}{_EXE}'
        if p.is_file():
            return p
    # pico-vscode.cmake sets picotool_DIR / pioasm_DIR as normal (not
    # cached) variables, so read the program path from build.ninja.
    if tool != 'arm-none-eabi-gcc':
        try:
            text = (bdir / 'build.ninja').read_text(encoding='utf-8', errors='replace')
        except OSError:
            text = ''
        rx = re.compile(r'([^\s"\']*[\\/]' + re.escape(tool) + r'(?:\.exe)?)(?=[\s"\']|$)')
        for m in rx.finditer(text.replace('$:', ':')):
            cand = Path(m.group(1))
            if cand.is_absolute() and cand.is_file():
                return cand
    # No installed copy found: the SDK built its own in the build dir.
    built = {'picotool': bdir / '_deps' / 'picotool' / f'picotool{_EXE}',
             'pioasm': bdir / 'pioasm-install' / 'pioasm' / f'pioasm{_EXE}'}.get(tool)
    if built is not None and built.is_file() and (bdir / 'CMakeCache.txt').is_file():
        return built
    return None


def resolve_tool(tool: str, repo: Path, rev: str) -> Path:
    """Path of ``tool`` by the lookup order in the module docstring."""
    env, _arg, pin_var, _cache = TOOLS[tool]
    tried: List[str] = []
    override = os.environ.get(env, '').strip()
    if override:
        if Path(override).is_file():
            return Path(override)
        raise FirmwareTreeError(f'{env}={override} is not a file')
    tried.append(f'${env} (not set)')
    for bdir in FLIGHT_BUILD_DIRS:
        found_c = tool_from_cache(tool, Path(repo) / bdir)
        if found_c is not None:
            return found_c
    tried.append('flight build CMakeCache.txt')
    pinned = _pinned_path(tool, _pins(Path(repo), rev).get(pin_var))
    if pinned is not None:
        if pinned.is_file():
            return pinned
        tried.append(str(pinned))
    found = shutil.which(tool)
    if found:
        return Path(found)
    tried.append('PATH')
    raise FirmwareTreeError(
        f'bench key: {tool} not found (tried {", ".join(tried)}). The bench key '
        f'hashes its version, so the gate stops here. Install the pinned '
        f'version, or set {env}=<full path>.')


def tool_version(tool: str, exe: Path) -> str:
    """First non-empty output line of the tool's version command."""
    arg = TOOLS[tool][1]
    try:
        r = subprocess.run([str(exe), arg], capture_output=True, text=True,
                           timeout=30, check=False)
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise FirmwareTreeError(f'bench key: cannot run {exe} {arg}: {exc}')
    lines = [ln.strip() for ln in ((r.stdout or '') + (r.stderr or '')).splitlines()
             if ln.strip()]
    if r.returncode != 0 or not lines:
        raise FirmwareTreeError(
            f'bench key: {exe} {arg} gave exit {r.returncode} and '
            f'{"no output" if not lines else repr(lines[0])}. The key never '
            f'hashes an empty version.')
    return lines[0]


def _norm_path(p: str) -> str:
    out = os.path.normpath(os.path.abspath(p)).replace('\\', '/')
    return out.lower() if os.name == 'nt' else out


def sdk_path(repo: Path, rev: str) -> Path:
    """Pico SDK directory by the lookup order in the module docstring."""
    repo = Path(repo)
    override = os.environ.get('RC_PICO_SDK', '').strip()
    if override:
        return Path(override)
    cached = {}
    for bdir in FLIGHT_BUILD_DIRS:
        v = cache_value(repo / bdir, 'PICO_SDK_PATH')
        if v:
            cached[bdir] = v
    if cached:
        if len({_norm_path(v) for v in cached.values()}) > 1:
            raise FirmwareTreeError(
                f'bench key: the flight builds use different Pico SDKs: {cached}. '
                f'Reconfigure them, or set RC_PICO_SDK.')
        return Path(next(iter(cached.values())))
    pin = _pins(repo, rev).get('sdkVersion')
    if pin and (_home() / '.pico-sdk' / 'cmake' / 'pico-vscode.cmake').is_file():
        return _home() / '.pico-sdk' / 'sdk' / pin
    if os.environ.get('PICO_SDK_PATH', '').strip():
        return Path(os.environ['PICO_SDK_PATH'].strip())
    return repo / 'pico-sdk'


# Git exports these to hooks (GIT_DIR and others). They point at the
# Rocket-Chip repo, so a "git -C <other repo>" call in a hook must drop them.
# List: git rev-parse --local-env-vars.
GIT_REPO_ENV_VARS = (
    'GIT_ALTERNATE_OBJECT_DIRECTORIES', 'GIT_CONFIG', 'GIT_CONFIG_PARAMETERS',
    'GIT_CONFIG_COUNT', 'GIT_OBJECT_DIRECTORY', 'GIT_DIR', 'GIT_WORK_TREE',
    'GIT_IMPLICIT_WORK_TREE', 'GIT_GRAFT_FILE', 'GIT_INDEX_FILE',
    'GIT_NO_REPLACE_OBJECTS', 'GIT_REPLACE_REF_BASE', 'GIT_PREFIX',
    'GIT_SHALLOW_FILE', 'GIT_COMMON_DIR')


def clean_git_env(env: Optional[Mapping[str, str]] = None) -> Dict[str, str]:
    """Return a copy of env without the repo-local GIT_* variables."""
    base = dict(os.environ if env is None else env)
    for name in GIT_REPO_ENV_VARS:
        base.pop(name, None)
    return base


def _sdk_git(sdk: Path, *args: str) -> str:
    try:
        # GIT_OPTIONAL_LOCKS=0: read only; do not refresh the SDK's index.
        r = subprocess.run(['git', '-C', str(sdk), *args], capture_output=True,
                           text=True, timeout=120, check=False,
                           env=dict(clean_git_env(), GIT_OPTIONAL_LOCKS='0'))
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise FirmwareTreeError(f'bench key: git -C {sdk} {" ".join(args)}: {exc}')
    if r.returncode != 0:
        raise FirmwareTreeError(
            f'bench key: git -C {sdk} {" ".join(args)} failed (exit {r.returncode}): '
            f'{(r.stderr or r.stdout).strip()[:200]}. The Pico SDK must be a git '
            f'checkout; the key never falls back to a version name.')
    return r.stdout


_SUB_STATE = {' ': 'ok', '-': 'uninit', '+': 'moved', 'U': 'conflict'}


def normalize_submodules(text: str) -> str:
    """``git submodule status --recursive`` -> 'path=state:sha; ...' sorted.
    The '(describe)' part is dropped: it depends on fetched tags."""
    items = []
    for line in text.splitlines():
        if not line.strip():
            continue
        state = _SUB_STATE.get(line[0], '?')
        parts = line[1:].split()
        if len(parts) < 2:
            raise FirmwareTreeError(f'bench key: bad submodule status line {line!r}')
        items.append(f'{parts[1]}={state}:{parts[0]}')
    return '; '.join(sorted(items)) if items else '(none)'


def sdk_inputs(sdk: Path, repo: Path) -> Dict[str, str]:
    """Key lines for the Pico SDK at ``sdk`` (fail closed)."""
    sdk = Path(sdk)
    if not sdk.is_dir():
        raise FirmwareTreeError(f'bench key: Pico SDK {sdk} is not a directory')
    top = _sdk_git(sdk, 'rev-parse', '--show-toplevel').strip()
    if _norm_path(top) != _norm_path(str(sdk)):
        raise FirmwareTreeError(
            f'bench key: Pico SDK {sdk} is not the top of a git checkout (git '
            f'top is {top}). An uninitialized submodule looks like this: run '
            f'git submodule update --init, or point RC_PICO_SDK at the SDK.')
    nsdk, nrepo = _norm_path(str(sdk)), _norm_path(str(repo))
    shown = ('<repo>/' + nsdk[len(nrepo) + 1:]) if nsdk.startswith(nrepo + '/') else nsdk
    describe = _sdk_git(sdk, 'describe', '--always', '--dirty', '--tags').strip()
    head = _sdk_git(sdk, 'rev-parse', 'HEAD').strip()
    subs = normalize_submodules(_sdk_git(sdk, 'submodule', 'status', '--recursive'))
    out = {'pico-sdk': shown, 'pico-sdk-describe': describe,
           'pico-sdk-head': head, 'pico-sdk-submodules': subs}
    for k, v in out.items():
        if not v or '\n' in v:
            raise FirmwareTreeError(f'bench key: empty or bad {k} value')
    return out


def tool_versions(repo: Path, rev: str) -> Dict[str, str]:
    """All [tool] key lines except presets, in key order (fail closed)."""
    out = {t: tool_version(t, resolve_tool(t, Path(repo), rev)) for t in TOOLS}
    out.update(sdk_inputs(sdk_path(Path(repo), rev), Path(repo)))
    return out


def key_input_header(versions: Mapping[str, str], presets: Mapping[str, str]) -> bytes:
    """The ``[tool]`` part of the bench key text (see module docstring)."""
    if not presets or any(not v.strip() for v in presets.values()):
        raise FirmwareTreeError('bench key: no CMake preset name')
    lines = [BENCH_KEY_FORMAT, '[tool]']
    for t in (*TOOLS, *SDK_KEYS):
        v = versions.get(t, '').strip()
        if not v or '\n' in v:
            raise FirmwareTreeError(f'bench key: empty or bad {t} line')
        lines.append(f'{t}: {v}')
    lines.append('preset: ' + ' '.join(f'{r}={presets[r]}' for r in sorted(presets)))
    return ('\n'.join(lines) + '\n').encode('utf-8')


def bench_key(repo: Path, rev: str, presets: Mapping[str, str],
              write: bool = False,
              versions: Optional[Mapping[str, str]] = None) -> str:
    """Bench key at ``rev``: the git-notes key for a PASS (format v2).

    ``presets`` maps role -> CMake preset. ``versions`` defaults to
    ``tool_versions(repo, rev)``; pass it to reuse one lookup.
    """
    repo = Path(repo)
    sections = load(repo, rev)
    if versions is None:
        versions = tool_versions(repo, rev)
    data = (key_input_header(versions, presets)
            + b'[elf]\n' + _listing(repo, rev, sections['elf'])
            + b'[gate]\n' + _listing(repo, rev, sections.get('gate', [])))
    return _hash(repo, data, write=write)


def dirty_firmware_paths(repo: Path) -> List[str]:
    """Working-tree firmware paths that differ from HEAD (incl. untracked)."""
    repo = Path(repo)
    sections = load(repo)
    out = _git(repo, 'status', '--porcelain=v1', '-z', '--untracked-files=all',
               '--', *sections['elf'])
    paths: List[str] = []
    fields = out.decode('utf-8', 'replace').split('\0')
    i = 0
    while i < len(fields):
        rec = fields[i]
        i += 1
        if len(rec) < 4:
            continue
        status, path = rec[:2], rec[3:]
        if 'R' in status or 'C' in status:
            i += 1  # skip the rename/copy source field
        paths.append(path)
    return paths


__all__ = [
    'LIST_REL', 'BENCH_KEY_FORMAT', 'TOOLS', 'SDK_KEYS', 'FLIGHT_BUILD_DIRS',
    'FirmwareTreeError', 'parse', 'load', 'entry_matches', 'in_section',
    'classes', 'elf_tree_id', 'cache_value', 'tool_from_cache', 'resolve_tool',
    'tool_version', 'sdk_path', 'normalize_submodules', 'sdk_inputs',
    'tool_versions', 'key_input_header', 'bench_key', 'dirty_firmware_paths',
]


def _main(argv: List[str]) -> int:
    """``python scripts/firmware_tree.py --key-inputs [<rev>]``: print the
    bench key [tool] lines (the role presets come from the pre-push gate)."""
    if len(argv) < 2 or argv[1] != '--key-inputs':
        print(__doc__)
        return 2
    rev = argv[2] if len(argv) > 2 else 'HEAD'
    repo = Path(_git(Path.cwd(), 'rev-parse', '--show-toplevel').decode().strip())
    try:
        for tool in TOOLS:
            exe = resolve_tool(tool, repo, rev)
            print(f'{tool}: {tool_version(tool, exe)}   ({exe})')
        sdk = sdk_path(repo, rev)
        for k, v in sdk_inputs(sdk, repo).items():
            print(f'{k}: {v}' + (f'   ({sdk})' if k == 'pico-sdk' else ''))
    except FirmwareTreeError as exc:
        print(f'ERROR: {exc}')
        return 1
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(_main(sys.argv))
