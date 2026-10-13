#!/usr/bin/env python3
"""List-completeness check: every repo file a flight build used is in [elf].

UNTESTED ON HARDWARE (it needs no board). Draft 2026-10-08.

The build ID (kFirmwareTreeId) and the bench key hash only the [elf]
entries of scripts/ci/firmware_paths.txt. If a build reads a repo file
that is not in [elf], that file can change while the build ID and the
bench key stay the same, and an old PASS would still match. This check
finds such a file by itself, so nobody has to remember the list.

    python scripts/ci/check_elf_paths.py <build dir> [<build dir> ...]

Run it after `cmake --build <build dir>` (pre-commit Gate 4 does this for
build_flight and build_station_flight). Per build dir it reads four
sources:
  deps       `ninja -t deps`: every header and source in the compiler
             dep files (one entry per object file).
  compdb     compile_commands.json: every compiled source file.
  inputs     `ninja -t inputs <ELF target>`: build-graph inputs, for
             example .pio programs (pioasm headers) and the linker script.
  regen      `ninja -t query build.ninja`, "input:" part: the CMake
             regeneration inputs (CMakeLists.txt, *.cmake). They change
             flags and defines. (`ninja -t inputs build.ninja` prints
             nothing for this edge with ninja 1.13, so the gate parses the
             query output instead.)
It ignores files outside the repo (SDK, toolchain) and generated files in
CMake build dirs (a repo top-level dir with a CMakeCache.txt). It accepts
the [elf-exempt] paths (git metadata and the list file itself; each one
has its reason in the list file). It prints an
info line when the build used a Pico SDK outside the repo. That SDK is not
in [elf], but the bench key hashes it (path, commit, dirty state and
submodules; scripts/firmware_tree.py).

Exit 0: every used repo file is in [elf]. Exit 1: a used file is not in
[elf] (add its path to [elf]), or a query failed (fail closed).
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
from collections import Counter
from pathlib import Path
from typing import Callable, Dict, Iterable, List, Optional, Sequence, Set

_SCRIPTS = Path(__file__).resolve().parents[1]
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
import firmware_tree as ft  # noqa: E402

ELF_TARGET = 'rocketchip.elf'
SOURCES = ('deps', 'compdb', 'inputs', 'regen')


class CheckError(RuntimeError):
    """A query failed or gave nothing. The check fails closed."""


# ---------------------------------------------------------------------------
# Parsers (pure; host-tested with canned output)
# ---------------------------------------------------------------------------

def parse_deps(text: str) -> List[str]:
    """`ninja -t deps` output: '<target>: #deps N, ...' then indented paths."""
    out: List[str] = []
    for line in text.splitlines():
        if line.startswith((' ', '\t')) and line.strip():
            out.append(line.strip())
    return out


def parse_inputs(text: str) -> List[str]:
    """`ninja -t inputs` output: one path per line."""
    return [ln.strip() for ln in text.splitlines() if ln.strip()]


def parse_query_inputs(text: str) -> List[str]:
    """`ninja -t query <target>`: the paths under "input:" (explicit, '|'
    implicit and '||' order-only), up to "outputs:"."""
    out: List[str] = []
    in_inputs = False
    for line in text.splitlines():
        s = line.strip()
        if s.startswith('input:'):
            in_inputs = True
            continue
        if s.startswith('outputs:') or s.startswith('validations:'):
            in_inputs = False
            continue
        if in_inputs and s:
            out.append(s.lstrip('|').strip())
    return out


def parse_compdb(text: str) -> List[str]:
    """compile_commands.json: absolute path of each "file"."""
    out: List[str] = []
    for entry in json.loads(text):
        f = entry.get('file', '')
        if f:
            out.append(os.path.join(entry.get('directory', ''), f))
    return out


# ---------------------------------------------------------------------------
# Queries (real build dir)
# ---------------------------------------------------------------------------

def cache_value(bdir: Path, name: str) -> Optional[str]:
    try:
        text = (bdir / 'CMakeCache.txt').read_text(encoding='utf-8', errors='replace')
    except OSError:
        return None
    prefix = name + ':'
    for line in text.splitlines():
        if line.startswith(prefix) and '=' in line:
            return line.split('=', 1)[1].strip() or None
    return None


def ninja_exe(bdir: Path) -> str:
    gen = cache_value(bdir, 'CMAKE_GENERATOR')
    if gen is None:
        raise CheckError(f'{bdir}: no CMakeCache.txt (configure it first)')
    if 'Ninja' not in gen:
        raise CheckError(f'{bdir}: generator is "{gen}"; this check needs Ninja')
    return cache_value(bdir, 'CMAKE_MAKE_PROGRAM') or 'ninja'


def run_ninja(bdir: Path, *args: str) -> str:
    cmd = [ninja_exe(bdir), '-C', str(bdir), *args]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, check=False)
    except OSError as exc:
        raise CheckError(f'cannot run {cmd[0]}: {exc}')
    if r.returncode != 0:
        raise CheckError(f'{" ".join(cmd)} failed (exit {r.returncode}): '
                         f'{(r.stderr or r.stdout).strip()[:300]}'
                         + ('  [-t inputs needs ninja 1.11 or newer]'
                            if 'inputs' in args else ''))
    return r.stdout


def query_build(bdir: Path, target: str = ELF_TARGET) -> Dict[str, List[str]]:
    """{source: [raw paths]} for one build dir. Raw paths are as the tool
    printed them (relative to bdir or absolute)."""
    compdb = bdir / 'compile_commands.json'
    if not compdb.is_file():
        raise CheckError(f'{compdb} missing (CMAKE_EXPORT_COMPILE_COMMANDS)')
    return {
        'deps': parse_deps(run_ninja(bdir, '-t', 'deps')),
        'compdb': parse_compdb(compdb.read_text(encoding='utf-8')),
        'inputs': parse_inputs(run_ninja(bdir, '-t', 'inputs', target)),
        'regen': parse_query_inputs(run_ninja(bdir, '-t', 'query', 'build.ninja')),
    }


# ---------------------------------------------------------------------------
# Classification
# ---------------------------------------------------------------------------

def _norm(p: str) -> str:
    return os.path.normcase(os.path.realpath(p))


class Result:
    def __init__(self) -> None:
        self.counts: Dict[str, Counter] = {}
        self.uncovered: Dict[str, Set[str]] = {}   # repo path -> sources
        self.external_roots: Counter = Counter()
        self.external_sdk: Set[str] = set()

    @property
    def ok(self) -> bool:
        return not self.uncovered


def classify(repo: Path, bdir: Path, raw: Dict[str, List[str]],
             elf: Sequence[str], res: Result, exempt: Sequence[str] = (),
             exists: Callable[[str], bool] = os.path.exists) -> None:
    root = _norm(str(repo))
    bnorm = _norm(str(bdir))
    build_tops = {_norm(str(d)) for d in Path(repo).iterdir()
                  if d.is_dir() and (d / 'CMakeCache.txt').is_file()} | {bnorm}
    nt = os.name == 'nt'
    elf_cmp = [e.lower() for e in elf] if nt else list(elf)
    exempt_cmp = [e.lower() for e in exempt] if nt else list(exempt)
    for src in SOURCES:
        c = res.counts.setdefault(f'{bdir.name}:{src}', Counter())
        seen: Set[str] = set()
        for p in raw.get(src, []):
            p = p.replace('\\', '/')
            full = _norm(p if os.path.isabs(p) else os.path.join(str(bdir), p))
            if full in seen:
                continue
            seen.add(full)
            c['unique'] += 1
            if not exists(full):
                c['missing'] += 1
                continue
            if any(full == b or full.startswith(b + os.sep) for b in build_tops):
                c['generated'] += 1
                continue
            if not (full == root or full.startswith(root + os.sep)):
                c['external'] += 1
                parts = Path(full).parts
                res.external_roots['/'.join(parts[:4])] += 1
                low = full.replace('\\', '/').lower()
                if '/.pico-sdk/sdk/' in low:
                    # <...>/.pico-sdk/sdk/<version>, forward slashes.
                    i = low.index('/.pico-sdk/sdk/') + len('/.pico-sdk/sdk/')
                    j = low.find('/', i)
                    res.external_sdk.add(full.replace('\\', '/')[:j if j > 0 else None])
                continue
            rel = os.path.relpath(full, root).replace('\\', '/')
            c['repo'] += 1
            key = rel.lower() if nt else rel
            if any(ft.entry_matches(key, e) for e in elf_cmp):
                c['covered'] += 1
            elif any(ft.entry_matches(key, e) for e in exempt_cmp):
                c['exempt'] += 1
            else:
                c['uncovered'] += 1
                res.uncovered.setdefault(rel, set()).add(f'{bdir.name}:{src}')


def check(repo: Path, bdirs: Iterable[Path],
          query: Callable[[Path], Dict[str, List[str]]] = query_build) -> Result:
    sections = ft.load(repo)
    elf, exempt = sections['elf'], sections.get('elf-exempt', [])
    res = Result()
    for bdir in bdirs:
        raw = query(bdir)
        if not raw.get('deps'):
            raise CheckError(f'{bdir}: ninja -t deps recorded nothing (build it first)')
        for src in SOURCES:
            if not raw.get(src):
                raise CheckError(f'{bdir}: {src} query returned nothing')
        classify(repo, bdir, raw, elf, res, exempt)
    return res


def report(res: Result) -> str:
    lines = []
    for key, c in res.counts.items():
        lines.append(f'  {key:<28} unique {c["unique"]:>5}  repo {c["repo"]:>5}  '
                     f'covered {c["covered"]:>5}  exempt {c["exempt"]:>2}  '
                     f'uncovered {c["uncovered"]:>3}  '
                     f'generated {c["generated"]:>4}  external {c["external"]:>5}  '
                     f'missing {c["missing"]:>3}')
    if res.external_sdk:
        lines.append('  INFO: the build used a Pico SDK outside the repo: '
                     + ', '.join(sorted(res.external_sdk)) + '. The bench key '
                     'hashes it (scripts/firmware_tree.py, pico-sdk-* lines).')
    for rel in sorted(res.uncovered):
        lines.append(f'  NOT IN [elf]: {rel}   (used by {", ".join(sorted(res.uncovered[rel]))})')
    return '\n'.join(lines)


def main(argv: List[str]) -> int:
    args = argv[1:]
    repo = Path(subprocess.run(['git', 'rev-parse', '--show-toplevel'],
                               capture_output=True, text=True).stdout.strip() or '.')
    if not args:
        args = ['build_flight', 'build_station_flight']
    bdirs = [Path(a) if os.path.isabs(a) else repo / a for a in args]
    try:
        res = check(repo, bdirs)
    except (CheckError, ft.FirmwareTreeError, ValueError) as exc:
        print(f'check_elf_paths: ERROR: {exc}')
        return 1
    print('check_elf_paths: files the flight build used, by source:')
    print(report(res))
    if not res.ok:
        print('check_elf_paths: FAIL. A repo file the build used is not in [elf] of\n'
              f'  {ft.LIST_REL}. The build ID and the bench key do not cover it.\n'
              '  Add its path to [elf], stage the list, and commit again.')
        return 1
    print('check_elf_paths: OK (every repo file the build used is in [elf])')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
