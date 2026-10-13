#!/usr/bin/env python3
"""Classify changed paths for the commit build and the push bench.

Path lists live in ONE tracked file: ``scripts/ci/firmware_paths.txt``
(read through ``scripts/firmware_tree.py``). ``cmake/rc_version.cmake``
reads the same file for the build ID (``kFirmwareTreeId``). Do not copy
the lists into code again (LL 40 dual-hardcode defect).

Emitted for ``eval`` in ``scripts/hooks/pre-commit`` (staged paths):

  TRIGGER_TARGET_BUILD=0|1   # flight target cross-compile, both roles
  TRIGGER_FLIGHT_BENCH=0|1   # informational at commit; bench runs at push
  TRIGGER_STATION_BENCH=0|1  # informational at commit; bench runs at push

``scripts/ci/pre_push_gate.py`` imports ``match()`` and ``radio()`` and
applies them to every path in the commits being pushed.

POLICY: "categories not enumerations" (council 2026-05-16, unanimous).
Any change that can produce a different flight image triggers the bench.
Lived cases: LL Entry 36 (bench_sim regex rot, 2026-04-11) and LL Entry 39
(8adab2d touched src/main.cpp + rc_log.h, outside the old enumeration, the
hook skipped bench_sim, Core 1 IMU reads broke). Precedent: R-25-exec
(docs/decisions/BENCH_TIER_DEPRECATION_2026-05-13.md).

Where the bench runs (2026-10-08 draft): ONCE PER PUSH, in the pre-push
gate, on the commit being pushed. Pre-commit stays fast (stdio ban,
clang-tidy, host ctest, flight target cross-compile).

Roles (Nathan, 2026-10-08): both roles build from one source list, so a
firmware ([elf]) change benches BOTH roles (vehicle and station). One
exception stays (2026-09-17): when every firmware path is in
[station-only], only the station bench runs. A [gate] change benches BOTH
roles. A radio push ([radio]) benches BOTH roles and the radio link: the
link has two ends.

Adding paths: any path that produces firmware bytes goes in [elf]; gate
machinery goes in [gate]. Removing paths: not without a council.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

_SCRIPTS = Path(__file__).resolve().parents[1]
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))
import firmware_tree  # noqa: E402

_SECTIONS: Optional[Dict[str, List[str]]] = None


def _sections() -> Dict[str, List[str]]:
    global _SECTIONS
    if _SECTIONS is None:
        _SECTIONS = firmware_tree.parse(
            (_SCRIPTS / 'ci' / 'firmware_paths.txt').read_text(encoding='utf-8'))
    return _SECTIONS


def _repo_root() -> str:
    return subprocess.check_output(
        ['git', 'rev-parse', '--show-toplevel'], text=True).strip()


def _git_staged_paths() -> list[str]:
    root = _repo_root()
    r = subprocess.run(
        ['git', 'diff', '--cached', '--name-only', '--diff-filter=ACMRD'],
        capture_output=True, text=True, cwd=root, check=False,
    )
    if r.returncode != 0:
        return []
    return [ln for ln in r.stdout.splitlines() if ln.strip()]


def is_elf(path: str, sections: Optional[Dict[str, List[str]]] = None) -> bool:
    return firmware_tree.in_section(path, sections or _sections(), 'elf')


def is_firmware(path: str, sections: Optional[Dict[str, List[str]]] = None) -> bool:
    s = sections or _sections()
    return firmware_tree.in_section(path, s, 'elf') or \
        firmware_tree.in_section(path, s, 'gate')


def radio(paths: List[str], sections: Optional[Dict[str, List[str]]] = None) -> bool:
    s = sections or _sections()
    return any(firmware_tree.in_section(p, s, 'radio') for p in paths)


def target_build(paths: List[str],
                 sections: Optional[Dict[str, List[str]]] = None) -> bool:
    s = sections or _sections()
    return any(is_elf(p, s) for p in paths)


def match(paths: List[str],
          sections: Optional[Dict[str, List[str]]] = None) -> Tuple[bool, bool]:
    """Return (vehicle bench needed, station bench needed).

    No [elf] or [gate] path: no bench. A [radio] or [gate] path: both
    roles. Every firmware path in [station-only]: station only. Any other
    firmware change: both roles.
    """
    s = sections or _sections()
    firmware = [p for p in paths if is_firmware(p, s)]
    if not firmware:
        return False, False
    if radio(firmware, s):
        return True, True
    if any(firmware_tree.in_section(p, s, 'gate') for p in firmware):
        return True, True
    if all(firmware_tree.in_section(p, s, 'station-only') for p in firmware):
        return False, True
    return True, True


def main() -> int:
    paths = _git_staged_paths()
    f, s = match(paths)
    print(f'TRIGGER_TARGET_BUILD={1 if target_build(paths) else 0}')
    print(f'TRIGGER_FLIGHT_BENCH={1 if f else 0}')
    print(f'TRIGGER_STATION_BENCH={1 if s else 0}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
