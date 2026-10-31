#!/usr/bin/env python3
# Fail when a file under src/ or include/ includes hardware/ or pico/.
# The optional argument is the folder that holds those two directories.
# No state.
# Book: none.

from __future__ import annotations

import re
import sys
from pathlib import Path

# #include, then any whitespace, then "..." or <...>.
# The path must start with hardware/ or pico/.
_INCLUDE_RE = re.compile(r'^[ \t]*#[ \t]*include[ \t]*[<"](?:hardware/|pico/)')

_SUFFIXES = (".cpp", ".h", ".h.in")


def _is_listed(path: Path) -> bool:
    return path.name.endswith(_SUFFIXES)


def _scan_file(path: Path, root: Path) -> list[str]:
    hits: list[str] = []
    rel = path.relative_to(root).as_posix()
    text = path.read_text(encoding="utf-8-sig", errors="replace")
    for number, line in enumerate(text.splitlines(), start=1):
        if _INCLUDE_RE.match(line):
            hits.append(f"{rel}:{number}")
    return hits


def _root_from_args(argv: list[str]) -> Path | None:
    if len(argv) > 2:
        print("usage: check_core_includes.py [root]", file=sys.stderr)
        return None
    if len(argv) == 2:
        return Path(argv[1]).resolve()
    return Path(__file__).resolve().parent.parent


def main(argv: list[str]) -> int:
    root = _root_from_args(argv)
    if root is None:
        return 1
    missing = False
    scanned = 0
    hits: list[str] = []
    for folder in ("src", "include"):
        base = root / folder
        if not base.is_dir():
            print(f"{base.as_posix()}: missing directory")
            missing = True
            continue
        for path in sorted(base.rglob("*")):
            if path.is_file() and _is_listed(path):
                scanned += 1
                hits.extend(_scan_file(path, root))
    if missing:
        return 1
    if scanned == 0:
        print(f"{root.as_posix()}: no source files scanned")
        return 1
    if hits:
        for hit in hits:
            print(hit)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
