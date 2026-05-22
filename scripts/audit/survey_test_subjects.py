#!/usr/bin/env python3
"""Survey host tests for subjects not in firmware (companion to find_dead_code.py).

Finds tests where the code-under-test is NOT compiled into the firmware binary.
These are "tests for code that doesn't exist" — they pass but protect nothing.

Per project policy 2026-05-22 (and user direction: ballooning host-test count is
a concern). Companion to find_dead_code.py's Class 4a check.
"""

from __future__ import annotations

import os
import re
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
CMAKE = REPO_ROOT / "CMakeLists.txt"
TEST_DIR = REPO_ROOT / "test"

STDLIB_PREFIXES = (
    "gtest", "pico", "hardware", "etl", "stdio", "stdlib", "string",
    "cmath", "math", "cstdint", "cstring", "cstdio", "cstdlib",
    "algorithm", "array", "vector", "memory", "functional", "utility",
    "limits", "type_traits", "tuple", "cassert", "assert", "inttypes",
    "stdbool", "stdarg", "stddef", "iostream", "fstream", "sstream",
    "iomanip", "complex", "numeric", "random", "chrono", "thread",
    "atomic", "mutex", "future", "condition_variable", "regex",
    "stdexcept", "exception", "system_error", "new",
)


def parse_add_executable(cmake_path: Path) -> set[str]:
    lines = cmake_path.read_text(encoding="utf-8").splitlines()
    in_block = False
    depth = 0
    files = set()
    for line in lines:
        if not in_block and re.match(r"\s*add_executable\s*\(\s*rocketchip\b", line):
            in_block = True
            depth = 0
        if in_block:
            depth += line.count("(") - line.count(")")
            stripped = line.split("#", 1)[0].strip()
            if stripped.startswith("src/") and stripped.endswith(".cpp"):
                files.add(stripped)
            if depth == 0 and "add_executable" not in line:
                break
    return files


def index_src_cpps() -> dict[str, list[str]]:
    """Map basename ->list of relative paths under src/."""
    index = {}
    src = REPO_ROOT / "src"
    for path in src.rglob("*.cpp"):
        rel = path.relative_to(REPO_ROOT).as_posix()
        index.setdefault(path.name, []).append(rel)
    return index


def extract_project_includes(test_path: Path) -> list[str]:
    text = test_path.read_text(encoding="utf-8")
    incs = re.findall(r'#include\s+["<]([^">]+\.h)[">]', text)
    out = []
    for inc in incs:
        first = inc.split("/")[0].lower()
        if first in STDLIB_PREFIXES:
            continue
        if first.endswith(".h") and first.split(".")[0] in STDLIB_PREFIXES:
            continue
        out.append(inc)
    return out


def candidate_cpps_for_include(inc: str, src_index: dict[str, list[str]]) -> list[str]:
    """Given an include like 'fusion/baro_kf.h' or 'rocketchip/rc_log.h',
    return possible .cpp paths under src/."""
    h = inc.replace("rocketchip/", "")
    base = Path(h).name
    if not base.endswith(".h"):
        return []
    cpp_name = base[:-2] + ".cpp"
    return src_index.get(cpp_name, [])


def main() -> int:
    add_exec = parse_add_executable(CMAKE)
    src_index = index_src_cpps()

    orphan_tests = []
    for test_file in sorted(TEST_DIR.glob("test_*.cpp")):
        incs = extract_project_includes(test_file)
        if not incs:
            continue
        candidates_by_include = []
        any_in_firmware = False
        for inc in incs:
            cands = candidate_cpps_for_include(inc, src_index)
            in_fw = any(c in add_exec for c in cands)
            candidates_by_include.append((inc, cands, in_fw))
            if in_fw:
                any_in_firmware = True
        if not any_in_firmware:
            orphan_tests.append((test_file.name, candidates_by_include))

    if not orphan_tests:
        print("No tests found with all subjects out of firmware.")
        return 0

    print(f"# Host-test orphan-subject survey\n")
    print(f"Found {len(orphan_tests)} tests where NO `#include` resolves to a "
          f"`.cpp` in `add_executable(rocketchip)`.\n")
    print(
        "**Caveats:** the resolver maps `#include \"foo/bar.h\"` to "
        "`src/.../bar.cpp` via basename. **High false-positive rate** for:\n"
        "- **Header-only modules**: a `.h` with no paired `.cpp` is normal "
        "(constants, enums, pure inline helpers, templates).\n"
        "- **Headers used transitively by production code**: if the header is "
        "`#include`d from another `.cpp` that IS in firmware, the test is "
        "legitimate even though the resolver can't trace the chain.\n"
        "- **Vendored library tests** (qp/c, mavlink): testing imported code, "
        "not project code.\n\n"
        "Hand-verify each finding by grepping `src/` for the header: "
        "`grep -rln 'include.*<name>\\.h' src/`. If any production `.cpp` "
        "uses the header, the test is legitimate. Only delete when:\n"
        "1. The header has a paired `.cpp` (i.e., not header-only), AND\n"
        "2. That `.cpp` is not in `add_executable(rocketchip)`, AND\n"
        "3. No production `.cpp` in firmware `#include`s the header.\n\n"
        "All three conditions need to be checked by hand for each finding.\n"
    )
    for fname, cands in orphan_tests:
        print(f"## `test/{fname}`\n")
        for inc, candidates, _ in cands:
            if candidates:
                marks = ", ".join(candidates)
                print(f"- `#include \"{inc}\"` ->candidate `.cpp`: {marks} (NOT in firmware)")
            else:
                print(f"- `#include \"{inc}\"` ->no `.cpp` found under `src/` (header-only or vendored)")
        print()
    return 1


if __name__ == "__main__":
    sys.exit(main())
