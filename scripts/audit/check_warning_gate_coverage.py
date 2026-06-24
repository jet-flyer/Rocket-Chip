#!/usr/bin/env python3
"""Warning-gate coverage check (L2-P5, 2026-06-24).

Asserts that EVERY RocketChip-authored ``src/`` translation unit's *actual*
compile command carries the per-file warning-gate flags. This is the mechanical
defense against the failure mode in ``LESSONS_LEARNED.md`` Entry 43:

A per-file ``set_source_files_properties(<f> PROPERTIES COMPILE_OPTIONS ...)``
**replaces** (does not append) the options set for ``${ROCKETCHIP_SOURCES}``.
An override that forgets to re-list a flag silently drops that file out of the
gate, and "0 warnings" then looks identical to a genuinely-clean tree. That is
exactly how ``eskf.cpp`` + ``eskf_codegen.cpp`` sat outside ``-Wshadow`` /
``-Wfloat-equal`` for as long as their ``-O2`` override existed.

Unlike the ``ROCKETCHIP_SOURCES``-membership check in ``BUILD_SYSTEM_AUDIT.md``,
this reads the real command out of ``compile_commands.json`` -- so it catches a
dropped flag on a file that IS in the source list (which membership checks miss).

Usage:
    python scripts/audit/check_warning_gate_coverage.py [compile_commands.json]

Default DB: build/compile_commands.json  (configure: cmake -B build -G Ninja).
Exit 0 = every authored src/ file gated; 1 = holes (listed); 2 = no DB (skip).
"""
import json
import os
import sys

# Per-file flags applied to ${ROCKETCHIP_SOURCES} at CMakeLists.txt:598, plus the
# global -Werror that makes them fatal. These are the flags a per-file
# COMPILE_OPTIONS override is at risk of silently dropping.
REQUIRED = ["-Werror", "-Wpedantic", "-Wshadow", "-Wfloat-equal"]

# Authored tree marker. Vendored code (Pico SDK, TinyUSB, lwGPS, ETL, ...) lives
# under .pico-sdk / EXTERNAL / lib and is adopted code, intentionally NOT held to
# our warning gate -- so it must be excluded, not reported as a hole.
_REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
_MARKER = os.path.basename(_REPO) + "/src/"  # e.g. "Rocket-Chip/src/"


def main() -> int:
    db_path = sys.argv[1] if len(sys.argv) > 1 else "build/compile_commands.json"
    if not os.path.isfile(db_path):
        print(f"[warning-gate] no {db_path} -- configure the firmware build first "
              f"(cmake -B build -G Ninja). SKIPPED.")
        return 2

    with open(db_path) as fh:
        db = json.load(fh)

    holes = []
    checked = 0
    for entry in db:
        path = entry["file"].replace("\\", "/")
        if _MARKER not in path:
            continue
        checked += 1
        cmd = entry.get("command", "") or " ".join(entry.get("arguments", []))
        missing = [flag for flag in REQUIRED if flag not in cmd]
        if missing:
            holes.append((path.split(_MARKER, 1)[1], missing))

    if holes:
        print(f"[warning-gate] FAIL: {len(holes)} authored src/ file(s) outside the "
              f"warning gate (LESSONS_LEARNED.md Entry 43):")
        for name, missing in sorted(holes):
            print(f"    src/{name}: missing {', '.join(missing)}")
        print("Fix: that file's set_source_files_properties(COMPILE_OPTIONS) override "
              "must re-carry EVERY base flag -- it replaces, it does not append.")
        return 1

    print(f"[warning-gate] OK: all {checked} authored src/ files carry "
          f"{', '.join(REQUIRED)}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
