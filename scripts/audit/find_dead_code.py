#!/usr/bin/env python3
"""Dead-code inventory tool.

Per `standards/CODING_STANDARDS.md` "Scratch Tools and Dead-Code Discipline"
(project policy 2026-05-22, four-persona council unanimous).

The project's CMakeLists.txt has TWO source lists that must remain in sync:

  - `add_executable(rocketchip ...)` — authoritative "what gets built"
  - `set(ROCKETCHIP_SOURCES ...)` — "what gets the -Wpedantic flag applied"

Drift between them is a known failure mode (the comment block at the
ROCKETCHIP_SOURCES definition warns about it explicitly). This tool surfaces:

  - Class 1: `src/**/*.cpp` files not in EITHER list (never compiled).
  - Class 2: Files in `add_executable` whose per-TU object emits zero
    externally-visible symbols (R-5 Unit D `telemetry_service.cpp` failure mode).
  - Class 3: Stale `.cpp.obj` files in build dir whose source no longer exists.
  - Class 4: Drift between `add_executable` and `ROCKETCHIP_SOURCES`.

Scope (out, per council):
  - AST dead-branch detection.
  - Dead CLI command detection.
  - Dead global variable detection.
  - Auto-deletion.

Usage:
    python scripts/audit/find_dead_code.py
    python scripts/audit/find_dead_code.py --build-dir build_station_flight
    python scripts/audit/find_dead_code.py --output docs/audits/DEAD_CODE_INVENTORY_<date>.md
"""

from __future__ import annotations

import argparse
import datetime
import re
import subprocess
import sys
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_CMAKE = REPO_ROOT / "CMakeLists.txt"
DEFAULT_BUILD_DIR = REPO_ROOT / "build_flight"
DEFAULT_NM = (
    "C:/Users/pow-w/.pico-sdk/toolchain/14_2_Rel1/bin/arm-none-eabi-nm.exe"
)


PROTECTED_DOCS = [
    "docs/SAD.md",
    "docs/SCAFFOLDING.md",
    "docs/AO_ARCHITECTURE.md",
    "docs/MULTICORE_RULES.md",
    "standards/CODING_STANDARDS.md",
    "README.md",
    "docs/PROJECT_STATUS.md",
]


def _parse_paren_block(lines: list[str], start_idx: int) -> list[str]:
    """Walk forward from start_idx until paren-balanced close, return cpp paths.

    start_idx is the line containing the opening `(`. Returns `src/...cpp` lines
    found inside (after stripping whitespace and `#` comments), in order.
    """
    depth = 0
    out = []
    for i in range(start_idx, len(lines)):
        line = lines[i]
        for ch in line:
            if ch == "(":
                depth += 1
            elif ch == ")":
                depth -= 1
        if depth == 0 and i > start_idx:
            return out
        stripped = line.split("#", 1)[0].strip()
        if stripped.startswith("src/") and stripped.endswith(".cpp"):
            out.append(stripped)
    return out


def parse_add_executable_sources(cmake_path: Path) -> list[str]:
    """Find `add_executable(rocketchip ...)` and extract its src/*.cpp entries."""
    lines = cmake_path.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        if re.match(r"\s*add_executable\s*\(\s*rocketchip\b", line):
            return _parse_paren_block(lines, i)
    raise RuntimeError("Could not find add_executable(rocketchip ...) block")


def parse_rocketchip_sources(cmake_path: Path) -> list[str]:
    """Find `set(ROCKETCHIP_SOURCES ...)` and extract its src/*.cpp entries."""
    lines = cmake_path.read_text(encoding="utf-8").splitlines()
    for i, line in enumerate(lines):
        if re.match(r"\s*set\s*\(\s*ROCKETCHIP_SOURCES\b", line):
            return _parse_paren_block(lines, i)
    raise RuntimeError("Could not find set(ROCKETCHIP_SOURCES ...) block")


def glob_src_cpps(src_root: Path) -> list[str]:
    out = []
    for path in src_root.rglob("*.cpp"):
        rel = path.relative_to(REPO_ROOT).as_posix()
        out.append(rel)
    return sorted(out)


def obj_path_for_source(build_dir: Path, src_rel: str) -> Path:
    return build_dir / "CMakeFiles" / "rocketchip.dir" / f"{src_rel}.obj"


def count_external_symbols(nm_path: str, obj_path: Path) -> int:
    try:
        result = subprocess.run(
            [nm_path, "--defined-only", "--extern-only", str(obj_path)],
            capture_output=True,
            text=True,
            check=True,
            timeout=10,
        )
    except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired) as exc:
        raise RuntimeError(f"nm failed on {obj_path}: {exc}") from exc
    lines = [ln for ln in result.stdout.splitlines() if ln.strip()]
    return len(lines)


def glob_stale_object_files(build_dir: Path) -> list[tuple[str, str]]:
    obj_root = build_dir / "CMakeFiles" / "rocketchip.dir" / "src"
    if not obj_root.exists():
        return []
    stale = []
    for obj in obj_root.rglob("*.cpp.obj"):
        rel_from_rocketchip_dir = obj.relative_to(
            build_dir / "CMakeFiles" / "rocketchip.dir"
        )
        expected_src = REPO_ROOT / str(rel_from_rocketchip_dir)[:-4]
        if not expected_src.exists():
            obj_rel = obj.relative_to(REPO_ROOT).as_posix()
            src_rel = expected_src.relative_to(REPO_ROOT).as_posix()
            stale.append((obj_rel, src_rel))
    return sorted(stale)


def grep_command_for_doc_walk(symbol_or_file: str) -> str:
    targets = " ".join(PROTECTED_DOCS)
    return f'grep -n "{symbol_or_file}" {targets}'


def render_report(
    build_dir: Path,
    add_executable_sources: list[str],
    rocketchip_sources: list[str],
    all_src_cpps: list[str],
    unbuilt: list[str],
    dce: list[str],
    stale_objs: list[tuple[str, str]],
    drift_in_pedantic_only: list[str],
    drift_in_executable_only: list[str],
) -> str:
    today = datetime.date.today().isoformat()
    lines: list[str] = []
    lines.append(f"# Dead-Code Inventory ({today})")
    lines.append("")
    lines.append(
        f"**Tool:** `scripts/audit/find_dead_code.py`  "
        f"**Build dir analyzed:** `{build_dir.relative_to(REPO_ROOT).as_posix()}`"
    )
    lines.append(
        f"**add_executable count:** {len(add_executable_sources)}  "
        f"**ROCKETCHIP_SOURCES count:** {len(rocketchip_sources)}  "
        f"**src/**/*.cpp count:** {len(all_src_cpps)}"
    )
    lines.append("")
    lines.append("## Methodology")
    lines.append("")
    lines.append("Per `standards/CODING_STANDARDS.md` \"Scratch Tools and Dead-Code Discipline\" (2026-05-22, four-persona council unanimous):")
    lines.append("")
    lines.append("- **Class 1 (unbuilt orphans):** `src/**/*.cpp` files not in EITHER source list. Never compiled.")
    lines.append("- **Class 2 (DCE candidates):** Files in `add_executable` whose per-TU object emits zero externally-visible symbols. R-5 Unit D failure mode.")
    lines.append("- **Class 3 (stale build cache):** `.cpp.obj` files in build dir whose source no longer exists.")
    lines.append("- **Class 4 (CMake source-list drift):** Files in `ROCKETCHIP_SOURCES` (pedantic-gated) but NOT in `add_executable` (compiled) — silent missing-build. Or vice versa: in `add_executable` but escaping the pedantic gate. The CMakeLists.txt block at the ROCKETCHIP_SOURCES definition warns about this drift explicitly.")
    lines.append("")
    lines.append("**Caveats:**")
    lines.append("")
    lines.append("- Class 2 false-positive: a TU consisting entirely of `static` definitions referenced internally emits zero external symbols by design. Verify by hand-reading before deleting.")
    lines.append("- Class 2 false-negative: link-time dead-stripping is not modeled (build has no LTO; cross-TU DCE is limited to per-TU static extraction).")
    lines.append("- Out of scope: AST dead-branch, dead CLI command, dead global variable detection.")
    lines.append("")

    if (
        not unbuilt
        and not dce
        and not stale_objs
        and not drift_in_pedantic_only
        and not drift_in_executable_only
    ):
        lines.append("## Findings")
        lines.append("")
        lines.append("**No findings.** The tree is clean against the current build.")
        lines.append("")
        return "\n".join(lines)

    lines.append("## Findings")
    lines.append("")

    if unbuilt:
        lines.append(f"### Class 1: Unbuilt orphans ({len(unbuilt)})")
        lines.append("")
        lines.append("These `.cpp` files exist in `src/` but are not in either source list. They are never compiled into either firmware binary.")
        lines.append("")
        for f in unbuilt:
            lines.append(f"- `{f}`")
            basename_no_ext = Path(f).stem
            lines.append(f"  - Doc-walk: `{grep_command_for_doc_walk(basename_no_ext)}`")
        lines.append("")

    if drift_in_pedantic_only:
        lines.append(
            f"### Class 4a: In ROCKETCHIP_SOURCES (pedantic-gated) but NOT in add_executable (built) ({len(drift_in_pedantic_only)})"
        )
        lines.append("")
        lines.append("These files are listed for the pedantic gate but the build never compiles them. Either the file should be added to `add_executable(rocketchip ...)`, or removed from `ROCKETCHIP_SOURCES` and deleted from `src/`.")
        lines.append("")
        for f in drift_in_pedantic_only:
            lines.append(f"- `{f}`")
            basename_no_ext = Path(f).stem
            lines.append(f"  - Doc-walk: `{grep_command_for_doc_walk(basename_no_ext)}`")
        lines.append("")

    if drift_in_executable_only:
        lines.append(
            f"### Class 4b: In add_executable (built) but NOT in ROCKETCHIP_SOURCES (pedantic-gated) ({len(drift_in_executable_only)})"
        )
        lines.append("")
        lines.append("These files compile into the binary but escape the `-Wpedantic` gate. Add to `ROCKETCHIP_SOURCES` to restore standards coverage.")
        lines.append("")
        for f in drift_in_executable_only:
            lines.append(f"- `{f}`")
        lines.append("")

    if dce:
        lines.append(f"### Class 2: DCE candidates ({len(dce)})")
        lines.append("")
        lines.append("Files in `add_executable` whose per-TU object emits zero externally-visible symbols. Hand-read each one — confirm there is no `static` machinery referenced from a paired header before treating as dead.")
        lines.append("")
        for f in dce:
            lines.append(f"- `{f}`")
            basename_no_ext = Path(f).stem
            lines.append(f"  - Doc-walk: `{grep_command_for_doc_walk(basename_no_ext)}`")
        lines.append("")

    if stale_objs:
        lines.append(f"### Class 3: Stale build cache ({len(stale_objs)})")
        lines.append("")
        lines.append("Compiled object files in the build directory whose source no longer exists. Not a tree-dead-code finding (the source IS gone), but signals previous migrations that didn't clean the build cache.")
        lines.append("")
        for obj_rel, src_rel in stale_objs:
            lines.append(f"- `{obj_rel}` (expected source `{src_rel}` missing)")
        lines.append("")
        bd = build_dir.relative_to(REPO_ROOT).as_posix()
        lines.append(f"Resolution: `cmake --build {bd} --target clean && cmake --build {bd}`.")
        lines.append("")

    lines.append("## Operator next steps")
    lines.append("")
    lines.append("For each Class 1 / 2 / 4 finding:")
    lines.append("")
    lines.append("1. **Trace history before acting** (policy rule, added 2026-05-22 after the `BaroKF` case). For each finding, run `git log --all --oneline -- <file>` and `grep -ni <symbol> CHANGELOG.md`. If history shows a deliberate intermediate state (e.g., kept for host tests after firmware removal), the right action is usually to restore the intended state by fixing the drifted metadata, NOT to delete.")
    lines.append("2. Run the doc-walk grep above for the finding. If matches are found, the deletion commit MUST update those docs in the same commit (per `SESSION_CHECKLIST.md` trigger-driven edit rule).")
    lines.append("3. Read the file. Confirm the apparent deadness — look for `static` machinery referenced from paired headers, ODR-shared template instantiations, friend-class arrangements, anything that could be load-bearing without an external symbol.")
    lines.append("4. If confirmed dead: delete + update docs + update `CMakeLists.txt`.")
    lines.append("5. If confirmed alive but flagged (e.g., paired-header `static` arrangement, deliberately-kept host-test target): document in the commit closing the finding.")
    lines.append("")
    lines.append("Per the policy: this report is advisory. No auto-deletion.")
    lines.append("")

    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description="Dead-code inventory tool.")
    parser.add_argument("--cmake", type=Path, default=DEFAULT_CMAKE)
    parser.add_argument("--build-dir", type=Path, default=DEFAULT_BUILD_DIR)
    parser.add_argument("--nm", type=str, default=DEFAULT_NM)
    parser.add_argument("--output", type=Path, default=None)
    args = parser.parse_args()

    if not args.cmake.exists():
        print(f"ERROR: {args.cmake} not found", file=sys.stderr)
        return 2
    if not args.build_dir.exists():
        print(f"ERROR: {args.build_dir} not found. Build first.", file=sys.stderr)
        return 2

    try:
        add_executable_sources = parse_add_executable_sources(args.cmake)
        rocketchip_sources = parse_rocketchip_sources(args.cmake)
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    src_root = REPO_ROOT / "src"
    all_src_cpps = glob_src_cpps(src_root)

    add_set = set(add_executable_sources)
    rs_set = set(rocketchip_sources)
    cpp_set = set(all_src_cpps)

    # Class 1: in src/ but in neither cmake list
    unbuilt = sorted([f for f in cpp_set if f not in add_set and f not in rs_set])

    # Class 4a: in ROCKETCHIP_SOURCES but not add_executable
    drift_in_pedantic_only = sorted([f for f in rs_set if f not in add_set])

    # Class 4b: in add_executable but not ROCKETCHIP_SOURCES
    drift_in_executable_only = sorted([f for f in add_set if f not in rs_set])

    # Class 2: per-TU external-symbol count == 0
    dce = []
    for src_rel in add_executable_sources:
        obj = obj_path_for_source(args.build_dir, src_rel)
        if not obj.exists():
            print(
                f"WARNING: object file missing for {src_rel} (build incomplete?)",
                file=sys.stderr,
            )
            continue
        try:
            n = count_external_symbols(args.nm, obj)
        except RuntimeError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2
        if n == 0:
            dce.append(src_rel)

    # Class 3: stale build artifacts
    stale_objs = glob_stale_object_files(args.build_dir)

    report = render_report(
        args.build_dir,
        add_executable_sources,
        rocketchip_sources,
        all_src_cpps,
        unbuilt,
        dce,
        stale_objs,
        drift_in_pedantic_only,
        drift_in_executable_only,
    )

    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(report, encoding="utf-8")
        print(f"Report written: {args.output}", file=sys.stderr)
    else:
        # Use UTF-8 to stdout (Windows default is cp1252; force here).
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stdout.write(report)

    has_findings = bool(
        unbuilt
        or dce
        or stale_objs
        or drift_in_pedantic_only
        or drift_in_executable_only
    )
    return 0 if not has_findings else 1


if __name__ == "__main__":
    sys.exit(main())
