#!/usr/bin/env python3
"""Git last-touch recency audit for authored Rocket-Chip trees.

Companion to ``scripts/audit/find_dead_code.py`` (Class 1–4 whole-TU orphans).
This tool ranks files by last author-date so stagnant areas stand out after
large reworks. Age alone is a **red flag**, not proof of deadness.

Project timeline note
---------------------
Early tree history (pre-baremetal) is not a useful age baseline. FreeRTOS /
exploratory work in Jan–early Feb 2026 predates the bare-metal Pico SDK pivot
that is the true starting point of the current codebase.

Default ``--epoch`` is **2026-02-03**, matching commit ``ddc2637``
("Pivot to bare-metal Pico SDK: remove FreeRTOS, clean all docs") and
CHANGELOG entry ``2026-02-03-002``. Stage 1 foundation completed the next day
(``499d869``, 2026-02-04). Note: QP/C AO framework arrived later (IVP-67
2026-03-15 / IVP-76 2026-03-27) and is *not* the baremetal epoch.

Calendar buckets stop at **91+ days** — a ``>180d`` tier is not useful while
the baremetal tree is still younger than that (and would mostly flag noise
from the pre-pivot era if raised later without rethinking the epoch).

Usage
-----
    python tools/recency_audit.py
    python tools/recency_audit.py --src-only --oldest 40
    python tools/recency_audit.py --tsv out/recency.tsv --summary out/summary.txt
    python tools/recency_audit.py --min-days 60 --prefix src/

See ``docs/tools/RECENCY_AUDIT.md``.
"""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
from collections import defaultdict
from datetime import date, datetime, timezone
from pathlib import Path

# Baremetal Pico SDK pivot (not first git commit, not FreeRTOS scaffold, not QP).
# Source of truth: git commit ddc2637 + CHANGELOG 2026-02-03-002.
DEFAULT_EPOCH = date(2026, 2, 3)

# Default trees — first-party authored surfaces only.
DEFAULT_TREES = (
    "src/",
    "include/rocketchip/",
    "scripts/",
    "test/",
    "tools/",
    "ground_station/",
    "starcom/",
    "profiles/",
    "CMakeLists.txt",
)

SKIP_SUBSTRINGS = (
    "pico-sdk",
    "EXTERNAL",
    "__pycache__",
    ".pyc",
    "/build/",
    "\\build\\",
)

# Age buckets tuned for a codebase whose modern era began ~DEFAULT_EPOCH.
# Upper open bucket is 91+ (not 180+) — see module docstring.
DEFAULT_BUCKETS: tuple[tuple[str, int | None], ...] = (
    ("0-14d", 14),
    ("15-30d", 30),
    ("31-60d", 60),
    ("61-90d", 90),
    ("91d+", None),  # open-ended
)


def find_repo_root(start: Path | None = None) -> Path:
    """Resolve git top-level from start (or cwd)."""
    cwd = start or Path.cwd()
    try:
        r = subprocess.run(
            ["git", "rev-parse", "--show-toplevel"],
            cwd=cwd,
            capture_output=True,
            text=True,
            check=True,
        )
    except (subprocess.CalledProcessError, FileNotFoundError) as exc:
        raise SystemExit(
            f"ERROR: not inside a git repo (cwd={cwd}): {exc}"
        ) from exc
    return Path(r.stdout.strip())


def git_ls_files(repo: Path, trees: list[str]) -> list[str]:
    r = subprocess.run(
        ["git", "ls-files", "--"] + trees,
        cwd=repo,
        capture_output=True,
        text=True,
        check=True,
    )
    out = []
    for p in r.stdout.splitlines():
        if not p:
            continue
        if any(s in p for s in SKIP_SUBSTRINGS):
            continue
        out.append(p.replace("\\", "/"))
    return out


def last_touch_map(repo: Path, trees: list[str]) -> dict[str, tuple[str, str]]:
    """path -> (ISO author date, subject) via one bulk git log walk.

    First time a path is seen in reverse-chronological log is its last touch.
    """
    last: dict[str, tuple[str, str]] = {}
    cmd = ["git", "log", "--format=%aI%x09%s", "--name-only", "--"] + trees
    proc = subprocess.run(
        cmd, cwd=repo, capture_output=True, text=True, check=True
    )
    cur_date: str | None = None
    cur_subj = ""
    for line in proc.stdout.splitlines():
        if not line.strip():
            continue
        # ISO date lines start with a digit and contain a tab-separated subject.
        if "\t" in line and line[0].isdigit():
            parts = line.split("\t", 1)
            cur_date = parts[0]
            cur_subj = parts[1] if len(parts) > 1 else ""
            continue
        p = line.strip().replace("\\", "/")
        if p and p not in last and cur_date:
            last[p] = (cur_date, cur_subj)
    return last


def fill_missing(
    repo: Path, paths: list[str], last: dict[str, tuple[str, str]]
) -> None:
    """Per-file log for anything the bulk walk missed (rare)."""
    for p in paths:
        if p in last:
            continue
        r = subprocess.run(
            ["git", "log", "-1", "--format=%aI\t%s", "--", p],
            cwd=repo,
            capture_output=True,
            text=True,
        )
        if r.stdout.strip():
            d, s = r.stdout.strip().split("\t", 1)
            last[p] = (d, s)


def parse_iso(iso: str) -> datetime:
    iso2 = iso.replace("Z", "+00:00")
    try:
        dt = datetime.fromisoformat(iso2)
    except ValueError:
        dt = datetime.fromisoformat(iso[:19]).replace(tzinfo=timezone.utc)
    if dt.tzinfo is None:
        dt = dt.replace(tzinfo=timezone.utc)
    return dt


def age_days(iso: str, as_of: datetime) -> int:
    dt = parse_iso(iso)
    return (as_of - dt.astimezone(timezone.utc)).days


def bucket_label(days: int, buckets: tuple[tuple[str, int | None], ...]) -> str:
    for label, upper in buckets:
        if upper is None:
            return label
        if days <= upper:
            return label
    return buckets[-1][0]


def dir_key(path: str) -> str:
    parts = path.split("/")
    if path.startswith("src/") and len(parts) >= 2:
        return "/".join(parts[:2])
    if path.startswith("include/rocketchip/") and len(parts) >= 3:
        return "/".join(parts[:3])
    if path.startswith("scripts/") and len(parts) >= 2:
        return "/".join(parts[:2])
    if path.startswith("test/") and len(parts) >= 2:
        return "/".join(parts[:2])
    if path.startswith("tools/"):
        return "/".join(parts[:2]) if len(parts) >= 2 else "tools"
    if path.startswith("ground_station/"):
        return "ground_station"
    if path.startswith("starcom/"):
        return "starcom"
    if path.startswith("profiles/"):
        return "profiles"
    return parts[0] if parts else path


def build_rows(
    paths: list[str],
    last: dict[str, tuple[str, str]],
    as_of: datetime,
) -> list[tuple[str, str, str, int]]:
    """Return sorted rows: (iso_date, path, subject, age_days)."""
    rows: list[tuple[str, str, str, int]] = []
    for p in paths:
        if p not in last:
            continue
        d, s = last[p]
        rows.append((d, p, s, age_days(d, as_of)))
    rows.sort(key=lambda x: x[0])  # oldest first
    return rows


def render_summary(
    rows: list[tuple[str, str, str, int]],
    *,
    as_of: date,
    epoch: date,
    oldest_n: int,
    newest_n: int,
    buckets: tuple[tuple[str, int | None], ...],
    min_days: int | None,
    prefix: str | None,
) -> str:
    lines: list[str] = []
    lines.append("# Recency audit summary")
    lines.append("")
    lines.append(f"- As-of: `{as_of.isoformat()}` (UTC calendar day)")
    lines.append(
        f"- Baremetal-era epoch: `{epoch.isoformat()}` "
        f"(pre-epoch calendar age is not a useful product signal — "
        f"FreeRTOS / exploratory work predates the bare-metal Pico SDK pivot)"
    )
    days_since_epoch = (as_of - epoch).days
    lines.append(
        f"- Days since epoch: **{days_since_epoch}** "
        f"(files near this age have essentially never been reworked "
        f"in the baremetal era)"
    )
    lines.append(
        "- Interpretation: age is a **red flag**, not proof of deadness. "
        "Pair with `scripts/audit/find_dead_code.py` for whole-TU orphans."
    )
    lines.append(f"- Files ranked: **{len(rows)}**")
    if rows:
        lines.append(
            f"- Date range: `{rows[0][0][:10]}` .. `{rows[-1][0][:10]}`"
        )
    lines.append("")

    # Buckets
    counts: dict[str, int] = {label: 0 for label, _ in buckets}
    for _d, _p, _s, a in rows:
        counts[bucket_label(a, buckets)] += 1
    lines.append("## Age buckets")
    lines.append("")
    lines.append("| Bucket | Count |")
    lines.append("|--------|------:|")
    for label, _ in buckets:
        lines.append(f"| {label} | {counts[label]} |")
    lines.append("")
    lines.append(
        "Buckets are short by design: the baremetal tree is months old, "
        "so a 180-day tier would not discriminate real stagnation."
    )
    lines.append("")

    # Dir aggregates
    agg: dict[str, list[tuple[str, int]]] = defaultdict(list)
    for d, p, _s, a in rows:
        agg[dir_key(p)].append((d, a))
    lines.append("## Directory aggregates (by oldest last-touch)")
    lines.append("")
    lines.append("| Oldest | Newest | Count | Directory |")
    lines.append("|--------|--------|------:|-----------|")
    agg_rows = []
    for k, v in agg.items():
        dates = sorted(x[0] for x in v)
        ages = [x[1] for x in v]
        agg_rows.append((dates[0], dates[-1], len(v), max(ages), k))
    agg_rows.sort(key=lambda x: x[0])
    for o, n, c, _max_a, k in agg_rows:
        lines.append(f"| {o[:10]} | {n[:10]} | {c} | `{k}` |")
    lines.append("")

    # Oldest / newest
    lines.append(f"## Oldest {oldest_n} files")
    lines.append("")
    lines.append("| Date | Age (d) | Path |")
    lines.append("|------|--------:|------|")
    for d, p, _s, a in rows[:oldest_n]:
        lines.append(f"| {d[:10]} | {a} | `{p}` |")
    lines.append("")

    lines.append(f"## Newest {newest_n} files")
    lines.append("")
    lines.append("| Date | Age (d) | Path |")
    lines.append("|------|--------:|------|")
    for d, p, _s, a in rows[-newest_n:]:
        lines.append(f"| {d[:10]} | {a} | `{p}` |")
    lines.append("")

    # Filtered list
    if min_days is not None or prefix:
        lines.append("## Filtered list")
        lines.append("")
        filt = rows
        if min_days is not None:
            filt = [r for r in filt if r[3] >= min_days]
            lines.append(f"Filter: age ≥ {min_days} days")
        if prefix:
            filt = [r for r in filt if r[1].startswith(prefix)]
            lines.append(f"Filter: path prefix `{prefix}`")
        lines.append("")
        lines.append("| Date | Age (d) | Path | Last subject |")
        lines.append("|------|--------:|------|--------------|")
        for d, p, s, a in filt:
            subj = s.replace("|", "\\|")[:72]
            lines.append(f"| {d[:10]} | {a} | `{p}` | {subj} |")
        lines.append("")
        lines.append(f"({len(filt)} files)")
        lines.append("")

    # Hotspots: files touched in last 30d by dir
    hot: dict[str, int] = defaultdict(int)
    for _d, p, _s, a in rows:
        if a <= 30 and (p.startswith("src/") or p.startswith("include/")):
            hot[dir_key(p)] += 1
    if hot:
        lines.append("## Recent rework hotspots (src/include, ≤30d)")
        lines.append("")
        lines.append("| Count | Directory |")
        lines.append("|------:|-----------|")
        for k, v in sorted(hot.items(), key=lambda x: -x[1]):
            lines.append(f"| {v} | `{k}` |")
        lines.append("")

    lines.append("## Operator next steps")
    lines.append("")
    lines.append(
        "1. Treat 61–90d and 91d+ **production** paths as review candidates, "
        "not delete candidates."
    )
    lines.append(
        "2. Prefer pairing with `python scripts/audit/find_dead_code.py` "
        "(whole-TU Class 1–4) and `survey_test_subjects.py`."
    )
    lines.append(
        "3. Before deleting anything: `git log --all --oneline -- <file>` + "
        "CHANGELOG history-trace per CODING_STANDARDS dead-code discipline."
    )
    lines.append(
        "4. Stable sealed modules (math, codegen, regression CSVs) often look "
        "old while remaining live — confirm with callers, not calendar alone."
    )
    lines.append("")
    return "\n".join(lines)


def write_tsv(path: Path, rows: list[tuple[str, str, str, int]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as f:
        f.write("path\tlast_date\tage_days\tsubject\n")
        for d, p, s, a in rows:
            s_clean = s.replace("\t", " ").replace("\n", " ")
            f.write(f"{p}\t{d}\t{a}\t{s_clean}\n")


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Rank authored files by git last-touch age. "
            "Age is a red flag, not proof of deadness. "
            "Companion to scripts/audit/find_dead_code.py."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            "Examples:\n"
            "  python tools/recency_audit.py\n"
            "  python tools/recency_audit.py --src-only --oldest 50\n"
            "  python tools/recency_audit.py --min-days 60 --prefix src/\n"
            "  python tools/recency_audit.py --tsv recency.tsv "
            "--summary summary.md\n"
            "\n"
            "Docs: docs/tools/RECENCY_AUDIT.md\n"
        ),
    )
    p.add_argument(
        "--repo",
        type=Path,
        default=None,
        help="Repo root (default: git rev-parse from cwd)",
    )
    p.add_argument(
        "--as-of",
        type=str,
        default=None,
        metavar="YYYY-MM-DD",
        help="Age reference date (default: today UTC)",
    )
    p.add_argument(
        "--epoch",
        type=str,
        default=DEFAULT_EPOCH.isoformat(),
        metavar="YYYY-MM-DD",
        help=(
            f"Baremetal-era floor for report context "
            f"(default: {DEFAULT_EPOCH.isoformat()})"
        ),
    )
    p.add_argument(
        "--src-only",
        action="store_true",
        help="Limit to src/ and include/rocketchip/",
    )
    p.add_argument(
        "--tree",
        action="append",
        dest="trees",
        default=None,
        metavar="PATH",
        help="Authored tree to include (repeatable). Default: standard set",
    )
    p.add_argument(
        "--prefix",
        type=str,
        default=None,
        help="Only list/filter paths with this prefix (also applied to rows)",
    )
    p.add_argument(
        "--min-days",
        type=int,
        default=None,
        metavar="N",
        help="In summary filtered section, only paths aged ≥ N days",
    )
    p.add_argument(
        "--oldest",
        type=int,
        default=40,
        metavar="N",
        help="How many oldest files to list (default: 40)",
    )
    p.add_argument(
        "--newest",
        type=int,
        default=20,
        metavar="N",
        help="How many newest files to list (default: 20)",
    )
    p.add_argument(
        "--tsv",
        type=Path,
        default=None,
        help="Write full ranking TSV to this path",
    )
    p.add_argument(
        "--summary",
        type=Path,
        default=None,
        help="Write markdown summary to this path (default: stdout only)",
    )
    p.add_argument(
        "--quiet",
        action="store_true",
        help="Do not print summary to stdout (use with --summary/--tsv)",
    )
    return p.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)

    try:
        repo = args.repo.resolve() if args.repo else find_repo_root()
    except SystemExit:
        raise
    except Exception as exc:  # noqa: BLE001 — surface path errors cleanly
        print(f"ERROR: bad --repo: {exc}", file=sys.stderr)
        return 2

    if args.as_of:
        as_of_date = date.fromisoformat(args.as_of)
    else:
        as_of_date = datetime.now(timezone.utc).date()
    as_of_dt = datetime(
        as_of_date.year,
        as_of_date.month,
        as_of_date.day,
        tzinfo=timezone.utc,
    )
    epoch = date.fromisoformat(args.epoch)

    if args.src_only:
        trees = ["src/", "include/rocketchip/"]
    elif args.trees:
        trees = list(args.trees)
    else:
        trees = list(DEFAULT_TREES)

    paths = git_ls_files(repo, trees)
    if args.prefix:
        paths = [p for p in paths if p.startswith(args.prefix)]

    last = last_touch_map(repo, trees)
    fill_missing(repo, paths, last)
    rows = build_rows(paths, last, as_of_dt)

    summary = render_summary(
        rows,
        as_of=as_of_date,
        epoch=epoch,
        oldest_n=min(args.oldest, len(rows)),
        newest_n=min(args.newest, len(rows)),
        buckets=DEFAULT_BUCKETS,
        min_days=args.min_days,
        prefix=args.prefix if args.min_days is not None or args.prefix else args.prefix,
    )
    # Always show filtered section when --min-days set; if only --prefix,
    # rows are already prefix-filtered so filtered section still useful.
    if args.prefix and args.min_days is None:
        # Re-render with filtered section listing all (already prefix-limited) rows
        # that are "notable" — skip if empty. Already handled: prefix alone
        # still produces filtered section via render_summary when prefix set.
        pass

    if args.tsv:
        write_tsv(args.tsv, rows)
        print(f"TSV written: {args.tsv}", file=sys.stderr)

    if args.summary:
        args.summary.parent.mkdir(parents=True, exist_ok=True)
        args.summary.write_text(summary, encoding="utf-8")
        print(f"Summary written: {args.summary}", file=sys.stderr)

    if not args.quiet:
        # Windows consoles often default to cp1252.
        if hasattr(sys.stdout, "reconfigure"):
            try:
                sys.stdout.reconfigure(encoding="utf-8")
            except Exception:  # noqa: BLE001
                pass
        sys.stdout.write(summary)
        if not summary.endswith("\n"):
            sys.stdout.write("\n")

    return 0


if __name__ == "__main__":
    # Allow running as `python tools/recency_audit.py` from repo root or elsewhere.
    if "GIT_DIR" not in os.environ:
        pass
    raise SystemExit(main())
