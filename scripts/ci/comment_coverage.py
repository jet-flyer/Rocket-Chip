#!/usr/bin/env python3
"""List functions that may break JSF AV rule 130, and junk comments.

LIMITS (read before you use the output):
- This script only finds candidates. It never passes a comment. A clean
  run does not mean the comments are correct or that rule 130 passes.
- A changes-only run (--since) is not foolproof. It is valid only from a
  commit where a person did a real full read of each file. If that read
  was skipped or was weak, old wrong comments stay hidden.
- A comment that sounds right but is wrong can only be caught by a human
  read of the comment against the code.
- A comment inside a function counts as coverage but skips the junk
  checks. A lone tag such as "// E36" can hide a gap.
- Claim comments are checked only with --since, and only when the
  comment names a changed function. File-level "No state." lines never
  appear.

A function is a gap if no comment is directly above it, above its
declaration in a header in scope, or inside it. A gap can still be fine
when the name says the purpose; a person decides that.

    py -3 scripts/ci/comment_coverage.py [--since COMMIT] [path ...]

Default paths: starcom/src, starcom/include, starcom/adapters.
Needs lizard (py -3 -m pip install lizard). Exit code is always 0.
"""
import argparse
import re
import subprocess
import sys
from pathlib import Path

import lizard

WARNING = (
    "NOTE: This script only finds candidates. It never passes a comment.\n"
    "A clean run does not mean the comments are correct.\n"
    "A changes-only (--since) run is not foolproof: it is valid only from\n"
    "a commit where a person did a real full read of each file.\n"
    "A comment that sounds right but is wrong is caught only by a human read.\n"
    "A comment inside a function counts as coverage but skips the junk\n"
    "checks, so a lone tag such as // E36 can hide a gap.\n"
    "Claims are checked only with --since, and only when the comment\n"
    "names a changed function; file-level 'No state.' lines never appear."
)
DEFAULT_PATHS = ["starcom/src", "starcom/include", "starcom/adapters"]
EXTS = {".cpp", ".h", ".hpp", ".cc"}
SKIP_ABOVE = re.compile(
    r"^\s*(template\s*<|\[\[|static\b|inline\b|constexpr\b)")
CODE_LIKE = re.compile(r"(;\s*$|^\s*[{}]\s*$|^\s*(if|for|while|return)\b.*[(;]"
                       r"|^\s*#\s*(include|define|if)|\w+\s*\(.*\)\s*;)")
CLAIM = re.compile(r"only caller|no state|always|never|only called", re.I)
WORD = re.compile(r"[A-Za-z]+")


def comment_text(line):
    s = line.strip()
    for p in ("///", "//", "/*", "*/", "*"):
        if s.startswith(p):
            s = s[len(p):]
    return s.replace("*/", "").strip()


def is_comment(line):
    s = line.strip()
    return s.startswith(("//", "/*", "*")) or s.endswith("*/")


def block_above(lines, idx):
    """Comment lines directly above 0-based line idx (signature start)."""
    i = idx - 1
    while i >= 0 and SKIP_ABOVE.match(lines[i]) and not is_comment(lines[i]):
        i -= 1
    out = []
    while i >= 0 and is_comment(lines[i]):
        out.insert(0, comment_text(lines[i]))
        i -= 1
    return out


def name_words(name):
    base = name.split("::")[-1]
    parts = re.findall(r"[A-Z]?[a-z]+|[A-Z]+(?![a-z])|\d+", base)
    return {p.lower() for p in parts} | {base.lower()}


def junk_reasons(texts, fname, raw):
    text = " ".join(t for t in texts if t)
    words = WORD.findall(text)
    reasons = []
    if re.fullmatch(r"\s*(TODO|FIXME|XXX)\W*", text, re.I):
        reasons.append("bare TODO")
    if len(text.split()) < 3:
        reasons.append("under 3 words")
    real = [w for w in words if re.search(r"[aeiouyAEIOUY]", w) and len(w) > 1]
    if words and len(real) < len(words) / 2:
        reasons.append("not real words")
    if words and {w.lower() for w in words} <= name_words(fname) | {
            "the", "a", "an", "function", "this"}:
        reasons.append("only repeats the name")
    if any(CODE_LIKE.search(comment_text(r)) for r in raw):
        reasons.append("may be commented-out code")
    return reasons


def header_comments(files):
    """Map short function name -> comment lines above a header declaration."""
    found = {}
    decl = re.compile(r"\b([A-Za-z_]\w*)\s*\(")
    for f in files:
        if f.suffix not in (".h", ".hpp"):
            continue
        lines = f.read_text(encoding="utf-8", errors="replace").splitlines()
        for i, line in enumerate(lines):
            if is_comment(line):
                continue
            m = decl.search(line)
            if m:
                above = block_above(lines, i)
                if "//" in line:
                    above = above + [line.split("//", 1)[1].strip()]
                if above:
                    found.setdefault(m.group(1), above)
    return found


def changed_lines(since, path):
    out = subprocess.run(["git", "diff", "-U0", since, "--", str(path)],
                         capture_output=True, text=True, check=False).stdout
    got = set()
    for m in re.finditer(r"^@@ -\S+ \+(\d+)(?:,(\d+))? @@", out, re.M):
        start, count = int(m.group(1)), int(m.group(2) or "1")
        got.update(range(start, start + max(count, 1)))
    return got


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("paths", nargs="*", default=DEFAULT_PATHS)
    ap.add_argument("--since", help="only functions changed since COMMIT")
    args = ap.parse_args()
    print(WARNING + "\n")
    files = []
    for root in map(Path, args.paths):
        files += [root] if root.is_file() else sorted(root.rglob("*"))
    files = sorted(p for p in files if p.suffix in EXTS)
    hdr = header_comments(files)
    gaps, junk, claims, total = [], [], [], 0
    changed_names = set()
    for f in files:
        lines = f.read_text(encoding="utf-8", errors="replace").splitlines()
        info = lizard.analyze_file(str(f))
        changed = changed_lines(args.since, f) if args.since else None
        for fn in info.function_list:
            s, e = fn.start_line, fn.end_line
            if changed is not None:
                if not changed & set(range(s, e + 1)):
                    continue
                changed_names.add(fn.name.split("::")[-1])
            total += 1
            where = f"{f.as_posix()}:{s} {fn.name}"
            raw = [l for l in lines[max(0, s - 12):s - 1] if is_comment(l)]
            above = block_above(lines, s - 1)
            inside = [comment_text(l) for l in lines[s - 1:e] if "//" in l
                      or "/*" in l]
            short = fn.name.split("::")[-1]
            if not above and not inside and short in hdr:
                above = hdr[short]
            if not above and not inside:
                gaps.append(where)
                continue
            if above:
                why = junk_reasons(above, fn.name, raw[-len(above):])
                if why:
                    text = " ".join(above)[:60]
                    junk.append(f"{where}: {', '.join(why)}: {text}")
    if changed_names:
        for f in files:
            text = f.read_text(encoding="utf-8", errors="replace")
            for i, line in enumerate(text.splitlines(), 1):
                if is_comment(line) and CLAIM.search(line) and any(
                        n in line for n in changed_names):
                    claims.append(f"{f.as_posix()}:{i}: {line.strip()[:70]}")
    print(f"Functions read: {total}")
    print(f"No comment above or inside ({len(gaps)}):")
    print("\n".join("  " + g for g in gaps))
    print(f"Comment above may be junk ({len(junk)}):")
    print("\n".join("  " + j for j in junk))
    if args.since:
        print(f"Claims about changed functions ({len(claims)}):")
        print("\n".join("  " + c for c in claims))
    print("\n" + WARNING)
    return 0


if __name__ == "__main__":
    sys.exit(main())
