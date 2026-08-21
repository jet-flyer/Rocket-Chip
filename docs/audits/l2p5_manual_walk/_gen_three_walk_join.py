"""Join WN / GWF / aligned-CW findings onto itinerary leaves.

Extractive grouping only. Does not write the three finding packs.
Claude source: L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md
  - one vote per pack
  - skip rows marked duplicate-of
  - skip REFUTED for live-finding lists
"""
from __future__ import annotations

import json
import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
ITIN = HERE / "L2P5_WALK_ITINERARY.md"
OWNER = HERE / "L2P5_WALK_FINDINGS.md"
GROK = HERE / "L2P5_GROK_WALK_FINDINGS.md"
CLAUDE = HERE / "L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md"
JSON_OUT = HERE / "_three_walk_join.json"
DUMP_OUT = HERE / "_three_walk_join.txt"

IDENT = re.compile(
    r"\b(?:g_[A-Za-z0-9_]+|k[A-Z][A-Za-z0-9_]*|RC_[A-Z0-9_]+|"
    r"SIG_[A-Z0-9_]+|AO_[A-Za-z0-9_]+|"
    r"version_string|drop-oldest|drop-newest|drop-on-full|"
    r"NOLINT(?:BEGIN|END)?|P10-9|LedPhaseValue|"
    r"fused_state|seqlock|std::atomic|eskf_to_fused_state|"
    r"kRcLogRingBytes|kFdPreArmFail|kLedPhaseFault)\b"
)
IDENT2 = re.compile(r"`([A-Za-z_][A-Za-z0-9_:]{2,})`")

WN_H = re.compile(
    r"^\*\*WN-(\d+)\*\* — \[[^\]]+\] · `([^`]+)` · \*\*(.+?)\*\*\s*$",
    re.M,
)
CW_H = re.compile(
    r"^\*\*(CW-[A-Z0-9-]+)\*\* — \[Claude\](?: \*\(lens re-run\)\*)?"
    r" · `([^`]+)` · \*\*(.+?)\*\*\s*$",
    re.M,
)
PATH_H = re.compile(r"^#### (.+)$", re.M)
TIER_H = re.compile(r"^## (.+)$", re.M)


def load_leaves() -> list[str]:
    return [
        m.group(1)
        for m in re.finditer(
            r"^\s*- \[[ x]\] `([^`]+)`",
            ITIN.read_text(encoding="utf-8"),
            re.M,
        )
    ]


def strip_src(p: str) -> str:
    p = p.strip().strip("`").replace("\\", "/")
    p = re.sub(r"^C:/Users/[^/]+/Documents/[^/]+/", "", p)
    if p.startswith("src/"):
        p = p[4:]
    p = re.sub(r":.*$", "", p)
    return p.split()[0].rstrip(",") if p else p


def build_leaf_index(leaves: list[str]) -> dict[str, str]:
    idx: dict[str, str] = {}
    for leaf in leaves:
        idx[leaf] = leaf
        m = re.match(r"^(.+)/([^/]+)\.\{cpp,h\}$", leaf)
        if m:
            d, stem = m.group(1), m.group(2)
            for prefix in ("", "src/"):
                idx[f"{prefix}{d}/{stem}.cpp"] = leaf
                idx[f"{prefix}{d}/{stem}.h"] = leaf
        elif leaf.endswith((".cpp", ".h")):
            idx[leaf] = leaf
            idx["src/" + leaf] = leaf
            if leaf.startswith("src/"):
                idx[leaf[4:]] = leaf
    return idx


def file_to_leaf(path: str, leaves: list[str], idx: dict[str, str]) -> str | None:
    p = strip_src(path)
    if p in idx:
        return idx[p]
    base = p.rsplit("/", 1)[-1]
    stem = re.sub(r"\.(cpp|h)$", "", base)
    parent = p.rsplit("/", 1)[0] if "/" in p else ""
    for leaf in leaves:
        if leaf == p or leaf.endswith("/" + p) or leaf.endswith("/" + base):
            return leaf
        m = re.match(r"^(.+)/([^/]+)\.\{cpp,h\}$", leaf)
        if not m:
            continue
        if stem == m.group(2) and (
            parent in ("", m.group(1), "src/" + m.group(1)) or p == m.group(2)
        ):
            return leaf
    return None


def leaf_from_section(path: str, leaves: list[str], idx: dict[str, str]) -> str:
    if path == "(project-wide)":
        return "(project-wide)"
    cands = re.findall(r"`([^`]+)`", path)
    if not cands:
        cands = [path]
    parent = ""
    for piece in cands:
        piece = piece.strip()
        if "/" not in piece and parent:
            piece = parent + "/" + piece
        if "/" in piece:
            parent = piece.rsplit("/", 1)[0]
        leaf = file_to_leaf(piece, leaves, idx)
        if leaf:
            return leaf
    leaf = file_to_leaf(path, leaves, idx)
    return leaf or f"(unmapped) {path}"


def loc_at(headings: list[tuple[int, str, str]], pos: int) -> tuple[str, str]:
    tier, path = "(none)", "(project-wide)"
    for hp, kind, val in headings:
        if hp > pos:
            break
        if kind == "tier":
            tier = val
        else:
            path = val
    return tier, path


def headings_of(text: str) -> list[tuple[int, str, str]]:
    out: list[tuple[int, str, str]] = []
    for m in TIER_H.finditer(text):
        title = m.group(1).strip()
        if title.startswith(
            (
                "What changed",
                "Reconciliation",
                "Completeness",
                "Index",
                "Method",
                "Limitations",
                "Coverage",
            )
        ):
            continue
        out.append((m.start(), "tier", title))
    for m in PATH_H.finditer(text):
        out.append((m.start(), "path", m.group(1).strip()))
    out.sort()
    return out


def idents_from(text: str) -> set[str]:
    found = set(IDENT.findall(text))
    for tok in IDENT2.findall(text):
        if len(tok) >= 4:
            found.add(tok)
    return found


def last_verdict(body: str) -> str:
    hits = re.findall(r"^- Verdict: ([A-Z]+(?:-[A-Z]+)?)", body, re.M)
    if not hits:
        hits = re.findall(r"^- Verdict: ([A-Z]+)", body, re.M)
    return hits[-1] if hits else ""


def parse_owner(text: str, leaves: list[str], idx: dict[str, str]) -> list[dict]:
    heads = headings_of(text)
    headers = list(WN_H.finditer(text))
    notes = []
    for i, m in enumerate(headers):
        end = headers[i + 1].start() if i + 1 < len(headers) else len(text)
        body = text[m.end() : end]
        _, path = loc_at(heads, m.start())
        leaf = leaf_from_section(path, leaves, idx)
        notes.append(
            {
                "pack": "WN",
                "id": f"WN-{int(m.group(1)):03d}",
                "kind": m.group(2),
                "title": m.group(3).strip(),
                "path": path,
                "leaf": leaf,
                "body": body.strip(),
                "verdict": "",
                "duplicate": False,
                "idents": sorted(idents_from(m.group(0) + "\n" + body)),
            }
        )
    return notes


def parse_grok(text: str, leaves: list[str], idx: dict[str, str]) -> list[dict]:
    chunks = re.split(r"^### (GWF-\d+) — `([^`]+)`\s*$", text, flags=re.M)
    notes = []
    i = 1
    while i + 2 < len(chunks):
        gid, leaf_h, body = chunks[i], chunks[i + 1], chunks[i + 2]
        i += 3
        fields = {}
        for line in body.splitlines():
            if line.startswith("## "):
                break
            if line.startswith("- ") and ":" in line:
                k, _, v = line[2:].partition(":")
                fields[k.strip().lower()] = v.strip()
        fpath = fields.get("file", leaf_h)
        leaf = (
            file_to_leaf(leaf_h, leaves, idx)
            or file_to_leaf(fpath, leaves, idx)
            or leaf_h
        )
        issue = fields.get("issue", "")
        notes.append(
            {
                "pack": "GWF",
                "id": gid,
                "kind": fields.get("lens", ""),
                "title": issue,
                "path": fpath,
                "leaf": leaf,
                "severity": fields.get("severity", "").lower(),
                "claim": fields.get("claim", ""),
                "body": body[:2000],
                "verdict": "",
                "duplicate": False,
                "idents": sorted(idents_from(issue + "\n" + body[:2500])),
            }
        )
    return notes


def parse_claude(text: str, leaves: list[str], idx: dict[str, str]) -> list[dict]:
    # Stop before the completeness critic so quoted CW-IDs are not parsed as rows.
    cut = text.find("\n## Completeness critic")
    if cut < 0:
        cut = text.find("\n## 8. Deliverable-format")
    work = text if cut < 0 else text[:cut]
    heads = headings_of(work)
    headers = list(CW_H.finditer(work))
    notes = []
    for i, m in enumerate(headers):
        end = headers[i + 1].start() if i + 1 < len(headers) else len(work)
        body = work[m.end() : end]
        _, path = loc_at(heads, m.start())
        leaf = leaf_from_section(path, leaves, idx)
        dup = bool(re.search(r"duplicate of \*\*CW-", body))
        notes.append(
            {
                "pack": "CW",
                "id": m.group(1),
                "kind": m.group(2),
                "title": m.group(3).strip(),
                "path": path,
                "leaf": leaf,
                "body": body.strip(),
                "verdict": last_verdict(body),
                "duplicate": dup,
                "idents": sorted(idents_from(m.group(0) + "\n" + body)),
            }
        )
    return notes


def owner_nothing(text: str, leaves: list[str], idx: dict[str, str]) -> list[tuple]:
    out = []
    parts = re.split(r"^#### ", text, flags=re.M)
    for part in parts[1:]:
        first, _, rest = part.partition("\n")
        path = first.strip().strip("`")
        if re.search(r"nothing of note", rest[:800], re.I):
            leaf = file_to_leaf(path, leaves, idx) or path
            has_wn = bool(re.search(r"\*\*WN-\d+\*\*", rest[:2000]))
            out.append((leaf, path, has_wn))
    return out


def overlap_score(a: set[str], b: set[str]) -> tuple[int, list[str]]:
    skip = {
        "this",
        "that",
        "file",
        "header",
        "comment",
        "contract",
        "owner",
        "true",
        "false",
        "void",
        "bool",
        "include",
        "the",
        "and",
        "for",
        "not",
        "with",
        "Claim",
        "Verdict",
        "CONFIRMED",
        "RESHAPED",
        "REFUTED",
    }
    aa = {x for x in a if x not in skip and len(x) >= 4}
    bb = {x for x in b if x not in skip and len(x) >= 4}
    both = sorted(aa & bb)
    weight = 0
    for t in both:
        if t.startswith(("g_", "k", "RC_", "SIG_", "AO_")) or t in (
            "version_string",
            "drop-oldest",
            "drop-newest",
            "NOLINT",
            "std::atomic",
            "seqlock",
            "P10-9",
            "LedPhaseValue",
            "eskf_to_fused_state",
            "kRcLogRingBytes",
        ):
            weight += 3
        else:
            weight += 1
    return weight, both[:12]


def main() -> None:
    leaves = load_leaves()
    idx = build_leaf_index(leaves)
    owner_text = OWNER.read_text(encoding="utf-8")
    grok_text = GROK.read_text(encoding="utf-8")
    claude_text = CLAUDE.read_text(encoding="utf-8")
    wn = parse_owner(owner_text, leaves, idx)
    gwf = parse_grok(grok_text, leaves, idx)
    cw_all = parse_claude(claude_text, leaves, idx)
    cw_live = [
        n
        for n in cw_all
        if not n["duplicate"] and n["verdict"] not in ("REFUTED",)
    ]

    lines: list[str] = []

    def emit(*a: object) -> None:
        lines.append(" ".join(str(x) for x in a))

    emit("leaves", len(leaves))
    emit("WN", len(wn), "GWF", len(gwf), "CW parsed", len(cw_all))
    emit(
        "CW duplicate",
        sum(1 for n in cw_all if n["duplicate"]),
        "REFUTED non-dup",
        sum(1 for n in cw_all if (not n["duplicate"]) and n["verdict"] == "REFUTED"),
        "live",
        len(cw_live),
    )
    un_wn = [n["id"] + " " + n["leaf"] for n in wn if str(n["leaf"]).startswith("(unmapped)")]
    un_cw = [n["id"] + " " + n["leaf"] for n in cw_all if str(n["leaf"]).startswith("(unmapped)")]
    emit("unmapped WN", len(un_wn), "GWF", sum(1 for n in gwf if str(n["leaf"]).startswith("(unmapped)")), "CW", len(un_cw))
    for row in un_wn[:12]:
        emit("  WN", row)
    for row in un_cw[:12]:
        emit("  CW", row)

    by_leaf: dict[str, dict[str, list]] = defaultdict(lambda: {"WN": [], "GWF": [], "CW": []})
    for n in wn:
        by_leaf[n["leaf"]]["WN"].append(n)
    for n in gwf:
        by_leaf[n["leaf"]]["GWF"].append(n)
    for n in cw_live:
        by_leaf[n["leaf"]]["CW"].append(n)

    emit("\n== owner nothing-of-note (coverage, not clash) ==")
    for leaf, path, has_wn in owner_nothing(owner_text, leaves, idx):
        ng = len(by_leaf[leaf]["GWF"]) if leaf in by_leaf else 0
        nc = len(by_leaf[leaf]["CW"]) if leaf in by_leaf else 0
        emit(f"  {leaf}  mixed_wn={has_wn} GWF={ng} CW_live={nc}")

    emit("\n== per-leaf WN GWF CW_live ==")
    for leaf in leaves + ["(project-wide)"]:
        b = by_leaf.get(leaf)
        if not b:
            continue
        nw, ng, nc = len(b["WN"]), len(b["GWF"]), len(b["CW"])
        if nw + ng + nc == 0:
            continue
        emit(f"{nw:3d} {ng:3d} {nc:3d}  {leaf}")

    triples = []
    grok_claude = []
    for leaf, b in sorted(by_leaf.items()):
        for g in b["GWF"]:
            gs = set(g["idents"])
            for c in b["CW"]:
                sc, toks = overlap_score(gs, set(c["idents"]))
                if sc < 4:
                    continue
                best_w = None
                best_ws = 0
                best_wt: list[str] = []
                for w in b["WN"]:
                    ws, wt = overlap_score(gs | set(c["idents"]), set(w["idents"]))
                    if ws > best_ws:
                        best_ws, best_w, best_wt = ws, w, wt
                rec = {
                    "leaf": leaf,
                    "gwf": g["id"],
                    "g_title": g["title"][:100],
                    "g_sev": g.get("severity", ""),
                    "cw": c["id"],
                    "c_title": c["title"][:100],
                    "c_verdict": c["verdict"],
                    "gc_score": sc,
                    "gc_toks": toks,
                    "wn": best_w["id"] if best_w else None,
                    "w_title": best_w["title"][:100] if best_w else None,
                    "w_score": best_ws,
                    "w_toks": best_wt,
                }
                if best_w and best_ws >= 4:
                    triples.append(rec)
                else:
                    grok_claude.append(rec)

    triples.sort(key=lambda r: (-(r["gc_score"] + r["w_score"]), r["leaf"]))
    grok_claude.sort(key=lambda r: (-r["gc_score"], r["leaf"]))
    emit(f"\n== ident-overlap triples (candidates only) {len(triples)} ==")
    for r in triples[:60]:
        emit(f"  {r['leaf']}")
        emit(f"    {r['wn']} ({r['w_score']}) {r['w_title']}")
        emit(f"    {r['gwf']} {r['g_sev']} {r['g_title']}")
        emit(f"    {r['cw']} {r['c_verdict']} {r['c_title']}")
        emit(f"    toks={r['gc_toks'][:8]}")
    emit(f"\n== Grok+Claude weak/no WN {len(grok_claude)} ==")
    for r in grok_claude[:70]:
        emit(f"  {r['leaf']}")
        emit(f"    {r['gwf']} {r['g_sev']} {r['g_title']}")
        emit(f"    {r['cw']} {r['c_verdict']} {r['c_title']}")
        emit(f"    wn_near={r['wn']} ({r['w_score']}) toks={r['gc_toks'][:8]}")

    DUMP_OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    payload = {
        "leaves": leaves,
        "counts": {
            "WN": len(wn),
            "GWF": len(gwf),
            "CW_parsed": len(cw_all),
            "CW_dup": sum(1 for n in cw_all if n["duplicate"]),
            "CW_live": len(cw_live),
        },
        "by_leaf": {
            leaf: {
                "WN": [{"id": n["id"], "title": n["title"], "kind": n["kind"]} for n in b["WN"]],
                "GWF": [
                    {
                        "id": n["id"],
                        "title": n["title"],
                        "sev": n.get("severity"),
                    }
                    for n in b["GWF"]
                ],
                "CW": [
                    {"id": n["id"], "title": n["title"], "verdict": n["verdict"]}
                    for n in b["CW"]
                ],
            }
            for leaf, b in by_leaf.items()
        },
        "triples": triples,
        "grok_claude": grok_claude,
    }
    JSON_OUT.write_text(json.dumps(payload, indent=1), encoding="utf-8")
    print("wrote", DUMP_OUT, "and", JSON_OUT)
    print("\n".join(lines[:40]))


if __name__ == "__main__":
    main()
