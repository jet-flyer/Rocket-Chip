"""Generate L2P5_WN_CITES.md and L2P5_WN_OVERLAP.md from frozen findings.

Extractive only: cites already in the text, plus uncited token overlap.
Does not name clusters. Findings.md is never written.
"""
from __future__ import annotations

import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
FINDINGS = HERE / "L2P5_WALK_FINDINGS.md"
CITES_OUT = HERE / "L2P5_WN_CITES.md"
OVERLAP_OUT = HERE / "L2P5_WN_OVERLAP.md"

HEADER = re.compile(
    r"^\*\*WN-(\d+)\*\* — \[([^\]]+)\] · `([^`]+)` · \*\*(.+?)\*\*\s*$",
    re.M,
)
PATH_H = re.compile(r"^#### (.+)$", re.M)
TIER_H = re.compile(r"^## (.+)$", re.M)

# Distinctive tokens / phrases. High-signal walk language, not English filler.
# Printed in the overlap report so a human can reject the pair.
TOKEN_SPECS: list[tuple[str, re.Pattern[str]]] = [
    ("NOLINT", re.compile(r"NOLINT", re.I)),
    ("@brief/@param/@file/@return", re.compile(r"@(?:brief|param|file|return)\b", re.I)),
    ("Doxygen", re.compile(r"\bDoxygen\b", re.I)),
    ("PICO_BOARD", re.compile(r"PICO_BOARD")),
    ("Starcom", re.compile(r"\bStarcom\b")),
    ("SPDX", re.compile(r"\bSPDX\b")),
    ("function-pointer / P10-9", re.compile(r"function.?pointer|\bfn-ptr\b|\bP10-9\b", re.I)),
    ("kick_watchdog", re.compile(r"kick_watchdog")),
    ("std::atomic", re.compile(r"std::atomic")),
    ("seqlock", re.compile(r"\bseqlock\b", re.I)),
    ("AUTO-GENERATED / codegen", re.compile(r"AUTO-GENERATED|\bcodegen\b|Do not edit", re.I)),
    ("council", re.compile(r"\bcouncil\b", re.I)),
    ("IVP-N", re.compile(r"\bIVP-[\w.]+")),
    ("Stage N / Stage T/L", re.compile(r"\bStage\s+(?:\d+|[TL]\b)")),
    ("HW-agnostic / board pack", re.compile(r"HW-agnostic|hardware-agnostic|board pack", re.I)),
    ("FCC / Part 15 / ISM", re.compile(r"\bFCC\b|Part 15|\bISM\b")),
    ("Go/No-Go", re.compile(r"Go/No-Go|go_nogo", re.I)),
    ("pyro / FIRE_PYRO", re.compile(r"\bFIRE_PYRO\b|\bpyro\b", re.I)),
    ("magic-number", re.compile(r"magic.?number", re.I)),
    ("tombstone / archaeology", re.compile(r"\btombstone\b|\barchaeology\b", re.I)),
]


def parse(text: str) -> list[dict]:
    headers = list(HEADER.finditer(text))
    notes: list[dict] = []
    last_path = "(none)"
    last_tier = "(none)"
    # Precompute path/tier at each header by scanning forward through headings
    headings: list[tuple[int, str, str]] = []
    for m in TIER_H.finditer(text):
        headings.append((m.start(), "tier", m.group(1).strip()))
    for m in PATH_H.finditer(text):
        raw = m.group(1).strip()
        raw = raw.strip("`")
        headings.append((m.start(), "path", raw))
    headings.sort()

    def loc_at(pos: int) -> tuple[str, str]:
        tier, path = "(none)", "(none)"
        for hp, kind, val in headings:
            if hp > pos:
                break
            if kind == "tier":
                tier = val
            else:
                path = val
        return tier, path

    for i, m in enumerate(headers):
        start = m.end()
        end = headers[i + 1].start() if i + 1 < len(headers) else len(text)
        body = text[start:end]
        nid = int(m.group(1))
        tier, path = loc_at(m.start())
        wn_out = {int(x) for x in re.findall(r"WN-(\d+)", body)}
        wn_out.discard(nid)
        w_out = sorted({int(x) for x in re.findall(r"\bW-(\d+)\b", body)})
        tokens = [name for name, pat in TOKEN_SPECS if pat.search(body) or pat.search(m.group(4))]
        notes.append(
            {
                "id": nid,
                "agent": m.group(2),
                "kind": m.group(3),
                "title": m.group(4).strip(),
                "path": path,
                "tier": tier,
                "body": body,
                "wn_out": wn_out,
                "w_out": w_out,
                "tokens": tokens,
            }
        )
    return notes


def write_cites(notes: list[dict]) -> str:
    by_id = {n["id"]: n for n in notes}
    inbound: dict[int, set[int]] = defaultdict(set)
    for n in notes:
        for o in n["wn_out"]:
            inbound[o].add(n["id"])

    lines: list[str] = [
        "# L2-P5 WN cite index (derived)",
        "",
        "Generated from `L2P5_WALK_FINDINGS.md`. **Not a record.** Findings stay frozen.",
        "Extractive: only `WN-` / `W-` strings already in the note bodies.",
        "Delete when every WN is landed (same rule as the walk whiteboard).",
        "Does not name clusters.",
        "",
        f"Notes: **{len(notes)}**. Cite edges (outbound WN): "
        f"**{sum(len(n['wn_out']) for n in notes)}**.",
        "",
        "## Hubs (most inbound WN cites)",
        "",
        "| Inbound | WN | Title |",
        "|--------:|----|-------|",
    ]
    hub_order = sorted(
        inbound.items(), key=lambda kv: (-len(kv[1]), kv[0])
    )
    for nid, ins in hub_order[:25]:
        title = by_id[nid]["title"] if nid in by_id else "(cited, no header — check typo)"
        lines.append(f"| {len(ins)} | WN-{nid:03d} | {title} |")

    isolated = [
        n
        for n in notes
        if not n["wn_out"] and not inbound[n["id"]]
    ]
    lines += [
        "",
        f"## Isolated (no WN in or out) — {len(isolated)}",
        "",
        "These have no `WN-` cite either way. They may still mention a `W-` row or a token.",
        "",
    ]
    for n in isolated:
        w = f" · W-{','.join(str(x) for x in n['w_out'])}" if n["w_out"] else ""
        lines.append(f"- **WN-{n['id']:03d}** `{n['kind']}` · {n['title']} · `{n['path']}`{w}")

    lines += ["", "## Per-note (outbound / inbound)", ""]
    for n in sorted(notes, key=lambda x: x["id"]):
        outs = ", ".join(f"WN-{x:03d}" for x in sorted(n["wn_out"])) or "—"
        ins = ", ".join(f"WN-{x:03d}" for x in sorted(inbound[n["id"]])) or "—"
        ws = ", ".join(f"W-{x}" for x in n["w_out"]) or "—"
        lines += [
            f"### WN-{n['id']:03d} — {n['title']}",
            "",
            f"`{n['kind']}` · `{n['path']}`",
            f"- **Out (this note cites):** {outs}",
            f"- **In (cites this note):** {ins}",
            f"- **W-rows mentioned:** {ws}",
            "",
        ]
    return "\n".join(lines) + "\n"


def linked(a: int, b: int, by_id: dict) -> bool:
    return b in by_id[a]["wn_out"] or a in by_id[b]["wn_out"]


def write_overlap(notes: list[dict]) -> str:
    by_id = {n["id"]: n for n in notes}
    token_members: dict[str, list[int]] = defaultdict(list)
    for n in notes:
        for t in n["tokens"]:
            token_members[t].append(n["id"])

    lines: list[str] = [
        "# L2-P5 WN uncited overlap (derived)",
        "",
        "Generated from `L2P5_WALK_FINDINGS.md`. **Not a record.** Findings stay frozen.",
        "A note appears here when it shares a **distinctive token** with another note",
        "and those two notes **do not cite each other** (`WN-` either way).",
        "The shared token is printed so the pair can be rejected.",
        "This is a candidate list, not a grouping. No cluster names.",
        "Delete when every WN is landed (same rule as the walk whiteboard).",
        "",
    ]

    any_block = False
    for token_name, _pat in TOKEN_SPECS:
        members = sorted(set(token_members.get(token_name, [])))
        if len(members) < 2:
            continue
        uncited_pairs: list[tuple[int, int]] = []
        for i, a in enumerate(members):
            if a not in by_id:
                continue
            for b in members[i + 1 :]:
                if b not in by_id:
                    continue
                if not linked(a, b, by_id):
                    uncited_pairs.append((a, b))
        cited_n = len(members) * (len(members) - 1) // 2 - len(uncited_pairs)
        lines += [
            f"## {token_name}",
            "",
            f"{len(members)} notes contain this token. "
            f"{cited_n} pairs already have a WN cite; "
            f"**{len(uncited_pairs)} pairs do not.**",
            "",
            "Members:",
            "",
        ]
        for nid in members:
            n = by_id[nid]
            lines.append(f"- **WN-{nid:03d}** `{n['kind']}` · {n['title']} · `{n['path']}`")
        if uncited_pairs:
            any_block = True
            # Large sets (e.g. "council") explode combinatorially — members list is the view.
            if len(uncited_pairs) > 40:
                lines += [
                    "",
                    f"Uncited pairs: **{len(uncited_pairs)}** (not listed; too many).",
                    "Use the member list above — that is the candidate pile.",
                    "",
                ]
            else:
                lines += ["", "Uncited pairs:", ""]
                for a, b in uncited_pairs:
                    lines.append(
                        f"- WN-{a:03d} ↔ WN-{b:03d} — {by_id[a]['title']} / {by_id[b]['title']}"
                    )
                lines.append("")
        else:
            lines += ["", "All pairs among these members already have a WN cite.", ""]
        lines.append("")

    if not any_block:
        lines += ["_(No uncited pairs under the current token list.)_", ""]
    return "\n".join(lines) + "\n"


def main() -> None:
    text = FINDINGS.read_text(encoding="utf-8")
    notes = parse(text)
    if len(notes) != 327:
        print(f"WARNING: parsed {len(notes)} notes, expected 327")
    CITES_OUT.write_text(write_cites(notes), encoding="utf-8")
    OVERLAP_OUT.write_text(write_overlap(notes), encoding="utf-8")
    print(f"Wrote {CITES_OUT.name} and {OVERLAP_OUT.name} ({len(notes)} notes)")


if __name__ == "__main__":
    main()
