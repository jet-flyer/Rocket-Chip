"""Draft WN clusters for remediation planning. Findings.md is never written.

This is an agent grouping overlay, not extractive truth. Owner reviews buckets,
not each WN. Re-run after rule tweaks.
"""
from __future__ import annotations

import re
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
FINDINGS = HERE / "L2P5_WALK_FINDINGS.md"
OUT = HERE / "L2P5_WN_CLUSTERS.md"

HEADER = re.compile(
    r"^\*\*WN-(\d+)\*\* — \[([^\]]+)\] · `([^`]+)` · \*\*(.+?)\*\*\s*$",
    re.M,
)
PATH_H = re.compile(r"^#### (.+)$", re.M)

# (id, short name, what later work looks like)
CLUSTERS: list[tuple[str, str, str]] = [
    (
        "nolint",
        "In-source NOLINT / suppressions",
        "Same policy: in-source NOLINT is disallowed. Fix, named constant, or deviation log.",
    ),
    (
        "doxygen-density",
        "Doxygen + header comment-density policy",
        "Inventory first, then keep-consistently-or-drop (WN-054 / WN-081). Not a mass delete yet.",
    ),
    (
        "archaeology",
        "Process archaeology in comments",
        "IVP / Stage / council / session essays and tombstones. Trim to live contracts; rest to docs.",
    ),
    (
        "hw-agnostic",
        "HW leakage vs domain code",
        "Write the HW-agnostic rule, then dispose these. Board/SKU/PICO/pin essays in domain files.",
    ),
    (
        "starcom",
        "Starcom / radio-telem supersession",
        "Defer polish. Surfaces likely replaced or reshaped by Starcom / CCSDS.",
    ),
    (
        "codegen",
        "Generated files / codegen hygiene",
        "Start-from-scratch codegen audit. Do not hand-edit outputs.",
    ),
    (
        "spdx",
        "SPDX / third-party license inventory",
        "Dedicated attribution pass (WN-004). Overlaps generated-file SPDX.",
    ),
    (
        "rf-legal",
        "RF / regulatory config hazards",
        "Warnings + doc SSOT at frequency/power/band knobs (WN-100).",
    ),
    (
        "safety-ssot",
        "Safety / ops SSOT (Go/No-Go, pyro, guards)",
        "One owner for vital ops paths; comments-as-contract on pyro/guards.",
    ),
    (
        "early-impl",
        "Early-impl / design re-eval",
        "Keep-with-why or planned rework: seqlock, snapshot, drivers, flash map, PCM, etc.",
    ),
    (
        "rc-os",
        "RC_OS / CLI structure",
        "Gated on the RC_OS rework. Do not half-refactor menus first.",
    ),
    (
        "test-in-flight",
        "Test / inject / debug in the flight tree",
        "Fault-inject, debug menus, test_mode surfaces in mainline src.",
    ),
    (
        "packaging",
        "File earn-rent / naming / packaging",
        "Sparse headers, vague names, folder-for-one-pair, job-pack boilerplate.",
    ),
    (
        "version-config",
        "Version / identity / config.h grab-bag",
        "Stale versions, over-strong SSOT banners, unused tier defines.",
    ),
    (
        "fusion-invariants",
        "Fusion / math / cal live invariants",
        "Mission-shaped defaults, rate comments, numerical contracts — not comment style.",
    ),
    (
        "fn-ptr",
        "P10-9 function pointers",
        "Unlogged live sites. Disposition: accept / fix / QP-defer. Working list also on main WB.",
    ),
    (
        "one-off",
        "One-off / leftover",
        "Does not share a later sitting with a pile above. Handle individually at disposition.",
    ),
]


def parse(text: str) -> list[dict]:
    headers = list(HEADER.finditer(text))
    paths = [
        (m.start(), m.group(1).strip().strip("`")) for m in PATH_H.finditer(text)
    ]

    def path_at(pos: int) -> str:
        p = "(project-wide)"
        for hp, val in paths:
            if hp > pos:
                break
            p = val
        return p

    notes = []
    for i, m in enumerate(headers):
        body = text[m.end() : headers[i + 1].start() if i + 1 < len(headers) else len(text)]
        nid = int(m.group(1))
        cites = {int(x) for x in re.findall(r"WN-(\d+)", body) if int(x) != nid}
        notes.append(
            {
                "id": nid,
                "kind": m.group(3),
                "title": m.group(4).strip(),
                "path": path_at(m.start()),
                "body": body,
                "blob": m.group(4) + "\n" + body,
                "cites": cites,
            }
        )
    return notes


def has(blob: str, *pats: str) -> bool:
    return any(re.search(p, blob, re.I) for p in pats)


# Hand-placed after a pass over leftovers. Specific later-sitting identity.
ID_MAP: dict[int, str] = {
    2: "early-impl",
    7: "version-config",
    8: "version-config",
    18: "version-config",
    20: "hw-agnostic",
    22: "hw-agnostic",
    23: "hw-agnostic",
    26: "hw-agnostic",
    27: "hw-agnostic",
    28: "hw-agnostic",
    31: "packaging",
    32: "packaging",
    37: "starcom",
    52: "fn-ptr",
    57: "packaging",
    63: "hw-agnostic",
    75: "fusion-invariants",
    80: "hw-agnostic",
    89: "early-impl",
    96: "early-impl",
    102: "hw-agnostic",
    110: "hw-agnostic",
    113: "packaging",
    118: "test-in-flight",
    120: "archaeology",
    121: "archaeology",
    125: "archaeology",
    127: "hw-agnostic",
    129: "test-in-flight",
    130: "archaeology",
    139: "early-impl",
    140: "packaging",
    143: "archaeology",
    145: "archaeology",
    148: "archaeology",
    153: "fusion-invariants",
    159: "archaeology",
    162: "hw-agnostic",
    171: "packaging",
    174: "archaeology",
    177: "packaging",
    191: "fusion-invariants",
    192: "archaeology",
    193: "archaeology",
    198: "packaging",
    200: "archaeology",
    203: "fusion-invariants",
    207: "archaeology",
    211: "archaeology",
    216: "hw-agnostic",
    219: "archaeology",
    220: "hw-agnostic",
    222: "archaeology",
    225: "packaging",
    228: "packaging",
    235: "starcom",
    238: "archaeology",
    239: "packaging",
    240: "packaging",
    246: "packaging",
    249: "packaging",
    252: "test-in-flight",
    254: "archaeology",
    257: "safety-ssot",
    264: "packaging",
    267: "early-impl",
    270: "test-in-flight",
    272: "archaeology",
    273: "early-impl",
    274: "safety-ssot",
    280: "fusion-invariants",
    284: "early-impl",
    288: "rc-os",
    290: "rc-os",
    306: "packaging",
    308: "packaging",
    310: "archaeology",
    311: "early-impl",
    312: "early-impl",
}


def assign(n: dict) -> str:
    b = n["blob"]
    t = n["title"]
    cites = n["cites"]
    nid = n["id"]
    kind = n["kind"]
    path = n["path"]

    if nid in ID_MAP:
        return ID_MAP[nid]

    # --- specific first ---
    if has(b, r"NOLINT"):
        return "nolint"
    if nid in {4} or (has(t, r"SPDX|third-party|license") and not has(t, r"generated")):
        if has(b, r"SPDX") and nid == 4:
            return "spdx"
        if nid == 4:
            return "spdx"
    if has(b, r"function.?pointer|\bfn-ptr\b|P10-9|kick_watchdog|g_gpsFn"):
        if has(t, r"function.pointer|fn-ptr|P10-9|callback") or has(
            b, r"P10-9|function-pointer ban"
        ):
            return "fn-ptr"

    if has(b, r"AUTO-GENERATED|Do not edit") or has(t, r"generated|codegen|regen"):
        if has(t, r"generated|codegen|regen") or has(b, r"AUTO-GENERATED"):
            return "codegen"

    if nid == 100 or has(t, r"regulatory|legal-config|FCC|Part 15"):
        return "rf-legal"

    if has(t, r"Go/No-Go|go_nogo|FIRE_PYRO|pyro intent|kGuardManaged|launch abort"):
        return "safety-ssot"
    if has(t, r"\bpyro\b|FIRE_PYRO") or (
        has(b, r"FIRE_PYRO|Go/No-Go") and kind == "invariant" and "flight_director" in path
    ):
        return "safety-ssot"
    if nid in {182, 184, 176, 172, 142}:
        return "safety-ssot"

    if has(t, r"Starcom") or nid in {46, 41, 97, 38, 40}:
        return "starcom"
    if 46 in cites or 41 in cites or 97 in cites:
        if has(b, r"Starcom") or has(t, r"radio|telem|MAVLink|scheduler|SX1276|LoRa"):
            return "starcom"

    if has(t, r"HW-agnostic|HW-specific|board pack|PICO_BOARD|SKU|pin") or nid in {
        63,
        68,
        309,
        78,
        108,
        109,
        320,
    }:
        if has(t, r"HW|board|PICO|SKU|pin|Feather|Fruit Jam|GPIO"):
            return "hw-agnostic"
    if 63 in cites or 68 in cites:
        if has(b, r"HW-agnostic|board pack|PICO_BOARD"):
            return "hw-agnostic"

    if has(t, r"fault.inject|test code|test_mode|debug sub-menu|Grok-triage debug"):
        return "test-in-flight"
    if nid in {258, 259, 260, 261, 262, 326} or "fault_inject" in path:
        return "test-in-flight"

    if path.startswith("cli/") or nid in {313, 314, 315, 316, 318, 321, 322, 323, 324, 325, 327}:
        if nid == 317 or nid == 319:
            return "nolint"
        if nid == 320:
            return "hw-agnostic"
        return "rc-os"

    if has(t, r"seqlock|SensorSnapshot|still the right path|early|prior-art|bespoke driver|PCM|flash.layout|I2C bus"):
        if has(t, r"re-eval|re-evaluate|still the right|early|prior.art|bespoke|PCM|seqlock|SensorSnapshot"):
            return "early-impl"
    if nid in {42, 45, 59, 62, 86, 166, 78, 79}:
        if nid in {78} and has(t, r"HW"):
            return "hw-agnostic"
        if nid in {42, 45, 59, 62, 86, 166, 79}:
            return "early-impl"

    if has(t, r"version|SSOT|config\.h|kFirmware|Build tag|tier / feature"):
        return "version-config"
    if nid in {9, 10, 11, 12, 13, 16} or path.endswith("version.h") or path.endswith("config.h"):
        if has(t, r"comment|banner|IVP|density") and kind == "comment" and nid not in {9, 10, 12}:
            pass
        elif nid in {9, 10, 12, 13, 16} or "version.h" in path:
            return "version-config"

    if has(t, r"earn rent|sparse|vague|filename|packaging|folder|breakout\?|own public header|three files"):
        return "packaging"
    if nid in {30, 35, 178, 186, 197, 276, 74, 45}:
        if nid == 45:
            return "early-impl"
        return "packaging"

    if kind == "invariant" and has(
        t, r"default|rate|Hz|mission|wind|ZUPT|sigma|noise|WMM|sentinel|cal sample"
    ):
        return "fusion-invariants"
    if nid in {132, 133, 134, 126, 157, 195}:
        return "fusion-invariants"

    if nid in {54, 81} or 54 in cites or 81 in cites:
        if has(t, r"Doxygen|density|comment-density|@brief|header exemption"):
            return "doxygen-density"
        if has(b, r"WN-054|WN-081") and has(t, r"Doxygen|density|comment ratio"):
            return "doxygen-density"
    if has(t, r"Doxygen|comment-density|comment density|comment ratio|high comment"):
        return "doxygen-density"

    if nid in {85, 189, 76} or has(t, r"IVP|Stage |council|archaeology|tombstone|process"):
        return "archaeology"
    if kind == "comment" and has(b, r"\bIVP-|\bStage \d|council|W-6|W-16"):
        return "archaeology"
    if kind == "comment" and has(t, r"banner|essay|history|narrative|tag"):
        return "archaeology"

    if nid == 163 or has(t, r"design doc"):
        return "early-impl"

    if kind == "ownership":
        return "early-impl" if has(t, r"re-eval|ownership|SSOT|home") else "one-off"

    return "one-off"


def main() -> None:
    notes = parse(FINDINGS.read_text(encoding="utf-8"))
    by_cluster: dict[str, list[dict]] = defaultdict(list)
    for n in notes:
        cid = assign(n)
        n["cluster"] = cid
        by_cluster[cid].append(n)

    lines = [
        "# L2-P5 WN clusters — draft for remediation planning",
        "",
        "Agent grouping overlay. **Findings stay frozen.** This is not extractive truth",
        "and not disposition (no fix/accept/defer yet). Review **buckets**, not each WN.",
        "Delete or replace when real disposition starts. Same close rule as the walk WB:",
        "every WN is landed before L2-P5 closes.",
        "",
        f"Assigned: **{len(notes)}** notes into **{sum(1 for c,_,__ in CLUSTERS if by_cluster[c])}** buckets.",
        "",
        "## Bucket sizes",
        "",
        "| n | Bucket | Later sitting |",
        "|--:|--------|---------------|",
    ]
    for cid, name, later in CLUSTERS:
        members = by_cluster[cid]
        if not members:
            continue
        lines.append(f"| {len(members)} | {name} | {later} |")

    lines += [
        "",
        "## ID lists (cite this in the plan)",
        "",
    ]
    for cid, name, _later in CLUSTERS:
        members = sorted(by_cluster[cid], key=lambda x: x["id"])
        if not members:
            continue
        ids = ", ".join(f"WN-{n['id']:03d}" for n in members)
        lines += [f"**{name}:** {ids}", ""]

    for cid, name, later in CLUSTERS:
        members = sorted(by_cluster[cid], key=lambda x: x["id"])
        if not members:
            continue
        lines += [
            "",
            f"## {name} ({len(members)})",
            "",
            later,
            "",
        ]
        for n in members:
            lines.append(
                f"- **WN-{n['id']:03d}** `{n['kind']}` · {n['title']} · `{n['path']}`"
            )

    missing = {n["id"] for n in notes} - {n["id"] for ms in by_cluster.values() for n in ms}
    if missing:
        lines += ["", "## ERROR unassigned", "", str(sorted(missing))]

    OUT.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print("sizes:")
    for cid, name, _ in CLUSTERS:
        print(f"  {len(by_cluster[cid]):3d}  {name}")


if __name__ == "__main__":
    main()
