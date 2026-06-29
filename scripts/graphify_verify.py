#!/usr/bin/env python3
"""Content-level verification of a curated graphify graph against the baseline.

WHY THIS EXISTS
---------------
Node-COUNT parity is NOT graph parity. A graph can hit "2448 == 2448" while
silently swapping a node's label, dropping a doc->code bridge, or losing edges
between surviving nodes (the "equal == parity fallacy"). This was nearly missed
once: count matched, but one bridge of 370 existed only in the baseline and not
the cache. It was caught only by checking CONTENT and EDGES, not the count.

Run this AFTER any change that regenerates the graph — especially after a full
LLM pass (`/graphify .`), which re-extracts doc nodes and could shift labels or
drop bridges that a count check would never reveal. It re-asserts the same
checks that caught the outlier, so we don't re-commit the fallacy.

CHECKS
------
1. NODE-SET parity: every baseline node id present; report extras/missing.
2. NODE CONTENT: for shared ids, core fields (label, file_type, source_file,
   norm_label) identical. `community` is excluded — clustering is recomputed
   each build and legitimately churns.
3. BRIDGES: count of doc->code edges; every baseline bridge reproduced
   (0 lost). Bridges are the connectivity-pass payload — the highest-value,
   most-fragile edges.
4. EDGES: target is a SUPERSET of baseline (0 missing); extras are listed for
   eyeball review, not failed (a fuller graph from later cache state is fine).
5. DETERMINISM (optional, --determinism): caller runs update+curate twice and
   passes both graphs; this asserts they are byte-identical edge/node sets.

EXIT
----
0 = all hard checks pass (node-set + content + bridges; edges superset).
1 = a hard check failed (missing baseline node, content mismatch, lost bridge,
    or a baseline edge absent). Extras alone never fail.

USAGE
-----
    python scripts/graphify_verify.py
    python scripts/graphify_verify.py --graph graphify-out/graph.json \
                                      --baseline graphify-out/claude-build-2026-06-28/graph.json
    # determinism: pass two independently-built graphs
    python scripts/graphify_verify.py --determinism A.json B.json
"""
from __future__ import annotations

import argparse
import json
import os
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO_ROOT, "graphify-out")
GRAPH = os.path.join(OUT_DIR, "graph.json")
BASELINE = os.path.join(OUT_DIR, "claude-build-2026-06-28", "graph.json")

CORE_FIELDS = ("label", "file_type", "source_file", "norm_label")


def _load(path):
    with open(path, encoding="utf-8") as fh:
        return json.load(fh)


def _ekey(e):
    return (str(e.get("source")), str(e.get("target")), e.get("relation"))


def _ok(msg):
    print(f"[verify] PASS  {msg}")


def _fail(msg):
    print(f"[verify] FAIL  {msg}")


def verify(graph_path, baseline_path):
    g = _load(graph_path)
    b = _load(baseline_path)
    gmap = {n["id"]: n for n in g["nodes"]}
    bmap = {n["id"]: n for n in b["nodes"]}
    code = {n["id"] for n in b["nodes"] if n.get("file_type") == "code"}
    failed = False

    # 1. node-set parity
    extra = sorted(set(gmap) - set(bmap))
    missing = sorted(set(bmap) - set(gmap))
    if missing:
        _fail(f"node-set: {len(missing)} baseline node(s) MISSING, e.g. {missing[:5]}")
        failed = True
    else:
        _ok(f"node-set: all {len(bmap)} baseline nodes present "
            f"({len(extra)} extra — listed below if any)")
    if extra:
        print(f"[verify]   extra nodes ({len(extra)}): {extra[:10]}"
              + (" ..." if len(extra) > 10 else ""))

    # 2. node content on shared ids
    shared = set(gmap) & set(bmap)
    content_bad = []
    for nid in shared:
        for f in CORE_FIELDS:
            if gmap[nid].get(f) != bmap[nid].get(f):
                content_bad.append((nid, f, gmap[nid].get(f), bmap[nid].get(f)))
    if content_bad:
        _fail(f"node-content: {len(content_bad)} field mismatch(es), e.g. {content_bad[:3]}")
        failed = True
    else:
        _ok(f"node-content: {len(shared)} shared nodes identical on {CORE_FIELDS}")

    # 3. bridges
    def bridges(graph):
        out = set()
        for e in graph["links"]:
            k = _ekey(e)
            if (k[0] in code) ^ (k[1] in code):
                out.add(k)
        return out
    gb, bb = bridges(g), bridges(b)
    lost = sorted(bb - gb)
    if lost:
        _fail(f"bridges: {len(lost)} baseline doc->code bridge(s) LOST, e.g. {lost[:3]}")
        failed = True
    else:
        _ok(f"bridges: all {len(bb)} baseline doc->code bridges reproduced "
            f"(graph has {len(gb)})")

    # 4. edges superset
    ge = {_ekey(e) for e in g["links"]}
    be = {_ekey(e) for e in b["links"]}
    miss_e = sorted(be - ge)
    extra_e = sorted(ge - be)
    if miss_e:
        _fail(f"edges: {len(miss_e)} baseline edge(s) MISSING, e.g. {miss_e[:3]}")
        failed = True
    else:
        _ok(f"edges: superset of baseline ({len(be)} baseline edges all present; "
            f"{len(extra_e)} extra)")
    if extra_e:
        print(f"[verify]   extra edges ({len(extra_e)}) — eyeball, not a failure:")
        for k in extra_e[:10]:
            print(f"[verify]      {k[0][:38]} -[{k[2]}]-> {k[1][:38]}")

    return not failed


def verify_determinism(path_a, path_b):
    a, bb = _load(path_a), _load(path_b)
    an = {n["id"] for n in a["nodes"]}
    bn = {n["id"] for n in bb["nodes"]}
    ae = {_ekey(e) for e in a["links"]}
    be = {_ekey(e) for e in bb["links"]}
    ok = True
    if an != bn:
        _fail(f"determinism: node sets differ (+{len(an-bn)}/-{len(bn-an)})")
        ok = False
    else:
        _ok(f"determinism: node sets identical ({len(an)})")
    if ae != be:
        _fail(f"determinism: edge sets differ (+{len(ae-be)}/-{len(be-ae)})")
        ok = False
    else:
        _ok(f"determinism: edge sets identical ({len(ae)})")
    return ok


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--graph", default=GRAPH)
    ap.add_argument("--baseline", default=BASELINE)
    ap.add_argument("--determinism", nargs=2, metavar=("A", "B"),
                    help="verify two independently-built graphs are identical")
    args = ap.parse_args(argv)

    if args.determinism:
        ok = verify_determinism(*args.determinism)
    else:
        ok = verify(args.graph, args.baseline)
    print(f"[verify] {'ALL CHECKS PASSED' if ok else 'VERIFICATION FAILED'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
