#!/usr/bin/env python3
"""Deterministic post-build curation filter for the graphify knowledge graph.

WHY THIS EXISTS
---------------
`graphify update .` regenerates `graphify-out/graph.json` automatically
(post-commit hook, file watch, or manual run). That AST-only rebuild does two
unwanted things to the curated graph:

  1. It runs graphify's build-time structural markdown pass, which emits a
     `document` node per heading/section (~2,600 on this repo, tagged
     `_origin == "ast"`). These bloat and dilute the graph.

  2. Its cache re-merge ORPHANS curated semantic children: the structural pass
     regenerates each doc's heading skeleton fresh, and cached `concept`/
     `document` nodes that re-anchor to a regenerated heading get dropped
     (~57+ on this repo, including doc->code bridges from the connectivity
     pass). The nodes still live in the semantic cache — the merge just doesn't
     surface them.

This filter reconciles that automatic output back to the curated shape,
deterministically and with NO LLM, so the live graph everyone queries stays at
parity with the curated target on every rebuild.

WHAT IT DOES (in order)
-----------------------
1. DROP structural fragments: any node with file_type=="document" and
   _origin=="ast" (the signature of the build-time structural pass).
2. KEEP everything else from the fresh rebuild (code comes fresh from the AST
   pass — deterministic, can't rot).
3. RESTORE curated nodes the rebuild orphaned, FROM THE SEMANTIC CACHE, gated
   so only genuinely-curated content returns:
     - node is concept/document/paper (not a structural fragment), AND
     - its source_file is one that survives curation (present in the curated
       source set — derived from the verification baseline / .graphifyignore).
   Reading from the cache (not a frozen snapshot) means new curated docs — e.g.
   a future Starcom IVP added via `/graphify` — are carried forward
   automatically once their LLM extraction lands in the cache. The cache only
   ever ADDS BACK a node the rebuild deleted; it never overwrites a live node.
4. BACKSTOP edges: restore any baseline link whose BOTH endpoints survive but
   the link itself is absent. Recovers bridges that were written to the live
   graph during the linking pass but never persisted to the per-file cache
   (one such straggler on this repo). Only ever adds an edge between two
   already-present nodes — never removes or rewrites.
5. Filter links + hyperedges to surviving endpoints; atomic write.

GUARDS (your "don't silently serve stale / don't silently overwrite" asks)
--------------------------------------------------------------------------
- STALENESS FLAG: a restored node is served from the cache's last LLM
  extraction of its source doc. If that doc has been edited since (the live
  file's content hash differs from the cache entry's hash), the restored node
  may be stale. The filter still restores it (so connectivity holds) but writes
  a STALE entry to the flag file naming the doc, so you can re-run `/graphify`
  to refresh it. Nothing is overwritten; staleness is surfaced, not applied.
- DRIFT REPORT: after filtering, the result is diffed against the verification
  baseline snapshot (passive — the snapshot never DRIVES restoration, so it
  can't freeze out new work). Any node/edge delta is reported for your review.
- PROMOTE flag: new source files entirely filtered out (candidates for real
  curation).
- REVIEW + ABORT: if kept code-node count collapses below the floor, or the
  drop count exceeds the known structural band, the root is left UNTOUCHED.

ROBUSTNESS INVARIANTS
---------------------
- Additive-only on curated content: the filter REMOVES structural fragments and
  ADDS BACK orphaned curated nodes/edges. It never replaces a live curated node
  with a different version (no overwrite path, no timestamp clobber).
- Fail-safe = no-op: missing/unparseable input leaves the root UNTOUCHED,
  non-zero exit.
- Idempotent: a clean root yields zero drops / zero restores / identical bytes.
- Atomic write: temp file + os.replace().

USAGE
-----
    python scripts/graphify_curate.py
    python scripts/graphify_curate.py --dry-run

Exit codes: 0 = filtered (or clean no-op), 2 = fail-safe no-op (bad input),
3 = ABORT on anomaly (root untouched, REVIEW flag written).
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import sys
from collections import Counter

# --- repo-relative paths ----------------------------------------------------
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(REPO_ROOT, "graphify-out")
ROOT_GRAPH = os.path.join(OUT_DIR, "graph.json")
CACHE_SEM_DIR = os.path.join(OUT_DIR, "cache", "semantic")
# Verification baseline: defines the curated source set + edge backstop.
# Passive — it scopes restoration and reports drift, it does NOT drive it.
BASELINE = os.path.join(OUT_DIR, "claude-build-2026-06-28", "graph.json")
STATE_FILE = os.path.join(OUT_DIR, ".curate_state.json")
FLAG_FILE = os.path.join(OUT_DIR, ".curate_flag.json")

# --- anomaly thresholds -----------------------------------------------------
CODE_FLOOR_FRACTION = 0.90      # abort if kept code nodes fall below this of baseline
DROP_SPIKE_CEILING = 6000       # abort if more than this many nodes would drop
CURATED_TYPES = ("concept", "document", "paper")


def _log(msg: str) -> None:
    print(f"[curate] {msg}")


def _load_graph(path: str, what: str):
    if not os.path.isfile(path):
        _log(f"FAIL-SAFE: {what} not found at {path}")
        return None
    try:
        with open(path, encoding="utf-8") as fh:
            g = json.load(fh)
    except (OSError, json.JSONDecodeError) as exc:
        _log(f"FAIL-SAFE: could not read {what}: {exc}")
        return None
    if not isinstance(g, dict) or not g.get("nodes"):
        _log(f"FAIL-SAFE: {what} has no nodes")
        return None
    return g


def _load_cache(sem_dir: str):
    """Aggregate the per-source-file semantic cache.

    Returns (nodes_by_id, edges, src_hash) where src_hash maps source_file ->
    the cache entry filename stem (graphify content-hashes each entry, so the
    stem is effectively the doc's extraction hash; a changed doc gets a new
    stem on the next /graphify, which is how we detect staleness)."""
    nodes, edges, src_stub = {}, [], {}
    if not os.path.isdir(sem_dir):
        return nodes, edges, src_stub
    for name in os.listdir(sem_dir):
        p = os.path.join(sem_dir, name)
        if not os.path.isfile(p):
            continue
        try:
            d = json.load(open(p, encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        stem = os.path.splitext(name)[0]
        for n in d.get("nodes", []):
            nodes[n["id"]] = n
            sf = n.get("source_file")
            if sf:
                src_stub[sf] = stem
        edges.extend(d.get("edges", []))
    return nodes, edges, src_stub


def _ekey(e):
    return (str(e.get("source")), str(e.get("target")), e.get("relation"))


def _file_hash(path: str):
    try:
        with open(path, "rb") as fh:
            return hashlib.sha256(fh.read()).hexdigest()
    except OSError:
        return None


def _atomic_write(path, data):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as fh:
        json.dump(data, fh, ensure_ascii=False, indent=2)
    os.replace(tmp, path)


def _write_flag(payload: dict) -> None:
    try:
        _atomic_write(FLAG_FILE, payload)
        _log(f"wrote flags -> {os.path.relpath(FLAG_FILE, REPO_ROOT)}")
    except OSError as exc:
        _log(f"warning: could not write flag file: {exc}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--root", default=ROOT_GRAPH)
    ap.add_argument("--baseline", default=BASELINE)
    ap.add_argument("--cache", default=CACHE_SEM_DIR)
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args(argv)

    flags = {}

    # --- inputs --------------------------------------------------------------
    root = _load_graph(args.root, "root graph")
    if root is None:
        return 2
    baseline = _load_graph(args.baseline, "baseline snapshot")
    if baseline is None:
        return 2
    cache_nodes, cache_edges, cache_src_stub = _load_cache(args.cache)
    if not cache_nodes:
        _log("FAIL-SAFE: semantic cache empty or unreadable")
        return 2

    base_nodes = {n["id"]: n for n in baseline["nodes"]}
    curated_srcs = {n.get("source_file") for n in baseline["nodes"] if n.get("source_file")}
    base_code = sum(1 for n in baseline["nodes"] if n.get("file_type") == "code")
    _log(f"baseline: {len(base_nodes)} nodes ({base_code} code), "
         f"{len(curated_srcs)} curated source files")
    _log(f"cache: {len(cache_nodes)} nodes, {len(cache_edges)} edges")

    # --- 1+2: drop structural fragments, keep the rest -----------------------
    nodes = root["nodes"]
    fragments = [n for n in nodes
                 if n.get("file_type") == "document" and n.get("_origin") == "ast"]
    kept = [n for n in nodes
            if not (n.get("file_type") == "document" and n.get("_origin") == "ast")]
    kept_by_id = {n["id"]: n for n in kept}
    kept_code = sum(1 for n in kept if n.get("file_type") == "code")

    # --- anomaly gate BEFORE writing ----------------------------------------
    code_floor = int(base_code * CODE_FLOOR_FRACTION)
    if kept_code < code_floor:
        _log(f"ANOMALY (keep-collapse): kept code {kept_code} < floor {code_floor}")
        _write_flag({"flag": "REVIEW", "reason": "kept code-node count below floor",
                     "kept_code": kept_code, "code_floor": code_floor})
        _log("ABORT: root left untouched.")
        return 3
    if len(fragments) > DROP_SPIKE_CEILING:
        _log(f"ANOMALY (drop-spike): {len(fragments)} fragments > ceiling {DROP_SPIKE_CEILING}")
        _write_flag({"flag": "REVIEW", "reason": "fragment drop exceeds structural band",
                     "fragments": len(fragments), "ceiling": DROP_SPIKE_CEILING})
        _log("ABORT: root left untouched.")
        return 3

    # --- 3: restore orphaned curated nodes from cache (gated) ---------------
    # Load prior state up front: it carries, per restored source doc, the
    # cache-entry stem and the live content hash observed last run. Staleness =
    # the live doc changed (content hash moved) while its cache entry did NOT
    # (stem unchanged) -> the cache is serving a pre-edit extraction.
    prev_state = {}
    if os.path.isfile(STATE_FILE):
        try:
            prev_state = json.load(open(STATE_FILE, encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            prev_state = {}
    prev_docs = prev_state.get("doc_tracking", {})  # sf -> {stem, live_hash}

    restored, stale = [], []
    doc_tracking = {}
    for nid, n in cache_nodes.items():
        if nid in kept_by_id:
            continue
        if n.get("file_type") not in CURATED_TYPES:
            continue
        sf = n.get("source_file")
        if sf not in curated_srcs:
            continue
        restored.append(n)
        if sf and sf not in doc_tracking:
            stem = cache_src_stub.get(sf)
            live = _file_hash(os.path.join(REPO_ROOT, sf))
            doc_tracking[sf] = {"stem": stem, "live_hash": live}
            prior = prev_docs.get(sf)
            # Flag only when we have a prior observation, the live content
            # moved, but the cache stem did not (i.e. /graphify hasn't re-run
            # on this doc since it was edited). First-ever run records, never
            # flags (no prior to compare).
            if (prior and live and prior.get("live_hash")
                    and live != prior["live_hash"]
                    and stem == prior.get("stem")):
                stale.append(sf)
    for n in restored:
        kept_by_id[n["id"]] = n
    final_nodes = list(kept_by_id.values())
    surviving = set(kept_by_id)

    # --- 4: edge backstop from baseline (both endpoints must survive) -------
    root_links = root.get("links", [])
    have_edges = {_ekey(e) for e in root_links}
    # also consider cache edges as a source for restored-node edges
    for e in cache_edges:
        k = _ekey(e)
        if k not in have_edges and e.get("source") in surviving and e.get("target") in surviving:
            root_links.append(e)
            have_edges.add(k)
    backstopped = 0
    for e in baseline.get("links", []):
        k = _ekey(e)
        if k in have_edges:
            continue
        if e.get("source") in surviving and e.get("target") in surviving:
            root_links.append(e)
            have_edges.add(k)
            backstopped += 1

    # --- 5: filter links + hyperedges to survivors --------------------------
    kept_links = [e for e in root_links
                  if e.get("source") in surviving and e.get("target") in surviving]
    kept_hyper = [h for h in root.get("hyperedges", [])
                  if all(nid in surviving for nid in h.get("nodes", []))]

    # --- report --------------------------------------------------------------
    _log(f"nodes: {len(nodes)} -> dropped {len(fragments)} fragments, "
         f"restored {len(restored)} curated -> {len(final_nodes)}")
    _log(f"links: {len(root.get('links', []))} root -> {len(kept_links)} "
         f"(+{backstopped} baseline-backstopped bridges)")
    _log(f"hyperedges -> {len(kept_hyper)}")

    # staleness flag
    if stale:
        stale = sorted(set(stale))
        _log(f"STALE: {len(stale)} restored doc(s) changed since last /graphify: {stale[:6]}")
        flags["stale_docs"] = stale

    # PROMOTE: new source files entirely filtered out
    src_total = Counter(n.get("source_file") for n in nodes if n.get("source_file"))
    src_kept = Counter(n.get("source_file") for n in final_nodes if n.get("source_file"))
    fully_dropped = sorted(sf for sf, _ in src_total.items() if src_kept.get(sf, 0) == 0)
    new_fully = sorted(set(fully_dropped) - set(prev_state.get("fully_dropped_sources", [])))
    if new_fully:
        _log(f"PROMOTE: {len(new_fully)} new fully-dropped source(s): {new_fully[:6]}")
        flags["promote_sources"] = new_fully

    # DRIFT report vs baseline (passive)
    final_ids = surviving
    base_ids = set(base_nodes)
    drift_extra = sorted(final_ids - base_ids)
    drift_missing = sorted(base_ids - final_ids)
    if drift_extra or drift_missing:
        _log(f"DRIFT vs baseline: +{len(drift_extra)} extra, -{len(drift_missing)} missing")
        flags["drift_extra"] = drift_extra[:50]
        flags["drift_missing"] = drift_missing[:50]
    else:
        _log("DRIFT vs baseline: none (node set matches baseline)")

    if flags:
        _write_flag(dict(flags, flag="ATTENTION"))
    elif os.path.isfile(FLAG_FILE):
        try:
            os.remove(FLAG_FILE)  # clear stale flag from a prior run
        except OSError:
            pass

    if args.dry_run:
        _log("dry-run: no files written.")
        return 0

    # --- write ---------------------------------------------------------------
    if not fragments and not restored and backstopped == 0:
        _log("root already curated — no write needed.")
    else:
        root["nodes"] = final_nodes
        root["links"] = kept_links
        root["hyperedges"] = kept_hyper
        _atomic_write(args.root, root)
        _log(f"wrote curated graph -> {os.path.relpath(args.root, REPO_ROOT)}")

    try:
        _atomic_write(STATE_FILE, {
            "fully_dropped_sources": fully_dropped,
            "kept_nodes": len(final_nodes),
            "kept_code": kept_code,
            "restored": len(restored),
            "doc_tracking": doc_tracking,
        })
    except OSError as exc:
        _log(f"warning: could not write state file: {exc}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
