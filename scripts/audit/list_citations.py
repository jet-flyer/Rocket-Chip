#!/usr/bin/env python3
"""L2-P3 citation inventory — produce a structured list of every distinct
cited source referenced from `standards/*.md`.

Closes L2-P3 from the 2026-05-07 master standards audit. The disposition
was: codify the 2026-05-07 audit's ad-hoc grep into a maintained
inventory so future audits walk against a deterministic population
(per the L2-P2 exhaustive-coverage rule + L2-P4 scope-language rule in
`standards/AUDIT_GUIDANCE.md` Tier 3.8 + line 90/92).

Population: every distinct citation found in:
  - standards/CODING_STANDARDS.md
  - standards/HW_GATE_DISCIPLINE.md
  - standards/AUDIT_GUIDANCE.md
  - standards/ACCEPTED_STANDARDS_DEVIATIONS.md
  - standards/RP2350_ERRATA.md
  - standards/DEBUG_OUTPUT.md
  - standards/GIT_WORKFLOW.md
  - standards/VENDOR_GUIDELINES.md (if present)

Citation patterns covered:
  - JSF AV Rule N
  - P10 Rule N (Power of 10)
  - JPL Rule N
  - MISRA-C YYYY Rule N.N
  - NASA SWE Handbook §N.N (and short forms)
  - LL Entry N
  - IVP-N
  - RP2350 datasheet §N.N.N  (R-26 was a §1.4.3 → §14.9.1 fix in this class)
  - SX1276 datasheet §N
  - DO-178C cross-references
  - URLs
  - file:line cross-references (e.g. src/foo.cpp:123)
  - Bare doc-path references
  - FCC Part N
  - RP2350 erratum cross-refs

Output: deduplicated inventory grouped by citation category, with
cite-site lists for each distinct source. Diff-friendly across audit
cycles — running on a later HEAD shows added/removed citations cleanly.

Exit codes:
    0 = inventory produced
    2 = environment skip (standards/ missing)
"""

from __future__ import annotations

import os
import re
import sys
from typing import List, Tuple


def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


STANDARDS_DOCS = [
    'standards/CODING_STANDARDS.md',
    'standards/HW_GATE_DISCIPLINE.md',
    'standards/AUDIT_GUIDANCE.md',
    'standards/ACCEPTED_STANDARDS_DEVIATIONS.md',
    'standards/RP2350_ERRATA.md',
    'standards/DEBUG_OUTPUT.md',
    'standards/GIT_WORKFLOW.md',
    'standards/VENDOR_GUIDELINES.md',
]


# Each pattern is (category, regex). Categories that overlap (e.g., a
# URL inside a datasheet-section line) report under each match.

CITATION_PATTERNS: List[Tuple[str, re.Pattern]] = [
    # JSF AV C++ rule citations
    ('JSF', re.compile(r'JSF[\s-]*AV[\s-]*(?:C\+\+\s*)?Rule\s+(\d+)', re.IGNORECASE)),
    # Power of 10 / P10 rule citations
    ('P10', re.compile(r'(?:Power[\s-]*of[\s-]*10|P10)[\s-]*Rule\s+(\d+)', re.IGNORECASE)),
    # JPL institutional C rule
    ('JPL', re.compile(r'JPL\s+(?:Institutional\s+)?(?:C\s+)?Rule\s+([\d\.]+)', re.IGNORECASE)),
    # MISRA-C rules
    ('MISRA', re.compile(r'MISRA[\s-]*C(?:[\s-]*(?:1998|2004|2012))?\s+Rule\s+([\d\.]+)', re.IGNORECASE)),
    # NASA SWE Handbook §N.N (multiple wordings)
    ('NASA-SWE', re.compile(r'NASA\s+(?:Software\s+Engineering\s+)?(?:SWE\s+)?Handbook(?:\s+\(SWEHB\))?\s+§?(\d+\.\d+)', re.IGNORECASE)),
    ('NASA-SWE', re.compile(r'\bNASA\s+SWE\s+§(\d+\.\d+)')),
    ('NASA-SWE', re.compile(r'\bSWE-?\s*Handbook\s+§(\d+\.\d+)')),
    # LL entries
    ('LL', re.compile(r'LL\s+Entry\s+(\d+)', re.IGNORECASE)),
    ('LL', re.compile(r'LESSONS_LEARNED\s+Entry\s+(\d+)', re.IGNORECASE)),
    # IVP and Stage
    ('IVP', re.compile(r'\bIVP-(\d+[a-z]?)\b', re.IGNORECASE)),
    ('Stage', re.compile(r'\bStage\s+(\d+[A-Z]?)\b')),
    # RP2350 datasheet sections — note: §1.4.3 was the rotted citation in R-26
    ('RP2350-datasheet', re.compile(r'RP2350\s+datasheet\s+(?:§|section\s+|sec\.\s*)([\d\.]+)', re.IGNORECASE)),
    # SX1276 datasheet sections
    ('SX1276-datasheet', re.compile(r'SX1276[A-Z/]*\s+datasheet\s+(?:§|section\s+|sec\.\s*)([\d\.]+)', re.IGNORECASE)),
    # Generic "datasheet §N.N.N" form (no chip qualifier on same phrase) — captures
    # the same class of rot as RP2350-datasheet when the chip context is upstream
    # in the doc. Real example: standards/RP2350_ERRATA.md:211, :284 say
    # "datasheet §12.4.3" with the RP2350 context implied from the doc title.
    ('datasheet-section', re.compile(r'\bdatasheet\s+§(\d+\.\d+(?:\.\d+)?)', re.IGNORECASE)),
    # DO-178C cross-references
    ('DO-178C', re.compile(r'\bDO-178C\b(?:\s+§?(\d+\.\d+))?')),
    # File-path references with line numbers (file:line)
    ('file-line', re.compile(r'\b([a-zA-Z_]+/[\w/\.]+\.\w+):(\d+)\b')),
    # Bare file paths to other docs
    ('doc-path', re.compile(r'`(docs/[\w/\.\-]+\.md)`')),
    # URLs
    ('URL', re.compile(r'(https?://[^\s\)\]\>]+)')),
    # FCC Part 15 + Errata
    ('FCC', re.compile(r'FCC\s+Part\s+(\d+(?:\.\d+)?)', re.IGNORECASE)),
    # RP2350 erratum cross-refs (E-N or RP2350-EN)
    ('Erratum', re.compile(r'\bRP2350[\s-]*E\s*(\d+)\b')),
    # Project-internal cycle references (R-N)
    ('R-finding', re.compile(r'\bR-(\d+(?:[a-z])?(?:-[\w]+)?)\b')),
]


def extract_citations(text: str) -> List[Tuple[int, str, str]]:
    """Return list of (line_number, category, citation_text) tuples."""
    found = []
    for line_no, line in enumerate(text.split('\n'), start=1):
        for category, pat in CITATION_PATTERNS:
            for m in pat.finditer(line):
                cite = m.group(0)
                # Truncate URLs at trailing punctuation that markdown often pulls in
                cite = cite.rstrip(').,;:!?')
                found.append((line_no, category, cite))
    return found


def main() -> int:
    repo = _repo_root()
    print('=' * 72)
    print('  L2-P3 citation inventory')
    print('  source: standards/AUDIT_GUIDANCE.md Tier 3.8 (L2-P2 + L2-P4 rules)')
    print('=' * 72)
    print()

    missing = []
    all_citations: List[Tuple[str, int, str, str]] = []

    for rel_path in STANDARDS_DOCS:
        full = os.path.join(repo, rel_path)
        if not os.path.exists(full):
            missing.append(rel_path)
            continue
        with open(full, 'r', encoding='utf-8') as f:
            text = f.read()
        cites = extract_citations(text)
        for line_no, category, cite_text in cites:
            all_citations.append((rel_path, line_no, category, cite_text))

    if missing:
        print(f'NOTE: skipped (not present): {", ".join(missing)}')
        print()

    if not all_citations:
        print('No standards/*.md files readable — environment skip.')
        return 2

    # Distinct-source population: dedupe by (category, citation_text).
    # The walking-population for the L2-P2 rule is "distinct cited sources,"
    # not "every cite-site." A datasheet section cited 5 times across 3 docs
    # is ONE distinct source.
    distinct_sources: dict = {}  # (category, cite_text) -> list of (doc, line)
    for doc, line_no, category, cite_text in all_citations:
        key = (category, cite_text)
        distinct_sources.setdefault(key, []).append((doc, line_no))

    # Sort: by category, then by citation text.
    sorted_keys = sorted(distinct_sources.keys(), key=lambda k: (k[0], k[1]))

    # Group by category for readable output.
    by_category: dict = {}
    for cat, cite in sorted_keys:
        by_category.setdefault(cat, []).append(cite)

    print(f'Total cite-sites:          {len(all_citations)}')
    print(f'Total distinct sources:    {len(distinct_sources)}')
    print(f'Citation categories:       {len(by_category)}')
    print()

    # Detailed inventory by category.
    for category in sorted(by_category.keys()):
        cites = by_category[category]
        print(f'--- {category} ({len(cites)} distinct) ---')
        for cite in cites:
            sites = distinct_sources[(category, cite)]
            site_str = ', '.join(f'{os.path.basename(d)}:{ln}' for d, ln in sites[:5])
            if len(sites) > 5:
                site_str += f', +{len(sites)-5} more'
            print(f'  {cite}  [{len(sites)} site{"s" if len(sites)!=1 else ""}: {site_str}]')
        print()

    return 0


if __name__ == '__main__':
    sys.exit(main())
