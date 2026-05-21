#!/usr/bin/env python3
"""L2-P3 verification: walk the citation inventory and verify each
citation against its source.

Companion to `scripts/audit/list_citations.py` which produces the
inventory. This script consumes the same population and verifies each
distinct source per its category:

  Phase 1 - URL liveness
    Every https?:// citation in standards/*.md is HEAD-checked via
    curl. Reports ALIVE / REDIRECT / BOT-BLOCKED / STALE-4XX /
    TRANSIENT-5XX / UNREACHABLE. Browser-like User-Agent + follow
    redirects + ignore-cert-on-failure-retry.

  Phase 2 - File-path liveness
    Every `docs/*.md` citation must point to a real file. Every
    `file.cpp:NNN` citation must point to a real file at HEAD with at
    least NNN lines.

  Phase 3 - Internal cross-doc references
    LL Entry N - check `.claude/LESSONS_LEARNED.md` has `## Entry N`.
    IVP-N - check `docs/IVP.md` references this IVP.
    R-N findings - check `docs/PROBLEM_REPORTS.md` has the row.

  Phase 4 - External PDF citation checklist (informational)
    For datasheet-section / JSF / P10 / NASA SWE citations, emit a
    checklist of `pdf_section_lookup.py` invocations the auditor can
    run to verify each PDF cite. This script does NOT auto-run them
    (PDF fetches are expensive + the helper handles caching). The
    checklist lets the auditor batch the work.

Per the L2-P3 disposition codified in standards/AUDIT_GUIDANCE.md
Tier 3.8: this script produces a deterministic verification report so
audit cycles can re-fire it mechanically against the same population.

Exit codes:
  0 = all verifications passed (or only NOTE-level findings)
  1 = at least one verification failed (stale URL, missing file,
      missing LL entry, etc.)
  2 = environment skip (network down, standards/ missing, etc.)

Companion files:
  - scripts/audit/list_citations.py (population producer)
  - scripts/audit/pdf_section_lookup.py (PDF section verifier, called
    by the Phase 4 checklist)
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from typing import Dict, List, Optional, Tuple

if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')


def _repo_root() -> str:
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.dirname(os.path.dirname(here))


# Re-use the population producer's file list + patterns
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


# ---------------------------------------------------------------------------
# Extraction (mirrors list_citations.py patterns, narrowed to the categories
# this script can verify mechanically)
# ---------------------------------------------------------------------------

_RE_URL = re.compile(r'(https?://[^\s\)\]\>]+)')
_RE_DOC_PATH = re.compile(r'`(docs/[\w/\.\-]+\.md)`')
_RE_FILE_LINE = re.compile(r'`?\b([a-zA-Z_][\w/]*\.\w+):(\d+)\b`?')
_RE_LL_ENTRY = re.compile(r'\bLL\s+Entry\s+(\d+)', re.IGNORECASE)
_RE_IVP = re.compile(r'\bIVP-(\d+[a-z]?)\b', re.IGNORECASE)
_RE_R_FINDING = re.compile(r'\bR-(\d+(?:[a-z])?(?:-[\w]+)?)\b')

# Phase 4 — categories we emit as a checklist (don't auto-verify)
_RE_DATASHEET = re.compile(r'\bdatasheet\s+§(\d+\.\d+(?:\.\d+)?)', re.IGNORECASE)
_RE_JSF_RULE = re.compile(r'JSF[\s-]*AV[\s-]*(?:C\+\+\s*)?Rule\s+(\d+)', re.IGNORECASE)
_RE_P10_RULE = re.compile(r'(?:Power[\s-]*of[\s-]*10|P10)[\s-]*Rule\s+(\d+)', re.IGNORECASE)
_RE_NASA_SWE = re.compile(r'NASA\s+SWE\s+§(\d+(?:\.\d+)?)')


def extract_citations(text: str, doc_path: str) -> Dict[str, List[Tuple[int, str, Optional[str]]]]:
    """Return {category: [(line_no, cite_text, optional_aux)]}."""
    found: Dict[str, List[Tuple[int, str, Optional[str]]]] = {
        'URL': [], 'doc-path': [], 'file-line': [], 'LL': [], 'IVP': [],
        'R-finding': [], 'datasheet-section': [], 'JSF': [], 'P10': [],
        'NASA-SWE': [],
    }
    for line_no, line in enumerate(text.split('\n'), start=1):
        for m in _RE_URL.finditer(line):
            url = m.group(0).rstrip(').,;:!?')
            found['URL'].append((line_no, url, None))
        for m in _RE_DOC_PATH.finditer(line):
            found['doc-path'].append((line_no, m.group(1), None))
        for m in _RE_FILE_LINE.finditer(line):
            # Only count if matches a source file extension AND looks
            # repo-relative (not ~/.path or absolute /usr/path). We
            # can't verify external paths like SDK installs.
            path = m.group(1)
            if not re.search(r'\.(cpp|c|h|hpp|py|sh|md|txt|json|cmake)$', path, re.IGNORECASE):
                continue
            # Skip paths inside user home / SDK installs / .pico-sdk /
            # etc. Those are documented externally and verify_citations
            # can't reach them. Look at the 3 chars before the match
            # to detect ~ or .pico-sdk-style preamble.
            ctx_start = max(0, m.start() - 30)
            ctx = line[ctx_start:m.start()]
            if '~/' in ctx or '.pico-sdk' in ctx or 'sdk/sdk/' in ctx:
                continue
            found['file-line'].append((line_no, m.group(0), path))
        for m in _RE_LL_ENTRY.finditer(line):
            found['LL'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_IVP.finditer(line):
            found['IVP'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_R_FINDING.finditer(line):
            found['R-finding'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_DATASHEET.finditer(line):
            found['datasheet-section'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_JSF_RULE.finditer(line):
            found['JSF'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_P10_RULE.finditer(line):
            found['P10'].append((line_no, m.group(0), m.group(1)))
        for m in _RE_NASA_SWE.finditer(line):
            found['NASA-SWE'].append((line_no, m.group(0), m.group(1)))
    return found


# ---------------------------------------------------------------------------
# Phase 1 — URL liveness
# ---------------------------------------------------------------------------

_BROWSER_UA = 'Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 ' \
              '(KHTML, like Gecko) Chrome/121.0.0.0 Safari/537.36'


def check_url_liveness(url: str, timeout_s: int = 10) -> Tuple[str, str]:
    """Returns (verdict, detail). Verdicts:
       ALIVE / REDIRECT / BOT-BLOCKED / STALE-4XX / TRANSIENT-5XX /
       UNREACHABLE.
    """
    # First try HEAD with browser UA, follow redirects, ignore cert
    try:
        result = subprocess.run(
            ['curl', '-sIL', '-k', '-A', _BROWSER_UA,
             '--max-time', str(timeout_s),
             '-o', os.devnull,
             '-w', '%{http_code} | hops=%{num_redirects} | final=%{url_effective}',
             url],
            capture_output=True, text=True, timeout=timeout_s + 5,
        )
        if not result.stdout:
            return ('UNREACHABLE', f'no response: {result.stderr.strip()[:100]}')
        out = result.stdout.strip()
        # Parse "<code> | hops=N | final=<url>"
        parts = out.split('|')
        if len(parts) < 1:
            return ('UNREACHABLE', f'unexpected curl output: {out}')
        code_str = parts[0].strip()
        try:
            code = int(code_str)
        except ValueError:
            return ('UNREACHABLE', f'non-numeric status: {out}')
        hops = parts[1].strip() if len(parts) > 1 else ''
        final = parts[2].strip() if len(parts) > 2 else ''
        detail = f'HTTP {code}, {hops}'
        if hops and 'hops=0' not in hops:
            detail += f', {final}'

        if 200 <= code < 300:
            if hops and 'hops=0' not in hops:
                return ('REDIRECT', detail)
            return ('ALIVE', detail)
        if code == 403 or code == 406:
            return ('BOT-BLOCKED', detail)
        if 400 <= code < 500:
            return ('STALE-4XX', detail)
        if 500 <= code < 600:
            # Retry once after brief pause
            try:
                result2 = subprocess.run(
                    ['curl', '-sIL', '-k', '-A', _BROWSER_UA,
                     '--max-time', str(timeout_s),
                     '-o', os.devnull, '-w', '%{http_code}',
                     url],
                    capture_output=True, text=True, timeout=timeout_s + 5,
                )
                if result2.stdout.strip().startswith('2'):
                    return ('ALIVE', f'recovered on retry (initial {detail})')
            except Exception:
                pass
            return ('TRANSIENT-5XX', detail)
        return ('UNREACHABLE', detail)
    except subprocess.TimeoutExpired:
        return ('UNREACHABLE', f'curl timeout >{timeout_s}s')
    except FileNotFoundError:
        return ('UNREACHABLE', 'curl not in PATH')
    except Exception as e:
        return ('UNREACHABLE', f'curl error: {e}')


# ---------------------------------------------------------------------------
# Phase 2 — File-path liveness
# ---------------------------------------------------------------------------


def check_doc_path(rel_path: str, repo: str) -> Tuple[str, str]:
    full = os.path.join(repo, rel_path)
    if os.path.exists(full):
        return ('EXISTS', rel_path)
    return ('MISSING-FILE', rel_path)


def check_file_line(cite: str, repo: str) -> Tuple[str, str]:
    m = re.match(r'`?([a-zA-Z_][\w/]*\.\w+):(\d+)`?', cite)
    if not m:
        return ('PARSE-ERROR', cite)
    path = m.group(1)
    line_no = int(m.group(2))
    full = os.path.join(repo, path)
    if not os.path.exists(full):
        return ('MISSING-FILE', cite)
    try:
        with open(full, 'r', encoding='utf-8', errors='replace') as f:
            lines = sum(1 for _ in f)
        if line_no > lines:
            return ('MISSING-LINE', f'{cite} (file has {lines} lines)')
        return ('EXISTS', cite)
    except Exception as e:
        return ('READ-ERROR', f'{cite}: {e}')


# ---------------------------------------------------------------------------
# Phase 3 — Internal cross-doc references
# ---------------------------------------------------------------------------


def _load_ll_entries(repo: str) -> set:
    full = os.path.join(repo, '.claude', 'LESSONS_LEARNED.md')
    if not os.path.exists(full):
        return set()
    nums = set()
    with open(full, 'r', encoding='utf-8') as f:
        for line in f:
            m = re.match(r'^##\s+Entry\s+(\d+)', line)
            if m:
                nums.add(m.group(1))
    return nums


def _load_ivp_refs(repo: str) -> set:
    """Return set of IVP numbers mentioned in docs/IVP.md."""
    full = os.path.join(repo, 'docs', 'IVP.md')
    if not os.path.exists(full):
        return set()
    nums = set()
    pat = re.compile(r'\bIVP-(\d+[a-z]?)\b', re.IGNORECASE)
    with open(full, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            for m in pat.finditer(line):
                nums.add(m.group(1).lower())
    return nums


def _load_r_findings(repo: str) -> set:
    """Return set of R-N IDs mentioned in PROBLEM_REPORTS.md."""
    full = os.path.join(repo, 'docs', 'PROBLEM_REPORTS.md')
    if not os.path.exists(full):
        return set()
    nums = set()
    pat = re.compile(r'\bR-(\d+(?:[a-z])?(?:-[\w]+)?)\b')
    with open(full, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            for m in pat.finditer(line):
                nums.add(m.group(1))
    return nums


# ---------------------------------------------------------------------------
# Phase 4 — PDF checklist
# ---------------------------------------------------------------------------

# Mapping: citation category -> (PDF URL, --prefix flag).
_PDF_MAP = {
    'JSF': ('https://www.stroustrup.com/JSF-AV-rules.pdf', 'Rule'),
    'P10': ('https://spinroot.com/gerard/pdf/P10.pdf', 'Rule'),
    'NASA-SWE': ('https://swehb.nasa.gov/spaces/SWEHBVC/pages/50888991/Topics+Pages',
                  '§'),  # NASA SWE is HTML pages not a single PDF; checklist is manual
    'datasheet-section': ('https://datasheets.raspberrypi.com/rp2350/rp2350-datasheet.pdf',
                           '§'),  # Default to RP2350; auditor swaps for other chips
}


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main() -> int:
    repo = _repo_root()
    print('=' * 72)
    print('  L2-P3 citation verification')
    print('  source: standards/AUDIT_GUIDANCE.md Tier 3.8')
    print('=' * 72)
    print()

    # Collect citations across all standards/*.md
    all_cites: Dict[str, List[Tuple[str, int, str, Optional[str]]]] = {}
    missing_docs = []
    for rel in STANDARDS_DOCS:
        full = os.path.join(repo, rel)
        if not os.path.exists(full):
            missing_docs.append(rel)
            continue
        with open(full, 'r', encoding='utf-8') as f:
            text = f.read()
        cats = extract_citations(text, rel)
        for cat, items in cats.items():
            for line_no, cite, aux in items:
                all_cites.setdefault(cat, []).append((rel, line_no, cite, aux))

    if missing_docs:
        print(f'NOTE: standards files missing: {missing_docs}')

    fail = 0
    note = 0

    # --- Phase 1: URL liveness ---
    urls = sorted(set(c for _, _, c, _ in all_cites.get('URL', [])))
    print(f'\n=== Phase 1: URL liveness ({len(urls)} distinct) ===')
    for url in urls:
        verdict, detail = check_url_liveness(url)
        symbol = {'ALIVE': '✓', 'REDIRECT': '↪', 'BOT-BLOCKED': '🛇',
                  'STALE-4XX': '✗', 'TRANSIENT-5XX': '!', 'UNREACHABLE': '?'}.get(verdict, '?')
        print(f'  [{verdict:13}] {symbol} {url}')
        if verdict in ('STALE-4XX',):
            fail += 1
        elif verdict in ('TRANSIENT-5XX', 'BOT-BLOCKED', 'UNREACHABLE', 'REDIRECT'):
            note += 1

    # --- Phase 2: Internal file paths ---
    doc_paths = sorted(set(c for _, _, c, _ in all_cites.get('doc-path', [])))
    print(f'\n=== Phase 2a: doc-path liveness ({len(doc_paths)} distinct) ===')
    for dp in doc_paths:
        verdict, detail = check_doc_path(dp, repo)
        symbol = {'EXISTS': '✓', 'MISSING-FILE': '✗'}.get(verdict, '?')
        if verdict != 'EXISTS':
            print(f'  [{verdict:13}] {symbol} {dp}')
            fail += 1
    print(f'  ({len([dp for dp in doc_paths if check_doc_path(dp, repo)[0] == "EXISTS"])}'
          f' EXIST, {len([dp for dp in doc_paths if check_doc_path(dp, repo)[0] != "EXISTS"])} missing)')

    file_lines = sorted(set(c for _, _, c, _ in all_cites.get('file-line', [])))
    print(f'\n=== Phase 2b: file:line liveness ({len(file_lines)} distinct) ===')
    for fl in file_lines:
        verdict, detail = check_file_line(fl, repo)
        symbol = {'EXISTS': '✓', 'MISSING-FILE': '✗', 'MISSING-LINE': '✗'}.get(verdict, '?')
        if verdict != 'EXISTS':
            print(f'  [{verdict:13}] {symbol} {detail}')
            fail += 1
    print(f'  ({len([fl for fl in file_lines if check_file_line(fl, repo)[0] == "EXISTS"])}'
          f' EXIST)')

    # --- Phase 3: Internal cross-doc refs ---
    ll_universe = _load_ll_entries(repo)
    ll_cites = sorted(set(aux for _, _, _, aux in all_cites.get('LL', []) if aux))
    print(f'\n=== Phase 3a: LL Entry refs ({len(ll_cites)} distinct, '
          f'{len(ll_universe)} entries in LESSONS_LEARNED.md) ===')
    for n in ll_cites:
        if n not in ll_universe:
            print(f'  [NOT-FOUND   ] ✗ LL Entry {n} not in .claude/LESSONS_LEARNED.md')
            fail += 1
    if not any(n not in ll_universe for n in ll_cites):
        print(f'  ({len(ll_cites)} all resolve)')

    ivp_universe = _load_ivp_refs(repo)
    ivp_cites = sorted(set(aux.lower() for _, _, _, aux in all_cites.get('IVP', []) if aux))
    print(f'\n=== Phase 3b: IVP refs ({len(ivp_cites)} distinct, '
          f'{len(ivp_universe)} IVPs in docs/IVP.md) ===')
    for n in ivp_cites:
        if n not in ivp_universe:
            print(f'  [NOT-FOUND   ] ✗ IVP-{n} not in docs/IVP.md')
            fail += 1
    if not any(n not in ivp_universe for n in ivp_cites):
        print(f'  ({len(ivp_cites)} all resolve)')

    r_universe = _load_r_findings(repo)
    r_cites = sorted(set(aux for _, _, _, aux in all_cites.get('R-finding', []) if aux))
    print(f'\n=== Phase 3c: R-finding refs ({len(r_cites)} distinct, '
          f'{len(r_universe)} rows in PROBLEM_REPORTS.md) ===')
    not_found_r = [n for n in r_cites if n not in r_universe]
    for n in not_found_r:
        print(f'  [NOT-FOUND   ] ! R-{n} not in PROBLEM_REPORTS.md (may be intentional — '
              'errata workarounds, deprecated tier numbers)')
        note += 1
    if not not_found_r:
        print(f'  ({len(r_cites)} all resolve)')

    # --- Phase 4: PDF checklist (informational) ---
    print('\n=== Phase 4: PDF citation checklist (manual verification) ===')
    print('  Run these commands to verify each cited rule/section against its PDF.')
    print('  pdf_section_lookup.py caches PDFs at ~/.pdf_cache/ — re-runs are cheap.\n')
    for cat in ['JSF', 'P10', 'datasheet-section']:
        cites = sorted(set(aux for _, _, _, aux in all_cites.get(cat, []) if aux))
        if not cites:
            continue
        pdf_url, prefix = _PDF_MAP.get(cat, ('', 'auto'))
        print(f'  --- {cat} ({len(cites)} distinct) ---')
        for n in cites:
            print(f'    python scripts/audit/pdf_section_lookup.py \\\n'
                  f'      "{pdf_url}" {n} --prefix={prefix}')
        print()

    # NASA SWE is HTML, not PDF — manual browser check
    nasa_cites = sorted(set(aux for _, _, _, aux in all_cites.get('NASA-SWE', []) if aux))
    if nasa_cites:
        print(f'  --- NASA-SWE ({len(nasa_cites)} distinct) ---')
        print('    (HTML pages, not PDF — verify in browser)')
        for n in nasa_cites:
            print(f'    https://swehb.nasa.gov/display/SWEHBVC/{n}+-+...  '
                  f'(check Topics page TOC for canonical URL slug)')
        print()

    # --- Summary ---
    print('=' * 72)
    print(f'  Summary: {fail} failures + {note} notes')
    print('=' * 72)

    if fail > 0:
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
