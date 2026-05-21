#!/usr/bin/env python3
"""Generic helper: look up a section number in a (potentially massive) PDF.

Use cases:
  - RP2350 datasheet section currency check (e.g., is §12.4.3 still
    where the ADC pad IE behavior is documented?)
  - SX1276 datasheet section check
  - JSF AV / P10 / JPL rule-number existence in their respective PDFs
  - Any future vendor doc the project cites

Designed to be useful for many one-off verification jobs without
requiring each job to write its own PDF parser. Per the "reusable
single-use scripts" memory: a single-use job can become infrastructure
if generalized at low cost.

This script does NOT verify section content matches expected wording
beyond a substring match on the title line. For content-matching the
auditor reads the PDF directly.

What this DOES verify mechanically:
  - The PDF is reachable (URL alive OR local file exists).
  - The PDF has extractable text.
  - The cited section number appears in the PDF's table-of-contents
    or section headings.
  - Optionally: the title line near the section number contains the
    expected title keywords.

What this does NOT do:
  - Deep content verification ("the cited paragraph says what we
    claim it says"). Read the PDF for that.
  - Page-number reconciliation (PDF page vs section page).
  - Cross-reference resolution between sections.

Usage:
  python scripts/audit/pdf_section_lookup.py <pdf_url_or_path> <section_number> [title_keywords]

Examples:
  python scripts/audit/pdf_section_lookup.py \\
    https://www.stroustrup.com/JSF-AV-rules.pdf 1
  python scripts/audit/pdf_section_lookup.py \\
    ~/.pdf_cache/rp2350-datasheet.pdf 12.4.3 ADC

Exit codes:
  0 = section found
  1 = section number not present in extracted text
  2 = environment skip (PDF unreachable / extraction failed / pypdf not installed)

Cache: PDFs downloaded by URL are cached at ~/.pdf_cache/ keyed by
filename. Re-runs skip the download. Use --no-cache to force fresh
fetch.
"""

from __future__ import annotations

import argparse
import hashlib
import os
import re
import subprocess
import sys
from typing import List, Optional, Tuple

# Reconfigure stdout for UTF-8 — PDF text often contains §, em-dash,
# smart quotes that crash cp1252 on Windows.
if hasattr(sys.stdout, 'reconfigure'):
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')


_CACHE_DIR = os.path.join(os.path.expanduser('~'), '.pdf_cache')


def _ensure_cache_dir() -> None:
    os.makedirs(_CACHE_DIR, exist_ok=True)


def _cache_path_for(url: str) -> str:
    _ensure_cache_dir()
    # Use URL hash + the original filename to disambiguate (so two
    # different URLs that end in the same filename don't clobber each
    # other).
    h = hashlib.md5(url.encode('utf-8')).hexdigest()[:8]
    name = url.rstrip('/').split('/')[-1].split('?')[0]
    if not name.lower().endswith('.pdf'):
        name = name + '.pdf'
    return os.path.join(_CACHE_DIR, f'{h}_{name}')


def fetch_pdf(url: str, no_cache: bool = False) -> Optional[str]:
    """Fetch PDF from URL, cache locally. Returns local path or None on failure."""
    cache_path = _cache_path_for(url)
    if not no_cache and os.path.exists(cache_path) and os.path.getsize(cache_path) > 1024:
        return cache_path
    # Use curl — handles SSL cert quirks better than urllib for this use
    # case (per feedback_curl_more_reliable_than_webfetch memory).
    print(f'  Fetching {url} -> {cache_path}', file=sys.stderr)
    try:
        result = subprocess.run(
            ['curl', '-sL', '-k', '--max-time', '60',
             '-o', cache_path, '-w', '%{http_code}',
             url],
            capture_output=True, text=True, timeout=120,
        )
        status = result.stdout.strip()
        if status.startswith('2') and os.path.exists(cache_path) and os.path.getsize(cache_path) > 1024:
            return cache_path
        print(f'  Fetch failed: HTTP {status}', file=sys.stderr)
        return None
    except (subprocess.TimeoutExpired, FileNotFoundError) as e:
        print(f'  Fetch error: {e}', file=sys.stderr)
        return None


def extract_text(pdf_path: str) -> Optional[str]:
    """Extract text from PDF using pypdf if available."""
    try:
        from pypdf import PdfReader
    except ImportError:
        print('  pypdf not installed — pip install pypdf', file=sys.stderr)
        return None
    try:
        reader = PdfReader(pdf_path)
        chunks = []
        for page in reader.pages:
            try:
                chunks.append(page.extract_text() or '')
            except Exception as e:
                # Some pages can throw; skip and continue.
                chunks.append(f'[page-extract-error: {e}]')
        return '\n'.join(chunks)
    except Exception as e:
        print(f'  PDF read error: {e}', file=sys.stderr)
        return None


def search_section(text: str, section: str,
                    title_keywords: Optional[str] = None,
                    prefix: Optional[str] = None) -> Tuple[bool, List[str]]:
    """Search for section number in extracted text. Returns (found, context_lines).

    Args:
      text: extracted PDF text
      section: section/rule number (e.g., "12.4.3" or "1")
      title_keywords: optional space-separated keywords to cross-check
      prefix: required prefix word before the section number, e.g.:
        - "Rule" for "JSF AV Rule N", "AV Rule N", or "Rule N"
        - "§" for datasheet sections
        - None (default) tries multiple patterns

    Search patterns when prefix is None:
      1. "§N.N.N"
      2. "Section N.N.N"
      3. "Sec. N.N.N"
      4. Line-start "N.N.N TITLE..." (catches TOC entries / section headers)

    Search patterns when prefix is "Rule":
      1. "AV Rule N" (JSF AV style)
      2. "Rule N:" (P10 style)
      3. "Rule N\\b" (generic)

    For loose-match risk (e.g. "Section 1" matching every "1" in the
    doc): pass --prefix=Rule when you want to verify a numbered rule
    in a standards PDF, not a generic document section.
    """
    contexts: List[str] = []
    sec_escaped = re.escape(section)

    if prefix == 'Rule':
        # Strict rule-number patterns
        patterns = [
            re.compile(r'(?:JSF\s+)?AV\s+Rule\s+' + sec_escaped + r'\b(?!\d)', re.IGNORECASE),
            re.compile(r'\bRule\s+' + sec_escaped + r'[:\s]', re.IGNORECASE),
            re.compile(r'(?:^|\n)\s*' + sec_escaped + r'\.\s+Rule[:\s]', re.IGNORECASE),
        ]
    elif prefix == '§':
        patterns = [
            re.compile(r'§\s*' + sec_escaped + r'\b'),
            re.compile(r'\bSection\s+' + sec_escaped + r'\b', re.IGNORECASE),
            re.compile(r'\bSec\.\s*' + sec_escaped + r'\b', re.IGNORECASE),
        ]
    else:
        # Default: try multiple
        patterns = [
            re.compile(r'§\s*' + sec_escaped + r'\b'),
            re.compile(r'(?:^|\n)\s*' + sec_escaped + r'\s+[A-Z]'),  # heading: "12.4.3 ADC ..."
            re.compile(r'\bSection\s+' + sec_escaped + r'\b', re.IGNORECASE),
            re.compile(r'\bSec\.\s*' + sec_escaped + r'\b', re.IGNORECASE),
        ]

    for pat in patterns:
        for m in pat.finditer(text):
            start = max(0, m.start() - 50)
            end = min(len(text), m.end() + 200)
            ctx = text[start:end].replace('\n', ' ').strip()
            contexts.append(ctx)
            if len(contexts) >= 5:
                break
        if contexts:
            break

    if not contexts:
        return (False, [])

    if title_keywords:
        keywords = [k.lower() for k in title_keywords.split()]
        for ctx in contexts:
            ctx_lower = ctx.lower()
            if any(k in ctx_lower for k in keywords):
                return (True, contexts)
        return (True, contexts)
    return (True, contexts)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                      formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('pdf', help='PDF URL or local path')
    parser.add_argument('section', help='Section number to look up (e.g., 12.4.3)')
    parser.add_argument('title', nargs='?', default=None,
                        help='Optional space-separated title keywords to cross-check')
    parser.add_argument('--prefix', choices=['Rule', '§', 'auto'], default='auto',
                        help='What kind of citation to search for: "Rule" for '
                             'numbered rules (JSF AV Rule N, P10 Rule N), "§" for '
                             'datasheet/handbook sections (default tries both)')
    parser.add_argument('--no-cache', action='store_true',
                        help='Force fresh fetch even if cached')
    args = parser.parse_args()
    prefix = None if args.prefix == 'auto' else args.prefix

    # Resolve pdf to local path
    if args.pdf.startswith('http://') or args.pdf.startswith('https://'):
        pdf_path = fetch_pdf(args.pdf, no_cache=args.no_cache)
        if pdf_path is None:
            print(f'SKIP: could not fetch {args.pdf}')
            return 2
    else:
        pdf_path = os.path.expanduser(args.pdf)
        if not os.path.exists(pdf_path):
            print(f'SKIP: file not found: {pdf_path}')
            return 2

    text = extract_text(pdf_path)
    if text is None:
        return 2

    found, contexts = search_section(text, args.section, args.title, prefix=prefix)
    if not found:
        print(f'NOT-FOUND: §{args.section} not present in PDF text extraction')
        if args.title:
            print(f'  (title keywords were: {args.title})')
        return 1

    print(f'FOUND: §{args.section} in {os.path.basename(pdf_path)}')
    for i, ctx in enumerate(contexts[:3], 1):
        print(f'  [{i}] ...{ctx}...')
    if args.title:
        keywords = [k.lower() for k in args.title.split()]
        text_lower = ' '.join(contexts).lower()
        matched = [k for k in keywords if k in text_lower]
        missing = [k for k in keywords if k not in text_lower]
        if missing:
            print(f'  TITLE-PARTIAL: matched keywords {matched}; missing {missing}')
        else:
            print(f'  TITLE-MATCH: all keywords found: {matched}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
