# CCSDS Proximity-1 navigator

A static, self-contained HTML page (`index.html`, works from `file://`, no network) for reading the four CCSDS books Starcom cites, and for seeing where our code cites them. Docs tooling only: nothing here is firmware or source.

Open `index.html` in a browser.

## The two-mode rule

| Mode | Contains | Does not contain |
|---|---|---|
| STANDARD (default, light theme) | CCSDS text as printed, page numbers, modal kind, Annex A (PICS) rows, cross-references printed in the books | code, statuses |
| IMPLEMENTATION (dark/amber theme, permanent banner "IMPLEMENTATION VIEW: this is our code, not the standard") | everything above, plus code citations (file:line) and evidence chips | any status not derived from `evidence.json` |

Review status shown in IMPLEMENTATION mode is derived only from `data/evidence.json`. A clause with no entry shows `unchecked`. Neither the generator nor the page decides Implemented / Not yet touched / Incorrectly done.

## Books

| Id | PDF (under `standards/starcom/ccsds/`) |
|---|---|
| 133.0-B-2 | `CCSDS-133.0-B-2-EC2.pdf` (Space Packet Protocol) |
| 211.0-B-6 | `CCSDS-211.0-B-6-EC1.pdf` (Data Link Layer) |
| 211.2-B-3 | `CCSDS-211.2-B-3.pdf` (Coding and Synchronization Sublayer) |
| 211.1-B-4 | `CCSDS-211.1-B-4-EC1.pdf` (Physical Layer) |

Titles and issue/date are taken from each book's cover and Authority page.

## Regenerate

Needs Python 3 and PyMuPDF (`pip install pymupdf`). Nothing else (stdlib only).

```
python standards/starcom/ccsds/navigator/build_navigator.py
```

This rewrites `data/standard.json`, `data/implementation.json`, `index.html` and `starcom/docs/SHALL_MATRIX.md`. It never writes `data/evidence.json` if that file already exists.

Options: `--repo-root`, `--pdf-dir`, `--out-dir`, `--matrix`, `--skip-impl` (keep existing implementation.json), `--include-docs` (also scan `.md` files), `--no-html`, `--html-only` (re-embed the JSON into `index.html`; run this after editing `evidence.json`).

The page layout is in `navigator_template.html` (placeholders `__STANDARD_JSON__`, `__IMPLEMENTATION_JSON__`, `__EVIDENCE_JSON__`); `build_navigator.py` embeds the three JSON files into it to produce `index.html`.

Output is deterministic: the same PDFs give the same `standard.json`.

### When to regenerate

The standard side (`standard.json`, `SHALL_MATRIX.md`) only needs regenerating when CCSDS releases a new issue of a book (replace the PDF, update the file name in `build_navigator.py` if it changed). `implementation.json` is a scan of our code and is regenerated whenever the citations in code change.

## Data files (`data/`)

- `standard.json`: per book: id, cover title, issue/date, clause tree (number, title, page, pdf_page, parent), clause content, every sentence containing `shall` / `should` / `may` (verbatim, clause, page, kind), Annex A PICS rows as printed, explicit cross-references (exact source sentence and target), reference list entries as printed. `does_not_apply_when` and `option_tag` are `null`; humans fill them later.
- `implementation.json`: explicit clause/table/figure citations found in `src/`, `starcom/`, `test/`, `tests/`, `include/` (code file extensions only; `graphify-out`, `standards`, `docs`, `third_party`, build dirs skipped). Each record: book, target kind, target, matched text, file, line, line text, `resolved` (whether the cited clause exists in the extracted clause tree). Book mentions with no clause number are listed separately under `mentions_without_clause`.
- `evidence.json`: **the only hand-edited file.** Starts as `{}`.

### evidence.json format

Key is `"<book id> <clause>"`; value is a list of entries:

```json
{
  "211.2-B-3 3.4.5": [
    {"level": "host-test", "ref": "starcom/tests/unit/test_x.cpp:42", "date": "2026-10-03", "by": "Nathan"}
  ]
}
```

`level` is one of `read-in-code`, `host-test`, `board-confirmed`. `ref` is file:line, a test name, or a note. After editing, run `python build_navigator.py --html-only`.

## Extraction conventions

- Text is the PDF text in reading order, page headers/footers removed, whitespace collapsed, line-end hyphen joins made without a space.
- Page numbers: `page` is the printed page label ("3-9", "A-4"); `pdf_page` is the PDF page index (1-based). Both are shown. In 211.1-B-4 the front-matter pages are printed with labels 1-1 to 1-7, which repeat the body labels, so the `pdf_page` disambiguates.
- A sentence that crosses a page break is joined and recorded with its starting page (`page_end` holds the last page).
- `shall`/`should` are matched case-insensitively, `may` lower-case only. Counts of `shall` words match the raw occurrence counts for all four books.
- Shalls in front matter (Statement of Intent, Foreword) have clause `unattributed`.
- Figure text (rotated and figure-font lines) is not extracted.
- Superscripts in equations (211.2 Annex C) are flattened by the PDF text layer, e.g. X^32 reads "X32".
- Cross-references are only explicit textual ones ("see 3.4.5", "according to ...", "reference [3]"); reference-list entries are stored verbatim and are not auto-linked to our books.
- Docs (`.md`) are not scanned for citations by default.
