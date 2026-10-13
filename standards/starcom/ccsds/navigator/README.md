# CCSDS Proximity-1 navigator

A static, self-contained HTML page (`index.html`, works from `file://`, no network) for reading the four CCSDS books Starcom cites, and for seeing where our code cites them. Docs tooling only: nothing here is firmware or source.

Open `index.html` in a browser.

## The two-mode rule

| Mode | Contains | Does not contain |
|---|---|---|
| STANDARD (default, light theme) | CCSDS text as printed, page numbers, modal kind, Annex A (PICS) rows, cross-references printed in the books | code, statuses |
| IMPLEMENTATION (dark/amber theme, permanent banner "IMPLEMENTATION VIEW: this is our code, not the standard") | everything above, plus code citations (file:line) and evidence chips | any status not derived from `evidence.json` |

Review status shown in IMPLEMENTATION mode is derived only from `data/evidence.json`. A clause with no entry shows `unchecked`. Neither the generator nor the page decides Implemented / Not yet touched / Incorrectly done.

## Theme (light / dark)

The button at the top right switches between a light and a dark theme. The first visit follows the operating-system setting (`prefers-color-scheme`); after that the choice is remembered in `localStorage` (if the browser blocks `localStorage` on `file://`, the page still works and just follows the OS setting each time). `#th=light` or `#th=dark` in the URL forces a theme for that visit.

The theme is independent of the mode, so there are four looks: standard-light, standard-dark, implementation-light, implementation-dark. STANDARD always uses the neutral/blue accent; IMPLEMENTATION always uses the amber accent and shows the permanent banner `IMPLEMENTATION VIEW: this is our code, not the standard`. Text/background contrast is at least 4.5:1 for the text colour pairs in all four looks.

## Search

Search is offline: an inverted index is built in the page from the embedded JSON when it loads. Nothing is fetched and nothing is sent anywhere.

| Query | What it does |
|---|---|
| `3.4.5`, `3.4.5.1`, `C1.3`, `annex c` | Jumps to that clause: it is the first result and is opened in the detail panel; its sub-clauses follow, then clauses whose text mentions the number. |
| `randomizer` | Words are matched by stem, so randomizer, randomization, randomize and randomized match each other. A word that is not in the index is matched as a prefix (`rando`). |
| `frame sequence` | All words must be present (any order). If no clause has all of them, clauses with at least half of the words are shown, with a note. |
| `"frame sequence"` | Quoted: the words must be adjacent, in that order. |
| `-ldpc` | Excludes clauses containing the word. |

Results are ranked with BM25 (clause titles count three times as much as body text). Each result shows the book, clause number, title, a highlighted snippet and a badge: `shall`, `should`, `may` when the best-matching passage is a normative sentence, otherwise `clause title`, `clause text` or `Annex A row`. Under each result, `related clauses` lists the clauses it refers to and the clauses that refer to it, taken from the cross-references recorded in the data; a bold link means that clause is also in the result list. The selector next to the box limits the search to all books or to the selected book. `shall only` searches only `shall` sentences.

STANDARD mode searches CCSDS text only. IMPLEMENTATION mode also searches code-citation file paths and the cited line text, shown with a `code citation` badge after the CCSDS-text hits.

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
- Equations: a span set in a smaller size and raised or lowered against the baseline is written `^` (superscript) or `_` (subscript) before it, for example `X^32`, `M_0`, `M_{n-33}` (braces when it has more than one character). Only the position taken from the PDF is used. These four characters are the only characters added to the PDF text.
- Display equations that finish a shall sentence (211.2-B-3 C1.3, C1.4, C2.3) are stored with the sentence as `continues_in_next_block`; the sentence itself ends where the PDF line ends.
- Figure text: text in the figure font, rotated text, and lines set smaller than 9 pt are not extracted. A clause that has such text on its page carries `graphic_text_omitted` and the page shows "Not extracted: text inside figures or drawn tables on p. X". Example: the data-rate tables in 211.0-B-6 B1.2.6.2 and B1.2.6.3 are drawn in a small font and are not extracted.
- Known defects: ruled figure drawings can be read as a table block with the figure's labels in cell order (211.2-B-3 C2.3 for Figure C-2, and 3.4.5.2.10 for Figure 3-4); NOTE 2 of 211.2-B-3 3.4.5.2.10 starts with the word "Figure" and is typed as a caption.
- A sentence that crosses a page break is joined and recorded with its starting page (`page_end` holds the last page).
- `shall`/`should` are matched case-insensitively, `may` lower-case only. Counts of `shall` words match the raw occurrence counts for all four books.
- Shalls in front matter (Statement of Intent, Foreword) have clause `unattributed`.
- Cross-references are only explicit textual ones ("see 3.4.5", "according to ...", "reference [3]"); reference-list entries are stored verbatim and are not auto-linked to our books.
- Docs (`.md`) are not scanned for citations by default.
