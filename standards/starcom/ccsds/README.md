# CCSDS Blue Books (current issues)

PDFs only. Research notes stay in `starcom/docs/`. Index and URL-only rows: [`../README.md`](../README.md).

The stored PDFs are the current issues, with the editorial changes (EC) and technical corrigenda (Cor.) incorporated, as listed on the ccsds.org Blue Books page <https://ccsds.org/publications/bluebooks/> (the hyphenated address `https://ccsds.org/publications/blue-books/` returns HTTP 404). Older issues are not stored in this repository. File names show the issue and the EC / Cor. level: `CCSDS-<book>-B-<issue>[-EC<n>][-Cor<n>].pdf`.

Last verified against ccsds.org: 2026-10-01.

## Current issues

| Book | Title | Current issue + cover date | EC / corrigendum | Stored file | Canonical ccsds.org URL | Last verified | Issue Starcom was originally written against (historical, for dev reference) |
|---|---|---|---|---|---|---|---|
| 131.0-B-6 | TM Synchronization and Channel Coding | Issue 6, April 2026 | EC 1 ("Editorial Correction 1"), May 2026. The cover and the publications-page entry do not mention EC1; only the document-control page does | `CCSDS-131.0-B-6-EC1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2026/06/131x0b6ec1.pdf> | 2026-10-01 | 131.0-B-5 (Issue 5, September 2023). `standards/starcom/ccsds/CCSDS-131.0-B-5.pdf` was stored from 2026-08-25 until this change. 211.2-B-3 itself cites 131.0-B-3 (its 1.7 [2]) |
| 133.0-B-2 | Space Packet Protocol | Issue 2, June 2020 | EC 1 October 2020; EC 2 September 2024 | `CCSDS-133.0-B-2-EC2.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/133x0b2e2.pdf> (also <https://ccsds.org/Pubs/133x0b2e2.pdf>) | 2026-10-01 | Same as current (the PDF stored on 2026-08-25 is byte-identical to the current file) |
| 211.0-B-6 | Proximity-1 Space Link Protocol - Data Link Layer | Issue 6, July 2020 | EC 1 ("Editorial Change 1"), September 2024 | `CCSDS-211.0-B-6-EC1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/211x0b6e1.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 211.1-B-4 | Proximity-1 Space Link Protocol - Physical Layer | Issue 4, December 2013 | EC 1 January 2018 | `CCSDS-211.1-B-4-EC1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/211x1b4e1.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 211.2-B-3 | Proximity-1 Space Link Protocol - Coding and Synchronization Sublayer | Issue 3, October 2019 | none (the document-control page lists no EC) | `CCSDS-211.2-B-3.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/211x2b3.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 232.0-B-4 | TC Space Data Link Protocol | Issue 4, October 2021 | Cor. 1 October 2023; EC 1 October 2024 | `CCSDS-232.0-B-4-EC1-Cor1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/232x0b4e1c1.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 232.1-B-2 | Communications Operation Procedure-1 | Issue 2, September 2010 | EC 1 December 2018; EC 2 April 2019; Cor. 1 April 2019 | `CCSDS-232.1-B-2-EC2-Cor1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/232x1b2e2c1.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 732.1-B-3 | Unified Space Data Link Protocol | Issue 3, June 2024 | EC 1 October 2024 | `CCSDS-732.1-B-3-EC1.pdf` | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2025/01/732x1b3e1.pdf> | 2026-10-01 | Same as current (stored PDF byte-identical) |
| 231.0-B-4 (not stored) | TC Synchronization and Channel Coding | Issue 4, July 2021 | EC 1 October 2024; Cor. 1 July 2026 | none | <https://ccsds.org/wp-content/uploads/gravity_forms/5-448e85c647331d9cbaf66c096458bdd5/2026/07/231x0b4e1c1.pdf> | 2026-10-01 | 231.0-B-3 (Issue 3, September 2017, Historical) was the issue read for 6.2 and 6.3.1 (cited by 211.2-B-3 as [E3]); see `starcom/docs/CONFORMANCE.md` |

The Blue Books page entry for 231.0-B-4 reads: "The current version of this document contains all updates through technical corrigendum 1, dated July 2026." The older file `https://ccsds.org/Pubs/231x0b4e1.pdf` (HTTP 200, 52 pages) has no Cor. 1 in its document-control page; it is not the current file.

## Stored PDF check (2026-10-01)

Each file was downloaded with curl (HTTP 200, `file` reports a PDF), compared by sha256 with the file already stored, and its cover and document-control page were read with a PDF text extractor.

| Stored file | Pages | sha256 | Cover | Document control / cover note | Before this change |
|---|---|---|---|---|---|
| `CCSDS-131.0-B-6-EC1.pdf` | 99 | `1049a52bb74026c0d0e8abbd5bfd40ddc85de5dba1c5a31a9cbe0d408884f957` | "CCSDS 131.0-B-6 BLUE BOOK April 2026" | "EC 1 Editorial Correction 1 May 2026" | Replaced: `CCSDS-131.0-B-5.pdf` (Issue 5, September 2023, sha256 `46e42acff79265c9e6b7b295cafcad77fcc3d142ea507feb257dbd043db370b0`), removed |
| `CCSDS-133.0-B-2-EC2.pdf` | 54 | `a7d3acaa7d8af5f306917dff454b69de187855c45a51bb76ac4636fb5b23e34e` | "CCSDS 133.0-B-2 BLUE BOOK June 2020" | EC 1 October 2020; EC 2 September 2024 | Already current; renamed from `CCSDS-133.0-B-2.pdf` |
| `CCSDS-211.0-B-6-EC1.pdf` | 165 | `59b16ea2ccb85de890c860d17a6116b365448c21fd2a2ceb2a628ea24a5a3720` | "CCSDS 211.0-B-6 BLUE BOOK July 2020" | EC 1 September 2024 | Already current; renamed from `CCSDS-211.0-B-6.pdf` |
| `CCSDS-211.1-B-4-EC1.pdf` | 36 | `d68682b2371d74a48faffe5f95ddbca85000fb3dd71c72ef31dd5e572f7d7ac0` | "CCSDS 211.1-B-4 December 2013" | EC 1 January 2018 | Already current; renamed from `CCSDS-211.1-B-4.pdf` |
| `CCSDS-211.2-B-3.pdf` | 47 | `e07ab19a7484197b49bc743fb816bd29e8d4584f05e8fb50e2ad261657e3095d` | "CCSDS 211.2-B-3 October 2019" | no EC listed | Already current; name unchanged |
| `CCSDS-232.0-B-4-EC1-Cor1.pdf` | 136 | `0892557c3f28373498c2e1a9f089cd8a63e46eddff2325c90ff1c39ea6c4d792` | "CCSDS 232.0-B-4 BLUE BOOK October 2021"; note "includes all updates through Technical Corrigendum 1, dated October 2023" | Cor. 1 October 2023; EC 1 October 2024 | Already current; renamed from `CCSDS-232.0-B-4.pdf` |
| `CCSDS-232.1-B-2-EC2-Cor1.pdf` | 79 | `526d310281fa2a462a05b2db11accce56149e235a438b17ef5528a802698f0ae` | "CCSDS 232.1-B-2 September 2010"; note "includes all updates through Technical Corrigendum 1, dated April 2019" | EC 1 December 2018; EC 2 April 2019; Cor. 1 April 2019 | Already current; renamed from `CCSDS-232.1-B-2.pdf` |
| `CCSDS-732.1-B-3-EC1.pdf` | 197 | `3d931ae1b9ffb6a9282fedcc79e619356e61f67729d1e3f9bf433f8e719420ff` | "CCSDS 732.1-B-3 BLUE BOOK June 2024" | EC 1 October 2024 | Already current; renamed from `CCSDS-732.1-B-3.pdf` |

## Not normative: pink sheets (draft Recommended Standards)

Do not cite these as requirements. They are drafts under agency review (review window 06/27/2026 to 09/25/2026 on the ccsds.org review pages).

| Draft | Date on cover | URL |
|---|---|---|
| 211.0-P-6.2 | April 2026 | <https://ccsds.org/wp-content/uploads/2026/06/211x0p62.pdf> |
| 211.1-P-4.2 | April 2026 | <https://ccsds.org/wp-content/uploads/2026/06/211x1p42.pdf> |
| 211.2-P-3.2 | April 2026 | <https://ccsds.org/wp-content/uploads/2026/06/211x2p32.pdf> |
| 131.0-P-5.1 (the pink sheet before 131.0-B-6; review 08/28/2025 to 11/26/2025) | cover reads "CCSDS 131.0-B-5.1 PINK BOOK August 2024"; its document-control page reads "Draft August 2025" | <https://ccsds.org/wp-content/uploads/2025/08/131x0p51.pdf> |

## Do-not-cite URLs

| URL | Result 2026-10-01 |
|---|---|
| `https://ccsds.org/Pubs/131x0b5.pdf` | HTTP 200, serves the SUPERSEDED 131.0-B-5 (Issue 5, September 2023) |
| `https://ccsds.org/Pubs/131x0b6.pdf` and `.../131x0b6e1.pdf` | HTTP 404 |
| `https://ccsds.org/Pubs/231x0b4e1.pdf` | HTTP 200, 231.0-B-4 without Cor. 1 (July 2026); not the current file |
| `https://ccsds.org/Pubs/231x0b4e1c1.pdf` | HTTP 404 |
| `https://ccsds.org/Pubs/133x0b2.pdf` and `.../133x0b2e1.pdf` | HTTP 404 |
| `https://ccsds.org/Pubs/211x0b5.pdf`, `https://ccsds.org/Pubs/211x2b2.pdf` (also on `public.ccsds.org`) | HTTP 404 |
| `https://ccsds.org/publications/blue-books/` | HTTP 404 |

## ccsds.org listing facts (2026-10-01)

- 131.0-B-6: the Blue Books entry (<https://ccsds.org/publications/bluebooks/entry/4803/>) and the PDF cover do not mention EC1.
- 211.1-B-4: the Blue Books description reads "This document has been reconfirmed by the CCSDS Management Council through June 2024."
- 232.1-B-2: the Blue Books description reads "This document has been reconfirmed by the CCSDS Management Council through April 2021."
- The 211.2-P-3.2 document-control page lists "CCSDS 211.2-B-4 ... Issue 4 July 2025 ... Current issue: New LDPC coding options". The Blue Books, All Active, Space Link Services and Silver Books pages list no 211.2-B-4.
- The Silver Books description of 131.0-B-5-S reads "superseded by CCSDS 131.0-B65".
- 131.0-B-6 document control: the Issue 5 row is labelled "CCSDS 131.0-B-6" in the issue table.

Index of the other listings used: Blue Books <https://ccsds.org/publications/bluebooks/>, All Active <https://ccsds.org/publications/allpubs/>, All Active and Obsolete <https://ccsds.org/publications/ccsdsallpubs/>, Silver Books <https://ccsds.org/publications/silverbooks/>, Space Link Services <https://ccsds.org/publications/sls/>, Green Books <https://ccsds.org/publications/greenbooks/>.

401.0-B (Earth stations and spacecraft) is not in this folder. URL and the reason it is a different physical layer from 211.1: [`../README.md`](../README.md) URL-only table, and `starcom/docs/DESIGN.md` note 2026-10-01.
