# Starcom audits

Starcom-owned. Extract-ready with the library. Not Rocket-Chip `docs/audits/` and not a firmware gate. This procedure covers the Starcom library only. It never replaces the RC master audit (`standards/AUDIT_GUIDANCE.md`), and a Starcom report says so in its first line.

This folder is the house-standards remainder after IVP 23's initial camelBack pass. Blue Book claims live in `../CONFORMANCE.md`, not here.

| File | What |
|------|------|
| [`IVP23_REPORT.md`](IVP23_REPORT.md) | Living Grey / JSF 151 map + NASA identifier resolution. |
| [`IVP23_TIDY_RUN.md`](IVP23_TIDY_RUN.md) | First host clang-tidy run (core only), 2026-08-31, on the pre-camelBack fork. |
| `CLANG_TIDY_2026-08-31_core.txt` | Raw tidy log from that run. |
| [`STARCOM_AUDIT_2026-10-29.md`](STARCOM_AUDIT_2026-10-29.md) | 2026-10-29 Starcom audit. Section 1: Tiers 1–3 coding-standards findings (JSF AV / P10 / JPL / house), pinned clang-tidy 21.1.8; Tier 1 failed (`.clang-tidy` comment defect, Windows header filter), Tier 3 provisional. Section 2: triage and code-fix plan. Section 3: CCSDS conformance walk (Duke), parked until the FSK pivot, with all 549 clause verdicts and the step rules C-1..C-14. Section 4: Claude Code review RV-1..RV-12 dispositions. |

Tools: `starcom/.clang-tidy`, `starcom/scripts/run_clang_tidy.ps1` (and `.sh`). Must pass `--config-file starcom/.clang-tidy`. Do not call RC `scripts/audit/full_tree_clang_tidy.sh`.

## Starcom audit procedure

Starcom runs the consumer project's master audit tiers where they fit a sans-I/O host library, plus its own CCSDS conformance step. Tier numbers below are the master guide's. "N/A (inference)" means the tier targets firmware or boards, which the core does not have; it is a reading of the tier text, not a ruling.

### Tier applicability

| Tier | Starcom | Notes |
|------|---------|-------|
| 1.1 tool self-checks | Applies | Run the tidy config on a known-bad fixture (e.g. a `goto` and a recursion) and confirm each configured check fires. |
| 1.2 SPIN, 1.4 bench regex, 1.6 build parity | N/A (inference) | No SPIN models, bench script or firmware build tiers in this tree. |
| 1.3 commit/push hooks | Consumer-owned | Starcom has no hooks of its own. |
| 1.5 host ctest | Applies | `ctest` (`starcom.unit`) on a host build. |
| 2.1 baseline sweeps | Applies (tidy, lizard) | No cppcheck gate exists for Starcom (gap). Stack: GCC `-Wstack-usage=1024` in `cmake/CompilerWarnings.cmake`. |
| 2.2 SPIN, 2.3 Pico toolchain drift, 2.4 firmware build-system | N/A (inference) | Record the host compiler and clang-tidy versions instead. |
| 2.5 / 2.5a deviations | Applies | Owner 2026-09-02: zero Starcom rows in the consumer's accepted-deviations register (`IVP23_REPORT.md`). |
| 2.6 prior-cycle delta | Applies | Prior cycle: IVP 23 (`IVP23_TIDY_RUN.md`, 2026-08-31, 328 diagnostics, core only, pre-camelBack). |
| 3.1–3.4, 3.8 coding catalogs | Applies | JSF AV / Power of 10 / JPL / house rules as they reach the core and adapters. |
| 3.5–3.7 (agent behavior, dev code, version strings) | Mostly N/A (inference) | No dev-tier code or serial banner in the core. Check `include/starcom/version.hpp.in` for version-string drift. |
| 3.9 CCSDS conformance | Applies | Below. |
| 4.x runtime / fault injection, 5.x bench / pre-flight | N/A for the core (inference) | Board behavior is the consumer's audit. |
| 6–7 drift, disposition | Applies | Reports and raw logs go in this folder. |

### Tooling

- Config: `starcom/.clang-tidy`, always passed as `--config-file`. Scripts: `scripts/run_clang_tidy.ps1` / `.sh`. Do not call the consumer's full-tree tidy script.
- `-All` / `--all` adds `adapters/` only. Neither mode scans `tests/`. Both write their log into this folder, so run them in a scratch checkout.
- Host test link: `tests/CMakeLists.txt` links with `-Wl,--wrap=malloc|calloc|realloc` (D-5 heap trap). That needs a GNU-ld-compatible linker (MinGW g++ on Windows). clang++ with lld-link fails with `undefined symbol: __real_malloc`.
- No cppcheck gate for Starcom.

### 3.9 CCSDS conformance step (G1–G13 wording)

- **Population:** every `shall` in the books `../CONFORMANCE.md` claims. Use `../SHALL_MATRIX.md` as the population, `../../standards/ccsds/navigator/data/evidence.json` as the record and `../CONFORMANCE.md` as the claim. Coverage is exhaustive, not sampled.
- **Current-issue check first:** confirm each cited book's issue on the ccsds.org Blue/Magenta/Green/Silver listing pages, and record the URL and date. A Silver book may be cited only when it is labeled Historical with a reason. A newer issue named only in a draft is "unverified" until it appears on the listing.
- **Binding vs informative:** only Blue (Recommended Standards) and Magenta (Recommended Practices) shalls are graded. Green and Yellow are informative (context only). Pink, Red and Orange are drafts or experimental and never grade a verdict; note them as "upcoming change".
- **Per shall, record:** book, issue and clause; the verbatim text; code file:line at the pinned SHA; the backing test; and the verdict. Write levels to `evidence.json` (`level`: read-in-code / host-test / board-confirmed). Never invent clause numbers. If the text is not found, the verdict is UNVERIFIED.
- **Verdicts:** **PASS** (meets the shall, with a test pointer) · **PASS (doc)** (read in code only) · **DEVIATION-STATED** (already declared in `CONFORMANCE.md`/`COVERAGE.md`; cite the line) · **FAIL (gap)** (a mandatory shall unmet and undeclared) · **N/A** (optional item not claimed, or not applicable; cite why) · **UNVERIFIED**.
- **Deviations:** an accepted CCSDS non-conformance is a stated deviation row in `../CONFORMANCE.md` (plus the 133.0 PICS exceptions field). It does not go in the consumer's coding-standards deviation register.
- **Claim sync:** each `CONFORMANCE.md` row must agree with its `COVERAGE.md` section and PICS table. A test pointer must sit in `starcom/tests/` to count.
- **Pinning:** every verdict cites a pushed commit, never uncommitted edits.
- **Independence:** the standards reviewer grades, a second agent supplies code pointers, and a third records evidence. The author of a claim does not grade it alone.
- **Report name:** `CCSDS_CONFORMANCE_AUDIT_<YYYY-MM-DD>.md` in this folder. Claims stay in `CONFORMANCE.md`; the report records the walk and the dispositions.

### Rules added 2026-10-29

- **CCSDS step parked.** The CCSDS conformance work is parked until the FSK pivot (Nathan, 2026-10-29). Where it stands: `STARCOM_AUDIT_2026-10-29.md`, Section 3.
- **CCSDS step rules.** Duke's rules C-1..C-14 (`STARCOM_AUDIT_2026-10-29.md`, Section 3.4) apply when the work resumes. C-1..C-12 and C-14 are adopted. C-13 (only `starcom/tests/` counts as test evidence) is proposed, pending Nathan's call on the 53 Space Packet service rows (parked). Until then, the claim-sync bullet above still says a test pointer must sit in `starcom/tests/` to count.
- **Claim sync by changed files.** Each claim row lists the source files it cites. Re-read every claim whose cited files changed since the claim's last-checked commit (`git log <last-checked>..HEAD -- <files>`). A phrase grep for known-stale strings is not a claim sync. This needs a "last-checked commit" column on each claim row in `../CONFORMANCE.md` and `../COVERAGE.md`; the rows do not have one yet.
- **Pace.** Audit work is slow and hand-read, with a spot check before posting (Nathan, 2026-10-29 12:16 PM CDT). Implementation detail: one book per sitting and a 10-row PASS sample (Duke's C-11).
- **Master-guide rules apply.** The tier-status table, evidence rule, counting rule, report scope rule, planted-defect probe and the "Audit integrity rules" section of `standards/AUDIT_GUIDANCE.md` apply to every Starcom report.
- **Fail means fix (Nathan, 2026-10-29).** Every FAIL is REMEDIATE. Starcom has no deviation-register rows, so no FAIL is accepted or deferred.
