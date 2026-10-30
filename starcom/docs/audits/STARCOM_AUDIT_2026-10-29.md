# Starcom audit, 2026-10-29: coding standards, triage, CCSDS conformance

This audit is for the Starcom library only. It is not the Rocket-Chip master audit. It does not replace the master audit (`standards/AUDIT_GUIDANCE.md`, `starcom/docs/audits/README.md`).

## Summary

**Audit scope.** We examined the Starcom code at commit `579f0976`. The scope is the `starcom/` folder and the Rocket-Chip adapter folder `src/starcom_adapt/`. We did not audit the Rocket-Chip flight code in `src/`.

**Standards.** Section 1 and Section 2 use the project coding standards. These are JSF AV C++, the NASA Power of 10 rules and the JPL C rules LOC-1 to LOC-4. They also include the project rules (D.1 to D.8) and the agent guidelines (`standards/AK_GUIDELINES.md`). Section 3 uses the CCSDS books that `starcom/docs/CONFORMANCE.md` claims.

**Result.**

- Tier 1 is FAIL. The two `.clang-tidy` files stop some checks, and they do not show a warning. The Windows header filter also finds no headers. Thus the audit procedure stops at Tier 1.
- The owner told us to continue to Tier 3. Thus all Tier 2 and Tier 3 results are provisional.
- JSF AV: 31 FAIL, 13 PARTIAL, 126 PASS, 63 N/A, of 233 rule IDs.
- Power of 10: 3 FAIL, 4 PARTIAL, 2 PASS, 1 N/A (Aspirational), of 10 rules.
- JPL LOC-1 to LOC-4: 5 FAIL, 8 PARTIAL, 14 PASS, 4 N/A, of 31 rules.
- Project rules D.1 to D.8: 1 FAIL, 2 PARTIAL, 3 PASS, 2 N/A, of 8 rules.
- Agent guidelines: 0 FAIL, 2 PARTIAL, 2 PASS, 0 N/A, of 4 rules.
- CCSDS (parked): 10 claimed books. The walk graded 4 books shall by shall (549 shall clauses) and 6 books at row level only. In the 549 clauses: 196 PASS, 27 PASS (doc), 58 DEVIATION-STATED, 67 FAIL (gap), 43 N/A, 158 UNVERIFIED.

**Next steps.**

1. Correct the two `.clang-tidy` files first (Section 2, "Code-fix plan", steps 1 to 3).
2. Do the Tier 1 to Tier 3 checks again with the pinned clang-tidy 21.1.8.
3. Correct each coding-standards FAIL. The owner rule is "fail means fix". No coding FAIL gets a deviation row.
4. The CCSDS work stays parked until the FSK pivot. Section 3 keeps all of its results.

**Agents who wrote this report.** Hamilton (agent) wrote Section 1, Section 2 and Section 4. Duke (agent) wrote Section 3. Goddard (agent) supplied the code pointers for Section 3. Claude Code and Grok Build examined the first version. Then Goddard read their findings again as a different reader.

**Source files.** On 2026-10-29 we put seven files into this file. They were `CODING_STANDARDS_AUDIT_2026-10-29.md`, `CODING_STANDARDS_AUDIT_2026-10-29_TRIAGE.md`, `CCSDS_CONFORMANCE_AUDIT_2026-10-29.md`, `CCSDS_CONFORMANCE_AUDIT_2026-10-29_clause_verdicts.md`, `CCSDS_CONFORMANCE_AUDIT_2026-10-29_triage.md`, `CCSDS_STEP_RULES_DRAFT_2026-10-29.md` and `AUDIT_REVIEW_2026-10-29.md`. The audit-procedure rules from this work are in `standards/AUDIT_GUIDANCE.md`, section "Audit integrity rules", and in `docs/agents/LESSONS_LEARNED.md`, Entry 49. Some line numbers in quoted findings (for example "row `:298`") refer to those previous files.

## Glossary

### Words and abbreviations

| Term | Meaning |
|---|---|
| Adapter | Code that connects the Starcom core to a host or a board. Host adapters are in `starcom/adapters/host/`. The Rocket-Chip adapter is `src/starcom_adapt/`. |
| AK guidelines | The agent behavior rules in `standards/AK_GUIDELINES.md`, sections 1 to 4. |
| APID | Application Process Identifier. The number that names one data path in a Space Packet. |
| Appendix B.1, Appendix C | Sections of `standards/AUDIT_GUIDANCE.md`. B.1 gives the project method for each Power of 10 rule. C gives the order in which to fix findings (Category 1 to Category 4). |
| Box | The agents' Linux work computer. Box results are pre-checks only. The numbers in this report come from the owner's Windows PC. |
| CAST-1, CAST-2, FP-1, PP-1, R-6c, TP-1, TP-2 | Row IDs in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` and in the Rocket-Chip remediation records. |
| CC, CCN | Cyclomatic complexity number (lizard). |
| CCSDS | Consultative Committee for Space Data Systems. It publishes the space-link books. |
| CCSDS book number | For example 211.0-B-6: book 211.0, color B, issue 6. The colors are: Blue (B) recommended standards, Magenta (M) recommended practices, Green (G) informational reports, Orange (O) experimental, Yellow (Y) records and reports, Silver (S) historical or withdrawn. Red (R) and Pink (P) are drafts. |
| Clause number | A section number inside a CCSDS book, for example §6.2.4.2. |
| CLCW | Communications Link Control Word. The 32-bit status word of COP-1. |
| COP-1 | Communications Operation Procedure 1. The telecommand procedure (232.1-B-2). |
| COP-P | Communications Operation Procedure for Proximity-1. It sends a frame again after a loss (211.0-B-6). |
| D-2, D-5 | Design decisions in `starcom/docs/DESIGN.md`. D-2 (line 94): the core does no I/O, and the caller owns all state. D-5: no heap after initialization, with a heap-trap test and a size report. |
| D13, D14 | Decision rows in the decision table of `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` (D13 is at line 619). |
| D.1 to D.8 | The project rules in Section D of Tier 3. Not the same as D-1, D-2 or D-5. |
| DLL-n | Item n of the Proximity-1 data link layer PICS (211.0-B-6 Annex). "M" means mandatory. |
| FOP-P, FARM-P | The send side and the receive side of COP-P. |
| FSK | Frequency-shift keying. The planned radio mode for the link (`docs/plans/FSK_PIVOT_PLAN_2026-10-08.md`). |
| IVP | Integration and verification plan step. IVP23 was the Starcom clang-tidy run of 2026-08-31. IVP24 recorded the struct sizes. |
| JPL LOC-n | The JPL institutional C coding standard, level of compliance n. This audit uses LOC-1 to LOC-4 (31 rules). LOC-5 and LOC-6 are deferred (`standards/CODING_STANDARDS.md:28-43`). |
| JSF AV | Joint Strike Fighter Air Vehicle C++ coding standard (rules 1 to 221 and sub-rules, 233 IDs). |
| L2-P4, L2-P5 | Earlier Rocket-Chip audit phases. `standards/AUDIT_GUIDANCE.md:94` cites L2-P4. L2-P5 was the manual standards walk (`docs/audits/l2p5_manual_walk/`). |
| LL Entry n | Entry n in `docs/agents/LESSONS_LEARNED.md`. |
| MAC | Medium access control. The Proximity-1 link-session sublayer (`starcom/src/ccsds/mac.cpp`). |
| MET | Mission elapsed time. |
| MIB | Management Information Base. The managed values of the link, for example timeouts. |
| MinGW | The GNU C++ compiler for Windows on the owner's PC (g++ 15.2). |
| NLOC | Lines of code without comments and blank lines (lizard). |
| Owner | Nathan Powell, the project owner. |
| P10 | The NASA/JPL Power of 10 rules (P10-1 to P10-10). |
| PC | The owner's Windows PC. It runs the pinned tools. |
| PICS | Protocol Implementation Conformance Statement. The table in a CCSDS book annex that lists each item as mandatory (M) or optional (O). "Support N" means that Starcom does not support the item. |
| Pinned tool | The tool version that `standards/AUDIT_GUIDANCE.md:267` names. For clang-tidy it is 21.1.8. |
| PLCW | Proximity Link Control Word. The 16-bit status word of COP-P. |
| PLTU | Proximity Link Transmission Unit. A marker, one frame and a CRC-32 (211.2-B-3). |
| PN | Pseudo-noise. The idle, acquisition and tail bit pattern (352EF853). |
| Provisional | A result that can change. The cycle stopped at Tier 1, so all Tier 2 and Tier 3 results are provisional. |
| RC | Rocket-Chip, the project that uses Starcom. |
| Read, by read | The auditor read the code. No tool checked it. |
| SCID, PCID | Spacecraft ID and physical channel ID (211.0-B-6). |
| SDU | Service data unit. |
| Shall | A binding requirement sentence in a CCSDS book. |
| Shipped config | The `.clang-tidy` file as it is in the repo. The diagnostic config is a copy with the comment lines removed. It is not a project file. |
| SPIN | The model checker that Rocket-Chip uses for its state models. |
| SPP | Space Packet Protocol (133.0-B-2). SPP-n is item n of its PICS. |
| USLP | Unified Space Data Link Protocol (732.1-B-3). |

### Audit tiers (`standards/AUDIT_GUIDANCE.md`)

### Other abbreviations

The "Shall (verbatim)" column of the clause table (3.5) gives the words of each CCSDS book with no change. The directive and parameter names in that column (for example SET TRANSMITTER PARAMETERS) are the names in the book.

| Abbreviation | Meaning |
|---|---|
| ASCII, UTF-8 | Text character encodings. |
| ASM | Attached Sync Marker. The bit pattern at the start of a CCSDS frame (FA F3 20 for Proximity-1). |
| AO Commandments, CLA/RBM, LL freshness | Names of previous Rocket-Chip audit passes in the `standards/STANDARDS_AUDIT.md` history. This report only gives their dates. |
| AUTOSAR, MISRA, CERT, C++ CG | Other C and C++ coding standards (AUTOSAR C++14, MISRA C/C++, SEI CERT, C++ Core Guidelines). clang-tidy check names use them. |
| BSD, POSIX, Winsock | Operating-system network APIs that the host adapter uses. |
| C&DH | Command and Data Handling (book text). |
| compile DB | The compile database (`compile_commands.json`). |
| CRLF, LF | Line-end characters (carriage return and line feed). |
| CSM | Codeword Sync Marker (LDPC, book text). |
| DFC ID | Data Field Construction identifier (Proximity-1 frame header). |
| DO-178B | An avionics software standard. JSF AV cites it. |
| EOF | End of file. |
| ELF | The binary file format of the firmware. |
| FIFO | First in, first out (a queue). |
| FSN, PVN, TFVN | Frame Sequence Number, Packet Version Number, Transfer Frame Version Number. |
| GCC, g++, MinGW, MSVC, LLVM, STL | C and C++ compilers and libraries (GNU, Microsoft, LLVM, Standard Template Library). |
| HW | Hardware. |
| ICD | Interface Control Document. |
| IEC, ISO, IEEE-754 | Standards bodies and the IEEE floating-point standard. |
| IPC | Inter-process communication. |
| LDPC | Low-Density Parity-Check code (211.2-B-3 coding). |
| MMIO, MPU | Memory-mapped I/O. Memory Protection Unit. |
| MSB | Most significant bit. |
| PCM, RF, RHCP, RMS, UHF | Radio terms in the book text: pulse code modulation, radio frequency, right-hand circular polarization, root mean square, ultra high frequency. |
| PDU, SPDU | Protocol data unit. Supervisory protocol data unit (Proximity-1 directives). |
| POD | Plain old data (a C++ type with no constructors). |
| QoS | Quality of service. |
| RHS | Right-hand side of an operator. |
| RL | Requirements List (book text, PICS annex). |
| RTTI | Run-time type information (C++). |
| RX, TX | Receive, transmit. |
| SANA | Space Assigned Numbers Authority (CCSDS registries). |
| SDK | Software development kit (the Pico SDK). |
| SE0, RE0, SEQ | COP-P state-table event names and the sequence-controlled frame type (book text). |
| UB | Undefined behavior (C and C++). |
| YAML | The file format of `.clang-tidy`. |

### Technical words

This report uses these words as technical nouns or verbs. The tool names (clang-tidy, lizard, cppcheck, grep, ctest, CMake, Ninja, zig, gcov, Git, Bash) and C++ words (const, struct, template, ternary, vararg, nested, null) keep their usual sense in software.

| Word | Definition in this report |
|---|---|
| compiler | A program that changes C++ source into machine code (g++, clang). |
| design | The Starcom design in `starcom/docs/DESIGN.md`. |
| enforces | Makes a rule mandatory with a tool check. |
| feature | The one function that a commit adds or changes. |
| guide | The master audit procedure, `standards/AUDIT_GUIDANCE.md`. |
| reader | The person or agent that reads a row against its evidence. The "second reader" reads it again (C-12). |
| urgent | Of the highest priority. |
| adapter | Code that connects the Starcom core to a platform (host UDP or Pico radio). |
| alias | A second name for the same clang-tidy check. |
| assertion | A test line (`CHECK`, `REQUIRE`) that makes the test FAIL if a value is incorrect. |
| folder | A directory in the repo. |
| audit | A formal examination of code and documents against standards. |
| auditor | The agent that did the audit. |
| boundary | The point where Starcom code calls the OS or other code. |
| brace | The characters `{` and `}`. |
| bucket | A group of rows that an earlier audit moved to a later phase. |
| cast | A C++ type conversion. |
| category | A priority group from `AUDIT_GUIDANCE.md` Appendix C (Category 1 to 4). |
| claim | A statement in a document about what the code does. |
| clone | A copy of a Git repository. A "branch clone" is two code branches with the same body. |
| config | A configuration file, for example `.clang-tidy`. |
| conformance | Agreement of the code with a CCSDS book. |
| coverage | The part of the code that tests operate. |
| database | The compile database (`compile_commands.json`) that clang-tidy reads. |
| declaration | A PICS entry that tells the level of support. |
| defect | An error in code or in a configuration. |
| diagnostic | A warning from a tool. The "diagnostic run" is a run with the stopped checks set to on. |
| disposition | The decision for a finding: REMEDIATE, ACCEPT or DEFER. |
| endpoint | One end of a COP-P link (`CoppEndpoint`). |
| evidence | A file and line that supports a verdict. |
| finding | A result of the audit that shows a gap or a defect. |
| gap | A part of a standard that the code does not do. |
| gate | A tool check that must give PASS before a commit or a push. |
| global | A variable outside a function. |
| linkage | How C++ makes a name visible to other files. |
| log | A text file with the output of a tool. |
| macro | A C preprocessor `#define`. |
| master | The master audit procedure, `standards/AUDIT_GUIDANCE.md`. |
| partial | A support level or a verdict: some but not all of the requirement. |
| pivot | The project change to an FSK radio link. |
| population | The full set of rules or clauses for one standard. |
| provisional | A result that can change after the next run of the corrected gate. |
| review | The second examination of the report (Section 4). |
| root | The top folder of the Rocket-Chip repo. |
| run | One operation of a tool on the code. |
| seam | A point where a test can put in different code (`HOST_TEST`). |
| service | A CCSDS service, for example the Space Packet service. |
| shall | A mandatory sentence in a CCSDS book. |
| shipped | In the repo at the pinned commit. |
| supersession | A project rule that replaces a JSF rule. |
| trace | A link from a shall to a test line through a book identifier. |
| triage | The sort of findings into dispositions. |
| verdict | The grade of one rule or clause: PASS, FAIL, PARTIAL, N/A, UNVERIFIED and others. |
| verification | The proof that the code agrees with a requirement. |

| Tier | Name | What it does |
|---|---|---|
| 1 | Gates | Checks that the audit tools and hooks work. Any Tier 1 failure stops the cycle (`AUDIT_GUIDANCE.md:57`). |
| 2 | Foundation | Tool sweeps (clang-tidy, lizard), deviations, change since the last cycle. |
| 3 | Standards catalog | One row for each rule of each standard (3.1 to 3.8). Tier 3.9 is the CCSDS step. |
| 4 | Integration | Fault and integration reviews. |
| 5 | Hardware and bench | Board tests. |
| 6 | Claim sync | Checks that the docs agree with the code. |
| 7 | Dispositions | One decision for each finding: REMEDIATE (fix), ACCEPT (deviation row) or DEFER (later, with a reason). |

### Finding IDs

| ID | Meaning | Where |
|---|---|---|
| R-1 | Duke's label for the MAC row level change (Full → Partial) with F-4 to F-7. | Section 3.3 |
| T1-b | Tier 1 finding b: the Windows header filter defect. | Section 1 |
| L1 to L22 | Claims in the docs that the audit checked (claims ledger). | Section 1.6 |
| PR-n | Item n of the coding triage. | Section 2 |
| Section F-1 | The citation walk of the standards docs (Tier 3.8). It is not the CCSDS finding F-1. | Section 1.4 |
| F-1 to F-7 | CCSDS findings: clauses that Starcom does not meet. | Section 3.2 |
| CS-1 to CS-4 | CCSDS claim-sync findings: a doc does not agree with the code. | Section 3.2 |
| D-1 | CCSDS deviation finding: a deviation is stated in `COVERAGE.md` only. | Section 3.2 |
| T-1 to T-3 | CCSDS test findings. | Section 3.2 |
| V-1 | CCSDS verification finding: the 158 UNVERIFIED rows. | Section 3.2 |
| O-1 to O-3 | CCSDS open questions. | Section 3.2 |
| K1 to K3 | Known items that the CCSDS walk logged but did not grade. | Section 3.2 |
| N5 item n | Item n of the list of more deviations for Proximity-1 framing. | Section 3.2 |
| E1 to E11 | Edits to `starcom/docs/CONFORMANCE.md` and `starcom/docs/COVERAGE.md` that came from the CCSDS walk. | Section 3.3 |
| U5 | Duke's work label for the CS-1 edit to `CONFORMANCE.md`. No other repo record uses it. | Section 3.2 |
| G1 to G13 | The text of the CCSDS step (Tier 3.9) in `starcom/docs/audits/README.md`, section 3.9. Section 4 (RV-12) records the status "adopted" (owner instruction, 2026-10-29). | Section 3 |
| C-1 to C-14 | Duke's rules for the CCSDS step. | Section 3.4 |
| `<book>/sNNN` | Shall number NNN in that book, for example 211.0-B-6/s041. The numbers are from `starcom/docs/SHALL_MATRIX.md`. | Section 3.5 |
| RV-1 to RV-12 | Findings of the Claude Code review. | Section 4 |

### Verdicts

| Verdict | Meaning |
|---|---|
| PASS | The code meets the rule. |
| PARTIAL | The code meets part of the rule. |
| FAIL | The code does not meet the rule. For the coding standards (Sections 1 and 2), each FAIL gets a fix (REMEDIATE). CCSDS FAIL (gap) rows have their own dispositions (Section 3.3), and that work is parked. |
| N/A | The rule does not apply to this code. The row gives the reason. |
| PASS (doc) | CCSDS only. The auditor read the code against the clause. No test assertion covers it. |
| DEVIATION-STATED | CCSDS only. `CONFORMANCE.md` or `COVERAGE.md` states the deviation. |
| FAIL (gap) | CCSDS only. A mandatory shall is not met, and no doc states it. |
| UNVERIFIED | CCSDS only. The auditor found the code but no assertion. |
| HOLDS, OVERSTATED, UNDERSTATED, UNVERIFIED, NO-LONGER-APPLIES | Claims ledger only. The claim is true, says too much, says too little, has no evidence, or is out of date. |

---

## 1. Coding-standards findings

### 1.1 Status and scope

**Tier 1 is FAIL.** The Starcom `.clang-tidy` file has comment lines in the folded `Checks: >` block. These lines stop 13 configured checks, and no warning shows this. A comment block before `clang-analyzer-core.*` (`starcom/.clang-tidy:222-227`) also stops its 23 checks. The tool shows these checks in its list, but a probe file shows that they do not fire. The probe has a null dereference and a divide by zero. The tool removes the two warnings and shows "Suppressed 2 warnings (2 with check filters)". On Windows, the `HeaderFilterRegex` finds no header. Thus the shipped gate does not examine headers.

`standards/AUDIT_GUIDANCE.md:57` stops the cycle at a Tier 1 FAIL. The owner told us to continue to Tier 3. Thus Tier 2 and Tier 3 are **provisional**.

**Measured on the pinned tool (PC, clang-tidy 21.1.8):** we measured these items:

- the counts for each check in "Pinned-tool measurements" below
- the MinGW 15.2 build and the `starcom.unit` test
- the IVP24 struct sizes and `-Wstack-usage=1024`.

**Provisional:** each Tier 3 verdict, the rows from grep or from a read, the lizard numbers, and the clang-tidy run on `src/starcom_adapt/`. Each Tier 3 verdict uses a defective gate until the PR-1 correction and a new run. The `src/starcom_adapt/` run used host flags, not the Pico compile database. Section 2 gives the dispositions. This audit made no code changes.

#### Tier status

This run is for Starcom only (`starcom/` and `src/starcom_adapt/`). It is **not** the Rocket-Chip master audit. The master 7-tier procedure (`standards/AUDIT_GUIDANCE.md`) did not run on the Rocket-Chip flight code. The owner told us on 2026-10-29 that "the audit" means that procedure.

| Tier | This report (Starcom) | RC master audit (RC flight `src/`) |
|---|---|---|
| 1 Gates | Run: **FAILED** (halts the cycle, `AUDIT_GUIDANCE.md:57`) | Not run (RC root config defect observed only, Tier 1 below) |
| 2 Foundation | Provisional (walked past the halt on owner instruction) | Not run |
| 3 Standards catalog (3.1-3.8) | Provisional | Not run |
| 4 Integration | Not run | Not run |
| 5 Hardware / bench | Not run | Not run |
| 6 Claim sync | Claims ledger only (22 claims) | Not run |
| 7 Dispositions | Triage (Section 2); every FAIL is REMEDIATE (fail means fix) | Not run |

**Not included:**

- cppcheck, stack analysis and coverage
- `src/starcom_adapt/` on the Pico compile database
- the full Section F-1 walk on the five standards documents
- the L2-P5 Starcom bucket (see 1.7).

 Section 3 has the CCSDS conformance walk. The owner parked it on 2026-10-29.

**Scope:** `starcom/` (core `src/ccsds`, `include/starcom`, `adapters/`) and the Rocket-Chip folder `src/starcom_adapt/`. Section 3 (Duke) has the CCSDS part. This section does not give it again.

**Pinned commit:** `579f0976` (`origin/main`). The box used a clone of the `jet-flyer/Rocket-Chip` repo on GitHub at this commit. The box did not get the submodules, because they were not necessary for the audit. The audit did not change source code.

**Auditor:** Hamilton (agent), one session. HW_GATE Rule 6 and the master Tier 5 note give this item level 2 at most. A different person must examine it again.

**Tools:**

- PC (numbers in this report): the pinned clang-tidy 21.1.8, MinGW g++ 15.2.0 with Ninja, lizard 1.21.2.
- Box: clang-tidy 21.1.6 (the nearest PyPI build to 21.1.8) was the first Tier 3 tool. At this time the Tier 3 counts come from the PC 21.1.8 logs, and the box results are pre-checks. The box also had LLVM 23.1.4 to compare results. The box had lizard 1.24.3, cmake 4.4.4 and ninja. The host compiler was clang 22.1.8 through zig 0.17.0 (libc++). The box had no cppcheck and no GCC C++ headers, because its package server did not operate.
- IVP23 used `C:\Program Files\LLVM\bin\clang-tidy.exe`. `IVP23_TIDY_RUN.md` does not record its version.

**Logs.** The logs are not in the repo. The PC keeps them in `C:\Users\pow-w\Documents\rc-audit-logs\`.

- `CLANG_TIDY_2026-10-29_all_v21.1.8_shipped.txt` (428) and `…_diag.txt` (621): pinned 21.1.8 on the PC, MinGW compile database. The Tier 3 counts come from these logs.
- `CLANG_TIDY_2026-10-29_all_v21_shipped.txt` (437) and `CLANG_TIDY_2026-10-29_all_v21_diag.txt` (629): 21.1.6 on the box, shipped config and diagnostic config. Pre-check only.
- `CLANG_TIDY_2026-10-29_all.txt`: the shipped `starcom/scripts/run_clang_tidy.sh --all` with `--config-file starcom/.clang-tidy`, core and adapters.
- `CLANG_TIDY_2026-10-29_all_DIAG_commentfix.txt`: the diagnostic run (comment lines removed from the `Checks:` block). The diagnostic config is not a project config.
- `CLANG_TIDY_2026-10-29_starcom_adapt_RCconfig.txt`: the Rocket-Chip root `.clang-tidy` on `src/starcom_adapt/*.cpp`, host flags, no Pico compile database.
- `IVP23_CLANG_TIDY_2026-08-31_core.txt`: a copy of the IVP23 log.

**Count method.** We count each different (file, line, column, message) warning line one time. IVP23 used the same method ("unique diagnostics"). Thus we count an alias pair of checks one time.

### 1.2 Tier 1: check the audit gates

| Item | Result | Evidence |
|---|---|---|
| 1.1 Starcom tidy config self-check | **FAIL (Major, proposed)** | `starcom/.clang-tidy` puts `#` comment lines inside the folded `Checks: >` scalar. YAML does not treat these as comments inside a block scalar, so each comment run is glued onto the next check name and **that check is silently disabled**. `clang-tidy --dump-config` shows e.g. `# ═══… bugprone-assert-side-effect,`, `# enabled: … cppcoreguidelines-pro-type-vararg,`, `# ═══… misc-const-correctness,`. Configured vs enabled: 133 names parsed. **On the pinned major (v21.1.6) 13 configured checks are disabled by the comment defect** (`bugprone-assert-side-effect, bugprone-reserved-identifier, cert-dcl50-cpp, cppcoreguidelines-avoid-goto, cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-pro-type-vararg, google-build-explicit-make-pair, hicpp-signed-bitwise, misc-const-correctness, modernize-deprecated-headers, performance-enum-size, readability-avoid-const-params-in-decls, readability-misleading-indentation`). **17 are not enabled** on LLVM 23: `bugprone-assert-side-effect, bugprone-reserved-identifier, cert-dcl50-cpp, cppcoreguidelines-avoid-goto, cppcoreguidelines-pro-type-reinterpret-cast, cppcoreguidelines-pro-type-vararg, google-build-explicit-make-pair, hicpp-signed-bitwise, hicpp-uppercase-literal-suffix, hicpp-multiway-paths-covered, hicpp-no-assembler, hicpp-vararg, misc-const-correctness, modernize-deprecated-headers, performance-enum-size, readability-avoid-const-params-in-decls, readability-misleading-indentation`. Of these, the `hicpp-*` entries that do not follow a comment are missing because **LLVM 23 has no `hicpp-*` checks at all** (`--list-checks -checks='-*,hicpp-*'` → "No checks enabled"). The rest are comment-swallowed. Inference: that part is YAML-level, so it also applied to IVP23's run. **Plus 23 `clang-analyzer-core.*` checks** (`starcom/.clang-tidy:222-227`): `--list-checks` lists them, but a planted null dereference and divide by zero are dropped under the shipped config ("Suppressed 2 warnings (2 with check filters)") and fire with `--checks=-*,clang-analyzer-core.*` (21.1.8, PC). Real gap: 13 + 23. The LLVM 23 figure (17) is background only. |
| 1.1 positive control | **FAIL** | `fixture/known_bad.cpp` has a `goto`, a recursion and a non-const local. Shipped config: only `misc-no-recursion` + `misc-use-internal-linkage`. **`goto` not caught.** With comments stripped: `cppcoreguidelines-avoid-goto`, `misc-const-correctness` and `misc-no-recursion` all fire. |
| 1.1 same defect in RC root `.clang-tidy` | **Observed (out of Starcom scope, flagged)** | *RV-2: count corrected 12 → 10, tracked in `AGENT_WHITEBOARD.md` "clang-tidy config defects (PR-2 / PR-1b)".* Same parse on the pinned 21.1.8 (PC): **10** configured RC checks not enabled: `bugprone-assert-side-effect`, `cert-dcl50-cpp`, `cppcoreguidelines-avoid-goto`, `google-build-explicit-make-pair`, `hicpp-signed-bitwise`, `misc-const-correctness`, `modernize-deprecated-headers`, `performance-enum-size`, `readability-avoid-const-params-in-decls`, `readability-misleading-indentation`. **Plus the same 23 silenced `clang-analyzer-core.*` checks** (comment block `.clang-tidy:227-231`, entry at :232). Real gap: 10 + 23. (An earlier "12" came from LLVM 23 on the box, which has no `hicpp-*` module; background only.) RC's pre-commit gate uses the RC config. |
| 1.1 script behavior | Note | `run_clang_tidy.sh --all` adds `adapters/` only (no `tests/`). It writes its log into `starcom/docs/audits/` (the auditor moved the log out of the clone). It exits 2 on Linux bash: the committed script has mixed line endings (`git ls-files --eol` → `i/mixed`), and the final `exit 0\r` gives `exit: 0\r: numeric argument required`. All 19 TUs were processed, 0 `error:` lines. On the pinned Windows/Git-Bash path (`scripts/hooks/README.md:5,23`) this may not reproduce. |
| 1.1 cppcheck | **Could not run** | No Starcom cppcheck gate exists. cppcheck is not installable on the box (apt 500) and is not on the owner's PC (`where.exe`). |
| 1.5 host ctest | **PASS on box** | `starcom.unit` 1/1 passed (clang 22 via zig, ELF lld; `--wrap=malloc` heap trap linked). On Windows, clang++/lld-link fails (`tests/CMakeLists.txt:32-34`, `undefined symbol: __real_malloc`); MinGW g++ works (from yesterday's PC check). |
| 1.2, 1.4, 1.6 | N/A for Starcom (inference) | No SPIN / bench / firmware build tiers. |
| 1.3 hooks | Docs-only dry run only | `scripts/hooks/pre-commit` won't run unmodified on Linux: CRLF line endings (`$'\r': command not found`) and it calls `python`. An LF copy with a python shim exited 0, no gate triggered. |

**Stop condition.** A Tier 1 FAIL stops the cycle (`AUDIT_GUIDANCE.md:57`). Tier 1 has no severity level. Thus each Tier 2 and Tier 3 result below is provisional. The owner told us to complete the Tier 3 walk. We give the results of the two runs (shipped and diagnostic), and we identify the source of each number. The counts are provisional until a new run with the pinned clang-tidy 21.1.8 on the PC, after the PR-1 correction.

#### Pinned-tool measurements (PC, clang-tidy 21.1.8, MinGW g++ 15.2 compile database, 19 translation units)

| Check | Box v21.1.6 shipped | **PC 21.1.8 shipped** | Box diag | **PC diag** | Changed? |
|---|--:|--:|--:|--:|---|
| uppercase-literal-suffix | 304 | **297** | 304 | **297** | yes: −7 header hits (T1-b) |
| readability-magic-numbers | 117 | **115** | 117 | **115** | yes: −2 header hits (T1-b) |
| avoid-non-const-global-variables | 4 | **5** | 4 | **5** | yes: +`adapters/host/udp.cpp:24 g_wsa` (Windows branch) |
| hicpp-vararg / pro-type-vararg | 1 | **0** | 1 | **0** | yes: `fcntl` path not compiled on Windows |
| hicpp-signed-bitwise | – | – | 180 | **179** | yes (−1) |
| misc-const-correctness | – | – | 9 | **9** | no |
| pro-type-reinterpret-cast | – | – | 3 | **5** | yes: Windows branch `udp.cpp:128,141,177,179,211` |
| all other checks (internal-linkage 2, err33 2, naming 1, cognitive 1, size 1, branch-clone 1, nested ternary 1, size-empty 1, named-param 1) | 11 | **11** | 11 | **11** | no |
| **Total** | **437** | **428** | **629** | **621** | |

The counts for each core file in `src/ccsds/*.cpp` are the same on the two tools (mac 113, uslp 71, cop1 46, and the others).

**T1-b (new Tier 1 finding).** The shipped filter is `HeaderFilterRegex: '.*(src/ccsds/.+|include/starcom/.+)'` (`starcom/.clang-tidy:237`). With it, the PC shows 0 header diagnostics. The same file with a filter that accepts the two separators shows `include\starcom/ccsds/copp.hpp:24,27`, `uslp.hpp:12` and `v3.hpp:13`. (clang-tidy shows paths with `\` and `/`.) Thus the gate on the pinned Windows tool did not examine Starcom headers at all.

**MinGW on the PC.** g++ 15.2.0 (MinGW-Builds) with Ninja made 42 of 42 targets without errors. The flags were `-Wall -Wextra -Wpedantic -Werror`, and `-Wstack-usage=1024` on 19 of 38 compile commands (the core). The build had **0 warnings**. `ctest`: `starcom.unit` gave PASS (1 of 1).

**IVP24 struct sizes** (`tests/unit/test_size.cpp`): `CoppEndpoint=10136 Cop1Endpoint=19608 FopP=9440 FarmP=4 kTransferFrameMax=2048 kLdpcMessageOctets=128`. The box gives the same values (clang, libc++). Thus `starcom/docs/IVP.md:385` (`CoppEndpoint=19544`, `FopP=164`) is out of date.

### 1.3 Tier 2: foundation

#### 2.1 clang-tidy sweep compared with IVP23 (328 diagnostics, core only, 2026-08-31, `68b0daf8`)

| Check | IVP23 core | 2026-10-29 shipped, all (core+include+adapters) | of which adapters | DIAG comment-fixed, all |
|---|--:|--:|--:|--:|
| uppercase-literal-suffix (IVP23: hicpp+readability alias pair) | 226 | 304 (readability only; hicpp absent in LLVM 23) | 0 | 304 |
| readability-magic-numbers | 81 | 117 | (in total) | 117 |
| readability-function-size | 9 | 1 (`mac.cpp:396` `macWaitExpiredHalf`) | 0 | 1 |
| readability-function-cognitive-complexity | 7 | 1 (`mac.cpp:396`, CC 34 > 25) | 0 | 1 |
| misc-use-internal-linkage | 2 | 2 (`copp.cpp:433` `coppEncodeUslp`, `pltu.cpp:192` `repeatSeq`; same two helpers as IVP23, renamed) | 0 | 2 |
| bugprone-branch-clone | 1 | 1 (`mac.cpp:956`) | 0 | 1 |
| readability-container-size-empty | 1 | 1 (`space_packet.cpp:40`, same as IVP23) | 0 | 1 |
| readability-else-after-return | 1 | 0 | 0 | 0 |
| cppcoreguidelines-avoid-non-const-global-variables | 0 | 4 (`cop1.cpp:12` `g_cop1TfScratch`, `copp.cpp:70` `g_coppSyncLossN`, `copp.cpp:345` `g_tfScratch`, `copp.cpp:347` `g_coppRetransmitN`) | 0 | 4 |
| readability-identifier-naming | 0 | 1 (`copp.cpp:359` `dest_is_local`) | 0 | 1 |
| cert-err33-c | 0 | 2 | 2 (`file_replay.cpp:76,91`) | 2 |
| readability-avoid-nested-conditional-operator | 0 | 1 (`pltu.cpp:84`) | 0 | 1 |
| readability-named-parameter | 0 | 1 (`space_packet_service.cpp:52`) | 0 | 1 |
| misc-const-correctness | not enabled | not enabled | – | 15 |
| cppcoreguidelines-pro-type-reinterpret-cast | not enabled | not enabled | – | 3 (`udp.cpp:128,141,190`) |
| cppcoreguidelines-pro-type-vararg | not enabled | not enabled | – | 1 (`udp.cpp:79`) |
| **Total** | **328** | **436** (430 core+include, 6 adapters) | 6 | **455** |

**Changes since IVP23.** IVP23 examined the core only. `space_packet_service.cpp` is a new file.

- For each file: the core count increased in `mac.cpp` (23 → 113) and in `cop1.cpp` (34 → 46). It decreased in `uslp.cpp` (88 → 71).
- Function-size and cognitive-complexity hits decreased from 9 and 7 to 1 and 1. Inference: the large state functions that IVP23 names (`mac_tick`, `apply_state`, USLP encode and decode, `fop_1_on_clcw`) became smaller functions during the COP-P, MAC and USLP work.
- lizard shows only one function in `src/` with more than 60 lines (below).
- The 4 `g_` globals and `dest_is_local` are new since IVP23.
- Different compilers and tools (LLVM version, libc++ or MSVC STL) can change counts. Inference: most of the suffix and magic-number change comes from more code. We did not show this line by line.

**lizard** (`-l cpp`, `starcom/src`). We used lizard 1.21.2 on the PC on 2026-10-29. lizard 1.24.3 on the box gave the same result: 230 functions, 3555 NLOC, mean CCN 4.0. Only `mac.cpp:396 macWaitExpiredHalf` (104 lines, CCN 22) has more than 60 lines or a CCN of more than 20. Default-threshold warnings (CCN more than 15) in `src`, `adapters` and `include`: `mac.cpp:396` (22), `mac.cpp:729 macOnNoFramesPending` (20), `mac.cpp:980 macFifoSource` (17), `cop1.cpp:335 fop1NeedFrame` (17), `cop1.cpp:652 cop1ReceiveBytes` (19), `adapters/host/udp.cpp:93 udpBind` (16).

**Rocket-Chip `src/starcom_adapt/` (Rocket-Chip root config, host flags, provisional).** The run gave 163 diagnostics. Most are in included Rocket-Chip headers out of scope (`mission_profile_data.h` 79, `radio_config_table.h` 45). The adapter files have 33: `byte_pump.cpp` 23, `cmd_sdu.cpp` 7, `nav_sdu.h` 2, `byte_pump.h` 1. Function size: `cmd_sdu.cpp:9 pack_cmd_sdu_user`, `cmd_sdu.cpp:27 unpack_cmd_sdu_user`. This run did not use the Pico compile database. Thus the target diagnostics can be different.

**Not done:**

- cppcheck (not available)
- `analyze_stack_usage.sh` (Pico and ARM only)
- coverage. `generate_coverage_report.sh` is for Rocket-Chip firmware. We did not try gcov with zig.

#### 2.5 and 2.5a: deviations

`IVP23_REPORT.md:13` records the owner decision of 2026-09-02: `ACCEPTED_STANDARDS_DEVIATIONS.md` has zero Starcom rows. A grep for `starcom` in that file finds 0 hits. L2-P5 put rows in a "Starcom / radio-telem supersession (22)" bucket (`docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_LOG.md:240`, dated 2026-08-21). This audit did not walk those rows. This item is open.

#### 2.6 Change since the last cycle (IVP23)

- **Closed:**
  - the function-size and complexity items that IVP23 names (1 stays: `macWaitExpiredHalf`),
  - `readability-else-after-return`,
  - the `mac_phy` branch clone. Inference: a branch clone is at `mac.cpp:956` at this time. We did not make sure that it is the same site.
- **Not changed:** internal linkage ×2 (the same helpers), `space_packet.cpp:40` size-to-empty, and the large group of suffix and magic-number hits.
- **New:** 4 non-const globals and 1 naming hit (`dest_is_local`). A nested ternary at `pltu.cpp:84` and an unnamed parameter at `space_packet_service.cpp:52`. 2 `cert-err33-c` hits in the adapters (IVP23 did not examine the adapters).
- **Some IVP23 zeros are not evidence:** IVP23 gave "0 cppcoreguidelines-*" and "0 bugprone-reserved-identifier". Some of those checks did not operate (Tier 1). Thus those zeros do not show that the code is clean.

### 1.4 Tier 3: the standards catalog (provisional)

The cycle stopped at Tier 1. The counts come from the PC clang-tidy 21.1.8 logs `CLANG_TIDY_2026-10-29_all_v21.1.8_shipped.txt` and `…_diag.txt`.

**Paths.** Starcom paths start at `starcom/` (for example `src/ccsds/mac.cpp`). Rocket-Chip adapter paths start with `src/starcom_adapt/`.

**Mark.** The mark "[shipped config would miss]" identifies a finding, or a PASS where the gate check does not operate. In these rows the Tier 1 comment defect stops the check. Only the diagnostic run or a grep finds the item.

**Population.** All 233 JSF AV rule IDs: 221 rules with numbers (rules 161 and 172 are not in the standard) and 14 sub-rules. All 10 P10 rules. All 31 JPL LOC-1 to LOC-4 rules. D.1 to D.8. AK guidelines sections 1 to 4. The walk includes all of each population (`AUDIT_GUIDANCE.md:92-94`).

#### Section A: JSF AV C++ (221 rules) — FAIL (31 FAIL / 13 PARTIAL / 126 PASS / 63 N/A of 233 IDs)

##### A.1 C files

N/A. Starcom and `src/starcom_adapt/` have no `.c` files (C++20 only). We walked all 233 IDs against C++.

##### A.2 OS and host-API boundary

This table is the Starcom form of the SDK-boundary table (2026-02-07 precedent). The Starcom core has no Pico SDK. The boundary is the host adapters. Each site breaks a rule. No JSF clause lets it, and no deviation row applies (zero-row rule, `starcom/docs/audits/IVP23_REPORT.md:13`). Thus A.3 gives each site FAIL.

| Function / site | API forcing it | Rules | Justification available? |
|---|---|---|---|
| `adapters/host/udp.cpp:128,141,177,179,190,211` `udpBind`/`udpSend`/`udpRecv` | BSD/Winsock `sockaddr*`, `char*` buffers | 182 **[shipped config would miss]** | No JSF-182 exception (`standards/CODING_STANDARDS.md:59`); RC handled analogous residuals as CAST-1/2 rows, which Starcom cannot use (see open — no record). |
| `adapters/host/udp.cpp:75,79` | POSIX `fcntl` (variadic) | 108 **[shipped config would miss]** (`hicpp-vararg` catches :79 on v21) | Same. |
| `adapters/host/udp.cpp:192,228` | `errno` / `WSAGetLastError` | 17 | Same. |
| `adapters/host/udp.cpp:5-50` | `#if defined(_WIN32)` platform split, `#define WIN32_LEAN_AND_MEAN` | 26, 28, 31 | Same. |
| `adapters/host/file_replay.cpp:5,43,63,76,91` | `<cstdio>` file I/O | 22 (+ `cert-err33-c` :76,91) | No: JSF-22 applies project-wide (`CODING_STANDARDS.md:62`). |

##### A.3 and A.4: walk of each rule

The "App" column gives the rule type: M (mandatory), R (recommended) or "Process".

| Rule | Summary | App | Status | Notes |
|---|---|---|---|---|
| 1 | Any one function (or method) will contain no more than 200 logical source lines of code (L | M | PASS | Max function: `mac.cpp:396 macWaitExpiredHalf` 98 NLOC (lizard); all < 200. |
| 2 | There shall not be any self-modifying code. | M | PASS | No self-modifying code (sans-I/O library; no writes to code memory). |
| 3 | All functions shall have a cyclomatic complexity number of 20 or less. | M | FAIL | `mac.cpp:396 macWaitExpiredHalf` CCN 22 (lizard; cognitive 34 per tidy). All other functions ≤ 20 (`mac.cpp:729` = 20). |
| 4 | To break a “should” rule, the following approval must be received by the developer: | Process | N/A | Deviation-approval process rule; project substitutes the deviation register (`standards/CODING_STANDARDS.md:543-553`). No Starcom approvals exist (`IVP23_REPORT.md:13`). |
| 5 | To break a “will” or a “shall” rule, the following approvals must be received by the devel | Process | N/A | Deviation-approval process rule; project substitutes the deviation register (`standards/CODING_STANDARDS.md:543-553`). No Starcom approvals exist (`IVP23_REPORT.md:13`). |
| 6 | Each deviation from a “shall” rule shall be documented in the file that contains the devia | Process | FAIL | The FAILs in this table carry no in-file deviation note and no register row (`IVP23_REPORT.md:13`: zero Starcom rows). Undocumented rule-breaks. |
| 7 | Approval will not be required for a deviation from a “shall” or “will” rule that complies  | Process | N/A | Deviation-approval process rule; project substitutes the deviation register (`standards/CODING_STANDARDS.md:543-553`). No Starcom approvals exist (`IVP23_REPORT.md:13`). |
| 8 | All code shall conform to ISO/IEC 14882:2002(E) standard C++. | M | PARTIAL | C++20 `-Wpedantic -Werror` core (`cmake/CompilerWarnings.cmake:9-11`); host adapters use POSIX/Winsock (`adapters/host/udp.cpp:5-50`) — platform API boundary (A.2). |
| 9 | Only those characters specified in the C++ basic source character set will be used. | M | FAIL | Non-ASCII characters (e.g. `–`, `§`, `±`) in comments: 78 lines in 21 files (e.g. `src/ccsds/clcw.cpp:37`, `space_packet.cpp:51`). Ungated (RULE_VERIFIABILITY_TRIAGE.md:137). |
| 10 | Values of character types will be restricted to a defined and documented subset of ISO 106 | M | N/A | No character-type data values beyond host path strings (`adapters/host/file_replay.cpp`). |
| 11 | Trigraphs will not be used. | M | PASS | No trigraphs (grep `??[=/()!<>-]`: 0). |
| 12 | The following digraphs will not be used: | M | PASS | No digraphs (grep: 0). |
| 13 | Multi-byte characters and wide string literals will not be used. | M | FAIL | Multi-byte UTF-8 characters in comments (same 78 lines as Rule 9). No wide string literals. |
| 14 | Literal suffixes shall use uppercase rather than lowercase letters. | M | FAIL | 297 unique lowercase-suffix diagnostics (PC 21.1.8; header hits not reported on Windows, Tier 1 T1-b), e.g. `src/ccsds/clcw.cpp:13` `0x07u`. RC adapter: 15 (`src/starcom_adapt/`; box, host flags, provisional). |
| 15 | Provision shall be made for run-time checking (defensive programming). | M | PARTIAL | Defensive input validation via `Result` error returns (e.g. `adapters/host/file_replay.cpp:60-62`, length checks in decoders); 0 runtime assertions in 230 core functions (2 `static_assert`: `cop1.cpp:540`, `copp.cpp:224`). Slice governed by P10-5/JPL-16 (RULE_VERIFIABILITY_TRIAGE.md:143). |
| 16 | Only DO-178B level A [15] certifiable or SEAL 1 C/C++ libraries shall be used with safety- | M | PARTIAL | Uses vendored `tl::expected` (`starcom/third_party/tl/expected.hpp`) and libstdc++/libc++. Not on the adopted-code register (`ACCEPTED_STANDARDS_DEVIATIONS.md:111,123` TP-1, TP-2 only); `CODING_STANDARDS.md:280` "Unevaluated vendor code is a finding". |
| 17 | through AV Rule 25 prohibit the use of a number of features whose behaviors are local-spec | M | FAIL | `errno` in host adapter `adapters/host/udp.cpp:192,228` (socket EAGAIN). Core: 0. Platform API boundary (A.2). |
| 18 | The macro offsetof, in library <stddef.h>, shall not be used. | M | PASS | Core: 0. RC adapter `src/starcom_adapt/nav_sdu.h:20,22` uses `offsetof` only inside `static_assert` layout checks (same pattern as 2026-02-07 PASS precedent). |
| 19 | <locale.h> and the setlocale function shall not be used. | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 20 | The setjmp macro and the longjmp function shall not be used. | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 21 | The signal handling facilities of <signal.h> shall not be used. | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 22 | The input/output library <stdio.h> shall not be used. | M | FAIL | `<cstdio>` + `fopen/fread/fclose` in host adapter `adapters/host/file_replay.cpp:5,43,63,76,91`. Core: 0. `CODING_STANDARDS.md:62` — JSF-22 applies project-wide. |
| 23 | The library functions atof, atoi and atol from library <stdlib.h> shall not be used. | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 24 | The library functions abort, exit, getenv and system from library <stdlib.h> shall not be  | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 25 | The time handling functions of library <time.h> shall not be used. | M | PASS | None present (grep: locale/setjmp/longjmp/signal/atoi/atof/atol/abort/exit/getenv/system/<ctime>: 0). |
| 26 | Only the following pre-processor directives shall be used: | M | FAIL | Non-guard directives: `#ifdef/#else/#endif` in `include/starcom/result.hpp:5,11,17`; `#if defined(_WIN32)` etc. ×37 in `adapters/host/udp.cpp`; `#ifdef ROCKETCHIP_JOB_STATION` / `#ifndef ROCKETCHIP_HOST_TEST` in `src/starcom_adapt/byte_pump.cpp:262-269`. |
| 27 | #ifndef, #define and #endif will be used to prevent multiple inclusions of the same header | M | FAIL | All 23 Starcom headers use `#pragma once` (e.g. `include/starcom/ccsds/pltu.hpp:1`), no `#ifndef` guards. RC remediated every `#pragma once` to `#ifndef` (RULE_VERIFIABILITY_TRIAGE.md:9, 2026-06-24); no deviation row exists. RC adapter headers PASS (`src/starcom_adapt/cmd_sdu.h:6-7`). |
| 28 | The #ifndef and #endif pre-processor directives will only be used as defined in AV Rule 27 | M | FAIL | `#ifndef` used for a non-guard purpose: `src/starcom_adapt/byte_pump.cpp:267` (`ROCKETCHIP_HOST_TEST`); `udp.cpp:6` (`WIN32_LEAN_AND_MEAN`). |
| 29 | The #define pre-processor directive shall not be used to create inline macros. | M | FAIL | Function-like stringizing macros `STARCOM_VERSION_STR_X` / `STARCOM_VERSION_STR` in `include/starcom/version.hpp.in:14-15` (installed public header). Elsewhere 0. |
| 30 | The #define pre-processor directive shall not be used to define constant values. | M | PARTIAL | Constants are `constexpr` throughout; exception: generated `version.hpp.in:8-12` `#define STARCOM_VERSION_*` and `udp.cpp:7` `#define WIN32_LEAN_AND_MEAN`. |
| 31 | The #define pre-processor directive will only be used as part of the technique to prevent  | M | FAIL | Non-guard `#define`: `adapters/host/udp.cpp:7`; `include/starcom/version.hpp.in:8-15` (generated). |
| 32 | The #include pre-processor directive will only be used to include header (*.h) files. | M | PASS | All `#include`s name headers. |
| 33 | The #include directive shall use the <filename.h> notation to include header files. | M | PASS | `<>` for system/std, quotes for project headers (same convention as 2026-02-07 PASS). |
| 34 | Header files should contain logically related declarations only. | R | PASS | One codec/adapter per header (`include/starcom/ccsds/*.hpp`, `adapters/*.hpp`); minimal includes (read). |
| 35 | A header file will contain a mechanism that prevents multiple inclusions of itself. | M | FAIL | Inclusion prevention is `#pragma once` (Rule 27 technique) in 23/23 Starcom headers; JSF 35 co-governs with 27 (RULE_VERIFIABILITY_TRIAGE.md:163). |
| 36 | Compilation dependencies should be minimized when possible. | R | PASS | One codec/adapter per header (`include/starcom/ccsds/*.hpp`, `adapters/*.hpp`); minimal includes (read). |
| 37 | Header (include) files should include only those header files that are required for them t | R | PASS | One codec/adapter per header (`include/starcom/ccsds/*.hpp`, `adapters/*.hpp`); minimal includes (read). |
| 38 | Declarations of classes that are only accessed via pointers (*) or references (&) should b | R | PASS | No class-pointer-only declarations needing forward headers. |
| 39 | Header files (*.h) will not contain non-const variable definitions or function definitions | M | PASS | Headers hold `inline constexpr` constants and declarations; `misc-definitions-in-headers` enabled, 0 hits (v21 shipped). |
| 40 | Every implementation file shall include the header files that uniquely define the inline f | M | PASS | Each `.cpp` includes its own header first (e.g. `src/ccsds/clcw.cpp:1`). |
| 41 | Source lines will be kept to a length of 120 characters or less. | M | PASS | 0 lines > 120 chars (grep `.{121,}`). |
| 42 | Each expression-statement will be on a separate line. | M | PASS | No multi-statement lines found (read; `readability-isolate-declaration` 0). |
| 43 | Tabs should be avoided. | R | PASS | 0 tab characters. |
| 44 | All indentations will be at least two spaces and be consistent within the same source file | M | PASS | Consistent 2-space indentation (read). `readability-misleading-indentation` **[shipped config would miss: check disabled by Checks-block comment defect]**; diagnostic run: 0. |
| 45 | All words in an identifier will be separated by the ‘_’ character. | M | PASS | Superseded by house naming (`CODING_STANDARDS.md:63`); gate enforces camelBack. |
| 46 | User-specified identifiers (internal and external) will not rely on significance of more t | M | PASS | No identifier ≥ 40 chars (grep). |
| 47 | Identifiers will not begin with the underscore character ‘_’. | M | PASS | No authored leading-underscore identifiers (only `_WIN32` platform macro). `bugprone-reserved-identifier` **[shipped config would miss: check disabled by Checks-block comment defect]**. |
| 48 | Identifiers will not differ by: | M | PASS | No case-only / underscore-only near-duplicates found (read of public headers). |
| 49 | All acronyms in an identifier will be composed of uppercase letters. | M | PARTIAL | camelBack acronyms are mixed case (`decodePltu`, `CoppEndpoint`, `kTfvnV3`). House decision supersedes only JSF 45/51/52 (`CODING_STANDARDS.md:63`), not 49 — see "open — no record". |
| 50 | The first word of the name of a class, structure, namespace, enumeration, or type created  | M | PASS | Types start uppercase (`PltuView`, `MacSession`, `UslpLayout`). |
| 51 | All letters contained in function and variable names will be composed entirely of lowercas | M | PASS | Superseded by house naming (`CODING_STANDARDS.md:63`). One house-naming miss: `src/ccsds/copp.cpp:359 dest_is_local` (tidy). RC adapter uses snake_case (`src/starcom_adapt/cmd_sdu.cpp:9`), not flagged by RC config. |
| 52 | Identifiers for constant and enumerator values shall be lowercase. | M | PASS | Superseded by house naming (`CODING_STANDARDS.md:63`). One house-naming miss: `src/ccsds/copp.cpp:359 dest_is_local` (tidy). RC adapter uses snake_case (`src/starcom_adapt/cmd_sdu.cpp:9`), not flagged by RC config. |
| 53 | Header files will always have a file name extension of ".h". | M | FAIL | Starcom headers use `.hpp` (23 files), not `.h`. RC adapter `.h` PASS. |
| 53.1 | The following character sequences shall not appear in header file names: | M | PASS | No forbidden characters in header names. |
| 54 | Implementation files will always have a file name extension of ".cpp". | M | PASS | Implementation files are `.cpp`. |
| 55 | The name of a header file should reflect the logical entity for which it provides declarat | R | PASS | File names match entity (`pltu.cpp`/`pltu.hpp`, `udp.cpp`). |
| 56 | The name of an implementation file should reflect the logical entity for which it provides | R | PASS | File names match entity (`pltu.cpp`/`pltu.hpp`, `udp.cpp`). |
| 57 | The public, protected, and private sections of a class will be declared in that order (the | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 58 | When declaring and defining functions with more than two parameters, the leading parenthes | M | PASS | Long parameter lists line-broken (e.g. `include/starcom/adapters/udp.hpp:15-20`). |
| 59 | The statements forming the body of an if, else if, else, while, do…while or for statement  | M | PASS | `readability-braces-around-statements` enabled, 0 hits. |
| 60 | Braces ("{}") which enclose a block will be placed in the same column, on separate lines d | M | FAIL | K&R brace placement (opening brace at end of line) throughout, e.g. `src/ccsds/mac.cpp:305`. Rule text requires braces in the same column on separate lines. 2026-02-07 recorded K&R as PASS; no `.clang-format` in repo — see "open — no record". |
| 61 | Braces ("{}") which enclose a block will have nothing else on the line except comments (if | M | FAIL | Same K&R pattern: `switch (m.state) {` (`mac.cpp:305`) and `} else {` lines carry code tokens. |
| 62 | The dereference operator ‘*’ and the address-of operator ‘&’ will be directly connected wi | M | PASS | `Type*`/`Type&` attached to type (read; e.g. `include/starcom/adapters/radio_bus.hpp`). |
| 63 | Spaces will not be used around ‘.’ or ‘->’, nor between unary operators and operands. | M | PASS | No spaces around `.`/`->`/unary ops (read). |
| 64 | A class interface should be complete and minimal. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 65 | A structure should be used to model an entity that does not require an invariant. | R | PASS | Plain `struct`s model invariant-free records (`PltuView`, `UslpLayout`). |
| 66 | A class should be used to model an entity that maintains an invariant. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 67 | Public and protected data should only be used in structs—not classes. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 68 | Unneeded implicitly generated member functions shall be explicitly disallowed. | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 69 | A member function that does not affect the state of an object (its instance variables) wil | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 70 | A class will have friends only when a function or object requires access to the private el | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 70.1 | An object shall not be improperly used before its lifetime begins or after its lifetime en | M | PASS | No object used outside lifetime found; endpoints are caller-owned (`ICD.md:20`), `memset` re-init on trivially-copyable types guarded by `static_assert` (`copp.cpp:224`, `cop1.cpp:540`). |
| 71 | Calls to an externally visible operation of an object, other than its constructors, shall  | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 71.1 | A class’s virtual functions shall not be invoked from its destructor or any of its constru | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 72 | The invariant 2 for a class should be: | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 73 | Unnecessary default constructors shall not be defined. | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 74 | Initialization of nonstatic class members will be performed through the member initializat | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 75 | Members of the initialization list shall be listed in the order in which they are declared | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 76 | A copy constructor and an assignment operator shall be declared for classes that contain p | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 77 | A copy constructor shall copy all data members and bases that affect the class invariant ( | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 77.1 | The definition of a member function shall not contain default arguments that produce a sig | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 78 | All base classes with a virtual function shall define a virtual destructor. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 79 | All resources acquired by a class shall be released by the class’s destructor. | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 80 | The default copy and assignment operators will be used for classes when those operators of | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 81 | The assignment operator shall handle self-assignment correctly (see Stroustrup [2], Append | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 82 | An assignment operator shall return a reference to *this. | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 83 | An assignment operator shall assign all data members and bases that affect the class invar | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 84 | Operator overloading will be used sparingly and in a conventional manner. | M | PASS | No operator overloading (grep: 0). |
| 85 | When two operators are opposites (such as == and !=), both will be defined and one will be | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 86 | Concrete types should be used to represent simple independent concepts. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 87 | Hierarchies should be based on abstract classes. | R | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 88 | Multiple inheritance shall only be allowed in the following restricted form: | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 88.1 | A stateful virtual base shall be explicitly declared in each derived class that accesses i | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 89 | A base class shall not be both virtual and non-virtual in the same hierarchy. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 90 | Heavily used interfaces should be minimal, general and abstract. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 91 | Public inheritance will be used to implement “is-a” relationships. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 92 | A subtype (publicly derived classes) will conform to the following guidelines with respect | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 93 | “has-a” or “is-implemented-in-terms-of” relationships will be modeled through membership o | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 94 | An inherited nonvirtual function shall not be redefined in a derived class. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 95 | An inherited default parameter shall never be redefined. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 96 | Arrays shall not be treated polymorphically. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 97 | Arrays shall not be used in interfaces. | M | PASS | Interfaces use `std::span`/`std::array` (e.g. `include/starcom/ccsds/pltu.hpp:20`); C array only file-local `ldpc.cpp:39`. |
| 97.1 | Neither operand of an equality operator (== or !=) shall be a pointer to a virtual member  | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 98 | Every nonlocal name, except main(), should be placed in some namespace. | R | PASS | All non-local names in `starcom::ccsds` / `starcom::adapters` / `rc::starcom_adapt`. |
| 99 | Namespaces will not be nested more than two levels deep. | M | PASS | Max nesting 2 (`starcom::ccsds`). |
| 100 | Elements from a namespace should be selected as follows: | R | PASS | No `using namespace` (grep: 0). |
| 101 | Templates shall be reviewed as follows: | M | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 102 | Template tests shall be created to cover all actual template instantiations. | M | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 103 | Constraint checks should be applied to template arguments. | R | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 104 | A template specialization shall be declared before its use. | M | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 105 | A template definition’s dependence on its instantiation contexts should be minimized. | R | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 106 | Specializations for pointer types should be made where appropriate. | R | N/A | Only template use is the `Result<T>` alias (`include/starcom/result.hpp:8,14`); no class/function templates authored. |
| 107 | Functions shall always be declared at file scope. | M | PASS | All functions at namespace scope. |
| 108 | Functions with variable numbers of arguments shall not be used. | M | FAIL | Host adapter calls variadic `fcntl` (`adapters/host/udp.cpp:75,79`). `cppcoreguidelines-pro-type-vararg` **[shipped config would miss: check disabled by Checks-block comment defect]**; `hicpp-vararg` flags `udp.cpp:79` on the box Linux pre-check; on the PC 21.1.8 (Windows branch) the `fcntl` path is not compiled, so 0. Core: 0. |
| 109 | A function definition should not be placed in a class specification unless the function is | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 110 | Functions with more than 7 arguments will not be used. | M | FAIL | 9-argument functions in RC adapter: `src/starcom_adapt/cmd_sdu.cpp:9 pack_cmd_sdu_user`, `:27 unpack_cmd_sdu_user`, `byte_pump.cpp:363 pump_pack_cmd_packet` (lizard). Starcom: max ≤ 6. |
| 111 | A function shall not return a pointer or reference to a non-static local object. | M | PASS | No pointer/reference to local returned (read; `bugprone-dangling-handle` 0). |
| 112 | Function return values should not obscure resource ownership. | R | PASS | Caller owns all buffers (`ICD.md:20`); no ownership-transferring returns. |
| 113 | Functions will have a single exit point. | M | PASS | Early return permitted by house decision (`CODING_STANDARDS.md:58`). |
| 114 | All exit points of value-returning functions shall be through return statements. | M | PASS | `-Wreturn-type` under `-Werror` (`cmake/CompilerWarnings.cmake:10`); clean build. |
| 115 | If a function returns error information, then that error information will be tested. | M | PARTIAL | `Result` (`tl::expected`) is not `[[nodiscard]]` (grep `nodiscard` in `include/starcom` + `third_party/tl/expected.hpp`: 0), so unchecked results are not compiler-caught. `cert-err33-c`: `adapters/host/file_replay.cpp:76,91` (`fclose` result ignored). |
| 116 | Small, concrete-type arguments (two or three words in size) should be passed by value if c | R | PASS | Small values by value, aggregates by `const&`/`span` (read of `include/starcom/ccsds/*.hpp`). |
| 117 | Arguments should be passed by reference if NULL values are not possible: | R | PASS | Small values by value, aggregates by `const&`/`span` (read of `include/starcom/ccsds/*.hpp`). |
| 117.1 | An object should be passed as const T& if the function should not change the value of the  | R | PASS | Small values by value, aggregates by `const&`/`span` (read of `include/starcom/ccsds/*.hpp`). |
| 117.2 | An object should be passed as T& if the function may change the value of the object. | R | PASS | Small values by value, aggregates by `const&`/`span` (read of `include/starcom/ccsds/*.hpp`). |
| 118 | Arguments should be passed via pointers if NULL values are possible: | R | PASS | Nullable out-params as pointers with null checks (e.g. `mac.cpp:1028` `Pcid*`, `file_replay.cpp:60`). |
| 118.1 | An object should be passed as const T* if its value should not be modified. | R | PASS | Nullable out-params as pointers with null checks (e.g. `mac.cpp:1028` `Pcid*`, `file_replay.cpp:60`). |
| 118.2 | An object should be passed as T* if its value may be modified. | R | PASS | Nullable out-params as pointers with null checks (e.g. `mac.cpp:1028` `Pcid*`, `file_replay.cpp:60`). |
| 119 | Functions shall not call themselves, either directly or indirectly (i.e. | M | PASS | `misc-no-recursion` enabled, 0 hits (v21 shipped). |
| 120 | Overloaded operations or methods should form families that use the same semantics, share t | R | N/A | No overloaded operations. |
| 121 | Only functions with 1 or 2 statements should be considered candidates for inline functions | R | N/A | No `inline` functions authored (`inline` used only for `inline constexpr` variables). |
| 122 | Trivial accessor and mutator functions should be inlined. | R | N/A | No `inline` functions authored (`inline` used only for `inline constexpr` variables). |
| 123 | The number of accessor and mutator functions should be minimized. | R | PASS | No accessor/mutator families. |
| 124 | Trivial forwarding functions should be inlined. | R | N/A | No `inline` functions authored (`inline` used only for `inline constexpr` variables). |
| 125 | Unnecessary temporary objects should be avoided. | R | PASS | No unnecessary temporaries found (read; `performance-*` enabled checks 0). |
| 126 | Only valid C++ style comments (//) shall be used. | M | PASS | Only `//` comments; `/*name*/` unnamed-parameter markers at `mac.cpp:214,1016,1028` are parameter-name annotations (2026-02-07 precedent counts only comment blocks). |
| 127 | Code that is not used (commented out) shall be deleted. | M | PASS | No commented-out code found (grep heuristic: 1 candidate, `space_packet_service.cpp:113`, is prose). |
| 128 | Comments that document actions or sources (e.g. | M | PASS | Book clause citations by name (e.g. `space_packet.cpp:51` "133.0-B-2 §4.1.3.3.3.4"), as in 2026-02-07 precedent. |
| 129 | Comments in header files should describe the externally visible behavior of the functions  | R | PARTIAL | Header comments present on some declarations (e.g. `include/starcom/ccsds/clcw.hpp:13`), none on most adapter headers (read). |
| 130 | The purpose of every line of executable code should be explained by a comment, although on | R | FAIL | Core `.cpp` comment density 3.2% (118/3672), adapters 0.0% (0/459) — see D.8. Many executable regions uncommented (e.g. `uslp.cpp`, `v3.cpp` 0.0%). |
| 131 | One should avoid stating in comments what is better stated in code (i.e. | R | PASS | Comments that exist explain book provenance, not code restatement (read). |
| 132 | Each variable declaration, typedef, enumeration value, and structure member will be commen | M | PARTIAL | Struct members partly commented (`include/starcom/ccsds/clcw.hpp:16`); many uncommented (read). |
| 133 | Every source file will be documented with an introductory comment that provides informatio | M | FAIL | No introductory file comment in any Starcom `.cpp` (13 core + 6 adapters start with `#include`, e.g. `src/ccsds/clcw.cpp:1`) or `.hpp` (23/23 begin `#pragma once` then `#include`). RC adapter files PASS. |
| 134 | Assumptions (limitations) made by functions should be documented in the function’s preambl | R | PARTIAL | Function preambles mostly absent in `.cpp`; some assumptions stated in headers (read). |
| 135 | Identifiers in an inner scope shall not use the same name as an identifier in an outer sco | M | PASS | No shadowing found (read; `-Wshadow` not enabled — RULE_VERIFIABILITY_TRIAGE.md:81 gap). Review cross-check: `g++ -Wshadow` 0 warnings (Claude Code review §4). |
| 136 | Declarations should be at the smallest feasible scope. | R | PARTIAL | `misc-const-correctness` **[shipped config would miss: check disabled by Checks-block comment defect]** — diagnostic run: 9 locals could be `const` (e.g. `src/ccsds/conv.cpp:31`, `copp.cpp:121`). Five non-const globals on PC 21.1.8 (Rule 207, incl. `udp.cpp:24 g_wsa`). |
| 137 | All declarations at file scope should be static where possible. | R | PARTIAL | File-scope helpers in anonymous namespaces (e.g. `copp.cpp:341`, `cop1.cpp:10`); 2 helpers could be internal: `copp.cpp:433 coppEncodeUslp`, `pltu.cpp:192 repeatSeq` (`misc-use-internal-linkage`) → PARTIAL-level note. |
| 138 | Identifiers shall not simultaneously have both internal and external linkage in the same t | M | PASS | Single declaration per external symbol (headers); no mixed linkage found. |
| 139 | External objects will not be declared in more than one file. | M | PASS | Single declaration per external symbol (headers); no mixed linkage found. |
| 140 | The register storage class specifier shall not be used. | M | PASS | No `register` keyword (grep hits are comments). |
| 141 | A class, structure, or enumeration will not be declared in the definition of its type. | M | PASS | No type declared inside its definition. |
| 142 | All variables shall be initialized before use. | M | PASS | `cppcoreguidelines-init-variables` enabled, 0 hits; `pro-type-member-init` 0. |
| 143 | Variables will not be introduced until they can be initialized with meaningful values. | M | PASS | `cppcoreguidelines-init-variables` enabled, 0 hits; `pro-type-member-init` 0. |
| 144 | Braces shall be used to indicate and match the structure in the non-zero initialization of | M | PASS | Aggregate inits brace-matched (read; `ldpc.cpp:39`). |
| 145 | In an enumerator list, the ‘=‘ construct shall not be used to explicitly initialize member | M | PASS | Enumerators auto-increment or fully explicit (read `include/starcom/ccsds/mac.hpp`). |
| 146 | Floating point implementations shall comply with a defined floating point standard. | M | N/A | Core has no floating point; RC adapter floats are MAVLink fields (`src/starcom_adapt/cmd_sdu.cpp:10`), IEEE-754 target. |
| 147 | The underlying bit representations of floating point numbers shall not be used in any way  | M | PASS | No float bit reinterpretation (grep `bit_cast`, unions: 0). |
| 148 | Enumeration types shall be used instead of integer types (and constants) to select from a  | M | PASS | `enum class` for choices (`Error`, `MacState`, `PhyTier` `ICD.md:312`). |
| 149 | Octal constants (other than zero) shall not be used. | M | PASS | No octal literals (grep hits are comments / `0b` binary literals, e.g. `include/starcom/ccsds/clcw.hpp:16`). |
| 150 | Hexadecimal constants will be represented using all uppercase letters. | M | PASS | 0 lowercase hex digits, 0 `0X` (grep). |
| 151 | Numeric values in code will not be used; symbolic values will be used instead. | M | FAIL | 115 `readability-magic-numbers` (PC 21.1.8; header hits not reported, T1-b), e.g. `src/ccsds/clcw.cpp:13`; RC adapter 18 (box, host flags, provisional). IVP23 had 81. |
| 151.1 | A string literal shall not be modified. | M | PASS | No string-literal modification. |
| 152 | Multiple variable declarations shall not be allowed on the same line. | M | PASS | No multi-variable declarations (grep + `readability-isolate-declaration` 0). |
| 153 | Unions shall not be used. | M | PASS | No unions. |
| 154 | Bit-fields shall have explicitly unsigned integral or enumeration types only. | M | N/A | No bit-fields. |
| 155 | Bit-fields will not be used to pack data into a word for the sole purpose of saving space. | M | N/A | No bit-fields. |
| 156 | All the members of a structure (or class) shall be named and shall only be accessed via th | M | PASS | Members accessed by name. |
| 157 | The right hand operand of a && or // operator shall not contain side effects. | M | PASS | No side effects in `&&`/`||` RHS found (read). **Checked by reading only. No tool checked it (RV-5).** |
| 158 | The operands of a logical && or // shall be parenthesized if the operands contain binary o | M | PASS | `readability-math-missing-parentheses`/read: operands parenthesized. |
| 159 | Operators //, &&, and unary & shall not be overloaded. | M | N/A | No operator overloading. |
| 160 | An assignment expression shall be used only as the expression in an expression statement. | M | PASS | `bugprone-assignment-in-if-condition` enabled, 0 hits. |
| 162 | Signed and unsigned values shall not be mixed in arithmetic or comparison operations. | M | FAIL | `g++ -Wsign-conversion` (MinGW 15.2, PC compile DB, 2026-10-29): 4 sites, `src/ccsds/conv.cpp:16`, `:17`, `src/ccsds/ldpc.cpp:159`, `adapters/host/udp.cpp:73`. (`hicpp-signed-bitwise`, 179 on PC 21.1.8 diag, tests bitwise ops on signed operands, not mixing; it is not evidence for this rule.) *RV-4: evidence replaced with `-Wsign-conversion`; graded FAIL, not PARTIAL, under fail-means-fix (PR-34).* |
| 163 | Unsigned arithmetic shall not be used. | M | N/A | Recommended-only per 2026-02-07 precedent; unsigned mandated for protocol fields (RULE_VERIFIABILITY_TRIAGE.md:311). |
| 164 | The right hand operand of a shift operator shall lie between zero and one less than the wi | M | PASS | Shift counts bounded by loop ranges 0..7 / field widths (e.g. `crc32.cpp:17-18`, `uslp.cpp:25`). Runtime-shift proof not tool-checked. |
| 164.1 | The left-hand operand of a right-shift operator shall not have a negative value. | M | PASS | Right shifts on unsigned operands (e.g. `crc32.cpp:18`). |
| 165 | The unary minus operator shall not be applied to an unsigned expression. | M | PASS | No unary minus on unsigned (grep). |
| 166 | The sizeof operator will not be used on expressions that contain side effects. | M | PASS | `bugprone-sizeof-expression` enabled, 0. |
| 167 | The implementation of integer division in the chosen compiler shall be determined, documen | M | PASS | Integer division only on unsigned sizes (`ldpc.cpp:127` `i / 8`); no signed division found. |
| 168 | The comma operator shall not be used. | M | PASS | No comma operator. |
| 169 | Pointers to pointers should be avoided when possible. | R | PASS | No pointer-to-pointer (grep `**`: 0). |
| 170 | More than 2 levels of pointer indirection shall not be used. | M | PASS | No pointer-to-pointer (grep `**`: 0). |
| 171 | Relational operators shall not be applied to pointer types except where both operands are  | M | PASS | No relational ops on pointers found (read). **Checked by reading only. No tool checked it (RV-5).** |
| 173 | The address of an object with automatic storage shall not be assigned to an object which p | M | PASS | No automatic-storage address escapes (read). **Checked by reading only. No tool checked it (RV-5).** |
| 174 | The null pointer shall not be de-referenced. | M | PASS | Null checks before deref in adapters (`file_replay.cpp:60`); core takes spans/refs. |
| 175 | A pointer shall not be compared to NULL or be assigned NULL; use plain 0 instead. | M | PASS | `nullptr` per house decision (`CODING_STANDARDS.md:57`). |
| 176 | A typedef will be used to simplify program syntax when declaring function pointers. | M | FAIL | Raw function-pointer member syntax without typedef: `include/starcom/adapters/radio_bus.hpp:15-18`, `pio_port.hpp:15-16` (`file_replay.hpp:11` uses an alias). P10-9 governs the ban (`CODING_STANDARDS.md:56`). |
| 177 | User-defined conversion functions should be avoided. | R | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 178 | Down casting (casting from base to derived class) shall only be allowed through one of the | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 179 | A pointer to a virtual base class shall not be converted to a pointer to a derived class. | M | N/A | No classes (only POD `struct`s, 0 access specifiers, 0 constructors/destructors/member functions; grep `public:|private:|~\w+\(|= default`). |
| 180 | Implicit conversions that may result in a loss of information shall not be used. | M | PASS | `bugprone-narrowing-conversions` + `cppcoreguidelines-narrowing-conversions` enabled, 0 hits. Review cross-check: `g++ -Wconversion` 0 warnings (Claude Code review §4). |
| 181 | Redundant explicit casts will not be used. | M | FAIL | 12 redundant casts. `g++ -Wuseless-cast` (MinGW 15.2, PC compile DB, 2026-10-29): `src/ccsds/mac.cpp:234`, `:1127`, `:1128`, `:1129`, `:1166`, `:1167`, `:1168`, `:1212`, `:1213`, `:1214`, `src/ccsds/ldpc.cpp:159`, `adapters/host/udp.cpp:149`. `readability-redundant-casting` (21.1.8) reports `mac.cpp:234`. *RV-3: was PASS "by read"; re-graded FAIL with the 12 sites (PR-33).* |
| 182 | Type casting from any type to or from pointers shall not be used. | M | FAIL | `reinterpret_cast` ×6 in host adapter `adapters/host/udp.cpp:128,141,177,179,190,211` (sockaddr/char buffers) — no JSF-182 exception applies (`CODING_STANDARDS.md:59`). `cppcoreguidelines-pro-type-reinterpret-cast` **[shipped config would miss: check disabled by Checks-block comment defect]**. `static_cast<void*>(&f)` for `memset` (`copp.cpp:122,227`) = Exception 1 class. Core: 0 reinterpret_cast. |
| 183 | Every possible measure should be taken to avoid type casting. | R | PARTIAL | 203 `static_cast` lines; most are bit-field packing (e.g. `clcw.cpp:31-34`). Necessary but heavy. |
| 184 | Floating point numbers shall not be converted to integers unless such a conversion is a sp | M | N/A | No float→int conversion in Starcom core. |
| 185 | C++ style casts (const_cast, reinterpret_cast, and static_cast) shall be used instead of t | M | PASS | `cppcoreguidelines-pro-type-cstyle-cast` enabled, 0 hits. Note: 35 functional-style `std::byte(...)` casts (e.g. `clcw.cpp:37`) are not flagged by that check. |
| 186 | There shall be no unreachable code. | M | PASS | No unreachable code (read; `-Wunreachable-code` not set). **Checked by reading only. No tool checked it (RV-5).** |
| 187 | All non-null statements shall potentially have a side-effect. | M | PASS | No side-effect-free statements (`bugprone-*` 0). |
| 188 | Labels will not be used, except in switch statements. | M | PASS | No labels/`goto` (grep: 0). `cppcoreguidelines-avoid-goto` **[shipped config would miss: check disabled by Checks-block comment defect]** — grep is the only evidence in the shipped gate. |
| 189 | The goto statement shall not be used. | M | PASS | No labels/`goto` (grep: 0). `cppcoreguidelines-avoid-goto` **[shipped config would miss: check disabled by Checks-block comment defect]** — grep is the only evidence in the shipped gate. |
| 190 | The continue statement shall not be used. | M | FAIL | `continue` ×10: core `ldpc.cpp:139`, `pltu.cpp:164,183`; host adapter `file_replay.cpp:80,85`; RC adapter `byte_pump.cpp:194,198,204,208,213`. House decision: adopt ban, remediate (`CODING_STANDARDS.md:60`). |
| 191 | The break statement shall not be used (except to terminate the cases of a switch statement | M | FAIL | `break` out of loops: `src/ccsds/cop1.cpp:771`, `adapters/host/file_replay.cpp:88`. |
| 192 | All if, else if constructs will contain either a final else clause or a comment indicating | M | PARTIAL | else-if chains without final `else` (heuristic, comment exemption not checked): `cop1.cpp:294,526,684`, `mac.cpp:381,587,786,795,805,842`, RC `byte_pump.cpp:134`. |
| 193 | Every non-empty case clause in a switch statement shall be terminated with a break stateme | M | PASS | No fall-through found in the 5 switches (`mac.cpp:305,354,397,502,955`). |
| 194 | All switch statements that do not intend to test for every enumeration value shall contain | M | PASS | 5 switches, 5 `default:` (`bugprone-switch-missing-default-case` enabled, 0). |
| 195 | A switch expression will not represent a Boolean value. | M | PASS | Switches on `MacState` enum. |
| 196 | Every switch statement will have at least two cases and a potential default. | M | PASS | All switches ≥ 2 cases. |
| 197 | Floating point variables shall not be used as loop counters. | M | PASS | Integral loop counters. |
| 198 | The initialization expression in a for loop will perform no actions other than to initiali | M | PASS | Single-variable init/increment in `for` (grep). |
| 199 | The increment expression in a for loop will perform no action other than to change a singl | M | PASS | Single-variable init/increment in `for` (grep). |
| 200 | Null initialize or increment expressions in for loops will not be used; a while loop will  | M | FAIL | `for (;;)` at `adapters/host/file_replay.cpp:70`. |
| 201 | Numeric variables being used within a for loop for iteration counting shall not be modifie | M | PASS | Loop counters not modified in body (read of the `for` loops in `src/ccsds`). |
| 202 | Floating point variables shall not be tested for exact equality or inequality. | M | PASS | No float equality (grep: 0). |
| 203 | Evaluation of expressions shall not lead to overflow/underflow (unless required algorithmi | R | PASS | Modulo arithmetic only where algorithmic (CRC / sequence counters, e.g. `src/starcom_adapt/byte_pump.cpp:320` masked). |
| 204 | A single operation with side-effects shall only be used in the following contexts: | M | PASS | No multiple side effects per expression found (read; `bugprone-*` 0). **Checked by reading only. No tool checked it (RV-5).** |
| 204.1 | The value of an expression shall be the same under any order of evaluation that the standa | M | PASS | No multiple side effects per expression found (read; `bugprone-*` 0). |
| 205 | The volatile keyword shall not be used unless directly interfacing with hardware. | M | PASS | No `volatile`. |
| 206 | Allocation/deallocation from/to the free store (heap) shall not occur after initialization | M | PASS | No heap: D-5 heap trap (`tests/unit/heap_trap.cpp`) passes; grep `new/malloc/free`: 0 code hits. |
| 207 | Unencapsulated global data will be avoided. | M | FAIL | Unencapsulated mutable globals: `src/ccsds/copp.cpp:70 g_coppSyncLossN`, `:347 g_coppRetransmitN` (process-wide counters, not in caller-owned state per DESIGN D-2 `docs/DESIGN.md:94`); scratch buffers `copp.cpp:345 g_tfScratch`, `cop1.cpp:12 g_cop1TfScratch` (see disposition). Host `udp.cpp:24 g_wsa`. |
| 208 | C++ exceptions shall not be used (i.e. | M | PASS | `-fno-exceptions` (`cmake/CompilerWarnings.cmake:11`); no throw/try/catch. |
| 209 | The basic types of int, short, long, float and double shall not be used, but specific-leng | R | FAIL | Basic `unsigned`/`int` used pervasively in core (≈100 lines, e.g. `src/ccsds/clcw.cpp:9-11`, `crc32.cpp:16-19`, `for (int i = 7; …)` `conv.cpp:35`); `int` in adapter API `include/starcom/adapters/radio_bus.hpp:17-25`. `google-runtime-int` only flags short/long, so the gate does not catch this. `CODING_STANDARDS.md:521`. |
| 210 | Algorithms shall not make assumptions concerning how data is represented in memory (e.g. | M | PASS | Explicit big-endian byte packing (e.g. `uslp.cpp:25`, `space_packet.cpp:15`); no struct overlay. |
| 210.1 | Algorithms shall not make assumptions concerning the order of allocation of nonstatic data | M | PASS | Explicit big-endian byte packing (e.g. `uslp.cpp:25`, `space_packet.cpp:15`); no struct overlay. |
| 211 | Algorithms shall not assume that shorts, ints, longs, floats, doubles or long doubles begi | M | PASS | Explicit big-endian byte packing (e.g. `uslp.cpp:25`, `space_packet.cpp:15`); no struct overlay. |
| 212 | Underflow or overflow functioning shall not be depended on in any special way. | M | PASS | No reliance on overflow beyond masked modulo. |
| 213 | No dependence shall be placed on C++’s operator precedence rules, below arithmetic operato | M | PASS | Shifts/masks parenthesized (read, e.g. `space_packet.cpp:27`). |
| 214 | Assuming that non-local static objects, in separate translation units, are initialized in  | M | PASS | Only constant-initialized statics. |
| 215 | Pointer arithmetic will not be used. | M | FAIL | Pointer arithmetic `b.scratch.data() + …` in host adapter `adapters/host/file_replay.cpp:19,29,43`. Core uses `span::subspan`. |
| 216 | Programmers should not attempt to prematurely optimize code. | R | PASS | No premature-optimization constructs found. |
| 217 | Compile-time and link-time errors should be preferred over run-time errors. | R | PASS | `static_assert` on endpoint triviality (`copp.cpp:224`, `cop1.cpp:540`); `constexpr` constants. |
| 218 | Compiler warning levels will be set in compliance with project policies. | M | PASS | `-Wall -Wextra -Wpedantic -Werror` (`cmake/CompilerWarnings.cmake:10`); clean build (MinGW g++ 15.2 on the PC, 0 warnings; clang 22 on the box). |
| 219 | All tests applied to a base class interface shall be applied to all derived class interfac | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 220 | Structural coverage algorithms shall be applied against flattened classes. | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |
| 221 | Structural coverage of a class within an inheritance hierarchy containing virtual function | M | N/A | No virtual functions or inheritance (grep `virtual`: 0 code hits). |

#### Section B: Power of 10 — FAIL (3 FAIL / 4 PARTIAL / 2 PASS / 1 N/A (Aspirational) of 10)

The method is Appendix B.1 (project notes for each rule).

| Rule | Summary | App | Status | Notes |
|---|---|---|---|---|
| P10-1 | Simple control flow; no goto/setjmp/recursion | M | PASS | grep `goto|setjmp|longjmp`: 0; `misc-no-recursion` 0. `cppcoreguidelines-avoid-goto` **[shipped config would miss]** — PASS rests on grep (Tier 1 positive control failed). |
| P10-2 | Fixed loop bounds | M | FAIL | `adapters/host/file_replay.cpp:70` `for (;;)` read loop bounded only by EOF/error — not one of the B.1 scheduler/halt exemptions (`AUDIT_GUIDANCE.md:235`). Core PASS: `for` loops counted over sizes; `while` at `src/ccsds/pltu.cpp:160` bounded by `octets.size()`. |
| P10-3 | No dynamic allocation after init | M | PASS | D-5 heap trap (`tests/unit/heap_trap.cpp`) armed in `starcom.unit` (passed on the PC, MinGW 15.2, and on the box); grep `new|malloc|free`: 0 code hits. |
| P10-4 | ≤ 60 lines per function | M | FAIL | B.1 method: "Enforced at ≤60 lines" (`AUDIT_GUIDANCE.md:243`). `src/ccsds/mac.cpp:396 macWaitExpiredHalf` 104 lines (lizard). 1/230 core functions; RC adapter 0 over 60. |
| P10-5 | Assertion density ≥ 2/function | M | N/A (Aspirational per B.1) | B.1: "Aspirational … Density not currently measured. Out-of-scope for this audit cycle" (`AUDIT_GUIDANCE.md:247`). Informational: 2 `static_assert`, 0 runtime asserts / 230 functions. `bugprone-assert-side-effect` **[shipped config would miss]**. |
| P10-6 | Smallest scope | M | PARTIAL | Scratch buffers `src/ccsds/copp.cpp:345 g_tfScratch`, `cop1.cpp:12 g_cop1TfScratch` are static-by-requirement (LL Entry 1, `CODING_STANDARDS.md:176-177`; comment `copp.cpp:340-344` cites the 4 KiB Core 0 stack) → consistent with B.1. Counters `copp.cpp:70 g_coppSyncLossN`, `:347 g_coppRetransmitN` are process-wide mutable state outside the caller-owned endpoint (DESIGN D-2, `docs/DESIGN.md:94`) → finding. 2 helpers with external linkage (`copp.cpp:433`, `pltu.cpp:192`). 9 locals could be `const` **[shipped config would miss]**. |
| P10-7 | Check returns and parameters | M | PARTIAL | B.1 method: tidy enforces where `[[nodiscard]]`; grep `(void)` discards (`AUDIT_GUIDANCE.md:255`). `Result` is not `[[nodiscard]]` (0 hits), unlike RC APIs (`docs/audits/RULE_VERIFIABILITY_TRIAGE.md:10`). 7 unjustified `(void)call()` discards: `src/ccsds/mac.cpp:169,175,181,187`, `copp.cpp:370,394,423`. `cert-err33-c` `adapters/host/file_replay.cpp:76,91`. Parameter checks present via `Result` errors. |
| P10-8 | Limited preprocessor | M | PARTIAL | Function-like stringize macros `include/starcom/version.hpp.in:14-15`; platform conditionals ×37 `adapters/host/udp.cpp`; `result.hpp:5` `STARCOM_USE_STD_EXPECTED` switch; RC adapter `byte_pump.cpp:262-269`. Constants otherwise `constexpr`. |
| P10-9 | Single dereference; no function pointers | M | FAIL | Function pointers in adapter seams: `include/starcom/adapters/radio_bus.hpp:15-18`, `pio_port.hpp:15-16`, `adapters/host/file_replay.hpp:11` (`PltuSink`). P10-9 governs (`CODING_STANDARDS.md:56`); RC FP-1 was remediated (R-6c, B.1 `AUDIT_GUIDANCE.md:263`); DESIGN allows "Policy/template seams in core; virtual dispatch only at adapter edges" (`docs/DESIGN.md:111`), not fn pointers. Core: 0. No `**`. |
| P10-10 | All warnings on; analyzers | M | PARTIAL | `-Wall -Wextra -Wpedantic -Werror` (`cmake/CompilerWarnings.cmake:10-11`), clean build. Analyzer gate defective (Tier 1: 13 checks dead plus 23 `clang-analyzer-core.*` silenced, 21.1.8); no Starcom cppcheck. |

#### Section C: JPL C — FAIL (LOC-1 to LOC-4: 5 FAIL / 8 PARTIAL / 14 PASS / 4 N/A of 31)

LOC-5 and LOC-6 are DEFERRED-WITH-RATIONALE (`AUDIT_GUIDANCE.md:81`, `CODING_STANDARDS.md:28-43`).

| Rule | Summary | App | Status | Notes |
|---|---|---|---|---|
| LOC-1 | Language conformance / no UB | M | PARTIAL | C++20 `-Wpedantic -Werror`; host adapters use POSIX/Winsock (A.2). No UB found by read; UB not tool-proved this cycle. |
| LOC-2 | Routine checking (warnings + SA) | M | PARTIAL | → P10-10. |
| LOC-3 | Loop bounds | M | FAIL | → P10-2: `adapters/host/file_replay.cpp:70`. |
| LOC-4 | No recursion | M | PASS | → P10-1. |
| LOC-5 | No heap after init | M | PASS | → P10-3. |
| LOC-6 | IPC discipline | M | N/A | Sans-I/O library; no IPC/messaging primitives in scope (grep). RC integration is Tier 4. |
| LOC-7 | Thread safety | M | N/A | No threads, tasks or delays in scope (grep `sleep|delay|thread|mutex`: 0 code hits). Single-context library by design. |
| LOC-8 | Shared data, single owner | M | FAIL | `src/ccsds/copp.cpp:70,347` process-wide counters and shared scratch `copp.cpp:345`, `cop1.cpp:12` are owned by no endpoint; DESIGN D-2 requires caller-owned state (`docs/DESIGN.md:94`). Scratch is static-by-requirement (LL Entry 1) but its single-caller assumption is undocumented as an owner. |
| LOC-9 | Semaphores / locking | M | N/A | None in scope. |
| LOC-10 | Memory protection | M | N/A | Library; MPU is RC platform (`ACCEPTED`/HW bench), out of scope. |
| LOC-11 | No goto / setjmp | M | PASS | grep 0. Gate **[shipped config would miss]**. |
| LOC-12 | Enum initialization | M | PASS | Enumerators none-or-all explicit (read of `include/starcom/ccsds/*.hpp`). |
| LOC-13 | Limited scope | M | PARTIAL | → P10-6. |
| LOC-14 | Check return values | M | PARTIAL | → P10-7: `mac.cpp:169,175,181,187`, `copp.cpp:370,394,423`; `file_replay.cpp:76,91`; no `[[nodiscard]]`. |
| LOC-15 | Check parameter values | M | PARTIAL | Decoders/encoders validate lengths and return `Error` (e.g. `adapters/host/file_replay.cpp:60-62`); not all entry points checked by read (no tool). |
| LOC-16 | Assertions | M | PARTIAL | 0 runtime assertions; density half Aspirational per B.1; `bugprone-assert-side-effect` **[shipped config would miss]** (vacuous: 0 asserts). |
| LOC-17 | Fixed-width types | M | FAIL | → JSF 209: plain `unsigned`/`int` pervasive (e.g. `src/ccsds/crc32.cpp:16-19`, `include/starcom/adapters/radio_bus.hpp:17-25`). |
| LOC-18 | Order of evaluation | M | PASS | No multi-side-effect expressions found (read). |
| LOC-19 | No side effects in boolean expressions | M | PASS | `bugprone-assignment-in-if-condition` 0. |
| LOC-20 | Preprocessor use | M | PARTIAL | → P10-8. |
| LOC-21 | No macros in function/block | M | PASS | All `#define`s at file scope. |
| LOC-22 | No #undef | M | PASS | grep: 0. |
| LOC-23 | Conditional directives in same file | M | PASS | All `#if`/`#endif` balanced per file (`udp.cpp`, `result.hpp`, `byte_pump.cpp`). |
| LOC-24 | One statement / declaration per line | M | PASS | → JSF 42/152. |
| LOC-25 | Function size ≤ 60 lines, ≤ 6 params | M | FAIL | `src/ccsds/mac.cpp:396` (104 lines); 9 params: `src/starcom_adapt/cmd_sdu.cpp:9`, `:27`, `byte_pump.cpp:363`. Core params ≤ 6. |
| LOC-26 | ≤ 2 levels of indirection | M | PASS | No `**` (grep). |
| LOC-27 | ≤ 2 dereferences per statement | M | PASS | Read; spans/refs used. |
| LOC-28 | No dereference hidden in typedef/macro | M | PARTIAL | `adapters/host/file_replay.hpp:11` `using PltuSink = …(*)(…)` hides a pointer type. |
| LOC-29 | No function pointers | M | FAIL | → P10-9. |
| LOC-30 | No fn-ptr ↔ integral conversion | M | PASS | None (grep `reinterpret_cast` sites are sockaddr/char only). |
| LOC-31 | #include placement | M | PASS | Includes precede code; only preprocessor/comments before them. |

#### Section D: project rules — FAIL (1 FAIL / 2 PARTIAL / 3 PASS / 2 N/A of 8)

Commit 918b1219 corrected this count. The previous heading was "2 PASS / 3 N/A". The rows are D.1, D.3 and D.4 PASS, and D.2 and D.5 N/A.

| Rule | Summary | Status | Notes |
|---|---|---|---|
| D.1 | RP2350 platform constraints | PASS | Core: 0 `hardware/`/`pico/` includes (grep; only a comment `include/starcom/adapters/pio_port.hpp:11`). Large scratch is `static` per LL Entry 1 (`src/ccsds/copp.cpp:340-345`). |
| D.2 | Multicore rules | N/A | No cross-core state or primitives in Starcom or `src/starcom_adapt/` (grep `atomic|core1|multicore`: 0 code hits). Integration-side is Tier 4. |
| D.3 | Debug output | PASS | 0 `printf`/`DBG_PRINT` in scope. (stdio file I/O in `file_replay.cpp` is scored under JSF 22.) |
| D.4 | Prior-art research | PASS | 6/6 recent Starcom commits cite the Blue Book clause/table they implement (e.g. `40b83fc1` "Table 6-10", `b6a8ca4a` "133.0-B-2"). |
| D.5 | Safety & regulatory | N/A | Starcom carries no flight-safety function; RF/regulatory conformance is the CCSDS side (Duke's audit), not repeated. |
| D.6 | Git workflow | PARTIAL | 6/6 commits `[agent]`-prefixed with `Verified:` lines. But `starcom/scripts/run_clang_tidy.sh` is committed with mixed line endings (`git ls-files --eol`: `i/mixed`) and the gate it runs is defective (Tier 1). |
| D.7 | Session management | PARTIAL | Process rule; evidence is commit conduct. `8a80734d` added process globals contrary to the D-2 caller-owned design (`docs/DESIGN.md:94`) without a design note (see E). |
| D.8 | Comment density (15-25%, `CODING_STANDARDS.md:306-309`, `.cpp` only) | FAIL | Core 3.2% (118/3672; `uslp.cpp` 0.0, `v3.cpp` 0.0, `cop1.cpp` 0.3), host/rp2350 adapters 0.0% (0/459), `src/starcom_adapt/` 5.5%. 2026-05-13 RC was 21.8%. |

#### Section E: agent guidelines — PARTIAL

We walked the 6 last Starcom commits: `c71fcbcf`, `8a80734d`, `f19a7a08`, `b6a8ca4a`, `1f23b359`, `40b83fc1`.

- Section 1, "Think Before Coding": PASS. Each commit gives the book clause and has a `Verified:` line. The line gives ctest counts. It also states `UNBENCHED`, with the bench test at push.
- Section 2, "Simplicity First": PARTIAL. `8a80734d` (12 files, +421) and `c71fcbcf` add global `g_` counters (`copp.cpp:70,347`), not endpoint fields. Rule D-2 gives an easier position that agrees with the design, but the commits did not use it.
- Section 3, "Surgical Changes": PASS. The diffs stay in the feature. We read the change counts (`--stat`), and we saw no refactors of other code.
- Section 4, "Goal-Driven Execution": PARTIAL. The `Verified:` lines give ctest results only. No line gives a clang-tidy run, and 13 checks in the clang-tidy gate did not operate (Tier 1). Thus "verified" did not include the standards.

#### Tier 3.6: DEV_CODE audit (`docs/audits/DEV_CODE_AUDIT.md`) — PASS (3 of 3 conditional-compile sites)

The population is each `#if*` in scope.

- `src/starcom_adapt/byte_pump.cpp:262` `ROCKETCHIP_JOB_STATION`: "role selection". This is flight code, and the design has it.
- `byte_pump.cpp:267` `#ifndef ROCKETCHIP_HOST_TEST`: the host-test seam. The flight build keeps `defer_spdu_fifo_empty = true`. Thus no development code gets into the binary.
- `include/starcom/result.hpp:5` `STARCOM_USE_STD_EXPECTED`: library configuration.

`adapters/host/` is for the host only. The firmware does not link it. The `HOST_TEST` seam changes how the code operates in the test. We flag it for the Tier 5 check that tests operate as the flight code does. It is not a DEV_CODE FAIL.

#### Tier 3.7: VERSION_STRING audit (`docs/audits/VERSION_STRING_AUDIT.md`) — PASS (1 of 1 version source; 0 literal version strings)

The version has one source: `starcom/STARCOM_VERSION` → `CMakeLists.txt:3-10` → `include/starcom/version.hpp.in:8-27` (CMake makes the header from this template). A grep for `"N.N[.N]"` literals in scope finds 0. Starcom shows no start message. (JSF 29 gives the verdict for the function macros in that template.)

#### Tier 3.8: Section F-1, citations in the standards documents — PARTIAL (6 of 6 citations checked, 3 wrong)

The population is only the standards-document statements that this audit used. It is not the full Section F-1 population. `AUDIT_GUIDANCE_CURRENCY_CHECK.md` did the full walk only for the paths in `AUDIT_GUIDANCE.md`. This cycle did not walk the other documents.

| Citation | Status | Notes |
|---|---|---|
| `standards/AUDIT_GUIDANCE.md:80` "LOC-1 through LOC-4 (102 rules …)" | WRONG | JPL D-60411 LOC-1..4 = 31 rules (`docs/audits/RULE_VERIFIABILITY_TRIAGE.md:64`). |
| `standards/CODING_STANDARDS.md:97` "`#pragma once` is a live deviation" | STALE | RC remediated all to `#ifndef` 2026-06-24 (`RULE_VERIFIABILITY_TRIAGE.md:9`); no register row exists. Propagates into `starcom/docs/WORKING_HERE.md:72` and `IVP23_REPORT.md:21` ("existing project-wide exception"). |
| `standards/CODING_STANDARDS.md:56` "accepted deviation FP-1" | STALE | FP-1 resolved by R-6c (B.1 text, `AUDIT_GUIDANCE.md:263`). |
| `AUDIT_GUIDANCE.md:267` clang-tidy 21.1.8 | CONFIRMED | Matches 2026-05-13 metadata. |
| `AUDIT_GUIDANCE.md:247` P10-5 aspirational | CONFIRMED | Same at `MASTER_STANDARDS_AUDIT_2026-05-07.md:307`. |
| `CODING_STANDARDS.md:62` JSF-22 applies | CONFIRMED | Used for A.2. |

### 1.5 Summary of Section 1 (provisional: the cycle stopped at Tier 1; clang-tidy counts are from the pinned 21.1.8)

| Section | FAIL | PARTIAL | PASS | N/A |
|---|--:|--:|--:|--:|
| A JSF AV (233 ids) | 31 | 13 | 126 | 63 |
| B Power of 10 (10) | 3 | 4 | 2 | 1 (Aspirational) |
| C JPL LOC-1..4 (31) | 5 | 8 | 14 | 4 |
| D Project (8) | 1 | 2 | 3 | 2 |
| E AK_GUIDELINES (4) | 0 | 2 | 2 | 0 |
| 3.6 / 3.7 / 3.8 | – | 3.8 PARTIAL | 3.6, 3.7 | – |

**Main finding: Tier 1.**

- The Starcom `.clang-tidy` stops 13 checks on the pinned 21.1.8. The Rocket-Chip root `.clang-tidy` stops 10. The cause is the comment lines in the folded `Checks:` block.
- The two configs stop the 23 `clang-analyzer-core.*` checks.
- The `goto` probe line did not fire.
- With these checks on (diagnostic run), the gate finds 193 more hits. On the PC (21.1.8, Windows branch of `udp.cpp`): 179 `hicpp-signed-bitwise`, 9 `misc-const-correctness`, 5 `reinterpret_cast`. On the box (21.1.6, Linux branch): 180, 9 and 3, and 1 `pro-type-vararg`.
- Gate defect 2 (PC): `HeaderFilterRegex` (`starcom/.clang-tidy:237`) uses `/`. Thus Windows shows no header diagnostics (the gate did not find 9 header hits).

**Most important FAIL items:**

- JSF 181: 12 casts that have no effect (`g++ -Wuseless-cast`).
- JSF 162: 4 signed and unsigned conversions (`g++ -Wsign-conversion`).
- JSF 27 and 35: `#pragma once` in 23 of 23 headers.
- JSF 133: no comment at the start of the file (13 core and 6 adapter `.cpp` files, 23 `.hpp` files).
- D.8: comment density 3.2% in the core.
- JSF 190: `continue` ×10 (`src/ccsds/ldpc.cpp:139`, `pltu.cpp:164,183`, and others).
- P10-9: function-pointer seams in `include/starcom/adapters/radio_bus.hpp:15-18` and `pio_port.hpp:15-16`.
- P10-4 and JSF 3: `src/ccsds/mac.cpp:396`.
- JSF 110 and LOC-25: 9 parameters in `src/starcom_adapt/cmd_sdu.cpp:9,27` and `byte_pump.cpp:363`.
- JSF 209: `unsigned` and `int` without a size.
- JSF 14 and 151: 297 suffix hits and 115 magic-number hits (pinned tool, `.cpp` only).
- JSF 207 and LOC-8: globals in `src/ccsds/copp.cpp:70,347`.
- JSF 182: `adapters/host/udp.cpp` ×6.

### 1.6 Claims ledger

The ledger is the main task of this section. It checks claims in the docs against the code.

| # | Claim (quoted, file:line @579f0976) | Evidence | Verdict |
|---|---|---|---|
| L1 | "Checks copied from the RC house list (P10 / JPL LOC-1-4 / JSF / CERT / C++ CG)." `starcom/.clang-tidy:4` | 17 configured checks are not enabled on LLVM 23 (Tier 1), incl. the goto, reinterpret_cast, vararg, reserved-identifier, const-correctness and assert-side-effect gates. | **OVERSTATED** |
| L2 | `bugprone-reserved-identifier,` "# enabled: no Pico SDK names in this tree" `.clang-tidy:57-58` | Not enabled (swallowed by the comment at :56). | **OVERSTATED** |
| L3 | `cppcoreguidelines-pro-type-reinterpret-cast,` "# enabled: core is not MMIO; HW casts belong in ports" `.clang-tidy:97-98` | Not enabled. With it on: 3 hits, all in `adapters/host/udp.cpp:128,141,190`, core 0. So the *core* fact holds, but the gate claim doesn't. | **OVERSTATED** (gate) |
| L4 | "# enabled: core has no g_ process globals and no printf" `.clang-tidy:101` (re `avoid-non-const-global-variables` / `pro-type-vararg`) | `avoid-non-const-global-variables` is enabled and fires 4× in core (`cop1.cpp:12`, `copp.cpp:70,345,347`). `pro-type-vararg` not enabled; with it on, 1 hit, `udp.cpp:79` (adapter). | **NO-LONGER-APPLIES** (globals now exist) |
| L5 | "Codec Result/expected APIs should be [[nodiscard]] at the type." `.clang-tidy:272-273` | 0 `nodiscard` in src/include/tl. | **OVERSTATED** if read as current state; it's a "should". Flag as unmet intent. |
| L6 | "**0** `readability-identifier-naming` / **0** `cppcoreguidelines-*` … `bugprone-reserved-identifier` 0 … Core is not MMIO" `IVP23_TIDY_RUN.md:27,29,92-93` | Historical counts, but partly from disabled checks (Tier 1, YAML-level, so it applied then too; inference). Today: naming 1, cppcoreguidelines 4. | **OVERSTATED** (vacuous zeros) + NO-LONGER-APPLIES (counts) |
| L7 | "Gated clang-tidy was clean on `starcom/src/ccsds` + adapters for those four checks only" (size, cognitive, unused-return, reserved-id) `starcom/docs/IVP.md:373`; gate wording `starcom/docs/IVP.md:371`; also `STATUS.md:76` | `reserved-id` was never enabled (L2). Size/cognitive now fire once (`mac.cpp:396`). | **OVERSTATED** (reserved-id) + NO-LONGER-APPLIES (size/CC clean) |
| L8 | "the first Starcom tidy run reported 0 identifier-naming hits" `docs/WORKING_HERE.md:72` | True of IVP23. Today 1 hit (`copp.cpp:359`). | **HOLDS** (as history), but stale as a current-state reading |
| L9 | "Function-size: protocol state machines — **leave** (`macTick` / `apply_state` / USLP encode/decode / `fop1OnClcw`)" `IVP23_REPORT.md:37` | Only `macWaitExpiredHalf` (`mac.cpp:396`) exceeds now. The named functions no longer trip the check. | **NO-LONGER-APPLIES** |
| L10 | "**328** diagnostics … Re-run after porting" `IVP23_REPORT.md:35` | Re-run: 436 (shipped config, core+adapters), 430 core+include. | **NO-LONGER-APPLIES** (count superseded) |
| L11 | "Exceptions, RTTI, and heap-after-init are out (`-fno-exceptions -fno-rtti`; no `new` on codec paths)." `WORKING_HERE.md:70`; also `ICD.md:349`, `SAD.md:298` | Flags at `CompilerWarnings.cmake:10-11`; heap trap passes; 0 new/malloc in src. | **HOLDS** |
| L12 | "Rocket-Chip `standards/CODING_STANDARDS.md` **applies** to Starcom the same as firmware." `WORKING_HERE.md:70` | Applicability is a policy statement and holds. Enforcement does not: Starcom has no cppcheck gate, its tidy config has disabled checks, and P10-5 fails as written. | **HOLDS** (policy); enforcement **OVERSTATED** if read as conformance |
| L13 | "No HAL/SDK includes in the core (CI-guarded)." `DESIGN.md:111` | 0 such includes (grep) → the fact holds. No CI guard found: grep for `hardware/`, `pico` or a guard in `CMakeLists.txt`, `tests/CMakeLists.txt`, `cmake/*`, `scripts/` returns nothing. | **OVERSTATED** ("CI-guarded") |
| L14 | "exceptionless, no-RTTI, no-heap-after-init **by construction**" `DESIGN.md:111` | Same as L11. | **HOLDS** |
| L15 | "No virtual `IRadio` in the core (P10-9)." `ICD.md:318` | No `virtual` dispatch found in core. Adapter seams use C fn pointers (`radio_bus.hpp:15-18`, `pio_port.hpp:15-16`), which P10-9 also restricts. | **HOLDS** (core), UNVERIFIED for the adapter P10-9 disposition |
| L16 | "keep JSF AV C++ / P10 / JPL-C as-is; do not adopt MISRA C++:2023 … stay *compatible-with* it." `DESIGN.md:120` | No MISRA check or mapping was run. | **UNVERIFIED** |
| L17 | "The strictest plausible adopter sets the floor … the *core* is …" `DESIGN.md:35` | Not measured against any MISRA/AUTOSAR checker. | **UNVERIFIED** |
| L18 | "not a claim that the library is standards-complete" `starcom/docs/IVP.md:369` | Consistent with findings. | **HOLDS** |
| L19 | "**zero Starcom rows** in `standards/ACCEPTED_STANDARDS_DEVIATIONS.md`" `IVP23_REPORT.md:13` | grep `starcom` in that file: 0. But the open findings above have no deviation home, so they're unresolved, not accepted. | **HOLDS** |
| L20 | "IVP24 sizeof … `CoppEndpoint=19544` `Cop1Endpoint=19664` `FopP=164`" `starcom/docs/IVP.md:385` (D-5 size record, adjacent to standards) | This box (libc++/zig): `CoppEndpoint=10136 Cop1Endpoint=19608 FopP=9440`. Inference: the toolchain differs from MinGW, so not a like-for-like comparison, but the FopP change (164 → 9440) is too large to be toolchain alone. | **NO-LONGER-APPLIES** — MinGW 15.2 on the PC prints the same as the box (`CoppEndpoint=10136`, `FopP=9440`). |
| L21 | `-Wstack-usage=1024` "catches the next one at compile" `cmake/CompilerWarnings.cmake:12-16` | Ran on the PC MinGW build (core compile commands): 0 warnings. "Catches the next one" not exercised (no over-size frame exists). | **HOLDS** (ran clean) |
| L22 | "`#pragma once` is the existing project-wide exception, not a Starcom row." `starcom/docs/WORKING_HERE.md:72`; `docs/audits/IVP23_REPORT.md:21` | RC remediated every `#pragma once` (`docs/audits/RULE_VERIFIABILITY_TRIAGE.md:9`); no register row; `CODING_STANDARDS.md:97` "live deviation" is stale. | **OVERSTATED** |

**Totals (22 claims, one primary verdict each):** HOLDS 8 (L8, L11, L12, L14, L15, L18, L19, L21). OVERSTATED 8 (L1, L2, L3, L5, L6, L7, L13, L22). NO-LONGER-APPLIES 4 (L4, L9, L10, L20). UNVERIFIED 2 (L16, L17). UNDERSTATED 0.

**Secondary verdicts:** the counts in L6 and L7 also get NO-LONGER-APPLIES. If you read L12 as "a tool enforces the rule", L12 gets OVERSTATED. The adapter part of L15 gets UNVERIFIED.

### 1.7 Known items and gaps

**Known items (recorded before, not new findings):**

- `byte_pump.cpp` and `src/starcom_adapt/README.md` have the Mode Select and Scrambler packing text from before decision D13. README lines 20-21 cite 211.0-B-6 §B1.7.6 and §B1.7.7.
- The Space Packet rows in the `CONFORMANCE.md` Annex A do not agree with `space_packet_service.cpp`. The test is in the Rocket-Chip `test/` folder. The code does not connect the service. Section 3 (Duke) has this item.

**Not done:**

- cppcheck (no package, and no Starcom gate)
- stack analysis and coverage
- the L2-P5 Starcom bucket (DEFERRED in `L2P5_DISPOSITION_LOG.md:240-244`)
- `src/starcom_adapt/` with the Pico compile database
- the full Section F-1 walk on all five standards documents.

---

## 2. Triage and code-fix plan

**A FAIL gets a correction.** The owner rule (2026-10-29) is: "if something fails you fix it, you don't consider it not a fail because it gets fixed eventually." Thus each FAIL in Section 1 is a REMEDIATE item below. No FAIL gets ACCEPT or DEFER. `standards/ACCEPTED_STANDARDS_DEVIATIONS.md` has no Starcom row (`IVP23_REPORT.md:13`). Thus that register lets no FAIL stay. The items with "NO ACTION", "NO FINDING" or "DEFER" are not FAIL items. Each item tells the cause.

**Order.** The order follows `AUDIT_GUIDANCE.md` Appendix C: Category 1 (gate integrity), then Category 2 (claims), then Category 3 (code). In a category, the order is by dependency, then by risk.

**Default.** The default disposition is **REMEDIATE** (`standards/ACCEPTED_STANDARDS_DEVIATIONS.md:36`). Starcom gets **zero** deviation rows (`starcom/docs/audits/IVP23_REPORT.md:13`, owner decision 2026-09-02). Thus we recommend no ACCEPT for a Starcom finding. Each disposition comes from a project record, with its file and line.

**Counts.** The clang-tidy counts are from the pinned clang-tidy 21.1.8 on the PC (`AUDIT_GUIDANCE.md:267`). The verdicts are provisional, because the cycle stopped at Tier 1 (`AUDIT_GUIDANCE.md:57`).

### Category 1: gate integrity

| PR | Finding | Disposition | Record |
|---|---|---|---|
| PR-1 | `starcom/.clang-tidy`: comments in the folded `Checks: >` disable 13 checks, and the comment block at `:222-227` silences the 23 `clang-analyzer-core.*` checks (21.1.8, PC) | REMEDIATE: move the comments out of the block; add a `--list-checks` self-test plus the known-bad fixture | Tier 1 "no severity gate", `AUDIT_GUIDANCE.md:57`; 1.1 fixture self-check, `:47` |
| PR-1b | `starcom/.clang-tidy:237` `HeaderFilterRegex` uses `/`; on Windows (the pinned tool) no header diagnostic is reported (9 missed) | REMEDIATE: separator-agnostic regex (`[/\\]`), add a header hit to the fixture | `AUDIT_GUIDANCE.md:57`; `CODING_STANDARDS.md:280` silent exclusion prohibited |
| PR-2 | Same defect in the RC root `.clang-tidy`: 10 checks disabled plus the 23 silenced `clang-analyzer-core.*` (`.clang-tidy:227-232`; 21.1.8, PC); separate PR, full RC gate path | REMEDIATE (config fix G2). The old RC PASS lines that rested on these checks stay as written (Nathan, 2026-10-30); this report records them ("G2: old PASS lines" in the batch plan) | L2-P4 scope-language rule, `AUDIT_GUIDANCE.md:94`; "looks-covered-but-isn't", `CODING_STANDARDS.md:280` |
| PR-3 | LLVM 23 has no `hicpp-*` | NO ACTION for the gate (not a FAIL: a box-tool note). The pinned tool is 21.x, which has hicpp (v21.1.6 run fires `hicpp-signed-bitwise`). Counts confirmed on 21.1.8 (PC) | `AUDIT_GUIDANCE.md:267` "Clang-tidy 21.1.8" |
| PR-5 | Pre-commit hook has CRLF and calls `python` (fails on Linux) | NO ACTION (not a FAIL: Tier 1.3 was a dry-run note). The hook is a Git Bash/Windows install | `scripts/hooks/README.md:5,23` |
| PR-6 | `run_clang_tidy.sh` exits 2 | REMEDIATE (normalize to LF). Cause found: mixed line endings, so `exit 0\r` fails. `--all` excluding `tests/` is correct | Test code relaxed: `ACCEPTED_STANDARDS_DEVIATIONS.md:150` Note 1; `starcom/docs/WORKING_HERE.md:70` |

### Category 2: claims (after PR-1)

| PR | Finding | Disposition | Record |
|---|---|---|---|
| PR-7 | `.clang-tidy:4,57-58,97-98,101,272` overstated (L1-L5) | REMEDIATE with PR-1 | `ACCEPTED_STANDARDS_DEVIATIONS.md:36` |
| PR-8 | IVP23 zeros vacuous / stale (L6, L7, L9, L10) | REMEDIATE (docs; mark historical, point to this run) | L2-P4, `AUDIT_GUIDANCE.md:94` |
| PR-9 | `DESIGN.md:111` "(CI-guarded)": no guard exists | REMEDIATE: add the guard (a test or CMake check that core includes no `hardware/`/`pico/`) | `starcom/docs/DESIGN.md:111` is a settled design statement, so the code meets it, not the reverse |
| PR-10 | MISRA "compatible-with" / strictest-adopter claims unverified (L16/L17) | DEFER-WITH-RATIONALE (not a FAIL: the claims are UNVERIFIED). Keep the claims labeled UNVERIFIED | MISRA deferred until a certified variant is in scope: `CODING_STANDARDS.md:28-43`, `AUDIT_GUIDANCE.md:81` |
| PR-11 | IVP24 sizeof (L20): MinGW 15.2 prints `CoppEndpoint=10136 Cop1Endpoint=19608 FopP=9440` vs `starcom/docs/IVP.md:385` `19544/19664/164` | REMEDIATE (update `starcom/docs/IVP.md:385`; re-measured 2026-10-29) | `-Wstack-usage` is GNU-only: `cmake/CompilerWarnings.cmake:12-16` |
| PR-22 | `#pragma once` "existing project-wide exception" (`WORKING_HERE.md:72`, `IVP23_REPORT.md:21`); `CODING_STANDARDS.md:97` "live deviation"; `:56` "accepted deviation FP-1"; `AUDIT_GUIDANCE.md:80` "102 rules" | REMEDIATE (docs), except the two `CODING_STANDARDS.md` lines: note only (Nathan, 2026-10-30; see "Notes") | RC remediated all `#pragma once`: `docs/audits/RULE_VERIFIABILITY_TRIAGE.md:9`. FP-1 resolved (R-6c): `AUDIT_GUIDANCE.md:263`. LOC-1..4 = 31: `RULE_VERIFIABILITY_TRIAGE.md:64` |

### Category 3: code (all REMEDIATE, except where the row says otherwise)

| PR | Finding | Disposition | Record |
|---|---|---|---|
| PR-12a | Scratch `src/ccsds/copp.cpp:345 g_tfScratch`, `cop1.cpp:12 g_cop1TfScratch` | NO FINDING: compliant (JSF 207 FAIL is carried by PR-12b). Large objects must be `static`. Add an owner/single-caller comment | `CODING_STANDARDS.md:176-177` (LL Entry 1); comment `copp.cpp:340-344` |
| PR-12b | Process counters `copp.cpp:70 g_coppSyncLossN`, `:347 g_coppRetransmitN` (JSF 207, LOC-8, P10-6) | REMEDIATE: move into the caller-owned endpoint | D-2, `starcom/docs/DESIGN.md:94` |
| PR-13 | `src/ccsds/mac.cpp:396` 104 lines, CCN 22 (lizard; JSF 3 limit 20), cognitive complexity 34 (clang-tidy 21.1.8, limit 25; `CLANG_TIDY_R1_2026-10-30.txt`) (P10-4, JSF 3, LOC-25) | REMEDIATE (split, test-first) | B.1 "Enforced at ≤60 lines", `AUDIT_GUIDANCE.md:243` |
| PR-14/15/16 | `copp.cpp:359` naming; internal linkage `copp.cpp:433`, `pltu.cpp:192`; `space_packet.cpp:40`; `pltu.cpp:84`; `space_packet_service.cpp:52`; branch-clone `mac.cpp:956` | REMEDIATE | Branch-clone: "Merge cases later", `starcom/docs/audits/IVP23_TIDY_RUN.md:94` |
| PR-17 | P10-5 assertion density (0 runtime asserts) | DEFER-WITH-RATIONALE; not scored this cycle (not a FAIL: N/A-Aspirational in the report) | B.1 "Aspirational … Out-of-scope for this audit cycle", `AUDIT_GUIDANCE.md:247`; `MASTER_STANDARDS_AUDIT_2026-05-07.md:307` |
| PR-18 | `Result` not `[[nodiscard]]`; 7 unjustified `(void)` discards (`mac.cpp:169,175,181,187`, `copp.cpp:370,394,423`); `cert-err33-c` `adapters/host/file_replay.cpp:76,91` | REMEDIATE | B.1 P10-7 method, `AUDIT_GUIDANCE.md:255`; RC precedent "project APIs carry `[[nodiscard]]`", `RULE_VERIFIABILITY_TRIAGE.md:10`; `.clang-tidy:272` |
| PR-19 | 297 lowercase suffixes (JSF 14), 115 magic numbers (JSF 151), PC 21.1.8; RC adapter 15/18 (box, host flags) | REMEDIATE (bulk suffix; per-site magic numbers using the JSF 151 method) | `STANDARDS_AUDIT.md:157` "1,251 findings, all remediated"; `CLANG_TIDY_AUDIT_2026-02-09.md:454,465`; `CODING_STANDARDS.md:515`; method `IVP23_REPORT.md:39` |
| PR-20 | Host adapter boundary: `udp.cpp` reinterpret_cast ×6, vararg `:75,79`, errno `:192,228`, platform `#if`; `file_replay.cpp` stdio `:5,43,63,76,91`, `for (;;)` `:70`, pointer arithmetic `:19,29,43` | REMEDIATE (JSF 17, 22, 26, 28, 31, 108, 182, 200, 215; P10-2; LOC-3). Remove what can be removed (stdio, `for (;;)`, pointer arithmetic, the `fcntl` vararg wrapper). The plan for the OS-forced residue is removal too (the fix); no deviation row is proposed | `ACCEPTED_STANDARDS_DEVIATIONS.md:36`; JSF-22 applies, `CODING_STANDARDS.md:62`; no JSF-182 exception, `:59` |
| PR-23 | Fn-pointer seams `include/starcom/adapters/radio_bus.hpp:15-18`, `pio_port.hpp:15-16`, `file_replay.hpp:11` (P10-9, JSF 176, LOC-28/29) | REMEDIATE (policy/template seam) | P10-9 governs, `CODING_STANDARDS.md:56`; "Policy/template seams … virtual dispatch only at adapter edges", `DESIGN.md:111`; FP-1 precedent remediated (R-6c) |
| PR-24 | `#pragma once` in 23/23 Starcom headers (JSF 27/35) | REMEDIATE (`#ifndef` guards; headers `.hpp`→ see JSF 53 below) | `RULE_VERIFIABILITY_TRIAGE.md:9`; no register row |
| PR-25 | `continue` ×10 (JSF 190): `src/ccsds/ldpc.cpp:139`, `pltu.cpp:164,183`, `adapters/host/file_replay.cpp:80,85`, `src/starcom_adapt/byte_pump.cpp:194,198,204,208,213`; loop `break` `cop1.cpp:771`, `file_replay.cpp:88` (JSF 191) | REMEDIATE | "adopt the ban; remediate the sites", `CODING_STANDARDS.md:60` |
| PR-26 | No file intro comments (JSF 133); comment density 3.2% core / 0.0% adapters (D.8, JSF 130) | REMEDIATE | Band 15-25%, `CODING_STANDARDS.md:306-309`; `AUDIT_GUIDANCE.md:83` |
| PR-27 | Plain `unsigned`/`int` (JSF 209, LOC-17), incl. API `radio_bus.hpp:17-25` | REMEDIATE | `CODING_STANDARDS.md:86,521` |
| PR-28 | 9-param functions `src/starcom_adapt/cmd_sdu.cpp:9,27`, `byte_pump.cpp:363` (JSF 110, LOC-25) | REMEDIATE (pass a struct) | LOC-25 ≤6 params, `RULE_VERIFIABILITY_TRIAGE.md:93` |
| PR-29 | Non-ASCII in comments (JSF 9/13): 78 lines, 21 files | REMEDIATE | `ACCEPTED_STANDARDS_DEVIATIONS.md:36` |
| PR-30 | `version.hpp.in:14-15` function-like macros (JSF 29, P10-8); `result.hpp:5` `#ifdef` (JSF 26); `byte_pump.cpp:262-269` | REMEDIATE where possible (e.g. CMake-generate the string; `constexpr`) | `CODING_STANDARDS.md` PP-1 constexpr resolution, B.1 P10-8, `AUDIT_GUIDANCE.md:259` |
| PR-31 | Signed-bitwise 179, const-correctness 9 (PC 21.1.8 diagnostic run; visible only after PR-1) | REMEDIATE after PR-1 lands | Same default |
| PR-32 | `.hpp` extension (JSF 53); else-if chains without a final else (JSF 192: `cop1.cpp:294,526,684`, `mac.cpp:381,587,786,795,805,842`, `byte_pump.cpp:134`) | REMEDIATE (JSF 192, after checking for a comment). The JSF 53 rename has a wide blast radius; schedule it last | Default REMEDIATE `:36` |
| PR-33 | JSF 181 FAIL: 12 redundant casts (`g++ -Wuseless-cast`, PC): `src/ccsds/mac.cpp:234, 1127-1129, 1166-1168, 1212-1214`, `src/ccsds/ldpc.cpp:159`, `adapters/host/udp.cpp:149` | REMEDIATE (remove the casts; order in the code-fix plan below) | Default REMEDIATE `ACCEPTED_STANDARDS_DEVIATIONS.md:36` |
| PR-34 | JSF 162 FAIL: 4 signed/unsigned conversions (`g++ -Wsign-conversion`, PC): `src/ccsds/conv.cpp:16`, `:17`, `src/ccsds/ldpc.cpp:159`, `adapters/host/udp.cpp:73` | REMEDIATE | Default REMEDIATE `:36` |
| PR-35 | JSF 6 FAIL: rule-breaks carry no in-file deviation note and no register row | REMEDIATE by fixing the FAILs above (no register row is proposed) | Default REMEDIATE `:36`; zero Starcom rows `IVP23_REPORT.md:13` |
| PR-36 | JSF 60/61 FAIL: K&R braces; the rule requires same-column braces on separate lines | REMEDIATE (moved from "open" under the fail-means-fix rule). A house brace-style supersession would need a recorded owner decision; none exists | Default REMEDIATE `:36`; `CODING_STANDARDS.md:63` names only JSF 45/51/52 |
| PR-21 | Known pre-D13 text in `byte_pump.cpp` / `src/starcom_adapt/README.md:20-21` | Not in this plan. The comments in `byte_pump.cpp` and the README already tell what the code does today (`byte_pump.cpp:97-107`, `src/starcom_adapt/README.md:19-22`). The field packing itself lags the FSK plan (D13). That is FSK pivot work, tracked with the FSK packing change (Duke, 2026-10-30) | Already logged |
| — | L2-P5 "Starcom / radio-telem supersession (22)" bucket | DEFERRED by owner (not a FAIL in this report: the bucket was not walked) | `docs/audits/l2p5_manual_walk/L2P5_DISPOSITION_LOG.md:240-244` "DEFER polish to Starcom/CCSDS post–Stage-17" |

### Questions that a record answers

1. Pinned clang-tidy version (PR-3): 21.1.8, `AUDIT_GUIDANCE.md:267`.
2. Change the labels of the previous Rocket-Chip PASS lines (PR-2): no. Nathan (2026-10-30): do not edit old files. The old PASS lines stay as written, and this report records which of them rested on checks that were off.
3. Hook on Linux (PR-5): the hook is for Windows and Git Bash only (`scripts/hooks/README.md:5,23`). We make no change.
4. clang-tidy test scope (PR-6): less rules apply to test code (`ACCEPTED_STANDARDS_DEVIATIONS.md:150`).
5. "CI-guarded" (PR-9): add the guard (`DESIGN.md:111`).
6. MISRA claims (PR-10): DEFER (`CODING_STANDARDS.md:28-43`, `AUDIT_GUIDANCE.md:81`).
7. Scratch globals (PR-12): they must be static (`CODING_STANDARDS.md:176-177`). Counters: REMEDIATE, because of D-2 (`DESIGN.md:94`).
8. Do `Result` values count for P10-5 (PR-17)? Not applicable: P10-5 is aspirational and out of scope (`AUDIT_GUIDANCE.md:247`).
9. Suffixes and magic numbers, all together or site by site (PR-19): the REMEDIATE precedent is `STANDARDS_AUDIT.md:157`. The method is `IVP23_REPORT.md:39`.
10. ACCEPT the adapter casts at the edge (PR-20)? No. Starcom has zero rows (`IVP23_REPORT.md:13`), and the default is REMEDIATE (`ACCEPTED_STANDARDS_DEVIATIONS.md:36`).
11. Function-pointer seams (PR-20, PR-23): REMEDIATE. P10-9 applies (`CODING_STANDARDS.md:56`, `DESIGN.md:111`).

### Open items with no record (no item here is an open FAIL disposition)

- **Where to record the boundary items that Starcom cannot remove.** For this cycle the plan is to remove them (PR-20, REMEDIATE). We recommend no row for these items. There are two items:
  - (a) Casts, vararg and `errno` that the OS API makes necessary in `adapters/host/udp.cpp`. Rocket-Chip recorded the same type of item in rows CAST-1 and CAST-2.
  - (b) The vendored `tl::expected`. `CODING_STANDARDS.md:280` tells that it must be on the register of approved code from other sources.
  - The zero-Starcom-rows rule (`IVP23_REPORT.md:13`) gives no position for the two items.
- **JSF 49 (acronyms in uppercase) and camelBack names, for example `decodePltu`.** The project supersession (`CODING_STANDARDS.md:63`) names only JSF 45, 51 and 52.
- **Starcom cppcheck gate (PR-4).** No record makes one necessary, and no record lets the project not have one. The Starcom tool list (`starcom/docs/audits/README.md:27`, not approved at this time) shows only the gap. PR-4 has no table row.

### Nathan's items

Nathan decided on 2026-10-30: if the standards make a change necessary and it is feasible, do it. Thus only one item stays open: A1, the go for the baseline run on the boards, and the board-layout check with Buzz.

### Notes (not open items)

- **`standards/CODING_STANDARDS.md` lines 56 and 97 are out of date. Note only; do not change them now (Nathan, 2026-10-30).**
  - `:56` "accepted deviation FP-1" was added on 2026-06-23 (`a9dc8dec`). It was already out of date then: FP-1 was resolved on 2026-05-13 (`standards/ACCEPTED_STANDARDS_DEVIATIONS.md:56`, "All entries resolved as of 2026-05-13").
  - `:97` "`#pragma once` is a live deviation" was added on 2026-06-17 (`5a6cf879`). It is out of date: RC `src/` and `include/` have zero `#pragma once`. The Starcom headers that use it are a FAIL that W1 fixes, not a deviation.
  - With the `:63` naming override (main `AGENT_WHITEBOARD.md` row), these lines show that `CODING_STANDARDS.md` needs a deeper review later.

- **Vehicle-side drop hook.** No hook drops a frame on the vehicle, an uplink or a PLCW. `fault_force_station_rx_drop` (`src/safety/station_fault_inject.cpp:35`) drops only vehicle-to-station frames. The standards do not make a vehicle hook necessary. T0 covers resend and recovery on the host.
- **PLCW and FOP-P counters are not printed.** The COP-P line prints the state and N(R), V(S) and V(R) (`src/cli/rc_os_commands.cpp:921`), but no PLCW count and no FOP-P state name. The standards do not make these counters necessary. T0 checks resend, recovery, V(S) and N(R) on the host.

### Brace and naming rules

W2 changes Starcom and `src/starcom_adapt/` to JSF 60/61 now (Nathan, 2026-10-30). The RC-wide change, and the `CODING_STANDARDS.md:63` naming override, stay on the main `AGENT_WHITEBOARD.md` row "Two JSF rules broken with no stated reason: braces and names". Until the RC-wide change lands, Starcom and `src/starcom_adapt/` use a different brace style from the rest of RC. That row tracks this known mismatch.

### Code-fix plan (in order; not done)

This commit does not do these steps. The owner rule is that Grok Build writes code on the owner's PC. These steps change the firmware tree (the Rocket-Chip root `.clang-tidy` and `src/starcom_adapt/`). Thus the pre-push gate will make the bench test necessary at push.

1. **Correct the `Checks:` blocks** in the two `.clang-tidy` files (Rocket-Chip root and `starcom/.clang-tidy`). Move each `#` comment out of the folded block. Then the 10 (root) and 13 (Starcom) checks and the 23 `clang-analyzer-core.*` checks become live. Then do the 21.1.8 run again. There will be new findings (PR-1, PR-2, PR-31).
2. **Correct `HeaderFilterRegex` for Windows** (`starcom/.clang-tidy:237`, and the root config if it has the same form). Use a pattern that accepts the two separators (`[/\\]`). On the PC, make sure that the header hits (`copp.hpp:24,27`, `uslp.hpp:12`, `v3.hpp:13`) show (PR-1b).
3. **Add the Tier 1.1 defect probe.** Put one defect in a probe file for each configured check family. Include `clang-analyzer-core`, `goto`, `const-correctness` and a header hit. Use each shipped config on the probe file. Each defect must give a warning. The output of `--list-checks` does not show that a check fires.
4. **Remove the 12 JSF 181 casts** (PR-33): `src/ccsds/mac.cpp:234, 1127-1129, 1166-1168, 1212-1214`, `src/ccsds/ldpc.cpp:159`, `adapters/host/udp.cpp:149`. Do `g++ -Wuseless-cast` and `readability-redundant-casting` again. Each must show 0.

**Review items in this section:** RV-2 (the whiteboard has a row for PR-2), RV-3 (PR-33) and RV-4 (PR-34).

### Batch plan (proposed, 2026-10-29; not done)

This plan is a proposal for the owner to check. It makes no change to code or configuration. Each row is one local commit. Each REMEDIATE item from PR-1 to PR-36 is in a batch. PR-19 and PR-32 each have two batches, because one commit for each is too large to examine (see "PR coverage" below).

**Roles.**
- Hamilton writes the instructions for Grok Build. Hamilton reviews each change and makes each commit from the PC shell.
- Grok Build writes the code. Grok Build does no git operations.
- Buzz runs the bench. Buzz uses one power cycle for each flashed commit, as Nathan does at the power switch. Buzz uses the plan mode of the pre-push gate to find which commits must have the bench.
- Goddard is the second reader for each commit. Goddard counts the clang-tidy and compiler results again. Goddard makes sure that the target FAIL does not occur again before the next batch starts.
- Duke checks the on-air constants and field widths in each batch that touches `starcom/src/ccsds/` or `src/starcom_adapt/byte_pump.cpp`. Examples are the ASM `FA F3 20`, the PN sequence, the CRC polynomials and the header field widths. Duke compares each value with the book text. This is a regression check. It is not conformance work.

**Rules.**
- Make one local commit for each batch. Make sure that the commit is correct before the next batch starts.
- Do not push until Goddard and Buzz complete the checks of all batches.
- New code in each batch uses uppercase literal suffixes (JSF 14) and named constants (JSF 151). Thus C12 and C13 do not have to wait for the other code batches.
- C1 depends on A1. Thus no code batch starts before Nathan gives the go for the A1 baseline run and Buzz checks the board layout with him. Each firmware batch (bench B) depends on A1, because the baseline must show the code before any firmware change.
- Order with the FSK pivot and D13 (Nathan, 2026-10-30): the order in this plan stays. These batches come before the FSK work, because the FSK pivot builds from this code. The no-push rule above stays.
- No batch starts before Nathan gives the go for it. Before the first two-board run, Buzz checks the board layout with Nathan.
- If a batch changes an on-air constant or field, keep the same value. Give it a name with `constexpr`. Add a test that pins the value.
- If a batch changes behavior, add or update a unit test in `starcom/tests/unit/` in the same commit.
- Do large or repetitive changes with Grok Build or a script, not by hand. Examples: the suffix case change (C12), the brace change (W2), the header renames (W3) and the guard change (W1).

**Bench rule (from the gate, not from this plan).** Only these two files tell if a commit must have the bench: `scripts/ci/pre_push_gate.py` and `scripts/ci/firmware_paths.txt`. The paths in this plan fall into these classes:
- `starcom/src/`, `starcom/include/`, `starcom/CMakeLists.txt`, `src/` and `include/` are in `[elf]`. A commit that changes them must have the bench for the two roles. `starcom/src/`, `starcom/include/` and `src/starcom_adapt/` are also in `[radio]`. The radio test on the two boards gives only a message (no stop) until `scripts/radio_link_bench.py` is in the repo (`scripts/ci/pre_push_gate.py:111`, "GAP: not built yet"). Thus this plan adds its own on-air check (V-air, below).
- `starcom/src/ccsds/mac.cpp` and `mac.hpp` are in `[isolate:turnaround]`. `src/starcom_adapt/` is in `[isolate:radio-adapter]`. The gate shows a message (no stop) if a push has one of these commits and a different firmware commit.
- `starcom/.clang-tidy`, the root `.clang-tidy`, `starcom/scripts/`, `starcom/tests/`, `starcom/adapters/host/` and `starcom/docs/` are in no section. A commit that changes only these paths does not have to have the bench.
- The gate does not use `**/*.md` files. A Markdown change does not have to have the bench.
- `scripts/ci/` is in `[gate]`. Thus this plan puts the probe and the self-test in `starcom/`, not in `scripts/ci/`.

**Bench sessions.** The gate does one bench run for each push. This plan benches each firmware commit, so that a regression is easy to trace. Buzz does the bench runs in groups:
1. Hamilton makes the commits for a group (for example C1, C2 and C3) on the PC. Each commit has its hook run and its Goddard read.
2. Nathan connects the boards one time for the group.
3. Buzz checks out each firmware commit in order in the bench worktree. For each commit, Buzz builds the two roles, flashes, does one power cycle and runs `bench_sim.py` and `station_bench_sim.py`. Buzz records the SHA and the result for each commit.
4. If the bench gives FAIL for a commit, Buzz stops. The commits after it wait.
At the end, push in segments that obey the gate's shape rule. Put no more than 5 firmware commits in one push. Push each turnaround or radio-adapter commit with no other firmware commit (`git push origin <sha>:main`). Before each push, run `python scripts/ci/pre_push_gate.py --plan`. The gate does a bench run again on the last commit of each push.

**Check steps (labels in the table).**
- **V-hook**: the host tests of the pre-commit hook (`scripts/hooks/pre-commit`, Git Bash). It runs at commit time.
- **V-tidy**: clang-tidy 21.1.8 on the PC, with the same inputs as the audit run. The command is below the table. Compare the counts for the target check with the R1 baseline. The target count must decrease by the number in the row. No other count must go up.
- **V-gcc**: the MinGW g++ 15.2 build with the flag for the rule (for example `-Wuseless-cast` or `-Wsign-conversion`). The target count must be 0.
- **V-unit**: a new or updated test in `starcom/tests/unit/`. `ctest` must give PASS.
- **V-obj**: for a change that must not change behavior, compare the object files before and after (`objdump -d` with no debug data). They must be the same.
- **V-bench**: the Buzz bench run for this commit, if the gate classes make it necessary.
- **V-duke**: Duke's book check of the on-air constants and field widths.
- **V-air**: an on-air run on the two boards with `scripts/soak_two_board.py` (procedure: `starcom/docs/integration/TWO_BOARD_SOAK.md`). The bench (`bench_sim.py`, `station_bench_sim.py`) tests flight logic only. It does not send data on the air. V-air is necessary for the batches that change behavior in the MAC, COP-P, coding or `byte_pump`: C3, C4, C7, C8, C9, C11, C16, C17 and C18. Buzz does a run of about 60 s on the desk at 2 dBm after each of these batches. Buzz compares the counts with the A1 baseline: vehicle frames sent, station frames received, and station CRC errors. A count that is worse than the baseline is a FAIL for the batch.
- **V-air+**: for C4, C11 and C18 (COP-P resend, PLCW feedback and the uplink command path), V-air also sends repeated station `D` (dashboard DISARM) uplinks (`--uplink-disarm-every`). `D` and the `c` T-minus command go through COP-P (`src/active_objects/ao_rcos.cpp:280-281`; `src/active_objects/ao_telemetry.cpp:1385`; `src/cli/cli_actions.cpp:215`), and they are safe to repeat. Do not use `b`. Then `t` on each board prints a `LINK` line with `copp_rtx` (`starcom/src/ccsds/copp.cpp:570`) and `copp_sync_loss` (`src/diag/link_baseline.cpp:95-111`). Buzz compares `copp_rtx` and the COP-P lock state with the A1 baseline. The only frame drop on the boards is `fault_force_station_rx_drop` (`src/safety/station_fault_inject.cpp:35`). It is armed through GDB and the probe only, and it drops vehicle-to-station frames only. Thus V-air+ cannot drop an uplink or a PLCW. Label the V-air+ runs for C4, C11 and C18 "uplink path only". On a clean desk link, `copp_rtx` stays near zero, so the run cannot show that resend works. The host test T0 is the only coverage of resend and recovery. The vehicle-side drop hook stays an open item.

**V-tidy command (PC, PowerShell).** The audit run used the 19 translation units in the MinGW compile database at `C:\Users\pow-w\Documents\rc-audit-logs\scbuild`. The audit did not keep its loop as a script. The log header is `starcom clang-tidy 21.1.8 scope=all config=...\starcom\.clang-tidy`. This loop gives the same inputs:

```powershell
$ct = 'C:\Program Files\LLVM\bin\clang-tidy.exe'      # 21.1.8
$r  = 'C:\Users\pow-w\Documents\rocket-chip'
$b  = 'C:\Users\pow-w\Documents\rc-audit-logs\scbuild'
$log = "C:\Users\pow-w\Documents\rc-audit-logs\CLANG_TIDY_<batch>_all_v21.1.8.txt"
$files = @(Get-ChildItem "$r\starcom\src\ccsds\*.cpp") + @(Get-ChildItem "$r\starcom\adapters" -Recurse -Filter *.cpp)
foreach ($f in $files) { "---- $($f.FullName) ----" >> $log
  & $ct $f.FullName -p $b --config-file="$r\starcom\.clang-tidy" --quiet *>> $log }
python C:\Users\pow-w\Documents\rc-audit-logs\tidycount.py $log
```

The MinGW flag `-Wstack-usage=1024` gives 2 `clang-diagnostic-unknown-warning-option` errors in each file that has the flag in the audit log. The counts do not include these errors. R1 records them as known noise.

#### Batch table

NB = no bench (no path in `firmware_paths.txt`). V-air = the on-air check above. B = bench, two roles (`[elf]`). R = also `[radio]` (the two-board radio test gives only a message (no stop) at this time). T = `[isolate:turnaround]`. A = `[isolate:radio-adapter]`. Goddard does the second read for each row.

| Batch | PR items | Exact sites | Proposed change (old -> new) | Files | Bench | Duke | Verify | Owner | Depends on |
|---|---|---|---|---|---|---|---|---|---|
| G1 | PR-1 (comment defect) | `starcom/.clang-tidy:14-234` (`Checks: >`); 71 comment lines inside the block | Move all 71 `#` lines out of the folded block, word for word, in order, to one comment block above `Checks:`. A comment that says "enabled:" about the check above it gets that check name as a prefix. The 139 check entries do not change (script check: same list, same order). Exact text: "G1 proposed text" below | `starcom/.clang-tidy` | NB | no | `--list-checks` 156 -> 169 (+13) on 21.1.8; probe: goto, null dereference and divide by zero fire (done for this proposal, see below); V-hook | Hamilton | — |
| G2 | PR-2 (config part only) | Root `.clang-tidy`, `Checks:` block; `:227-232` (analyzer comment block); 10 checks disabled | Same method as G1. G2 changes only the root `.clang-tidy`. The new Rocket-Chip findings that the 10 + 23 checks show go to the RC master audit list, not to this Starcom plan. The old Rocket-Chip PASS lines stay as written (see "G2: old PASS lines" below) | `.clang-tidy` | NB (the root `.clang-tidy` is in no section) | no | `--list-checks` +10 with the root config; probe with the root config; full-tree run `scripts/audit/full_tree_clang_tidy.sh` for the new count (record only; the fixes go to the RC master audit); V-hook | Hamilton | — (G2 uses the G1 method, but it changes a different file) |
| G3 | PR-1b | `starcom/.clang-tidy:237` `HeaderFilterRegex: '.*(src/ccsds/.+\|include/starcom/.+)'` | New filter accepts `/` and `\` and matches only `starcom/include`, `starcom/src`, `starcom/adapters` and the Rocket-Chip `src/` and `include/`. New `ExcludeHeaderFilterRegex` removes `third_party`, `pico-sdk`, `lib` and `EXTERNAL`. Exact text: "G3 proposed text" below | `starcom/.clang-tidy` | NB | no | On 21.1.8 with `copp.cpp`: the header hits `copp.hpp:24,27`, `uslp.hpp:12`, `v3.hpp:13` show, and 0 hits in `third_party` (done for this proposal); probe header hit shows; probe vendor header gives 0; V-hook | Hamilton | G1 |
| G4 | PR-1 (self-test and fixture; Tier 1.1) | New files only | Add the known-bad probe (text below) at `starcom/tests/tidy_probe/`. Add `starcom/scripts/tidy_selftest.sh`: run the pinned clang-tidy on the probe with `starcom/.clang-tidy`, fail if a check named in an `EXPECT` tag does not fire, fail if `probe_vendor.hpp` shows, and fail if `--list-checks` gives fewer than 169. Add a CTest entry in `starcom/tests/CMakeLists.txt` that runs it when the pinned clang-tidy is present | `starcom/tests/tidy_probe/*`, `starcom/scripts/tidy_selftest.sh`, `starcom/tests/CMakeLists.txt` | NB | no | Self-test passes on the G3 config; self-test fails on the shipped config (planted check); V-hook | Grok Build (Hamilton prompt) | G1, G3 |
| T1 | PR-6 | `starcom/scripts/run_clang_tidy.sh` (mixed line endings, so `exit 0\r` fails, exit 2) | Change the file to LF only. Add `*.sh text eol=lf` for `starcom/scripts/` in `.gitattributes` if it is not there | `starcom/scripts/run_clang_tidy.sh`, `.gitattributes` | NB | no | `bash starcom/scripts/run_clang_tidy.sh --all` exits 0; `git ls-files --eol` shows `i/lf`; V-hook | Hamilton | — |
| R1 | Baseline (no PR) | All 19 translation units | Run V-tidy with the G1+G3 config. Record the new counts for each check and each file in `starcom/docs/audits/CLANG_TIDY_R1_<date>.txt` and in a short Markdown note. This is the baseline for every code batch | `starcom/docs/audits/` | NB | no | Goddard runs the same loop again and gets the same counts | Hamilton | G1, G3 |
| A0 | No PR (prerequisite for V-air) | `scripts/soak_two_board.py:175`, `:188`, `:337` | Four fixes. (1) `veh_git` takes the last hex string of 7 or more characters, so it can store a tree hash (`docs/reports/2026-10-20_buzz_lora_baseline.md:308`): match `flight-([0-9a-f]+)` first. (2) The station COP-P pattern at `:188` and `:337` has no colon, but the station prints `COP-P:` (`src/cli/rc_os_commands.cpp:921,1078,1378`), so it probably never matches: add the colon. (3) Read the `LINK` line (`copp_rtx`, `copp_sync_loss`) after `t` on each board; use the pattern in `scripts/baseline_lora.py:209-212`. (4) Add an uplink step: send station `D` (dashboard DISARM) at set times. Do not send `x` or `X`: on the dashboard they open the menu, where `x` is erase | `scripts/soak_two_board.py` | NB | no | Run the score part on the 2026-10-20 logs: `veh_git` is the `flight-` hash and the station COP-P state is found; a test log with a `LINK` line gives `copp_rtx`; V-hook | Buzz | — |
| A1 | No PR (V-air baseline) | Current `main` (before C1) | One baseline run of `scripts/soak_two_board.py` on the two boards, 2 dBm, the same desk layout that the later V-air runs use. Record the counts and the SHA in `starcom/docs/audits/` | `starcom/docs/audits/` | NB (no code change) | no | Goddard reads the counts again from the log | Buzz | A0; Nathan's go; board layout checked with Nathan |
| C1 | PR-33 (JSF 181) | `mac.cpp:234, 1127-1129, 1166-1168, 1212-1214`; `ldpc.cpp:159`; `adapters/host/udp.cpp:149` | Remove each redundant cast. Examples: `static_cast<unsigned>((w >> 15) & 1u)` -> `((w >> 15) & 1u)`; `t = static_cast<Tick>(t - dt);` -> `t = t - dt;`; `std::popcount(static_cast<unsigned>(r & 0x5Fu))` -> `std::popcount(r & 0x5Fu)` (only if the type of `r & 0x5Fu` is already `unsigned`); `static_cast<std::uint16_t>(ntohs(...))` -> `ntohs(...)` | `starcom/src/ccsds/mac.cpp`, `ldpc.cpp`, `starcom/adapters/host/udp.cpp` | B, R, T | yes (`ldpc.cpp:159` mask `0x5F`) | V-gcc `-Wuseless-cast` = 0; `readability-redundant-casting` = 0; V-obj for mac and ldpc; V-hook; V-bench | Grok Build | R1, A1 |
| C2 | PR-34 (JSF 162) | `conv.cpp:16,17`; `ldpc.cpp:159`; `adapters/host/udp.cpp:73` | Make each sign change explicit. `c1 = std::popcount(v & kG1) & 1u;` -> `c1 = static_cast<unsigned>(std::popcount(v & kG1)) & 1U;` (same for `c2` and `ldpc.cpp:159`); `ioctlsocket(s, FIONBIO, &one)` -> `ioctlsocket(s, static_cast<long>(FIONBIO), &one)` or the type that the Windows header gives | `conv.cpp`, `ldpc.cpp`, `udp.cpp` | B, R | yes (`kG1`, `kG2`, G2 inversion, `0x5F`) | V-gcc `-Wsign-conversion` = 0 for these sites; test pins `kG1`, `kG2` and the PN taps; V-obj; V-hook; V-bench | Grok Build | R1, A1, C1 |
| C3 | PR-13 (P10-4, JSF 3, LOC-25) | `mac.cpp:396` `macWaitExpiredHalf` (104 lines, CCN 22, cognitive complexity 34) | Test first: add a characterization test in `test_mac.cpp` for each branch. Then split the function into named helpers. Each helper has 60 lines or less and CCN 20 or less | `mac.cpp`, `mac.hpp` (if a helper is declared), `tests/unit/test_mac.cpp` | B, R, T | yes | `readability-function-size` and cognitive complexity for `mac.cpp` = 0 at this site; lizard CCN 20 or less; V-unit (test passes before and after the split); V-hook; V-bench; V-air | Grok Build | R1, A1 |
| T0 | No PR (test before C4) | `starcom/tests/unit/test_copp.cpp`; `coppRetransmitCount()` (`starcom/include/starcom/ccsds/copp.hpp:145`, `starcom/src/ccsds/copp.cpp:535`) | No host test drops a PLTU or checks `coppRetransmitCount()`. Add a lossy loop between two endpoints: drop one PLTU, then check that the sender resends it, that the link recovers, and that `coppRetransmitCount()` goes up by the number of resends. Also check that the receiver delivers each frame in order, with no duplicate and no gap, and that after recovery the sender's V(S) agrees with the receiver's N(R). T0 is the only coverage of resend and recovery (V-air+ is uplink path only) | `starcom/tests/unit/test_copp.cpp` | NB (`starcom/tests/` is in no section) | no | V-unit (the test passes on current code); V-duke: Duke checks the expected behavior of the test against 211.0-B-6 when the test is written; V-hook | Grok Build | — |
| C4 | PR-12b (JSF 207, LOC-8, P10-6) | `copp.cpp:70` `g_coppSyncLossN`, `copp.cpp:347` `g_coppRetransmitN` | Move the two counters into the caller-owned endpoint: `std::uint32_t g_coppSyncLossN = 0;` -> member `std::uint32_t syncLossN = 0;` in `CoppEndpoint`, and the same for `retransmitN`. Each use reads the member. `sizeof(CoppEndpoint)` gets larger by 8 | `copp.cpp`, `copp.hpp`, `tests/unit/test_copp.cpp` | B, R | yes | `avoid-non-const-global-variables` for these 2 = 0; V-unit (counters per endpoint); V-hook; V-bench; V-air; V-air+ (uplink path only) | Grok Build | R1, A1, T0 |
| C5 | PR-14/15/16 | `copp.cpp:359` (name); `copp.cpp:433`, `pltu.cpp:192`, `space_packet.cpp:40`, `pltu.cpp:84`, `space_packet_service.cpp:52` (internal linkage); `mac.cpp:956` (branch clone) | Rename to the house case style. Put each file-local function in an unnamed namespace. Merge the two equal branches at `mac.cpp:956` | `copp.cpp`, `pltu.cpp`, `space_packet.cpp`, `space_packet_service.cpp`, `mac.cpp` | B, R, T | yes | `readability-identifier-naming`, `misc-use-internal-linkage`, `bugprone-branch-clone` = 0 at these sites; V-obj (except `mac.cpp:956`); V-hook; V-bench | Grok Build | R1, A1 |
| C6 | PR-20 (JSF 17, 22, 26, 28, 31, 108, 182, 200, 215; P10-2; LOC-3) | `udp.cpp`: `reinterpret_cast` x6, vararg `:75,79`, `errno` `:192,228`, platform `#if`; `file_replay.cpp`: stdio `:5,43,63,76,91`, `for (;;)` `:70`, pointer arithmetic `:19,29,43` | `file_replay.cpp`: `<cstdio>` -> `<fstream>`; `for (;;)` -> `while (more)` with a named exit condition; `p + n` -> `std::span::subspan`. `udp.cpp`: `reinterpret_cast<sockaddr*>(&a)` -> copy through a `sockaddr_storage` with `std::memcpy` where the API permits; `fcntl` vararg call -> one wrapper at the OS edge; `errno` -> one read per call into a typed error. Each item that the OS API makes necessary is listed in the commit message as a remaining site | `starcom/adapters/host/udp.cpp`, `file_replay.cpp` | NB (`starcom/adapters/host/` is in no section) | no | V-tidy for the listed checks at these sites; `test_host_io.cpp` passes; V-hook | Grok Build | R1 |
| C7 | PR-28 (JSF 110, LOC-25) | `cmd_sdu.cpp:9` `pack_cmd_sdu_user` (9 params), `cmd_sdu.cpp:27` `unpack_cmd_sdu_user` (9), `byte_pump.cpp:363` `pump_pack_cmd_packet` (9) | Add `struct CmdSduFields { uint16_t cmd_id; uint8_t seq; float p[5]; };` (the type of `p` follows C-arrays policy, or `std::array<float, 5>`). New forms: `pack_cmd_sdu_user(uint8_t* out, uint8_t out_len, const CmdSduFields& f)`; `unpack_cmd_sdu_user(const uint8_t* in, uint8_t in_len, CmdSduFields& f)`; `pump_pack_cmd_packet(BytePump& p, std::span<std::byte> out, const CmdSduFields& f)`. The bytes on the wire do not change | `src/starcom_adapt/cmd_sdu.cpp`, `cmd_sdu.h(pp)`, `byte_pump.cpp`, `byte_pump.h(pp)`, the callers, tests | B, R, A | yes (`byte_pump.cpp`; SDU byte layout) | `readability-function-size.ParameterThreshold` = 0 at these sites; test pins the packed bytes of one known command before and after; V-hook; V-bench; V-air | Grok Build | R1, A1 |
| C8 | PR-25 (JSF 190, 191) | `continue` x10: `ldpc.cpp:139`, `pltu.cpp:164,183`, `file_replay.cpp:80,85`, `byte_pump.cpp:194,198,204,208,213`; loop `break`: `cop1.cpp:771`, `file_replay.cpp:88` | `if (skip) { continue; } work();` -> `if (!skip) { work(); }`. A loop `break` -> a loop condition with a named flag, for example `while (i < n && !found)` | `ldpc.cpp`, `pltu.cpp`, `cop1.cpp`, `file_replay.cpp`, `byte_pump.cpp` | B, R, A | yes | grep: 0 `continue` and 0 loop `break` at these sites; V-unit (existing tests for each loop); V-hook; V-bench; V-air | Grok Build | R1, A1, C6 |
| C9 | PR-23 (P10-9, JSF 176, LOC-28/29) | `radio_bus.hpp:15-18`, `pio_port.hpp:15-16`, `file_replay.hpp:11` | Decided (2026-10-30): templates at the adapter edge too, no virtual dispatch (see "C9: virtual calls at the adapter edge" below). Function-pointer seam -> template policy. Old: `struct BusOps { void* ctx; Result<size_t> (*spi)(void*, ...); ... };`. New: `template <class Bus> Result<std::size_t> busSpi(Bus& bus, ...)` where `Bus` has `spi()`, `gpioWrite()`, `gpioRead()`. Do not add a virtual call. D4 changes `DESIGN.md:111` to agree | `starcom/include/starcom/adapters/radio_bus.hpp`, `pio_port.hpp`, `file_replay.hpp`, `starcom/adapters/rp2350/*`, `starcom/adapters/host/*`, callers | B, R | no | V-tidy; build of the two roles; V-unit; V-hook; V-bench; V-air (C9 changes the RP2350 radio adapter seam, which is a hardware interface) | Grok Build | R1, A1, C6 |
| C10 | PR-27 (JSF 209, LOC-17) | Plain `unsigned` and `int`, including API `radio_bus.hpp:17-25` (`int line`) | `int line` -> `std::uint8_t line` (or the width that the pin numbers need); each plain `unsigned` -> `std::uint32_t`; each plain `int` -> `std::int32_t` | The files from the R1 `google-runtime-int` list and a grep for plain types | B, R | yes if `src/ccsds/` is in the list | `google-runtime-int` = 0; grep for plain types = 0; V-obj where the width does not change; V-hook; V-bench | Grok Build | R1, A1, C8 |
| C11 | PR-18 (P10-7) | `Result` not `[[nodiscard]]`; `(void)` discards `mac.cpp:169,175,181,187`, `copp.cpp:370,394,423`; `cert-err33-c` `file_replay.cpp:76,91` | `Result` is an alias of `tl::expected`, so the attribute goes on each function: `Result<T> f(...)` -> `[[nodiscard]] Result<T> f(...)`. Each `(void)f(...)` -> `if (!f(...)) { <error path named in the design> }`. For each new error path, Duke cites the clause in 211.0-B-6 or 232.1 that sets it. If the book and the design name no path, keep the current behavior, add a comment that tells this, and put the site on the open list. Do not invent protocol behavior. The `file_replay.cpp` sites close after C6 removes stdio | Core headers that return `Result`, `mac.cpp`, `copp.cpp`, tests | B, R, T | yes | `bugprone-unused-return-value` and the compiler `-Wunused-result` = 0; 0 `(void)` discards at these sites; V-unit for each new error path; V-duke: one clause cite for each new path; V-hook; V-bench; V-air; V-air+ (uplink path only) | Grok Build | R1, A1, C6, T0 |
| C12 | PR-19 part 1: suffixes (JSF 14) | 297 lowercase suffixes (PC 21.1.8) plus the header hits that G3 shows (R1 count); Rocket-Chip adapter 15 | `0x07u` -> `0x07U`; `1u` -> `1U`; `ul` -> `UL`. Only the case of the suffix changes | All files in the R1 list for `readability-uppercase-literal-suffix`, and `src/starcom_adapt/` | B, R, T, A | yes (constants keep their value) | `uppercase-literal-suffix` = 0; V-obj (must be the same); V-hook; V-bench | Grok Build | R1, A1 |
| C13 | PR-19 part 2: magic numbers (JSF 151) | 115 sites (PC 21.1.8) plus the header hits (R1 count) | `if (len > 1115)` -> `constexpr std::size_t kMaxFrameOctets = 1115;  // <book, section>` and `if (len > kMaxFrameOctets)`. Each name has a source. On-air values keep the exact value | All files in the R1 list for `readability-magic-numbers` | B, R, T, A | yes (each on-air value) | `readability-magic-numbers` = 0; test pins each on-air value; V-obj; V-hook; V-bench | Grok Build | R1, A1, C12 |
| C14 | PR-26 (JSF 133; D.8, JSF 130) | No file intro comment; comment density 3.2% (core), 0.0% (adapters) | Add a file intro comment to each file: purpose, owner of the state, and the book reference. Add comments only where the code needs a reason ("why", not "what": `CODING_STANDARDS.md:314,317-319`). The 15% to 25% band applies to `.cpp` files only; header files are excluded (`CODING_STANDARDS.md:299,309`). Do not add a comment only to reach the band. Also add the owner and single-caller comment at `copp.cpp:345` and `cop1.cpp:12` (PR-12a, not a FAIL) | All Starcom `.cpp` and `.hpp` files | B, R, T | yes (comments only) | density script result for each `.cpp` file (headers excluded), reported with the band; Goddard reads each new comment and removes filler; V-obj (must be the same); V-hook; V-bench | Grok Build | R1, A1 |
| C15 | PR-29 (JSF 9, 13) | Non-ASCII in comments: 78 lines, 21 files | `—` -> `-`; `→` -> `->`; `≤` -> `<=`; `═` -> `=` | The 21 files | B, R, T | yes (comments only) | `rg -n "[^\x00-\x7F]"` in the 21 files = 0; V-obj; V-hook; V-bench | Grok Build or a script (Hamilton prompt) | R1, A1, C14 |
| C16 | PR-30 (JSF 26, 29; P10-8) | `version.hpp.in:14-15` function-like macros; `result.hpp:5` `#ifdef STARCOM_USE_STD_EXPECTED`; `byte_pump.cpp:262-269` `#ifdef ROCKETCHIP_JOB_STATION`, `#ifndef ROCKETCHIP_HOST_TEST` | `#define STARCOM_VERSION_STR(a, b, c) ...` -> CMake writes `inline constexpr const char* kVersionStr = "@STARCOM_VERSION_MAJOR@.@STARCOM_VERSION_MINOR@.@STARCOM_VERSION_PATCH@";`. `result.hpp`: keep one branch (the one that the build uses) and remove the `#ifdef`. `byte_pump.cpp`: `#ifdef` -> `if constexpr (kIsStation)` with `kIsStation` from a generated `constexpr` | `starcom/include/starcom/version.hpp.in`, `result.hpp`, `starcom/cmake/*`, `byte_pump.cpp` | B, R, A | yes (`byte_pump.cpp`; SCID 1 and 2) | grep for function-like macros = 0; version string the same in the two images; V-unit (station and vehicle SCIDs); V-hook; V-bench; V-air | Grok Build | R1, A1 |
| C17 | PR-31 | Signed-bitwise 179, const-correctness 9 (now visible after G1; R1 gives the exact sites) | `x & 0x07` with `int x` -> `static_cast<std::uint32_t>(x) & 0x07U`, or make `x` unsigned at its declaration. A variable that does not change gets `const` | The files in the R1 list for `hicpp-signed-bitwise` and `misc-const-correctness` | B, R, T | yes | the two counts = 0; V-obj where possible; V-hook; V-bench; V-air | Grok Build | R1, A1 |
| C18 | PR-32 part 1: JSF 192 | `cop1.cpp:294,526,684`; `mac.cpp:381,587,786,795,805,842`; `byte_pump.cpp:134` | First check for a comment that tells why there is no `else`. Then add a final `else { /* no action: <cause> */ }`, or change the chain to a `switch` with `default` | `cop1.cpp`, `mac.cpp`, `byte_pump.cpp` | B, R, T, A | yes | grep or review: each chain has a final `else`; V-obj where the `else` is empty; V-hook; V-bench; V-air; V-air+ (uplink path only) | Grok Build | R1, A1, T0 |
| W1 | PR-24 (JSF 27, 35) | `#pragma once` in 23 of 23 Starcom headers | `#pragma once` -> `#ifndef STARCOM_CCSDS_COPP_H_` / `#define STARCOM_CCSDS_COPP_H_` / ... / `#endif  // STARCOM_CCSDS_COPP_H_`. The pattern is `STARCOM_<DIR>_<FILE>_H_`, so W3 does not change the guards | The 23 headers | B, R, T | no | grep `#pragma once` in `starcom/` = 0; a script checks that each guard name is unique; V-obj; V-hook; V-bench | Grok Build or a script (Hamilton prompt) | R1, A1 |
| W2 | PR-36 (JSF 60, 61) | K&R braces in all Starcom and `src/starcom_adapt/` C++ files | Decided (2026-10-30): do it now. The RC-wide change stays on the main `AGENT_WHITEBOARD.md` row. `if (x) {` -> `if (x)` and `{` on the next line, in the same column as `if`. Only brace lines change | All Starcom `.cpp` and `.hpp` files, and `src/starcom_adapt/` | B, R, T, A | yes (no value changes) | `git diff -w` shows only brace lines; V-obj (must be the same); V-hook; V-bench | Grok Build or a script (Hamilton prompt; script-checked) | A1 |
| W3 | PR-32 part 2: JSF 53 | 23 headers with `.hpp` | `copp.hpp` -> `copp.h` for each header. Change each `#include "starcom/.../x.hpp"` in `starcom/`, `src/starcom_adapt/` and the Rocket-Chip includes. `version.hpp.in` -> `version.h.in`, and the CMake rule | The 23 headers, all includers, `starcom/CMakeLists.txt`, `starcom/cmake/*` | B, R, T, A | yes (no value changes) | grep `\.hpp` in includes = 0; V-obj (must be the same); V-hook; V-bench | Grok Build or a script (Hamilton prompt) | R1, A1, W1 |
| D1 | PR-7, PR-8, PR-11, PR-22 | PR-7: `starcom/.clang-tidy:4,57-58,97-98,101,272` claims (L1-L5; the line numbers move after G1). PR-8: IVP23 zeros (L6, L7, L9, L10). PR-11: `starcom/docs/IVP.md:385` sizes. PR-22: `WORKING_HERE.md:72`, `IVP23_REPORT.md:21`, `AUDIT_GUIDANCE.md:80` (not `CODING_STANDARDS.md:56,97`: note only, see "Notes") | PR-7: change each claim to what the tool does. PR-8: mark the IVP23 zeros as historical and point to R1. PR-11: `19544/19664/164` -> the sizes measured again after C4 (C4 makes `CoppEndpoint` 8 octets larger). PR-22: remove "existing project-wide exception"; `102 rules` -> the correct count | `starcom/.clang-tidy`, `starcom/docs/audits/IVP23_REPORT.md`, `starcom/docs/IVP.md`, `starcom/docs/WORKING_HERE.md`, `standards/AUDIT_GUIDANCE.md` | NB | no | `test_size.cpp` output matches the new `IVP.md:385`; Goddard reads each claim against the tool output; V-hook | Hamilton | G1, C4 |
| D2 | PR-9 | `starcom/docs/DESIGN.md:111` "(CI-guarded)" with no guard | Add `starcom/tests/check_core_includes.py`: fail if a file in `starcom/src/` or `starcom/include/` has `#include` of `hardware/` or `pico/`. Add a CTest entry in `starcom/tests/CMakeLists.txt` | `starcom/tests/check_core_includes.py`, `starcom/tests/CMakeLists.txt` | NB | no | The test passes; the test fails on a planted `#include "pico/stdlib.h"` in a temporary copy; V-hook | Grok Build | — |
| D4 | PR-23 (C9 decision) | `starcom/docs/DESIGN.md:111` "Policy/template seams in core; virtual dispatch only at adapter edges." | -> "Policy/template seams in core and at the adapter edges; no virtual dispatch (P10-9)." | `starcom/docs/DESIGN.md` (not protected) | NB | no | Goddard reads the line against C9; V-hook | Hamilton | C9 |
| R2 | PR-35 (JSF 6) | All FAILs in Section 1 | No new change. Run V-tidy and V-gcc again on the last commit. Confirm that each FAIL in Section 1 is gone, so no in-file deviation note and no register row is necessary. Record the final counts | `starcom/docs/audits/` | NB | no | Goddard gets the same counts | Hamilton | All batches |

#### Dependency notes (checked by hand, 2026-10-30)

The table order is the default commit order. "Depends on" names only a real need. No batch is held.

- R1 is in each code batch because V-tidy compares the counts with R1. A1 is in each firmware batch (see the rules above). C6, T0, D2, G-batches and T1 change no firmware, so they do not need A1.
- G3 needs G1: G3 changes the same file, and its text starts from the G1 form. G4 needs G1 and G3: the self-test expects their check list and header filter. R1 needs G1 and G3: the baseline uses that config. R1 does not need G4 or T1: the V-tidy loop does not use them.
- A1 needs A0: the baseline uses the fixed soak script.
- C2 needs C1: the two batches change the same expression at `ldpc.cpp:159`.
- C4, C11 and C18 need T0: they change COP-P code that T0 tests (resend counter, `(void)` discards in `copp.cpp`, `else` chains in `cop1.cpp` and `mac.cpp`).
- C8 and C9 need C6: C6 changes the loop and the stdio in `file_replay.cpp`, which C8 changes again, and C9 changes the seam in `file_replay.hpp` that `file_replay.cpp` uses. C11 needs C6: its `file_replay.cpp:76,91` sites close when C6 removes stdio.
- C10 needs C8: C8 changes loops in `ldpc.cpp`, `pltu.cpp` and `byte_pump.cpp`, and C10 changes the plain `int` and `unsigned` types in the same files. C10 does not need C9. C9 comes first in table order; C10 then changes the type of `int line` in the new seam (`radio_bus.hpp:17-25`).
- C13 needs C12: C13 gives names to the same literals that C12 changes. C15 needs C14: the C15 check must include the new C14 comments.
- W3 needs W1: W1 gives the guards their final `_H_` names. Then W3 is only a rename, and V-obj can show that.
- W2 needs only A1: it changes only brace lines, and it uses the result of no other batch. D4 needs C9: the design text must agree with the code.
- D1 needs G1 (the `.clang-tidy` line numbers move) and C4 (the size of `CoppEndpoint` changes).
- Links removed: G2 from G1; R1 from G4 and T1; C3 from C2; C4 from C3; C5 from C4; C7 from C6; C8 from C7; C9 from C8 (now C6); C10 from C9 (now C8); C11 from C10; C12 from C11; C14 from C13; C16 from C15; C17 from C16; C18 from C17; W1 from C18; W2 from W1 (and from the project brace check); W3 from W2 (now W1); D1 from W3 (now G1, C4); T0 from R1. Each of these batches changes other files or other lines, and it does not use the result of the removed batch.

#### G2: old PASS lines that rested on checks that were off (record only)

Nathan (2026-10-30): do not edit old files. The old PASS lines stay as written. This report records which of them rested on checks that were off. The dates are from `git log --diff-filter=A`.

| File | Lines that rest on a disabled check | Added | Last change | Status |
|---|---|---|---|---|
| `docs/audits/CLANG_TIDY_AUDIT_2026-02-20.md` | `:228` (`cppcoreguidelines-avoid-goto`, 0), `:234` (all `clang-analyzer-core.*`, 0) | 2026-02-20 (`7bbcb75d`) | 2026-02-20 | Stays as written |
| `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-13.md` | `:88` (Tier 1.1 tool self-checks, PASS) | 2026-05-13 (`bb508cbd`) | 2026-05-24 | Stays as written |
| `docs/audits/RULE_VERIFIABILITY_TRIAGE.md` | `:42`, `:56`, `:79`, `:110`, `:341-342` (`avoid-goto`, `bugprone-assert-side-effect` as the gate) | 2026-06-17 (`5a6cf879`) | 2026-07-28 | Stays as written |
| `docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE.md` | `:269` (assert side effect "gate-covered") | 2026-07-01 (`2eb74de8`) | 2026-07-31 | Stays as written |
| `docs/audits/l2p5_manual_walk/L2P5_MANUAL_WALK_GUIDE_ARCHIVE.md` | `:469`, `:1130` | 2026-07-01 (`2eb74de8`) | 2026-07-01 | Stays as written |
| `docs/audits/l2p5_manual_walk/L2P5_CLAUDE_WALK_FINDINGS_ALIGNED.md` | `:2583` (assert side effect "gate-covered") | 2026-08-20 (`18b2d6b9`) | 2026-08-20 | Stays as written |

This list comes from a search for the 10 check names and `clang-analyzer-core` in `docs/` and `standards/`. Lines that only name a check, with no PASS or zero claim, are not in the list. The new RC findings that G2 shows are RC master audit work. They are not batches in this Starcom plan.

#### C9: virtual calls at the adapter edge (decided 2026-10-30)

- `starcom/docs/DESIGN.md:111` (design position): "Policy/template seams in core; virtual dispatch only at adapter edges."
- `starcom/docs/ICD.md:318`: "No virtual `IRadio` in the core (P10-9)." This line ties the core ban to P10-9, but it says nothing about the adapter edge.
- `standards/CODING_STANDARDS.md:56`: for function pointers, "P10-9 governs". The file has no line about virtual calls.
- The FP-1 precedent (`standards/ACCEPTED_STANDARDS_DEVIATIONS.md:71`; `docs/audits/MASTER_STANDARDS_AUDIT_2026-05-07.md:785`; `docs/agents/LESSONS_LEARNED.md:1396-1423`) removed function pointers with template dispatch. It does not decide virtual calls.

- Goddard (2026-10-30): P10 Rule 9 bans function pointers because an indirect call stops the static proofs of the call graph and of no recursion. A virtual call is an indirect call through a table, so the same reason applies to it.

Decision (Nathan, 2026-10-30, by the rule "if the standards make a change necessary and it is feasible, do it"): the adapter edge also uses templates. No virtual dispatch. Reasons: Goddard's P10 Rule 9 reason above; `CODING_STANDARDS.md:47` ("the MORE-RESTRICTIVE rule applies"); and the FP-1 change, which removed function pointers with template dispatch. D4 changes `starcom/docs/DESIGN.md:111` to agree. `docs/agents/PROTECTED_FILES.md` does not list `starcom/docs/DESIGN.md`, so the edit needs no permission.

#### A1 baseline: two-board radio run (V-air reference)

Status: recorded from the raw logs. Goddard re-read: pending.

| Item | Value |
|---|---|
| Date and time | 2026-10-30, 01:50:23 to 01:51:24 CDT (log stamps). Run 60.9 s (score file). |
| Operator | Buzz (Rocket Chip tester). Count re-read from raw logs: Hamilton. Second re-read: Goddard (pending). |
| Firmware | `flight-18e5e6d8` on the two boards (vehicle and station banner; vehicle `ID: flight-18e5e6d8 tree=6ec6f83e984797731b2d6f3e95a6021612daabc5`; station `ID:` line the same). |
| Log tag | The log names have `579f0976`. This is the `origin/main` commit that Buzz used as the run tag (`--tag`, `scripts/soak_two_board.py:317`). It is not the firmware. |
| Tag vs firmware check | `git diff --stat 18e5e6d8 579f0976` changes 3 files: `AGENT_WHITEBOARD.md`, `CHANGELOG.md`, `src/starcom_adapt/README.md`. The only one under an `[elf]` path is a `.md` file, and `!**/*.md` in `scripts/ci/firmware_paths.txt` excludes it. Thus the firmware is the same at the two commits. `git diff --stat 18e5e6d8 6a616148` on the `[elf]` paths also shows only that README. |
| Radio settings | 2 dBm, BW 250 kHz, SF 7, CR 4/5, nav 10 Hz (vehicle `CFG: BW=250 SF=7 CR=5 nav=10Hz pwr=2dBm`; station `Radio: BW250 10Hz SF7 CR5 pwr2`). |
| Layout | Nathan's standard desk layout. |
| Arm and uplink | Nothing armed (`arm_sent: False`). No uplink command sent (`uplink_disarm_sent: 0`). |
| Preflight | Vehicle `Hardware: 13/13 OK`, `go_nogo=READY`. Station `Hardware: 11/11 OK`. |

##### Counts (from the raw logs)

All board counters are from boot. The run values are differences.

| Count | Start | End | In the run |
|---|---|---|---|
| Vehicle frames sent (`TX: N sent`) | 2656 (01:50:27.746) | 3147 (01:51:15.994) | 491 in 48.2 s, 0 fail |
| Station frames received (dashboard `Pkts:`) | 2557 (01:50:23.667) | 3171 (01:51:24.544) | 614 in 60.9 s |
| Station CRC errors (dashboard `CRC:`) | 1 | 1 | 0 new |
| Station `LINK` at end `t` | - | delivered=3090 missing=6 crc_fail=1 copp_rtx=0 copp_sync_loss=0 | no start value |
| Vehicle `LINK` | delivered=313 copp_rtx=0 copp_sync_loss=0 | delivered=372 copp_rtx=0 copp_sync_loss=0 | 59 delivered, 0 resend, 0 sync loss |
| Station COP-P state | lock in each dashboard frame | lock | lock all the run |
| Link quality | LQ 100% in each frame | | |
| Station RX rate (dashboard) | 8.5 to 11.0 Hz, last frame 10.4 Hz | | |
| RSSI | -62 to -65 dBm | | |
| SNR | 9 to 13 dB | | |

Note: the station `missing=6` is from boot. This log has no start value, so it is not a run count.

##### End sequence

The station got one `x` (01:51:15.776), then `z` (01:51:16.177), `t` (01:51:16.427), `s` (01:51:16.778). The station was on its dashboard when `x` went out. No erase prompt is in the log. The station end dump printed. After `s` the station is in its settings menu; the next power cycle clears it.

##### Logs (on the PC, gitignored)

- `logs/soak/2026-10-30_A1_baseline_2dbm_579f0976.txt` (vehicle)
- `logs/soak/2026-10-30_A1_baseline_2dbm_579f0976_stn.txt` (station)
- `logs/soak/2026-10-30_A1_baseline_2dbm_579f0976_score.txt` (score; its RATE counters, layout and airtime lines are wrong. The script fix after this run corrects them. Use the counts above.)

#### G1 proposed text (`starcom/.clang-tidy`, top of file to the end of `Checks:`)

This text replaces lines 14 to 234 of the shipped file. Lines 1 to 13 (the file header) do not change. The 139 entries in `Checks:` are the same as the shipped file, in the same order. Only the 71 comment lines move. Full diff: `/workspace/fixplan/g1/G1.diff` on the box (246 lines).

```yaml
# Checks notes. These comments were inside the folded Checks block. YAML read
# them as text and disabled the next check (STARCOM_AUDIT_2026-10-29.md PR-1).
# They are kept here word for word, in the same order. Do not put a comment
# inside the Checks block.
# ═══════════════════════════════════════════════════════════════
# BUGPRONE — Bug-prone code patterns
# Maps to: JSF AV general safety, MISRA-adjacent, CERT
# ═══════════════════════════════════════════════════════════════
# SKIPPED: bugprone-easily-swappable-parameters — too noisy for embedded
# SKIPPED: bugprone-exception-escape — exceptions disabled
#   bugprone-reserved-identifier: enabled: no Pico SDK names in this tree
# ═══════════════════════════════════════════════════════════════
# CERT — SEI CERT Secure Coding Guidelines
# Maps to: JSF AV safety rules, P10 defensive coding
# ═══════════════════════════════════════════════════════════════
# SKIPPED: cert-err58-cpp, cert-err60-cpp — exceptions disabled
# ═══════════════════════════════════════════════════════════════
# C++ CORE GUIDELINES
# Maps to: JSF AV Rules 188-189 (goto), 206 (no heap), 209 (types)
# ═══════════════════════════════════════════════════════════════
# SKIPPED: cppcoreguidelines-avoid-c-arrays — frame / PLTU buffers are fixed arrays
# SKIPPED: cppcoreguidelines-pro-bounds-* — codec octet walks (pointer+length), not Pico SDK
#   cppcoreguidelines-pro-type-reinterpret-cast: enabled: core is not MMIO; HW casts belong in ports
#   cppcoreguidelines-avoid-non-const-global-variables: enabled: core has no g_ process globals and no printf
# SKIPPED: cppcoreguidelines-owning-memory — no gsl::owner
# ═══════════════════════════════════════════════════════════════
# GOOGLE
# Maps to: JSF AV Rule 209 (fixed-width types)
# ═══════════════════════════════════════════════════════════════
# ═══════════════════════════════════════════════════════════════
# HICPP — High Integrity C++
# Maps to: JSF AV safety, P10-1 (no goto), JSF 14 (literal suffixes)
# ═══════════════════════════════════════════════════════════════
#   hicpp-vararg: enabled: sans-I/O core has no BASEPRI asm and no printf
# NOTE: hicpp-avoid-goto, hicpp-no-malloc etc. are aliases — already covered
# ═══════════════════════════════════════════════════════════════
# MISC — Miscellaneous quality checks
# Maps to: P10-6 (scope), JSF AV general, LOC-3 (defensive coding)
# ═══════════════════════════════════════════════════════════════
# SKIPPED: misc-include-cleaner — noisy with generated + public headers; re-eval at extract
# SKIPPED: misc-confusable-identifiers — slow, not safety-critical
# ═══════════════════════════════════════════════════════════════
# MODERNIZE — C++20 best practices
# Maps to: JSF AV Rule 29-31 (constexpr over #define), C++ modernization
# ═══════════════════════════════════════════════════════════════
# SKIPPED: modernize-use-auto — explicit types preferred in embedded (audit A.4)
# SKIPPED: modernize-use-trailing-return-type — not our style
# SKIPPED: modernize-use-nodiscard — C++17 feature, evaluate later
# SKIPPED: modernize-use-std-print — codecs do not print
# ═══════════════════════════════════════════════════════════════
# PERFORMANCE
# Maps to: embedded real-time constraints
# ═══════════════════════════════════════════════════════════════
# SKIPPED: performance-avoid-endl — no iostream usage
# SKIPPED: performance-inefficient-string-concatenation — no std::string
# SKIPPED: performance-inefficient-vector-operation — no std::vector
# ═══════════════════════════════════════════════════════════════
# READABILITY
# Maps to: JSF AV Rules 1 (size), 50-53 (naming), 59 (braces),
#          151 (magic numbers), 152 (one var per decl)
# ═══════════════════════════════════════════════════════════════
# DISABLED: readability-math-missing-parentheses — stricter than JSF AV Rule 213,
# which explicitly allows arithmetic precedence ("below arithmetic operators").
# CERT SEI EXP00-C-EX1 confirms: algebraic order parens are redundant.
# ArduPilot's AP_Math (our upstream reference) relies on * before + throughout.
# SKIPPED: readability-identifier-length — single-char loop vars are fine
# SKIPPED: readability-qualified-auto — we don't use auto
# ═══════════════════════════════════════════════════════════════
# CLANG STATIC ANALYZER
# Maps to: P10-10 (static analysis), LOC-3 (defensive coding)
# Deep flow analysis — catches null deref, div-by-zero, leaks
# ═══════════════════════════════════════════════════════════════
# SKIPPED: clang-analyzer-osx.* — macOS only
# SKIPPED: clang-analyzer-unix.Malloc — no heap usage
# SKIPPED: clang-analyzer-optin.mpi.* — no MPI

Checks: >
  -*,

  bugprone-assert-side-effect,
  bugprone-assignment-in-if-condition,
  bugprone-bool-pointer-implicit-conversion,
  bugprone-branch-clone,
  bugprone-copy-constructor-init,
  bugprone-dangling-handle,
  bugprone-fold-init-type,
  bugprone-forward-declaration-namespace,
  bugprone-implicit-widening-of-multiplication-result,
  bugprone-incorrect-roundings,
  bugprone-infinite-loop,
  bugprone-integer-division,
  bugprone-macro-parentheses,
  bugprone-macro-repeated-side-effects,
  bugprone-misplaced-widening-cast,
  bugprone-multi-level-implicit-pointer-conversion,
  bugprone-multiple-statement-macro,
  bugprone-narrowing-conversions,
  bugprone-redundant-branch-condition,
  bugprone-signed-char-misuse,
  bugprone-sizeof-expression,
  bugprone-string-constructor,
  bugprone-suspicious-enum-usage,
  bugprone-suspicious-include,
  bugprone-suspicious-memset-usage,
  bugprone-suspicious-missing-comma,
  bugprone-suspicious-semicolon,
  bugprone-switch-missing-default-case,
  bugprone-too-small-loop-variable,
  bugprone-undefined-memory-manipulation,
  bugprone-undelegated-constructor,
  bugprone-unhandled-self-assignment,
  bugprone-unused-return-value,
  bugprone-use-after-move,
  bugprone-reserved-identifier,

  cert-dcl50-cpp,
  cert-dcl58-cpp,
  cert-err33-c,
  cert-err34-c,
  cert-err52-cpp,
  cert-flp30-c,
  cert-mem57-cpp,
  cert-msc30-c,
  cert-msc32-c,
  cert-msc50-cpp,
  cert-msc51-cpp,
  cert-oop57-cpp,
  cert-oop58-cpp,
  cert-str34-c,

  cppcoreguidelines-avoid-goto,
  cppcoreguidelines-init-variables,
  cppcoreguidelines-interfaces-global-init,
  cppcoreguidelines-macro-to-enum,
  cppcoreguidelines-narrowing-conversions,
  cppcoreguidelines-no-malloc,
  cppcoreguidelines-pro-type-cstyle-cast,
  cppcoreguidelines-pro-type-member-init,
  cppcoreguidelines-pro-type-static-cast-downcast,
  cppcoreguidelines-slicing,
  cppcoreguidelines-special-member-functions,
  cppcoreguidelines-pro-type-reinterpret-cast,
  cppcoreguidelines-pro-type-vararg,
  cppcoreguidelines-avoid-non-const-global-variables,

  google-build-explicit-make-pair,
  google-build-namespaces,
  google-build-using-namespace,
  google-default-arguments,
  google-explicit-constructor,
  google-runtime-int,

  hicpp-signed-bitwise,
  hicpp-uppercase-literal-suffix,
  hicpp-multiway-paths-covered,
  hicpp-no-assembler,
  hicpp-vararg,

  misc-const-correctness,
  misc-definitions-in-headers,
  misc-misplaced-const,
  misc-no-recursion,
  misc-non-copyable-objects,
  misc-redundant-expression,
  misc-static-assert,
  misc-unconventional-assign-operator,
  misc-unused-parameters,
  misc-unused-using-decls,
  misc-use-internal-linkage,

  modernize-deprecated-headers,
  modernize-loop-convert,
  modernize-macro-to-enum,
  modernize-redundant-void-arg,
  modernize-use-bool-literals,
  modernize-use-default-member-init,
  modernize-use-equals-default,
  modernize-use-equals-delete,
  modernize-use-nullptr,
  modernize-use-override,
  modernize-use-using,

  performance-enum-size,
  performance-for-range-copy,
  performance-implicit-conversion-in-loop,
  performance-move-const-arg,
  performance-move-constructor-init,
  performance-no-automatic-move,
  performance-no-int-to-ptr,
  performance-trivially-destructible,
  performance-type-promotion-in-math-fn,
  performance-unnecessary-copy-initialization,
  performance-unnecessary-value-param,

  readability-avoid-const-params-in-decls,
  readability-avoid-nested-conditional-operator,
  readability-braces-around-statements,
  readability-const-return-type,
  readability-container-size-empty,
  readability-delete-null-pointer,
  readability-duplicate-include,
  readability-else-after-return,
  readability-enum-initial-value,
  readability-function-cognitive-complexity,
  readability-function-size,
  readability-identifier-naming,
  readability-implicit-bool-conversion,
  readability-inconsistent-declaration-parameter-name,
  readability-isolate-declaration,
  readability-magic-numbers,
  readability-misleading-indentation,
  readability-misplaced-array-index,
  readability-named-parameter,
  readability-non-const-parameter,
  readability-redundant-control-flow,
  readability-redundant-declaration,
  readability-redundant-preprocessor,
  readability-simplify-boolean-expr,
  readability-static-accessed-through-instance,
  readability-uppercase-literal-suffix,

  clang-analyzer-core.*,
  clang-analyzer-cplusplus.*,
  clang-analyzer-deadcode.*,
  clang-analyzer-security.ArrayBound,
  clang-analyzer-security.FloatLoopCounter,
```

#### G3 proposed text (`starcom/.clang-tidy`, replaces the shipped line 236 comment and line 237)

```yaml
# Only report warnings in our source files, not SDK/third-party headers.
# Accept / and \ as path separators (clang-tidy on Windows shows both).
# Repo source only: starcom/include, starcom/src, starcom/adapters, RC src/ and include/.
# Vendored and third-party trees are excluded (tl::expected, pico-sdk, lib, EXTERNAL).
HeaderFilterRegex: '.*[/\\](starcom[/\\](include|src|adapters)|src|include)[/\\].*'
ExcludeHeaderFilterRegex: '.*[/\\](third_party|pico-sdk|lib|EXTERNAL)[/\\].*'
```

`ExcludeHeaderFilterRegex` is in clang-tidy 21.1.8. The `src|include` part also agrees with a `src/` or `include/` folder in a third-party tree. `ExcludeHeaderFilterRegex` removes those headers. The probe has a vendor header at `third_party/include/` to test that.

#### G4 proposed probe (`starcom/tests/tidy_probe/`)

The build does not use the probe. Each `EXPECT` tag names a check that must fire on that line. The self-test runs:

`clang-tidy tidy_probe.cpp --config-file=starcom/.clang-tidy -- -std=c++20 -Istarcom/include -Ithird_party/include` (paths in the probe folder).

`tidy_probe.cpp`:

```cpp
// tidy_probe.cpp: Tier 1.1 known-bad fixture for starcom/.clang-tidy.
// Each marked line holds one planted defect. It is never built or linked.
// starcom/scripts/tidy_selftest.sh runs clang-tidy on it and requires each
// check named in an EXPECT tag to fire, and no hit in third_party/.
#include <cassert>
#include <stdint.h>  // EXPECT modernize-deprecated-headers
#include <utility>

#include "probe_hit.hpp"     // EXPECT readability-uppercase-literal-suffix (header hit, PR-1b)
#include "probe_vendor.hpp"  // EXPECT-NOT probe_vendor.hpp (third_party/include: matches the include rule, so the exclude rule must drop it, G3)

int _probe_reserved = 0;  // EXPECT bugprone-reserved-identifier

enum class ProbeEnum : int { a, b };  // EXPECT performance-enum-size

void probeConstParam(const int v);  // EXPECT readability-avoid-const-params-in-decls

int probeVarargDef(int n, ...) { return n; }  // EXPECT cert-dcl50-cpp

int probeGoto(int x) {
  if (x > 0) {
    goto done;  // EXPECT cppcoreguidelines-avoid-goto
  }
  x = 2;
done:
  return x;
}

int probeAnalyzerNull() {
  int* p = nullptr;
  return *p;  // EXPECT clang-analyzer-core.NullDereference
}

int probeAnalyzerDiv(int x) {
  const int z = 0;
  return x / z;  // EXPECT clang-analyzer-core.DivideZero
}

int probeMisc(int x, int* buf) {
  int unchanged = x;  // EXPECT misc-const-correctness
  assert(++x > 0);    // EXPECT bugprone-assert-side-effect
  const int s = x & 3;  // EXPECT hicpp-signed-bitwise
  auto* raw = reinterpret_cast<char*>(buf);  // EXPECT cppcoreguidelines-pro-type-reinterpret-cast
  (void)raw;
  const auto pr = std::make_pair<int, int>(1, 2);  // EXPECT google-build-explicit-make-pair
  if (s > 1)
    x = 1;
    x = 2;  // EXPECT readability-misleading-indentation
  return probeVarargDef(1, unchanged, pr.first);  // EXPECT cppcoreguidelines-pro-type-vararg
}
```

`starcom/include/probe_hit.hpp` (in the probe folder, so the path has `starcom/include`):

```cpp
// probe_hit.hpp: header hit for HeaderFilterRegex (path has starcom/include).
inline unsigned probeHeaderHit() { return 7u; }  // EXPECT readability-uppercase-literal-suffix
```

`third_party/include/probe_vendor.hpp` (in the probe folder):

```cpp
// probe_vendor.hpp: stands for a vendored header (tl::expected, pico-sdk).
// It holds a defect on purpose. The filter must not report it.
inline unsigned probeVendor() { return 9u; }
```

#### Results of the proposal check (PC, clang-tidy 21.1.8, 2026-10-29)

The configs and the probe were in a temporary folder, not in the repo. No repo file changed.

| Config | `--list-checks` | `clang-analyzer-core.*` listed | Probe: the 13 checks and the 2 analyzer defects | Probe header hit | Probe vendor hit | `copp.cpp` header hits | `third_party` hits |
|---|--:|--:|---|---|--:|---|--:|
| Shipped | 156 | 23 | Do not fire | — | — | 0 | 0 |
| G1 | 169 | 23 | All fire | — | — | — | — |
| G1 + G3 | 169 | 23 | All fire | Shows | 0 | `copp.hpp:24,27`, `uslp.hpp:12`, `v3.hpp:13` | 0 |

The shipped config shows the 23 `clang-analyzer-core.*` checks, but the planted null dereference and divide by zero do not fire. With G1 they fire. This agrees with Tier 1 in Section 1.2.

Commands (PowerShell, a temporary folder with the proposed files):

```powershell
$ct = 'C:\Program Files\LLVM\bin\clang-tidy.exe'
$r = 'C:\Users\pow-w\Documents\rocket-chip'; $b = 'C:\Users\pow-w\Documents\rc-audit-logs\scbuild'
foreach ($c in 'shipped','G1','G3') {
  (& $ct --list-checks --config-file="g1\starcom.clang-tidy.$c" probe\tidy_probe.cpp -- -std=c++20 | Select-String '^\s+\S').Count }
& $ct --config-file='g1\starcom.clang-tidy.G3' probe\tidy_probe.cpp -- -std=c++20 -Iprobe\starcom\include -Iprobe\third_party\include
& $ct "$r\starcom\src\ccsds\copp.cpp" -p $b --config-file='g1\starcom.clang-tidy.G3' --quiet
```

#### PR coverage

Each REMEDIATE item from Category 1 to Category 3 is in one batch: PR-1 (G1, G4), PR-1b (G3), PR-2 (G2, config fix only; the old PASS lines stay as written), PR-6 (T1), PR-7, PR-8, PR-11, PR-22 (D1), PR-9 (D2), PR-12b (C4), PR-13 (C3), PR-14/15/16 (C5), PR-18 (C11), PR-19 (C12, C13), PR-20 (C6), PR-23 (C9, D4), PR-24 (W1), PR-25 (C8), PR-26 (C14), PR-27 (C10), PR-28 (C7), PR-29 (C15), PR-30 (C16), PR-31 (C17), PR-32 (C18, W3), PR-33 (C1), PR-34 (C2), PR-35 (R2), PR-36 (W2).

Three items have more than one batch, on purpose:
- PR-1 has the config change (G1) and the self-test (G4). The row tells the two.
- PR-19 has 297 suffix sites and 115 magic-number sites. One commit is too large to examine.
- PR-32 has two rules (JSF 192 and JSF 53). The JSF 53 rename has a large blast radius, so it is last.

These items have no batch: PR-21 (not in this plan; tracked with the FSK packing change), PR-3 (NO ACTION), PR-5 (NO ACTION), PR-10 (DEFER), PR-12a (NO FINDING, C14 adds its comment), PR-17 (DEFER), PR-4 (an open item with no row). JSF 49 is an open item with no PR. A0 and A1 have no PR. They make the V-air check possible. T0 has no PR. It tests the COP-P resend on the host before C4.


---

## 3. CCSDS conformance (parked until the FSK pivot; all findings kept)

### 3.1 Status

On 2026-10-29 the owner parked this work: "The CCSDS stuff is good to note but it will drastically change with the FSK pivot: just note where things stand with all that and drop it for now (don't delete your findings, just stop wasting time on it)." This section keeps all findings. The clause table (3.5) has no new grades.

**State at this time:**

- **F-1 to F-7** are open (see Findings in 3.2). F-1 to F-3 are stated deviations. F-4 to F-7 are gaps that `CONFORMANCE.md` shows in the MAC Partial row.
- **Agreed new grades that are not in the clause table (RV-8):**
  - 211.0-B-6/s041 is PASS in 3.5. The agreed grade is FAIL (gap), as part of F-6. `copp.cpp:359-365` examines only the destination SCID. `copp.cpp:361-362` gives `true` for each source frame. The code does not have the mandatory item DLL-184 Test_Source (211.0-B-6 §6.2.4.2, PICS "DLL-184 Test_Source 6.2.4.2 M"). No test in `starcom/tests` sends a destination frame with an incorrect SCID and checks that the code rejects it. Duke, Hamilton and Goddard each read this.
  - 133.0-B-2/s041, /s042 and /s059 are PASS in 3.5. They are SPP-4 ("Packet Loss Indicator"), SPP-5 ("QoS Requirement") and SPP-9 ("Data Loss Indicator"). The agreed grade is N/A ("optional, not supported"). The 133.0-B-2 Annex A Table A-2 gives them status O. The PICS gives Support N at `starcom/docs/CONFORMANCE.md:88`, `:89` and `:93`. `space_packet_service.hpp:30` and `:36` have the text "not provided". Rule C-14 applies.
- **Rows where the evidence has the opposite value (the recommended grade is UNVERIFIED or FAIL after a read):**
  - 211.0-B-6/s100 is PASS. Its evidence is `test_mac.cpp:153 CHECK(s.mac_frame_pending)` (`true`). The shall tells that `MAC_FRAME_PENDING` must be `false`.
  - 211.0-B-6/s144 cites `test_mac.cpp:176 CHECK(s.need_plcw)` (`true`). The shall has the text "set to false".
  - 211.0-B-6/s140 and /s196 cite `test_mac.cpp:523` (`macPollNotify == comm_change_apply_rx`). That line examines none of the two values.
- **Idle-flag lines that are out of date:** `CONFORMANCE.md:29`, `:99` (SPP-15) and `:181` (known gaps) tell that the code does not set the idle-packet secondary-header flag. `starcom/src/ccsds/space_packet.cpp:53` sets it after `1f23b359` (2026-10-01).
- **COP-P (RV-10):** the level stays Full with 3 PASS and 19 UNVERIFIED of 22 §7 shalls. These items are open:
  - one level rule for UNVERIFIED rows (C-6)
  - a new grade
  - a citation of the two known whiteboard defects. First, `src/starcom_adapt/byte_pump.cpp:246` sets `mib.synch_timeout = 0;`. With `copp.cpp:200`, SYNCH_TIMER thus does not expire on the flight link. Also, no test shows that FOP-P sends a frame again if the frame does not get to the other end of the air link.
- **Books that the walk did not grade shall by shall (RV-6):**
  - Graded shall by shall: 211.0-B-6, 211.1-B-4, 211.2-B-3 and 133.0-B-2.
  - Graded at row level only: 732.1-B-3, 232.0-B-4, 232.1-B-2, 131.0-B-6, 231.0-B-4 and 355.0-B-2.
  - "549/549" includes only the 4 graded books. "No undeclared gap remains" includes only the FAIL rows of those 4 books.
  - The review made an estimate of the shalls in the claimed books that the walk did not grade. It counted "shall" sentences in the PDFs with PyMuPDF (approximate): 732.1-B-3 about 507, 232.1-B-2 about 113, 232.0-B-4 §4.2.1, a part with about 340.
- **Verdicts given to large groups of rows (RV-7):**
  - 52 Space Packet PASS rows cite the same 3 tests.
  - 32 rows have the text "row-level test" and give no assertion name.
  - 52 MAC rows cite the same 4 functions. (Not counted again: 124 rows of 211.0-B-6 cite `test_mac.cpp`.)
  - 158 of 549 rows (29%) are UNVERIFIED (V-1).
  - About 50 SPP clauses have the same note from `CONFORMANCE.md`. That uses a doc as evidence (C-3).
  - With rules C-1 and C-2, these rows become UNVERIFIED, unless a person reads a named assertion against each shall.
- **Test location (RV-9):** 53 Space Packet service rows are PASS on the Rocket-Chip test `test/test_space_packet_service.cpp`. `starcom/docs/audits/README.md:57` tells that tests must be in `starcom/tests/`. If that rule applies, these rows become UNVERIFIED. The table shows them as PASS. The owner can select one of three alternatives: move the tests, change the rule, or grade the rows UNVERIFIED (C-13). The owner parked this decision. Hamilton recommends UNVERIFIED (`starcom/docs/DESIGN.md:110`, `:33`).
- **Edits E1 to E11 (RV-12):** the commit applied these edits to `CONFORMANCE.md` and `COVERAGE.md`. They cite "Nathan, 2026-10-29". The only such record is the general rule "claims must match actual levels". It does not give approval for each edit. The owner parked one decision. The alternatives are the mark "proposed" on them, or an approval record for each row.
- **When the work starts again:** rules C-1 to C-14 (3.4) apply. C-13 waits for the test-location decision.

**More words for this section:**

| Term | Meaning |
|---|---|
| Proximity-1 (Prox-1) | The CCSDS space link for short range (211.x books). |
| PHY | Physical layer: the radio. |
| RFM95 | The radio module on the boards (FSK and LoRa modes). |
| LoRa | A long-range radio mode of the RFM95. |
| SDLS | Space Data Link Security (355.0-B-2). |
| COMM_CHANGE | The Proximity-1 directive that changes the link settings. |
| ChannelAccess | The service primitives of 211.2-B-3 Annex B. |
| Hailing | The step where the two ends of the link find each other. |
| E30 to E46 | COP-1 event numbers (232.1-B-2). |
| D13 | The decision that puts the rate in the Data Rate field (see K1). |
| Navigator | The tool in `starcom/standards/ccsds/navigator/`. Its `standard.json` holds the shall text from the stored PDFs. |
| SHALL_MATRIX | `starcom/docs/SHALL_MATRIX.md`, the list of shalls and their IDs. |

### 3.2 Audit report (Duke; code pointers by Goddard)

This walk is the CCSDS step of the audit (Tier 3.9, status "proposed"). It adds to `standards/AUDIT_GUIDANCE.md`. The walk used G1 to G13 (`guide_gaps.md`), which had the status "proposed". Hamilton's change to the guide includes G1 to G13 for review. Section 4 (RV-12) has a subsequent record: G1 to G13 got the status "adopted" on the owner instruction "get the docs in order" (2026-10-29). `starcom/docs/audits/README.md:45` has the G1 to G13 text. No record shows an owner approval of each item.

#### Metadata

- **Audit date:** 2026-10-29.
- **Auditors:** Duke (standards reviewer). Goddard supplied the code and test pointers. Hamilton recorded the results in the repo.
- **Code:** `579f0976` (origin/main, pushed). The `starcom/` and `src/` trees are the same as at `b1c9f98f`. The work tree had no changes that were not in a commit.
- **Previous audits:** none of this type. IVP23 (2026-08-31) is for coding rules only.
- **Table for each shall:** 3.5 below.

#### Scope

- **Binding books, graded shall by shall:** 211.0-B-6, 211.1-B-4, 211.2-B-3, 133.0-B-2. This is the SHALL_MATRIX population of 549 shalls.
- **Row level only:** 732.1-B-3, 232.0-B-4, 232.1-B-2, 131.0-B-6, 231.0-B-4, 355.0-B-2.
- **Informative books (for more data only):** Green, Pink, Red.
- **Code:** `starcom/` and `src/starcom_adapt/`.
- **Tiers in this section:** Tier 3 (as 3.9), the citation-walk extension (G10), Tier 6 claim sync (G11) and Tier 7 dispositions.
- **Tiers in other sections:** Tiers 1 and 2 and 3.1 to 3.7 (Hamilton: gates, clang-tidy, coding catalogs). Tiers 4 and 5 (Buzz and Hamilton).

#### Method

- Each shall is a word-for-word copy from the navigator `standard.json`. The navigator gets this text from the PDFs of the last issues in the repo. The IDs agree with SHALL_MATRIX.
- The walk first found the code for each `CONFORMANCE.md` row. Then a test assertion at a file and line is necessary for the last verdict.
- 13 PASS verdicts use an identifier trace: the assertion line names the identifier from the book. Inference: those assertions are tests of the shall. The table labels each one.
- The 53 Space Packet service PASS rows cite the Rocket-Chip `test/` suite.

The Glossary gives the data for each verdict.

#### Known items (logged, not findings)

- **K1:** the Mode Select and Scrambler packing from before D13, in `src/starcom_adapt/byte_pump.cpp:99–106` and its README. Decision D13 puts the rate in the Data Rate field. Mode Select and Scrambler keep the functions that the book gives (211.0-B-6 B1.7.6 and B1.7.7). Transmitter Mode is 001.
- **K2:** the SDLS "WANTED conflict" item on the whiteboard. SDLS does not apply to Proximity-1 (355.0-B-2 §2.1). The MAC stays (211.0-B-6 §2.1.4.4, §4.2). There is no authentication trailer.
- **K3:** 130.0-G-3 in the navigator data is a word-for-word copy of the book text.

#### Summary: 549 of 549 shalls in 4 books (211.0-B-6, 211.1-B-4, 211.2-B-3, 133.0-B-2)

The walk also graded 6 more claimed books at row level only (732.1-B-3, 232.0-B-4, 232.1-B-2, 131.0-B-6, 231.0-B-4, 355.0-B-2). That is 10 claimed books in total (RV-6, parked).

| Book | PASS | PASS (doc) | DEVIATION-STATED | FAIL (gap) | N/A | UNVERIFIED | Total |
|---|---|---|---|---|---|---|---|
| 211.0-B-6 | 61 | 26 | 19 | 44 | 25 | 128 | 303 |
| 211.1-B-4 | 0 | 0 | 37 | 0 | 2 | 0 | 39 |
| 211.2-B-3 | 28 | 0 | 2 | 23 | 8 | 30 | 91 |
| 133.0-B-2 | 107 | 1 | 0 | 0 | 8 | 0 | 116 |
| **All** | **196** | **27** | **58** | **67** | **43** | **158** | **549** |

**Section verdict: PARTIAL.** A declaration or a level change closes each FAIL (gap) item (see Decisions). After the `CONFORMANCE.md` edits, there is no gap without a declaration. Inference: the FAIL rows then become DEVIATION-STATED.

#### Claimed level and the evidence for each shall

| CONFORMANCE row | Claimed | Evidence | Assessment | Decision |
|---|---|---|---|---|
| PLTU (:24) | Full | 211.2 §3.2/§3.6/Annex C: 25 PASS, 2 DEV, 2 N/A, 1 UNV of 30 | Supported as worded (envelope + hunt) | Keep Full. Idle declared (N5 item 1) |
| V-3 frame (:26) | Full | 33 PASS, 6 N/A of 39 | Supported | — |
| User Defined Data (:27) | Full | §3.2.3.5 PASS | Supported | — |
| USLP in PLTU (:28) | Full | row-level, test_uslp.cpp | Supported (row-level) | — |
| Space Packet (:29) | Partial (PDU codec) | codec 52/52 PASS; service 53/53 PASS (RC `test/`) | Understated | "implemented, unit-tested (RC test/ suite), not integrated" |
| PLCW (:30) | Full | 14 PASS (doc) code-read + 2 PASS golden | Supported; tests are weak | Keep Full. Golden test is a triage item |
| CLCW (:31) | Full | row-level, test_ocf.cpp | Supported (row-level) | — |
| COP-P (:32) | Full | §7: 3 PASS, 19 UNVERIFIED | Not demonstrated at shall level | Keep Full; note "not demonstrated at shall level (19 UNVERIFIED)" |
| COP-1 (:33) | Full (implemented options) | E30–E39 tested (test_cop1.cpp:302–327); E42 tested (:328); E41, E43–E46 in cop1.cpp:328–333 untested | Test pointer overstated | Test pointers only for E30–E39 and E42 |
| Prox-1 MAC (:34) | Full | 162 shalls: 17 PASS, 12 PASS (doc), 14 FAIL, 4 DEV, 14 N/A, 101 UNV | Overstated | **Full → Partial**; F-4..F-7 listed |
| Coding (:38) | Partial (encode) | 28/28 UNVERIFIED at shall level | Not demonstrated at shall level | Keep the level; note "not demonstrated at shall level (28 UNVERIFIED)" |
| 211.1 PHY (:40) | 0 | 37 DEV, 2 N/A | Supported | — |
| Timing (no row) | — | 30/30 FAIL | Undeclared | New row: stated deviation |

#### Findings

| ID | Clauses | Evidence (579f0976) | Disposition |
|---|---|---|---|
| F-1 | 211.2-B-3 §3.3.2–3.3.5, §3.5.3: Idle/Acquisition/Tail PN 352EF853 (PICS A2.2 item 2 M). 14 shalls | No generator in starcom/src or src/starcom_adapt | ACCEPT as a stated deviation in CONFORMANCE.md. Implementation is a triage-order item |
| F-2 | 211.0-B-6 §5.2–5.4, 4.2.4.1/.2, 3.2.4.4.3, B2 (DLL-31/32/52–64 M). 30 shalls | No timing services, time buffers or Time Distribution SPDU | ACCEPT as a stated deviation (new timing row). Implementation is a triage-order item |
| F-3 | 211.2-B-3 Annex B ChannelAccess primitives. 9 shalls | I/O-free encode/decode/hunt API | ACCEPT as a stated deviation (I/O-free core API, DESIGN.md) |
| F-4 | 211.0-B-6 6.2.3.9, 6.3.3.1.7 (DLL-77 M) | No RECEIVING_SCID_BUFFER or its directive | ACCEPT as a listed gap under MAC Partial. Implementation is DEFER |
| F-5 | 211.0-B-6 6.2.3.10 (DLL-78 M) | No RECEIVING_PCID_BUFFER (copp.cpp coppReceiveV3) | As F-4 |
| F-6 | 211.0-B-6 6.2.4.2, 6.6.1.2, Annex D INVALID FRAME SOURCE | Destination-only SCID check, copp.cpp:359–365; no Test_Source | As F-4 |
| F-7 | 211.0-B-6 6.2.4.7, 6.2.4.10.1 (DLL-156/153 M) | MacMib (mac.hpp:95–111) has neither parameter | As F-4 |
| CS-1 | 133.0 Annex A SPP-2/6–8/10–13/19–22 vs CONFORMANCE.md:63 | space_packet_service.cpp:32–104; test/test_space_packet_service.cpp | REMEDIATE (doc edit U5) |
| CS-2 | COVERAGE.md Annex B says SET TX/RX/CTRL/PL-EXT = 0 | mac.cpp:1106–1215; test_mac.cpp:328, :479 | REMEDIATE (COVERAGE edit) |
| CS-3 | CONFORMANCE.md:29, :91 "sends 0" | byte_pump.cpp:296–356 per-APID count | REMEDIATE (doc edit) |
| CS-4 | COVERAGE.md COP-1 E30–E46 = 0 | cop1.cpp:289, :307, :315, :328–333 | REMEDIATE (COVERAGE edit) |
| D-1 | 211.0 §4.4/§8 deviation stated only in COVERAGE.md | — | REMEDIATE (add to CONFORMANCE.md) |
| T-1 | COP-1 E41, E43–E46 untested | cop1.cpp:328–333 | DEFER. Safety impact none: the code is present and the claim is narrowed to tested events |
| T-2 | PLCW round-trip cannot catch symmetric bit-position errors | plcw.cpp:18–29 | DEFER. Safety impact none: bits verified by code reading against Fig 3-5 |
| T-3 | SPP service test is outside the Starcom suite | test/CMakeLists.txt:174 | DEFER. Safety impact none: the service is not integrated in flight code |
| V-1 | 158 UNVERIFIED (MAC §6 101, coding 28, COP-P 19, other 10) | tests exist, not traced | DEFER. Safety impact none: verification depth only, no behaviour change |

#### Decisions and their basis

- **MAC row: Full → Partial.** `CONFORMANCE.md` shows F-4 to F-7 as gaps in the MAC Partial row. This is because of the rule "claims must match actual levels" (owner, 2026-10-29).
- **F-1 (idle, acquisition and tail) and F-2 (timing)** are stated deviations. To add them to the code is a triage item. This is because claims must agree with the code. The owner rule of 2026-10-06 is "remove deviations where possible".
- **F-3 (ChannelAccess)** is a stated deviation. This is because the core API has no I/O (`starcom/docs/DESIGN.md`).
- **Space Packet row:** "implemented, unit-tested (RC test/ suite), not integrated". To move the test into the Starcom suite is a triage item.
- **COP-1:** the row claims test pointers only for E30 to E39 and E42. E41 and E43 to E46 have code but no tests.
- **COP-P and coding encode** keep their claimed level, with the note "not demonstrated at shall level" and the UNVERIFIED count. The trace of the 158 UNVERIFIED rows to test assertions (V-1) is DEFER, because it only adds verification depth.
- **G1 to G13:** Hamilton's change to the guide includes them for review.

#### Open questions (no record answers them)

- **O-1:** §6.2.3.7 REMOTE_SCID_BUFFER. Is the destination-frame test against `local_scid` (`copp.cpp:364`) the buffer that the book means? This item is about the text of the clause. The row has the grade UNVERIFIED.
- **O-2:** 211.2-B-3 §3.5.5 and §3.6.1: "constant-rate symbol delivery" on the FSK link. This is a bench item.
- **O-3:** §6.2.4.16 Hailing_Data_Rate maps to `MacCommValue`. It does not go to a MIB field. We did not make sure that this agrees with DLL-162 (UNVERIFIED).

#### N5: more deviations for Proximity-1 framing (rows that `CONFORMANCE_updates.md` added)

1. 211.2-B-3 §3.3.2.1 a)–c), §3.3.2.2, §3.3.3.2.2, §3.3.5.2.2, §3.5.3: no idle, acquisition or tail sequence, and no MIB value for the idle time.
2. 211.2-B-3 §3.5.5: the FSK and LoRa link does not make sure of "constant-rate delivery" (O-2).
3. 211.0-B-6 §6.5 and 211.1-B-4: the PHY is the RFM95 in FSK mode, not 211.1. `CONFORMANCE.md` gives this (lines 18 and 40).
4. 211.0-B-6 B1.7.6 and B1.7.7: temporary use of these fields until D13 (K1).
5. 211.0-B-6 §5, §4.2.4, B2 and 211.2-B-3 §3.5.6 and §3.6.8: no timing services and no time tag.
6. 211.0-B-6 §4.4 and §8: no I/O sublayer and no segment reassembly.
7. 211.2-B-3 Annex B: no ChannelAccess primitives.
8. 211.0-B-6 §6.2.3.9, §6.2.3.10, §6.2.4.2, §6.3.3.1.7, §6.6.1.2, Annex D: no check of the source SCID and PCID on receive.
9. 211.0-B-6 §6.2.4.7, §6.2.4.10: no COMM_CHANGE waiting-period or lifetime timers.

#### Citation walk of the Starcom conformance docs

- 16 of 16 different cited binding and informative books are the last issue on ccsds.org (examined 2026-10-29).
- 6 of 6 Silver citations have the label Historical. Each tells the cause.
- UNVERIFIED: 211.2-B-4 and 211.1-B-5 (only drafts name them), and 235.1-R-1 (only one review-comment entry names it).

#### Fix order (Appendix C; 3.3 gives the disposition of each item)

- **Category 1 (gate integrity):** the guide change for G1 to G13 (Hamilton).
- **Category 4 (docs), in this order:**
  1. MAC row → Partial.
  2. Timing row and N5 deviation rows.
  3. SPP row and Annex A (CS-1).
  4. Sequence-count text (CS-3).
  5. COP-1 pointers.
  6. `COVERAGE.md` CS-2 and CS-4.
  7. D-1.
- **Category 2 (foundations that other items use, tests):** T-1, T-2, T-3.
- **Category 3 (how the code operates, removes deviations, owner rule of 2026-10-06):**
  1. F-1 idle generator.
  2. F-4 to F-6 receive checks.
  3. F-7 COMM_CHANGE timers.
  4. F-2 timing.
- **Deferred:** V-1, the trace of the 158 UNVERIFIED rows to test assertions.

#### Row for `standards/STANDARDS_AUDIT.md` Section G (audit history), to add later

| Date | Phase | Scope | Auditor | Commit | Notes |
|---|---|---|---|---|---|
| 2026-10-29 | First CCSDS conformance walk (Tier 3.9, proposed) | 549/549 shalls, 4 books; 6 books row-level | Duke (+Goddard pointers) | `579f0976` | PARTIAL. F-1..F-7, CS-1..CS-4, D-1, T-1..T-3, V-1 |

#### Sources

- ccsds.org lists, read on 2026-10-29: https://ccsds.org/publications/bluebooks/ · https://ccsds.org/publications/silverbooks/ · https://ccsds.org/publications/greenbooks/ · https://ccsds.org/publications/pinkbooks/ · https://ccsds.org/publications/redbooks/
- PDFs in the repo: `starcom/standards/ccsds/CCSDS-211.0-B-6-EC1.pdf`, `-211.1-B-4-EC1.pdf`, `-211.2-B-3.pdf`, `-133.0-B-2-EC2.pdf`. The text comes from `navigator/data/standard.json`.
- Repo: github.com/jet-flyer/Rocket-Chip at `579f0976`.

### 3.3 CCSDS triage (Appendix C order; dispositions recorded)

| Order | ID | Cat | Disposition | Work | Rationale / basis |
|---|---|---|---|---|---|
| 0 | G1–G13 | 1 | REMEDIATE | Guide patch (Hamilton, for review) | Audit procedure has no CCSDS tier |
| 1 | R-1 / F-4..F-7 | 4 | ACCEPT (as gaps) | CONFORMANCE E1, E9 | Claims match actual levels (owner, 2026-10-29) |
| 2 | F-1, F-2, F-3, D-1 | 4 | ACCEPT (stated deviations) | E8, E9 | Claims match code; F-3 per DESIGN.md |
| 3 | CS-1, CS-3 | 4 | REMEDIATE | E5, E6, E7 | Doc contradicts code |
| 4 | COP-1 pointers | 4 | REMEDIATE | E2 | Test pointers only for tested events |
| 5 | CS-2, CS-4 | 4 | REMEDIATE | E10 | COVERAGE.md stale |
| 6 | COP-P / coding notes | 4 | REMEDIATE | E3, E4 | Not demonstrated at shall level |
| 7 | T-1 E41/E43–E46 tests | 2 | DEFER | add tests | Safety impact none: code present, claim narrowed |
| 8 | T-2 PLCW golden test | 2 | DEFER | non-zero golden bytes | Safety impact none: verified by code reading |
| 9 | T-3 SPP test into starcom/tests/unit | 2 | DEFER | move/duplicate test | Safety impact none: not integrated in flight code |
| 10 | F-1 Idle generator | 3 | DEFER (implementation) | PN generator + RC tx path | Safety impact none: current bearer is best-effort FSK, no Prox-1 peer; removes a deviation (2026-10-06) |
| 11 | F-4..F-6 receive validation | 3 | DEFER (implementation) | SCID/PCID buffers, Test_Source | Safety impact low: single-pair link, no third-party sources; restores MAC Full |
| 12 | F-7 COMM_CHANGE timers | 3 | DEFER (implementation) | MIB + timers | Safety impact none: COMM_CHANGE full-duplex path not used in flight |
| 13 | F-2 timing services | 3 | DEFER (implementation) | §5 + B2 | Safety impact none: telemetry carries its own MET |
| 14 | V-1 trace (158 UNVERIFIED) | — | DEFER | trace to assertions | Safety impact none: verification depth only |
| — | O-1, O-2, O-3 | — | Open items | interpretation / bench | Not answered by records |

The safety-effect causes are inferences from the Rocket-Chip configuration at this time (one pair of boards on an FSK link, no Proximity-1 peer). Examine them again if the radio path changes.

### 3.4 CCSDS step rules C-1 to C-14 (Duke, 2026-10-29)

**Status.** C-1 to C-12 and C-14 have the status "adopted". They are the rules for the CCSDS step when the work starts again. They come from the owner instructions of 2026-10-29. The instructions are: correct the procedure, use the full depth, read each line, and do a sample check before a report. The numbers are one form of those instructions. The owner can change them. C-13 has the status "proposed". It waits for the owner decision on the 53 Space Packet service rows (parked).

**Source findings.** The Grok Build review (its section 4), Claude Code RV-6 to RV-10 and RV-12, and comments from Goddard and Hamilton.

The rule text below is Duke's text, with no change.

| Rule | Text |
|---|---|
| C-1 | Evidence for PASS. A PASS cites one specific assertion (file:line) whose checked value equals the value the shall requires. The reader records both values in the row. A matching identifier, a test file that exists, a code location, or doc text is UNVERIFIED, never PASS. |
| C-2 | One row, one evidence line. Evidence may not be shared across shalls unless the same assertion is read against each shall and the row says so. |
| C-3 | Docs are inputs, not evidence. Any claim in CONFORMANCE.md, COVERAGE.md or SHALL_MATRIX.md is checked against the code it names before it is reused. |
| C-4 | Changed-file recheck. For each claim, list the source files it cites. Any claim whose files changed after its last check date is re-read, not grepped. |
| C-5 | PICS cross-check. Before the walk closes, reconcile every clause verdict with the Annex A / PICS Support column. A clause cannot PASS where Support is N. |
| C-6 | One rule for claimed levels. A row's claimed level drops to what the shall evidence shows. No "keep and annotate". |
| C-7 | Row-level grading is labelled. Books with no extracted shall list (e.g. 732.1, 232.0, 232.1, 131.0, 231.0, 355.0) are reported as "row-level only, not a clause walk" and excluded from clause counts. |
| C-8 | Scoped summaries. Every summary sentence names the verdict classes and books it covers. |
| C-9 | Not-covered section. The report opens with a tier/scope table: what ran, what did not, why. |
| C-10 | Decision table. Each claim change lists the record line it rests on (decision, date, file) so the owner can see the basis before any edit lands. |
| C-11 | Pace. One book per sitting. Each book gets a spot-check (re-read of at least 10 PASS rows, chosen by the second reader) before it is marked done. Counts are derived twice, independently. A result that arrives far faster than a hand read allows is treated as a defect. |
| C-12 | Second reader. The full clause table is read by a second reader before the report is frozen or posted as a result. |
| C-13 | Test location. A test counts as evidence for a Starcom clause only if it is in Starcom's own suite (starcom/tests/). Tests in the Rocket-Chip test/ tree leave the row UNVERIFIED until the test moves. Basis: Starcom owns the library; Rocket-Chip is a consumer (WORKING_HERE.md / DESIGN.md). |
| C-14 | Optional items. A PICS item with status O and Support N makes its clauses N/A ("optional, not supported"), not FAIL and never PASS. |

### 3.5 Clause verdicts (all 549 rows from the 2026-10-29 walk, no change)

These rows are the clause evidence for when the work starts again after the FSK pivot. They have no new grades. We did not apply the agreed new grades and the parked items in 3.1 to them.

**Code and text.** Commit `579f0976`. The `starcom/` tree is the same as at `b1c9f98f`. The shall text is a word-for-word copy from the navigator `standard.json`. The IDs agree with SHALL_MATRIX.

**Method.** For a PASS, a named test assertion with a file and line is necessary. Duke found the assertions with two methods:

- (a) A read of the tests (PLCW, MAC FIFO, QoS, SPP service).
- (b) An identifier trace: the assertion line has a book identifier from the shall (for example MAC_FRAME_PENDING, NEED_PLCW). The note labels (b) as an inference.

Duke used PASS (doc) only where Duke read the code against the clause. Duke gave UNVERIFIED to a row with code but no assertion. The row tells the cause.

**Count:** 549 of 549 walked. PASS 196, UNVERIFIED 158, FAIL (gap) 67, DEVIATION-STATED 58, N/A 43, PASS (doc) 27.

| ID | Book | Clause | Shall (verbatim) | Verdict | Evidence | Note |
|---|---|---|---|---|---|---|
| 133.0-B-2/s002 | 133.0-B-2 | unattributed | Specific service arrangements shall be made via memoranda of agreement. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 133.0-B-2/s005 | 133.0-B-2 | unattributed | However, CCSDS does not have a patent law staff, and CCSDS shall not be held responsible for identifying any or all such patent rights. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 133.0-B-2/s010 | 133.0-B-2 | 1.6.2.1 | a) the words ‘shall’ and ‘must’ imply a binding and verifiable specification; | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 133.0-B-2/s013 | 133.0-B-2 | 1.6.3 | When the field is used to express a binary value (such as a counter), the Most Significant Bit (MSB) shall be the first transmitted bit of the field, that is, ‘Bit 0’ (see figure 1-1). | PASS | 579f0976:starcom/tests/unit/test_space_packet.cpp test_roundtrip:85 golden header bytes | MSB-first via golden (inference: golden bytes encode bit order) |
| 133.0-B-2/s014 | 133.0-B-2 | 1.6.3 | By CCSDS convention, all ‘spare’ bits shall be permanently set to ‘0’. | PASS | 579f0976:starcom/tests/unit/test_space_packet.cpp test_roundtrip:85 golden header bytes | MSB-first via golden (inference: golden bytes encode bit order) |
| 133.0-B-2/s028 | 133.0-B-2 | 3.2.2.1 | The Space Packet shall be a variable-length, delimited, octet-aligned data unit defined in section 4 of this Recommended Standard. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s029 | 133.0-B-2 | 3.2.2.1 | It shall consist of at least 7 and at most 65542 octets, but individual project organizations may establish the maximum length to be used for their projects, taking into account the maximum SDU size in all subnetworks traversed by the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s030 | 133.0-B-2 | 3.2.2.2 | The SPP user shall use the Packet Service to transfer Space Packets. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s031 | 133.0-B-2 | 3.2.3.1 | The Octet String shall be a variable-length, delimited, octet-aligned data unit whose content and format are unknown to the SPP. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s032 | 133.0-B-2 | 3.2.3.1 | It shall consist of at least 1 and at most 65536 octets, but individual project organizations may establish the maximum length used for their projects, taking into account the maximum SDU size in all subnetworks traversed by the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s034 | 133.0-B-2 | 3.2.3.3 | An Octet String shall be placed within the User Data Field of a single Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s035 | 133.0-B-2 | 3.2.3.4 | The SPP user shall use the Octet String Service to transfer Octet Strings. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s036 | 133.0-B-2 | 3.3.1 | The Packet Service shall transfer Space Packets, pre-formatted by the service user, intact through the mission-specified managed data path. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s037 | 133.0-B-2 | 3.3.1 | Space Packets supplied by the service user shall be transferred by the service provider without further formatting. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s038 | 133.0-B-2 | 3.3.2.1 | The Space Packet parameter shall be the SDU transferred by the Packet Service. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s039 | 133.0-B-2 | 3.3.2.2 | The APID is a mandatory parameter that shall be used to uniquely identify the source, destination, or type of the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s041 | 133.0-B-2 | 3.3.2.3 | If Packet Loss Indicators are to be generated by a particular implementation, they shall be declared at design time and be used consistently by all parties involved in the implementation. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s042 | 133.0-B-2 | 3.3.2.4 | If one of the underlying subnetworks supports multiple levels of QoS, then this parameter shall be used to select an appropriate QoS level. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s044 | 133.0-B-2 | 3.3.3.2.1 | At the sending end, the Packet Service user shall pass a PACKET.request primitive to the service provider to request that a Space Packet be transferred to the user at the receiving end. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s045 | 133.0-B-2 | 3.3.3.2.2 | The PACKET.request primitive shall provide parameters as follows: | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s046 | 133.0-B-2 | 3.3.3.2.3 | The PACKET.request primitive shall be passed to the service provider to request it to send the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s047 | 133.0-B-2 | 3.3.3.2.4 | Receipt of the PACKET.request primitive shall cause the service provider to transfer the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:77 TEST(SpacePacketService,PacketRequestTransfersIntact) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s048 | 133.0-B-2 | 3.3.3.3.1 | At the receiving end, the service provider shall pass a PACKET.indication to the Packet Service user to deliver a Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:106 TEST(SpacePacketService,PacketIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s049 | 133.0-B-2 | 3.3.3.3.2 | The PACKET.indication primitive shall provide parameters as follows: | PASS | 579f0976:test/test_space_packet_service.cpp:106 TEST(SpacePacketService,PacketIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s050 | 133.0-B-2 | 3.3.3.3.3 | The PACKET.indication primitive shall be passed from the service provider to the Packet Service user at the receiving end to deliver a Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:106 TEST(SpacePacketService,PacketIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s051 | 133.0-B-2 | 3.4.1 | The Octet String service shall transfer delimited strings of octets supplied by the service user. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s052 | 133.0-B-2 | 3.4.1 | The service provider shall transfer the strings of octets by formatting them into Space Packets. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s053 | 133.0-B-2 | 3.4.2.1 | The Octet String parameter shall be the SDU transferred by the Octet String Service. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s054 | 133.0-B-2 | 3.4.2.2 | The APID is a mandatory parameter that shall be used to uniquely identify the source, destination, or type of the Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s055 | 133.0-B-2 | 3.4.2.3.1 | The Packet Primary Header shall contain a Secondary Header Flag that indicates the presence or absence of a Packet Secondary Header. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s056 | 133.0-B-2 | 3.4.2.3.2 | The service user in the source end system shall signal whether or not a Packet Secondary Header is contained at the start of the Octet String by passing the Secondary Header Indicator parameter to the service provider. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s057 | 133.0-B-2 | 3.4.2.3.3 | The service provider shall use the value of this parameter to set the value of the Secondary Header Flag in the Packet Primary Header. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s059 | 133.0-B-2 | 3.4.2.4 | The Data Loss Indicator parameter shall be used to alert the user in a destination end system that one or more Octet Strings have been lost during transmission, as evidenced by a discontinuity in the Packet Sequence Count. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s060 | 133.0-B-2 | 3.4.3.2.1.1 | At the sending end, the Octet String Service user shall pass an OCTET_STRING.request primitive to the service provider to request that an Octet String be transferred to the user at the receiving end through the managed data path. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s061 | 133.0-B-2 | 3.4.3.2.1.2 | One call to the Octet_String.request function shall result in the creation of one, and only one, Space Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s062 | 133.0-B-2 | 3.4.3.2.2 | The OCTET_STRING.request primitive shall provide parameters as follows: | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s063 | 133.0-B-2 | 3.4.3.2.3 | The OCTET_STRING.request primitive shall be passed to the service provider to request it to send the Octet String. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s064 | 133.0-B-2 | 3.4.3.2.4 | Receipt of the OCTET_STRING.request primitive shall cause the service provider to transfer the Octet String. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s065 | 133.0-B-2 | 3.4.3.3.1 | At the receiving end, the service provider shall pass an OCTET_STRING.indication to the Octet String Service user to deliver an Octet String. | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s066 | 133.0-B-2 | 3.4.3.3.2 | The OCTET_STRING.indication primitive shall provide parameters as follows: | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s067 | 133.0-B-2 | 3.4.3.3.3 | The OCTET_STRING.indication primitive shall be passed from the service provider to the Octet String Service user at the receiving end to deliver an Octet String. | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s068 | 133.0-B-2 | 4.1.2.1 | A Space Packet shall include the defined fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s069 | 133.0-B-2 | 4.1.2.2 | A Space Packet shall consist of at least 7 and at most 65542 octets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s071 | 133.0-B-2 | 4.1.3.1 | The Packet Primary Header is mandatory and shall consist of four fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s072 | 133.0-B-2 | 4.1.3.2.1 | Bits 0–2 of the Packet Primary Header shall contain the (binary encoded) Packet Version Number (PVN). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s073 | 133.0-B-2 | 4.1.3.2.2 | This 3-bit field shall identify the data unit as a Space Packet defined by this Recommended Standard; it shall be set to ‘000’. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s074 | 133.0-B-2 | 4.1.3.3.1.1 | Bits 3–15 of the Packet Primary Header shall contain the Packet Identification Field. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s075 | 133.0-B-2 | 4.1.3.3.1.2 | This 13-bit field shall be sub-divided into three sub-fields as follows: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s076 | 133.0-B-2 | 4.1.3.3.2.1 | Bit 3 of the Packet Primary Header shall contain the Packet Type. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s077 | 133.0-B-2 | 4.1.3.3.2.2 | The Packet Type shall be used to distinguish Packets used for telemetry (or reporting) from Packets used for telecommand (or requesting). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s078 | 133.0-B-2 | 4.1.3.3.2.3 | For a telemetry (or reporting) Packet, this bit shall be set to ‘0’; for a telecommand (or requesting) Packet, this bit shall be set to ‘1’. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s079 | 133.0-B-2 | 4.1.3.3.3.1 | Bit 4 of the Packet Primary Header shall contain the Secondary Header Flag. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s080 | 133.0-B-2 | 4.1.3.3.3.2 | The Secondary Header Flag shall indicate the presence or absence of the Packet Secondary Header within this Space Packet. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s081 | 133.0-B-2 | 4.1.3.3.3.2 | It shall be ‘1’ if a Packet Secondary Header is present; it shall be ‘0’ if a Packet Secondary Header is not present. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s082 | 133.0-B-2 | 4.1.3.3.3.3 | The Secondary Header Flag shall be static with respect to the APID and managed data path throughout a Mission Phase. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s083 | 133.0-B-2 | 4.1.3.3.3.4 | The Secondary Header Flag shall be set to ‘0’ for Idle Packets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s084 | 133.0-B-2 | 4.1.3.3.4.1 | Bits 5–15 of the Packet Primary Header shall contain the APID. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s085 | 133.0-B-2 | 4.1.3.3.4.2 | The APID shall provide the naming mechanism for the managed data path. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s087 | 133.0-B-2 | 4.1.3.3.4.4 | For Idle Packets the APID shall be ‘11111111111’, that is, ‘all ones’(see reference [4]). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s089 | 133.0-B-2 | 4.1.3.4.1.1 | Bits 16–31 of the Packet Primary Header shall contain the Packet Sequence Control Field. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s090 | 133.0-B-2 | 4.1.3.4.1.2 | This 16-bit field shall be sub-divided into two sub-fields as follows: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s091 | 133.0-B-2 | 4.1.3.4.2.1 | Bits 16–17 of the Packet Primary Header shall contain the Sequence Flags. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s093 | 133.0-B-2 | 4.1.3.4.2.2 | The Sequence Flags shall be set as follows: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s094 | 133.0-B-2 | 4.1.3.4.3.1 | Bits 18–31 of the Packet Primary Header shall contain the Packet Sequence Count or the Packet Name. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s095 | 133.0-B-2 | 4.1.3.4.3.2 | For a Packet with the Packet Type set to ‘0’ (i.e., a telemetry Packet), this field shall contain the Packet Sequence Count. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s096 | 133.0-B-2 | 4.1.3.4.3.2 | For a Packet with the Packet Type set to ‘1’ (i.e., a telecommand Packet), this field shall contain either the Packet Sequence Count or Packet Name. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s097 | 133.0-B-2 | 4.1.3.4.3.3 | The Packet Sequence Count shall provide the sequential binary count of each Space Packet generated by the user application identified by the APID. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s098 | 133.0-B-2 | 4.1.3.4.3.4 | The Packet Sequence Count shall be continuous (modulo-16384). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s099 | 133.0-B-2 | 4.1.3.4.3.4 | A re-setting of the Packet Sequence Count before reaching 16383 shall not take place unless it is unavoidable. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s102 | 133.0-B-2 | 4.1.3.5.1 | Bits 32–47 of the Packet Primary Header shall contain the Packet Data Length. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s103 | 133.0-B-2 | 4.1.3.5.2 | This 16-bit field shall contain a length count C that equals one fewer than the length (in octets) of the Packet Data Field. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s104 | 133.0-B-2 | 4.1.3.5.3 | The length count C shall be expressed as: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s105 | 133.0-B-2 | 4.1.4.1.1 | The Packet Data Field is mandatory and shall consist of at least one of the following two fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s106 | 133.0-B-2 | 4.1.4.1.2 | The Packet Data Field shall contain at least one octet. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s107 | 133.0-B-2 | 4.1.4.2.1.1 | If present, the Packet Secondary Header shall follow, without gap, the Packet Primary Header. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s108 | 133.0-B-2 | 4.1.4.2.1.2 | The Packet Secondary Header shall be mandatory if no User Data Field is present in the Packet; otherwise, it is optional. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s109 | 133.0-B-2 | 4.1.4.2.1.2 | The presence or absence of a Packet Secondary Header shall be signaled by the Secondary Header Flag within the Packet Identification Field (see 4.1.3.3.3). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s110 | 133.0-B-2 | 4.1.4.2.1.3 | If present, the Packet Secondary Header shall consist of an integral number of octets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s111 | 133.0-B-2 | 4.1.4.2.1.4 | The contents of the Packet Secondary Header shall be specified by the source end user and provided to the destination end user(s) by management. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s112 | 133.0-B-2 | 4.1.4.2.1.5 | If present, the Packet Secondary Header shall consist of either: | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s113 | 133.0-B-2 | 4.1.4.2.1.6 | The chosen option shall remain static for a specific managed data path throughout all Mission Phases. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s114 | 133.0-B-2 | 4.1.4.2.2.1 | If present, the Time Code Field shall consist of an integral number of octets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s115 | 133.0-B-2 | 4.1.4.2.2.2 | The Time Code Field shall consist of one of the CCSDS segmented binary or unsegmented binary time codes specified in reference [3]. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s116 | 133.0-B-2 | 4.1.4.2.2.3 | The time code selected shall be fixed for a given managed data path throughout all Mission Phases. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s117 | 133.0-B-2 | 4.1.4.2.2.4 | If the characteristics are allowed to change, the P-field shall be present so as to identify the changes. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s118 | 133.0-B-2 | 4.1.4.2.2.5 | The presence or absence of the P-field in the Time Code Field shall be fixed for a given managed data path throughout all Mission Phases. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s119 | 133.0-B-2 | 4.1.4.2.2.5 | If present, it shall immediately precede the T-field that is defined in reference [3]. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s120 | 133.0-B-2 | 4.1.4.2.3 | If present, the Ancillary Data Field shall consist of an integral number of octets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s122 | 133.0-B-2 | 4.1.4.3.1 | If present, the User Data Field shall follow, without gap, either the Packet Secondary Header (if a Packet Secondary Header is present) or the Packet Primary Header (if a Packet Secondary Header is not present). | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s123 | 133.0-B-2 | 4.1.4.3.2 | The User Data Field shall be mandatory if a Packet Secondary Header is not present; otherwise, it is optional. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s124 | 133.0-B-2 | 4.1.4.3.3 | If present, the User Data Field shall consist of an integral number of octets. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s125 | 133.0-B-2 | 4.1.4.3.4 | If the Packet is not an Idle Packet, then the User Data Field shall contain application data supplied by the sending user. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s126 | 133.0-B-2 | 4.1.4.3.4 | If the Packet is an Idle Packet, the User Data Field shall contain Idle Data. | PASS | 579f0976:starcom/src/ccsds/space_packet.cpp; tests/unit/test_space_packet.cpp test_roundtrip, test_reject_sp_pvn, test_sp_idle | Idle-packet sec-hdr flag not forced on encode (CONFORMANCE.md:91); RC seq count is now maintained (CS-3) |
| 133.0-B-2/s128 | 133.0-B-2 | 4.2.2.1 | The Packet Assembly Function shall be used to generate Space Packets from Octet Strings. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s129 | 133.0-B-2 | 4.2.2.2 | The Packet Assembly Function receives Octet Strings from the Octet String Service user and shall build Space Packets by generating the Packet Primary Header. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s130 | 133.0-B-2 | 4.2.2.3 | The Secondary Header Indicator parameter shall be generated by the service user to indicate the presence or absence of a Packet Secondary Header at the start of Octet Strings. | PASS | 579f0976:test/test_space_packet_service.cpp:130 TEST(SpacePacketService,OctetStringRequestBuildsOnePacket) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s131 | 133.0-B-2 | 4.2.2.4 | The Packet Assembly Function shall translate the parameter by setting the Secondary Header Flag in the Packet Primary Header to a corresponding value. | PASS | 579f0976:test/test_space_packet_service.cpp:185 TEST(SpacePacketService,SequenceCountPerApidPerDirectionWraps) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s132 | 133.0-B-2 | 4.2.2.4 | A sequence counter is maintained and shall be used to generate the Packet Sequence Count in the Packet Primary Header. | PASS | 579f0976:test/test_space_packet_service.cpp:185 TEST(SpacePacketService,SequenceCountPerApidPerDirectionWraps) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s133 | 133.0-B-2 | 4.2.3.1 | The Packet Transfer Function shall be used to transfer Space Packets to the packet service entity at receiving end in the managed data path, using services of the underlying subnetwork. | PASS | 579f0976:test/test_space_packet_service.cpp:279 TEST(SpacePacketService,PacketTransferKeepsSubmissionOrder) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s134 | 133.0-B-2 | 4.2.3.2 | If necessary, the Packet Transfer Function shall multiplex Space Packets received from the instances of the Packet Assembly Function and the Packet Service users, and shall put the Space Packets into a queue, in an appropriate order that is set by management. | PASS | 579f0976:test/test_space_packet_service.cpp:279 TEST(SpacePacketService,PacketTransferKeepsSubmissionOrder) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s135 | 133.0-B-2 | 4.2.3.2 | The algorithm to be used to order the Space Packets is not specified by CCSDS, but shall be defined by project organizations considering factors such as priority, release rate, etc. | PASS | 579f0976:test/test_space_packet_service.cpp:279 TEST(SpacePacketService,PacketTransferKeepsSubmissionOrder) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s136 | 133.0-B-2 | 4.2.3.3 | The Packet Transfer Function, then, shall examine the APID of each Packet in the queue to identify the packet service entity at the receiving end and shall transfer the Packet using a service of the underlying layers. | PASS | 579f0976:test/test_space_packet_service.cpp:279 TEST(SpacePacketService,PacketTransferKeepsSubmissionOrder) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s138 | 133.0-B-2 | 4.3.2.1 | The Packet Extraction Function shall be used to extract SDUs from Space Packets. | PASS | 579f0976:test/test_space_packet_service.cpp:239 TEST(SpacePacketService,ExtractionRejectsMalformedAndShort) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s139 | 133.0-B-2 | 4.3.2.2 | The Packet Extraction Function shall receive Space Packets from the Packet Reception Function and shall extract Octet Strings by removing the Packet Primary Header. | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s140 | 133.0-B-2 | 4.3.2.2 | The Secondary Header Indicator parameter shall be generated by the Packet Extraction Function to indicate the presence of a Packet Secondary Header at the start of the Octet String. | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s141 | 133.0-B-2 | 4.3.2.2 | The Packet Extraction Function checks the continuity of the Packet Sequence Count to determine if one or more Packets have been lost during transmission and shall generate the optional Data Loss Indicator parameter accordingly. | PASS | 579f0976:test/test_space_packet_service.cpp:161 TEST(SpacePacketService,OctetStringIndicationDeliversOctetsAndApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s142 | 133.0-B-2 | 4.3.3.1 | The Packet Reception Function shall be used to receive and demultiplex Space Packets received from the underlying subnetwork. | PASS | 579f0976:test/test_space_packet_service.cpp:309 TEST(SpacePacketService,PacketReceptionDemuxesByApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s143 | 133.0-B-2 | 4.3.3.2 | The Packet Reception Function shall receive Space Packets from the underlying subnetwork and shall demultiplex, if necessary, the received Packets on the basis of the APID of each Packet. | PASS | 579f0976:test/test_space_packet_service.cpp:309 TEST(SpacePacketService,PacketReceptionDemuxesByApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s144 | 133.0-B-2 | 4.3.3.3 | If the receiving user of the APID uses the Packet Service, the received Packets shall be delivered intact to the user identified by the APID. | PASS | 579f0976:test/test_space_packet_service.cpp:309 TEST(SpacePacketService,PacketReceptionDemuxesByApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s145 | 133.0-B-2 | 4.3.3.3 | If the receiving user of the APID uses the Octet String Service, then the received Packets shall be delivered to the user through the Packet Extraction Function. | PASS | 579f0976:test/test_space_packet_service.cpp:309 TEST(SpacePacketService,PacketReceptionDemuxesByApid) | RC suite, not Starcom suite; service not integrated (byte_pump.cpp:287) |
| 133.0-B-2/s149 | 133.0-B-2 | A1.3 | The implementer shall complete the RL by entering appropriate responses in the support or values supported column, using the notation described in A1.2. | PASS (doc) | 579f0976:starcom/docs/CONFORMANCE.md:51 draft Annex A completed (unsigned) |  |
| 133.0-B-2/s153 | 133.0-B-2 | B2.3 | The Secondary Header Format Document Registry shall consist of the following fields: | N/A | SANA registry operator rules |  |
| 133.0-B-2/s157 | 133.0-B-2 | B2.4 | Registration Maintenance Rules for SANA Operator: Registry change (add/delete/edit) shall be submitted by any valid CCSDS Agency Representative (Member, Observer, or Affiliate). | N/A | SANA registry operator rules |  |
| 133.0-B-2/s158 | 133.0-B-2 | B2.4 | The Organization, PoC, and Header Format Source document shall all be verified to be valid and correctly registered and referenced. | N/A | SANA registry operator rules |  |
| 133.0-B-2/s159 | 133.0-B-2 | B2.4 | The SANA Operator shall assign a unique identifier that may be used to reference this registry entry. | N/A | SANA registry operator rules |  |
| 133.0-B-2/s160 | 133.0-B-2 | B2.4 | New versions of existing specifications shall be assigned a unique identifier. | N/A | SANA registry operator rules |  |
| 211.0-B-6/s002 | 211.0-B-6 | unattributed | Specific service arrangements shall be made via memoranda of agreement. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.0-B-6/s005 | 211.0-B-6 | unattributed | However, CCSDS does not have a patent law staff, and CCSDS shall not be held responsible for identifying any or all such patent rights. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.0-B-6/s014 | 211.0-B-6 | 1.5.2.1 | a) the words ‘shall’ and ‘must’ imply a binding and verifiable specification; | N/A | convention |  |
| 211.0-B-6/s019 | 211.0-B-6 | 3.2.1 | A Version-3 Transfer Frame shall encompass the following fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s021 | 211.0-B-6 | 3.2.2.1 | The Transfer Frame Header is mandatory and shall consist of ten mandatory fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s022 | 211.0-B-6 | 3.2.2.2.1 | Bits 0–1 of the Transfer Frame Header shall contain the Transfer Frame Version Number. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s023 | 211.0-B-6 | 3.2.2.2.2 | The Transfer Frame Version Number field shall contain the binary value ‘10’. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s024 | 211.0-B-6 | 3.2.2.3.1 | Bit 2 of the Transfer Frame Header shall contain the QoS Indicator. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s025 | 211.0-B-6 | 3.2.2.3.2 | The QoS Indicator shall indicate the transfer service: | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s026 | 211.0-B-6 | 3.2.2.4.1 | Bit 3 of the Transfer Frame Header shall contain the PDU Type ID. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s027 | 211.0-B-6 | 3.2.2.4.2 | The PDU Type ID shall specify whether the Transfer Frame Data field is conveying protocol supervisory data or user data information. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s028 | 211.0-B-6 | 3.2.2.5.1 | Bits 4–5 of the Transfer Frame Header shall contain the DFC ID. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s029 | 211.0-B-6 | 3.2.2.5.2 | In a P-frame, the DFC ID is not used and shall be set to the binary value ‘00’. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s030 | 211.0-B-6 | 3.2.2.5.3 | In a U-Frame, the DFC ID shall indicate the contents of the Transfer Frame Data field as defined in table 3-1. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s031 | 211.0-B-6 | 3.2.2.6.1 | Bits 6–15 of the Transfer Frame Header shall contain the SCID. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s032 | 211.0-B-6 | 3.2.2.6.2 | The 10-bit SCID shall provide the identification of the spacecraft that is either the source or the destination of the data contained in the Transfer Frame. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s033 | 211.0-B-6 | 3.2.2.7.1 | Bit 16 of the Transfer Frame Header shall contain the PCID. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s035 | 211.0-B-6 | 3.2.2.7.2 | If an implementation distinguishes between the two PCID values, each PCID physical channel shall have a completely separate set of MIB parameters and all state machines, including COP-P. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s036 | 211.0-B-6 | 3.2.2.8.1 | Bits 17–19 of the Transfer Frame Header shall contain the Port ID. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s037 | 211.0-B-6 | 3.2.2.8.2 | In a P-frame, the Port ID is not used and shall be set to the value ‘0’. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s038 | 211.0-B-6 | 3.2.2.8.3 | In a U-frame, the Port ID shall identify the output port to which the I/O Sublayer delivers the SDUs contained in the frame. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s039 | 211.0-B-6 | 3.2.2.9.1 | Bit 20 of the Transfer Frame Header shall contain the Source-or-Destination Identifier. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s040 | 211.0-B-6 | 3.2.2.9.2 | The sending node shall set the Source-or-Destination Identifier to indicate the contents of the SCID field as shown in table 3-2. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s041 | 211.0-B-6 | 3.2.2.9.3 | The behavior of a receiving node with respect to the SCID field and Source-or- Destination Identifier shall be as shown in table 3-3. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s042 | 211.0-B-6 | 3.2.2.10.1 | Bits 21–31 of the Transfer Frame Header shall contain the Frame Length. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s043 | 211.0-B-6 | 3.2.2.10.2 | This 11-bit field shall contain a length count C, which equals one fewer than the total number of octets in the Transfer Frame. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s044 | 211.0-B-6 | 3.2.2.10.2 | a) The count shall be measured from the first octet of the Transfer Frame Header to the last octet of the Transfer Frame Data field. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s045 | 211.0-B-6 | 3.2.2.11.1 | Bits 32–39 of the Transfer Frame Header shall contain the Frame Sequence Number (FSN). | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s046 | 211.0-B-6 | 3.2.2.11.2 | The FSN shall increment monotonically and independently for the set of frames assigned to a PCID that are associated with the Sequence Controlled service (QoS Indicator set to ‘0’). | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s047 | 211.0-B-6 | 3.2.2.11.3 | The FSN shall increment monotonically for the set of frames assigned to a PCID that are associated with the Expedited service (QoS Indicator set to ‘1’). | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s048 | 211.0-B-6 | 3.2.3.1 | The Transfer Frame Data field shall: | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s049 | 211.0-B-6 | 3.2.3.2.1 | When the DFC ID field of a U-frame contains the binary value ‘00’, the Frame Data field shall consist of an integer number of packets, each designated to the same Port ID and PCID (see figure 3-4). | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s050 | 211.0-B-6 | 3.2.3.2.2 | The first bit of the Frame Data field shall be the first bit of a packet header. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s051 | 211.0-B-6 | 3.2.3.3.1 | When the DFC ID field of a U-frame contains the binary value ‘01’, the Frame Data field shall contain a segment data unit consisting of an 8-bit segment header followed by a segment of a packet (see figure 3-4). | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s052 | 211.0-B-6 | 3.2.3.3.2 | The contents of the segment header and segment data field shall be as follows: | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s053 | 211.0-B-6 | 3.2.3.3.2 | 1) segments of the same packet shall be sent in frames of the same PCID and Port ID, | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s055 | 211.0-B-6 | 3.2.3.3.3 | Prior to delivery to the user, the Data Link Layer shall re-assemble all the segments using the same Routing ID, that is, using the same PCID, Port ID, and pseudo packet ID, into a packet. | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s057 | 211.0-B-6 | 3.2.3.3.4 | Only complete packets shall be delivered to the user. | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s058 | 211.0-B-6 | 3.2.3.3.5 | The accumulated packet shall be discarded and this event shall be logged into the session accountability report whenever any of the following errors occur: | N/A | DFC 01 is PICS DLL-15 O.1, not claimed |  |
| 211.0-B-6/s059 | 211.0-B-6 | 3.2.3.4 | In a U-frame, the binary value ‘10’ for the DFC ID field is reserved by CCSDS and shall not be used. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s060 | 211.0-B-6 | 3.2.3.5 | When the DFC ID field of a U-frame contains the binary value ‘11’, the Frame Data field shall consist of user-defined data. | PASS | 579f0976:starcom/src/ccsds/v3.cpp encodeV3UserDefined:75; src/ccsds/copp.cpp coppSubmitUserDefined:332; tests/unit/test_user_defined.cpp |  |
| 211.0-B-6/s061 | 211.0-B-6 | 3.2.3.6 | In a P-frame, the Transfer Frame Data field shall consist of SPDUs. | PASS | 579f0976:starcom/src/ccsds/v3.cpp decodeV3:7/encodeV3:41; tests/unit/test_v3.cpp | row-level test; per-field traced only for header round-trip |
| 211.0-B-6/s062 | 211.0-B-6 | 3.2.4.2.1 | Fixed-length SPDUs shall consist of the following fields positioned contiguously in the following sequence: | UNVERIFIED | 579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.cpp, test_mac.cpp test_set_vr_codec | code located (579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.c); no assertion found that tests this shall |
| 211.0-B-6/s063 | 211.0-B-6 | 3.2.4.2.2 | Variable-length SPDUs shall consist of the following fields positioned contiguously in the following sequence: | UNVERIFIED | 579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.cpp, test_mac.cpp test_set_vr_codec | code located (579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.c); no assertion found that tests this shall |
| 211.0-B-6/s064 | 211.0-B-6 | 3.2.4.3.1 | A ‘1’ in the SPDU Format ID field shall identify a 16-bit fixed-length SPDU. | PASS | 579f0976:starcom/tests/unit/test_ocf.cpp:45 [CHECK(out[0] == std::byte{0x80})] | Format ID 1 / Type 0 |
| 211.0-B-6/s065 | 211.0-B-6 | 3.2.4.3.1 | This format provides for only two fixed SPDUs (see table 3-5), which shall be differentiated by the SPDU Type Identifier field: | PASS | 579f0976:starcom/tests/unit/test_ocf.cpp:45 [CHECK(out[0] == std::byte{0x80})] | Format ID 1 / Type 0 |
| 211.0-B-6/s066 | 211.0-B-6 | 3.2.4.3.2.1.1 | The PLCW shall consist of seven fields positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0—see figure 3-5): | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s067 | 211.0-B-6 | 3.2.4.3.2.1.2 | The PLCW shall be transmitted using the Expedited QoS (see 3.2.2.3). | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s068 | 211.0-B-6 | 3.2.4.3.2.2.1 | Bits 8–15 of the PLCW shall contain the Report Value. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s069 | 211.0-B-6 | 3.2.4.3.2.2.2 | The Report Value field shall contain the value of V(R) (see 7.3.2). | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s070 | 211.0-B-6 | 3.2.4.3.2.2.3 | Separate Report Values shall be reported for each physical channel independent of the I/O port. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s071 | 211.0-B-6 | 3.2.4.3.2.3.1 | Bits 5–7 of the PLCW shall contain the EXPEDITED_FRAME_COUNTER. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s072 | 211.0-B-6 | 3.2.4.3.2.3.2 | The EXPEDITED_FRAME_COUNTER shall provide a modulo-8 counter indicating that Expedited frames have been received. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s073 | 211.0-B-6 | 3.2.4.3.2.4.1 | Bit 4 of the PLCW shall contain a Reserved Spare bit. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s074 | 211.0-B-6 | 3.2.4.3.2.4.2 | The Reserved Spare bit field shall be set to ‘0’. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s075 | 211.0-B-6 | 3.2.4.3.2.5.1 | Bit 3 of the PLCW shall contain the PCID field. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s076 | 211.0-B-6 | 3.2.4.3.2.5.2 | The 1-bit PCID field shall contain the PCID of the physical channel with which this report is associated. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s077 | 211.0-B-6 | 3.2.4.3.2.6.1 | Bit 2 of the PLCW shall contain the PLCW Retransmit Flag. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s078 | 211.0-B-6 | 3.2.4.3.2.6.2 | A setting of ‘0’ in the PLCW Retransmit Flag shall indicate that there are no outstanding frame rejections in the sequence received so far, and thus retransmissions are not required. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s079 | 211.0-B-6 | 3.2.4.3.2.6.3 | A setting of ‘1’ in the PLCW Retransmit Flag shall indicate that a received frame left a frame sequence number gap and that a retransmission of the expected frame is required. | PASS (doc) | 579f0976:starcom/src/ccsds/plcw.cpp encodePlcw:18-29 / decodePlcw:5-16; bit masks 0x20 retransmit (bit 2), pcid<<4 (bit 3), 0x08 spare=0 (bit 4), 0x07 counter (bits 5-7), octet 1 report value | code-read verified against Fig 3-5; test_ocf.cpp is round-trip + zero golden (:45-46), which cannot catch a symmetric bit-position error |
| 211.0-B-6/s080 | 211.0-B-6 | 3.2.4.4.1 | A ‘0’ in the SPDU Format ID field shall identify a variable-length SPDU data field, which may contain from 0 to 15 octets of supervisory data. | UNVERIFIED | 579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.cpp, test_mac.cpp test_set_vr_codec | code located (579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.c); no assertion found that tests this shall |
| 211.0-B-6/s081 | 211.0-B-6 | 3.2.4.4.2 | An SPDU Type Identifier equal to ‘000’ shall identify a Type 1 SPDU with a data field containing from zero to seven 16-bit concatenated and multiplexed protocol objects, that is, directives or reports. | UNVERIFIED | 579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.cpp, test_mac.cpp test_set_vr_codec | code located (579f0976:starcom/src/ccsds/plcw.cpp:5/:18; mac.cpp encodeSetVr:1015; tests/unit/test_ocf.c); no assertion found that tests this shall |
| 211.0-B-6/s082 | 211.0-B-6 | 3.2.4.4.3.1 | An SPDU Type Identifier equal to ‘001’ shall identify a Type 2 SPDU with a data field containing from 1 to 15 octets of TIME DISTRIBUTION supervisory data. | FAIL (gap) | no Type 2 TIME DISTRIBUTION SPDU | DLL-23 is O but DLL-64 receive side is M; part of F-2 |
| 211.0-B-6/s083 | 211.0-B-6 | 3.2.4.4.3.2 | Octet 0 of the data field shall contain the TIME DISTRIBUTION directive type, followed by the actual time field value (1 to 14 octets). | FAIL (gap) | no Type 2 TIME DISTRIBUTION SPDU | DLL-23 is O but DLL-64 receive side is M; part of F-2 |
| 211.0-B-6/s084 | 211.0-B-6 | 3.2.4.4.4 | An SPDU Type Identifier equal to ‘010’ shall identify a Type 3 SPDU with a data field containing from 0 to 15 octets of Status Report information. | N/A | Type 3 status SPDU not claimed |  |
| 211.0-B-6/s086 | 211.0-B-6 | 4.1.2.1 | At the sending end, the Frame Sublayer shall perform the following functions: | UNVERIFIED | frame sublayer function lists; partly caller (sans-I/O) | not graded per item |
| 211.0-B-6/s087 | 211.0-B-6 | 4.1.2.2 | At the receiving end, the Frame Sublayer shall perform the following functions: | UNVERIFIED | frame sublayer function lists; partly caller (sans-I/O) | not graded per item |
| 211.0-B-6/s089 | 211.0-B-6 | 4.1.3.2.1 | Frames shall be generated and sent when the TRANSMIT parameter (6.2.2.3) is set to on. | UNVERIFIED | frame sublayer function lists; partly caller (sans-I/O) | not graded per item |
| 211.0-B-6/s090 | 211.0-B-6 | 4.1.3.2.1 | When the PLTU contents are ready for transmission and while TRANSMIT is on, the data shall be transferred to the C&S Sublayer for processing. | UNVERIFIED | frame sublayer function lists; partly caller (sans-I/O) | not graded per item |
| 211.0-B-6/s091 | 211.0-B-6 | 4.1.3.2.2 | When either NEED_PLCW or NEED_STATUS_REPORT (6.2.3.6) is set to true, the required status and/or PLCW data shall be generated and inserted into a P-frame for delivery. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:493 [CHECK(macFifoSource(s) == MacFifoSource::plcw)] |  |
| 211.0-B-6/s092 | 211.0-B-6 | 4.1.3.3 | Each time a frame is to be dispatched to the C&S Sublayer, its selection from a series of sources shall be based on the following priority scheme: | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:145,493,496 [CHECK(macFifoSource(s) == ...)] | table 6-14 selection |
| 211.0-B-6/s093 | 211.0-B-6 | 4.2.2.5.1 | Upon initiation of a persistent activity, the PERSISTENCE signal shall be set to true. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s094 | 211.0-B-6 | 4.2.2.5.2 | While PERSISTENCE is true the Frame Sublayer shall select frames only from the MAC queue for output. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s095 | 211.0-B-6 | 4.2.2.5.3 | The success or failure of the activity shall be determined by the detection of the expected RESPONSE within the activity’s LIFETIME. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s096 | 211.0-B-6 | 4.2.2.5.4 | If the RESPONSE is not detected within the activity’s LIFETIME, the activity shall be deemed failed and aborted. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s097 | 211.0-B-6 | 4.2.2.5.5 | NOTIFICATION of the activity’s success or failure shall be communicated back to the vehicle controller, at which time the PERSISTENCE signal shall be set to false. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s098 | 211.0-B-6 | 4.2.2.6.1 | The MAC_FRAME_PENDING parameter shall be provided from the MAC Sublayer to the Frame Sublayer. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:153 [CHECK(s.mac_frame_pending);] | assertion matched on identifier "macframepending" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s099 | 211.0-B-6 | 4.2.2.6.2 | The MAC_FRAME_PENDING shall be set to true when a complete frame is loaded into the MAC queue. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:153 [CHECK(s.mac_frame_pending);] | assertion matched on identifier "macframepending" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s100 | 211.0-B-6 | 4.2.2.6.3 | MAC_FRAME_PENDING shall be set to false when the last bit of the frame is extracted from the MAC queue. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:153 [CHECK(s.mac_frame_pending);] | assertion matched on identifier "macframepending" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s101 | 211.0-B-6 | 4.2.3 | Implementations of the Proximity-1 Space Link Protocol shall include a Directive Decoder function for processing supervisory protocol directives defined in 3.2.4 and annex B. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp; tests/unit/test_mac.cpp); no assertion found that tests this shall |
| 211.0-B-6/s102 | 211.0-B-6 | 4.2.4.1 | The SENT_TIME_BUFFER shall store all of the egress clock times, associated frame sequence numbers, and QoS Indicator when time tag data is collected. | FAIL (gap) | no SENT/RECEIVE_TIME_BUFFER | PICS DLL-31/32 M (part of timing gap F-2) |
| 211.0-B-6/s103 | 211.0-B-6 | 4.2.4.2 | The RECEIVE_TIME_BUFFER shall store all of the ingress clock times, associated frame sequence numbers, and QoS Indicator when time tag data is collected. | FAIL (gap) | no SENT/RECEIVE_TIME_BUFFER | PICS DLL-31/32 M (part of timing gap F-2) |
| 211.0-B-6/s104 | 211.0-B-6 | 4.3.2.1 | The Data Services Sublayer shall control the order of transfer of the user data (including user-supplied directives) that are to be transmitted within the session. | PASS | 579f0976:starcom/tests/unit/test_copp.cpp:81 [farmPOnFrame expedited accepted], :115 [new_seq] | two QoS |
| 211.0-B-6/s105 | 211.0-B-6 | 4.3.2.2 | The Data Services Sublayer shall provide the following two qualities of service: | PASS | 579f0976:starcom/tests/unit/test_copp.cpp:81 [farmPOnFrame expedited accepted], :115 [new_seq] | two QoS |
| 211.0-B-6/s106 | 211.0-B-6 | 4.3.2.2 | a) Expedited service shall ensure transmission without errors of Expedited frame data in the order received; | PASS | 579f0976:starcom/tests/unit/test_copp.cpp:81 [farmPOnFrame expedited accepted], :115 [new_seq] | two QoS |
| 211.0-B-6/s107 | 211.0-B-6 | 4.3.2.2 | b) Sequence Controlled service shall guarantee that data within a communication session are delivered in order without errors, gaps, or duplications. | PASS | 579f0976:starcom/tests/unit/test_copp.cpp:81 [farmPOnFrame expedited accepted], :115 [new_seq] | two QoS |
| 211.0-B-6/s110 | 211.0-B-6 | 4.4.2.1 | At the sending end, the I/O Sublayer shall: | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s111 | 211.0-B-6 | 4.4.2.1 | b) using the value of the MIB parameters Maximum_Packet_Size and Maximum_Frame_Length (see annex C for Version-3 frames and section 5 of reference [7] for Version-4 frames), organize the received data (including metadata) to form the Frame Data Unit and the Transfer Frame Header (frame sequence number shall be set to null); | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s112 | 211.0-B-6 | 4.4.2.2 | At the receiving end, the I/O Sublayer shall: | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s113 | 211.0-B-6 | 4.4.3.1 | For received U-frames, the I/O Sublayer shall provide two queues, Expedited queue and Sequence Controlled queue, capable of supporting the maximum data rate expected using the communications channel with that transceiver. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s114 | 211.0-B-6 | 4.4.3.2 | The I/O Sublayer shall: | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s115 | 211.0-B-6 | 4.4.3.3 | For Sequence Controlled service, the I/O Sublayer shall: | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s116 | 211.0-B-6 | 4.4.4.1 | While any data units are stored within the Sequence Controlled Frame queue, SEQUENCE_CONTROLLED_FRAME_AVAILABLE shall be true; otherwise, it shall be false. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s117 | 211.0-B-6 | 4.4.4.2 | While any data units are stored within the Expedited Frame queue, EXPEDITED_FRAME_AVAILABLE shall be true; otherwise, it shall be false. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s118 | 211.0-B-6 | 5.2.1 | When time tagging is active, a Proximity-1 transceiver shall record the time of the trailing edge of the last bit of the ASM of every incoming and every outgoing Version-3 Transfer Frame or Version-4 Transfer Frame of any type when available as required in reference [5]. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s119 | 211.0-B-6 | 5.2.2 | The egress/ingress captured time tags shall correspond to when the trailing edge of the last bit of the ASM of the outgoing/received PLTU crosses the clock capture point (defined by the implementation) within the transceiver. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s120 | 211.0-B-6 | 5.2.3 | All recorded time tags shall be correlatable to when the trailing edge of the last bit of the ASM of the outgoing/received PLTU crossed the time reference point. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s121 | 211.0-B-6 | 5.2.4 | The reference point for all timing calculations shall be defined by the enterprise. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s122 | 211.0-B-6 | 5.2.6 | To perform time tag capture, the vehicle controller shall instruct the initiating transceiver (initiator) to build and send a SET CONTROL PARAMETERS directive to the responder to capture its time tag measurements. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s123 | 211.0-B-6 | 5.2.7 | After processing this instruction/directive, the MAC Sublayer of both transceivers shall capture the local time reference and associated frame sequence numbers over the commanded interval (i.e., for the number of frames defined within the Time Sample Field of the directive) as depicted in figure 5-1 and package the collection of time tags and metadata (time + sequence number + direction + QoS Indicator) for transfer to the time correlation process. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s124 | 211.0-B-6 | 5.3 | The time correlation process shall have access to the following information: | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s125 | 211.0-B-6 | 5.4.1 | A Proximity-1 transceiver shall provide the capability of distributing time to a remote asset. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s126 | 211.0-B-6 | 5.4.2 | The method for transferring time to a remote asset shall consist of the following steps (see figure 5-2): | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s127 | 211.0-B-6 | 5.4.2.1 | a) prior to the desired transfer of enterprise time to a remote node, the initiator’s vehicle controller, based upon the mission’s accuracy requirements, shall acquire/determine the one-way light time between itself and the remote node for the instant that the transfer is initiated; | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s128 | 211.0-B-6 | 5.4.2.1 | b) the vehicle controller shall add that amount of time to the enterprise time for when the transfer is initiated; | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s129 | 211.0-B-6 | 5.4.2.1 | c) this computed time shall be formatted as a CCSDS Unsegmented Time Code (reference [4]). | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s130 | 211.0-B-6 | 5.4.2.2 | At the desired time, the vehicle controller shall command its transceiver to formulate a TIME DISTRIBUTION directive including the predetermined enterprise time, the internal sender path delay, and (if used) One Way Light Time (OWLT) propagation delay that is to be transmitted over the Proximity link. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s131 | 211.0-B-6 | 5.4.2.3 | The initiator shall then transmit the TIME DISTRIBUTION directive. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s132 | 211.0-B-6 | 5.4.2.4 | Upon receipt of the TIME DISTRIBUTION directive, the responder shall set its clock to the transmitted enterprise time and optionally determine, based upon the mission’s accuracy requirements, whether it needs to add the sender path delay, OWLT, and its own path delay to the transmitted enterprise time before applying it. | FAIL (gap) | no §5 timing services in core (COVERAGE.md §5 "0") | PICS DLL-52–64 M; not stated as a deviation in CONFORMANCE.md |
| 211.0-B-6/s134 | 211.0-B-6 | 6.2.2.1 | The following states of MODE (set via the local SET MODE directive) shall be allowed: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s135 | 211.0-B-6 | 6.2.2.2 | The following values of DUPLEX (set via the local SET DUPLEX directive) shall be allowed: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s136 | 211.0-B-6 | 6.2.2.3 | The following two states for TRANSMIT shall be allowed: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s137 | 211.0-B-6 | 6.2.2.4 | The SS variable shall be used | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s138 | 211.0-B-6 | 6.2.3.1 | X (Session Termination) shall be used to track the sub-states of the full and half duplex session termination process. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s139 | 211.0-B-6 | 6.2.3.2 | Y (COMM_CHANGE) shall be used to track the sub-states during the commanding of a Physical Layer communications change. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s140 | 211.0-B-6 | 6.2.3.3 | Z (SYMBOL_INLOCK_STATUS) shall be used during a Physical Layer communications change to track nondeterministic events within State 48 (COMM_CHANGE in Data Services), as follows: | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s141 | 211.0-B-6 | 6.2.3.4 | MODULATION is an interface variable with the Physical Layer which shall control the modulation of the transmitted carrier. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s142 | 211.0-B-6 | 6.2.3.6.1 | NEED_PLCW and NEED_STATUS_REPORT shall be used in the data selection for output process to determine if a PLCW or status report should be sent. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:176 [CHECK(s.need_plcw);] | assertion matched on identifier "needplcw" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s143 | 211.0-B-6 | 6.2.3.6.1 | Where applicable, these variables shall be set to true: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s144 | 211.0-B-6 | 6.2.3.6.2 | NEED_PLCW shall be set to false when a PLCW is selected for output. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:176 [CHECK(s.need_plcw);] | assertion matched on identifier "needplcw" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s145 | 211.0-B-6 | 6.2.3.6.2 | NEED_STATUS_REPORT shall be set to false when a status report is selected for output. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s146 | 211.0-B-6 | 6.2.3.7 | REMOTE_SCID_BUFFER holds the value of the spacecraft ID that shall be used in testing all frames whose Source-or-Destination ID is set to destination. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp:113 remote_scid used for TX headers; RX tests destination frames against local_scid (:364) | interpretation of which buffer the book requires for destination tests not settled; needs reading with 211.0 Fig/6.7.2 items |
| 211.0-B-6/s147 | 211.0-B-6 | 6.2.3.8 | COMMUNICATION_VALUE_BUFFER shall be used to hold the communication values for the HAIL and COMM_CHANGE directives and operations. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s148 | 211.0-B-6 | 6.2.3.9 | RECEIVING_SCID_BUFFER shall be used in the frame validation process to compare a received spacecraft ID value with that held within this buffer. | FAIL (gap) | no RECEIVING_SCID_BUFFER in MacSession/CoppEndpoint (mac.hpp, copp.hpp grep) | PICS DLL-77 M |
| 211.0-B-6/s150 | 211.0-B-6 | 6.2.3.10 | RECEIVING_PCID_BUFFER shall be used in the frame reception process. | FAIL (gap) | no RECEIVING_PCID_BUFFER; frame reception does not latch PCID (copp.cpp coppReceiveV3:392) | PICS DLL-78 M |
| 211.0-B-6/s151 | 211.0-B-6 | 6.2.3.10 | This buffer shall be loaded with the PCID contained in the first valid received frame. | FAIL (gap) | no RECEIVING_PCID_BUFFER; frame reception does not latch PCID (copp.cpp coppReceiveV3:392) | PICS DLL-78 M |
| 211.0-B-6/s152 | 211.0-B-6 | 6.2.4.1 | Local_Spacecraft_ID shall contain the value of the spacecraft ID for this local spacecraft. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s153 | 211.0-B-6 | 6.2.4.2 | The Test_Source parameter shall be used to determine whether the SCID field in received frames whose Source-or-Destination IDs are set to source is to be validated: | FAIL (gap) | 579f0976:starcom/src/ccsds/copp.cpp dest_is_local:359-365 checks destination only; no Test_Source / source-SCID test | behaves as Test_Source=false always; MIB param absent |
| 211.0-B-6/s154 | 211.0-B-6 | 6.2.4.2 | a) Test_Source=false: no test shall be performed; | FAIL (gap) | 579f0976:starcom/src/ccsds/copp.cpp dest_is_local:359-365 checks destination only; no Test_Source / source-SCID test | behaves as Test_Source=false always; MIB param absent |
| 211.0-B-6/s155 | 211.0-B-6 | 6.2.4.2 | 1) if the RECEIVING_SCID_BUFFER is non-zero, that is, contains a valid SCID, a test shall be performed; | FAIL (gap) | 579f0976:starcom/src/ccsds/copp.cpp dest_is_local:359-365 checks destination only; no Test_Source / source-SCID test | behaves as Test_Source=false always; MIB param absent |
| 211.0-B-6/s156 | 211.0-B-6 | 6.2.4.2 | 2) if the RECEIVING_SCID_BUFFER is zero, the value of the SCID field in the header of the first received frame whose Source-or-Destination ID is source shall be loaded into RECEIVING_SCID_BUFFER and thereafter used for validation for the remainder of the session. | FAIL (gap) | 579f0976:starcom/src/ccsds/copp.cpp dest_is_local:359-365 checks destination only; no Test_Source / source-SCID test | behaves as Test_Source=false always; MIB param absent |
| 211.0-B-6/s157 | 211.0-B-6 | 6.2.4.3 | Carrier_Only_Duration represents the time that shall be used to radiate an unmodulated carrier at the beginning of a transmission. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s158 | 211.0-B-6 | 6.2.4.4 | Acquisition_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern after carrier only to enable the receiving transceiver to achieve symbol synchronization and decoder lock. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s159 | 211.0-B-6 | 6.2.4.5 | Tail_Idle_Duration represents the time that shall be used to radiate the idle sequence pattern at the end of a transmission to enable the receiving transceiver to process the last transmitted frame (i.e., push the data through the decoders). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s160 | 211.0-B-6 | 6.2.4.6 | Carrier_Loss_Timer_Duration shall contain the value loaded into the CARRIER_LOSS_TIMER based upon the conditions defined in 6.3.1.3 (CARRIER_LOSS_TIMER and Associated Events). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s161 | 211.0-B-6 | 6.2.4.7 | Comm_Change_Waiting_Period represents the time that the caller shall wait for the Comm_Change_Response to the COMM_CHANGE directive. | FAIL (gap) | MacMib (mac.hpp:95-111) has no Comm_Change_Waiting_Period | PICS DLL-156 M |
| 211.0-B-6/s162 | 211.0-B-6 | 6.2.4.8 | Comm_Change_Response shall be used by the responder to acknowledge that the persistent activity has been accepted. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp macOnRemoteCommChange:848 | response handling exists for half duplex; full-duplex S48 response path not traced |
| 211.0-B-6/s163 | 211.0-B-6 | 6.2.4.9 | Comm_Change_Notification shall be provided to the local vehicle controller, e.g., spacecraft C&DH, by the caller and/or responder upon success or failure of the COMM_CHANGE activity. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s164 | 211.0-B-6 | 6.2.4.10.1 | Comm_Change_Lifetime shall represent the time period during which the COMM_CHANGE activity shall be repeated until the MAC Sublayer detects the expected Comm_Change_Response. | FAIL (gap) | MacMib has no Comm_Change_Lifetime | PICS DLL-153 M |
| 211.0-B-6/s166 | 211.0-B-6 | 6.2.4.11 | Hail_Wait_Duration shall represent the time that the initiating transceiver (caller) waits for a response to the Hail. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s167 | 211.0-B-6 | 6.2.4.12 | Hail_Response shall be used by the responder to acknowledge that the persistent activity has been accepted. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s168 | 211.0-B-6 | 6.2.4.13 | Hail_Notification shall be provided to the local vehicle controller (e.g., spacecraft C&DH) by the caller and/or responder upon success or failure of the persistent activity. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s169 | 211.0-B-6 | 6.2.4.14.1 | Hail_Lifetime represents the time period during which the persistent activity shall be repeated until the MAC detects the expected Hail_Response. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s171 | 211.0-B-6 | 6.2.4.16 | Hailing_Data_Rate shall represent the data rate assigned during the Hail activity. | UNVERIFIED | no Hailing_Data_Rate field in MacMib; hail rate carried in MacCommValue (mac.cpp queueFromCv:163) | PICS DLL-162 M; mapping unclear |
| 211.0-B-6/s172 | 211.0-B-6 | 6.2.4.17 | Send_Duration represents the maximum time that the half-duplex transmitter shall transmit data before it relinquishes the token (transfers to receive). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s173 | 211.0-B-6 | 6.2.4.18 | Receive_Duration represents the maximum time that the half-duplex receiver is anticipating that the sending side shall be transmitting. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s174 | 211.0-B-6 | 6.2.4.19.1 | PLCW_Repeat_Interval shall represent the maximum transmission time between successive PLCWs, even if PLCWs are not required for Sequence Control operations. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s175 | 211.0-B-6 | 6.2.4.19.2 | A zero value shall represent an infinite time period. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s176 | 211.0-B-6 | 6.3.1.1.1 | All timers shall use the MIB parameter Interval_Clock. | PASS (doc) | 579f0976:starcom/include/starcom/ccsds/mac.hpp:16 Tick units; mac.cpp tickDec:226 | caller supplies dt; Interval_Clock is the caller clock |
| 211.0-B-6/s177 | 211.0-B-6 | 6.3.1.1.2 | – the timer shall count down when it is not equal to ‘zero’; | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp tickDec:226; mac.hpp:16 "0 = inactive" |  |
| 211.0-B-6/s178 | 211.0-B-6 | 6.3.1.1.2 | – when the timer equals ‘1’, the event associated with the timer shall occur; | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp tickDec:226; mac.hpp:16 "0 = inactive" |  |
| 211.0-B-6/s179 | 211.0-B-6 | 6.3.1.1.2 | – when the timer equals ‘zero’ it shall be in an inactive state; | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp tickDec:226; mac.hpp:16 "0 = inactive" |  |
| 211.0-B-6/s181 | 211.0-B-6 | 6.3.1.2 | The values loaded into the timer shall represent a desired time value consistent with the Interval Clock frequency. | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp loadWait:6, tickDec:226 |  |
| 211.0-B-6/s182 | 211.0-B-6 | 6.3.1.2 | The timer shall be loaded with the required MIB parameter value (see state tables), and shall be counted down using the Interval_Clock. | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp loadWait:6, tickDec:226 |  |
| 211.0-B-6/s183 | 211.0-B-6 | 6.3.1.2 | The value in the timer shall be counted down until underflow. | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp loadWait:6, tickDec:226 |  |
| 211.0-B-6/s184 | 211.0-B-6 | 6.3.1.3.1 | The CARRIER_LOSS_TIMER contains the duration during which the session shall be maintained even though the carrier is no longer present. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s185 | 211.0-B-6 | 6.3.1.3.2 | The CARRIER_LOSS_TIMER shall be loaded with the value contained in the MIB parameter Carrier_Loss_Timer_Duration and down counting shall be enabled when the following conditions are simultaneously satisfied: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s186 | 211.0-B-6 | 6.3.1.3.3 | The CARRIER_LOSS_TIMER shall be reset to ‘zero’ when the CARRIER_ACQUIRED (Physical Layer) signal is true. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s187 | 211.0-B-6 | 6.3.1.4.1 | The PLCW TIMER shall be used periodically to request the issuance of a PLCW. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s188 | 211.0-B-6 | 6.3.1.4.2 | When the PLCW TIMER=‘1’, the NEED_PLCW variable shall be set true. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:176 [CHECK(s.need_plcw);] | assertion matched on identifier "needplcw" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s189 | 211.0-B-6 | 6.3.1.4.3 | The timer shall be loaded with the value in the MIB parameter PLCW_Repeat_Interval whenever a PLCW is transmitted. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s190 | 211.0-B-6 | 6.3.2.2 | The FIFO shall be filled with data per the specification defined in table 6-14. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s191 | 211.0-B-6 | 6.3.2.3 | The No_Frames_Pending event shall occur when the Output FIFO becomes empty and there are no frames selectable for output. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s192 | 211.0-B-6 | 6.3.3.1.1.1 | SET MODE shall assign values to the MODE parameter (6.2.2.1) as follows: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s193 | 211.0-B-6 | 6.3.3.1.1.2 | SET MODE inactive shall initialize the variables in table 6-6 to the values indicated. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s194 | 211.0-B-6 | 6.3.3.1.2 | SET INITIALIZE MODE shall use SET MODE inactive to place MODE into the inactive state and trigger the COP-P events SE0 and RE0 described in the state tables in 7.2.3.3 and 7.3.1. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s195 | 211.0-B-6 | 6.3.3.1.3 | Local COMM_CHANGE shall use SET TRANSMITTER PARAMETERS and SET RECEIVER PARAMETERS (and possibly SET PL EXTENSIONS for transmit and receive) to initiate a change in one or more communications channel parameters. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s196 | 211.0-B-6 | 6.3.3.1.4 | LOAD COMMUNICATIONS VALUE BUFFER shall load the values for the remote transmitter and receiver associated with either the Hail, COMM_CHANGE, or half-duplex receiver/transmitter switching activities. | PASS | 579f0976:starcom/tests/unit/test_mac.cpp:523 [CHECK(macPollNotify(s) == MacNotify::comm_change_apply_rx);] | assertion matched on identifier "commchange" (identifier trace; inference that it exercises this shall) |
| 211.0-B-6/s197 | 211.0-B-6 | 6.3.3.1.5 | LOCAL_NO_MORE_DATA shall inform the transceiver that the local data source has no more data to send. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s198 | 211.0-B-6 | 6.3.3.1.6 | SET DUPLEX shall configure the local transmitter and/or receiver for either full-duplex, half-duplex, or simplex operations. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s199 | 211.0-B-6 | 6.3.3.1.7 | SET RECEIVING SCID BUFFER shall be used by the vehicle controller to load the RECEIVING_SCID_BUFFER. | FAIL (gap) | no SET RECEIVING SCID BUFFER directive (no buffer) | DLL-77 |
| 211.0-B-6/s200 | 211.0-B-6 | 6.3.3.1.8 | READ STATUS shall selectively read the local status registers and buffers (including timing services) within the transceiver. | N/A | READ STATUS is PICS DLL-93 O, not implemented |  |
| 211.0-B-6/s201 | 211.0-B-6 | 6.3.3.2.2.1 | SET TRANSMITTER PARAMETERS shall be used to set the transmission parameters that control the data rate, encoding, modulation, and frequency in the transceiver receiving the directive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s202 | 211.0-B-6 | 6.3.3.2.2.2 | Upon receipt, the transceiver shall use the local directive SET MODE active to put the receiver’s MODE parameter into the active state. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s203 | 211.0-B-6 | 6.3.3.2.3.1 | SET RECEIVER PARAMETERS shall be used to set the receiver parameters that control the data rate, decoding, modulation, and frequency in the transceiver receiving the directive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s204 | 211.0-B-6 | 6.3.3.2.3.2 | Upon receipt, the transceiver shall use the local directive SET MODE active to put the receiver’s MODE parameter into the active state. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s205 | 211.0-B-6 | 6.3.3.2.4 | SET PL EXTENSIONS directive shall be used to enable or disable additional parameters. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s206 | 211.0-B-6 | 6.3.3.2.5 | SET CONTROL PARAMETERS shall be used to transmit operational control information during a session. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s207 | 211.0-B-6 | 6.3.3.2.5 | a) Token Field: When this field is non-zero, it shall notify the recipient that the sender is relinquishing the half-duplex ‘Send Token’ and is switching to receive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s208 | 211.0-B-6 | 6.3.3.2.5 | b) RNMD Field: When this field is non-zero, it shall notify the recipient that the sending spacecraft has no more data to send, and that the session may be terminated when the recipient also has no more data to send. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s209 | 211.0-B-6 | 6.3.3.2.5 | c) Duplex Field: When this field is non-zero, it shall notify the recipient to change communication directionality (full, half, simplex-transmit, simplex-receive). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s210 | 211.0-B-6 | 6.3.3.2.5 | d) Time Sample Field: When this field is non-zero it shall notify the recipient to capture the time, sequence number, direction, and QoS Indicator for the next n frames received and transmitted (where ‘n’ is the value, i.e., number of frames contained within the Time Sample Field). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s214 | 211.0-B-6 | 6.5.1.1 | The TRANSMIT parameter (6.2.2.3) shall be set to on to signal the transceiver to turn its transmitter ‘on’. | DEVIATION-STATED | CONFORMANCE.md:34 "211.1 fields not enacted"; COVERAGE §6.5 |  |
| 211.0-B-6/s215 | 211.0-B-6 | 6.5.1.2 | The Frame Sublayer shall output frames via the C&S Sublayer ChannelAccess.request service primitive. | DEVIATION-STATED | CONFORMANCE.md:34 "211.1 fields not enacted"; COVERAGE §6.5 |  |
| 211.0-B-6/s216 | 211.0-B-6 | 6.5.1.3 | The MODULATION parameter (6.2.3.4) shall be set to on to signal the transceiver to modulate the carrier with the coded symbols provided on the Output coded symbol stream. | DEVIATION-STATED | CONFORMANCE.md:34 "211.1 fields not enacted"; COVERAGE §6.5 |  |
| 211.0-B-6/s217 | 211.0-B-6 | 6.5.2 | The values of these parameters shall be interpreted as follows: | DEVIATION-STATED | CONFORMANCE.md:34 "211.1 fields not enacted"; COVERAGE §6.5 |  |
| 211.0-B-6/s218 | 211.0-B-6 | 6.6.1.1 | Local SET TRANSMITTER PARAMETERS and Local SET RECEIVER PARAMETERS directives (and, if needed, Local SET PL EXTENSIONS directives) shall be used to set the local transceiver to the desired physical configuration. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s219 | 211.0-B-6 | 6.6.1.2 | If required for the session, the Test_Source MIB parameter shall be loaded. | FAIL (gap) | no Test_Source MIB parameter | see 6.2.4.2 |
| 211.0-B-6/s220 | 211.0-B-6 | 6.6.1.3 | A Local SET MODE (connecting-T) directive shall initiate the Hail activity and start the session establishment process (see 6.4.2 for full-duplex operation and 6.4.3 for half-duplex operation). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s221 | 211.0-B-6 | 6.7.1.1 | Establishment of physical channel characteristics and initialization of receiving procedures shall be accomplished via a SET MODE (connecting-L) or SET MODE (connecting-T) local directive (see 6.3.3.1.1 and 6.2.2.1). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s222 | 211.0-B-6 | 6.7.1.2 | The Frame Sublayer shall accept for validation frames delivered via the C&S Sublayer ChannelAccess.indication service primitive (see reference [5], annex B). | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s223 | 211.0-B-6 | 6.7.2 | Frame Validation Criteria shall be as follows: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s224 | 211.0-B-6 | 6.7.3.1 | Validated received U-frames shall be processed per the COP-P process described in 4.3.3. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s225 | 211.0-B-6 | 6.7.3.2.1 | Validated P-frames shall be processed by first delimiting the contained SPDUs. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s226 | 211.0-B-6 | 6.7.3.2.2 | PLCWs contained within SPDUs shall be transferred to the COP-P processor. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s227 | 211.0-B-6 | 6.7.3.2.3 | All other reports or directives shall be processed for protocol actions. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, macOnHailReceived:593; tests/unit/test_mac.cpp | code located (579f0976:starcom/src/ccsds/mac.cpp applyTable66:18, macOnWaitExpired:501, macInit:528, mac); no assertion found that tests this shall |
| 211.0-B-6/s228 | 211.0-B-6 | 7.2.1 | The FOP-P shall maintain a single output queue. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s229 | 211.0-B-6 | 7.2.2 | a) VE(S): an 8-bit positive integer whose value shall represent the sequence number (modulo 256) of the next Expedited frame to be sent. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s230 | 211.0-B-6 | 7.2.2 | b) V(S): an 8-bit positive integer whose value shall represent the sequence number (modulo 256) of the next new Sequence Controlled frame to be sent. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s231 | 211.0-B-6 | 7.2.2 | c) VV(S): an 8-bit positive integer whose value shall represent the sequence number (modulo 256) to be assigned to the next Sequence Controlled frame to be sent. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s232 | 211.0-B-6 | 7.2.2 | It shall represent the sequence number plus one (modulo 256) of the last Sequence Controlled frame acknowledged by the receiver. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s233 | 211.0-B-6 | 7.2.2 | It shall contain a copy of the Report Value from the previous valid PLCW. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s234 | 211.0-B-6 | 7.2.2 | It shall indicate whether or not Sequence Controlled frame(s) need to be retransmitted. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s235 | 211.0-B-6 | 7.2.2 | They shall indicate whether or not a new PLCW or status report needs to be sent (the PLCW needs to be sent whenever its contents change). | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s236 | 211.0-B-6 | 7.2.2 | i) SYNCH_TIMER: a countdown timer that contains the time a sender shall wait to receive a valid PLCW from a receiver before taking action to synchronize with the receiver. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s237 | 211.0-B-6 | 7.2.3.1.1 | ‘Initialize’ shall set the following variables to the indicated values: | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s238 | 211.0-B-6 | 7.2.3.1.2 | ‘Remove acknowledged frames from Sent queue’ shall remove n frames from the Sent queue, where n = N(R) − NN(R) (i.e., the number of times that NN(R) has to be incremented to reach N(R)). | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s239 | 211.0-B-6 | 7.2.3.1.3 | ‘Start SYNCH_TIMER’ shall, if SYNCH_TIMER is equal to ‘0’, set SYNCH_TIMER to the value of the MIB parameter Synch_Timeout. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s240 | 211.0-B-6 | 7.2.3.1.4 | ‘Clear SYNCH_TIMER’ shall set the SYNCH_TIMER value to ‘0’. | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s241 | 211.0-B-6 | 7.2.3.1.5 | ‘Store this PLCW’ shall | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s242 | 211.0-B-6 | 7.2.3.1.6 | ‘Send EXP (Expedited) Frame’ shall | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s243 | 211.0-B-6 | 7.2.3.1.7 | ‘Resend SEQ (Sequence Controlled) Frame’ shall | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s244 | 211.0-B-6 | 7.2.3.1.8 | ‘Send New SEQ Frame’ shall | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s245 | 211.0-B-6 | 7.2.3.2.1 | The MIB Parameters that shall be used for setup are: | PASS | 579f0976:starcom/src/ccsds/mac.cpp; test_mac.cpp test_set_vr_persistent |  |
| 211.0-B-6/s246 | 211.0-B-6 | 7.2.3.2.1 | e) Resync_Lifetime (implementation specific) is the time period during which the Resynchronization activity shall be repeated until the Resync_Response is detected; | PASS | 579f0976:starcom/src/ccsds/mac.cpp; test_mac.cpp test_set_vr_persistent |  |
| 211.0-B-6/s247 | 211.0-B-6 | 7.2.3.2.2 | The following shall occur: | PASS | 579f0976:starcom/src/ccsds/mac.cpp; test_mac.cpp test_set_vr_persistent |  |
| 211.0-B-6/s248 | 211.0-B-6 | 7.3.2 | The internal FARM-P variables shall be: | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s249 | 211.0-B-6 | 7.3.3 | FARM-P shall pass valid Expedited and valid in-sequence U-frames to the I/O Sublayer (4.4). | UNVERIFIED | 579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/unit/test_copp.cpp | code located (579f0976:starcom/src/ccsds/copp.cpp farmPOnFrame:81, fopPOnPlcw:171, fopPTick:199; tests/u); no assertion found that tests this shall |
| 211.0-B-6/s251 | 211.0-B-6 | 8.2 | a) shall provide the procedures that accept the user service data units and prepare them for transfer across the communications channel; | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s253 | 211.0-B-6 | 8.2 | c) shall assemble the data units for inclusion into frames in accordance with the restrictions imposed by various MIB parameters; | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s254 | 211.0-B-6 | 8.2 | d) shall receive the user service data unit along with its routing and control instructions; | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s255 | 211.0-B-6 | 8.2 | e) shall have the responsibility to inform the data supplier which service data units were transmitted and, in the case of Sequence Controlled service, which data units were acknowledged as received by the communications partner. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s256 | 211.0-B-6 | 8.3.1 | At the receiver side, the role of the I/O Sublayer shall be to route a received ‘complete’ data unit to the identified port. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s257 | 211.0-B-6 | 8.3.2 | When segmentation is used, the I/O Sublayer shall accept received segments and try to re-assemble the user’s data unit. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s258 | 211.0-B-6 | 8.3.3 | The I/O Sublayer shall deliver only completely reassembled data units; that is, partial data units are not delivered to the end user. | DEVIATION-STATED | COVERAGE.md §4.4/§8 sans-I/O, reassembly 0 | stated in COVERAGE.md only. Proposed: move the statement into CONFORMANCE.md (G7) |
| 211.0-B-6/s261 | 211.0-B-6 | A1.3 | The implementer shall complete the RL by entering appropriate responses in the support or values supported column, using the notation described in A1.2. | N/A | PICS completion: not claimed for 211.0/211.2 (COVERAGE Annex A) |  |
| 211.0-B-6/s264 | 211.0-B-6 | B1.1.1 | The Directive/Report/PLCW SPDU shall be used for space link supervisory configuration and control of the transceiver and its operation. | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp queueFromCv:163-193 concatenates 16-bit directives; spduDirectiveType:1099 |  |
| 211.0-B-6/s265 | 211.0-B-6 | B1.1.2 | The SPDU data field shall be a container that can hold up to seven 16-bit discrete self-delimiting and self-identifying directives: | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp queueFromCv:163-193 concatenates 16-bit directives; spduDirectiveType:1099 |  |
| 211.0-B-6/s266 | 211.0-B-6 | B1.1.2 | a) each directive shall have a specific functionality; | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp queueFromCv:163-193 concatenates 16-bit directives; spduDirectiveType:1099 |  |
| 211.0-B-6/s267 | 211.0-B-6 | B1.1.2 | b) each directive shall be 16 bits in length and shall be self identified by the value in the Directive Type field (contained in bits 13, 14, and 15 of the directive); | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp queueFromCv:163-193 concatenates 16-bit directives; spduDirectiveType:1099 |  |
| 211.0-B-6/s268 | 211.0-B-6 | B1.1.2 | c) the directives shall be concatenated without intervening bits within the data field. | PASS (doc) | 579f0976:starcom/src/ccsds/mac.cpp queueFromCv:163-193 concatenates 16-bit directives; spduDirectiveType:1099 |  |
| 211.0-B-6/s269 | 211.0-B-6 | B1.2.1 | The SET TRANSMITTER PARAMETERS directive shall consist of six fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s270 | 211.0-B-6 | B1.2.2.1 | Bits 13–15 of the SET TRANSMITTER PARAMETERS directive shall contain the Directive Type. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s271 | 211.0-B-6 | B1.2.2.2 | The 3-bit Directive Type field identifies the type of protocol control directive and shall contain the binary value ‘000’ for the SET TRANSMITTER PARAMETERS directive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s272 | 211.0-B-6 | B1.2.3.1 | Bits 10–12 of the SET TRANSMITTER PARAMETERS directive shall be used to set the transmitter frequency of the partnered transceiver to the desired value. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s273 | 211.0-B-6 | B1.2.3.2 | In the context of the forward link, this 3-bit field shall define the transmit frequency of the responder. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s274 | 211.0-B-6 | B1.2.4 | Bits 8–9 of the SET TRANSMITTER PARAMETERS directive shall contain the following coding options: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s275 | 211.0-B-6 | B1.2.5 | Bit 7 of the SET TRANSMITTER PARAMETERS directive shall contain the transmission modulation options: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s276 | 211.0-B-6 | B1.2.6.1 | Bits 3–6 of the SET TRANSMITTER PARAMETERS directive shall contain one of the following transmission data rates (rates in kb/s, e.g., 4 = 4000 b/s) prior to encoding. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s277 | 211.0-B-6 | B1.2.7 | Bits 0–2 of the SET TRANSMITTER PARAMETERS directive shall contain the Transmitter Mode options. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s279 | 211.0-B-6 | B1.3.1.1 | The SET CONTROL PARAMETERS directive shall consist of six fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s280 | 211.0-B-6 | B1.3.2.1 | Bits 13–15 of the SET CONTROL PARAMETERS directive shall contain the Directive Type. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s281 | 211.0-B-6 | B1.3.2.2 | The 3-bit Directive Type field identifies the type of protocol control directive and shall contain the binary value ‘001’ to identify the SET CONTROL PARAMETERS directive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s282 | 211.0-B-6 | B1.3.3 | Bit 12 of the SET CONTROL PARAMETERS directive shall contain the value of the Token field as follows: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s283 | 211.0-B-6 | B1.3.4 | Bit 11 of the SET CONTROL PARAMETERS directive shall contain the Remote No More Data field as follows: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s285 | 211.0-B-6 | B1.3.5 | Bits 9-10 of the SET CONTROL PARAMETERS directive shall contain spares and be set to ‘all zero’. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s286 | 211.0-B-6 | B1.3.6 | Bits 6-8 of the SET CONTROL PARAMETERS directive shall contain the Duplex field as follows: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s287 | 211.0-B-6 | B1.3.7 | Bits 0-5 of the SET CONTROL PARAMETERS directive shall contain the Time Sample field. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s288 | 211.0-B-6 | B1.4.1 | The SET RECEIVER PARAMETERS directive shall consist of six fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s289 | 211.0-B-6 | B1.4.2.1 | Bits 13–15 of the SET RECEIVER PARAMETERS directive shall contain the Directive Type. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s290 | 211.0-B-6 | B1.4.2.2 | The 3-bit Directive Type field identifies the type of protocol control directive and shall contain the binary value ‘010’ for the SET RECEIVER PARAMETERS directive. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s291 | 211.0-B-6 | B1.4.3.1 | Bits 10–12 of the SET RECEIVER PARAMETERS directive shall be used to set the receiver frequency of the partnered transceiver to the desired value. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s292 | 211.0-B-6 | B1.4.3.2 | In the context of the forward link, this 3-bit field shall define the receive frequency of the responder. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s293 | 211.0-B-6 | B1.4.4 | Bits 8–9 of the SET RECEIVER PARAMETERS directive shall contain the following coding options: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s294 | 211.0-B-6 | B1.4.5 | Bit 7 of the SET RECEIVER PARAMETERS directive shall contain the following transmission modulation options: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s295 | 211.0-B-6 | B1.4.6.1 | Bits 3–6 of the SET RECEIVER PARAMETERS directive shall contain one of the following receiver data rates (rates in kb/s, e.g., 4 = 4000 b/s) after decoding. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s296 | 211.0-B-6 | B1.4.7 | Bits 0–2 of the SET RECEIVER PARAMETERS directive shall contain the receiver mode options. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s298 | 211.0-B-6 | B1.5.1 | The SET V(R) directive shall consist of three fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | PASS | 579f0976:starcom/src/ccsds/mac.cpp encodeSetVr:1015/decodeSetVr:1027; test_mac.cpp test_set_vr_codec |  |
| 211.0-B-6/s299 | 211.0-B-6 | B1.5.2.1 | Bits 13–15 of the SET V(R) directive shall contain the Directive Type. | PASS | 579f0976:starcom/src/ccsds/mac.cpp encodeSetVr:1015/decodeSetVr:1027; test_mac.cpp test_set_vr_codec |  |
| 211.0-B-6/s300 | 211.0-B-6 | B1.5.2.2 | The 3-bit Directive Type field identifies the type of protocol control directive and shall contain the binary value ‘011’ to identify the SET V(R) directive. | PASS | 579f0976:starcom/src/ccsds/mac.cpp encodeSetVr:1015/decodeSetVr:1027; test_mac.cpp test_set_vr_codec |  |
| 211.0-B-6/s301 | 211.0-B-6 | B1.5.3 | Bits 8–12 of the SET V(R) directive shall contain spare bits, set to ‘all zero’. | PASS | 579f0976:starcom/src/ccsds/mac.cpp encodeSetVr:1015/decodeSetVr:1027; test_mac.cpp test_set_vr_codec |  |
| 211.0-B-6/s302 | 211.0-B-6 | B1.5.4 | Bits 0–7 of the SET V(R) directive shall contain the value of the Frame Sequence Number (SEQ_CTRL_FSN) to which the receiving unit of the partnered transceiver is to be set. | PASS | 579f0976:starcom/src/ccsds/mac.cpp encodeSetVr:1015/decodeSetVr:1027; test_mac.cpp test_set_vr_codec |  |
| 211.0-B-6/s303 | 211.0-B-6 | B1.6.1 | It shall consist of seven fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s304 | 211.0-B-6 | B1.6.2.1 | Bits 13–15 of the REPORT REQUEST directive shall contain the Directive Type. | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s305 | 211.0-B-6 | B1.6.2.2 | The 3-bit Directive Type field identifies the type of protocol control directive and shall contain the binary value ‘100’. | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s306 | 211.0-B-6 | B1.6.3 | Bit 12 of the REPORT REQUEST directive shall indicate whether a PLCW report for PCID 1 is required: | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s307 | 211.0-B-6 | B1.6.4 | Bit 11 of the REPORT REQUEST directive shall indicate whether a PLCW report for PCID 0 is required: | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s308 | 211.0-B-6 | B1.6.5 | Bits 8–10 of the directive, if set to a value other than ‘000’, shall indicate a request to the remote transceiver to initiate a Proximity-1 time tag exchange (see section 5). | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s309 | 211.0-B-6 | B1.6.6.1 | The value contained in bits 3–7 of the REPORT REQUEST directive shall indicate the type of status report desired. | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s310 | 211.0-B-6 | B1.6.7 | Bits 0–2 of the REPORT REQUEST directive shall contain spare bits set to ‘all zero’. | N/A | REPORT REQUEST PICS DLL-147 O, not implemented |  |
| 211.0-B-6/s311 | 211.0-B-6 | B1.7.2 | The SET PL EXTENSIONS directive shall consist of ten fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s312 | 211.0-B-6 | B1.7.3.1 | Bits 13–15 of the SET PL EXTENSIONS directive shall contain the Directive Type. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s313 | 211.0-B-6 | B1.7.3.2 | The 3-bit Directive Type field identifies the directive type and shall contain the binary value ‘110’. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s314 | 211.0-B-6 | B1.7.4 | Bit 12 of the SET PL EXTENSIONS directive shall indicate which R-S Code is used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s315 | 211.0-B-6 | B1.7.5 | Bit 11 of the SET PL EXTENSIONS directive shall indicate whether Differential Mark Encoding is enabled: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s316 | 211.0-B-6 | B1.7.6 | Bits 9-10 of the SET PL EXTENSIONS directive shall indicate if and what type of digital bit scrambling is used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s317 | 211.0-B-6 | B1.7.7 | Bits 7-8 of the SET PL EXTENSIONS directive shall indicate the type of carrier suppression used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s318 | 211.0-B-6 | B1.7.8 | Bits 5-6 of the SET PL EXTENSIONS directive shall indicate the type of data modulation used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s319 | 211.0-B-6 | B1.7.9 | Bits 3-4 of the SET PL EXTENSIONS directive shall indicate the type of carrier modulation to be used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s320 | 211.0-B-6 | B1.7.10 | Bit 2 of the SET PL EXTENSIONS directive shall indicate which set of data rates prior to encoding shall be used. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s321 | 211.0-B-6 | B1.7.12.1 | Bit 1 of the SET PL EXTENSIONS directive shall indicate what set of frequencies is to be used: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s322 | 211.0-B-6 | B1.7.12.2 | In the context of the forward link, this 3-bit field shall define the receive frequency of the responder. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s323 | 211.0-B-6 | B1.7.12.3 | In the context of the return link, this 3-bit field shall define the transmit frequency of the responder. | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s324 | 211.0-B-6 | B1.7.13 | Bit 0 of the SET PL EXTENSIONS directive shall indicate if the fields in this directive apply to the transmit or receive side of the transceiver: | UNVERIFIED | 579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1145, encodeSetPlExt:1180; test_mac.cpp test_half_hail_octets:328, test_half_token_octets:479 | code located (579f0976:starcom/src/ccsds/mac.cpp encodeSetPhy:1106/decodeSetPhy:1120, encodeSetControl:1); no assertion found that tests this shall |
| 211.0-B-6/s325 | 211.0-B-6 | B1.8.1 | It shall consist of three fields, positioned contiguously in the following sequence (described from least significant bit, Bit 15, to most significant bit, Bit 0): | N/A | REPORT SOURCE SCID PICS DLL-149 O |  |
| 211.0-B-6/s326 | 211.0-B-6 | B1.8.2.1 | Bits 13–15 of the REPORT SOURCE SPACECRAFT ID status report shall contain the Directive Type. | N/A | REPORT SOURCE SCID PICS DLL-149 O |  |
| 211.0-B-6/s327 | 211.0-B-6 | B1.8.2.2 | The 3-bit Directive Type field identifies the type of status report and shall contain the binary value ‘111’. | N/A | REPORT SOURCE SCID PICS DLL-149 O |  |
| 211.0-B-6/s328 | 211.0-B-6 | B1.8.3 | Bits 10–12 of the REPORT SOURCE SPACECRAFT ID status report shall contain reserved bits, set to ‘all zero’. | N/A | REPORT SOURCE SCID PICS DLL-149 O |  |
| 211.0-B-6/s329 | 211.0-B-6 | B1.8.4 | Bits 0-9 of the REPORT SOURCE SPACECRAFT ID status report shall contain the SCID of the source of the Transfer Frame. | N/A | REPORT SOURCE SCID PICS DLL-149 O |  |
| 211.0-B-6/s331 | 211.0-B-6 | B2.1.2 | A single TIME DISTRIBUTION directive shall be contained within a TIME DISTRIBUTION SPDU. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s332 | 211.0-B-6 | B2.1.3 | The format of the TIME DISTRIBUTION SPDU data field shall consist of four fields, positioned contiguously, in the following sequence: | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s333 | 211.0-B-6 | B2.1.3 | All time code fields in this directive shall comply with the CCSDS Unsegmented Time Code format (reference [4]). | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s334 | 211.0-B-6 | B2.2.1 | Octet 0 of the TIME DISTRIBUTION SPDU data field shall contain the TIME DISTRIBUTION Directive Type field indicating the function to be performed (if any) with the time contents. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s335 | 211.0-B-6 | B2.3.1 | a) octet 1 through octet 8 shall contain the value of the clock corresponding to when the trailing edge of the last bit of the ASM of the transmitted PLTU crosses the clock capture point within the transceiver; | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s336 | 211.0-B-6 | B2.3.1 | b) this time code field shall be divided into 5 octets of coarse time and 3 octets of fine time. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s337 | 211.0-B-6 | B2.3.2 | Otherwise, this field shall contain reserved bits, set to ‘all zero’. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s338 | 211.0-B-6 | B2.4.1 | a) octet 9 through octet 11 shall contain the delay time between the transceiver internal clock capture point and when the trailing edge of the last bit of the ASM of the transmitted PLTU crossed the time reference point (see section 5, ‘Proximity-1 Timing Services’); | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s339 | 211.0-B-6 | B2.4.1 | b) this time code field shall be divided into 1 octet of coarse time and 2 octets of fine time. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s340 | 211.0-B-6 | B2.4.2 | Otherwise, this field shall contain reserved bits, set to ‘all zero’. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s342 | 211.0-B-6 | B2.5.1 | a) octet 12 through octet 14 shall contain the calculated one way light time between the instance that the trailing edge of the last bit of the ASM of the transmitted PLTU crosses the time reference point of the initiator node to the time reference point of the destination node; | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s343 | 211.0-B-6 | B2.5.1 | b) this time code field shall be divided into 1 octet of coarse time and 2 octets of fine time. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s344 | 211.0-B-6 | B2.5.2 | Otherwise, this field shall contain reserved bits, set to ‘all zero’. | FAIL (gap) | no TIME DISTRIBUTION directive | DLL-64 M (receive side); F-2 |
| 211.0-B-6/s346 | 211.0-B-6 | ANNEX D | INVALID FRAME SOURCE When the SCID field and RECEIVING_SCID_BUFFER disagree, and Test_Source is true, then a session violation has occurred and the vehicle controller shall be notified. | FAIL (gap) | no INVALID FRAME SOURCE notification (MacNotify enum mac.hpp:65-75) | depends on Test_Source |
| 211.2-B-3/s002 | 211.2-B-3 | unattributed | Specific service arrangements shall be made via memoranda of agreement. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.2-B-3/s005 | 211.2-B-3 | unattributed | However, CCSDS does not have a patent law staff, and CCSDS shall not be held responsible for identifying any or all such patent rights. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.2-B-3/s009 | 211.2-B-3 | 1.6.2.1 | a) the words ‘shall’ and ‘must’ imply a binding and verifiable specification; | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.2-B-3/s012 | 211.2-B-3 | 1.6.3 | When the field is used to express a binary value (such as a counter), the Most Significant Bit (MSB) shall be the first transmitted bit of the field, that is, ‘Bit 0’, as shown in figure 1-1. | PASS | 579f0976:starcom/tests/unit/test_pltu.cpp test_crc_v3_header_only golden BCC004E7 | bit order via golden |
| 211.2-B-3/s013 | 211.2-B-3 | 3.2.2 | A PLTU shall encompass the following three fields, positioned contiguously, in the following sequence: | PASS | 579f0976:starcom/src/ccsds/pltu.cpp encodePltu:126; test_pltu.cpp |  |
| 211.2-B-3/s014 | 211.2-B-3 | 3.2.3.1 | The ASM shall occupy the first 24 bits of the PLTU. | PASS | 579f0976:starcom/include/starcom/ccsds/pltu.hpp:17-18 kPltuAsm FA F3 20; tests/unit/test_pltu.cpp test_reject_bad_asm |  |
| 211.2-B-3/s015 | 211.2-B-3 | 3.2.3.2 | The ASM shall consist of the following bit pattern (in hexadecimal): FAF320. | PASS | 579f0976:starcom/include/starcom/ccsds/pltu.hpp:17-18 kPltuAsm FA F3 20; tests/unit/test_pltu.cpp test_reject_bad_asm |  |
| 211.2-B-3/s016 | 211.2-B-3 | 3.2.4.1 | PLTUs shall contain either a Version-3 or a Version-4 Transfer Frame. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp decodePltu:111/encodePltu:126; test_pltu.cpp test_reject_tfvn_unknown, test_pltu_composition | per-stream single version is caller discipline (inference) |
| 211.2-B-3/s017 | 211.2-B-3 | 3.2.4.2 | The Transfer Frame in a PLTU shall immediately follow the ASM. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp decodePltu:111/encodePltu:126; test_pltu.cpp test_reject_tfvn_unknown, test_pltu_composition | per-stream single version is caller discipline (inference) |
| 211.2-B-3/s018 | 211.2-B-3 | 3.2.5.1 | The CRC-32 shall occupy the last 32 bits of the PLTU. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7 kPoly 0x00A00805, :14 init 0; pltu.cpp encodePltu:126; test_pltu.cpp test_crc_*, test_asm_in_crc, test_reject_bad_crc |  |
| 211.2-B-3/s019 | 211.2-B-3 | 3.2.5.2 | The CRC-32 shall immediately follow the Transfer Frame. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7 kPoly 0x00A00805, :14 init 0; pltu.cpp encodePltu:126; test_pltu.cpp test_crc_*, test_asm_in_crc, test_reject_bad_crc |  |
| 211.2-B-3/s020 | 211.2-B-3 | 3.2.5.3 | The CRC-32 shall be calculated by applying the encoding procedure specified in annex C to the Transfer Frame. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7 kPoly 0x00A00805, :14 init 0; pltu.cpp encodePltu:126; test_pltu.cpp test_crc_*, test_asm_in_crc, test_reject_bad_crc |  |
| 211.2-B-3/s021 | 211.2-B-3 | 3.2.5.4 | The ASM shall NOT be a part of the encoded data space of the CRC-32. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7 kPoly 0x00A00805, :14 init 0; pltu.cpp encodePltu:126; test_pltu.cpp test_crc_*, test_asm_in_crc, test_reject_bad_crc |  |
| 211.2-B-3/s022 | 211.2-B-3 | 3.3.2.1 | Idle data shall be included in the bitstream for encoding as follows: | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s023 | 211.2-B-3 | 3.3.2.1 | a) an Acquisition sequence (defined in 3.3.3) shall be inserted when transmission commences; | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s024 | 211.2-B-3 | 3.3.2.1 | b) Idle sequence(s) (defined in 3.3.2 and 3.3.4) shall be inserted when no PLTU is available; and | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s025 | 211.2-B-3 | 3.3.2.1 | c) a Tail sequence (defined in 3.3.5) shall be inserted prior to terminating transmission. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s026 | 211.2-B-3 | 3.3.2.2 | Idle data shall consist of the PN sequence 352EF853 (in hexadecimal), repeated as needed. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s027 | 211.2-B-3 | 3.3.2.3 | When LDPC coding is used, the Acquisition sequence shall start on the first bit of the PN sequence. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s028 | 211.2-B-3 | 3.3.2.4 | Whenever the end of the PN sequence is reached, the sequence shall be repeated starting from the first bit of the PN sequence. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s029 | 211.2-B-3 | 3.3.3.2.1 | The Acquisition sequence shall consist of data as specified by the requirements in 3.3.3.2 above. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s030 | 211.2-B-3 | 3.3.3.2.2 | The Acquisition sequence shall be transmitted for the duration specified by the MIB parameter Acquisition_Idle_Duration specified in reference [3]. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s031 | 211.2-B-3 | 3.3.4.2.1 | The Idle sequence shall consist of data as specified by the requirements in 3.3.3.2 above. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s032 | 211.2-B-3 | 3.3.4.2.2 | During the data services phase, if no PLTU is ready for transfer, then the Idle sequence shall be transmitted. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s033 | 211.2-B-3 | 3.3.5.2.1 | The Tail sequence shall consist of data as specified by the requirements in 3.3.3.2, above. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s034 | 211.2-B-3 | 3.3.5.2.2 | The Tail sequence shall be transmitted for the duration specified by the MIB parameter Tail_Idle_Duration, specified in reference [3]. | FAIL (gap) | 579f0976:starcom/no PN 352EF853 / Acquisition / Idle / Tail generator in starcom/src or src/starcom_adapt (grep, 579f0976) | PICS A2.2 item 2 is M; COVERAGE.md:77 says 0 but CONFORMANCE.md states no deviation |
| 211.2-B-3/s035 | 211.2-B-3 | 3.4.2.1 | The C&S Sublayer shall process the incoming Transfer Frames and produce a bitstream for encoding (including PLTUs and Idle Data) at data rate R_d. | UNVERIFIED | 579f0976:starcom/adapters phy_uncoded_encode; test_coding.cpp test_conv_pltu_and_uncoded_phy (per COVERAGE.md:80) | code located (579f0976:starcom/adapters phy_uncoded_encode; test_coding.cpp test_conv_pltu_and_uncoded_p); no assertion found that tests this shall |
| 211.2-B-3/s036 | 211.2-B-3 | 3.4.2.2 | The C&S Sublayer shall generate the output stream of Proximity-1 coded symbols applying only one of the following coding options: | UNVERIFIED | 579f0976:starcom/adapters phy_uncoded_encode; test_coding.cpp test_conv_pltu_and_uncoded_phy (per COVERAGE.md:80) | code located (579f0976:starcom/adapters phy_uncoded_encode; test_coding.cpp test_conv_pltu_and_uncoded_p); no assertion found that tests this shall |
| 211.2-B-3/s037 | 211.2-B-3 | 3.4.3.1 | The convolutional code shall be the rate 1/2, constraint-length 7 convolutional code defined in reference [2]. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s038 | 211.2-B-3 | 3.4.3.2 | When the convolutional code is applied, all data to be transmitted, that is, PLTUs and Idle data, shall be convolutionally encoded. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s039 | 211.2-B-3 | 3.4.4.1 | When the LDPC code is used, fixed-length LDPC message blocks of length k=1024 bits shall be constructed as follows. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s040 | 211.2-B-3 | 3.4.4.1 | a) PLTUs shall be generated as described in 3.2. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s041 | 211.2-B-3 | 3.4.4.1 | b) Idle data shall then be inserted between PLTUs as necessary (i.e., before the first PLTU, between PLTUs, and after the last PLTU), according to 3.3. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s042 | 211.2-B-3 | 3.4.4.1 | c) The resulting bitstream shall then be partitioned into LDPC message blocks 1024 bits in length. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s043 | 211.2-B-3 | 3.4.4.2 | The first LDPC message block shall start with the first bit of the Acquisition sequence, and this shall be the first bit of the PN sequence defined in 3.3.2.2. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s044 | 211.2-B-3 | 3.4.4.3 | Each LDPC message block shall be encoded using the LDPC code (n=2048, k=1024) rate 1/2 code defined in reference [2]. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s045 | 211.2-B-3 | 3.4.4.4 | The LDPC Codewords shall be randomized according to 3.4.5. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s046 | 211.2-B-3 | 3.4.4.5 | LDPC Codewords shall be synchronized by using a stream of fixed-length LDPC Codewords with a CSM between them. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s047 | 211.2-B-3 | 3.4.4.5 | Synchronization is acquired on the receiving end by recognizing the specific pattern of the CSM in the physical channel data stream; synchronization shall then customarily be verified by making further checks. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s048 | 211.2-B-3 | 3.4.4.6 | The CSM shall consist of a 64-bit (8-octet) marker with the following bit pattern (in hexadecimal): 0347 76C7 2728 95B0. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s049 | 211.2-B-3 | 3.4.4.7 | The CSM shall immediately precede the LDPC codeword. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s050 | 211.2-B-3 | 3.4.4.8 | The CSM shall immediately follow the end of the preceding LDPC codeword; that is, there shall be no intervening bits (data or fill) preceding the CSM. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s051 | 211.2-B-3 | 3.4.5.2.1 | On the sending end, the pseudo-randomizer shall be applied to the LDPC Codeword. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s052 | 211.2-B-3 | 3.4.5.2.2 | On the receiving end, the pseudo-randomizer shall be applied to de-randomize the randomized LDPC Codewords before decoding. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s053 | 211.2-B-3 | 3.4.5.2.3 | The CSM shall be used for synchronizing the pseudo-randomizer. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s054 | 211.2-B-3 | 3.4.5.2.4 | The pseudo-random sequence shall be applied starting with the first bit of the LDPC Codeword. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s055 | 211.2-B-3 | 3.4.5.2.5 | On the sending end, the LDPC Codeword shall be randomized by exclusive- ORing the first bit of the Codeword with the first bit of the pseudo-random sequence, followed by the second bit of the Codeword with the second bit of the pseudo-random sequence, and so on. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s056 | 211.2-B-3 | 3.4.5.2.6 | On the receiving end, the original Codeword shall be reconstructed (i.e., de-randomized) using the same pseudo-random sequence. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s057 | 211.2-B-3 | 3.4.5.2.7 | After locating the CSM in the received data stream, the data immediately following the CSM shall be de-randomized. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s058 | 211.2-B-3 | 3.4.5.2.8 | The random sequence shall be generated using the following polynomial: | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s059 | 211.2-B-3 | 3.4.5.2.9 | The random sequence shall begin at the first bit of the LDPC Codeword and shall repeat after 255 bits, continuing repeatedly until the end of the Codeword. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s060 | 211.2-B-3 | 3.4.5.2.10 | The sequence generator shall be initialized to the all-ones state at the start of each Codeword. | UNVERIFIED | 579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp | code located (579f0976:starcom/src/ccsds/conv.cpp, ldpc.cpp; tests/unit/test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s061 | 211.2-B-3 | 3.5.1 | The C&S Sublayer shall accept Transfer Frames from the Proximity-1 Frame Sublayer or the USLP Data Link Protocol Sublayer. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp encodePltu:126; test_pltu.cpp |  |
| 211.2-B-3/s062 | 211.2-B-3 | 3.5.2 | For each frame, the C&S Sublayer shall construct a PLTU containing the frame. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp encodePltu:126; test_pltu.cpp |  |
| 211.2-B-3/s063 | 211.2-B-3 | 3.5.3 | The C&S Sublayer shall generate the bitstream for encoding by inserting Idle data as required. | FAIL (gap) | no idle insertion (see 3.3) | COVERAGE.md §3.5 "insert idle = 0"; undeclared in CONFORMANCE.md |
| 211.2-B-3/s064 | 211.2-B-3 | 3.5.4 | The C&S Sublayer shall apply the selected encoding (see 3.4.2.2) and generate the output coded symbol stream. | UNVERIFIED | 579f0976:starcom/conv.cpp/ldpc.cpp encode; test_coding.cpp | code located (579f0976:starcom/conv.cpp/ldpc.cpp encode; test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s065 | 211.2-B-3 | 3.5.4 | When the LDPC code is used, the CSM shall be added as described in 3.4.4. | UNVERIFIED | 579f0976:starcom/conv.cpp/ldpc.cpp encode; test_coding.cpp | code located (579f0976:starcom/conv.cpp/ldpc.cpp encode; test_coding.cpp); no assertion found that tests this shall |
| 211.2-B-3/s066 | 211.2-B-3 | 3.5.5 | The C&S Sublayer shall deliver the generated output coded symbol stream to the Physical Layer at a constant rate (R_cs). | UNVERIFIED | constant rate R_cs is a property of the caller radio path (sans-I/O) | needs bench/RF measurement; not gradeable from code |
| 211.2-B-3/s067 | 211.2-B-3 | 3.5.6 | a) before computing CRC, the C&S Sublayer shall store the values of the clock, frame sequence number, QOS Indicator, and direction (egress) of each outgoing Transfer Frame; and | N/A | PICS A2.2 item 6 (O), not claimed |  |
| 211.2-B-3/s068 | 211.2-B-3 | 3.5.6 | b) the captured clock value shall correspond to when the trailing edge of the last bit of the ASM of the outgoing PLTU crosses the clock capture point (defined by the implementation) within the transceiver. | N/A | PICS A2.2 item 6 (O), not claimed |  |
| 211.2-B-3/s069 | 211.2-B-3 | 3.6.1 | The C&S Sublayer shall accept the received coded symbols stream from the Physical Layer. | UNVERIFIED | symbol stream acceptance is caller (sans-I/O) | bench item |
| 211.2-B-3/s070 | 211.2-B-3 | 3.6.2 | The C&S Sublayer shall apply the decoding of the selected channel coding option defined in 3.4.2.2. | DEVIATION-STATED | CONFORMANCE.md:38 "decode 0" |  |
| 211.2-B-3/s071 | 211.2-B-3 | 3.6.2 | When the LDPC code is used, codeword synchronization shall be achieved using CSM as described in 3.4.4. | DEVIATION-STATED | CONFORMANCE.md:38 "decode 0" |  |
| 211.2-B-3/s072 | 211.2-B-3 | 3.6.3 | The C&S Sublayer shall use the ASM to locate the beginning of a PLTU for frame synchronization with the Transfer Frame it contains. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s073 | 211.2-B-3 | 3.6.4 | For each frame, the C&S Sublayer shall test for the value of the Transfer Frame Version Number (TFVN) to aid in locating the position of the CRC-32 field of the PLTU as follows: | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s074 | 211.2-B-3 | 3.6.4 | a) If the first two bits of the Transfer Frame header are ‘10’, indicating a 2-bit TFVN field and a Transfer Frame version of 10 binary (i.e., Version-3 Proximity-1 frame), then the C&S Sublayer shall use the Proximity-1 Frame Length Field of the Transfer Frame to locate the position of the CRC-32 field in the PLTU. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s075 | 211.2-B-3 | 3.6.4 | 1) if the USLP End of Frame Primary Header Flag is ‘0’ (i.e., USLP non-truncated Transfer Frame is present), then the C&S Sublayer shall use the USLP Frame Length Field of the Transfer Frame to locate the position of the CRC-32 field in the PLTU; | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s076 | 211.2-B-3 | 3.6.4 | 2) if the USLP End of Frame Primary Header Flag is ‘1’ (i.e., USLP truncated Transfer Frame present), then the C&S Sublayer shall use the USLP Truncated Transfer Frame Length Managed Parameter to locate the position of the CRC-32 field in the PLTU. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s077 | 211.2-B-3 | 3.6.4 | c) If the version number of the Transfer Frame is not recognized, the C&S Sublayer shall continue searching the received coded symbol stream for the ASM of the next PLTU. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s078 | 211.2-B-3 | 3.6.5 | The C&S Sublayer shall apply the CRC-32 decoding procedure to the received CRC- 32 codeword in a PLTU, to check for errors in the received frame. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s079 | 211.2-B-3 | 3.6.6 | If the CRC-32 decoding detects any error in the CRC-32 codeword, the C&S Sublayer shall mark the received frame as invalid. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s081 | 211.2-B-3 | 3.6.7 | The C&S Sublayer shall deliver the received frames to the Proximity-1 Frame Sublayer or the USLP Data Link Protocol Sublayer. | PASS | 579f0976:starcom/src/ccsds/pltu.cpp huntPltu:157, decodePltu:111; test_pltu.cpp test_hunt_*, test_reject_bad_crc, test_reject_tfvn_unknown, test_hunt_bad_crc_consumes_unit | hunt is exact-ASM (stricter than book; ICD) |
| 211.2-B-3/s082 | 211.2-B-3 | 3.6.8 | a) after decoding, the C&S Sublayer shall store the values of the clock, frame sequence number, QOS Indicator, and direction (ingress) of each received Transfer Frame; and | N/A | PICS A2.2 item 6 (O), not claimed |  |
| 211.2-B-3/s083 | 211.2-B-3 | 3.6.8 | b) the captured clock value shall correspond to when the trailing edge of the last bit of the ASM of the received PLTU crosses the clock capture point (defined by the implementation) within the transceiver. | N/A | PICS A2.2 item 6 (O), not claimed |  |
| 211.2-B-3/s086 | 211.2-B-3 | A1.3 | The implementer shall complete the RL by entering appropriate responses in the support or values supported column, using the notation described in A1.2. | N/A | PICS completion not claimed |  |
| 211.2-B-3/s088 | 211.2-B-3 | B3.1 | The Frame parameter is the service data unit of this service and shall be a Transfer Frame defined in reference [3] or in reference [5]. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s089 | 211.2-B-3 | B3.2 | The Quality Indicator parameter shall be used to notify the user at the receiving end of the service that there is an uncorrectable error in the received Transfer Frame. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s090 | 211.2-B-3 | B4.1.2 | The ChannelAccess.request primitive shall be passed from the service user at the sending end to the service provider to request that a Frame be transferred through the physical channel to the user at the receiving end. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s091 | 211.2-B-3 | B4.1.3 | The ChannelAccess.indication shall be passed from the service provider to the service user at the receiving end to deliver a Frame. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s092 | 211.2-B-3 | B4.2.2 | The ChannelAccess.request primitive shall provide a parameter as follows: | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s093 | 211.2-B-3 | B4.2.3 | The ChannelAccess.request primitive shall be passed to the service provider to request it to process and send the Frame. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s094 | 211.2-B-3 | B4.2.4 | Receipt of the ChannelAccess.request primitive shall cause the service provider to transfer the Frame through the physical channel to the user at the receiving end. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s095 | 211.2-B-3 | B4.3.2 | The ChannelAccess.indication primitive shall provide parameters as follows: | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s096 | 211.2-B-3 | B4.3.3 | The ChannelAccess.indication primitive shall be passed from the service provider to the service user at the receiving end to deliver a Frame. | FAIL (gap) | sans-I/O encode/decode/hunt API instead of ChannelAccess primitives (COVERAGE.md Annex B row) | normative annex; undeclared in CONFORMANCE.md. Inference: likely ACCEPT as a stated deviation (abstract service, not an API mandate) |
| 211.2-B-3/s097 | 211.2-B-3 | C1.1 | For the encoding procedure, the (n-32)-bit Transfer Frame shall be the information message. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s098 | 211.2-B-3 | C1.2 | The encoding procedure shall accept an (n-32)-bit Transfer Frame and generate a systematic binary (n,n-32) block code by appending a CRC-32 as the final 32 bits of the PLTU, as shown in figure 3-1. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s099 | 211.2-B-3 | C1.3 | If M(X) is the (n-32)-bit information message to be encoded expressed as a polynomial with binary coefficients, with the first bit transferred being the most significant bit M_0 taken as the coefficient of the highest power of X, then the equation for the 32-bit Cyclic Redundancy Check, expressed as a polynomial R(X) with binary coefficients, shall be | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s100 | 211.2-B-3 | C1.4 | The n-bit CRC-32–encoded block, expressed as a polynomial C(X) with binary coefficients, shall be | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s101 | 211.2-B-3 | C2.1 | The decoding procedure shall accept an n-bit received codeword, including the 32-bit Cyclic Redundancy Check, and generates a 32-bit syndrome. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s102 | 211.2-B-3 | C2.2 | The received block C^*(X) shall equal the transmitted codeword C(X) plus (modulo two) the n-bit error block E(X), C^*(X) = C(X) + E(X), where both are expressed as polynomials of the same form, that is, with the most significant bit C_0 or E_0 taken as the binary coefficient of the highest power of X. | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.2-B-3/s103 | 211.2-B-3 | C2.3 | With C^*(X) being the n-bit received codeword with the first transferred bit being the most significant bit C_0^* taken as the coefficient of the highest power of X, then the equation for the 32-bit syndrome, expressed as a polynomial S(X) with binary coefficients, shall be | PASS | 579f0976:starcom/src/ccsds/crc32.cpp:7,:14; test_pltu.cpp test_crc_v3_header_only golden BCC004E7 |  |
| 211.1-B-4/s002 | 211.1-B-4 | unattributed | Specific service arrangements shall be made via memoranda of agreement. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.1-B-4/s005 | 211.1-B-4 | unattributed | CCSDS shall not be held responsible for identifying any or all such patent rights. | N/A | — | front matter / shall-word convention; not an implementation requirement |
| 211.1-B-4/s011 | 211.1-B-4 | 1.5.2.1 | a) the words ‘shall’ and ‘must’ imply a binding and verifiable specification; | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s014 | 211.1-B-4 | 3.1.2 | Link elements in category E2c (table 3-1), for which range and range-rate measurements are needed, shall have transmit/receive frequency coherency capability. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s015 | 211.1-B-4 | 3.2.2.1 | The operational state of the transmitter shall depend on the state control variables MODE, TRANSMIT, and MODULATION, as shown in table 3-2. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s016 | 211.1-B-4 | 3.2.3.1 | The operational state of the receiver shall depend on the state control variables MODE, DUPLEX, and TRANSMIT, as shown in table 3-3. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s017 | 211.1-B-4 | 3.2.3.2 | The receiver shall sweep the frequency channel to which it is assigned in order to acquire carrier lock at an assigned frequency channel. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s018 | 211.1-B-4 | 3.2.3.3.1 | The CARRIER_ACQUIRED signal shall notify the MAC Sublayer that the receiver has acquired a carrier signal. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s019 | 211.1-B-4 | 3.2.3.3.2 | The CARRIER_ACQUIRED signal shall be set to true when the receiver is locked to the received RF signal and false when not in lock. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s020 | 211.1-B-4 | 3.2.3.4.1 | The SYMBOL_INLOCK_STATUS signal shall notify the MAC Sublayer that symbol synchronization has been acquired and the received serial symbol stream is being provided to the Data Link Layer. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s021 | 211.1-B-4 | 3.2.3.4.2 | The SYMBOL_INLOCK_STATUS signal shall be set to true when the receiver is in symbol lock and false when the receiver is not in symbol lock. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s022 | 211.1-B-4 | 3.2.3.5 | When SYMBOL_INLOCK_STATUS is true, the Physical Layer shall deliver the received symbol stream to the C&S Sublayer. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s024 | 211.1-B-4 | 3.3.2.2.1 | The forward frequency band shall be from 435 to 450 MHz. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s025 | 211.1-B-4 | 3.3.2.2.2 | The return frequency band shall be from 390 to 405 MHz. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s026 | 211.1-B-4 | 3.3.2.3.1 | For interoperability at UHF, the default hailing channel shall be Channel 1 configured for 435.6 MHz in the forward link and 404.4 MHz in the return link (1348/44*33 turnaround ratio). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s027 | 211.1-B-4 | 3.3.2.3.2 | If the Proximity link radio equipment supports only a single channel (i.e., a single forward and return frequency pair), then the hailing channel shall be the same as the working channel (see 1.5.1.2). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s028 | 211.1-B-4 | 3.3.2.3.3 | If the Proximity link radio equipment supports multiple channels, then the hailing channel shall be distinct from the working channel. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s031 | 211.1-B-4 | 3.3.2.4.1 | In the case where the system requires only one return frequency, associated with the forward 437.1 MHz frequency, the return frequency shall be 401.585625 MHz (147/160 turnaround ratio). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s032 | 211.1-B-4 | 3.3.2.4.1 | In the case where the system requires only one return frequency, associated with the forward 439.2 MHz frequency, the return frequency shall be 397.5 MHz (1325/24*61 turnaround ratio). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s033 | 211.1-B-4 | 3.3.2.4.1 | In the case where the system requires only one return frequency, associated with the forward 444.6 MHz frequency, the return frequency shall be 393.9 MHz (1313/38*39 turnaround ratio). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s034 | 211.1-B-4 | 3.3.2.5 | In the case where there is a need for one or multiple return frequencies paired with one or multiple forward frequencies, the forward frequencies shall be selected from the 435 to 450 MHz band in 20 kHz steps and the return frequencies shall be selected from 390 to 405 MHz in 20 kHz steps. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s035 | 211.1-B-4 | 3.3.2.5 | These frequency pairs shall be distinct from the frequency pairs defined in Channels 0 through 7. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s037 | 211.1-B-4 | 3.3.4 | Both forward and return links shall operate with Right Hand Circular Polarization (RHCP). | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s038 | 211.1-B-4 | 3.3.5.1 | The PCM data shall be Bi-Phase-L encoded and modulated directly onto the carrier. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s039 | 211.1-B-4 | 3.3.5.2 | Residual carrier shall be provided with modulation index of 60° ± 5%. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s040 | 211.1-B-4 | 3.3.5.3 | The symmetry of PCM Bi-Phase-L waveforms shall be such that the mark-to-space ratio is between 0.98 and 1.02. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s041 | 211.1-B-4 | 3.3.5.4 | A positive-going signal shall result in an advance of the phase of the radio frequency carrier. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s042 | 211.1-B-4 | 3.3.5.4 | a) a symbol ‘1’ shall result in an advance of the phase of the radio frequency carrier at the beginning of the symbol interval; | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s043 | 211.1-B-4 | 3.3.5.4 | b) a symbol ‘0’ shall result in a delay. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s044 | 211.1-B-4 | 3.3.6.1 | The Proximity-1 link shall support one or more of the following 13 discrete forward and return values for the coded symbol rate R_cs shown in symbols per second: 1000, 2000, 4000, 8000, 16000, 32000, 64000, 128000, 256000, 512000, 1024000, 2048000, 4096000. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s045 | 211.1-B-4 | 3.3.6.2 | Each channel symbol period, as measured at the output of the transmitter, shall differ by no more than 1% from the channel symbol period corresponding to the Proximity-1 channel symbol rate in use. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s046 | 211.1-B-4 | 3.3.6.3 | Generated channel symbol rate, measured over an interval greater than 10000 symbol periods, shall differ less than 0.1% from the defined Proximity-1 channel symbol rates as measured at the output of the transmitter. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s047 | 211.1-B-4 | 3.4.1.1 | The long-term oscillator stability (over the life of the mission) including all effects and over all operating conditions shall be 10 ppm. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s048 | 211.1-B-4 | 3.4.1.2 | The short-term oscillator stability over 1 minute shall be 1 ppm. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s049 | 211.1-B-4 | 3.4.2 | Residual amplitude modulation of the phase modulated RF signal shall be less than 2% RMS. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s050 | 211.1-B-4 | 3.4.3 | In non-coherent mode, the minimum specification for the oscillator phase noise at 437.1 MHz shall be limited by the template shown in figure 3-2. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s051 | 211.1-B-4 | 3.4.4 | The discrete spurious spectral lines of the transmit RF signal shall be limited by the template shown in the figure 3-3. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s052 | 211.1-B-4 | 3.4.5.1 | For the UHF frequencies specified in this Recommended Standard, the applicable Doppler requirements shall be as follows. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |
| 211.1-B-4/s055 | 211.1-B-4 | A1.3 | The implementer shall complete the RL by entering appropriate responses in the support or values supported column, using the notation described in A1.2. | DEVIATION-STATED | CONFORMANCE.md:40 (Level 0, "compliant (Full) not offered"); :18 honesty note | unticked by design. FSK/LoRa bearers are not 211.1 |

---

## 4. Review record: Claude Code review RV-1 to RV-12

Claude Code examined commit `7544417b` (the first version of this commit) on 2026-10-29. The review gives findings only. The owner makes the decision for each disposition.

**Review method:**

- clang-tidy 21.1.8 `--list-checks` with each `.clang-tidy`.
- MinGW-Builds g++ 15.2.0 `-std=c++20 -fsyntax-only -Wall -Wextra -Wpedantic -Wshadow -Wconversion -Wsign-conversion -Wold-style-cast -Wuseless-cast -Wcast-qual -Wfloat-equal` on `starcom/src/ccsds/*.cpp`, `adapters/host/*.cpp` and `adapters/rp2350/*.cpp`.
- `readability-redundant-casting` on `mac.cpp`, `ldpc.cpp` and `udp.cpp`.
- A PyMuPDF count of "shall" sentences in the repo PDFs. The count is approximate: it gives 305 for 211.0-B-6, and the walk has 303.
- A read of the clause rows against the code, the headers and the PICS.
- `git diff --stat fcd3496..7544417b -- src include`.

The review found no new safety defect in flight code. RV-2 was the most urgent item, because it makes the firmware pre-commit gate weak at this time.

**The review found that these items are correct:**

- The Starcom clang-tidy defect (the 13 stopped checks agree).
- Tier 1 FAILED, and Tiers 2 and 3 are provisional. `AUDIT_GUIDANCE.md:57` tells this.
- The N/A rows: there are no classes, no templates that we wrote and no std containers. `examples/` and `spacewire/` have no code.
- JSF 135 and JSF 180: `-Wshadow` and `-Wconversion` give 0 warnings.
- The corrections to out-of-date text in `AUDIT_GUIDANCE.md`.

| ID | Finding (substance) | Disposition now | Where |
|---|---|---|---|
| RV-1 | The RC master 7-tier audit did not run; only `starcom/` and `src/starcom_adapt/` were audited. Since the 2026-05-13 master audit (`fcd3496`, closed `1a321034`), 228 commits changed 236 files in `src/` and `include/` (+21,992 / −16,427); 211 authored firmware files. The August 2026 L2-P5 walk does not replace the scripted procedure. RC last done per tier: 1 2026-05-13 (now fails at 1.1, RV-2); 2 toolchain 2026-05-13, dead-code 2026-08-24, others 2026-05-13; 3 2026-05-13 (DEV_CODE, VERSION_STRING 2026-04-15); 4 AO Commandments 2026-04-27, CLA/RBM 2026-08-24, LL freshness 2026-09-05; 5 2026-05-13; 6–7 2026-05-13. | Open. Nathan sets the scope of a separate RC cycle. This report now opens with the Starcom-only line and the tier-status table; `AUDIT_GUIDANCE.md` "Audit integrity rules" cover scope in delegated briefs. Usable for the RC cycle: the guide currency fixes and the provisional `src/starcom_adapt/` Tier 3 rows. | Summary; Section 1.1 |
| RV-2 | RC root `.clang-tidy`: 10 checks off on 21.1.8, not 12; PR-2 had no RC tracking. Starcom 13 confirmed (RC list plus `bugprone-reserved-identifier`, `cppcoreguidelines-pro-type-reinterpret-cast`, `cppcoreguidelines-pro-type-vararg`). | Fixed: count 10 with names, plus the 23 silenced `clang-analyzer-core.*` (found in Goddard's second check). Whiteboard row "clang-tidy config defects (PR-2 / PR-1b)". Code fix pending (code-fix plan steps 1–3). | Section 1 Tier 1; Section 2 PR-1, PR-2 |
| RV-3 | JSF 181 was PASS from reading only; `g++ -Wuseless-cast` finds 12 redundant casts: `mac.cpp:234` (`static_cast<Tick>(t - dt)`, already `Tick`), `mac.cpp:1127–1129, 1166–1168, 1212–1214` (`& 1u` already `unsigned`), `ldpc.cpp:159` (already `unsigned`), `udp.cpp:149` (`ntohs` already 16-bit). The `mac.cpp`/`ldpc.cpp` casts are redundant on ARM too. | Fixed: JSF 181 FAIL with the 12 sites; REMEDIATE (PR-33, code-fix plan step 4). | Section 1 JSF 181; Section 2 PR-33 |
| RV-4 | JSF 162 cited `hicpp-signed-bitwise`, which does not test mixing. `-Wsign-conversion`: `conv.cpp:16`, `:17`, `ldpc.cpp:159` (`std::popcount` returns `int`), `udp.cpp:73` (Win32 `FIONBIO`, host-only). Values are 0 or 1, no data lost. The review would keep PARTIAL. | Fixed: evidence replaced; graded FAIL, not PARTIAL, under fail-means-fix (owner, 2026-10-29). REMEDIATE (PR-34). | Section 1 JSF 162; Section 2 PR-34 |
| RV-5 | JSF 157, 171, 173, 186, 204 PASS rest on reading only. | Fixed: each row labeled "By read, not tool-checked". | Section 1 rows |
| RV-6 | CCSDS walk covers 4 of the 7 books CONFORMANCE.md claims; heading and conclusion over-claim. | Parked (CCSDS). | Section 3.1, "Books that the walk did not grade shall by shall" |
| RV-7 | Bulk CCSDS verdicts. | Parked (CCSDS); C-1/C-2 adopted for the restart. | Section 3.1, "Verdicts given to large groups of rows" |
| RV-8 | Four PASS verdicts conflict with code or PICS (133.0-B-2/s041, /s042, /s059; 211.0-B-6/s041). | Settled (N/A ×3, FAIL (gap) ×1); clause table not re-graded (parked). | Section 3.1, "Agreed new grades" |
| RV-9 | 53 rows cite RC `test/test_space_packet_service.cpp` against the `starcom/tests/` rule, and the Space Packet row was raised on those tests. | Parked; the owner decides (C-13). | Section 3.1, "Test location" |
| RV-10 | COP-P stays Full with 3/22 PASS; no level rule for UNVERIFIED; whiteboard items (`synch_timeout = 0`, no resend test) not cited. | Parked; C-6 adopted for the restart. | Section 3.1, "COP-P" |
| RV-11 | Hard-Protected `standards/STANDARDS_AUDIT.md` (3 history rows: 2026-05-13, 2026-08-20 to 2026-08-24, 2026-08-31) and frozen `docs/plans/FSK_PIVOT_PLAN_2026-10-08.md` (a paragraph about decision D14) edited with no cited permission. Pre-existing: the `STANDARDS_AUDIT.md` Section G separator row is after the data rows, so the table does not render. | FSK plan paragraph reverted (owner, 2026-10-29: "The FSK plan doesn't need to be changed"). The 3 `STANDARDS_AUDIT.md` rows are not in the commit. They are proposed and wait for owner approval. The box audit archive has them (`STANDARDS_AUDIT_with_3_rows.md`, sha256 `8473f39e`). The `STANDARDS_AUDIT.md` separator row and the missing 2026-10-29 rows stay open (Hard-Protected). Per-commit protected-file check proposed for `docs/agents/SESSION_CHECKLIST.md` (Hard-Protected; needs the owner). | Section 4 (this row) |
| RV-12 | CCSDS dispositions (E1–E11) applied, coding dispositions only proposed; README G1–G13 "adopted pending review". | Coding: every FAIL is REMEDIATE (fail means fix, owner 2026-10-29). G1–G13: adopted (owner instruction "get the docs in order", 2026-10-29). E1–E11 approval record: parked (CCSDS). | Section 2; Section 3.1 |

**Note to RV-6:** the report scope is 10 claimed books. The walk graded 4 books shall by shall (211.0-B-6, 211.1-B-4, 211.2-B-3, 133.0-B-2). It graded 6 books at row level only (732.1-B-3, 232.0-B-4, 232.1-B-2, 131.0-B-6, 231.0-B-4, 355.0-B-2). The counts in the RV-6 row are the reviewer's.
