# CFDP reuse study for the Starcom `cfdp` module

- **Date:** 2026-11-21
- **Author:** Goddard
- **Scope:** Whether an existing CFDP (CCSDS File Delivery Protocol, CCSDS 727.0-B-5) implementation can be reused for Starcom's planned `cfdp` module, whose first job is USB log offload with Class 1 plus transaction closure. Candidates: nasa/CF (cFS), F´ `Svc/Ccsds/CfdpManager`, cfdp-py, LibreCube python-cfdp, EmbeddedCFDP, cfdp-rs and ION-DTN. Also checks the reuse claims Hamilton raised in the room. Read-only research; nothing was committed, pushed or reported upstream.
- **Repo checked:** Rocket Chip `origin/main` at `a6d5d4b9` (fetched 2026-11-21) and `origin/fprime` at `b31bc13c`.
- **Status:** Frozen once committed. A later report replaces this one.

Every finding is labeled. **[Fact]** means it was read from code, a measurement, or a cited source. **[Inference]** means it is Goddard's estimate or judgment and has not been verified.

Method notes:
- Third-party sources are shallow clones at the commits below. Line counts are `wc -l`, which counts every line including comments and blanks (cloc was not available).
- The standard was read from https://ccsds.org/Pubs/727x0b5e1.pdf (727.0-B-5, July 2020).
- Every file and line citation below was re-checked against the same clones on 2026-11-21. The nasa/CF `dev` branch was still at `0dbc2b7` upstream. GitHub issue states were re-checked through the GitHub API the same day.

| Source | Commit / version read |
|---|---|
| nasa/CF | `dev` (default branch) at `0dbc2b7` (2026-10-29). `cf_version.h` reports 7.0.2. Tags go up to `v7.0.2-rc1` |
| nasa/fprime | `devel` at `633c89d` (2026-11-19). CfdpManager is in releases v4.3.0 (2026-08-20), v4.4.0, v4.4.1 and v4.4.2 (2026-11-04). It is not in v4.2.3, v4.1.0 or v4.0.0 (GitHub contents API returned 404 for those) |
| ustgt-irs/cfdp-py | `3170e66` (2026-09-14), with spacepackets 0.32.0 |
| LibreCube python-cfdp (gitlab.com/librecube/lib/python-cfdp) | `a393baa` (2025-12-19) |
| OpenSpaceCode/EmbeddedCFDP | `d3c7bee` (2026-10-03). 22 commits; repo created 2026-07-22 |

## 0. Rocket Chip and Starcom constraints that decide this

- **[Fact]** The Starcom core is sans-I/O: "Bytes and `tick(now)` in; events and bytes out" (`starcom/docs/DESIGN.md:138`). Protocol state machines must be "plain portable C++ (table-testable, no framework dependency)" (`starcom/docs/WORKING_HERE.md:46`). Exceptions, RTTI and heap-after-init are out (`WORKING_HERE.md:70`). DESIGN.md lists a "malloc-after-init hard gate" in CI (line 118). `standards/CODING_STANDARDS.md` applies to Starcom too (`WORKING_HERE.md:70`); it consolidates JSF AV C++, Power of Ten and JPL-C (`CODING_STANDARDS.md:18-20`).
- **[Fact]** Rocket Chip has no filesystem. Logs go to a raw flash region, `kFlashLogStart`..`kFlashLogEnd` in `include/rocketchip/flash_layout.h:60-61`. `git grep` for littlefs, fatfs or `lfs_` in `src/`, `include/` and `CMakeLists.txt` finds nothing.
- **[Fact]** Both NASA engines call open, read, lseek, write, rename and remove, use temp and fail directories, and scan directories, all from inside the state machine (§1 and §2 below).
- **[Inference]** Reusing either one means building a fake filesystem over the flash log, and the file I/O would sit inside the core. That breaks the sans-I/O rule.

## 1. nasa/CF (cFS CFDP application)

- **[Fact]** License: Apache-2.0 (`LICENSE`).
- **[Fact]** `README.md:8`: "CF is a cFS application for providing CFDP (CCSDS File Delivery Protocol) CCSDS 727.0-B-5 compliant services."

### Engine and glue (`fsw/src`, `wc -l`)
| Part | Files | Lines |
|---|---|---|
| Engine (state machines, codec, chunks, lists, checksum, timer, utilities, types) | cf_cfdp.c/.h, cf_cfdp_r.c/.h, cf_cfdp_s.c/.h, cf_cfdp_dispatch.c/.h, cf_codec.c/.h, cf_chunk.c/.h, cf_clist.c/.h, cf_crc.c/.h, cf_timer.c/.h, cf_utils.c/.h, cf_cfdp_pdu.h, cf_logical_pdu.h, cf_cfdp_types.h | 12,718 |
| of which PDU codec | cf_codec.c/.h, cf_cfdp_pdu.h, cf_logical_pdu.h | 2,728 |
| of which tx/rx state machines plus engine core | cf_cfdp_r.c (1,204), cf_cfdp_s.c (1,038), cf_cfdp_dispatch.c (191), cf_cfdp.c (2,491) | 4,924 |
| of which chunk/gap tracking | cf_chunk.c/.h | 762 |
| of which checksum (modular only, despite the file name) | cf_crc.c/.h | 174 |
| cFS glue (app main, commands, SB interface, dispatch, EDS) | cf_app.*, cf_cmd*, cf_dispatch.*, cf_eds_dispatch.c, cf_cfdp_sbintf.*, fsw/inc/*.h | 7,426 |
| All `fsw/src/*.c` | | 11,243 |
| All .c/.h in `fsw/src` plus `fsw/inc` | | 20,310 |

### cFE, OSAL and PSP calls the engine makes (glue files excluded)
- **[Fact]** `cf_cfdp.c`: `CFE_EVS_SendEvent` x26; `CFE_SB_CreatePipe`, `SubscribeLocal`, `DeletePipe` x2, `AllocateMessageBuffer`, `TransmitBuffer`, `TimeStampMsg`, `ValueToMsgId` x2; `CFE_MSG_Init`, `CFE_MSG_PTR`; `CFE_ES_PerfLogEntry`/`Exit`, `CFE_ES_WriteToSysLog`; `OS_mkdir`, `OS_strnlen`, `OS_DirectoryOpen`/`Read`/`Close`, `OS_ObjectIdDefined`, `OS_CountSemGetIdByName`, `OS_TaskDelay`.
- **[Fact]** `cf_cfdp_r.c` and `cf_cfdp_s.c`: `CFE_EVS_SendEvent` x15 and x13; `OS_remove`, `OS_mv`; `cf_cfdp_s.c` also calls `OS_FileOpenCheck`.
- **[Fact]** `cf_utils.c`: `OS_OpenCreate`, `OS_read`, `OS_write`, `OS_lseek`, `OS_close`; perf logging.
- **[Fact]** `cf_codec.c`, `cf_chunk.c`, `cf_clist.c`, `cf_crc.c` and `cf_timer.c` make no cFE or OSAL calls. These are the portable pieces.
- **[Fact]** Configuration comes from a cFE table (`CFE_TBL_*` in `cf_app.c`). The engine reads `config_table` fields such as ticks_per_second and channel settings through the engine struct.
- **[Inference]** Shim estimate: about 40 distinct API names. About 6 are trivial (perf logging, syslog, strnlen, events turned into a log hook). The software-bus pipe and buffer calls (about 10) are the PDU in/out path. The file and directory calls (about 12) need a filestore over flash. The config table needs a replacement struct. Mapping cFE's 1-tick `wakeup` to QP is easy. The real work is the filestore and taking the SB out of `cf_cfdp.c`, roughly 1-2k lines of shim plus edits inside `cf_cfdp.c`. The result would still be C99 with packed structs and I/O inside the engine, which does not meet Starcom's C++ sans-I/O rule.

### Memory
- **[Fact]** No malloc, calloc or free calls in `fsw/src`.
- **[Fact]** Everything is static. `CF_Engine_t` (`fsw/src/cf_cfdp_types.h:527-545`) holds fixed arrays `transactions[CF_NUM_TRANSACTIONS]`, `histories[CF_NUM_HISTORIES]`, `channels[CF_NUM_CHANNELS]`, `chunks[...]` and `chunk_mem[...]`, inside the global `CF_AppData`.
- **[Fact]** Defaults: 2 channels, `CF_NUM_HISTORIES_PER_CHANNEL` 256, `CF_NAK_MAX_SEGMENTS` 58, `CF_MAX_PDU_SIZE` 512 (`fsw/inc/cf_interface_cfg.h:51, 65, 100`; `fsw/inc/cf_internal_cfg.h:101`). Filenames are `CFE_MISSION_MAX_PATH_LEN`.
- **[Fact]** The stack "must be no less than 16384 bytes" (`docs/dox_src/cfs_cf.dox:1368-1369`).

### Features and limitations (`docs/dox_src/cfs_cf.dox`)
- **[Fact]** "CF implements Class 1 and 2 send and receive, as well as logic to cancel, suspend, resume, abandon, freeze, etc." (line 115).
- **[Fact]** Entity-ID and transaction-sequence-number lengths are fixed at build time, and segmentation control is not supported (lines 1361-1385).
- **[Fact]** "This version of CF does not support keep alive procedures that are detailed in section 4.1.6.5" (line 1387).
- **[Fact]** Troubleshooting (line 1439): "The current version of CF does not support PDU CRC. The ground system needs to set the "CRC Flag" to false."
- **[Fact]** Large-file PDUs are rejected (`CF_PDU_LARGE_FILE_ERR_EID`, `fsw/src/cf_cfdp.c:734-736`).
- **[Fact]** Only the modular checksum is implemented. Open issue nasa/CF#99 says CF "does not check this field [checksum type] at all".
- **[Fact]** The 727.0-B-5 Class 1 sender ICS (Annex A4.1.2) marks "CFDP-1S-06 CRC procedures 4.1 M" and "CFDP-1S-25 Null checksum algorithm implemented 4.2.2.4 M". CF meets neither.

### Class 1 closure in CF
- **[Fact]** Sender: `CF_CFDP_S1_CheckState_DATA_EOF` (`fsw/src/cf_cfdp_s.c:690-712`) waits for a FIN when `close_req` is set. But `txn->flags.com.close_req` is only assigned from a received Metadata PDU (`fsw/src/cf_cfdp.c:794`), and `CF_TxFile_Payload` (`config/default_cf_msgdefs.h:390-399`) has no closure field.
- **[Inference]** So a CF sender always sends closure = 0, and the S1-with-closure path cannot be reached from a command. No Check timer for S1 closure (727.0-B-5 §4.6.3.2.2) was found either; that part was only a quick read.

### Finding: CF puts the closure-requested flag on the wrong bit
- **[Fact]** `fsw/src/cf_codec.c:108`: `CF_CFDP_PduMd_CLOSURE_REQUESTED = CF_INIT_FIELD(1, 7)`, which is shift 7, mask 0x80 (`CF_INIT_FIELD` at `cf_codec.c:40`). It is used on encode at `:582` and on decode at `:909`, and those are its only two uses.
- **[Fact]** 727.0-B-5 Table 5-9 (Metadata PDU contents) lays out that octet as "Reserved for future use 1 bit; Closure requested 1 bit; Reserved for future use 2 bits; Checksum type 4 bits", so closure requested is 0x40. Duke confirmed this against the book.
- **[Fact]** Four other implementations use 0x40: spacepackets `cfdp/pdu/metadata.py:186, 228`; LibreCube `src/cfdp/pdu/metadata.py:65, 137`; EmbeddedCFDP `src/cfdp_directive.c:326, 374`; F´ `Types/MetadataPdu.cpp:124, 181`. F´ fixed the same bug in nasa/fprime#6136, "CfdpManager: Metadata PDU closure-requested flag uses bit 0x80 instead of 0x40" (closed 2026-10-16).
- **[Fact]** Byte check: spacepackets 0.32.0 encodes a Class 1 Metadata PDU with closure requested as `24000a0001030207400000000a01610162` (reproduced 2026-11-21). The first Metadata parameter octet is `0x40`, which CF's shift-7 field reads as 0.
- **[Inference]** Effect: when a spec-correct sender asks for closure, CF's R1 receiver does not send the FIN. In the other direction, any closure request CF encodes sets the reserved bit.
- **[Fact]** A search of nasa/CF issues for "0x80", "closure requested", "CLOSURE_REQUESTED" and "segmentation_control" found no existing issue for this (re-checked 2026-11-21). CF PR #482 (closed) added engine support for Class 1 closure but does not touch the bit position.

### Open issues that matter (all open on 2026-11-21)
- **[Fact]** #343 "PDU processing when crc_flag true". #491 out-of-bounds heap read in `CF_CFDP_CopyStringFromLV` (security). #520 received FD offset not bounded against file size. #535 chunk list drops data when full (affects lossy Class 2); PR #536 proposes a fix. #389 receiving entity not handling EOF (cancel) correctly. #99 alternate checksum type. #540 received files not retained when the temp path is 20 or more characters.

### Standards and compliance statements
- **[Fact]** CCSDS: the claim is "727.0-B-5 compliant" (`README.md:8`). The user guide says PDUs "are compliant with the CFDP standard protocol defined in the CCSDS 727.0-B-5 Blue Book" and "The ASIST and ITOS ground systems contain a compliant peer that may be used" (`cfs_cf.dox:97-103`).
- **[Fact]** No statement of formal conformance testing was found: no 720.4-Y-1 Yellow Book test series, no published ICS proforma, no interoperability report in the repo or docs. Searching issues for "interoperab" returned nothing.
- **[Inference]** The ASIST and ITOS lines name possible peers; they do not report test results.
- **[Fact]** Coding standard: nasa/CF#42 (closed 2021-12-09), "CF Code Style and Coding standards compliance", refers to cleaning up code "to better comply with GSFC coding standards". #369 (open) "Audit CF for cyclomatic complexity compliance". No MISRA, NPR 7150.2 class or JPL-C statement appears in the CF repo.
- **[Fact]** The cFS bundle README (https://github.com/nasa/cFS) says: "A Government-use (Distro C) version of cFS with features, certifiability, and security measures for a full flight mission is available through a Software User Agreement." It also mentions the team's "experience in helping flight projects implement mission-ready flight software using cFS for various classes of missions, including human-rated (class A) missions."
- **[Inference]** So the open-source CF repo does not claim an NPR 7150.2 class; certifiability is described as a separate government version.
- **[Fact]** `docs/cf_FunctionalRequirements.csv` (184 lines by `wc -l`) is functional only, for example "CF5040 CF shall support CFDP file transfers in "Unacknowledged"(Class-1) Service Mode." It has no mapping to conformance clauses.

## 2. F´ `Svc/Ccsds/CfdpManager`

- **[Fact]** Path: https://github.com/nasa/fprime/tree/devel/Svc/Ccsds/CfdpManager. The modular checksum is separate, in `CFDP/Checksum/` (237 lines for `Checksum.cpp` and `.hpp`).
- **[Fact]** History: first merged as commit `20b39184` on 2026-08-04, "CCSDS File Delivery Protocol - CfdpManager (#5138)". It first shipped in v4.3.0, whose release notes list "new `Svc::Ccsds::CfdpManager` for standards-based file transfer". It has been patched often since; the original notes counted about 10 CfdpManager commits on 2026-11-02 and 2026-11-03 alone (not re-counted).
- **[Fact]** License: Apache-2.0.
- **[Fact]** Derived from nasa/CF v3.0.0. `ATTRIBUTION.md` says "Portions of this code are derived from the NASA Core Flight System (cFS) CFDP (CF) Application" with "Version: 3.0.0". It maps Engine to cf_cfdp.c, TransactionTx to cf_cfdp_s.c, TransactionRx to cf_cfdp_r.c, and Channel, Utils, Chunk and Clist to their CF equivalents. The `Types/` PDU classes are new code written from the Blue Book.

| Part | Files | Lines |
|---|---|---|
| Engine (CF-derived) | Engine.*, Transaction*.cpp/.hpp, Channel.*, Chunk.*, Clist.*, Utils.*, Timer.* | 7,970 |
| PDU codec (new) | Types/*.cpp/.hpp/.fpp (no tests) | 3,242 |
| F´ component glue | CfdpManager.cpp/.hpp, *.fpp, *.fppi | 2,269 |
| Checksum | CFDP/Checksum (Checksum.cpp/.hpp) | 237 |

- **[Fact]** Dependencies (occurrences outside tests): `Fw::Buffer` x71, `Fw::SerialBuffer`/`SerialBufferBase` x105, `Fw::String` x60, `Fw::SerializeStatus` x114, `FW_ASSERT` (24 in Engine.cpp, 21 in Channel.cpp, 19 in Clist.cpp, 11 in Chunk.cpp). Also `Os::File`, `Os::FileSystem` (removeFile, moveFile), `Os::Directory`, `Os::Queue`, `Os::FilePathUtils`, and generated `*Ac.hpp` types from FPP.
- **[Fact]** The engine calls straight into the component: `m_cfdpManager->` appears 54 times in TransactionRx.cpp and 27 in TransactionTx.cpp, mostly `log_WARNING_LO_*` events (39), telemetry counters and `get*Param`.
- **[Inference]** Outside F´ you would need Fw, Os and FPP autocode, or stubs for all of it. That is not realistic in Starcom.
- **[Fact]** Memory: no runtime heap. `CfdpManager::configure(Fw::MemAllocator&...)` allocates the Engine, transactions, chunks and histories once at init with placement new (`CfdpManager.cpp:35-40`, `Channel.cpp:124-163`). F´ docs: "dynamic memory allocation during runtime is forbidden by flight software coding standards" (`docs/user-manual/framework/memory-management/memory-allocation.md:3`).
- **[Fact]** Timers count whole seconds, driven by the `run1Hz` port (`CfdpManager.fpp:23`, `Timer.cpp`).
- **[Fact]** Component kind: `active component CfdpManager` (`CfdpManager.fpp:6`). The F´ bare-metal guide says "baremetal F´ systems should avoid using **Active Components** where possible" (`run-baremetal.md:24`). Files on bare metal need MicroFs (RAM only) or a custom `Os::FileSystem` (`run-baremetal.md:70-76`).
- **[Fact]** Class 1 closure is not supported in either direction. Sender: `Engine.cpp:197-199`, "Class 1: closure not requested (0), Class 2: closure requested (1)". Receiver: `getClosureRequested()` is never called outside `Types/` and tests, and the R1 FIN entry is `nullptr` (`TransactionRx.cpp:137`). Tracked as nasa/fprime#6213 (open, opened 2026-10-16): "CFDP Class 1 implementation does not fulfill closure request ... Capturing as lowest-of-low priority."
- **[Fact]** PDU CRC is not supported: `Types/PduHeader.cpp:27` ("CRC not currently supported"), and line 197 rejects CRC-flagged PDUs.
- **[Fact]** Standards: the SDD has a requirements table (CFDP-001 to CFDP-015) verified by unit and system test. It makes no interoperability or conformance-test claim. The F´ docs contain no MISRA or NPR 7150.2 statement.

### Rocket Chip `origin/fprime` branch
- **[Fact]** Docs only: 5 files and 205 added lines (`CHANGELOG.md`, `docs/plans/FPRIME_BRANCH_PLAN.md`, `docs/plans/FPRIME_WHITEBOARD.md`, and two 2026-10-14 reports: the ChibiOS OSAL map and RTOS findings). Branched from `c7d70cc6` (2026-10-14); last commit `b31bc13c` (2026-10-15).
- **[Fact]** Approach: an F´ project under `fprime/` with nasa/fprime as a submodule, ChibiOS as the first deployment with a custom `fprime-chibios` OSAL. The plan notes "nasa/fprime `devel` ships ... `CfdpManager`" and "Starcom comes into F´ later" (`FPRIME_BRANCH_PLAN.md:11, 108`).

## 3. Other candidates

- **cfdp-py** (https://github.com/ustgt-irs/cfdp-py; README badges point at us-irs)
  - **[Fact]** Apache-2.0, 3,801 lines of Python under `src` (`wc -l`), built on spacepackets.
  - **[Fact]** README: "Unacknowledged (class 1) file transfers for both the sending and destination side". Not implemented: suspend, inactivity handling, keep-alive and prompt.
  - **[Fact]** Class 1 closure works on both sides: `source.py:871` (`if self._params.closure_requested:`) waits for a FIN, and `dest.py:936-940` sends a FIN when the mode is unacknowledged and closure is requested. The closure bit is 0x40.
  - **[Fact]** The LibreCube cross-test is an example, not a CI suite: `examples/cfdp-libre-cube-crosstest/` ("This mini application cross tests the tmtccmd CFDP support and the LibreCube CFDP").
  - **[Inference]** Good choice as the ground-side test partner.
- **LibreCube python-cfdp**
  - **[Fact]** MIT. README: Class 1, Class 2, filestore requests, proxy and directory listing; "The protocol is tested for cross-support as outlined in CCSDS CFDP Yellow Book." `tests/README.md` says the tests follow "closely the tests provided in the CCSDS Yellow Book 720.4-Y-1" (F1/F2/F3).
  - **[Inference]** The only candidate with Yellow-Book-style test series. Good second ground peer.
- **EmbeddedCFDP**
  - **[Fact]** Apache-2.0 (`LICENSE` file; GitHub API `spdx_id: Apache-2.0`). This corrects an earlier room note that said the license was not stated.
  - **[Fact]** C99, 2,957 lines in `src` plus `include`, plus about 2,700 lines of tests. No malloc.
  - **[Fact]** Codec only: every §5 PDU, TLV/LV, the PDU CRC-16 (its README cites CCSDS 232.0-B-3 §4.2.1.3), and the modular and null checksums. The README puts "timers, retransmission and the transaction state machine" out of scope. 22 commits, 2 stars.
  - **[Inference]** Useful as a cross-check of encodings.
- **cfdp-rs** (Rust, us-irs)
  - **[Fact]** README: Class 1 and Class 2 on source and destination, and a `python-interop` example with cfdp-py.
  - **[Inference]** Not a fit for C++ firmware.
- **ION-DTN** (JPL, BSD-3, https://github.com/nasa-jpl/ION-DTN)
  - **[Fact, not re-checked]** The SourceForge page lists "An implementation of Class 1 CCSDS File Delivery Protocol (CFDP)", and the `bpcp` docs say "ION's CFDP implementation requires the use of reliable convergence layers". Not cloned; these quotes were not re-checked on 2026-11-21.
  - **[Inference]** Tied to ION's Bundle Protocol stack. Not a fit.

## 4. Spec citations for closure (727.0-B-5)

- **[Fact]** §2 overview (page 2-7): "Optionally, a Finished PDU may also be sent upon completion of a transaction in Unacknowledged mode. This option, termed 'transaction closure', enables the source entity to be informed of the outcome of a transaction that did not require acknowledgments, possibly because it was conducted using an underlying communication system that was itself known to be reliable." The "known to be reliable" wording is in this §2 overview, not in §4.6.2 (which is "Optional copy file procedures at the receiving entity").
- **[Fact]** §4.6.3.1: closure requires "duplex transmission paths".
- **[Fact]** §4.6.3.2: on the sender, a Check timer starts after EOF; a FIN stops it and gives a Notice of Completion; if it expires, a Check Limit Reached fault is declared.
- **[Fact]** §4.6.3.3: the receiver issues a Finished (complete) PDU if the Closure Requested flag is '1'.
- **[Fact]** §7.2.3.2: "Transaction Closure Requested" event diagram for Class 1.
- **[Fact]** Table 5-9: closure requested is the second bit of the first Metadata parameter octet (0x40).
- **[Fact]** Table 5-7 (Finished PDU contents): file status '10' is "Delivered file retained in filestore successfully".

## 5. Check of Hamilton's claims

| Claim | Verdict | Evidence |
|---|---|---|
| nasa/CF v7 on dev is about 11.2k lines of C | Confirmed | `dev` is the default branch; `cf_version.h` 7.0.2; `wc -l fsw/src/*.c` = 11,243; with headers and `fsw/inc` it is 20,310 |
| No malloc | Confirmed | No malloc, calloc or free calls in `fsw/src`; static arrays in `CF_Engine_t` |
| GSFC/cFS conventions | Confirmed, weakly | nasa/CF#42 on GSFC coding standards (closed 2021); no MISRA or NPR claim in the repo |
| Engine supports Class 1 closure, but the transmit command can't enable it | Confirmed, with a correction | `CF_TxFile_Payload` has no closure field and `close_req` is set only from a received Metadata PDU (`cf_cfdp.c:794`). But CF also reads and writes the flag at 0x80 (`cf_codec.c:108`) where 727.0-B-5 Table 5-9 puts it at 0x40, so CF's R1 also ignores closure requests from spec-correct senders such as cfdp-py. No CF issue exists for this |
| F´ CfdpManager is a port of older CF v3, merged Aug 2026 | Confirmed | `ATTRIBUTION.md` "Version: 3.0.0"; commit `20b39184` on 2026-08-04 (#5138); first release v4.3.0 (2026-08-20) |
| Class 1 closure gap is fprime#6213 | Confirmed | Open, opened 2026-10-16 |
| #6377 notes CF v7 fixes it | Confirmed that #6377 says so; the claim is incomplete | The #6377 row for #6213 reads "Fixed: `CF_CFDP_R_CheckState_FILESTORE` enters FINACK when `close_req` is set". That is true of CF's state machine, but CF decodes `close_req` from the wrong bit, so porting CF v7's logic without the F´ #6136 bit fix would not interoperate |
| EmbeddedCFDP has an Apache-2.0 LICENSE on main | Confirmed | `LICENSE` file; GitHub API `spdx_id: Apache-2.0` |
| The closure clause is 727.0-B-5 §4.6.3 and §7 | Confirmed | §4.6.3.1-3 (procedures) and §7.2.3.2 (Class 1 closure event diagram). The "known to be reliable" rationale is in the §2 overview (page 2-7), not §4.6.2 |

## 6. Recommendation

Options considered: (a) write our own from the book only; (b) port nasa/CF; (c) adopt F´ CfdpManager; (d) write our own and mirror nasa/CF's structure.

**[Inference]** Recommended: **(d)**, with cfdp-py as the independent ground peer, LibreCube as a second peer, and EmbeddedCFDP's encodings as a cross-check. Reasons:

1. **Neither NASA engine gives us Class 1 closure today.** F´ lacks it entirely (#6213). CF lacks it on the sender side, and its receiver reads the wrong bit. The one feature needed now is the one both are missing.
2. **Both engines do file I/O inside the state machine, and Rocket Chip has no filesystem.** Reuse means a fake POSIX filestore over the flash log plus a shim for the cFE SB, EVS and tables, or for F´ Fw, Os and FPP. A Starcom-native sender can read through a small "file source" interface (`read(offset, span)`) that maps straight onto the flash log region and keeps the core sans-I/O.
3. **Size.** A Class 1 sender plus receiver with closure is a small subset: header, Metadata, File Data, EOF and FIN codecs, the modular checksum (plus null and the PDU CRC-16 for honest ICS coverage), and two small state machines with a Check timer. CF's engine is 12.7k lines and F´'s is about 11.2k lines (engine plus codec) plus glue; most of that is Class 2, channels, playback, polling, history and commands.
4. **Language and standards fit.** CF is C99 with packed structs and a global `CF_AppData`. F´ is C++ tied to the framework. The house standard (JSF/P10/JPL-C, camelBack, no heap after init, sans-I/O) is easier to meet by writing fresh.
5. **Mirroring CF's structure helps.** Use the same split (codec, chunk/gap list, tx state machine, rx state machine, dispatch) and the same state names (S1, S2, R1, R2; FILEDATA, EOF, FINACK). Port `cf_chunk.c`'s algorithm when Class 2 arrives, since it has no cFE calls (762 lines, Apache-2.0, attribution needed), taking the nasa/CF#536 fix into account. That keeps a later move to F´ easy, because CfdpManager has the same lineage.
6. **F´ migration.** When Rocket Chip moves to F´, the choice is to adopt CfdpManager or wrap Starcom's `cfdp` as a component. Today CfdpManager would need #6213 fixed (#6136 is already in) for USB offload with closure. It is also an active component, which the F´ bare-metal guide advises against (fine on the ChibiOS deployment). PDU-level interop with cfdp-py is what guarantees either path works.

Why not the others:
- **(b) Port CF:** too much shim work, and it inherits the closure-bit bug, the lack of PDU CRC and null checksum, and open security issues #491 and #520.
- **(c) Adopt F´ CfdpManager:** it cannot be used outside F´ without Fw, Os and FPP, and it lacks Class 1 closure.
- **(a) Book only:** workable, but (d) costs nothing extra and makes the F´ path easier.

Optional upstream contribution, not done and needing Nathan's approval: report the CF closure-requested bit position (`cf_codec.c:108`) to nasa/CF.

## 7. Later corrections and decisions (2026-11-21)

These came after the study was first written.

- **[Fact]** Duke confirmed the closure-bit finding against the book: 727.0-B-5 Table 5-9 puts closure requested at 0x40, while `fsw/src/cf_codec.c:108` uses shift 7 (0x80), on encode at `:582` and decode at `:909`. The bug affects both sending and receiving. Hamilton has added it to the whiteboard as a look-into-later item; reporting it upstream is still Nathan's call.
- **[Fact]** The "Delivered file retained in filestore successfully" file status is in 727.0-B-5 Table 5-7 (Finished PDU contents). An earlier room message cited Table 5-11; that was wrong. The plan has the board erase a log only after a Finished PDU with no error, Data Complete, and this status.
- **[Fact]** The Class 1 closure "known to be reliable" wording is in the §2 overview (page 2-7), not §4.6.2 (as already noted in §4 and §5).
- **Nathan's decisions:**
  - CFDP Class 1 with closure is planned for USB offload, in the Starcom library as its own `cfdp` module.
  - Build against reuse stays open until implementation. The recommendation in §6 is input to that decision, not the decision.
