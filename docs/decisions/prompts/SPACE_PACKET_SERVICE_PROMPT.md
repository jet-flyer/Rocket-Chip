# Grok Build task: CCSDS 133.0-B-2 Space Packet Protocol service layer

**Status: Not started. Must wait for `docs/decisions/prompts/SEQ_COUNT_FIX_PROMPT.md` (per-APID, per-direction sequence count) to be verified on the board.**

TASK: Add the CCSDS 133.0-B-2 Space Packet Protocol service layer to Starcom (host-testable, no board dependency).

READ FIRST
- starcom/docs/CONFORMANCE.md (Space Packet row, Annex A PICS, managed-parameter table, APID table, Known gaps)
- src/starcom_adapt/byte_pump.cpp (current pack/unpack code)
- standards/starcom/ccsds/CCSDS-133.0-B-2-EC2.pdf (133.0-B-2 with EC1 and EC2; the clauses quoted below)
- AGENT_WHITEBOARD.md (open items)
Use the branch/worktree already set up. Do NOT touch the ~12 modified files from the other session (src/flight_director/*, test/*, scripts/config_wizard/core/*). No git push. Never use --no-verify.

PREREQUISITE
The per-APID, per-direction 14-bit sequence-count fix (pump_pack_nav_packet, pump_pack_cmd_packet, pump_pack_ack_packet; 133.0-B-2 4.1.3.4.3.3/.4) must already be in the tree. If it is not, stop and report; do not build on a counter that is always 0.

SCOPE (mandatory Annex A rows only: SPP-2, 6, 7, 8, 10, 11, 12, 13, 19-22; SPP-9 is optional, skip it)
The codec (clause 4.1 packet structure) already exists. Add the service layer on top of it:
- Octet String Service
- Packet.request / Packet.indication (3.3.3.2, 3.3.3.3)
- Packet Assembly, Packet Transfer (sending side): 4.2.2, 4.2.3
- Packet Extraction, Packet Reception (receiving side): 4.3.2, 4.3.3

RULES
1. Every behavior you implement must cite the 133.0-B-2 clause number in a code comment. If a behavior has no clause you can quote, do not implement it; list it as an open question in your report instead.
2. Do not add features beyond the rows above.
3. Keep it host-testable (no Pico SDK dependency in the new logic), matching the existing byte_pump style and standards/CODING_STANDARDS.md.
4. Add host unit tests for each service function, including: the sequence count increments per APID and per direction and wraps at 14 bits; a malformed or too-short packet is rejected by extraction; indication delivers the right octets and APID.
5. Do not change wire format or existing packet layouts.
6. Commit on milestones with clear messages.
7. Sequence counters are per APID AND per direction. In the code comment cite 4.1.3.4.3.3, 4.1.3.3.4.2 and the 2.2.1 NOTE. State in the comment that the NOTE says "should" and that the "should" is implemented per project policy (full Blue Book compliance wherever there is no reason not to). Two senders sharing APID 0x003 are not addressed by the text: add no extra rule for it and list it as an open question in your report.

DONE WHEN
- Host tests pass and the build is clean with the project's hooks.
- Report back: files changed, which Annex A rows are now covered (with clause per row), anything skipped, and open questions.
- Do NOT edit starcom/docs/CONFORMANCE.md yourself; Hamilton updates the docs after review.

CODE-COMMENT CITATIONS (not part of the service-layer code; for a code-owner pass)
These code comments still cite 131.0-B-5 and need a pass to 131.0-B-6: `starcom/include/starcom/ccsds/conv.hpp:11`, `starcom/include/starcom/ccsds/ldpc.hpp:12`, `starcom/src/ccsds/conv.cpp:9`, `starcom/src/ccsds/ldpc.cpp:9` and `:11`, `starcom/tests/unit/test_coding.cpp:2`. In B-6, B-5 Annex B / Tables 7-3 and 7-4 (`ldpc.cpp:9` / `:11`) are Tables 8-3 / 8-4; the patent note is still Annex B B3.2. Comment text only. The same item is in `SEQ_COUNT_FIX_PROMPT.md` (item 4).
