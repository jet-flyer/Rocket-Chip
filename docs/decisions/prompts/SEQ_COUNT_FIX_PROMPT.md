# Grok Build task: per-APID, per-direction sequence count (133.0-B-2)

**Status: Not started.** After the change is built, mark this file "awaiting board verify"; it is done only after board verification. The Space Packet service-layer task (`docs/decisions/prompts/SPACE_PACKET_SERVICE_PROMPT.md`) waits for this one to be verified on the board.

## Context

Every on-air packet carries Packet Sequence Count 0. `pump_pack_nav_packet`, `pump_pack_cmd_packet` and `pump_pack_ack_packet` in `src/starcom_adapt/byte_pump.cpp` set only the APID and the packet type. The walk-test CSV `seq` column is not useful until this is fixed. Starcom docs: `starcom/docs/CONFORMANCE.md` (Known gaps, APID allocation, Annex A).

Rules for the session: use the branch/worktree already set up. Do not touch the modified files from the other session (`src/flight_director/*`, `test/*`, `scripts/config_wizard/core/*`). No git push. Never use --no-verify. Do not edit `starcom/docs/CONFORMANCE.md`; Hamilton updates docs after review.

## Item 1: sequence count always 0

Counter per APID AND per direction, 14 bits, continuous, wrapping modulo 16384, in `pump_pack_nav_packet`, `pump_pack_cmd_packet` and `pump_pack_ack_packet`.

Clauses (133.0-B-2):
- 4.1.3.4.3.3: "Packet Sequence Counts are unique and independent per each user application as identified by the APID and are not shared across multiple APIDs."
- 4.1.3.4.3.4: continuous (modulo-16384).
- 4.1.3.3.4.2: "The APID shall provide the naming mechanism for the managed data path."
- 2.2.1 NOTE: "two separate managed data paths, one for each direction, should be used". The NOTE says "should"; the "should" is implemented per project policy (full Blue Book compliance wherever there is no reason not to).

The code comment must cite 4.1.3.4.3.3, 4.1.3.3.4.2 and the 2.2.1 NOTE, and state that the NOTE says "should" and the "should" is implemented per project policy. APID 0x003 is sent by two different senders (command from the station, ACK from the vehicle); the text does not address two senders on one APID. Add no extra rule for it; it is an open question in the report (also on the whiteboard).

## Item 2: receive-side loss detection

Add receive-side sequence-count gap detection with an `rx_lost` counter (per APID, per direction). The receiver today copies the received count into `rx_snapshot.seq` and does nothing else (CONFORMANCE.md Known gaps). Cite the 133.0-B-2 clause for each behavior in a code comment; a behavior with no quotable clause is not implemented and goes in the report as an open question.

## Item 3: idle packet secondary-header flag

For an idle packet (APID 0x7FF, all ones, 4.1.3.3.4.4) force the Packet Secondary Header Flag to 0 on encode (4.1.3.3.3.4). Nothing sends an idle packet today; `encodeSpacePacket` does not force the flag.

## Item 4: code-owner pass, 131.0-B-5 citations in code comments (comments only)

These code comments still cite 131.0-B-5 and need a code-owner pass to 131.0-B-6:
- `starcom/include/starcom/ccsds/conv.hpp:11` ("131.0-B-5 §3.3")
- `starcom/include/starcom/ccsds/ldpc.hpp:12` ("131.0-B-5 §7.4")
- `starcom/src/ccsds/conv.cpp:9` ("131.0-B-5 §3.3")
- `starcom/src/ccsds/ldpc.cpp:9` ("131.0-B-5 annex B notes US Patent 7,343,539") and `:11` ("131.0-B-5 Tables 7-3 / 7-4")
- `starcom/tests/unit/test_coding.cpp:2` ("131.0-B-5 §3.3 / §7.4")

In B-6, the B-5 Tables 7-3 / 7-4 are Tables 8-3 / 8-4. The patent note is still Annex B B3.2. Clause mapping: `starcom/docs/CONFORMANCE.md` (B-5 to B-6 clause map). This item is comment text only: no code change.

## Done when

- Host tests pass: sequence count increments per APID and per direction, wraps at 14 bits; rx_lost counts gaps; idle packet flag is 0.
- Build clean with the project hooks.
- No wire-format or packet-layout change.
- Report: files changed, clause per behavior, anything skipped, open questions (APID 0x003 two senders).
