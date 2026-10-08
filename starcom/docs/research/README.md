# Starcom research notes

Historical craft notes (relocated, append-only content):

- `ccsds_domain_claude.md`, `ccsds_domain_grok.md`
- `library_craft_claude.md`, `library_craft_grok.md`

CCSDS compliance / deviation research (2026-10, LOOK — not decided):

**Survey and pattern**

- `ccsds-deviation-survey.md` — other teams' exception-row patterns; then-vs-now; Duke R-row fold-in
- `ccsds-deviation-rationale.md` — reason types from the other-project survey
- `deviation-clause-map.md` — Duke clause map
- `reevaluate-candidates.md` — skipped helpers / kitchen-sink re-eval (`space_packet_service` confirmed)

**Book color / Magenta / Prox-1 vs USLP**

- `green-orange-yellow-review.md` — Green / Orange / Yellow sweep (informative only)
- `magenta_review.md` — Magenta book review
- `prox1_vs_uslp.md` — Prox-1 vs USLP re-eval notes
- `prox1-only-mac-and-lunar.md` — Prox-1 silence on message auth; lunar Pink / Red draft facts

**Issue / R-row / radio checks**

- `current-issue-recheck.md` — Duke current-issue checks
- `check2-deviation-by-issue.md` — deviation rows by book issue
- `check2-R-rows.md` — R2, R3, R4, R6, R7, R8
- `check3-radio-reasons.md` — OreSat AX5043 EOL, OpenLST CC1110 rate/sync, SX-USP sync vs chip (datasheet/doc only)

**OTS / BT / compliance draft**

- `ots-gmsk-bt-lunar.md` — OTS radios, GMSK BT, lunar S-band (Pink states BTs=0.25; no shall on that sentence)
- `compliance-record-draft.md` — draft Starcom compliance / deviation record (facts only; no INFERENCE labels)
- `compliance-inferences-parked.md` — claims parked out of the draft until a book quote or check makes them facts

RC product-utility research (`rc-utility-2026-10-07/`; 2026-10-07, landed 2026-10-08; status: research — nothing decided):

- `rc-utility-2026-10-07/rc-product-utility-map.md` — main map (v2i): Tier 1 / 2 / 3 element verdicts, PHY forms A / A′ / B / B′ / C / D, ranging, GPS limits, PIO column, rework map (§9), IRL tests (§10), Pico analyzer (§10.2), open checks (§13), parked items (§14); 2026-10-07; research
- `rc-utility-2026-10-07/rc_utility_calcs.py` — calc script for the map's [S#] numbers (`python3 rc_utility_calcs.py [--fast]`); 2026-10-07; research (calc only, nothing board-measured)
- `rc-utility-2026-10-07/rc_utility_calcs_output.txt` — output of the calc script; 2026-10-07; research (calc only)
- `rc-utility-2026-10-07/rc-customer-operating-points.md` — Goddard: customer apogee / slant-range operating points, [OP-S#] sources; 2026-10-07; research
- `rc-utility-2026-10-07/goddard-g1-g4-g6.md` — Goddard: FCC rule quotes (G1), ranging accuracy (G4 / G5), 1.9–2.4 GHz rules (G6); 2026-10-07; research
- `rc-utility-2026-10-07/gps-cutoff-and-extreme-gear.md` — Goddard: GPS export-rule history, receiver gate logic, extreme-flight gear; 2026-10-07; research (not legal advice)
- `rc-utility-2026-10-07/d-checks-product-map.md` — Duke: D-checks, Prox-1 clause quotes for the map; 2026-10-07; research
- `rc-utility-2026-10-07/lunar-positioning-clauses.md` — Duke: lunar-draft clause search (PN ranging, Doppler / coherency, time-correlation scope, Annex E self-conflicts; 93 quotes checked against page footers) **[draft]** books, not stable; 2026-10-07; research
- `rc-utility-2026-10-07/phy_legality_us.md` — Goddard: US legality of the Prox-1 PHY on a rocket or drone; 2026-10-02 (older; uses INFERENCE / UNVERIFIED labels; the map parks those claims in §14); research (not legal advice)

**Source conflict C17 (ROOM, Duke; map §13):** 211.2-P-3.2 [draft] Document Control p. iv lists "211.2-B-4 ... July 2025 Current issue". 211.0-P-6.2 ref [6] and 235.1-R-1 ref [4] call 211.2-B-4 "forthcoming". The live ccsds.org Blue Book list shows only 211.2-B-3. The map keeps citing 211.2-B-3 (`lunar-positioning-clauses.md` l.371–379).

**Known errors in `standards/RF_COMPLIANCE.md`** (recorded in `goddard-g1-g4-g6.md` and map §2.6; the file is **not** edited here): l.33 "measured 6 dB BW ≈ 500 kHz" has no source (measured sources give 630–713 kHz); l.18 / l.20 cite "15.247(b)(3)(ii)", which does not exist (the antenna rule is (b)(4)); l.99, l.110, l.145–146, l.166 use 3 + 3 dBi (product pages give 2.9 and 2 dBi); l.37 / l.122 claim a module grant that covers all LoRa bandwidths (the only HopeRF RFM95-family grant found is 2ASEORFM95C, RFM95C).

**Doc errors found by Buzz (2026-10-08; for Nathan to decide; files not edited):** (1) `docs/hardware/HARDWARE.md` l.400–410 pin-naming table says D12 = GPIO12, D13 = GPIO13; CircuitPython `pins.c` and the Adafruit guide say the D12 pad = GPIO4 and the D13 pad = GPIO7 (the LED pin). (2) `docs/hardware/HARDWARE.md` l.442–444 gives SPI on GPIO16 / 18 / 19; the board uses GPIO20 / 22 / 23. (3) `include/rocketchip/board_feather_rp2350.h` l.43–44 pyro pins 12 / 13 are GPIO12 / 13, which are only on the HSTX back connector (pads D2P / D2N), not on the D12 / D13 header pads. This can be intentional, but the comment does not say so.

Third-party PDFs and full texts (datasheets, FCC test reports, Semtech app notes, eCFR, KDB) are not in the repo. The files cite them by URL, FCC ID or document number.

Primary sources win. Living claims stay in `CONFORMANCE.md` / `COVERAGE.md`. These files are research, not published ticks.
