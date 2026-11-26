# FSK Pivot Plan: Addendum 2026-11-25

This file adds items to `FSK_PIVOT_PLAN_2026-10-08.md`. The plan stays frozen (Nathan, 2026-11-25). Do not edit the plan. Add new items here.

Rules: 211.0-B-6, 211.1-B-4 and 211.2-B-3 govern. **[draft]** marks a Pink sheet or Red Book. "Inference" marks a statement that nobody has measured or found in a source. Each item points to the place that holds the detail.

## A1. Carrier acquisition on each turn

- Book: CARRIER_ACQUIRED (211.1-B-4 §3.2.3.3) and SYMBOL_INLOCK_STATUS (211.1-B-4 §3.2.3.4). Table 3-2 gives the carrier-only state (note 2, Carrier_Only_Duration) and the acquisition sequence (note 3).
- Mapping (Duke, Goddard, 2026-11-25): preamble detect stands in for CARRIER_ACQUIRED. Sync-word or valid-header detect stands in for SYMBOL_INLOCK_STATUS. The register map for FSK and LoRa is in the proposed update of the CARRIER_LOSS_TIMER row, `starcom/docs/CONFORMANCE.md` (proposal waits for Nathan).
- Code fact (branch `grok/r37-commchange`, 2ee912d6): the driver reads only RxDone and CRC error (`src/drivers/rfm95w.cpp`). CARRIER_ACQUIRED is a software flag. A decoded frame sets it. It clears after `tail_idle_duration` (10 ms) with no decoded frame (`byte_pump.cpp`, `pump_tick`).
- Inference (not tested): in half duplex, the carrier-loss timer can re-arm after each frame.
- Plan links: N2 (the RFM95 cannot send the plain carrier that book hailing needs), T3-B (book PHY features), K5 and the PD3 note ("The FSK code must define" the carrier-only tick), D8 (chip preamble).

## A2. Real carrier-only tick on the SX1276

- Open check. Goddard (not verified): FSK mode with zero frequency deviation, or continuous mode, can possibly send an unmodulated carrier. LoRa mode cannot, because its preamble is chirps.
- Open check: whether a pure tone fits FCC 15.247 (6 dB bandwidth of 500 kHz minimum for digital modulation) or must use 15.249 limits. This is from memory. Check it against the current eCFR text.
- Result now: the preamble mapping in A1 applies. A real carrier-only tick is optional.

## A3. Half-duplex hold-off after a missed turn

- Plan K5 has E48 and E50 (Receive_Duration, 211.0-B-6 §6.2.4.18).
- The plan does not have the chain after that (211.0-B-6 Table 6-10 to 6-12):
  - Station: Maximum_Failed_Token_Passes, then E83 (S50 to S80), then E84 (S80 to S11) after Reconnect_Wait_Duration.
  - Vehicle: CARRIER_LOSS_TIMER, then E85 (S60 to S2).
- Detail: `starcom/docs/PRIOR_ART.md` section 7.

## A4. Rate change in one direction

- SET TRANSMITTER PARAMETERS (211.0-B-6 §6.3.3.2.2) and SET RECEIVER PARAMETERS (§6.3.3.2.3) are separate directives. Local COMM_CHANGE changes "one or more communications channel parameters" (§6.3.3.1.3). A change of the downlink rate only fits this text.
- Open: the map from our presets to the rate codes (211.0-B-6 §B1.2.6). Precedent: Odyssey (Annex F, F4.2.4).
- 211.1-P-4.2 **[draft]** §5.1.7.1 allows any coded symbol rate from 1000 to 4096000 symbols/s (PICS 6.2 says 4000000 for return; the drafts disagree). Duke: its note sends the exact rate through "Annex E of reference [3]". Duke has not read that annex yet. Check it before the rate-code map.
- Detail: `starcom/docs/PRIOR_ART.md` sections 5 and 6. Plan links: D2, D13.

## A5. Variable-length SPDU header

- Problem report R-38 in `docs/PROBLEM_REPORTS.md` (branch `grok/r37-commchange`). Deferred to FSK planning because it changes the bytes on air.

## A6. Lunar GMSK **[draft]**

- In 211.1-P-4.2 **[draft]**, GMSK is a data mode (§5.1.6.3.1). The hail is filtered PCM/PM/Bi-Phase-L with a residual carrier: §5.1.6.2.4 **[draft]** says it "must" be used for hailing, and the book treats "must" as binding (Duke, 2026-11-25). It is required, per §5.1.6.2.4 **[draft]**. Hail modulation is mandatory; coding and rate are recommended: §5.1.2.4 **[draft]** "should" recommends only LDPC 1/2 K=1024 and 2000 sym/s. This agrees with §1.5.2.1 **[draft]**, and the Mars section §4.1.2.2 **[draft]** says the same with "shall" (Goddard, 2026-11-25). So a GMSK-only radio does not meet the draft hail requirement. On the RFM95, with or without the Forgix FPGA, the hail is a stated deviation: the chip modulates only frequency (FSK, OOK, LoRa) and has no PM (plan N2). Manchester (`DcFree` in RegPacketConfig1) on FSK is not PCM/PM/Bi-Phase-L. A book hail needs an I/Q radio path (plan T3-B).
- A "shall" in S-band: the hailing channel is distinct from the working channel (§5.1.2.2 **[draft]**).
- No conflict with 235.1-R-1 Annex K **[draft]** (informative, Table K-2). Its GMSK rows are fields of the link establishment directive (bits 16-19 modulation, 48-63 symbol rate, 64-95 frequency). They give the requested working-link configuration, not the hail waveform (Duke, 2026-11-25). Both books are drafts.
- The SX1276 Gaussian filter has BT 0.3, not 0.25. Plan D4 and test S17 already cover this test.
- The draft GMSK also needs pre-coding "as shown in Figure 5-2" (GMSK Precoder, §5.1.6.3.1). The plan does not have this. Inference (Goddard): firmware can do the pre-coding before bytes go into the FIFO.
- Suppressed carrier still needs carrier acquisition (A1).
- Detail: `starcom/docs/research/prox1-only-mac-and-lunar.md` Q2.1a and `ots-gmsk-bt-lunar.md`.

## A7. RC link-loss logic that overlaps the MAC carrier-loss timer

The Starcom MAC carrier-loss timer (211.0-B-6 §6.3.1.3) tells when the partner is no longer heard. Rocket Chip has older timers that do the same check on the age of the last received frame. Found on main `0b4fcfc0`:

| Code | What it does | Overlap |
|------|--------------|---------|
| `src/active_objects/ao_radio.cpp` `handle_link_quality` | Last-RX age: 2 s gap, 5 s lost. Publishes `SIG_RADIO_STATUS`. | Loss detection: same job as the MAC timer. |
| `src/safety/rf_link_health.h` + `ao_rf_manager.cpp` (Stage T, IVP-T14, `docs/plans/STAGE_T_T14_DESIGN.md`) | 4-state link machine (ACQ, TENTATIVE, TRACK, DEGRADED). Forced ACQ at 2 s and 5 missed frames. LQ sliding window. Anchor deadman `max(500 ms, 5 x nav period)`. | Loss detection and the ACQ/TRACK states overlap the MAC session states. LQ and the anchor deadman have no MAC equivalent. |
| `src/active_objects/station_bar_mode.h` | Station LED bar from the raw last-RX gap and the MAC mode. | Partly on the MAC already. |
| `src/safety/fault_inject.cpp` (R-9b) | Forces `last_rx_ms` to 0 to inject link loss. | Must inject at the MAC signal after the change. |
| CLI and dashboard (`rc_os_commands.cpp`, `rc_os_dashboard.cpp`) | Show last-RX age. | Display only. Read from one source. |

- Boundary from the book (Duke, 2026-11-25): the protocol owns the timers and reports to the vehicle controller. 211.0-B-6 Annex D gives notification 9 (CARRIER_LOSS_TIMER underflows) and notification 5 (no carrier received in half duplex). §6.3.1.3.1 gives the decision to end the session or reconnect to the vehicle controller. So the loss timing goes into Starcom. The RC link-lost latch and the `vehicle_lost` intent stay in RC, and those notifications trigger them, not a separate clock on `last_rx_ms`. Result: one timer, one threshold, and the safety decision stays in RC.
- Check before any cut (Goddard inference, not tested): the anchor deadman and anchor filter may also drive TDMA slot timing. Find which parts are still live next to the Starcom MAC.
- Do this in the driver rewrite, not in the R-37 fix. The Stage T values came from a Council design round, so any removal is proposed first.
