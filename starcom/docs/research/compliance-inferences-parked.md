# Compliance inferences parked

Companion to `/workspace/out/compliance-record-draft.md`.
Prepared 2026-10-06 (CT). These sentences were removed from the compliance draft because they are inferences, not facts.
A fact needs a quoted book clause (section + page) or a file path + line.
None of these is decided. They are not part of the compliance record.

| ID | Exact sentence removed | Why it was an inference | Check that could turn it into a fact |
|---|---|---|---|
| X04 | INFERENCE: this counter is not the 133.0 service parameter. | Interpretive leap: equates BP rx_lost with / against the 133.0 service parameter without a quote that defines the relationship. | Code check vs 133.0-B-2 §3.3.2.3 / §3.4.2.4. |
| X05 | LoRa packet bearer (INFERENCE: a packet radio has no continuous symbol stream to fill). | No datasheet quote that LoRa has no continuous stream between packets. | Datasheet / bench of LoRa packet mode. |
| X05 | INFERENCE: on the FSK link in continuous mode, that phase is a continuous symbol stream, so idle fill between PLTUs applies. | Maps book data-services phase to FSK continuous mode without a quote equating them. | State FSK link mode; then apply §3.3.4.2.2. |
| X24 | Yes (INFERENCE): each half-duplex turn ends a transmission, and the FSK receiver must finish the last PLTU. | Assumes FSK turn end needs Tail; book does not say that for FSK. | Bench Tail after last PLTU. |
| X12 | chip (INFERENCE: the LoRa radio cannot use the 211.1 rates and channels) | Reason not stated in repo. | Find stated reason or use not stated. |
| X13 | Yes. INFERENCE: the LoRa catalog index has no FSK meaning, so the catalog changes with FSK. | No FSK catalog table cited. | Publish FSK catalog or state unused. |
| X18 | chip (INFERENCE: one transceiver at each end) | Reason not stated in repo. | Cite stated reason. |
| X19 | INFERENCE: the air path uses COP-P (BP), and COP-1 runs on the host loop. | Split not proven by a single cite. | Trace COP-1 vs COP-P callers. |
| X21 | INFERENCE: 235.1-R-1 E2.2.10 NOTE 2 names suppressed-carrier modulation; whether it bears on uncoded FSK is open. | Applies suppressed-carrier note to FSK without a book sentence that does so. | Design call; books silent on FSK+that NOTE. |
| X23 | INFERENCE (Goddard row 4): transfer across the channel (FARM-P acceptance) is not command execution, so the application ACK serves a different need. | Equates FARM-P acceptance with not command execution without a book quote. | Design/ops statement. |
| P01 | INFERENCE: no 211.1 category covers FSK transmit. | Generalizes from E2d TX=PSK; book silent on an FSK-TX category in those words. | Keep Table 3-1 fact; design for FSK TX category. |
| P02 | INFERENCE: the point still applies to a Part 15 command link. | 880.0-G-3 examples are 2.4/5 GHz; applying to 902–928 is a leap. | None as fact. |
| P02 | INFERENCE: 401 covers agency bands. | Negative search turned into a scope claim. | 401.0-B-32 scope section. |
| P05 | INFERENCE (Duke GOY 2.5): that option is FSK receive on an E2d element, not an FSK transmit waveform. | Reading beyond Table 3-1 / fn 2 quotes. | Keep quotes only. |
| P06 | chip (INFERENCE: no repo file states the reason) | chip without stated reason. | Use not stated. |
| P07 | chip (INFERENCE: no repo file states the reason) | chip without stated reason. | Use not stated. |
| P08 | INFERENCE: the Rssi flag can cover part of the need (energy only). | Coverage claim without measurement. | FSK bench. |
| P09 | INFERENCE: PreambleDetect is close to symbol lock. It is not the same signal. | Equates flags without measurement. | FSK bench. |
| P09 | PreambleDetect may stand in (INFERENCE). | Same. | FSK bench. |
| P10 | INFERENCE: the LoRa basis for 10 ms goes away with FSK. | Assumes FSK drops LoRa-derived value. | FSK sitting picks the value. |
| M01 | INFERENCE (Duke GOY 2.20): neither book gives a hobby path. | Negative search stated as a rule. | Keep as books do not say. |
| M01 | INFERENCE: the FSK link keeps Version-3 frames with SCIDs 1 and 2. | Assumes FSK keeps SCIDs. | Design statement. |
| X22-design | INFERENCE: these do not conflict (Version-3 vs Version-4 frame). | Harmonizes 355.0 / 350.0 / 700.1 without a book sentence that reconciles them. | Frame-version design decision. |
| X22-design | INFERENCE: so the WANTED item needs a frame-version decision (open item 31). | Depends on the conflict inference. | Same. |
| R02 | INFERENCE: that the raw field counts as a "Time Code Field" and not as mission-specific ancillary data. | Classifies the legacy field without a book award of that class. | Decide field class vs 133.0 §4.1.4.2.2.2. |
| R02 | INFERENCE (risk): a PICS pass reads dead code as live (the OreSat UPID docs-vs-code pattern, survey Room fact 6). | Risk assessment, not a measured fact in this repo. | PICS pass process. |
| R03 | INFERENCE (not bench-tested): on a PLTU, octets 2–3 are the last ASM octet (0x20) and the first Version-3 header octet, so the relay sees the same "seq" on most frames and drops them as duplicates. | Layout reading not bench-tested. | Bench relay on PLTU stream. |
| R03 | INFERENCE: the CLI `lost` count does not update under Starcom. | Not measured. | Run CLI under Starcom. |
| R06 | INFERENCE (Goddard): the dashboard `retries_used` and the retry-stats table always show zero retries. | Goddard inference, not measured here. | Dashboard check. |
| R07 | INFERENCE (Goddard): cleanup only, not a compliance item. | Assessment of compliance relevance. | None; cleanup task. |
| R09 | INFERENCE: if we enable it, it is a PHY choice outside the book and needs its own row. | Policy leap from datasheet mismatch. | If whitening enabled, add a row (fact of enablement). |
| R10 | INFERENCE: they could replace the `pump_tick` gap timer. | Capability assessment without bench. | FSK bench. |
| R10 | INFERENCE: the replacement. | Same. | FSK bench. |
| R11 | INFERENCE: the LoRa reason goes stale at the FSK pivot. | Staleness prediction. | FSK sitting. |
| R12 | INFERENCE: the LoRa basis goes stale at the FSK pivot. | Same. | FSK sitting. |
| R15 | INFERENCE: a raw `memcpy` layout meets this clause, so this is not a deviation. | Applies §4.1.4.3.4 to memcpy layout. | Mission field-map decision. |
| R15 | INFERENCE (risk): drift between boards or compilers. | Risk. | ABI check. |
| R15 | INFERENCE: not a deviation. | Same as first. | Same. |
| W6 | INFERENCE: not our band | 401 scope leap. | 401 scope section. |
| W9 | P05 (noncoherent FSK: INFERENCE) | Assumes SX1276 FSK is noncoherent without datasheet cite here. | Datasheet detection mode. |
| W2/X21 | INFERENCE (X21, W2): the 130.1-G-3 four-point omit test, written for the TM link and for PM systems, needs its own check on the SX1276 FSK link. | Applies a TM/TC Green Book omit test to Prox-1 FSK without a Prox-1 quote requiring that test. | Optional bench only; Prox-1 rule remains 211.2-B-3 §3.4.5. |
| FSK-column | INFERENCE (FSK pivot impact column, all rows): each "Yes" or "No" is my assessment unless the cell quotes a source. | Column assessments without a book or file cite. | Keep "Yes" only when the design brief / row source says the FSK sitting touches the item. |
| R02-risk | INFERENCE (risk): a PICS pass reads dead code as live (the OreSat UPID docs-vs-code pattern, survey Room fact 6). | Risk assessment, not measured in this repo. | PICS pass process excludes host-only codecs or documents them. |
| R15-risk | INFERENCE (risk): drift between boards or compilers. | Risk, not measured. | ABI / endian check across boards. |
| R09-whitening | INFERENCE: if we enable it, it is a PHY choice outside the book and needs its own row. | Policy leap from datasheet ≠ book. | If whitening is enabled, add an exception row (fact of enablement). |

Total parked: 45.
