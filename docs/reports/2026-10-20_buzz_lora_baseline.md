# LoRa link baseline (2026-10-20)

Author: Buzz (bench run), written up by Hamilton. Purpose: the LoRa side of the FSK pivot no-regression check, metrics per docs/plans/FSK_PIVOT_PLAN_2026-10-08.md section 5.1. Raw vehicle and station logs are archived locally on the bench PC at Documents/rc-bench-archive/baseline_lora_2026-10-20 (stn_raw.log, veh_raw.log, summary.md; not in git). The run script is tracked as scripts/baseline_lora.py, unchanged from this run (ports COM5 vehicle, COM7 station are hardcoded); the FSK run reuses it.

## Single run on flight-3e9c38e2 (SUPERSEDED)

SUPERSEDED by the three-run repeat on flight-c71fcbcf below. This single run is kept as a record. Its figures (16.7 percent uplink loss, SNR 9.8 dB) are from before the repeat; use the three-run table for the baseline.

Measured build flight-3e9c38e2 (identical to main 47005521).
Date: 2026-10-20.
Vehicle flight id: 3e9c38e2 (board-measured).
Vehicle tree: 1e8503b49bf52a070c0f29fb181fc0a66353eefa (board-measured).
Vehicle banner: version 0.16.3-dev, build flight, board Adafruit Feather RP2350 HSTX, air starcom (board-measured).
Station flight id: 3e9c38e2 (board-measured).
Station tree: 1e8503b49bf52a070c0f29fb181fc0a66353eefa (board-measured).
Station banner: version 0.16.3-dev, build flight, board Adafruit Fruit Jam, air starcom (board-measured).

LoRa parameters:
- SF 7 (board-measured, CFG line).
- BW 250 kHz (board-measured, CFG line).
- CR 5 (board-measured, CFG line; 5 means 4/5).
- Nav rate 10 Hz (board-measured, CFG line).
- Station CFG nav 10 Hz, SF 7, BW 250 kHz, CR 5 (board-measured).
- Frequency 915000000 Hz (not exposed by a live register read; firmware init constant kDefaultFreqHz).
- Preamble 8 symbols (not exposed by a live register read; firmware init writes RegPreambleLsb 0x08).
- Nav on-air size 69 bytes (not exposed by firmware counters; source kRadioConfigNavPltuBytes).
- Useful nav payload 51 bytes (not exposed by firmware counters; source kNavSduUserBytes).
- Station report PLTU 14 bytes (not exposed by firmware counters; source byte_pump kReportPltuBytes).

Power:
- Vehicle CFG pwr 2 dBm (board-measured).
- Vehicle PA register: 2 dBm (reg) (board-measured).
- Station CFG pwr 2 dBm (board-measured).
- Station PA register: 2 dBm (reg) (board-measured).
- TX_POWER catalog class is locked. No runtime CLI sets power alone. Power was not changed.
- The debug HW line prints a fixed '20dBm' string. That string is not the PA register.

Distance: not measured. Layout (Nathan, 2026-10-20; photo kept locally on the bench PC outside the repo, not committed): the station Fruit Jam sits on top of the PC tower with its antenna near the edge; the vehicle board is on the floor below, in a parts bin. Spacing about 1-1.5 m on a diagonal, with partial blockage from the case edge (Buzz estimate from the photo, confirmed by Nathan; not tape-measured). Antenna orientation not recorded; the FSK run must use the same placement.

Vehicle counter span: 300.201 s (board-measured, RATE window_ms end minus start).
Station counter span: 300.249 s (board-measured, RATE window_ms end minus start).
Pad log span: 302.1 s (board-measured, SOAK_START to SOAK_END).
RATE dumps in that window: vehicle 6, station 6 (board-measured).

## Frame delivery

Vehicle TX sent delta: 3055 (board-measured).
The TX sent field is tx_count. It increments on TX done.
Vehicle tx_start delta: 3055 (board-measured).
Vehicle tx_done delta: 3055 (board-measured).
Station RX pkts delta: 3034 (board-measured).
Station rx_count increments on a non-empty radio packet. A LoRa CRC fail with an empty FIFO does not increment it.
Station CRC err delta from the RX status line: 1 (board-measured).
Station RATE rx_crc_ok delta: 3034 (board-measured).
Station RATE rx_crc_fail delta: 0 (board-measured).
On this build the station RATE rx_crc_ok counter moved with RX pkts. The CLI CRC err field is a different counter. Both are reported.
The status-line seq field stayed 0. Nav sequence used for gaps is the pad Seq field.
Missing radio frames (TX sent minus station RX): 21 (computed).
Frame loss rate: 0.687 percent (computed).
Dashboard frames with a new Pkts value: 2971 (board-measured).
Seq steps compared: 2604 (board-measured).
Longest run of missed seq numbers: 1 (computed from dashboard Seq, modulo 16384).
Total missed seq numbers in that series: 3 (computed).
A missed run is the gap between successive distinct Seq values, minus one.
Same-seq paints with a new Pkts value are extra radio frames. They are not nav-seq gaps.
Idle pad redraws that repeat Pkts are not extra frames.
Vehicle COP-P at last soak line: lock  N(R)=0 V(S)=0 V(R)=0  MAC A/s50 (board-measured).
Station COP-P at last soak line: lock  N(R)=0 V(S)=0 V(R)=0  MAC A/s60  nav (board-measured).

## TX faults

tx_busy_drop delta: 0 (board-measured).
tx_hold_replace delta: 0 (board-measured).
TX fail field at the last soak status: 0 (board-measured).
That fail field is tx_consec_fail. It is a consecutive count. It is not a cumulative fault total.
A cumulative TX-timeout counter is not exposed by firmware.
tx_start minus tx_done over the window: 0 (computed).
A positive value is in-flight work or a start that did not finish. It is not a named fault counter.

## Signal

RSSI n=2971 mean=-48.52 dBm min=-50 dBm max=-46 dBm sample stdev=0.65 dB (board-measured samples, computed stats).
Each sample is one pad paint with a new Pkts value. The firmware prints RSSI as an integer dBm.
SNR n=2971 mean=9.80 dB min=7 dB max=13 dB sample stdev=0.52 dB (board-measured samples, computed stats).

## Air time per delivered payload byte

Nav on-air time: 64128 us (computed, Semtech SX1276 §4.1.1.6, same integer form as rfm95w_airtime_us).
TX-start and TX-done are counters. They are not timestamps. Measured air time is not exposed by firmware.
Useful bytes per nav frame: 51 (firmware constant, not a live counter).
Air time per delivered payload byte: 1257.41 us/byte (computed).
The quotient uses whole nav on-air time, including preamble, header, and CRC, divided by the 51-byte nav user payload.
Station report on-air time: 23168 us for 14 bytes (computed, same formula).

## Throughput and rates

Time on air per nav frame: 64.128 ms (computed).
Time on air per station report frame: 23.168 ms (computed).
Payload bytes/s: 439.81 (computed).
That rate is observed distinct Seq paints times 51 bytes, divided by the pad log span.
Offered telemetry rate from nav_submit delta: 8.628 Hz (computed over the vehicle counter span).
Configured nav: 10 Hz (board-measured).
nav_submit delta: 2590 (board-measured).
pltu_post delta: 3055 (board-measured).
Delivered telemetry rate from distinct Seq paints: 8.624 Hz (computed).
Vehicle TX rate: 10.177 Hz (computed from TX sent over the vehicle counter span).
Delivered radio-packet rate from station RX delta: 10.105 Hz (computed).
Station TX rate: 1.455 Hz (computed from station_tx delta).
station_tx delta: 437 (board-measured).
Vehicle RX pkts delta: 364 (board-measured).
Vehicle RX CRC err delta: 1 (board-measured).
Uplink frames not in the vehicle RX counter: 73 (computed, station_tx minus vehicle RX).
Uplink loss rate: 16.70 percent (computed).
The two snapshots are about 0.4 s apart. That skew is a few frames, not this whole gap.
Vehicle duty cycle: 65.26 percent (computed).
Duty uses TX sent delta times computed nav on-air time, divided by the vehicle counter span.
Station duty cycle: 3.37 percent (computed).
Station duty uses station_tx delta times computed 14-byte report on-air time.
A station frame that is not a 14-byte report makes this duty low or high.
Sum of the two duty figures: 68.63 percent (computed).
The sum is channel occupancy only if the two transmissions do not overlap.

## Turnaround and first frame

Key-up to first good frame on a cold start was not a separate test. The link was already up.
First station packet after the vehicle CLI reboot stamp: 0.069 s (board-measured).
That first packet is still the pre-reboot sequence. It was already on the air.
The first frame after the vehicle came back is in Recovery.
RX window miss rate: not exposed by firmware.
The pad RX Hz figure is a 2 s sliding count of rx_crc_ok. It is not a miss counter.
Half-duplex switch time is not exposed by firmware.

## Command path

No command was sent on this run. ARM was not sent.
Command ACK time: not exercised. The pad can print ACK <ms> when a command completes.
Command resend count: not exercised.
No [CMD] line was in the station log.
COP-P retransmit counters for the command path are not a LoRa baseline target. FSK names them.
Station diag printed CmdRetryStats. See stn_raw.log. Rows with a zero sent count are omitted by the firmware.

## Recovery

The vehicle got one CLI soft reboot. The key is debug-menu k, after q.
The station was not rebooted.
Time from the REBOOT_K stamp until the station showed packets again: 4.100 s (board-measured).
Idle 1 Hz repaints of the last packet are not a resume.
The resume sample is the next Pkts increase after a silence of at least 1 s.
First RX after the stamp, including a frame already on the air: 0.069 s (board-measured).
Resume Pkts=22903 Seq=0 (board-measured).

## Board cost

Radio interrupt time: not exposed by firmware.
CPU time in the radio path: not exposed by firmware.
Flash-write stall: not exposed by firmware. This run did not flush the log.
Vehicle AO_Radio queue depth=33 use=0 high=7 (board-measured, diag).
Station AO_Radio queue depth=33 use=0 high=2 (board-measured, diag).
Vehicle MSP depth=0 bytes (board-measured, diag). This is stack, not radio CPU time.
Station MSP depth=0 bytes (board-measured, diag). This is stack, not radio CPU time.
Vehicle rc_log dropped=0 bytes high_water=1540 bytes (board-measured).
Station rc_log dropped=0 bytes high_water=1726 bytes (board-measured).

## FSK only: sync

N/A for LoRa. LoRa has no ASM. This baseline does not count ASM misses.
COP-P loss of synchronization is not a separate counter in this firmware.

## Metrics that need new firmware counters

These are the holes an FSK comparison will hit if the firmware stays as it is.
- Measured time on air. TX-start and TX-done are counts, not a pair of timestamps.
- Cumulative TX failures. The CLI fail field is consecutive only.
- One CRC counter. On this run the CRC err field and RATE rx_crc_fail did not move together.
- Longest drop run inside the firmware, so the host does not infer it from pad paints.
- RX window misses. The 2 s RX Hz display is not a miss counter.
- Half-duplex key-up to first good frame, as a timer.
- Command ACK time and resend count as a retained counter. The pad holds ACK text for about 3 s, then clears it.
- Radio interrupt time and CPU time in the radio path.
- Flash-write stall time.
- A live readback of frequency and preamble length. Both are init constants today.
- ASM miss count and COP-P loss-of-sync count. LoRa does not use ASM. FSK needs both, kept apart from CRC fails.

## Notes

- **Superseded by the three-run repeat below. Inference (Goddard, not measured):** the 16.7 percent uplink loss is likely half-duplex scheduling, not RF. The vehicle transmits about 65 percent of the time, and station frames sent during vehicle TX are lost; SNR (9.8 dB) is not the limit. Next run should log each uplink loss against vehicle TX state. FSK's shorter air time may improve uplink for this reason alone.

## Three-run repeat on flight-c71fcbcf

This section is three later soaks. The section above is the single flight-3e9c38e2 run. The numbers come from the local summaries `Documents/rc-bench-archive/baseline_lora_2026-10-20/run1/summary.md`, `run2/summary.md`, and `run3/summary.md`. Raw logs stay on the bench PC. They are not in this repo.

Both boards ran firmware flight-c71fcbcf (board-measured). Both boards reported tree 16d5da71e7b3317c3294c3395f32e3bedd4c7928 (board-measured). The branch is grok/t1-0-counters. That firmware commit is not on main.

Each run used the same radio settings (board-measured CFG). SF 7. BW 250 kHz. CR 5, which means 4/5. Nav rate 10 Hz. The vehicle PA readback was 2 dBm. The station PA readback was 2 dBm. Frequency is 915 MHz. That value is the init constant kDefaultFreqHz.

Each soak is 300 s. The three runs are back to back on 2026-10-20, about 10:59-11:16 PM CDT. Station log stamps, CDT: run 1 SOAK_START 22:59:59.100 and SOAK_END 23:05:01.214. Run 2 SOAK_START 23:06:07.638 and SOAK_END 23:11:09.679. Run 3 SOAK_START 23:12:00.514 and SOAK_END 23:17:02.620.

Layout: unchanged from bench photo (photo kept local, not in repo).

These three runs finished before the DIO jumpers were soldered (operator note).

Rate figures use the RATE counter window (board-measured). Vehicle windows: 300.103 s, 300.010 s, 300.104 s. Station windows: 300.150 s, 300.050 s, 300.149 s. Pad log spans: 302.1 s, 301.9 s, 302.0 s (board-measured).

Spread in the table is the maximum minus the minimum (computed). Counts, LINK fields, air_ms, air_bytes, station_tx, copp_rtx, and the recovery time are board-measured. Rates, loss percents, delivery percents, payload bytes/s, RSSI stats, SNR stats, duty cycle, and the air-time quotient are computed.

Station LINK delivered equals station RX pkts on each run (board-measured). Downlink delivery is station RX divided by vehicle TX sent (computed).

| Metric | Run 1 | Run 2 | Run 3 | Spread |
| --- | ---: | ---: | ---: | ---: |
| Vehicle TX sent | 3056 | 3061 | 3066 | 10 |
| Station LINK delivered | 2991 | 2985 | 3026 | 41 |
| Station LINK missing | 28 | 41 | 27 | 14 |
| Station LINK crc_fail | 29 | 46 | 25 | 21 |
| Downlink delivery % | 97.87 | 97.52 | 98.70 | 1.18 |
| Frame loss % | 2.127 | 2.483 | 1.305 | 1.178 |
| Longest missed run | 1 | 1 | 1 | 0 |
| Vehicle LINK delivered | 387 | 378 | 361 | 26 |
| Vehicle LINK missing | 0 | 0 | 0 | 0 |
| Vehicle LINK crc_fail | 0 | 0 | 0 | 0 |
| station_tx | 459 | 446 | 439 | 20 |
| Uplink loss % | 15.69 | 15.25 | 17.77 | 2.52 |
| RSSI mean dBm | -46.52 | -46.77 | -46.32 | 0.45 |
| RSSI min dBm | -49 | -49 | -48 | 1 |
| RSSI max dBm | -44 | -45 | -44 | 1 |
| RSSI sample stdev dB | 0.69 | 0.64 | 0.60 | 0.09 |
| SNR mean dB | 9.56 | 9.54 | 9.49 | 0.07 |
| SNR min dB | 4 | 5 | 5 | 1 |
| SNR max dB | 13 | 13 | 13 | 0 |
| Nav offered Hz | 8.634 | 8.653 | 8.660 | 0.026 |
| Nav delivered Hz | 8.543 | 8.524 | 8.579 | 0.055 |
| Vehicle TX rate Hz | 10.183 | 10.203 | 10.216 | 0.033 |
| Station TX rate Hz | 1.529 | 1.486 | 1.463 | 0.066 |
| Payload bytes/s | 435.72 | 434.70 | 437.51 | 2.81 |
| Vehicle air_ms | 178199 | 179093 | 179577 | 1378 |
| Vehicle air_bytes | 186994 | 188109 | 188674 | 1680 |
| Station air_ms | 10634 | 10333 | 10171 | 463 |
| Station air_bytes | 6426 | 6244 | 6146 | 280 |
| Vehicle duty cycle % | 65.30 | 65.43 | 65.52 | 0.22 |
| Station duty cycle % | 3.54 | 3.44 | 3.39 | 0.15 |
| Vehicle copp_rtx | 0 | 0 | 0 | 0 |
| Station copp_rtx | 0 | 0 | 0 | 0 |
| Recovery after vehicle reboot s | 3.794 | 3.404 | 4.171 | 0.767 |
| Air time us/byte | 1257.41 | 1257.41 | 1257.41 | 0 |

RSSI and SNR use pad paints that have a new Pkts value. Sample counts are 2920, 2927, and 2943. Air time per delivered payload byte is 1257.41 us/byte on each run (computed). The quotient is the whole nav on-air time, 64128 us, divided by the 51-byte nav user payload.

The vehicle got one CLI soft reboot on each run. The station stayed up. Recovery time is the gap from the REBOOT_K stamp until the station Pkts count rose after a silence of at least 1 s (board-measured).

### Uplink loss

Vehicle LINK crc_fail is 0 on each run (board-measured). Vehicle RX CRC err is 0 on each run (board-measured). Uplink loss is about 15 to 18 percent (computed). The three values are 15.69 percent, 15.25 percent, and 17.77 percent.

INFERENCE: the uplink loss is likely half-duplex. The vehicle transmits about 65 percent of the time. Station frames can arrive while the vehicle transmits. No RX-window-miss counter exists to confirm that cause.

INFERENCE (Duke, not checked in station code): uplink lost 15 to 18 percent while COP-P logged 0 retransmits. COP-P (CCSDS 211.0-B-6) only retransmits sequence-controlled frames, so the lost uplink frames were likely sent expedited. Confirm the station uplink frame type before FSK work.

### CRC counters

LINK crc_fail equals the old CLI CRC err field (board-measured). The three values are 29, 46, and 25. RATE rx_crc_fail stays 0 on each run (board-measured).

### copp_sync_loss

copp_sync_loss: not measured (sync timeout disabled). The soak MIB keeps the SYNCH timer from expiring.

### Not measured

- Flash-write stall. These runs did not flush the log.
- Command ACK time. No command was sent.
- Command resends. No command was sent.
- RX window misses.
- Half-duplex switch time.
- Radio IRQ time.
- CPU time in the radio path.
- Live frequency readback.
- Live preamble readback.

### Script notes

The summary line "The expected build was flight-3e9c38e2" is a stale script constant. The boards reported flight-c71fcbcf (board-measured).

The summary line "Distance: unknown" is present because the script has no bench note.

Run 1 was started by `baseline_lora.py --help` (operator note). The script ignores that argument. Only `--analyze-only` selects another path. Run 1 was a valid full run. The station log has SOAK_START, a 300 s soak, SOAK_END, and the vehicle reboot.
