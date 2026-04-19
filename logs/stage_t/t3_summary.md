# IVP-T3 Summary — CCSDS vs MAVLink protocol A/B

**Date:** 2026-04-19
**Plan:** `docs/plans/STAGE_T_RADIO_DIAGNOSTICS.md`
**Gate:** Soft — **BLOCKED** (firmware config variant failed same mode as T2)

## What was attempted

Built and flashed both vehicle and station with
`ROCKETCHIP_STAGE_T3_MAVLINK=ON`, which flips
`kDefaultRocketRadioConfig.protocol` from `kCcsds` to `kMavlink`. Same
LoRa config (SF7/BW125/CR4-5/20 dBm/5 Hz), different over-the-air framing.

Goal: N=30 per protocol, same as T1/T2, compare first-try and CRC-error
rates.

## Result

Station booted on T3 MAVLink build, showed RSSI LED bar green (vehicle
carrier detected at the modem), but `rx_count = 0` — station received
ZERO packets at the decoder layer over the ~1-minute verification window,
same symptom as the T2 firmware-cheat build.

Did not run ack_stress. Rolled both boards back to known-good.

## Same failure mode as T2

T2 station firmware cheat-mode → 0 RX packets.
T3 protocol flip → 0 RX packets.

Both are compile-flag-gated changes to radio-path code, both broke the
station's ability to receive at the decode level while the RF front-end
appeared to still detect vehicle carrier (RSSI present).

Hypothesis (not verified): there's a subtle codegen / ODR / initialization
issue when the compile flag changes code that's on the radio hot path.
Could be:
- `kDefaultRocketRadioConfig` referenced at different translation units
  with inconsistent compile flags (header-only constexpr usually safe,
  but worth checking).
- MAVLink encoder's output packet crossing the 128-byte `RadioTxEvt::buf`
  boundary (encode_heartbeat + encode_attitude can produce ~60 bytes,
  under limit — but worth verifying).
- Modem config re-init timing on first boot with modified flags.
- Something in the SX1276 driver's explicit-header mode handling with
  variable-length MAVLink payloads vs fixed 54-byte CCSDS payloads.

Not root-caused. Debugging deferred — same class of bug as T2, not worth
chasing during the Stage T QA run.

## Impact on the fix council

**T3's value was "does framing affect collision loss rate."** T2 already
established that collision loss is set by the half-duplex window geometry
(TX airtime > RX window), and framing is third-order below that. T3 data
would have been a nice-to-have baseline for post-fix comparison.

Without T3 data the fix council loses the ability to say "we went from
X% MAVLink to Y% MAVLink" post-fix. That's a minor instrumentation
regret, not a missing foundation for the fix decision.

## Gate

**Soft gate — BLOCKED, not skipped.** The code path exists
(`ROCKETCHIP_STAGE_T3_MAVLINK=ON` in CMakeLists) so the test CAN be
re-run if a future session diagnoses the firmware-variant issue.

Both T2 firmware cheat and T3 MAVLink flip should be revisited together
since they look like the same underlying bug.

## Files

- `src/flight_director/mission_profile_data.h` — protocol `#ifdef`
  (OFF by default, has NO effect on default builds).
- `CMakeLists.txt` — `ROCKETCHIP_STAGE_T3_MAVLINK` option.
- No CSVs generated — no run completed.
