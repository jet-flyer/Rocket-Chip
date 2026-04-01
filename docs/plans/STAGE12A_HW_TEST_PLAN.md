# Stage 12A Hardware Test Plan

**Date:** 2026-03-30
**Boards:** 3 (TX vehicle, RX station Fruit Jam, Relay)
**Prerequisites:** All three firmwares built and flashed

---

## Build Commands

```bash
# 1. TX Vehicle (Feather RP2350)
cmake -B build_tx -G Ninja -DPICO_BOARD=adafruit_feather_rp2350
cmake --build build_tx/

# 2. RX Station (Fruit Jam)
cmake -B build_rx -G Ninja -DPICO_BOARD=adafruit_fruit_jam -DROCKETCHIP_JOB_STATION=1
cmake --build build_rx/

# 3. Relay (which board? — needs user input)
cmake -B build_relay -G Ninja -DPICO_BOARD=<board> -DROCKETCHIP_JOB_RELAY=1
cmake --build build_relay/
```

---

## Test 1: TX → RX Direct (Baseline)

**Purpose:** Verify basic telemetry pipeline still works after Stage 12A extraction.

**Setup:**
- TX board powered, serial connected (COM port)
- RX board (Fruit Jam) powered, serial connected to PC

**Steps:**
1. Power TX board, verify boot banner shows `[PASS] Radio` and `[MODE] TX`
2. Power RX board, verify boot banner shows `[PASS] Radio` and `[MODE] RX — MAVLink output`
3. On RX serial, press `m` to switch to CSV output (easier to read)
4. Wait 10 seconds — should see `RX,<seq>,<rssi>,<snr>` lines appearing at TX rate
5. Press `t` on TX board — verify radio status shows packet count climbing
6. Press `t` on RX board — verify radio status shows `phase=3` (kRxContinuous)
7. Press `r` on TX board — cycle rate 2→5→10→2, verify RX packet rate changes

**Pass criteria:**
- RX receives packets within 5 seconds of TX power-on
- Zero CRC errors over 60 seconds
- Rate cycling works (packet interval changes)
- RSSI reasonable (-40 to -80 dBm at desk range)

---

## Test 2: Fruit Jam Station GPS (IVP-97c Verification)

**Purpose:** Verify GPS detection and status on Fruit Jam with PA1010D connected via STEMMA QT.

**Setup:**
- Fruit Jam (RX station build) with PA1010D GPS on STEMMA QT I2C
- Near a window or outdoors for GPS fix

**Steps:**
1. Power Fruit Jam, verify boot banner shows GPS detected: `[PASS] GPS: PA1010D (I2C)`
2. Wait for GPS fix (LED behavior, typically 35-60s cold start)
3. Press `g` — verify GPS status: fix type, satellites, HDOP, lat/lon/alt
4. Verify lat/lon is plausible (matches your known location)
5. Press `d` — verify station position displayed
6. While TX board is sending: verify `d` shows station position (vehicle distance is stubbed for now — shows "needs received position")

**Pass criteria:**
- GPS detected at boot (I2C probe-first)
- GPS acquires fix within 2 minutes near window
- `g` shows fix_type >= 2, sats >= 4
- Lat/lon within ~50m of known location
- No I2C errors from GPS (verify with `s` status — check I2C error count)

**Failure diagnosis:**
- GPS not detected → check STEMMA QT cable, verify I2C0 (GPIO 20/21) on Fruit Jam
- GPS detected but no fix → LL Entry 31: check I2C bus state (SDA/SCL not stuck LOW). Move closer to window.
- I2C errors → LL Entry 24: 500µs settling delay after GPS reads. Check if `busy_wait_us(500)` is in the GPS read path.

---

## Test 3: Fruit Jam 5-NeoPixel RSSI Bar (IVP-97a Verification)

**Purpose:** Verify per-pixel RSSI visualization on Fruit Jam's 5 NeoPixels.

**Setup:**
- Fruit Jam running RX station firmware
- TX board transmitting

**Steps:**
1. Place TX board next to Fruit Jam (<1m) — expect 5 green pixels (strong signal)
2. Move TX board to ~5m away — expect 3-4 pixels, some yellow
3. Move TX board to another room / behind wall — expect 1-2 pixels, red
4. Power off TX board, wait 5+ seconds — expect all pixels off, pixel 0 dim red pulse (no signal)
5. Power TX board back on — pixels should return to green within a few seconds

**Note:** The RSSI bar calls `ws2812_set_rssi_bar()` from AO_Radio's link quality feedback. This is currently not wired into the AO tick loop — the RSSI bar function exists but the AO_Radio doesn't call it yet. **This test will likely show no RSSI bar activity.** If so, this is expected — wiring the RSSI bar to the AO_Radio RX tick is needed. Flag this as a follow-up if the bar doesn't light up.

**Pass criteria (if wired):**
- 5 green at close range
- Fewer pixels at distance
- Red/off at no signal
- Smooth transitions (no flickering)

---

## Test 4: TX → Relay → RX (3-Node Test)

**Purpose:** Verify relay extends range by forwarding packets.

**Setup:**
- TX board in one room
- Relay board in hallway / doorway (line of sight to both)
- RX board (Fruit Jam) in another room / around corner

**Steps:**
1. Start with TX and RX in same room — verify direct RX works (Test 1)
2. Move RX to another room where direct reception is marginal or lost
3. Verify: without relay, RX shows gaps or no packets
4. Power on relay board in between — verify boot banner shows relay mode
5. Wait 10 seconds — RX should start receiving packets again (relayed)
6. Check relay board LED — should blink blue on each forwarded packet
7. On RX, verify sequence numbers are sequential (no duplicates from relay dedup)
8. Run for 60 seconds — check RX CRC error count (should be 0)

**Pass criteria:**
- RX receives packets through relay that it couldn't receive directly
- Zero CRC errors on relayed packets
- No duplicate sequence numbers
- Relay LED blinks on each forward

**Failure diagnosis:**
- Relay doesn't forward → check CCSDS CRC validation in relay path. Verify TX is sending valid CCSDS (not MAVLink).
- Duplicates on RX → seq dedup not working. Check `g_lastRelaySeq` logic.
- RX still receives without relay → boards are too close. Move RX further away.

---

## Test 5: Station MAVLink → QGC (IVP-61 Regression)

**Purpose:** Verify MAVLink re-encode to QGC still works after Stage 12A refactoring.

**Setup:**
- TX board transmitting
- RX board (Fruit Jam) connected to PC via USB
- QGroundControl running on PC

**Steps:**
1. Power both boards
2. On Fruit Jam serial, press `m` to ensure MAVLink output mode
3. Open QGC, connect to Fruit Jam's COM port (115200 baud)
4. QGC should show: heartbeat received, attitude display, map position (if GPS)
5. Verify attitude updates match TX board orientation (tilt TX board → QGC attitude changes)
6. Run for 2 minutes — verify no "Communication Lost" from QGC

**Pass criteria:**
- QGC connects and shows telemetry within 10 seconds
- Attitude display updates at TX rate
- No "Communication Lost" for 2+ minutes
- Map shows position (if TX board has GPS fix)

**Known issue:** QGC USB CDC buffer timing (IVP-62 blocker). If "Communication Lost" appears after initial connect, this is the pre-existing issue — not a Stage 12A regression. The Fruit Jam LoRa bridge path should work because radio naturally drops old packets.

---

## Test 6: GPS Altitude Cross-Reference (IVP-97c)

**Purpose:** Verify station GPS altitude can be compared with vehicle baro altitude.

**Setup:**
- Both boards at same location (same floor / room)
- Both powered, TX transmitting, RX receiving
- RX (Fruit Jam) has GPS connected

**Steps:**
1. On RX, press `g` — note station GPS altitude (MSL)
2. On TX, press `s` — note vehicle baro altitude (should be relative to launch site)
3. Compare: station GPS alt should be within ~10m of the vehicle's MSL baro reading (they're at the same physical location)
4. Move TX board up one floor (if possible) — vehicle baro alt should increase by ~3m per floor, while station GPS alt stays the same
5. The difference (`vehicle_baro_alt - station_gps_alt`) should approximate AGL

**Pass criteria:**
- Both altitudes are plausible (not zero, not wildly wrong)
- At same location: difference < 15m (GPS vertical accuracy ~5m, baro offset varies)
- If moved vertically: difference changes by approximately the height change

---

## Post-Test Checklist

- [ ] All 6 tests documented with results (pass/fail/notes)
- [ ] Any failures diagnosed and logged
- [ ] Binary sizes recorded for all 3 builds
- [ ] Serial output captures saved for traceability
- [ ] Commit test results to repo if significant findings
