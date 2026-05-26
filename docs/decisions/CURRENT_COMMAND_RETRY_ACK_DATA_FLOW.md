# Current Command / Retry / ACK Data Flow (Pre-CCSDS Analysis)

**Purpose:** Precise reference map of how the current STOP-GAP command delivery + reliability mechanism works. This is the baseline for any future CCSDS TC-layer + COP-1 (or equivalent) rework.

**Date:** 2026-05 (analysis performed during session)
**Scope:** Station-initiated tracked commands (ARM, DISARM, ABORT, SET_RADIO_CONFIG, QUERY) through to vehicle dispatch and ACK round-trip.
**Status:** Work in progress — tracing + diagram generation active.

**Key Artifacts:**
- This document (textual map with citations)
- `docs/decisions/CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot` (Graphviz visualization matching project conventions in `docs/audits/cla_rbm/dot/`)

---

## High-Level Architecture

Commands originate on the **station** (via CLI dashboard or GCS MAVLink) and are sent over the half-duplex LoRa link using MAVLink `COMMAND_LONG` frames (with the `confirmation` field carrying a sequence number for tracking).

The reliability layer is a custom retry + ACK protocol layered on top of MAVLink + a small CCSDS APID 0x003 ACK packet for the return path.

Major Active Objects involved:
- `AO_Telemetry` (station side owns retry state machine and ACK tracking)
- `AO_Radio` (TX/RX scheduling + RadioScheduler)
- `AO_RfManager` (link state, anchor timing, `ok_to_retry()` gate)
- Vehicle `AO_Telemetry` (decode + dispatch)
- `AO_FlightDirector` (actual command execution + ACK result decision)

---

## Textual Data Flow Map (with citations)

### 1. Station-Side Command Origination

**Paths into tracked commands:**

- CLI dashboard (`src/cli/rc_os_dashboard.cpp`) or direct keys → calls `AO_Telemetry_send_tracked_command(...)`
- GCS MAVLink path (via `AO_Telemetry_feed_usb_byte`) can also trigger tracked commands in some cases.

**Core state (station side, in `ao_telemetry.cpp`):**
- `s_pending_cmd` struct (lines 362-373): holds `pending`, `seq`, `cmd_id`, `sent_ms`, `retries_left`, and all 5 parameters (p1..p5) for replay.
- `s_last_cmd_result` (lines 378-384): latched result for dashboard "hold" display.
- `s_retry_stats[]` (per `CmdClass`) for instrumentation (T14b).

**Entry point:**
```cpp
// src/active_objects/ao_telemetry.cpp:94
void AO_Telemetry_send_tracked_command(uint16_t command, float p1, ... p5)
```

(See implementation around line 440+ for initial send + state setup.)

### 2. Transmission + Retry Loop (Station)

- First send happens immediately in `send_tracked_command`.
- Retry timer is driven externally by `AO_Telemetry_cmd_retry_tick(uint32_t now_ms)` called from the station's 10 Hz tick path.
- Timeout is dynamic (set via `AO_Telemetry_set_ack_retry_timeout_ms` from airtime calculation in AO_Radio).

**Retry logic (key function):**
- `AO_Telemetry_cmd_retry_tick` (lines 1028+)
- If timeout elapsed and `retries_left > 0`: decrement and call `resend_pending_cmd()`
- `resend_pending_cmd()` (lines 1001+): repacks the exact same MAVLink `COMMAND_LONG` using the cached params and sequence number, then posts `SIG_RADIO_TX` to `AO_Radio`.

**Instrumentation:**
- Every send/retry/ACK/fail updates `s_retry_stats[cls]`.

### 3. Radio Path (Station → Vehicle)

- `AO_Telemetry` posts `SIG_RADIO_TX` with the MAVLink frame.
- `AO_Radio` handles actual LoRa transmission (subject to `RadioScheduler` / half-duplex windows).
- `AO_RfManager` provides `AO_RfManager_ok_to_retry()` and anchor timing to avoid collisions with vehicle nav packets.

### 4. Vehicle-Side Reception & Dispatch

- `AO_Radio` receives packet → posts `SIG_RADIO_RX` to `AO_Telemetry`.
- In `handle_rx_packet` / MAVLink path (around line 336+):
  - Decode `COMMAND_LONG`
  - Call `dispatch_command(...)` (line 275)
  - `dispatch_command` switches on `cmd.command` and calls into Flight Director signals or direct handlers (ARM/DISARM/ABORT via `SIG_*`, config via dedicated paths).
- Result is captured and `stage_cmd_ack(...)` is called (line 311).

**ACK staging (vehicle side):**
- `s_pending_ack` + `s_pending_ack_valid` (lines 162-164)
- `stage_cmd_ack` populates `CommandAckPayload` (including config echo for QUERY commands).
- ACK is sent on next TX opportunity via `send_pending_ack_if_any()` using `ccsds_encode_cmd_ack` (APID 0x003, 22 bytes).

### 5. ACK Return Path (Vehicle → Station)

- Vehicle encodes small CCSDS ACK packet (primary header + MET + 10-byte payload + CRC).
- Station receives via `SIG_RADIO_RX` → `ao_telemetry` MAVLink + CCSDS decode path.
- `station_on_query_ack` / general ACK handling (around lines 440-462) matches against pending command using sequence number.
- On match: record success/failure, latch `s_last_cmd_result`, clear `s_pending_cmd.pending`.

### 6. Failure / Exhaustion Paths

- After `kAckMaxRetries` (currently 8) without ACK: failure is recorded, dashboard is notified, pending state cleared.
- No automatic escalation to Flight Director abort or similar — higher layers (operator or FD) decide.

---

## Visual Diagram (Graphviz)

See the companion file:
**`CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot`**

Render with:
```bash
dot -Tsvg CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.dot -o CURRENT_COMMAND_RETRY_ACK_DATA_FLOW.svg
```

The diagram follows the project's existing conventions from `docs/audits/cla_rbm/dot/`:
- Layered clusters (Station vs Vehicle)
- Color coding by role (command/retry logic highlighted)
- Explicit signal and function labels on edges
- Focus on the tracked command + ACK loop

---

## Key Observations for CCSDS TC-Layer Planning

- The current retry state machine lives almost entirely inside `AO_Telemetry` (station side).
- Sequence numbers are carried in the MAVLink `confirmation` field today.
- ACKs already use a dedicated CCSDS APID (0x003) — this is a small foothold.
- `AO_RfManager` already provides a clean `ok_to_retry()` hook and link quality state that a proper FARM-style receiver could leverage.
- Multi-parameter commands (especially `SET_RADIO_CONFIG`) depend on the full parameter caching in `resend_pending_cmd`.
- Dashboard observability (`PendingCmdStatus`, retry stats) is valuable and should be preserved or mapped to CCSDS equivalents.

This map will be updated as the analysis deepens and the `.dot` is refined.

**Next steps in this document:**
- Expand with exact line numbers for every major transition.
- Add error/fault-injection paths.
- Cross-reference with `AO_RfManager` and `RadioScheduler` timing.
