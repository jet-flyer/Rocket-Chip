#!/usr/bin/env python3
"""
Replay a fidelity CSV into Open MCT over WebSocket (live conductor path).

Default: omct_big_daddy_fidelity.csv -> ws://127.0.0.1:8091/
Also refreshes fixtures/live.csv as a rolling window for the historical poller.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, List, Optional, Set

import websockets
from websockets.server import WebSocketServerProtocol

DEFAULT_CSV = Path(__file__).resolve().parents[1] / "fixtures" / "omct_big_daddy_fidelity.csv"
ROLLING_CSV = Path(__file__).resolve().parents[1] / "fixtures" / "live.csv"
ROLLING_WINDOW_S = 120.0

# Keys published to oMCT (match dictionary)
PUBLISH_KEYS = [
    "seq",
    "flight_state",
    "chute_detected",
    "phase_event",
    "met_ms",
    "alt_m",
    "max_alt_m",
    "baro_alt_m",
    "vvel_mps",
    "speed_mps",
    "accel_g",
    "lat",
    "lon",
    "gps_sats",
    "gps_fix",
    "health",
    "batt_v",
    "temp_c",
    "imu_temp_c",
    "baro_temp_c",
    "die_temp_c",
    "rssi",
    "snr",
    "lq_pct",
    "rx_hz",
    "q_w",
    "q_x",
    "q_y",
    "q_z",
]


def load_rows(path: Path) -> List[dict]:
    lines = [L for L in path.read_text(encoding="utf-8").splitlines() if L.strip() and not L.strip().startswith("#")]
    return list(csv.DictReader(lines))


class Hub:
    def __init__(self) -> None:
        self.clients: Set[WebSocketServerProtocol] = set()
        self.subs: Dict[WebSocketServerProtocol, Set[str]] = {}
        self.lock = asyncio.Lock()
        self.rolling: List[tuple] = []

    async def register(self, ws: WebSocketServerProtocol) -> None:
        async with self.lock:
            self.clients.add(ws)
            self.subs[ws] = set()

    async def unregister(self, ws: WebSocketServerProtocol) -> None:
        async with self.lock:
            self.clients.discard(ws)
            self.subs.pop(ws, None)

    async def handle_text(self, ws: WebSocketServerProtocol, message: str) -> None:
        parts = message.strip().split()
        if len(parts) != 2:
            return
        op, key = parts[0].lower(), parts[1]
        async with self.lock:
            bucket = self.subs.setdefault(ws, set())
            if op == "subscribe":
                bucket.add(key)
            elif op == "unsubscribe":
                bucket.discard(key)

    async def publish(self, point: dict) -> None:
        raw = json.dumps(point)
        async with self.lock:
            targets = [ws for ws, keys in self.subs.items() if point["id"] in keys]
        dead = []
        for ws in targets:
            try:
                await ws.send(raw)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.unregister(ws)

    def record_row(self, ts_ms: int, row: dict) -> None:
        iso = datetime.fromtimestamp(ts_ms / 1000.0, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"
        epoch = ts_ms / 1000.0
        self.rolling.append((iso, row, epoch))
        cut = time.time() - ROLLING_WINDOW_S
        self.rolling = [r for r in self.rolling if r[2] >= cut]
        try:
            ROLLING_CSV.parent.mkdir(parents=True, exist_ok=True)
            # Prefer fidelity column set for live.csv during facsimile replay
            keys = ["timestamp"] + [k for k in PUBLISH_KEYS if k in row or True]
            # unique preserve order
            seen = set()
            fieldnames = []
            for k in keys:
                if k not in seen:
                    seen.add(k)
                    fieldnames.append(k)
            with ROLLING_CSV.open("w", encoding="utf-8", newline="\n") as f:
                w = csv.DictWriter(f, fieldnames=fieldnames)
                w.writeheader()
                for iso_s, r, _ in self.rolling:
                    out = {"timestamp": iso_s}
                    for k in PUBLISH_KEYS:
                        out[k] = r.get(k, "")
                    w.writerow(out)
        except Exception as ex:
            print("[feed] rolling csv", ex, flush=True)


async def pump(hub: Hub, rows: List[dict], rate: float, loop_forever: bool) -> None:
    if not rows:
        print("[feed] no rows", flush=True)
        return
    # Inter-sample from met_ms when present
    while True:
        t0_wall = time.time()
        met0 = float(rows[0].get("met_ms") or 0)
        for row in rows:
            met = float(row.get("met_ms") or 0)
            target = t0_wall + ((met - met0) / 1000.0) / max(rate, 1e-6)
            delay = target - time.time()
            if delay > 0:
                await asyncio.sleep(delay)
            ts = int(time.time() * 1000)
            # normalize aliases
            if not row.get("rssi"):
                row["rssi"] = row.get("rssi_dbm", "")
            if not row.get("snr"):
                row["snr"] = row.get("snr_db", "")
            if not row.get("baro_alt_m"):
                row["baro_alt_m"] = row.get("baro", "")
            for key in PUBLISH_KEYS:
                raw = row.get(key, "")
                if raw == "" or raw is None:
                    continue
                try:
                    val: float | int = float(raw)
                    if float(val).is_integer() and key in (
                        "seq",
                        "flight_state",
                        "chute_detected",
                        "phase_event",
                        "gps_sats",
                        "gps_fix",
                        "health",
                        "met_ms",
                    ):
                        val = int(val)
                except ValueError:
                    continue
                await hub.publish({"id": key, "timestamp": ts, "value": val})
            hub.record_row(ts, row)
        if not loop_forever:
            print("[feed] done", flush=True)
            return
        print("[feed] loop", flush=True)


async def main_async(args: argparse.Namespace) -> None:
    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"missing csv: {csv_path}")
    rows = load_rows(csv_path)
    print(f"[feed] {len(rows)} rows from {csv_path}", flush=True)
    hub = Hub()

    async def handler(ws: WebSocketServerProtocol) -> None:
        await hub.register(ws)
        try:
            async for message in ws:
                if isinstance(message, bytes):
                    message = message.decode("utf-8", "ignore")
                await hub.handle_text(ws, message)
        finally:
            await hub.unregister(ws)

    async with websockets.serve(handler, args.bind, args.ws_port):
        print(f"[feed] ws://{args.bind}:{args.ws_port}/", flush=True)
        await pump(hub, rows, rate=args.rate, loop_forever=args.loop)


def main() -> None:
    p = argparse.ArgumentParser(description="Facsimile CSV -> Open MCT WebSocket live feed")
    p.add_argument("--csv", default=str(DEFAULT_CSV))
    p.add_argument("--ws-port", type=int, default=8091)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--rate", type=float, default=1.0, help="Playback speed (2=2x realtime)")
    p.add_argument("--loop", action="store_true", help="Loop the hop forever")
    args = p.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
