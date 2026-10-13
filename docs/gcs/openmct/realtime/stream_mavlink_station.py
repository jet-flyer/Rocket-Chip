#!/usr/bin/env python3
"""Stream station USB MAVLink (QGCS bridge) into Open MCT WebSocket.

Fruit Jam with gcs_mavlink WIP speaks MAVLink v2 on CDC, not the ANSI ``m`` dash.
This feeds the same ws://127.0.0.1:8091/ contract as feed_facsimile / stream_station,
using dictionary keys (baro_alt_m, batt_v, rssi, ...).
"""
from __future__ import annotations

import argparse
import asyncio
import json
import math
import time
from typing import Dict, Optional, Set

import websockets
from pymavlink import mavutil
from websockets.server import WebSocketServerProtocol


class Hub:
    def __init__(self) -> None:
        self.clients: Set[WebSocketServerProtocol] = set()
        self.subs: Dict[WebSocketServerProtocol, Set[str]] = {}
        self.lock = asyncio.Lock()

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
            # If nobody subscribed yet, still push to all (first connect race).
            if not targets and self.clients:
                targets = list(self.clients)
        dead = []
        for ws in targets:
            try:
                await ws.send(raw)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.unregister(ws)


def _ts() -> int:
    return int(time.time() * 1000)


async def mav_pump(hub: Hub, port: str, baud: int) -> None:
    loop = asyncio.get_running_loop()
    print(f"[mav] opening {port} @ {baud}", flush=True)
    conn = await loop.run_in_executor(
        None,
        lambda: mavutil.mavlink_connection(port, baud=baud, autoreconnect=True),
    )
    print("[mav] reading", flush=True)
    seq = 0
    while True:
        msg = await loop.run_in_executor(None, lambda: conn.recv_match(blocking=True, timeout=0.2))
        if msg is None:
            await asyncio.sleep(0.01)
            continue
        mtype = msg.get_type()
        if mtype == "BAD_DATA":
            continue
        ts = _ts()
        seq += 1
        await hub.publish({"id": "seq", "timestamp": ts, "value": seq})

        if mtype == "HEARTBEAT":
            await hub.publish({"id": "flight_state", "timestamp": ts, "value": int(msg.custom_mode)})
        elif mtype == "SYS_STATUS":
            mv = int(msg.voltage_battery)
            if 0 < mv < 65535:
                await hub.publish({"id": "batt_v", "timestamp": ts, "value": round(mv / 1000.0, 3)})
        elif mtype == "ATTITUDE":
            # approx speed/attitude glass; gauges use baro/radio mostly
            await hub.publish({"id": "vvel_mps", "timestamp": ts, "value": 0.0})
        elif mtype == "GLOBAL_POSITION_INT":
            baro_m = float(msg.relative_alt) / 1000.0
            alt_m = float(msg.alt) / 1000.0
            vn = float(msg.vx) / 100.0
            ve = float(msg.vy) / 100.0
            vd = float(msg.vz) / 100.0
            speed = math.sqrt(vn * vn + ve * ve + vd * vd)
            await hub.publish({"id": "baro_alt_m", "timestamp": ts, "value": baro_m})
            await hub.publish({"id": "baro", "timestamp": ts, "value": baro_m})
            await hub.publish({"id": "alt_m", "timestamp": ts, "value": alt_m})
            await hub.publish({"id": "vvel_mps", "timestamp": ts, "value": -vd})
            await hub.publish({"id": "speed_mps", "timestamp": ts, "value": speed})
            if msg.lat or msg.lon:
                await hub.publish({"id": "lat", "timestamp": ts, "value": msg.lat / 1e7})
                await hub.publish({"id": "lon", "timestamp": ts, "value": msg.lon / 1e7})
        elif mtype == "GPS_RAW_INT":
            await hub.publish({"id": "gps_fix", "timestamp": ts, "value": int(msg.fix_type)})
            await hub.publish({"id": "gps_sats", "timestamp": ts, "value": int(msg.satellites_visible)})
            if msg.fix_type >= 2:
                await hub.publish({"id": "lat", "timestamp": ts, "value": msg.lat / 1e7})
                await hub.publish({"id": "lon", "timestamp": ts, "value": msg.lon / 1e7})
                await hub.publish({"id": "alt_m", "timestamp": ts, "value": msg.alt / 1000.0})
        elif mtype == "RADIO_STATUS":
            # rssi is often 0-255; map loosely if present
            await hub.publish({"id": "rssi", "timestamp": ts, "value": int(msg.rssi)})
            await hub.publish({"id": "snr", "timestamp": ts, "value": float(msg.remrssi)})
            await hub.publish({"id": "lq_pct", "timestamp": ts, "value": int(msg.rssi)})


async def ws_handler(ws: WebSocketServerProtocol, hub: Hub) -> None:
    await hub.register(ws)
    try:
        async for message in ws:
            if isinstance(message, bytes):
                message = message.decode("utf-8", "ignore")
            await hub.handle_text(ws, message)
    finally:
        await hub.unregister(ws)


async def main_async(args: argparse.Namespace) -> None:
    hub = Hub()

    async def handler(ws: WebSocketServerProtocol) -> None:
        await ws_handler(ws, hub)

    async with websockets.serve(handler, args.bind, args.ws_port):
        print(f"[mav] ws://{args.bind}:{args.ws_port}/", flush=True)
        await mav_pump(hub, args.port, args.baud)


def main() -> None:
    p = argparse.ArgumentParser(description="Station USB MAVLink -> Open MCT WS")
    p.add_argument("--port", default="COM7")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--ws-port", type=int, default=8091)
    p.add_argument("--bind", default="127.0.0.1")
    args = p.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
