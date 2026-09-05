"""
Stream station ANSI dash (USB CDC) to Open MCT over WebSocket.

Default: COM7 @ 115200, ws://0.0.0.0:8091/
Emits JSON lines: {"id":"rssi"|"snr"|"seq","timestamp":<ms UTC>,"value":...}

Deps: pyserial, websockets
  pip install pyserial websockets

Usage (desk PC):
  python docs/gcs/openmct/realtime/stream_station.py
  python docs/gcs/openmct/realtime/stream_station.py --port COM7 --ws-port 8091
"""
from __future__ import annotations

import argparse
import asyncio
import json
import re
import time
from typing import Dict, Optional, Set

import serial
import websockets
from websockets.server import WebSocketServerProtocol

RSSI_RE = re.compile(rb"RSSI:\s*(?:\x1b\[[0-9;]*m)*(-?\d+)\s*dBm", re.I)
SNR_RE = re.compile(rb"SNR:\s*(-?\d+)\s*dB", re.I)
PKT_RE = re.compile(rb"Pkts:\s*(\d+)")


def open_cdc(port: str) -> serial.Serial:
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.05
    s.dsrdtr = False
    s.rtscts = False
    s.dtr = False
    s.rts = False
    s.open()
    s.dtr = True
    time.sleep(0.4)
    return s


class LinkHub:
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
        dead = []
        for ws in targets:
            try:
                await ws.send(raw)
            except Exception:
                dead.append(ws)
        for ws in dead:
            await self.unregister(ws)


async def serial_pump(hub: LinkHub, port: str, enter_dash: bool) -> None:
    loop = asyncio.get_running_loop()
    ser = await loop.run_in_executor(None, open_cdc, port)
    if enter_dash:
        await loop.run_in_executor(None, ser.write, b"m")
        await asyncio.sleep(0.5)
        await loop.run_in_executor(None, ser.reset_input_buffer)

    buf = bytearray()
    pending: Optional[tuple[str, str]] = None
    last_seq = "0"
    last_key = None
    print(f"[stream] reading {port}", flush=True)

    try:
        while True:
            chunk = await loop.run_in_executor(None, ser.read, 8192)
            if not chunk:
                await asyncio.sleep(0.01)
                continue
            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    if len(buf) > 20000:
                        del buf[:-4000]
                    break
                line = bytes(buf[: nl + 1])
                del buf[: nl + 1]

                pm = PKT_RE.search(line)
                if pm:
                    last_seq = pm.group(1).decode()
                    if pending is not None:
                        rssi, snr = pending
                        pending = None
                        key = (last_seq, rssi, snr)
                        if key == last_key:
                            continue
                        last_key = key
                        ts = int(time.time() * 1000)
                        await hub.publish({"id": "seq", "timestamp": ts, "value": int(last_seq)})
                        await hub.publish({"id": "rssi", "timestamp": ts, "value": int(rssi)})
                        await hub.publish({"id": "snr", "timestamp": ts, "value": float(snr)})
                    continue

                rm = RSSI_RE.search(line)
                sm = SNR_RE.search(line)
                if rm and sm:
                    pending = (rm.group(1).decode(), sm.group(1).decode())
    finally:
        ser.close()



async def ws_handler(ws: WebSocketServerProtocol, hub: LinkHub) -> None:
    await hub.register(ws)
    try:
        async for message in ws:
            if isinstance(message, bytes):
                message = message.decode("utf-8", "ignore")
            await hub.handle_text(ws, message)
    finally:
        await hub.unregister(ws)


async def main_async(args: argparse.Namespace) -> None:
    hub = LinkHub()

    async def handler(ws: WebSocketServerProtocol) -> None:
        await ws_handler(ws, hub)

    async with websockets.serve(handler, args.bind, args.ws_port):
        print(f"[stream] ws://{args.bind}:{args.ws_port}/", flush=True)
        await serial_pump(hub, args.port, enter_dash=not args.no_m)


def main() -> None:
    p = argparse.ArgumentParser(description="Station USB dash -> Open MCT WebSocket")
    p.add_argument("--port", default="COM7", help="Station CDC port")
    p.add_argument("--ws-port", type=int, default=8091)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--no-m", action="store_true", help="Do not send m to enter dash")
    args = p.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()