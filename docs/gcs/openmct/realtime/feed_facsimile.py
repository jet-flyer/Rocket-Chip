#!/usr/bin/env python3
"""
Replay fidelity CSV into Open MCT over WebSocket - on demand.

Default: listen idle on ws://127.0.0.1:8091/ and http://127.0.0.1:8092/
  - WS text: play | stop | reset
  - HTTP: GET/POST /play /stop /reset /status
Does NOT auto-loop. Optional --loop only after a play finishes.
"""
from __future__ import annotations

import argparse
import asyncio
import csv
import json
import time
from datetime import datetime, timezone
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Dict, List, Optional, Set
from urllib.parse import urlparse

import websockets
from websockets.server import WebSocketServerProtocol

DEFAULT_CSV = Path(__file__).resolve().parents[1] / "fixtures" / "omct_big_daddy_fidelity.csv"
ROLLING_CSV = Path(__file__).resolve().parents[1] / "fixtures" / "live.csv"
ROLLING_WINDOW_S = 120.0

PUBLISH_KEYS = [
    "seq", "flight_state", "chute_detected", "phase_event", "met_ms",
    "alt_m", "max_alt_m", "baro_alt_m", "vvel_mps", "speed_mps", "accel_g",
    "lat", "lon", "gps_sats", "gps_fix", "health", "batt_v", "temp_c",
    "imu_temp_c", "baro_temp_c", "die_temp_c", "rssi", "snr", "lq_pct", "rx_hz",
    "q_w", "q_x", "q_y", "q_z",
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
        self.cmd_q: asyncio.Queue[str] = asyncio.Queue()
        self.state = "idle"  # idle | playing | stopped
        self.loop: Optional[asyncio.AbstractEventLoop] = None

    def submit(self, cmd: str) -> None:
        if self.loop is None:
            return
        self.loop.call_soon_threadsafe(self.cmd_q.put_nowait, cmd)

    async def register(self, ws: WebSocketServerProtocol) -> None:
        async with self.lock:
            self.clients.add(ws)
            self.subs[ws] = set()

    async def unregister(self, ws: WebSocketServerProtocol) -> None:
        async with self.lock:
            self.clients.discard(ws)
            self.subs.pop(ws, None)

    async def handle_text(self, ws: WebSocketServerProtocol, message: str) -> None:
        msg = message.strip().lower()
        if msg in ("play", "stop", "reset", "status"):
            if msg == "status":
                await ws.send(json.dumps({"id": "_status", "value": self.state}))
            else:
                await self.cmd_q.put(msg)
            return
        parts = msg.split()
        if len(parts) != 2:
            return
        op, key = parts[0], parts[1]
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
        self.rolling.append((iso, dict(row), epoch))
        cut = time.time() - ROLLING_WINDOW_S
        self.rolling = [r for r in self.rolling if r[2] >= cut]
        try:
            ROLLING_CSV.parent.mkdir(parents=True, exist_ok=True)
            fieldnames = ["timestamp"] + PUBLISH_KEYS
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


def start_http(hub: Hub, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def _ok(self, body: str, code: int = 200) -> None:
            data = body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", "text/plain; charset=utf-8")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)

        def do_OPTIONS(self) -> None:  # noqa: N802
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", "*")
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            path = urlparse(self.path).path.rstrip("/") or "/"
            if path in ("/play", "/stop", "/reset"):
                hub.submit(path.lstrip("/"))
                self._ok(path.lstrip("/") + " queued")
            elif path == "/status":
                self._ok(hub.state)
            elif path in ("/", "/index.html"):
                html = """<!doctype html><meta charset=utf-8>
<title>RC Facsimile Control</title>
<body style="font:16px system-ui;background:#1a1a1a;color:#eee;padding:24px">
<h1>Facsimile hop</h1>
<p>Status: <b id=s>…</b></p>
<p>
<button onclick="go('play')">Play</button>
<button onclick="go('stop')">Stop</button>
<button onclick="go('reset')">Reset</button>
</p>
<script>
async function go(c){ await fetch('/'+c,{method:'POST'}); tick(); }
async function tick(){ s.textContent = await (await fetch('/status')).text(); }
tick(); setInterval(tick, 500);
</script></body>"""
                data = html.encode("utf-8")
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(data)))
                self.end_headers()
                self.wfile.write(data)
            else:
                self._ok("not found", 404)

        def do_POST(self) -> None:  # noqa: N802
            self.do_GET()

        def log_message(self, fmt: str, *args) -> None:
            return

    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    import threading
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


async def play_once(hub: Hub, rows: List[dict], rate: float, stop_flag: asyncio.Event) -> None:
    hub.state = "playing"
    print("[feed] PLAY", flush=True)
    t0_wall = time.time()
    met0 = float(rows[0].get("met_ms") or 0)
    for row in rows:
        if stop_flag.is_set():
            hub.state = "stopped"
            print("[feed] STOP", flush=True)
            return
        # drain play/reset while playing? stop handled via flag; reset cancels
        met = float(row.get("met_ms") or 0)
        target = t0_wall + ((met - met0) / 1000.0) / max(rate, 1e-6)
        while True:
            if stop_flag.is_set():
                hub.state = "stopped"
                print("[feed] STOP", flush=True)
                return
            delay = target - time.time()
            if delay <= 0:
                break
            await asyncio.sleep(min(delay, 0.05))
        if not row.get("rssi"):
            row["rssi"] = row.get("rssi_dbm", "")
        if not row.get("snr"):
            row["snr"] = row.get("snr_db", "")
        ts = int(time.time() * 1000)
        for key in PUBLISH_KEYS:
            raw = row.get(key, "")
            if raw == "" or raw is None:
                continue
            try:
                val: float | int = float(raw)
                if float(val).is_integer() and key in (
                    "seq", "flight_state", "chute_detected", "phase_event",
                    "gps_sats", "gps_fix", "health", "met_ms",
                ):
                    val = int(val)
            except ValueError:
                continue
            await hub.publish({"id": key, "timestamp": ts, "value": val})
        hub.record_row(ts, row)
    hub.state = "idle"
    print("[feed] DONE (idle)", flush=True)


async def control_loop(hub: Hub, rows: List[dict], rate: float, loop_after: bool) -> None:
    stop_flag = asyncio.Event()
    play_task: Optional[asyncio.Task] = None
    print("[feed] idle - send play (WS) or open http://127.0.0.1:8092/", flush=True)
    while True:
        cmd = await hub.cmd_q.get()
        if cmd == "play":
            if play_task and not play_task.done():
                print("[feed] already playing", flush=True)
                continue
            stop_flag.clear()
            async def _run() -> None:
                while True:
                    await play_once(hub, rows, rate, stop_flag)
                    if stop_flag.is_set() or not loop_after:
                        break
                    print("[feed] loop - playing again", flush=True)
            play_task = asyncio.create_task(_run())
        elif cmd == "stop":
            stop_flag.set()
        elif cmd == "reset":
            stop_flag.set()
            if play_task and not play_task.done():
                await asyncio.sleep(0.05)
            stop_flag.clear()
            hub.state = "idle"
            print("[feed] RESET -> idle", flush=True)


async def main_async(args: argparse.Namespace) -> None:
    csv_path = Path(args.csv)
    if not csv_path.exists():
        raise SystemExit(f"missing csv: {csv_path}")
    rows = load_rows(csv_path)
    print(f"[feed] {len(rows)} rows from {csv_path}", flush=True)
    hub = Hub()
    hub.loop = asyncio.get_running_loop()
    start_http(hub, args.http_port)
    print(f"[feed] control http://127.0.0.1:{args.http_port}/", flush=True)

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
        if args.autoplay:
            await hub.cmd_q.put("play")
        await control_loop(hub, rows, rate=args.rate, loop_after=args.loop)


def main() -> None:
    try:
        import sys
        if hasattr(sys.stdout, 'reconfigure'):
            sys.stdout.reconfigure(errors='replace')
            sys.stderr.reconfigure(errors='replace')
    except Exception:
        pass
    p = argparse.ArgumentParser(description="On-demand facsimile CSV -> Open MCT WebSocket")
    p.add_argument("--csv", default=str(DEFAULT_CSV))
    p.add_argument("--ws-port", type=int, default=8091)
    p.add_argument("--http-port", type=int, default=8092)
    p.add_argument("--bind", default="127.0.0.1")
    p.add_argument("--rate", type=float, default=1.0)
    p.add_argument("--loop", action="store_true", help="After play finishes, play again until stop")
    p.add_argument("--autoplay", action="store_true", help="Start playing immediately (legacy)")
    args = p.parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()
