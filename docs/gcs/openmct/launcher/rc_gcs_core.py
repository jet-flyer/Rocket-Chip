#!/usr/bin/env python3
"""
Shared process / feed logic for the Rocket Chip GCS launchers.

  rc_gcs.pyw      (tkinter web launcher)  imports the small helpers: port_busy, list_ports, kill_stray
  rc_gcs_app.pyw  (desktop app window)    also uses Controller + the :8093 control server

No tkinter in here, so it is safe to import from anywhere.
"""
from __future__ import annotations

import collections
import json
import re
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlparse

HERE = Path(__file__).resolve().parent
OMCT = HERE.parent                      # docs/gcs/openmct
REPO = OMCT.parents[2]
RT = OMCT / "realtime"
PY = Path(sys.executable)
if PY.name.lower() == "pythonw.exe":    # children need a real console-less python
    PY = PY.with_name("python.exe")

WEB_PORT, WS_PORT, CTL_PORT, APP_PORT = 5000, 8091, 8092, 8093
DASH_URL = f"http://localhost:{WEB_PORT}/hello-world/"
SHELL_URL = f"http://localhost:{WEB_PORT}/launcher/app_shell.html"
FEED_SCRIPTS = ("feed_facsimile.py", "stream_station.py", "stream_mavlink_station.py")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)
MODE_KEYS = ("fac", "dash", "mav")


def port_busy(port: int, host: str = "127.0.0.1") -> bool:
    with socket.socket() as s:
        s.settimeout(0.3)
        return s.connect_ex((host, port)) == 0


def list_ports() -> list[str]:
    try:
        from serial.tools import list_ports as lp
        return [f"{p.device} - {p.description}" for p in sorted(lp.comports(), key=lambda p: p.device)]
    except Exception:
        return []


def kill_stray(names: tuple[str, ...]) -> None:
    """Stop feed/web processes started outside this window (e.g. old minimized consoles)."""
    pat = "|".join(n.replace(".", r"\.") for n in names)
    ps = ("Get-CimInstance Win32_Process -Filter \"Name='python.exe'\" | "
          f"Where-Object {{ $_.CommandLine -match '{pat}' }} | "
          "ForEach-Object { Stop-Process -Id $_.ProcessId -Force }")
    subprocess.run(["powershell", "-NoProfile", "-Command", ps],
                   creationflags=NO_WINDOW, capture_output=True, timeout=15)


# ---------------------------------------------------------------- job object
_job = None


def _make_job():
    """Windows job object: children die with this process even if it is killed hard."""
    if sys.platform != "win32":
        return None
    try:
        import ctypes
        from ctypes import wintypes

        class BASIC(ctypes.Structure):
            _fields_ = [("PerProcessUserTimeLimit", ctypes.c_int64), ("PerJobUserTimeLimit", ctypes.c_int64),
                        ("LimitFlags", wintypes.DWORD), ("MinimumWorkingSetSize", ctypes.c_size_t),
                        ("MaximumWorkingSetSize", ctypes.c_size_t), ("ActiveProcessLimit", wintypes.DWORD),
                        ("Affinity", ctypes.c_size_t), ("PriorityClass", wintypes.DWORD),
                        ("SchedulingClass", wintypes.DWORD)]

        class IOC(ctypes.Structure):
            _fields_ = [(n, ctypes.c_uint64) for n in ("a", "b", "c", "d", "e", "f")]

        class EXT(ctypes.Structure):
            _fields_ = [("Basic", BASIC), ("Io", IOC), ("ProcessMemoryLimit", ctypes.c_size_t),
                        ("JobMemoryLimit", ctypes.c_size_t), ("PeakProcessMemoryUsed", ctypes.c_size_t),
                        ("PeakJobMemoryUsed", ctypes.c_size_t)]

        k32 = ctypes.windll.kernel32
        k32.CreateJobObjectW.restype = wintypes.HANDLE
        h = k32.CreateJobObjectW(None, None)
        info = EXT()
        info.Basic.LimitFlags = 0x2000  # JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE
        if not k32.SetInformationJobObject(wintypes.HANDLE(h), 9, ctypes.byref(info), ctypes.sizeof(info)):
            return None
        return h
    except Exception:
        return None


def _adopt(proc: subprocess.Popen) -> None:
    global _job
    try:
        import ctypes
        from ctypes import wintypes
        if _job is None:
            _job = _make_job() or False
        if _job:
            ctypes.windll.kernel32.AssignProcessToJobObject(wintypes.HANDLE(_job), wintypes.HANDLE(int(proc._handle)))
    except Exception:
        pass


# ---------------------------------------------------------------- controller
class Controller:
    """Owns the :5000 web server (if it had to start one) and the single :8091 feed."""

    def __init__(self) -> None:
        self.web: subprocess.Popen | None = None
        self.feed: subprocess.Popen | None = None
        self.feed_mode = ""
        self.lock = threading.RLock()
        self.log: collections.deque[str] = collections.deque(maxlen=300)

    # -- logging / processes
    def say(self, msg: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        for line in str(msg).rstrip().splitlines() or [""]:
            self.log.append(f"{stamp} {line}")

    def _pump(self, proc: subprocess.Popen, tag: str) -> None:
        for line in proc.stdout:  # type: ignore[union-attr]
            if tag == "web" and "HTTP/1." in line:   # drop per-request noise from http.server
                continue
            self.say(f"[{tag}] {line}")
        self.say(f"[{tag}] exited ({proc.wait()})")

    def _spawn(self, args: list[str], tag: str) -> subprocess.Popen:
        self.say(f"> {' '.join(args)}")
        p = subprocess.Popen([str(PY), "-u", *args], cwd=REPO, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, errors="replace",
                             creationflags=NO_WINDOW)
        _adopt(p)
        threading.Thread(target=self._pump, args=(p, tag), daemon=True).start()
        return p

    @staticmethod
    def _stop(p: subprocess.Popen | None) -> None:
        if p and p.poll() is None:
            p.terminate()
            try:
                p.wait(3)
            except subprocess.TimeoutExpired:
                p.kill()

    # -- web server
    def ensure_web(self) -> bool:
        with self.lock:
            if port_busy(WEB_PORT):
                self.say(f"web :{WEB_PORT} already running - reusing it")
                return True
            self.web = self._spawn(["-m", "http.server", str(WEB_PORT), "--bind", "127.0.0.1",
                                    "--directory", str(OMCT)], "web")
        for _ in range(40):
            if port_busy(WEB_PORT):
                return True
            time.sleep(0.25)
        self.say(f"web :{WEB_PORT} did not come up")
        return False

    # -- feed
    def feed_running(self) -> bool:
        return self.feed is not None and self.feed.poll() is None

    def stop_feed(self, quiet: bool = False) -> None:
        with self.lock:
            if self.feed_running():
                self._stop(self.feed)
                self.say("feed stopped")
            elif not quiet:
                self.say("no feed running from this app")
            self.feed = None
            self.feed_mode = ""

    def start_feed(self, mode: str, port: str = "", rate: object = 1, pad_dip: bool = True) -> tuple[bool, str]:
        if mode not in MODE_KEYS:
            return False, f"unknown mode {mode!r}"
        port = (port or "").split(" ")[0].strip()
        if mode != "fac" and not re.fullmatch(r"COM\d{1,3}", port, re.I):
            return False, "pick a station COM port first"
        try:
            rate_f = float(rate or 1)
            if not 0.05 <= rate_f <= 50:
                raise ValueError
        except (TypeError, ValueError):
            return False, "bad replay rate"
        with self.lock:
            self.stop_feed(quiet=True)
            if port_busy(WS_PORT):
                self.say(f"feed port :{WS_PORT} held by another process - stopping it")
                kill_stray(FEED_SCRIPTS)
                time.sleep(0.4)
            if mode == "fac":
                args = [str(RT / "feed_facsimile.py"), "--rate", f"{rate_f:g}"]
                if not pad_dip:
                    args.append("--pad-flat")
            else:
                script = "stream_station.py" if mode == "dash" else "stream_mavlink_station.py"
                args = [str(RT / script), "--port", port.upper(), "--ws-port", str(WS_PORT)]
            self.feed_mode = mode
            self.feed = self._spawn(args, mode)
        if mode == "fac":
            self.say("facsimile idle - press Play (dashboard picks it up on its own)")
        return True, "started"

    def fac_cmd(self, cmd: str) -> tuple[bool, str]:
        if cmd not in ("play", "stop", "reset"):
            return False, "bad command"
        for _ in range(12):                      # feed may still be binding :8092
            if port_busy(CTL_PORT):
                break
            time.sleep(0.25)
        else:
            return False, "facsimile not running - Start feed in Facsimile mode first"
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{CTL_PORT}/{cmd}", data=b"", timeout=2) as r:
                txt = r.read().decode()
            self.say(f"[fac] {txt}")
            return True, txt
        except Exception as e:  # noqa: BLE001
            return False, f"facsimile control error: {e}"

    def fac_status(self) -> str:
        if not port_busy(CTL_PORT):
            return ""
        try:
            with urllib.request.urlopen(f"http://127.0.0.1:{CTL_PORT}/status", timeout=1) as r:
                return r.read().decode()
        except Exception:
            return ""

    def status(self) -> dict:
        ours = self.feed_running()
        return {
            "web": port_busy(WEB_PORT),
            "feed": port_busy(WS_PORT),
            "feedOurs": ours,
            "feedMode": self.feed_mode if ours else "",
            "fac": self.fac_status(),
            "log": list(self.log)[-50:],
        }

    def shutdown(self) -> None:
        self.stop_feed(quiet=True)
        with self.lock:
            self._stop(self.web)
            self.web = None


# ---------------------------------------------------------------- control HTTP
_LOCAL_HOSTS = {"localhost", "127.0.0.1", "[::1]", "::1"}


def _host_of(value: str) -> str:
    v = (value or "").strip()
    if v.startswith("["):
        return v.split("]")[0] + "]"
    return v.split(":")[0].lower()


def make_control_server(ctl: Controller, port: int = APP_PORT) -> ThreadingHTTPServer:
    """127.0.0.1-only JSON API used by launcher/app_shell.html."""

    class H(BaseHTTPRequestHandler):
        server_version = "RCGCS"

        def _origin_ok(self) -> str | None:
            o = self.headers.get("Origin")
            if not o:
                return ""
            u = urlparse(o)
            return o if (u.scheme in ("http", "https") and _host_of(u.netloc) in _LOCAL_HOSTS) else None

        def _send(self, code: int, obj: object) -> None:
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            o = self._origin_ok()
            if o:
                self.send_header("Access-Control-Allow-Origin", o)
                self.send_header("Vary", "Origin")
            self.end_headers()
            self.wfile.write(data)

        def _guard(self) -> bool:
            if _host_of(self.headers.get("Host", "")) not in _LOCAL_HOSTS:   # DNS-rebinding guard
                self._send(403, {"ok": False, "error": "bad host"})
                return False
            if self._origin_ok() is None:
                self._send(403, {"ok": False, "error": "bad origin"})
                return False
            return True

        def do_OPTIONS(self) -> None:  # noqa: N802
            o = self._origin_ok()
            if not o:
                self.send_response(403)
                self.send_header("Content-Length", "0")
                self.end_headers()
                return
            self.send_response(204)
            self.send_header("Access-Control-Allow-Origin", o)
            self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
            self.send_header("Access-Control-Allow-Headers", "Content-Type")
            self.send_header("Access-Control-Max-Age", "600")
            self.end_headers()

        def do_GET(self) -> None:  # noqa: N802
            if not self._guard():
                return
            path = urlparse(self.path).path.rstrip("/")
            if path == "/api/status":
                self._send(200, ctl.status())
            elif path == "/api/ports":
                self._send(200, {"ports": list_ports()})
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def do_POST(self) -> None:  # noqa: N802
            if not self._guard():
                return
            path = urlparse(self.path).path.rstrip("/")
            try:
                n = int(self.headers.get("Content-Length") or 0)
                body = json.loads(self.rfile.read(n) or b"{}") if n else {}
                if not isinstance(body, dict):
                    raise ValueError
            except Exception:  # noqa: BLE001
                self._send(400, {"ok": False, "error": "bad json"})
                return
            if path == "/api/start":
                ok, msg = ctl.start_feed(str(body.get("mode", "")), str(body.get("port", "")),
                                         body.get("rate", 1), bool(body.get("padDip", True)))
                if not ok:
                    ctl.say(msg)
                self._send(200 if ok else 400, {"ok": ok, "message": msg})
            elif path == "/api/stop":
                ctl.stop_feed()
                self._send(200, {"ok": True})
            elif path.startswith("/api/fac/"):
                ok, msg = ctl.fac_cmd(path.rsplit("/", 1)[1])
                if not ok:
                    ctl.say(f"[fac] {msg}")
                self._send(200 if ok else 409, {"ok": ok, "message": msg})
            else:
                self._send(404, {"ok": False, "error": "not found"})

        def log_message(self, fmt: str, *args: object) -> None:
            return

    srv = ThreadingHTTPServer(("127.0.0.1", port), H)
    srv.daemon_threads = True
    return srv
