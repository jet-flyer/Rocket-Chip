#!/usr/bin/env python3
"""
Rocket Chip GCS launcher - one window to run the Open MCT glass.

Starts the static web server (:5000), and exactly one telemetry feed on :8091:
  - Facsimile  : feed_facsimile.py (recorded flight + pad countdown, Play/Stop/Reset)
  - Live (dash): stream_station.py  (station USB text dashboard scrape)
  - Live (MAVLink): stream_mavlink_station.py (station USB MAVLink v2)
Run with pythonw (no console). Desktop shortcut: make_shortcut.ps1.
"""
from __future__ import annotations

import queue
import socket
import subprocess
import sys
import threading
import time
import tkinter as tk
import urllib.request
import webbrowser
from pathlib import Path
from tkinter import ttk

HERE = Path(__file__).resolve().parent
OMCT = HERE.parent                      # docs/gcs/openmct
REPO = OMCT.parents[2]
RT = OMCT / "realtime"
PY = Path(sys.executable)
if PY.name.lower() == "pythonw.exe":    # children need a real console-less python
    PY = PY.with_name("python.exe")

WEB_PORT, WS_PORT, CTL_PORT = 5000, 8091, 8092
DASH_URL = f"http://localhost:{WEB_PORT}/hello-world/"
PLAY_URL = f"http://127.0.0.1:{CTL_PORT}/"
FEED_SCRIPTS = ("feed_facsimile.py", "stream_station.py", "stream_mavlink_station.py")
NO_WINDOW = getattr(subprocess, "CREATE_NO_WINDOW", 0)

MODES = {
    "Facsimile (recorded flight)": "fac",
    "Live - station text dashboard": "dash",
    "Live - station MAVLink": "mav",
}


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


class App:
    def __init__(self, root: tk.Tk) -> None:
        self.root = root
        root.title("Rocket Chip GCS")
        root.geometry("620x520")
        root.minsize(560, 440)
        self.web: subprocess.Popen | None = None
        self.feed: subprocess.Popen | None = None
        self.feed_mode = ""
        self.logq: queue.Queue[str] = queue.Queue()

        pad = {"padx": 8, "pady": 4}
        top = ttk.LabelFrame(root, text="Data source")
        top.pack(fill="x", **pad)
        self.mode = tk.StringVar(value=next(iter(MODES)))
        for i, label in enumerate(MODES):
            ttk.Radiobutton(top, text=label, value=label, variable=self.mode,
                            command=self._mode_changed).grid(row=i, column=0, sticky="w", padx=6)

        opts = ttk.Frame(top)
        opts.grid(row=0, column=1, rowspan=3, sticky="nw", padx=12)
        ttk.Label(opts, text="Replay rate").grid(row=0, column=0, sticky="w")
        self.rate = tk.StringVar(value="1")
        self.rate_box = ttk.Combobox(opts, textvariable=self.rate, width=6, values=["0.5", "1", "2", "4"])
        self.rate_box.grid(row=0, column=1, sticky="w")
        self.pad_dip = tk.BooleanVar(value=True)
        self.pad_chk = ttk.Checkbutton(opts, text="Pad countdown with link sag (colour test)", variable=self.pad_dip)
        self.pad_chk.grid(row=1, column=0, columnspan=3, sticky="w")
        ttk.Label(opts, text="Station port").grid(row=2, column=0, sticky="w")
        self.port = tk.StringVar()
        self.port_box = ttk.Combobox(opts, textvariable=self.port, width=34, state="readonly")
        self.port_box.grid(row=2, column=1, sticky="w")
        ttk.Button(opts, text="Refresh", command=self._refresh_ports).grid(row=2, column=2, padx=4)

        run = ttk.Frame(root)
        run.pack(fill="x", **pad)
        self.start_btn = ttk.Button(run, text="Start feed", command=self.start_feed)
        self.start_btn.pack(side="left")
        ttk.Button(run, text="Stop feed", command=self.stop_feed).pack(side="left", padx=4)
        ttk.Separator(run, orient="vertical").pack(side="left", fill="y", padx=8)
        ttk.Button(run, text="Launch oMCT in current browser window", command=self.open_dash).pack(side="left")

        self.play = ttk.LabelFrame(root, text="Facsimile playback")
        self.play.pack(fill="x", **pad)
        for c in ("play", "stop", "reset"):
            ttk.Button(self.play, text=c.capitalize(), command=lambda c=c: self.fac_cmd(c)).pack(side="left", padx=4, pady=4)
        self.fac_state = ttk.Label(self.play, text="")
        self.fac_state.pack(side="left", padx=12)

        stat = ttk.Frame(root)
        stat.pack(fill="x", **pad)
        self.web_lbl = ttk.Label(stat, text="Web: ?")
        self.web_lbl.pack(side="left")
        self.feed_lbl = ttk.Label(stat, text="Feed: ?")
        self.feed_lbl.pack(side="left", padx=16)

        self.log = tk.Text(root, height=12, bg="#111", fg="#ddd", insertbackground="#ddd", wrap="none")
        self.log.pack(fill="both", expand=True, **pad)

        root.protocol("WM_DELETE_WINDOW", self.on_close)
        self._refresh_ports()
        self._mode_changed()
        self.ensure_web()
        self._tick()
        self._drain()

    # ---------- helpers ----------
    def say(self, msg: str) -> None:
        self.logq.put(msg.rstrip() + "\n")

    def _drain(self) -> None:
        try:
            while True:
                self.log.insert("end", self.logq.get_nowait())
                self.log.see("end")
        except queue.Empty:
            pass
        n = int(self.log.index("end-1c").split(".")[0])
        if n > 2000:
            self.log.delete("1.0", f"{n - 1500}.0")
        self.root.after(150, self._drain)

    def _pump(self, proc: subprocess.Popen, tag: str) -> None:
        for line in proc.stdout:  # type: ignore[union-attr]
            self.say(f"[{tag}] {line}")
        self.say(f"[{tag}] exited ({proc.wait()})")

    def _spawn(self, args: list[str], tag: str) -> subprocess.Popen:
        self.say(f"> {' '.join(args)}")
        p = subprocess.Popen([str(PY), "-u", *args], cwd=REPO, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, errors="replace",
                             creationflags=NO_WINDOW)
        threading.Thread(target=self._pump, args=(p, tag), daemon=True).start()
        return p

    def _refresh_ports(self) -> None:
        ports = list_ports()
        self.port_box["values"] = ports
        if ports and not any(self.port.get() == p for p in ports):
            com7 = [p for p in ports if p.startswith("COM7 ")]
            self.port.set((com7 or ports)[0])

    def _mode_changed(self) -> None:
        fac = MODES[self.mode.get()] == "fac"
        for w in (self.rate_box, self.pad_chk):
            w.state(["!disabled"] if fac else ["disabled"])
        # "readonly" alone does not clear a prior "disabled" flag.
        self.port_box.state(["disabled"] if fac else ["!disabled", "readonly"])

    # ---------- servers ----------
    def ensure_web(self) -> None:
        if port_busy(WEB_PORT):
            self.say(f"web :{WEB_PORT} already running - reusing it")
            return
        self.web = self._spawn(["-m", "http.server", str(WEB_PORT),
                                "--directory", str(OMCT)], "web")

    def open_dash(self) -> None:
        """Start the web server if it is down, wait for it, then open a tab in the existing browser window."""
        def go() -> None:
            if not port_busy(WEB_PORT):
                self.say("web server down - starting it")
                self.root.after(0, self.ensure_web)
                for _ in range(40):
                    if port_busy(WEB_PORT):
                        break
                    time.sleep(0.25)
                else:
                    self.say(f"web :{WEB_PORT} did not come up - see log")
                    return
            webbrowser.open(DASH_URL, new=2)
            self.say(f"opened {DASH_URL} (Ctrl+F5 there if the layout looks old)")
        threading.Thread(target=go, daemon=True).start()

    def start_feed(self) -> None:
        self.stop_feed(quiet=True)
        if port_busy(WS_PORT):
            self.say(f"feed port :{WS_PORT} held by another window - stopping it")
            kill_stray(FEED_SCRIPTS)
        mode = MODES[self.mode.get()]
        if mode == "fac":
            args = [str(RT / "feed_facsimile.py"), "--rate", self.rate.get() or "1"]
            if not self.pad_dip.get():
                args.append("--pad-flat")
        else:
            port = (self.port.get() or "").split(" ")[0]
            if not port:
                self.say("pick a station COM port first")
                return
            script = "stream_station.py" if mode == "dash" else "stream_mavlink_station.py"
            args = [str(RT / script), "--port", port, "--ws-port", str(WS_PORT)]
        self.feed_mode = self.mode.get()
        self.feed = self._spawn(args, mode)
        if mode == "fac":
            self.say("facsimile idle - press Play (dashboard picks it up on its own)")

    def stop_feed(self, quiet: bool = False) -> None:
        if self.feed and self.feed.poll() is None:
            self.feed.terminate()
            try:
                self.feed.wait(3)
            except subprocess.TimeoutExpired:
                self.feed.kill()
            self.say("feed stopped")
        elif not quiet:
            self.say("no feed running from this window")
        self.feed = None

    def fac_cmd(self, cmd: str) -> None:
        def go() -> None:
            try:
                with urllib.request.urlopen(f"http://127.0.0.1:{CTL_PORT}/{cmd}", data=b"", timeout=2) as r:
                    self.say(f"[fac] {r.read().decode()}")
            except Exception:
                self.say("[fac] facsimile not running - Start feed in Facsimile mode first")
        threading.Thread(target=go, daemon=True).start()

    def _tick(self) -> None:
        def poll() -> None:
            web = port_busy(WEB_PORT)
            ws = port_busy(WS_PORT)
            ours = self.feed is not None and self.feed.poll() is None
            st = ""
            if port_busy(CTL_PORT):
                try:
                    with urllib.request.urlopen(f"http://127.0.0.1:{CTL_PORT}/status", timeout=1) as r:
                        st = r.read().decode()
                except Exception:
                    pass
            def paint() -> None:
                self.web_lbl.config(text=f"Web :{WEB_PORT}  {'UP' if web else 'DOWN'}",
                                    foreground="#2e7d32" if web else "#c62828")
                if ours:
                    txt, col = f"Feed: {self.feed_mode}", "#2e7d32"
                elif ws:
                    txt, col = "Feed: running outside this window", "#b45f06"
                else:
                    txt, col = "Feed: stopped", "#c62828"
                self.feed_lbl.config(text=txt, foreground=col)
                self.fac_state.config(text=f"status: {st}" if st else "")
            self.root.after(0, paint)
        threading.Thread(target=poll, daemon=True).start()
        self.root.after(1000, self._tick)

    def on_close(self) -> None:
        self.stop_feed(quiet=True)
        if self.web and self.web.poll() is None:
            self.web.terminate()
        self.root.destroy()


def main() -> None:
    root = tk.Tk()
    try:
        ttk.Style().theme_use("vista")
    except tk.TclError:
        pass
    try:
        root.iconbitmap(str(HERE / "rc_gcs.ico"))
    except tk.TclError:
        pass
    App(root)
    if "--selftest" in sys.argv:
        root.after(1500, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
