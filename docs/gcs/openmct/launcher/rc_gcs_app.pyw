#!/usr/bin/env python3
"""
Rocket Chip GCS - stand-alone desktop app window for the Open MCT glass.

One window: launcher/app_shell.html (collapsible control bar + the dashboard in an iframe).
Starts the :5000 static server (reuses it if already up), a 127.0.0.1-only control API on
:8093, then opens the window. Closing the window stops the feed, web server and control
server that this app started.

Window host: pywebview (Edge WebView2) if importable, else Edge in --app mode.
Options: --edge (force the Edge fallback), --headless (servers only, for testing).
Run with pythonw (no console). Desktop shortcut: make_shortcut.ps1.
The older tk + browser launcher is still rc_gcs.pyw.
"""
from __future__ import annotations

import atexit
import os
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rc_gcs_core as core  # noqa: E402

TITLE = "Rocket Chip GCS"
ICON = core.HERE / "rc_gcs.ico"
DATA = Path(os.environ.get("LOCALAPPDATA") or tempfile.gettempdir()) / "RocketChipGCS"
EDGE_CANDIDATES = (
    r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe",
    r"C:\Program Files\Microsoft\Edge\Application\msedge.exe",
)

ctl = core.Controller()
_cleaned = False


def _logfile() -> None:
    """pythonw has no stdout/stderr; keep a small log so failures can be diagnosed."""
    DATA.mkdir(parents=True, exist_ok=True)
    f = open(DATA / "app.log", "a", buffering=1, encoding="utf-8", errors="replace")
    if sys.stdout is None:
        sys.stdout = f
    if sys.stderr is None:
        sys.stderr = f


def cleanup() -> None:
    global _cleaned
    if _cleaned:
        return
    _cleaned = True
    try:
        ctl.shutdown()
    except Exception:  # noqa: BLE001
        pass
    srv = getattr(cleanup, "srv", None)
    if srv is not None:
        try:
            srv.shutdown()
            srv.server_close()
        except Exception:  # noqa: BLE001
            pass


def set_app_id() -> None:
    """Own taskbar identity so the taskbar shows our icon instead of python's."""
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("RocketChip.GCS.App")
    except Exception:  # noqa: BLE001
        pass


def run_pywebview() -> bool:
    try:
        import webview
    except Exception as e:  # noqa: BLE001
        print(f"pywebview unavailable: {e}")
        return False

    try:
        webview.create_window(TITLE, core.SHELL_URL, width=1600, height=950,
                                  min_size=(800, 500), background_color="#2c2c2c")
        (DATA / "webview").mkdir(parents=True, exist_ok=True)
        webview.start(private_mode=False, storage_path=str(DATA / "webview"), icon=str(ICON))
        return True
    except Exception as e:  # noqa: BLE001
        print(f"pywebview failed: {e!r}")
        return False


def run_edge() -> bool:
    exe = next((p for p in EDGE_CANDIDATES if Path(p).exists()), None)
    if not exe:
        print("msedge.exe not found")
        return False
    profile = DATA / "edge-profile"
    profile.mkdir(parents=True, exist_ok=True)
    p = subprocess.Popen([exe, f"--app={core.SHELL_URL}", f"--user-data-dir={profile}",
                          "--window-size=1600,950", "--no-first-run", "--no-default-browser-check"])
    p.wait()
    return True


def main() -> int:
    _logfile()
    set_app_id()
    if core.port_busy(core.APP_PORT):
        print("already running (control port 8093 answering) - not opening a second window")
        return 0
    try:
        srv = core.make_control_server(ctl)
    except OSError as e:
        print(f"cannot bind control port {core.APP_PORT}: {e}")
        return 1
    cleanup.srv = srv  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    atexit.register(cleanup)
    ctl.ensure_web()

    try:
        if "--headless" in sys.argv:
            while True:
                time.sleep(1)
        used = False
        if "--edge" not in sys.argv:
            used = run_pywebview()
        if not used:
            print("falling back to Edge app mode")
            used = run_edge()
        if not used:
            print("no window host available")
            return 1
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
