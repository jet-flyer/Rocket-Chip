#!/usr/bin/env python3
"""
Rocket Chip GCS - stand-alone desktop app window for the Open MCT glass.

One window: launcher/app_shell.html (collapsible control bar + the dashboard in an iframe).
Starts the :5000 static server (reuses it only if it really answers; a dead/stale one is
replaced), a 127.0.0.1-only control API on :8093, waits until the web server serves the
page, then opens the window. Closing the window stops the feed, web server and control
server that this app started.

Window host, in order: Google Chrome in --app mode (dedicated profile under
%LOCALAPPDATA%\\RocketChipGCS\\chrome-profile), another Chromium-like browser (Chromium,
Brave), then Edge --app. Options: --chrome / --edge / --webview (force one host; --webview
needs the optional pywebview package), --headless (servers only, for testing).
Run with pythonw (no console). Desktop shortcuts: make_shortcut.ps1.
The older tk + browser launcher is still rc_gcs.pyw.
"""
from __future__ import annotations

import atexit
import os
import shutil
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
PROFILE = DATA / "chrome-profile"
EDGE_PROFILE = DATA / "edge-profile"
WINDOW_ARGS = ("--window-size=1600,950", "--no-first-run", "--no-default-browser-check",
               "--hide-crash-restore-bubble")


def _env_dirs() -> list[str]:
    return [d for d in (os.environ.get("ProgramFiles"), os.environ.get("ProgramFiles(x86)"),
                        os.environ.get("LOCALAPPDATA")) if d]


def _app_paths(exe: str) -> list[str]:
    """Default value of the App Paths registry key (HKCU first, then HKLM)."""
    out: list[str] = []
    try:
        import winreg
        for root in (winreg.HKEY_CURRENT_USER, winreg.HKEY_LOCAL_MACHINE):
            try:
                with winreg.OpenKey(root, rf"SOFTWARE\Microsoft\Windows\CurrentVersion\App Paths\{exe}") as k:
                    out.append(winreg.QueryValue(k, None))
            except OSError:
                pass
    except Exception:  # noqa: BLE001
        pass
    return out


def _find(exe: str, rel_dirs: tuple[str, ...]) -> str | None:
    cands = [str(Path(d) / rel / exe) for rel in rel_dirs for d in _env_dirs()]
    cands += _app_paths(exe)
    w = shutil.which(exe)
    if w:
        cands.append(w)
    return next((c for c in cands if c and Path(c).is_file()), None)


def find_chrome() -> str | None:
    return _find("chrome.exe", (r"Google\Chrome\Application",))


def find_chromium_like() -> tuple[str, str] | None:
    for name, exe, rel in (("Chromium", "chromium.exe", (r"Chromium\Application",)),
                           ("Brave", "brave.exe", (r"BraveSoftware\Brave-Browser\Application",))):
        p = _find(exe, rel)
        if p:
            return name, p
    return None


def find_edge() -> str | None:
    return _find("msedge.exe", (r"Microsoft\Edge\Application",))


ctl = core.Controller()
_cleaned = False


def _logfile() -> None:
    """pythonw has no stdout/stderr; keep a small log so failures can be diagnosed."""
    DATA.mkdir(parents=True, exist_ok=True)
    log = DATA / "app.log"
    try:
        if log.stat().st_size > 200_000:      # keep it small: start over
            log.unlink()
    except OSError:
        pass
    f = open(log, "a", buffering=1, encoding="utf-8", errors="replace")
    if sys.stdout is None:
        sys.stdout = f
    if sys.stderr is None:
        sys.stderr = f


def log(msg: str) -> None:
    print(f"{time.strftime('%Y-%m-%d %H:%M:%S')} {msg}", flush=True)


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
    core.kill_profile_procs(PROFILE)           # a window that outlived us would point at dead servers


def set_app_id() -> None:
    """Own taskbar identity so the taskbar shows our icon instead of python's."""
    try:
        import ctypes
        ctypes.windll.shell32.SetCurrentProcessExplicitAppUserModelID("RocketChip.GCS.App")
    except Exception:  # noqa: BLE001
        pass


def run_chromium(exe: str, profile: Path, label: str) -> bool:
    """Open SHELL_URL as a chromeless app window and block until that window is closed."""
    core.kill_profile_procs(profile)           # orphan from a crashed run would swallow our launch
    profile.mkdir(parents=True, exist_ok=True)
    args = [exe, f"--app={core.SHELL_URL}", f"--user-data-dir={profile}", *WINDOW_ARGS]
    log(f"opening {label}: {' '.join(args)}")
    try:
        p = subprocess.Popen(args)
    except OSError as e:
        log(f"{label} failed to start: {e!r}")
        return False
    started = time.time()
    p.wait()
    # With a dedicated profile the first process IS the browser, so wait() normally blocks until
    # the window closes. If it handed off and exited early, keep waiting on the profile's processes.
    if time.time() - started < 5:
        log(f"{label} launcher exited early ({p.returncode}); watching profile processes instead")
        time.sleep(2)
    while core.profile_procs(profile):
        time.sleep(1)
    if time.time() - started < 5:
        log(f"{label} window never stayed open")
        return False
    return True


def run_pywebview() -> bool:
    try:
        import webview
    except Exception as e:  # noqa: BLE001
        log(f"pywebview unavailable: {e}")
        return False
    try:
        webview.create_window(TITLE, core.SHELL_URL, width=1600, height=950,
                              min_size=(800, 500), background_color="#2c2c2c")
        (DATA / "webview").mkdir(parents=True, exist_ok=True)
        webview.start(private_mode=False, storage_path=str(DATA / "webview"), icon=str(ICON))
        return True
    except Exception as e:  # noqa: BLE001
        log(f"pywebview failed: {e!r}")
        return False


def open_window() -> bool:
    argv = sys.argv
    if "--webview" in argv:
        return run_pywebview()
    if "--edge" in argv:
        e = find_edge()
        return bool(e) and run_chromium(e, EDGE_PROFILE, "Edge")
    chrome = find_chrome()
    if chrome and run_chromium(chrome, PROFILE, "Chrome"):
        return True
    if not chrome:
        log("Google Chrome not found")
    alt = find_chromium_like()
    if alt:
        log(f"falling back to {alt[0]}")
        if run_chromium(alt[1], PROFILE, alt[0]):
            return True
    edge = find_edge()
    if edge:
        log("falling back to Edge app mode (install Google Chrome to use it)")
        return run_chromium(edge, EDGE_PROFILE, "Edge")
    log("no Chrome, Chromium-like browser or Edge found")
    return False


def main() -> int:
    _logfile()
    set_app_id()
    log("start")
    if core.port_busy(core.APP_PORT):
        log("already running (control port 8093 answering) - not opening a second window")
        return 0
    try:
        srv = core.make_control_server(ctl)
    except OSError as e:
        log(f"cannot bind control port {core.APP_PORT}: {e}")
        return 1
    cleanup.srv = srv  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    atexit.register(cleanup)
    try:
        if not ctl.ensure_web():
            log("web server :5000 is not serving the GCS page - not opening a window")
            for line in list(ctl.log)[-15:]:
                log(f"  {line}")
            return 1
        log("web server answering")
        if "--headless" in sys.argv:
            while True:
                time.sleep(1)
        if not open_window():
            log("no window host available")
            return 1
        log("window closed")
    except KeyboardInterrupt:
        pass
    finally:
        cleanup()
    return 0


if __name__ == "__main__":
    sys.exit(main())
