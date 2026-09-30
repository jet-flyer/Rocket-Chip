# launcher

Two entry points for the Open MCT glass. Both run the same feeds and share `rc_gcs_core.py`; pick one (they share the :5000/:8091 ports, so run one at a time).

- **`rc_gcs_app.pyw`** - stand-alone app window ("Rocket Chip GCS"). One window: a collapsible control bar (`app_shell.html`: data source, COM port, Start/Stop feed, Facsimile Play/Stop/Reset, status dots, log drawer) above the dashboard. Uses pywebview (Edge WebView2); falls back to an Edge `--app` window. Closing the window stops the feed and servers it started. A second launch while it is running exits quietly.
- **`rc_gcs.pyw`** - the original tk launcher ("Rocket Chip GCS (web launcher)"): pick Facsimile / Live text dashboard / Live MAVLink and a COM port, Start feed, then open the dashboard in your normal browser. `python rc_gcs.pyw --selftest` opens and closes the window.

Ports: **5000** static web (`http://localhost:5000/hello-world/`), **8091** telemetry WebSocket (one feed owner), **8092** facsimile control (`/play /stop /reset /status`), **8093** app control API (`/api/status|ports|start|stop|fac/*`, 127.0.0.1 only, used by the app window).

Desktop icons: run `powershell -ExecutionPolicy Bypass -File docs\gcs\openmct\launcher\make_shortcut.ps1` once. It creates "Rocket Chip GCS" (app) and "Rocket Chip GCS (web launcher)" with the same icon and replaces any older shortcut.
