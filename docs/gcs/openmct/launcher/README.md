# launcher

Desktop window for the Open MCT glass: pick Facsimile, Live (station text dashboard) or Live (MAVLink) and a COM port, then Start feed / Open dashboard. Facsimile Play/Stop/Reset are in the window. It starts the :5000 web server if nothing is serving it, and owns the single :8091 feed (any stray feed console is stopped on Start).

Desktop icon: run `powershell -ExecutionPolicy Bypass -File docs\gcs\openmct\launcher\make_shortcut.ps1` once. It points `pythonw` at `rc_gcs.pyw` (no console).
