# Replug log

USB VBUS unplug (slave POR, STEMMA 3V3 drops) when CDC or the MCU is stuck after a non-POR flash/debug path. Newest first. One line: date | board | image | likely cause.

- 2026-09-10 | vehicle Feather COM5 | `flight-bb4fc12` dirty halt-write | mixed up probe USB vs board USB; unplugging the Feather (real POR) brought RSSI green
- 2026-09-10 | vehicle Feather COM5 after USB replug (probe still on) | same `flight-bb4fc12` dirty halt-write | still Core 0 `pc=0xeffffffe` IPSR=HardFault, COM5 open-hang, station `Pkts=0` / red bar — had unplugged the probe, not the Feather; CMSIS-DAP kept 3V3 up so it was not a POR
- 2026-09-10 | vehicle Feather COM5 | `flight-bb4fc12` dirty (`build_flight_starcom` halt-write, always-on air restore on `main`) | vector-resume left Core 0 `pc=0xeffffffe` / CDC "unknown firmware" and open-hang (leftover IPSR/HardFault, not a POR)
