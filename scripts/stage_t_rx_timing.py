#!/usr/bin/env python3
"""Compute the offset of station-packet RX events relative to vehicle's
most recent TX->RXW transition. This tells us WHERE in the vehicle's
RX window the successful station packets landed. Used to pick T2's
gate_ms value (council ask: 10th-percentile of observed RX window width).
"""
import re

for run in range(1, 6):
    log = f"logs/stage_t/t1_vehicle_run{run}.log"
    last_rxw_entry_us = None
    last_txa_entry_us = None
    offsets_in_rxw = []
    rxw_durations = []
    for line in open(log, encoding="utf-8", errors="replace"):
        m = re.search(r"state TX->RXW t=(\d+)", line)
        if m:
            last_rxw_entry_us = int(m.group(1))
            continue
        m = re.search(r"state RXW->TX t=(\d+)", line)
        if m:
            tx_us = int(m.group(1))
            if last_rxw_entry_us is not None:
                rxw_durations.append((tx_us - last_rxw_entry_us) / 1000.0)
            continue
        m = re.search(r"\[STAGE_T\] rx state=RXW .* t=(\d+)", line)
        if m and last_rxw_entry_us is not None:
            rx_us = int(m.group(1))
            offset_ms = (rx_us - last_rxw_entry_us) / 1000.0
            if 0 <= offset_ms < 250:
                offsets_in_rxw.append(offset_ms)

    rxw_durations = [d for d in rxw_durations if 0 < d < 250]
    label = f"Run {run}"
    if offsets_in_rxw:
        offsets_in_rxw.sort()
        n = len(offsets_in_rxw)
        p10 = offsets_in_rxw[n // 10] if n >= 10 else offsets_in_rxw[0]
        p50 = offsets_in_rxw[n // 2]
        p90 = offsets_in_rxw[int(n * 0.9)] if n >= 10 else offsets_in_rxw[-1]
        print(f"{label} rx-offsets (from RXW entry): "
              f"N={n}  p10={p10:.1f}ms  p50={p50:.1f}ms  p90={p90:.1f}ms  "
              f"max={offsets_in_rxw[-1]:.1f}ms")
    if rxw_durations:
        rxw_durations.sort()
        n = len(rxw_durations)
        p10 = rxw_durations[n // 10]
        p50 = rxw_durations[n // 2]
        p90 = rxw_durations[int(n * 0.9)]
        print(f"{label} RXW-durations:         "
              f"N={n}  p10={p10:.1f}ms  p50={p50:.1f}ms  p90={p90:.1f}ms  "
              f"max={rxw_durations[-1]:.1f}ms")
    print()
