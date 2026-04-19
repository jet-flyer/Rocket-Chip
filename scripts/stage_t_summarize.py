#!/usr/bin/env python3
"""Summarize Stage T IVP-T1 runs."""
import csv
import statistics

runs = []
for i in range(1, 6):
    path = f"logs/stage_t/t1_run{i}.csv"
    rows = list(csv.DictReader(open(path)))
    acked = sum(1 for r in rows if r['ack_host_t'])
    first_try = sum(1 for r in rows if r['ack_retry_slot'] == '0')
    retry_1 = sum(1 for r in rows if r['ack_retry_slot'] == '1')
    retry_2 = sum(1 for r in rows if r['ack_retry_slot'] == '2')
    retry_3 = sum(1 for r in rows if r['ack_retry_slot'] == '3')
    failed = sum(1 for r in rows if r['failed'] == '1')
    n = len(rows)
    runs.append({
        'n': i, 'sent': n, 'acked': acked,
        'first_try': first_try, 'r1': retry_1, 'r2': retry_2, 'r3': retry_3,
        'failed': failed,
    })
    pct = lambda x: f"{100.0*x/n:.1f}%"
    label = "<1ft" if i == 1 else "3-4ft"
    print(f"Run {i} ({label:6s}) N={n:2d}  ACK={acked:2d} ({pct(acked)})  "
          f"first={first_try} r1={retry_1} r2={retry_2} r3={retry_3} fail={failed}")

print()
pooled_n = sum(r['sent'] for r in runs[1:])
pooled_ack = sum(r['acked'] for r in runs[1:])
pooled_first = sum(r['first_try'] for r in runs[1:])
pooled_fail = sum(r['failed'] for r in runs[1:])
pooled_r1 = sum(r['r1'] for r in runs[1:])
pooled_r2 = sum(r['r2'] for r in runs[1:])
pooled_r3 = sum(r['r3'] for r in runs[1:])
print(f"=== Pooled 3-4 ft (Runs 2-5): N={pooled_n} ===")
print(f"  Total ACK:   {pooled_ack} ({100.0*pooled_ack/pooled_n:.1f}%)")
print(f"  First-try:   {pooled_first} ({100.0*pooled_first/pooled_n:.1f}%)")
print(f"  Retry 1:     {pooled_r1}")
print(f"  Retry 2:     {pooled_r2}")
print(f"  Retry 3:     {pooled_r3}")
print(f"  Failed:      {pooled_fail} ({100.0*pooled_fail/pooled_n:.1f}%)")

first_try_rates = [100.0*r['first_try']/r['sent'] for r in runs[1:]]
total_ack_rates = [100.0*r['acked']/r['sent'] for r in runs[1:]]
print()
print(f"=== Run-to-run variance (3-4 ft runs 2-5) ===")
print(f"  First-try rate: mean={statistics.mean(first_try_rates):.1f}%  "
      f"sd={statistics.stdev(first_try_rates):.1f}%  "
      f"range=[{min(first_try_rates):.1f}, {max(first_try_rates):.1f}]%")
print(f"  Total ACK rate: mean={statistics.mean(total_ack_rates):.1f}%  "
      f"sd={statistics.stdev(total_ack_rates):.1f}%  "
      f"range=[{min(total_ack_rates):.1f}, {max(total_ack_rates):.1f}]%")

print()
print(f"=== Run 1 control (<1 ft, reproduces IVP-132a.5 conditions) ===")
r = runs[0]
print(f"  First-try: {100.0*r['first_try']/r['sent']:.1f}%")
print(f"  Total ACK: {100.0*r['acked']/r['sent']:.1f}%")

print()
print("=== Vehicle-side RX events (from STAGE_T logs) ===")
for i in range(1, 6):
    log = f"logs/stage_t/t1_vehicle_run{i}.log"
    rx_ok = rx_err = 0
    try:
        for line in open(log, encoding='utf-8', errors='replace'):
            if '[STAGE_T] rx ' in line:
                if 'crc=ok' in line:
                    rx_ok += 1
                elif 'crc=err' in line:
                    rx_err += 1
    except FileNotFoundError:
        print(f"  Run {i}: (no log)")
        continue
    print(f"  Run {i}: rx_crc_ok={rx_ok}  rx_crc_err={rx_err}")
