#!/usr/bin/env python3
"""Stage T IVP-T1 runner: station ack_stress + parallel vehicle serial capture.

Usage:
    python scripts/stage_t_run.py --run N [--count 100] [--interval 1.0]

Runs scripts/ack_stress_test.py on the station while simultaneously tailing
the vehicle's USB-CDC serial for [STAGE_T] log lines. Outputs:
    logs/stage_t/t1_run<N>.csv             (station per-command CSV)
    logs/stage_t/t1_run<N>.log             (full station transcript)
    logs/stage_t/t1_vehicle_run<N>.log     (vehicle STAGE_T + all serial)

Vehicle = COM7 (VID:PID 2E8A:0009 w/ Stage T logging)
Station = COM9 (VID:PID 2E8A:0009 Fruit Jam)

The host harness runs ack_stress_test.py as a subprocess on the station,
while the main thread drains vehicle serial to the log file. Both
terminate together when ack_stress finishes.
"""
import argparse
import os
import subprocess
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("pyserial required", file=sys.stderr)
    sys.exit(1)


def tail_vehicle(port: str, log_path: str, stop_event: threading.Event):
    try:
        ser = serial.Serial(port, 115200, timeout=0.1)
    except Exception as e:
        print(f"[stage_t] failed to open vehicle {port}: {e}", file=sys.stderr)
        return
    time.sleep(0.2)
    ser.reset_input_buffer()
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(f"# Stage T vehicle capture from {port}\n")
        f.write(f"# Started: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.flush()
        buf = b""
        while not stop_event.is_set():
            try:
                chunk = ser.read(512)
            except Exception as e:
                f.write(f"# read error: {e}\n")
                break
            if not chunk:
                continue
            buf += chunk
            # Flush complete lines
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                try:
                    text = line.decode("utf-8", errors="replace").rstrip("\r")
                except Exception:
                    text = "<decode-err>"
                # Prefix every line with host wall-clock us for merge
                ts = time.time()
                f.write(f"{ts:.6f} {text}\n")
            f.flush()
        # Flush residual buffer
        if buf:
            try:
                text = buf.decode("utf-8", errors="replace").rstrip()
                ts = time.time()
                f.write(f"{ts:.6f} {text}\n")
            except Exception:
                pass
    try:
        ser.close()
    except Exception:
        pass


def main():
    ap = argparse.ArgumentParser(description="Stage T IVP-T1 runner")
    ap.add_argument("--run", type=int, required=True,
                    help="Run number (1..5) — used for output filenames")
    ap.add_argument("--count", type=int, default=100,
                    help="Commands to send (default 100)")
    ap.add_argument("--interval", type=float, default=1.0,
                    help="Send interval seconds (default 1.0 = 1 Hz)")
    ap.add_argument("--station-port", default="COM9",
                    help="Station serial port (default COM9)")
    ap.add_argument("--vehicle-port", default="COM7",
                    help="Vehicle serial port (default COM7)")
    args = ap.parse_args()

    os.makedirs("logs/stage_t", exist_ok=True)
    station_log = f"logs/stage_t/t1_run{args.run}.log"
    station_csv = f"logs/stage_t/t1_run{args.run}.csv"
    vehicle_log = f"logs/stage_t/t1_vehicle_run{args.run}.log"

    print(f"=== Stage T IVP-T1 run {args.run} ===")
    print(f"  Station: {args.station_port} -> {station_csv}")
    print(f"  Vehicle: {args.vehicle_port} -> {vehicle_log}")
    print(f"  {args.count} commands at {1.0/args.interval:.2f} Hz "
          f"(~{args.count * args.interval:.0f}s)")

    stop = threading.Event()
    veh_thread = threading.Thread(
        target=tail_vehicle,
        args=(args.vehicle_port, vehicle_log, stop),
        daemon=True)
    veh_thread.start()
    # Give vehicle serial a moment to start draining
    time.sleep(1.0)

    # Run ack_stress_test.py as subprocess on station
    # Set --duration high so --count is the limiting factor
    duration = max(args.count * args.interval * 1.5, 300)
    cmd = [
        sys.executable, "scripts/ack_stress_test.py",
        "--port", args.station_port,
        "--count", str(args.count),
        "--interval", str(args.interval),
        "--duration", str(duration),
        "--csv", station_csv,
    ]
    print(f"  Launching: {' '.join(cmd)}")
    try:
        with open(station_log, "w", encoding="utf-8") as f:
            proc = subprocess.Popen(cmd, stdout=subprocess.PIPE,
                                    stderr=subprocess.STDOUT,
                                    bufsize=1, universal_newlines=True)
            for line in proc.stdout:
                sys.stdout.write(line)
                sys.stdout.flush()
                f.write(line)
                f.flush()
            proc.wait()
    except KeyboardInterrupt:
        print("\n[stage_t] interrupted, stopping vehicle capture")
    finally:
        stop.set()
        veh_thread.join(timeout=3.0)

    print(f"\n=== Run {args.run} complete ===")
    print(f"  Station CSV:  {station_csv}")
    print(f"  Station log:  {station_log}")
    print(f"  Vehicle log:  {vehicle_log}")


if __name__ == "__main__":
    main()
