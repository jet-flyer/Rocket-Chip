#!/usr/bin/env python3
"""STEMMA-safe probe flash. Procedure: docs/FLASHING.md.

Implements: park `u` (DTR held low) → halt both cores → write_image +
verify → MSP/PC from 0x10000000 → resume Core 0. Never `program`.
Never `reset halt`. No CDC → refuse. USB unplug is recovery, not a step.

Requires OpenOCD on :4444 (scripts/start_openocd_pico_sdk.ps1).
"""
from __future__ import annotations

import argparse
import socket
import sys
import time
from pathlib import Path

from _rc_test_common import (  # noqa: E402
    TARGET_VEHICLE_ANY,
    enter_cli_menu,
    find_target_port,
    open_classified_port,
)

REPO = Path(__file__).resolve().parents[1]
DEFAULT_ELF = REPO / "build_flight" / "rocketchip.elf"
OCD_HOST = "127.0.0.1"
OCD_PORT = 4444


def wait_port(timeout: float = 25.0):
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            port, banner = find_target_port(TARGET_VEHICLE_ANY, override=None)
        except Exception:
            port, banner = None, None
        if port is not None:
            return port, banner
        time.sleep(0.4)
    return None, None


def _open_no_dtr(port: str):
    # pyserial defaults pulse DTR on open/close. That USB-resets the
    # RP2350 into bootrom so CLI `u` never holds.
    import serial

    ser = serial.Serial()
    ser.port = port
    ser.baudrate = 115200
    ser.timeout = 2
    ser.dsrdtr = True
    ser.rtscts = False
    ser.dtr = False
    ser.rts = False
    ser.open()
    ser.dtr = False
    ser.rts = False
    return ser


def park() -> bool:
    port, _ = wait_port()
    if port is None:
        print("park: no CDC — refuse flash (would SYSRESETREQ a live bus)")
        return False
    ser = None
    try:
        ser = _open_no_dtr(port)
        time.sleep(0.3)
        ser.read(4000)
        ser.write(b"q")
        ser.flush()
        time.sleep(0.3)
        ser.read(4000)
        ser.write(b"u")
        ser.flush()
        time.sleep(0.5)
        print("park: sent u (DTR held low)")
        return True
    except Exception as exc:
        print("park", type(exc).__name__, exc)
        return False
    finally:
        if ser is not None:
            try:
                ser.dtr = False
                ser.rts = False
                ser.close()
            except Exception:
                pass


def ocd_session(commands: list[str], timeout: float = 180.0) -> str:
    sock = socket.create_connection((OCD_HOST, OCD_PORT), 5)
    sock.settimeout(timeout)
    buf = b""

    def read_until_prompt() -> str:
        nonlocal buf
        while b"> " not in buf:
            chunk = sock.recv(4096)
            if not chunk:
                raise RuntimeError("OpenOCD telnet closed")
            buf += chunk
        idx = buf.find(b"> ")
        out = buf[:idx]
        buf = buf[idx + 2 :]
        return out.decode("utf-8", "replace")

    replies = [read_until_prompt()]
    for cmd in commands:
        sock.sendall(cmd.encode("ascii") + b"\n")
        replies.append(read_until_prompt())
    sock.close()
    return "\n".join(replies)


def flash(elf: Path) -> int:
    elf_posix = elf.resolve().as_posix()
    commands = [
        "rp2350.cm0 cortex_m smp off",
        "targets rp2350.cm1",
        "halt",
        "targets rp2350.cm0",
        "halt",
        f"flash write_image erase {elf_posix}",
        f"verify_image {elf_posix}",
        "targets rp2350.cm0",
        "set vec [read_memory 0x10000000 32 2]",
        "reg msp [lindex $vec 0]",
        "reg pc [lindex $vec 1]",
        "reg xpsr 0x01000000",
        "resume",
    ]
    text = ocd_session(commands)
    print(text[-3000:])
    ok = "verified" in text.lower()
    print("flash", "OK" if ok else "CHECK", "park+write+vector resume cm0, no reset")
    return 0 if ok else 1


def dump() -> int:
    time.sleep(12)
    port, banner = wait_port()
    raw = getattr(banner, "raw", "") or ""
    print(raw[:500])
    if port is None:
        print("dump: no CDC")
        return 2
    with open_classified_port(port, target=TARGET_VEHICLE_ANY) as ser:
        enter_cli_menu(ser)
        ser.write(b"q")
        ser.flush()
        time.sleep(0.35)
        ser.read(4000)
        ser.write(b"b")
        ser.flush()
        time.sleep(1.6)
        hw = ser.read(24000).decode("utf-8", "replace")
        for line in hw.splitlines():
            if any(
                s in line
                for s in ("ICM", "DPS", "PMTK", "GPS", "FAIL", "N/A", "Hardware")
            ):
                print(line)
        ser.write(b"s")
        ser.flush()
        time.sleep(1.2)
        first = ser.read(8000).decode("utf-8", "replace")
        time.sleep(2.0)
        ser.write(b"s")
        ser.flush()
        time.sleep(1.2)
        second = ser.read(8000).decode("utf-8", "replace")
        for label, blob in (("s1", first), ("s2", second)):
            print("---", label, "---")
            for line in blob.splitlines():
                if "I=" in line or "last_quiesce" in line or "POR=" in line:
                    print(line)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", default=str(DEFAULT_ELF))
    parser.add_argument("--no-park", action="store_true")
    parser.add_argument("--dump", action="store_true", help="read banner/b/s after write")
    parser.add_argument("--dump-only", action="store_true")
    args = parser.parse_args()
    if args.dump_only:
        return dump()
    if not args.no_park:
        if not park():
            return 2
    rc = flash(Path(args.elf))
    if args.dump:
        dump_rc = dump()
        if rc != 0:
            return rc
        return dump_rc
    return rc


if __name__ == "__main__":
    sys.exit(main())
