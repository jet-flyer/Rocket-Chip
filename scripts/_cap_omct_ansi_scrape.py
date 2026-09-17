import serial, time, re, datetime as dt
from pathlib import Path

out = Path(r"C:\Users\pow-w\Documents\Rocket-Chip-main\logs\gcs\omct_hello_2026-09-04.csv")
out.parent.mkdir(parents=True, exist_ok=True)

RSSI_RE = re.compile(rb"RSSI:\s*(?:\x1b\[[0-9;]*m)*(-?\d+)\s*dBm", re.I)
SNR_RE = re.compile(rb"SNR:\s*(-?\d+)\s*dB", re.I)
PKT_RE = re.compile(rb"Pkts:\s*(\d+)")

def open_cdc(port):
    s = serial.Serial()
    s.port = port
    s.baudrate = 115200
    s.timeout = 0.2
    s.dsrdtr = False
    s.rtscts = False
    s.dtr = False
    s.rts = False
    s.open()
    s.dtr = True
    time.sleep(0.4)
    return s

veh = None
try:
    veh = open_cdc("COM5")
    print("vehicle COM5 open")
except Exception as e:
    print("vehicle skip", e)

stn = open_cdc("COM7")
stn.reset_input_buffer()
stn.write(b"m")
time.sleep(0.5)
stn.reset_input_buffer()

rows = []
last_key = None
last_seq = "0"
pending = None  # (rssi, snr) waiting for pkt line
buf = bytearray()
t0 = time.time()
dur = 5.0
while time.time() - t0 < dur:
    chunk = stn.read(8192)
    if veh:
        try:
            veh.read(8192)
        except Exception:
            pass
    if not chunk:
        continue
    buf.extend(chunk)
    while True:
        nl = buf.find(b"\n")
        if nl < 0:
            if len(buf) > 20000:
                del buf[:-4000]
            break
        line = bytes(buf[: nl + 1])
        del buf[: nl + 1]
        pm = PKT_RE.search(line)
        if pm:
            last_seq = pm.group(1).decode()
            if pending is not None:
                rssi, snr = pending
                pending = None
                key = (last_seq, rssi, snr)
                if key != last_key:
                    last_key = key
                    ts = dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00","Z")
                    rows.append((ts, last_seq, rssi, snr))
            continue
        rm = RSSI_RE.search(line)
        sm = SNR_RE.search(line)
        if rm and sm:
            pending = (rm.group(1).decode(), sm.group(1).decode())

stn.close()
if veh:
    veh.close()

with out.open("w", encoding="utf-8", newline="\n") as f:
    f.write("timestamp,seq,rssi,snr\n")
    for r in rows:
        f.write(",".join(r) + "\n")

print(f"wrote {out} rows={len(rows)}")
if rows:
    print("first", ",".join(rows[0]))
    print("last", ",".join(rows[-1]))


