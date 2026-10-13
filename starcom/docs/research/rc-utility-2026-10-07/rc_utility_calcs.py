#!/usr/bin/env python3
"""
rc_utility_calcs.py -- reproducible numbers for rc-product-utility-map.md
Prepared 2026-10-07 (CT). Run: python3 rc_utility_calcs.py [--fast]
Every input is cited in the comment next to it. Nothing here is board-measured.
Sections:
  S1  free-space path loss and range ceilings (915 MHz, SX1276 FSK + LoRa)
  S2  frequency scaling 915 -> 2250 / 2440 MHz
  S3  Part 15.249 field-strength limit as EIRP
  S4  polarization mismatch (linear-linear)
  S5  residual carrier power split (211.1 60 deg)
  S6  frame overhead bytes and airtime (LoRa and FSK)
  S7  MAC turnaround time and energy
  S8  Doppler at 915 MHz (example inputs)
  S9  convolutional K=7 r=1/2 hard-decision Monte Carlo (noncoherent BFSK model)
  S10 LDPC (2048,1024) airtime and decoder cost estimate
  S11 SDLS (355.0 USLP baseline) overhead airtime
  S12 time code (301.0 CUC) sizes
  S13 ranging: time-tag resolution, SX1280 LSB
  S14 slant-range slots (geometry only, no customer data)
  --- v2 additions ---
  S15 Tier 1 PHY trade: LoRa BW500 SF7-9, hopping FSK, fixed FSK (15.249), frame times
  S16 FSK hop plan: 20 dB BW (Carson approx), channel spacing, retune gap in bits, PSD
  S17 Ground-antenna variable column (RX gain) and station-TX limits (15.247(b)(4), 15.249)
  S18 Ranging: SX1280 exchange time, DWM3000 EIRP cap, two-station + baro geometry
  S19 PIO / MCU timing budgets (bit periods, FIFO fill, PIO tick -> range)
  S20 GPS export-control numbers (unit conversions only)
"""
import math, sys
import numpy as np

C = 299_792_458.0  # m/s
FAST = "--fast" in sys.argv

def fspl_db(d_km, f_mhz):
    # Friis free-space path loss: 20log10(d_km)+20log10(f_MHz)+32.45
    return 20*math.log10(d_km) + 20*math.log10(f_mhz) + 32.45

def range_km(allowed_pl_db, f_mhz):
    return 10**((allowed_pl_db - 20*math.log10(f_mhz) - 32.45)/20)

hdr = lambda s: print("\n" + "="*78 + "\n" + s + "\n" + "="*78)

# ---------------------------------------------------------------- S1
hdr("S1  FSPL and free-space range ceilings at 915 MHz (LOS, no cable loss)")
F = 915.0
P_TX = 20.0       # dBm, SX1276 PA_BOOST max (DS Rev 7 Table 6 p.14 IDDT row "+20 dBm on PA_BOOST")
G_GND = 2.0       # dBi, TBS Immortal T V2 ground, TBS spec page (prod:xf_immortal_t_v2_ee)
G_VEH = 2.9       # dBi, VAS 915 MHz XFire Pro vehicle, TBS page (prod:vas_915mhz_xfpro)
print(f"FSPL(1 km, 915 MHz) = {fspl_db(1, F):.2f} dB ; +20 dB per decade of distance")
# SX1276 DS Rev 7 Table 8 p.16, Band 1 (RFS_F_HF): shared path / split path + LnaBoost
# Conditions: 0.1% BER, bit synchronizer on (p.16 header text). RxBw per footnotes p.17.
fsk = [  # (label, kb/s, shared dBm, split dBm)
    ("1.2 kb/s FDA 5k", 1.2, -119, -123),
    ("4.8 kb/s FDA 5k", 4.8, -115, -119),
    ("38.4 kb/s FDA 40k", 38.4, -105, -109),
    ("250 kb/s FDA 62.5k", 250, -92, -96),
]
# SX1276 DS Rev 7 Table 10 p.20 RFS_L*_HF (split path, LnaBoost): 1% PER, 64-byte payload, CR4/6 (p.19 conditions)
lora = [("LoRa SF7 BW125", 5.47, None, -123), ("LoRa SF7 BW250", 10.94, None, -120),
        ("LoRa SF7 BW500", 21.88, None, -116), ("LoRa SF12 BW125", 0.29, None, -136)]
def lora_rb(sf, bw_khz, cr=1):  # DS Rev 7 sec 4.1.1.1: Rb = SF * (4/(4+CR)) / (2^SF/BW)
    return sf*(4/(4+cr))/((2**sf)/(bw_khz*1e3))/1e3
print(f"LoRa raw bit rates (CR4/5): SF7/125 {lora_rb(7,125):.2f}, SF7/250 {lora_rb(7,250):.2f}, "
      f"SF7/500 {lora_rb(7,500):.2f}, SF12/125 {lora_rb(12,125):.3f} kb/s")
print("\nCeiling = max distance where Prx >= sensitivity + margin. Pol/pattern margin NOT included here (see S4).")
print(f"{'mode':22s} {'sens':>8s} | {'0 dBi m0':>9s} {'0 dBi m10':>9s} | {'2+2.9 dBi m0':>12s} {'2+2.9 m10':>10s} | {'15.249 m10':>10s}")
EIRP_249 = None
def eirp_15249():
    E = 50e-3; d = 3.0       # 47 CFR 15.249(a),(c): 50 mV/m at 3 m (phy_legality_us.md sec 2)
    return 10*math.log10((E*d)**2/30/1e-3)   # P = (E d)^2/30 W (isotropic far-field)
EIRP_249 = eirp_15249()
for lab, rb, sh, sp in fsk + lora:
    for which, s in (("shared", sh), ("split", sp)):
        if s is None: continue
        r = []
        for g in (0.0, G_GND + G_VEH):
            for m in (0, 10):
                r.append(range_km(P_TX + g - s - m, F))
        r249 = range_km(EIRP_249 + G_GND - s - 10, F)   # 15.249 caps EIRP, so TX antenna gain is inside EIRP
        print(f"{lab+' '+which:22s} {s:8.0f} | {r[0]:9.1f} {r[1]:9.1f} | {r[2]:12.1f} {r[3]:10.1f} | {r249:10.2f}")

print("\nS1b Tier 2 parts, same antennas (2.0 + 2.9 dBi), 10 dB margin, free space. Criteria differ per datasheet row.")
t2 = [  # (part/mode, f MHz, TX dBm, sens dBm, source)
 ("SX1276 FSK 38.4k split", 915, 20.0, -109, "SX1276 DS Rev7 T8 p.16, 0.1% BER"),
 ("SX1262 FSK 38.4k boost", 915, 22.0, -109, "SX1261/2 DS Rev1.1 T3-8 p.19 / p.21"),
 ("SX1262 FSK 250k boost", 915, 22.0, -104, "SX1261/2 DS Rev1.1 T3-8 p.19"),
 ("SX1262 LoRa SF7/125", 915, 22.0, -124, "SX1261/2 DS Rev1.1 T3-8 p.19"),
 ("LR1121 FSK 38.4k boost", 915, 22.0, -111, "LR1121 DS Rev2.1 T3-8 p.17; +22 dBm sec 1.2.1"),
 ("LR1121 FSK 250k boost", 915, 22.0, -105, "LR1121 DS Rev2.1 T3-8 p.17"),
 ("LR1121 LoRa SF7/125 S-band", 2100, 13.0, -118, "LR1121 DS Rev2.1 T3-9; +13 dBm HF sec 1.2.1"),
 ("SX1280 LoRa SF7/1625 HS", 2440, 12.5, -108, "SX1280 DS Rev3.2 p.22 / p.26"),
 ("LR1110 FSK 4.8k RxBoosted", 915, 22.0, -119, "LR1110 DS Rev1.5 T3-12 p.19 (room note B10)"),
 ("LR1110 FSK 38.4k RxBoosted", 915, 22.0, -111, "LR1110 DS Rev1.5 T3-12 p.19 (room note B10)"),
 ("LR1110 FSK 250k RxBoosted", 915, 22.0, -105, "LR1110 DS Rev1.5 T3-12 p.19 (room note B10)"),
 ("LR1110 LoRa SF7/BW500", 915, 22.0, -121, "LR1110 DS Rev1.5 T3-12 p.19, 1% PER 64 B (room note B10)"),
 ("LR1110 LoRa SF12/BW500", 915, 22.0, -134, "LR1110 DS Rev1.5 T3-12 p.19 (room note B10)"),
 ("SX1280 FSK 250k", 2440, 12.5, -98, "SX1280 DS Rev3.3 p.25 (room note B10)"),
 ("SX1280 FSK 1M HS", 2440, 12.5, -94, "SX1280 DS Rev3.3 p.25 (room note B10)"),
 ("SX1280 LoRa SF12/BW203", 2440, 12.5, -132, "SX1280 DS Rev3.3 p.25 (room note B10)"),
 ("AT86RF215 2FSK 50ks noFEC", 915, 14.5, -109, "AT86RF215 DS 42415E p.1 / T10-17 p.195 (868 MHz, PER<10%)"),
 ("AT86RF215 2FSK 50ks FEC", 915, 14.5, -114, "AT86RF215 DS 42415E T10-17 p.195 (25 kb/s info)"),
]
for lab, f, ptx, sens, src in t2:
    r = range_km(ptx + G_GND + G_VEH - sens - 10, f)
    print(f"  {lab:28s} {f:5d} MHz {ptx:5.1f} dBm {sens:5.0f} dBm -> {r:7.1f} km   [{src}]")
print(f"\nAntenna gain sum {G_GND+G_VEH:.1f} dB multiplies range by {10**((G_GND+G_VEH)/20):.3f}")
print("Repo standards/RF_COMPLIANCE.md uses 3 dBi + 3 dBi; TBS pages give 2.0 and 2.9 dBi.")

# ---------------------------------------------------------------- S2
hdr("S2  Frequency scaling of FSPL (same gains, power, sensitivity)")
for f2 in (2250.0, 2440.0):
    d = 20*math.log10(f2/F)
    print(f"{f2:.0f} MHz vs 915 MHz: +{d:.2f} dB FSPL; range ceiling / {10**(d/20):.2f}")

# ---------------------------------------------------------------- S3
hdr("S3  47 CFR 15.249 fundamental limit as EIRP")
print(f"50 mV/m at 3 m -> EIRP = {EIRP_249:.2f} dBm ({10**(EIRP_249/10):.3f} mW)")
print(f"vs +20 dBm conducted + 2.9 dBi (= {P_TX+G_VEH:.1f} dBm EIRP): {P_TX+G_VEH-EIRP_249:.1f} dB lower; "
      f"range / {10**((P_TX+G_VEH-EIRP_249)/20):.1f}")


print("\nS3b SX1276 FSK occupied bandwidth bound vs the 15.247(a)(2) 500 kHz 6 dB minimum")
# DS Rev 7 Table 7 p.15: FDA + BRF/2 <= 250 kHz. Carson BW = 2*(FDA + BR/2) is ~98% power BW, wider than the 6 dB BW.
for fda, br in ((62.5, 250), (200, 100), (100, 300), (5, 4.8), (20, 38.4)):
    ok = fda + br/2 <= 250
    print(f"  FDA {fda:6.1f} kHz, BR {br:6.1f} kb/s: chip-legal={ok}, Carson BW = {2*(fda+br/2):6.1f} kHz")
print("  Max Carson BW allowed by the chip = 2*250 = 500 kHz. The 6 dB BW of a chip setting is NOT computed here;")
print("  it must be measured (open check B8 / IRL test T5).")
print("\nS3c FHSS dwell vs MAC contact (15.247(a)(1)(i): <=0.4 s per channel per 20 s if 20 dB BW < 250 kHz, >=50 channels)")
print("  RC vehicle Send_Duration N=11 at 10 Hz = 1.1 s (starcom_adapt/README.md) > 0.4 s -> must hop inside one contact.")
print("  SX1276 hop time 20-50 us (DS Rev 7 Table 7 p.15, TS_HOP; room note cites p.16).")
print(f"  50 channels in 910-928 MHz (XFire Pro stated range) -> spacing {18e3/50:.0f} kHz; in 902-928 -> {26e3/50:.0f} kHz.")

# ---------------------------------------------------------------- S4
hdr("S4  Linear-to-linear polarization mismatch 20log10(cos theta)")
for th in (0, 30, 45, 60, 70, 80, 85, 89):
    L = -20*math.log10(max(math.cos(math.radians(th)), 1e-12))
    print(f"theta {th:2d} deg -> loss {L:6.2f} dB")

# ---------------------------------------------------------------- S5
hdr("S5  Residual carrier (211.1-B-4 3.3.5.2: 60 deg +/-5%)")
for b in (57.0, 60.0, 63.0):
    pc = math.cos(math.radians(b))**2
    print(f"beta {b:.0f} deg: carrier {100*pc:.1f}% , data {100*(1-pc):.1f}% -> data loss {-10*math.log10(1-pc):.2f} dB")
print("Bi-Phase-L: two symbol transitions per bit -> main lobe about 2x NRZ (CCSDS 413.0-G-3 background).")

# ---------------------------------------------------------------- S6
hdr("S6  Overhead bytes and airtime")
ASM, V3, SPH, CRC32 = 3, 5, 6, 4      # 211.2-B-3 3.2.3 (FAF320), 211.0-B-6 3.2.2, 133.0-B-2 4.1.3, 211.2 3.2.5
NAV_USER = 51                          # include/rocketchip/telemetry_state.h static_assert
nav_pltu = ASM + V3 + SPH + NAV_USER + CRC32
print(f"Nav PLTU = {ASM}+{V3}+{SPH}+{NAV_USER}+{CRC32} = {nav_pltu} B (repo kRadioConfigNavPltuBytes = 69)")
print(f"Overhead {nav_pltu-NAV_USER} B = {100*(nav_pltu-NAV_USER)/nav_pltu:.1f}% of the PLTU")
REPORT = ASM + V3 + 2 + CRC32
print(f"PLCW / token P-frame PLTU = {REPORT} B (byte_pump.cpp static_assert 14)")

def lora_toa_ms(pl, sf, bw_khz, cr=1, npre=8, ih=0, crc=1, de=0):
    # SX1276 DS Rev 7 sec 4.1.1.7 p.31
    ts = (2**sf)/(bw_khz*1e3)
    tpre = (npre + 4.25)*ts
    num = 8*pl - 4*sf + 28 + 16*crc - 20*ih
    npay = 8 + max(math.ceil(num/(4*(sf-2*de)))*(cr+4), 0)
    return 1e3*(tpre + npay*ts)
print("\nLoRa ToA (explicit header, chip CRC on, 8-sym preamble, CR4/5):")
for bw in (125, 250, 500):
    t69 = lora_toa_ms(69, 7, bw); t51 = lora_toa_ms(51, 7, bw); t14 = lora_toa_ms(14, 7, bw)
    print(f"  SF7/BW{bw}: 69 B {t69:6.2f} ms | 51 B raw user {t51:6.2f} ms | 14 B report {t14:5.2f} ms | "
          f"overhead cost {t69-t51:5.2f} ms ({100*(t69-t51)/t69:.0f}%)")

def fsk_toa_ms(payload, rb_kbps, pre=3, sync=3, lenbyte=1, chipcrc=0):
    # SX1276 packet: preamble + sync + length + payload + CRC (DS Rev 7 sec 2.1.13.2-3 pp.72-73)
    return 8*(pre + sync + lenbyte + payload + chipcrc)/rb_kbps
print("\nFSK packet ToA. Option A: ASM FAF320 used AS the chip sync word (3 B), PLTU after ASM = 66 B,")
print("  book CRC-32 kept, chip CRC off. Option B: chip 2-B sync + full 69-B PLTU + chip CRC-16 (double sync, double CRC).")
for rb in (4.8, 38.4, 50.0, 100.0, 250.0):
    a = fsk_toa_ms(nav_pltu-ASM, rb, sync=3, chipcrc=0)
    b = fsk_toa_ms(nav_pltu, rb, sync=2, chipcrc=2)
    raw = fsk_toa_ms(NAV_USER, rb, sync=3, chipcrc=2)
    print(f"  {rb:6.1f} kb/s: A {a:7.2f} ms | B {b:7.2f} ms | raw 51 B + chip CRC {raw:7.2f} ms | max nav Hz (A, 1 pkt/slot) {1000/a:6.1f}")
print("Preamble 3 B = 24 bits >= 12-bit minimum (DS Rev 7 sec 2.1.3.3 p.51).")
print("FSK FIFO = 64 B (DS p.66); a 69-B PLTU (or 70+ B packet) needs FifoThreshold refill (DS p.76).")

# ---------------------------------------------------------------- S7
hdr("S7  MAC turnaround cost (byte_pump.cpp flight_mac_mib lines 27-29: 10+10+10 ms)")
turn = 30.0
I_TX20 = 120.0   # mA at +20 dBm PA_BOOST, DS Rev 7 Table 6 p.14
for nav_ms, n in ((100, 11), (100, 1)):
    send = n*nav_ms
    print(f"N={n:2d}: vehicle send {send} ms ; one turn {turn:.0f} ms = {100*turn/(send+turn):.1f}% of the cycle (silent timer, not radiated)")
print(f"If the 10 ms carrier-only were RADIATED at +20 dBm: {I_TX20*0.010:.2f} mA*s per turn, no FSK receiver benefit (see S8).")

# ---------------------------------------------------------------- S8
hdr("S8  Doppler at 915 MHz (example inputs, not customer data)")
for v in (100, 300, 1000):
    print(f"v = {v:5d} m/s: df = {915e6*v/C:7.0f} Hz ({v/C*1e6:.2f} ppm; 1 ppm = 915 Hz)")
for g in (10, 20):
    a = 9.80665*g
    print(f"{g} g: df/dt = {915e6*a/C:6.0f} Hz/s ; drift in a 50 ms packet after lock = {915e6*a/C*0.05:5.1f} Hz")
print("211.1-B-4 3.4.5.1: UHF Doppler +/-10 kHz, 100 Hz/s (non-coherent), 200 Hz/s (coherent) -- Mars UHF figures.")

# ---------------------------------------------------------------- S9
hdr("S9  CCSDS K=7 r=1/2 conv code, HARD decisions, Monte Carlo")
# Code: 131.0-B-5 3.3 / B-6 4.3 (G1=171o, G2=133o). G2 inversion does not change BSC performance.
# Channel models (assumptions, see map): noncoherent BFSK p = 0.5 exp(-Es/2N0); coherent BPSK p = Q(sqrt(2Es/N0)).
G1, G2 = 0o171, 0o133
def parity(x):
    return bin(x).count("1") & 1
ns_out = np.zeros((64, 2, 2), dtype=np.uint8)   # [state, u] -> (c1,c2)
nxt = np.zeros((64, 2), dtype=np.int64)
for s in range(64):
    for u in range(2):
        reg = (u << 6) | s
        ns_out[s, u] = (parity(reg & G1), parity(reg & G2))
        nxt[s, u] = reg >> 1
pred = np.zeros((64, 2), dtype=np.int64); pred_u = np.zeros(64, dtype=np.int64)
for ns in range(64):
    pred_u[ns] = ns >> 5
    pred[ns] = [((ns << 1) & 63) | 0, ((ns << 1) & 63) | 1]
o_p0 = ns_out[pred[:, 0], pred_u]; o_p1 = ns_out[pred[:, 1], pred_u]   # (64,2)

def encode(bits):  # bits (B,T) with tail zeros already appended
    B, T = bits.shape; s = np.zeros(B, dtype=np.int64); out = np.zeros((B, T, 2), dtype=np.uint8)
    for t in range(T):
        u = bits[:, t]; out[:, t] = ns_out[s, u]; s = nxt[s, u]
    return out
def viterbi(r):  # r (B,T,2) hard bits; terminated in state 0
    B, T, _ = r.shape
    pm = np.full((B, 64), 10**6, dtype=np.int32); pm[:, 0] = 0
    dec = np.zeros((T, B, 64), dtype=np.uint8)
    for t in range(T):
        r0 = r[:, t, 0][:, None]; r1 = r[:, t, 1][:, None]
        bm0 = (r0 != o_p0[None, :, 0]).astype(np.int32) + (r1 != o_p0[None, :, 1])
        bm1 = (r0 != o_p1[None, :, 0]).astype(np.int32) + (r1 != o_p1[None, :, 1])
        m0 = pm[:, pred[:, 0]] + bm0; m1 = pm[:, pred[:, 1]] + bm1
        d = m1 < m0; dec[t] = d; pm = np.where(d, m1, m0)
    s = np.zeros(B, dtype=np.int64); out = np.zeros((B, T), dtype=np.uint8); ar = np.arange(B)
    for t in range(T-1, -1, -1):
        out[:, t] = s >> 5; d = dec[t, ar, s]; s = ((s << 1) & 63) | d
    return out

def p_ncfsk(es_n0):  return 0.5*np.exp(-es_n0/2)
def p_bpsk(es_n0):
    from math import erfc
    return 0.5*erfc(math.sqrt(es_n0))
rng = np.random.default_rng(20261007)
K = 552            # info bits = 69-byte PLTU
T = K + 6          # 6 tail bits (needed in packet mode; Prox-1 stream mode has no tail bits)
NF = 400 if FAST else 1500
def sim(model, ebn0_db):
    ebn0 = 10**(ebn0_db/10); esn0 = ebn0*K/(2*T)   # rate K/(2T) incl. tail
    p = model(esn0)
    bits = rng.integers(0, 2, size=(NF, T), dtype=np.uint8); bits[:, K:] = 0
    c = encode(bits)
    flips = (rng.random(c.shape) < p).astype(np.uint8)
    dhat = viterbi(c ^ flips)
    errs = (dhat[:, :K] != bits[:, :K])
    return errs.mean(), errs.any(axis=1).mean()
def uncoded(model, ebn0_db):
    p = model(10**(ebn0_db/10)); return p, 1-(1-p)**K
def find_db(fn, target, lo, hi, idx):
    # bisection on a monotone estimate (MC noise accepted; report +/- step)
    for _ in range(8 if FAST else 10):
        mid = (lo+hi)/2
        if fn(mid)[idx] > target: lo = mid
        else: hi = mid
    return (lo+hi)/2
results = {}
for name, model, lo, hi in (("noncoh BFSK", p_ncfsk, 4.0, 16.0), ("coh BPSK", p_bpsk, 1.0, 12.0)):
    u_ber = find_db(lambda x: uncoded(model, x), 1e-3, lo, hi, 0)
    u_per = find_db(lambda x: uncoded(model, x), 1e-2, lo, hi, 1)
    c_ber = find_db(lambda x: sim(model, x), 1e-3, lo-3, hi, 0)
    c_per = find_db(lambda x: sim(model, x), 1e-2, lo-3, hi, 1)
    results[name] = (u_ber, c_ber, u_per, c_per)
    print(f"{name:12s}: Eb/N0 for BER 1e-3 uncoded {u_ber:5.2f} dB, coded {c_ber:5.2f} dB -> gain {u_ber-c_ber:4.2f} dB")
    print(f"{'':12s}  Eb/N0 for 1% PER (69 B) uncoded {u_per:5.2f} dB, coded {c_per:5.2f} dB -> gain {u_per-c_per:4.2f} dB")

print("\nS9b What 0.1% BER means for a 69-B frame (the SX1276 FSK sensitivity criterion, DS p.16):")
print(f"  PER = 1-(1-1e-3)^552 = {1-(1-1e-3)**552:.2f}  (LoRa sensitivity criterion is 1% PER, 64-B payload, DS p.19)")
for name,(u_ber,c_ber,u_per,c_per) in results.items():
    print(f"  {name}: uncoded Eb/N0 shift from BER 1e-3 to 1% PER(69 B) = {u_per-u_ber:.2f} dB (model)")
print(f"(Monte Carlo, {NF} frames per point, bisection; treat as +/-0.3 dB.)")
print("Gain is at EQUAL INFORMATION RATE. At a FIXED chip bit rate the coded frame takes 2x airtime "
      f"({2*T/K:.3f}x incl. tail); at FIXED airtime the info rate halves.")
print("Cross-check: 130.1-G-3 p.3-6: soft-decision gain about 5.5 dB at BER 1e-5 (BPSK, AWGN);")
print("             130.1-G-3 4.5 p.4-7: hard decision loses > 2 dB vs soft.")
print("Vendor cross-check (different code: 802.15.4g NRNSC K=4 + interleaver, AT86RF215 Table 10-17 p.195):")
print("  2FSK h=1: FEC at 100 ksym/s (50 kb/s info) -111 dBm vs no FEC at 50 ksym/s (50 kb/s) -109 dBm -> 2 dB net")
print("  2FSK h=0.5/1: FEC 100 ksym/s h=0.5 -109 vs no-FEC 50 ksym/s h=1 -109 -> 0 dB net")
print(f"Viterbi cost: {64*2} add-compare-select per info bit; 69-B PLTU = {T*64*2} ACS; traceback memory {T*64//8} B (1 bit/state/step).")

# ---------------------------------------------------------------- S10
hdr("S10 LDPC (2048,1024) r=1/2 (211.2-B-3 3.4.4; 131.0-B-5 7.4)")
CW, CSM = 2048, 64
print(f"Codeword + CSM = {CW+CSM} bits = {(CW+CSM)//8} B per 1024 info bits (128 B); Rd/Rcs = {1024/(CW+CSM):.5f} (211.0 B1.7.11 lists .48484)")
print(f"One 69-B PLTU in one codeword: {(CW+CSM)//8} B on air vs 69 B uncoded = {(CW+CSM)/8/69:.2f}x (rest is idle PN fill)")
for rb in (38.4, 50, 100, 250):
    print(f"  {rb:6.1f} kb/s: one codeword {(CW+CSM)/rb:6.1f} ms vs uncoded PLTU {8*69/rb:5.1f} ms")
M = 512; edges = 15*M; cols = 5*M
print(f"H1/2 (131.0-B-5 7.4.2.2): 15 circulant/permutation blocks x M={M} -> {edges} edges, {cols} variable nodes ({M} punctured)")
for it in (10, 25, 50):
    cyc = it*2*edges*10   # ASSUMPTION: about 10 cycles per edge update (min-sum, int8) -- order of magnitude only
    print(f"  {it:2d} iterations: ~{cyc/1e6:5.1f} Mcycles -> ~{cyc/150e6*1e3:5.1f} ms at 150 MHz (RP2350 DS p.13) [ESTIMATE, bench to confirm]")
print(f"  RAM (int8 messages + int8 LLR): ~{(edges+cols)/1024:.1f} KiB (+ index tables ~{2*edges/1024:.0f} KiB if 16-bit) [ESTIMATE]")

# ---------------------------------------------------------------- S11
hdr("S11 SDLS 355.0-B-2 USLP baseline (Annex E4.2/E4.3, p.E-4/E-5)")
SH, ST = 14, 16
for rb in (38.4, 50, 100, 250):
    print(f"  {rb:6.1f} kb/s: +{SH+ST} B = +{8*(SH+ST)/rb:5.2f} ms per frame")
print(f"  LoRa SF7/BW250: 69 B {lora_toa_ms(69,7,250):.2f} ms -> 69+30 B {lora_toa_ms(99,7,250):.2f} ms")
print(f"  As a share of the nav PLTU: {100*(SH+ST)/(nav_pltu+SH+ST):.0f}% of the protected frame")
print("  355.0-B-2 2.1 p.2-1: 'not applicable for use with the Proximity-1 Space Data Link Protocol.'")

# ---------------------------------------------------------------- S12
hdr("S12 CCSDS Unsegmented Code (301.0-B-4 3.2, p.3-1/3-2) vs raw uint32 ms")
print("raw met_ms: 4 B, 1 ms, wraps after %.1f days" % (2**32/1000/86400))
for co, fi in ((2, 2), (3, 1), (4, 0), (4, 2)):
    print(f"CUC implicit P-field, {co} coarse + {fi} fine: {co+fi} B, resolution {1/(256**fi)*1e3 if fi else 1000:.4g} ms, "
          f"range {256**co/86400:.2f} days")

# ---------------------------------------------------------------- S13
hdr("S13 Ranging numbers")
for dt in (0.1e-6, 1e-6, 10e-6, 100e-6):
    print(f"round-trip timing error {dt*1e6:6.1f} us -> range error {C*dt/2:8.1f} m")
for rb in (38.4, 50.0, 100.0, 250.0):
    tb = 1/(rb*1e3)
    print(f"one bit period at {rb:6.1f} kb/s = {tb*1e6:5.1f} us -> two-way range error ~ {C*tb/2/1e3:5.2f} km (DIO2 SyncAddress edge, DS Table 30)")
for bw in (0.406, 0.8125, 1.625):
    print(f"SX1280 raw LSB (DS Rev 3.2 Table 14-63): 150/(2^12*{bw}) = {150/(4096*bw):.4f} m")

# ---------------------------------------------------------------- S14
hdr("S14 Slant range SLOTS (geometry only; fill with Goddard's cited apogee/downrange)")
print("slant = sqrt(apogee^2 + downrange^2). No customer statistic is assumed here.")
def slant(h_km, x_km): return math.hypot(h_km, x_km)
print("Example grid (NOT customer data):")
for h in (1, 3, 10):
    print("  apogee %2d km: " % h + ", ".join(f"x={x} km -> {slant(h,x):.1f} km" for x in (0, 2, 5)))


# ================================================================ v2 additions
# ---------------------------------------------------------------- S15
hdr("S15 Tier 1 PHY trade (SX1276 split path, +20 dBm, 2.9 dBi vehicle, 2.0 dBi ground, 10 dB margin, free space)")
# LoRa BW500 sensitivities: SX1276 DS Rev 7 Table 10 p.20 (split path), SF7 -116, SF8 -119, SF9 -122 (room note B, p.19-20)
G = G_GND + G_VEH
rows = [("LoRa BW500 SF7 (15.247 DTS)", P_TX, -116, lora_rb(7,500)),
        ("LoRa BW500 SF8 (15.247 DTS)", P_TX, -119, lora_rb(8,500)),
        ("LoRa BW500 SF9 (15.247 DTS)", P_TX, -122, lora_rb(9,500)),
        ("FSK 38.4k hopping (15.247 FHSS)", P_TX, -109, 38.4),
        ("FSK 4.8k hopping (15.247 FHSS)", P_TX, -119, 4.8),
        ("FSK 250k hopping (20 dB BW >= 250 kHz rule set)", P_TX, -96, 250.0)]
for lab, ptx, sens, rb in rows:
    print(f"  {lab:50s} TX {ptx:4.1f} dBm sens {sens:5d} dBm Rb {rb:6.2f} kb/s -> {range_km(ptx+G-sens-10, F):7.1f} km")
for lab, sens in (("FSK 1.2k fixed (15.249)", -123), ("FSK 4.8k fixed (15.249)", -119), ("FSK 38.4k fixed (15.249)", -109)):
    print(f"  {lab:50s} EIRP {EIRP_249:5.2f} dBm sens {sens:5d} -> {range_km(EIRP_249 + G_GND - sens - 10, F):7.2f} km")
print("  Note: 15.247(b)(3) DTS / (b)(1)-(2) FHSS 1 W conducted; SX1276 max +20 dBm, so the chip, not the rule, sets TX.")
print("  250k row: 20 dB BW >= 250 kHz -> >=25 channels; 0.25 W (24 dBm) cap if <50 channels (15.247(b)(2)); +20 dBm is below both.")
print("\nS15b 69-B nav PLTU on air, FSK option A (3 B preamble + ASM as 3 B sync + 1 len + 66 B) = 73 B = 584 bits;")
print("  room figure 616 bits (77 B) uses a longer preamble. Both shown:")
for rb in (1.2, 4.8, 38.4, 250.0):
    print(f"  {rb:6.1f} kb/s: 584 bits {584/rb:7.1f} ms | 616 bits {616/rb:7.1f} ms")
print("  LoRa BW500 nav (69 B, CR4/5, 8-sym preamble, explicit header, chip CRC):")
for sf in (7, 8, 9):
    print(f"   SF{sf}: {lora_toa_ms(69, sf, 500):6.2f} ms")
print("  Unlimited-length mode (DS p.74) drops the length byte: -8 bits per frame,"
      f" e.g. {8/38.4:.3f} ms at 38.4k, {8/250*1e3:.0f} us at 250k.")

# ---------------------------------------------------------------- S16
hdr("S16 FSK hop plan numbers (SX1276)")
for fda, br in ((20, 38.4), (5, 4.8), (62.5, 250)):
    print(f"  Carson approx 2*(Fdev+Rb/2): Fdev {fda} kHz, Rb {br} kb/s -> {2*(fda+br/2):6.1f} kHz "
          f"({'<' if 2*(fda+br/2)<250 else '>='} 250 kHz: {'50 ch, 1 W' if 2*(fda+br/2)<250 else '25 ch; 0.25 W if <50 ch'})")
print("  Carson is an approximation; the 15.247 20 dB BW must be measured (IRL test T5).")
for band, lo, hi in (("902-928", 902, 928), ("910-928 (XFire Pro)", 910, 928)):
    for n in (25, 50):
        print(f"  {band:20s} {n} ch -> spacing {(hi-lo)*1e3/n:6.0f} kHz")
for ts in (20e-6, 50e-6):
    print(f"  TS_HOP {ts*1e6:.0f} us = " + ", ".join(f"{ts*rb*1e3:.2f} bits @ {rb} kb/s" for rb in (4.8, 38.4, 250)))
print("  Dwell rule (15.247(a)(1)(i)): <=0.4 s per channel in any 20 s window (20 dB BW < 250 kHz, >=50 channels).")
for n in (50, 64, 80):
    print(f"   {n} channels used equally: time per channel per 20 s = 20/{n} = {20/n:.3f} s (limit 0.4 s; margin {0.4-20/n:+.3f} s)")
print("   With exactly 50 channels used equally, every channel sits AT the 0.4 s limit, whatever the dwell length.")
print("   Dwell per hop must also be <= 0.4 s; one MAC contact (1.1 s) therefore spans >= 3 hops.")
print("\nS16b Average PSD per 3 kHz at +20 dBm if spread flat (assumption for LoRa chirp; FSK is NOT flat):")
for bw in (500, 635.1, 713):
    print(f"  {bw:6.1f} kHz: {20 - 10*math.log10(bw/3):5.2f} dBm/3 kHz (limit 8 dBm/3 kHz, 15.247(e))")
print("  FSK concentrates power near +/-Fdev; peak PSD per 3 kHz must be measured (IRL test T5).")

# ---------------------------------------------------------------- S17
hdr("S17 Ground antenna as a variable (downlink: ground gain is RX gain)")
ground = [("TBS Immortal T V2 (baseline)", 2.0, 0.0),
          ("VAS 915 LongShot (linear)", 7.5, 0.0),
          ("VAS 900 Crosshair Xtreme (CP) vs linear vehicle", 10.25, 3.0),
          ("Laird PC9013N yagi 13 dBd", 13 + 2.15, 0.0)]
print("  dBi = dBd + 2.15. Linear-to-circular mismatch = 3 dB (applied to the CP row).")
modes = [("FSK 4.8k fixed 15.249", None, -119), ("FSK 38.4k fixed 15.249", None, -109),
         ("FSK 38.4k hop/DTS +20", P_TX, -109), ("LoRa BW500 SF7 +20", P_TX, -116), ("LoRa BW500 SF9 +20", P_TX, -122)]
print(f"  {'ground antenna':48s} {'net dBi':>7s} | " + " | ".join(f"{m[0]:>22s}" for m in modes))
for name, g, pol in ground:
    gn = g - pol
    cells = []
    for lab, ptx, sens in modes:
        eirp = EIRP_249 if ptx is None else ptx + G_VEH
        cells.append(f"{range_km(eirp + gn - sens - 10, F):19.1f} km")
    print(f"  {name:48s} {gn:7.2f} | " + " | ".join(cells))
print("\n  Uplink (station transmits): TX gain counts.")
for name, g, pol in ground:
    pmax_247 = 30 - max(0.0, g - 6)
    p_249 = EIRP_249 - g
    print(f"  {name:48s} 15.247(b)(4) max conducted {pmax_247:5.2f} dBm (SX1276 +20 fits: {20 <= pmax_247}); "
          f"15.249 conducted for -1.25 dBm EIRP = {p_249:6.2f} dBm")

# ---------------------------------------------------------------- S18
hdr("S18 Ranging numbers")
def sx1280_rng_ms(sf, bw_hz, npre=12, nrng=15):  # SX1280 DS Rev 3.3 sec 7.5.4 p.56 (Goddard G4 file)
    return 1e3*(2**sf)/bw_hz*(npre + 2*nrng + 22.25)
for sf in (6, 9, 10):
    t = sx1280_rng_ms(sf, 1.625e6)
    print(f"  SX1280 ranging exchange SF{sf}/1625 kHz: {t:6.2f} ms ; 80 exchanges {80*t/1e3:5.2f} s")
eirp_uwb = -41.3 + 10*math.log10(500)
print(f"  DWM3000 UWB: -41.3 dBm/MHz (DS, 'most regions') x 500 MHz channel -> {eirp_uwb:.1f} dBm EIRP total")
print("  Two stations + baro altitude, example geometry (NOT customer data):")
for base, R in ((1.0, 3.0), (2.0, 3.0), (1.0, 1.5)):
    half = math.degrees(math.atan((base/2)/R))
    k = 1/math.sin(math.radians(half))
    print(f"   baseline {base} km, rocket {R} km out on the bisector: half-angle {half:5.2f} deg -> "
          f"along-track error ~ {k:4.1f} x range error (1 m -> {k:4.1f} m; 3 m -> {3*k:4.1f} m)")
print("  (Simplified 2D: error across the bisector direction ~ sigma_r / sin(half-angle); along it ~ sigma_r / cos.)")

# ---------------------------------------------------------------- S19
hdr("S19 PIO / MCU timing budgets (RP2350 at 150 MHz; RP2350 DS: PIO runs at system clock, 1 instr/cycle)")
fsys = 150e6
print(f"  1 PIO cycle = {1e9/fsys:.2f} ns -> two-way range equivalent {C/fsys/2:.2f} m")
for rb in (4.8, 38.4, 250):
    tb = 1/(rb*1e3)
    print(f"  {rb:6.1f} kb/s: bit {tb*1e6:7.2f} us = {tb*fsys:7.0f} CPU cycles ; 64-B FIFO fills in {64*8*tb*1e3:7.2f} ms ; "
          f"32-B threshold {32*8*tb*1e3:6.2f} ms")
print("  Continuous mode: the MCU (or PIO) must present each bit on DATA per DCLK edge: one event per bit period above.")

# ---------------------------------------------------------------- S20
hdr("S20 GPS export-control numbers (conversions only)")
print(f"  1,000 knots = {1000*1852/3600:.1f} m/s ; 60,000 ft = {60000*0.3048:.0f} m ; 600 m/s = {600*3600/1852:.0f} knots")
print(f"  Doppler at 915 MHz for 600 m/s: {915e6*600/C:.0f} Hz")

# ---------------------------------------------------------------- S21
hdr("S21 GPS L1 Doppler, Doppler rate, PLL jerk limit, oscillator g-sensitivity (v2c)")
F_L1 = 1575.42e6                      # GPS L1 carrier (154 x 10.23 MHz)
lam = C / F_L1
G0 = 9.80665
kd = F_L1 / C
print(f"  L1 wavelength {lam:.4f} m ; Doppler per m/s of line-of-sight speed = f/c = {kd:.4f} Hz per m/s")
for v in (300, 500, 515, 600):
    print(f"   {v:4d} m/s line of sight -> {kd*v:7.1f} Hz")
print(f"  Doppler rate per 1 g of line-of-sight acceleration: {kd*G0:.2f} Hz/s ; 4 g -> {4*kd*G0:.1f} Hz/s ; "
      f"40 m/s^2 -> {kd*40:.1f} Hz/s ; 20 m/s^3 jerk -> {kd*20:.1f} Hz/s^2")
print("  Third-order PLL steady-state jerk error: theta_e = (d3R/dt3) / w0^3, with w0 = Bn / 0.7845 (Kaplan form;")
print("  reproduces Ebinuma & Kato 2012: 25 Hz -> about 78 g/s). Threshold: theta_e <= 45 deg = lambda/8.")
for bn in (10, 15, 18, 25):
    w0 = bn/0.7845
    jmax = (lam/8) * w0**3
    print(f"   Bn {bn:2d} Hz: w0 {w0:5.2f} rad/s -> max LOS jerk {jmax:7.1f} m/s^3 = {jmax/G0:5.1f} g/s (jerk term alone, no noise)")
print("  Example jerk (NOT customer data): 0 -> 10 g in 0.1 s = "
      f"{10*G0/0.1:.0f} m/s^3 = {10/0.1:.0f} g/s along the line of sight")
gam = 1e-9   # OCXO g-sensitivity used in Sensors 2015, 15, 21673 (Ariane case)
print(f"  Oscillator g-sensitivity: df = Gamma * a * f_L1. Gamma = 1e-9 /g -> {gam*F_L1:.3f} Hz per g "
      f"(= {gam*F_L1/kd:.3f} m/s apparent LOS speed per g)")

# ---------------------------------------------------------------- S22
hdr("S22 UWB (DW1000 / DWM1000) free-space range ceilings (v2c)")
eirp = -41.3 + 10*math.log10(500)     # 15.519(c) / DS 'most regions', 500 MHz channel
def uwb_range_m(eirp_dbm, sens_dbm, f_hz, g_rx=0.0):
    allowed = eirp_dbm + g_rx - sens_dbm
    pl1 = 20*math.log10(4*math.pi*f_hz/C)
    return 10**((allowed - pl1)/20)
cases = [("APS017 Table 2 check, -16 dBm, -102 dBm, ch 2 (3993.6 MHz)", -16, -102, 3993.6e6),
         ("APS017 Table 2 check, -16 dBm, -102 dBm, ch 5 (6489.6 MHz)", -16, -102, 6489.6e6),
         ("DWM1000 ch 5, 110 kb/s, 1 % PER (-102 dBm/500 MHz)", eirp, -102, 6489.6e6),
         ("DWM1000 ch 5, 110 kb/s, 10 % PER (-106 dBm/500 MHz)", eirp, -106, 6489.6e6),
         ("DWM1000 ch 2, 110 kb/s, 10 % PER (-105, 'about 1 dB less')", eirp, -105, 3993.6e6),
         ("DWM1000 ch 5, 6.8 Mb/s, 10 % PER (-94 dBm/500 MHz)", eirp, -94, 6489.6e6)]
print(f"  EIRP limit over 500 MHz: {eirp:.1f} dBm ; 0 dBi RX, 0 dB margin, free space")
for name, e, s, f in cases:
    print(f"   {name:62s} -> {uwb_range_m(e, s, f):6.0f} m")
print("  A ground RX antenna gain G raises only the vehicle->ground leg (TX EIRP is capped, and TX gain counts in EIRP):")
for g in (6, 10, 15):
    print(f"   ch 5, -106 dBm, RX {g:2d} dBi -> {uwb_range_m(eirp, -106, 6489.6e6, g):6.0f} m (one-way leg only)")
print("  L3 slant 1.30-4.46 km (map 2.1) vs 141 m two-way ceiling at 0 dBi: ratio "
      f"{1300/uwb_range_m(eirp,-106,6489.6e6):.0f}x to {4460/uwb_range_m(eirp,-106,6489.6e6):.0f}x")
