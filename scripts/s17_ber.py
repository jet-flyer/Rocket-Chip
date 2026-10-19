"""S17 deliverable 2b: noncoherent BER vs Eb/N0 by Monte Carlo, AWGN, h = 0.5.
Run: python3 s17_ber.py  -> ber_results.json, s17_ber.png   (about 10-20 min on 8 cores)
Receiver (method choices that affect the result, all stated):
- Predetection filter: Gaussian IF filter, two-sided 3 dB bandwidth Bi (BiT = Bi/Rb),
  implemented as a complex-baseband Gaussian LPF with one-sided 3 dB bandwidth Bi/2.
  BiT swept over 0.8, 1.0, 1.2, 1.4, 1.6 for BT 0.25 and 0.3 (Simon & Wang 1984 Table I puts the 1-bit optimum near 1.2 for BT 0.32 and 1.4 for BT 0.25); 1.0 only for the context rows.
- Detector D1: 1-bit differential detection, z_k = Im{ r(kT+T/2) r*(kT-T/2) }, decide sign.
- Detector D2: limiter-discriminator with integrate-and-dump over one bit
  (sum of sample-to-sample phase increments over the bit, so FM clicks are included), decide sign.
- Ideal bit timing (decision window centred on each bit), no frequency offset, no fading.
- sps = 8 samples/bit; noise variance per complex sample = sps / (Eb/N0) (signal power 1).
- Each point runs until >= 300 errors on both detectors or 4e7 bits. Seed fixed per point.\nA curve stops at the first Eb/N0 where both detectors are below 3e-7.
Eb/N0 at a target BER is linear interpolation of log10(BER) between grid points (0.5 dB grid).
"""
import json, sys
import numpy as np
from multiprocessing import Pool
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from s17_common import CASES, modulate, gauss_lpf_taps

SPS = 8
CHUNK = 2 ** 19
MIN_ERR, MAX_BITS = 300, 40_000_000
EBN0 = np.arange(4.0, 32.01, 0.5)
BT = dict(CASES)


def run_point(args):
    name, bit, ebn0 = args
    bt = BT[name]
    seed = int(1000 * ebn0) + int(100 * bit) * 100000 + [n for n, _ in CASES].index(name) * 10 ** 8
    rng = np.random.default_rng(seed)
    taps = gauss_lpf_taps(bit / 2.0, SPS)
    sigma2 = SPS / 10 ** (ebn0 / 10)
    e1 = e2 = nb = 0
    h = SPS // 2
    while (min(e1, e2) < MIN_ERR) and nb < MAX_BITS:
        a = rng.choice([-1.0, 1.0], CHUNK)
        s, _ = modulate(a, bt, SPS)
        n = rng.standard_normal((2, s.size)) * np.sqrt(sigma2 / 2)
        r = np.convolve(s + n[0] + 1j * n[1], taps, mode="same")
        k = np.arange(8, CHUNK - 8)
        c = k * SPS
        z1 = np.imag(r[c + h] * np.conj(r[c - h]))
        dphi = np.angle(r[1:] * np.conj(r[:-1]))  # dphi[i] = phase(r[i+1]) - phase(r[i])
        cs = np.concatenate(([0.0], np.cumsum(dphi)))
        z2 = cs[c + h] - cs[c - h]
        e1 += int(np.sum(np.sign(z1) != a[k]))
        e2 += int(np.sum(np.sign(z2) != a[k]))
        nb += k.size
    return name, bit, float(ebn0), e1 / nb, e2 / nb, nb, e1, e2


def run_curve(nb_pair):
    """Run one curve from low to high Eb/N0; stop once both detectors are below 3e-7."""
    name, bi = nb_pair
    rows = []
    for e in EBN0:
        r = run_point((name, bi, float(e)))
        rows.append(r[2:])
        print(name, bi, e, f"{r[3]:.2e} {r[4]:.2e}", r[5], flush=True)
        if max(r[3], r[4]) < 3e-7:
            break
    return f"{name}|{bi}", rows


def ebn0_at(ebn0, ber, target):
    ber = np.maximum(np.asarray(ber), 1e-12)
    lb = np.log10(ber)
    for i in range(len(lb) - 1):
        if lb[i] >= np.log10(target) >= lb[i + 1]:
            x0, x1, y0, y1 = ebn0[i], ebn0[i + 1], lb[i], lb[i + 1]
            return round(float(x0 + (np.log10(target) - y0) * (x1 - x0) / (y1 - y0)), 2)
    return None


def main():
    tasks = []
    for name, _ in CASES:
        bits = (0.8, 1.0, 1.2, 1.4, 1.6) if name in ("BT 0.3", "BT 0.25") else (1.0,)
        for bi in bits:
            for e in EBN0:
                tasks.append((name, bi, float(e)))
    curves = sorted({(n, b) for n, b, _ in tasks})
    out = {}
    with Pool(8) as p:
        for key, rows in p.imap_unordered(run_curve, curves):
            out[key] = rows
            print(key, "done", len(rows), "points", flush=True)
    summary = {}
    for key, rows in out.items():
        rows.sort()
        e = np.array([r[0] for r in rows])
        d1 = [r[1] for r in rows]; d2 = [r[2] for r in rows]
        summary[key] = {
            "D1_diff_1e-3": ebn0_at(e, d1, 1e-3), "D1_diff_1e-5": ebn0_at(e, d1, 1e-5),
            "D2_disc_1e-3": ebn0_at(e, d2, 1e-3), "D2_disc_1e-5": ebn0_at(e, d2, 1e-5),
            "curve": [dict(ebn0=r[0], ber_d1=r[1], ber_d2=r[2], bits=r[3], err_d1=r[4], err_d2=r[5]) for r in rows]}
    json.dump(summary, open("ber_results.json", "w"), indent=1)
    for k, v in summary.items():
        print(k, {kk: vv for kk, vv in v.items() if kk != "curve"})
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
    for a, det, lab in ((ax[0], "ber_d1", "1-bit differential"), (ax[1], "ber_d2", "limiter-discriminator + I&D")):
        for key, v in summary.items():
            name, bi = key.split("|")
            if name in ("BT 0.3", "BT 0.25") or bi == "1.0":
                xs = [c["ebn0"] for c in v["curve"] if c[det] > 0]
                ys = [c[det] for c in v["curve"] if c[det] > 0]
                ls = "-" if name in ("BT 0.3", "BT 0.25") else ":"
                if name in ("BT 0.3", "BT 0.25") and bi != "1.0":
                    ls = "--"
                a.semilogy(xs, ys, ls, label=f"{name}, BiT {bi}", lw=1)
        a.axhline(1e-3, color="grey", lw=0.5); a.axhline(1e-5, color="grey", lw=0.5)
        a.set_ylim(1e-7, 0.5); a.grid(alpha=0.3, which="both"); a.set_xlabel("Eb/N0 (dB)")
        a.set_title(f"S17 GFSK h=0.5, {lab}")
    ax[0].set_ylabel("BER"); ax[1].legend(fontsize=7)
    plt.tight_layout(); plt.savefig("s17_ber.png", dpi=130); plt.close()


if __name__ == "__main__":
    main()
