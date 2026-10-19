"""S17 deliverable 2a: transmit-side ISI of the Gaussian frequency pulse (noise-free, no RX filter).
Run: python3 s17_isi.py -> isi_results.json, s17_pulse.png, s17_eye.png
Metrics (all normalized to T, so bit-rate independent):
- c0 = fraction of a bit's frequency-pulse area inside its own bit period; leak = 1 - c0.
- p(0) = peak of an isolated bit's frequency pulse (fraction of full deviation).
- Frequency eye at bit centre: worst case over all patterns = p(0) - sum_{k!=0} p(kT) = 2 p(0) - 1
  (because sum_k p(t-kT) = 1). Ideal (unshaped) = 1.
- Phase-change eye over one bit (what a 1-bit differential detector sees): worst case
  dphi = (pi/2)(2 c0 - 1); ideal pi/2. Detector output sin(dphi) also reported.
"""
import json
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from s17_common import CASES, freq_pulse, modulate

def metrics(bt):
    t = np.linspace(-6, 6, 120001)
    p = freq_pulse(t, bt)
    dt = t[1] - t[0]
    area = p.sum() * dt
    c0 = p[np.abs(t) < 0.5].sum() * dt / area
    p0 = float(freq_pulse(0.0, bt)) if bt is not None else 1.0
    pk = [float(freq_pulse(k, bt)) if bt is not None else 0.0 for k in (1, 2, 3)]
    feye = 2 * p0 - 1
    dphi = (np.pi / 2) * (2 * c0 - 1)
    return dict(inside_own_bit=round(c0, 4), leak_outside=round(1 - c0, 4),
                p0=round(p0, 4), p1=round(pk[0], 4), p2=round(pk[1], 5),
                freq_eye_worst=round(feye, 4), freq_eye_dB=round(20 * np.log10(feye), 2),
                phase_eye_worst_deg=round(np.degrees(dphi), 2),
                diff_det_eye_sin=round(float(np.sin(dphi)), 4),
                diff_det_eye_dB=round(20 * np.log10(np.sin(dphi)), 2))

def main():
    res = {name: metrics(bt) for name, bt in CASES}
    for k, v in res.items():
        print(k, v)
    json.dump(res, open("isi_results.json", "w"), indent=2)
    t = np.linspace(-3, 3, 2001)
    plt.figure(figsize=(7, 4.5))
    for name, bt in CASES:
        plt.plot(t, freq_pulse(t, bt), label=name)
    for x in (-0.5, 0.5):
        plt.axvline(x, color="grey", ls=":", lw=0.8)
    plt.xlabel("t / T (bit periods)"); plt.ylabel("frequency pulse p(t), fraction of fdev")
    plt.title("S17 Gaussian frequency pulse (one bit)"); plt.grid(alpha=0.3); plt.legend()
    plt.tight_layout(); plt.savefig("s17_pulse.png", dpi=130); plt.close()
    # instantaneous-frequency eye diagrams for 0.25 and 0.3
    sps = 64
    bits = np.random.default_rng(5).choice([-1.0, 1.0], 2000)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4), sharey=True)
    for a, (name, bt) in zip(ax, [("BT 0.3", 0.3), ("BT 0.25", 0.25)]):
        _, f = modulate(bits, bt, sps)
        f = f / 0.25  # normalize to fdev
        for k in range(10, 1900):
            seg = f[k * sps - sps: k * sps + sps + 1]
            a.plot(np.arange(-sps, sps + 1) / sps, seg, color="C0", alpha=0.03, lw=0.6)
        e = res[name]["freq_eye_worst"]
        a.axhline(e, color="r", ls="--", lw=0.8); a.axhline(-e, color="r", ls="--", lw=0.8)
        a.set_title(f"{name}: worst eye {e:.3f} of fdev"); a.set_xlabel("t / T"); a.grid(alpha=0.3)
    ax[0].set_ylabel("instantaneous frequency / fdev")
    plt.tight_layout(); plt.savefig("s17_eye.png", dpi=130); plt.close()

if __name__ == "__main__":
    main()
