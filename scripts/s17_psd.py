"""S17 deliverable 1: normalized PSD and occupied bandwidth (calculated, not measured).
Run: python3 s17_psd.py  -> psd_results.json, s17_psd.png, s17_psd_zoom.png
Method: Welch average of a long random-bit GFSK baseband record.
sps=32, segment = 128 bits (RBW = Rb/128 bin spacing), Blackman-Harris window, 50 % overlap,
2^19 random bits (seed 17). PSD in dB relative to its own peak (density, so independent of RBW
once the bin is much narrower than the spectrum)."""
import json
import numpy as np
from scipy.signal import welch
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from s17_common import CASES, modulate

SPS, NBITS, SEG_BITS, SEED = 32, 2 ** 19, 128, 17


def occupied(f, psd):
    """f sorted, psd linear. Returns dict of two-sided widths in units of Rb."""
    df = f[1] - f[0]
    p = psd / psd.sum()
    c = np.cumsum(p)
    lo = np.interp(0.005, c, f + df / 2)
    hi = np.interp(0.995, c, f + df / 2)
    out = {"obw99": hi - lo}
    c2 = np.interp(0.0005, c, f + df / 2), np.interp(0.9995, c, f + df / 2)
    out["obw99_9"] = c2[1] - c2[0]
    pdb = 10 * np.log10(psd / psd.max())
    for lvl in (20, 40, 60):
        idx = np.where(pdb >= -lvl)[0]
        # outermost frequency still at/above the level, two-sided width
        out[f"bw_{lvl}dB"] = f[idx[-1]] - f[idx[0]]
    return out


def main():
    rng = np.random.default_rng(SEED)
    bits = rng.choice([-1.0, 1.0], NBITS)
    res, curves = {}, {}
    for name, bt in CASES:
        s, _ = modulate(bits, bt, SPS)
        nper = SEG_BITS * SPS
        f, pxx = welch(s, fs=SPS, window="blackmanharris", nperseg=nper, noverlap=nper // 2,
                       return_onesided=False, detrend=False, scaling="density")
        o = np.argsort(f)
        f, pxx = f[o], pxx[o]
        res[name] = {k: round(float(v), 3) for k, v in occupied(f, pxx).items()}
        curves[name] = (f, 10 * np.log10(pxx / pxx.max()))
        print(name, res[name])
    meta = dict(sps=SPS, nbits=NBITS, seg_bits=SEG_BITS, seed=SEED, h=0.5, units="multiples of Rb, two-sided")
    json.dump({"meta": meta, "results": res}, open("psd_results.json", "w"), indent=2)

    for fname, xlim, ylim in (("s17_psd.png", (0, 3), (-100, 3)), ("s17_psd_zoom.png", (0, 1.2), (-60, 3))):
        plt.figure(figsize=(8, 5))
        for name, (f, pdb) in curves.items():
            m = f >= 0
            plt.plot(f[m], pdb[m], lw=1.2 if "0.2" in name or "0.3" in name else 0.8,
                     label=name, ls="-" if name in ("BT 0.25", "BT 0.3") else "--")
        for lvl in (-20, -40):
            plt.axhline(lvl, color="grey", lw=0.5, ls=":")
        plt.xlim(*xlim); plt.ylim(*ylim); plt.grid(alpha=0.3)
        plt.xlabel("(f - fc) / Rb"); plt.ylabel("PSD, dB relative to peak")
        plt.title("S17 calculated GFSK spectrum, h = 0.5 (fdev = 0.25 x Rb)")
        plt.legend(); plt.tight_layout(); plt.savefig(fname, dpi=130); plt.close()


if __name__ == "__main__":
    main()
