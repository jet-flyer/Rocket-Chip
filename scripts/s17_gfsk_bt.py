"""S17 PC part: GFSK h=0.5 spectrum and ISI, BT 0.25 vs 0.3 (plus 0.5, 1.0 for context).
Pure simulation, no board. Frequencies normalised to bit rate Rb (offset/Rb)."""
import numpy as np
from scipy.signal import welch
from scipy.special import erfc

SPS = 64            # samples per bit
NBITS = 200_000
H = 0.5
rng = np.random.default_rng(17)

def gauss_freq_pulse(bt, span=8):
    # Frequency pulse g(t): a unit-height rect of width 1 bit, convolved with a Gaussian.
    # The continuous pulse has area 1. phase = pi*h*cumsum(freq)/SPS steps one bit by pi*h.
    t = (np.arange(-span*SPS//2, span*SPS//2) + 0.5) / SPS   # in bit periods
    q = lambda x: 0.5*erfc(x/np.sqrt(2))
    s = 2*np.pi*bt/np.sqrt(np.log(2))
    g = (q(s*(t-0.5)) - q(s*(t+0.5)))   # unit-height rect convolved with gaussian
    return t, g

def isi_metrics(bt):
    t, g = gauss_freq_pulse(bt)
    total = g.sum()
    inbit = g[np.abs(t) <= 0.5].sum()
    peak = g.max()               # rect height 1 -> NRZ peak deviation fraction for isolated bit
    # worst-case eye: centre sample of a bit = g(0) - sum |g(kT)| for k!=0
    idx0 = np.argmin(np.abs(t))
    taps = [g[idx0 + k*SPS] for k in range(-3, 4) if 0 <= idx0+k*SPS < len(g)]
    centre = g[idx0]
    side = sum(abs(x) for k, x in zip(range(-3,4), taps) if k != 0)
    eye = (centre - side)        # normalised to full deviation (1.0)
    return inbit/total, peak, centre, side, eye

def spectrum(bt):
    t, g = gauss_freq_pulse(bt)
    bits = rng.integers(0, 2, NBITS)*2 - 1
    up = np.zeros(NBITS*SPS); up[::SPS] = bits
    freq = np.convolve(up, g, mode='same')          # deviation, +-1 = +-fdev
    phase = np.pi*H*np.cumsum(freq)/SPS
    sig = np.exp(1j*phase)
    f, p = welch(sig, fs=SPS, nperseg=SPS*256, return_onesided=False)
    o = np.argsort(f); f, p = f[o], p[o]
    p = p / p.sum()
    return f, p

def occ_bw(f, p, frac):
    c = np.cumsum(p); lo = f[np.searchsorted(c, (1-frac)/2)]; hi = f[np.searchsorted(c, 1-(1-frac)/2)]
    return hi - lo

def rel_db_at(f, p, off):
    pk = p.max()
    m = (np.abs(np.abs(f) - off) < 0.02)
    return 10*np.log10(p[m].mean()/pk)

def acp_db(f, p, ch_bw, ch_off):
    inch = p[np.abs(f) <= ch_bw/2].sum()
    adj = p[(f >= ch_off - ch_bw/2) & (f <= ch_off + ch_bw/2)].sum()
    return 10*np.log10(adj/inch)

print(f"GFSK h={H}, SPS={SPS}, {NBITS} random bits, freq offsets in units of Rb")
print("BT   | energy in own bit | isolated-bit peak dev | worst-case eye (frac of full dev) | 99% OBW | 99.9% OBW | PSD @1.0Rb | @1.5Rb | @2.0Rb | ACP(1Rb ch @ +1.5Rb)")
for bt in (0.25, 0.3, 0.5, 1.0):
    inb, pk, c, s, eye = isi_metrics(bt)
    f, p = spectrum(bt)
    print(f"{bt:<4} | {inb:17.3f} | {pk:21.3f} | {eye:33.3f} | {occ_bw(f,p,0.99):7.3f} | {occ_bw(f,p,0.999):9.3f} | {rel_db_at(f,p,1.0):10.1f} | {rel_db_at(f,p,1.5):6.1f} | {rel_db_at(f,p,2.0):6.1f} | {acp_db(f,p,1.0,1.5):8.1f}")
