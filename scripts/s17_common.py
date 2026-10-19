"""S17 PC part: shared Gaussian-FSK (GFSK) model. Everything is normalized to the
bit period T = 1, so results hold for any bit rate (the SX1276 Gaussian filter
BT product scales with the bit period).

Model (stated so the method is reproducible):
- Frequency pulse p(t) = Phi((t+T/2)/s) - Phi((t-T/2)/s), s = sqrt(ln 2)/(2*pi*B),
  i.e. an NRZ rect of width T through a Gaussian LPF with 3 dB bandwidth B (BT = B*T).
  p has area T and sum_k p(t-kT) = 1. 'unshaped' = rect (no filter).
- Instantaneous frequency f(t) = fdev * sum_k a_k p(t-kT), a_k = +/-1,
  fdev = h/(2T) = 0.25*Rb for h = 0.5 (deviation = 0.25 x bit rate).
- Complex baseband s(t) = exp(j*2*pi*int f dt), constant envelope.
"""
import numpy as np
from scipy.special import ndtr

H = 0.5
CASES = [("unshaped", None), ("BT 1.0", 1.0), ("BT 0.5", 0.5), ("BT 0.3", 0.3), ("BT 0.25", 0.25)]
L_SPAN = 4  # pulse truncated to +/- L_SPAN bit periods (tail beyond is < 1e-12 for BT >= 0.25)


def freq_pulse(t, bt):
    """Normalized frequency pulse p(t) (peak <= 1, area = 1 bit period). t in bit periods."""
    t = np.asarray(t, float)
    if bt is None:
        return ((t >= -0.5) & (t < 0.5)).astype(float)
    s = np.sqrt(np.log(2)) / (2 * np.pi * bt)
    return ndtr((t + 0.5) / s) - ndtr((t - 0.5) / s)


def pulse_taps(bt, sps):
    """Pulse sampled at sample midpoints t = (j+0.5)/sps, j = -L*sps .. L*sps-1.
    Tap j is the frequency over the sample interval [j/sps, (j+1)/sps) (midpoint rule),
    so the phase at integer sample times is integrated without a half-sample skew.
    For 'unshaped' this gives an exact rect of sps ones."""
    j = np.arange(-L_SPAN * sps, L_SPAN * sps)
    return freq_pulse((j + 0.5) / sps, bt)


def modulate(bits_pm, bt, sps, h=H):
    """Return complex baseband GFSK and its instantaneous frequency (in units of Rb).
    Bit k's pulse is centered on t = k (sample index k*sps).
    f[m] is the frequency at the sample midpoint t = (m+0.5)/sps."""
    nb = bits_pm.size
    imp = np.zeros(nb * sps)
    imp[::sps] = bits_pm
    taps = pulse_taps(bt, sps)
    f = np.convolve(imp, taps, mode="full")[L_SPAN * sps: L_SPAN * sps + nb * sps]
    f *= h / 2.0  # fdev = h/(2T); here in units of Rb (T=1)
    # exclusive cumulative sum: phase[n] is the phase at t = n/sps exactly
    phase = 2 * np.pi * np.concatenate(([0.0], np.cumsum(f)[:-1])) / sps
    return np.exp(1j * phase), f


def gauss_lpf_taps(bit_bw, sps):
    """Complex-baseband Gaussian LPF with one-sided 3 dB bandwidth bit_bw*Rb
    (the equivalent IF bandpass filter has two-sided 3 dB bandwidth 2*bit_bw*Rb).
    Unit DC gain, symmetric (zero delay when applied with mode='same')."""
    s = np.sqrt(np.log(2)) / (2 * np.pi * bit_bw)  # time-domain sigma in bit periods
    n = np.arange(-int(np.ceil(6 * s * sps)) - 1, int(np.ceil(6 * s * sps)) + 2)
    g = np.exp(-0.5 * (n / (s * sps)) ** 2)
    return g / g.sum()
