# S17 PC part: GFSK BT 0.25 vs 0.3 (simulation)

- Date: 2026-10-18
- Author: Buzz
- Scope: FSK pivot plan rev 2, row S17, PC part only (no board). Feeds D4 / Q9. Does not pick a BT value (Nathan, 2026-10-12).
- Script: `scripts/s17_gfsk_bt.py` (numpy/scipy, seed 17, 64 samples/bit, 200,000 random bits, modulation index h = 0.5).

## Source

- Source [draft, information only per plan rev 2 Decision 5]: CCSDS 211.1-P-4.2 §5.1.6.3.1, p. 5-17. Duke checked ccsds.org on 2026-10-18. 211.1-P-4.2, April 2026, is the current Pink Book. Link: https://ccsds.org/wp-content/uploads/2026/06/211x1p42.pdf
- Quote: "Suppressed carrier Gaussian Minimum Shift Keying (GMSK) modulation BTs=0.25 (where B refers to the one-sided 3-dB bandwidth of the filter) with pre-coding as shown in Figure 5-2."
- Plan rev 2 uses that mark for decision 5 of Nathan, rev 1 §9. Pink sheets and the Red Book stay [draft] references. This mark is not row D5 (DIO wires).
- The current Blue Book is CCSDS 211.1-B-4. Duke checked ccsds.org on 2026-10-18. It is the only 211.1 Blue Book. Section 3.3.5 specifies Bi-Phase-L PCM data modulated directly onto the carrier (§3.3.5.1) and a residual carrier with modulation index 60° ± 5% (§3.3.5.2). The book has no GMSK clause and no BT clause.

## Supporting runs

The results table comes from `scripts/s17_gfsk_bt.py`. These scripts are supporting runs. Run each command from the repo root.

- `scripts/s17_common.py`. Shared model. The bit period is T = 1. The frequency pulse is p(t) = Φ((t + T/2)/s) − Φ((t − T/2)/s), with s = √(ln 2) / (2 π BT). The pulse area is T. The deviation is h / (2T) = 0.25 × bit rate at h = 0.5. The pulse span is ±4 bit periods. The cases are unshaped, BT 1.0, BT 0.5, BT 0.3, and BT 0.25.
- `scripts/s17_isi.py`. Noise-free transmit ISI. No receiver filter. Command: `python3 scripts/s17_isi.py`. The worst-case frequency eye at the bit centre is 2 p(0) − 1. The eye figure uses 2000 random bits, seed 5, and 64 samples per bit, for BT 0.3 and BT 0.25. The script writes `isi_results.json`, `s17_pulse.png`, and `s17_eye.png` in the current directory.
- `scripts/s17_psd.py`. Welch PSD. Command: `python3 scripts/s17_psd.py`. Settings: 32 samples per bit, 2^19 random bits, seed 17, Blackman-Harris window, segment of 128 bits, 50% overlap. It reports the 99% and 99.9% occupied bandwidth and the widths at −20 dB, −40 dB, and −60 dB. The script writes `psd_results.json`, `s17_psd.png`, and `s17_psd_zoom.png` in the current directory.
- `scripts/s17_ber.py`. Monte Carlo BER in AWGN. Command: `python3 scripts/s17_ber.py`. This run is long. The modulation index is h = 0.5. The record uses 8 samples per bit. The predetection filter is a Gaussian IF filter. BiT is 0.8, 1.0, 1.2, 1.4, and 1.6 for BT 0.25 and BT 0.3. BiT is 1.0 for the other rows. Detector D1 is 1-bit differential detection. Detector D2 is a limiter-discriminator with integrate-and-dump. Each point stops at 300 errors on both detectors or at 4e7 bits. The Eb/N0 grid is 4.0 dB to 32.0 dB in steps of 0.5 dB. The results table does not use this script.

## Results, calc (simulation)

Ideal Gaussian filter. Frequencies are in units of bit rate Rb.

| BT | Pulse energy in own bit | Isolated-bit peak deviation | Worst-case eye (fraction of full deviation) | 99% OBW | 99.9% OBW | PSD at 1.0 Rb | at 1.5 Rb | at 2.0 Rb | Adjacent 1 Rb channel at +1.5 Rb |
|---|---|---|---|---|---|---|---|---|---|
| 0.25 | 0.589 | 0.654 | 0.309 | 0.859 Rb | 1.062 Rb | -54.2 dB | -84.6 dB | -120.1 dB | -63.4 dB |
| 0.3 | 0.651 | 0.742 | 0.485 | 0.906 Rb | 1.125 Rb | -46.0 dB | -73.6 dB | -104.4 dB | -54.6 dB |
| 0.5 | 0.789 | 0.941 | 0.881 | 1.031 Rb | 1.305 Rb | -33.0 dB | -60.0 dB | -75.2 dB | -41.0 dB |
| 1.0 | 0.894 | 1.000 | 1.000 | 1.117 Rb | 1.969 Rb | -26.5 dB | -38.3 dB | -47.0 dB | -33.4 dB |

Sanity check (fact): the 99% occupied bandwidth for 0.25 and 0.5 matches the classic GMSK values (Murota and Hirade, "GMSK Modulation for Digital Mobile Radio Telephony," IEEE Trans. Commun., vol. COM-29, no. 7, pp. 1044–1050, July 1981, https://doi.org/10.1109/TCOM.1981.1095089: about 0.86 Rb and 1.04 Rb).

## Reading (inference)

- Main lobe: 0.3 is about 5% wider than 0.25 at 99% power (0.906 vs 0.859 Rb). That is small.
- Intersymbol interference: 0.25 is clearly worse. Its worst-case eye is 0.31 of full deviation against 0.49 for 0.3, so 0.3 should give equal or better PER at the same signal level.
- Skirts: 0.25 is about 8 to 11 dB lower beyond 1 Rb and about 9 dB lower in an adjacent 1 Rb channel. This only matters if a nearby channel or the mask (row S8) is tight.
- Net: 0.3, the closest SX1276 setting, gives up a little spectral tidiness for a cleaner eye. The board half of S17 (PER at shaping 00/01/10/11) is still needed, because the SX1276's real filter is not ideal and the datasheet gives no BT 0.3 performance figure (plan Q19).

## Limits

- Ideal Gaussian filter and ideal discriminator. No noise, no receiver filter, no SX1276 implementation effects.
- The eye metric is the worst-case sum of neighbour taps at the bit centre, not a BER curve.
- The Figure 5-2 precoder is not in the model. `scripts/s17_ber.py` uses only noncoherent detectors. The runs cover GFSK at h = 0.5. They do not cover the full Prox-1 suppressed-carrier GMSK link.
- INFERENCE: the precoder does not change the spectrum results or the eye results.
