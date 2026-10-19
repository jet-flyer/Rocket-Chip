# S17 PC part: GFSK BT 0.25 vs 0.3 (simulation)

- Date: 2026-10-18
- Author: Buzz
- Scope: FSK pivot plan rev 2, row S17, PC part only (no board). Feeds D4 / Q9. Does not pick a BT value (Nathan, 2026-10-12).
- Script: `scripts/s17_gfsk_bt.py` (numpy/scipy, seed 17, 64 samples/bit, 200,000 random bits, modulation index h = 0.5).

## Results (simulation, ideal Gaussian filter; frequencies in units of bit rate Rb)

| BT | Pulse energy in own bit | Isolated-bit peak deviation | Worst-case eye (fraction of full deviation) | 99% OBW | 99.9% OBW | PSD at 1.0 Rb | at 1.5 Rb | at 2.0 Rb | Adjacent 1 Rb channel at +1.5 Rb |
|---|---|---|---|---|---|---|---|---|---|
| 0.25 | 0.589 | 0.654 | 0.309 | 0.859 Rb | 1.062 Rb | -54.2 dB | -84.6 dB | -120.1 dB | -63.4 dB |
| 0.3 | 0.651 | 0.742 | 0.485 | 0.906 Rb | 1.125 Rb | -46.0 dB | -73.6 dB | -104.4 dB | -54.6 dB |
| 0.5 | 0.789 | 0.941 | 0.881 | 1.031 Rb | 1.305 Rb | -33.0 dB | -60.0 dB | -75.2 dB | -41.0 dB |
| 1.0 | 0.894 | 1.000 | 1.000 | 1.117 Rb | 1.969 Rb | -26.5 dB | -38.3 dB | -47.0 dB | -33.4 dB |

Sanity check (fact): the 99% occupied bandwidth for 0.25 and 0.5 matches the classic GMSK values (Murota and Hirade, IEEE Trans. Commun. 1981: about 0.86 Rb and 1.04 Rb).

## Reading (inference)

- Main lobe: 0.3 is about 5% wider than 0.25 at 99% power (0.906 vs 0.859 Rb). That is small.
- Intersymbol interference: 0.25 is clearly worse. Its worst-case eye is 0.31 of full deviation against 0.49 for 0.3, so 0.3 should give equal or better PER at the same signal level.
- Skirts: 0.25 is about 8 to 11 dB lower beyond 1 Rb and about 9 dB lower in an adjacent 1 Rb channel. This only matters if a nearby channel or the mask (row S8) is tight.
- Net: 0.3, the closest SX1276 setting, gives up a little spectral tidiness for a cleaner eye. The board half of S17 (PER at shaping 00/01/10/11) is still needed, because the SX1276's real filter is not ideal and the datasheet gives no BT 0.3 performance figure (plan Q19).

## Limits

- Ideal Gaussian filter and ideal discriminator. No noise, no receiver filter, no SX1276 implementation effects.
- The eye metric is the worst-case sum of neighbour taps at the bit centre, not a BER curve.
