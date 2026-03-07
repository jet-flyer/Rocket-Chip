# RF Compliance Reference

**Status:** Active — check on every radio code change
**Last Updated:** 2026-03-05 (Stage 7 pre-work)
**Applies to:** `src/drivers/rfm95w.cpp`, `ground_station/radio_rx.cpp`, telemetry service, any code touching radio TX/RX parameters

---

## Regulatory Framework

### FCC Part 15.247 (US — Primary)

RocketChip operates in the **902–928 MHz ISM band** under FCC Part 15.247, which governs spread spectrum and digital modulation systems. Key provisions:

| Parameter | Limit | Source |
|-----------|-------|--------|
| Max peak conducted output power | **+30 dBm (1 W)** | 47 CFR 15.247(b)(3) |
| Max antenna gain before power backoff | **6 dBi** | 47 CFR 15.247(b)(3)(ii) |
| Max EIRP (at 6 dBi antenna) | **+36 dBm (4 W)** | Derived: 30 + 6 |
| Antenna gain > 6 dBi | Reduce TX power 1 dB per 1 dB excess | 47 CFR 15.247(b)(3)(ii) |
| Min 6 dB bandwidth (digital modulation) | **500 kHz** | 47 CFR 15.247(a)(2) |
| Max PSD in any 3 kHz band | **+8 dBm** | 47 CFR 15.247(e) |
| Spurious emissions | **30 dB below fundamental peak** | 47 CFR 15.247(d) |
| Duty cycle | **No FCC limit** | Not specified in 15.247 |
| Frequency range | **902.0–928.0 MHz** | 47 CFR 15.247(a) |

### LoRa CSS Classification Under FCC

LoRa uses Chirp Spread Spectrum (CSS), which is neither classical DSSS nor FHSS. The FCC classifies LoRa under **digital modulation techniques** per 47 CFR 15.247(a)(2), not as DSSS or FHSS.

**Bandwidth implications for digital modulation mode:**
- The regulation requires a minimum **6 dB bandwidth of 500 kHz**
- LoRa BW500 mode meets this directly (measured 6 dB BW ≈ 500 kHz)
- LoRa BW125 and BW250 modes have 6 dB bandwidth **below** 500 kHz
- BW125/BW250 qualify under the **FHSS provisions** of 15.247(a)(1) when using frequency hopping, OR can be certified under alternate provisions depending on module FCC grant

**In practice:** The RFM95W module's FCC grant (via HopeRF/Semtech module certification) covers LoRa operation at all standard bandwidths. The host product inherits the module's certification when operating within the module's rated parameters. See Module Certification section below.

### Duty Cycle

FCC Part 15.247 imposes **no duty cycle limit** for the 902–928 MHz band. This is a US-specific advantage — ETSI (Europe) limits duty cycle to 1% or 10% depending on sub-band.

Our self-imposed duty cycle limits are engineering constraints, not regulatory:
- **70% soft cap** — above this, thermal and battery concerns dominate
- **51% at 10 Hz CCSDS** — comfortable margin
- **~90% at 10 Hz MAVLink** — exceeds soft cap, auto-drops to 5 Hz (45%)

---

## Module Certification

### RFM95W (SX1276)

The RFM95W is manufactured by HopeRF (Shenzhen HOPE Microelectronics Co., Ltd) and contains the Semtech SX1276 LoRa transceiver IC.

| Parameter | Value |
|-----------|-------|
| Manufacturer | HopeRF (HOPE Microelectronics) |
| FCC Grantee Code | 2ASEO |
| FCC ID | 2ASEORFM95C (RFM95C variant) |
| Chipset | Semtech SX1276 |
| Frequency range | 862–1020 MHz (915 MHz variant) |
| Max TX power (PA_BOOST) | +20 dBm |
| Modulation | LoRa CSS, FSK, OOK |
| Integration | Adafruit LoRa Radio FeatherWing #3231 / Breakout #3072 |

**Module certification benefit:** When the RFM95W is used as a pre-certified module and operated within its rated parameters (frequency, power, modulation), the host product (RocketChip) does **not** require separate FCC intentional radiator certification. The host product must:
1. Not modify the module's antenna interface
2. Operate within the module's certified power and frequency limits
3. Include the module's FCC ID in user-facing documentation
4. Meet unintentional radiator limits for the host board (Part 15 Subpart B)

Reference: FCC KDB 996369 — Module Certification requirements under 47 CFR 15.212.

---

## RocketChip Radio Configurations

### Default: SF7 / BW250 / CR 4/5 (Stage 7+)

| Parameter | Value | Notes |
|-----------|-------|-------|
| Spreading Factor | SF7 | Minimum airtime, adequate sensitivity |
| Bandwidth | 250 kHz | Halves airtime vs BW125, -3 dB sensitivity cost |
| Coding Rate | 4/5 | Minimum redundancy, lowest overhead |
| Frequency | 915.0 MHz | US ISM center |
| TX Power (bench) | 5 dBm | Temporary — IVP-57 testing |
| TX Power (field) | 20 dBm | Full PA_BOOST — module max |
| Preamble | 8 symbols | SX1276 default |
| Sync Word | 0x12 | LoRa private network default |
| CRC | Enabled | Hardware CRC-16 |

### EIRP Calculation (Field Configuration)

| Component | Value | Notes |
|-----------|-------|-------|
| TX conducted power | +20 dBm | PA_BOOST max |
| TX antenna gain | +3 dBi | VAS XFire Pro (omnidirectional) |
| Cable/connector loss | -0.5 dB | Conservative estimate, short pigtail |
| **EIRP** | **+22.5 dBm** | Well under +36 dBm limit |

**Compliance:** EIRP of +22.5 dBm is **13.5 dB below** the FCC limit of +36 dBm. No power backoff required.

### EIRP Calculation (Bench Configuration)

| Component | Value | Notes |
|-----------|-------|-------|
| TX conducted power | +5 dBm | IVP-57 bench testing |
| TX antenna gain | +3 dBi | VAS XFire Pro |
| Cable/connector loss | -0.5 dB | |
| **EIRP** | **+7.5 dBm** | |

### Alternative Configurations (Available via Mission Profile)

| Config | Use Case | Airtime (54B) | Max Rate | Sensitivity | Range (est.) |
|--------|----------|---------------|----------|-------------|-------------|
| SF7/BW125/CR4-5 | Long range / HAB | ~103 ms | 5 Hz max | -123 dBm | 5–15 km |
| SF7/BW250/CR4-5 | **Default flight** | ~51 ms | 10 Hz | -120 dBm | 3–10 km |
| SF6/BW500/CR4-5 | High rate / short range | ~14 ms | 50 Hz theoretical | -111 dBm | 1–5 km |

BW125 does not meet the 500 kHz digital modulation bandwidth requirement on its own, but operates within the module's FCC grant parameters. BW250 is borderline. BW500 clearly meets the 500 kHz threshold.

---

## Link Budget (Field, Default Config)

### Free Space Path Loss (FSPL) at 915 MHz

FSPL(dB) = 20·log10(d_m) + 20·log10(f_Hz) - 147.55

| Distance | FSPL | Notes |
|----------|------|-------|
| 100 m | 71.7 dB | Bench/close range |
| 500 m | 85.6 dB | Typical field test |
| 1 km | 91.7 dB | Model rocket apogee (low-mid power) |
| 5 km | 105.6 dB | High-power rocket |
| 10 km | 111.7 dB | HAB / extreme range |

### Full Link Budget at 10 km

| Parameter | Value |
|-----------|-------|
| TX power | +20 dBm |
| TX antenna gain (VAS XFire Pro) | +3 dBi |
| RX antenna gain (TBS Immortal T) | +3 dBi |
| FSPL at 10 km / 915 MHz | -111.7 dB |
| Fade margin (tumble + polarization) | -5 dB |
| **Received power** | **-90.7 dBm** |
| RX sensitivity (SF7/BW250) | -120 dBm |
| **Link margin** | **+29.3 dB** |

### Link Margin vs Distance

| Distance | Received Power | Margin | Status |
|----------|---------------|--------|--------|
| 100 m | -42.7 dBm | +77.3 dB | Excellent |
| 500 m | -56.6 dBm | +63.4 dB | Excellent |
| 1 km | -62.7 dBm | +57.3 dB | Excellent |
| 5 km | -76.6 dBm | +43.4 dB | Good |
| 10 km | -90.7 dBm | +29.3 dB | Adequate |
| 20 km | -96.7 dBm | +23.3 dB | Marginal |

**Antenna assumptions:**
- TX: VAS XFire Pro (~3 dBi, vertical omni, SMA)
- RX: TBS Immortal T (~2–3 dBi, omni, SMA) — using 3 dBi for calculations
- Both are real products with published gain patterns, not theoretical isotropic

**Fade margin:** 5 dB accounts for rocket tumble (random polarization alignment), multipath from ground reflection, and atmospheric absorption (negligible at 915 MHz for <20 km).

---

## Airtime Reference

LoRa airtime for explicit header mode, CR 4/5, CRC enabled, 8-symbol preamble:

### BW250 (Default)

| Payload | SF6 | SF7 | SF8 | SF9 |
|---------|-----|-----|-----|-----|
| 16 B | 5.6 ms | 12.5 ms | 25.1 ms | 46.1 ms |
| 32 B | 7.7 ms | 17.7 ms | 30.2 ms | 56.3 ms |
| 54 B | 10.5 ms | 25.1 ms | 40.4 ms | 76.8 ms |
| 64 B | 11.3 ms | 27.6 ms | 45.6 ms | 87.0 ms |
| 105 B | 16.1 ms | 40.4 ms | 66.0 ms | 128.0 ms |
| 128 B | 18.2 ms | 45.6 ms | 76.8 ms | 148.5 ms |

### BW125 (Long Range Alternative)

| Payload | SF7 | SF8 | SF9 |
|---------|-----|-----|-----|
| 54 B | 51.5 ms | 82.4 ms | 154.6 ms |
| 105 B | 82.4 ms | 133.1 ms | 256.0 ms |

### Duty Cycle at Various Rates (BW250, SF7, 54B CCSDS)

| TX Rate | Airtime/s | Duty Cycle | Status |
|---------|-----------|------------|--------|
| 2 Hz | 50.2 ms | 5.0% | Idle/ground |
| 5 Hz | 125.5 ms | 12.6% | Reduced rate |
| 10 Hz | 251.0 ms | 25.1% | Default flight |

*Note: Plan estimates used ~51ms per packet. Refined calculation using Semtech LoRa modem calculator shows ~25ms at BW250/SF7 for 54B. Field measurement at IVP-59 Budget Checkpoint #2 will provide ground truth.*

---

## Power Budget (HAB / Long-Duration Use)

For high-altitude balloon and extended missions where battery life matters:

| Component | Active Current | Duty | Avg Current |
|-----------|---------------|------|-------------|
| SX1276 TX @ 20 dBm | ~120 mA | 25% (10 Hz) | 30 mA |
| SX1276 TX @ 20 dBm | ~120 mA | 5% (2 Hz) | 6 mA |
| SX1276 RX | ~12 mA | varies | ~2 mA (at 2 Hz with RX window) |
| SX1276 Sleep | ~0.2 µA | idle periods | negligible |
| RP2350 active | ~15–25 mA | 100% | 20 mA |
| ICM-20948 | ~3 mA | 100% | 3 mA |
| PA1010D GPS | ~25 mA | 100% | 25 mA |
| DPS310 baro | ~0.5 mA | 100% | 0.5 mA |
| NeoPixel | ~5–8 mA | varies | 5 mA |

**Total system (flight, 10 Hz TX):** ~84 mA
**Total system (ground, 2 Hz TX):** ~62 mA
**Total system (sleep, no TX):** ~54 mA

With a 2000 mAh LiPo:
- Flight mode: ~24 hours (dominated by GPS + MCU, not radio)
- Ground/idle: ~32 hours
- HAB endurance at 2 Hz: ~32 hours

Radio TX is NOT the dominant power consumer. GPS and MCU together draw ~45 mA continuously. For HAB endurance optimization, GPS duty cycling (1 Hz fixes with sleep between) would save more than reducing TX rate.

---

## Compliance Checklist

**Run this checklist on every PR that modifies radio-related code:**

### Frequency
- [ ] Operating frequency within 902.0–928.0 MHz
- [ ] Frequency set via `rfm95w_set_frequency()` with validated input

### Power
- [ ] TX power ≤ +20 dBm (module max, `rfm95w_set_tx_power()` clamps to 2–20)
- [ ] EIRP = TX power + antenna gain - losses ≤ +36 dBm
- [ ] If antenna gain > 6 dBi: TX power reduced by (gain - 6) dB

### Bandwidth
- [ ] LoRa BW setting is one of: 125, 250, or 500 kHz
- [ ] Operating within module's FCC grant parameters

### Modulation
- [ ] LoRa CSS mode (kLoRaMode = 0x80 set in RegOpMode)
- [ ] No custom modulation schemes outside SX1276 supported modes

### Spurious Emissions
- [ ] No firmware changes that could affect RF output spectrum
- [ ] Using module's reference design antenna interface (Adafruit breakout)

### Module Integration
- [ ] Not modifying antenna matching network or RF trace
- [ ] Operating within module's certified power/frequency/modulation limits
- [ ] FCC ID (2ASEORFM95C) referenced in product documentation

---

## Regulatory Notes for Non-US Operation

| Region | Regulation | Band | Key Differences |
|--------|-----------|------|-----------------|
| US | FCC Part 15.247 | 902–928 MHz | No duty cycle limit, 1 W max |
| EU | ETSI EN 300 220 | 863–870 MHz | **1% or 10% duty cycle**, 25 mW (14 dBm) default |
| AU | ACMA | 915–928 MHz | Similar to FCC, 1 W max |
| JP | ARIB STD-T108 | 920–928 MHz | 20 mW max, listen-before-talk |

**RocketChip ships US-only initially.** EU/JP compliance requires Mission Profile changes (frequency, power, duty cycle enforcement). The telemetry service rate management provides the infrastructure for duty cycle enforcement when needed.

---

## References

- [47 CFR 15.247 — eCFR](https://www.ecfr.gov/current/title-47/chapter-I/subchapter-A/part-15/subpart-C/subject-group-ECFR2f2e5828339709e/section-15.247) — Primary regulation text
- [FCC KDB 996369 — Module Certification](https://apps.fcc.gov/oetcf/kdb/forms/FTSSearchResultPage.cfm?switch=P&id=44637) — Module integration requirements
- [Sunfire LoRa FCC Certification Guide](https://www.sunfiretesting.com/LoRa-FCC-Certification-Guide/) — Practical LoRa certification walkthrough
- [Semtech AN1200.22 — LoRa Modulation Basics](https://www.frugalprototype.com/wp-content/uploads/2016/08/an1200.22.pdf) — CSS modulation theory
- [Mouser AN120062 — FCC Pre-Compliance for LoRaWAN](https://www.mouser.com/pdfDocs/AN120062_BestPracticesforFCC_Rev_1_0_FINAL.pdf) — Test methodology
- [DigiKey — Unlicensed 915 MHz Band](https://www.digikey.com/en/articles/unlicensed-915-mhz-band-fits-many-applications-and-allows-higher-transmit-power) — Overview of ISM band regulations
- [Raveon AN203 — FCC Part 15 ISM Regulations](https://raveon.com/wp-content/uploads/2019/05/AN203FCC-ISM.pdf) — Practical regulatory guide
- SX1276 Datasheet (Semtech DS_SX1276-7-8-9_W_APP_V7) — IC specifications
- RFM95W Datasheet (HopeRF) — Module specifications
