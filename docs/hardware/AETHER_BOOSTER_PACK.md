# Aether Booster Pack — Air Data Sensing System

**Version:** 0.1 Research Draft
**Last Updated:** 2026-02-22
**Status:** Research / Feasibility
**Roadmap Position:** Post-MVP Booster Pack (R&D Tier)

---

## 1. Overview

Aether is a proposed Booster Pack providing aerodynamic air data measurement — airspeed, angle of attack (α), and angle of sideslip (β) — via flush pressure ports or a multi-hole pressure probe. Named after Aether, the Greek personification of the upper atmosphere.

Aether addresses a fundamental sensor gap: RocketChip's core sensor stack (IMU + barometer + optional GPS) loses measurement authority in the transonic regime (Mach 0.8–1.2) and cannot directly measure aerodynamic flow angles at any speed. Barometric altitude becomes unreliable due to shock-induced pressure artifacts, GPS receivers on COTS modules often lose lock under high-g/high-dynamics, and IMU-only propagation drifts without correction. For UAS applications, the absence of angle of attack measurement means no direct stall warning — a leading cause of small UAS crashes.

The Booster Pack has two proposed form factors serving different markets:

| Variant | Form Factor | Primary Market | Measures |
|---------|-------------|----------------|----------|
| Aether-R (Rocket) | Flush nosecone pressure ports | HPR transonic/supersonic flight | Dynamic pressure, airspeed, optionally α |
| Aether-U (UAS) | Multi-hole pressure probe | Drones, fixed-wing UAS | Airspeed, α, β (full 3D air data) |

Both variants share the same sensor PCB, pressure-to-aerodata algorithms, and ESKF measurement update interface. The difference is the pneumatic front-end: flush ports in a nosecone vs. a probe on a boom.

### 1.1 Related Documents

| Document | Path | Description |
|----------|------|-------------|
| Sensor Architecture | `docs/SENSOR_ARCHITECTURE.md` | Transport-neutral driver pattern |
| ESKF Plan | `docs/PHASE5_ESKF_PLAN.md` | Sensor fusion architecture |
| Hardware Reference | `docs/hardware/HARDWARE.md` | Component specs, GPIO, pin assignments |
| Expansion Connector ICD | `docs/icd/EXPANSION_CONNECTOR_ICD.md` | Physical connector interface |
| Project Overview | `docs/PROJECT_OVERVIEW.md` | Product tiers and Booster Pack summary |

---

## 2. Background and Heritage

### 2.1 Flush Air Data Systems (FADS)

Flush Air Data Systems measure air data parameters using pressure ports mounted flush with a vehicle's surface. The concept originated in the 1960s with the X-15 program and was subsequently used on the F-14, F/A-18 HARV, X-33, X-43A, and the Space Shuttle.

**Space Shuttle SEADS (Shuttle Entry Air Data System):**
The Shuttle implementation is the most thoroughly documented FADS. SEADS used 14 coated columbium (niobium) pressure ports in the Reinforced Carbon-Carbon (RCC) nose cap, measuring pressure distribution from 56 miles altitude to the ground during reentry. The system used no mechanical devices — the fuselage itself functioned as both a pitot-static probe and a differential-pressure flow direction sensor. The full system consisted of 20 flush orifices, each routed to a pair of absolute pressure transducers.

SEADS was validated across the transonic regime (Mach 0.25–1.40) for determining angle of attack, angle of sideslip, Mach number, and freestream dynamic pressure. The F-14 flight test program validated flush port air data at Mach 0.60–1.60, angles of attack up to 26°, and sideslip angles up to 11°.

Key design elements of SEADS relevant to Aether:
- **Penetration assemblies:** Hardware to place pressure port openings flush at the vehicle surface, with pressure tubes transmitting surface pressure to transducers mounted internally
- **Support manifolds:** Reduced pneumatic tube lengths between ports and transducers
- **Cruciform port pattern:** Top/bottom pair resolves α, left/right pair resolves β, center region provides stagnation/total pressure
- **Redundancy:** More ports than unknowns enables least-squares solutions (Triples algorithm) and graceful degradation on port blockage

**Solving for air data from surface pressures:**
A minimum of 5 ports solves for 5 unknowns: total pressure (P₀), static pressure (P∞), angle of attack (α), angle of sideslip (β), and Mach number (M). The pressure distribution over a known body shape follows predictable aerodynamic models (modified Newtonian theory for blunt bodies, potential flow for slender bodies). The relationship between port pressures and freestream conditions is captured in calibration coefficients derived from CFD, wind tunnel testing, or analytical solutions.

### 2.2 Multi-Hole Pressure Probes (MHP)

Multi-hole probes are self-contained air data instruments mounted on a boom ahead of the vehicle. They contain their own known geometry (hemispherical or conical tip) with multiple pressure ports, and are calibrated independently of the vehicle they're mounted on. This decouples the sensing from the airframe geometry.

**Probe configurations:**
- **5-hole:** Minimum for full 3D air data (airspeed, α, β). One center hole + 4 peripheral holes in cruciform arrangement
- **7-hole:** Extends measurement range to higher flow angles (~60° vs ~30° for 5-hole) by adding data reduction zones for separated flow on the lee side
- **9-hole:** Additional ports improve accuracy and enable redundancy

**State of the art for small UAS (2025):**
Recent work (arxiv 2505.03331, May 2025) produced a 9-hole multi-hole pressure probe weighing 9 grams, 3D-printed as a single component with sensing electronics integrated directly inside. It uses five SDP31 differential pressure sensors on a PCB within the 3D-printed structure, with silicone sleeves providing airtight seals. Performance: airspeed range 3–27 m/s, α/β range ±35°, mean absolute error 0.44 m/s and 0.16° (wind tunnel), 1.65 m/s and 3.20° (outdoor flight). The design was released to the public domain.

An earlier embedded-sensor probe patent (US7010970B2) demonstrated MEMS pressure transducers embedded directly at the probe tip, eliminating pneumatic tubing entirely and achieving maximum frequency response. The system outputs airspeed, α, and β directly as voltage signals via a self-contained 12V control computer.

### 2.3 Existing HPR Practice

No flush air data system or multi-hole probe implementation exists in the high-power rocketry hobby. The closest documented work:

- **Calibrated static/stagnation ports:** At least one HPR project used calibrated static ports on the airframe body and a nose tip total pressure port, with calibration via CFD. This is essentially a simplified 2-port system.
- **Pitot tubes:** Used on some student sounding rockets (e.g., Northeastern University Project Redshift) with stagnation pressure at the nosecone tip paired with a separate static pressure transducer.
- **Standard practice:** HPR altimeters use barometric pressure through vent holes in the airframe body tube, placed behind the nosecone shoulder. Community consensus is that this data is noisy during powered flight and transonic passage, useful primarily for apogee detection (where velocity is near zero and static pressure ≈ atmospheric pressure). Velocity measurement is typically derived from accelerometer integration.

The transonic measurement gap is widely acknowledged but unaddressed in hobby rocketry.

---

## 3. Feasibility Analysis

### 3.1 Reynolds Number at HPR Scale

The primary concern with scaling FADS from full-size aerospace vehicles to HPR scale is whether the flow physics remain valid. Reynolds number governs boundary layer behavior, which directly affects the surface pressure distributions that FADS relies on.

**Typical HPR transonic conditions (4-inch / 0.1m diameter rocket at Mach 1, sea level):**

```
Re_diameter = ρ × V × D / μ
            = 1.225 × 340 × 0.1 / 1.8×10⁻⁵
            ≈ 2.3 × 10⁶

Re_nosecone = ρ × V × L / μ  (for 4:1 ogive, L ≈ 0.4m)
            ≈ 9.2 × 10⁶
```

**Assessment:** Re > 10⁶ is firmly in the fully turbulent boundary layer regime. This is within the range used for transonic wind tunnel testing. The boundary layer behavior is more predictable (and less sensitive to surface roughness) in the turbulent regime than in transitional flow. Reynolds number is NOT a limiting factor for FADS or probe-based sensing at HPR scale.

For comparison, the small UAS multi-hole probe cited above operates successfully at Reynolds numbers orders of magnitude lower (Re ~ 10⁴–10⁵). The pressure-angle relationships are even more robust at HPR Reynolds numbers.

### 3.2 Transonic Flow Considerations

Between Mach 0.8 and 1.2, shock positions on the rocket body are inherently unsteady — shocks oscillate in position and strength. This creates specific challenges:

- **Pressure fluctuations at body-mounted ports:** Any port located where shocks pass will see large, rapid pressure transients. This is noise for FADS, not signal. Port placement must avoid regions of shock oscillation — the nosecone tip (stagnation region) and very forward nosecone positions are more stable than aft shoulder regions.
- **Higher oscillation frequencies at small scale:** Shock oscillation frequency scales inversely with body dimension. HPR-scale vehicles will see higher frequency fluctuations than full-scale aircraft. Pressure sensors need adequate bandwidth (1 kHz+) and the data reduction algorithm needs appropriate filtering.
- **Short transonic dwell time:** HPR rockets typically transit the transonic regime in 1–3 seconds. The air data system must acquire, filter, and output useful data within this window. This favors fast MEMS sensors with embedded processing over pneumatically-tubed remote transducers.

### 3.3 Nosecone Geometry Coupling (FADS Only)

A flush FADS array requires a calibrated pressure model that maps surface pressures to freestream conditions. This model depends on the nosecone geometry: ogive, Von Kármán, conical, elliptical, etc.

**Mitigations:**
- Standard HPR nosecone shapes have well-characterized analytical pressure distributions. Modified Newtonian theory provides good first-order calibration for blunt bodies; potential flow solutions work for slender ogives. CFD refinement is straightforward for axisymmetric bodies.
- Aether-R could ship with a calibration tool that takes nosecone shape parameters (type, fineness ratio, base diameter) and generates pressure model coefficients.
- For custom or 3D-printed nosecones, a single CFD run or even a test-stand calibration with known airflow would suffice.

This coupling is the primary argument for the probe variant (Aether-U) for general-purpose use, and for limiting Aether-R to dedicated research vehicles where the nosecone geometry is known and controlled.

### 3.4 Static Pressure Reference

Dynamic pressure measurement requires both stagnation (total) pressure and static (freestream) pressure. Obtaining a clean static reference is the harder problem at all scales.

**Options for Aether-R (rocket):**
1. **Internal avionics bay barometer:** The existing DPS310 inside a sealed-then-vented bay provides a lagged, filtered static reference. Not perfect during transonic passage, but it avoids external shock contamination. The lag can be characterized and compensated.
2. **Dedicated flush static ports:** Ports placed on the cylindrical body section well aft of the nosecone, away from shock interactions. Requires careful placement analysis per vehicle.
3. **Model-aided estimation:** Use the ESKF's altitude/velocity state to estimate static pressure, then use the stagnation port measurement to derive dynamic pressure as a correction. This creates a feedback loop — the air data measurement improves the ESKF state, which improves the static estimate, which improves the air data measurement.

**For Aether-U (probe):**
The multi-hole probe carries its own static ports on the probe body, downstream of the tip. This is self-contained and vehicle-independent — the primary advantage of the probe approach.

### 3.5 Feasibility Summary

| Factor | Aether-R (Flush/Rocket) | Aether-U (Probe/UAS) |
|--------|------------------------|----------------------|
| Reynolds number | ✅ Favorable (Re > 10⁶) | ✅ Proven at much lower Re |
| Geometry coupling | ⚠️ Requires per-nosecone calibration | ✅ Self-contained, vehicle-independent |
| Static pressure | ⚠️ Needs careful reference strategy | ✅ Integrated in probe |
| Transonic operation | ✅ Primary use case, high-value data | N/A (UAS are subsonic) |
| Structural concerns | ✅ Flush = no protrusions | ⚠️ Boom mount needs structural analysis |
| Existing precedent | ⚠️ None in HPR hobby | ✅ Active academic/commercial work |
| ESKF integration | ✅ Direct measurement update | ✅ Direct measurement update |

---

## 4. Proposed Architecture

### 4.1 Shared Electronics (Both Variants)

Both Aether-R and Aether-U use the same core PCB:

**Pressure Sensors:**
Candidate MEMS differential pressure sensors (evaluation needed):

| Sensor | Range | Resolution | Interface | Notes |
|--------|-------|------------|-----------|-------|
| Sensirion SDP31 | ±500 Pa | 0.002 Pa | I2C | Used in public-domain 9g probe. Low range — subsonic UAS only |
| TE MS5611 | 10–1200 mbar abs | 0.012 mbar | SPI/I2C | Fast (1 kHz+ capable), good resolution, proven in rocketry |
| Honeywell RSC series | Various | 0.01% FS | SPI | High accuracy, wide range options, higher cost |
| TDK ICP-10125 | 30–110 kPa | 0.4 Pa | I2C | Waterproof, very low noise, newer option |
| Amphenol AUAV series | Dual airspeed + altitude | MEMS piezoresistive | I2C | Purpose-built for UAV, combines two functions |

For Aether-R (transonic), the sensors must handle the full dynamic pressure range: at Mach 1.2 sea level, q ≈ 100 kPa. The SDP31 is insufficient; the MS5611 or RSC series are more appropriate.

For Aether-U (subsonic UAS), the SDP31's ±500 Pa range covers airspeeds up to ~28 m/s, which is adequate for most small UAS.

**Processing:**
The Booster Pack PCB runs the pressure-to-aerodata algorithm onboard:
- Polynomial regression (proven, lightweight, deterministic execution time)
- Or: pre-trained neural network (better accuracy at extreme angles, slightly higher compute)

Both approaches map raw differential pressures to calibrated α, β, airspeed, and dynamic pressure outputs.

**Interface to RocketChip Core:**
- SPI (preferred, consistent with production sensor architecture — 1.2% overhead at 1 kHz vs 30% for I2C)
- Output: `air_data_t` struct following the transport-neutral pattern in `docs/SENSOR_ARCHITECTURE.md`

```c
typedef struct {
    float dynamic_pressure_pa;  // q = 0.5 * ρ * V²
    float airspeed_ms;          // Calibrated airspeed (m/s)
    float alpha_deg;            // Angle of attack (degrees) — NaN if not available
    float beta_deg;             // Angle of sideslip (degrees) — NaN if not available
    float mach;                 // Mach number estimate — NaN if not available
    uint8_t num_ports_active;   // Number of ports returning valid data
    uint8_t quality;            // 0-100 confidence metric
    uint32_t timestamp_us;      // Microsecond timestamp
} air_data_t;
```

### 4.2 Aether-R: Flush Rocket Air Data

**Concept of operations:**
Aether-R is a dedicated nosecone assembly (or nosecone insert) with flush pressure ports plumbed to a sensor PCB mounted behind the nosecone bulkhead. It targets HPR vehicles in the 54mm–98mm (2.1"–3.9") diameter range that reach transonic or supersonic speeds.

**Port configurations (tiered):**

| Config | Ports | Measures | Use Case |
|--------|-------|----------|----------|
| Minimal | 1 (stagnation tip) | Dynamic pressure, airspeed | Transonic velocity bridging for ESKF |
| Basic | 3 (tip + top/bottom) | + Angle of attack (α) | TVC development, drag studies |
| Full | 5 (tip + cruciform) | + Angle of sideslip (β), Mach | Complete air data for research |
| Redundant | 7–9 (full + extras) | Same, with fault tolerance | High-reliability research campaigns |

**Physical implementation:**
- 3D-printed or machined nosecone tip insert with precision orifices
- Short pneumatic paths (< 50mm) to MEMS sensors on the Booster Pack PCB
- PCB mounts directly behind the nosecone bulkhead on standoffs
- Alternative: flush diaphragm sensors mounted directly at the port locations (eliminates tubing entirely, maximizes frequency response, but requires custom sensor packaging)

**Calibration strategy:**
- Ship with analytical pressure models for standard nosecone shapes (ogive, Von Kármán, conical, haack series) parameterized by fineness ratio
- Provide a desktop calibration tool that generates coefficients from nosecone geometry input
- Advanced users can refine with CFD or wind tunnel data

### 4.3 Aether-U: Multi-Hole Probe for UAS

**Concept of operations:**
Aether-U is a boom-mounted multi-hole pressure probe connected to a sensor PCB on the Booster Pack. It targets fixed-wing UAS, VTOL, and RC aircraft that need stall warning, wind estimation, or precision flight control.

**Probe design:**
Based on the public-domain 9-hole design (arxiv 2505.03331) adapted for RocketChip integration:
- 3D-printed probe body with hemispherical or conical tip
- 5 differential pressure sensors (SDP31 or equivalent) on integrated PCB
- Central hole at tip + 8 peripheral holes at 45° angle for α/β sensitivity
- 4 static ports on the probe body perpendicular to flow
- Target weight: < 15g including PCB and connector
- Probe diameter: ~8–12mm
- Mounting: carbon fiber or aluminum boom, 2–4 body diameters ahead of the airframe

**Calibration:**
- Factory-calibrated (probe geometry is fixed and known)
- Multivariate polynomial regression model stored on Booster Pack
- Calibration valid across the probe's operating range without per-vehicle adjustment
- This is the key advantage over flush FADS: calibrate once, fly on anything

**Stall detection integration:**
Aether-U enables direct stall warning based on measured α exceeding a configured threshold, rather than inferring stall from airspeed alone. This can feed:
- Audible/visual warnings via the RocketChip status system
- Autopilot α-limiting for autonomous flight
- Real-time α/β data to the ESKF for improved state estimation during high-AoA maneuvering

---

## 5. ESKF Integration

### 5.1 Measurement Model

Air data from Aether enters the ESKF as a measurement update. The measurement model relates the ESKF state (position, velocity, attitude) to expected air data readings:

**Dynamic pressure measurement:**
```
q_measured = P_total - P_static
q_predicted = 0.5 * ρ(altitude) * |V_airspeed|²

V_airspeed = V_inertial - V_wind
```

This provides a velocity magnitude constraint that is independent of GPS and barometric altitude — exactly the measurement that's missing during transonic flight.

**Angle of attack / sideslip measurements:**
```
α_measured = atan2(V_body_z, V_body_x)    (body-frame vertical vs forward velocity)
β_measured = asin(V_body_y / |V_airspeed|) (body-frame lateral velocity)
```

These constrain the relationship between the ESKF's attitude estimate and the velocity vector, providing observability that gyro integration alone cannot.

### 5.2 Flight-Phase-Aware Weighting

Consistent with RocketChip's existing flight-phase-aware sensor weighting approach, Aether measurements should have phase-dependent trust:

| Flight Phase | Dynamic Pressure | α/β | Rationale |
|-------------|-----------------|------|-----------|
| Pad / Pre-launch | Disabled | Disabled | No meaningful airflow |
| Boost (subsonic) | High weight | Medium weight | Clean airflow, but thrust effects on α |
| Boost (transonic) | Medium weight | Low weight | Shock oscillations add noise, but this is the highest-value phase for the measurement |
| Coast (supersonic) | High weight | High weight | Clean flow, no thrust contamination |
| Coast (subsonic) | High weight | High weight | Best conditions for air data |
| Descent / Recovery | Disabled | Disabled | Tumbling invalidates probe assumptions |

For UAS applications, weighting is simpler — air data is valid whenever the vehicle is in forward flight above a minimum airspeed threshold (typically 3–5 m/s).

---

## 6. Market Analysis

### 6.1 Rocket Market (Aether-R)

**No existing product** provides transonic air data for HPR. The closest alternatives are:
- Barometric altimeters (lose accuracy in transonic regime)
- Accelerometer integration (drifts without correction)
- GPS (may lose lock under high dynamics)
- Pitot tubes (structural liability at transonic+, single-axis only)

**Target users:**
- HPR flyers doing TVC development who need α feedback
- University/student rocketry teams (IREC, Spaceport America Cup) seeking aerodynamic data
- Research groups validating CFD models against flight data
- Anyone attempting altitude records who needs accurate transonic velocity

**Unique value:** Flight-measured aerodynamic coefficients on HPR-scale vehicles in actual transonic flight. This data essentially does not exist — wind tunnel data at these Reynolds numbers exists, but flight-measured aero coefficients on hobby-scale rockets in real transonic conditions would be genuinely novel.

### 6.2 UAS Market (Aether-U)

**Significant unserved demand** for affordable α/β sensing on small UAS:
- ArduPilot's airspeed sensing is pitot-tube only — single axis, no α, no β
- PX4 is the same
- No open-source autopilot ecosystem has an integrated multi-hole probe air data solution
- Commercial multi-hole probe systems cost $2,000–$10,000+
- Small UAS crashes due to undetected stall are common and preventable

**Target users:**
- Fixed-wing UAS operators needing stall protection
- VTOL transition vehicles (where α changes rapidly and unpredictably)
- Atmospheric research UAS needing wind vector measurement
- FPV and racing drone developers exploring envelope expansion
- Educational institutions teaching flight dynamics

**Target BOM cost:** < $50 for the Booster Pack (sensors + PCB + probe)

---

## 7. Open Questions and Next Steps

### 7.1 Open Questions

1. **Sensor selection:** The SDP31 (proven in the public-domain probe) is limited to ±500 Pa. Is this sufficient for UAS, and what sensor covers the transonic rocket case? The MS5611 and Honeywell RSC series need evaluation for noise, bandwidth, and transonic dynamic range.

2. **Flush diaphragm vs. tubing (Aether-R):** Mounting MEMS sensors directly at each port location eliminates pneumatic lag but requires custom sensor packaging or very short rigid tubes. What's the minimum practical tube length for acceptable acoustic response at 1 kHz sample rates?

3. **Static pressure strategy (Aether-R):** How much does the internal avionics bay barometer degrade as a static reference during transonic flight? Is model-aided estimation (ESKF feedback) sufficient, or are dedicated body-mounted static ports necessary?

4. **Calibration validation:** Can analytical models (modified Newtonian + corrections) achieve sufficient accuracy for standard HPR nosecone shapes, or is CFD mandatory? What accuracy is "good enough" for ESKF measurement updates vs. standalone air data?

5. **ESKF observability:** What is the actual velocity uncertainty of the 24-state ESKF propagating on IMU-only through a ~2-second transonic window? If it's already small, the value proposition of Aether-R shifts from ESKF correction to pure research data collection.

6. **Probe structural loads (Aether-U):** What are the aerodynamic loads on an 8–12mm probe at typical UAS speeds (15–30 m/s)? At HPR speeds (>100 m/s) a boom-mounted probe is impractical — this constrains Aether-U to subsonic UAS applications.

7. **Connector standardization:** Should Aether use the existing Booster Pack expansion connector, or does the pneumatic interface (tubing routing) require a different physical form factor?

### 7.2 Recommended Next Steps

1. **Literature deep-dive:** Obtain and review NASA TP-1999-209012 (FADS calibration for blunt forebodies) and the Northeastern Project Redshift flight data papers for practical implementation details.

2. **Sensor benchmarking:** Acquire SDP31 (for UAS variant) and MS5611 (for rocket variant) breakout boards. Characterize noise floor, bandwidth, and step response on the RP2350 via SPI.

3. **Probe prototype:** 3D-print a 5-hole probe following the public-domain design. Bench-test with a desk fan and known-angle fixturing to validate the polynomial calibration approach.

4. **CFD study:** Run OpenFOAM or similar on a standard 4:1 ogive nosecone with 5 port locations. Generate pressure coefficient tables across Mach 0.5–1.5 and α 0°–10°. This validates whether analytical models are sufficient or CFD calibration is required.

5. **ESKF integration design:** Define the measurement model Jacobians for air data updates. Determine observability conditions and expected state estimation improvement via covariance analysis before building hardware.

6. **Council review:** Once sensor benchmarking and probe prototype results are available, run a formal council review on the Aether architecture with the defined expert personas.

---

## 8. References

### 8.1 FADS / SEADS

- Cobleigh, B.R., et al. "Flush Airdata Sensing (FADS) System Calibration Procedure and Results for Blunt Forebodies." NASA/TP-1999-209012.
- Pruett, C.D., et al. "An innovative air data system for the Space Shuttle Orbiter — Data analysis techniques." NASA, 1982.
- Siemers, P.M., et al. "Shuttle Entry Air Data System (SEADS) — flight verification of an advanced air data system concept." AIAA-88-2104, 1988.
- "Shuttle Entry Air Data System (SEADS) hardware development. Volume 1: Summary." NASA, 1991. (NTRS 19910011866)
- Larson, T.J., et al. "Use of nose cap and fuselage pressure orifices for determination of air data for space shuttle orbiter below supersonic speeds." NASA, 1980. (NTRS 19800023881)
- Whitmore, S.A., et al. "Design and Calibration of the X-33 Flush Airdata Sensing (FADS) System." NASA, 1998.
- Ali, A.N., et al. "Analysis and Results from a Flush Airdata Sensing (FADS) System in Close Proximity to Firing Rocket Nozzles." NASA Dryden, 2014. (NTRS 20140009987)

### 8.2 Multi-Hole Probes

- "Miniature multihole airflow sensor for lightweight aircraft over wide speed and angular range." arXiv:2505.03331, May 2025. **(Public domain design — primary reference for Aether-U.)**
- Rediniotis, O.K., et al. "Embedded-sensor multi-hole probes." US Patent 7,010,970 B2.
- Samy, I., et al. "Subsonic tests of a flush air data sensing system applied to a fixed-wing micro air vehicle." J. Intell. Robot. Syst., 2009.
- "Neural-Network-Based Flush Air Data Sensing System Demonstrated on a Mini Air Vehicle." (Demonstrates ANN-based data reduction for small-scale FADS.)
- Johansen, E.S., et al. "Use and calibration of 5-hole pressure probes to measurement of airflow velocity." (Open-source "Oxford probe" design reference.)

### 8.3 HPR Aerodynamics

- Braeunig, R. "Drag Coefficient Prediction." (HPR transonic drag analysis methods and sounding rocket data.)
- "Aerodynamics Modeling of Sounding Rockets: A Computational Fluid Dynamics Study." (Maxus / Terrier-Black Brant CFD validation.)
- "Experimental and Numerical Analysis of the Flow Patterns around a Sounding Rocket in the Transonic Regime." Instituto de Aeronáutica e Espaço, Brazil. (PSP and pressure tap data on 1:34 scale sounding rocket model, Mach 0.6–1.0.)

### 8.4 UAS Air Data and Stall Detection

- "Aerodynamic Sensing for a Fixed Wing UAS Operating at High Angles of Attack." University of Michigan, AIAA 2012-4416. (Demonstrates pressure-based stall detection on small UAS.)
- "Estimation of Airspeed, Angle of Attack, and Sideslip for Small UAVs Using a Micro-Pitot Tube." Electronics, 2021. (IMU + differential pressure fusion approach.)
- Amphenol All Sensors. "AUAV Series Dual Airspeed and Altitude Pressure Sensor." (Commercial MEMS sensor purpose-built for UAV air data.)
