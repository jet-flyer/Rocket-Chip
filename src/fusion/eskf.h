#ifndef ROCKETCHIP_FUSION_ESKF_H
#define ROCKETCHIP_FUSION_ESKF_H

// ESKF: 24-state Error-State Kalman Filter.
// Pure C++ — no Pico SDK dependencies.
//
// Nominal state: quaternion (body-to-NED), position, velocity, biases,
//   earth mag field, body mag bias, wind, baro bias.
// Error state: 24-dimensional per eskf_state.h.
// States 15-23 use runtime inhibit flags (ArduPilot pattern).
// Reference: Solà (2017) "Quaternion kinematics for the error-state KF"
//   arXiv:1711.02508 — 560+ citations (Semantic Scholar, 2026).
//   Industry standard reference used by PX4 ECL-EKF2 and ArduPilot EKF2/3.
//   Tutorial/consolidation paper, not novel research — standardizes the
//   quaternion error-state formulation used across robotics and aerospace.
//
// All noise parameters from ICM-20948 DS-000189 v1.3 Tables 1-2
// unless noted as empirical. Every constant has a source citation.

#include "fusion/eskf_state.h"
#include "math/mat.h"
#include "math/quat.h"
#include "math/vec3.h"

namespace rc {

struct ESKF {
    // =================================================================
    // Nominal state (true state estimate)
    // =================================================================
    Quat q;             // Body-to-NED rotation quaternion
    Vec3 p;             // NED position (m)
    Vec3 v;             // NED velocity (m/s)
    Vec3 accel_bias;    // Accelerometer bias estimate (m/s²)
    Vec3 gyro_bias;     // Gyroscope bias estimate (rad/s)

    // --- Extended states (runtime inhibit flags, ArduPilot pattern) ---
    Vec3 earth_mag;         // Earth magnetic field NED (uT)
    Vec3 body_mag_bias;     // Hard-iron bias body frame (uT)
    float wind_n_{};        // Horizontal wind North (m/s)
    float wind_e_{};        // Horizontal wind East (m/s)
    float baro_bias_{};     // Barometric altitude bias (m)

    // =================================================================
    // Runtime inhibit flags — ArduPilot EKF3 pattern
    // When true: P diagonal = 0, cross-covariances = 0, no compute cost.
    // ArduPilot: inhibitMagStates, inhibitWindStates (NavEKF3_core.h).
    // =================================================================
    bool inhibit_mag_states_{true};    // earth_mag[15-17] + body_mag_bias[18-20]
    bool inhibit_wind_states_{true};   // wind_NE[21-22]
    bool inhibit_baro_bias_{true};     // baro_bias[23]

    // =================================================================
    // Error-state covariance (24×24, symmetric)
    // =================================================================
    Mat24 P;

    // =================================================================
    // Physical constants
    // =================================================================

    // WGS84 standard gravity
    static constexpr float kGravity = 9.80665f;  // m/s²

    // =================================================================
    // IMU noise parameters — ICM-20948 DS-000189 v1.3
    // Reviewed IVP-47 Step 6: all values verified correct.
    // Bias walk values conservative for stationary/low-vibration.
    // IVP-52 note: may need 10-100× increase during boost/coast phases.
    // =================================================================

    // Gyro noise spectral density: 0.015 dps/√Hz (Table 1, BW=10Hz, FS_SEL=0)
    // Conversion: 0.015 × π/180 = 2.618e-4 rad/s/√Hz
    static constexpr float kSigmaGyro = 2.618e-4f;  // rad/s/√Hz

    // Accel noise spectral density: 230 µg/√Hz (Table 2, BW=10Hz)
    // Conversion: 230e-6 × 9.80665 = 2.256e-3 m/s²/√Hz
    static constexpr float kSigmaAccel = 2.256e-3f;  // m/s²/√Hz

    // Gyro bias random walk — no datasheet spec (ICM-20948 does not publish
    // bias instability / Allan deviation). Empirical starting point.
    // ArduPilot EKF3 GBIAS_P_NSE = 1e-3 rad/s (100× larger, multirotor vibration).
    // Conservative for ground use. Tune via NEES if bias states don't converge.
    static constexpr float kSigmaGyroBiasWalk = 1.0e-5f;  // rad/s²/√Hz

    // Accel bias random walk — no datasheet spec. Empirical starting point.
    // ArduPilot EKF3 ABIAS_P_NSE = 2e-2 m/s² (200× larger, multirotor vibration).
    // Conservative for ground use. Tune via NEES if bias states don't converge.
    static constexpr float kSigmaAccelBiasWalk = 1.0e-4f;  // m/s³/√Hz

    // =================================================================
    // Extended state noise — ArduPilot EKF3 defaults (Gauss→uT converted)
    // ArduPilot uses Gauss internally: 1 Gauss = 100 uT.
    // EK3_MAGE_P_NSE = 1e-3 Gauss/s = 0.1 uT/s
    // EK3_MAGB_P_NSE = 1e-4 Gauss/s = 0.01 uT/s
    // EK3_WIND_P_NSE = 0.1 m/s/s (Plane/Rover default)
    // =================================================================

    // Earth magnetic field process noise (NED frame drift)
    // ArduPilot EK3_MAGE_P_NSE default = 1e-3 Gauss/s = 0.1 uT/s
    static constexpr float kSigmaEarthMagWalk = 0.1f;  // uT/s

    // Body magnetic field bias process noise (hard-iron drift)
    // ArduPilot EK3_MAGB_P_NSE default = 1e-4 Gauss/s = 0.01 uT/s
    static constexpr float kSigmaBodyMagBiasWalk = 0.01f;  // uT/s

    // Wind velocity process noise (horizontal gust variance)
    // ArduPilot EK3_WIND_P_NSE: 0.1 (Plane/Rover), 0.2 (Copter).
    // Parachute descent has high drag + low inertia = sensitive to gusts,
    // more like Copter than Plane. 0.2 matches Copter default.
    static constexpr float kSigmaWindWalk = 0.2f;  // m/s/s

    // Barometric altitude bias process noise.
    // Neither ArduPilot EKF3 nor PX4 ECL model baro bias as a dynamic state
    // (both treat it as a static offset). Primarily useful for HAB/long-duration
    // flights where temperature-driven baro drift matters (hours, not seconds).
    // For a 30-120s rocket flight, baro drift is negligible — keep inhibited.
    // DPS310 temperature coefficient: ±0.5 Pa/K → ~0.04 m/K.
    static constexpr float kSigmaBaroBiasWalk = 0.01f;  // m/s

    // =================================================================
    // P initialization — Solà (2017) typical values + ICM-20948 ZRO specs
    // =================================================================

    // Attitude: ~18° initial uncertainty (Solà typical for gravity-only init)
    static constexpr float kInitPAttitude = 0.1f;  // rad²

    // Position: ~10m uncertainty (no GPS fix at init)
    static constexpr float kInitPPosition = 100.0f;  // m²

    // Velocity: ~1 m/s uncertainty (stationary start assumed)
    static constexpr float kInitPVelocity = 1.0f;  // m²/s²

    // Accel bias: ~0.1 m/s² = ~10mg (within ±50mg board-level ZRO, Table 2)
    static constexpr float kInitPAccelBias = 0.01f;  // (m/s²)²

    // Gyro bias: ~0.017 rad/s = ~1°/s (within ±5°/s ZRO, Table 1)
    static constexpr float kInitPGyroBias = 3.0e-4f;  // (rad/s)²

    // Extended state P initialization (used when inhibit flag is cleared)
    static constexpr float kInitPEarthMag = 100.0f;       // uT² (~10 uT uncertainty)
    static constexpr float kInitPBodyMagBias = 25.0f;     // uT² (~5 uT uncertainty)
    static constexpr float kInitPWind = 25.0f;            // m²/s² (~5 m/s uncertainty)
    static constexpr float kInitPBaroBias = 1.0f;         // m² (~1 m uncertainty)

    // =================================================================
    // P diagonal clamping — council review RF-2
    // =================================================================

    static constexpr float kClampPAttitude = 1.0f;      // rad² (~57° max)
    static constexpr float kClampPPosition = 10000.0f;   // m² (~100m max)
    static constexpr float kClampPVelocity = 100.0f;     // m²/s² (~10 m/s max)

    // Extended state clamping
    static constexpr float kClampPEarthMag = 400.0f;      // uT² (~20 uT max)
    static constexpr float kClampPBodyMagBias = 100.0f;   // uT² (~10 uT max)
    static constexpr float kClampPWind = 2500.0f;         // m²/s² (~50 m/s max)
    static constexpr float kClampPBaroBias = 25.0f;       // m² (~5 m max)

    // =================================================================
    // Stationarity check — council review RF-5
    // =================================================================

    // Accel magnitude tolerance around gravity: ±0.1g
    static constexpr float kStationaryAccelTol = 0.981f;  // m/s² (0.1 × g)

    // Max gyro rate during "stationary" condition
    static constexpr float kStationaryGyroMax = 0.02f;  // rad/s (~1.1°/s)

    // =================================================================
    // Health sentinel — IVP-48 (velocity divergence guard)
    // Max plausible velocity: above hobby rocket burnout (~Mach 1.5 ≈ 510 m/s)
    // but well below any divergence trajectory (1688 m/s after ~3 min of accel-bias
    // at 9.8 m/s²). Catches silent ICM-20948 zero-output fault before vel grows
    // large enough to corrupt bias estimates.
    // =================================================================

    // Source: Mach 1.5 at sea level ≈ 510 m/s — 500 m/s gives ~1s headroom.
    static constexpr float kMaxHealthyVelocity = 500.0f;  // m/s

    // =================================================================
    // Barometric altitude measurement — IVP-43
    // DPS310 @ 16x oversampling noise (same as BaroKF)
    // =================================================================

    static constexpr float kSigmaBaro = 0.029f;                        // m
    static constexpr float kRBaro = kSigmaBaro * kSigmaBaro;           // ~0.000841 m²
    static constexpr float kBaroInnovationGate = 3.0f;                 // 3σ gate

    // =================================================================
    // Magnetometer heading measurement — IVP-44
    // AK09916 noise: 0.1 µT RMS at ~45 µT total field → ~0.002 rad.
    // Soft iron residuals dominate — start conservative at 5°.
    // Source: PHASE5_ESKF_PLAN.md, ArduPilot EKF3 MAG_NOISE.
    //
    // H ≈ [0, 0, 1, 0...0] (yaw-only, level-flight approximation).
    // Valid while roll/pitch < ~45°. Above that, cross-coupling from
    // roll/pitch into heading exceeds kSigmaMagHeading and the update
    // effectively becomes a slow correction rather than a tight lock.
    // Full attitude-dependent H deferred to Titan tier.
    //
    // Max trackable spin rate at 10Hz mag update: ~5 rev/s (900°/s).
    // Beyond this, wrap_pi() on the innovation aliases. The ESKF
    // gyro-driven predict (200Hz) tracks arbitrarily fast rotations;
    // only the mag correction has this limit.
    // =================================================================

    static constexpr float kSigmaMagHeading = 0.087f;                   // rad (~5°)
    static constexpr float kRMagHeading = kSigmaMagHeading * kSigmaMagHeading;  // ~0.00757 rad²
    // ArduPilot EKF3 MAG_GATE = 300σ — physically untriggerable since
    // max innovation (π rad via wrap_pi) < 300×√R ≈ 26 rad.
    // Interference detection handles bad data, R controls correction
    // strength. Previous 3σ gate caused mNIS=124.99 death spiral.
    static constexpr float kMagInnovationGate = 300.0f;                  // ArduPilot match
    static constexpr float kMagInterferenceThreshold = 0.25f;           // 25% deviation → inflate R
    static constexpr float kMagInterferenceRejectThreshold = 0.50f;     // 50% deviation → hard reject (council)
    static constexpr float kMagInterferenceRScale = 10.0f;              // R inflation factor (25-50% band)
    static constexpr float kMagMinMagnitude = 1.0f;                     // µT — below this, reject

    // Consecutive rejection threshold for state machine use (IVP-52).
    // With 300σ gate, automatic reset is unnecessary (gate never fires).
    // The state machine can call reset_mag_heading() when this is exceeded.
    static constexpr uint32_t kMagResetAfterRejects = 50;  // ~5s at 10Hz mag

    // Tilt-conditional R inflation (IVP-47, ArduPilot fuseEulerYaw pattern).
    // At large tilt, the H≈[0,0,1] linearization has cross-coupling from
    // roll/pitch uncertainty into heading. Inflate R proportionally between
    // threshold and max; hard-reject above max.
    static constexpr float kMagTiltThresholdRad = 0.524f;   // ~30° — cross-coupling >5° heading error
    static constexpr float kMagTiltMaxRad = 1.047f;         // ~60° — heading extraction unreliable
    // R multiplier at max tilt. Range 50-500 acceptable; 100 chosen as
    // conservative middle. Higher values weaken correction more at tilt.
    static constexpr float kMagTiltRInflationMax = 100.0f;

    // =================================================================
    // Zero-velocity pseudo-measurement (ZUPT) — IVP-44b
    // Constrains horizontal velocity when stationary. Without GPS or
    // ZUPT, horizontal v/p states are unobservable and diverge within
    // ~30s, corrupting accel bias estimation via positive feedback.
    //
    // Stationarity detection uses kStationaryAccelTol and
    // kStationaryGyroMax (same as init() RF-5). When both are met,
    // inject v=[0,0,0] as three scalar measurements.
    //
    // ArduPilot EKF3: InertialNav zero-velocity when onGround.
    // PX4 ECL-EKF2: zero_velocity_update() with accel/gyro checks.
    // =================================================================

    // ZUPT measurement noise: how precisely we know v=0.
    // Must be loose enough that velocity covariance P_v doesn't collapse.
    // If P_v → 0, the Kalman gain for subsequent updates vanishes and
    // the ZUPT can't fight accel-bias-driven velocity accumulation.
    // ArduPilot EKF3 uses ~0.5 m/s for on-ground zero-velocity.
    // PX4 ECL uses similar order. We match ArduPilot.
    static constexpr float kSigmaZupt = 0.5f;                           // m/s
    static constexpr float kRZupt = kSigmaZupt * kSigmaZupt;            // 0.25 m²/s²
    static constexpr float kZuptInnovationGate = 5.0f;                   // 5σ gate (generous)

    // =================================================================
    // GPS position measurement — IVP-46
    // MT3333 CEP50 = 3m (PA1010D datasheet). σ ≈ CEP50 / 0.833 ≈ 3.6m.
    // Use 3.5m base, scaled by HDOP. Council condition C-1.
    // ArduPilot EK3_GPS_POS_NOISE default 0.5m is for u-blox M8/M9 with SBAS.
    // =================================================================

    // Earth radius: ArduPilot RADIUS_OF_EARTH (AP_Math/definitions.h)
    static constexpr float kEarthRadius = 6378100.0f;  // m

    static constexpr float kSigmaGpsPosBase = 3.5f;           // m (horizontal, HDOP=1)
    static constexpr float kSigmaGpsVertScale = 2.0f;         // vertical σ multiplier
    static constexpr float kRGpsPosDefault = kSigmaGpsPosBase * kSigmaGpsPosBase;  // 12.25 m²

    // GPS velocity noise — ArduPilot EK3_VELNE_M_NSE default = 0.5 m/s.
    // Fixed R (no sAcc from NMEA). Scale with sAcc when UBX available (IVP-48).
    static constexpr float kSigmaGpsVel = 0.5f;               // m/s
    static constexpr float kRGpsVel = kSigmaGpsVel * kSigmaGpsVel;  // 0.25 m²/s²

    // Innovation gates — 5σ, matching ArduPilot GPS gating
    static constexpr float kGpsPositionGate = 5.0f;
    static constexpr float kGpsVelocityGate = 5.0f;

    // Min ground speed for velocity update — below this, GPS course unreliable
    static constexpr float kGpsMinSpeedForVel = 0.5f;  // m/s

    // Origin quality gate — reject first fix with poor geometry. Council C-4.
    static constexpr float kGpsMaxHdopForOrigin = 4.0f;

    // Moving origin threshold — re-center NED frame when |p.xy| exceeds this.
    // Keeps flat-earth error < 1m. Relevant for HABs (50-100km downrange).
    static constexpr float kOriginResetDistance = 10000.0f;  // m

    // =================================================================
    // NED frame origin — double precision for lat/lon (council C-5).
    // Float32 gives ~1.19m resolution at equator, marginal for origin.
    // =================================================================
    double origin_lat_rad_{};
    double origin_lon_rad_{};
    float origin_alt_m_{};
    float cos_origin_lat_{};   // precomputed cos(origin_lat) for NED conversion
    bool has_origin_{};

    // =================================================================
    // Methods
    // =================================================================

    // Set NED origin from first quality GPS fix (HDOP <= kGpsMaxHdopForOrigin).
    // Resets position/velocity P to GPS-derived uncertainty (council fix).
    // Returns false if origin already set or HDOP too high.
    bool set_origin(double latRad, double lonRad, float altM, float hdop);

    // Reset NED origin to new position, adjusting p for continuity.
    // Used for moving-origin when |p.xy| > kOriginResetDistance.
    void reset_origin(double newLatRad, double newLonRad, float newAltM);

    // Convert WGS84 geodetic to NED. Double lat/lon for precision (council C-5).
    // Subtraction (lat - origin_lat) in double before float cast.
    Vec3 geodetic_to_ned(double latRad, double lonRad, float altM) const;

    // GPS position update: 3 sequential scalar updates (N, E, D).
    // R_horiz = (kSigmaGpsPosBase * max(hdop, 1))².
    // R_vert = R_horiz * kSigmaGpsVertScale² (or VDOP-based if vdop > 0).
    bool update_gps_position(const Vec3& gpsNed, float hdop = 0.0f,
                              float vdop = 0.0f);

    // GPS velocity update: 2 sequential scalar updates (N, E only).
    // Vertical velocity from GPS unreliable — baro + IMU handle vertical.
    bool update_gps_velocity(float vNorth, float vEast);

    // Initialize from stationary accelerometer reading.
    // Derives initial attitude from gravity vector direction.
    // Returns false if stationarity check fails (RF-5).
    bool init(const Vec3& accelMeas, const Vec3& gyroMeas);

    // Propagate (predict) step: integrate IMU measurements forward by dt.
    // Uses sparse FPFT exploiting F_x block structure (R-1).
    // Solà (2017) §5.3.
    void predict(const Vec3& accelMeas, const Vec3& gyroMeas, float dt);

    // Propagate using dense FPFT (verification path only).
    // Same result as predict() but slower — used to validate sparse path.
    void predict_dense(const Vec3& accelMeas, const Vec3& gyroMeas, float dt);

    // Barometric altitude measurement update (IVP-43).
    // z = altitude_agl_m (positive up, from calibration_get_altitude_agl).
    // h(x) = -p.z (NED down negated). H = [0 0 0 | 0 0 -1 | 0...].
    // Returns false if input is non-finite or innovation is gated out.
    // Solà (2017) §7.2, Joseph form for P update.
    bool update_baro(float altitudeAglM);

    // Magnetometer heading measurement update (IVP-44).
    // mag_body: calibrated magnetometer reading in body frame (µT).
    // expected_magnitude: expected field magnitude from cal (µT, 0 = skip
    //   interference check). Two-tier interference detection (council):
    //   >25% deviation inflates R by 10x, >50% hard rejects.
    // declination_rad: magnetic declination (rad, East-positive). Added to
    //   measured magnetic heading so the ESKF tracks true heading.
    //   GPS-derived WMM lookup (IVP-46) or user parameter. 0 = magnetic heading.
    //   ArduPilot: MagDeclination(), PX4: get_mag_declination().
    // h(x) = -atan2(m_level.y, m_level.x) + declination.
    // H ≈ [0, 0, 1, 0...0] (yaw-only approximation — see constants block).
    // Returns false if non-finite, magnitude too low, interference reject,
    //   or innovation gated out. Solà (2017) §6.2, Joseph form.
    bool update_mag_heading(const Vec3& magBody, float expectedMagnitude,
                            float declinationRad = 0.0f);

    // Force-reset yaw to measured heading (IVP-47).
    // Resets q yaw, P[yaw,yaw] = kInitPAttitude, zeros yaw cross-covariances.
    // Navigation states may briefly wobble; settles in 2-3s with GPS.
    // Public API for state machine (IVP-52) and explicit heading alignment.
    void reset_mag_heading(float headingMeasured);

    // Zero-velocity pseudo-measurement update (IVP-44b).
    // Checks stationarity from raw IMU data (accel magnitude ≈ g,
    // gyro rates < threshold). If stationary, applies v=[0,0,0] as
    // three sequential scalar measurements on velocity states [6..8].
    // Returns false if not stationary or innovation gated out.
    // Call at predict() rate (200Hz) — stationarity check is cheap.
    bool update_zupt(const Vec3& accelMeas, const Vec3& gyroMeas);

    // Error state reset with Jacobian G after measurement update.
    // Absorbs error state into nominal, resets error to zero.
    // Solà (2017) §7.2, council RF-1.
    void reset(const Vec3& deltaTheta);

    // Clamp P diagonal to prevent runaway (council RF-2).
    // Also zeroes inhibited state blocks (R-8: codegen adds Q_d to all 24 states,
    // clamp_covariance() then zeroes the inhibited blocks back to 0).
    void clamp_covariance();

    // Health check: NaN detection, P bounds, quaternion norm (council RF-3).
    // Inhibit-aware (R-1): skips P diagonal check for inhibited state indices.
    bool healthy() const;

    // =================================================================
    // Inhibit flag control — ArduPilot pattern
    // When enabling (inhibit=false): sets P diagonal to initial variance (R-7).
    // When disabling (inhibit=true): zeros P diagonal + cross-covariances.
    // =================================================================
    void set_inhibit_mag(bool inhibit);
    void set_inhibit_wind(bool inhibit);
    void set_inhibit_baro_bias(bool inhibit);

    // =================================================================
    // Diagnostics
    // =================================================================
    float last_propagation_dt_{};
    float last_baro_nis_{};        // IVP-43: Normalized Innovation Squared
    float last_mag_nis_{};         // IVP-44: Normalized Innovation Squared
    uint32_t mag_consecutive_rejects_{};  // IVP-47: heading reset trigger counter
    uint32_t mag_total_rejects_{};        // IVP-47: lifetime reject count
    uint32_t mag_total_accepts_{};        // IVP-47: lifetime accept count
    uint32_t mag_resets_{};               // IVP-47: heading reset count
    float last_zupt_nis_{};        // IVP-44b: max ZUPT NIS across 3 axes
    float last_gps_pos_nis_{};     // IVP-46: max NIS across 3 position axes
    float last_gps_vel_nis_{};     // IVP-46: max NIS across 2 velocity axes
    bool last_zupt_active_{};      // IVP-44b: true when stationarity detected

    // Per-sensor diagnostic counters (IVP-47)
    uint32_t baro_total_accepts_{};
    uint32_t baro_total_rejects_{};
    uint32_t gps_pos_total_accepts_{};
    uint32_t gps_pos_total_rejects_{};
    uint32_t gps_vel_total_accepts_{};
    uint32_t gps_vel_total_rejects_{};
    uint32_t zupt_total_accepts_{};
    uint32_t zupt_total_rejects_{};
    bool initialized_{};

private:
    // Build the error-state transition matrix F_x (24×24).
    // F_x = I + dt * F_delta, where F_delta encodes the linearized dynamics.
    // States 15-23 have identity F propagation (no coupling to core states).
    // Output parameter to avoid stack pressure (LL Entry 1).
    static void build_F(Mat24& out, const Quat& q, const Vec3& accelBody,
                         const Vec3& gyroBody, float dt);

    // Build continuous-time process noise Q_c (24×24 diagonal).
    // Output parameter to avoid stack pressure (LL Entry 1).
    static void build_Qc(Mat24& out);

    // Zero P rows/columns for a contiguous block of states.
    // Used by inhibit flag control and clamp_covariance().
    void zero_p_block(int32_t startIdx, int32_t count);

    // Sparse P ← G*P*G^T for attitude-only reset (G=I except 3×3 block).
    void reset_covariance_attitude(const float ga[3][3]);

    // Clamp sub-functions (decomposed for readability-function-cognitive-complexity)
    void clamp_core_covariance();
    void clamp_extended_covariance();

    // Common propagation logic shared by predict() and predict_dense().
    void propagate_nominal(const Vec3& accelMeas, const Vec3& gyroMeas,
                           float dt);

    // Inject error-state correction δx into the nominal state.
    // Applies position, velocity, accel bias, gyro bias deltas + quaternion reset.
    // Called by all measurement update functions after computing Kalman gain.
    void inject_error_state(const Mat<eskf::kStateSize, 1>& dx);

    // Scalar Kalman update with Joseph-form P correction.
    // hIdx: column index of the single non-zero H entry.
    // hValue: value of H at that entry (+1.0F or -1.0F).
    // innovation: measurement residual (z - h(x)).
    // r: scalar measurement noise variance.
    // SAFETY: single-threaded, Core 0 only, never called from ISR.
    // NOTE: -fno-threadsafe-statics (Pico SDK). Static locals zero-init at load.
    void scalar_kalman_update(int32_t hIdx, float hValue,
                              float innovation, float r);
};

} // namespace rc

#endif // ROCKETCHIP_FUSION_ESKF_H
