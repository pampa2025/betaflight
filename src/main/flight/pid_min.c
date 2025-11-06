// Minimal Betaflight-style PID controller
// Purpose: provide a tiny, dependency-free core that mirrors the essential
// math of pid.c for one axis, suitable for growing feature-by-feature and
// later translating to TypeScript.

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Gains, scaled to runtime floating values
typedef struct {
    float Kp; // proportional gain
    float Ki; // integral gain
    float Kd; // derivative gain
    float Kf; // feedforward gain (applied to setpoint change)
} PidMinCoefficients;

// I-term relax type (declare before use in config)
typedef enum {
    PID_MIN_ITERM_RELAX_OFF = 0,
    PID_MIN_ITERM_RELAX_SETPOINT = 1, // scale I error by HPF-based factor
    PID_MIN_ITERM_RELAX_GYRO = 2      // reconstruct I error using deadband against HPF magnitude
} PidMinItermRelaxType;

// Minimal configuration for clamping and optional D lowpass
typedef struct {
    float pidSumLimit;       // clamp output sum to [-limit, +limit]; 0 disables
    float itermLimit;        // clamp integrator to [-limit, +limit]; 0 disables
    float integratorLeak;    // 0..1 fraction leaked per update; 0 disables
    bool  useFeedforward;    // include setpoint change feedforward
    float dLowpassCutoffHz;  // PT1 cutoff for gyro used by D-term; 0 disables
    // I-term relax (minimal): reduces integral accumulation during setpoint transients
    bool  iRelaxEnabled;     // master enable for I-term relax
    float iRelaxCutoffHz;    // PT1 cutoff (Hz) for setpoint lowpass used to derive HPF magnitude
    float iRelaxSetpointThreshold; // threshold (deg/s) for relax factor (default ~40)
    PidMinItermRelaxType iRelaxType; // OFF, SETPOINT, or GYRO
} PidMinConfig;

// Internal state per axis
typedef struct {
    float integrator;       // I-term accumulator
    float prevGyroFiltered; // last filtered gyro (for D-term stability)
    float prevSetpoint;     // last setpoint (for feedforward)
    float prevSetpointLpf;  // last lowpassed setpoint (for I-term relax)
    bool  initialized;      // guard first update
} PidMinState;

// (enum moved earlier so it is available to PidMinConfig)

// --- Launch Control (minimal) -----------------------------------------------
// Modes mirror Betaflight intent but kept minimal here.
typedef enum {
    PID_MIN_AXIS_ROLL = 0,
    PID_MIN_AXIS_PITCH = 1,
    PID_MIN_AXIS_YAW = 2
} PidMinAxis;

typedef enum {
    PID_MIN_LAUNCH_MODE_PITCHONLY = 0, // affect only pitch; roll/yaw P&I disabled
    PID_MIN_LAUNCH_MODE_FULL = 1       // affect all axes; yaw I limited
} PidMinLaunchMode;

typedef struct {
    bool enabled;              // launch control feature enabled
    PidMinLaunchMode mode;     // pitch-only or full
    float angleLimitDeg;       // pitch angle limit in degrees (<=0 disables)
    float kiOverride;          // Ki override while active (0 uses normal Ki)
} PidMinLaunchConfig;

// Constants derived from Betaflight behavior (kept simple)
#define LAUNCH_CONTROL_MAX_RATE        100.0f
#define LAUNCH_CONTROL_MIN_RATE        5.0f
#define LAUNCH_CONTROL_ANGLE_WINDOW    10.0f
#define LAUNCH_CONTROL_YAW_ITERM_LIMIT 50.0f

static inline float clampf(float x, float lo, float hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static inline float scaleRangef(float x, float inLo, float inHi, float outLo, float outHi) {
    const float spanIn = (inHi - inLo);
    if (spanIn <= 0.0f) return outLo;
    float t = (x - inLo) / spanIn;
    t = clampf(t, 0.0f, 1.0f);
    return outLo + t * (outHi - outLo);
}

// Symmetric deadband: subtracts magnitude up to `deadband` and preserves sign
static inline float fapplyDeadband(float x, float deadband) {
    const float ax = fabsf(x);
    const float ad = fabsf(deadband);
    if (ax <= ad) return 0.0f;
    const float sign = (x >= 0.0f) ? 1.0f : -1.0f;
    return sign * (ax - ad);
}

// PT1 alpha: alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
static inline float pt1Alpha(float cutoffHz, float dt) {
    if (cutoffHz <= 0.0f || dt <= 0.0f) return 1.0f; // no filtering
    const float rc = 1.0f / (2.0f * (float)M_PI * cutoffHz);
    return dt / (rc + dt);
}

// Initialize state
void pidMinInit(PidMinState *s) {
    if (!s) return;
    s->integrator = 0.0f;
    s->prevGyroFiltered = 0.0f;
    s->prevSetpoint = 0.0f;
    s->prevSetpointLpf = 0.0f;
    s->initialized = false;
}

// Reset integrator and derivative history
void pidMinReset(PidMinState *s) {
    if (!s) return;
    s->integrator = 0.0f;
    s->prevGyroFiltered = 0.0f;
    s->prevSetpoint = 0.0f;
    s->prevSetpointLpf = 0.0f;
    s->initialized = false;
}

// Core PID update for one axis
// Inputs:
// - c: gains
// - cfg: minimal runtime config
// - s: state
// - setpoint: desired rate (deg/s)
// - gyro: measured rate (deg/s)
// - dt: timestep seconds (>0)
// Returns: unclamped PID sum (clamped to pidSumLimit if configured)
float pidMinUpdate(const PidMinCoefficients *c,
                   const PidMinConfig *cfg,
                   PidMinState *s,
                   float setpoint,
                   float gyro,
                   float dt)
{
    if (!c || !cfg || !s) return 0.0f;
    if (dt <= 0.0f) return 0.0f;

    // Initialize filtered reference on first run
    if (!s->initialized) {
        s->prevGyroFiltered = gyro;
        s->prevSetpoint = setpoint;
        s->prevSetpointLpf = setpoint;
        s->initialized = true;
    }

    // Error (rate)
    const float errorRate = setpoint - gyro;
    // I-term relax: derive integrator error considering setpoint transients
    float iErrorRate = errorRate;
    if (cfg->iRelaxEnabled && cfg->iRelaxType != PID_MIN_ITERM_RELAX_OFF) {
        const float alphaRelax = (cfg->iRelaxCutoffHz > 0.0f) ? pt1Alpha(cfg->iRelaxCutoffHz, dt) : 1.0f;
        const float setpointLpf = s->prevSetpointLpf + alphaRelax * (setpoint - s->prevSetpointLpf);
        const float setpointHpf = fabsf(setpoint - setpointLpf);
        s->prevSetpointLpf = setpointLpf;
        if (cfg->iRelaxType == PID_MIN_ITERM_RELAX_SETPOINT) {
            const float threshold = (cfg->iRelaxSetpointThreshold > 0.0f) ? cfg->iRelaxSetpointThreshold : 40.0f;
            const float relaxFactor = fmaxf(0.0f, 1.0f - setpointHpf / threshold);
            iErrorRate *= relaxFactor;
        } else if (cfg->iRelaxType == PID_MIN_ITERM_RELAX_GYRO) {
            iErrorRate = fapplyDeadband(setpointLpf - gyro, setpointHpf);
        }
    }

    // P-term
    float P = c->Kp * errorRate;

    // I-term with leak and clamp
    s->integrator += c->Ki * iErrorRate * dt;
    if (cfg->integratorLeak > 0.0f) {
        const float leak = clampf(cfg->integratorLeak, 0.0f, 1.0f);
        s->integrator *= (1.0f - leak);
    }
    if (cfg->itermLimit > 0.0f) {
        s->integrator = clampf(s->integrator, -cfg->itermLimit, cfg->itermLimit);
    }
    float I = s->integrator;

    // D-term: derivative of filtered gyro (measurement derivative)
    const float alpha = (cfg->dLowpassCutoffHz > 0.0f) ? pt1Alpha(cfg->dLowpassCutoffHz, dt) : 1.0f;
    const float gyroFiltered = s->prevGyroFiltered + alpha * (gyro - s->prevGyroFiltered);
    const float dGyro = gyroFiltered - s->prevGyroFiltered;
    s->prevGyroFiltered = gyroFiltered;
    float D = c->Kd * (-(dGyro) / dt);

    // Feedforward: setpoint change
    float F = 0.0f;
    if (cfg->useFeedforward && c->Kf != 0.0f) {
        const float dSetpoint = setpoint - s->prevSetpoint;
        F = c->Kf * dSetpoint;
    }
    s->prevSetpoint = setpoint;

    // Sum and clamp
    float sum = P + I + D + F;
    if (cfg->pidSumLimit > 0.0f) {
        sum = clampf(sum, -cfg->pidSumLimit, cfg->pidSumLimit);
    }
    return sum;
}

// Compute launch-control setpoint for an axis, mirroring Betaflight semantics in a minimal way.
// - For FULL mode, rate is generated for all axes from stick deflection
// - For PITCHONLY mode, only pitch gets a rate; roll/yaw setpoint becomes 0
// - Pitch is clamped near the angle limit with a taper window
static inline float pidMinApplyLaunchSetpoint(PidMinAxis axis,
                                              const PidMinLaunchConfig *lc,
                                              float rcDeflection,            // [-1..1]
                                              float currentPitchAngleDeg,    // degrees
                                              float trimPitchDeg)            // degrees
{
    if (!lc || !lc->enabled) {
        return 0.0f;
    }

    // Only produce setpoint for pitch in pitch-only mode
    if (lc->mode == PID_MIN_LAUNCH_MODE_PITCHONLY && axis != PID_MIN_AXIS_PITCH) {
        return 0.0f;
    }

    // Stick scaling (Betaflight uses a window to map small deflection to min rate)
    const float stick = clampf(rcDeflection, -0.5f, 0.5f); // use center +/-0.5 for sensitivity
    float rate = LAUNCH_CONTROL_MAX_RATE * (stick * 2.0f); // map +/-0.5 to +/-max

    // Angle limiting on pitch only
    if (axis == PID_MIN_AXIS_PITCH && lc->angleLimitDeg > 0.0f) {
        const float currentAngle = currentPitchAngleDeg - trimPitchDeg;
        if (currentAngle >= lc->angleLimitDeg) {
            rate = 0.0f;
        } else {
            const float angleDelta = lc->angleLimitDeg - currentAngle;
            if (angleDelta <= LAUNCH_CONTROL_ANGLE_WINDOW) {
                // Taper the rate down near the limit
                const float targetRate = rate;
                rate = scaleRangef(angleDelta,
                                   0.0f,
                                   LAUNCH_CONTROL_ANGLE_WINDOW,
                                   (rate >= 0.0f) ? LAUNCH_CONTROL_MIN_RATE : -LAUNCH_CONTROL_MIN_RATE,
                                   targetRate);
            }
        }
    }

    return rate;
}

// Minimal update that includes Launch Control gating for P/I/D/F.
// If launchActive+enabled, replaces setpoint with launch setpoint and applies Ki override,
// disables D and feedforward, limits yaw I in FULL mode, and disables P/I for non-pitch axes in PITCHONLY mode.
float pidMinUpdateWithLaunch(const PidMinCoefficients *c,
                             const PidMinConfig *cfg,
                             const PidMinLaunchConfig *lc,
                             PidMinState *s,
                             PidMinAxis axis,
                             float setpoint,
                             float gyro,
                             float dt,
                             bool launchActive,
                             float rcDeflection,
                             float currentPitchAngleDeg,
                             float trimPitchDeg)
{
    if (!c || !cfg || !s) return 0.0f;
    if (dt <= 0.0f) return 0.0f;

    // Initialize filtered reference on first run
    if (!s->initialized) {
        s->prevGyroFiltered = gyro;
        s->prevSetpoint = setpoint;
        s->prevSetpointLpf = setpoint;
        s->initialized = true;
    }

    // Apply launch setpoint if active and enabled
    if (launchActive && lc && lc->enabled) {
        const float lcSetpoint = pidMinApplyLaunchSetpoint(axis, lc, rcDeflection, currentPitchAngleDeg, trimPitchDeg);
        setpoint = lcSetpoint;
    }

    const float errorRate = setpoint - gyro;
    // I-term relax under launch-aware update
    float iErrorRate = errorRate;
    if (cfg->iRelaxEnabled && cfg->iRelaxType != PID_MIN_ITERM_RELAX_OFF) {
        const float alphaRelax = (cfg->iRelaxCutoffHz > 0.0f) ? pt1Alpha(cfg->iRelaxCutoffHz, dt) : 1.0f;
        const float setpointLpf = s->prevSetpointLpf + alphaRelax * (setpoint - s->prevSetpointLpf);
        const float setpointHpf = fabsf(setpoint - setpointLpf);
        s->prevSetpointLpf = setpointLpf;
        if (cfg->iRelaxType == PID_MIN_ITERM_RELAX_SETPOINT) {
            const float threshold = (cfg->iRelaxSetpointThreshold > 0.0f) ? cfg->iRelaxSetpointThreshold : 40.0f;
            const float relaxFactor = fmaxf(0.0f, 1.0f - setpointHpf / threshold);
            iErrorRate *= relaxFactor;
        } else if (cfg->iRelaxType == PID_MIN_ITERM_RELAX_GYRO) {
            iErrorRate = fapplyDeadband(setpointLpf - gyro, setpointHpf);
        }
    }

    // P-term
    float P = c->Kp * errorRate;

    // Ki override while launch active
    float effectiveKi = c->Ki;
    if (launchActive && lc && lc->enabled && lc->kiOverride > 0.0f) {
        effectiveKi = lc->kiOverride;
    }

    // I-term with leak and clamp
    s->integrator += effectiveKi * iErrorRate * dt;
    if (cfg->integratorLeak > 0.0f) {
        const float leak = clampf(cfg->integratorLeak, 0.0f, 1.0f);
        s->integrator *= (1.0f - leak);
    }

    // Launch-specific yaw I limit in FULL mode, disable yaw I in PITCHONLY
    if (launchActive && lc && lc->enabled) {
        if (axis == PID_MIN_AXIS_YAW) {
            const float yawLimit = (lc->mode == PID_MIN_LAUNCH_MODE_FULL) ? LAUNCH_CONTROL_YAW_ITERM_LIMIT : 0.0f;
            s->integrator = clampf(s->integrator, -yawLimit, yawLimit);
        }
    }
    // Generic I-term limit if set (applies when not restricted by yaw rule above)
    if (!(launchActive && lc && lc->enabled && axis == PID_MIN_AXIS_YAW)) {
        if (cfg->itermLimit > 0.0f) {
            s->integrator = clampf(s->integrator, -cfg->itermLimit, cfg->itermLimit);
        }
    }
    float I = s->integrator;

    // In pitch-only mode, keep pitch I non-negative to avoid back-driving
    if (launchActive && lc && lc->enabled && lc->mode == PID_MIN_LAUNCH_MODE_PITCHONLY && axis == PID_MIN_AXIS_PITCH) {
        if (I < 0.0f) {
            I = 0.0f;
            s->integrator = 0.0f;
        }
    }

    // D-term: disabled under launch active; still update filter state
    const float alpha = (cfg->dLowpassCutoffHz > 0.0f) ? pt1Alpha(cfg->dLowpassCutoffHz, dt) : 1.0f;
    const float gyroFiltered = s->prevGyroFiltered + alpha * (gyro - s->prevGyroFiltered);
    const float dGyro = gyroFiltered - s->prevGyroFiltered;
    s->prevGyroFiltered = gyroFiltered;
    float D = 0.0f;
    if (!(launchActive && lc && lc->enabled)) {
        D = c->Kd * (-(dGyro) / dt);
    }

    // Feedforward: disabled under launch active
    float F = 0.0f;
    if (cfg->useFeedforward && c->Kf != 0.0f && !(launchActive && lc && lc->enabled)) {
        const float dSetpoint = setpoint - s->prevSetpoint;
        F = c->Kf * dSetpoint;
    }
    s->prevSetpoint = setpoint;

    // P/I disabled for non-pitch axes in pitch-only mode
    if (launchActive && lc && lc->enabled && lc->mode == PID_MIN_LAUNCH_MODE_PITCHONLY) {
        if (axis == PID_MIN_AXIS_ROLL || axis == PID_MIN_AXIS_YAW) {
            P = 0.0f;
            I = 0.0f;
        }
    }

    float sum = P + I + D + F;
    if (cfg->pidSumLimit > 0.0f) {
        sum = clampf(sum, -cfg->pidSumLimit, cfg->pidSumLimit);
    }
    return sum;
}

// Unified update: preferred single entry point for both launch and normal phases.
// Use launchActive flag and lc config to control behavior. When launchActive is false
// or lc is NULL/disabled, this behaves like pidMinUpdate.
float pidMinUpdateUnified(const PidMinCoefficients *c,
                          const PidMinConfig *cfg,
                          const PidMinLaunchConfig *lc,
                          PidMinState *s,
                          PidMinAxis axis,
                          float setpoint,
                          float gyro,
                          float dt,
                          bool launchActive,
                          float rcDeflection,
                          float currentPitchAngleDeg,
                          float trimPitchDeg)
{
    return pidMinUpdateWithLaunch(c, cfg, lc, s, axis, setpoint, gyro, dt,
                                  launchActive, rcDeflection, currentPitchAngleDeg, trimPitchDeg);
}

// TypeScript mapping notes (for later conversion)
// interface PidMinCoefficients { Kp: number; Ki: number; Kd: number; Kf: number }
// interface PidMinConfig {
//   pidSumLimit: number;
//   itermLimit: number;
//   integratorLeak: number; // 0..1
//   useFeedforward: boolean;
//   dLowpassCutoffHz: number;
// }
// class PidMinState {
//   integrator = 0; prevGyroFiltered = 0; prevSetpoint = 0; initialized = false;
// }
// enum PidMinAxis { Roll=0, Pitch=1, Yaw=2 }
// enum PidMinLaunchMode { PitchOnly=0, Full=1 }
// interface PidMinLaunchConfig { enabled: boolean; mode: PidMinLaunchMode; angleLimitDeg: number; kiOverride: number }
// function applyLaunchSetpoint(axis: PidMinAxis, lc: PidMinLaunchConfig, rcDeflection: number, currentPitchAngleDeg: number, trimPitchDeg: number): number { /* mirror C logic */ }
// function updateWithLaunch(c: PidMinCoefficients, cfg: PidMinConfig, lc: PidMinLaunchConfig, s: PidMinState, axis: PidMinAxis, setpoint: number, gyro: number, dt: number, launchActive: boolean, rcDeflection: number, currentPitchAngleDeg: number, trimPitchDeg: number): number { /* mirror C logic */ }
// function pt1Alpha(fc: number, dt: number): number { const rc = 1/(2*Math.PI*fc); return dt/(rc+dt) }
// function update(c: PidMinCoefficients, cfg: PidMinConfig, s: PidMinState, setpoint: number, gyro: number, dt: number): number {
//   // Mirror the C logic above; guard dt > 0; keep math NaN-safe.
// }

// Example usage in C (line comments only)
// ------------------------------------------------------------
// PidMinCoefficients c = { .Kp = 0.9f, .Ki = 0.3f, .Kd = 0.02f, .Kf = 0.0f };
// PidMinConfig cfg = {
//   .pidSumLimit = 0.0f,
//   .itermLimit = 100.0f,
//   .integratorLeak = 0.05f,
//   .useFeedforward = false,
//   .dLowpassCutoffHz = 40.0f,
//   // I-term relax (optional)
//   .iRelaxEnabled = true,
//   .iRelaxCutoffHz = 15.0f,
//   .iRelaxSetpointThreshold = 40.0f,
//   .iRelaxType = PID_MIN_ITERM_RELAX_SETPOINT // or PID_MIN_ITERM_RELAX_GYRO, or OFF
// };
// PidMinLaunchConfig lc = { .enabled = true, .mode = PID_MIN_LAUNCH_MODE_PITCHONLY, .angleLimitDeg = 30.0f, .kiOverride = 0.15f };
// PidMinState sRoll, sPitch, sYaw; pidMinInit(&sRoll); pidMinInit(&sPitch); pidMinInit(&sYaw);
// bool launchActive = true;
// float dt = 0.001f;
// float rcDeflection = 0.4f; float pitchDeg = 10.0f; float trimDeg = 0.0f;
// float outRoll = pidMinUpdateUnified(&c, &cfg, &lc, &sRoll, PID_MIN_AXIS_ROLL, 0.0f, 0.0f, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// float outPitch = pidMinUpdateUnified(&c, &cfg, &lc, &sPitch, PID_MIN_AXIS_PITCH, 0.0f, 0.0f, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// float outYaw = pidMinUpdateUnified(&c, &cfg, &lc, &sYaw, PID_MIN_AXIS_YAW, 0.0f, 0.0f, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// // When your simulator decides launch phase is over:
// // launchActive = false; lc.enabled = false;
// // Continue calling pidMinUpdateUnified(...) with launchActive = false (behaves like normal PID)