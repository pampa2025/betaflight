# Betaflight Feedforward: Full Implementation Guide

This tutorial explains Betaflight’s feedforward feature end-to-end, with full source excerpts and context. It covers where feedforward is generated, how its parameters are normalized, and how it’s consumed by the PID controller in both rate (acro) and angle/horizon modes.

## Overview

- Feedforward anticipates stick motion to reduce perceived latency, improving snap and precision.
- Generation happens in `rc.c` from RC setpoints and timing; consumption happens in `pid.c`.
- Parameters are user-facing in `pid.h` and normalized in `pid_init.c` so behavior is consistent across radios/packet rates.

## Key Files and Symbols

- `src/main/fc/rc.c`
  - `updateFeedforwardFilters`, `calculateFeedforward`, `getFeedforward`.
- `src/main/flight/pid_init.c`
  - Feedforward parameter normalization and derived runtime state.
- `src/main/flight/pid.c`
  - Rate-mode feedforward consumption inside `pidController`.
  - Angle-mode feedforward in `pidLevel`.
- `src/main/fc/rc.h`
  - `feedforwardData_t` per-axis state.
- `src/main/flight/pid.h`
  - `feedforwardAveraging_t`, `pidProfile_t` user params, `pidRuntime_t` derived state.

---

## Parameters and Types

The relevant enums and fields are defined in `pid.h`.

```c
// File: src/main/flight/pid.h (excerpt)
typedef enum feedforwardAveraging_e {
    FEEDFORWARD_AVERAGING_OFF,
    FEEDFORWARD_AVERAGING_2_POINT,
    FEEDFORWARD_AVERAGING_3_POINT,
    FEEDFORWARD_AVERAGING_4_POINT,
} feedforwardAveraging_t;

// pidProfile_t includes user-configurable feedforward settings
uint8_t feedforward_transition;         // Feedforward attenuation around centre sticks
uint8_t feedforward_averaging;          // Number of packets to average when averaging is on
uint8_t feedforward_smooth_factor;      // Amount of lowpass type smoothing for feedforward steps
uint8_t feedforward_jitter_factor;      // Number of RC steps below which to attenuate feedforward
uint8_t feedforward_boost;              // amount of setpoint acceleration to add to feedforward, 10 means 100% added
uint8_t feedforward_max_rate_limit;     // Maximum setpoint rate percentage for feedforward
uint8_t feedforward_yaw_hold_gain;      // Amount of sustained high-pass yaw setpoint to add to feedforward, zero disables
uint8_t feedforward_yaw_hold_time ;     // Time constant of the sustained yaw hold element in ms

// pidRuntime_t holds normalized/derived feedforward state
feedforwardAveraging_t feedforwardAveraging;
float feedforwardSmoothFactor;
uint8_t feedforwardJitterFactor;
float feedforwardJitterFactorInv;
float feedforwardBoostFactor;
float feedforwardTransition;
float feedforwardTransitionInv;
uint8_t feedforwardMaxRateLimit;
float feedforwardYawHoldGain;
float feedforwardYawHoldTime;
bool feedforwardInterpolate; // Whether to interpolate an FF value for duplicate/identical data values
pt3Filter_t angleFeedforwardPt3[XYZ_AXIS_COUNT];
```

---

## Feedforward Normalization

Betaflight normalizes feedforward parameters in `pid_init.c` so the perceived behavior is mostly invariant to radio packet rate and other system settings.

```c
// File: src/main/flight/pid_init.c (full normalization block)
#ifdef USE_FEEDFORWARD
    // Feedforward normalization and parameters:
    // - `feedforward_transition`: center-stick attenuation factor [0..1].
    // - Smoothing normalization: scale to 250 Hz RC packet rate so behavior is radio-rate invariant.
    //   Using first-order low-pass tau: tau = (rxDt * s) / (1 - s), where s = 0.01 * smooth_factor.
    // - `jitter_factor`: attenuate FF when consecutive RC steps are small (center stick).
    // - `boost_factor`: small acceleration term added to FF to preempt motor lag.
    // - `yaw_hold`: high-pass sustained yaw component with time constant; gain normalized for short taus.
    pidRuntime.feedforwardTransition = pidProfile->feedforward_transition / 100.0f;
    pidRuntime.feedforwardTransitionInv = (pidProfile->feedforward_transition == 0) ? 0.0f : 100.0f / pidProfile->feedforward_transition;
    pidRuntime.feedforwardAveraging = pidProfile->feedforward_averaging;
    // feedforward_smooth_factor effect previously would change based on packet looprate
    // normalizing to 250hz packet rate as that is the most commonly used ELRS packet rate
    float scaledSmoothFactor = 0.01f * pidProfile->feedforward_smooth_factor; // convert percent to [0..1]
    float rxDt = 1.0f / 250.0f; // reference RC rate (seconds per packet)
    float feedforwardSmoothingTau = (rxDt * scaledSmoothFactor) / (1.0f - scaledSmoothFactor); // first-order LPF tau
    pidRuntime.feedforwardSmoothFactor = feedforwardSmoothingTau;
    pidRuntime.feedforwardJitterFactor = pidProfile->feedforward_jitter_factor;
    pidRuntime.feedforwardJitterFactorInv = 1.0f / (1.0f + pidProfile->feedforward_jitter_factor);
    pidRuntime.feedforwardBoostFactor = 0.001f * pidProfile->feedforward_boost;
    pidRuntime.feedforwardMaxRateLimit = pidProfile->feedforward_max_rate_limit;
    pidRuntime.feedforwardInterpolate = !(rxRuntimeState.serialrxProvider == SERIALRX_CRSF);
    pidRuntime.feedforwardYawHoldTime = 0.001f * pidProfile->feedforward_yaw_hold_time; // ms -> s time constant
    pidRuntime.feedforwardYawHoldGain = pidProfile->feedforward_yaw_hold_gain;
    // Normalize/maintain boost when time constant is small:
    // ~1.5x at 50ms, ~2x at 25ms, ~3x at 10ms to preserve perceived response.
    if (pidProfile->feedforward_yaw_hold_time < 100) {
        pidRuntime.feedforwardYawHoldGain *= 150.0f / (float)(pidProfile->feedforward_yaw_hold_time + 50);
    }
#endif
```

Notes:

- Smoothing normalization translates a percent slider into a first-order delay (tau) at an assumed 250 Hz RC rate.
- `feedforwardInterpolate` disables extrapolation on CRSF (which typically has clean timing), enabling it for other links.
- Yaw hold scales gain when the time constant is short to keep perceived effect stable.

---

## RC Feedforward Generation (rc.c)

Feedforward is generated per axis from RC setpoint deltas and timing. It accounts for duplicate packets, applies smoothing and jitter attenuation, adds boost, and optionally adds a yaw hold.

### updateFeedforwardFilters

```c
// File: src/main/fc/rc.c (full function)
#ifdef USE_FEEDFORWARD
// Feedforward smoothing cutoffs track the smoothed Rx link rate so that
// the perceived feedforward time constants remain stable across packet rates.
// We derive a PT1 gain from an equivalent delay (pid->feedforwardSmoothFactor)
// and update both setpoint speed and setpoint speed delta filters per axis.
static FAST_CODE_NOINLINE void updateFeedforwardFilters(const pidRuntime_t *pid) {
    float pt1K = pt1FilterGainFromDelay(pid->feedforwardSmoothFactor, 1.0f / smoothedRxRateHz);
    for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
        pt1FilterUpdateCutoff(&feedforwardData.filterSetpointSpeed[axis], pt1K);
        pt1FilterUpdateCutoff(&feedforwardData.filterSetpointDelta[axis], pt1K);
    }
    DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 6, lrintf(pt1K * 1000.0f));
    DEBUG_SET(DEBUG_RC_SMOOTHING, 4, lrintf(pt1K * 1000.0f));
    DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 7, lrintf(smoothedRxRateHz));
}
#endif
```

### calculateFeedforward

```c
// File: src/main/fc/rc.c (full function)
#ifdef USE_FEEDFORWARD
/*
 * Feedforward generation for rate mode.
 *
 * Pipeline:
 *  1) Compute setpoint speed = Δsetpoint * Rx rate (per axis). Handle duplicate frames
 *     by extrapolating the first duplicate and zeroing speed thereafter.
 *  2) Smooth setpoint speed (PT1) and its delta (acceleration) to remove stair-steps.
 *  3) Build feedforward as FF = setpointSpeed (+ boost from acceleration on roll/pitch).
 *  4) Apply jitter attenuator: ((|Δrc| + prev|Δrc|)/2 + 1) * jitterFactorInv, clamped ≤ 1,
 *     which preserves large stick moves and suppresses tiny frame jitter.
 *  5) On yaw, add a short “hold” term to counteract motor lag during quick yaw moves.
 *  6) Optionally transition FF near center sticks and apply averaging.
 */
static FAST_CODE_NOINLINE void calculateFeedforward(const pidRuntime_t *pid, flight_dynamics_index_t axis)
{
    const float rxInterval = currentRxIntervalUs * 1e-6f; // seconds
    const float rcCommandDelta = rcCommand[axis] - feedforwardData.prevRcCommand[axis];
    float rcCommandDeltaAbs = fabsf(rcCommandDelta);
    const bool isDuplicate = rcCommandDeltaAbs == 0;
    const float setpoint = rawSetpoint[axis];
    const float setpointDelta = setpoint - feedforwardData.prevSetpoint[axis];

    feedforwardData.prevRcCommand[axis] = rcCommand[axis];
    feedforwardData.prevSetpoint[axis] = setpoint;

    float rxRate = currentRxRateHz;

    float setpointSpeed = 0.0f;
    float setpointSpeedUnsmoothed = 0.0f;
    float setpointSpeedDelta = 0.0f;
    float feedforward = 0.0f;

    if (pid->feedforwardInterpolate) {
        // for Rx links which send frequent duplicate data packets, sometimes on one axis, use a per-axis duplicate test
        float prevRxInterval = feedforwardData.prevRxInterval[axis];
        // extrapolate setpointSpeed when a duplicate is detected, to minimise steps in feedforward
        if (!isDuplicate) {
            // movement!
            // but, if the packet before this was also a duplicate,
            // calculate setpointSpeed over the last two intervals
            if (feedforwardData.isPrevPacketDuplicate[axis]) {
                // Adjust rxRate if previous packet was duplicate
                rxRate = 1.0f / (rxInterval + prevRxInterval);
            }
            setpointSpeed = setpointDelta * rxRate;
        } else {
            // no movement
            if (!feedforwardData.isPrevPacketDuplicate[axis]) {
                // extrapolate a replacement setpointSpeed value for the first duplicate after normal movement
                // but not when about to hit max deflection
                if (fabsf(setpoint) < 0.90f * maxRcRate[axis]) {
                    // this is a single packet duplicate, and we assume that it is of approximately normal duration
                    // hence no multiplication of prevSetpointSpeedDelta by rxInterval / prevRxInterval
                    setpointSpeed = feedforwardData.prevSetpointSpeed[axis] + feedforwardData.prevSetpointSpeedDelta[axis];
                    // pretend that there was stick movement also, to hold the same jitter value
                    rcCommandDeltaAbs = feedforwardData.prevRcCommandDeltaAbs[axis];
                }
            } else {
                // for second and all subsequent duplicates...
                // force setpoint speed to zero
                setpointSpeed = 0.0f;
                // zero the acceleration by setting previous speed to zero
                // feedforward will smoothly decay and be attenuated by the jitter reduction value for zero rcCommandDelta
                feedforwardData.prevSetpointSpeed[axis] = 0.0f;
            }
        }
        feedforwardData.prevRxInterval[axis] = rxInterval;
    } else {
        setpointSpeed = setpointDelta * currentRxRateHz;
    }
    feedforwardData.isPrevPacketDuplicate[axis] = isDuplicate;

    // Jitter attenuation factor calculation
    float jitterAttenuator = ((rcCommandDeltaAbs + feedforwardData.prevRcCommandDeltaAbs[axis]) * 0.5f + 1.0f) * pid->feedforwardJitterFactorInv;
    jitterAttenuator = MIN(jitterAttenuator, 1.0f);
    feedforwardData.prevRcCommandDeltaAbs[axis] = rcCommandDeltaAbs;

    setpointSpeedUnsmoothed = setpointSpeed;

    // Smooth the setpointSpeed value
    setpointSpeed = pt1FilterApply(&feedforwardData.filterSetpointSpeed[axis], setpointSpeed);

    // Calculate setpointDelta from smoothed setpoint speed
    setpointSpeedDelta = setpointSpeed - feedforwardData.prevSetpointSpeed[axis];
    feedforwardData.prevSetpointSpeed[axis] = setpointSpeed;

    // Smooth the setpointDelta (2nd order smoothing)
    setpointSpeedDelta = pt1FilterApply(&feedforwardData.filterSetpointDelta[axis], setpointSpeedDelta);
    feedforwardData.prevSetpointSpeedDelta[axis] = setpointSpeedDelta;

    // Calculate feedforward boost
    const float feedforwardBoost = setpointSpeedDelta * rxRate * pid->feedforwardBoostFactor;
    feedforward = setpointSpeed;

    if (axis == FD_ROLL || axis == FD_PITCH) {
        feedforward += feedforwardBoost;
        feedforward *= jitterAttenuator;
        if (pid->feedforwardMaxRateLimit && feedforward * setpoint > 0.0f) {
            const float limit = (maxRcRate[axis] - fabsf(setpoint)) * pid->feedforwardMaxRateLimit;
            feedforward = (limit > 0.0f) ? constrainf(feedforward, -limit, limit) : 0.0f;
        }
    } else {
        feedforward *= jitterAttenuator;
        const float gain = pt1FilterGainFromDelay(pid->feedforwardYawHoldTime, rxInterval);
        pt1FilterUpdateCutoff(&feedforwardYawHoldLpf, gain);
        const float setpointLpfYaw = pt1FilterApply(&feedforwardYawHoldLpf, setpoint);
        const float feedforwardYawHold = pid->feedforwardYawHoldGain * (setpoint - setpointLpfYaw);
		DEBUG_SET(DEBUG_FEEDFORWARD, 6, lrintf(feedforward * 0.01f));         // basic yaw ff without hold
		DEBUG_SET(DEBUG_FEEDFORWARD, 7, lrintf(feedforwardYawHold * 0.01f));  // with yaw ff hold element
        feedforward += feedforwardYawHold;
    }

    // Apply feedforward transition if configured
    const bool useTransition = (pid->feedforwardTransition != 0.0f) && (rcDeflectionAbs[axis] < pid->feedforwardTransition);
    if (useTransition) {
        feedforward *= rcDeflectionAbs[axis] * pid->feedforwardTransitionInv;
    }

    if (axis == gyro.gyroDebugAxis) {
        DEBUG_SET(DEBUG_FEEDFORWARD, 0, lrintf(setpoint));
        DEBUG_SET(DEBUG_FEEDFORWARD, 1, lrintf(setpointSpeed * 0.01f));
        DEBUG_SET(DEBUG_FEEDFORWARD, 2, lrintf(feedforwardBoost * 0.01f));
        DEBUG_SET(DEBUG_FEEDFORWARD, 3, lrintf(rcCommandDeltaAbs * 10.0f));
        DEBUG_SET(DEBUG_FEEDFORWARD, 4, lrintf(jitterAttenuator * 100.0f));
        DEBUG_SET(DEBUG_FEEDFORWARD, 5, (int16_t)(feedforwardData.isPrevPacketDuplicate[axis]));
        // 6 and 7 used for feedforward yaw hold logging

        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 0, lrintf(jitterAttenuator * 100.0f)); // jitter attenuation factor in percent
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 1, lrintf(maxRcRate[axis]));           // max Setpoint rate (badly named)
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 2, lrintf(setpoint));                  // setpoint used for FF, unsmoothed
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 3, lrintf(feedforward * 0.01f));       // un-smoothed final feedforward
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 4, lrintf(setpointSpeedUnsmoothed * 0.01f));
        DEBUG_SET(DEBUG_FEEDFORWARD_LIMIT, 5, lrintf(setpointSpeed * 0.01f));      // compare to 4 to check ff smoothing
        // 6 for feedforward pt1K, 7 for smoothedRxRateHz
    }

    // Final smoothing if configured
    if (feedforwardAveraging) {
        feedforward = laggedMovingAverageUpdate(&feedforwardDeltaAvg[axis].filter, feedforward);
    }

    feedforwardRaw[axis] = feedforward;
}
#endif // USE_FEEDFORWARD
```

### getFeedforward

```c
// File: src/main/fc/rc.c (full function)
float getFeedforward(int axis)
{
#ifdef USE_RC_SMOOTHING_FILTER
    return rxConfig()->rc_smoothing ? feedforwardSmoothed[axis] : feedforwardRaw[axis];
#else
    return feedforwardRaw[axis];
#endif
}
```

### Per-Axis State

```c
// File: src/main/fc/rc.h (full struct)
typedef struct feedforwardData_s {
    float prevRcCommand[XYZ_AXIS_COUNT];
    float prevRcCommandDeltaAbs[XYZ_AXIS_COUNT];
    float prevSetpoint[XYZ_AXIS_COUNT];
    float prevSetpointSpeed[XYZ_AXIS_COUNT];
    float prevSetpointSpeedDelta[XYZ_AXIS_COUNT];
    bool isPrevPacketDuplicate[XYZ_AXIS_COUNT];
    float prevRxInterval[XYZ_AXIS_COUNT];
    pt1Filter_t filterSetpointSpeed[XYZ_AXIS_COUNT];
    pt1Filter_t filterSetpointDelta[XYZ_AXIS_COUNT];
} feedforwardData_t;
```

---

## PID Consumption (pid.c)

Feedforward is consumed in two places:
- In acro/rate mode inside `pidController` where `pidData[axis].F = Kf * pidSetpointDelta`.
- In Angle/Horizon via `pidLevel` where `angleFeedforward` is built from RC feedforward and filtered.

### Rate Mode Consumption Snippet

Below is the complete section that sources feedforward and applies it to `F`. Angle axes suppress PID feedforward to avoid reintroducing RC step artifacts; acro axes use `getFeedforward(axis)`.

```c
// File: src/main/flight/pid.c (excerpt from pidController)
        // -----feedforward source (setpoint delta)
        // In acro/rate mode, feedforward comes from rc.c (`getFeedforward`) which is pre-smoothed, center-attenuated,
        // and jitter-managed. In Angle mode the setpoint already embeds stick-based FF via `pidLevel`, so PID FF is
        // forced to zero for now to avoid reintroducing RC steps.
        float pidSetpointDelta = 0;

#if defined(USE_FEEDFORWARD) && defined(USE_ACC)
        if (FLIGHT_MODE(ANGLE_MODE) && pidRuntime.axisInAngleMode[axis]) {
            // this axis is fully under self-levelling control
            // it will already have stick based feedforward applied in the input to their angle setpoint
            // a simple setpoint Delta can be used to for PID feedforward element for motor lag on these axes
            // however RC steps come in, via angle setpoint
            // and setpoint RC smoothing must have a cutoff half normal to remove those steps completely
            // the RC stepping does not come in via the feedforward, which is very well smoothed already
            // if uncommented, and the forcing to zero is removed, the two following lines will restore PID feedforward to angle mode axes
            // but for now let's see how we go without it (which was the case before 4.5 anyway)
//            pidSetpointDelta = currentPidSetpoint - pidRuntime.previousPidSetpoint[axis];
//            pidSetpointDelta *= pidRuntime.pidFrequency * pidRuntime.angleFeedforwardGain;
            pidSetpointDelta = 0.0f;
        } else {
            // the axis is operating as a normal acro axis, so use normal feedforard from rc.c
            pidSetpointDelta = getFeedforward(axis);
        }
#endif
        pidRuntime.previousPidSetpoint[axis] = currentPidSetpoint; // this is the value sent to blackbox, and used for D-max setpoint

        // -----calculate feedforward component
        // F multiplies the setpoint delta by Kf. During Launch Control `Kf` is set to zero to keep attitude consistent.
        const float feedforwardGain = launchControlActive ? 0.0f : pidRuntime.pidCoefficient[axis].Kf;
        pidData[axis].F = feedforwardGain * pidSetpointDelta;
```

### Angle Mode Feedforward (pidLevel)

Angle feedforward uses the RC feedforward normalized by the current Acro rates and heavily filters it before combining with the angle error rate. It is explicitly zeroed in `POS_HOLD_MODE` when autopilot is in control to prevent stick lag.

```c
// File: src/main/flight/pid.c (full pidLevel function)
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(int axis, const pidProfile_t *pidProfile, const rollAndPitchTrims_t *angleTrim,
                                                        float currentPidSetpoint, float horizonLevelStrength)
{
    // Applies only to axes that are in Angle mode
    // We now use Acro Rates, transformed into the range +/- 1, to provide setpoints
    float angleLimit = pidProfile->angle_limit;
    float angleFeedforward = 0.0f;
    // if user changes rates profile, update the max setpoint for angle mode
    const float maxSetpointRateInv = 1.0f / getMaxRcRate(axis);

#ifdef USE_FEEDFORWARD
    angleFeedforward = angleLimit * getFeedforward(axis) * pidRuntime.angleFeedforwardGain * maxSetpointRateInv;
    //  angle feedforward must be heavily filtered, at the PID loop rate, with limited user control over time constant
    // it MUST be very delayed to avoid early overshoot and being too aggressive
    angleFeedforward = pt3FilterApply(&pidRuntime.angleFeedforwardPt3[axis], angleFeedforward);
#endif

    float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;
    // use acro rates for the angle target in both horizon and angle modes, converted to -1 to +1 range using maxRate

#ifdef USE_WING
    if (axis == FD_PITCH) {
        angleTarget += (float)pidProfile->angle_pitch_offset / 10.0f;
    }
#endif // USE_WING

#ifdef USE_GPS_RESCUE
    angleTarget += gpsRescueAngle[axis] / 100.0f; // Angle is in centidegrees, stepped on roll at 10Hz but not on pitch
#endif
#if defined(USE_POSITION_HOLD) && !defined(USE_WING)
    if (FLIGHT_MODE(POS_HOLD_MODE)) {
        angleFeedforward = 0.0f; // otherwise the lag of the PT3 carries recent stick inputs into the hold
        if (isAutopilotInControl()) {
            // sticks are not deflected
            angleTarget = autopilotAngle[axis]; // autopilotAngle in degrees
            angleLimit = 85.0f; // allow autopilot to use whatever angle it needs to stop
        }
        // limit pilot requested angle to half the autopilot angle to avoid excess speed and chaotic stops
        angleLimit = fminf(0.5f * autopilotConfig()->maxAngle, angleLimit);
    }
#endif

    angleTarget = constrainf(angleTarget, -angleLimit, angleLimit);

    const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f; // stepped at 500hz with some 4ms flat spots
    const float errorAngle = angleTarget - currentAngle;
    float angleRate = errorAngle * pidRuntime.angleGain + angleFeedforward;

    // minimise cross-axis wobble due to faster yaw responses than roll or pitch, and make co-ordinated yaw turns
    // by compensating for the effect of yaw on roll while pitched, and on pitch while rolled
    // earthRef code here takes about 76 cycles, if conditional on angleEarthRef it takes about 100.  sin_approx costs most of those cycles.
    float sinAngle = sin_approx(DEGREES_TO_RADIANS(pidRuntime.angleTarget[axis == FD_ROLL ? FD_PITCH : FD_ROLL]));
    sinAngle *= (axis == FD_ROLL) ? -1.0f : 1.0f; // must be negative for Roll
    const float earthRefGain = FLIGHT_MODE(GPS_RESCUE_MODE | ALT_HOLD_MODE) ? 1.0f : pidRuntime.angleEarthRef;
    angleRate += pidRuntime.angleYawSetpoint * sinAngle * earthRefGain;
    pidRuntime.angleTarget[axis] = angleTarget;  // set target for alternate axis to current axis, for use in preceding calculation

    // smooth final angle rate output to clean up attitude signal steps (500hz), GPS steps (10 or 100hz), RC steps etc
    // this filter runs at ATTITUDE_CUTOFF_HZ, currently 50hz, so GPS roll may be a bit steppy
    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    if (FLIGHT_MODE(ANGLE_MODE| GPS_RESCUE_MODE | POS_HOLD_MODE)) {
        currentPidSetpoint = angleRate;
    } else {
        // can only be HORIZON mode - crossfade Angle rate and Acro rate
        currentPidSetpoint = currentPidSetpoint * (1.0f - horizonLevelStrength) + angleRate * horizonLevelStrength;
    }

    //logging
    if (axis == FD_ROLL) {
        DEBUG_SET(DEBUG_ANGLE_MODE, 0, lrintf(angleTarget * 10.0f)); // target angle
        DEBUG_SET(DEBUG_ANGLE_MODE, 1, lrintf(errorAngle * pidRuntime.angleGain * 10.0f)); // un-smoothed error correction in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 2, lrintf(angleFeedforward * 10.0f)); // feedforward amount in degrees
        DEBUG_SET(DEBUG_ANGLE_MODE, 3, lrintf(currentAngle * 10.0f)); // angle returned

        DEBUG_SET(DEBUG_ANGLE_TARGET, 0, lrintf(angleTarget * 10.0f));
        DEBUG_SET(DEBUG_ANGLE_TARGET, 1, lrintf(sinAngle * 10.0f)); // modification factor from earthRef
        // debug ANGLE_TARGET 2 is yaw attenuation
        DEBUG_SET(DEBUG_ANGLE_TARGET, 3, lrintf(currentAngle * 10.0f)); // angle returned
    }

    DEBUG_SET(DEBUG_CURRENT_ANGLE, axis, lrintf(currentAngle * 10.0f)); // current angle
    return currentPidSetpoint;
}
```

---

## How It Fits Together

- RC path computes `setpoint`, `setpointSpeed`, and `setpointSpeedDelta` per axis and derives `feedforwardRaw[axis]` via smoothing, jitter attenuation, boost, optional yaw hold, and center-stick transition.
- PID path consumes feedforward:
  - In acro: `pidSetpointDelta = getFeedforward(axis)` and `F = Kf * pidSetpointDelta`.
  - In angle/horizon: `angleFeedforward = angleLimit * getFeedforward(axis) * angleFeedforwardGain / maxRate`, heavily filtered and added to the angle error rate.
- Normalization in `pid_init.c` keeps smoothing and yaw hold consistent across radio packet rates.

## Tuning Tips

- `feedforward_smooth_factor`: higher values increase smoothing delay; start ~60–70 for typical ELRS.
- `feedforward_boost`: adds acceleration; small values (10–20) reduce lag in roll/pitch.
- `feedforward_jitter_factor`: suppresses micro-jitter around center; small single digits are common.
- `feedforward_max_rate_limit`: caps FF near stick extremes to avoid overshoot.
- `feedforward_yaw_hold_*`: improves yaw snap; try `gain=15` and `time=100ms` on 5"; reduce time for smaller props.
- `feedforward_transition`: attenuates feedforward near center sticks; useful if hover feels twitchy.

## References

- `src/main/fc/rc.c`: feedforward generation and filters
- `src/main/flight/pid_init.c`: feedforward normalization
- `src/main/flight/pid.c`: consumption in rate and angle modes
- `src/main/fc/rc.h`: per-axis feedforward state
- `src/main/flight/pid.h`: enums and profiles/runtime fields