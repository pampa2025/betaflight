// Pure-function Betaflight rate-mode feedforward (TypeScript)
//
// Ports `rc.c` feedforward generation in a functional style:
// - No classes; per-axis state is a plain object you pass in and get back.
// - Single update function computes the feedforward value and next state.
// - Angle/Horizon are ignored; yaw-only hold term is included for yaw axis.
//
// Usage sketch:
//   import { PidMinAxis } from './pid_min_launch_control';
//   const rt = pidMinFeedforwardNormalize({ ... }, 'CRSF');
//   let sRoll = makePidMinFeedforwardState(rt);
//   const { value, state } = pidMinFeedforwardUpdate(
//     sRoll, rt, PidMinAxis.Roll,
//     setpoint, rcCmd, rxRateHz, rxIntervalUs, maxRcRateRoll
//   );
//   sRoll = state; // next state

import { PidMinAxis } from './pid_min_launch_control';

// Helpers
function clampf(x: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, x));
}

// PT1 gain from equivalent delay (tau) and dt
function pt1GainFromDelay(delaySec: number, dtSec: number): number {
  if (delaySec <= 0) return 1; // no delay => immediate
  return dtSec / (delaySec + dtSec);
}

function pt1Update(prevY: number, x: number, k: number): number {
  return prevY + k * (x - prevY);
}

// User profile and runtime params
export interface FeedforwardProfile {
  feedforward_transition: number;          // percent (0..100)
  feedforward_averaging: number;           // enum index (0..3)
  feedforward_smooth_factor: number;       // percent (0..100)
  feedforward_jitter_factor: number;       // small integer
  feedforward_boost: number;               // integer, 10 => 0.010
  feedforward_max_rate_limit: number;      // scalar; 0 disables
  feedforward_yaw_hold_gain: number;       // gain
  feedforward_yaw_hold_time: number;       // ms
}

export interface PidMinFeedforwardRuntime {
  feedforwardTransition: number;           // fraction 0..1
  feedforwardTransitionInv: number;        // inverse if nonzero
  feedforwardAveraging: number;            // enum index (0..3)
  feedforwardSmoothFactor: number;         // tau in seconds
  feedforwardJitterFactor: number;
  feedforwardJitterFactorInv: number;      // 1/(1+jitter)
  feedforwardBoostFactor: number;          // scale applied to accel term
  feedforwardMaxRateLimit: number;         // scalar limit factor
  feedforwardYawHoldGain: number;          // gain for yaw high-pass component
  feedforwardYawHoldTime: number;          // tau seconds for yaw LPF
  feedforwardInterpolate: boolean;         // duplicate packet interpolation enabled
}

export function pidMinFeedforwardNormalize(profile: FeedforwardProfile, rxProvider: 'CRSF' | string): PidMinFeedforwardRuntime {
  const transitionPct = clampf(profile.feedforward_transition, 0, 100);
  const transitionFrac = transitionPct / 100;
  const smoothFrac = clampf(profile.feedforward_smooth_factor, 0, 100) * 0.01; // 0..1
  const rxDt = 1 / 250; // normalize at 250 Hz
  const feedforwardSmoothingTau = smoothFrac > 0 ? (rxDt * smoothFrac) / (1 - smoothFrac) : 0; // sec

  let yawHoldGain = profile.feedforward_yaw_hold_gain;
  const yawHoldMs = profile.feedforward_yaw_hold_time;
  if (yawHoldMs < 100) {
    yawHoldGain *= 150 / (yawHoldMs + 50);
  }
  return {
    feedforwardTransition: transitionFrac,
    feedforwardTransitionInv: transitionPct === 0 ? 0 : 100 / transitionPct,
    feedforwardAveraging: profile.feedforward_averaging,
    feedforwardSmoothFactor: feedforwardSmoothingTau,
    feedforwardJitterFactor: profile.feedforward_jitter_factor,
    feedforwardJitterFactorInv: 1 / (1 + profile.feedforward_jitter_factor),
    feedforwardBoostFactor: profile.feedforward_boost * 0.001,
    feedforwardMaxRateLimit: profile.feedforward_max_rate_limit,
    feedforwardYawHoldGain: yawHoldGain,
    feedforwardYawHoldTime: profile.feedforward_yaw_hold_time * 0.001, // s
    feedforwardInterpolate: !(rxProvider === 'CRSF'),
  };
}

// Per-axis feedforward state (plain data)
export interface PidMinFeedforwardState {
  prevRcCommand: number;
  prevRcCommandDeltaAbs: number;
  prevSetpoint: number;
  prevSetpointSpeed: number;
  prevSetpointSpeedDelta: number;
  isPrevPacketDuplicate: boolean;
  prevRxInterval: number;
  setpointSpeedFiltered: number;
  setpointDeltaFiltered: number;
  yawSetpointLpf: number; // for yaw hold high-pass
  avgSize: number; // 0,2,3,4
  avgBuf: number[];
  avgIdx: number;
  avgCount: number;
  avgSum: number;
  feedforwardRaw: number;
  initialized: boolean;
}

export function makePidMinFeedforwardState(rt: Readonly<PidMinFeedforwardRuntime>): PidMinFeedforwardState {
  const enumToWindow = [0, 2, 3, 4];
  const win = enumToWindow[clampf(rt.feedforwardAveraging, 0, 3)];
  const buf = win > 0 ? new Array(win).fill(0) : [];
  return {
    prevRcCommand: 0,
    prevRcCommandDeltaAbs: 0,
    prevSetpoint: 0,
    prevSetpointSpeed: 0,
    prevSetpointSpeedDelta: 0,
    isPrevPacketDuplicate: false,
    prevRxInterval: 0,
    setpointSpeedFiltered: 0,
    setpointDeltaFiltered: 0,
    yawSetpointLpf: 0,
    avgSize: win,
    avgBuf: buf,
    avgIdx: 0,
    avgCount: 0,
    avgSum: 0,
    feedforwardRaw: 0,
    initialized: false,
  };
}

// Pure update: one axis
export function pidMinFeedforwardUpdate(
  prev: Readonly<PidMinFeedforwardState>,
  rt: Readonly<PidMinFeedforwardRuntime>,
  axis: PidMinAxis,
  setpoint: number,
  rcCmd: number,
  currentRxRateHz: number,
  currentRxIntervalUs: number,
  maxRcRateAxis: number
): { value: number; state: PidMinFeedforwardState } {
  const rxInterval = currentRxIntervalUs * 1e-6; // seconds
  const rcCommandDelta = rcCmd - prev.prevRcCommand;
  let rcCommandDeltaAbs = Math.abs(rcCommandDelta);
  const isDuplicate = rcCommandDeltaAbs === 0;
  const setpointDelta = setpoint - prev.prevSetpoint;

  let rxRate = currentRxRateHz;
  let setpointSpeed = 0;
  let feedforward = 0;

  if (rt.feedforwardInterpolate) {
    const prevRxInterval = prev.prevRxInterval;
    if (!isDuplicate) {
      if (prev.isPrevPacketDuplicate) {
        rxRate = 1 / (rxInterval + prevRxInterval);
      }
      setpointSpeed = setpointDelta * rxRate;
    } else {
      if (!prev.isPrevPacketDuplicate) {
        if (Math.abs(setpoint) < 0.90 * maxRcRateAxis) {
          setpointSpeed = prev.prevSetpointSpeed + prev.prevSetpointSpeedDelta;
          rcCommandDeltaAbs = prev.prevRcCommandDeltaAbs;
        }
      } else {
        setpointSpeed = 0;
        // emulate immediate zeroing of prev speed for delta calculation
      }
    }
  } else {
    setpointSpeed = setpointDelta * currentRxRateHz;
  }

  // Jitter attenuation
  let jitterAttenuator = ((rcCommandDeltaAbs + prev.prevRcCommandDeltaAbs) * 0.5 + 1.0) * rt.feedforwardJitterFactorInv;
  jitterAttenuator = Math.min(jitterAttenuator, 1.0);

  // Smoothing of setpoint speed and its delta
  const dt = 1 / Math.max(1e-6, currentRxRateHz);
  const k = pt1GainFromDelay(rt.feedforwardSmoothFactor, dt);
  const setpointSpeedFiltered = pt1Update(prev.setpointSpeedFiltered, setpointSpeed, k);
  const prevSpeedForDelta = isDuplicate && prev.isPrevPacketDuplicate ? 0 : prev.prevSetpointSpeed;
  const setpointSpeedDeltaRaw = setpointSpeedFiltered - prevSpeedForDelta;
  const setpointDeltaFiltered = pt1Update(prev.setpointDeltaFiltered, setpointSpeedDeltaRaw, k);

  const feedforwardBoost = setpointDeltaFiltered * rxRate * rt.feedforwardBoostFactor;
  feedforward = setpointSpeedFiltered;

  if (axis === PidMinAxis.Roll || axis === PidMinAxis.Pitch) {
    feedforward += feedforwardBoost;
    feedforward *= jitterAttenuator;
    if (rt.feedforwardMaxRateLimit !== 0 && feedforward * setpoint > 0) {
      const limit = (maxRcRateAxis - Math.abs(setpoint)) * rt.feedforwardMaxRateLimit;
      feedforward = limit > 0 ? clampf(feedforward, -limit, limit) : 0;
    }
  } else {
    // Yaw: jitter + hold term
    feedforward *= jitterAttenuator;
    const gain = pt1GainFromDelay(rt.feedforwardYawHoldTime, rxInterval);
    const yawSetpointLpfNext = pt1Update(prev.yawSetpointLpf, setpoint, gain);
    const feedforwardYawHold = rt.feedforwardYawHoldGain * (setpoint - yawSetpointLpfNext);
    feedforward += feedforwardYawHold;
  }

  // Centre transition attenuation
  const rcDeflectionAbs = maxRcRateAxis > 0 ? Math.abs(setpoint) / maxRcRateAxis : 0;
  const useTransition = rt.feedforwardTransition !== 0 && rcDeflectionAbs < rt.feedforwardTransition;
  if (useTransition) {
    feedforward *= rcDeflectionAbs * rt.feedforwardTransitionInv;
  }

  // Optional packet averaging
  let avgSum = prev.avgSum;
  let avgIdx = prev.avgIdx;
  let avgCount = prev.avgCount;
  let avgBuf = prev.avgBuf.slice();
  if (prev.avgSize > 0) {
    avgSum -= avgBuf[avgIdx] || 0;
    avgBuf[avgIdx] = feedforward;
    avgSum += feedforward;
    avgIdx = (avgIdx + 1) % prev.avgSize;
    avgCount = Math.min(avgCount + 1, prev.avgSize);
    const avg = avgSum / avgCount;
    feedforward = avg;
  }

  const next: PidMinFeedforwardState = {
    prevRcCommand: rcCmd,
    prevRcCommandDeltaAbs: rcCommandDeltaAbs,
    prevSetpoint: setpoint,
    prevSetpointSpeed: setpointSpeedFiltered,
    prevSetpointSpeedDelta: setpointDeltaFiltered,
    isPrevPacketDuplicate: isDuplicate,
    prevRxInterval: rxInterval,
    setpointSpeedFiltered,
    setpointDeltaFiltered,
    yawSetpointLpf: axis === PidMinAxis.Yaw ? pt1Update(prev.yawSetpointLpf, setpoint, pt1GainFromDelay(rt.feedforwardYawHoldTime, rxInterval)) : prev.yawSetpointLpf,
    avgSize: prev.avgSize,
    avgBuf,
    avgIdx,
    avgCount,
    avgSum,
    feedforwardRaw: feedforward,
    initialized: true,
  };

  return { value: feedforward, state: next };
}

// ---- Multi-axis convenience wrapper ----
export type Axis3<T> = { roll: T; pitch: T; yaw: T };

export function makePidMinFeedforwardMultiState(rt: Readonly<PidMinFeedforwardRuntime>): Axis3<PidMinFeedforwardState> {
  return {
    roll: makePidMinFeedforwardState(rt),
    pitch: makePidMinFeedforwardState(rt),
    yaw: makePidMinFeedforwardState(rt),
  };
}

export function pidMinFeedforwardUpdateAll(
  prev: Axis3<PidMinFeedforwardState>,
  rt: Readonly<PidMinFeedforwardRuntime>,
  setpoint: Axis3<number>,
  rcCmd: Axis3<number>,
  rxRateHz: number,
  rxIntervalUs: number,
  maxRcRate: Axis3<number>
): { values: Axis3<number>; state: Axis3<PidMinFeedforwardState> } {
  const roll = pidMinFeedforwardUpdate(prev.roll, rt, PidMinAxis.Roll, setpoint.roll, rcCmd.roll, rxRateHz, rxIntervalUs, maxRcRate.roll);
  const pitch = pidMinFeedforwardUpdate(prev.pitch, rt, PidMinAxis.Pitch, setpoint.pitch, rcCmd.pitch, rxRateHz, rxIntervalUs, maxRcRate.pitch);
  const yaw = pidMinFeedforwardUpdate(prev.yaw, rt, PidMinAxis.Yaw, setpoint.yaw, rcCmd.yaw, rxRateHz, rxIntervalUs, maxRcRate.yaw);
  return {
    values: { roll: roll.value, pitch: pitch.value, yaw: yaw.value },
    state: { roll: roll.state, pitch: pitch.state, yaw: yaw.state },
  };
}