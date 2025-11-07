// Pure-function feedforward optimized for 60 Hz joystick input (Web FPV)
//
// Scenario:
// - Input source: Web Gamepad API delivering pitch/roll/yaw in [-1, 1]
// - Input rate ~60 Hz, control loop can be faster (e.g., 200–1000 Hz)
//
// Memo: Converting Betaflight RC Feedforward to Web Joystick
// -----------------------------------------------------------------------------
// Context
// - Betaflight’s RC feedforward pipeline is designed for high-rate, potentially
//   irregular radio inputs (200–1000 Hz), with transport noise, duplicate frames,
//   and protocol quantization. It builds a derivative-like F-term from setpoint
//   changes while attenuating jitter and dealing with RX artifacts.
// - Web simulators receive local joystick axes as smooth floats in [-1, 1] at
//   ~60 Hz via the Gamepad API. Timing is more regular; no RF artifacts or
//   protocol decoding noise. The main challenge is bridging sparse samples into
//   a faster control loop while preserving responsive stick feel.
//
// Key Differences vs RC
// - No jitter attenuation needed: browser inputs are local and stable; extra
//   jitter filters would suppress intentional user motion and add latency.
// - Sample rate mismatch: 60 Hz inputs feeding a control loop at 200–1000 Hz.
//   We use exponential smoothing to interpolate between samples.
// - Domain: joystick is normalized [-1, 1]; we explicitly map to target angular
//   rates (deg/s) per axis via maxRateDegS, rather than working with RC rates.
//
// Pipeline (Web)
// 1) Clamp joystick to [-1, 1].
// 2) Apply deadband and cubic expo for center shaping (optional).
// 3) Exponential smoothing towards shaped command using tau (ms) to bridge
//    sparse samples over the faster loop (dt).
// 4) Compute derivative of the smoothed command (units 1/s), then apply a PT1
//    filter to the derivative (anti-noise, preserves intent).
// 5) Map smoothed command to targetRateDegS via per-axis maxRateDegS.
// 6) Feedforward value = derivativeGain * dCmdLpf * maxRateDegS.
//    - Units: dCmdLpf (1/s) * maxRateDegS (deg/s) → deg/s; derivativeGain is
//      dimensionless scaling.
// 7) Return both F-term (deg/s) and targetRateDegS for easy PID integration.
//
// Why No RC Jitter Attenuation Here
// - RC jitter attenuation targets RX timing/transport artifacts (variable frame
//   spacing, duplicates, quantization). On web joystick, the “jitter” you see
//   is primarily the 60 Hz stepping, which is intentional user input cadence.
// - The chosen smoothing + derivative PT1 already addresses the only relevant
//   discontinuities while keeping crisp response.
//
// Integration with pid_min.ts
// - Set cfg.feedforwardMode = 'web' and provide webFeedforwardRuntime.
// - pidMinUpdateUnified will call this module and override setpoint with the
//   returned targetRateDegS before applying Launch Control. If cfg.useFeedforward
//   is true and Launch doesn’t disable F, the returned F-term is summed.
// - Pass webCtx per axis: { joystick: axisValue }.
//
// Tuning Guidance
// - smoothingTauMs: 20–40 ms typical. Higher → smoother, lower → snappier.
// - derivativeCutoffHz: 20–40 Hz typical. Higher preserves more snap.
// - derivativeGain: start at 1.0; scale up/down to taste.
// - deadband: 0.01–0.03 to reduce micro-deflections; expo: 0.1–0.3 for finer
//   control near center while retaining authority.
// - maxRateDegS: typical agile sim values: roll/pitch 600, yaw 400.
//
// Edge Cases & Practices
// - dt must be the control loop dt (seconds). We guard division with max(dt, 1e-6).
// - If joystick input is unavailable, pass 0 (neutral) instead of NaN.
// - Pauses or frame drops: the smoother will hold the last command and decay
//   derivative; you can raise tau slightly if resume feels abrupt.
// - Axis-specific shaping: extend runtime to per-axis deadband/expo if needed.
//
// Extensibility
// - Optional light averaging on derivative (small ring buffer) if your runtime
//   environment exhibits irregular timing (remote streaming, GC pauses).
// - Adaptive smoothing based on measured input intervals.
// - Per-axis parameters for finer tailoring.
// - If you need a “raw” feel, set smoothingTauMs = 0 and derivativeCutoffHz = 0
//   (this approximates a discrete-derivative feedforward with minimal latency).
//
// Validation Tips
// - Log joystick, cmdSmooth, dCmd, dCmdLpf, targetRateDegS, and F-term.
// - Perform step tests and ramps; check that F spikes on steps are proportional
//   and decay smoothly, and that targetRate tracks the shaped command.
// - Ensure overall PID loop remains stable across your controlRateHz.
//
// Design goals:
// - Smooth setpoint transitions between sparse 60 Hz samples
// - Derivative-based feedforward computed on the smoothed signal
// - Optional deadband and expo to shape input
// - Rate mapping per-axis from [-1, 1] → [−maxRateDegS, +maxRateDegS]
// - Pure-functional state transitions (no mutation)
//
// What this is NOT:
// - A drop-in replacement for Betaflight RC feedforward; it’s tailored to web joystick
// - It avoids jitter attenuation and packet-duplicate handling used for RC receivers
//
// Usage:
// const rt = normalizePidMinWebFeedforwardRuntime({ inputRateHz: 60, controlRateHz: 500, ... });
// let s = makePidMinWebFeedforwardState();
// // Each control tick (dt = 1/controlRateHz):
// const { value: F, state: sNext } = pidMinWebFeedforwardUpdate(s, rt, 'roll', joy.roll /* -1..1 */, dt);
// s = sNext;
// // F is an additive term you can sum into your axis PID output.

export type AxisName = 'roll' | 'pitch' | 'yaw';
export type Axis3<T> = { roll: T; pitch: T; yaw: T };

// ---- Runtime configuration ---------------------------------------------------
export interface PidMinWebFeedforwardRuntime {
  // Input and control loop rates
  inputRateHz: number; // e.g., 60 for Gamepad API
  controlRateHz: number; // your sim loop rate (e.g., 200..1000)

  // Input shaping
  deadband: number; // 0..1 deadband around center
  expo: number; // 0..1 cubic expo amount

  // Smoothing for sparse samples
  smoothingTauMs: number; // time constant (ms) for exponential smoothing of input

  // Derivative-based feedforward
  derivativeGain: number; // scales d(smoothedCmd)/dt into feedforward
  derivativeCutoffHz: number; // PT1 cutoff to filter derivative (anti-noise)

  // Rate mapping per axis from [-1, 1] → deg/s (peak target rate)
  maxRateDegS: Axis3<number>;
}

export function normalizePidMinWebFeedforwardRuntime(
  inRt: Partial<PidMinWebFeedforwardRuntime>
): PidMinWebFeedforwardRuntime {
  const inputRateHz = clampf(inRt.inputRateHz ?? 60, 1, 1000);
  const controlRateHz = clampf(inRt.controlRateHz ?? 500, 50, 4000);
  const deadband = clampf(inRt.deadband ?? 0.02, 0, 0.2);
  const expo = clampf(inRt.expo ?? 0.2, 0, 1);
  const smoothingTauMs = clampf(inRt.smoothingTauMs ?? 25, 1, 500); // ~25ms good for 60Hz
  const derivativeGain = inRt.derivativeGain ?? 1.0; // dimensionless scaling
  const derivativeCutoffHz = clampf(inRt.derivativeCutoffHz ?? 30, 0, 200); // PT1 on derivative
  const mr = inRt.maxRateDegS ?? { roll: 600, pitch: 600, yaw: 400 };

  return {
    inputRateHz,
    controlRateHz,
    deadband,
    expo,
    smoothingTauMs,
    derivativeGain,
    derivativeCutoffHz,
    maxRateDegS: {
      roll: clampf(mr.roll, 10, 2000),
      pitch: clampf(mr.pitch, 10, 2000),
      yaw: clampf(mr.yaw, 10, 2000),
    },
  };
}

// ---- State ------------------------------------------------------------------
export interface PidMinWebFeedforwardState {
  // Smoothed command in [-1, 1]
  cmdSmooth: number;
  // Last smoothed command (for derivative)
  prevCmdSmooth: number;
  // Lowpassed derivative (deg/s per unit cmd), dimension depends on mapping step
  dCmdLpf: number;
  // Initialized flag
  initialized: boolean;
}

export function makePidMinWebFeedforwardState(): PidMinWebFeedforwardState {
  return {
    cmdSmooth: 0,
    prevCmdSmooth: 0,
    dCmdLpf: 0,
    initialized: false,
  };
}

// ---- Helpers ----------------------------------------------------------------
function clampf(x: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, x));
}

function applyDeadband(x: number, db: number): number {
  if (db <= 0) return x;
  const m = Math.abs(x);
  if (m <= db) return 0;
  // re-scale to keep full-scale reachable after deadband removal
  const s = Math.sign(x);
  return s * clampf((m - db) / (1 - db), 0, 1);
}

function applyExpoCubic(x: number, expo: number): number {
  if (expo <= 0) return x;
  return (1 - expo) * x + expo * x * x * x;
}

function pt1Alpha(cutoffHz: number, dt: number): number {
  if (cutoffHz <= 0) return 1; // no filtering
  const rc = 1 / (2 * Math.PI * cutoffHz);
  return dt / (dt + rc);
}

function expSmoothingAlpha(tauMs: number, dt: number): number {
  const tau = Math.max(1e-6, tauMs / 1000);
  return 1 - Math.exp(-dt / tau);
}

// ---- Update -----------------------------------------------------------------
// Returns feedforward value (deg/s scaled) and next state
export function pidMinWebFeedforwardUpdate(
  prev: Readonly<PidMinWebFeedforwardState>,
  rt: Readonly<PidMinWebFeedforwardRuntime>,
  axis: AxisName,
  joystickCmd: number, // normalized [-1, 1]
  dt: number // control loop dt in seconds
): { value: number; state: PidMinWebFeedforwardState; targetRateDegS: number } {
  const bounded = clampf(joystickCmd, -1, 1);
  // Input shaping
  const deads = applyDeadband(bounded, rt.deadband);
  const shaped = applyExpoCubic(deads, rt.expo);

  // Smooth to bridge sparse samples; exponential smoothing towards shaped
  const aSmooth = expSmoothingAlpha(rt.smoothingTauMs, dt);
  const cmdSmoothNext = prev.initialized ? prev.cmdSmooth + aSmooth * (shaped - prev.cmdSmooth) : shaped;

  // Derivative of smoothed command
  const dCmd = prev.initialized ? (cmdSmoothNext - prev.cmdSmooth) / Math.max(dt, 1e-6) : 0;
  const aDeriv = pt1Alpha(rt.derivativeCutoffHz, dt);
  const dCmdLpfNext = prev.initialized ? prev.dCmdLpf + aDeriv * (dCmd - prev.dCmdLpf) : 0;

  // Map smoothed command to target angular rate (deg/s)
  const maxRate = rt.maxRateDegS[axis];
  const targetRateDegS = cmdSmoothNext * maxRate;

  // Feedforward is derivative gain scaled by rate range (deg/s per unit cmd → deg/s)
  const feedforward = rt.derivativeGain * dCmdLpfNext * maxRate;

  const next: PidMinWebFeedforwardState = {
    cmdSmooth: cmdSmoothNext,
    prevCmdSmooth: cmdSmoothNext,
    dCmdLpf: dCmdLpfNext,
    initialized: true,
  };

  return { value: feedforward, state: next, targetRateDegS };
}

// ---- Multi-axis convenience --------------------------------------------------
export function makePidMinWebFeedforwardMultiState(): Axis3<PidMinWebFeedforwardState> {
  return { roll: makePidMinWebFeedforwardState(), pitch: makePidMinWebFeedforwardState(), yaw: makePidMinWebFeedforwardState() };
}

export function pidMinWebFeedforwardUpdateAll(
  prev: Axis3<PidMinWebFeedforwardState>,
  rt: Readonly<PidMinWebFeedforwardRuntime>,
  joystick: Axis3<number>,
  dt: number
): { values: Axis3<number>; targetRatesDegS: Axis3<number>; state: Axis3<PidMinWebFeedforwardState> } {
  const roll = pidMinWebFeedforwardUpdate(prev.roll, rt, 'roll', joystick.roll, dt);
  const pitch = pidMinWebFeedforwardUpdate(prev.pitch, rt, 'pitch', joystick.pitch, dt);
  const yaw = pidMinWebFeedforwardUpdate(prev.yaw, rt, 'yaw', joystick.yaw, dt);
  return {
    values: { roll: roll.value, pitch: pitch.value, yaw: yaw.value },
    targetRatesDegS: { roll: roll.targetRateDegS, pitch: pitch.targetRateDegS, yaw: yaw.targetRateDegS },
    state: { roll: roll.state, pitch: pitch.state, yaw: yaw.state },
  };
}