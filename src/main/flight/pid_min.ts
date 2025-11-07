// Minimal Betaflight-style PID controller (TypeScript)
// Mirrors src/main/flight/pid_min.c with a tiny, dependency-free core
// and a minimal Launch Control feature for one axis.
import {
	PidMinItermRelaxType,
	computeItermRelax,
	PidMinItermRelaxConfig,
	PidMinItermRelaxResult,
} from './pid_min_iterm_relax';
export { PidMinItermRelaxType } from './pid_min_iterm_relax';
import {
	PidMinAxis,
	PidMinLaunchMode,
	PidMinLaunchConfig,
	computeLaunchEffects,
	PidMinLaunchEffects,
} from './pid_min_launch_control';
export {
	PidMinAxis,
	PidMinLaunchMode,
	PidMinLaunchConfig,
} from './pid_min_launch_control';
import { clampf, pt1Alpha } from './utils';
import {
    PidMinDmaxConfig,
    PidMinDmaxState,
    computeDmaxMultiplier,
    makeDefaultDmaxState,
} from './pid_min_dmax';
import {
    PidMinFeedforwardRuntime,
    PidMinFeedforwardState,
    pidMinFeedforwardUpdate,
    makePidMinFeedforwardState,
} from './pid_min_feedforward';

// Coefficients (gains)
export interface PidMinCoefficients {
	Kp: number; // proportional gain
	Ki: number; // integral gain (per second)
	Kd: number; // derivative gain (per second)
	Kf: number; // feedforward gain on setpoint change
}

// Runtime configuration
export interface PidMinConfig {
    pidSumLimit: number; // clamp output sum to [-limit, +limit]; 0 disables
    itermLimit: number; // clamp integrator to [-limit, +limit]; 0 disables
    integratorLeak: number; // 0..1 fraction leaked per update; 0 disables
    useFeedforward: boolean; // include setpoint change feedforward
    dLowpassCutoffHz: number; // PT1 cutoff for gyro used by D-term; 0 disables
    // I-term relax (minimal)
    iRelax: PidMinItermRelaxConfig; // nested configuration for I-term relax
    // D-Max boost (optional)
    dMax?: PidMinDmaxConfig | null;
    // Advanced feedforward integration (optional)
    // When enabled, F-term is sourced from rc.c-style feedforward rather than simple Kf * Δsetpoint
    feedforwardMode?: 'simple' | 'advanced';
    feedforwardRuntime?: PidMinFeedforwardRuntime | null;
}

// Per-axis state
export class PidMinState {
    integrator = 0; // I-term accumulator
    prevGyroFiltered = 0; // last filtered gyro (for D-term stability)
    prevSetpoint = 0; // last setpoint (for feedforward)
    prevSetpointLpf = 0; // last lowpassed setpoint (for I-term relax)
    initialized = false; // guard first update
    dmax: PidMinDmaxState = makeDefaultDmaxState(); // D-Max per-axis state
    // Advanced feedforward per-axis state (optional)
    ff: PidMinFeedforwardState | null = null;
}

// Init/reset
export function pidMinInit(s: PidMinState): void {
    s.integrator = 0;
    s.prevGyroFiltered = 0;
    s.prevSetpoint = 0;
    s.prevSetpointLpf = 0;
    s.initialized = false;
    s.dmax = makeDefaultDmaxState();
    s.ff = null;
}

export function pidMinReset(s: PidMinState): void {
	pidMinInit(s);
}

// Core PID update for one axis
// (removed: legacy non-launch update; unified pure update below handles both phases)

// Launch Control: compute per-axis setpoint
// (moved to pid_min_launch_control.ts)

// PID update with Launch Control gating
// (removed: mutable unified update; unified pure update below is the single API)

// Unified update alias (mutable state): use one function for both phases.
// Single API: unified pure update below

// Pure unified update: returns next state without mutating the input.
// This is ideal for simulators or functional pipelines.
export function pidMinUpdateUnified(
    c: PidMinCoefficients,
    cfg: PidMinConfig,
    lc: PidMinLaunchConfig | null | undefined,
    prev: Readonly<PidMinState>,
    axis: PidMinAxis,
    setpoint: number,
    gyro: number,
    dt: number,
    launchActive: boolean,
    rcDeflection: number,
    currentPitchAngleDeg: number,
    trimPitchDeg: number,
    ffCtx?: { rcCmd: number; rxRateHz: number; rxIntervalUs: number; maxRcRate: number }
): { output: number; state: PidMinState } {
	if (!c || !cfg || !prev) return { output: 0, state: new PidMinState() };
	if (dt <= 0) return { output: 0, state: new PidMinState() };

	// Derive initial refs without mutating prev
	const initialized = !!prev.initialized;
	const gyroFilteredPrev = initialized ? prev.prevGyroFiltered : gyro;
	const setpointPrev = initialized ? prev.prevSetpoint : setpoint;
	const setpointLpfPrev = initialized ? prev.prevSetpointLpf : setpoint;
	let integrator = initialized ? prev.integrator : 0;
	const dmaxPrev: PidMinDmaxState = initialized
		? prev.dmax
		: makeDefaultDmaxState();

	const launchEffects: PidMinLaunchEffects = computeLaunchEffects(
		axis,
		lc,
		launchActive,
		setpoint,
		rcDeflection,
		currentPitchAngleDeg,
		trimPitchDeg,
		c.Ki
	);
	setpoint = launchEffects.setpoint;

	const errorRate = setpoint - gyro;

	const iRelaxCfg: PidMinItermRelaxConfig = cfg.iRelax;
	const itermRelax: PidMinItermRelaxResult = computeItermRelax(
		setpoint,
		gyro,
		dt,
		setpointLpfPrev,
		iRelaxCfg
	);
	let iErrorRate = itermRelax.iErrorRate;
	let setpointLpfNext = itermRelax.nextPrevSetpointLpf;

	// P
	let P = c.Kp * errorRate;

	// Ki override under launch
	let effectiveKi = launchEffects.effectiveKi;

	// I integrate + leak
	integrator += effectiveKi * iErrorRate * dt;
	if (cfg.integratorLeak > 0) {
		const leak = clampf(cfg.integratorLeak, 0, 1);
		integrator *= 1 - leak;
	}

	// Launch yaw I-term limit in FULL mode; zero limit in PITCHONLY
	if (launchEffects.yawItermLimit !== null) {
		const yawLimit = launchEffects.yawItermLimit;
		integrator = clampf(integrator, -yawLimit, yawLimit);
	}
	// Generic I-term limit if not constrained by yaw rule above
	if (launchEffects.yawItermLimit === null) {
		if (cfg.itermLimit > 0) {
			integrator = clampf(integrator, -cfg.itermLimit, cfg.itermLimit);
		}
	}
	let I = integrator;

	// In pitch-only mode, keep pitch I non-negative
	if (launchEffects.enforcePitchINonNegative && axis === PidMinAxis.Pitch) {
		if (I < 0) {
			I = 0;
			integrator = 0;
		}
	}

	// D update; disabled during launch for output, but keep filter state
	const alpha =
		cfg.dLowpassCutoffHz > 0 ? pt1Alpha(cfg.dLowpassCutoffHz, dt) : 1;
	const gyroFiltered = gyroFilteredPrev + alpha * (gyro - gyroFilteredPrev);
	const dGyro = gyroFiltered - gyroFilteredPrev;
	// D-Max multiplier and state update (always update filter states)
	const deltaGyroDt = -dGyro / dt;
	const setpointDelta = setpoint - setpointPrev;
	let dMultiplier = 1.0;
	let dmaxNext: PidMinDmaxState = dmaxPrev;
	if (cfg.dMax && cfg.dMax.enabled) {
		const dmaxRes = computeDmaxMultiplier(cfg.dMax, dmaxPrev, {
			axis,
			deltaGyroDt,
			setpointDelta,
			dt,
		});
		dMultiplier = dmaxRes.multiplier;
		dmaxNext = dmaxRes.state;
	}

	let D = 0;
	if (!launchEffects.disableD) {
		D = c.Kd * deltaGyroDt * dMultiplier;
	}

    // Feedforward: simple (Kf * Δsetpoint) or advanced rc-style, gated by launch
    let F = 0;
    let ffNext: PidMinFeedforwardState | null = prev.ff || null;
    if (cfg.useFeedforward && !launchEffects.disableFeedforward) {
        const mode = cfg.feedforwardMode ?? 'simple';
        if (mode === 'advanced' && cfg.feedforwardRuntime && ffCtx) {
            const prevFf = ffNext ?? makePidMinFeedforwardState(cfg.feedforwardRuntime);
            const { value, state } = pidMinFeedforwardUpdate(
                prevFf,
                cfg.feedforwardRuntime,
                axis,
                setpoint,
                ffCtx.rcCmd,
                ffCtx.rxRateHz,
                ffCtx.rxIntervalUs,
                ffCtx.maxRcRate
            );
            F = value;
            ffNext = state;
        } else if (c.Kf !== 0) {
            const dSetpoint = setpoint - setpointPrev;
            F = c.Kf * dSetpoint;
        }
    }

	// P/I disabled for non-pitch axes in pitch-only mode
	if (launchEffects.disableP) {
		P = 0;
	}
	if (launchEffects.disableI) {
		I = 0;
	}

	let sum = P + I + D + F;
	if (cfg.pidSumLimit > 0) {
		sum = clampf(sum, -cfg.pidSumLimit, cfg.pidSumLimit);
	}

    const next = new PidMinState();
    next.integrator = integrator;
    next.prevGyroFiltered = gyroFiltered;
    next.prevSetpoint = setpoint;
    next.prevSetpointLpf = setpointLpfNext;
    next.initialized = true;
    next.dmax = dmaxNext;
    next.ff = ffNext;

    return { output: sum, state: next };
}

// Usage examples (safe line comments)
// ------------------------------------------------------------
// Setup
// const c: PidMinCoefficients = { Kp: 0.9, Ki: 0.3, Kd: 0.02, Kf: 0 };
// const cfg: PidMinConfig = {
//   pidSumLimit: 0,
//   itermLimit: 100,
//   integratorLeak: 0.05,
//   useFeedforward: false,
//   dLowpassCutoffHz: 40,
//   // I-term relax (optional)
//   iRelax: {
//     iRelaxEnabled: true,
//     iRelaxCutoffHz: 15,               // typical default
//     iRelaxSetpointThreshold: 40,      // typical default
//     iRelaxType: PidMinItermRelaxType.Setpoint // or Gyro, or Off
//   }
//   // D-Max (optional)
//   dMax: {
//     enabled: true,
//     dMaxPercent: { roll: 1.25, pitch: 1.25, yaw: 1.10 },
//     gain: 37,
//     advance: 20,
//     // rangeCutoffHz: 85, lowpassCutoffHz: 35, // defaults
//   }
// };
// const lc: PidMinLaunchConfig = { enabled: true, mode: PidMinLaunchMode.PitchOnly, angleLimitDeg: 30, kiOverride: 0.15 };
// let launchActive = true;
// const state = { roll: new PidMinState(), pitch: new PidMinState(), yaw: new PidMinState() };
//
// Per-tick (pure functional updates)
// const dt = 0.001;
// const setpoints = { roll: 0, pitch: 0, yaw: 0 };
// const gyro = { roll: 0, pitch: 0, yaw: 0 };
// const rcDeflection = 0.4;
// const pitchDeg = 10;
// const trimDeg = 0;
// const r = pidMinUpdateUnified(c, cfg, lc, state.roll, PidMinAxis.Roll, setpoints.roll, gyro.roll, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// state.roll = r.state;
// const p = pidMinUpdateUnified(c, cfg, lc, state.pitch, PidMinAxis.Pitch, setpoints.pitch, gyro.pitch, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// state.pitch = p.state;
// const y = pidMinUpdateUnified(c, cfg, lc, state.yaw, PidMinAxis.Yaw, setpoints.yaw, gyro.yaw, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// state.yaw = y.state;
// const outputs = { roll: r.output, pitch: p.output, yaw: y.output };
//
// Notes on I-term relax:
// - Setpoint mode scales I accumulation when setpoint changes (reduces windup).
// - Gyro mode reconstructs error using setpoint LPF and HPF deadband.
// - Disable by setting iRelaxEnabled: false or iRelaxType: Off.
//
// Transition out of launch
// if (/* condition to exit */ false) { launchActive = false; lc.enabled = false; }
//
// Single API returns next state; create a wrapper yourself if needed.
