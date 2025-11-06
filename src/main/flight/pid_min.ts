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
export { PidMinAxis, PidMinLaunchMode, PidMinLaunchConfig } from './pid_min_launch_control';
import { clampf, pt1Alpha, fapplyDeadband, scaleRangef } from './utils';

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
    iRelaxEnabled: boolean; // master enable
    iRelaxCutoffHz: number; // PT1 cutoff (Hz) for setpoint LPF
    iRelaxSetpointThreshold: number; // threshold (deg/s) for relax factor (default ~40)
    iRelaxType: PidMinItermRelaxType; // OFF, SETPOINT, GYRO
}

// Per-axis state
export class PidMinState {
    integrator = 0; // I-term accumulator
    prevGyroFiltered = 0; // last filtered gyro (for D-term stability)
    prevSetpoint = 0; // last setpoint (for feedforward)
    prevSetpointLpf = 0; // last lowpassed setpoint (for I-term relax)
    initialized = false; // guard first update
}

// I-term relax type
// (moved to pid_min_iterm_relax.ts)

// Launch Control types
// (moved to pid_min_launch_control.ts)

// Constants (kept simple, aligned with C version)
// (moved to pid_min_launch_control.ts)

// Utilities moved to utils.ts

// Init/reset
export function pidMinInit(s: PidMinState): void {
    s.integrator = 0;
    s.prevGyroFiltered = 0;
    s.prevSetpoint = 0;
    s.prevSetpointLpf = 0;
    s.initialized = false;
}

export function pidMinReset(s: PidMinState): void {
	pidMinInit(s);
}

// Core PID update for one axis
export function pidMinUpdate(
	c: PidMinCoefficients,
	cfg: PidMinConfig,
	s: PidMinState,
	setpoint: number, // deg/s desired rate
	gyro: number, // deg/s measured rate
	dt: number // seconds since last update
): number {
	if (!c || !cfg || !s) return 0;
	if (dt <= 0) return 0;

    if (!s.initialized) {
        s.prevGyroFiltered = gyro;
        s.prevSetpoint = setpoint;
        s.prevSetpointLpf = setpoint;
        s.initialized = true;
    }

    const errorRate = setpoint - gyro;
    const itermRelaxCfg: PidMinItermRelaxConfig = {
        iRelaxEnabled: cfg.iRelaxEnabled,
        iRelaxCutoffHz: cfg.iRelaxCutoffHz,
        iRelaxSetpointThreshold: cfg.iRelaxSetpointThreshold,
        iRelaxType: cfg.iRelaxType,
    };
    const itermRelax: PidMinItermRelaxResult = computeItermRelax(
        setpoint,
        gyro,
        dt,
        s.prevSetpointLpf,
        itermRelaxCfg
    );
    let iErrorRate = itermRelax.iErrorRate;
    s.prevSetpointLpf = itermRelax.nextPrevSetpointLpf;

	// P
	let P = c.Kp * errorRate;

    // I
    s.integrator += c.Ki * iErrorRate * dt;
	if (cfg.integratorLeak > 0) {
		const leak = clampf(cfg.integratorLeak, 0, 1);
		s.integrator *= 1 - leak;
	}
	if (cfg.itermLimit > 0) {
		s.integrator = clampf(s.integrator, -cfg.itermLimit, cfg.itermLimit);
	}
	let I = s.integrator;

	// D on measurement (filtered gyro derivative)
	const alpha =
		cfg.dLowpassCutoffHz > 0 ? pt1Alpha(cfg.dLowpassCutoffHz, dt) : 1;
	const gyroFiltered = s.prevGyroFiltered + alpha * (gyro - s.prevGyroFiltered);
	const dGyro = gyroFiltered - s.prevGyroFiltered;
	s.prevGyroFiltered = gyroFiltered;
	const D = c.Kd * (-dGyro / dt);

	// Feedforward on setpoint change
	let F = 0;
	if (cfg.useFeedforward && c.Kf !== 0) {
		const dSetpoint = setpoint - s.prevSetpoint;
		F = c.Kf * dSetpoint;
	}
	s.prevSetpoint = setpoint;

	// Sum and clamp
	let sum = P + I + D + F;
	if (cfg.pidSumLimit > 0) {
		sum = clampf(sum, -cfg.pidSumLimit, cfg.pidSumLimit);
	}
	return sum;
}

// Launch Control: compute per-axis setpoint
// (moved to pid_min_launch_control.ts)

// PID update with Launch Control gating
export function pidMinUpdateWithLaunch(
	c: PidMinCoefficients,
	cfg: PidMinConfig,
	lc: PidMinLaunchConfig | null | undefined,
	s: PidMinState,
	axis: PidMinAxis,
	setpoint: number,
	gyro: number,
	dt: number,
	launchActive: boolean,
	rcDeflection: number,
	currentPitchAngleDeg: number,
	trimPitchDeg: number
): number {
	if (!c || !cfg || !s) return 0;
	if (dt <= 0) return 0;

    if (!s.initialized) {
        s.prevGyroFiltered = gyro;
        s.prevSetpoint = setpoint;
        s.prevSetpointLpf = setpoint;
        s.initialized = true;
    }

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
    const itermRelaxCfg2: PidMinItermRelaxConfig = {
        iRelaxEnabled: cfg.iRelaxEnabled,
        iRelaxCutoffHz: cfg.iRelaxCutoffHz,
        iRelaxSetpointThreshold: cfg.iRelaxSetpointThreshold,
        iRelaxType: cfg.iRelaxType,
    };
    const itermRelax2: PidMinItermRelaxResult = computeItermRelax(
        setpoint,
        gyro,
        dt,
        s.prevSetpointLpf,
        itermRelaxCfg2
    );
    let iErrorRate = itermRelax2.iErrorRate;
    s.prevSetpointLpf = itermRelax2.nextPrevSetpointLpf;

	// P
	let P = c.Kp * errorRate;

	// Ki override during Launch Control
    let effectiveKi = launchEffects.effectiveKi;

    // I integrate + leak
    s.integrator += effectiveKi * iErrorRate * dt;
	if (cfg.integratorLeak > 0) {
		const leak = clampf(cfg.integratorLeak, 0, 1);
		s.integrator *= 1 - leak;
	}

	// Launch-specific yaw I limit in FULL mode, or zero limit in PITCHONLY
    if (launchEffects.yawItermLimit !== null) {
        const yawLimit = launchEffects.yawItermLimit;
        s.integrator = clampf(s.integrator, -yawLimit, yawLimit);
    }
	// Generic I-term limit if not constrained by yaw rule above
    if (launchEffects.yawItermLimit === null) {
        if (cfg.itermLimit > 0) {
            s.integrator = clampf(s.integrator, -cfg.itermLimit, cfg.itermLimit);
        }
    }
    let I = s.integrator;

	// In pitch-only mode, keep pitch I non-negative
    if (launchEffects.enforcePitchINonNegative && axis === PidMinAxis.Pitch) {
        if (I < 0) {
            I = 0;
            s.integrator = 0;
        }
    }

	// D disabled during Launch Control; still update filter state
	const alpha =
		cfg.dLowpassCutoffHz > 0 ? pt1Alpha(cfg.dLowpassCutoffHz, dt) : 1;
	const gyroFiltered = s.prevGyroFiltered + alpha * (gyro - s.prevGyroFiltered);
	const dGyro = gyroFiltered - s.prevGyroFiltered;
	s.prevGyroFiltered = gyroFiltered;
    let D = 0;
    if (!launchEffects.disableD) {
        D = c.Kd * (-dGyro / dt);
    }

	// Feedforward disabled during Launch Control
    let F = 0;
    if (cfg.useFeedforward && c.Kf !== 0 && !launchEffects.disableFeedforward) {
        const dSetpoint = setpoint - s.prevSetpoint;
        F = c.Kf * dSetpoint;
    }
    s.prevSetpoint = setpoint;

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
	return sum;
}

// Unified update alias (mutable state): use one function for both phases.
export const pidMinUpdateUnified = pidMinUpdateWithLaunch;

// Pure unified update: returns next state without mutating the input.
// This is ideal for simulators or functional pipelines.
export function pidMinUpdateUnifiedPure(
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
	trimPitchDeg: number
): { output: number; state: PidMinState } {
	if (!c || !cfg || !prev) return { output: 0, state: new PidMinState() };
	if (dt <= 0) return { output: 0, state: new PidMinState() };

	// Derive initial refs without mutating prev
	const initialized = !!prev.initialized;
	const prevGyroFiltered0 = initialized ? prev.prevGyroFiltered : gyro;
	const prevSetpoint0 = initialized ? prev.prevSetpoint : setpoint;
	const prevSetpointLpf0 = initialized ? prev.prevSetpointLpf : setpoint;
	let integrator = initialized ? prev.integrator : 0;

    const launchEffects2: PidMinLaunchEffects = computeLaunchEffects(
        axis,
        lc,
        launchActive,
        setpoint,
        rcDeflection,
        currentPitchAngleDeg,
        trimPitchDeg,
        c.Ki
    );
    setpoint = launchEffects2.setpoint;

	const errorRate = setpoint - gyro;

    const itermRelaxCfg3: PidMinItermRelaxConfig = {
        iRelaxEnabled: cfg.iRelaxEnabled,
        iRelaxCutoffHz: cfg.iRelaxCutoffHz,
        iRelaxSetpointThreshold: cfg.iRelaxSetpointThreshold,
        iRelaxType: cfg.iRelaxType,
    };
    const itermRelax3: PidMinItermRelaxResult = computeItermRelax(
        setpoint,
        gyro,
        dt,
        prevSetpointLpf0,
        itermRelaxCfg3
    );
    let iErrorRate = itermRelax3.iErrorRate;
    let setpointLpfNext = itermRelax3.nextPrevSetpointLpf;

	// P
	let P = c.Kp * errorRate;

	// Ki override under launch
    let effectiveKi = launchEffects2.effectiveKi;

	// I integrate + leak
	integrator += effectiveKi * iErrorRate * dt;
	if (cfg.integratorLeak > 0) {
		const leak = clampf(cfg.integratorLeak, 0, 1);
		integrator *= 1 - leak;
	}

	// Launch yaw I-term limit in FULL mode; zero limit in PITCHONLY
    if (launchEffects2.yawItermLimit !== null) {
        const yawLimit = launchEffects2.yawItermLimit;
        integrator = clampf(integrator, -yawLimit, yawLimit);
    }
	// Generic I-term limit if not constrained by yaw rule above
    if (launchEffects2.yawItermLimit === null) {
        if (cfg.itermLimit > 0) {
            integrator = clampf(integrator, -cfg.itermLimit, cfg.itermLimit);
        }
    }
    let I = integrator;

	// In pitch-only mode, keep pitch I non-negative
    if (launchEffects2.enforcePitchINonNegative && axis === PidMinAxis.Pitch) {
        if (I < 0) {
            I = 0;
            integrator = 0;
        }
    }

	// D update; disabled during launch for output, but keep filter state
	const alpha =
		cfg.dLowpassCutoffHz > 0 ? pt1Alpha(cfg.dLowpassCutoffHz, dt) : 1;
	const gyroFiltered = prevGyroFiltered0 + alpha * (gyro - prevGyroFiltered0);
	const dGyro = gyroFiltered - prevGyroFiltered0;
    let D = 0;
    if (!launchEffects2.disableD) {
        D = c.Kd * (-dGyro / dt);
    }

	// Feedforward disabled under launch
    let F = 0;
    if (cfg.useFeedforward && c.Kf !== 0 && !launchEffects2.disableFeedforward) {
        const dSetpoint = setpoint - prevSetpoint0;
        F = c.Kf * dSetpoint;
    }

	// P/I disabled for non-pitch axes in pitch-only mode
    if (launchEffects2.disableP) {
        P = 0;
    }
    if (launchEffects2.disableI) {
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
//   iRelaxEnabled: true,
//   iRelaxCutoffHz: 15,               // typical default
//   iRelaxSetpointThreshold: 40,      // typical default
//   iRelaxType: PidMinItermRelaxType.Setpoint // or Gyro, or Off
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
// const r = pidMinUpdateUnifiedPure(c, cfg, lc, state.roll, PidMinAxis.Roll, setpoints.roll, gyro.roll, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// state.roll = r.state;
// const p = pidMinUpdateUnifiedPure(c, cfg, lc, state.pitch, PidMinAxis.Pitch, setpoints.pitch, gyro.pitch, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
// state.pitch = p.state;
// const y = pidMinUpdateUnifiedPure(c, cfg, lc, state.yaw, PidMinAxis.Yaw, setpoints.yaw, gyro.yaw, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
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
// Optional mutable variant (identical behavior, mutates state internally)
// const sR = new PidMinState();
// const outR = pidMinUpdateUnified(c, cfg, lc, sR, PidMinAxis.Roll, setpoints.roll, gyro.roll, dt, launchActive, rcDeflection, pitchDeg, trimDeg);
