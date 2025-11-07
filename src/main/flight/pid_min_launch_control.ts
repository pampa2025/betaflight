// Launch Control module (pure functions)
// Decouples launch-specific logic from pid_min.ts
import { clampf, scaleRangef } from './utils';

// Axis and mode enums
export enum PidMinAxis {
	Roll = 0,
	Pitch = 1,
	Yaw = 2,
}

export enum PidMinLaunchMode {
	PitchOnly = 0,
	Full = 1,
}

export interface PidMinLaunchConfig {
	enabled: boolean; // feature enabled
	mode: PidMinLaunchMode; // pitch-only or full
	angleLimitDeg: number; // pitch angle limit in degrees (<=0 disables)
	kiOverride: number; // Ki override while active (0 uses normal Ki)
	// Optional runtime overrides (with sane defaults)
	maxRateDegS?: number; // default LAUNCH_CONTROL_MAX_RATE
	minRateDegS?: number; // default LAUNCH_CONTROL_MIN_RATE
	angleWindowDeg?: number; // default LAUNCH_CONTROL_ANGLE_WINDOW
	yawItermLimitDegS?: number; // default LAUNCH_CONTROL_YAW_ITERM_LIMIT (Full mode)
}

// Constants (kept minimal and encapsulated here)
export const LAUNCH_CONTROL_MAX_RATE = 100.0;
export const LAUNCH_CONTROL_MIN_RATE = 5.0;
export const LAUNCH_CONTROL_ANGLE_WINDOW = 10.0;
export const LAUNCH_CONTROL_YAW_ITERM_LIMIT = 50.0;

// Utilities consolidated in shared utils.ts

// Compute per-axis launch setpoint
export function pidMinApplyLaunchSetpoint(
	axis: PidMinAxis,
	lc: PidMinLaunchConfig | null | undefined,
	rcDeflection: number, // [-1..1]
	currentPitchAngleDeg: number, // degrees
	trimPitchDeg: number // degrees
): number {
	if (!lc || !lc.enabled) return 0;
	if (lc.mode === PidMinLaunchMode.PitchOnly && axis !== PidMinAxis.Pitch) {
		return 0;
	}
	const stick = clampf(rcDeflection, -0.5, 0.5);
	const maxRate = lc.maxRateDegS ?? LAUNCH_CONTROL_MAX_RATE;
	const minRate = lc.minRateDegS ?? LAUNCH_CONTROL_MIN_RATE;
	const angleWindow = lc.angleWindowDeg ?? LAUNCH_CONTROL_ANGLE_WINDOW;
	let rate = maxRate * (stick * 2.0); // map +/-0.5 to +/-max
	if (axis === PidMinAxis.Pitch && lc.angleLimitDeg > 0) {
		const currentAngle = currentPitchAngleDeg - trimPitchDeg;
		if (currentAngle >= lc.angleLimitDeg) {
			rate = 0;
		} else {
			const angleDelta = lc.angleLimitDeg - currentAngle;
			if (angleDelta <= angleWindow) {
				const targetRate = rate;
				rate = scaleRangef(
					angleDelta,
					0,
					angleWindow,
					rate >= 0 ? minRate : -minRate,
					targetRate
				);
			}
		}
	}
	return rate;
}

// Effects to apply during launch
export interface PidMinLaunchEffects {
	setpoint: number;
	effectiveKi: number;
	disableD: boolean;
	disableFeedforward: boolean;
	disableP: boolean;
	disableI: boolean;
	yawItermLimit: number | null; // clamp integrator to [-limit, limit] if not null
	enforcePitchINonNegative: boolean;
}

// Compute launch effects in a pure manner
export function computeLaunchEffects(
	axis: PidMinAxis,
	lc: PidMinLaunchConfig | null | undefined,
	launchActive: boolean,
	setpoint: number,
	rcDeflection: number,
	currentPitchAngleDeg: number,
	trimPitchDeg: number,
	ki: number
): PidMinLaunchEffects {
	if (!launchActive || !lc || !lc.enabled) {
		return {
			setpoint,
			effectiveKi: ki,
			disableD: false,
			disableFeedforward: false,
			disableP: false,
			disableI: false,
			yawItermLimit: null,
			enforcePitchINonNegative: false,
		};
	}

	const lcSetpoint = pidMinApplyLaunchSetpoint(
		axis,
		lc,
		rcDeflection,
		currentPitchAngleDeg,
		trimPitchDeg
	);

	const effectiveKi = lc.kiOverride > 0 ? lc.kiOverride : ki;
	const disableD = true;
	const disableFeedforward = true;

	let disableP = false;
	let disableI = false;
	let yawItermLimit: number | null = null;
	let enforcePitchINonNegative = false;

	if (lc.mode === PidMinLaunchMode.PitchOnly) {
		if (axis === PidMinAxis.Roll || axis === PidMinAxis.Yaw) {
			disableP = true;
			disableI = true;
		}
		if (axis === PidMinAxis.Pitch) {
			enforcePitchINonNegative = true;
		}
		// Yaw limit in pitch-only mode is zero (no yaw I during launch)
		if (axis === PidMinAxis.Yaw) {
			yawItermLimit = 0.0;
		}
	} else if (lc.mode === PidMinLaunchMode.Full) {
		if (axis === PidMinAxis.Yaw) {
			yawItermLimit = lc.yawItermLimitDegS ?? LAUNCH_CONTROL_YAW_ITERM_LIMIT;
		}
	}

	return {
		setpoint: lcSetpoint,
		effectiveKi,
		disableD,
		disableFeedforward,
		disableP,
		disableI,
		yawItermLimit,
		enforcePitchINonNegative,
	};
}
