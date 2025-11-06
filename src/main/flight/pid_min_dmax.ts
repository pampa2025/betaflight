// D-Max module (pure functions)
// Mirrors Betaflight's dynamic D-term boost logic in pid.c/pid_init.c.
// - Boosts Kd towards a higher Dmax based on gyro derivative activity and RC setpoint activity.
// - Uses PT2 filters: one to isolate propwash-range activity on gyro derivative, one to smooth the multiplier.
// - All operations are pure; state is provided and returned explicitly.

import { pt1Alpha } from './utils';
import { PidMinAxis } from './pid_min_launch_control';

// Defaults derived from C implementation
export const D_MAX_RANGE_HZ = 85; // PT2 cutoff for propwash-range activity
export const D_MAX_LOWPASS_HZ = 35; // PT2 cutoff to smooth the boost; keep >0
export const D_MAX_GYRO_GAIN_FACTOR = 0.00008;
export const D_MAX_SETPOINT_GAIN_FACTOR = 0.00008; // same factor for setpoint path
// Match Betaflight's PT2 cutoff correction, ensuring -3 dB at the requested cutoff
const CUTOFF_CORRECTION_PT2 = 1.553773974;

export interface PidMinDmaxPercentByAxis {
	roll: number; // >= 1.0; fraction of Dmax relative to base Kd for roll
	pitch: number; // >= 1.0; fraction of Dmax relative to base Kd for pitch
	yaw: number; // >= 1.0; fraction of Dmax relative to base Kd for yaw
}

export interface PidMinDmaxConfig {
	enabled: boolean;
	// Percentage multiplier of D relative to base Kd per axis (>=1 means Dmax > D)
	dMaxPercent: PidMinDmaxPercentByAxis;
	// Gains analogous to Betaflight's profile params
	gain: number; // d_max_gain; scales how much gyro activity contributes
	advance: number; // d_max_advance; scales setpoint activity contribution
	// Optional overrides for filter cutoffs; defaults mirror C
	rangeCutoffHz?: number; // default 85
	lowpassCutoffHz?: number; // default 35
}

// Minimal PT2 state for cascaded first-order sections
export interface Pt2CascadeState {
	s1: number;
	s2: number;
}

export interface PidMinDmaxState {
	range: Pt2CascadeState; // PT2 over gyro derivative (delta)
	lowpass: Pt2CascadeState; // PT2 over multiplier
}

export function makeDefaultDmaxState(): PidMinDmaxState {
	return { range: { s1: 0, s2: 0 }, lowpass: { s1: 0, s2: 0 } };
}

// Apply a simple PT2 by cascading two identical PT1 sections.
// Returns next state and output y.
function pt2ApplyCascade(
	prev: Pt2CascadeState,
	x: number,
	cutoffHz: number,
	dt: number
): { y: number; state: Pt2CascadeState } {
	if (dt <= 0 || cutoffHz <= 0) {
		// no filtering; follow input
		const y = x;
		return { y, state: { s1: y, s2: y } };
	}
    const alpha = pt1Alpha(cutoffHz * CUTOFF_CORRECTION_PT2, dt);
	const s1 = prev.s1 + alpha * (x - prev.s1);
	const s2 = prev.s2 + alpha * (s1 - prev.s2);
	return { y: s2, state: { s1, s2 } };
}

function percentForAxis(
	axis: PidMinAxis,
	percents: PidMinDmaxPercentByAxis
): number {
	switch (axis) {
		case PidMinAxis.Roll:
			return percents.roll;
		case PidMinAxis.Pitch:
			return percents.pitch;
		case PidMinAxis.Yaw:
			return percents.yaw;
		default:
			return 1.0;
	}
}

export interface PidMinDmaxInputs {
	axis: PidMinAxis;
	deltaGyroDt: number; // fixed-time derivative on gyro (dr/dt)
	setpointDelta: number; // RC setpoint delta used by feedforward path
	dt: number; // seconds
}

export interface PidMinDmaxResult {
	multiplier: number; // clamp [1 .. dMaxPercent]
	state: PidMinDmaxState; // next filter states
	debug?: { gyroFactor: number; setpointFactor: number; boost: number };
}

// Compute dynamic D multiplier based on gyro and setpoint activity.
// Pure: consumes previous state and returns next state.
export function computeDmaxMultiplier(
	cfg: PidMinDmaxConfig,
	prev: Readonly<PidMinDmaxState>,
	inputs: Readonly<PidMinDmaxInputs>
): PidMinDmaxResult {
	const rangeHz = cfg.rangeCutoffHz ?? D_MAX_RANGE_HZ;
	const lowpassHz = cfg.lowpassCutoffHz ?? D_MAX_LOWPASS_HZ;
	const dmaxLpfInv = lowpassHz > 0 ? 1 / lowpassHz : 0; // matches C scaling

	const gyroGain = D_MAX_GYRO_GAIN_FACTOR * cfg.gain * dmaxLpfInv;
	const setpointGain = D_MAX_SETPOINT_GAIN_FACTOR * cfg.advance * dmaxLpfInv;

	const percent = percentForAxis(inputs.axis, cfg.dMaxPercent);
	// Start at 1.0 (no boost). If percent <= 1, feature has no effect.
	let dMaxMultiplierPre = 1.0;
	let nextRange = prev.range;
	let nextLowpass = prev.lowpass;

	if (percent > 1.0 && cfg.enabled) {
		// Filter gyro derivative through PT2 to isolate propwash range
		const rangeOut = pt2ApplyCascade(
			prev.range,
			inputs.deltaGyroDt,
			rangeHz,
			inputs.dt
		);
		nextRange = rangeOut.state;
		const dMaxGyroFactor = Math.abs(rangeOut.y) * gyroGain;
		const dMaxSetpointFactor = Math.abs(inputs.setpointDelta) * setpointGain;
		const dMaxBoost = Math.max(dMaxGyroFactor, dMaxSetpointFactor);

		dMaxMultiplierPre += (percent - 1.0) * dMaxBoost;
		// Smooth the multiplier with a PT2
		const lpOut = pt2ApplyCascade(
			prev.lowpass,
			dMaxMultiplierPre,
			lowpassHz,
			inputs.dt
		);
		nextLowpass = lpOut.state;
		dMaxMultiplierPre = lpOut.y;
		// Limit to max percent
		dMaxMultiplierPre = Math.min(dMaxMultiplierPre, percent);
		return {
			multiplier: dMaxMultiplierPre,
			state: { range: nextRange, lowpass: nextLowpass },
			debug: {
				gyroFactor: dMaxGyroFactor,
				setpointFactor: dMaxSetpointFactor,
				boost: dMaxBoost,
			},
		};
	}

	// No effect or disabled: multiplier is 1, maintain state as simple follows
	const lpOut = pt2ApplyCascade(prev.lowpass, 1.0, lowpassHz, inputs.dt);
	nextLowpass = lpOut.state;
	return { multiplier: 1.0, state: { range: nextRange, lowpass: nextLowpass } };
}
