// I-term relax module (pure functions)
// Provides type and pure helper to compute integral error rate
// based on setpoint transients with LPF/HPF decomposition.
import { pt1Alpha, fapplyDeadband } from './utils';

// Relax modes
export enum PidMinItermRelaxType {
	Off = 0,
	Setpoint = 1,
	Gyro = 2,
}

// Utilities consolidated in shared utils.ts

export interface PidMinItermRelaxConfig {
    iRelaxEnabled: boolean;
    iRelaxCutoffHz: number;
    iRelaxSetpointThreshold: number;
    iRelaxType: PidMinItermRelaxType;
}

export interface PidMinItermRelaxResult {
    iErrorRate: number;
    nextPrevSetpointLpf: number;
    relaxFactor: number; // 1 when not in Setpoint mode
    setpointLpf: number;
    setpointHpf: number;
}

// Pure computation of I-term relax contribution
// Inputs are immutable; returns next state and computed error
export function computeItermRelax(
    setpoint: number,
    gyro: number,
    dt: number,
    prevSetpointLpf: number,
    cfg: PidMinItermRelaxConfig
): PidMinItermRelaxResult {
	const errorRate = setpoint - gyro;
	let iErrorRate = errorRate;
	let nextPrevSetpointLpf = prevSetpointLpf;
	let relaxFactor = 1.0;
	let setpointLpf = prevSetpointLpf;
	let setpointHpf = 0.0;

	if (cfg.iRelaxEnabled && cfg.iRelaxType !== PidMinItermRelaxType.Off) {
		const alphaRelax =
			cfg.iRelaxCutoffHz > 0 ? pt1Alpha(cfg.iRelaxCutoffHz, dt) : 1;
		setpointLpf = prevSetpointLpf + alphaRelax * (setpoint - prevSetpointLpf);
		setpointHpf = Math.abs(setpoint - setpointLpf);
		nextPrevSetpointLpf = setpointLpf;

		if (cfg.iRelaxType === PidMinItermRelaxType.Setpoint) {
			const threshold =
				cfg.iRelaxSetpointThreshold > 0 ? cfg.iRelaxSetpointThreshold : 40.0;
			relaxFactor = Math.max(0, 1 - setpointHpf / threshold);
			iErrorRate *= relaxFactor;
		} else if (cfg.iRelaxType === PidMinItermRelaxType.Gyro) {
			iErrorRate = fapplyDeadband(setpointLpf - gyro, setpointHpf);
		}
	}

	return {
		iErrorRate,
		nextPrevSetpointLpf,
		relaxFactor,
		setpointLpf,
		setpointHpf,
	};
}