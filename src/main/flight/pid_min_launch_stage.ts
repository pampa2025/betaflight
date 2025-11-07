// Launch Stage Detector (pure functions)
// Robustly determines whether the system is in the launch phase,
// handling ground or elevated (table) starts via a baseline altitude.
//
// Design goals:
// - Pure-functional: caller owns state; this module returns next state.
// - Debounced entry/exit using altitude, vertical speed, optional accel Z.
// - Baseline altitude captured on arming to support table launch (elevated start).
// - Minimal inputs: armed, lcEnabled, altitude, vertical speed, stick/throttle.
//
// Typical wiring:
// - Call updateLaunchStage(...) once per control tick with dt in ms.
// - Use the returned `launchActive` boolean to drive pid_min.ts `launchActive`.
// - Pass `rcDeflection` as your pitch stick/joystick deflection [-1..1].
// - Record `baselineAlt` automatically on arming rising edge.
//
// Defaults target a simulator: small entry threshold, modest exit threshold,
// vertical-speed exit for robustness, and short debounces.

export interface PidMinLaunchStageConfig {
	// Altitude thresholds relative to baseline (captured at arming)
	altEnterMeters: number; // <= this is considered "on ground"
	altExitMeters: number; // >= this indicates airborne

	// Exit velocity threshold (upward)
	vExitMetersPerSec: number; // >= this indicates leaving ground

	// Command thresholds to enter launch
	rcEnterThreshold: number; // abs(rcDeflection) >= threshold
	throttleEnterThreshold: number; // throttle >= threshold

	// Debounce windows
	debounceMsEnter: number; // continuous ms required to enter
	debounceMsExit: number; // continuous ms required to exit

	// Optional: use accel Z to aid exit detection (upward thrust)
	useAccelZ?: boolean;
	accelZExitThreshold?: number; // m/s^2, net upward accel
}

export interface PidMinLaunchStageInputs {
	armed: boolean;
	lcEnabled: boolean; // feature enabled; if false, launch never active
	altitude: number; // meters
	verticalSpeed: number; // m/s (positive up)
	throttle: number; // [0..1]
	rcDeflection: number; // [-1..1] (e.g., pitch stick)
	motorsSpinning?: boolean; // optional RPM/switch hint
	accelZ?: number; // optional m/s^2 (positive up)
	timeStepMs: number; // dt in milliseconds
}

export class PidMinLaunchStageState {
	baselineAlt: number | null = null; // meters at arming
	launchActive = false;
	enterAccumMs = 0; // running debounce accumulator
	exitAccumMs = 0; // running debounce accumulator
	armedPrev = false; // track rising edge for baseline capture
}

export function makeDefaultLaunchStageConfig(): PidMinLaunchStageConfig {
	return {
		altEnterMeters: 0.1,
		altExitMeters: 0.4,
		vExitMetersPerSec: 0.7,
		rcEnterThreshold: 0.1,
		throttleEnterThreshold: 0.2,
		debounceMsEnter: 120,
		debounceMsExit: 150,
		useAccelZ: false,
		accelZExitThreshold: 0.5,
	};
}

export function makeLaunchStageState(): PidMinLaunchStageState {
	return new PidMinLaunchStageState();
}

// Core detector update
// Returns the current launchActive flag and the next state.
export function updateLaunchStage(
	cfg: PidMinLaunchStageConfig,
	prev: Readonly<PidMinLaunchStageState>,
	inputs: Readonly<PidMinLaunchStageInputs>
): { launchActive: boolean; state: PidMinLaunchStageState } {
	const next = new PidMinLaunchStageState();
	next.baselineAlt = prev.baselineAlt;
	next.launchActive = prev.launchActive;
	next.enterAccumMs = prev.enterAccumMs;
	next.exitAccumMs = prev.exitAccumMs;
	next.armedPrev = prev.armedPrev;

	const dtMs = inputs.timeStepMs > 0 ? inputs.timeStepMs : 0;

	// Capture baseline altitude on arming rising edge (supports table starts)
	if (inputs.armed && !next.armedPrev) {
		next.baselineAlt = inputs.altitude;
	}
	next.armedPrev = inputs.armed;

	let launchActive = next.launchActive;

	// Hard gate: disabled or disarmed means no launch
	if (!inputs.lcEnabled || !inputs.armed) {
		launchActive = false;
		next.enterAccumMs = 0;
		next.exitAccumMs = 0;
	} else {
		const baselineAlt = next.baselineAlt;
		const altAbove = baselineAlt != null ? inputs.altitude - baselineAlt : 0;

		const wantLaunchCmd =
			Math.abs(inputs.rcDeflection) >= cfg.rcEnterThreshold ||
			inputs.throttle >= cfg.throttleEnterThreshold ||
			!!inputs.motorsSpinning;

		const onGround = baselineAlt != null && altAbove <= cfg.altEnterMeters;

		// Debounced enter: require command + on-ground
		if (!launchActive && wantLaunchCmd && onGround) {
			next.enterAccumMs += dtMs;
			if (next.enterAccumMs >= cfg.debounceMsEnter) {
				launchActive = true;
				next.enterAccumMs = 0;
				next.exitAccumMs = 0;
			}
		} else {
			next.enterAccumMs = 0;
		}

		// Debounced exit: altitude or vertical speed (optionally accel Z)
		const exitByAlt = baselineAlt != null && altAbove >= cfg.altExitMeters;
		const exitByV = inputs.verticalSpeed >= cfg.vExitMetersPerSec;
		const exitByAccel =
			!!cfg.useAccelZ &&
			inputs.accelZ !== undefined &&
			inputs.accelZ >= (cfg.accelZExitThreshold ?? 0.5);

		const exitCondition = exitByAlt || exitByV || exitByAccel;

		if (launchActive && exitCondition) {
			next.exitAccumMs += dtMs;
			if (next.exitAccumMs >= cfg.debounceMsExit) {
				launchActive = false;
				next.exitAccumMs = 0;
				next.enterAccumMs = 0;
			}
		} else {
			next.exitAccumMs = 0;
		}
	}

	next.launchActive = launchActive;
	return { launchActive, state: next };
}

/*
Usage example (TypeScript):
------------------------------------------------------------
import {
  makeDefaultLaunchStageConfig,
  makeLaunchStageState,
  updateLaunchStage,
} from './pid_min_launch_stage';

const cfg = makeDefaultLaunchStageConfig();
let s = makeLaunchStageState();

function tick(dtMs: number, sim) {
  const { armed, lcEnabled, altitude, verticalSpeed, throttle, joystick, accelZ } = sim;
  const r = updateLaunchStage(cfg, s, {
    armed,
    lcEnabled,
    altitude,
    verticalSpeed,
    throttle,
    rcDeflection: joystick.pitch,
    motorsSpinning: throttle > 0.1,
    accelZ,
    timeStepMs: dtMs,
  });
  s = r.state;
  const launchActive = r.launchActive;
  // Pass launchActive to pid_min.ts pidMinUpdateUnified / pidMinUpdateWebAll.
}

Notes:
- Table launch is naturally supported because baselineAlt is captured at arming.
  Altitude comparisons use (current - baselineAlt), not absolute AMSL.
- If you lack altitude or vertical speed, you can still enter/exit using command
  thresholds and accelZ (set useAccelZ=true), but altitude-based hysteresis is
  recommended for stability in simulators.
*/
