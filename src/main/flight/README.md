# Betaflight PID-Min (TypeScript) — Web Simulator Adaptation

This directory contains a minimal, pure-functional TypeScript adaptation of key Betaflight flight control features, tailored for use in a web FPV drone simulator. It focuses on clarity, functional purity, and explicit runtime configuration, while keeping behavior faithful to the original C implementations.

## Overview

- Core PID: `pid_min.ts` implements a minimal Betaflight-style PID with P/I/D/F terms.
- Feedforward:
  - RC-style (advanced): `pid_min_feedforward.ts` ports the setpoint-change feedforward pipeline used by Betaflight for radio control inputs.
  - Web joystick: `pid_min_feedforward_web.ts` provides a simulator-focused pipeline for 60 Hz Gamepad API inputs.
- D-Max: `pid_min_dmax.ts` implements dynamic D-term scaling (boost) with per-axis state.
- Launch Control: `pid_min_launch_control.ts` implements pitch-only and full launch gating and setpoint manipulation.
- I-term Relax: `pid_min_iterm_relax.ts` implements minimal relax modes with LPF/HPF logic and deadband.

All modules follow a pure-functional pattern: updates return a next state without mutating inputs, which makes them predictable and easy to integrate into simulation loops.

## Web Joystick Feedforward (Converted from RC Feedforward)

File: `pid_min_feedforward_web.ts`

### Why a Web-specific Path?

- RC receivers often deliver high-rate, irregular frames with transport artifacts; Betaflight’s feedforward includes jitter attenuation and duplicate-packet handling.
- Web Gamepad inputs are local, normalized floats in `[-1, 1]` at ~60 Hz and generally stable. The main challenge is bridging sparse samples to faster control loops without losing stick responsiveness.

### Pipeline (Web)

1. Clamp joystick to `[-1, 1]`.
2. Optional shaping: apply `deadband` and cubic `expo` near center.
3. Exponential smoothing with `smoothingTauMs` to interpolate between 60 Hz samples across the control loop `dt`.
4. Compute derivative of the smoothed command and apply a PT1 lowpass with `derivativeCutoffHz`.
5. Map smoothed command to `targetRateDegS = cmdSmooth * maxRateDegS[axis]`.
6. Feedforward F-term: `F = derivativeGain * dCmdLpf * maxRateDegS[axis]` (units deg/s).
7. Return both `F` and `targetRateDegS` for clean PID integration.

### Integration in `pid_min.ts`

- `PidMinConfig` adds `feedforwardMode?: 'simple' | 'advanced' | 'web'` and `webFeedforwardRuntime?: PidMinWebFeedforwardRuntime`.
- `pidMinUpdateUnified(...)` accepts `webCtx?: { joystick: number }` per axis.
- When in `'web'` mode and runtime + `webCtx` provided:
  - The module computes `targetRateDegS` and overrides the PID setpoint before launch gating.
  - F-term is summed if `useFeedforward` is true and launch does not disable F.
- A convenience 3-axis wrapper is provided: `pidMinUpdateWebAll(...)` (see file for signature).

### dt-first Config Helpers

- `normalizePidMinConfigForWeb(...)`: fills in sensible defaults for a web simulator and ensures parameters are set without hard-coding Betaflight’s 2 kHz assumptions. All filters compute from `dt`.
- `normalizePidMinLaunchConfig(...)`: sets defaults for Launch Control including rate limits and angle window; supports overrides.

### Web Convenience Wrappers

- `pidMinUpdateWebAllWithAxisDeflection(...)`: same as `pidMinUpdateWebAll(...)` but accepts `rcDeflection` per axis. Use this when Launch Control mode is `Full` so independent sticks affect launch setpoints for roll/pitch/yaw.
- `pidMinUpdateWebAllWithStage(...)`: wires in the launch-stage detector to compute `launchActive` from altitude/vertical speed with debounced hysteresis and table-start support, then calls the per-axis wrapper. Returns PID outputs/state plus the next stage state and `launchActive`.

### Controller Wrapper (React / R3F)

To simplify integration, use `web_controller.ts` which hides the fixed-step accumulator and provides a single `stepFrame(...)` you call from `useFrame`:

```ts
import { useMemo } from 'react';
import { useFrame } from '@react-three/fiber';
import { makeWebFlightController } from './web_controller';
import {
	normalizePidMinConfigForWeb,
	normalizePidMinLaunchConfig,
	PidMinLaunchMode,
} from './pid_min';

const controller = useMemo(
	() =>
		makeWebFlightController({
			coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0.0 },
			cfg: normalizePidMinConfigForWeb(
				{ feedforwardMode: 'web' },
				{ controlRateHz: 300, inputRateHz: 60 }
			),
			launch: normalizePidMinLaunchConfig({
				mode: PidMinLaunchMode.Full,
				angleLimitDeg: 25,
			}),
			controlRateHz: 300,
		}),
	[]
);

useFrame((state) => {
	const frameDt = state.clock.getDelta();
	const joystick = readGamepadAxes(); // { roll, pitch, yaw } in [-1,1]
	const gyro = getSimulatedGyroDegS(); // current angular rates from your physics
	const sensors = { armed, altitude, verticalSpeed, throttle };

	const snap = controller.stepFrame({
		frameDt,
		gyro,
		joystick,
		sensors, // omit to skip stage detector
		rcDeflectionAxis: joystick,
		pitchAngleDeg: 0,
		trimPitchDeg: 0,
	});

	applyToPhysics(snap.outputs, controller.controlDt);
});
```

Notes:

- `controller.controlDt` is the fixed control step (e.g., `1/300 s`).
- When `sensors` are provided, the wrapper uses `pidMinUpdateWebAllWithStage(...)` and returns `snap.stage` and `snap.launchActive`.
- Without `sensors`, the wrapper uses `pidMinUpdateWebAllWithAxisDeflection(...)` when `Full` mode, otherwise `pidMinUpdateWebAll(...)`.
- Joystick can be sampled each frame; internal smoothing bridges 60 Hz to the faster control rate.

### Suggested Defaults

## Framework-agnostic Control Loop

File: `web_control_loop.ts`

Runs the PID at a fixed rate (default 60 Hz) fully decoupled from rendering. You provide small callbacks to read joystick/gyro and consume PID outputs. Optional launch-stage updates are delivered via a telemetry callback.

Example:

```ts
import { startWebControlLoop } from './web_control_loop';
import {
	normalizePidMinConfigForWeb,
	normalizePidMinLaunchConfig,
	PidMinLaunchMode,
} from './pid_min';

const handle = startWebControlLoop({
	coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0.0 },
	cfg: normalizePidMinConfigForWeb(
		{ feedforwardMode: 'web' },
		{ controlRateHz: 60, inputRateHz: 60 }
	),
	launch: normalizePidMinLaunchConfig({
		mode: PidMinLaunchMode.Full,
		angleLimitDeg: 25,
	}),
	controlRateHz: 300, // e.g. 300–500 Hz for higher fidelity
	inputRateHz: 60, // typical Gamepad rate; feedforward bridges to control
	scheduler: 'raf', // prefer RAF when available for smoother timing
	maxStepsPerTick: 8, // optional safety limit when catching up
	readJoystick: () => readGamepadAxes(), // { roll, pitch, yaw }
	readGyro: () => getSimGyroDegS(), // { roll, pitch, yaw } deg/s
	readSensors: () => ({ armed, altitude, verticalSpeed, throttle }),
	onOutputs: (outputs, dt) => applyToPhysics(outputs, dt),
	onStageUpdate: (stage, launchActive) => {
		/* telemetry, UI, logging */
	},
});

// Later when stopping the sim:
// handle.stop();
```

Notes:

- The loop uses drift-compensated timing to keep the control step near `1/controlRateHz`.
- When `scheduler: 'raf'` (default if available), the loop synchronizes with the browser’s frame clock but runs multiple fixed control steps per RAF tick as needed.
- `inputRateHz` informs the web feedforward smoothing so 60 Hz stick input maps cleanly into higher-rate PID updates.
- `maxStepsPerTick` can prevent unbounded catch-up if the tab stalls; tune based on your physics budget.
- Joystick sampling at 60 Hz is bridged by the web feedforward smoothing; running the PID at 60 Hz is fine if you prefer simplicity.
- Rendering can subscribe to physics state independently; no framework integration is required.

- `smoothingTauMs`: 20–40 ms
- `derivativeCutoffHz`: 20–40 Hz
- `derivativeGain`: ~1.0
- `deadband`: 0.01–0.03
- `expo`: 0.1–0.3
- `maxRateDegS`: roll/pitch 600, yaw 400

### Why No RC Jitter Attenuation?

RC jitter attenuation targets RX transport artifacts (frame variance, duplicates, quantization). The web joystick’s 60 Hz steps are primarily intentional user motion, and the smoothing + derivative PT1 already handles the relevant discontinuities. Extra jitter filtering would mostly add latency.

## RC-style Feedforward (Advanced)

File: `pid_min_feedforward.ts`

- Ports the Betaflight rate-mode feedforward pipeline, including:
  - Handling duplicate packets and irregular RX timing.
  - Jitter attenuation on setpoint changes.
  - Boost behavior and optional max rate limiting (axis-specific).
  - Yaw hold and transition logic near center.
- Integrated into `pid_min.ts` when `feedforwardMode === 'advanced'` with `ffCtx` providing `{ rcCmd, rxRateHz, rxIntervalUs, maxRcRate }`.
- Use this for faithful RC simulations or when your inputs behave like an RX link.

## D-Max (Dynamic D-term Boost)

File: `pid_min_dmax.ts`

### Conversion Approach

- Compute a multiplier for the D-term based on gyro change rate (`deltaGyroDt`) and setpoint delta.
- Maintain per-axis filter state to avoid abrupt changes; state is returned each update.
- Parameters mirror Betaflight’s config: percentage boost per axis, gain, advance, and optional range/lowpass cutoffs.

### Integration

- In `pid_min.ts`, `computeDmaxMultiplier(...)` returns `{ multiplier, state }`.
- The D-term is scaled by `multiplier` when not gated by launch control.

## Launch Control

File: `pid_min_launch_control.ts`

### Conversion Approach

- Consolidate launch behavior into `computeLaunchEffects(...)`:
  - Manipulate setpoint per axis
  - Gate P/I/D/F contributions based on mode (PitchOnly vs Full)
  - Override Ki during launch
  - Apply yaw-specific I-term limit in full mode
  - Enforce non-negative pitch I where applicable

### Integration

- `pid_min.ts` computes `launchEffects` before PID terms:
  - Applies `launchEffects.setpoint` to the target
  - Honors `disableP/I/D/Feedforward` flags
  - Uses `effectiveKi` and `yawItermLimit` when set

## I-term Relax (Minimal)

File: `pid_min_iterm_relax.ts`

### Conversion Approach

- Implement two relax modes similar to Betaflight:
  - Setpoint mode: scale I accumulation when setpoint changes
  - Gyro mode: reconstruct error using LPF/HPF with deadband
- Return the relaxed I-error and next lowpassed setpoint for consistent behavior.

### Integration

- In `pid_min.ts`, `computeItermRelax(...)` provides:
  - `iErrorRate` used for I accumulation with leak and clamping
  - `nextPrevSetpointLpf` carried forward in state

## Pure-functional Style

- Each module exports state-creation helpers (e.g., `makePidMinFeedforwardState`) and pure update functions that return `{ value, state }` or `{ output, state }`.
- `PidMinState` carries only the minimal per-axis state needed for the next tick (integrator, LPF states, previous setpoints, D-Max, optional feedforward states).
- This pattern simplifies deterministic simulation and eases unit testing.

## Example (Web Mode)

```ts
import {
	PidMinCoefficients,
	PidMinConfig,
	PidMinState,
	pidMinUpdateWebAll,
} from './pid_min';
import { PidMinItermRelaxType } from './pid_min_iterm_relax';
import { normalizePidMinWebFeedforwardRuntime } from './pid_min_feedforward_web';

const loopHz = 500;
const dt = 1 / loopHz;

const c: PidMinCoefficients = { Kp: 0.9, Ki: 0.3, Kd: 0.02, Kf: 0 };
const webRt = normalizePidMinWebFeedforwardRuntime({
	inputRateHz: 60,
	controlRateHz: loopHz,
	deadband: 0.02,
	expo: 0.2,
	smoothingTauMs: 25,
	derivativeGain: 1.0,
	derivativeCutoffHz: 30,
	maxRateDegS: { roll: 600, pitch: 600, yaw: 400 },
});

const cfg: PidMinConfig = normalizePidMinConfigForWeb(
	{ feedforwardMode: 'web' },
	webRt
);

let state = {
	roll: new PidMinState(),
	pitch: new PidMinState(),
	yaw: new PidMinState(),
};

function step(
	gyro: { roll: number; pitch: number; yaw: number },
	joystick: { roll: number; pitch: number; yaw: number }
) {
	const { outputs, state: next } = pidMinUpdateWebAll(
		c,
		cfg,
		/* lc */ null,
		state,
		gyro,
		joystick,
		dt,
		/* launchActive */ false,
		/* rcDeflection */ 0,
		/* currentPitchAngleDeg */ 0,
		/* trimPitchDeg */ 0
	);
	state = next;
	return outputs;
}

// Example (Web Mode + Launch Stage + Full-mode launch)
// Assuming you collect altitude, verticalSpeed, throttle, and optional accelZ
import {
	makeDefaultLaunchStageConfig,
	makeLaunchStageState,
} from './pid_min_launch_stage';

const stageCfg = makeDefaultLaunchStageConfig();
const lc = normalizePidMinLaunchConfig({
	enabled: true,
	mode: PidMinLaunchMode.Full,
	angleLimitDeg: 30,
	kiOverride: 0.15,
});
let stageState = makeLaunchStageState();

function stepWithStage(
	gyro: { roll: number; pitch: number; yaw: number },
	joystick: { roll: number; pitch: number; yaw: number },
	sensors: {
		armed: boolean;
		altitude: number;
		verticalSpeed: number;
		throttle: number;
		motorsSpinning?: boolean;
		accelZ?: number;
	}
) {
	const {
		outputs,
		state: nextState,
		stage: nextStage,
		launchActive,
	} = pidMinUpdateWebAllWithStage(
		c,
		cfg,
		lc,
		stageCfg,
		stageState,
		state,
		gyro,
		joystick,
		dt,
		sensors,
		/* rcDeflectionAxis */ joystick,
		/* currentPitchAngleDeg */ 0,
		/* trimPitchDeg */ 0
	);
	state = nextState;
	stageState = nextStage;
	// launchActive indicates current stage according to detector
	return outputs;
}
```

## Validation and Testing

- Log intermediate values (e.g., smoothed commands, derivatives, multipliers) to verify behavior matches expectations.
- Start with axis-specific step tests and ramps at known joystick inputs.
- Tune feedforward and D-Max parameters cautiously; keep `dt` and loop rate consistent.

## Roadmap

- Optional adapters to switch between `'web'` and `'advanced'` feedforward at runtime.
- Extended per-axis configuration for web input shaping (deadband/expo).
- Unit tests for each module with representative scenarios.
