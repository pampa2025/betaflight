# Gamepad Utilities

Helpers to read joystick axes and throttle from the browser Gamepad API in a framework‑agnostic way.

## Quick Start

```ts
import { readGamepadAxes, readGamepadThrottle } from './web_gamepad';

// Roll/Pitch/Yaw reading with defaults:
// roll=RSX(2), pitch=RSY(3 inverted), yaw=LSX(0)
const rpy = readGamepadAxes();

// Throttle from LSY (axis 1) mapped to [0..1]
const throttle = readGamepadThrottle();
```

## Configuration

```ts
import { normalizeGamepadReadConfig } from './web_gamepad';

const cfg = normalizeGamepadReadConfig({
	// Axis indices depend on controller/driver. Defaults target common pads:
	mapping: { roll: 2, pitch: 3, yaw: 0 },
	// Invert per axis (pitch Y is typically inverted):
	invert: { roll: false, pitch: true, yaw: false },
	// Small deadband around center to remove stick noise:
	deadband: 0.02,
	// Optional cubic expo (0..1). Defaults to 0 to let feedforward handle expo.
	expo: 0.0,
});

const rpyCustom = readGamepadAxes(cfg);
```

Throttle config:

```ts
import {
	normalizeGamepadThrottleConfig,
	readGamepadThrottle,
} from './web_gamepad';

const tcfg = normalizeGamepadThrottleConfig({
	axis: 1,
	invert: false,
	deadband: 0.0,
});
const throttle = readGamepadThrottle(tcfg);
```

## With the Control Loop

```ts
import { startWebControlLoop } from './web_control_loop';
import { readGamepadAxes, readGamepadThrottle } from './web_gamepad';

const handle = startWebControlLoop({
	readJoystick: () => readGamepadAxes(),
	readGyro: () => ({ roll: 0, pitch: 0, yaw: 0 }),
	readSensors: () => ({ throttle: readGamepadThrottle() }),
	onOutputs: (outputs, dt) => {
		// Apply outputs to your physics with fixed dt
		applyPhysics(outputs, dt);
	},
});
```

## Notes

- Fallbacks return zeros when no gamepad is connected or the API is unavailable.
- `offsets` allow subtracting per‑axis center bias before shaping.
- Clamp is enabled by default to keep axes in `[-1, 1]` and throttle in `[0, 1]`.

## Detect and Log Connected Gamepads

Use the detection/logger utility to print connect/disconnect events and optionally poll summaries.

```ts
import { startGamepadLogger, getConnectedGamepads } from './web_gamepad_detect';

// Start logger: scan immediately and poll summaries every second
const logger = startGamepadLogger({
	initialScan: true,
	pollIntervalMs: 1000,
	logAxes: true,
	logButtons: true,
});

// Stop later when you no longer need logs
// logger.stop();

// Enumerate connected pads programmatically
const pads = getConnectedGamepads();
pads.forEach((gp) => {
	console.info('Connected pad:', gp.index, gp.id, gp.mapping);
});
```

Notes:

- Some browsers only fire `gamepadconnected` after the user interacts (presses a button). The logger’s `initialScan` enumerates current pads regardless.
- Use `indexFilter: [0]` to restrict logging to a specific pad index.
