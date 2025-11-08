// Example usage: build parameters from Betaflight profile and run pid_min
import { PidMinAxis } from './pid_min_launch_control';
import { PidMinState, pidMinInit, pidMinUpdateUnified } from './pid_min';
import { buildParamsForCurrent, changePidProfile } from './profiles';

// Select profile 0 (defaults). Change if needed.
changePidProfile(0);

// Prepare axis-specific coefficients and config
const { coeff, cfg, dmax } = buildParamsForCurrent(PidMinAxis.Pitch);
if (dmax) cfg.dMax = dmax;

// Create state
const state = new (PidMinState)();
pidMinInit(state);

// Example update
const dt = 0.002; // 2 ms loop
const setpoint = 0; // deg/s target
const gyro = 0; // deg/s measured
const launchActive = false;
const rcDeflection = 0;
const currentPitchAngleDeg = 0;
const trimPitchDeg = 0;

const { output, state: next } = pidMinUpdateUnified(
  coeff,
  cfg,
  null,
  state,
  PidMinAxis.Pitch,
  setpoint,
  gyro,
  dt,
  launchActive,
  rcDeflection,
  currentPitchAngleDeg,
  trimPitchDeg
);

// The PID output for this tick
console.log('PID output', output);
// Next state for subsequent updates
// eslint-disable-next-line no-unused-vars
const nextState = next;