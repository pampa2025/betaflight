// Simple simulator test: mimic joystick operations and run PID (no launch mode).
// Runs a fixed-step loop and logs the PID outputs for inspection.

import { Axis3, PidMinCoefficients, PidMinConfig, PidMinState, pidMinUpdateWebAll, normalizePidMinConfigForWeb } from './pid_min';
import { normalizePidMinWebFeedforwardRuntime, pidMinWebFeedforwardUpdateAll, makePidMinWebFeedforwardMultiState, PidMinWebFeedforwardState } from './pid_min_feedforward_web';

type Scenario = {
  name: string;
  joystickAt: (t: number) => Axis3<number>;
};

function makeStepPitchScenario(stepTimeSec: number, magnitude: number): Scenario {
  return {
    name: `pitch-step-${magnitude}@${stepTimeSec}s`,
    joystickAt: (t: number) => ({ roll: 0, pitch: t >= stepTimeSec ? magnitude : 0, yaw: 0 }),
  };
}

function makeSineYawScenario(freqHz: number, amplitude: number): Scenario {
  return {
    name: `yaw-sine-${amplitude}@${freqHz}Hz`,
    joystickAt: (t: number) => ({ roll: 0, pitch: 0, yaw: amplitude * Math.sin(2 * Math.PI * freqHz * t) }),
  };
}

function makeRampRollScenario(durationSec: number, final: number): Scenario {
  return {
    name: `roll-ramp-to-${final}@${durationSec}s`,
    joystickAt: (t: number) => {
      const u = Math.max(0, Math.min(1, t / durationSec));
      return { roll: final * u, pitch: 0, yaw: 0 };
    },
  };
}

function makeCompositeScenario(): Scenario {
  // All axes active: roll ramp, pitch step, yaw sine
  return {
    name: 'composite-roll-ramp-pitch-step-yaw-sine',
    joystickAt: (t: number) => {
      const rollU = Math.max(0, Math.min(1, t / 2.0));
      const roll = 0.4 * rollU;               // ramp to +0.4 over 2s
      const pitch = t >= 1.0 ? 0.6 : 0.0;     // step to +0.6 at 1s
      const yaw = 0.5 * Math.sin(2 * Math.PI * 1.0 * t); // 1 Hz sine, amp 0.5
      return { roll, pitch, yaw };
    },
  };
}

export function runPidWebTest(opts?: {
  controlRateHz?: number;
  durationSec?: number;
  coeffs?: PidMinCoefficients;
  webRtOverrides?: Partial<Parameters<typeof normalizePidMinWebFeedforwardRuntime>[0]>;
  scenario?: Scenario;
  logEveryN?: number;        // console pretty-log frequency when csv is false
  csv?: boolean;             // when true, emit CSV rows
  csvHeaders?: boolean;      // when csv is true, print header row (default true)
  csvEveryN?: number;        // emit CSV every N steps (default 1)
  decimals?: number;         // CSV decimals (default 4)
}): void {
  const controlRateHz = opts?.controlRateHz ?? 60;
  const dt = 1 / controlRateHz;
  const durationSec = opts?.durationSec ?? 5;
  const steps = Math.max(1, Math.floor(durationSec * controlRateHz));
  const coeffs: PidMinCoefficients = opts?.coeffs ?? { Kp: 0.9, Ki: 0.3, Kd: 0.02, Kf: 0 };
  const webRt = normalizePidMinWebFeedforwardRuntime({
    inputRateHz: 60,
    controlRateHz,
    deadband: 0.02,
    expo: 0.2,
    smoothingTauMs: 25,
    derivativeGain: 1.0,
    derivativeCutoffHz: 30,
    maxRateDegS: { roll: 600, pitch: 600, yaw: 400 },
    ...opts?.webRtOverrides,
  });
  const cfg: PidMinConfig = normalizePidMinConfigForWeb({
    feedforwardMode: 'web',
    webFeedforwardRuntime: webRt,
    pidSumLimit: 600, // clamp output sum to keep magnitudes realistic
  });
  let ffState: Axis3<PidMinWebFeedforwardState> = makePidMinWebFeedforwardMultiState();

  let state: Axis3<PidMinState> = { roll: new PidMinState(), pitch: new PidMinState(), yaw: new PidMinState() };
  let gyro: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };
  let outputs: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };
  const alphaPlant = 0.7; // crude plant LPF: gyro follows outputs

  const scenario = opts?.scenario ?? makeCompositeScenario();
  const logEveryN = Math.max(1, opts?.logEveryN ?? Math.floor(controlRateHz / 10)); // ~6 Hz logs
  const useCsv = !!opts?.csv;
  const csvEveryN = Math.max(1, opts?.csvEveryN ?? 1);
  const decimalsRaw = typeof opts?.decimals === 'number' ? opts!.decimals! : 4;
  const decimals = Math.max(0, Math.min(12, decimalsRaw));

  if (!useCsv) {
    console.info(`[pid-web-test] scenario=${scenario.name} rate=${controlRateHz}Hz duration=${durationSec}s`);
  } else if (opts?.csvHeaders !== false) {
    // Header: t,joy_*,target_*,gyro_*,out_*
    console.log('t,joy_roll,joy_pitch,joy_yaw,target_roll,target_pitch,target_yaw,gyro_roll,gyro_pitch,gyro_yaw,out_roll,out_pitch,out_yaw');
  }

  for (let i = 0; i < steps; i++) {
    const t = i * dt;
    const joystick = scenario.joystickAt(t);

    const ffRes = pidMinWebFeedforwardUpdateAll(ffState, webRt, joystick, dt);
    ffState = ffRes.state;

    const res = pidMinUpdateWebAll(
      coeffs,
      cfg,
      /* lc */ null,
      state,
      gyro,
      joystick,
      dt,
      /* launchActive */ false,
      /* rcDeflection */ 0,
      /* currentPitchAngleDeg */ 0,
      /* trimPitchDeg */ 0,
    );

    state = res.state;
    outputs = res.outputs;

    // crude plant model: gyro follows outputs with LPF
    gyro = {
      roll: alphaPlant * gyro.roll + (1 - alphaPlant) * outputs.roll,
      pitch: alphaPlant * gyro.pitch + (1 - alphaPlant) * outputs.pitch,
      yaw: alphaPlant * gyro.yaw + (1 - alphaPlant) * outputs.yaw,
    };

    if (useCsv) {
      if (i % csvEveryN === 0) {
        console.log([
          t.toFixed(decimals),
          joystick.roll.toFixed(decimals), joystick.pitch.toFixed(decimals), joystick.yaw.toFixed(decimals),
          ffRes.targetRatesDegS.roll.toFixed(decimals), ffRes.targetRatesDegS.pitch.toFixed(decimals), ffRes.targetRatesDegS.yaw.toFixed(decimals),
          gyro.roll.toFixed(decimals), gyro.pitch.toFixed(decimals), gyro.yaw.toFixed(decimals),
          outputs.roll.toFixed(decimals), outputs.pitch.toFixed(decimals), outputs.yaw.toFixed(decimals),
        ].join(','));
      }
    } else if (i % logEveryN === 0) {
      console.info(
        `[t=${t.toFixed(2)}] joy={${joystick.roll.toFixed(2)}, ${joystick.pitch.toFixed(2)}, ${joystick.yaw.toFixed(2)}} ` +
          `gyro={${gyro.roll.toFixed(2)}, ${gyro.pitch.toFixed(2)}, ${gyro.yaw.toFixed(2)}} ` +
          `out={${outputs.roll.toFixed(2)}, ${outputs.pitch.toFixed(2)}, ${outputs.yaw.toFixed(2)}}`
      );
    }
  }

  if (!useCsv) console.info('[pid-web-test] done');
}

// If run directly (bundlers or ts-node), execute default test
declare const require: any; // avoid TS DOM typings requirements
declare const module: any;
if (typeof require !== 'undefined' && typeof module !== 'undefined' && require.main === module) {
  runPidWebTest({ csv: true, csvHeaders: true });
}