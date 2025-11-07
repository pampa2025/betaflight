// High-level wrapper for React / React Three Fiber
// Provides a clean single-call per frame that hides the fixed-step accumulator
// and wires the web joystick + optional launch stage detector to the PID update.
//
// Usage sketch (React Three Fiber):
//   const controller = useMemo(() => makeWebFlightController({
//     coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0.0 },
//     cfg: normalizePidMinConfigForWeb({ feedforwardMode: 'web' }, { controlRateHz: 300, inputRateHz: 60 }),
//     launch: normalizePidMinLaunchConfig({ mode: PidMinLaunchMode.Full, angleLimitDeg: 25 }),
//     stage: makeDefaultLaunchStageConfig(),
//     controlRateHz: 300,
//   }), []);
//
//   useFrame((state) => {
//     const dt = state.clock.getDelta();
//     const joystick = readGamepadAxes(); // returns { roll, pitch, yaw } in [-1,1]
//     const gyro = getSimulatedGyroDegS(); // current angular rates from your physics
//     const sensors = { armed, altitude, verticalSpeed, throttle };
//     const snap = controller.stepFrame({ frameDt: dt, gyro, joystick, sensors, pitchAngleDeg, trimPitchDeg });
//     applyToPhysics(snap.outputs, controller.controlDt);
//   });
//
// If you don’t have stage inputs, omit `sensors` and the controller will call the
// non-stage web wrapper.

import { Axis3, PidMinCoefficients, PidMinConfig, PidMinState, normalizePidMinConfigForWeb, pidMinUpdateWebAll, pidMinUpdateWebAllWithAxisDeflection, pidMinUpdateWebAllWithStage, normalizePidMinLaunchConfig, PidMinLaunchConfig } from './pid_min';
import { PidMinLaunchStageConfig, PidMinLaunchStageState, makeDefaultLaunchStageConfig } from './pid_min_launch_stage';
import { PidMinLaunchMode } from './pid_min_launch_control';

export type StageInputs = {
  armed: boolean;
  altitude: number;
  verticalSpeed: number;
  throttle: number;
  motorsSpinning?: boolean;
  accelZ?: number;
};

export type WebControllerSnapshot = {
  outputs: Axis3<number>;
  state: Axis3<PidMinState>;
  stage?: PidMinLaunchStageState;
  launchActive?: boolean;
};

export type WebFlightController = {
  readonly controlDt: number;
  stepFrame(args: {
    frameDt: number;
    gyro: Axis3<number>;
    joystick: Axis3<number>;
    sensors?: StageInputs;
    rcDeflectionAxis?: Axis3<number>; // when using FULL mode and independent sticks
    pitchAngleDeg?: number;
    trimPitchDeg?: number;
  }): WebControllerSnapshot;
};

export function makeWebFlightController(args: {
  coeffs: PidMinCoefficients;
  cfg: PidMinConfig | Partial<PidMinConfig>;
  launch?: PidMinLaunchConfig | Partial<PidMinLaunchConfig> | null;
  stage?: PidMinLaunchStageConfig | null;
  controlRateHz?: number;
  initialState?: Axis3<PidMinState>;
}): WebFlightController {
  const coeffs = args.coeffs;
  const cfg = (isFullCfg(args.cfg) ? args.cfg as PidMinConfig : normalizePidMinConfigForWeb(args.cfg as Partial<PidMinConfig>));
  const lc = args.launch == null ? null : normalizePidMinLaunchConfig(args.launch as Partial<PidMinLaunchConfig>);
  const stageCfg = args.stage ?? makeDefaultLaunchStageConfig();
  const controlRateHz = args.controlRateHz ?? 300;
  const controlDt = 1 / controlRateHz;

  // Persistent state
  let pidState: Axis3<PidMinState> = args.initialState ?? { roll: new PidMinState(), pitch: new PidMinState(), yaw: new PidMinState() };
  let stageState: PidMinLaunchStageState = new PidMinLaunchStageState();
  let lastOutputs: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };
  let acc = 0;

  function stepFrame(params: {
    frameDt: number;
    gyro: Axis3<number>;
    joystick: Axis3<number>;
    sensors?: StageInputs;
    rcDeflectionAxis?: Axis3<number>;
    pitchAngleDeg?: number;
    trimPitchDeg?: number;
  }): WebControllerSnapshot {
    const frameDt = params.frameDt > 0 ? params.frameDt : controlDt;
    const gyro = params.gyro;
    const joystick = params.joystick;
    const sensors = params.sensors;
    const rcDeflectionAxis = params.rcDeflectionAxis ?? joystick; // default: use joystick deflection per axis
    const pitchAngleDeg = params.pitchAngleDeg ?? 0;
    const trimPitchDeg = params.trimPitchDeg ?? 0;

    acc += frameDt;

    // Run fixed-step updates; keep at least one step per frame for stability
    let localStage: PidMinLaunchStageState | undefined = undefined;
    let localLaunchActive: boolean | undefined = undefined;

    while (acc >= controlDt) {
      if (sensors) {
        const res = pidMinUpdateWebAllWithStage(
          coeffs,
          cfg,
          lc ?? undefined,
          stageCfg,
          stageState,
          pidState,
          gyro,
          joystick,
          controlDt,
          {
            armed: sensors.armed,
            altitude: sensors.altitude,
            verticalSpeed: sensors.verticalSpeed,
            throttle: sensors.throttle,
            motorsSpinning: sensors.motorsSpinning,
            accelZ: sensors.accelZ,
          },
          rcDeflectionAxis,
          pitchAngleDeg,
          trimPitchDeg
        );
        pidState = res.state;
        stageState = res.stage;
        lastOutputs = res.outputs;
        localStage = res.stage;
        localLaunchActive = res.launchActive;
      } else if (lc && lc.mode === PidMinLaunchMode.Full) {
        const res = pidMinUpdateWebAllWithAxisDeflection(
          coeffs,
          cfg,
          lc,
          pidState,
          gyro,
          joystick,
          controlDt,
          /* launchActive */ false,
          rcDeflectionAxis,
          pitchAngleDeg,
          trimPitchDeg
        );
        pidState = res.state;
        lastOutputs = res.outputs;
      } else {
        const res = pidMinUpdateWebAll(
          coeffs,
          cfg,
          lc ?? undefined,
          pidState,
          gyro,
          joystick,
          controlDt,
          /* launchActive */ false,
          /* rcDeflection */ rcDeflectionAxis.pitch,
          pitchAngleDeg,
          trimPitchDeg
        );
        pidState = res.state;
        lastOutputs = res.outputs;
      }

      acc -= controlDt;
    }

    return { outputs: lastOutputs, state: pidState, stage: localStage ?? stageState, launchActive: localLaunchActive };
  }

  return { controlDt, stepFrame };
}

function isFullCfg(cfg: PidMinConfig | Partial<PidMinConfig>): cfg is PidMinConfig {
  // Heuristic: check for required fields that won’t exist together in a partial
  return (
    typeof (cfg as any).pidSumLimit === 'number' &&
    typeof (cfg as any).itermLimit === 'number' &&
    typeof (cfg as any).integratorLeak === 'number' &&
    typeof (cfg as any).dLowpassCutoffHz === 'number' &&
    !!(cfg as any).iRelax
  );
}