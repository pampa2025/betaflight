// Framework-agnostic fixed-step control loop for web simulators
// Decouples PID updates from rendering; targets a stable loop rate (default 60 Hz)
// and bridges 60 Hz Gamepad input to PID via the web feedforward path.
//
// Usage (no React required):
//   const handle = startWebControlLoop({
//     coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0.0 },
//     cfg: normalizePidMinConfigForWeb({ feedforwardMode: 'web' }, { controlRateHz: 60, inputRateHz: 60 }),
//     launch: normalizePidMinLaunchConfig({ mode: PidMinLaunchMode.Full, angleLimitDeg: 25 }),
//     controlRateHz: 60,
//     readJoystick: () => readGamepadAxes(),           // { roll, pitch, yaw } in [-1,1]
//     readGyro: () => getSimGyroDegS(),                // { roll, pitch, yaw } deg/s
//     readSensors: () => ({ armed, altitude, verticalSpeed, throttle }),
//     onOutputs: (outputs, dt) => applyToPhysics(outputs, dt),
//     onStageUpdate: (stage, launchActive) => {/* optional telemetry */},
//   });
//   // Later: handle.stop();

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

export type WebControlLoopOptions = {
  coeffs: PidMinCoefficients;
  cfg: PidMinConfig | Partial<PidMinConfig>;
  launch?: PidMinLaunchConfig | Partial<PidMinLaunchConfig> | null;
  stage?: PidMinLaunchStageConfig | null;
  controlRateHz?: number; // default 60
  inputRateHz?: number;   // default 60, typical for Gamepad API
  scheduler?: 'raf' | 'timeout'; // default: use RAF if available, else timeout
  maxStepsPerTick?: number; // optional safety limit when catching up
  readJoystick: () => Axis3<number>;
  readGyro: () => Axis3<number>;
  readSensors?: () => StageInputs;
  pitchAngleDeg?: () => number; // optional attitude source
  trimPitchDeg?: () => number;  // optional trim source
  onOutputs: (outputs: Axis3<number>, dt: number) => void;
  onStageUpdate?: (stage: PidMinLaunchStageState, launchActive: boolean) => void;
  initialState?: Axis3<PidMinState>;
};

export type WebControlLoopHandle = {
  stop: () => void;
  readonly isRunning: boolean;
  readonly controlDt: number;
};

export function startWebControlLoop(opts: WebControlLoopOptions): WebControlLoopHandle {
  const coeffs = opts.coeffs;
  const controlRateHz = opts.controlRateHz ?? 60;
  const cfg = (isFullCfg(opts.cfg)
    ? (opts.cfg as PidMinConfig)
    : normalizePidMinConfigForWeb(opts.cfg as Partial<PidMinConfig>, { controlRateHz, inputRateHz: opts.inputRateHz ?? 60 })
  );
  const lc = opts.launch == null ? null : normalizePidMinLaunchConfig(opts.launch as Partial<PidMinLaunchConfig>);
  const stageCfg = opts.stage ?? makeDefaultLaunchStageConfig();
  const controlDt = 1 / controlRateHz;

  let pidState: Axis3<PidMinState> = opts.initialState ?? { roll: new PidMinState(), pitch: new PidMinState(), yaw: new PidMinState() };
  let stageState: PidMinLaunchStageState = new PidMinLaunchStageState();
  let isRunning = true;

  // Fixed-step scheduler with drift compensation; prefer RAF for high rates
  const intervalMs = 1000 / controlRateHz;
  let accMs = 0;
  const maxSteps = typeof opts.maxStepsPerTick === 'number' && opts.maxStepsPerTick! > 0 ? opts.maxStepsPerTick! : Number.POSITIVE_INFINITY;

  const canUseRaf = typeof (globalThis as any).requestAnimationFrame === 'function' && typeof (globalThis as any).cancelAnimationFrame === 'function';
  const useRaf = opts.scheduler ? (opts.scheduler === 'raf') : canUseRaf;
  let timer: number | null = null;
  let rafId: number | null = null;

  const stepControl = () => {
    const joystick = opts.readJoystick();
    const gyro = opts.readGyro();
    const sensors = opts.readSensors ? opts.readSensors() : undefined;
    const pitchAngleDeg = opts.pitchAngleDeg ? opts.pitchAngleDeg() : 0;
    const trimPitchDeg = opts.trimPitchDeg ? opts.trimPitchDeg() : 0;

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
        joystick,
        pitchAngleDeg,
        trimPitchDeg
      );
      pidState = res.state;
      stageState = res.stage;
      opts.onOutputs(res.outputs, controlDt);
      if (opts.onStageUpdate) opts.onStageUpdate(res.stage, res.launchActive);
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
        joystick,
        pitchAngleDeg,
        trimPitchDeg
      );
      pidState = res.state;
      opts.onOutputs(res.outputs, controlDt);
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
        /* rcDeflection */ joystick.pitch,
        pitchAngleDeg,
        trimPitchDeg
      );
      pidState = res.state;
      opts.onOutputs(res.outputs, controlDt);
    }
  };

  if (useRaf) {
    let lastTs: number | null = null;
    const tickRaf = (ts: number) => {
      if (!isRunning) return;
      if (lastTs == null) lastTs = ts;
      accMs += (ts - lastTs);
      lastTs = ts;
      let stepsRun = 0;
      while (accMs >= intervalMs && stepsRun < maxSteps) {
        stepControl();
        accMs -= intervalMs;
        stepsRun++;
      }
      rafId = (globalThis as any).requestAnimationFrame(tickRaf) as number;
    };
    rafId = (globalThis as any).requestAnimationFrame(tickRaf) as number;
  } else {
    let prev = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
    const tickTimeout = () => {
      if (!isRunning) return;
      const now = (typeof performance !== 'undefined' && performance.now) ? performance.now() : Date.now();
      accMs += (now - prev);
      prev = now;
      let stepsRun = 0;
      while (accMs >= intervalMs && stepsRun < maxSteps) {
        stepControl();
        accMs -= intervalMs;
        stepsRun++;
      }
      const nextDelay = Math.max(0, intervalMs - accMs);
      timer = setTimeout(tickTimeout, nextDelay) as unknown as number;
    };
    timer = setTimeout(tickTimeout, intervalMs) as unknown as number;
  }

  const stop = () => {
    if (!isRunning) return;
    isRunning = false;
    if (timer != null) clearTimeout(timer);
    if (rafId != null && canUseRaf) (globalThis as any).cancelAnimationFrame(rafId);
  };

  return { stop, isRunning, controlDt };
}

function isFullCfg(cfg: PidMinConfig | Partial<PidMinConfig>): cfg is PidMinConfig {
  return (
    typeof (cfg as any).pidSumLimit === 'number' &&
    typeof (cfg as any).itermLimit === 'number' &&
    typeof (cfg as any).integratorLeak === 'number' &&
    typeof (cfg as any).dLowpassCutoffHz === 'number' &&
    !!(cfg as any).iRelax
  );
}