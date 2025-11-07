(() => {
  // src/main/flight/utils.ts
  function clampf(x, lo, hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
  }
  function pt1Alpha(cutoffHz, dt) {
    if (cutoffHz <= 0 || dt <= 0) return 1;
    const rc = 1 / (2 * Math.PI * cutoffHz);
    return dt / (rc + dt);
  }
  function fapplyDeadband(x, deadband) {
    if (deadband <= 0) return x;
    if (x > deadband) return x - deadband;
    if (x < -deadband) return x + deadband;
    return 0;
  }
  function scaleRangef(x, inLo, inHi, outLo, outHi) {
    const spanIn = inHi - inLo;
    if (spanIn === 0) return outLo;
    const t = clampf((x - inLo) / spanIn, 0, 1);
    return outLo + t * (outHi - outLo);
  }

  // src/main/flight/pid_min_iterm_relax.ts
  function computeItermRelax(setpoint, gyro, dt, prevSetpointLpf, cfg) {
    const errorRate = setpoint - gyro;
    let iErrorRate = errorRate;
    let nextPrevSetpointLpf = prevSetpointLpf;
    let relaxFactor = 1;
    let setpointLpf = prevSetpointLpf;
    let setpointHpf = 0;
    if (cfg.iRelaxEnabled && cfg.iRelaxType !== 0 /* Off */) {
      const alphaRelax = cfg.iRelaxCutoffHz > 0 ? pt1Alpha(cfg.iRelaxCutoffHz, dt) : 1;
      setpointLpf = prevSetpointLpf + alphaRelax * (setpoint - prevSetpointLpf);
      setpointHpf = Math.abs(setpoint - setpointLpf);
      nextPrevSetpointLpf = setpointLpf;
      if (cfg.iRelaxType === 1 /* Setpoint */) {
        const threshold = cfg.iRelaxSetpointThreshold > 0 ? cfg.iRelaxSetpointThreshold : 40;
        relaxFactor = Math.max(0, 1 - setpointHpf / threshold);
        iErrorRate *= relaxFactor;
      } else if (cfg.iRelaxType === 2 /* Gyro */) {
        iErrorRate = fapplyDeadband(setpointLpf - gyro, setpointHpf);
      }
    }
    return {
      iErrorRate,
      nextPrevSetpointLpf,
      relaxFactor,
      setpointLpf,
      setpointHpf
    };
  }

  // src/main/flight/pid_min_launch_control.ts
  var LAUNCH_CONTROL_MAX_RATE = 100;
  var LAUNCH_CONTROL_MIN_RATE = 5;
  var LAUNCH_CONTROL_ANGLE_WINDOW = 10;
  var LAUNCH_CONTROL_YAW_ITERM_LIMIT = 50;
  function pidMinApplyLaunchSetpoint(axis, lc, rcDeflection, currentPitchAngleDeg, trimPitchDeg) {
    if (!lc || !lc.enabled) return 0;
    if (lc.mode === 0 /* PitchOnly */ && axis !== 1 /* Pitch */) {
      return 0;
    }
    const stick = clampf(rcDeflection, -0.5, 0.5);
    const maxRate = lc.maxRateDegS ?? LAUNCH_CONTROL_MAX_RATE;
    const minRate = lc.minRateDegS ?? LAUNCH_CONTROL_MIN_RATE;
    const angleWindow = lc.angleWindowDeg ?? LAUNCH_CONTROL_ANGLE_WINDOW;
    let rate = maxRate * (stick * 2);
    if (axis === 1 /* Pitch */ && lc.angleLimitDeg > 0) {
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
  function computeLaunchEffects(axis, lc, launchActive, setpoint, rcDeflection, currentPitchAngleDeg, trimPitchDeg, ki) {
    if (!launchActive || !lc || !lc.enabled) {
      return {
        setpoint,
        effectiveKi: ki,
        disableD: false,
        disableFeedforward: false,
        disableP: false,
        disableI: false,
        yawItermLimit: null,
        enforcePitchINonNegative: false
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
    let yawItermLimit = null;
    let enforcePitchINonNegative = false;
    if (lc.mode === 0 /* PitchOnly */) {
      if (axis === 0 /* Roll */ || axis === 2 /* Yaw */) {
        disableP = true;
        disableI = true;
      }
      if (axis === 1 /* Pitch */) {
        enforcePitchINonNegative = true;
      }
      if (axis === 2 /* Yaw */) {
        yawItermLimit = 0;
      }
    } else if (lc.mode === 1 /* Full */) {
      if (axis === 2 /* Yaw */) {
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
      enforcePitchINonNegative
    };
  }

  // src/main/flight/pid_min_dmax.ts
  var D_MAX_RANGE_HZ = 85;
  var D_MAX_LOWPASS_HZ = 35;
  var D_MAX_GYRO_GAIN_FACTOR = 8e-5;
  var D_MAX_SETPOINT_GAIN_FACTOR = 8e-5;
  var CUTOFF_CORRECTION_PT2 = 1.553773974;
  function makeDefaultDmaxState() {
    return { range: { s1: 0, s2: 0 }, lowpass: { s1: 0, s2: 0 } };
  }
  function pt2ApplyCascade(prev, x, cutoffHz, dt) {
    if (dt <= 0 || cutoffHz <= 0) {
      const y = x;
      return { y, state: { s1: y, s2: y } };
    }
    const alpha = pt1Alpha(cutoffHz * CUTOFF_CORRECTION_PT2, dt);
    const s1 = prev.s1 + alpha * (x - prev.s1);
    const s2 = prev.s2 + alpha * (s1 - prev.s2);
    return { y: s2, state: { s1, s2 } };
  }
  function percentForAxis(axis, percents) {
    switch (axis) {
      case 0 /* Roll */:
        return percents.roll;
      case 1 /* Pitch */:
        return percents.pitch;
      case 2 /* Yaw */:
        return percents.yaw;
      default:
        return 1;
    }
  }
  function computeDmaxMultiplier(cfg, prev, inputs) {
    const rangeHz = cfg.rangeCutoffHz ?? D_MAX_RANGE_HZ;
    const lowpassHz = cfg.lowpassCutoffHz ?? D_MAX_LOWPASS_HZ;
    const dmaxLpfInv = lowpassHz > 0 ? 1 / lowpassHz : 0;
    const gyroGain = D_MAX_GYRO_GAIN_FACTOR * cfg.gain * dmaxLpfInv;
    const setpointGain = D_MAX_SETPOINT_GAIN_FACTOR * cfg.advance * dmaxLpfInv;
    const percent = percentForAxis(inputs.axis, cfg.dMaxPercent);
    let dMaxMultiplierPre = 1;
    let nextRange = prev.range;
    let nextLowpass = prev.lowpass;
    if (percent > 1 && cfg.enabled) {
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
      dMaxMultiplierPre += (percent - 1) * dMaxBoost;
      const lpOut2 = pt2ApplyCascade(
        prev.lowpass,
        dMaxMultiplierPre,
        lowpassHz,
        inputs.dt
      );
      nextLowpass = lpOut2.state;
      dMaxMultiplierPre = lpOut2.y;
      dMaxMultiplierPre = Math.min(dMaxMultiplierPre, percent);
      return {
        multiplier: dMaxMultiplierPre,
        state: { range: nextRange, lowpass: nextLowpass },
        debug: {
          gyroFactor: dMaxGyroFactor,
          setpointFactor: dMaxSetpointFactor,
          boost: dMaxBoost
        }
      };
    }
    const lpOut = pt2ApplyCascade(prev.lowpass, 1, lowpassHz, inputs.dt);
    nextLowpass = lpOut.state;
    return { multiplier: 1, state: { range: nextRange, lowpass: nextLowpass } };
  }

  // src/main/flight/pid_min_feedforward.ts
  function clampf2(x, lo, hi) {
    return Math.max(lo, Math.min(hi, x));
  }
  function pt1GainFromDelay(delaySec, dtSec) {
    if (delaySec <= 0) return 1;
    return dtSec / (delaySec + dtSec);
  }
  function pt1Update(prevY, x, k) {
    return prevY + k * (x - prevY);
  }
  function makePidMinFeedforwardState(rt) {
    const enumToWindow = [0, 2, 3, 4];
    const win = enumToWindow[clampf2(rt.feedforwardAveraging, 0, 3)];
    const buf = win > 0 ? new Array(win).fill(0) : [];
    return {
      prevRcCommand: 0,
      prevRcCommandDeltaAbs: 0,
      prevSetpoint: 0,
      prevSetpointSpeed: 0,
      prevSetpointSpeedDelta: 0,
      isPrevPacketDuplicate: false,
      prevRxInterval: 0,
      setpointSpeedFiltered: 0,
      setpointDeltaFiltered: 0,
      yawSetpointLpf: 0,
      avgSize: win,
      avgBuf: buf,
      avgIdx: 0,
      avgCount: 0,
      avgSum: 0,
      feedforwardRaw: 0,
      initialized: false
    };
  }
  function pidMinFeedforwardUpdate(prev, rt, axis, setpoint, rcCmd, currentRxRateHz, currentRxIntervalUs, maxRcRateAxis) {
    const rxInterval = currentRxIntervalUs * 1e-6;
    const rcCommandDelta = rcCmd - prev.prevRcCommand;
    let rcCommandDeltaAbs = Math.abs(rcCommandDelta);
    const isDuplicate = rcCommandDeltaAbs === 0;
    const setpointDelta = setpoint - prev.prevSetpoint;
    let rxRate = currentRxRateHz;
    let setpointSpeed = 0;
    let feedforward = 0;
    if (rt.feedforwardInterpolate) {
      const prevRxInterval = prev.prevRxInterval;
      if (!isDuplicate) {
        if (prev.isPrevPacketDuplicate) {
          rxRate = 1 / (rxInterval + prevRxInterval);
        }
        setpointSpeed = setpointDelta * rxRate;
      } else {
        if (!prev.isPrevPacketDuplicate) {
          if (Math.abs(setpoint) < 0.9 * maxRcRateAxis) {
            setpointSpeed = prev.prevSetpointSpeed + prev.prevSetpointSpeedDelta;
            rcCommandDeltaAbs = prev.prevRcCommandDeltaAbs;
          }
        } else {
          setpointSpeed = 0;
        }
      }
    } else {
      setpointSpeed = setpointDelta * currentRxRateHz;
    }
    let jitterAttenuator = ((rcCommandDeltaAbs + prev.prevRcCommandDeltaAbs) * 0.5 + 1) * rt.feedforwardJitterFactorInv;
    jitterAttenuator = Math.min(jitterAttenuator, 1);
    const dt = 1 / Math.max(1e-6, currentRxRateHz);
    const k = pt1GainFromDelay(rt.feedforwardSmoothFactor, dt);
    const setpointSpeedFiltered = pt1Update(prev.setpointSpeedFiltered, setpointSpeed, k);
    const prevSpeedForDelta = isDuplicate && prev.isPrevPacketDuplicate ? 0 : prev.prevSetpointSpeed;
    const setpointSpeedDeltaRaw = setpointSpeedFiltered - prevSpeedForDelta;
    const setpointDeltaFiltered = pt1Update(prev.setpointDeltaFiltered, setpointSpeedDeltaRaw, k);
    const feedforwardBoost = setpointDeltaFiltered * rxRate * rt.feedforwardBoostFactor;
    feedforward = setpointSpeedFiltered;
    if (axis === 0 /* Roll */ || axis === 1 /* Pitch */) {
      feedforward += feedforwardBoost;
      feedforward *= jitterAttenuator;
      if (rt.feedforwardMaxRateLimit !== 0 && feedforward * setpoint > 0) {
        const limit = (maxRcRateAxis - Math.abs(setpoint)) * rt.feedforwardMaxRateLimit;
        feedforward = limit > 0 ? clampf2(feedforward, -limit, limit) : 0;
      }
    } else {
      feedforward *= jitterAttenuator;
      const gain = pt1GainFromDelay(rt.feedforwardYawHoldTime, rxInterval);
      const yawSetpointLpfNext = pt1Update(prev.yawSetpointLpf, setpoint, gain);
      const feedforwardYawHold = rt.feedforwardYawHoldGain * (setpoint - yawSetpointLpfNext);
      feedforward += feedforwardYawHold;
    }
    const rcDeflectionAbs = maxRcRateAxis > 0 ? Math.abs(setpoint) / maxRcRateAxis : 0;
    const useTransition = rt.feedforwardTransition !== 0 && rcDeflectionAbs < rt.feedforwardTransition;
    if (useTransition) {
      feedforward *= rcDeflectionAbs * rt.feedforwardTransitionInv;
    }
    let avgSum = prev.avgSum;
    let avgIdx = prev.avgIdx;
    let avgCount = prev.avgCount;
    let avgBuf = prev.avgBuf.slice();
    if (prev.avgSize > 0) {
      avgSum -= avgBuf[avgIdx] || 0;
      avgBuf[avgIdx] = feedforward;
      avgSum += feedforward;
      avgIdx = (avgIdx + 1) % prev.avgSize;
      avgCount = Math.min(avgCount + 1, prev.avgSize);
      const avg = avgSum / avgCount;
      feedforward = avg;
    }
    const next = {
      prevRcCommand: rcCmd,
      prevRcCommandDeltaAbs: rcCommandDeltaAbs,
      prevSetpoint: setpoint,
      prevSetpointSpeed: setpointSpeedFiltered,
      prevSetpointSpeedDelta: setpointDeltaFiltered,
      isPrevPacketDuplicate: isDuplicate,
      prevRxInterval: rxInterval,
      setpointSpeedFiltered,
      setpointDeltaFiltered,
      yawSetpointLpf: axis === 2 /* Yaw */ ? pt1Update(prev.yawSetpointLpf, setpoint, pt1GainFromDelay(rt.feedforwardYawHoldTime, rxInterval)) : prev.yawSetpointLpf,
      avgSize: prev.avgSize,
      avgBuf,
      avgIdx,
      avgCount,
      avgSum,
      feedforwardRaw: feedforward,
      initialized: true
    };
    return { value: feedforward, state: next };
  }

  // src/main/flight/pid_min_feedforward_web.ts
  function normalizePidMinWebFeedforwardRuntime(inRt) {
    const inputRateHz = clampf3(inRt.inputRateHz ?? 60, 1, 1e3);
    const controlRateHz = clampf3(inRt.controlRateHz ?? 500, 50, 4e3);
    const deadband = clampf3(inRt.deadband ?? 0.02, 0, 0.2);
    const expo = clampf3(inRt.expo ?? 0.2, 0, 1);
    const smoothingTauMs = clampf3(inRt.smoothingTauMs ?? 25, 1, 500);
    const derivativeGain = inRt.derivativeGain ?? 1;
    const derivativeCutoffHz = clampf3(inRt.derivativeCutoffHz ?? 30, 0, 200);
    const mr = inRt.maxRateDegS ?? { roll: 600, pitch: 600, yaw: 400 };
    return {
      inputRateHz,
      controlRateHz,
      deadband,
      expo,
      smoothingTauMs,
      derivativeGain,
      derivativeCutoffHz,
      maxRateDegS: {
        roll: clampf3(mr.roll, 10, 2e3),
        pitch: clampf3(mr.pitch, 10, 2e3),
        yaw: clampf3(mr.yaw, 10, 2e3)
      }
    };
  }
  function makePidMinWebFeedforwardState() {
    return {
      cmdSmooth: 0,
      prevCmdSmooth: 0,
      dCmdLpf: 0,
      initialized: false
    };
  }
  function clampf3(x, lo, hi) {
    return Math.max(lo, Math.min(hi, x));
  }
  function applyDeadband(x, db) {
    if (db <= 0) return x;
    const m = Math.abs(x);
    if (m <= db) return 0;
    const s = Math.sign(x);
    return s * clampf3((m - db) / (1 - db), 0, 1);
  }
  function applyExpoCubic(x, expo) {
    if (expo <= 0) return x;
    return (1 - expo) * x + expo * x * x * x;
  }
  function pt1Alpha2(cutoffHz, dt) {
    if (cutoffHz <= 0) return 1;
    const rc = 1 / (2 * Math.PI * cutoffHz);
    return dt / (dt + rc);
  }
  function expSmoothingAlpha(tauMs, dt) {
    const tau = Math.max(1e-6, tauMs / 1e3);
    return 1 - Math.exp(-dt / tau);
  }
  function pidMinWebFeedforwardUpdate(prev, rt, axis, joystickCmd, dt) {
    const bounded = clampf3(joystickCmd, -1, 1);
    const deads = applyDeadband(bounded, rt.deadband);
    const shaped = applyExpoCubic(deads, rt.expo);
    const aSmooth = expSmoothingAlpha(rt.smoothingTauMs, dt);
    const cmdSmoothNext = prev.initialized ? prev.cmdSmooth + aSmooth * (shaped - prev.cmdSmooth) : shaped;
    const dCmd = prev.initialized ? (cmdSmoothNext - prev.cmdSmooth) / Math.max(dt, 1e-6) : 0;
    const aDeriv = pt1Alpha2(rt.derivativeCutoffHz, dt);
    const dCmdLpfNext = prev.initialized ? prev.dCmdLpf + aDeriv * (dCmd - prev.dCmdLpf) : 0;
    const maxRate = rt.maxRateDegS[axis];
    const targetRateDegS = cmdSmoothNext * maxRate;
    const feedforward = rt.derivativeGain * dCmdLpfNext * maxRate;
    const next = {
      cmdSmooth: cmdSmoothNext,
      prevCmdSmooth: cmdSmoothNext,
      dCmdLpf: dCmdLpfNext,
      initialized: true
    };
    return { value: feedforward, state: next, targetRateDegS };
  }
  function makePidMinWebFeedforwardMultiState() {
    return { roll: makePidMinWebFeedforwardState(), pitch: makePidMinWebFeedforwardState(), yaw: makePidMinWebFeedforwardState() };
  }
  function pidMinWebFeedforwardUpdateAll(prev, rt, joystick, dt) {
    const roll = pidMinWebFeedforwardUpdate(prev.roll, rt, "roll", joystick.roll, dt);
    const pitch = pidMinWebFeedforwardUpdate(prev.pitch, rt, "pitch", joystick.pitch, dt);
    const yaw = pidMinWebFeedforwardUpdate(prev.yaw, rt, "yaw", joystick.yaw, dt);
    return {
      values: { roll: roll.value, pitch: pitch.value, yaw: yaw.value },
      targetRatesDegS: { roll: roll.targetRateDegS, pitch: pitch.targetRateDegS, yaw: yaw.targetRateDegS },
      state: { roll: roll.state, pitch: pitch.state, yaw: yaw.state }
    };
  }

  // src/main/flight/pid_min_launch_stage.ts
  var PidMinLaunchStageState = class {
    baselineAlt = null;
    // meters at arming
    launchActive = false;
    enterAccumMs = 0;
    // running debounce accumulator
    exitAccumMs = 0;
    // running debounce accumulator
    armedPrev = false;
    // track rising edge for baseline capture
  };
  function makeDefaultLaunchStageConfig() {
    return {
      altEnterMeters: 0.1,
      altExitMeters: 0.4,
      vExitMetersPerSec: 0.7,
      rcEnterThreshold: 0.1,
      throttleEnterThreshold: 0.2,
      debounceMsEnter: 120,
      debounceMsExit: 150,
      useAccelZ: false,
      accelZExitThreshold: 0.5
    };
  }
  function updateLaunchStage(cfg, prev, inputs) {
    const next = new PidMinLaunchStageState();
    next.baselineAlt = prev.baselineAlt;
    next.launchActive = prev.launchActive;
    next.enterAccumMs = prev.enterAccumMs;
    next.exitAccumMs = prev.exitAccumMs;
    next.armedPrev = prev.armedPrev;
    const dtMs = inputs.timeStepMs > 0 ? inputs.timeStepMs : 0;
    if (inputs.armed && !next.armedPrev) {
      next.baselineAlt = inputs.altitude;
    }
    next.armedPrev = inputs.armed;
    let launchActive = next.launchActive;
    if (!inputs.lcEnabled || !inputs.armed) {
      launchActive = false;
      next.enterAccumMs = 0;
      next.exitAccumMs = 0;
    } else {
      const baselineAlt = next.baselineAlt;
      const altAbove = baselineAlt != null ? inputs.altitude - baselineAlt : 0;
      const wantLaunchCmd = Math.abs(inputs.rcDeflection) >= cfg.rcEnterThreshold || inputs.throttle >= cfg.throttleEnterThreshold || !!inputs.motorsSpinning;
      const onGround = baselineAlt != null && altAbove <= cfg.altEnterMeters;
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
      const exitByAlt = baselineAlt != null && altAbove >= cfg.altExitMeters;
      const exitByV = inputs.verticalSpeed >= cfg.vExitMetersPerSec;
      const exitByAccel = !!cfg.useAccelZ && inputs.accelZ !== void 0 && inputs.accelZ >= (cfg.accelZExitThreshold ?? 0.5);
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

  // src/main/flight/pid_min.ts
  var PidMinState = class {
    integrator = 0;
    // I-term accumulator
    prevGyroFiltered = 0;
    // last filtered gyro (for D-term stability)
    prevSetpoint = 0;
    // last setpoint (for feedforward)
    prevSetpointLpf = 0;
    // last lowpassed setpoint (for I-term relax)
    initialized = false;
    // guard first update
    dmax = makeDefaultDmaxState();
    // D-Max per-axis state
    // Advanced feedforward per-axis state (optional)
    ff = null;
    // Web feedforward per-axis state (optional)
    webFf = null;
  };
  function pidMinUpdateUnified(c, cfg, lc, prev, axis, setpoint, gyro, dt, launchActive, rcDeflection, currentPitchAngleDeg, trimPitchDeg, ffCtx, webCtx) {
    if (!c || !cfg || !prev) return { output: 0, state: new PidMinState() };
    if (dt <= 0) return { output: 0, state: new PidMinState() };
    const initialized = !!prev.initialized;
    const gyroFilteredPrev = initialized ? prev.prevGyroFiltered : gyro;
    const setpointPrev = initialized ? prev.prevSetpoint : setpoint;
    const setpointLpfPrev = initialized ? prev.prevSetpointLpf : setpoint;
    let integrator = initialized ? prev.integrator : 0;
    const dmaxPrev = initialized ? prev.dmax : makeDefaultDmaxState();
    let webF = 0;
    let webFfNext = prev.webFf || null;
    const modePre = cfg.feedforwardMode ?? "simple";
    if (modePre === "web" && cfg.webFeedforwardRuntime && webCtx) {
      const axisName = axis === 0 /* Roll */ ? "roll" : axis === 1 /* Pitch */ ? "pitch" : "yaw";
      const prevWeb = webFfNext ?? makePidMinWebFeedforwardState();
      const { value, state, targetRateDegS } = pidMinWebFeedforwardUpdate(
        prevWeb,
        cfg.webFeedforwardRuntime,
        axisName,
        webCtx.joystick,
        dt
      );
      setpoint = targetRateDegS;
      webF = value;
      webFfNext = state;
    }
    const launchEffects = computeLaunchEffects(
      axis,
      lc,
      launchActive,
      setpoint,
      rcDeflection,
      currentPitchAngleDeg,
      trimPitchDeg,
      c.Ki
    );
    setpoint = launchEffects.setpoint;
    const errorRate = setpoint - gyro;
    const iRelaxCfg = cfg.iRelax;
    const itermRelax = computeItermRelax(
      setpoint,
      gyro,
      dt,
      setpointLpfPrev,
      iRelaxCfg
    );
    let iErrorRate = itermRelax.iErrorRate;
    let setpointLpfNext = itermRelax.nextPrevSetpointLpf;
    let P = c.Kp * errorRate;
    let effectiveKi = launchEffects.effectiveKi;
    integrator += effectiveKi * iErrorRate * dt;
    if (cfg.integratorLeak > 0) {
      const leak = clampf(cfg.integratorLeak, 0, 1);
      integrator *= 1 - leak;
    }
    if (launchEffects.yawItermLimit !== null) {
      const yawLimit = launchEffects.yawItermLimit;
      integrator = clampf(integrator, -yawLimit, yawLimit);
    }
    if (launchEffects.yawItermLimit === null) {
      if (cfg.itermLimit > 0) {
        integrator = clampf(integrator, -cfg.itermLimit, cfg.itermLimit);
      }
    }
    let I = integrator;
    if (launchEffects.enforcePitchINonNegative && axis === 1 /* Pitch */) {
      if (I < 0) {
        I = 0;
        integrator = 0;
      }
    }
    const alpha = cfg.dLowpassCutoffHz > 0 ? pt1Alpha(cfg.dLowpassCutoffHz, dt) : 1;
    const gyroFiltered = gyroFilteredPrev + alpha * (gyro - gyroFilteredPrev);
    const dGyro = gyroFiltered - gyroFilteredPrev;
    const deltaGyroDt = -dGyro / dt;
    const setpointDelta = setpoint - setpointPrev;
    let dMultiplier = 1;
    let dmaxNext = dmaxPrev;
    if (cfg.dMax && cfg.dMax.enabled) {
      const dmaxRes = computeDmaxMultiplier(cfg.dMax, dmaxPrev, {
        axis,
        deltaGyroDt,
        setpointDelta,
        dt
      });
      dMultiplier = dmaxRes.multiplier;
      dmaxNext = dmaxRes.state;
    }
    let D = 0;
    if (!launchEffects.disableD) {
      D = c.Kd * deltaGyroDt * dMultiplier;
    }
    let F = 0;
    let ffNext = prev.ff || null;
    let webFfStored = webFfNext;
    if (cfg.useFeedforward && !launchEffects.disableFeedforward) {
      const mode = cfg.feedforwardMode ?? "simple";
      if (mode === "web" && cfg.webFeedforwardRuntime && webCtx) {
        F = webF;
      } else if (mode === "advanced" && cfg.feedforwardRuntime && ffCtx) {
        const prevFf = ffNext ?? makePidMinFeedforwardState(cfg.feedforwardRuntime);
        const { value, state } = pidMinFeedforwardUpdate(
          prevFf,
          cfg.feedforwardRuntime,
          axis,
          setpoint,
          ffCtx.rcCmd,
          ffCtx.rxRateHz,
          ffCtx.rxIntervalUs,
          ffCtx.maxRcRate
        );
        F = value;
        ffNext = state;
      } else if (c.Kf !== 0) {
        const dSetpoint = setpoint - setpointPrev;
        F = c.Kf * dSetpoint;
      }
    }
    if (launchEffects.disableP) {
      P = 0;
    }
    if (launchEffects.disableI) {
      I = 0;
    }
    let sum = P + I + D + F;
    if (cfg.pidSumLimit > 0) {
      sum = clampf(sum, -cfg.pidSumLimit, cfg.pidSumLimit);
    }
    const next = new PidMinState();
    next.integrator = integrator;
    next.prevGyroFiltered = gyroFiltered;
    next.prevSetpoint = setpoint;
    next.prevSetpointLpf = setpointLpfNext;
    next.initialized = true;
    next.dmax = dmaxNext;
    next.ff = ffNext;
    next.webFf = webFfStored;
    return { output: sum, state: next };
  }
  function normalizePidMinConfigForWeb(inCfg, inWebRt) {
    const feedforwardMode = inCfg.feedforwardMode ?? "web";
    const webRt = inCfg.webFeedforwardRuntime ?? (inWebRt ? normalizePidMinWebFeedforwardRuntime(inWebRt) : normalizePidMinWebFeedforwardRuntime({}));
    const dMax = inCfg.dMax ?? {
      enabled: true,
      dMaxPercent: { roll: 1.25, pitch: 1.25, yaw: 1.1 },
      gain: 37,
      advance: 20,
      rangeCutoffHz: 85,
      lowpassCutoffHz: 35
    };
    const iRelax = inCfg.iRelax ?? {
      iRelaxEnabled: true,
      iRelaxCutoffHz: 15,
      iRelaxSetpointThreshold: 40,
      iRelaxType: 1 /* Setpoint */
    };
    return {
      pidSumLimit: inCfg.pidSumLimit ?? 0,
      itermLimit: inCfg.itermLimit ?? 100,
      integratorLeak: inCfg.integratorLeak ?? 0.05,
      useFeedforward: inCfg.useFeedforward ?? true,
      dLowpassCutoffHz: inCfg.dLowpassCutoffHz ?? 40,
      iRelax,
      dMax,
      feedforwardMode,
      feedforwardRuntime: inCfg.feedforwardRuntime ?? null,
      webFeedforwardRuntime: webRt
    };
  }
  function normalizePidMinLaunchConfig(inLc) {
    return {
      enabled: inLc.enabled ?? true,
      mode: inLc.mode ?? 0 /* PitchOnly */,
      angleLimitDeg: inLc.angleLimitDeg ?? 30,
      kiOverride: inLc.kiOverride ?? 0.15,
      maxRateDegS: inLc.maxRateDegS ?? 100,
      minRateDegS: inLc.minRateDegS ?? 5,
      angleWindowDeg: inLc.angleWindowDeg ?? 10,
      yawItermLimitDegS: inLc.yawItermLimitDegS ?? 50
    };
  }
  function pidMinUpdateWebAll(c, cfg, lc, prev, gyro, joystick, dt, launchActive, rcDeflection, currentPitchAngleDeg, trimPitchDeg) {
    const r = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.roll,
      0 /* Roll */,
      0,
      gyro.roll,
      dt,
      launchActive,
      rcDeflection,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.roll }
    );
    const p = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.pitch,
      1 /* Pitch */,
      0,
      gyro.pitch,
      dt,
      launchActive,
      rcDeflection,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.pitch }
    );
    const y = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.yaw,
      2 /* Yaw */,
      0,
      gyro.yaw,
      dt,
      launchActive,
      rcDeflection,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.yaw }
    );
    return {
      outputs: { roll: r.output, pitch: p.output, yaw: y.output },
      state: { roll: r.state, pitch: p.state, yaw: y.state }
    };
  }
  function pidMinUpdateWebAllWithAxisDeflection(c, cfg, lc, prev, gyro, joystick, dt, launchActive, rcDeflectionAxis, currentPitchAngleDeg, trimPitchDeg) {
    const r = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.roll,
      0 /* Roll */,
      0,
      gyro.roll,
      dt,
      launchActive,
      rcDeflectionAxis.roll,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.roll }
    );
    const p = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.pitch,
      1 /* Pitch */,
      0,
      gyro.pitch,
      dt,
      launchActive,
      rcDeflectionAxis.pitch,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.pitch }
    );
    const y = pidMinUpdateUnified(
      c,
      cfg,
      lc,
      prev.yaw,
      2 /* Yaw */,
      0,
      gyro.yaw,
      dt,
      launchActive,
      rcDeflectionAxis.yaw,
      currentPitchAngleDeg,
      trimPitchDeg,
      void 0,
      { joystick: joystick.yaw }
    );
    return {
      outputs: { roll: r.output, pitch: p.output, yaw: y.output },
      state: { roll: r.state, pitch: p.state, yaw: y.state }
    };
  }
  function pidMinUpdateWebAllWithStage(c, cfg, lc, stageCfg, stagePrev, prev, gyro, joystick, dt, stageInputs, rcDeflectionAxis, currentPitchAngleDeg, trimPitchDeg) {
    const stageRes = updateLaunchStage(stageCfg, stagePrev, {
      armed: stageInputs.armed,
      lcEnabled: !!(lc && lc.enabled),
      altitude: stageInputs.altitude,
      verticalSpeed: stageInputs.verticalSpeed,
      throttle: stageInputs.throttle,
      rcDeflection: rcDeflectionAxis.pitch,
      // typical: pitch deflection drives stage
      motorsSpinning: stageInputs.motorsSpinning,
      accelZ: stageInputs.accelZ,
      timeStepMs: dt * 1e3
    });
    const launchActive = stageRes.launchActive;
    const pidRes = pidMinUpdateWebAllWithAxisDeflection(
      c,
      cfg,
      lc,
      prev,
      gyro,
      joystick,
      dt,
      launchActive,
      rcDeflectionAxis,
      currentPitchAngleDeg,
      trimPitchDeg
    );
    return { outputs: pidRes.outputs, state: pidRes.state, stage: stageRes.state, launchActive };
  }

  // src/main/flight/web_control_loop.ts
  function startWebControlLoop(opts) {
    const coeffs = opts.coeffs;
    const controlRateHz = opts.controlRateHz ?? 60;
    const cfg = isFullCfg(opts.cfg) ? opts.cfg : normalizePidMinConfigForWeb(opts.cfg, { controlRateHz, inputRateHz: opts.inputRateHz ?? 60 });
    const lc = opts.launch == null ? null : normalizePidMinLaunchConfig(opts.launch);
    const stageCfg = opts.stage ?? makeDefaultLaunchStageConfig();
    const controlDt = 1 / controlRateHz;
    let pidState = opts.initialState ?? { roll: new PidMinState(), pitch: new PidMinState(), yaw: new PidMinState() };
    let stageState = new PidMinLaunchStageState();
    let isRunning = true;
    const intervalMs = 1e3 / controlRateHz;
    let accMs = 0;
    const maxSteps = typeof opts.maxStepsPerTick === "number" && opts.maxStepsPerTick > 0 ? opts.maxStepsPerTick : Number.POSITIVE_INFINITY;
    const canUseRaf = typeof globalThis.requestAnimationFrame === "function" && typeof globalThis.cancelAnimationFrame === "function";
    const useRaf = opts.scheduler ? opts.scheduler === "raf" : canUseRaf;
    let timer = null;
    let rafId = null;
    const stepControl = () => {
      const joystick = opts.readJoystick();
      const gyro = opts.readGyro();
      const sensors = opts.readSensors ? opts.readSensors() : void 0;
      const pitchAngleDeg = opts.pitchAngleDeg ? opts.pitchAngleDeg() : 0;
      const trimPitchDeg = opts.trimPitchDeg ? opts.trimPitchDeg() : 0;
      if (sensors) {
        const res = pidMinUpdateWebAllWithStage(
          coeffs,
          cfg,
          lc ?? void 0,
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
            accelZ: sensors.accelZ
          },
          joystick,
          pitchAngleDeg,
          trimPitchDeg
        );
        pidState = res.state;
        stageState = res.stage;
        opts.onOutputs(res.outputs, controlDt);
        if (opts.onStageUpdate) opts.onStageUpdate(res.stage, res.launchActive);
      } else if (lc && lc.mode === 1 /* Full */) {
        const res = pidMinUpdateWebAllWithAxisDeflection(
          coeffs,
          cfg,
          lc,
          pidState,
          gyro,
          joystick,
          controlDt,
          /* launchActive */
          false,
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
          lc ?? void 0,
          pidState,
          gyro,
          joystick,
          controlDt,
          /* launchActive */
          false,
          /* rcDeflection */
          joystick.pitch,
          pitchAngleDeg,
          trimPitchDeg
        );
        pidState = res.state;
        opts.onOutputs(res.outputs, controlDt);
      }
    };
    if (useRaf) {
      let lastTs = null;
      const tickRaf = (ts) => {
        if (!isRunning) return;
        if (lastTs == null) lastTs = ts;
        accMs += ts - lastTs;
        lastTs = ts;
        let stepsRun = 0;
        while (accMs >= intervalMs && stepsRun < maxSteps) {
          stepControl();
          accMs -= intervalMs;
          stepsRun++;
        }
        rafId = globalThis.requestAnimationFrame(tickRaf);
      };
      rafId = globalThis.requestAnimationFrame(tickRaf);
    } else {
      let prev = typeof performance !== "undefined" && performance.now ? performance.now() : Date.now();
      const tickTimeout = () => {
        if (!isRunning) return;
        const now = typeof performance !== "undefined" && performance.now ? performance.now() : Date.now();
        accMs += now - prev;
        prev = now;
        let stepsRun = 0;
        while (accMs >= intervalMs && stepsRun < maxSteps) {
          stepControl();
          accMs -= intervalMs;
          stepsRun++;
        }
        const nextDelay = Math.max(0, intervalMs - accMs);
        timer = setTimeout(tickTimeout, nextDelay);
      };
      timer = setTimeout(tickTimeout, intervalMs);
    }
    const stop = () => {
      if (!isRunning) return;
      isRunning = false;
      if (timer != null) clearTimeout(timer);
      if (rafId != null && canUseRaf) globalThis.cancelAnimationFrame(rafId);
    };
    return { stop, isRunning, controlDt };
  }
  function isFullCfg(cfg) {
    return typeof cfg.pidSumLimit === "number" && typeof cfg.itermLimit === "number" && typeof cfg.integratorLeak === "number" && typeof cfg.dLowpassCutoffHz === "number" && !!cfg.iRelax;
  }

  // src/main/flight/demo/web/main.ts
  var el = {
    outRoll: document.getElementById("outRoll"),
    outPitch: document.getElementById("outPitch"),
    outYaw: document.getElementById("outYaw"),
    gyroRoll: document.getElementById("gyroRoll"),
    gyroPitch: document.getElementById("gyroPitch"),
    gyroYaw: document.getElementById("gyroYaw"),
    joyRoll: document.getElementById("joyRoll"),
    joyPitch: document.getElementById("joyPitch"),
    joyYaw: document.getElementById("joyYaw"),
    pitchAngle: document.getElementById("pitchAngle"),
    startBtn: document.getElementById("startBtn"),
    stopBtn: document.getElementById("stopBtn"),
    rateInput: document.getElementById("rateInput")
  };
  var sim = {
    outputs: { roll: 0, pitch: 0, yaw: 0 },
    gyro: { roll: 0, pitch: 0, yaw: 0 },
    pitchAngleDeg: 0,
    trimPitchDeg: 0,
    armed: true,
    altitudeMeters: 0,
    vSpeedMps: 0,
    throttle01: 0,
    motorsSpinning: true,
    accelZMps2: 0
  };
  var handle = null;
  var lastJoystick = { roll: 0, pitch: 0, yaw: 0 };
  var plotBufSize = 600;
  var targetBufs = { roll: new Array(plotBufSize).fill(0), pitch: new Array(plotBufSize).fill(0), yaw: new Array(plotBufSize).fill(0) };
  var gyroBufs = { roll: new Array(plotBufSize).fill(0), pitch: new Array(plotBufSize).fill(0), yaw: new Array(plotBufSize).fill(0) };
  var plotIdxs = { roll: 0, pitch: 0, yaw: 0 };
  var plotCounts = { roll: 0, pitch: 0, yaw: 0 };
  var plotCanvases = {
    roll: document.getElementById("plotRoll"),
    pitch: document.getElementById("plotPitch"),
    yaw: document.getElementById("plotYaw")
  };
  var plotCtxs = {
    roll: plotCanvases.roll.getContext("2d"),
    pitch: plotCanvases.pitch.getContext("2d"),
    yaw: plotCanvases.yaw.getContext("2d")
  };
  var plotYRange = 800;
  function appendPlotSample(axis, target, gyro) {
    const idx = plotIdxs[axis];
    targetBufs[axis][idx] = target;
    gyroBufs[axis][idx] = gyro;
    plotIdxs[axis] = (idx + 1) % plotBufSize;
    plotCounts[axis] = Math.min(plotCounts[axis] + 1, plotBufSize);
  }
  function renderPlotAxis(axis) {
    const canvas = plotCanvases[axis];
    const ctx = plotCtxs[axis];
    const w = canvas.width;
    const h = canvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = "#14141c";
    ctx.fillRect(0, 0, w, h);
    ctx.strokeStyle = "#232330";
    ctx.lineWidth = 1;
    for (let i = 0; i <= 4; i++) {
      const y = i / 4 * h;
      ctx.beginPath();
      ctx.moveTo(0, y);
      ctx.lineTo(w, y);
      ctx.stroke();
    }
    ctx.fillStyle = "#9ea0a8";
    ctx.font = "12px system-ui";
    ctx.fillText(`\xB1${plotYRange} deg/s`, 8, 14);
    ctx.fillText("target: teal, gyro: orange", w - 200, 14);
    const count = plotCounts[axis];
    if (count === 0) return;
    const toY = (v) => Math.round(h / 2 - v / plotYRange * (h / 2 - 8));
    const stepX = w / (plotBufSize - 1);
    const tb = targetBufs[axis];
    const gb = gyroBufs[axis];
    const baseIdx = plotIdxs[axis];
    ctx.strokeStyle = "#11c2b3";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < plotBufSize; i++) {
      const idx = (baseIdx + i) % plotBufSize;
      const x = i * stepX;
      const y = toY(tb[idx]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.strokeStyle = "#ff8a3d";
    ctx.lineWidth = 2;
    ctx.beginPath();
    for (let i = 0; i < plotBufSize; i++) {
      const idx = (baseIdx + i) % plotBufSize;
      const x = i * stepX;
      const y = toY(gb[idx]);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
  }
  function readGamepadAxes() {
    const pads = navigator.getGamepads?.() || [];
    const gp = pads[0];
    if (!gp) return lastJoystick;
    const roll = gp.axes[0] ?? 0;
    const pitch = gp.axes[1] ?? 0;
    const yaw = gp.axes[2] ?? gp.axes[3] ?? 0;
    lastJoystick = { roll, pitch, yaw };
    return lastJoystick;
  }
  function readGyro() {
    return sim.gyro;
  }
  function updatePlant(outputs, dt) {
    const tau = 0.02;
    const alpha = dt / (dt + tau);
    sim.outputs = outputs;
    sim.gyro.roll += alpha * (outputs.roll - sim.gyro.roll);
    sim.gyro.pitch += alpha * (outputs.pitch - sim.gyro.pitch);
    sim.gyro.yaw += alpha * (outputs.yaw - sim.gyro.yaw);
    sim.pitchAngleDeg += sim.gyro.pitch * dt;
  }
  function renderTelemetry() {
    const j = lastJoystick;
    el.outRoll.textContent = `roll: ${sim.outputs.roll.toFixed(1)}`;
    el.outPitch.textContent = `pitch: ${sim.outputs.pitch.toFixed(1)}`;
    el.outYaw.textContent = `yaw: ${sim.outputs.yaw.toFixed(1)}`;
    el.gyroRoll.textContent = `roll: ${sim.gyro.roll.toFixed(1)} deg/s`;
    el.gyroPitch.textContent = `pitch: ${sim.gyro.pitch.toFixed(1)} deg/s`;
    el.gyroYaw.textContent = `yaw: ${sim.gyro.yaw.toFixed(1)} deg/s`;
    el.joyRoll.textContent = `roll: ${j.roll.toFixed(2)}`;
    el.joyPitch.textContent = `pitch: ${j.pitch.toFixed(2)}`;
    el.joyYaw.textContent = `yaw: ${j.yaw.toFixed(2)}`;
    el.pitchAngle.textContent = `${sim.pitchAngleDeg.toFixed(1)} deg`;
  }
  function startLoop() {
    const controlRateHz = Math.max(60, Math.min(1e3, Number(el.rateInput.value) || 300));
    if (handle) handle.stop();
    el.startBtn.disabled = true;
    el.stopBtn.disabled = false;
    const webRuntime = normalizePidMinWebFeedforwardRuntime({ controlRateHz, inputRateHz: 60 });
    let ffState = makePidMinWebFeedforwardMultiState();
    handle = startWebControlLoop({
      coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0 },
      cfg: normalizePidMinConfigForWeb({ feedforwardMode: "web" }, { controlRateHz, inputRateHz: 60 }),
      launch: normalizePidMinLaunchConfig({ mode: 1 /* Full */, angleLimitDeg: 25 }),
      controlRateHz,
      inputRateHz: 60,
      scheduler: "raf",
      maxStepsPerTick: 8,
      readJoystick: () => readGamepadAxes(),
      readGyro: () => readGyro(),
      pitchAngleDeg: () => sim.pitchAngleDeg,
      trimPitchDeg: () => sim.trimPitchDeg,
      onOutputs: (outputs, dt) => {
        updatePlant(outputs, dt);
        const ff = pidMinWebFeedforwardUpdateAll(ffState, webRuntime, lastJoystick, dt);
        ffState = ff.state;
        appendPlotSample("roll", ff.targetRatesDegS.roll, sim.gyro.roll);
        appendPlotSample("pitch", ff.targetRatesDegS.pitch, sim.gyro.pitch);
        appendPlotSample("yaw", ff.targetRatesDegS.yaw, sim.gyro.yaw);
        renderTelemetry();
      }
    });
  }
  function stopLoop() {
    if (handle) handle.stop();
    handle = null;
    el.startBtn.disabled = false;
    el.stopBtn.disabled = true;
  }
  el.startBtn.addEventListener("click", startLoop);
  el.stopBtn.addEventListener("click", stopLoop);
  setTimeout(startLoop, 0);
  function tickUi() {
    renderTelemetry();
    requestAnimationFrame(tickUi);
  }
  requestAnimationFrame(tickUi);
  function tickPlot() {
    renderPlotAxis("roll");
    renderPlotAxis("pitch");
    renderPlotAxis("yaw");
    requestAnimationFrame(tickPlot);
  }
  requestAnimationFrame(tickPlot);
})();
//# sourceMappingURL=bundle.js.map
