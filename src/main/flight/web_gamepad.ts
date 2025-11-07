// Gamepad utilities: framework-agnostic helpers to read joystick axes
// and optional throttle from the browser Gamepad API.
//
// Provides shaping options (deadband, expo) and per-axis inversion/mapping.
// Defaults target common pads: roll=RSX (axis 2), pitch=RSY inverted (axis 3), yaw=LSX (axis 0).
// Throttle defaults to LSY (axis 1) mapped to [0..1].

import type { Axis3 } from './pid_min';

export interface GamepadAxesMapping {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface GamepadAxesInvert {
  roll: boolean;
  pitch: boolean;
  yaw: boolean;
}

export interface GamepadAxesOffsets {
  roll: number;
  pitch: number;
  yaw: number;
}

export interface GamepadReadConfig {
  index?: number; // prefer this gamepad index; otherwise first connected
  mapping: GamepadAxesMapping;
  invert: GamepadAxesInvert;
  offsets: GamepadAxesOffsets; // center offsets to subtract before shaping
  deadband: number; // 0..0.2 typical
  expo: number; // 0..1 cubic strength; default 0 (feedforward does expo)
  clamp: boolean; // clamp final values to [-1,1]
  fallback: Axis3<number>; // used when no pad available
}

export interface GamepadThrottleConfig {
  index?: number;
  axis: number;       // default 1 (LSY)
  invert: boolean;    // invert before mapping to [0..1]
  deadband: number;   // small deadband around stick center (mapped to [0..1])
  clamp: boolean;     // clamp final throttle to [0,1]
  fallback: number;   // used when no pad available
}

export function normalizeGamepadReadConfig(inCfg?: Partial<GamepadReadConfig>): GamepadReadConfig {
  return {
    index: inCfg?.index,
    mapping: inCfg?.mapping ?? { roll: 2, pitch: 3, yaw: 0 },
    invert: inCfg?.invert ?? { roll: false, pitch: true, yaw: false },
    offsets: inCfg?.offsets ?? { roll: 0, pitch: 0, yaw: 0 },
    deadband: clamp01(inCfg?.deadband ?? 0.02),
    expo: clamp01(inCfg?.expo ?? 0.0),
    clamp: inCfg?.clamp ?? true,
    fallback: inCfg?.fallback ?? { roll: 0, pitch: 0, yaw: 0 },
  };
}

export function normalizeGamepadThrottleConfig(inCfg?: Partial<GamepadThrottleConfig>): GamepadThrottleConfig {
  return {
    index: inCfg?.index,
    axis: inCfg?.axis ?? 1,
    invert: inCfg?.invert ?? false,
    deadband: clamp01(inCfg?.deadband ?? 0.0),
    clamp: inCfg?.clamp ?? true,
    fallback: clamp01(inCfg?.fallback ?? 0.0),
  };
}

export function readGamepadAxes(inCfg?: Partial<GamepadReadConfig>, padOrIndex?: Gamepad | number): Axis3<number> {
  const cfg = normalizeGamepadReadConfig(inCfg);
  const pad = resolveGamepad(padOrIndex ?? cfg.index);
  if (!pad) return cfg.fallback;

  const rawRoll = axisAt(pad, cfg.mapping.roll);
  const rawPitch = axisAt(pad, cfg.mapping.pitch);
  const rawYaw = axisAt(pad, cfg.mapping.yaw);

  const r = shapeAxis(rawRoll - cfg.offsets.roll, cfg.invert.roll, cfg.deadband, cfg.expo, cfg.clamp);
  const p = shapeAxis(rawPitch - cfg.offsets.pitch, cfg.invert.pitch, cfg.deadband, cfg.expo, cfg.clamp);
  const y = shapeAxis(rawYaw - cfg.offsets.yaw, cfg.invert.yaw, cfg.deadband, cfg.expo, cfg.clamp);
  return { roll: r, pitch: p, yaw: y };
}

export function readGamepadThrottle(inCfg?: Partial<GamepadThrottleConfig>, padOrIndex?: Gamepad | number): number {
  const cfg = normalizeGamepadThrottleConfig(inCfg);
  const pad = resolveGamepad(padOrIndex ?? cfg.index);
  if (!pad) return cfg.fallback;

  const raw = axisAt(pad, cfg.axis);
  const v = cfg.invert ? -raw : raw;
  const vDb = applyDeadband(v, cfg.deadband);
  // Map [-1..1] to [0..1]
  let t = (vDb + 1) / 2;
  if (cfg.clamp) t = clamp01(t);
  return t;
}

// --- internals --------------------------------------------------------------

function resolveGamepad(padOrIndex?: Gamepad | number): Gamepad | null {
  try {
    const nav: any = (typeof navigator !== 'undefined') ? navigator : null;
    const list: (Gamepad | null)[] | null = nav && typeof nav.getGamepads === 'function' ? nav.getGamepads() : null;
    if (!list) return null;
    if (typeof padOrIndex === 'number') {
      const p = list[padOrIndex];
      return p && p.connected ? p : null;
    }
    if (padOrIndex && padOrIndex.connected) return padOrIndex;
    for (const gp of list) {
      if (gp && gp.connected) return gp;
    }
    return null;
  } catch {
    return null;
  }
}

function axisAt(pad: Gamepad, idx: number): number {
  if (!pad || !pad.axes || idx < 0 || idx >= pad.axes.length) return 0;
  const v = pad.axes[idx] ?? 0;
  // Some drivers report slight out-of-range; clamp gently
  return v < -1 ? -1 : v > 1 ? 1 : v;
}

function shapeAxis(raw: number, invert: boolean, deadband: number, expo: number, clamp: boolean): number {
  const v = invert ? -raw : raw;
  const vDb = applyDeadband(v, deadband);
  // Expo: cubic mix; keep default 0 so feedforward handles expo
  const vExpo = expoCubic(vDb, expo);
  return clamp ? clampSym(vExpo) : vExpo;
}

function applyDeadband(x: number, d: number): number {
  const ad = Math.abs(x);
  if (ad <= d) return 0;
  const sign = Math.sign(x) || 1;
  const scaled = (ad - d) / (1 - d);
  return sign * scaled;
}

function expoCubic(x: number, expo: number): number {
  const e = clamp01(expo);
  if (e <= 0) return x;
  return x * (1 - e) + (x * x * x) * e;
}

function clampSym(x: number): number { return x < -1 ? -1 : x > 1 ? 1 : x; }
function clamp01(x: number): number { return x < 0 ? 0 : x > 1 ? 1 : x; }