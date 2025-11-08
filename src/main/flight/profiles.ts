// Profile configuration (TypeScript port of Betaflight PID and Rate profiles)
// - Defines TypeScript interfaces for `pidProfile_t` and `controlRateConfig_t`
// - Provides defaults based on reset functions in C (`resetPidProfile`, `pgResetFn_controlRateProfiles`)
// - Includes helpers to select active profiles and convert to pid_min.ts runtime types

import { PidMinAxis } from './pid_min_launch_control';
import {
  PidMinItermRelaxConfig,
  PidMinItermRelaxType,
} from './pid_min_iterm_relax';
import {
  PidMinCoefficients,
  PidMinConfig,
} from './pid_min';
import { PidMinDmaxConfig } from './pid_min_dmax';

// Counts (mirrors target/common_pre.h)
export const PID_PROFILE_COUNT = 4;
export const CONTROL_RATE_PROFILE_COUNT = 4;

// Axis indexing (mirrors pidIndex_e and FD_* aliases)
export enum PidIndex {
  Roll = 0,
  Pitch = 1,
  Yaw = 2,
  Level = 3,
  Mag = 4,
}

export enum FilterType {
  PT1 = 0,
  // BIQUAD = 1, PT2 = 2, PT3 = 3 (not needed in pid_min mapping)
}

export enum TpaMode {
  PD = 0,
  D = 1,
}

export enum TpaCurveType {
  Classic = 0,
  Hyperbolic = 1,
}

export enum TpaSpeedType {
  Basic = 0,
  Advanced = 1,
}

export enum YawType {
  Rudder = 0,
  DiffThrust = 1,
}

export enum ItermRelaxEnable {
  Off = 0,
  RP = 1,
  RPY = 2,
  RP_INC = 3,
  RPY_INC = 4,
}

export enum ItermRelaxType {
  Gyro = 0,
  Setpoint = 1,
}

export enum RatesType {
  Betaflight = 0,
  Raceflight = 1,
  Kiss = 2,
  Actual = 3,
  Quick = 4,
}

export enum ThrottleLimitType {
  Off = 0,
  Scale = 1,
  Clip = 2,
}

// Scale constants (mirrors pid.h)
export const PTERM_SCALE = 0.032029;
export const ITERM_SCALE = 0.244381;
export const DTERM_SCALE = 0.000529;
export const FEEDFORWARD_SCALE = 0.013754; // applied to F% (F * 0.01)

// Gains container (pidf_t)
export interface Pidf {
  P: number;
  I: number;
  D: number;
  F: number; // 0..1000, interpreted as percent in BF (scaled to coefficient)
  S: number; // wing-term (not used in pid_min.ts)
}

// PID profile (pidProfile_t)
export interface PidProfile {
  // Filters
  yaw_lowpass_hz: number;
  dterm_lpf1_static_hz: number;
  dterm_notch_hz: number;
  dterm_notch_cutoff: number;

  // Gains per PID index
  pid: Pidf[]; // length PID_ITEM_COUNT

  // Filter types and limits
  dterm_lpf1_type: number; // FilterType
  itermWindup: number;
  pidSumLimit: number;
  pidSumLimitYaw: number;
  pidAtMinThrottle: number; // PID_STABILISATION_ON/OFF
  angle_limit: number;

  horizon_limit_degrees: number;
  horizon_ignore_sticks: number; // boolean as uint8

  // Betaflight PID controller parameters
  anti_gravity_gain: number;
  yawRateAccelLimit: number;
  rateAccelLimit: number;
  crash_dthreshold: number;
  crash_gthreshold: number;
  crash_setpoint_threshold: number;
  crash_time: number;
  crash_delay: number;
  crash_recovery_angle: number;
  crash_recovery_rate: number;
  crash_limit_yaw: number;
  itermLimit: number;
  dterm_lpf2_static_hz: number;
  crash_recovery: number;
  throttle_boost: number;
  throttle_boost_cutoff: number;
  iterm_rotation: number;
  iterm_relax_type: number; // ItermRelaxType
  iterm_relax_cutoff: number;
  iterm_relax: number; // ItermRelaxEnable
  acro_trainer_angle_limit: number;
  acro_trainer_debug_axis: number;
  acro_trainer_gain: number;
  acro_trainer_lookahead_ms: number;
  abs_control_gain: number;
  abs_control_limit: number;
  abs_control_error_limit: number;
  abs_control_cutoff: number;
  dterm_lpf2_type: number; // FilterType
  dterm_lpf1_dyn_min_hz: number;
  dterm_lpf1_dyn_max_hz: number;
  launchControlMode: number;
  launchControlThrottlePercent: number;
  launchControlAngleLimit: number;
  launchControlGain: number;
  launchControlAllowTriggerReset: number; // boolean uint8
  use_integrated_yaw: number; // boolean uint8
  integrated_yaw_relax: number;
  thrustLinearization: number;
  d_max: [number, number, number];
  d_max_gain: number;
  d_max_advance: number;
  motor_output_limit: number;
  auto_profile_cell_count: number;
  transient_throttle_limit: number;
  profileName: string;

  // Dynamic idle
  dyn_idle_min_rpm: number;
  dyn_idle_p_gain: number;
  dyn_idle_i_gain: number;
  dyn_idle_d_gain: number;
  dyn_idle_max_increase: number;

  // Feedforward
  feedforward_transition: number;
  feedforward_averaging: number;
  feedforward_smooth_factor: number;
  feedforward_jitter_factor: number;
  feedforward_boost: number;
  feedforward_max_rate_limit: number;
  feedforward_yaw_hold_gain: number;
  feedforward_yaw_hold_time: number;

  dterm_lpf1_dyn_expo: number;
  level_race_mode: number; // boolean
  vbat_sag_compensation: number;

  // Simplified tuning
  simplified_pids_mode: number;
  simplified_master_multiplier: number;
  simplified_roll_pitch_ratio: number;
  simplified_i_gain: number;
  simplified_d_gain: number;
  simplified_pi_gain: number;
  simplified_d_max_gain: number;
  simplified_feedforward_gain: number;
  simplified_dterm_filter: number; // boolean
  simplified_dterm_filter_multiplier: number;
  simplified_pitch_pi_gain: number;

  // Anti-gravity
  anti_gravity_cutoff_hz: number;
  anti_gravity_p_gain: number;

  // TPA
  tpa_mode: number; // TpaMode
  tpa_rate: number;
  tpa_breakpoint: number;
  angle_feedforward_smoothing_ms: number;
  angle_earth_ref: number;
  horizon_delay_ms: number;
  tpa_low_rate: number;
  tpa_low_breakpoint: number;
  tpa_low_always: number; // boolean

  // Helpers
  ez_landing_threshold: number;
  ez_landing_limit: number;
  ez_landing_speed: number;
  landing_disarm_threshold: number;

  // SPA/TPA advanced
  spa_center: [number, number, number];
  spa_width: [number, number, number];
  spa_mode: [number, number, number];
  tpa_curve_type: number; // TpaCurveType
  tpa_curve_stall_throttle: number;
  tpa_curve_pid_thr0: number;
  tpa_curve_pid_thr100: number;
  tpa_curve_expo: number;
  tpa_speed_type: number; // TpaSpeedType
  tpa_speed_basic_delay: number;
  tpa_speed_basic_gravity: number;
  tpa_speed_adv_prop_pitch: number;
  tpa_speed_adv_mass: number;
  tpa_speed_adv_drag_k: number;
  tpa_speed_adv_thrust: number;
  tpa_speed_max_voltage: number;
  tpa_speed_pitch_offset: number;
  yaw_type: number; // YawType
  angle_pitch_offset: number;

  // Chirp
  chirp_lag_freq_hz: number;
  chirp_lead_freq_hz: number;
  chirp_amplitude_roll: number;
  chirp_amplitude_pitch: number;
  chirp_amplitude_yaw: number;
  chirp_frequency_start_deci_hz: number;
  chirp_frequency_end_deci_hz: number;
  chirp_time_seconds: number;
}

// Control rate profile (controlRateConfig_t)
export interface ControlRateConfig {
  thrMid8: number;
  thrExpo8: number;
  rates_type: number; // RatesType
  rcRates: [number, number, number];
  rcExpo: [number, number, number];
  rates: [number, number, number];
  throttle_limit_type: number; // ThrottleLimitType
  throttle_limit_percent: number;
  rate_limit: [number, number, number]; // uint16
  profileName: string;
  quickRatesRcExpo: number;
  thrHover8: number;
}

// Defaults (mirrors resetPidProfile and pgResetFn_controlRateProfiles)
// Note: `F` is used by pid_min to enable feedforward; `S` is not used in pid_min.
export const PID_ROLL_DEFAULT: Pidf = { P: 45, I: 80, D: 30, F: 120, S: 0 };
export const PID_PITCH_DEFAULT: Pidf = { P: 47, I: 84, D: 34, F: 125, S: 0 };
export const PID_YAW_DEFAULT: Pidf = { P: 45, I: 80, D: 0, F: 120, S: 0 };
export const D_MAX_DEFAULT: [number, number, number] = [40, 46, 0];

export function makeDefaultPidProfile(): PidProfile {
  // Defaults mirror Betaflight `resetPidProfile`. Inline comments mark parameters
  // currently consumed by pid_min conversion. Unmarked fields are present for
  // completeness and not used by pid_min today.
  return {
    pid: [PID_ROLL_DEFAULT, PID_PITCH_DEFAULT, PID_YAW_DEFAULT, { P: 50, I: 75, D: 75, F: 50, S: 0 }, { P: 40, I: 0, D: 0, F: 0, S: 0 }], // used by pid_min (P/I/D/F for axis coefficients; S unused)
    pidSumLimit: 500, // used by pid_min (output clamp)
    pidSumLimitYaw: 400,
    yaw_lowpass_hz: 100,
    dterm_notch_hz: 0,
    dterm_notch_cutoff: 0,
    itermWindup: 80,
    pidAtMinThrottle: 1,
    angle_limit: 60,

    horizon_limit_degrees: 135,
    horizon_ignore_sticks: 0,

    anti_gravity_gain: 80,
    yawRateAccelLimit: 0,
    rateAccelLimit: 0,
    crash_time: 500,
    crash_delay: 0,
    crash_recovery_angle: 10,
    crash_recovery_rate: 100,
    crash_dthreshold: 50,
    crash_gthreshold: 400,
    crash_setpoint_threshold: 350,
    crash_recovery: 0,
    crash_limit_yaw: 200,
    itermLimit: 400,
    throttle_boost: 5,
    throttle_boost_cutoff: 15,
    iterm_rotation: 0,
    iterm_relax: ItermRelaxEnable.RP, // used by pid_min (mode)
    iterm_relax_cutoff: 15, // used by pid_min (Hz)
    iterm_relax_type: ItermRelaxType.Setpoint, // used by pid_min (Setpoint vs Gyro)
    acro_trainer_angle_limit: 20,
    acro_trainer_lookahead_ms: 50,
    acro_trainer_debug_axis: 0,
    acro_trainer_gain: 75,
    abs_control_gain: 0,
    abs_control_limit: 90,
    abs_control_error_limit: 20,
    abs_control_cutoff: 11,
    dterm_lpf1_static_hz: 75, // used by pid_min (D-term PT1 cutoff)
    dterm_lpf2_static_hz: 150,
    dterm_lpf1_type: FilterType.PT1,
    dterm_lpf2_type: FilterType.PT1,
    dterm_lpf1_dyn_min_hz: 75,
    dterm_lpf1_dyn_max_hz: 150,
    launchControlMode: 0,
    launchControlThrottlePercent: 20,
    launchControlAngleLimit: 0,
    launchControlGain: 40,
    launchControlAllowTriggerReset: 1,
    use_integrated_yaw: 0,
    integrated_yaw_relax: 200,
    thrustLinearization: 0,
    d_max: D_MAX_DEFAULT, // used by pid_min (per-axis D-Max percent when enabled)
    d_max_gain: 37, // used by pid_min (gyro gain factor)
    d_max_advance: 20, // used by pid_min (setpoint gain factor)
    motor_output_limit: 100,
    auto_profile_cell_count: 0,
    transient_throttle_limit: 0,
    profileName: '',

    dyn_idle_min_rpm: 0,
    dyn_idle_p_gain: 50,
    dyn_idle_i_gain: 50,
    dyn_idle_d_gain: 50,
    dyn_idle_max_increase: 150,

    feedforward_transition: 0,
    feedforward_averaging: 1,
    feedforward_smooth_factor: 65,
    feedforward_jitter_factor: 7,
    feedforward_boost: 15,
    feedforward_max_rate_limit: 90,
    feedforward_yaw_hold_gain: 15,
    feedforward_yaw_hold_time: 100,

    dterm_lpf1_dyn_expo: 5,
    level_race_mode: 0,
    vbat_sag_compensation: 0,

    simplified_pids_mode: 2, // PID_SIMPLIFIED_TUNING_RPY
    simplified_master_multiplier: 100,
    simplified_roll_pitch_ratio: 100,
    simplified_i_gain: 100,
    simplified_d_gain: 100,
    simplified_pi_gain: 100,
    simplified_d_max_gain: 100,
    simplified_feedforward_gain: 100,
    simplified_pitch_pi_gain: 100,
    simplified_dterm_filter: 1,
    simplified_dterm_filter_multiplier: 100,

    anti_gravity_cutoff_hz: 5,
    anti_gravity_p_gain: 100,

    tpa_mode: TpaMode.D,
    tpa_rate: 65,
    tpa_breakpoint: 1350,
    angle_feedforward_smoothing_ms: 80,
    angle_earth_ref: 100,
    horizon_delay_ms: 500,
    tpa_low_rate: 20,
    tpa_low_breakpoint: 1050,
    tpa_low_always: 0,

    ez_landing_threshold: 25,
    ez_landing_limit: 15,
    ez_landing_speed: 50,
    landing_disarm_threshold: 0,

    spa_center: [0, 0, 0],
    spa_width: [0, 0, 0],
    spa_mode: [0, 0, 0],
    tpa_curve_type: TpaCurveType.Classic,
    tpa_curve_stall_throttle: 30,
    tpa_curve_pid_thr0: 200,
    tpa_curve_pid_thr100: 70,
    tpa_curve_expo: 20,
    tpa_speed_type: TpaSpeedType.Basic,
    tpa_speed_basic_delay: 1000,
    tpa_speed_basic_gravity: 50,
    tpa_speed_adv_prop_pitch: 370,
    tpa_speed_adv_mass: 1000,
    tpa_speed_adv_drag_k: 1000,
    tpa_speed_adv_thrust: 2000,
    tpa_speed_max_voltage: 2520,
    tpa_speed_pitch_offset: 0,
    yaw_type: YawType.Rudder,
    angle_pitch_offset: 0,

    chirp_lag_freq_hz: 3,
    chirp_lead_freq_hz: 30,
    chirp_amplitude_roll: 230,
    chirp_amplitude_pitch: 230,
    chirp_amplitude_yaw: 180,
    chirp_frequency_start_deci_hz: 2,
    chirp_frequency_end_deci_hz: 6000,
    chirp_time_seconds: 20,
  };
}

export function makeDefaultControlRateConfig(): ControlRateConfig {
  // Control rate defaults mirror `pgResetFn_controlRateProfiles`. These are not
  // consumed by pid_min, but kept for completeness and UI/config management.
  return {
    thrMid8: 50,
    thrExpo8: 0,
    rates_type: RatesType.Actual,
    rcRates: [7, 7, 7],
    rcExpo: [0, 0, 0],
    rates: [67, 67, 67],
    throttle_limit_type: ThrottleLimitType.Off,
    throttle_limit_percent: 100,
    rate_limit: [2000, 2000, 2000], // CONTROL_RATE_CONFIG_RATE_LIMIT_MAX
    profileName: '',
    quickRatesRcExpo: 0,
    thrHover8: 50,
  };
}

// Arrays mirroring PG arrays in C
export const pidProfiles: PidProfile[] = Array.from({ length: PID_PROFILE_COUNT }, () => makeDefaultPidProfile());
export const controlRateProfiles: ControlRateConfig[] = Array.from({ length: CONTROL_RATE_PROFILE_COUNT }, () => makeDefaultControlRateConfig());

// Active indices (systemConfig equivalents)
export let currentPidProfileIndex = 0;
export let currentRateProfileIndex = 0;

export function loadPidProfile(index: number): PidProfile {
  currentPidProfileIndex = Math.max(0, Math.min(index, PID_PROFILE_COUNT - 1));
  return pidProfiles[currentPidProfileIndex];
}

export function changePidProfile(index: number): void {
  currentPidProfileIndex = Math.max(0, Math.min(index, PID_PROFILE_COUNT - 1));
}

export function copyPidProfile(dstIndex: number, srcIndex: number): void {
  if (dstIndex === srcIndex) return;
  if (dstIndex < 0 || dstIndex >= PID_PROFILE_COUNT) return;
  if (srcIndex < 0 || srcIndex >= PID_PROFILE_COUNT) return;
  pidProfiles[dstIndex] = JSON.parse(JSON.stringify(pidProfiles[srcIndex]));
}

export function loadControlRateProfile(index: number): ControlRateConfig {
  currentRateProfileIndex = Math.max(0, Math.min(index, CONTROL_RATE_PROFILE_COUNT - 1));
  return controlRateProfiles[currentRateProfileIndex];
}

export function changeControlRateProfile(index: number): void {
  currentRateProfileIndex = Math.max(0, Math.min(index, CONTROL_RATE_PROFILE_COUNT - 1));
}

export function copyControlRateProfile(dstIndex: number, srcIndex: number): void {
  if (dstIndex === srcIndex) return;
  if (dstIndex < 0 || dstIndex >= CONTROL_RATE_PROFILE_COUNT) return;
  if (srcIndex < 0 || srcIndex >= CONTROL_RATE_PROFILE_COUNT) return;
  controlRateProfiles[dstIndex] = JSON.parse(JSON.stringify(controlRateProfiles[srcIndex]));
}

// Converter: build pid_min.ts coefficients and config for an axis from a PidProfile
export function buildPidMinParams(
  axis: PidMinAxis,
  profile: PidProfile
): { coeff: PidMinCoefficients; cfg: PidMinConfig; dmax?: PidMinDmaxConfig } {
  const idx = axis === PidMinAxis.Roll ? PidIndex.Roll : axis === PidMinAxis.Pitch ? PidIndex.Pitch : PidIndex.Yaw;
  const gains = profile.pid[idx];
  const baseD = gains.D > 0 ? gains.D : 1; // avoid div-by-zero
  const percentByAxis = {
    roll: profile.d_max[0] > 0 ? Math.max(1, profile.d_max[0] / (idx === PidIndex.Roll ? baseD : gains.D)) : 1,
    pitch: profile.d_max[1] > 0 ? Math.max(1, profile.d_max[1] / (idx === PidIndex.Pitch ? baseD : gains.D)) : 1,
    yaw: profile.d_max[2] > 0 ? Math.max(1, profile.d_max[2] / (idx === PidIndex.Yaw ? baseD : gains.D)) : 1,
  };

  const coeff: PidMinCoefficients = {
    Kp: PTERM_SCALE * gains.P,
    Ki: ITERM_SCALE * gains.I,
    Kd: DTERM_SCALE * gains.D,
    Kf: FEEDFORWARD_SCALE * (gains.F * 0.01),
  };

  const iRelaxEnabled = profile.iterm_relax !== ItermRelaxEnable.Off;
  const iRelaxType = profile.iterm_relax_type === ItermRelaxType.Setpoint ? PidMinItermRelaxType.Setpoint : PidMinItermRelaxType.Gyro;

  const cfg: PidMinConfig = {
    pidSumLimit: profile.pidSumLimit,
    itermLimit: profile.itermLimit,
    integratorLeak: 0, // Betaflight doesn't use leak by default; keep 0
    useFeedforward: gains.F > 0,
    dLowpassCutoffHz: profile.dterm_lpf1_static_hz,
    iRelax: {
      iRelaxEnabled,
      iRelaxCutoffHz: profile.iterm_relax_cutoff,
      iRelaxSetpointThreshold: 40.0, // ITERM_RELAX_SETPOINT_THRESHOLD
      iRelaxType,
    },
    dMax: null,
    feedforwardMode: 'simple',
    feedforwardRuntime: null,
    webFeedforwardRuntime: null,
  };

  let dmax: PidMinDmaxConfig | undefined;
  if (profile.d_max_gain > 0 || profile.d_max_advance > 0) {
    dmax = {
      enabled: true,
      dMaxPercent: percentByAxis,
      gain: profile.d_max_gain,
      advance: profile.d_max_advance,
    };
  }

  return { coeff, cfg, dmax };
}

// Simple example: get axis params for current profile
export function buildParamsForCurrent(
  axis: PidMinAxis
): { coeff: PidMinCoefficients; cfg: PidMinConfig; dmax?: PidMinDmaxConfig } {
  return buildPidMinParams(axis, pidProfiles[currentPidProfileIndex]);
}