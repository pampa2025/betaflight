# Betaflight PID Features — Practical Guide for a Web Simulator

This guide summarizes PID-related features in Betaflight, what each is for, how it works conceptually, and how you might treat it in a browser-based simulator. Field names correspond to the `PidProfile` you ported, with code references to `pid.c` and `pid_init.c` where behavior is implemented.

## Scope
- Focuses on rate/attitude control features wired into Betaflight’s PID loop.
- Crash/launch/recovery features are described for completeness; you can ignore them for a pure web sim unless you simulate crashes/launches.

## Porting Status (TypeScript)
- Implemented
  - Minimal PID controller: `src/main/flight/pid_min.ts`
  - Profile mapping and example: `src/main/flight/profiles.ts`, `src/main/flight/pid_min_example.ts`
  - I‑term Relax: `src/main/flight/pid_min_iterm_relax.ts`
  - D‑Max: `src/main/flight/pid_min_dmax.ts`
  - Feedforward (core + web smoothing): `src/main/flight/pid_min_feedforward.ts`, `src/main/flight/pid_min_feedforward_web.ts`
  - Launch Control (effects + staging): `src/main/flight/pid_min_launch_control.ts`, `src/main/flight/pid_min_launch_stage.ts`
  - Web control loop and inputs: `src/main/flight/web_control_loop.ts`, `src/main/flight/web_controller.ts`, `src/main/flight/web_gamepad.ts`, `src/main/flight/web_gamepad_detect.ts`, `src/main/flight/pid_web_test.ts`, `src/main/flight/utils.ts`

- Pending / optional (not ported yet)
  - Anti‑Gravity, Absolute Control
  - Crash Recovery & Yaw Spin Recovery
  - Dynamic D‑term LPF scheduling
  - Yaw P‑term lowpass, Throttle Boost
  - Thrust Linearization
  - Dynamic Idle, VBAT Sag Compensation
  - EZ Landing & Disarm
  - Angle/Horizon extras (earth reference, delays, limits)
  - Motor Output Limit & Transient Throttle Limit
  - Chirp Excitation, SPA (wing)
  - Advanced TPA curve and wing speed model

- Suggested next steps
  - Add basic TPA (rate/breakpoint) to `pid_min.ts`
  - Wire feedforward averaging/jitter/boost toggles into the web loop
  - Schedule a simple dynamic D‑LPF (min/max Hz + expo) if needed

## Quick Categories
- Stability & integrator tools: Anti‑Gravity, I‑term Relax, I‑term Rotation, Absolute Control, Thrust Linearization.
- Feedforward & setpoint tools: Transition, Averaging, Smoothing normalization, Jitter factor, Boost, Yaw Hold.
- Dynamic behavior: D‑Max, Dynamic D‑LPF, TPA and advanced TPA curve/speed model.
- Mode and angle helpers: Angle feedforward smoothing, Earth reference for yaw, Horizon delay, Level race mode, Angle/horizon limits.
- Safety/utility: Crash Recovery, Yaw Spin Recovery, Launch Control, EZ Landing/Disarm, Motor Output Limit, Transient Throttle Limit.
- Mixer/power: Dynamic Idle, VBAT Sag Compensation.
- Diagnostics: Chirp excitation.

---

## Anti‑Gravity
- What: Temporarily boosts `I` accumulation and adds a small `P` assist when throttle changes quickly, helping hold attitude during aggressive throttle moves.
- How: Computes an accelerator `itermAccelerator` from throttle derivative, added to `Ki * error * dT`; also small center‑stick `P` boost on roll/pitch. Enabled per loop when anti‑gravity is active.
- Fields: `anti_gravity_gain`, `anti_gravity_p_gain`, `anti_gravity_cutoff_hz`.
- Code: `pid_init.c::pidInitConfig` (antiGravityGain, antiGravityPGain), `pid.c::pidController` Anti‑Gravity section.
- Simulator: Optional. Good realism under throttle pumps. You can approximate with a simple throttle‑rate detector.

## I‑term Relax
- What: Prevents integrator wind‑up during fast setpoint changes (stick moves), reducing bounce‑back.
- How: Modifies the error used by I (either setpoint‑based or gyro‑based detection) when transients or jitter are present.
- Fields: `iterm_relax` (mode), `iterm_relax_type` (Setpoint/Gyro), `iterm_relax_cutoff`.
- Code: `pid_init.c::pidInitConfig`, `pid.c::applyItermRelax`.
- Simulator: Recommended. You already ported this.

## I‑term Rotation
- What: Rotates the accumulated integrator when the reference frame rotates (useful for certain flight modes), keeping I aligned.
- How: Updates integrator storage via `rotateItermAndAxisError()` before applying PID.
- Fields: `iterm_rotation`.
- Code: `pid_init.c::pidInitConfig`, `pid.c::rotateItermAndAxisError` call.
- Simulator: Optional unless you simulate frame rotation effects with non‑standard axes.

## Absolute Control (AC)
- What: Compensates for model mismatch by transferring a small fraction of `P` into `I` (reducing steady‑state error and bias).
- How: Computes an `iCorrection = -acGain * PTERM_SCALE / ITERM_SCALE * Kp` and applies it to `Ki`; clamps by error/limit/cutoff parameters.
- Fields: `abs_control_gain`, `abs_control_limit`, `abs_control_error_limit`, `abs_control_cutoff`.
- Code: `pid_init.c::pidInitConfig` under `USE_ABSOLUTE_CONTROL` and adjustments near Ki; minor FF corrections in `pid.c`.
- Simulator: Optional. Useful to simulate persistent biases or trim errors.

## Thrust Linearization
- What: Compensates for non‑linear motor/thrust response so stick feel remains linear.
- How: Maps throttle through a quadratic form (`thrustLinearization - 0.5 * thrustLinearization^2`) used for compensation.
- Fields: `thrustLinearization`.
- Code: `pid_init.c::pidInitConfig` under `USE_THRUST_LINEARIZATION`.
- Simulator: Optional. If your motor model is simple linear, you can skip.

## Feedforward Subsystem
- What: Predictive term that multiplies setpoint delta (stick velocity) by `Kf` to preempt motor lag.
- Components:
  - Transition: `feedforward_transition` attenuates around center stick.
  - Averaging: `feedforward_averaging` selects averaging depth on RC samples.
  - Smoothing normalization: `feedforward_smooth_factor` normalized to 250 Hz packet rate.
  - Jitter factor: `feedforward_jitter_factor` attenuates FF when RC steps are tiny.
  - Boost: `feedforward_boost` adds small acceleration assist.
  - Max rate limit: `feedforward_max_rate_limit` caps FF.
  - Yaw hold: `feedforward_yaw_hold_gain` and `feedforward_yaw_hold_time` keep a sustained yaw component.
- Fields: all above; `pid[axis].F` enables FF via `Kf`.
- Code: `pid_init.c::pidInitConfig` under `USE_FEEDFORWARD`; FF in `pid.c::pidController`.
- Simulator: Recommended to capture crisp stick feel. You already scale `F` into `Kf`; advanced modules are optional.

## D‑Max (Dynamic D Boost)
- What: Boosts `Kd` towards a higher `Dmax` during propwash‑range activity, improving damping only when needed.
- How: PT2 filter detects gyro derivative energy in a specific band; multiplies `Kd` by a factor up to `dMaxPercent[axis]`, smoothed by a lowpass.
- Fields: `d_max[axis]`, `d_max_gain`, `d_max_advance`.
- Code: `pid_init.c::pidInitConfig` and `pid.c::pidController` under `USE_D_MAX`.
- Simulator: Optional. You’ve ported a minimal D‑Max model; include if you want realistic propwash behavior.

## Dynamic D‑term LPF
- What: Varies D‑term LPF cutoff based on throttle/conditions to balance noise vs responsiveness.
- How: Chooses filter family and min/max Hz, with an expo curve to schedule the cutoff.
- Fields: `dterm_lpf1_type`, `dterm_lpf1_dyn_min_hz`, `dterm_lpf1_dyn_max_hz`, `dterm_lpf1_dyn_expo`.
- Code: `pid_init.c::pidInitFilters/pidInitConfig` under `USE_DYN_LPF`.
- Simulator: Optional; your minimal PID uses a fixed PT1.

## TPA — Throttle PID Attenuation
- What: Reduces `P` and `D` at higher throttle to avoid oscillations when prop loading rises.
- How: Uses breakpoints/multipliers from `tpa_rate/breakpoint`; optional low‑throttle TPA via `tpa_low_rate/breakpoint`.
- Fields: `tpa_mode`, `tpa_rate`, `tpa_breakpoint`, `tpa_low_rate`, `tpa_low_breakpoint`, `tpa_low_always`.
- Code: `pid_init.c::pidInitConfig` TPA mapping; applied via `getTpaFactor` in `pid.c`.
- Simulator: Recommended basic TPA (rate/breakpoint). Low TPA optional.

### Advanced TPA Curve & Speed Model (WING only)
- What: Physics‑informed TPA based on estimated airspeed and aerodynamic parameters.
- How: Hyperbolic curve via `tpa_curve_*`, speed model via `tpa_speed_*` (mass, drag, prop pitch).
- Fields: `tpa_curve_type`, `tpa_curve_stall_throttle`, `tpa_curve_pid_thr0`, `tpa_curve_pid_thr100`, `tpa_curve_expo`; `tpa_speed_type`, `tpa_speed_basic_*`, `tpa_speed_adv_*`.
- Code: `pid_init.c::tpaCurveInit` and `tpaSpeedInit` (within `USE_WING`).
- Simulator: Skip unless simulating fixed‑wing.

## Angle/Horizon Extras
- Angle feedforward smoothing: `angle_feedforward_smoothing_ms` lowers FF cutoff for angle mode to avoid RC steps.
- Earth reference for yaw: `angle_earth_ref` attenuates yaw setpoint when roll/pitch are large.
- Horizon delay: `horizon_delay_ms` smooths horizon levelling transitions.
- Level race mode: `level_race_mode` selects roll‑only vs roll+pitch levelling.
- Limits: `angle_limit`, `horizon_limit_degrees`, stick limit shaping via `pid[PID_LEVEL].D`.
- Fields: above plus `pid[PID_LEVEL]` gains and `angle_pitch_offset`.
- Code: `pid_init.c::pidInitConfig` and `pid.c::pidLevel`, angle mode blocks.
- Simulator: Implement only if simulating Angle/Horizon.

## Crash Recovery & Yaw Spin Recovery
- What: Detects instability (gyro/D spikes, setpoint limits) and reduces/zeroes PIDs to recover; yaw spin recovery zeroes yaw setpoint.
- How: Thresholds on gyro error, D‑term delta, time since level entered, and setpoint size.
- Fields: `crash_time`, `crash_delay`, `crash_recovery_angle`, `crash_recovery_rate`, `crash_dthreshold`, `crash_gthreshold`, `crash_setpoint_threshold`, `crash_limit_yaw`, `crash_recovery`.
- Code: `pid_init.c::pidInitConfig` (runtime thresholds), `pid.c::handleCrashRecovery/detectAndSetCrashRecovery`.
- Simulator: Not needed for your web simulator unless you model crashes.

## Launch Control
- What: Assists take‑off by gating P/I/D/FF, enforcing angle limits, and applying a dedicated Ki on pitch.
- How: Modes include Full or Pitch‑only. Disables some axes and clamps yaw I to prevent windup.
- Fields: `launchControlMode`, `launchControlThrottlePercent`, `launchControlAngleLimit`, `launchControlGain`, `launchControlAllowTriggerReset`.
- Code: `pid_init.c::pidInitConfig` under `USE_LAUNCH_CONTROL`, `pid.c::applyLaunchControl`.
- Simulator: Optional; skip unless you simulate takeoff assistance.

## Dynamic Idle
- What: Raises minimum motor RPM dynamically under propwash to prevent motor stall.
- How: Mixer side computes idle increases from P/I/D activity and caps by `dyn_idle_max_increase`.
- Fields: `dyn_idle_min_rpm`, `dyn_idle_p_gain`, `dyn_idle_i_gain`, `dyn_idle_d_gain`, `dyn_idle_max_increase`.
- Code: `mixer_init.c` (idle runtime derivation).
- Simulator: Skip unless you model ESC/motor idle behavior.

## VBAT Sag Compensation
- What: Compensates for battery voltage sag by nudging throttle.
- How: Computes a sag factor and applies to mixer throttle.
- Fields: `vbat_sag_compensation`.
- Code: `mixer_init.c`.
- Simulator: Skip unless you simulate battery voltage dynamics.

## Yaw Lowpass (P‑term)
- What: Simple PT1 on yaw P to tame noisy yaw response.
- Fields: `yaw_lowpass_hz`.
- Code: `pid_init.c::pidInitFilters` (ptermYawLowpass), applied in `pid.c`.
- Simulator: Optional; include a basic PT1 if yaw noise is a problem.

## Throttle Boost
- What: Slight transient boost when throttle increases to improve punch.
- Fields: `throttle_boost`, `throttle_boost_cutoff`.
- Code: `pid_init.c::pidInitConfig` and `pidInitFilters`.
- Simulator: Optional.

## EZ Landing & Disarm Threshold
- What: Softens landing and disarms when sustained impact is detected.
- Fields: `ez_landing_threshold`, `ez_landing_limit`, `ez_landing_speed`, `landing_disarm_threshold`.
- Code: `mixer_init.c` and `pid_init.c` (`useEzDisarm`).
- Simulator: Not needed.

## SPA — Setpoint PID Attenuation (Wing)
- What: Reduces PID terms near high stick deflection for wing models; optionally freezes I.
- How: Computes `spa[axis] = 1 − smoothStepUpTransition(|setpoint|, center, width)`, then multiplies selected terms.
- Fields: `spa_center[axis]`, `spa_width[axis]`, `spa_mode[axis]`.
- Code: `pid.c::calculateSpaValues` and `applySpa` under `USE_WING`.
- Simulator: Skip for multirotor; only relevant to fixed‑wing.

## Chirp Excitation
- What: Injects a swept‑frequency sine to characterize response for tuning.
- How: Generates chirp per axis with lag/lead compensation and logs phase.
- Fields: `chirp_lag_freq_hz`, `chirp_lead_freq_hz`, `chirp_amplitude_*`, `chirp_frequency_start_deci_hz`, `chirp_frequency_end_deci_hz`, `chirp_time_seconds`.
- Code: `pid_init.c::pidInitConfig` and chirp block in `pid.c`.
- Simulator: Optional; handy for virtual tuning experiments.

## Yaw Types
- What: Choose yaw control strategy.
- Modes: `Rudder` (standard multirotor) vs `DiffThrust` (wing/differential thrust). Diff thrust can skip yaw PID and treat yaw differently.
- Fields: `yaw_type`.
- Code: `pid.c` checks for `YAW_TYPE_DIFF_THRUST` to skip yaw terms in places.
- Simulator: Use `Rudder` for multirotor.

## Motor Output Limit & Transient Throttle Limit
- What: Scale/clip motor outputs globally; limit airmode throttle offset transients.
- Fields: `motor_output_limit`, `transient_throttle_limit`.
- Code: `mixer` and `pid_init.c` (`airmodeThrottleOffsetLimit`).
- Simulator: Optional; implement if you want realistic saturation behavior.

## Angle Pitch Offset
- What: Adds a constant offset to pitch in angle mode for trims.
- Fields: `angle_pitch_offset`.
- Code: Used in `pidLevel` within `pid.c`.
- Simulator: Optional.

---

## Recommendations for a Web Simulator
- Implement (high value): I‑term Relax, Feedforward basics (`Kf`, smoothing), Basic TPA, D‑term PT1, D‑Max (optional but nice), Anti‑Gravity (optional).
- Mode features (only if simulating Angle/Horizon): Angle feedforward smoothing, Earth reference, Horizon delay/limits.
- Skip for multirotor: SPA (wing), Advanced TPA speed model.
- Skip unless modeling hardware effects: Dynamic Idle, VBAT Sag Compensation, Thrust Linearization, EZ Landing, Crash/Yaw Recovery, Launch Control.

---

## Pointers into the Code
- `src/main/flight/pid_init.c`: `pidInitConfig`, `pidInitFilters`, advanced TPA init.
- `src/main/flight/pid.c`: main `pidController` loop, crash/yaw recovery, SPA functions, feedforward and D‑Max application.
- `src/main/flight/mixer_init.c`: dynamic idle, VBAT sag and EZ landing runtime mapping.

This overview should help you decide which features to carry over to your web simulator for the fidelity you want, and which to omit to keep things simple.