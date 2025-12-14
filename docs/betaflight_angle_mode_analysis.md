# Betaflight Self-Leveling (Angle Mode) Analysis

> **Date:** December 14, 2025  
> **Purpose:** Document the RC stick-to-angle mapping in Betaflight's self-leveling flight mode for ongoing development reference.

---

## Table of Contents

1. [Overview](#overview)
2. [Key Files and Locations](#key-files-and-locations)
3. [Data Flow Pipeline](#data-flow-pipeline)
4. [Detailed Function Analysis](#detailed-function-analysis)
5. [Configuration Parameters](#configuration-parameters)
6. [Mathematical Formulas](#mathematical-formulas)
7. [Simplified Implementation](#simplified-implementation)
8. [References](#references)

---

## Overview

Betaflight's **Angle Mode** (also called self-leveling or self-stable mode) converts RC controller stick inputs into target rotation angles for pitch and roll axes. Unlike Acro/Rate mode where sticks control rotation rate (degrees/second), Angle mode maps stick position directly to a target tilt angle.

### Flight Modes Related to Self-Leveling

| Mode              | Description                                                |
| ----------------- | ---------------------------------------------------------- |
| `ANGLE_MODE`      | Pure angle control - stick position = target angle         |
| `HORIZON_MODE`    | Hybrid - angle control at center, rate control at extremes |
| `GPS_RESCUE_MODE` | Uses angle control for automated return                    |
| `POS_HOLD_MODE`   | Position hold using angle control                          |
| `ALT_HOLD_MODE`   | Altitude hold (often combined with angle)                  |

---

## Key Files and Locations

### Core Implementation Files

| File                           | Purpose                                             |
| ------------------------------ | --------------------------------------------------- |
| `src/main/flight/pid.c`        | Main PID controller, contains `pidLevel()` function |
| `src/main/flight/pid_init.c`   | PID initialization, angle gain configuration        |
| `src/main/fc/rc.c`             | RC command processing, stick-to-rate conversion     |
| `src/main/fc/runtime_config.h` | Flight mode flags definition                        |
| `src/main/flight/imu.c`        | Attitude estimation (provides current angle)        |

### Key Functions

| Function             | File       | Purpose                                                   |
| -------------------- | ---------- | --------------------------------------------------------- |
| `updateRcCommands()` | rc.c       | Converts raw RC data to `rcCommand[]`                     |
| `processRcCommand()` | rc.c       | Converts `rcCommand` to rate setpoint                     |
| `applyRates()`       | rc.c       | Applies rate curves (Betaflight, KISS, etc.)              |
| `pidLevel()`         | pid.c      | **Core angle mode logic** - converts rate to angle target |
| `pidController()`    | pid.c      | Main PID loop, calls `pidLevel()` when in angle mode      |
| `pidInitConfig()`    | pid_init.c | Initializes angle gain from profile                       |

---

## Data Flow Pipeline

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           RC RECEIVER DATA                                   │
│                      (PWM: 1000-2000, centered at 1500)                     │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  updateRcCommands() - rc.c:720-731                                          │
│  ────────────────────────────────                                           │
│  • Subtract midrc (1500) → range: -500 to +500                              │
│  • Apply deadband                                                           │
│  • Store in rcCommand[ROLL], rcCommand[PITCH], rcCommand[YAW]               │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  processRcCommand() - rc.c:640-710                                          │
│  ─────────────────────────────────                                          │
│  • Normalize: rcCommandf = rcCommand[axis] / 500  → range: -1.0 to +1.0     │
│  • Apply rates curve: angleRate = applyRates(axis, rcCommandf)              │
│  • Store in rawSetpoint[axis] (degrees/second)                              │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  pidController() - pid.c:1160-1450                                          │
│  ─────────────────────────────────                                          │
│  • Check if ANGLE_MODE or HORIZON_MODE active                               │
│  • For roll/pitch: call pidLevel() to convert rate setpoint to angle-based  │
│  • Continue with PID calculation                                            │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  pidLevel() - pid.c:590-670  *** CORE ANGLE MODE FUNCTION ***               │
│  ────────────────────────────────────────────────────────────               │
│  1. angleTarget = angleLimit × currentPidSetpoint × (1/maxRate)             │
│  2. currentAngle = attitude.raw[axis] / 10.0  (from IMU)                    │
│  3. errorAngle = angleTarget - currentAngle                                 │
│  4. angleRate = errorAngle × angleGain + angleFeedforward                   │
│  5. Return angleRate as new setpoint for PID                                │
└─────────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PID OUTPUT                                      │
│                    (Motor commands to achieve target angle)                  │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## Detailed Function Analysis

### 1. updateRcCommands() - Initial RC Processing

**Location:** `src/main/fc/rc.c` lines 720-731

```c
for (int axis = 0; axis < 3; axis++) {
    // Center around midrc, constrain to -500 to 500
    float rc = constrainf(rcData[axis] - rxConfig()->midrc, -500.0f, 500.0f);

    // Apply deadband
    float rcDeadband = (axis == YAW) ? rcControlsConfig()->yaw_deadband
                                     : rcControlsConfig()->deadband;
    rcCommand[axis] = fapplyDeadband(rc, rcDeadband);
}
```

**Output:** `rcCommand[]` array with values from -500 to +500 (after deadband)

---

### 2. processRcCommand() - Rate Conversion

**Location:** `src/main/fc/rc.c` lines 640-710

```c
for (int axis = FD_ROLL; axis <= FD_YAW; axis++) {
    // Normalize to -1.0 to 1.0
    float rcCommandf;
    if (axis == FD_YAW) {
        rcCommandf = rcCommand[axis] / rcCommandYawDivider;
    } else {
        rcCommandf = rcCommand[axis] / rcCommandDivider;
    }
    rcDeflection[axis] = rcCommandf;

    // Apply configured rates curve
    angleRate = applyRates(axis, rcCommandf, rcCommandfAbs);

    // Store as raw setpoint (degrees/second)
    rawSetpoint[axis] = constrainf(angleRate, -rateLimit, rateLimit);
}
```

### 3. Rate Curves (applyRates functions)

Betaflight supports multiple rate curve types:

#### Betaflight Rates (default)

```c
// rc.c:186-200
float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
if (rcRate > 2.0f) {
    rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
}

// Apply expo
if (currentControlRateProfile->rcExpo[axis]) {
    const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
}

float angleRate = 200.0f * rcRate * rcCommandf;

// Apply super rate
if (currentControlRateProfile->rates[axis]) {
    const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * rates/100), 0.01f, 1.00f));
    angleRate *= rcSuperfactor;
}
```

---

### 4. pidLevel() - Core Angle Mode Logic

**Location:** `src/main/flight/pid.c` lines 590-670

This is the **key function** that converts rate setpoints to angle-based control:

```c
STATIC_UNIT_TESTED FAST_CODE_NOINLINE float pidLevel(
    int axis,
    const pidProfile_t *pidProfile,
    const rollAndPitchTrims_t *angleTrim,
    float currentPidSetpoint,      // Rate setpoint from RC (deg/s)
    float horizonLevelStrength     // 0-1 for horizon mode blending
) {
    // Maximum allowed angle from profile (default 60°)
    float angleLimit = pidProfile->angle_limit;

    // Inverse of max rate for normalization
    const float maxSetpointRateInv = 1.0f / getMaxRcRate(axis);

    // === STEP 1: Convert rate setpoint to angle target ===
    // This maps the rate value back to a normalized stick position,
    // then scales by angle limit
    float angleTarget = angleLimit * currentPidSetpoint * maxSetpointRateInv;

    // === STEP 2: Apply constraints and offsets ===
    angleTarget = constrainf(angleTarget, -angleLimit, angleLimit);

    // === STEP 3: Get current angle from IMU ===
    // attitude.raw is in decidegrees (0.1° units)
    const float currentAngle = (attitude.raw[axis] - angleTrim->raw[axis]) / 10.0f;

    // === STEP 4: Calculate angle error ===
    const float errorAngle = angleTarget - currentAngle;

    // === STEP 5: Apply angle P gain to get rate output ===
    float angleRate = errorAngle * pidRuntime.angleGain + angleFeedforward;

    // === STEP 6: Apply smoothing filter ===
    angleRate = pt3FilterApply(&pidRuntime.attitudeFilter[axis], angleRate);

    // === STEP 7: Return based on flight mode ===
    if (FLIGHT_MODE(ANGLE_MODE | GPS_RESCUE_MODE | POS_HOLD_MODE)) {
        // Pure angle mode - use calculated angle rate
        return angleRate;
    } else {
        // Horizon mode - blend angle and acro
        return currentPidSetpoint * (1.0f - horizonLevelStrength)
             + angleRate * horizonLevelStrength;
    }
}
```

---

### 5. Angle Gain Initialization

**Location:** `src/main/flight/pid_init.c` line 471

```c
void pidInitConfig(const pidProfile_t *pidProfile) {
    // Angle P gain (level_p / 10)
    pidRuntime.angleGain = pidProfile->pid[PID_LEVEL].P / 10.0f;

    // Angle feedforward gain
    pidRuntime.angleFeedforwardGain = pidProfile->pid[PID_LEVEL].F / 100.0f;

    // Earth reference for coordinated turns
    pidRuntime.angleEarthRef = pidProfile->angle_earth_ref / 100.0f;
}
```

---

## Configuration Parameters

### Profile Settings (pidProfile_t)

| Parameter          | Default | Range | Description                                |
| ------------------ | ------- | ----- | ------------------------------------------ |
| `angle_limit`      | 60      | 10-80 | Maximum tilt angle in degrees              |
| `pid[PID_LEVEL].P` | 50      | 0-200 | Angle P gain (strength of correction)      |
| `pid[PID_LEVEL].I` | 50      | 0-200 | Horizon gain                               |
| `pid[PID_LEVEL].D` | 75      | 0-200 | Horizon transition point                   |
| `pid[PID_LEVEL].F` | 0       | 0-200 | Angle feedforward                          |
| `angle_earth_ref`  | 100     | 0-100 | Earth reference gain for coordinated turns |

### CLI Commands

```bash
# View current settings
get angle_limit
get level_p
get level_f

# Set angle limit to 45 degrees
set angle_limit = 45

# Set angle P gain
set level_p = 50
```

---

## Mathematical Formulas

### Complete Angle Mode Pipeline

#### Step 1: Normalize Stick Input

$$x_{norm} = \frac{rcCommand}{500}$$

Where $x_{norm} \in [-1, 1]$

#### Step 2: Apply Rate Curve (Betaflight default)

$$rate = 200 \times rcRate \times x_{norm} \times superFactor$$

#### Step 3: Convert to Angle Target

$$\theta_{target} = angleLimit \times \frac{rate}{maxRate}$$

This simplifies to:
$$\theta_{target} = angleLimit \times x_{norm} \times \frac{currentRate}{maxRate}$$

#### Step 4: Calculate Angle Error

$$e_{\theta} = \theta_{target} - \theta_{current}$$

Where $\theta_{current}$ comes from IMU attitude estimation.

#### Step 5: Generate Rate Command

$$\omega_{cmd} = e_{\theta} \times K_p + FF$$

Where:

- $K_p$ = `angleGain` = `level_p / 10`
- $FF$ = angle feedforward term

### Simplified Direct Mapping (No PID)

For a simplified implementation without PID, the mapping can be:

#### Linear

$$\theta = x \times maxAngle$$

#### Quadratic (Softer Center)

$$\theta = sign(x) \times x^2 \times maxAngle$$

#### Expo (Adjustable)

$$\theta = x \times (1 - expo + expo \times x^2) \times maxAngle$$

---

## Simplified Implementation

For development purposes, here's a simplified TypeScript implementation that mimics the angle mapping without the full PID infrastructure:

### Quick Reference

```typescript
// Quadratic mapping (recommended for smooth control)
function stickToAngleQuadratic(stick: number, maxAngle: number = 60): number {
  const x = Math.max(-1, Math.min(1, stick / 500));
  return x * Math.abs(x) * maxAngle;
}

// Example usage
const pitchAngle = stickToAngleQuadratic(250, 60); // 250 stick → 15°
const rollAngle = stickToAngleQuadratic(-500, 60); // -500 stick → -60°
```

### Comparison Table

| Stick Input | Linear (°) | Quadratic (°) | Cubic (°) |
| ----------- | ---------- | ------------- | --------- |
| 0 (center)  | 0          | 0             | 0         |
| 125 (25%)   | 15         | 3.75          | 0.94      |
| 250 (50%)   | 30         | 15            | 7.5       |
| 375 (75%)   | 45         | 33.75         | 25.3      |
| 500 (100%)  | 60         | 60            | 60        |

### Full Implementation Files

See these files for complete implementations:

- **TypeScript:** `src/main/flight/angle_direct.ts`
- **C Header:** `src/main/flight/angle_direct.h`
- **C Example:** `src/main/flight/angle_direct_example.c`

---

## References

### Source Files (Betaflight 4.x)

1. **pid.c** - Main PID controller

   - `pidLevel()` function: lines 590-670
   - `pidController()` main loop: lines 1160-1450

2. **pid_init.c** - PID initialization

   - `pidInitConfig()`: lines 456-550
   - Angle gain setup: line 471

3. **rc.c** - RC processing

   - `updateRcCommands()`: lines 720-780
   - `processRcCommand()`: lines 640-710
   - Rate curve functions: lines 186-260

4. **runtime_config.h** - Flight mode definitions
   - `ANGLE_MODE`, `HORIZON_MODE` flags

### Documentation

- [Betaflight PID Feature Guide](betaflight_pid_feature_guide.md)
- [Betaflight Feedforward Tutorial](betaflight-feedforward-tutorial.md)

---

## Development Notes

### Key Insights

1. **Rate-First Architecture**: Even in Angle mode, Betaflight first converts stick to rate, then converts rate to angle target. This allows sharing the same rate curve logic.

2. **Angle Gain**: The `level_p` setting controls how aggressively the quad corrects angle errors. Higher = snappier response but potentially more oscillation.

3. **Earth Reference**: The `angle_earth_ref` feature compensates for yaw-induced roll/pitch coupling during turns.

4. **Filtering**: The final angle rate passes through a PT3 filter at 50Hz cutoff to smooth IMU noise.

### For Custom Implementation

If implementing a simplified angle controller:

1. **Minimum required inputs:**

   - Stick position (-500 to 500)
   - Current attitude from IMU (degrees)
   - Max angle limit

2. **Minimum control loop:**

   ```
   angleTarget = stickToAngle(stick, maxAngle)
   error = angleTarget - currentAngle
   rateCommand = error * Kp
   ```

3. **The rate command then feeds into motor mixing** (not covered here).

---

_Last updated: December 14, 2025_
