/**
 * Betaflight Rate Curve Implementation (TypeScript Port)
 * 
 * This file contains the port of the Betaflight RC rate calculation logic.
 * 
 * Source Information:
 * - Original C File: src/main/fc/rc.c
 * - Original Function: applyBetaflightRates (lines ~186 in the inspected version)
 * - Usage Context: Called within processRcCommand loop in rc.c
 * 
 * The logic converts normalized RC stick inputs into desired angular velocities (setpoints)
 * for the PID controller.
 * 
 * Architecture Note:
 * Rate handling happens BEFORE the PID loop.
 * 1. RC Stick Input -> Normalized to [-1.0, 1.0]
 * 2. Rate Application (this file) -> Calculates Setpoint Rate (deg/s)
 * 3. PID Controller -> Compares Setpoint Rate with Gyro Rate to calculate motor output
 */

import { ControlRateConfig } from './profiles'; // Adjust import path as needed
import { clampf } from './utils'; // Adjust import path as needed

/**
 * Constant defined in src/main/fc/rc.c
 * #define RC_RATE_INCREMENTAL 14.54f
 * Used to adjust the curve slope when RC Rate > 2.0
 */
const RC_RATE_INCREMENTAL = 14.54;

/**
 * Helper function matching Betaflight's power3 macro in src/main/common/maths.h
 * #define power3(x) ((x)*(x)*(x))
 */
function power3(x: number): number {
    return x * x * x;
}

/**
 * Helper alias to match C style constrainf, using our existing clampf utility.
 */
function constrainf(amt: number, low: number, high: number): number {
    return clampf(amt, low, high);
}

/**
 * Calculates the angular velocity (setpoint) based on stick input using Betaflight rate curves.
 * 
 * Port of static float applyBetaflightRates(const int axis, float rcCommandf, const float rcCommandfAbs)
 * from src/main/fc/rc.c
 * 
 * Parameter Mapping to C struct controlRateConfig_t:
 * - rcRates[] -> ControlRateConfig.rcRates[] (Standard RC Rate)
 * - rcExpo[]  -> ControlRateConfig.rcExpo[]  (Expo Factor)
 * - rates[]   -> ControlRateConfig.rates[]   (Super Rate / Rate PC)
 * 
 * @param axis - The axis index (0: Roll, 1: Pitch, 2: Yaw)
 * @param rcCommandf - The stick input normalized to [-1.0, 1.0] (0 is center)
 * @param rcCommandfAbs - The absolute value of the stick input [0.0, 1.0]
 * @param currentControlRateProfile - The rate configuration profile
 * @returns The calculated angular rate in degrees per second
 */
export function applyBetaflightRates(
    axis: number, 
    rcCommandf: number, 
    rcCommandfAbs: number, 
    currentControlRateProfile: ControlRateConfig
): number {
    
    // ---------------------------------------------------------
    // 1. Apply Expo
    // Source: rc.c
    // if (currentControlRateProfile->rcExpo[axis]) {
    //     const float expof = currentControlRateProfile->rcExpo[axis] / 100.0f;
    //     rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    // }
    // ---------------------------------------------------------
    if (currentControlRateProfile.rcExpo[axis]) {
        const expof = currentControlRateProfile.rcExpo[axis] / 100.0;
        rcCommandf = rcCommandf * power3(rcCommandfAbs) * expof + rcCommandf * (1 - expof);
    }

    // ---------------------------------------------------------
    // 2. Apply RC Rate
    // Source: rc.c
    // float rcRate = currentControlRateProfile->rcRates[axis] / 100.0f;
    // if (rcRate > 2.0f) {
    //     rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0f);
    // }
    // ---------------------------------------------------------
    let rcRate = currentControlRateProfile.rcRates[axis] / 100.0;
    if (rcRate > 2.0) {
        rcRate += RC_RATE_INCREMENTAL * (rcRate - 2.0);
    }

    // ---------------------------------------------------------
    // 3. Calculate Base Angle Rate
    // Source: rc.c
    // float angleRate = 200.0f * rcRate * rcCommandf;
    // ---------------------------------------------------------
    let angleRate = 200.0 * rcRate * rcCommandf;

    // ---------------------------------------------------------
    // 4. Apply Super Rate (Rate PC)
    // Source: rc.c
    // if (currentControlRateProfile->rates[axis]) {
    //     const float rcSuperfactor = 1.0f / (constrainf(1.0f - (rcCommandfAbs * (currentControlRateProfile->rates[axis] / 100.0f)), 0.01f, 1.00f));
    //     angleRate *= rcSuperfactor;
    // }
    // ---------------------------------------------------------
    if (currentControlRateProfile.rates[axis]) {
        const rcSuperfactor = 1.0 / (constrainf(1.0 - (rcCommandfAbs * (currentControlRateProfile.rates[axis] / 100.0)), 0.01, 1.00));
        angleRate *= rcSuperfactor;
    }

    return angleRate;
}
