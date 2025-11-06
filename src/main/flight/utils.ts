// Shared flight math utilities
// Centralizes common helpers for reuse across modules

export function clampf(x: number, lo: number, hi: number): number {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

export function pt1Alpha(cutoffHz: number, dt: number): number {
    if (cutoffHz <= 0 || dt <= 0) return 1;
    const rc = 1 / (2 * Math.PI * cutoffHz);
    return dt / (rc + dt);
}

export function fapplyDeadband(x: number, deadband: number): number {
    if (deadband <= 0) return x;
    if (x > deadband) return x - deadband;
    if (x < -deadband) return x + deadband;
    return 0;
}

export function scaleRangef(
    x: number,
    inLo: number,
    inHi: number,
    outLo: number,
    outHi: number
): number {
    const spanIn = inHi - inLo;
    if (spanIn === 0) return outLo;
    const t = clampf((x - inLo) / spanIn, 0, 1);
    return outLo + t * (outHi - outLo);
}