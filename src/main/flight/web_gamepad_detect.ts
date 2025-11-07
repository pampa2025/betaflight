// Gamepad detection and logging utilities for browser environments.
// Framework-agnostic: prints to console and exposes helpers to subscribe to
// connect/disconnect events, enumerate pads, and start a polling logger.

export interface GamepadLoggerOptions {
  initialScan: boolean;      // log currently connected pads immediately
  pollIntervalMs: number;    // 0 disables polling; otherwise logs summaries periodically
  logAxes: boolean;          // include axes values in summaries
  logButtons: boolean;       // include pressed buttons in summaries
  indexFilter?: number[];    // restrict logging to specific indices
}

export function normalizeLoggerOptions(inOpts?: Partial<GamepadLoggerOptions>): GamepadLoggerOptions {
  return {
    initialScan: inOpts?.initialScan ?? true,
    pollIntervalMs: inOpts?.pollIntervalMs ?? 0,
    logAxes: inOpts?.logAxes ?? true,
    logButtons: inOpts?.logButtons ?? true,
    indexFilter: inOpts?.indexFilter,
  };
}

export function getConnectedGamepads(indexFilter?: number[]): Gamepad[] {
  const list = getGamepadsSafe();
  if (!list) return [];
  const pads = Array.from(list).filter((gp): gp is Gamepad => !!gp && gp.connected);
  if (indexFilter && indexFilter.length > 0) return pads.filter(gp => indexFilter.includes(gp.index));
  return pads;
}

export function formatGamepadHeader(gp: Gamepad): string {
  return `[gamepad] #${gp.index} id="${gp.id}" mapping="${gp.mapping}" axes=${gp.axes.length} buttons=${gp.buttons.length}`;
}

export function formatGamepadSummary(gp: Gamepad, opts?: { logAxes?: boolean; logButtons?: boolean }): string {
  const base = formatGamepadHeader(gp);
  const parts: string[] = [base];
  if (opts?.logAxes) {
    const axesStr = gp.axes.map(a => Number.isFinite(a) ? a.toFixed(2) : '0.00').join(',');
    parts.push(`axes=[${axesStr}]`);
  }
  if (opts?.logButtons) {
    const pressed = gp.buttons
      .map((b, i) => (b && b.pressed) ? i : -1)
      .filter(i => i >= 0);
    parts.push(`pressedButtons=${pressed.length ? pressed.join(',') : 'none'}`);
  }
  return parts.join(' ');
}

export function logGamepad(gp: Gamepad): void {
  console.info(formatGamepadHeader(gp));
}

export function onGamepadConnection(
  onConnect?: (gp: Gamepad) => void,
  onDisconnect?: (gp: Gamepad) => void,
): () => void {
  const apiAvailable = isGamepadApiAvailable();
  if (!apiAvailable) {
    console.warn('[gamepad] Gamepad API not available in this environment');
    return () => {};
  }
  const connectHandler = (ev: GamepadEvent) => {
    const gp = ev.gamepad;
    console.info(`[gamepad] connected ${formatGamepadHeader(gp)}`);
    onConnect?.(gp);
  };
  const disconnectHandler = (ev: GamepadEvent) => {
    const gp = ev.gamepad;
    console.info(`[gamepad] disconnected ${formatGamepadHeader(gp)}`);
    onDisconnect?.(gp);
  };
  window.addEventListener('gamepadconnected', connectHandler);
  window.addEventListener('gamepaddisconnected', disconnectHandler);
  return () => {
    window.removeEventListener('gamepadconnected', connectHandler);
    window.removeEventListener('gamepaddisconnected', disconnectHandler);
  };
}

export function startGamepadLogger(inOpts?: Partial<GamepadLoggerOptions>): { stop: () => void } {
  const opts = normalizeLoggerOptions(inOpts);
  if (!isGamepadApiAvailable()) {
    console.warn('[gamepad] Gamepad API not available; logger disabled');
    return { stop: () => {} };
  }

  const unsubscribe = onGamepadConnection();

  if (opts.initialScan) {
    const pads = getConnectedGamepads(opts.indexFilter);
    if (pads.length === 0) {
      console.info('[gamepad] no connected gamepads');
    } else {
      for (const gp of pads) console.info(`[gamepad] detected ${formatGamepadHeader(gp)}`);
    }
  }

  let timer: any = null;
  if (opts.pollIntervalMs > 0) {
    timer = setInterval(() => {
      const pads = getConnectedGamepads(opts.indexFilter);
      for (const gp of pads) {
        console.info(formatGamepadSummary(gp, { logAxes: opts.logAxes, logButtons: opts.logButtons }));
      }
    }, Math.max(50, Math.floor(opts.pollIntervalMs)));
  }

  return {
    stop: () => {
      unsubscribe();
      if (timer) clearInterval(timer);
    },
  };
}

// --- internals --------------------------------------------------------------

function isGamepadApiAvailable(): boolean {
  try {
    const nav: any = (typeof navigator !== 'undefined') ? navigator : null;
    return !!(nav && typeof nav.getGamepads === 'function');
  } catch {
    return false;
  }
}

function getGamepadsSafe(): (Gamepad | null)[] | null {
  try {
    const nav: any = (typeof navigator !== 'undefined') ? navigator : null;
    return nav && typeof nav.getGamepads === 'function' ? nav.getGamepads() : null;
  } catch {
    return null;
  }
}