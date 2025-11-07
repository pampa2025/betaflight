import { startWebControlLoop } from '../../web_control_loop';
import {
	normalizePidMinConfigForWeb,
	normalizePidMinLaunchConfig,
	PidMinLaunchMode,
	Axis3,
} from '../../pid_min';
import {
	pidMinWebFeedforwardUpdateAll,
	normalizePidMinWebFeedforwardRuntime,
	PidMinWebFeedforwardState,
	makePidMinWebFeedforwardMultiState,
} from '../../pid_min_feedforward_web';

type TeleEl = {
	outRoll: HTMLElement;
	outPitch: HTMLElement;
	outYaw: HTMLElement;
	gyroRoll: HTMLElement;
	gyroPitch: HTMLElement;
	gyroYaw: HTMLElement;
	joyRoll: HTMLElement;
	joyPitch: HTMLElement;
	joyYaw: HTMLElement;
	pitchAngle: HTMLElement;
	startBtn: HTMLButtonElement;
	stopBtn: HTMLButtonElement;
	rateInput: HTMLInputElement;
};

const el: TeleEl = {
	outRoll: document.getElementById('outRoll')!,
	outPitch: document.getElementById('outPitch')!,
	outYaw: document.getElementById('outYaw')!,
	gyroRoll: document.getElementById('gyroRoll')!,
	gyroPitch: document.getElementById('gyroPitch')!,
	gyroYaw: document.getElementById('gyroYaw')!,
	joyRoll: document.getElementById('joyRoll')!,
	joyPitch: document.getElementById('joyPitch')!,
	joyYaw: document.getElementById('joyYaw')!,
	pitchAngle: document.getElementById('pitchAngle')!,
	startBtn: document.getElementById('startBtn') as HTMLButtonElement,
	stopBtn: document.getElementById('stopBtn') as HTMLButtonElement,
	rateInput: document.getElementById('rateInput') as HTMLInputElement,
};

// Simple sim state with a crude plant: gyro follows outputs via a PT1 filter
const sim = {
	outputs: { roll: 0, pitch: 0, yaw: 0 } as Axis3<number>,
	gyro: { roll: 0, pitch: 0, yaw: 0 } as Axis3<number>,
	pitchAngleDeg: 0,
	trimPitchDeg: 0,
	armed: true,
	altitudeMeters: 0,
	vSpeedMps: 0,
	throttle01: 0,
	motorsSpinning: true,
	accelZMps2: 0,
};

let handle: { stop: () => void } | null = null;
let lastJoystick: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };

// Multi-axis plot buffers for target vs gyro
type AxisKey = 'roll' | 'pitch' | 'yaw';
const plotBufSize = 600;
const targetBufs: Axis3<number[]> = {
	roll: new Array(plotBufSize).fill(0),
	pitch: new Array(plotBufSize).fill(0),
	yaw: new Array(plotBufSize).fill(0),
};
const gyroBufs: Axis3<number[]> = {
	roll: new Array(plotBufSize).fill(0),
	pitch: new Array(plotBufSize).fill(0),
	yaw: new Array(plotBufSize).fill(0),
};
let plotIdxs: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };
let plotCounts: Axis3<number> = { roll: 0, pitch: 0, yaw: 0 };
const plotCanvases: Axis3<HTMLCanvasElement> = {
	roll: document.getElementById('plotRoll') as HTMLCanvasElement,
	pitch: document.getElementById('plotPitch') as HTMLCanvasElement,
	yaw: document.getElementById('plotYaw') as HTMLCanvasElement,
};
const plotCtxs: Axis3<CanvasRenderingContext2D> = {
	roll: plotCanvases.roll.getContext('2d')!,
	pitch: plotCanvases.pitch.getContext('2d')!,
	yaw: plotCanvases.yaw.getContext('2d')!,
};
let plotYRange = 800;

function appendPlotSample(axis: AxisKey, target: number, gyro: number) {
	const idx = plotIdxs[axis];
	targetBufs[axis][idx] = target;
	gyroBufs[axis][idx] = gyro;
	plotIdxs[axis] = (idx + 1) % plotBufSize;
	plotCounts[axis] = Math.min(plotCounts[axis] + 1, plotBufSize);
}

function renderPlotAxis(axis: AxisKey) {
	const canvas = plotCanvases[axis];
	const ctx = plotCtxs[axis];
	const w = canvas.width;
	const h = canvas.height;
	ctx.clearRect(0, 0, w, h);
	ctx.fillStyle = '#14141c';
	ctx.fillRect(0, 0, w, h);
	ctx.strokeStyle = '#232330';
	ctx.lineWidth = 1;
	for (let i = 0; i <= 4; i++) {
		const y = (i / 4) * h;
		ctx.beginPath();
		ctx.moveTo(0, y);
		ctx.lineTo(w, y);
		ctx.stroke();
	}
	ctx.fillStyle = '#9ea0a8';
	ctx.font = '12px system-ui';
	ctx.fillText(`±${plotYRange} deg/s`, 8, 14);
	ctx.fillText('target: teal, gyro: orange', w - 200, 14);
	const count = plotCounts[axis];
	if (count === 0) return;
	const toY = (v: number) => Math.round(h / 2 - (v / plotYRange) * (h / 2 - 8));
	const stepX = w / (plotBufSize - 1);
	const tb = targetBufs[axis];
	const gb = gyroBufs[axis];
	const baseIdx = plotIdxs[axis];
	ctx.strokeStyle = '#11c2b3';
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
	ctx.strokeStyle = '#ff8a3d';
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

function readGamepadAxes(): Axis3<number> {
	const pads = navigator.getGamepads?.() || [];
	const gp = pads[0];
	if (!gp) return lastJoystick;
	const roll = gp.axes[0] ?? 0;
	const pitch = gp.axes[1] ?? 0;
	const yaw = gp.axes[2] ?? gp.axes[3] ?? 0;
	lastJoystick = { roll, pitch, yaw };
	return lastJoystick;
}

function readGyro(): Axis3<number> {
	return sim.gyro;
}

function updatePlant(outputs: Axis3<number>, dt: number) {
	const tau = 0.02; // plant time constant ~20ms
	const alpha = dt / (dt + tau);
	sim.outputs = outputs;
	sim.gyro.roll += alpha * (outputs.roll - sim.gyro.roll);
	sim.gyro.pitch += alpha * (outputs.pitch - sim.gyro.pitch);
	sim.gyro.yaw += alpha * (outputs.yaw - sim.gyro.yaw);
	sim.pitchAngleDeg += sim.gyro.pitch * dt; // integrate pitch from rate
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
	const controlRateHz = Math.max(
		60,
		Math.min(1000, Number(el.rateInput.value) || 300)
	);
	if (handle) handle.stop();
	el.startBtn.disabled = true;
	el.stopBtn.disabled = false;

	// feedforward runtime/state for plotting target rates
	const webRuntime = normalizePidMinWebFeedforwardRuntime({
		controlRateHz,
		inputRateHz: 60,
	});
	let ffState: Axis3<PidMinWebFeedforwardState> =
		makePidMinWebFeedforwardMultiState();

	handle = startWebControlLoop({
		coeffs: { Kp: 0.9, Ki: 0.15, Kd: 0.02, Kf: 0.0 },
		cfg: normalizePidMinConfigForWeb(
			{ feedforwardMode: 'web' },
			{ controlRateHz, inputRateHz: 60 }
		),
		launch: normalizePidMinLaunchConfig({
			mode: PidMinLaunchMode.Full,
			angleLimitDeg: 25,
		}),
		controlRateHz,
		inputRateHz: 60,
		scheduler: 'raf',
		maxStepsPerTick: 8,
		readJoystick: () => readGamepadAxes(),
		readGyro: () => readGyro(),
		pitchAngleDeg: () => sim.pitchAngleDeg,
		trimPitchDeg: () => sim.trimPitchDeg,
		onOutputs: (outputs, dt) => {
			updatePlant(outputs, dt);
			// compute target rates for plotting (web feedforward mapping)
			const ff = pidMinWebFeedforwardUpdateAll(
				ffState,
				webRuntime,
				lastJoystick,
				dt
			);
			ffState = ff.state;
			appendPlotSample('roll', ff.targetRatesDegS.roll, sim.gyro.roll);
			appendPlotSample('pitch', ff.targetRatesDegS.pitch, sim.gyro.pitch);
			appendPlotSample('yaw', ff.targetRatesDegS.yaw, sim.gyro.yaw);
			renderTelemetry();
		},
	});
}

function stopLoop() {
	if (handle) handle.stop();
	handle = null;
	el.startBtn.disabled = false;
	el.stopBtn.disabled = true;
}

// Hook up UI
el.startBtn.addEventListener('click', startLoop);
el.stopBtn.addEventListener('click', stopLoop);

// Auto-start after a tick
setTimeout(startLoop, 0);

// Keep UI fresh even when outputs don't change (just for UX)
function tickUi() {
	renderTelemetry();
	requestAnimationFrame(tickUi);
}
requestAnimationFrame(tickUi);

// Render plots at RAF cadence
function tickPlot() {
	renderPlotAxis('roll');
	renderPlotAxis('pitch');
	renderPlotAxis('yaw');
	requestAnimationFrame(tickPlot);
}
requestAnimationFrame(tickPlot);
