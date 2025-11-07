Tiny standalone demo for the web control loop.

How to run:

1) Bundle the demo with esbuild:

   npx esbuild src/main/flight/demo/web/main.ts --bundle --format=esm --outfile=src/main/flight/demo/web/bundle.js

2) Serve the repo root and open the demo:

   python3 -m http.server 5173

   Then open http://localhost:5173/src/main/flight/demo/web/

Notes:
- The demo uses RAF scheduling and a fixed-step PID control loop at the selected control rate.
- Gamepad input is sampled at ~60 Hz and bridged to the control loop via web feedforward smoothing.
- Press any gamepad button once so the browser recognizes the controller.