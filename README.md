# hexapod-cv

Vision and control pipeline for a hexapod robot: wireless camera ingest,
color-based object detection, an operator GUI for manual/auto-track
driving and servo calibration, ESP32-S3 firmware implementing a tripod
gait engine and bench-testing tools, and a software simulator for
exercising gait and the link failsafe with no hardware at all. No robot
is assembled yet — every PC-side piece runs and is tested against mocks
(`MockRobotLink`) or the simulator (`SimRobotLink`) with no camera or
robot attached, and the firmware's safety-relevant logic (link watchdog,
arm gates, dwell protection, limit enforcement, kinematics, gait) is unit
tested on the host with no hardware either.

![demo placeholder](docs/demo.gif)
<!-- TODO: replace with a real screenshot or GIF of the live viewer -->

## Features

- Pulls an MJPEG stream over WiFi from a phone/iPod running DroidCam
- Background-thread reader that always serves the latest frame — stale
  frames are dropped instead of queued, so display latency tracks network
  latency rather than a growing backlog
- HSV-threshold color detector behind a `Detector` interface, so a future
  YOLOv8 backend can be swapped in without touching the stream or display code
- Live HSV trackbar tuner for re-calibrating thresholds when lighting changes
- Automatic reconnect with exponential backoff if the stream is unreachable
  or drops mid-run
- On-screen FPS and frame-age (ms) overlay to quantify latency
- JSON-over-UDP command protocol (`transport/`) with intent-based commands,
  a link-timeout failsafe, and a `MockRobotLink` that runs with zero
  hardware or network — see `docs/protocol.md`
- Operator GUI (`app.py`) with manual WASD/arrow-key driving, an
  AUTO_TRACK mode that steers toward the largest detection
  (`control/tracker.py`), a calibration tab (arm/disarm gate, per-servo
  trim, bulk offset read/export/import), and a Bench Test tab for
  characterizing loose servos before assembly — direct raw-pulse control,
  its own arm gate, stall/dwell protection (`control/bench.py`) — all of
  it runs fully against `MockRobotLink` with no camera and no robot
  attached
- ESP32-S3 firmware (`firmware/`, PlatformIO) implementing the full
  protocol: link watchdog, both arm gates, per-channel dwell protection,
  and limit enforcement that's impossible to bypass from any command path
  (`GatedServoDriver`), plus a tripod gait engine (continuous speed
  scaling, strafing, runtime body height, slew-rate-limited joints) that
  converts `walk`/`turn`/`body_height` into servo pulses through that same
  enforcement point. A `ROBOT_ASSEMBLED` build flag selects what "safe"
  means on a link timeout or OTA start — release every servo (default, no
  legs attached yet) or hold the last commanded position (once assembled
  — releasing a loaded joint would drop it). WiFi with AP fallback, OTA
  updates gated on nothing being armed and gait being idle, a plain-text
  serial mirror for bench testing without WiFi. See `docs/protocol.md` and
  `docs/HOW_TO_USE.md`
- Pure-Python kinematics/gait reference (`robot/`) that firmware's C++ is
  ported from and cross-checked against via generated golden fixtures
  (`scripts/gen_kinematics_golden.py`, `scripts/gen_gait_golden.py`) —
  two independent implementations kept in lockstep, not shared through a
  compiled extension (see `reference/ANALYSIS.md` Section 7 for why)
- A software simulator (`simulator/`) that runs the same gait engine over
  time on a background thread behind a `RobotLink` implementation
  (`SimRobotLink`), so the operator GUI can drive it with zero changes —
  a top-down 2D view (`SimView`) shows body position/heading and which
  legs are in stance vs. swing, with its own heartbeat (matching
  `UDPRobotLink`'s) so holding a key doesn't spuriously trip the link
  timeout, and a "Simulate link drop" button to actually watch the
  failsafe trip live, since nothing else in normal use ever makes the
  simulated link go quiet. Select it with `operator_config.yaml`'s
  `link.mode: sim` or `python app.py --link-mode sim`

## Tech stack

- Python 3.10+
- OpenCV (`opencv-python`)
- NumPy
- PySide6 (operator GUI)
- PyYAML (config)
- pytest
- ESP32-S3, C++/PlatformIO, ArduinoJson, Adafruit PWM Servo Driver
  Library, Unity (firmware host-side tests)

## Installation

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## DroidCam setup (iPod touch 6th gen, iOS 12.5.8)

1. Install the DroidCam app and connect the iPod to the same WiFi network
   as the processing laptop.
2. In the app, set the video format to **MJPEG** at **640x480**.
3. Lock **exposure** and **white balance** so HSV thresholds stay stable —
   auto-exposure will drift the color readings mid-session.
4. Disable **auto-lock** on the device so the stream doesn't die when the
   screen sleeps.
5. Note the IP address DroidCam shows on screen and set it in
   [config.py](config.py) as `StreamConfig.url`, e.g.
   `http://192.168.1.42:4747/video/640x480`.

## Usage

Run the live viewer:

```bash
python main.py
```

Press `q` or `Esc` to quit.

Re-tune HSV thresholds when lighting changes:

```bash
python tune_hsv.py
```

Adjust the trackbars until the mask (right half of the window) isolates
only the target object, then press `q`. The final `lower`/`upper` values
are printed ready to paste into [config.py](config.py).

Run the tests:

```bash
pytest
```

Run the operator GUI (works with no camera and no robot attached —
defaults to `MockRobotLink`, configured in
[operator_config.yaml](operator_config.yaml)):

```bash
python app.py
python app.py --link-mode sim   # drive the software simulator instead of MockRobotLink/real hardware
```

WASD/arrow keys walk, Q/E turn, Space is an immediate stop reachable
regardless of focus. Any manual key press drops out of AUTO_TRACK. The
Calibrate tab arms/disarms per-servo trim writes and can export/import the
full servo profile table as YAML — export after every calibration
session, since that (not the arm/disarm gate) is what makes a bad write
cheap to undo. The Bench Test tab drives one PCA9685 channel directly (no
leg, no IK) for testing a loose servo before assembly, behind its own arm
gate, with automatic stall protection. Against `--link-mode sim`, a
Simulator tab shows a live top-down view of gait — stance/swing legs and
body position/heading — driven by the same commands.

See [docs/GUI_GUIDE.md](docs/GUI_GUIDE.md) for what every control does,
[docs/HOW_TO_USE.md](docs/HOW_TO_USE.md) for physical operating sequence
(USB is for flashing only — driving and calibration run over WiFi with no
cable connected), and [docs/protocol.md](docs/protocol.md) for the wire
protocol this all runs over.

Build and test the firmware (no ESP32-S3 hardware required for the tests):

```bash
cd firmware
pio test -e native          # host-side Unity tests for the safety-relevant logic
pio run -e esp32-s3-devkitc-1   # cross-compile for the real target
```

Flashing requires `firmware/include/secrets.h` (copy from
`secrets.h.example`, gitignored) with your WiFi/AP/OTA credentials filled
in — see `docs/HOW_TO_USE.md` Section 1 for the full first-time bring-up
and OTA reflashing procedure.

## Project structure

```
hexapod-cv/
├── config.py               # Phase 1 viewer config (stream URL, HSV range, window size)
├── main.py                 # Phase 1 live viewer: wires stream + detector + display
├── tune_hsv.py              # interactive HSV threshold tuner
├── operator_config.py / .yaml  # operator GUI config (stream, detection, link, tracker, ui)
├── app.py                  # operator GUI entry point
├── perception/
│   ├── detection.py         # Detection dataclass, Detector interface
│   └── hsv_detector.py      # HSV-threshold Detector implementation
├── stream/
│   └── mjpeg_stream.py      # background-thread MJPEG reader with reconnect
├── transport/               # PC<->robot command protocol (see docs/protocol.md)
│   ├── protocol.py           # commands, telemetry, encode/decode, validation
│   ├── link.py                # RobotLink ABC: telemetry tracking, failsafe cross-check
│   ├── udp_link.py             # UDP transport: heartbeat resend, telemetry receive
│   ├── mock_link.py             # zero-hardware RobotLink for development
│   └── constants.yaml / generated_constants.py  # shared timeout/version constants
├── control/
│   ├── tracker.py             # AUTO_TRACK steering policy, no Qt, pytest-covered
│   └── bench.py                # bench dwell-guard + sweep math, no Qt, pytest-covered
├── robot/                   # pure Python FK/IK + gait, firmware's C++ is ported from and checked against
│   ├── kinematics.py          # FK/IK for one leg, joint angle clamps
│   └── gait.py                  # tripod phase engine, speed/strafe/height, slew-rate limiting
├── simulator/                # hardware-free RobotLink that runs gait over time
│   ├── robot_state.py         # wraps robot/gait.py's GaitState + a dead-reckoned body pose
│   ├── sim_link.py              # SimRobotLink: RobotLink impl, background stepping + heartbeat
│   └── sim_view.py               # QPainter top-down 2D view (body pose, per-leg stance/swing)
├── ui/
│   ├── main_window.py         # wires video/telemetry/keys/AUTO_TRACK together
│   ├── video_panel.py          # camera feed + detection overlay
│   ├── control_panel.py         # status, telemetry, sliders, mode, e-stop
│   ├── calibration_tab.py        # arm/disarm, per-servo trim, profile export/import
│   ├── bench_tab.py                # raw-pulse bench testing: park/sweep/range-finder
│   ├── log_panel.py                 # scrolling command/event log
│   └── servo_names.py                # servo_index -> leg/joint name for calibration/bench tabs
├── reference/               # ported reference firmware + critical analysis (ANALYSIS.md)
├── firmware/                # ESP32-S3 firmware (PlatformIO) -- implements the full protocol
│   ├── platformio.ini         # native (host tests) + esp32-s3-devkitc-1 (real target) environments
│   ├── include/                 # Config.h (incl. ROBOT_ASSEMBLED), generated constants, secrets.h.example
│   ├── lib/core/                  # pure logic: watchdog, arm gates, dwell guard, protocol codec,
│   │                                 gated servo driver, kinematics, gait engine, servo map --
│   │                                 unit tested on the host, no hardware
│   ├── src/                        # ESP32-only: PCA9685/NVS/WiFi/OTA drivers, main.cpp wiring
│   └── test/                        # Unity tests for lib/core, run via `pio test -e native`
├── docs/
│   ├── protocol.md           # wire protocol design and rationale
│   ├── GUI_GUIDE.md           # what every app control does
│   └── HOW_TO_USE.md           # physical operating sequence, USB vs WiFi
└── tests/                   # pytest suite for all pure logic
```

## Limitations

- Tested only against DroidCam's MJPEG output; other MJPEG sources may
  behave differently.
- HSV thresholding is sensitive to lighting changes — re-run `tune_hsv.py`
  whenever the environment changes.
- Firmware has never run against real servos or a real PCA9685 board —
  verified by real cross-compilation for the target and by host-side unit
  tests, not by physical hardware, which no session so far has had access
  to. `firmware/include/Config.h`'s `PCA9685_OSC_FREQ_BOARD_*_HZ` are
  nominal placeholders until measured per-board (procedure in
  `docs/HOW_TO_USE.md`).
- Gait exists but nothing is assembled — every joint angle bound (femur/
  tibia clamps in `robot/kinematics.py`/`firmware/lib/core/Kinematics.h`),
  the body-height mm range, the per-joint slew-rate limit, and the
  `ServoMap` board/channel wiring are placeholders pending real hardware,
  the same way `PCA9685_OSC_FREQ_BOARD_*_HZ` already was. Only coxa's
  clamp (`[45, 135]°` servo-equivalent) is carried over from the
  reference firmware's own validated value.
- The calibration and bench tabs' auto-disarm countdowns are client-side
  estimates, not wire-verified — accurate for how `app.py` constructs a
  link, would drift against a link built with a non-default arm timeout.
- The simulator's body position/heading (`simulator/robot_state.py`) is
  dead-reckoned purely for the top-down view to have something to draw —
  physical fidelity is explicitly not the goal, its constants aren't
  measured against anything real, and there's nothing on the firmware
  side to cross-check it against (a real hexapod's body moves because its
  legs push against the ground, not because anything computes a
  body-frame transform).
- `stream/` is named that way (not `io/`) to avoid shadowing Python's
  standard-library `io` module when the project root is on `sys.path`.

## License

Not yet licensed for reuse.
