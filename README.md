# hexapod-cv

Vision and control pipeline for a hexapod robot: wireless camera ingest,
color-based object detection, an operator GUI for manual/auto-track
driving and servo calibration, and ESP32-S3 firmware for bench-testing
loose servos before assembly. No robot is assembled yet — every PC-side
piece runs and is tested against mocks (`MockRobotLink`) with no camera or
robot attached, and the firmware's safety-relevant logic (link watchdog,
arm gates, dwell protection, limit enforcement) is unit tested on the host
with no hardware either.

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
- ESP32-S3 firmware (`firmware/`, PlatformIO) implementing the
  bench-testing subset of the protocol: link watchdog, both arm gates,
  per-channel dwell protection, and limit enforcement that's impossible to
  bypass from any command path (`GatedServoDriver`) — no gait/IK yet,
  nothing is assembled. WiFi with AP fallback, OTA updates gated on
  nothing being armed, a plain-text serial mirror for bench testing
  without WiFi. See `docs/protocol.md` and `docs/HOW_TO_USE.md`

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
```

WASD/arrow keys walk, Q/E turn, Space is an immediate stop reachable
regardless of focus. Any manual key press drops out of AUTO_TRACK. The
Calibrate tab arms/disarms per-servo trim writes and can export/import the
full servo profile table as YAML — export after every calibration
session, since that (not the arm/disarm gate) is what makes a bad write
cheap to undo. The Bench Test tab drives one PCA9685 channel directly (no
leg, no IK) for testing a loose servo before assembly, behind its own arm
gate, with automatic stall protection.

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
├── ui/
│   ├── main_window.py         # wires video/telemetry/keys/AUTO_TRACK together
│   ├── video_panel.py          # camera feed + detection overlay
│   ├── control_panel.py         # status, telemetry, sliders, mode, e-stop
│   ├── calibration_tab.py        # arm/disarm, per-servo trim, profile export/import
│   ├── bench_tab.py                # raw-pulse bench testing: park/sweep/range-finder
│   ├── log_panel.py                 # scrolling command/event log
│   └── servo_names.py                # servo_index -> leg/joint name for calibration/bench tabs
├── reference/               # ported reference firmware + critical analysis (ANALYSIS.md)
├── firmware/                # ESP32-S3 firmware (PlatformIO) -- bench-testing subset of the protocol
│   ├── platformio.ini         # native (host tests) + esp32-s3-devkitc-1 (real target) environments
│   ├── include/                 # Config.h, generated constants, secrets.h.example
│   ├── lib/core/                  # pure logic: watchdog, arm gates, dwell guard, protocol codec,
│   │                                 gated servo driver -- unit tested on the host, no hardware
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
- No gait/IK yet — firmware decodes and acknowledges `walk`/`turn`/`stop`/
  `body_height`/`pan_tilt`/`face`, but they have no hardware effect.
  Nothing is assembled yet for them to act on. See `reference/ANALYSIS.md`
  Section 7 for the planned architecture once that phase starts.
- The calibration and bench tabs' auto-disarm countdowns are client-side
  estimates, not wire-verified — accurate for how `app.py` constructs a
  link, would drift against a link built with a non-default arm timeout.
- `MockRobotLink` re-arms `calibration_mode`/`bench_mode` on every
  received `armed=True`, including transport resends — firmware instead
  only arms on the false→true transition, since otherwise an idle-but-
  armed session with a healthy link would never auto-disarm. Known
  discrepancy, not yet fixed on the PC side. See `docs/protocol.md`
  Section 6.
- Bench mode's stall/dwell protection is now enforced independently by
  both the GUI and firmware (firmware releases the channel entirely
  rather than moving it to neutral, and works even if the GUI has
  crashed) — real accident-proofing still needs the firmware's future
  gait engine and bench mode to be mutually exclusive too, once that
  exists. See `docs/protocol.md` Section 8.
- `stream/` is named that way (not `io/`) to avoid shadowing Python's
  standard-library `io` module when the project root is on `sys.path`.

## License

Not yet licensed for reuse.
