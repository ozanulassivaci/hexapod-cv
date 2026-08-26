# hexapod-cv

Vision and control pipeline for a hexapod robot: wireless camera ingest,
color-based object detection, and an operator GUI for manual/auto-track
driving and servo calibration. No robot hardware is assembled yet — every
piece runs and is tested against mocks (`MockRobotLink`) with no camera or
robot attached.

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

## Tech stack

- Python 3.10+
- OpenCV (`opencv-python`)
- NumPy
- PySide6 (operator GUI)
- PyYAML (config)
- pytest

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
├── reference/               # ported firmware + critical analysis (ANALYSIS.md)
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
- No ESP32-S3 firmware yet — the protocol, GUI, and `MockRobotLink` are
  all built and tested, but nothing has run against real servos. See
  `reference/ANALYSIS.md` Section 7 for the planned firmware architecture.
- The calibration and bench tabs' auto-disarm countdowns are client-side
  estimates, not wire-verified — accurate for how `app.py` constructs a
  link, would drift against a link built with a non-default arm timeout.
- Bench mode's stall/dwell protection is enforced by the GUI, not by
  firmware (which doesn't exist yet) — real accident-proofing needs the
  firmware's gait engine and bench mode to be mutually exclusive too. See
  `docs/protocol.md` Section 8.
- `stream/` is named that way (not `io/`) to avoid shadowing Python's
  standard-library `io` module when the project root is on `sys.path`.

## License

Not yet licensed for reuse.
