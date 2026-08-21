# hexapod-cv

Vision pipeline for a hexapod robot — Phase 1: wireless camera ingest and
color-based object detection.

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

## Tech stack

- Python 3.10+
- OpenCV (`opencv-python`)
- NumPy
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

## Project structure

```
hexapod-cv/
├── config.py              # all tunable parameters (stream URL, HSV range, window size)
├── main.py                # wires stream + detector + display together
├── tune_hsv.py             # interactive HSV threshold tuner
├── perception/
│   ├── detection.py        # Detection dataclass, Detector interface
│   └── hsv_detector.py     # HSV-threshold Detector implementation
├── stream/
│   └── mjpeg_stream.py     # background-thread MJPEG reader with reconnect
└── tests/                  # pytest suite for the pure detection logic
```

## Limitations

- Tested only against DroidCam's MJPEG output; other MJPEG sources may
  behave differently.
- HSV thresholding is sensitive to lighting changes — re-run `tune_hsv.py`
  whenever the environment changes.
- No serial/network output to the robot yet — this phase only validates
  the camera and detection pipeline. Servo control and locomotion are
  later phases.
- `stream/` is named that way (not `io/`) to avoid shadowing Python's
  standard-library `io` module when the project root is on `sys.path`.

## License

Not yet licensed for reuse.
