# hexapod-cv

## Purpose and phase

Vision and control pipeline for a hexapod robot. Currently **Phase 1**:
camera ingest (MJPEG over WiFi from DroidCam) plus color-based object
detection (HSV) on the PC. No robot hardware is assembled yet — nothing is
built, so all code in this repo must run and be fully testable with zero
hardware attached.

Later phases (not yet started, seams should exist but not be filled in):
YOLOv8 detection backend, PC GUI robot control replacing the RC transmitter
entirely, ESP32-S3 firmware for gait/IK/servo output, vision-guided
locomotion tying detection output to robot commands.

## Hardware

- Chassis: Emre Kalem hexapod, 18x MG996R (3 DOF × 6 legs)
- Leg geometry (from reference firmware): coxa 38.0 mm, femur 86.25 mm,
  tibia 160.5 mm
- Controller: ESP32-S3-DevKitC-1, I2C SDA=GPIO8, SCL=GPIO9
- Servo drivers: 2x PCA9685, I2C addresses 0x40 and 0x41, 9 servos each,
  50 Hz. **6.0V rail is a hard ceiling** — the boards are rated 6V max, do
  not exceed it in any code or doc that mentions supply voltage
- Camera: iPod touch 6 (iOS 12.5.8) running DroidCam, MJPEG over WiFi,
  640x480
- Optional, not yet in scope: SSD1306/SSD1309 OLED face; mini pan-tilt on
  spare PCA9685 channels
- Nothing is assembled. Every module must have a hardware-free path to run
  and test (mocks, synthetic data) — this is a hard requirement, not a
  nice-to-have.

## Architecture rules

- Detector backends stay swappable behind the existing `Detector` ABC
  (`perception/detection.py`). HSV now, YOLO later. Nothing outside
  `perception/` should know which backend is active.
- Robot transport stays swappable behind a `RobotLink` ABC. A
  `MockRobotLink` must always work with no hardware and no network —
  required for testing GUI/protocol/logic code in isolation.
- Pure logic — protocol (de)serialization, IK, gait math, command mapping —
  stays free of PyQt and sockets so it can be unit tested with pytest. If a
  function needs a socket or a Qt widget to be exercised, it's in the wrong
  module.
- Network/hardware links are treated as high-latency and unreliable by
  design, not as an edge case. `MJPEGStream` already establishes the
  pattern (background thread, always serve the latest frame, drop stale
  ones instead of queuing) — the future `RobotLink`/ESP32 side follows the
  same principle for commands: the control loop free-runs at a fixed rate
  off the latest received command, it does not wait on or get paced by
  packet arrival. See `reference/ANALYSIS.md` §7 for the full reasoning.
- Degrees (or radians internally, degrees at boundaries) everywhere in
  kinematics code. PCA9685 tick/pulse conversion lives in exactly one
  leaf-level function, parameterized by a per-channel calibration record —
  it never leaks into IK/gait/FK.
- Servo hardware addressing (board address, channel, pulse calibration,
  sign, trim) is channel-scoped and lives in a flat `SERVO_MAP`-style
  table, not inside per-leg structs. Per-leg structs hold kinematic
  identity only (mount origin, mount angle, segment lengths, current
  target) and reference into the map by servo ID.

## Conventions

- Python 3.10+, type hints on all function signatures, dataclasses for
  structured data
- pytest for all pure logic (kinematics, gait, protocol, detection math);
  do not write tests that require a live socket, camera stream, or
  hardware
- Conventional Commits, one logical change per commit
- Config lives in YAML (or typed dataclasses loaded from it) — no magic
  numbers scattered in code
- MIT-licensed reference firmware (see below) may be ported and republished
  with attribution to Emre Kalem

## Documentation map

- `docs/GUI_GUIDE.md` — what every control in the app does, written for
  someone who has never seen the codebase
- `docs/HOW_TO_USE.md` — physical workflows and order of operations
  (flashing, bench testing, calibration, driving, shutdown) and the safety
  rules around them
- `docs/protocol.md` — wire format and command reference
- `reference/ANALYSIS.md` — assessment of the ported reference firmware

## Keeping docs in sync

`GUI_GUIDE.md` and `HOW_TO_USE.md` are part of the deliverable, not
optional follow-up. In any session where you do one of the following,
update the affected doc in the same session, in the same logical commit as
the change:

- add, remove or rename a control, tab, keybinding or slider
- change what an existing control does, or the units/range it operates in
- add or change a physical procedure, wiring step, or power-sequencing step
- change anything safety-relevant: limits, timeouts, arming behavior,
  failsafe conditions, or anything that could damage a servo or the laptop
- add a concept the user would need explained (as offsets and arming were)

Two hard requirements:
1. Never leave a doc describing behavior that no longer exists. A stale
   safety instruction is worse than no instruction.
2. At the end of every session, state explicitly which docs you updated
   and why — or state that no doc change was needed and why not. Do not
   leave this implicit.

Keep the writing style consistent with what's already there: plain
language, short sentences, numbered steps, written for a tired reader
working late.

## Reference code

`reference/Hexapod_Arduino.ino` and `reference/Controller_Arduino_Code.ino`
are the original MIT-licensed (Emre Kalem, 2025) AVR/NRF24 firmware for
this chassis. Full critical analysis: `reference/ANALYSIS.md` — read it
before touching gait/IK/robot-link code. Summary of what's being kept vs.
deliberately changed on port:

**Kept as-is (verified correct):**
- FK/IK math (`calculate_fk`/`calculate_ik`) — standard 3-DOF leg solve,
  reachability clamping on `D` before `acos`, correct
- Uniform per-leg rotation by `-origin.z`, same formula on all six legs, no
  per-leg sign flip — confirmed correct: all six legs are the same STL, no
  mirrored L/R variant, and the rotation-only (never reflection) math is
  exactly what a single-part, yawed-into-place leg needs (worked derivation
  in `ANALYSIS.md` §3); ported as-is, not reworked
- Tripod grouping by `legIndex % 2` — array order is consecutive around the
  physical ring, so this is the correct tripod pairing

**Changed on port:**
- `fabs(angles.z)` on the tibia → explicit `-angles.z` with a comment
  stating the invariant it depends on (gamma is provably ≤ 0; see
  `ANALYSIS.md` §2). Same output, fails loudly instead of silently if the
  invariant is ever broken
- Bang-bang direction/rotation (`{-1, 0, +1}` thresholding of analog input)
  → continuous scalar, used to scale step length and/or phase rate
  proportionally
- No strafing → add an independent lateral command axis
- No runtime body height → make it a live command, not a `setup()`-time
  constant
- `servoPins[3]` raw pin numbers → flat `SERVO_MAP` with board/channel
  addressing and per-servo calibration (architecture rule above)
- No link-timeout failsafe → required for the WiFi replacement of NRF24,
  more urgent than it was for RC (`ANALYSIS.md` §5, §7)
- Instant `write(90)` on boot, two unramped startup jumps → slow homing
  ramp; optional one-leg-at-a-time wave test before gait is armed, as an
  assembly-QA check (all six legs are the same part, so there's no L/R
  question left — this only catches per-leg assembly/calibration variance,
  e.g. a horn seated a spline tooth off) instead of finding it on the floor
- `constrain()` only on the coxa angle → add explicit bounds on femur too
  (tibia's bound is provable, see above, but keep it explicit)
- NRF24/RF24 radio stack — dropped entirely, replaced by the PC GUI over
  WiFi. `Controller_Arduino_Code.ino` (RC transmitter) is not being ported
  at all, context only

**Not carried forward as risk (verified non-issues, don't over-fix):**
float trig performance (ESP32-S3 has an FPU, AVR didn't and it was already
fine), RAM/flash headroom (no pressure on ESP32-S3 for this workload)
