# hexapod-cv

## Purpose and phase

Vision and control pipeline for a hexapod robot. Phase 1 (camera ingest,
HSV detection) and the bulk of Phase 2 are built and tested: ESP32-S3
firmware with a real gait/IK control loop (`firmware/lib/core/`), an
operator GUI replacing the RC transmitter (`app.py`, `ui/`), a Python
gait/IK reference the firmware is ported from and cross-checked against
(`robot/`), a software simulator (`simulator/`), and AUTO_TRACK closing
the full camera → detection → tracker → gait loop end to end
(`simulator/synthetic_camera.py`). All of it runs and is tested with zero
hardware attached — mocked or simulated (`MockRobotLink`, `SimRobotLink`,
`SyntheticCameraStream`) — this requirement doesn't relax as hardware
work starts.

**Hardware status: servos have arrived, nothing is assembled yet.**
Bring-up is one leg first, not all six at once — compute neutral servo
angles from the kinematics, print and assemble a single test leg,
discover real mechanical limits on that one leg, only then commit to the
full 18-servo build. See `reference/ANALYSIS.md` and `docs/protocol.md`
for what's implemented and why.

Not yet started: YOLOv8 detection backend (the `Detector` ABC already
supports swapping it in without touching stream/display code).

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
- Nothing is assembled yet; servos have arrived. Every module must have a
  hardware-free path to run and test (mocks, synthetic data) — this is a
  hard requirement, not a nice-to-have.

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
  design, not as an edge case. `MJPEGStream` established the pattern
  (background thread, always serve the latest frame, drop stale ones
  instead of queuing); firmware's control loop (`firmware/src/main.cpp`)
  and `SimRobotLink`/`SyntheticCameraStream` follow the same principle —
  free-run at a fixed rate off the latest received command/frame, never
  wait on or get paced by packet/frame arrival. See `reference/ANALYSIS.md`
  §7 for the full reasoning.
- Degrees (or radians internally, degrees at boundaries) everywhere in
  kinematics code. PCA9685 tick/pulse conversion lives in exactly one
  leaf-level function (`firmware/lib/core/AngleToPulse.cpp`), parameterized
  by a per-channel calibration record — it never leaks into IK/gait/FK.
- Servo hardware addressing (board address, channel) is channel-scoped and
  lives in a flat `SERVO_MAP`-style table (`firmware/lib/core/ServoMap.h`
  — currently placeholder wiring, fill in after assembly), not inside
  per-leg structs. Per-leg structs (`Kinematics.h`'s `LegGeometry`) hold
  kinematic identity only (mount origin, segment lengths) and reference
  into the map by servo ID. Per-servo calibration (offset, sign,
  bench-recorded limits) lives in `ServoProfile`/NVS, keyed by servo
  index, not in either of the above.
- Kinematics and gait math exist as two independent implementations —
  Python (`robot/kinematics.py`, `robot/gait.py`) and firmware C++
  (`firmware/lib/core/Kinematics.cpp`, `Gait.cpp`) — kept in lockstep via
  generated golden fixtures (`scripts/gen_kinematics_golden.py`,
  `scripts/gen_gait_golden.py` → `tests/fixtures/*_golden.json` and
  firmware's generated `golden_data.h`), not shared through a compiled
  extension. See `reference/ANALYSIS.md` §7 for why. If you change the
  math on one side, port the change to the other and regenerate the
  fixtures — a stale fixture means the two sides can silently drift.
- Every fault-triggered safe state (link timeout, OTA start, AP-fallback)
  goes through `firmware/lib/core/SafeState.h`'s `enterSafeState()` — no
  other code path may release or hold servos directly on a fault. Its
  behavior is selected once, at compile time, by `Config.h`'s
  `ROBOT_ASSEMBLED` flag: `Bench` (default) releases every servo;
  `Assembled` holds the last commanded position by simply not computing or
  sending a new one. This is a build flag, not a runtime command, and
  deliberately not cross-checked against physical reality — see
  `docs/protocol.md` §9 before changing that.

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
before touching gait/IK/robot-link code. The port is done (both
`robot/kinematics.py`/`robot/gait.py` and their firmware C++ equivalents,
`firmware/lib/core/Kinematics.cpp`/`Gait.cpp`); this is the record of what
was kept vs. deliberately changed, and one item flagged below is still not
built.

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

**Changed on port (all done unless noted):**
- `fabs(angles.z)` on the tibia → explicit `-angles.z` (`to_servo_deg()` /
  `toServoDeg()`), with a comment stating the invariant it depends on
  (gamma is provably ≤ 0; see `ANALYSIS.md` §2). Same output, fails loudly
  instead of silently if the invariant is ever broken
- Bang-bang direction/rotation (`{-1, 0, +1}` thresholding of analog input)
  → continuous scalar (`speed` 0–100), scaling both step length and phase
  rate together
- No strafing → independent lateral command axis (`vy`)
- No runtime body height → live command, mapped onto `home_z_mm()`'s range
  at read time, not a `setup()`-time constant
- `servoPins[3]` raw pin numbers → flat `SERVO_MAP` with board/channel
  addressing (architecture rule above); per-servo calibration lives
  separately in `ServoProfile`
- No link-timeout failsafe → implemented (`LinkWatchdog`,
  `ANALYSIS.md` §5, §7)
- Instant `write(90)` on boot, two unramped startup jumps → **partially
  done.** A per-joint slew-rate limit (`MAX_SLEW_DEG_PER_S` /
  `kMaxSlewDegPerS`) bounds every commanded angle change, closing the
  "unramped jump" gap. The optional one-leg-at-a-time wave test before
  gait is armed (an assembly-QA check for a horn seated a spline tooth
  off) is **not built** — still a real gap if you're looking for it.
- `constrain()` only on the coxa angle → explicit bounds on all three
  joints (`clamp_joint_angles()`); femur/tibia bounds are currently
  placeholders (full servo travel) pending real mechanical limits — see
  the Test Leg bring-up work in "Purpose and phase"
- NRF24/RF24 radio stack — dropped entirely, replaced by the PC GUI over
  WiFi. `Controller_Arduino_Code.ino` (RC transmitter) is not being ported
  at all, context only

**Not carried forward as risk (verified non-issues, don't over-fix):**
float trig performance (ESP32-S3 has an FPU, AVR didn't and it was already
fine), RAM/flash headroom (no pressure on ESP32-S3 for this workload)
