# Reference firmware analysis

Source: `Hexapod_Arduino.ino` (robot) and `Controller_Arduino_Code.ino` (RC
transmitter, context only — being replaced by a PC GUI over WiFi). Both MIT
licensed, Emre Kalem, 2025. Attribution required on port/republish.

This is a critical read, not a review of style. Where the math is correct I
say so and move on; where it's fragile, ambiguous, or unsafe, I say that
instead.

## 1. Forward kinematics — correct

`calculate_fk` is a standard 3-DOF serial-chain solve: coxa rotates in the
XY plane by `alpha = coxa_angle + mount_angle`, femur pitches by `beta` out
of that plane, tibia pitches by `beta + gamma` (cumulative, relative to the
femur's own line). The `res[]` chain (origin → coxa joint → femur joint →
foot) is built correctly and the world-frame foot position formula is right.

One thing worth naming explicitly because it matters later: `pos.z` in this
function is overloaded. When called from `setup()` it's the leg's **mount
angle in radians** (`origin.z`), not a height. `Vector.z` means "angle" for
`origin` and "height in mm" for `home`/`target`. It works because the two
uses never mix, but it's a landmine for anyone reading the struct cold — the
Python port should use two distinct fields (e.g. `mount_angle_rad` and
`height_mm`), not one overloaded `z`.

## 2. Inverse kinematics — correct, but `fabs()` on the tibia is a latent trap

`calculate_ik` rotates the target into the leg's local frame (see §3 for why
this is exactly the mechanism that answers the mirroring question), solves
the femur/tibia planar 2-link problem via law of cosines, and clamps `D`
into `[|femur - tibia|, femur + tibia]` before the `acos` calls — correct
and necessary to avoid a domain error (NaN) on an unreachable target.

**On `fabs(angles.z)` specifically**: I traced this through algebraically.
`res.z = acos(...) - PI`, and `acos` always returns a value in `[0, π]`
given a clamped, real-valued argument. So `res.z` (gamma, the raw tibia
angle from IK) is **provably in `[-π, 0]` for every reachable target** —
there is no code path where it comes out positive. `fabs()` therefore always
flips a value that is already known to be non-positive. It is not resolving
an ambiguous sign at runtime; the sign is fixed by construction.

That means it isn't a *bug* today, but it is a bad way to write it down, for
two concrete reasons:

- It silently changes the servo convention for one joint only. Coxa and
  femur both use `90 + angle_degrees` (a centered, bipolar convention — 90
  is neutral, and the written value tracks the *sign* of the math). Tibia
  uses `fabs(angle_degrees)` — an unsigned, zero-based convention where 0°
  means "straight" and the servo command never goes through 90 as a neutral
  point at all. Nothing in the code says this is intentional. It only
  happens to be safe because of the acos range proof above — if anyone ever
  changes the IK formula (adds the other elbow-branch solution, changes
  which term gets subtracted from `π`, etc.) `fabs()` will keep compiling
  and keep running, and will now be **silently wrong**, because it was
  never asserting the precondition it depends on.
- It's evidence of a real, undocumented hardware coupling: the tibia servo
  horn must physically be installed such that its 0° matches gamma = 0
  (straight leg). There's no way to verify or correct that in software —
  no per-servo trim, no homing check. If that horn is one spline tooth off
  (a very common hobby-servo assembly error), the knee is wrong by that
  offset on every single pose, forever, with no symptom in code.

Separately: classic 2-link IK has two solutions (elbow up / elbow down).
This code only ever computes one branch (`acos` returns the principal
value) and never checks whether it's the mechanically valid one for this
leg's actual range of motion — it's relying entirely on the geometry being
designed so the single computed branch stays valid across the whole
workspace. That's a normal simplification for a hexapod (you design around
it), but it's implicit, not validated, and worth a comment when ported.

**Recommendation for the port**: replace `fabs(angles.z)` with an explicit
`-angles.z`, and add a one-line comment stating the invariant ("IK always
returns gamma ≤ 0 for reachable targets; this negates it into the tibia
servo's zero-based convention"). Same physical output, but it stops relying
on a reader re-deriving the acos range to trust it, and it will fail loudly
(negative servo command) instead of silently if the invariant is ever
broken.

## 3. Leg mirroring — the math is correct, and here's the proof

This is the question you most need a clear answer on, so I worked it
through with numbers rather than asserting an opinion.

**What the origin-angle rotation actually does.** `origin.z = atan2(y, x)`
is each leg's mount angle around the hexagonal body. `calculate_ik` rotates
the body-frame target into that leg's local frame by `-origin.z` before
solving. This is a **pure rotation** (determinant +1) — it re-expresses
"where is the target relative to body center" as "where is the target
relative to this leg's own outward-pointing axis." It is emphatically not a
mirror/reflection (determinant −1). So the question "does this rotation
handle mechanical mirroring" doesn't quite have a yes/no answer as stated —
rotation and reflection are different operations, and only one of them is
present in this code. What matters is whether that's actually the right
operation for this hardware, which is what I checked next.

**Worked example.** Take the two middle legs: RM at `(0, -105.72)` →
`origin.z = -π/2`, and LM at `(0, 105.72)` → `origin.z = +π/2`. Ask both
legs to move their foot by the same body-frame nudge `+δ` on x ("swing
toward the front"), holding y and z fixed at their home values, and solve
`angles.x` (the coxa angle) for each:

- **RM**: home sits along `-y` from its origin, so before rotation
  `relativeTarget ≈ (δ, -R)` for leg-extension `R > 0`. Rotating by
  `-origin.z = +π/2` gives local `(R, δ)`. `angles.x = atan2(δ, R) ≈ +δ/R`.
  → coxa servo commanded **above** 90.
- **LM**: home sits along `+y`, so before rotation `relativeTarget ≈ (δ, R)`.
  Rotating by `-origin.z = -π/2` gives local `(R, -δ)`. `angles.x =
  atan2(-δ, R) ≈ -δ/R`. → coxa servo commanded **below** 90.

Same body-frame motion, opposite-signed servo commands. This isn't
coincidence — the six mount origins are exact y-mirrors of each other
across the three L/R pairs (RF/LF, RM/LM, RR/LR all share the same x, and
negate y), so `origin.z` for each pair is `θ` and `-θ`, and the algebra
above generalizes to all three pairs by that symmetry. **The coordinate
math genuinely produces mirrored servo commands for mirrored feet.**

**Why this is still not a complete guarantee.** A mirrored math output only
produces mirrored *physical motion* if the servo itself is mounted in a
mirrored physical orientation between the left and right leg of each pair —
i.e. the leg brackets are true mirror-image parts (mirrored STL/laser
pattern), not the same part just relocated and rotated 180° around the
ring. This is the standard construction for hexapod kits (mirrored bracket
pairs are cheap to produce and are the normal way to solve exactly this
problem), and everything about this firmware — no per-leg sign flag, no
comment addressing L/R at all, uniform formula applied identically across
all six legs — is consistent with the author having built it that way and
relying on the hardware to make the uniform formula correct.

**Bottom line**: I cannot verify the physical bracket geometry from source
code alone — that's a hardware fact this file doesn't contain. But the
*software* is doing the right thing (producing genuinely mirrored commands,
correctly derived), and the standard construction for this class of kit
supports it working as-is. The place this can still go wrong is assembly
error, not this code: if even one leg's bracket was printed/assembled as a
non-mirrored duplicate instead of a true mirror image (an easy mistake —
mirrored parts often look nearly identical at a glance), that single leg
will step backwards with no symptom anywhere in software except the robot
visibly limping on power-up. **Recommendation**: add a power-on, one-leg-
at-a-time "wave test" to the port (each leg lifts and taps forward in
sequence, operator visually confirms direction before gait is armed), and
give each leg a `sign: int` / trim field in its config record so a
mis-mirrored leg can be corrected in software instead of requiring a
reprint. This costs nothing when everything is assembled correctly and
saves a rebuild when it isn't.

Femur and tibia need no equivalent mirroring analysis: `l_forward` and `D`
are built from `l_xy` (a vector magnitude, rotation-invariant) and `z`
(never rotated), so beta and gamma come out identical regardless of which
way the leg points. Only the coxa/yaw axis needed the rotation, and it's
the only one that's had that logic applied. Consistent.

## 4. Gait — correct as a tripod pattern, structurally limited beyond that

Leg array order `[RF, RM, RR, LR, LM, LF]` is consecutive around the
physical ring (verified from the mount angles: sorted by angle they run
LF→LM→LR→RR→RM→RF, i.e. the declared array order traverses the ring in one
direction). `legIndex % 2` therefore alternates between genuinely adjacent
legs around the hexagon — {RF, RR, LM} and {RM, LR, LF} — which is the
textbook tripod grouping. This part is right.

Everything past "produce a tripod gait" is hard-coded, though:

- **Duty factor is fixed at 50/50** with a fixed 0.5 phase offset. There's
  no way to get a wave or ripple gait (more feet down at once — slower but
  more stable, useful on uneven ground or at low confidence) without
  rewriting `get_foot_target`'s phase logic.
- **Direction and rotation are bang-bang**, not continuous. `radioData[0]`
  and `radioData[3]` — real analog joystick readings — are thresholded down
  to `{-1, 0, +1}` in `loop()`, throwing away all the proportional control
  the hardware already provides. This is the single easiest win: pass the
  normalized analog value through instead of thresholding it, and use it to
  scale `STEP_LENGTH` and/or the `t += 0.015` phase-rate. (Scaling both
  together roughly preserves gait shape at different speeds; scaling only
  one changes stride length vs. cadence independently — worth deciding
  deliberately rather than defaulting to one.)
- **No strafing.** `step_x_move` only ever perturbs `res.x` (body-frame
  forward). Adding strafe means an independent `step_y_move` from a second
  command axis, and — more subtly — reconsidering whether "move in a
  straight commanded direction" should stay leg-forward-relative or become
  a true 2D body-frame translation vector once both axes exist together.
- **No body height control at runtime.** `DEFAULT_Z` is baked into
  `home.z` once, in `setup()`. Making it live means re-deriving `home.z`
  (or adding a runtime offset) on the fly, and re-checking that the IK
  reachability clamp (§2) doesn't start silently truncating targets near
  the height extremes — raising or lowering the body changes how much
  femur/tibia reach is left for the step itself.
- **Turning is a per-step position offset, not an integrated heading.**
  Rotation rotates each leg's `home_pos` by a fixed `ROTATION_STEP_RAD`
  (15°) every step, applied identically to all six legs — it's a
  legitimate simplified turn-in-place pattern for small angles, but there's
  no persistent heading state anywhere, and no proportional turn rate (same
  bang-bang problem as direction).

None of this is wrong for what it is — a minimal RC-driven tripod walker —
but it should be read as "proof the tripod pattern generator works," not as
a gait engine that speed/strafe/height can be bolted onto without touching
its core structure.

## 5. Safety gaps — everything that can hurt a servo or the chassis

1. **Only the coxa angle is clamped.** `ax = constrain(ax, 45, 135)`; `ay`
   and `az` are written with no `constrain()` at all. `az` happens to be
   provably safe today (§2), but that's a proof, not a guard — nothing
   catches it if the formula changes. `ay` (femur, `beta1 + beta2`) has no
   such proof: `beta1 = atan2(z, l_forward)` can swing toward ±90–180° if
   `l_forward` goes negative (target pulled behind the coxa joint, e.g. by
   a bad gait computation or a bogus target), and there is nothing bounding
   the result before it's written to the servo.
2. **No inter-leg / leg-to-chassis collision awareness at all.** Adjacent
   legs could in principle be commanded toward overlapping space during an
   aggressive direction+rotation combination; nothing checks for it.
3. **Position smoothing is not a velocity limit.** The exponential filter
   (`WALK_SMOOTHING_FACTOR` / `STOP_SMOOTHING_FACTOR = 0.4`) softens jumps
   between consecutive goal targets, but it's a position filter — a large
   discontinuity in `goalTarget` (mode changes, phase-branch boundaries)
   still produces a large single-tick delta (40% of the gap per 10ms), and
   nothing bounds servo slew rate directly.
4. **No failsafe on lost radio link.** `read_radio()` only overwrites
   `radioData` when `radio.available()` is true; on link loss the array
   keeps its last values indefinitely, and the robot keeps walking/turning
   forever in whatever direction was last commanded. No timeout, no
   stop-on-silence. This is the most important item to *not* carry forward
   as-is — a WiFi link dropping mid-walk is at least as likely as an NRF24
   dropout, arguably more so.
5. **Uncontrolled startup motion.** `Leg::init()` writes all three servos
   to 90° immediately on `attach()`, regardless of the leg's actual
   physical rest position (e.g. folded for storage) — then `setup()`
   `delay(1000)`s, then the first `loop()` iteration snaps every leg again
   to its computed home pose. That's two unramped, simultaneous 18-servo
   jumps at every power-on with no position feedback and no slow homing
   sequence — a classic way for a hobby hexapod to kick itself on boot.
6. **Simultaneous 18-servo moves with no staggering.** Related to #5: all
   six legs' three servos move together, every tick. Combined with no
   current/voltage awareness in firmware, a shared supply can sag hard at
   exactly the moments (startup, direction reversals) most servos are
   moving together — a power-system concern, but the firmware's "always
   move all 18 in lockstep" pattern is what triggers it.
7. **Reachability clamping is silent.** When `D` gets clamped in
   `calculate_ik`, the leg quietly does "the closest reachable point"
   instead — there's no signal anywhere that the requested target was
   infeasible. Not unsafe by itself, but it hides upstream bugs (a gait
   computation sending grossly out-of-range targets would produce no
   error, just a leg quietly doing something slightly different than
   requested).
8. **No powered-down/idle state.** Servos hold full torque indefinitely;
   there's no detach/sleep behavior, so manually repositioning a leg while
   powered fights an MG996R's full stall torque (~10 kgf·cm) — a
   hands-on-hardware safety note, not just a firmware one.

## 6. What breaks or degrades moving AVR + Servo.h → ESP32-S3 + PCA9685

- **Degrees→pulse mapping is not portable.** `Servo.write(0-180)` on AVR
  and a hand-rolled PCA9685 tick formula are not guaranteed to produce the
  same physical angle for the same MG996R unit, especially near the 45/135
  clamp boundaries — real servos rarely have perfectly linear, perfectly
  matched 500–2500 µs endpoints. This needs a per-servo calibration record,
  which the reference code has no concept of at all (see §7 SERVO_MAP).
- **Per-leg raw Arduino pin numbers don't generalize.** `servoPins[3]` as
  bare `int`s assumes one MCU pin per servo. Two PCA9685 boards at 9
  channels each need a `(board_addr, channel)` pair per servo, addressed
  over shared I2C — a structurally different addressing model, not a
  find-and-replace.
- **I2C is a bus, not per-pin timers.** Servo.h drives PWM directly per AVR
  pin; PCA9685 updates are I2C register writes with real transaction
  latency. Naively writing 18 servos as 18 separate I2C transactions per
  tick adds up — batch writes (PCA9685 supports auto-incrementing register
  writes to update many channels per transaction) are needed to stay
  comfortably inside a 20 ms (50 Hz) budget. The reference code has zero
  prior art here since it never touched a shared bus.
- **The NRF24 link disappears, but its failure mode must not be re-created.**
  Safety gap #4 above (no timeout on stale radio data) is the one thing
  that must be fixed, not ported, when moving to WiFi — a WiFi/socket
  transport needs its own explicit link-timeout failsafe, arguably more
  urgently than NRF24 needed one.
- **Blocking `delay()` doesn't compose with FreeRTOS.** The reference is a
  single synchronous loop; the ESP32 Arduino core is FreeRTOS-based and
  runs WiFi handling on its own task(s) regardless, but a naive port that
  still calls `delay(10)` in a loop task will block anything else scheduled
  on that task. The control loop should be a proper fixed-rate FreeRTOS
  task/timer, not a delay-paced `loop()`.
- **Non-issues, called out so they don't get "fixed" unnecessarily:**
  float trig (`sin`/`cos`/`atan2`/`acos`) — AVR has no FPU and the
  reference already runs this fine at its loop rate; ESP32-S3 has hardware
  float and headroom to spare, this is a pure win, not a risk. Likewise
  RAM/flash: the Mega's constraints motivated nothing defensive in this
  code, and the ESP32-S3 has no comparable pressure for this workload —
  don't import AVR-era resource paranoia into the port.
- `Serial.begin(9600)` should just become a faster baud (115200+) for
  USB-CDC — trivial, not worth more than a mention.

## 7. Proposed architecture for the port

The brief poses four specific questions. Answering each with reasoning,
since I disagree with the framing on none of them by accident — each has a
concrete failure mode I'm designing against.

### Gait engine: on the ESP32, not PC-computed-and-streamed

Run the gait pattern generator (phase advance, foot-target computation) on
the ESP32, driven by low-rate high-level intent from the PC (direction,
rotation, strafe, speed, height — a handful of floats), not by the PC
computing and streaming 50 Hz foot targets over WiFi.

WiFi is not deterministic, and this repo already has direct evidence of
that — `MJPEGStream` exists specifically because network latency and
frame drops are real and worth measuring (the frame-age overlay). Streaming
foot targets at gait rate makes correctness and smoothness hostage to
per-packet WiFi timing: a dropped or late packet at 50 Hz directly corrupts
the walking motion. Sending sparse, low-rate *intent* instead means the
ESP32 free-runs its local gait phase between packets, and a WiFi hiccup
degrades to "keeps doing what it was last told" for a bounded time before a
link-timeout failsafe engages (fixing safety gap #4), rather than "visibly
stutters or produces an invalid target." This is also just standard
practice — no RC receiver or flight controller runs its real-time control
loop on the far side of a radio link; the link carries intent, the vehicle
carries the loop.

Trade-off I'm accepting: firmware carries more logic (phase generator + IK,
not just IK), and gait tuning means reflashing instead of editing Python.
I'd mitigate that by writing the gait math twice on purpose — a pure,
unit-tested Python reference implementation (satisfying the "gait math…
free of PyQt and sockets, testable" architecture rule, and doubling as a
spec/simulator for tuning step length/height/timing without hardware) that
the C++ firmware implementation is ported from and checked against, plus
exposing enough runtime-tunable parameters (step length, step height, speed
and rotation scale) over the link that most day-to-day tuning never touches
firmware code at all.

### Control loop: fixed-rate, not event-driven on packet arrival

This follows directly from the above: if gait phase lives on the ESP32, it
has to advance on a steady clock regardless of when packets arrive — that's
the entire point. Event-driven-on-packet-arrival would make gait smoothness
a function of WiFi jitter again, reintroducing the exact problem the first
decision avoids. Structure it as a WiFi/socket task that only ever updates
a mutex- or atomic-protected "latest command" (the same "latest, drop
stale" pattern `MJPEGStream` already uses for frames — worth carrying the
principle across, not just the vision code), consumed by a separate
fixed-rate FreeRTOS control task. 50 Hz is a reasonable choice independent
of the reference's 10 ms cadence — it happens to match the PCA9685's native
PWM refresh rate exactly, so there's no risk of commanding position updates
faster than the output hardware can physically move anyway.

### Degrees vs. PCA9685 ticks: degrees everywhere except one leaf function

All kinematics — IK, FK, gait — stay in degrees (or radians internally,
degrees at boundaries, matching the reference's own convention). PCA9685
tick/pulse units should touch exactly one place: a thin, leaf-level
"servo output" function, degrees-in/ticks-out, parameterized by a
per-channel calibration record (min pulse, max pulse, sign, mechanical
trim) rather than one global linear formula — directly addressing the
calibration-drift port risk in §6. This is the same separation the
project's architecture rules already require ("pure logic… free of PyQt and
sockets so it can be unit tested") — the degrees-based math is testable
with nothing attached, and the pulse-conversion function is separately
testable as a pure function given a calibration record, which is exactly
what "all code must run and be testable with zero hardware" requires.

### Per-leg struct vs. flat SERVO_MAP: both, for different concerns

Not a strict either/or — collapsing them loses information either way.

Keep a per-leg structure (ported to a Python/C++ dataclass) for anything
inherently leg-scoped: mount origin, mount angle, segment lengths, current
target — IK is fundamentally a per-leg computation, and the reference's
`Leg` struct is the right shape for that part.

But don't embed raw hardware addressing inside it the way the reference
does with `servoPins[3]`. Once a servo needs `(board_addr, channel,
pulse_min, pulse_max, sign, trim)` instead of just an Arduino pin number,
that data is fundamentally *channel*-scoped, not leg-scoped: calibrating or
debugging on the bench means thinking "channel 7 on board 0x41," not "LR
leg's tibia," and a flat, iterable `SERVO_MAP` (ideally YAML-loaded, per
this project's config conventions) makes wiring mistakes (two channels
swapped) something you can spot and fix by editing one table row, with zero
risk of touching kinematics code. Concretely: a `Leg` dataclass holds
kinematic identity plus references (servo IDs) into the flat map; the map
is the single source of truth for how to command a physical channel, and
it's the only place PCA9685/calibration facts live.

This is a deliberate change from the reference, not a default: on AVR,
"a servo" and "an Arduino pin" were the same thing, so conflating leg
identity and hardware addressing in one struct cost nothing. Once
board+channel addressing and per-channel calibration exist, that
conflation stops being free.

### Module sketch (naming only — no implementation this session)

```
robot/
    protocol.py     # pure: command dataclass + (de)serialization, no sockets
    kinematics.py    # pure: FK/IK, ported from calculate_fk/calculate_ik, unit tested
    gait.py          # pure: phase/foot-target generator, Python reference for the firmware port
    link.py          # RobotLink ABC; WiFiRobotLink + MockRobotLink (always works, no hardware)
```

Firmware (ESP32-S3, description only): a WiFi/socket task updates a
lock-protected "latest command" struct; a fixed 50 Hz FreeRTOS control task
reads it, advances gait phase, runs per-leg IK, and writes through the flat
SERVO_MAP to both PCA9685 boards via batched I2C writes; a link-timeout
watchdog forces a stop if no command has arrived recently; startup runs a
slow homing ramp to neutral instead of an instant `write(90)`, and
optionally the one-leg-at-a-time wave test from §3 before gait is armed.
