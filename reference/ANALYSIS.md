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

## 3. Leg mirroring — corrected: single leg STL, no L/R variants

**Correction note**: the first pass of this analysis concluded the code
required mirrored (mirror-image) leg brackets to be correct, based on a
worked example. That conclusion was wrong — not the arithmetic, the
inference drawn from it. Confirmed by checking the STL files: all six legs
are printed from the **same** part, no mirrored variant exists. Re-derived
below with the correct diagnostic, and the corrected conclusion is the
opposite of the original one: uniform, non-mirrored parts are exactly what
this code requires, and mirrored parts are what would have broken it.

**Where the original derivation went wrong.** It tested "move the foot by
the same body-frame nudge `+δ` toward the front" for RM and LM, and found
opposite-signed coxa deltas (`+δ/R` for RM, `-δ/R` for LM). That arithmetic
is correct, but it doesn't test what it was used to argue: a body-frame
*translation* target naturally requires opposite-signed local coxa deltas
at two mount angles that are themselves mirror images (`origin.z = -90°`
vs `+90°`) — regardless of whether the underlying hardware is mirrored or
identical. It conflates "where around the ring the leg sits" (which
legitimately flips the sign for a translation test) with "which rotational
sense the servo drives" (which is the thing that actually depends on part
chirality). Those are different questions; the translation test only
answers the first one.

**The correct diagnostic: does the same servo-command direction always
produce the same absolute rotational sense?** From `calculate_fk`,
`alpha_world = angles.x + pos.z` where `pos.z = origin.z`. `calculate_ik`
solves `angles.x` as exactly the inverse of that relation, so structurally,
for every leg:

```
d(alpha_world) / d(angles.x) = 1
```

This holds identically for all six legs — it falls straight out of the
formula, no per-leg case analysis needed, because `origin.z` is a per-leg
*constant offset*, not something that changes the derivative. That means
increasing the coxa servo command always rotates the foot in the same
absolute (body-frame) rotational sense, for every leg, with no sign flip
anywhere. Concrete check with RM (`origin.z = -90°`) and LM (`origin.z =
+90°`), nudging `angles.x` by `+1°` at each leg's neutral pose:

- **RM**: `alpha_world` goes from `-90°` to `-89°` — rotating toward `0°`
  (front). CCW in standard math convention.
- **LM**: `alpha_world` goes from `+90°` to `+91°` — rotating toward `180°`
  (rear). Also CCW — same sense, `+1°` in, `+1°` out, both cases.

Both legs rotate CCW for the same `+1°` command. That RM's foot happens to
swing toward the front while LM's swings toward the rear is just where
each leg already was in the rotation — not a sign inconsistency. (This is
also why the translation test in the original pass showed opposite signs:
asking both legs to swing toward the front is asking for *opposite*
rotational senses, CCW for RM, CW for LM, given where each one starts. That
was always going to produce opposite-signed commands, mirrored parts or
not — it's not evidence either way, which is exactly the mistake.)

**Why "always the same rotational sense" is exactly what a single-part
design needs.** `origin.z = atan2(y, x)` and the `-origin.z` rotation
`calculate_ik` applies are both **pure rotations** (determinant +1), never
reflections (determinant −1), anywhere in this code. A pure rotation
preserves handedness. If all six legs are the same rigid part, just yawed
into place by a pure rotation, then "increasing the coxa servo command"
drives the *same* absolute rotational sense on every leg by construction —
which is precisely what the derivative above shows the code assumes. Code
and hardware match.

Had the legs instead been true mirror-image parts (a reflection, not a
rotation, applied to one side), the servo-to-motion relationship would
flip sign between L and R, and this uniform, rotation-only formula would
have been wrong for one side — needing an explicit per-leg sign flip that
appears nowhere in the source. **So the earlier conclusion had it
backwards: it's the mirrored-parts case that would have needed per-leg
sign inversion, and the single-part case — the one actually built — is
exactly the case this uniform formula is correct for, with no inversion
needed anywhere.**

Femur and tibia don't need separate treatment, for the same reason as
before, now on firmer ground: `l_forward` and `D` are built from `l_xy` (a
vector magnitude, rotation-invariant) and `z` (never rotated), so beta and
gamma come out identical regardless of mount angle. Since the whole leg —
coxa, femur, and tibia together — is one rigid part carried along by a
pure rotation, there's no chirality flip anywhere in the assembly for any
of the three joints to compensate for.

**What this changes about risk.** The architecture question is closed —
there's no more "which of two valid hardware constructions did we build"
uncertainty, and the uniform `90 + angle` treatment is correct as written,
for all three joints, on all six legs. The remaining risk is narrower:
plain assembly/calibration variance — a coxa horn seated one spline tooth
off, a leg not perfectly square on its mount — which can happen to any
single identical leg independent of any L/R question. **Recommendation
(revised, see §7 for how this lands in the port):** keep the power-on,
one-leg-at-a-time wave test — it's now clearly framed as an assembly-QA
check, not a hedge against an architecture that turned out not to exist.
Keep a per-leg trim field in the servo calibration record, but its default
is now known: `0` (no correction) for all six legs, uniformly, since the
uniform formula is confirmed correct — it exists purely to absorb bench-
measured assembly variance on an individual leg, not to encode an L/R
distinction that doesn't exist in this hardware.

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
