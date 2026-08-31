# Robot command protocol

Design doc for the PC↔robot command layer. Covers the wire format, every
command, the failsafe contract, and the reasoning behind the six design
questions raised before implementation. All six are resolved; §5 and §6
record the answers as given, including where they changed the design from
what was originally proposed. Implemented in `transport/`.

This layer implements the architecture already committed to in
`reference/ANALYSIS.md` §7 and `CLAUDE.md`: intent-based commands (not
streamed foot targets), a fixed-rate control loop on the robot side that
free-runs off the latest received command, and "latest, drop stale" applied
to commands the same way `MJPEGStream` already applies it to frames.

## 1. Wire format: JSON

Chosen over a packed binary struct. At the actual traffic rate here
(10-20 Hz), size is not a real cost either way — a JSON command packet is on
the order of 100-150 bytes, nowhere near a WiFi UDP packet's practical
budget. The deciding factors are the other two:

- **Drift risk.** A packed struct only decodes correctly if the PC's
  `struct.pack` layout and the firmware's C struct agree byte-for-byte on
  field order, width, and padding, forever. A mismatch doesn't necessarily
  fail loudly — it can just as easily produce a plausible, in-range, *wrong*
  value that range validation never catches, because validation only checks
  whether a number is in bounds, not whether it's the number that was meant.
  JSON fields are addressed by name, so this entire bug class doesn't exist:
  reordering fields on one side does not desynchronize the other.
- **Debuggability without USB.** This is an explicit requirement — being
  able to point a bare UDP listener (or `nc`, or a five-line Python script)
  at the traffic and read it directly, with no firmware-side debug build or
  tooling, is a large practical win for "why isn't the robot moving."

The counterargument — C parsing effort — doesn't hold up at this rate on
this hardware. ArduinoJson is mature and heavily used on ESP32; parsing a
few hundred bytes of JSON at 10-20 Hz is not a meaningful load on an
ESP32-S3. Every packet carries an explicit protocol version (`v`); any
version other than the one the receiver implements is a hard reject
(`ProtocolError`), not a best-effort parse — there is no deployed hardware
yet, so there is no backward-compatibility burden worth building for.

## 2. Heartbeat: resend the last intent, `ping` for idle liveness/RTT

The failsafe requirement is: if the operator holds a command steady for a
long time, the robot must keep executing it, not time out. That only works
if the PC keeps sending *something* the whole time it wants the robot to
keep acting — so the resend behavior is not a separate mechanism layered on
top of commands, it *is* the heartbeat. The PC resends the current intent
at a fixed rate (`HEARTBEAT_INTERVAL_S`, see §5) regardless of whether its
fields changed since the last send. This is exactly why the robot side
free-runs its control loop off "latest received command" rather than
pacing itself to packet arrival (`ANALYSIS.md` §7) — most resends carry no
new information, and that's fine, because the robot isn't waiting on them
to advance anything.

`ping` still exists as its own command, for a case resending intent doesn't
cover: an idle GUI with nothing active to command (robot sitting at `stop`)
still wants to show "connected, 23ms" without inventing a fake motion
command to keep the link warm. Any valid packet — resent intent or ping —
resets the receiver's timeout timer identically; the receiver does not
distinguish "heartbeat" from "command" as separate concepts, only "did a
valid, newer packet arrive recently."

## 3. Telemetry (robot → PC)

Sent as a synchronous reply to every packet the robot processes, not on an
independent timer. This piggybacks on whatever rate the PC is already
sending at (resent intent while driving, idle `ping` otherwise), so there's
no separate telemetry schedule to maintain in firmware, and it gives RTT a
natural request/response pairing: the PC records the send time of a
sequence number, and computes RTT when telemetry echoing that sequence
comes back.

Fields, and why each did or didn't make v1, working backward from "debug
the robot not moving without plugging in USB":

| Field | In v1? | Reasoning |
|---|---|---|
| `seq_echo` | yes | Echo of the last processed command's sequence number. Confirms receipt and is the RTT anchor. |
| `last_applied` | yes | Compact echo of the currently-active command. This is the single most useful field: it's what separates "the command never arrived" (link problem) from "it arrived and the robot is doing something else anyway" (firmware/actuation problem). |
| `fault_flags` | yes (bitfield defined, `LINK_TIMEOUT`/`SERVO_FAULT`/`IK_CLIP` set for real, `ESTOP`/`BROWNOUT` reserved) | Cheap — one integer — and it's the difference between "mystery" and "immediate answer." `ESTOP`/`BROWNOUT` are reserved now so the wire format doesn't change when those fault sources exist later. |
| `gait_phase` | yes, nullable | One float. Narrows a "not moving" report to "control loop alive but leg output isn't happening" when link, echo, and faults all look fine. `null` when not walking. |
| `rail_mv` | reserved, nullable, unpopulated | Needs an ADC + voltage divider that isn't in the current hardware list. Reserving the field now avoids a wire-format change later; nothing sets it yet, and this doc doesn't pretend otherwise. |
| `ik_clip_count`, `ik_clip_worst_mm` | yes | Cumulative count and worst-case magnitude of IK's reachability clamp firing (a leg's foot target was outside the femur/tibia annulus) since the gait engine last reset. `IK_CLIP` in `fault_flags` is the "is it happening right now" signal derived from the same tick; these are the "how much/how bad" detail behind it. See §10. |
| `joint_clip_count`, `joint_clip_worst_deg` | yes | Same idea, for the other IK clip mechanism: a solved joint angle outside its configured bound. Independent from the pair above — a target can be reachable in D but still solve to an out-of-bound joint angle, or vice versa. See §10. |
| `ok`, `error` | yes | Result of the most recently processed command, *any* type, not just calibration. Generalizes the "ack round-trip" from §6 into the one reply mechanism telemetry already provides — a rejected `calibrate` (not armed, bad index) reports why here instead of failing silently. |
| `calibration_armed` | yes | So a GUI can show current arm state continuously rather than inferring it from the last ack. |
| `bench_armed` | yes | Same idea, for the separate bench-mode gate (§8) — a GUI must be able to show these two arm states independently, since they're independent gates. |
| `link_timeout_s` | yes | The robot's own compiled-in failsafe timeout. This is the runtime cross-check from §5, not a debugging field as such — the PC compares it against its own `LINK_TIMEOUT_S` on every telemetry receipt and surfaces a loud warning on mismatch (`RobotLink.constants_warning`). |
| `robot_assembled` | yes | The robot's own compiled-in `ROBOT_ASSEMBLED` build flag (§9) — reported for a glanceable GUI display, *not* cross-checked, since there's no PC-side expected value to compare it against and no way to detect a stale flag from anything this system observes. |
| `profiles` | yes, nullable | Full servo profile table (offset, sign, bench-recorded limits, health note — see §7), populated only in reply to `read_offsets`; `null` otherwise. See §6 — this is the "cheap to undo" mechanism, not the arm/disarm gate. |

## 4. Sequence numbers

Single 32-bit unsigned counter, incremented on every outbound packet from
the PC (commands and pings share one sequence space — the receiver's
newest-packet tracking and the telemetry echo both need exactly one
counter, not one per packet type). 32 bits wraps after roughly 13,600 years
at 10 Hz, which is not why it's correct — it's correct because the
comparison is implemented as RFC 1982-style modular arithmetic
(`(candidate - reference) mod 2**32 < 2**31`), not a bare `>`, so wraparound
is handled by construction rather than by "it'll never happen in practice."

Receiver behavior: track the highest successfully-applied sequence number.
An incoming packet is applied only if it is strictly newer by the
wraparound-safe comparison; equal or older is a duplicate or a
reordered/delayed packet that lost the race, and is silently dropped —
not applied, not an error. This is the same "latest wins, stale is
discarded" rule `MJPEGStream` already applies to frames, applied here to
commands. A drop counter is kept for observability but a drop is expected
UDP behavior, not a fault.

## 5. Shared constants — decided: YAML source of truth, manual codegen

`transport/constants.yaml` is the single source of truth for
`protocol_version`, `sequence_bits`, `link_timeout_s`,
`heartbeat_interval_s`, and `calibration_arm_timeout_s`. Regenerating
`transport/generated_constants.py` (and, later, a firmware C header) is an
explicit, manual command (`make gen-protocol` /
`python scripts/gen_protocol_constants.py`) — deliberately **not** wired
into any build step. On a one-month deadline with no firmware yet, a
broken pre-build codegen hook is a worse failure mode than the constant
drift it would prevent: it would stop the firmware from compiling at all,
and debugging build tooling under deadline pressure is worse than the
problem being solved.

Because the generation step can be forgotten, it isn't what actually
protects the failsafe property. That's a **runtime cross-check** instead:
every `Telemetry` packet carries `link_timeout_s`, the value the robot
actually has compiled in. `RobotLink.constants_warning` compares it
against the PC's own `LINK_TIMEOUT_S` on every receipt and returns a
non-`None`, human-readable message on mismatch — meant to be surfaced
loudly in a GUI and the log, not silently swallowed. Drift gets caught the
first time the two sides talk to each other, regardless of whether anyone
ran the codegen step, which is a stronger guarantee than the test-based
alternative originally proposed (a test only catches drift if someone
remembers to run it; this catches it automatically, on connect, every
time).

`transport/generated_constants.py` is committed to version control like
any other source file — nothing regenerates it automatically, so a fresh
checkout with a stale-but-uncommitted generated file would otherwise be
silently broken.

## 6. `calibrate` and bulk offsets — decided: allow over UDP, arm/disarm gate, mitigate by making writes cheap to undo

Allowed over the same UDP link, with two guards beyond ordinary validation:

- **Not part of the heartbeat/resend stream.** Calibration writes
  (`calibrate`, `write_offsets`) are sent once, like any other `send()`
  call, and confirmed via the ack fields in the next telemetry reply
  (`Telemetry.ok`/`error`, correlated by `seq_echo`) rather than being
  continuously re-issued. `RobotLink.wait_for_ack()` /
  `send_and_wait()` give calibration flows a blocking round-trip to
  confirm one write landed before sending the next, without adding a
  special-cased blocking path to `send()` itself for every other command.
- **Explicit arm/disarm gate.** `calibration_mode(armed=True)` must be
  sent before `calibrate` or `write_offsets` will be applied; either is
  rejected (`ok=False, error="calibration not armed"`) otherwise. The
  gate auto-disarms after `CALIBRATION_ARM_TIMEOUT_S` of no
  calibration-related traffic — refreshed by any `calibrate`/
  `write_offsets` while armed, not a fixed wall-clock window regardless of
  activity, so an active multi-servo calibration session doesn't get
  kicked out mid-session while idling between servos still auto-disarms
  it if the operator walks away or the GUI crashes.

  **Clarification found while implementing firmware:** `calibration_mode`
  itself must arm only on the false→true transition, and must NOT refresh
  the window on every receipt. The transport resends whatever was last
  sent regardless of type (§2), so a resent `calibration_mode(armed=true)`
  is indistinguishable on the wire from a deliberate re-arm click. If
  receiving it (including resends) refreshed the window, arming and then
  doing nothing else would keep the gate open forever as long as the link
  stayed up — exactly the "operator walked away" case this timeout exists
  to catch, defeated by the same mechanism meant to catch it. The window
  is refreshed only by genuine gated writes (`calibrate`/`write_offsets`),
  matching this section's wording above, which already said that and not
  "refreshed by `calibration_mode`." Both the firmware implementation
  (`firmware/`) and `MockRobotLink` (`transport/mock_link.py`) implement
  this correctly, pinned by tests in both `tests/test_mock_link.py` and
  `firmware/test/test_arm_gate/` — this was a real, briefly-shipped
  divergence between the two (mock re-armed on every received
  `armed=True`, resends included), closed in a later session once found.

Serial-only was the alternative, and was rejected for a hardware reason
specific to this build: calibrating requires watching the servo move,
which means the servo rail must be live, and USB + servo rail live
simultaneously is off-limits per the hardware notes. Serial-only would
have forced exactly that combination.

The transport guards above are not the real mitigation, though — malformed
JSON doesn't parse, out-of-range values are rejected at construction, and
UDP already has a checksum, so those aren't realistic failure modes. The
realistic one is operator error (wrong servo index, wrong value), which
happens identically over serial and isn't fixed by which transport carries
it. So the protocol is designed to make a bad write cheap to undo rather
than to make it hard to send:

- `read_offsets` returns the full 18-servo table in one reply
  (`Telemetry.offsets`), not one servo at a time.
- `write_offsets` restores the full table (or any subset) in a single
  bulk command, so recovering from a bad calibration session is one write,
  not eighteen individual re-sends re-running the same risk it's
  recovering from.
- A GUI built on this (not this session's scope) exporting `read_offsets`
  to a YAML file after every calibration session, and re-importing via
  `write_offsets` to restore, turns a bad write from a lost evening into a
  five-second restore — the actual risk being mitigated is operator error,
  and the mitigation is cheap undo, not prevention.

## 7. Servo profile: merged storage, separate write paths

Bench testing (§8) discovers two more numbers per servo beyond calibration's
offset/sign: a measured safe minimum and maximum joint travel. The question
was whether these live in the same table as offset/sign or a separate one.

Merged into one `ServoProfile` record (`servo_index`, `offset_us`, `sign`,
`min_deg_from_neutral`, `max_deg_from_neutral`, `note`), for the same
reason constants got one YAML source of truth instead of two
independently-maintained copies (§5): both describe one physical unit,
looked up by the same key, and splitting them into two tables means two
export files that can silently drift apart — the exact failure mode this
project has been designing against throughout. Extending an existing
struct with two nullable numbers and a string costs far less protocol
surface than a second bulk read/write/export/import pipeline.

**Degrees from this joint's own neutral, not absolute pulse
microseconds.** The bring-up plan is one physical test leg first (three
servos: one coxa, one femur, one tibia) before the full 18-servo build —
its measured limits need to describe every servo of that *type*, not just
the three actually on the bench, since all six legs share the same
mechanical geometry. Absolute pulse microseconds don't transfer: each of
the other fifteen servos gets its own `offset_us`, correcting that unit's
own spline-mounting error so that "commanded at neutral" lands at the same
true physical pose everywhere — which means the same mechanical rotation
limit corresponds to a *different* absolute pulse on every unit. Degrees
relative to neutral sidestep this entirely: `offset_us`'s whole job is
making neutral mean the same physical pose regardless of mounting error,
so a bound expressed relative to neutral is unit-independent in exactly
the way an absolute pulse isn't. `record_limit` still carries `pulse_us`
on the wire — what the operator actually verifies safe on the bench, in
raw hardware terms — and the conversion happens where the value is merged
into the profile (firmware, using that servo's joint type from
`ServoMap.h` to pick the right neutral), not on the wire. Keeping
pulse↔degree conversion out of the wire format matches `ANALYSIS.md`
Section 7's "one leaf function" rule for pulse units generally, and bench
mode's own enforcement (a raw `bench_pulse` command, §8) converts the same
way before comparing, so a limit recorded on the bench and a pulse
commanded on the bench are always compared in the same frame.

`min_deg_from_neutral`/`max_deg_from_neutral` default to `None`, not `0`
or some plausible-looking placeholder — a servo that hasn't been
bench-tested must be distinguishable from one whose limit was recorded as
an actual value.

Storage is shared; the *write* paths are not, because offset and limits
have genuinely different risk profiles (a correction vs. a safety bound
the firmware is meant to enforce on every command regardless of source):

- `calibrate` / `write_offsets` only ever write `offset_us`/`sign`, gated
  by `calibration_mode`, exactly as before.
- `record_limit` only ever writes `min_deg_from_neutral` or
  `max_deg_from_neutral` (one bound per call, matching the range finder's
  actual workflow — nudge toward one end, mark it, then the other), gated
  by `bench_mode` (§8), a different arm switch entirely.
- `bench_health_note` writes `note`. Not gated by either arm switch — it's
  advisory record-keeping, not a physical actuation or a safety-relevant
  value, and you may want to note a unit that isn't even the one currently
  plugged into the bench rig.

A write to one path must never clobber what the other has recorded — a
calibration session touching offset/sign preserves whatever limits/note
already exist for that servo, and vice versa. `MockRobotLink` implements
this via a merge (`dataclasses.replace` onto the existing record), not an
overwrite; a real firmware implementation needs the same discipline.

Reading (`read_offsets`) and exporting always return the full profile
regardless of which write path last touched it — one file, one place to
look at everything known about a servo, per the same reasoning as above.

## 8. Bench mode: raw pulse control, its own arm gate

Nothing before this session could drive a servo without the full gait/IK
stack computing an angle first. Bench-testing loose servos before assembly
needs a direct path: a raw pulse to one PCA9685 channel, no kinematics —
and that path needs to be unreachable by accident during normal operation.

**A separate arm gate, not an extension of `calibration_mode`.** A stray or
malformed `calibrate` packet corrupts a trim number. A stray `bench_pulse`
packet drives a channel directly, with no kinematics and no clamping logic
standing between the packet and the servo — a materially worse failure
mode that earns its own explicit `bench_mode(armed)` switch, so arming
calibration writes and arming raw pulse control are never the same click.
Same auto-disarm-on-inactivity shape as `calibration_mode`
(`BENCH_ARM_TIMEOUT_S`, refreshed by bench-related traffic while armed),
but tracked entirely independently — arming one never arms or extends the
other. Same edge-only arming rule too (§6's clarification): `bench_mode`
arms only on the false→true transition and is never itself what refreshes
the window — only `bench_pulse`/`record_limit` do, for the identical
reason.

For real accident-proofing this needs to hold at the firmware level too:
arming bench mode suspends the gait control loop, not run alongside it.
Built once gait existed (a later session than the one that first wrote
this note): `bench_mode(armed=true)` is refused with `ok=false,
error="gait active, cannot arm bench mode"` while a walk/turn command's
velocity/rotation is non-idle, and — the other direction — the gait
control loop itself does not tick at all while bench mode is armed, so
the two can never command the same physical channel in the same tick
regardless of ordering. `MockRobotLink` mirrors the arm-refusal for
parity, since it has no gait loop of its own to also suspend.

**Addressed by `(board, channel)`, not `servo_index`.** A loose servo
being bench-tested hasn't been assigned a leg position yet — there is no
`servo_index` for it until the operator decides which future position this
unit becomes. `bench_pulse` is therefore stateless and channel-addressed:
nothing persists beyond "drive this physical PCA9685 pin now." The moment
something worth keeping is found (a limit, a health note), that's a
separate, `servo_index`-addressed action (`record_limit`,
`bench_health_note`) — the operator explicitly says which future position
the recording is for, independent of which channel the bench rig happens
to be wired to this session. Two different addressing schemes because
they answer two different questions ("what am I driving" vs. "what am I
recording"), not an oversight.

Bounds on `bench_pulse.pulse_us` (`BENCH_PULSE_MIN_US`/`MAX_US`, 500-2500)
are the generic hobby-servo envelope, not a per-unit safety limit — a
sanity check at the protocol level. The GUI's manual pulse slider always
spans this same full generic range, even after bench testing has recorded
a narrower `min_deg_from_neutral`/`max_deg_from_neutral` for that servo —
it does not currently narrow itself to the recorded limit, only the wire
enforcement (`GatedServoDriver`, this section) does. Until it does,
there's nothing narrowing what the slider *shows* except the generic
bound and the operator's own attention, one small nudge at a time; the
GUI does display the recorded limit as text once known (see
`docs/GUI_GUIDE.md`'s Bench Test tab section).

`NEUTRAL_PULSE_US` (1500, the standard hobby-servo center) is used purely
as a client-side convention for "safe to hold indefinitely" — it is not a
special wire value. Parking at nominal center (not a per-unit corrected
value) before pressing a horn onto the spline is deliberate: the small
per-unit deviation gets corrected later via `offset_us`, during
calibration, not baked into where the horn physically sits.

## 9. `robot_assembled` telemetry: reported, not cross-checked

Firmware's `ROBOT_ASSEMBLED` build flag (`firmware/include/Config.h`)
selects what "safe" means on a fault — release every servo (`Bench`) or
hold the last commanded position (`Assembled`), see `SafeState.h`. It's a
compile-time choice with a real hardware consequence if it's wrong in one
specific direction: still `Bench`-flagged on an actually-assembled,
standing robot means a fault drops it. Forgetting to flip it after
assembly is a real, plausible failure mode — a `#define` with no runtime
visibility is exactly the kind of thing that's easy to forget weeks after
it was last touched.

`Telemetry.robot_assembled` reports the compiled-in value on every
packet, same pattern as `link_timeout_s` (§5), and the GUI displays it as
a persistent, glanceable badge (`SAFETY: BENCH` / `SAFETY: ASSEMBLED`),
not just a log line — the point is that it's visible without having to go
looking for it.

**Deliberately not cross-checked, unlike `link_timeout_s`.** §5's
runtime cross-check works because there are two independent sources of
truth to compare — the PC's own `LINK_TIMEOUT_S` and the value firmware
reports — and a mismatch between them is meaningful. `robot_assembled`
has no PC-side equivalent to compare against: nothing on the PC has, or
could have, an independent opinion about whether the robot is physically
assembled. So the only question worth asking is whether a mismatch
between the flag and *physical reality* (not another wire value) is
detectable at all, from anything this system observes.

It was considered and rejected, not just left undone. The one candidate
signal available — "gait is being actively commanded while the flag says
`Bench`" — sounds plausible but doesn't work: driving gait while still
`Bench`-flagged is *also* exactly the correct, expected workflow for
testing gait right up through the first cautious walk on a newly
assembled robot, deliberately left on `Bench` one test longer as extra
margin before trusting `Assembled`'s hold-on-fault behavior. Legitimate
pre-flip testing and a forgotten flip produce the identical observable —
active gait, flag still `Bench` — so a warning built on that signal would
fire constantly during the exact legitimate use it would supposedly be
protecting against, training the operator to ignore it. No other signal
exists either: there's no load cell, no IMU, no per-channel current
sensing (`rail_mv` is reserved and unpopulated for exactly this reason),
nothing anywhere in this system that observes whether a leg is bearing
weight right now. The badge is informational — it tells a human what
firmware compiled with, so *their own* knowledge of whether the robot is
actually assembled is what catches a stale flag, not this system.
`MockRobotLink`/`SimRobotLink` report a fixed, constructor-set value for
the same reason (`MockRobotLink(robot_assembled=...)`; `SimRobotLink`
always reports `False`, since nothing is assembled in simulation either).

## 10. IK clip observability: counted always, warned only while gait is the only mode

`inverse_kinematics()` has always had two silent clip mechanisms
(`ANALYSIS.md` §5.7 flagged both, unfixed until now): D (leg extension)
gets clamped into the femur/tibia-reachable annulus before `acos`, and
each solved joint angle gets clamped to its configured bound. Both used
to just discard how far past the bound the input was — a target 50mm
past reach and a target 0.001mm past reach produced the identical,
indistinguishable clamped output.

`ik_clip_count`/`ik_clip_worst_mm` and `joint_clip_count`/
`joint_clip_worst_deg` (§3's table) make both observable: cumulative
per-leg-per-tick event counts plus the worst overshoot ever seen, since
the gait engine last reset (boot, or `SimRobotLink` construction — never
since the last packet, unlike most of this struct). `FAULT_IK_CLIP` in
`fault_flags` is the derived, level-triggered summary of the same
tick's result — set when the tick that just ran clipped either
mechanism on any leg, cleared the next tick if it doesn't recur. The
cumulative counters are the diagnostic detail; the fault bit is what a
GUI actually alarms on.

**Why a fault bit here, and not just a quiet counter.** The gait
envelope this project's own `STEP_HEIGHT_MM`/`BODY_HEIGHT_Z_*`
constants were tuned against has zero clip events across the full
theoretical command space (every phase, direction, speed, rotation,
body height — see `robot/gait.py`'s derivation comments). Given that,
a clip firing during actual gait today is not routine operation — it
means something diverged from the verified-safe envelope, which is
exactly the kind of thing that can mean a servo driven into a
mechanical stop with no other visible symptom. That's worth a real,
glanceable warning, not a number buried in a details panel nobody
opens until after something breaks.

**This will need revisiting once Test Leg exploration mode exists.**
Deliberately probing right up to and past a joint's limit is the entire
point of that mode — `joint_clip` firing there is expected, routine,
and would just be alarm-fatigue noise if treated the same as a clip
during real gait (the identical failure mode §9 above rejected a
cross-check signal over: a warning that fires during legitimate use
trains the operator to ignore it). Whichever code adds that mode needs
to stop setting `FAULT_IK_CLIP` while it's active — scoped there
deliberately, not solved preemptively here, since that mode doesn't
exist yet.

## Commands

Every command packet shares an envelope: `{"v": 1, "seq": <uint32>, "type":
<string>, ...type-specific fields}`. Range bounds below are enforced at
construction (Python `__post_init__`), not at encode time — an
out-of-range command cannot be built, let alone sent.

| type | fields | bounds |
|---|---|---|
| `walk` | `vx`, `vy` (float), `speed` (float) | `vx, vy ∈ [-1, 1]`, `speed ∈ [0, 100]` |
| `turn` | `rate` (float), `speed` (float) | `rate ∈ [-1, 1]`, `speed ∈ [0, 100]` |
| `stop` | — | — |
| `body_height` | `height` (float) | `height ∈ [0, 100]` |
| `pan_tilt` | `pan`, `tilt` (float, degrees) | `pan, tilt ∈ [-90, 90]` — placeholder range for an unspecified mini pan-tilt servo; documented assumption, not a measured spec, revisit once hardware is known |
| `face` | `mood` (enum) | `{neutral, happy, angry, surprised, sleepy}` — provisional set, no OLED face implemented yet |
| `calibrate` | `servo_index` (int), `offset_us` (int), `sign` (int) | `servo_index ∈ [0, 17]`, `offset_us ∈ [-500, 500]`, `sign ∈ {-1, 1}`; requires calibration mode armed |
| `calibration_mode` | `armed` (bool) | arms/disarms `calibrate` and `write_offsets`; auto-disarms after `CALIBRATION_ARM_TIMEOUT_S` of no calibration-related traffic (§6) |
| `read_offsets` | — | returns the full servo profile table via `Telemetry.profiles`; always allowed, armed or not — reading isn't a write |
| `write_offsets` | `offsets` (list of `{servo_index, offset_us, sign}`, 1-18 entries, no duplicate indices) | bulk restore of offset/sign only, never limits/note (§7); requires calibration mode armed |
| `bench_mode` | `armed` (bool) | arms/disarms `bench_pulse`/`record_limit`; independent of `calibration_mode` (§8) |
| `bench_pulse` | `board` (int), `channel` (int), `pulse_us` (int), `servo_index` (int, optional) | `board ∈ {0x40, 0x41}`, `channel ∈ [0, 15]`, `pulse_us ∈ [500, 2500]`, `servo_index ∈ [0, 17]` if given; requires bench mode armed |
| `record_limit` | `servo_index` (int), `bound` (`"min"`/`"max"`), `pulse_us` (int) | `servo_index ∈ [0, 17]`, `pulse_us ∈ [500, 2500]`; requires bench mode armed; converted to degrees from that servo's own neutral before storage (§7), rejected if that would make `min_deg_from_neutral >= max_deg_from_neutral` for that servo |
| `bench_health_note` | `servo_index` (int), `note` (str) | `servo_index ∈ [0, 17]`, `note` ≤ 500 chars; not gated |
| `ping` | — | — |

`walk` carries both `vx` and `vy` from the start, even though the reference
gait has no strafing today (`ANALYSIS.md` §4) — the protocol should not be
what blocks adding it. Firmware's gait engine now implements both axes,
plus continuous speed scaling and runtime body height — see the
"Deliverables status" section below for what's real as of which session.

**On `bench_pulse`'s optional `servo_index`**: found missing while
implementing firmware — `GatedServoDriver` needs a servo identity to look
up a bench-recorded limit against, but `board`/`channel` alone doesn't
give it one (a loose servo hasn't necessarily been assigned a leg
position yet, per this section's note above). When given, the receiver
converts the commanded `pulse_us` to degrees from that servo's own
neutral (the same conversion `record_limit` uses, §7) and enforces that
servo's recorded `min_deg_from_neutral`/`max_deg_from_neutral` (if any)
against it — a bench-recorded bound and a bench-commanded pulse are
always compared in the same frame; when `servo_index` is omitted, only
the generic envelope bound applies. Both firmware and `MockRobotLink`
implement this identically (`transport/mock_link.py`).

**On `calibrate`'s two fields**: `offset_us` is a per-servo pulse trim
(microsecond correction around center), and `sign` is a per-servo rotation
direction. These are deliberately *servo*-scoped, not *leg*-scoped, and are
a different concern from the leg-mirroring question resolved in
`ANALYSIS.md` §3. That analysis concluded the leg geometry needs no
per-leg direction inversion — all six legs are the same part, uniformly
rotated into place, and the existing `90 + angle` formula is correct as
written for all of them. `sign` here defends against a narrower, still-real
failure mode: one individual servo horn seated backward on its spline
during assembly (a per-unit mechanical accident, independent of which leg
it's on). Defaults reflect that it's confirmed-correct-by-default,
assembly-QA-override-when-needed, matching the framing in `ANALYSIS.md`
§3: `offset_us` defaults to `0` (no trim), `sign` defaults to `1` (no
flip) for all 18 servos, changed only from a bench measurement on a
specific unit.

## Failsafe contract

- The robot tracks time since the last packet (of any type) that passed
  sequence-freshness and validation.
- If that exceeds `LINK_TIMEOUT_S` with no fresh packet, the robot forces
  `stop` and sets the `LINK_TIMEOUT` fault bit, independent of whatever the
  last-applied command was. What "stop" concretely means depends on
  firmware's `ROBOT_ASSEMBLED` build flag (a compile-time choice, not a
  wire concept — see `docs/HOW_TO_USE.md`): `Bench` (the default) releases
  every servo; `Assembled` holds the last commanded position by simply not
  computing or sending any new one, since a loaded joint being released
  gives way under the robot's own weight. Both mean "no further commanded
  motion" from the protocol's point of view; only the physical result at
  the servo differs.
- Recovery is automatic: the next valid, newer packet clears the fault bit
  and resumes normal operation. There is no separate "re-arm" step.
- The PC-side link implementation applies the same freshness/newness rule
  to inbound telemetry, so `RobotLink.is_connected` reflects "have we heard
  from the robot recently," not just "is the socket open."

## Deliverables status

Implemented: `transport/protocol.py` (encode/decode, validation, sequence
comparison, `ServoProfile`, bench commands), `transport/link.py`
(`RobotLink` base with shared telemetry tracking, connection liveness, the
constants cross-check, and `wait_for_ack`/`send_and_wait`),
`transport/udp_link.py` (`UDPRobotLink`), `transport/mock_link.py`
(`MockRobotLink`, simulating both the calibration and bench arm/disarm
gates independently, and the profile-merge-not-overwrite discipline §7
depends on), `transport/constants.yaml` / `scripts/gen_protocol_constants.py`
/ `transport/generated_constants.py` for §5, an operator GUI (`app.py`,
`ui/`) with Operate/Calibrate/Bench Test tabs, and `control/tracker.py` /
`control/bench.py` for the AUTO_TRACK and bench dwell/sweep pure logic.
Tests cover encode/decode round-trips for every command and telemetry
(including the four bench commands), range rejection, malformed/wrong-
version/unknown-type decode rejection, sequence wraparound and out-of-order
handling, UDP heartbeat timing and telemetry receipt, `MockRobotLink`
behavior including both arm gates and the limit/note merge-preservation
guarantee, and the bench dwell-guard/sweep pure logic.

Firmware (`firmware/`, ESP32-S3/PlatformIO) implements the full protocol:
`bench_mode`/`bench_pulse`/`record_limit`/`bench_health_note`,
`calibration_mode`/`calibrate`/`write_offsets`/`read_offsets`,
`ping`, and — as of a later session than the one that first wrote most of
this document — real gait/IK for `walk`/`turn`/`body_height`, converted
through `Kinematics` → `AngleToPulse` → `GatedServoDriver` (the same
limit enforcement `bench_pulse` goes through, no separate/bypassable
path). `pan_tilt`/`face` are still decoded, validated, and recorded for
`last_applied` with no hardware effect — no pan-tilt or face hardware
exists yet. The link watchdog, both arm gates, per-channel dwell
protection, limit enforcement, kinematics, and the gait engine are all
pure logic in `firmware/lib/core`, unit tested on the host (`pio test -e
native`) the same way `transport/protocol.py` is tested with pytest, and
the kinematics/gait math specifically is cross-checked against
`robot/kinematics.py`/`robot/gait.py` (the Python reference
implementation) via generated golden fixtures
(`scripts/gen_kinematics_golden.py`, `scripts/gen_gait_golden.py`) rather
than shared via a compiled extension — see `reference/ANALYSIS.md`
Section 7 for why. A `simulator/` package (`SimRobotLink`, a `RobotLink`
implementation) runs the same gait engine over time on a background
thread, for GUI-driven testing of gait/failsafe behavior with no hardware
and no camera, selectable via `operator_config.yaml`'s `link.mode: sim`
or `python app.py --link-mode sim`. Verified by real cross-compilation
for the actual target and by unit tests; not verified against physical
hardware, which no session so far has had access to.
