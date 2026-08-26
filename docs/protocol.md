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
| `fault_flags` | yes (bitfield defined, mostly unset) | Cheap — one integer — and it's the difference between "mystery" and "immediate answer." Only `LINK_TIMEOUT` is meaningful without firmware; other bits (`ESTOP`, `SERVO_FAULT`, `BROWNOUT`) are reserved now so the wire format doesn't change when real fault sources exist. |
| `gait_phase` | yes, nullable | One float. Narrows a "not moving" report to "control loop alive but leg output isn't happening" when link, echo, and faults all look fine. `null` when not walking. |
| `rail_mv` | reserved, nullable, unpopulated | Needs an ADC + voltage divider that isn't in the current hardware list. Reserving the field now avoids a wire-format change later; nothing sets it yet, and this doc doesn't pretend otherwise. |
| `ok`, `error` | yes | Result of the most recently processed command, *any* type, not just calibration. Generalizes the "ack round-trip" from §6 into the one reply mechanism telemetry already provides — a rejected `calibrate` (not armed, bad index) reports why here instead of failing silently. |
| `calibration_armed` | yes | So a GUI can show current arm state continuously rather than inferring it from the last ack. |
| `link_timeout_s` | yes | The robot's own compiled-in failsafe timeout. This is the runtime cross-check from §5, not a debugging field as such — the PC compares it against its own `LINK_TIMEOUT_S` on every telemetry receipt and surfaces a loud warning on mismatch (`RobotLink.constants_warning`). |
| `offsets` | yes, nullable | Full 18-servo calibration table, populated only in reply to `read_offsets`; `null` otherwise. See §6 — this is the "cheap to undo" mechanism, not the arm/disarm gate. |

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
| `read_offsets` | — | returns the full 18-servo table via `Telemetry.offsets`; always allowed, armed or not — reading isn't a write |
| `write_offsets` | `offsets` (list of `{servo_index, offset_us, sign}`, 1-18 entries, no duplicate indices) | bulk restore of some or all of the table; same bounds as `calibrate` per entry; requires calibration mode armed |
| `ping` | — | — |

`walk` carries both `vx` and `vy` from the start, even though the reference
gait has no strafing today (`ANALYSIS.md` §4) — the protocol should not be
what blocks adding it.

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
  last-applied command was.
- Recovery is automatic: the next valid, newer packet clears the fault bit
  and resumes normal operation. There is no separate "re-arm" step.
- The PC-side link implementation applies the same freshness/newness rule
  to inbound telemetry, so `RobotLink.is_connected` reflects "have we heard
  from the robot recently," not just "is the socket open."

## Deliverables status

Implemented: `transport/protocol.py` (encode/decode, validation, sequence
comparison), `transport/link.py` (`RobotLink` base with shared telemetry
tracking, connection liveness, the constants cross-check, and
`wait_for_ack`/`send_and_wait`), `transport/udp_link.py`
(`UDPRobotLink`), `transport/mock_link.py` (`MockRobotLink`, including a
calibration arm/disarm simulation for GUI development without firmware),
and `transport/constants.yaml` / `scripts/gen_protocol_constants.py` /
`transport/generated_constants.py` for §5. Tests cover encode/decode
round-trips for every command and telemetry, range rejection (including
NaN/inf and bool-as-number), malformed/wrong-version/unknown-type decode
rejection, sequence wraparound and out-of-order handling, UDP heartbeat
timing and telemetry receipt (including out-of-order telemetry not
clobbering newer data while still counting as liveness), and
`MockRobotLink` behavior including the calibration gate and bulk
offset round-trip.

No GUI and no firmware were written this session, per scope.
