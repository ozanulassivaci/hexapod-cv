# Robot command protocol

Design doc for the PC↔robot command layer. Covers the wire format, every
command, the failsafe contract, and the reasoning behind the six open design
questions raised before implementation. Two of those questions (§5, §6) are
marked pending — see the note at each.

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

## 5. Shared constants — PENDING

Two values must be identical on both sides of the link or the failsafe
misfires: `LINK_TIMEOUT_S` and `HEARTBEAT_INTERVAL_S`. Two options were
raised:

- **Single YAML source of truth, codegen to both a Python module and a C
  header.** Makes disagreement structurally impossible — both sides are
  generated artifacts of the same file, so there's nothing to drift.
  Cost: build tooling (a codegen script now, a pre-build step in the
  firmware build later) in a project that has no firmware yet.
- **Duplicate the constants by hand, add a test that asserts both sides
  agree.** No tooling cost. Cost: it's a safety net, not a prevention — it
  only catches drift after an edit, and only if the test actually runs.

Leaning YAML + codegen, since this was explicitly flagged as a safety
property rather than a convenience, but the tooling investment is a real
trade-off against a project with no firmware to consume the generated
header yet. **Waiting on this answer before implementing `transport/`**,
since it affects how `protocol.py` defines/imports these two constants.

## 6. `calibrate` over UDP — PENDING

`calibrate` writes a per-servo trim offset (and, per the note below,
direction sign) toward NVS on the robot. Unlike motion commands, it is not
self-correcting by the next resend — a stray or corrupted packet doesn't
just get superseded a moment later, it can persist a bad value.

Leaning toward allowing it over the same UDP link but treating it as
categorically different from motion commands in the send path: not part of
the resend/heartbeat stream, gated behind an explicit request/ack
round-trip rather than fire-and-forget, so a single malformed or
out-of-order packet can't silently commit a write. Serial-only removes the
risk entirely but trades away being able to calibrate over the same link
used for everything else, for a workflow (bringing legs up one at a time)
that's likely to be run often. **Waiting on this answer** — it determines
whether `udp_link.py` needs to reject `CalibrateCommand` outright or
implement the ack round-trip.

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
| `calibrate` | `servo_index` (int), `offset_us` (int), `sign` (int) | `servo_index ∈ [0, 17]`, `offset_us ∈ [-500, 500]`, `sign ∈ {-1, 1}` |
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

`transport/protocol.py`, `transport/link.py`, `transport/udp_link.py`,
`transport/mock_link.py`, and tests are not yet implemented — waiting on
§5 and §6 above before writing code that depends on their outcome.
