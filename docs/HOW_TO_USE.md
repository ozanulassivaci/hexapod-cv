# How to use hexapod-cv — operational guide

This is about physical sequencing: what to plug in, what to power on, and
in what order. `docs/GUI_GUIDE.md` covers what the app's controls do —
read that too, this one assumes you already know the buttons exist.

Written for late-night use. Short sentences, numbered steps. If you're
tired, read the numbered list, not the prose around it.

## The one rule that matters most

**USB is for flashing the firmware. Nothing else.**

Driving the robot, reading telemetry, and every calibration or bench-test
write all happen over WiFi. During a normal driving session and during a
full calibration session, **the ESP32 is not connected to your laptop by
any cable.** It's running on its own, powered by the servo rail, talking
to your laptop only over the network.

This trips people up because "calibration" sounds like it should need a
wired connection. It doesn't. If you find yourself reaching for a USB
cable to calibrate or drive the robot, stop — that's not how this works.

Even *flashing new firmware* mostly doesn't need USB after the first time
— see Section 2's OTA path. USB is genuinely for the very first flash
(before the ESP32 has WiFi credentials to be reachable at all) and as a
fallback, not a routine requirement.

Why USB and the servo rail are never live at the same time: connecting USB
ties your laptop's ground to the ESP32's ground. The servo rail is a 6V
supply feeding 18 servos, sharing that same ground reference through the
PCA9685 boards. With both connected, current from the servo rail (and it's
not small — a single MG996R can pull ~2.5A when stalled) has a path back
through that shared ground toward your laptop's USB port. That's a real
risk to your laptop, not a theoretical one. Servo rail off, always, before
USB goes in.

## 1. First-time bring-up

WiFi credentials are compiled into the firmware, not entered at runtime —
you set them before this first flash, and this is the one time flashing
genuinely requires USB (there's no WiFi connection yet for OTA to use).

1. In `firmware/include/`, copy `secrets.h.example` to `secrets.h` and
   fill in your real WiFi SSID/password, a fallback AP SSID/password, and
   an OTA password. `secrets.h` is gitignored — it never gets committed.
2. Servo rail **OFF**.
3. Connect the ESP32 to your laptop with USB.
4. From `firmware/`, flash it: `pio run -e esp32-s3-devkitc-1 -t upload`.
5. Unplug USB.
6. Servo rail **ON**.
7. Launch the GUI: `python app.py`.
8. Check the LINK status light in the app. Green means the ESP32 found
   your WiFi network and the app found the ESP32. If it's red, see
   "Link won't connect" below before doing anything else.

If the ESP32 couldn't join your network at all, it falls back to hosting
its own WiFi network (the fallback AP SSID/password from `secrets.h`) so
you can still reach it — connect your laptop to that network directly and
`operator_config.yaml`'s `robot_host` will need to point at the ESP32's AP
address instead (printed over serial if you're on USB to check).

## 2. The routine flash cycle (updating firmware later)

Two ways to reflash, depending on what changed.

**2a. OTA (normal case, no USB, servo rail stays on).** WiFi credentials
already loaded from the first-time flash means the ESP32 is reachable over
the network — reflashing over WiFi removes the servo-rail-off cycle for
most updates, which is the entire reason it exists.

1. Make sure nothing is armed — Calibrate and Bench Test tabs both showing
   DISARMED — no bench servo is currently being held away from neutral,
   and the robot isn't actively walking (hit Space/STOP first if it is).
   **OTA is refused in any of those states**, and refused silently: the
   ESP32 simply won't respond to the update request at all, there's no
   error message telling you why. If an OTA upload seems to hang or fails
   to connect, this is the first thing to check.
2. Servo rail can stay **ON** — this is a WiFi update, not USB, so the
   ground-path risk "the one rule that matters most" describes doesn't
   apply here.
3. From `firmware/`: `pio run -e esp32-s3-devkitc-1 -t upload --upload-port <robot-ip>`
   (the OTA password from `secrets.h` is required and configured in
   `platformio.ini`/your upload settings).
4. Wait for it to finish — the ESP32 reboots into the new firmware on its
   own once the transfer completes.
5. Give it a few seconds, then confirm LINK goes green in the GUI again.

**2b. USB (fallback — WiFi credentials changed, OTA isn't reachable, or
OTA fails).** The ESP32 stays mounted on the robot the whole time — you're
not removing it, just switching which connection is active.

1. Servo rail **OFF**.
2. Plug in USB.
3. Flash: `pio run -e esp32-s3-devkitc-1 -t upload`.
4. Unplug USB.
5. Servo rail **ON**.
6. Launch the GUI.

Same order, every time. Don't reverse steps 1 and 2.

## 3. Bench-testing a loose servo before assembly

The servo needs real power to move (a single MG996R draws far more than a
laptop's USB port is meant to supply), so this still goes through the
servo rail — but the ESP32 itself talks to you over WiFi, same as always.

1. Servo rail **OFF** while you wire the loose servo to a PCA9685 channel.
   Don't wire a servo onto a live rail — a slipped wire against a powered
   pin is how boards die.
2. Servo rail **ON**.
3. Launch the GUI, confirm LINK is green.
4. Go to the **Bench Test** tab and follow the walkthrough in
   `docs/GUI_GUIDE.md`.
5. Servo rail **OFF** again before you unplug that servo or wire up the
   next one.

## Mounting horns for the single test leg

Before any exploration work starts, one leg gets its three horns mounted
at specific angles computed from the gait envelope (`robot/gait.py`,
`firmware/lib/core/Gait.h`, commit `b009cd4`). "Mount at servo angle X"
is ambiguous on its own — this section says exactly what to command,
what pose to hold the leg segment in while pressing the horn on, and how
to eyeball-check you landed on the right spline tooth.

**The three numbers, at a glance:**

| Joint | Servo angle | Pulse | By-eye check |
|---|---|---|---|
| Coxa  | 90.0°  | 1500us | Swings the same amount to both sides of the reference line |
| Tibia | 77.0°  | 1355us | ~103° fold from the femur's own straight line (knee-bend direction) |
| Femur | 124.2° | 1880us | ~34° up from horizontal, checked with a level |

Full procedure, pose to hold, and why below — read it before mounting
anything, this table is a reference to come back to, not a substitute
for the steps.

The numbers below assume: the leg is wired to a Bench Test tab channel
one servo at a time; `TIBIA_NEUTRAL_PULSE_US` is still the unverified
`Config.h` placeholder (500us, same as `BENCH_PULSE_MIN_US`) — if that
value changes before you do this, redo the tibia arithmetic first; and no
`offset_us` calibration exists yet, since bench mode drives raw pulses.
Mount now, calibrate precisely once the leg is on and Calibrate is
usable.

**Order: coxa, then tibia, then femur.** Coxa doesn't depend on anything
else. Tibia's check is relative to the femur segment's own straight line,
which works with no horn on the femur servo yet — you just need
something rigid to sight along. Femur goes last because by then coxa and
tibia are already fixed in place, so a simple gravity-referenced level
check becomes valid.

For each joint: wire it to a Bench Test channel, arm Bench Test, drag the
manual pulse slider to the value below (or nudge with +/- until the
readout matches), physically hold the leg segment in the stated pose,
then press the horn onto the spline. Keep the servo powered and holding
that exact pulse the whole time you're pressing — parking at neutral or
releasing would let the segment drift while you're trying to align it.

### 1. Coxa — 90.0 servo degrees, 1500us

- Command 1500us on the coxa channel.
- There's no body and no leg yet, so there's no compass direction to
  check against. Mount the horn (with the femur segment attached to it)
  pointing away from the servo body in any convenient direction — that
  becomes your reference line for the check below.
- **Check:** with the horn on, nudge the slider toward each side (about
  1420us and 1580us — the gait envelope's +-19.6 degree limits) and
  watch the leg swing. It should swing visibly the same amount to both
  sides of your reference line. If one side is clearly shorter than the
  other, the horn is a spline tooth (14.4 degrees) off — pull it and
  rotate one tooth toward the short side.
- Return the slider to 1500us before moving on.

### 2. Tibia — 77.0 servo degrees, 1355us

- Command 1355us on the tibia channel (the servo mounted at the femur
  segment's far end).
- Hold the femur segment straight — resting it flat against the table
  edge or any straightedge is enough. It doesn't need to be at its own
  final mounting angle yet.
- **Check:** the tibia segment should fold away from that straight line
  by about 103 degrees, measured on the inside of the bend — a bit more
  open than a right angle — folding the same direction a knee bends, not
  backward. By eye, it should look closer to "mostly folded, slightly
  open" than to "half-folded" (90 degrees) or "barely bent" (135+
  degrees). If it looks off by roughly one spline tooth's worth (14.4
  degrees more or less folded), pull the horn and shift one tooth.

### 3. Femur — 124.2 servo degrees, 1880us

- Coxa and tibia are already fixed at this point — no other joint's pose
  matters for this check.
- Command 1880us on the femur channel.
- **Check:** hold a level (or a phone inclinometer app) against the flat
  top or bottom face of the femur segment. It should read about 34
  degrees above horizontal, tilted up from the coxa — clearly angled
  upward, not flat and not steeply raised. If the level reads close to
  horizontal (0) or steep (45+), the horn is roughly a spline tooth off;
  pull it and shift one tooth toward 34 degrees.

### Headroom against a mounting error

25T splines only take a tooth every 14.4 degrees, so no matter how
carefully you align by eye, worst case you're 7.2 degrees off — the
checks above exist to make sure you picked the *nearest* tooth, not one
14.4 degrees away, not to make the mount exact. Whether 7.2 degrees is
safe to walk away from depends on how much room the gait envelope has
against the servo's own end of travel (not yet against real mechanical
limits — nothing is measured until the Test Leg exploration work
happens):

- **Coxa:** envelope needs 70.4-109.6 degrees of the servo's 0-180 range
  — 70.4 degrees of headroom on both ends. No concern.
- **Femur:** envelope needs 87.9-160.5 degrees — only 19.5 degrees of
  headroom before the servo's own nominal 180-degree end. Survives a
  7.2-degree mounting error with 12.3 degrees to spare, but this is the
  joint to watch once real bench data comes in for this specific unit —
  clone MG996Rs are already known not to reliably reach nominal 180 (see
  the Bench Test tab's conservative 600-2400us range), which eats into
  this margin from the servo side, independent of the horn mount.
- **Tibia:** envelope needs 39.6-114.4 degrees — 39.6 and 65.6 degrees of
  headroom on the low and high ends respectively. No concern.

Coxa's mounting angle is identical across all six legs (the raw midpoint
is exactly 0 degrees for every leg, corner or middle — confirmed by a
leg-invariance check across the full envelope sweep), so the earlier
0.35-degree corner/middle-leg discrepancy, whatever its source, doesn't
show up in this number and doesn't need resolving before mounting this
one test leg.

## Clamping the test leg for exploration work

Once the three horns are mounted, the leg needs to be held still enough
to actually explore its limits safely — but clamped in a way that leaves
the coxa free to do the one thing it's for: rotate.

**Clamp the coxa servo's own body/mounting bracket to the table edge —
never the horn, never the femur segment, never anything that's supposed
to move.** The coxa servo housing itself (or whatever fixed plate/stub it
sits in) is the only part of this assembly that's meant to stay
perfectly still. Everything downstream of the coxa horn — the femur,
tibia, and the coxa horn itself — needs to swing freely through its full
range without hitting the clamp, the table edge, or the table surface.

1. Servo rail **OFF** before you touch any wiring or clamping.
2. Position the leg so the coxa's rotation axis is vertical and the
   femur/tibia assembly hangs out past the table edge, with a clear
   swept volume below and to both sides — nothing the leg could hit while
   exploring femur/tibia range, since neither has a confirmed limit yet
   at this point.
3. Clamp the coxa servo's body/bracket down. Check by hand (servo rail
   still off, no power) that you can freely rotate the coxa horn through
   a wide arc and the femur/tibia assembly never touches the clamp, the
   table, or its own wiring bundle. Reposition now if it does — this is
   much easier to fix before the rail is live.
4. Route the three servo cables with enough slack for the full range of
   motion on every joint, and secure the slack so it can't catch on
   anything while a joint moves. A snagged cable mid-exploration can pull
   a connector loose or torque a horn off-tooth.
5. Servo rail **ON**. Proceed to the **Test Leg** tab
   (`docs/GUI_GUIDE.md`) for the actual exploration.

If you ever need to reposition the clamp after starting exploration:
servo rail **OFF** first, same as any other rewiring — don't loosen a
clamp with the rail live and a servo potentially mid-command.

## 4. A full calibration session

1. Robot fully assembled. Servo rail **ON**. No USB connected.
2. Launch the GUI, confirm LINK is green.
3. Go to the **Calibrate** tab and follow the walkthrough in
   `docs/GUI_GUIDE.md`.
4. Export to YAML before you consider the session done. Not optional —
   see GUI_GUIDE.md's "things that can cost you money" section for why.
5. Follow the shutdown order below.

## 5. A normal driving session

1. Servo rail **ON**. No USB.
2. Launch the GUI, confirm LINK is green before you start driving. If it's
   red, you can still click around the app, but nothing you send is
   reaching the robot.
3. Drive with WASD/arrows/Q/E, or switch to AUTO_TRACK. Space or the
   EMERGENCY STOP button works at any point.
4. Follow the shutdown order below when you're done.

## 6. Shutdown order

1. Stop the robot first — Space or EMERGENCY STOP — so it isn't mid-stride
   when power drops.
2. Close the GUI window normally (not force-quit). This releases the
   network link and, if you were on the Bench Test tab, parks any servo
   that was still being held away from neutral.
3. Servo rail **OFF**.
4. If your ESP32 is on a separate power switch from the servo rail, power
   it down last. If it shares power with the servo rail, this happens on
   its own in step 3.

## Flipping ROBOT_ASSEMBLED, once the robot is actually standing on its own legs

Firmware has two safe-state modes, chosen at compile time by
`ROBOT_ASSEMBLED` in `firmware/include/Config.h` — not something you set
at runtime from the GUI, and not sent over the wire. Default is `0`
(bench mode): on a link timeout, an OTA start, or a WiFi AP-fallback
transition, every servo releases. That's correct right now, and stays
correct for as long as there's no leg bearing weight — releasing costs
nothing when there's nothing to drop.

Once assembly is actually finished — all six legs mounted, the robot
standing on its own — releasing becomes the wrong answer: a loaded joint
gives way under the robot's own weight the instant it's released.
`ROBOT_ASSEMBLED 1` switches to holding the last commanded position
instead — the control loop simply stops sending new pulses; the PCA9685
keeps outputting whatever it was last told, with no further firmware
involvement needed to "hold."

1. Only do this once assembly is genuinely done and you've confirmed the
   robot stands under its own commanded pose. Don't flip it "to test" on
   a bench with no legs attached — see "Never do this" below for why.
2. In `firmware/include/Config.h`, change `#define ROBOT_ASSEMBLED 0` to
   `#define ROBOT_ASSEMBLED 1`.
3. Reflash — OTA is fine (see Section 2a), same as any other firmware
   change.
4. After it reboots, confirm the flip actually took effect. The easiest
   way: launch the GUI and look at the **SAFETY** badge — it now reports
   this over telemetry, so it reads "SAFETY: ASSEMBLED" the moment the
   GUI reconnects, no USB needed. The boot line and `status` serial
   command also print `safety_mode=`, useful if you're already on USB
   for some other reason.
5. Remember what the badge cannot tell you: it shows what firmware
   compiled with, not whether that's actually correct for the robot in
   front of you. Nothing in this system senses physical assembly state —
   see `docs/protocol.md` Section 9 for why that was considered and
   ruled out, not just skipped. Checking the badge against what you
   actually know about the robot is still on you.
6. From here on, a link timeout or OTA start mid-walk holds the pose
   instead of dropping — expected, and the entire point of the flip. It
   does not change bench-testing behavior: a single loose servo's dwell
   timeout still releases that one channel on its own, independent of
   this flag.

## Measuring PCA9685 oscillator frequency (once per board)

Each PCA9685 board has its own internal oscillator, nominally 25MHz —
but clone boards drift, sometimes by a few percent, and the two boards can
drift differently even from the same batch. That drift shifts every pulse
width on that specific board by the same ratio: "1500us" might actually be
coming out a few percent off. Do this once per board, before you trust
absolute pulse-width accuracy for anything — bench testing still works
fine on the un-measured nominal value in the meantime, this just makes it
precise.

Tools: a multimeter with a Hz (frequency) measurement mode — common even
on inexpensive digital multimeters — and, optionally, a servo tester as a
cross-check. No oscilloscope needed.

1. With the robot powered normally (servo rail on) and bench mode armed,
   command any channel on the board you're measuring to any pulse that
   isn't fully on or off (e.g. park it at neutral — see the Bench Test
   tab).
2. Set your multimeter to frequency (Hz) mode and probe that channel's
   output pin against ground.
3. Read the displayed frequency. It should read close to 50.00Hz; if the
   oscillator were running exactly at nominal, it would read exactly that.
4. Compute the board's real oscillator frequency:
   `measured_osc_freq_hz = 25,000,000 * (measured_frequency_hz / 50.0)`
5. In `firmware/include/Config.h`, set `PCA9685_OSC_FREQ_BOARD_A_HZ` (for
   the board at I2C address 0x40) or `PCA9685_OSC_FREQ_BOARD_B_HZ` (0x41)
   to your measured value.
6. Reflash (OTA is fine for this) and re-measure the same channel to
   confirm it now reads close to 50.00Hz.
7. Repeat for the other board — its oscillator can be off by a different
   amount.

Optional sanity check: drive a servo to the servo tester's own centered
preset (most have one, and it's a trustworthy independent reference near
dead-center) and note the horn's physical position. Then command the same
servo via the PCA9685 at 1500us with your corrected oscillator value and
confirm the horn lands in the same spot. This won't give you a precise
number on its own, but it's a good coarse cross-check that doesn't depend
on trusting the multimeter alone.

If your multimeter has a duty-cycle mode instead of frequency mode, the
same idea works: command a known intended duty cycle and compare it
against the meter's reading, same ratio math.

## Never do this

- **Never connect USB while the servo rail is live.** See "the one rule
  that matters most" above — this is the laptop-damage risk, not a
  formality.
- **Never wire or unwire a servo on a live rail.** A brief short from a
  slipped wire is how you lose a PCA9685 channel or worse.
- **Never leave a servo held away from neutral and walk away.** Two
  independent layers of dwell protection exist — the GUI's (moves it back
  to neutral) and the firmware's (goes limp — releases the channel
  entirely, since that's the faster and more immediately protective
  response, and works even if the GUI has crashed or the link has
  dropped). Either can fire first depending on timing. Both are a safety
  net, not permission to be careless — if you're stepping away, park it
  yourself first.
- **Never assume OTA will work while anything is armed.** It's refused
  silently — no error, the ESP32 just won't respond — while Calibrate or
  Bench Test mode is armed, while any servo is being held away from
  neutral, or while the robot is actively walking. Disarm, park, and stop
  first; see Section 2a.
- **Never flip `ROBOT_ASSEMBLED` to test it on a bare bench servo.**
  Once set, a link timeout holds the last commanded position instead of
  releasing — harmless on an assembled, standing robot, but on a bench
  rig it just means a bare servo keeps fighting to hold whatever pulse it
  last had instead of going limp, for no benefit. Flip it once, for real,
  when assembly is actually done — see "Flipping ROBOT_ASSEMBLED" above.
- **Never click through an import confirmation without reading it.**
  It names the file and the count of entries about to be written. A
  confident "Yes" on the wrong file silently overwrites good calibration.
- **Never treat the arm/disarm countdown as exact.** It's the app's own
  estimate, not a number the robot is confirming every second. See
  GUI_GUIDE.md.
- **Never force-quit the GUI as routine habit.** A normal window close
  cleans up the network link and parks any live bench servo; force-quit
  skips both. Only do it if the app is genuinely frozen.
- **Never exceed the 6V servo rail**, in any power supply configuration —
  the PCA9685 boards are rated for 6V maximum. This is a hard ceiling, not
  a nominal figure.

## When the link won't connect, check in this order

1. **Is the servo rail actually on?** If the ESP32 shares power with it,
   no rail power means no ESP32 at all. Check for any power LED on the
   board.
2. **Is the ESP32 on the same WiFi network as your laptop?** Not a guest
   network, not a different SSID — the exact same network, and one
   without client isolation (some guest/public networks block devices from
   reaching each other even when both are connected). If it couldn't join
   your network at all (or lost the connection mid-session and couldn't
   get back on), it falls back to hosting its own AP — check whether
   you're actually still on your normal network, or need to connect to the
   robot's own fallback network instead.
3. **Does `operator_config.yaml`'s `link.udp.robot_host` match the ESP32's
   actual IP right now?** If your network uses DHCP, the robot's address
   can change between sessions. Check your router's device list, or
   however your firmware reports its assigned IP.
4. **Is your laptop's firewall blocking the UDP port?** Especially after
   an OS update, or on a new machine.
5. **Check the app's log panel** for anything that looks like an error,
   and the LINK status light specifically (not the constants-warning
   banner, which is a separate, secondary check for once you already have
   a connection).
6. **Last resort: power-cycle the ESP32** (servo rail off, wait a few
   seconds, back on) and try again.

## If a servo buzzes, gets hot, or stops responding mid-session

**Buzzing.** It's being asked to hold or reach a position it physically
can't. Stop commanding it there immediately — release the slider toward
neutral, or hit EMERGENCY STOP. Don't wait to see if it stops on its own.
Note the buzz point in the servo's health note, and when you mark a safe
limit later, back off further than where the buzzing started.

**Getting hot.** Stop it immediately — EMERGENCY STOP or Park at neutral —
and let it cool before testing further. A hot servo has been under
sustained load, possibly stalled, for longer than it should have been.
This is exactly what the dwell-timeout protection exists to prevent; if it
happened anyway, check that `dwell_timeout_s` in `operator_config.yaml`
is actually set to something sane (a few seconds, not minutes).

**Stops responding.** Check the LINK status light first — if the whole
link is down, this isn't about that one servo. If the link is fine and
only this servo went quiet, it's likely failed. Note it as dead in its
health note, set it aside, and stop sending it commands — repeatedly
retrying a dead servo doesn't help and can mask a wiring problem you
should be looking at instead.
