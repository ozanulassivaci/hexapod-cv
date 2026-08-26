# Operator GUI guide

This explains the app in `app.py`. No codebase knowledge assumed. Written
for late-night use — short sentences, explicit steps.

Run it with:

```
python app.py
```

It works with no camera and no robot connected. That's normal, not broken.

By default the app talks to `MockRobotLink` — a fake robot that
acknowledges everything instantly but never actually moves or walks over
time. Run `python app.py --link-mode sim` instead (or set `link.mode:
sim` in `operator_config.yaml`) to drive the software simulator, which
does run gait over time and shows it in a fourth **Simulator** tab — see
below. Neither mode needs a camera, real servos, or an ESP32.

## The tabs, in one paragraph each

**Operate** is for driving the robot: camera view on the left, controls on
the right, a log at the bottom. This is where you walk, turn, and use
AUTO_TRACK.

**Calibrate** is for fine-tuning servos that are already mounted on the
robot. You nudge each servo's position slightly so the leg sits where it
should, then save that correction. This assumes the robot is basically
built.

**Bench Test** is for a single loose servo, not yet mounted on anything —
literally sitting on your desk with one cable to a PCA9685 channel. No leg,
no IK, just a direct pulse to that one channel. This is what you use
*before* assembly: mounting horns, finding out if a servo is dead, finding
its safe range of motion.

**Simulator** (only shown when you launched with `--link-mode sim`) is a
live top-down view of gait — body position/heading and which legs are
currently planted vs. lifted — driven by the exact same WASD/Q/E/speed/
body-height controls on the Operate tab. Not a physics simulation and not
meant to look realistic; it exists to catch a gait bug (wrong tripod
grouping, a joint that won't clamp, a command that does the wrong thing)
before it ever reaches a real servo. See "Using the Simulator tab" below.

If your servos just arrived and nothing is assembled yet, you want **Bench
Test**. If the robot is built and you're tuning it, you want **Calibrate**.
If you want to sanity-check gait logic itself — including the link
failsafe — with no hardware at all, you want **Simulator**.

## Two concepts you need before any of this makes sense

**Servo offset, and why it exists.** A servo horn (the little arm or disc
on top) attaches to the servo's output shaft over fine teeth (a "spline").
You can only press the horn on at one of those tooth positions, not at any
arbitrary angle. The nearest available tooth is rarely *exactly* where the
servo's true center is — it's off by some small amount, different for
every single unit. `offset_us` is a number, in microseconds, that corrects
for that specific unit's specific error. You find this number during
calibration, after the horn is already mounted, by nudging until the leg
sits right. You do **not** try to get it perfect while mounting the horn —
that's the bench-test park step, and it's deliberately not fussy (see the
walkthrough below).

**Arming, and why calibration/bench writes are gated.** "Arming" means
flipping an explicit switch before the app is allowed to write anything
that matters — a servo trim, a safety limit. Nothing gets written while
disarmed; the robot refuses it. This exists so a bug or a corrupted network
message can't quietly rewrite your calibration in the background — it has
to be deliberately unlocked first. It also **auto-disarms itself** after a
stretch of inactivity, so you can't forget it's on and leave it unlocked
overnight. You'll re-arm it often; that's expected, not a nuisance.

**The offset table, and where it actually lives.** Every servo's saved
correction (offset, safe limits, your notes) lives in the *robot's* memory,
not your laptop's. The app is a window into that data: it can read it, show
it to you, change it, or save a backup copy to a file on your computer
(export). If you're running against `MockRobotLink` (the default, no
hardware needed), that data lives only in the app's memory for as long as
the app stays open — **closing the app throws it away**. There is no
autosave. Export before you close, every time, if the session mattered.

## Operate tab, control by control

- **Camera / Link status (green or red).** Green means connected. Red
  means not connected — no camera reachable, or no robot responding. A red
  camera light with no camera plugged in is completely normal; the app is
  designed to run that way.
- **SAFETY badge (blue "BENCH" or purple "ASSEMBLED").** Which
  fault-response mode the *connected firmware* was compiled with —
  `ROBOT_ASSEMBLED` in `firmware/include/Config.h`, see
  `docs/HOW_TO_USE.md`. Blue/purple on purpose, not red/green: neither
  mode is "wrong" by itself, and this app cannot tell you whether it's
  the *right* one for the robot in front of you right now — that's a
  build-time flag with no sensor behind it. This badge exists so you
  don't have to go looking for the answer; it doesn't replace checking it
  against what you actually know about the robot's assembly state.
  Reads "SAFETY: n/a" before the first telemetry arrives.
- **Constants warning banner (only appears if something's wrong).** If it
  shows up, the robot and this app disagree about how long they'll wait
  before timing out the link. This is a real safety mismatch, not
  cosmetic — don't ignore it. If you see it, stop and figure out why before
  relying on the failsafe.
- **Telemetry readout (RTT, last applied, gait phase, faults).** Your
  diagnostic panel. If the robot "isn't doing anything," this is where you
  look first: is the link even up (RTT), did the robot receive what you
  think you sent (last applied), is it stuck in a fault state (faults).
- **WASD / arrow keys.** Walk. Holding a key keeps walking; releasing it
  stops. Two keys at once (like W+D) walks diagonally. Get it wrong and the
  robot walks the wrong direction — release the key, it stops immediately.
- **Q / E.** Turn in place. If you're also holding a walk key, the walk
  wins and turning is ignored — release the walk key first if you want to
  turn.
- **Space.** Immediate stop. Works no matter what you were doing,
  including AUTO_TRACK (which it also cancels). If you're ever unsure what
  the robot is about to do, hit Space first and figure it out after.
- **Speed / body height / pan / tilt sliders.** Live — they take effect as
  you drag them, no confirm step. Speed only matters while you're actively
  walking or turning. Get pan/tilt wrong and the camera just points
  somewhere unhelpful; nothing is damaged.
- **MANUAL / AUTO_TRACK toggle.** AUTO_TRACK steers automatically toward
  whatever the vision system is currently tracking. Any key press instantly
  drops you back to MANUAL — you can always interrupt it just by touching
  the keyboard.
- **EMERGENCY STOP (big red button).** Same as Space, but click instead of
  keyboard. Also parks any bench-test servo that's mid-sweep or held away
  from neutral. When in doubt, hit this.

## Calibrate tab, control by control

- **Arm / Disarm + countdown.** Arm before any write below will work. The
  countdown is the app's own estimate of when it'll auto-disarm, not a
  live number from the robot — see the note at the end of this doc. Trust
  the ARMED/DISARMED word, treat the seconds as approximate.
- **Servo selector.** Picks which of the 18 servos you're adjusting, shown
  by name (e.g. "RF coxa") next to the raw number, so you're not counting
  positions in your head.
- **Offset slider + sign.** Stages a correction for the selected servo.
  Nothing is sent until you click Apply.
- **Apply.** Sends the staged offset/sign to the robot. Requires armed.
  Rejected (with a reason) if not armed, or if the robot says no.
- **Read profiles from robot.** Pulls the full 18-servo table — offset,
  sign, bench-recorded limits, your health notes — and fills the table
  below. Always works, armed or not; reading isn't a write.
- **Profile table.** What the robot actually has right now, as of your
  last "Read." It does not update live — if you just recorded something on
  the Bench Test tab, click Read again to see it here.
- **Export to YAML.** Saves everything currently shown in the table to a
  file you choose. This is the real safety net for this whole tab — see
  "things that can cost you a session" below.
- **Import from YAML.** Loads a file and writes its offset/sign values
  back to the robot, after you confirm. Only restores offset/sign, not
  bench-recorded limits or notes — those live and travel with bench
  testing, not this import.

## Bench Test tab, control by control

- **The note at the top.** Read it once: this tab has no leg, no IK. The
  pulse number you set goes straight to one wire. Nothing here knows or
  cares about the robot's geometry.
- **Bench arm / Disarm + countdown.** Separate switch from Calibrate's.
  Arming this does not arm calibration, and vice versa — on purpose, so a
  mistake in one workflow can't reach into the other.
- **Board / channel.** Which physical PCA9685 pin the loose servo is
  plugged into right now. Changing this always re-parks at neutral first —
  it assumes you've physically moved the cable to a different servo.
- **Park at neutral.** Holds the servo at its standard center position,
  indefinitely, with no auto-timeout. This is the button you want while
  physically pressing a horn onto the spline — see the walkthrough.
- **Manual pulse slider.** Live, in microseconds. Moves the servo as you
  drag. Away from neutral, this is being watched by the stall protection
  below.
- **Range finder nudge buttons (−/+).** Small steps, for creeping toward a
  servo's mechanical limit carefully instead of guessing with the slider.
- **Mark current pulse as MIN / MAX limit.** Records whatever pulse is
  currently commanded as this servo's safe boundary, under whichever
  future position you've selected below. This is a real safety value the
  firmware is meant to respect later — don't mark a limit you haven't
  actually confirmed is safe.
- **Sweep: min / max / seconds, Start, ABORT SWEEP.** One slow pass from
  min to max and back to neutral, automatically. Defaults are deliberately
  narrow — widen them yourself once you trust the servo, don't start wide.
  ABORT is always live during a sweep and stops it immediately.
- **"This unit will become" servo selector.** Which future leg/joint
  position you're recording findings for. Independent of the board/channel
  above — the loose servo in your hand doesn't have a leg position yet,
  you're just deciding where its recorded data will end up.
- **Health note field + Save note.** Free text per servo — "buzzes at low
  end," "dead," whatever you'll want to remember at 2am on unit #14. Not
  gated by arming; you can always leave a note.
- **The stall warning banner.** Appears when a servo has been held away
  from neutral for a while and shows a countdown to when the app will force
  it back to neutral on its own. This is the app actively working against
  you leaving a servo stalled — see below.

## Using the Simulator tab

There's nothing to configure on this tab — it's a view, not a set of
controls. Everything driving it lives on the Operate tab.

- **The circle in the middle** is the robot's body. Blue means the link
  is up; gray means it's timed out (see below) — the same distinction the
  Operate tab's LINK light shows, drawn a different way.
- **The line from the circle** is heading — which way the body is
  currently facing, from Q/E turning.
- **The six dots** are feet. Green means that leg is currently planted
  (stance); orange means it's lifted mid-swing. Watching these while
  holding W is the actual point of this tab: at any moment exactly three
  should be green and three orange, alternating — a tripod gait. If that
  pattern looks wrong, something in the gait logic is wrong, and it's far
  cheaper to notice it here than after servos are mounted.
- **The `phase=` readout** top-left is the raw gait cycle position
  (0.0–1.0, wrapping). Frozen means gait isn't advancing — either you're
  not commanding any movement (idle is supposed to freeze it) or the link
  has timed out (see below).

- **"Simulate link drop" button**, above the view. Holding a key steady
  does *not* trip the failsafe on its own — like a real robot connection,
  the simulator keeps itself alive between your key presses automatically
  (a heartbeat), the same way a real link does, so there's no ordinary
  way to make it go quiet just by sitting still. This button is the
  deliberate exception: it briefly suppresses that heartbeat so you can
  actually watch the failsafe trip — the body turns gray, the status text
  switches to "DISCONNECTED (failsafe — gait frozen)", and `phase=`
  stops advancing, all while you keep holding a walk key. It recovers on
  its own once the drop window ends. Real hardware has no equivalent live
  control for this (short of physically interrupting WiFi) — it's
  simulator-only, for exactly this purpose.

## Walkthrough: bench-testing a loose servo and parking it to mount a horn

1. Plug the loose servo into a PCA9685 channel. Note which board and
   channel.
2. Launch `python app.py`. Go to the **Bench Test** tab.
3. Set **board** and **channel** to match your wiring.
4. Click **Arm bench mode**.
5. Click **Park at neutral**. The servo moves to center and holds — this
   has no timeout, it will sit there as long as you leave it armed.
6. Press the horn onto the spline while it's parked. Get it as close to
   straight as the spline teeth allow — don't chase perfection here, that's
   what calibration is for later.
7. If you want to check the servo's range before moving on: use the
   **manual slider** or **nudge buttons** to creep toward each end, and
   click **Mark current pulse as MIN/MAX limit** once you find where it's
   safe to stop, under whichever future position this unit will occupy.
8. Click **Disarm** when you're done with this unit, or just change
   board/channel to move to the next one (that also re-parks automatically).
9. Repeat for the next loose servo.

## Walkthrough: a full 18-servo calibration session ending in export

1. With the robot assembled and powered, launch `python app.py`. Go to the
   **Calibrate** tab.
2. Click **Arm**.
3. For servo 0: select it, adjust the **offset** slider and **sign** until
   the leg sits correctly, click **Apply**.
4. Repeat step 3 for every servo, 0 through 17. The countdown will keep
   resetting as long as you keep applying — you shouldn't get kicked out
   mid-session under normal pace.
5. When all 18 are done, click **Read profiles from robot** to confirm
   what actually landed.
6. Click **Export to YAML**. Save it somewhere you'll find again — the
   date and a short description in the filename is worth the ten seconds.
7. Click **Disarm**. If anything changed since your export, you'll get a
   prompt asking whether to export again first — say yes unless you're
   sure.

## Things that can cost you money

- **Leaving a servo held at an extreme pulse and walking away.** A stalled
  MG996R draws roughly 2.5A and can burn out in minutes. The Bench Test
  tab's dwell protection auto-returns to neutral after a timeout and warns
  you with a visible countdown before it does — but don't treat that as
  permission to be careless. If you're stepping away, click **Park at
  neutral** yourself first.
- **Closing the app (or the mock link resetting) without exporting.**
  Against `MockRobotLink`, nothing is saved anywhere except your last
  export. A long calibration or bench session with no export is one crash
  away from starting over. Export often, not just at the very end.
- **Importing an old or wrong YAML file.** You do get a confirmation
  dialog with the file path and count before anything is written — read
  it. A confidently-clicked "Yes" on the wrong file overwrites good
  calibration with stale data.
- **Marking a bench limit you haven't actually verified.** `record_limit`
  is meant to become a real safety boundary once firmware enforces it.
  Marking a limit based on a guess rather than an actual observed edge
  defeats the point of bench testing in the first place.
- **Ignoring the unexported-changes warning banner.** If it's showing, the
  robot's current state and your last export have diverged. It won't go
  away on its own — export again to clear it.

## About that countdown timer

Both arm/disarm countdowns (Calibrate and Bench Test) are the app's *own
guess*, not a live number the robot is sending every second. The app
remembers roughly when it last armed or refreshed things and counts down
locally from the known timeout length. It re-checks the actual
ARMED/DISARMED state with the robot on every update, so that word is
always right — but the seconds shown can be a little off, especially if
you've customized the timeout away from the app's default. Treat the
countdown as "roughly this long," not a precise deadline.
