# Operator GUI guide

This explains the app in `app.py`. No codebase knowledge assumed. Written
for late-night use — short sentences, explicit steps.

Run it with:

```
python app.py
```

It works with no camera and no robot connected. That's normal, not broken.

`operator_config.yaml`'s `link.mode` picks which robot the app talks to,
and the shipped file defaults to `udp` — real hardware, at the address in
that same file — now that a robot exists to bring up. For hardware-free
work, override it for one run with `python app.py --link-mode mock` (a
fake robot that acknowledges everything instantly but never actually
moves or walks over time) or `python app.py --link-mode sim` (the
software simulator, which does run gait over time and shows it in a
fourth **Simulator** tab — see below). Neither mode needs a camera, real
servos, or an ESP32. `--link-mode` always overrides the config file for
that run without editing it.

`python app.py --camera-mode synthetic` goes one step further and
replaces the real camera too, with a rendered target that reacts to the
simulator's own heading. AUTO_TRACK on this mode closes the entire
camera → detection → tracker → gait loop with nothing real anywhere in
it — the fastest way to see the whole pipeline work, or to reproduce a
tracking problem without a camera in hand. Implies `--link-mode sim`.

## The window itself

Opens at a large default size (90% of your screen's available area, capped
at 1600x1000) and is freely resizable — drag an edge, use your window
manager's maximise, or press **F11** to toggle true fullscreen (no window
chrome at all; F11 again returns to normal). Nothing about size or
position is remembered between runs — every launch starts from the same
computed default.

The video feed on the Operate tab and the two 2D views on the Test Leg
tab grow with the window — those are the two places extra screen space
actually helps. Everything else grows more modestly, sticking near a
comfortable working size and leaving any extra space as margin rather
than stretching a control table or a button row for no reason.

The **Bench Test** and **Test Leg** tabs scroll if the window is too
short to show everything at once (their content — several joints' worth
of controls — is taller than any panel-sized share of a smaller window
should be forced to be). On the Test Leg tab specifically, the Joint/IK
mode controls near the bottom stay in their own small scrollable area
even in a large window, on purpose: the 2D views above them are what
benefits from a bigger window, so they get first claim on any extra
height instead of splitting it with the controls below.

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

**Test Leg** is for the single fully-assembled test leg — three servos,
one coxa/femur/tibia, clamped to a table edge, before any of the other
five legs exist (`reference/ANALYSIS.md`'s bring-up plan). Unlike Bench
Test, this one *does* know about kinematics: it can drive all three
joints from a single foot X/Y/Z target, and it's where you actually find
and mark the leg's real mechanical limits, one joint at a time, with a
built-in safety rule against jumping into unverified territory.

**Simulator** (only shown when you launched with `--link-mode sim`) is a
live top-down view of gait — body position/heading and which legs are
currently planted vs. lifted — driven by the exact same WASD/Q/E/speed/
body-height controls on the Operate tab. Not a physics simulation and not
meant to look realistic; it exists to catch a gait bug (wrong tripod
grouping, a joint that won't clamp, a command that does the wrong thing)
before it ever reaches a real servo. See "Using the Simulator tab" below.

If your servos just arrived and nothing is assembled yet, you want **Bench
Test**. Once you have one leg assembled and clamped down but nothing else
built, you want **Test Leg**. If the robot is fully built and you're
tuning it, you want **Calibrate**. If you want to sanity-check gait logic
itself — including the link failsafe — with no hardware at all, you want
**Simulator**.

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
(export). If you're running against `MockRobotLink` (`--link-mode mock`,
no hardware needed), that data lives only in the app's memory for as long
as the app stays open — **closing the app throws it away**. There is no
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
- **Telemetry readout (RTT, last applied, gait phase, faults, IK clip,
  joint clip, drops, last accepted seq).** Your diagnostic panel. If the robot "isn't doing
  anything," this is where you look first: is the link even up (RTT),
  did the robot receive what you think you sent (last applied), is it
  stuck in a fault state (faults).
  - **faults** showing `IK_CLIP` means the gait engine's most recent
    tick had to force a leg's target back inside what's actually
    reachable, or a solved joint angle back inside its configured
    bound. Today, with nothing yet assembled, that should never happen
    during ordinary driving — the gait envelope was tuned to have zero
    margin violations across the full range of speed/direction/turn/
    height commands. If you see it, something is commanding gait
    outside that verified-safe range; worth figuring out why before
    trusting the robot's pose, not something to click past.
  - **IK clip** / **joint clip** show the cumulative count and worst
    overshoot (mm / degrees) behind that fault name, since the robot's
    gait engine last reset (boot, or reconnecting to `--link-mode sim`)
    — not since this window opened. A single old event from before you
    connected can leave a nonzero count showing with the `IK_CLIP` fault
    itself no longer active; the faults line is the "right now" signal,
    these two are the "how much/how bad so far" detail behind it.
  - **drops** and **robot last accepted seq** answer a question the LINK
    light on its own cannot: when nothing is getting through, are your
    packets not arriving, or arriving and being thrown away? Both come
    from the robot and count since *its* boot, not since this window
    opened.
    - `stale` drops climbing while LINK is red means packets are
      arriving fine and the robot is rejecting them as out-of-order.
      That normally resolves itself: leave the app alone for a couple of
      seconds and the robot re-baselines onto your sequence numbers (see
      `docs/protocol.md` §4). If it doesn't, restart the app rather than
      the robot — a stale count that keeps climbing means something is
      still sending on the old numbering.
    - `malformed` drops climbing means packets arrive but the robot
      can't parse them at all, which almost always means the PC and the
      firmware were built from different protocol versions. Reflash.
    - **robot last accepted seq** far above what a freshly-started app
      would be sending is the same story from the other side. `none yet`
      means the robot has not accepted a single packet since it booted.
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
- **0° / 90° / 180° convenience buttons.** Quick jumps for horn-pressing,
  not the servo's literal nominal extremes — 0°/180° actually command a
  conservative approximation (600us/2400us, not 500us/2500us), since clone
  servos commonly can't physically reach the true nominal ends and will
  stall trying. 90° holds indefinitely, same as Park. 0°/180° hold only
  briefly (a couple of seconds) and then auto-return to 90° on their own —
  a countdown shows underneath while that's happening. Clicking anything
  else (slider, nudge, Park, a different convenience button, a sweep or
  check below) cancels that countdown early.
- **Mark current pulse as MIN / MAX limit.** Records whatever pulse is
  currently commanded as this servo's safe boundary, under whichever
  future position you've selected below. This is a real safety value the
  firmware is meant to respect later — don't mark a limit you haven't
  actually confirmed is safe. Stored as degrees from this joint's own
  neutral, not the raw pulse number you were looking at — deliberately,
  so a limit measured once on this one test leg's coxa (say) applies
  correctly to all six coxa servos later, even though each gets its own
  separate mounting-error correction. The profile table (Calibrate tab)
  shows it back to you in those same degrees, not microseconds.
- **Sweep: min / max / seconds, Start, ABORT SWEEP.** One slow pass from
  min to max and back to neutral, automatically. Defaults are deliberately
  narrow — widen them yourself once you trust the servo, don't start wide.
  ABORT is always live during a sweep and stops it immediately.
- **Repeatability check.** Drives 90° → 0° → 90°, pausing a couple of
  seconds at each stop so you can actually watch it, then asks: did it
  return to the same point both times? There's no position sensor
  anywhere in this system — this only paces the sequence for you to look
  at, the yes/no judgment is entirely yours. Abort stops it and parks at
  neutral. Starting this (or the hold check below) cancels whatever else
  was running, and vice versa — only one automated sequence runs at a
  time.
- **Hold check.** Parks at neutral and waits 30 seconds so you can listen
  for hunting or buzzing — a servo that's stalled or miscalibrated will
  audibly fight to hold position even when told to sit still. Same
  "you make the call" pattern as the repeatability check: it only times
  the wait, then asks silent or hunting/buzzing?
- **"This unit will become" servo selector.** Which future leg/joint
  position you're recording findings for. Independent of the board/channel
  above — the loose servo in your hand doesn't have a leg position yet,
  you're just deciding where its recorded data will end up.
- **Physical unit ID (optional).** A free-text field for a number you've
  written on the servo itself with a marker. When filled in, repeatability
  and hold check results are logged against that instead of the servo
  selector above — useful if you're tracking a specific physical unit
  across sessions, or testing it before you've decided its final leg
  position. Leave it blank and results log against the servo selector as
  usual.
- **Health note field + Save note.** Free text per servo — "buzzes at low
  end," "dead," whatever you'll want to remember at 2am on unit #14. Not
  gated by arming; you can always leave a note.
- **The stall warning banner.** Appears when a servo has been held away
  from neutral for a while and shows a countdown to when the app will force
  it back to neutral on its own. This is the app actively working against
  you leaving a servo stalled — see below.

## Test Leg tab, control by control

This tab assumes the single test leg is already assembled and clamped
down, horns already mounted at the angles in `docs/HOW_TO_USE.md` — it's
for finding real mechanical limits and sanity-checking IK, not for the
bare-servo bring-up Bench Test covers.

- **Bench arm / disarm + countdown.** The exact same switch as the Bench
  Test tab's — there is only one bench-armed state on the robot. Arming
  here also shows as armed on Bench Test, and vice versa.
- **Leg position selector ("this test leg will become").** Which of the
  six leg positions (RF/RM/RR/LR/LM/LF) this leg's findings are recorded
  under — determines which three servo_index slots (coxa/femur/tibia for
  that leg) get written when you mark a limit.
- **Channel wiring (coxa / femur / tibia rows).** Which physical PCA9685
  board+channel each of the three servos is on. Set this up once; both
  Joint mode and IK mode below use it.
- **RELEASE LEG.** Instant, always visible regardless of mode — all three
  joints go limp at once. Not the same as parking at neutral: a park still
  commands and holds a pulse, this commands nothing. Also fires
  automatically on the app's global EMERGENCY STOP and on closing the
  window, same as Bench Test's own servo.
- **Side view / top view.** Two flat 2D views, not one 3D one — easier to
  read at a glance, and each maps onto something already computed
  elsewhere in this project. Side view is the femur/tibia plane, seen
  from directly beside the leg (independent of which way coxa is
  pointed). Top view is coxa's rotation, seen from above, with a line
  whose length is the leg's current horizontal reach. Both update the
  instant you click a step button or drive an IK target — not on some
  delay.
  - **The white stick figure** is the commanded pose, live. Each segment
    is colored on its own: green (comfortably inside its safe limit),
    amber (getting close), red (at or past it), or plain white/grey if
    nothing's been marked for that joint yet — grey deliberately isn't
    "assumed safe," it's "not assessed."
  - **The blue wedge** is the gait envelope — what gait's own math would
    actually ask that joint to do. If your marked limits don't
    comfortably contain the blue wedge, gait will get pulses refused
    once it's actually driving this leg.
  - **The red wedge** is a forbidden zone — past a limit you've actually
    marked. Nothing shows here until you've marked something.
  - **The faint grey shape (side view only)** is roughly how far the
    foot could reach given the joints' full nominal travel — context for
    how much of the theoretical range you're actually using, not a
    precise boundary.
  - **"COMMANDED POSE — NOT MEASURED"**, directly under the views. There
    is no position feedback anywhere in this system — nothing here can
    tell you the real leg matches what's drawn. This is exactly the
    situation where it wouldn't: the screen can look fine while the real
    leg is bound against plastic.
- **Joint mode / IK mode toggle.** Switches which set of controls is
  showing below. Both drive the same three servos through the same
  channel wiring above.

**Joint mode:**

- **Suggested order.** Coxa first (doesn't depend on the other two), then
  tibia (its check works before the femur horn is even on), then femur
  (last, once the other two are already fixed) — the identical order and
  reasoning as the horn-mounting procedure in `docs/HOW_TO_USE.md`.
- **Per-joint angle display + step buttons (±1° / ±2° / ±5° / ±10°).**
  Each joint moves independently. A larger step button is enabled in a
  direction as long as landing there stays inside *either* a limit
  you've marked, *or* the gait envelope — the range this joint will
  actually be commanded across during normal walking, computed already
  from the kinematics and not unknown territory. With nothing marked
  yet, that means full-size steps work immediately inside the envelope
  (where gait was always going to send this joint anyway), and only drop
  to 1-degree-only once a step would land *past* the envelope — that's
  the genuine unknown-territory boundary this rule exists to slow you
  down for. Buttons that would overshoot a boundary switch off one size
  at a time as you approach it (10° first, then 5°, then 2°), so you
  naturally end up creeping the last few degrees on 1° instead of
  slamming into a hard cutoff. As you mark real mechanical limits, the
  buttons within the now-known-safe side open back up the same way,
  extending as far as the mark allows.
- **Mark current as MIN / MAX (per joint).** Same underlying write as
  Bench Test's Mark buttons (`record_limit`), scoped to whichever joint's
  row you click it on. The value to record is 3-5 degrees back from where
  binding actually starts — not the binding point itself.
- **Known limits (per joint).** What's currently recorded, in degrees
  from that joint's own neutral — refreshes automatically after you mark
  something.

**IK mode:**

- **Foot X / Y / Z (mm).** In this leg's own local frame, not the robot's
  body frame — there's no body yet. Measure with a ruler from the coxa's
  own rotation axis: X forward along the coxa's zero direction (straight
  out from the mount), Y sideways, Z down.
- **Drive to this target.** Runs the same IK this project's gait uses,
  clamps the result the same way, and commands all three joints. This is
  deliberately not step-limited like Joint mode — it jumps straight to
  the computed target, so use Joint mode first to establish real limits;
  only limits already marked are enforced here.
- **Resulting joint angles.** What IK actually computed (after clamping),
  in degrees from each joint's own neutral — this is where a sign error
  in the physical build would show up as an obviously-wrong direction of
  movement for a given target.
- **Achieved foot position.** The foot position you'd actually get from
  the joint angles above, recomputed through forward kinematics — not
  just an echo of what you typed in. If it doesn't match your requested
  X/Y/Z, IK had to clamp something (the target was out of reach, or a
  joint angle hit its configured bound), and the tab says so explicitly.
- **"COMMANDED POSE — NOT MEASURED" banner.** Permanent, not just an IK
  mode thing. There is no position feedback anywhere in this system — the
  screen can say one thing while the real leg is bound against plastic.

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

## Walkthrough: finding a real limit on the assembled test leg

1. Leg clamped down, horns already mounted at the angles in
   `docs/HOW_TO_USE.md`, all three servos wired. Launch `python app.py`.
   Go to the **Test Leg** tab.
2. Set the three **channel wiring** rows to match your coxa/femur/tibia
   wiring, and pick the **leg position** this test leg's findings should
   be recorded under.
3. Click **Arm bench mode**. Make sure **Joint mode** is selected.
4. Pick a joint — coxa first, per the suggested order shown. Larger step
   buttons already work in both directions within the gait envelope, so
   use them to get through the already-known-safe range quickly. Once a
   button stops being available, you've reached the envelope edge —
   switch to **+1°** (or **-1°**) and continue creeping, pausing to watch
   the leg each time.
5. The moment you see or hear binding (a servo working against
   resistance, not just reaching the end of a comfortable range), stop.
   Step back 3-5 degrees the way you came — don't record the binding
   point itself.
6. Click **Mark current as MIN** (or **MAX**, depending on which
   direction you were exploring). The known-limits line updates.
7. Once one side is marked, larger steps open back up on that side too —
   as far as the mark allows, or the envelope, whichever reaches
   further — the other direction is still envelope-then-1-degree until
   you repeat the process there.
8. Repeat for tibia, then femur, in that order.
9. If something looks wrong at any point, click **RELEASE LEG** — it's
   always visible, works instantly, and doesn't wait for you to finish
   whatever step you were on.

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
- **Using Test Leg's IK mode before Joint mode has found real limits.**
  IK mode jumps straight to a computed target — it doesn't have Joint
  mode's step-size safety rule, since a jump-to-target action doesn't
  have "the position you were already verified at" for that rule to
  reason from. Only limits you've already marked in Joint mode are
  enforced. On a fresh, unmarked leg, an IK target can drive a joint
  somewhere nobody's confirmed is safe.
- **Ignoring the unexported-changes warning banner.** If it's showing, the
  robot's current state and your last export have diverged. It won't go
  away on its own — export again to clear it.

## About that countdown timer

Both arm/disarm countdowns (Calibrate, and Bench Test/Test Leg — which
share one countdown, since they share one bench-armed state on the robot)
are the app's *own guess*, not a live number the robot is sending every
second. The app
remembers roughly when it last armed or refreshed things and counts down
locally from the known timeout length. It re-checks the actual
ARMED/DISARMED state with the robot on every update, so that word is
always right — but the seconds shown can be a little off, especially if
you've customized the timeout away from the app's default. Treat the
countdown as "roughly this long," not a precise deadline.
