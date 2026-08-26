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

Why USB and the servo rail are never live at the same time: connecting USB
ties your laptop's ground to the ESP32's ground. The servo rail is a 6V
supply feeding 18 servos, sharing that same ground reference through the
PCA9685 boards. With both connected, current from the servo rail (and it's
not small — a single MG996R can pull ~2.5A when stalled) has a path back
through that shared ground toward your laptop's USB port. That's a real
risk to your laptop, not a theoretical one. Servo rail off, always, before
USB goes in.

## 1. First-time bring-up

1. Servo rail **OFF**.
2. Connect the ESP32 to your laptop with USB.
3. Flash the firmware (see the firmware project's own build instructions
   for the exact command — this doc doesn't prescribe a toolchain).
4. While still on USB, configure the ESP32 with your WiFi network's
   credentials, however the firmware you flashed exposes that step (serial
   config prompt, a config file baked in at build time, etc. — check the
   firmware's own docs, since this isn't pinned down yet in this repo).
5. Unplug USB.
6. Servo rail **ON**.
7. Launch the GUI: `python app.py`.
8. Check the LINK status light in the app. Green means the ESP32 found
   your WiFi network and the app found the ESP32. If it's red, see
   "Link won't connect" below before doing anything else.

## 2. The routine flash cycle (updating firmware later)

The ESP32 stays mounted on the robot the whole time — you're not removing
it, just switching which connection is active.

1. Servo rail **OFF**.
2. Plug in USB.
3. Flash the new firmware.
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

## Never do this

- **Never connect USB while the servo rail is live.** See "the one rule
  that matters most" above — this is the laptop-damage risk, not a
  formality.
- **Never wire or unwire a servo on a live rail.** A brief short from a
  slipped wire is how you lose a PCA9685 channel or worse.
- **Never leave a servo held away from neutral and walk away.** Bench
  Test's dwell protection will auto-return it to neutral after a timeout —
  but that's a safety net, not permission to be careless. If you're
  stepping away, park it yourself first.
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
   reaching each other even when both are connected).
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
