"""AUTO_TRACK policy: turn the robot to keep the largest detection centered.

No Qt imports -- the UI only ever calls decide() and sends whatever it
returns. Kept separate from ui/ specifically so it's unit-testable without
a display or an event loop.

Sign convention: positive Detection.cx means the target is to the right of
frame center (see perception/detection.py). This maps to a positive turn
rate here, i.e. rate>0 is assumed to steer toward the right. Nothing in
the reference firmware or the protocol pins down which physical direction
a positive turn rate drives once real hardware exists -- this is a
documented assumption, to be confirmed (and flipped here if wrong) once
there's a robot to test it against. simulator/synthetic_camera.py picks a
convention that makes this converge in closed-loop simulation, which is
the strongest check available without real hardware.
"""

from dataclasses import dataclass

from perception.detection import Detection
from transport.protocol import Command, StopCommand, TurnCommand


@dataclass(frozen=True)
class TrackerConfig:
    dead_zone: float
    """Half-width, in normalized cx units, of the zone around center where
    no corrective turn is sent (an explicit rate=0 hold instead) -- without
    this the robot oscillates around dead center as detection noise flips
    cx's sign frame to frame. Empirically, ~0.05 is the minimum that
    suppresses chatter under realistic per-frame detection jitter (see the
    design discussion this was tuned from) -- the default (0.1) carries
    roughly a 2x margin over that."""

    turn_gain: float
    """Proportional gain: turn rate = clamp(turn_gain * cx, -1, 1) outside
    the dead zone. Empirically stable (no sustained oscillation) across the
    full realistic camera-latency range (0-500ms) at the default value
    (0.8); instability only appeared in closed-loop testing around 5x that
    gain at high latency, a wide margin."""

    turn_speed: float
    """speed value (0..100) used for every TurnCommand this tracker emits."""

    lost_target_hold_s: float = 0.3
    """Seconds to keep repeating the last turn command after the target
    stops being detected, before falling back to an explicit stop. Found
    necessary empirically: with realistic single/few-frame detection
    dropout (contour noise, compression artifacts -- not the target
    actually leaving), reverting to StopCommand on the very first missed
    frame produced a command change on roughly 1 in 9 ticks -- visibly
    jerky stop-start motion, not smooth tracking, even though the target
    was still right there. 0.3s comfortably covers a few consecutive
    missed frames at a real MJPEG stream's practical rate (~15fps, ~67ms/
    frame) without meaningfully delaying the response to the target
    actually being gone."""

    def __post_init__(self) -> None:
        if not (0.0 <= self.dead_zone < 1.0):
            raise ValueError(f"dead_zone must be in [0, 1), got {self.dead_zone!r}")
        if self.turn_gain <= 0:
            raise ValueError(f"turn_gain must be positive, got {self.turn_gain!r}")
        if not (0.0 <= self.turn_speed <= 100.0):
            raise ValueError(f"turn_speed must be in [0, 100], got {self.turn_speed!r}")
        if self.lost_target_hold_s < 0:
            raise ValueError(f"lost_target_hold_s must be >= 0, got {self.lost_target_hold_s!r}")


class Tracker:
    """Given the current frame's detections (and the current time), decide
    what the robot should be doing right now. Always returns a concrete
    Command, never None -- there is no "do nothing" case.

    Not stateless anymore (it was, before lost_target_hold_s) -- it
    remembers the last turn it issued and when the target was last seen,
    the same "pure logic, caller supplies time explicitly" shape
    control/bench.py's DwellGuard already uses, for the same reason: unit
    testable with fabricated timestamps, no real waiting, no Qt."""

    def __init__(self, config: TrackerConfig) -> None:
        self.config = config
        self._last_turn: TurnCommand | None = None
        self._last_seen_at: float | None = None

    def decide(self, detections: list[Detection], now: float) -> Command:
        if detections:
            target = max(detections, key=_bbox_area)

            if abs(target.cx) <= self.config.dead_zone:
                command = TurnCommand(rate=0.0, speed=self.config.turn_speed)
            else:
                rate = target.cx * self.config.turn_gain
                rate = max(-1.0, min(1.0, rate))
                command = TurnCommand(rate=rate, speed=self.config.turn_speed)

            self._last_turn = command
            self._last_seen_at = now
            return command

        # No detection this tick -- briefly hold the last turn instead of
        # stopping on the very first missed frame (see
        # lost_target_hold_s's docstring for why this exists).
        if (
            self._last_turn is not None
            and self._last_seen_at is not None
            and now - self._last_seen_at <= self.config.lost_target_hold_s
        ):
            return self._last_turn

        return StopCommand()


def _bbox_area(detection: Detection) -> float:
    _x, _y, w, h = detection.bbox
    return w * h
