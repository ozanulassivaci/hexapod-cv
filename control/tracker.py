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
there's a robot to test it against.
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
    cx's sign frame to frame."""

    turn_gain: float
    """Proportional gain: turn rate = clamp(turn_gain * cx, -1, 1) outside
    the dead zone."""

    turn_speed: float
    """speed value (0..100) used for every TurnCommand this tracker emits."""

    def __post_init__(self) -> None:
        if not (0.0 <= self.dead_zone < 1.0):
            raise ValueError(f"dead_zone must be in [0, 1), got {self.dead_zone!r}")
        if self.turn_gain <= 0:
            raise ValueError(f"turn_gain must be positive, got {self.turn_gain!r}")
        if not (0.0 <= self.turn_speed <= 100.0):
            raise ValueError(f"turn_speed must be in [0, 100], got {self.turn_speed!r}")


class Tracker:
    """Stateless policy: given the current frame's detections, decide what
    the robot should be doing right now. Always returns a concrete Command,
    never None -- there is no "do nothing" case. With no detections the
    safe action is an explicit stop, not silence (silence would just let
    the heartbeat keep resending whatever was last commanded, which could
    be an old turn)."""

    def __init__(self, config: TrackerConfig) -> None:
        self.config = config

    def decide(self, detections: list[Detection]) -> Command:
        if not detections:
            return StopCommand()

        target = max(detections, key=_bbox_area)

        if abs(target.cx) <= self.config.dead_zone:
            return TurnCommand(rate=0.0, speed=self.config.turn_speed)

        rate = target.cx * self.config.turn_gain
        rate = max(-1.0, min(1.0, rate))
        return TurnCommand(rate=rate, speed=self.config.turn_speed)


def _bbox_area(detection: Detection) -> float:
    _x, _y, w, h = detection.bbox
    return w * h
