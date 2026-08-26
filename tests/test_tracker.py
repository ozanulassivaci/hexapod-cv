import pytest

from control.tracker import Tracker, TrackerConfig
from perception.detection import Detection
from transport.protocol import StopCommand, TurnCommand

DEFAULT_CONFIG = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0)


def make_detection(cx: float, w: int = 10, h: int = 10, label: str = "blue_object") -> Detection:
    return Detection(label=label, cx=cx, cy=0.0, bbox=(0, 0, w, h), confidence=1.0)


def test_no_detections_returns_stop():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([])
    assert command == StopCommand()


def test_centered_target_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.0)])
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_within_dead_zone_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    # dead_zone is 0.1, so this should NOT produce a corrective turn.
    command = tracker.decide([make_detection(cx=0.09)])
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_at_dead_zone_boundary_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.1)])
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_right_of_dead_zone_turns_right():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.5)])
    assert isinstance(command, TurnCommand)
    assert command.rate == pytest.approx(0.5 * 0.8)
    assert command.rate > 0


def test_target_left_of_dead_zone_turns_left():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=-0.5)])
    assert isinstance(command, TurnCommand)
    assert command.rate == pytest.approx(-0.5 * 0.8)
    assert command.rate < 0


def test_rate_is_clamped_to_plus_minus_one():
    config = TrackerConfig(dead_zone=0.0, turn_gain=5.0, turn_speed=30.0)
    tracker = Tracker(config)
    command = tracker.decide([make_detection(cx=1.0)])
    assert command.rate == 1.0

    command = tracker.decide([make_detection(cx=-1.0)])
    assert command.rate == -1.0


def test_picks_largest_detection_by_area():
    tracker = Tracker(DEFAULT_CONFIG)
    small_but_first = make_detection(cx=-0.5, w=5, h=5)  # area 25
    large = make_detection(cx=0.5, w=20, h=20)  # area 400
    command = tracker.decide([small_but_first, large])
    assert isinstance(command, TurnCommand)
    assert command.rate > 0  # steered toward the large one (cx=0.5), not the small one


def test_turn_speed_from_config_is_used():
    config = TrackerConfig(dead_zone=0.1, turn_gain=1.0, turn_speed=75.0)
    tracker = Tracker(config)
    command = tracker.decide([make_detection(cx=0.5)])
    assert command.speed == 75.0


@pytest.mark.parametrize(
    "kwargs",
    [
        {"dead_zone": -0.1, "turn_gain": 1.0, "turn_speed": 30.0},
        {"dead_zone": 1.0, "turn_gain": 1.0, "turn_speed": 30.0},
        {"dead_zone": 0.1, "turn_gain": 0.0, "turn_speed": 30.0},
        {"dead_zone": 0.1, "turn_gain": -1.0, "turn_speed": 30.0},
        {"dead_zone": 0.1, "turn_gain": 1.0, "turn_speed": -1.0},
        {"dead_zone": 0.1, "turn_gain": 1.0, "turn_speed": 101.0},
    ],
)
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)


def test_decide_never_returns_none():
    tracker = Tracker(DEFAULT_CONFIG)
    for detections in ([], [make_detection(cx=0.0)], [make_detection(cx=0.9)]):
        assert tracker.decide(detections) is not None
