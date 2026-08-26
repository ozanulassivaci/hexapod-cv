import pytest

from control.tracker import Tracker, TrackerConfig
from perception.detection import Detection
from transport.protocol import StopCommand, TurnCommand

DEFAULT_CONFIG = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0)


def make_detection(cx: float, w: int = 10, h: int = 10, label: str = "blue_object") -> Detection:
    return Detection(label=label, cx=cx, cy=0.0, bbox=(0, 0, w, h), confidence=1.0)


def test_no_detections_returns_stop():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([], now=0.0)
    assert command == StopCommand()


def test_centered_target_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.0)], now=0.0)
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_within_dead_zone_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    # dead_zone is 0.1, so this should NOT produce a corrective turn.
    command = tracker.decide([make_detection(cx=0.09)], now=0.0)
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_at_dead_zone_boundary_holds_zero_rate():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.1)], now=0.0)
    assert command == TurnCommand(rate=0.0, speed=30.0)


def test_target_right_of_dead_zone_turns_right():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=0.5)], now=0.0)
    assert isinstance(command, TurnCommand)
    assert command.rate == pytest.approx(0.5 * 0.8)
    assert command.rate > 0


def test_target_left_of_dead_zone_turns_left():
    tracker = Tracker(DEFAULT_CONFIG)
    command = tracker.decide([make_detection(cx=-0.5)], now=0.0)
    assert isinstance(command, TurnCommand)
    assert command.rate == pytest.approx(-0.5 * 0.8)
    assert command.rate < 0


def test_rate_is_clamped_to_plus_minus_one():
    config = TrackerConfig(dead_zone=0.0, turn_gain=5.0, turn_speed=30.0)
    tracker = Tracker(config)
    command = tracker.decide([make_detection(cx=1.0)], now=0.0)
    assert command.rate == 1.0

    command = tracker.decide([make_detection(cx=-1.0)], now=0.1)
    assert command.rate == -1.0


def test_picks_largest_detection_by_area():
    tracker = Tracker(DEFAULT_CONFIG)
    small_but_first = make_detection(cx=-0.5, w=5, h=5)  # area 25
    large = make_detection(cx=0.5, w=20, h=20)  # area 400
    command = tracker.decide([small_but_first, large], now=0.0)
    assert isinstance(command, TurnCommand)
    assert command.rate > 0  # steered toward the large one (cx=0.5), not the small one


def test_turn_speed_from_config_is_used():
    config = TrackerConfig(dead_zone=0.1, turn_gain=1.0, turn_speed=75.0)
    tracker = Tracker(config)
    command = tracker.decide([make_detection(cx=0.5)], now=0.0)
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
        {"dead_zone": 0.1, "turn_gain": 1.0, "turn_speed": 30.0, "lost_target_hold_s": -0.1},
    ],
)
def test_invalid_config_rejected(kwargs):
    with pytest.raises(ValueError):
        TrackerConfig(**kwargs)


def test_decide_never_returns_none():
    tracker = Tracker(DEFAULT_CONFIG)
    for detections in ([], [make_detection(cx=0.0)], [make_detection(cx=0.9)]):
        assert tracker.decide(detections, now=0.0) is not None


# --- lost_target_hold_s: brief hold before giving up -----------------------


def test_never_seen_a_target_stops_immediately():
    """No grace period applies if a target was never seen in the first
    place -- there is no "last turn" to hold."""
    tracker = Tracker(DEFAULT_CONFIG)
    assert tracker.decide([], now=0.0) == StopCommand()
    assert tracker.decide([], now=100.0) == StopCommand()


def test_holds_last_turn_within_grace_period():
    config = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.3)
    tracker = Tracker(config)
    seen = tracker.decide([make_detection(cx=0.5)], now=0.0)
    assert isinstance(seen, TurnCommand) and seen.rate > 0

    held = tracker.decide([], now=0.2)  # within the 0.3s grace window
    assert held == seen


def test_falls_back_to_stop_after_grace_period_expires():
    config = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.3)
    tracker = Tracker(config)
    tracker.decide([make_detection(cx=0.5)], now=0.0)

    still_held = tracker.decide([], now=0.3)  # exactly at the boundary -- still held
    assert isinstance(still_held, TurnCommand)

    given_up = tracker.decide([], now=0.31)  # just past it
    assert given_up == StopCommand()


def test_redetecting_within_grace_period_resets_the_clock():
    config = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.2)
    tracker = Tracker(config)
    tracker.decide([make_detection(cx=0.5)], now=0.0)
    tracker.decide([], now=0.1)  # briefly lost, still within grace
    reseen = tracker.decide([make_detection(cx=0.6)], now=0.15)  # target reappears
    assert isinstance(reseen, TurnCommand) and reseen.rate > 0

    # Grace window should now count from t=0.15, not t=0.0 -- still held at
    # t=0.3 (0.15s since redetection, under the 0.2s window), even though
    # that's 0.3s since the *original* sighting.
    held_again = tracker.decide([], now=0.3)
    assert held_again == reseen


def test_zero_hold_stops_immediately_on_first_missed_frame():
    config = TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.0)
    tracker = Tracker(config)
    tracker.decide([make_detection(cx=0.5)], now=0.0)
    assert tracker.decide([], now=0.01) == StopCommand()


def test_lost_target_hold_default_is_positive():
    # Regression pin: the default must actually smooth over single-frame
    # dropout, not be a no-op zero.
    assert TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0).lost_target_hold_s > 0.0
