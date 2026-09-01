import pytest

from control.bench import BenchSafetyConfig, DwellGuard, HoldCheck, MomentaryHold, RepeatabilityCheck, SweepPlan

DEFAULT_CONFIG = BenchSafetyConfig(neutral_pulse_us=1500, dwell_timeout_s=8.0)


# --- DwellGuard ----------------------------------------------------------


def test_neutral_pulse_never_trips_guard():
    guard = DwellGuard(DEFAULT_CONFIG)
    assert guard.observe(1500, now=0.0) is False
    assert guard.observe(1500, now=1000.0) is False  # arbitrarily long, still neutral


def test_non_neutral_pulse_does_not_trip_before_timeout():
    guard = DwellGuard(DEFAULT_CONFIG)
    assert guard.observe(1600, now=0.0) is False
    assert guard.observe(1600, now=7.9) is False


def test_non_neutral_pulse_trips_at_timeout():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    assert guard.observe(1600, now=8.0) is True


def test_returning_to_neutral_resets_the_clock():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    guard.observe(1500, now=5.0)  # back to neutral, resets
    assert guard.observe(1600, now=12.0) is False  # only 0s elapsed since re-leaving neutral


def test_moving_to_a_different_non_neutral_pulse_does_not_reset_the_clock():
    """Nudging around within the non-neutral range (manual slider, range
    finder) shouldn't let the operator dodge the dwell timer by wiggling
    the value -- only returning to exactly neutral resets it."""
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    guard.observe(1650, now=4.0)
    assert guard.observe(1620, now=8.0) is True


def test_remaining_s_none_at_neutral():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1500, now=0.0)
    assert guard.remaining_s(1500, now=0.0) is None


def test_remaining_s_none_before_first_observe():
    guard = DwellGuard(DEFAULT_CONFIG)
    assert guard.remaining_s(1600, now=0.0) is None


def test_remaining_s_counts_down():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    assert guard.remaining_s(1600, now=3.0) == pytest.approx(5.0)


def test_remaining_s_floors_at_zero():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    assert guard.remaining_s(1600, now=100.0) == 0.0


def test_reset_clears_tracking():
    guard = DwellGuard(DEFAULT_CONFIG)
    guard.observe(1600, now=0.0)
    guard.reset()
    assert guard.observe(1600, now=8.0) is False  # would have tripped without reset


def test_invalid_config_rejected():
    with pytest.raises(ValueError):
        BenchSafetyConfig(dwell_timeout_s=0)
    with pytest.raises(ValueError):
        BenchSafetyConfig(dwell_timeout_s=-1)


# --- SweepPlan -------------------------------------------------------------


def test_sweep_pulse_at_start_and_end():
    sweep = SweepPlan(min_us=1400, max_us=1600, duration_s=4.0)
    assert sweep.pulse_at(0.0) == 1400
    assert sweep.pulse_at(4.0) == 1600


def test_sweep_pulse_at_midpoint():
    sweep = SweepPlan(min_us=1400, max_us=1600, duration_s=4.0)
    assert sweep.pulse_at(2.0) == 1500


def test_sweep_pulse_at_clamps_before_start():
    sweep = SweepPlan(min_us=1400, max_us=1600, duration_s=4.0)
    assert sweep.pulse_at(-1.0) == 1400


def test_sweep_pulse_at_clamps_past_end():
    sweep = SweepPlan(min_us=1400, max_us=1600, duration_s=4.0)
    assert sweep.pulse_at(100.0) == 1600


def test_sweep_is_complete():
    sweep = SweepPlan(min_us=1400, max_us=1600, duration_s=4.0)
    assert sweep.is_complete(3.9) is False
    assert sweep.is_complete(4.0) is True
    assert sweep.is_complete(5.0) is True


@pytest.mark.parametrize(
    "kwargs",
    [
        {"min_us": 1600, "max_us": 1400, "duration_s": 4.0},
        {"min_us": 1500, "max_us": 1500, "duration_s": 4.0},
        {"min_us": 1400, "max_us": 1600, "duration_s": 0},
        {"min_us": 1400, "max_us": 1600, "duration_s": -1.0},
    ],
)
def test_invalid_sweep_plan_rejected(kwargs):
    with pytest.raises(ValueError):
        SweepPlan(**kwargs)


# --- MomentaryHold -----------------------------------------------------------


def test_momentary_hold_not_pending_until_started():
    hold = MomentaryHold(hold_s=2.0)
    assert hold.pending is False
    assert hold.observe(now=0.0) is False


def test_momentary_hold_observe_false_before_expiry():
    hold = MomentaryHold(hold_s=2.0)
    hold.start(now=0.0)
    assert hold.observe(now=1.9) is False
    assert hold.pending is True


def test_momentary_hold_observe_true_exactly_once_at_expiry():
    hold = MomentaryHold(hold_s=2.0)
    hold.start(now=0.0)
    assert hold.observe(now=2.0) is True
    assert hold.pending is False
    assert hold.observe(now=3.0) is False  # already cleared, not re-fired


def test_momentary_hold_cancel_clears_pending():
    hold = MomentaryHold(hold_s=2.0)
    hold.start(now=0.0)
    hold.cancel()
    assert hold.pending is False
    assert hold.observe(now=2.0) is False


def test_momentary_hold_remaining_s():
    hold = MomentaryHold(hold_s=2.0)
    assert hold.remaining_s(now=0.0) is None
    hold.start(now=0.0)
    assert hold.remaining_s(now=0.5) == pytest.approx(1.5)
    assert hold.remaining_s(now=10.0) == 0.0  # floors at zero, doesn't go negative


# --- RepeatabilityCheck ------------------------------------------------------


def test_repeatability_check_not_active_until_started():
    check = RepeatabilityCheck(step_hold_s=2.0)
    assert check.active is False
    assert check.current_target is None
    assert check.tick(now=100.0) is None


def test_repeatability_check_starts_at_neutral():
    check = RepeatabilityCheck(step_hold_s=2.0)
    check.start(now=0.0)
    assert check.active is True
    assert check.current_target == "neutral"


def test_repeatability_check_advances_through_full_sequence():
    check = RepeatabilityCheck(step_hold_s=2.0)
    check.start(now=0.0)
    assert check.tick(now=1.0) is None  # still holding at neutral
    assert check.tick(now=2.0) == "zero"
    assert check.current_target == "zero"
    assert check.tick(now=3.0) is None  # still holding at zero
    assert check.tick(now=4.0) == "neutral"
    assert check.tick(now=5.0) is None
    assert check.tick(now=6.0) == "done"
    assert check.active is False


def test_repeatability_check_cancel_stops_the_sequence():
    check = RepeatabilityCheck(step_hold_s=2.0)
    check.start(now=0.0)
    check.cancel()
    assert check.active is False
    assert check.tick(now=100.0) is None


# --- HoldCheck -----------------------------------------------------------


def test_hold_check_not_active_until_started():
    check = HoldCheck(hold_s=30.0)
    assert check.active is False
    assert check.tick(now=0.0) is False


def test_hold_check_tick_false_before_expiry():
    check = HoldCheck(hold_s=30.0)
    check.start(now=0.0)
    assert check.tick(now=29.9) is False
    assert check.active is True


def test_hold_check_tick_true_exactly_once_at_expiry():
    check = HoldCheck(hold_s=30.0)
    check.start(now=0.0)
    assert check.tick(now=30.0) is True
    assert check.active is False
    assert check.tick(now=31.0) is False  # already cleared


def test_hold_check_cancel_stops_the_wait():
    check = HoldCheck(hold_s=30.0)
    check.start(now=0.0)
    check.cancel()
    assert check.active is False
    assert check.tick(now=30.0) is False


def test_hold_check_remaining_s():
    check = HoldCheck(hold_s=30.0)
    assert check.remaining_s(now=0.0) is None
    check.start(now=0.0)
    assert check.remaining_s(now=10.0) == pytest.approx(20.0)
    assert check.remaining_s(now=1000.0) == 0.0
