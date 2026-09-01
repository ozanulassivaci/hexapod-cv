import pytest

from control.single_leg import EXPLORATION_ORDER, TEST_LEG, is_step_allowed


def test_smallest_step_always_allowed_even_unmarked():
    assert is_step_allowed(0.0, 1.0, 1, None, None, smallest_step_deg=1.0) is True
    assert is_step_allowed(0.0, 1.0, -1, None, None, smallest_step_deg=1.0) is True


def test_larger_step_disallowed_when_direction_unmarked():
    assert is_step_allowed(0.0, 5.0, 1, marked_min_deg=None, marked_max_deg=None, smallest_step_deg=1.0) is False


def test_larger_step_allowed_within_marked_range():
    # Marked max at +40, currently at +10 -- a +5 step lands at +15, well inside.
    assert is_step_allowed(10.0, 5.0, 1, marked_min_deg=None, marked_max_deg=40.0, smallest_step_deg=1.0) is True


def test_larger_step_disallowed_past_marked_range():
    # Marked max at +12, currently at +10 -- a +5 step would land at +15, past it.
    assert is_step_allowed(10.0, 5.0, 1, marked_min_deg=None, marked_max_deg=12.0, smallest_step_deg=1.0) is False


def test_larger_step_allowed_exactly_at_the_marked_boundary():
    # Landing exactly on the mark is still "within" -- inclusive, matches
    # ServoProfile's own >= / <= comparisons.
    assert is_step_allowed(10.0, 5.0, 1, marked_min_deg=None, marked_max_deg=15.0, smallest_step_deg=1.0) is True


def test_negative_direction_checks_against_marked_min():
    assert is_step_allowed(-10.0, 5.0, -1, marked_min_deg=-40.0, marked_max_deg=None, smallest_step_deg=1.0) is True
    assert is_step_allowed(-10.0, 5.0, -1, marked_min_deg=-12.0, marked_max_deg=None, smallest_step_deg=1.0) is False
    # Marked max is irrelevant to a negative-direction step.
    assert is_step_allowed(-10.0, 5.0, -1, marked_min_deg=None, marked_max_deg=999.0, smallest_step_deg=1.0) is False


def test_marked_bound_on_the_other_side_does_not_help():
    # Marked min doesn't unlock a positive-direction step, and vice versa.
    assert is_step_allowed(0.0, 5.0, 1, marked_min_deg=-90.0, marked_max_deg=None, smallest_step_deg=1.0) is False
    assert is_step_allowed(0.0, 5.0, -1, marked_min_deg=None, marked_max_deg=90.0, smallest_step_deg=1.0) is False


def test_test_leg_is_local_frame_at_the_origin():
    assert TEST_LEG.origin_x_mm == 0.0
    assert TEST_LEG.origin_y_mm == 0.0
    assert TEST_LEG.mount_angle_rad == 0.0


def test_exploration_order_is_coxa_tibia_femur():
    assert [joint for joint, _reason in EXPLORATION_ORDER] == ["coxa", "tibia", "femur"]
    assert all(reason for _joint, reason in EXPLORATION_ORDER)
