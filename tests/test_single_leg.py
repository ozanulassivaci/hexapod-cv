import pytest

from control.single_leg import EXPLORATION_ORDER, TEST_LEG, is_step_allowed, safety_zone_color


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


# --- safety_zone_color -------------------------------------------------


def test_safety_zone_unknown_with_nothing_marked():
    assert safety_zone_color(0.0, None, None, margin_deg=5.0) == "unknown"


def test_safety_zone_green_comfortably_inside():
    # Marked range [-40, 40], margin 5 -> safe range [-35, 35]. At 0,
    # worst-case margin is 35 on both sides, well past the 3*margin=15
    # green threshold.
    assert safety_zone_color(0.0, -40.0, 40.0, margin_deg=5.0) == "green"


def test_safety_zone_amber_within_three_times_margin():
    # safe_max = 40 - 5 = 35; at current=25, margin to safe edge = 10,
    # which is <= 3*5=15 (amber) but > 5 (not red).
    assert safety_zone_color(25.0, None, 40.0, margin_deg=5.0) == "amber"


def test_safety_zone_red_within_one_margin():
    # safe_max = 40 - 5 = 35; at current=33, margin to safe edge = 2 <= 5.
    assert safety_zone_color(33.0, None, 40.0, margin_deg=5.0) == "red"


def test_safety_zone_red_when_already_past_the_safe_edge():
    # Past the safe edge entirely -- still red, not some worse/different
    # state, since there isn't one to distinguish it with here.
    assert safety_zone_color(38.0, None, 40.0, margin_deg=5.0) == "red"


def test_safety_zone_uses_the_worse_of_both_sides():
    # Comfortable on the max side, tight on the min side -- overall
    # verdict must reflect the worse (tighter) side, not the better one.
    assert safety_zone_color(-33.0, -40.0, 999.0, margin_deg=5.0) == "red"


def test_safety_zone_boundary_values_are_inclusive():
    # Exactly one margin away from the safe edge is still red (<=, not <).
    assert safety_zone_color(30.0, None, 40.0, margin_deg=5.0) == "red"  # safe_max=35, margin exactly 5
    # Exactly three margins away is still amber (<=, not <).
    assert safety_zone_color(20.0, None, 40.0, margin_deg=5.0) == "amber"  # safe_max=35, margin exactly 15
