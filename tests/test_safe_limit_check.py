"""Synthetic mechanical-limit data throughout -- nothing is assembled yet
(CLAUDE.md), so there is no real Test Leg measurement to check against.
These numbers are illustrative only, not a prediction of what the real
test leg will measure; they exist to prove check_fit()/check_all() catch
both a genuine pass and a genuine failure, correctly attributed to the
right joint and quantified by how much.
"""

import pytest

from robot.gait import GAIT_ENVELOPE_COXA_MAX_DEG, GAIT_ENVELOPE_COXA_MIN_DEG
from robot.kinematics import SAFE_LIMIT_MARGIN_DEG
from robot.safe_limit_check import check_all, check_fit, joint_for_servo


def test_joint_for_servo_matches_coxa_femur_tibia_ordering():
    assert joint_for_servo(0) == "coxa"
    assert joint_for_servo(1) == "femur"
    assert joint_for_servo(2) == "tibia"
    assert joint_for_servo(3) == "coxa"  # next leg, same ordering
    assert joint_for_servo(17) == "tibia"  # last servo, leg 5


def test_generous_mechanical_limit_fits_with_spare_margin():
    # servo_index=0 is a coxa; GAIT_ENVELOPE_COXA_*_DEG is roughly +-19.6.
    result = check_fit(0, min_deg_from_neutral=-60.0, max_deg_from_neutral=60.0)
    assert result.fits is True
    assert result.low_spare_deg > 0.0
    assert result.high_spare_deg > 0.0


def test_tight_mechanical_limit_fails_on_both_ends_by_the_right_amount():
    result = check_fit(0, min_deg_from_neutral=-20.0, max_deg_from_neutral=20.0)
    assert result.fits is False
    assert result.joint == "coxa"
    # safe_min = -20 + 5 = -15; low_spare = env_min - safe_min
    expected_low_spare = GAIT_ENVELOPE_COXA_MIN_DEG - (-20.0 + SAFE_LIMIT_MARGIN_DEG)
    expected_high_spare = (20.0 - SAFE_LIMIT_MARGIN_DEG) - GAIT_ENVELOPE_COXA_MAX_DEG
    assert result.low_spare_deg == pytest.approx(expected_low_spare)
    assert result.high_spare_deg == pytest.approx(expected_high_spare)
    assert result.low_spare_deg < 0.0
    assert result.high_spare_deg < 0.0


def test_fails_on_only_the_end_that_is_actually_tight():
    # Generous on the low end, tight on the high end -- fits must be
    # False, but low_spare_deg alone must not implicate the low end.
    result = check_fit(0, min_deg_from_neutral=-60.0, max_deg_from_neutral=22.0)
    assert result.fits is False
    assert result.low_spare_deg > 0.0
    assert result.high_spare_deg < 0.0


def test_unmeasured_bound_is_unknown_not_a_pass():
    result = check_fit(0, min_deg_from_neutral=None, max_deg_from_neutral=None)
    assert result.fits is False
    assert result.low_spare_deg is None
    assert result.high_spare_deg is None


def test_check_all_reports_every_servo_with_its_own_joint():
    class FakeProfile:
        def __init__(self, min_deg_from_neutral, max_deg_from_neutral):
            self.min_deg_from_neutral = min_deg_from_neutral
            self.max_deg_from_neutral = max_deg_from_neutral

    # Generous limits for every coxa/femur (bipolar, valid range roughly
    # [-90, 90]) except one tibia (index 5, leg 1's tibia -- 5 % 3 == 2),
    # which needs its own realistic range (tibia is zero-based, degFrom
    # Neutral in [0, 180] -- see transport.protocol.pulse_us_to_deg_from_
    # neutral) and is deliberately set too tight there to fit.
    profiles = [FakeProfile(-90.0, 90.0) for _ in range(18)]
    for i in range(2, 18, 3):  # every tibia gets a valid, generous range
        profiles[i] = FakeProfile(20.0, 160.0)
    profiles[5] = FakeProfile(50.0, 55.0)  # too tight to fit

    results = check_all(profiles)
    assert len(results) == 18
    assert all(r.fits for i, r in enumerate(results) if i != 5)
    assert results[5].fits is False
    assert results[5].joint == "tibia"
    assert results[5].servo_index == 5
