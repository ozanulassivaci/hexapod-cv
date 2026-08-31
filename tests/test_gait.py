import json
import math
import pathlib

import pytest

from robot.gait import (
    BODY_HEIGHT_Z_CROUCHED_MM,
    BODY_HEIGHT_Z_TALL_MM,
    MAX_SLEW_DEG_PER_S,
    GaitState,
    home_z_mm,
    is_in_stance,
    step,
)
from robot.kinematics import LEGS

_FIXTURE_PATH = pathlib.Path(__file__).parent / "fixtures" / "gait_golden.json"


def _load_fixture() -> dict:
    with open(_FIXTURE_PATH) as f:
        return json.load(f)


# --- Golden fixture cross-check (Python vs. C++, see firmware/test/test_gait) --


@pytest.mark.parametrize("scenario", _load_fixture()["scenarios"], ids=lambda s: s["name"])
def test_golden_gait_scenarios(scenario):
    cmd = scenario["command"]
    state = GaitState.initial(body_height=cmd["body_height"])
    ticks_by_number = {t["tick"]: t for t in scenario["ticks"]}
    max_tick = max(ticks_by_number)

    for tick in range(1, max_tick + 1):
        state = step(
            state,
            scenario["dt_s"],
            vx=cmd["vx"],
            vy=cmd["vy"],
            speed=cmd["speed"],
            rotation=cmd["rotation"],
            body_height=cmd["body_height"],
        )
        expected = ticks_by_number.get(tick)
        if expected is None:
            continue
        assert state.phase == pytest.approx(expected["phase"], abs=1e-9)
        for actual_leg, expected_leg in zip(state.leg_angles, expected["leg_angles"]):
            assert actual_leg.coxa_deg == pytest.approx(expected_leg["coxa_deg"], abs=1e-6)
            assert actual_leg.femur_deg == pytest.approx(expected_leg["femur_deg"], abs=1e-6)
            assert actual_leg.tibia_deg == pytest.approx(expected_leg["tibia_deg"], abs=1e-6)


# --- Phase properties -------------------------------------------------------


def test_phase_wraps_into_unit_interval_over_long_run():
    state = GaitState.initial()
    for _ in range(5000):
        state = step(state, 0.02, vx=1.0, vy=0.0, speed=80.0, rotation=0.0, body_height=50.0)
        assert 0.0 <= state.phase < 1.0


def test_phase_frozen_when_speed_zero():
    state = GaitState.initial()
    state = step(state, 0.02, vx=1.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)
    assert state.phase == 0.0


def test_idle_holds_exactly_at_home_no_drift():
    """direction/rotation all zero (or speed zero) must return the foot
    exactly to home, not leave residual lift from wherever phase was --
    the same idle short-circuit get_foot_target uses in the reference."""
    state = GaitState.initial(body_height=50.0)
    # Walk a while so phase lands mid-swing for at least one leg, then go idle.
    for _ in range(17):
        state = step(state, 0.02, vx=1.0, vy=0.0, speed=80.0, rotation=0.0, body_height=50.0)
    for _ in range(200):  # long enough for the slew limit to fully converge
        state = step(state, 0.02, vx=0.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)

    from robot.gait import home_position

    for leg, foot in zip(LEGS, state.foot_positions()):
        home = home_position(leg, 50.0)
        assert foot.x == pytest.approx(home.x, abs=1e-2)
        assert foot.y == pytest.approx(home.y, abs=1e-2)
        assert foot.z == pytest.approx(home.z, abs=1e-2)


# --- Tripod grouping ---------------------------------------------------------


def test_tripod_grouping_always_three_and_three():
    for phase_hundredths in range(0, 100):
        phase = phase_hundredths / 100.0
        stances = [is_in_stance(i, phase) for i in range(6)]
        assert stances.count(True) == 3
        assert stances.count(False) == 3


def test_tripod_grouping_matches_leg_index_parity():
    """ANALYSIS.md Section 4: leg_index % 2 is the correct tripod
    grouping -- legs of the same parity are always in the same half of
    the cycle."""
    for phase_hundredths in range(0, 100):
        phase = phase_hundredths / 100.0
        even_stances = {is_in_stance(i, phase) for i in range(0, 6, 2)}
        odd_stances = {is_in_stance(i, phase) for i in range(1, 6, 2)}
        assert len(even_stances) == 1
        assert len(odd_stances) == 1
        assert even_stances != odd_stances


# --- Slew-rate bound: a real limit, not a typical-case property ------------


def test_slew_rate_bound_holds_for_worst_case_jump():
    """Construct the worst case -- a full mode-transition jump in one
    tick (idle standing tall, then immediately commanded to walk at max
    speed while also dropping body height) -- and assert no joint moves
    more than MAX_SLEW_DEG_PER_S * dt_s in that single tick. This is the
    actual fix for ANALYSIS.md safety gap #3 ("position smoothing is not
    a velocity limit"): a real bound, checked against an adversarial
    input, not sampled from ordinary gait motion."""
    dt_s = 0.02
    max_delta = MAX_SLEW_DEG_PER_S * dt_s + 1e-9

    before = GaitState.initial(body_height=100.0)
    after = step(before, dt_s, vx=1.0, vy=1.0, speed=100.0, rotation=1.0, body_height=0.0)

    for angles_before, angles_after in zip(before.leg_angles, after.leg_angles):
        assert abs(angles_after.coxa_deg - angles_before.coxa_deg) <= max_delta
        assert abs(angles_after.femur_deg - angles_before.femur_deg) <= max_delta
        assert abs(angles_after.tibia_deg - angles_before.tibia_deg) <= max_delta


def test_slew_rate_bound_holds_across_many_random_ticks():
    import random

    rng = random.Random(5)
    state = GaitState.initial(body_height=50.0)
    dt_s = 0.02
    max_delta = MAX_SLEW_DEG_PER_S * dt_s + 1e-9

    for _ in range(500):
        before = state
        state = step(
            state,
            dt_s,
            vx=rng.uniform(-1, 1),
            vy=rng.uniform(-1, 1),
            speed=rng.uniform(0, 100),
            rotation=rng.uniform(-1, 1),
            body_height=rng.uniform(0, 100),
        )
        for a_before, a_after in zip(before.leg_angles, state.leg_angles):
            assert abs(a_after.coxa_deg - a_before.coxa_deg) <= max_delta
            assert abs(a_after.femur_deg - a_before.femur_deg) <= max_delta
            assert abs(a_after.tibia_deg - a_before.tibia_deg) <= max_delta


# --- Body height mapping -----------------------------------------------------


def test_body_height_mapping_endpoints():
    assert home_z_mm(0.0) == pytest.approx(BODY_HEIGHT_Z_CROUCHED_MM)
    assert home_z_mm(100.0) == pytest.approx(BODY_HEIGHT_Z_TALL_MM)


def test_body_height_mapping_monotonic():
    prev = home_z_mm(0.0)
    for h in range(1, 101):
        current = home_z_mm(float(h))
        assert current < prev  # taller = more negative z, strictly, given CROUCHED > TALL
        prev = current


# --- Clip stats (ANALYSIS.md Section 5.7: was silent) ----------------------


def test_clip_stats_stay_zero_across_the_verified_safe_envelope():
    """The commandable envelope this module's own body-height/step-height
    constants were chosen against (see their derivation comments above)
    has zero D-clamp and zero joint-clamp events by construction -- if a
    future constant change reintroduces clipping during ordinary gait,
    this is the regression guard that catches it."""
    state = GaitState.initial(body_height=50.0)
    for i in range(200):
        vx = math.cos(i * 0.31)
        vy = math.sin(i * 0.17)
        speed = 50.0 + 50.0 * math.sin(i * 0.05)
        rotation = math.sin(i * 0.13)
        body_height = 50.0 + 50.0 * math.sin(i * 0.02)
        state = step(state, 0.02, vx=vx, vy=vy, speed=speed, rotation=rotation, body_height=body_height)
    assert state.ik_clip_count == 0
    assert state.joint_clip_count == 0
    assert state.ik_clip_worst_mm == 0.0
    assert state.joint_clip_worst_deg == 0.0
    assert state.clipped_this_tick is False


def test_clip_counters_increment_and_persist_across_ticks(monkeypatch):
    """foot_target()'s own bounded inputs (speed 0-100, direction/
    rotation magnitude <= 1, body_height 0-100) never produce an
    unreachable target -- that's exactly what the test above pins.
    Monkeypatching STEP_LENGTH_MM to something absurd, only for this
    test, is the one way to force a target past D_max through step()'s
    real public path instead of calling kinematics internals directly.
    Counters must accumulate (not reset) across ticks, and
    clipped_this_tick must reflect only the tick that just ran."""
    import robot.gait as gaitmod

    monkeypatch.setattr(gaitmod, "STEP_LENGTH_MM", 1000.0)

    state = GaitState.initial(body_height=50.0)
    assert state.ik_clip_count == 0

    state = step(state, 0.02, vx=0.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)
    # idle (speed=0) short-circuits to home, unaffected by STEP_LENGTH_MM
    # -- confirms a normal tick doesn't spuriously count anything before
    # the adversarial one below.
    assert state.ik_clip_count == 0
    assert state.clipped_this_tick is False

    extreme = step(state, 0.02, vx=1.0, vy=0.0, speed=100.0, rotation=0.0, body_height=50.0)
    assert extreme.ik_clip_count > 0
    assert extreme.ik_clip_worst_mm > 0.0
    assert extreme.clipped_this_tick is True

    again = step(extreme, 0.02, vx=0.0, vy=0.0, speed=0.0, rotation=0.0, body_height=50.0)
    # cumulative count never decreases even once the extreme input stops
    assert again.ik_clip_count == extreme.ik_clip_count
    assert again.ik_clip_worst_mm == extreme.ik_clip_worst_mm
    # but clipped_this_tick is level-triggered on the tick that just ran
    assert again.clipped_this_tick is False


def test_body_height_reachability_not_silently_clamped_at_extremes():
    """The configured height range must stay inside the IK-reachable
    annulus at every leg's home (x, y) -- if IK's D clamp silently
    engaged here, the configured range itself would be partly
    unreachable, which is a design bug worth catching, not shipping."""
    from robot.kinematics import FEMUR_LENGTH_MM, TIBIA_LENGTH_MM
    from robot.gait import home_position
    from robot.kinematics import inverse_kinematics

    d_min = abs(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM)
    d_max = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM

    for leg in LEGS:
        for height in (0.0, 50.0, 100.0):
            target = home_position(leg, height)
            local_x = target.x - leg.origin_x_mm
            local_y = target.y - leg.origin_y_mm
            l_xy = math.hypot(local_x, local_y)
            l_forward = l_xy - 38.0  # COXA_LENGTH_MM
            d = math.hypot(l_forward, target.z)
            assert d_min < d < d_max, f"{leg.name} height={height} D={d} outside ({d_min}, {d_max})"
            # cross-check: IK on this exact target shouldn't need its clamp either
            angles = inverse_kinematics(target, leg)
            assert math.isfinite(angles.femur_deg)
