import json
import math
import pathlib
import random

import pytest

from robot.kinematics import (
    COXA_MAX_DEG,
    COXA_MIN_DEG,
    FEMUR_LENGTH_MM,
    FEMUR_MAX_DEG,
    FEMUR_MIN_DEG,
    LEGS,
    TIBIA_LENGTH_MM,
    TIBIA_MAX_DEG,
    TIBIA_MIN_DEG,
    JointAngles,
    Point3,
    clamp_joint_angles,
    forward_kinematics,
    inverse_kinematics,
    to_servo_deg,
)

_FIXTURE_PATH = pathlib.Path(__file__).parent / "fixtures" / "kinematics_golden.json"
_EPS = 1e-6
_D_MIN = abs(FEMUR_LENGTH_MM - TIBIA_LENGTH_MM)
_D_MAX = FEMUR_LENGTH_MM + TIBIA_LENGTH_MM


def _random_reachable_target(leg, rng: random.Random) -> Point3:
    """A target guaranteed inside the reachable annulus (D strictly
    between the femur/tibia bounds) with a physically constructible
    l_xy >= 0, so inverse_kinematics never engages its D clamp -- needed
    for exact FK(IK(target)) == target checks. l_forward = l_xy -
    COXA_LENGTH_MM can legitimately go negative (target pulled behind the
    coxa joint), but l_xy itself is a distance and cannot -- combinations
    that would require l_xy < 0 are resampled, not clamped, since they
    don't correspond to any real point at all."""
    while True:
        d = rng.uniform(_D_MIN + 1.0, _D_MAX - 1.0)
        theta = rng.uniform(0.0, 2 * math.pi)
        l_forward = d * math.cos(theta)
        z = d * math.sin(theta)
        l_xy = l_forward + 38.0  # + COXA_LENGTH_MM, kept a plain literal to not import it here
        if l_xy >= 0.0:
            break
    local_angle = rng.uniform(-math.pi, math.pi)
    local_x = l_xy * math.cos(local_angle)
    local_y = l_xy * math.sin(local_angle)
    c, s = math.cos(leg.mount_angle_rad), math.sin(leg.mount_angle_rad)
    rel_x = local_x * c - local_y * s
    rel_y = local_x * s + local_y * c
    return Point3(leg.origin_x_mm + rel_x, leg.origin_y_mm + rel_y, z)


# --- FK/IK round trips ---------------------------------------------------


def test_fk_of_ik_recovers_reachable_target():
    rng = random.Random(1)
    for leg in LEGS:
        for _ in range(200):
            target = _random_reachable_target(leg, rng)
            angles = inverse_kinematics(target, leg)
            recovered = forward_kinematics(angles, leg)
            assert recovered.x == pytest.approx(target.x, abs=1e-3)
            assert recovered.y == pytest.approx(target.y, abs=1e-3)
            assert recovered.z == pytest.approx(target.z, abs=1e-3)


def test_ik_is_idempotent_through_fk():
    """IK(FK(IK(target))) == IK(target) -- the inverse-direction property,
    phrased to avoid the two-solution (elbow up/down) ambiguity of
    sampling raw angles directly: IK always returns the same acos branch,
    so re-running IK on its own FK output must reproduce its own prior
    output exactly (ANALYSIS.md Section 2's "only one branch is ever
    computed" -- this is what would break if a future change picked the
    other branch inconsistently)."""
    rng = random.Random(2)
    for leg in LEGS:
        for _ in range(200):
            target = _random_reachable_target(leg, rng)
            angles_1 = inverse_kinematics(target, leg)
            foot = forward_kinematics(angles_1, leg)
            angles_2 = inverse_kinematics(foot, leg)
            assert angles_2.coxa_deg == pytest.approx(angles_1.coxa_deg, abs=1e-3)
            assert angles_2.femur_deg == pytest.approx(angles_1.femur_deg, abs=1e-3)
            assert angles_2.tibia_deg == pytest.approx(angles_1.tibia_deg, abs=1e-3)


def test_unreachable_target_clamps_predictably_not_garbage():
    leg = LEGS[0]
    far = Point3(x=leg.origin_x_mm + 1000.0, y=leg.origin_y_mm, z=0.0)
    angles = inverse_kinematics(far, leg)
    assert all(math.isfinite(v) for v in (angles.coxa_deg, angles.femur_deg, angles.tibia_deg))
    clamped = clamp_joint_angles(angles)
    assert COXA_MIN_DEG <= clamped.coxa_deg <= COXA_MAX_DEG
    assert FEMUR_MIN_DEG <= clamped.femur_deg <= FEMUR_MAX_DEG
    assert TIBIA_MIN_DEG <= clamped.tibia_deg <= TIBIA_MAX_DEG


# --- Adversarial sweep: no NaN, no domain errors, ever --------------------


def test_no_nan_or_exception_across_adversarial_sweep():
    rng = random.Random(3)
    adversarial = [
        Point3(0.0, 0.0, 0.0),  # at the world origin
    ]
    for leg in LEGS:
        adversarial.append(Point3(leg.origin_x_mm, leg.origin_y_mm, 0.0))  # l_xy == 0 exactly
        adversarial.append(Point3(leg.origin_x_mm, leg.origin_y_mm, 10000.0))  # absurd height
    for _ in range(500):
        adversarial.append(
            Point3(rng.uniform(-2000, 2000), rng.uniform(-2000, 2000), rng.uniform(-2000, 2000))
        )

    for leg in LEGS:
        for target in adversarial:
            angles = inverse_kinematics(target, leg)
            assert math.isfinite(angles.coxa_deg)
            assert math.isfinite(angles.femur_deg)
            assert math.isfinite(angles.tibia_deg)


def test_negative_l_forward_femur_angle_is_clamped():
    """ANALYSIS.md Section 5.1: beta1 = atan2(z, l_forward) can swing
    toward +-90-180 degrees when l_forward goes negative (target pulled
    behind the coxa joint) -- must come out clamped, not passed through
    raw."""
    leg = LEGS[0]
    behind = Point3(x=leg.origin_x_mm, y=leg.origin_y_mm, z=-5.0)  # l_xy ~= 0 => l_forward ~= -COXA_LENGTH
    angles = clamp_joint_angles(inverse_kinematics(behind, leg))
    assert FEMUR_MIN_DEG <= angles.femur_deg <= FEMUR_MAX_DEG


# --- Invariants ------------------------------------------------------------


def test_tibia_raw_gamma_never_positive():
    """ANALYSIS.md Section 2's acos-range proof, checked live: gamma is
    provably <= 0 for the clamped D used here, for any target. If this
    ever fails, the -gamma servo-write convention in to_servo_deg() is
    silently wrong for whatever changed."""
    rng = random.Random(4)
    for leg in LEGS:
        for _ in range(300):
            target = Point3(rng.uniform(-500, 500), rng.uniform(-500, 500), rng.uniform(-500, 500))
            angles = inverse_kinematics(target, leg)
            assert angles.tibia_deg <= _EPS


def test_rotational_sense_symmetry_across_all_legs():
    """ANALYSIS.md Section 3: increasing the coxa command rotates the
    foot the same absolute (body-frame) rotational sense on every leg,
    with no per-leg sign flip -- d(alpha_world)/d(coxa_angle) == 1
    identically for all six legs. Guards against a future accidental
    per-leg sign flip regression."""
    delta_deg = 1.0
    for leg in LEGS:
        alpha_a = math.radians(0.0) + leg.mount_angle_rad
        alpha_b = math.radians(delta_deg) + leg.mount_angle_rad
        d_alpha_world_d_coxa = (alpha_b - alpha_a) / math.radians(delta_deg)
        assert d_alpha_world_d_coxa == pytest.approx(1.0)


def test_clamp_joint_angles_bounds_hold_for_extreme_input():
    extreme = JointAngles(coxa_deg=9999.0, femur_deg=-9999.0, tibia_deg=9999.0)
    clamped = clamp_joint_angles(extreme)
    assert clamped.coxa_deg == COXA_MAX_DEG
    assert clamped.femur_deg == FEMUR_MIN_DEG
    assert clamped.tibia_deg == TIBIA_MAX_DEG  # tibia's max is 0 (raw gamma convention)


def test_to_servo_deg_convention():
    angles = JointAngles(coxa_deg=10.0, femur_deg=-5.0, tibia_deg=-30.0)
    coxa_servo, femur_servo, tibia_servo = to_servo_deg(angles)
    assert coxa_servo == pytest.approx(100.0)
    assert femur_servo == pytest.approx(85.0)
    assert tibia_servo == pytest.approx(30.0)  # -gamma, not fabs(gamma) -- same result, explicit


# --- Golden fixture cross-check (Python vs. C++, see firmware/test/test_kinematics) --


def _load_fixture() -> dict:
    with open(_FIXTURE_PATH) as f:
        return json.load(f)


def _leg_by_name(name: str):
    return next(leg for leg in LEGS if leg.name == name)


@pytest.mark.parametrize("case", _load_fixture()["fk_cases"], ids=lambda c: c["case"])
def test_golden_fk_cases(case):
    leg = _leg_by_name(case["leg"])
    angles = JointAngles(coxa_deg=case["coxa_deg"], femur_deg=case["femur_deg"], tibia_deg=case["tibia_deg"])
    foot = forward_kinematics(angles, leg)
    assert foot.x == pytest.approx(case["expected_foot"]["x"], abs=1e-6)
    assert foot.y == pytest.approx(case["expected_foot"]["y"], abs=1e-6)
    assert foot.z == pytest.approx(case["expected_foot"]["z"], abs=1e-6)


@pytest.mark.parametrize("case", _load_fixture()["ik_cases"], ids=lambda c: c["case"])
def test_golden_ik_cases(case):
    leg = _leg_by_name(case["leg"])
    target = Point3(**case["target"])
    angles = inverse_kinematics(target, leg)
    assert angles.coxa_deg == pytest.approx(case["expected_coxa_deg"], abs=1e-6)
    assert angles.femur_deg == pytest.approx(case["expected_femur_deg"], abs=1e-6)
    assert angles.tibia_deg == pytest.approx(case["expected_tibia_deg"], abs=1e-6)


def test_fixture_leg_origins_match_module_constants():
    fixture = _load_fixture()
    for leg in LEGS:
        origin = fixture["leg_origins"][leg.name]
        assert origin["x"] == pytest.approx(leg.origin_x_mm)
        assert origin["y"] == pytest.approx(leg.origin_y_mm)
