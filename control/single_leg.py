"""Test Leg tab pure logic: the step-size safety rule and the local-frame
IK target used by Joint mode and IK mode respectively. No Qt imports --
same separation as control/bench.py and control/tracker.py.

Local frame, not a specific leg's real mount position (see
TEST_LEG.mount_angle_rad == 0): the operator is holding a bare leg
clamped to a table edge, no body, and IK mode's whole point is a ruler
check against the physical leg (docs/HOW_TO_USE.md). Foot X/Y/Z in this
leg's own local frame is directly ruler-measurable from the coxa's own
mounting point (X = forward along the coxa's own zero direction, Y =
lateral, Z = down) without the operator doing body-frame arithmetic in
their head. This costs nothing in sign-error-catching power: the mount-
angle rotation itself is a pure coordinate transform already proven
correct for every leg (ANALYSIS.md Section 3), so what this tool
actually verifies -- does the physical leg's joints move the way FK/IK's
own leg-local geometry says they should -- doesn't depend on which mount
angle is assumed.
"""

from robot.kinematics import Leg

TEST_LEG = Leg(name="TEST", origin_x_mm=0.0, origin_y_mm=0.0)

# Suggested bring-up order and why -- matches docs/HOW_TO_USE.md's horn-
# mounting order exactly, same underlying reasoning (each step's check
# depends on nothing not yet fixed).
EXPLORATION_ORDER = (
    ("coxa", "doesn't depend on any other joint's position"),
    ("tibia", "checked relative to the femur segment's own line, not the femur's final angle"),
    ("femur", "coxa and tibia are already fixed by this point"),
)


def is_step_allowed(
    current_deg: float,
    step_deg: float,
    direction: int,
    marked_min_deg: float | None,
    marked_max_deg: float | None,
    smallest_step_deg: float,
) -> bool:
    """Whether one specific step button should be enabled right now --
    what ui/test_leg_tab.py actually calls, once per button per tick.
    Same rule as max_allowed_step_deg, phrased as a per-button check
    instead of "which is the largest allowed"."""
    if step_deg <= smallest_step_deg:
        return True
    target = current_deg + direction * step_deg
    marked_bound = marked_max_deg if direction > 0 else marked_min_deg
    if marked_bound is None:
        return False
    return target <= marked_bound if direction > 0 else target >= marked_bound
