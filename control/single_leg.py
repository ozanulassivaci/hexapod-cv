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
    what ui/single_leg_tab.py actually calls, once per button per tick.
    The smallest step is always allowed (creeping forward one unverified
    degree at a time is exactly how a limit gets found and marked in the
    first place); a larger step is only offered if landing there would
    stay within the marked-safe range on that side."""
    if step_deg <= smallest_step_deg:
        return True
    target = current_deg + direction * step_deg
    marked_bound = marked_max_deg if direction > 0 else marked_min_deg
    if marked_bound is None:
        return False
    return target <= marked_bound if direction > 0 else target >= marked_bound


def safety_zone_color(
    current_deg: float,
    marked_min_deg: float | None,
    marked_max_deg: float | None,
    margin_deg: float,
) -> str:
    """"green"/"amber"/"red"/"unknown" -- how close current_deg sits to
    this joint's *safe* limit (the marked mechanical bound shrunk inward
    by margin_deg on each end, the same shrinking
    GatedServoDriver.h's LimitMode::Safe applies at runtime), for
    ui/single_leg_view.py's stick-figure coloring.

    "unknown" is a real, honest fourth state, not a default-safe green:
    a joint with nothing marked in either direction has no assessable
    safe limit yet, and showing green there would be indistinguishable
    from "verified and comfortably clear" -- exactly the silent gap this
    whole visualization exists to make visible instead.

    Thresholds are margin_deg itself, not an arbitrary separate number:
    red once within one margin's worth of the safe edge (i.e. as close
    as the margin itself would already put you outside the *mechanical*
    edge were it this tight), amber within three times that, green
    beyond -- ties the visualization directly to the same constant
    that's actually enforced, rather than a second, disconnected
    threshold someone has to remember to keep in sync.
    """
    margins = []
    if marked_min_deg is not None:
        margins.append(current_deg - (marked_min_deg + margin_deg))
    if marked_max_deg is not None:
        margins.append((marked_max_deg - margin_deg) - current_deg)
    if not margins:
        return "unknown"
    worst = min(margins)
    if worst <= margin_deg:
        return "red"
    if worst <= 3 * margin_deg:
        return "amber"
    return "green"
