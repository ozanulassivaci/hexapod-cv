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


def deg_from_neutral(joint: str, raw_deg: float) -> float:
    """Raw kinematic degrees (robot/kinematics.py's JointAngles.*_deg
    convention) -> degrees from this joint's own servo neutral
    (to_servo_deg()'s convention) -- the frame
    transport.protocol.ServoProfile.min/max_deg_from_neutral and
    robot/gait.py's GAIT_ENVELOPE_*_DEG constants are already stored in
    (robot/gait.py's own comment above those constants spells out why).
    Identity for coxa/femur -- their bipolar neutral (servo 90) sits at
    raw 0, so the two frames coincide and nothing needs converting.
    Negates for tibia -- its neutral is zero-based (servo 0, not 90;
    ANALYSIS.md Section 2), and raw tibia_deg is provably <= 0 (same
    section's acos-range proof), so the neutral-relative value is
    always >= 0: the negation, not the identity.

    An involution: negation is its own inverse, so this same function
    also converts the other way (deg-from-neutral -> raw) for a single
    value. It does NOT handle a (min, max) pair -- negating swaps which
    one is smaller for tibia, see raw_bounds_from_neutral() below."""
    return -raw_deg if joint == "tibia" else raw_deg


def raw_bounds_from_neutral(
    joint: str, min_deg_from_neutral: float | None, max_deg_from_neutral: float | None
) -> tuple[float | None, float | None]:
    """(min, max) in deg-from-neutral frame -> (min, max) in raw frame,
    e.g. transport.protocol.ServoProfile's stored bounds or
    robot/gait.py's GAIT_ENVELOPE_TIBIA_MIN/MAX_DEG, converted for use
    against a raw-frame current angle or a raw-frame drawing convention
    (ui/single_leg_view.py's stick figure). Identity for coxa/femur. For
    tibia, deg_from_neutral()'s per-value negation additionally swaps
    which bound is the min and which is the max -- negating an ordered
    pair reverses its order."""
    lo = deg_from_neutral(joint, min_deg_from_neutral) if min_deg_from_neutral is not None else None
    hi = deg_from_neutral(joint, max_deg_from_neutral) if max_deg_from_neutral is not None else None
    if joint == "tibia":
        lo, hi = hi, lo
    return lo, hi


def is_step_allowed(
    current_deg: float,
    step_deg: float,
    direction: int,
    marked_min_deg: float | None,
    marked_max_deg: float | None,
    smallest_step_deg: float,
    envelope_min_deg: float | None = None,
    envelope_max_deg: float | None = None,
) -> bool:
    """Whether one specific step button should be enabled right now --
    what ui/single_leg_tab.py actually calls, once per button per tick.
    The smallest step is always allowed (creeping forward one unverified
    degree at a time is exactly how a limit gets found and marked in the
    first place); a larger step is offered if landing there would stay
    within *either* the marked-safe range on that side, *or* the gait
    envelope on that side.

    All five angle-bearing arguments must already be in the same frame
    (deg-from-neutral, for any real caller -- see deg_from_neutral()
    above; the direction argument transforms the same way, since it's
    just a +-1 fed through the same linear map).

    Why the envelope counts as pre-verified, not unknown, territory:
    it's the exact range gait will command in normal operation (robot/
    gait.py's own derivation), so refusing full-size steps inside it
    buys no safety -- it only makes characterising a joint take ~300
    single-degree clicks per leg instead of a few button presses,
    which in practice means rushing past the checks this rule exists
    to slow down for. Nothing beyond the envelope gets this exemption:
    that's genuine discovery territory, and stays creep-only until a
    mark extends it.

    This also produces a graduated, not abrupt, falloff at the
    envelope's edge for free, with no separate easing formula needed:
    each button size is checked independently against where *its own*
    target would land, so approaching an edge disables the buttons that
    would overshoot it one size at a time (10 first, then 5, then 2),
    leaving 1 degree always available right up to the boundary --
    because that's the one step size no bound ever disables.
    """
    if step_deg <= smallest_step_deg:
        return True
    target = current_deg + direction * step_deg
    marked_bound = marked_max_deg if direction > 0 else marked_min_deg
    if marked_bound is not None and (target <= marked_bound if direction > 0 else target >= marked_bound):
        return True
    envelope_bound = envelope_max_deg if direction > 0 else envelope_min_deg
    if envelope_bound is not None and (target <= envelope_bound if direction > 0 else target >= envelope_bound):
        return True
    return False


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
