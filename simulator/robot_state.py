"""RobotState: wraps robot/gait.py's GaitState (leg joint angles, foot
positions, gait phase) and adds a dead-reckoned body pose (position,
heading) purely so sim_view.py has a body to draw. No sockets, no Qt --
SimRobotLink is the only thing that drives it, on its own background
thread; see that module for the fixed-rate stepping loop.

Body pose has no equivalent to check it against on the firmware side: a
real hexapod's body moves because its legs push against the ground, not
because anything computes a body-frame transform, and neither
robot/gait.py nor firmware/lib/core/Gait.cpp track it. Physical fidelity
is explicitly not the goal here (see the design discussion this was
built from) -- this is a visualization convenience, not a physics
simulation, and its constants are not measured against anything real.
"""

import math
import threading
from dataclasses import dataclass

from robot.gait import DEFAULT_BODY_HEIGHT, GaitState, is_in_stance, step
from robot.kinematics import Point3

# Dead-reckoning only, not measured against real hardware -- see the
# module docstring. Chosen so the top-down view visibly moves at a
# reasonable rate, nothing more.
BODY_LINEAR_MM_PER_S_AT_FULL_SPEED = 150.0
BODY_TURN_DEG_PER_S_AT_FULL_SPEED = 90.0


@dataclass(frozen=True)
class RobotSnapshot:
    """Immutable, safe to hand to any thread -- what sim_view.py polls."""

    body_x_mm: float
    body_y_mm: float
    heading_deg: float
    gait_phase: float
    foot_positions: tuple[Point3, ...]
    stance: tuple[bool, ...]  # per leg, True = foot planted (stance half of the tripod cycle)
    connected: bool
    # Straight passthrough of GaitState's cumulative clip stats -- see
    # that class's docstring in robot/gait.py. sim_link.py's _augment()
    # threads these into Telemetry the same way it already does gait_phase.
    ik_clip_count: int
    ik_clip_worst_mm: float
    joint_clip_count: int
    joint_clip_worst_deg: float
    clipped_this_tick: bool


class RobotState:
    """Not thread-safe to call step()/snapshot() from multiple threads
    concurrently without external synchronization beyond what's needed
    for a single writer (SimRobotLink's background thread) and multiple
    readers (snapshot() only) -- every mutable field is behind one lock,
    matching the "one Lock, one 'latest value'" pattern
    transport/link.py's RobotLink already uses."""

    def __init__(self, body_height: float = DEFAULT_BODY_HEIGHT) -> None:
        self._lock = threading.Lock()
        self._gait = GaitState.initial(body_height=body_height)
        self._body_x_mm = 0.0
        self._body_y_mm = 0.0
        self._heading_deg = 0.0

    def step(self, dt_s: float, vx: float, vy: float, speed: float, rotation: float, body_height: float) -> None:
        new_gait = step(self._gait, dt_s, vx, vy, speed, rotation, body_height)

        speed_frac = max(0.0, min(100.0, speed)) / 100.0
        magnitude = math.hypot(vx, vy)
        if magnitude > 1.0:
            vx, vy = vx / magnitude, vy / magnitude

        with self._lock:
            heading_rad = math.radians(self._heading_deg)
            # vx/vy are body-frame; rotate into world frame before integrating.
            world_dx = (vx * math.cos(heading_rad) - vy * math.sin(heading_rad)) * (
                speed_frac * BODY_LINEAR_MM_PER_S_AT_FULL_SPEED * dt_s
            )
            world_dy = (vx * math.sin(heading_rad) + vy * math.cos(heading_rad)) * (
                speed_frac * BODY_LINEAR_MM_PER_S_AT_FULL_SPEED * dt_s
            )
            self._body_x_mm += world_dx
            self._body_y_mm += world_dy
            self._heading_deg = (
                self._heading_deg + rotation * speed_frac * BODY_TURN_DEG_PER_S_AT_FULL_SPEED * dt_s
            ) % 360.0
            self._gait = new_gait

    def snapshot(self, connected: bool) -> RobotSnapshot:
        with self._lock:
            body_x, body_y, heading, gait = self._body_x_mm, self._body_y_mm, self._heading_deg, self._gait
        feet = gait.foot_positions()
        stance = tuple(is_in_stance(i, gait.phase) for i in range(6))
        return RobotSnapshot(
            body_x_mm=body_x,
            body_y_mm=body_y,
            heading_deg=heading,
            gait_phase=gait.phase,
            foot_positions=feet,
            stance=stance,
            connected=connected,
            ik_clip_count=gait.ik_clip_count,
            ik_clip_worst_mm=gait.ik_clip_worst_mm,
            joint_clip_count=gait.joint_clip_count,
            joint_clip_worst_deg=gait.joint_clip_worst_deg,
            clipped_this_tick=gait.clipped_this_tick,
        )
