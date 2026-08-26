"""Closed-loop regression tests: real HSVDetector + real Tracker + real
SimRobotLink, driven by SyntheticCameraStream -- the same pipeline
ui/main_window.py's AUTO_TRACK wires together, exercised with no Qt event
loop and no hardware. Encodes the empirical findings from the tuning
investigation this was built from (see control/tracker.py's
TrackerConfig docstrings for the numbers) as regression tests, not just
prose -- if a future change to Tracker/SyntheticCameraStream/SimRobotLink
breaks convergence, lost-target handling, or flicker smoothing, one of
these should fail.

Kept short (a few seconds of real wall-clock time each, tracker/camera
threads run on real timing) -- deep parameter sweeps belong in a
throwaway investigation script, not the permanent suite.
"""

import time

from config import DetectionConfig
from control.tracker import Tracker, TrackerConfig
from perception.hsv_detector import HSVDetector
from simulator.sim_link import SimRobotLink
from simulator.synthetic_camera import SyntheticCameraStream, SyntheticTarget

_DET_CFG = DetectionConfig()
_TICK_S = 0.033  # operator_config.yaml's default ui.tick_interval_s
_REALISTIC_LATENCY_S = 0.2
_REALISTIC_FPS = 15.0


def _make_detector() -> HSVDetector:
    return HSVDetector(
        lower=_DET_CFG.hsv.lower, upper=_DET_CFG.hsv.upper,
        min_contour_area=_DET_CFG.min_contour_area, label=_DET_CFG.label,
    )


def _run_loop(cam, tracker, link, duration_s, on_tick=None):
    detector = _make_detector()
    start = time.monotonic()
    log = []
    while time.monotonic() - start < duration_s:
        t = time.monotonic() - start
        if on_tick is not None:
            on_tick(t, cam)
        frame, _ts = cam.read()
        detections = detector.detect(frame) if frame is not None else []
        command = tracker.decide(detections, t)
        link.send(command)
        log.append((t, link.snapshot().heading_deg, detections, command))
        time.sleep(_TICK_S)
    return log


def test_closed_loop_converges_toward_target():
    link = SimRobotLink()
    target_bearing = 20.0
    cam = SyntheticCameraStream(
        link, [SyntheticTarget(target_bearing)], latency_s=_REALISTIC_LATENCY_S, fps=_REALISTIC_FPS
    ).start()
    tracker = Tracker(TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0))
    time.sleep(_REALISTIC_LATENCY_S + 0.05)

    log = _run_loop(cam, tracker, link, duration_s=4.0)
    cam.stop()
    link.close()

    final_heading = log[-1][1]
    # Should have moved substantially toward the target, not stalled near
    # its starting heading of 0.
    assert final_heading > target_bearing * 0.5


def test_no_sustained_oscillation_with_default_dead_zone_under_jitter():
    link = SimRobotLink()
    target = SyntheticTarget(20.0, jitter_deg=1.5)
    cam = SyntheticCameraStream(link, [target], latency_s=_REALISTIC_LATENCY_S, fps=_REALISTIC_FPS).start()
    tracker = Tracker(TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0))
    time.sleep(_REALISTIC_LATENCY_S + 0.05)

    log = _run_loop(cam, tracker, link, duration_s=5.0)
    cam.stop()
    link.close()

    tail = log[len(log) // 2 :]
    headings = [row[1] for row in tail]
    deltas = [b - a for a, b in zip(headings, headings[1:])]
    signs = [1 if d > 1e-6 else (-1 if d < -1e-6 else 0) for d in deltas]
    signs = [s for s in signs if s != 0]
    flips = sum(1 for a, b in zip(signs, signs[1:]) if a != b)
    assert flips == 0  # settled, not chattering, in the back half of the run


def test_lost_target_mid_turn_briefly_continues_then_stops():
    """Pins the fix: losing the target for a moment while actively
    turning must not freeze the robot on the very next tick."""
    link = SimRobotLink()
    target = SyntheticTarget(30.0)
    cam = SyntheticCameraStream(
        link, [target], latency_s=_REALISTIC_LATENCY_S, fps=_REALISTIC_FPS
    ).start()
    tracker = Tracker(
        TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.3)
    )
    time.sleep(_REALISTIC_LATENCY_S + 0.05)

    def drop_target_after(t, cam):
        if t >= 0.3:
            cam.targets = []

    log = _run_loop(cam, tracker, link, duration_s=2.0, on_tick=drop_target_after)
    cam.stop()
    link.close()

    heading_at_loss = next(row[1] for row in log if row[0] >= 0.3)
    # Some tick shortly after losing the target should still show a turn
    # in progress (the hold), not an immediate freeze.
    soon_after = [row for row in log if 0.3 < row[0] < 0.3 + 0.3]
    assert any(row[1] > heading_at_loss for row in soon_after)

    # But it must still give up eventually -- well past the grace period.
    final_heading = log[-1][1]
    late = [row for row in log if row[0] > 1.5]
    assert all(row[1] == final_heading for row in late)


def test_flicker_does_not_thrash_commands():
    """Pins the fix: intermittent single-frame detection dropout must not
    produce a command change on nearly every tick."""
    link = SimRobotLink()
    target = SyntheticTarget(20.0, flicker_probability=0.5)
    cam = SyntheticCameraStream(
        link, [target], latency_s=_REALISTIC_LATENCY_S, fps=_REALISTIC_FPS, rng_seed=7
    ).start()
    tracker = Tracker(
        TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0, lost_target_hold_s=0.3)
    )
    time.sleep(_REALISTIC_LATENCY_S + 0.05)

    log = _run_loop(cam, tracker, link, duration_s=6.0)
    cam.stop()
    link.close()

    commands = [row[3] for row in log]
    rates = [getattr(c, "rate", None) for c in commands]
    changes = sum(1 for a, b in zip(rates, rates[1:]) if a != b)
    change_fraction = changes / len(rates)
    assert change_fraction < 0.3  # well under "changes almost every tick"


def test_never_detected_stays_stopped():
    link = SimRobotLink()
    cam = SyntheticCameraStream(link, [], latency_s=0.05, fps=30).start()
    tracker = Tracker(TrackerConfig(dead_zone=0.1, turn_gain=0.8, turn_speed=30.0))
    time.sleep(0.1)

    log = _run_loop(cam, tracker, link, duration_s=1.0)
    cam.stop()
    link.close()

    assert all(row[1] == 0.0 for row in log)  # heading never moved
