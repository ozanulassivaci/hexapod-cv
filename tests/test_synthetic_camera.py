import time

import numpy as np

from perception.hsv_detector import HSVDetector
from simulator.sim_link import SimRobotLink
from simulator.synthetic_camera import (
    BACKGROUND_COLOR_BGR,
    SyntheticCameraStream,
    SyntheticTarget,
)

_HSV_LOWER = (100, 150, 50)
_HSV_UPPER = (140, 255, 255)


def make_detector() -> HSVDetector:
    return HSVDetector(lower=_HSV_LOWER, upper=_HSV_UPPER, min_contour_area=500, label="blue_object")


def test_read_before_start_returns_none():
    link = SimRobotLink()
    stream = SyntheticCameraStream(link, [SyntheticTarget(0.0)])
    frame, timestamp = stream.read()
    assert frame is None
    assert timestamp is None
    link.close()


def test_is_connected_only_after_start():
    link = SimRobotLink()
    stream = SyntheticCameraStream(link, [SyntheticTarget(0.0)])
    assert stream.is_connected is False
    stream.start()
    assert stream.is_connected is True
    stream.stop()
    assert stream.is_connected is False
    link.close()


def test_frame_arrives_after_latency():
    link = SimRobotLink()
    stream = SyntheticCameraStream(link, [SyntheticTarget(0.0)], latency_s=0.05, fps=30).start()
    time.sleep(0.2)
    frame, timestamp = stream.read()
    assert frame is not None
    assert isinstance(frame, np.ndarray)
    assert timestamp is not None
    stream.stop()
    link.close()


def test_frame_age_reflects_latency():
    link = SimRobotLink()
    latency_s = 0.15
    stream = SyntheticCameraStream(link, [SyntheticTarget(0.0)], latency_s=latency_s, fps=30).start()
    time.sleep(0.3)
    frame, timestamp = stream.read()
    age_s = time.monotonic() - timestamp
    assert age_s >= latency_s  # the frame is at least as old as the simulated pipeline delay
    stream.stop()
    link.close()


def test_centered_target_detected_near_frame_center():
    link = SimRobotLink()
    stream = SyntheticCameraStream(link, [SyntheticTarget(world_bearing_deg=0.0)], latency_s=0.02, fps=30).start()
    time.sleep(0.1)
    frame, _ = stream.read()
    detections = make_detector().detect(frame)
    assert len(detections) == 1
    assert abs(detections[0].cx) < 0.05
    stream.stop()
    link.close()


def test_target_appears_offset_from_bearing():
    link = SimRobotLink()
    half_fov = 30.0
    stream = SyntheticCameraStream(
        link, [SyntheticTarget(world_bearing_deg=15.0)], latency_s=0.02, fps=30, half_fov_deg=half_fov
    ).start()
    time.sleep(0.1)
    frame, _ = stream.read()
    detections = make_detector().detect(frame)
    assert len(detections) == 1
    assert detections[0].cx > 0.3  # 15/30 = 0.5, roughly
    stream.stop()
    link.close()


def test_target_outside_fov_not_detected():
    link = SimRobotLink()
    stream = SyntheticCameraStream(
        link, [SyntheticTarget(world_bearing_deg=80.0)], latency_s=0.02, fps=30, half_fov_deg=30.0
    ).start()
    time.sleep(0.1)
    frame, _ = stream.read()
    detections = make_detector().detect(frame)
    assert detections == []
    stream.stop()
    link.close()


def test_flicker_probability_one_never_renders_target():
    link = SimRobotLink()
    target = SyntheticTarget(world_bearing_deg=0.0, flicker_probability=1.0)
    stream = SyntheticCameraStream(link, [target], latency_s=0.02, fps=30, rng_seed=1).start()
    time.sleep(0.15)
    frame, _ = stream.read()
    detections = make_detector().detect(frame)
    assert detections == []
    stream.stop()
    link.close()


def test_background_does_not_trigger_detection_with_no_targets():
    link = SimRobotLink()
    stream = SyntheticCameraStream(link, [], latency_s=0.02, fps=30).start()
    time.sleep(0.1)
    frame, _ = stream.read()
    assert frame is not None
    assert np.all(frame == np.array(BACKGROUND_COLOR_BGR, dtype=np.uint8))
    detections = make_detector().detect(frame)
    assert detections == []
    stream.stop()
    link.close()


def test_heading_moves_target_across_frame():
    """The whole point of this module: apparent position depends on the
    robot's live heading, not just the target's fixed world bearing."""
    link = SimRobotLink()
    stream = SyntheticCameraStream(
        link, [SyntheticTarget(world_bearing_deg=20.0)], latency_s=0.02, fps=30, half_fov_deg=30.0
    ).start()
    time.sleep(0.1)
    frame, _ = stream.read()
    cx_before = make_detector().detect(frame)[0].cx

    # Directly mutate the simulated heading (bypassing gait entirely) to
    # isolate this test to the camera's own geometry, not gait dynamics.
    with link.state._lock:
        link.state._heading_deg = 20.0  # heading now matches the target's bearing exactly

    time.sleep(0.1)
    frame, _ = stream.read()
    cx_after = make_detector().detect(frame)[0].cx

    assert abs(cx_after) < abs(cx_before)  # target moved toward center as heading approached its bearing
    stream.stop()
    link.close()
