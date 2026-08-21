from perception.detection import normalize_center


def test_normalize_center_is_zero_at_frame_center():
    bbox = (45, 45, 10, 10)  # center at (50, 50)
    cx, cy = normalize_center(bbox, frame_width=100, frame_height=100)
    assert cx == 0.0
    assert cy == 0.0


def test_normalize_center_top_left_corner_is_minus_one():
    bbox = (0, 0, 0, 0)
    cx, cy = normalize_center(bbox, frame_width=100, frame_height=100)
    assert cx == -1.0
    assert cy == -1.0


def test_normalize_center_bottom_right_corner_is_plus_one():
    bbox = (100, 100, 0, 0)
    cx, cy = normalize_center(bbox, frame_width=100, frame_height=100)
    assert cx == 1.0
    assert cy == 1.0


def test_normalize_center_handles_non_square_frame():
    bbox = (160, 60, 0, 0)  # center of a 320x120 frame
    cx, cy = normalize_center(bbox, frame_width=320, frame_height=120)
    assert cx == 0.0
    assert cy == 0.0
