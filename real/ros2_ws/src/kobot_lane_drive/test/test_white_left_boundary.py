import cv2
import numpy as np

from kobot_lane_drive.white_left_boundary import (
    WhiteLeftConfig,
    _trim_forced_continuation,
    detect_white_left_boundary,
)


def test_detects_nearest_left_white_boundary_and_pixel_distance():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (82, 390), (132, 250), (245, 245, 245), 12, cv2.LINE_AA)

    result = detect_white_left_boundary(
        image,
        WhiteLeftConfig(minimum_path_score=0.20),
        render_debug=True,
    )

    assert result.state == "READY"
    assert result.path is not None
    assert result.left_boundary_x_px is not None
    assert result.left_boundary_x_px < image.shape[1] * 0.5
    assert result.desired_left_distance_px == 244.0
    assert result.debug_image is not None


def test_rejects_invalid_frame_shape():
    invalid = np.zeros((480, 640), dtype=np.uint8)
    try:
        detect_white_left_boundary(invalid)
    except ValueError as error:
        assert "BGR" in str(error)
    else:
        raise AssertionError("invalid frame should fail")


def test_stops_after_sustained_low_score_gap():
    score = np.ones((240, 320), dtype=np.float32)
    path = np.asarray([
        (100 + index, 192 - index)
        for index in range(50)
    ], dtype=np.int32)
    score[path[35:, 1], path[35:, 0]] = 0.10

    trimmed = _trim_forced_continuation(path, score, WhiteLeftConfig())

    assert trimmed.tolist() == path[:35].tolist()


def test_default_roi_stops_at_y300_in_480p_frame():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (82, 390), (132, 250), (245, 245, 245), 12, cv2.LINE_AA)

    result = detect_white_left_boundary(
        image,
        WhiteLeftConfig(minimum_path_score=0.20),
    )

    assert result.path is not None
    assert int(result.path[:, 1].min()) >= 300
