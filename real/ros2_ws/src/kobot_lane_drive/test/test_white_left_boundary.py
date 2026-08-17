import cv2
import numpy as np

from kobot_lane_drive.white_left_boundary import (
    WhiteLeftConfig,
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
