import numpy as np
import pytest

pytest.importorskip("shapely")

from kobot_lane_drive.metric_path import offset_left_lane, validate_camera_matrix


def test_validates_camera_matrix():
    validate_camera_matrix(np.asarray([[500, 0, 320], [0, 500, 240], [0, 0, 1]]))
    with pytest.raises(ValueError):
        validate_camera_matrix(np.zeros((3, 3)))


def test_shapely_offsets_left_boundary_to_the_right():
    lane = np.asarray([[0.2, 0.8, 0.0], [1.0, 0.8, 0.0], [2.0, 0.8, 0.0]])
    result = offset_left_lane(lane, target_distance_m=0.5)
    assert np.allclose(result.target_path_xy[:, 1], 0.3, atol=1e-6)
    assert result.measured_distance_m == pytest.approx(np.hypot(0.2, 0.8))
    assert result.lateral_error_m == pytest.approx(np.hypot(0.2, 0.8) - 0.5)


def test_rejects_insufficient_ground_points():
    with pytest.raises(ValueError):
        offset_left_lane(np.asarray([[0.1, 0.8, 0.0]]), 0.5)
