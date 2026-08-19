import numpy as np
from kobot_lane_drive.lane_selector import select_left_lane


def _lane(x_values):
    return np.column_stack((x_values, [260.0, 330.0, 410.0, 470.0])).astype(
        np.float32
    )


def test_selects_nearest_lane_left_of_center():
    lanes = [_lane([80, 90, 100, 110]), _lane([210, 230, 250, 270]), _lane([430, 420, 410, 400])]
    selected = select_left_lane(lanes, image_width=640, image_height=480)
    assert selected.state == "DETECTED"
    assert selected.source_index == 1
    assert selected.bottom_x == 270.0


def test_does_not_fall_back_to_right_lane():
    selected = select_left_lane([_lane([450, 440, 430, 420])], 640, 480)
    assert selected.state == "LOST"
    assert selected.points is None


def test_rejects_lane_without_near_field_points():
    lane = np.asarray([[100.0, 20.0], [110.0, 80.0], [120.0, 140.0]])
    selected = select_left_lane([lane], 640, 480)
    assert selected.state == "LOST"
