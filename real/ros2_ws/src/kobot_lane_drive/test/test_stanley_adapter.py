import numpy as np
from kobot_lane_drive.stanley_adapter import calculate_stanley_preview


def test_pythonrobotics_stanley_keeps_straight_center_path(package_root):
    vendor = package_root / "third_party" / "PythonRobotics"
    path = np.asarray([[0.2, 0.0], [1.0, 0.0], [2.0, 0.0]])
    result = calculate_stanley_preview(path, 0.5, vendor)
    assert abs(result.steering_angle_rad) < 1e-6
