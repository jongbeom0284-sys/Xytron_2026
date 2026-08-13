import cv2
import numpy as np

from kobot_vehicle.lane_preview_node import analyze_lane


def test_lane_preview_detects_two_lines_without_motor_output():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (120, 479), (275, 250), (245, 245, 245), 12)
    cv2.line(image, (520, 479), (365, 250), (245, 245, 245), 12)

    overlay, status = analyze_lane(image)

    assert status['state'] == 'both'
    assert status['confidence'] >= 0.65
    assert abs(status['offset_px']) < 30
    assert overlay.shape == image.shape
