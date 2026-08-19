import cv2
import numpy as np

from kobot_lane_drive.fast_lane import detect_fast_lane


def _track_image(brightness: int = 65) -> np.ndarray:
    image = np.full((480, 640, 3), brightness, dtype=np.uint8)
    cv2.line(image, (35, 450), (245, 205), (235, 235, 235), 10, cv2.LINE_AA)
    cv2.line(image, (605, 450), (395, 205), (235, 235, 235), 10, cv2.LINE_AA)
    for y_value in (435, 355, 275):
        half_length = 28 if y_value > 400 else 18
        cv2.line(
            image,
            (320, y_value + half_length),
            (320, y_value - half_length),
            (0, 220, 255),
            10,
            cv2.LINE_AA,
        )
    return image


def test_detects_both_outer_boundaries_and_center():
    result = detect_fast_lane(_track_image())

    assert result.state == "BOTH"
    assert result.confidence >= 0.70
    assert result.center_offset_px is not None
    assert abs(result.center_offset_px) < 18.0
    assert result.processing_ms > 0.0
    assert result.status()["motor_output"] is False
    assert result.status()["metric_calibrated"] is False
    assert result.status()["colour_space"] == "HSV"
    assert result.mask_rejected is False


def test_tolerates_global_brightness_change():
    dark = detect_fast_lane(_track_image(brightness=35))
    bright = detect_fast_lane(_track_image(brightness=115))

    assert dark.state == "BOTH"
    assert bright.state == "BOTH"
    assert abs(dark.center_offset_px) < 20.0
    assert abs(bright.center_offset_px) < 20.0


def test_never_uses_yellow_without_an_outer_boundary():
    image = np.full((480, 640, 3), 65, dtype=np.uint8)
    for y_value in (435, 355, 275):
        cv2.line(image, (320, y_value + 25), (320, y_value - 25), (0, 220, 255), 10)

    result = detect_fast_lane(image)

    assert result.state == "LOST"
    assert result.confidence == 0.0
    assert result.center_offset_px is None


def test_blank_frame_is_lost():
    result = detect_fast_lane(np.full((480, 640, 3), 60, dtype=np.uint8))

    assert result.state == "LOST"
    assert result.center_offset_px is None
    assert result.confidence == 0.0


def test_rejects_overexposed_neutral_roi_instead_of_driving_on_it():
    result = detect_fast_lane(np.full((480, 640, 3), 255, dtype=np.uint8))

    assert result.state == "LOST"
    assert result.center_offset_px is None
    assert result.confidence == 0.0
    assert result.mask_rejected is True
    assert result.white_support_ratio > 0.90


def test_rejects_large_warm_floor_region():
    image = np.full((480, 640, 3), (0, 210, 245), dtype=np.uint8)

    result = detect_fast_lane(image)

    assert result.state == "LOST"
    assert result.center_offset_px is None
    assert result.mask_rejected is True
    assert result.yellow_support_ratio > 0.90


def test_person_sized_bright_blob_does_not_replace_outer_boundaries():
    image = _track_image()
    cv2.rectangle(image, (275, 250), (365, 445), (230, 230, 230), -1)

    result = detect_fast_lane(image)

    assert result.state == "BOTH"
    assert result.center_offset_px is not None
    assert abs(result.center_offset_px) < 20.0
    assert result.mask_rejected is False


def test_moderately_bright_roi_is_not_treated_as_catastrophic_mask_growth():
    image = _track_image()
    cv2.rectangle(image, (0, 310), (639, 400), (255, 255, 255), -1)

    result = detect_fast_lane(image)

    assert 0.32 < result.white_support_ratio < 0.65
    assert result.mask_rejected is False
