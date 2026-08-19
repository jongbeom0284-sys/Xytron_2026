import numpy as np
import pytest

from kobot_lane_drive.image_reference_controller import (
    ImageReferenceController,
    PixelLaneReference,
    capture_pixel_reference,
    measure_lane_against_reference,
)


def _lane(near_x=270.0, far_x=230.0, width=640, height=480):
    near_y = 0.85 * (height - 1)
    far_y = 0.60 * (height - 1)
    return np.asarray(
        [
            [far_x - 12.0, far_y - 70.0],
            [far_x, far_y],
            [(near_x + far_x) * 0.5, (near_y + far_y) * 0.5],
            [near_x, near_y],
            [near_x + 8.0, min(height - 1.0, near_y + 45.0)],
        ],
        dtype=np.float32,
    )


def test_reference_is_captured_from_desired_position_image():
    reference = capture_pixel_reference(_lane(), 640, 480)
    assert reference.near_x_ratio == pytest.approx(270.0 / 639.0)
    assert reference.far_x_ratio == pytest.approx(230.0 / 639.0)


def test_reference_position_gives_zero_preview():
    reference = capture_pixel_reference(_lane(), 640, 480)
    controller = ImageReferenceController(reference)
    result = controller.calculate(_lane(), 640, 480, dt=0.1)
    assert result.steering_angle_rad == pytest.approx(0.0, abs=1e-7)
    assert result.measurement.near_error_ratio == pytest.approx(0.0, abs=1e-7)


def test_lane_moving_right_requests_right_steering():
    reference = capture_pixel_reference(_lane(), 640, 480)
    controller = ImageReferenceController(reference)
    result = controller.calculate(
        _lane(near_x=300.0, far_x=260.0), 640, 480, dt=0.1
    )
    assert result.measurement.near_error_ratio < 0.0
    assert result.measurement.far_error_ratio < 0.0
    assert result.steering_angle_rad < 0.0


def test_far_sample_anticipates_left_curve():
    reference = capture_pixel_reference(_lane(), 640, 480)
    controller = ImageReferenceController(reference)
    result = controller.calculate(
        _lane(near_x=270.0, far_x=180.0), 640, 480, dt=0.1
    )
    assert result.measurement.near_error_ratio == pytest.approx(0.0, abs=1e-7)
    assert result.measurement.far_error_ratio > 0.0
    assert result.steering_angle_rad > 0.0


def test_preview_is_limited():
    reference = PixelLaneReference(0.85, 0.60, 0.80, 0.80)
    controller = ImageReferenceController(
        reference,
        near_tunings=(10.0, 0.0, 0.0),
        far_tunings=(10.0, 0.0, 0.0),
        maximum_steering_rad=0.34,
    )
    result = controller.calculate(_lane(100.0, 100.0), 640, 480, dt=0.1)
    assert result.steering_angle_rad == pytest.approx(0.34)


def test_missing_fixed_row_blocks_instead_of_extrapolating():
    reference = PixelLaneReference(0.85, 0.60, 0.42, 0.36)
    short_lane = np.asarray(
        [[200.0, 350.0], [210.0, 400.0], [220.0, 470.0]], dtype=np.float32
    )
    with pytest.raises(RuntimeError, match="FAR_SAMPLE_UNAVAILABLE"):
        measure_lane_against_reference(short_lane, 640, 480, reference)


def test_reference_ratios_are_resolution_independent():
    low = capture_pixel_reference(_lane(), 640, 480)
    high_points = _lane(width=1280, height=960)
    high_points[:, 0] *= 2.0
    high = capture_pixel_reference(high_points, 1280, 960)
    assert high.near_x_ratio == pytest.approx(low.near_x_ratio, abs=0.002)
    assert high.far_x_ratio == pytest.approx(low.far_x_ratio, abs=0.002)

