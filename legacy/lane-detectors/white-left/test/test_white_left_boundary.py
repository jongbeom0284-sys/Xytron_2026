import cv2
import numpy as np

from kobot_lane_drive.white_left_boundary import (
    WhiteLeftConfig,
    _trim_forced_continuation,
    _trim_sharp_turn,
    detect_white_left_boundary,
    detect_white_right_boundary,
    render_dual_roi_overlay,
    select_lane_control,
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


def test_trims_sharp_turn_after_preserving_control_point():
    path = np.asarray([
        (25 + min(index, 20) * 2 - max(0, index - 20), 191 - index)
        for index in range(43)
    ], dtype=np.int32)

    trimmed = _trim_sharp_turn(path, WhiteLeftConfig())

    assert len(trimmed) < len(path)
    assert int(trimmed[-1, 1]) <= int(round(240 * 0.70))


def test_keeps_gently_curving_path():
    path = np.asarray([
        (25 + index + index * index // 180, 191 - index)
        for index in range(43)
    ], dtype=np.int32)

    trimmed = _trim_sharp_turn(path, WhiteLeftConfig())

    assert np.array_equal(trimmed, path)


def test_detects_mirrored_right_boundary_with_same_d():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (558, 390), (508, 250), (245, 245, 245), 12, cv2.LINE_AA)

    result = detect_white_right_boundary(
        image,
        WhiteLeftConfig(minimum_path_score=0.20),
    )

    assert result.state == "READY"
    assert result.path is not None
    assert result.right_boundary_x_px is not None
    assert result.right_boundary_x_px > image.shape[1] * 0.5
    assert result.desired_right_distance_px == 244.0


def test_overlay_draws_symmetric_rois_and_both_d_targets():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (82, 390), (132, 250), (245, 245, 245), 12, cv2.LINE_AA)
    cv2.line(image, (558, 390), (508, 250), (245, 245, 245), 12, cv2.LINE_AA)
    config = WhiteLeftConfig(minimum_path_score=0.20)
    left = detect_white_left_boundary(image, config)
    right = detect_white_right_boundary(image, config)

    overlay = render_dual_roi_overlay(image, config, left, right)

    assert overlay.shape == image.shape
    assert not np.array_equal(overlay, image)
    assert tuple(overlay[300, 0]) != tuple(image[300, 0])
    assert tuple(overlay[300, 639]) != tuple(image[300, 639])


def test_edge_pair_ransac_prefers_bottom_connected_lane_over_glare():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(image, (44, 384), (291, 300), (245, 245, 245), 12, cv2.LINE_AA)
    cv2.line(image, (72, 300), (72, 350), (255, 255, 255), 20, cv2.LINE_AA)

    result = detect_white_left_boundary(image, WhiteLeftConfig())

    assert result.state == "READY"
    assert result.left_boundary_x_px is not None
    assert result.left_boundary_x_px > 140.0


def test_each_frame_is_detected_without_previous_curve_state():
    first = np.full((480, 640, 3), 55, dtype=np.uint8)
    second = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(first, (45, 390), (125, 250), (245, 245, 245), 12, cv2.LINE_AA)
    cv2.line(second, (105, 390), (185, 250), (245, 245, 245), 12, cv2.LINE_AA)

    first_result = detect_white_left_boundary(first, WhiteLeftConfig())
    second_result = detect_white_left_boundary(second, WhiteLeftConfig())

    assert first_result.left_boundary_x_px is not None
    assert second_result.left_boundary_x_px is not None
    assert second_result.left_boundary_x_px - first_result.left_boundary_x_px > 45.0


def test_blank_frame_is_lost():
    image = np.full((480, 640, 3), 55, dtype=np.uint8)

    result = detect_white_left_boundary(image, WhiteLeftConfig())

    assert result.state == "LOST"
    assert result.path is None
    assert result.center_offset_px is None


def test_selects_both_lane_center_then_each_visible_side():
    both = np.full((480, 640, 3), 55, dtype=np.uint8)
    cv2.line(both, (82, 390), (132, 250), (245, 245, 245), 12, cv2.LINE_AA)
    cv2.line(both, (558, 390), (508, 250), (245, 245, 245), 12, cv2.LINE_AA)
    config = WhiteLeftConfig()
    left = detect_white_left_boundary(both, config)
    right = detect_white_right_boundary(both, config)

    used_lane, offset, confidence = select_lane_control(left, right)

    assert used_lane == "BOTH"
    assert offset == 0.5 * (left.center_offset_px + right.center_offset_px)
    assert confidence > 0.0

    blank = np.full_like(both, 55)
    lost_left = detect_white_left_boundary(blank, config)
    lost_right = detect_white_right_boundary(blank, config)
    assert select_lane_control(left, lost_right)[0] == "LEFT"
    assert select_lane_control(lost_left, right)[0] == "RIGHT"
    assert select_lane_control(lost_left, lost_right) == ("LOST", None, 0.0)

    single_distance = 180.0
    left_lane, left_offset, _ = select_lane_control(
        left,
        lost_right,
        single_distance,
    )
    right_lane, right_offset, _ = select_lane_control(
        lost_left,
        right,
        single_distance,
    )
    assert left_lane == "LEFT"
    assert right_lane == "RIGHT"
    assert left_offset == left.center_offset_px - 64.0
    assert right_offset == right.center_offset_px + 64.0
