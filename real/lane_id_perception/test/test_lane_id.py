import math
import time

import cv2
import numpy as np

from lane_id_perception.geometry import fit_quadratic_ransac
from lane_id_perception.lane_config import LaneVisionConfig
from lane_id_perception.lane_vision import LaneDetector


def draw_synthetic_lane(detector, left_offset=0.0, right_offset=0.0):
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (60, 60, 60)
    _homography, inv_homography = detector._get_homography(640, 480)
    xs = np.linspace(0.8, 7.2, 80)
    left_y = 0.65 + left_offset + 0.02 * (xs - 3.0) ** 2
    right_y = left_y - 1.35 + right_offset

    def project(ys):
        pts = []
        for x, y in zip(xs, ys):
            u, v = detector._ground_to_bev_pixel(float(x), float(y))
            pts.append([u, v])
        pts = np.asarray(pts, np.float32).reshape((-1, 1, 2))
        return cv2.perspectiveTransform(pts, inv_homography).astype(np.int32)

    yellow = project(left_y)
    for i in range(0, len(yellow) - 6, 14):
        cv2.polylines(img, [yellow[i : i + 8]], False, (0, 220, 255), 8, cv2.LINE_AA)
    white = project(right_y)
    cv2.polylines(img, [white], False, (245, 245, 245), 8, cv2.LINE_AA)
    return img


def draw_narrow_road_with_distractor(detector):
    cfg = detector.cfg
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    img[:] = (70, 180, 70)
    center_x = cfg.source_center_x_ratio * 640
    top_y = int(cfg.source_top_y_ratio * 480)
    bottom_y = int(cfg.source_bottom_y_ratio * 480)
    top_hw = int(cfg.source_top_half_width_ratio * 640 * 0.72)
    bottom_hw = int(cfg.source_bottom_half_width_ratio * 640 * 0.70)
    road = np.array(
        [
            [center_x - top_hw, top_y],
            [center_x + top_hw, top_y],
            [center_x + bottom_hw, bottom_y],
            [center_x - bottom_hw, bottom_y],
        ],
        dtype=np.int32,
    )
    cv2.fillConvexPoly(img, road, (60, 60, 60))
    cv2.line(img, (420, 292), (515, 458), (245, 245, 245), 8, cv2.LINE_AA)
    cv2.line(img, (315, 292), (360, 458), (0, 220, 255), 8, cv2.LINE_AA)
    cv2.rectangle(img, (48, 405), (88, 458), (255, 255, 255), -1)
    return img


def test_ransac_rejects_outliers():
    cfg = LaneVisionConfig()
    xs = np.linspace(0.7, 7.0, 90)
    ys = 0.04 * xs * xs - 0.12 * xs + 0.7
    points = list(zip(xs, ys))
    points.extend([(1.5, -1.5), (3.0, 2.5), (6.0, -2.0)])
    fit = fit_quadratic_ransac(points, cfg, seed=1)
    assert fit is not None
    assert fit.confidence > 0.75
    assert abs(float(fit.y(3.0)) - (0.04 * 9.0 - 0.36 + 0.7)) < 0.06


def test_lane_detector_uses_inner_edges_for_safe_center():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    img = draw_synthetic_lane(detector)

    result = detector.process(img, time.time(), frame_id=1, make_debug=False)
    assert result.has_path
    assert result.state == "both_boundaries"
    assert result.overall_lane_confidence > 0.6
    assert cfg.lane_width_min_m <= result.estimated_lane_width <= cfg.lane_width_max_m
    clearance = 0.5 * cfg.vehicle_width_m + cfg.safety_margin_m
    assert math.isclose(float(result.safe_left_y[0]), float(result.left_fit.y(result.sample_x[0])) - clearance, abs_tol=0.04)
    assert math.isclose(float(result.safe_right_y[0]), float(result.right_fit.y(result.sample_x[0])) + clearance, abs_tol=0.04)


def test_lane_detector_emits_dedicated_bev_debug_image():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)

    result = detector.process(draw_synthetic_lane(detector), time.time(), frame_id=1, make_debug=True)

    assert result.debug_image is not None
    assert result.bev_debug_image is not None
    assert result.bev_debug_image.shape == (cfg.bev_height_px, cfg.bev_width_px, 3)
    assert int(np.count_nonzero(result.bev_debug_image)) > 0


def test_lane_detector_emits_mask_debug_image_with_contours():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)

    result = detector.process(draw_synthetic_lane(detector), time.time(), frame_id=1, make_debug=True)

    assert result.mask_debug_image is not None
    assert result.mask_debug_image.shape == (480, 640, 3)
    assert int(np.count_nonzero(result.mask_debug_image)) > 0


def test_lane_detector_emits_front_pixel_detection_overlay():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)

    result = detector.process(draw_synthetic_lane(detector), time.time(), frame_id=1, make_debug=True)

    assert result.front_detection_image is not None
    assert result.front_detection_image.shape == (480, 640, 3)
    assert int(np.count_nonzero(result.front_detection_image)) > 0
    assert result.front_detection_image[360:470, 430:560].mean() < 180


def test_lane_masks_are_gated_by_detected_road_envelope():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    img = draw_narrow_road_with_distractor(detector)

    _yellow, white, road, _roi = detector._make_masks(img)

    assert int(np.count_nonzero(road)) > 0
    assert int(np.count_nonzero(white[405:458, 48:88])) == 0
    assert int(np.count_nonzero(white)) > 0


def test_lane_detector_rejects_large_single_frame_jump():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    t0 = time.time()
    first = detector.process(draw_synthetic_lane(detector), t0, frame_id=1, make_debug=False)
    shifted = detector.process(draw_synthetic_lane(detector, left_offset=1.0), t0 + 0.1, frame_id=2, make_debug=False)

    assert first.has_path
    assert shifted.has_path
    assert "predicted_previous" in shifted.state
    assert shifted.debug["fit_reject_reasons"]
    assert abs(float(shifted.lateral_error_near) - float(first.lateral_error_near)) < 0.08


def test_lane_detector_smooths_accepted_center_step():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    t0 = time.time()
    first = detector.process(draw_synthetic_lane(detector), t0, frame_id=1, make_debug=False)
    shifted = detector.process(draw_synthetic_lane(detector, left_offset=0.30), t0 + 0.1, frame_id=2, make_debug=False)

    assert first.has_path
    assert shifted.has_path
    assert shifted.debug["tracking"]["accepted_current"]
    raw_step = float(shifted.raw_center_y[np.argmin(np.abs(shifted.sample_x - 1.2))] - first.path_y[np.argmin(np.abs(first.sample_x - 1.2))])
    filtered_step = float(shifted.lateral_error_near - first.lateral_error_near)
    assert raw_step > 0.20
    assert 0.0 < filtered_step < raw_step * 0.6


def test_horizontal_components_are_rejected_as_intersection_noise():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    mask = np.zeros((cfg.bev_height_px, cfg.bev_width_px), dtype=np.uint8)
    cv2.rectangle(mask, (80, 300), (400, 314), 255, -1)

    filtered, stats = detector._filter_components(mask, color="white")

    assert stats["horizontal_count"] == 1
    assert stats["rejected"]["horizontal"] == 1
    assert int(np.count_nonzero(filtered)) == 0


def test_scanline_chains_keep_lane_ids_when_only_right_lane_is_visible():
    cfg = LaneVisionConfig()
    detector = LaneDetector(cfg)
    t0 = time.time()
    both = detector.process(draw_synthetic_lane(detector), t0, frame_id=1, make_debug=False)
    right_only_image = draw_synthetic_lane(detector)
    hsv = cv2.cvtColor(right_only_image, cv2.COLOR_BGR2HSV)
    yellow = cv2.inRange(hsv, np.array([15, 70, 80], np.uint8), np.array([42, 255, 255], np.uint8))
    right_only_image[yellow > 0] = (60, 60, 60)
    right_only = detector.process(right_only_image, t0 + 0.1, frame_id=2, make_debug=False)

    assert both.left_lane_id == right_only.left_lane_id == 1
    assert both.right_lane_id == right_only.right_lane_id == 2
    assert right_only.has_path
    assert not right_only.observed_left
    assert right_only.observed_right
    assert right_only.scanline_right_points.shape[0] >= cfg.scanline_min_slices


def test_lane_memory_timeout_uses_last_real_observation():
    cfg = LaneVisionConfig(memory_timeout_s=0.8)
    detector = LaneDetector(cfg)
    t0 = time.time()
    visible = detector.process(draw_synthetic_lane(detector), t0, frame_id=1, make_debug=False)
    blank = np.zeros((480, 640, 3), dtype=np.uint8)
    short_loss = detector.process(blank, t0 + 0.2, frame_id=2, make_debug=False)
    expired = detector.process(blank, t0 + 0.9, frame_id=3, make_debug=False)

    assert visible.has_path
    assert short_loss.has_path
    assert short_loss.state.startswith("predicted_previous")
    assert short_loss.search_mode == "near"
    assert expired.search_mode == "wide"
    assert not expired.has_path
    assert expired.state == "no_lane_stop"


def test_debug_views_include_scanline_lane_ids_and_true_mask():
    detector = LaneDetector(LaneVisionConfig())
    result = detector.process(draw_synthetic_lane(detector), time.time(), frame_id=1, make_debug=True)

    assert result.front_detection_image is not None
    assert result.mask_debug_image is not None
    assert result.scanline_left_points.shape[0] > 0
    assert result.scanline_right_points.shape[0] > 0
    assert float(result.mask_debug_image.mean()) < float(result.front_detection_image.mean())
