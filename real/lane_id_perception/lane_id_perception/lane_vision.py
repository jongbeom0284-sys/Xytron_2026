from __future__ import annotations

from dataclasses import dataclass, field
import math
from typing import Any

import cv2
import numpy as np

from .geometry import PolynomialFit, clamp, fit_quadratic_ransac
from .lane_config import LaneVisionConfig


@dataclass
class LaneDetectionResult:
    frame_id: int
    capture_timestamp: float
    state: str
    path_x: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    path_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    safe_left_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    safe_right_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    sample_x: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    left_fit: PolynomialFit | None = None
    right_fit: PolynomialFit | None = None
    left_lane_confidence: float = 0.0
    right_lane_confidence: float = 0.0
    overall_lane_confidence: float = 0.0
    estimated_lane_width: float = 0.0
    lateral_error_near: float = 0.0
    lateral_error_far: float = 0.0
    heading_error: float = 0.0
    estimated_curvature: float = 0.0
    min_safe_clearance_m: float = 0.0
    left_boundary_position: float = float("nan")
    right_boundary_position: float = float("nan")
    raw_center_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    previous_center_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    filtered_center_y: np.ndarray = field(default_factory=lambda: np.zeros(0, dtype=np.float32))
    rejected_candidate_count: int = 0
    intersection_like: bool = False
    yellow_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    white_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    road_edge_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    scanline_left_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    scanline_right_points: np.ndarray = field(default_factory=lambda: np.zeros((0, 2), dtype=np.float32))
    left_lane_id: int = 1
    right_lane_id: int = 2
    observed_left: bool = False
    observed_right: bool = False
    search_mode: str = "near"
    real_lane_age_s: float = float("inf")
    debug: dict[str, Any] = field(default_factory=dict)
    debug_image: np.ndarray | None = None
    front_detection_image: np.ndarray | None = None
    bev_debug_image: np.ndarray | None = None
    mask_debug_image: np.ndarray | None = None

    @property
    def has_path(self) -> bool:
        return self.path_x.size > 0 and self.path_y.size == self.path_x.size


class LaneDetector:
    """Deterministic lane detector for the right lane.

    The detector works in a calibrated bird's-eye-view approximation. It uses
    yellow paint right edge as the left boundary of the target lane and white
    paint left edge, or the road right edge, as the right boundary.
    """

    def __init__(self, cfg: LaneVisionConfig) -> None:
        self.cfg = cfg
        self.estimated_lane_width_m = cfg.initial_lane_width_m
        self.last_left_fit: PolynomialFit | None = None
        self.last_right_fit: PolynomialFit | None = None
        self.last_capture_timestamp: float | None = None
        self.last_real_detection_timestamp: float | None = None
        self.last_center_x: np.ndarray | None = None
        self.last_center_y: np.ndarray | None = None
        self.last_curvature: float | None = None
        self.intersection_hold_until = 0.0
        self.left_lane_id = 1
        self.right_lane_id = 2
        self._homography_key: tuple[int, int, bool] | None = None
        self._homography: np.ndarray | None = None
        self._inv_homography: np.ndarray | None = None

    def process(
        self,
        bgr: np.ndarray,
        capture_timestamp: float,
        frame_id: int = 0,
        make_debug: bool = True,
    ) -> LaneDetectionResult:
        if bgr is None or bgr.size == 0:
            return LaneDetectionResult(frame_id=frame_id, capture_timestamp=capture_timestamp, state="empty_image")

        h, w = bgr.shape[:2]
        real_lane_age = self._real_lane_age(capture_timestamp)
        wide_search = math.isfinite(real_lane_age) and real_lane_age >= self.cfg.wide_search_after_s
        homography, inv_homography = self._get_homography(w, h, wide_search=wide_search)
        yellow_mask, white_mask, road_mask, roi_mask = self._make_masks(bgr, wide_search=wide_search)
        yellow_bev = cv2.warpPerspective(yellow_mask, homography, (self.cfg.bev_width_px, self.cfg.bev_height_px))
        white_bev = cv2.warpPerspective(white_mask, homography, (self.cfg.bev_width_px, self.cfg.bev_height_px))
        road_bev = cv2.warpPerspective(road_mask, homography, (self.cfg.bev_width_px, self.cfg.bev_height_px))

        yellow_bev, yellow_stats = self._filter_components(yellow_bev, color="yellow")
        white_bev, white_stats = self._filter_components(white_bev, color="white")
        road_bev, road_stats = self._filter_components(road_bev, color="road")
        component_stats = {
            "yellow": yellow_stats,
            "white": white_stats,
            "road": road_stats,
        }
        intersection_like = self._intersection_like(component_stats)
        if intersection_like:
            self.intersection_hold_until = max(
                self.intersection_hold_until,
                capture_timestamp + max(0.0, self.cfg.intersection_hold_s),
            )

        yellow_points = self._extract_best_scanline_chain(
            yellow_bev,
            boundary="yellow_right_edge",
            expected="left",
            previous=self.last_left_fit,
        )
        white_points = self._extract_best_scanline_chain(
            white_bev,
            boundary="white_left_edge",
            expected="right",
            previous=self.last_right_fit,
        )
        road_edge_points = self._extract_best_scanline_chain(
            road_bev,
            boundary="road_right_edge",
            expected="right",
            previous=self.last_right_fit,
        )

        yellow_points = self._filter_ground_points(yellow_points, expected="left")
        white_points = self._filter_ground_points(white_points, expected="right")
        road_edge_points = self._filter_ground_points(road_edge_points, expected="right")

        left_fit = fit_quadratic_ransac(yellow_points, self.cfg, seed=frame_id * 2 + 17)
        right_fit = fit_quadratic_ransac(white_points, self.cfg, seed=frame_id * 2 + 29)
        fit_reject_reasons: list[str] = []
        left_fit, reason = self._reject_fit_jump(left_fit, self.last_left_fit, capture_timestamp, side="left")
        if reason:
            fit_reject_reasons.append(reason)
        observed_left = left_fit is not None
        observed_right = right_fit is not None
        right_fit, reason = self._reject_fit_jump(right_fit, self.last_right_fit, capture_timestamp, side="right")
        if reason:
            fit_reject_reasons.append(reason)
        right_source = "white"
        if right_fit is None and road_edge_points.shape[0] >= self.cfg.fit_min_points:
            right_fit = fit_quadratic_ransac(road_edge_points, self.cfg, seed=frame_id * 2 + 41)
            right_source = "road_edge" if right_fit is not None else "none"
            observed_right = right_fit is not None
            right_fit, reason = self._reject_fit_jump(right_fit, self.last_right_fit, capture_timestamp, side="right")
            if reason:
                fit_reject_reasons.append(reason)
            observed_right = right_fit is not None

        result = self._build_path(
            frame_id=frame_id,
            capture_timestamp=capture_timestamp,
            left_fit=left_fit,
            right_fit=right_fit,
            right_source=right_source,
            yellow_points=yellow_points,
            white_points=white_points,
            road_edge_points=road_edge_points,
            component_stats=component_stats,
            fit_reject_reasons=fit_reject_reasons,
            intersection_like=intersection_like,
            observed_left=observed_left,
            observed_right=observed_right,
            wide_search=wide_search,
            real_lane_age=real_lane_age,
        )
        if make_debug:
            result.front_detection_image = self._make_front_detection_overlay(
                bgr=bgr,
                roi_mask=roi_mask,
                road_mask=road_mask,
                white_mask=white_mask,
                yellow_mask=yellow_mask,
                result=result,
                inv_homography=inv_homography,
            )
            result.bev_debug_image = self._make_bev_debug(yellow_bev, white_bev, road_bev, result)
            result.mask_debug_image = self._make_mask_debug(
                bgr=bgr,
                roi_mask=roi_mask,
                road_mask=road_mask,
                white_mask=white_mask,
                yellow_mask=yellow_mask,
                result=result,
                inv_homography=inv_homography,
            )
            result.debug_image = self._draw_debug(
                bgr=bgr,
                roi_mask=roi_mask,
                yellow_mask=yellow_mask,
                white_mask=white_mask,
                yellow_bev=yellow_bev,
                white_bev=white_bev,
                road_bev=road_bev,
                result=result,
                inv_homography=inv_homography,
                bev_debug=result.bev_debug_image,
            )
        return result

    def _build_path(
        self,
        frame_id: int,
        capture_timestamp: float,
        left_fit: PolynomialFit | None,
        right_fit: PolynomialFit | None,
        right_source: str,
        yellow_points: np.ndarray,
        white_points: np.ndarray,
        road_edge_points: np.ndarray,
        component_stats: dict[str, dict[str, Any]],
        fit_reject_reasons: list[str],
        intersection_like: bool,
        observed_left: bool,
        observed_right: bool,
        wide_search: bool,
        real_lane_age: float,
    ) -> LaneDetectionResult:
        state = "no_lane_stop"
        left_conf = left_fit.confidence if left_fit else 0.0
        right_conf = right_fit.confidence if right_fit else 0.0
        width_reject_reason = ""

        if left_fit is not None and right_fit is not None:
            measured_width = self._lane_width_at(left_fit, right_fit, x=3.0)
            width_ok = self.cfg.lane_width_min_m <= measured_width <= self.cfg.lane_width_max_m
            width_jump = abs(measured_width - self.estimated_lane_width_m) > self.cfg.max_lane_width_jump_m
            if width_ok and not width_jump:
                alpha = clamp(self.cfg.lane_width_alpha, 0.0, 1.0)
                self.estimated_lane_width_m = (
                    (1.0 - alpha) * self.estimated_lane_width_m + alpha * measured_width
                )
            else:
                width_reject_reason = "lane_width_range" if not width_ok else "lane_width_jump"
                if self._memory_available(capture_timestamp):
                    left_fit = self.last_left_fit
                    right_fit = self.last_right_fit
                    observed_left = False
                    observed_right = False
                    left_conf = (left_fit.confidence if left_fit else 0.0) * 0.40
                    right_conf = (right_fit.confidence if right_fit else 0.0) * 0.40
                    state = "predicted_previous_" + width_reject_reason
                elif left_conf <= right_conf:
                    left_fit = None
                    observed_left = False
                    left_conf = 0.0
                else:
                    right_fit = None
                    observed_right = False
                    right_conf = 0.0

        if state.startswith("predicted_previous_"):
            pass
        elif left_fit is not None and right_fit is not None:
            state = "both_boundaries"
        elif left_fit is not None:
            right_fit = left_fit.shifted(-self.estimated_lane_width_m)
            right_conf = right_fit.confidence
            state = "yellow_only_inferred_right"
        elif right_fit is not None:
            left_fit = right_fit.shifted(self.estimated_lane_width_m)
            left_conf = left_fit.confidence
            state = "white_only_inferred_left" if right_source == "white" else "road_edge_only_inferred_left"
        elif self._memory_available(capture_timestamp):
            left_fit = self.last_left_fit
            right_fit = self.last_right_fit
            observed_left = False
            observed_right = False
            left_conf = (left_fit.confidence if left_fit else 0.0) * 0.35
            right_conf = (right_fit.confidence if right_fit else 0.0) * 0.35
            state = "predicted_previous"

        rejected_count = int(sum(int(stats.get("rejected_total", 0)) for stats in component_stats.values()))

        result = LaneDetectionResult(
            frame_id=frame_id,
            capture_timestamp=capture_timestamp,
            state=state,
            left_fit=left_fit,
            right_fit=right_fit,
            left_lane_confidence=float(left_conf),
            right_lane_confidence=float(right_conf),
            estimated_lane_width=float(self.estimated_lane_width_m),
            rejected_candidate_count=rejected_count,
            intersection_like=intersection_like,
            yellow_points=yellow_points,
            white_points=white_points,
            road_edge_points=road_edge_points,
            scanline_left_points=yellow_points,
            scanline_right_points=white_points if white_points.size else road_edge_points,
            left_lane_id=self.left_lane_id,
            right_lane_id=self.right_lane_id,
            observed_left=bool(observed_left),
            observed_right=bool(observed_right),
            search_mode="wide" if wide_search else "near",
            real_lane_age_s=float(real_lane_age),
        )
        result.debug.update(
            {
                "component_stats": component_stats,
                "fit_reject_reasons": fit_reject_reasons,
                "width_reject_reason": width_reject_reason,
                "intersection_hold_until": self.intersection_hold_until,
                "lane_ids": {"left": self.left_lane_id, "right": self.right_lane_id},
                "observed": {"left": bool(observed_left), "right": bool(observed_right)},
                "search_mode": "wide" if wide_search else "near",
                "real_lane_age_s": None if not math.isfinite(real_lane_age) else float(real_lane_age),
            }
        )

        if left_fit is None or right_fit is None:
            result.overall_lane_confidence = 0.0
            return result

        xs = np.linspace(max(self.cfg.fit_min_x_m, 0.75), min(self.cfg.fit_max_x_m, self.cfg.ground_x_max_m), 48)
        left_y = np.asarray(left_fit.y(xs), dtype=np.float64)
        right_y = np.asarray(right_fit.y(xs), dtype=np.float64)
        valid_order = left_y > right_y
        if not np.any(valid_order):
            result.state = "invalid_boundary_order"
            result.overall_lane_confidence = 0.0
            return result

        clearance = 0.5 * self.cfg.vehicle_width_m + self.cfg.safety_margin_m
        safe_left = left_y - clearance
        safe_right = right_y + clearance
        corridor = safe_left - safe_right
        min_corridor = float(np.min(corridor))
        center_y = 0.5 * (safe_left + safe_right)

        if min_corridor < self.cfg.min_safe_corridor_m:
            result.state = "unsafe_corridor_" + state

        near_idx = int(np.argmin(np.abs(xs - 1.2)))
        far_idx = int(np.argmin(np.abs(xs - 4.5)))
        raw_center_y = center_y.copy()
        raw_curvature = self._curvature(raw_center_y, xs, far_idx)
        confidence = clamp(0.5 * left_conf + 0.5 * right_conf, 0.0, 1.0)
        if "inferred" in state:
            confidence *= 0.75
        if state.startswith("predicted_previous"):
            confidence *= 0.45
        if min_corridor < self.cfg.min_safe_corridor_m:
            confidence *= 0.40
        if wide_search:
            confidence *= clamp(self.cfg.wide_confidence_scale, 0.0, 1.0)

        center_y, confidence, state, tracking_debug = self._stabilize_center(
            xs=xs,
            center_y=center_y,
            confidence=confidence,
            curvature=raw_curvature,
            capture_timestamp=capture_timestamp,
            state=state,
            intersection_like=intersection_like,
        )
        dy_dx = float(np.gradient(center_y, xs)[near_idx]) if xs.size > 3 else 0.0
        curvature = self._curvature(center_y, xs, far_idx)

        result.sample_x = xs.astype(np.float32)
        result.path_x = xs.astype(np.float32)
        result.path_y = center_y.astype(np.float32)
        result.raw_center_y = raw_center_y.astype(np.float32)
        result.previous_center_y = tracking_debug.get("previous_center_y", np.zeros(0, dtype=np.float32))
        result.filtered_center_y = result.path_y.copy()
        result.safe_left_y = safe_left.astype(np.float32)
        result.safe_right_y = safe_right.astype(np.float32)
        result.overall_lane_confidence = float(confidence)
        result.lateral_error_near = float(center_y[near_idx])
        result.lateral_error_far = float(center_y[far_idx])
        result.heading_error = float(math.atan(dy_dx))
        result.estimated_curvature = float(curvature)
        result.min_safe_clearance_m = float(0.5 * min_corridor)
        result.left_boundary_position = float(left_y[near_idx])
        result.right_boundary_position = float(right_y[near_idx])
        if result.state.startswith("unsafe_corridor_"):
            result.state = "unsafe_corridor_" + state
        else:
            result.state = state
        result.debug.update(
            {
                "right_source": right_source,
                "clearance_m": clearance,
                "raw_curvature": raw_curvature,
                "tracking": tracking_debug,
            }
        )

        accepted_current = bool(tracking_debug.get("accepted_current", True))
        has_real_observation = bool(observed_left or observed_right)
        if accepted_current and has_real_observation:
            self.last_left_fit = left_fit
            self.last_right_fit = right_fit
            self.last_real_detection_timestamp = capture_timestamp
            result.real_lane_age_s = 0.0
            result.debug["real_lane_age_s"] = 0.0
        self.last_center_x = xs.astype(np.float64)
        self.last_center_y = center_y.astype(np.float64)
        self.last_curvature = curvature
        self.last_capture_timestamp = capture_timestamp
        return result

    def _real_lane_age(self, capture_timestamp: float) -> float:
        if self.last_real_detection_timestamp is None:
            return float("inf")
        return max(0.0, float(capture_timestamp - self.last_real_detection_timestamp))

    def _memory_available(self, capture_timestamp: float) -> bool:
        if self.last_real_detection_timestamp is None:
            return False
        if self.last_left_fit is None or self.last_right_fit is None:
            return False
        return (capture_timestamp - self.last_real_detection_timestamp) <= self.cfg.memory_timeout_s

    def _lane_width_at(self, left_fit: PolynomialFit, right_fit: PolynomialFit, x: float) -> float:
        return float(left_fit.y(x) - right_fit.y(x))

    def _curvature(self, y: np.ndarray, x: np.ndarray, idx: int) -> float:
        if x.size < 5:
            return 0.0
        idx = max(1, min(int(idx), x.size - 2))
        dy = np.gradient(y, x)
        ddy = np.gradient(dy, x)
        denom = (1.0 + dy[idx] * dy[idx]) ** 1.5
        return float(ddy[idx] / max(1e-6, denom))

    def _make_masks(
        self,
        bgr: np.ndarray,
        wide_search: bool = False,
    ) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        h, w = bgr.shape[:2]
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        roi_mask = np.zeros((h, w), dtype=np.uint8)
        y0 = int(clamp(self.cfg.roi_y0_ratio, 0.0, 0.95) * h)
        center_x = self.cfg.source_center_x_ratio * w
        top_y_ratio = self.cfg.wide_source_top_y_ratio if wide_search else self.cfg.source_top_y_ratio
        top_half_width_ratio = (
            self.cfg.wide_source_top_half_width_ratio if wide_search else self.cfg.source_top_half_width_ratio
        )
        top_y = int(top_y_ratio * h)
        bottom_y = int(self.cfg.source_bottom_y_ratio * h)
        top_hw = int(top_half_width_ratio * w)
        bottom_hw = int(self.cfg.source_bottom_half_width_ratio * w)
        pts = np.array(
            [
                [center_x - top_hw, max(y0, top_y)],
                [center_x + top_hw, max(y0, top_y)],
                [center_x + bottom_hw, bottom_y],
                [center_x - bottom_hw, bottom_y],
            ],
            dtype=np.int32,
        )
        cv2.fillConvexPoly(roi_mask, pts, 255)

        raw_yellow = cv2.inRange(
            hsv,
            np.array([self.cfg.yellow_h_min, self.cfg.yellow_s_min, self.cfg.yellow_v_min], dtype=np.uint8),
            np.array([self.cfg.yellow_h_max, 255, 255], dtype=np.uint8),
        )
        raw_white = cv2.inRange(
            hsv,
            np.array([0, 0, self.cfg.white_v_min], dtype=np.uint8),
            np.array([180, self.cfg.white_s_max, 255], dtype=np.uint8),
        )
        road = cv2.inRange(
            hsv,
            np.array([0, 0, self.cfg.road_v_min], dtype=np.uint8),
            np.array([180, self.cfg.road_s_max, self.cfg.road_v_max], dtype=np.uint8),
        )
        road = cv2.bitwise_and(road, roi_mask)

        k = max(1, int(self.cfg.morphology_kernel_px))
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (k, k))
        road = cv2.morphologyEx(road, cv2.MORPH_OPEN, kernel)
        road = cv2.morphologyEx(road, cv2.MORPH_CLOSE, kernel)

        envelope_kernel_size = max(7, int(self.cfg.morphology_kernel_px) * 5)
        if envelope_kernel_size % 2 == 0:
            envelope_kernel_size += 1
        envelope_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (envelope_kernel_size, envelope_kernel_size))
        road_envelope = cv2.dilate(road, envelope_kernel, iterations=1)
        road_envelope = cv2.bitwise_and(road_envelope, roi_mask)

        yellow = cv2.bitwise_and(raw_yellow, road_envelope)
        white = cv2.bitwise_and(raw_white, road_envelope)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, kernel)
        yellow = cv2.morphologyEx(yellow, cv2.MORPH_CLOSE, kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_OPEN, kernel)
        white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, kernel)
        return yellow, white, road, roi_mask

    def _filter_components(self, mask: np.ndarray, color: str) -> tuple[np.ndarray, dict[str, Any]]:
        num, labels, stats, centroids = cv2.connectedComponentsWithStats((mask > 0).astype(np.uint8), 8)
        out = np.zeros_like(mask)
        debug_stats: dict[str, Any] = {
            "input_count": int(max(0, num - 1)),
            "kept_count": 0,
            "horizontal_count": 0,
            "rejected_total": 0,
            "rejected": {
                "area": 0,
                "size": 0,
                "horizontal": 0,
                "position": 0,
            },
        }
        for label in range(1, num):
            x, y, w, h, area = stats[label]
            if area < self.cfg.min_component_area_px or area > self.cfg.max_component_area_px:
                debug_stats["rejected"]["area"] += 1
                continue
            if h < self.cfg.min_component_height_px or w < self.cfg.min_component_width_px:
                debug_stats["rejected"]["size"] += 1
                continue
            aspect = float(w) / max(1.0, float(h))
            if (
                color in {"yellow", "white"}
                and area >= self.cfg.horizontal_component_min_area_px
                and aspect >= self.cfg.horizontal_component_aspect_ratio
            ):
                debug_stats["horizontal_count"] += 1
                debug_stats["rejected"]["horizontal"] += 1
                continue
            cx, cy = centroids[label]
            gx, gy = self._bev_pixel_to_ground(float(cx), float(cy))
            if color == "yellow" and gy < -0.35:
                debug_stats["rejected"]["position"] += 1
                continue
            if color == "white" and gy > 0.65:
                debug_stats["rejected"]["position"] += 1
                continue
            if color == "road" and area < self.cfg.min_component_area_px * 3:
                debug_stats["rejected"]["area"] += 1
                continue
            out[labels == label] = 255
            debug_stats["kept_count"] += 1
        debug_stats["rejected_total"] = int(sum(debug_stats["rejected"].values()))
        return out, debug_stats

    def _intersection_like(self, component_stats: dict[str, dict[str, Any]]) -> bool:
        yellow = component_stats.get("yellow", {})
        white = component_stats.get("white", {})
        horizontal = int(yellow.get("horizontal_count", 0)) + int(white.get("horizontal_count", 0))
        candidates = int(yellow.get("input_count", 0)) + int(white.get("input_count", 0))
        return (
            horizontal >= max(1, self.cfg.intersection_horizontal_count)
            or candidates >= max(1, self.cfg.intersection_component_count)
        )

    def _reject_fit_jump(
        self,
        fit: PolynomialFit | None,
        previous: PolynomialFit | None,
        capture_timestamp: float,
        side: str,
    ) -> tuple[PolynomialFit | None, str]:
        if fit is None or previous is None or not self._memory_available(capture_timestamp):
            return fit, ""
        xs = np.asarray([1.2, 3.0, 5.5], dtype=np.float64)
        xs = xs[(xs >= self.cfg.fit_min_x_m) & (xs <= self.cfg.fit_max_x_m)]
        if xs.size == 0:
            return fit, ""
        offset = float(np.max(np.abs(np.asarray(fit.y(xs)) - np.asarray(previous.y(xs)))))
        if offset > self.cfg.previous_fit_max_offset_m:
            return None, f"{side}_fit_jump_{offset:.2f}m"
        return fit, ""

    def _stabilize_center(
        self,
        xs: np.ndarray,
        center_y: np.ndarray,
        confidence: float,
        curvature: float,
        capture_timestamp: float,
        state: str,
        intersection_like: bool,
    ) -> tuple[np.ndarray, float, str, dict[str, Any]]:
        debug: dict[str, Any] = {
            "accepted_current": True,
            "reject_reasons": [],
            "previous_center_y": np.zeros(0, dtype=np.float32),
            "alpha": 1.0,
            "max_step_m": 0.0,
        }
        if self.last_center_x is None or self.last_center_y is None or not self._memory_available(capture_timestamp):
            return center_y, confidence, state, debug

        previous_y = np.interp(xs, self.last_center_x, self.last_center_y)
        debug["previous_center_y"] = previous_y.astype(np.float32)
        near_idx = int(np.argmin(np.abs(xs - 1.2)))
        center_jump = abs(float(center_y[near_idx] - previous_y[near_idx]))
        curvature_jump = 0.0 if self.last_curvature is None else abs(float(curvature - self.last_curvature))
        hold_active = capture_timestamp <= self.intersection_hold_until

        if intersection_like or hold_active:
            debug["reject_reasons"].append("intersection_hold")
        if center_jump > self.cfg.max_center_jump_m:
            debug["reject_reasons"].append(f"center_jump_{center_jump:.2f}m")
        if curvature_jump > self.cfg.max_curvature_jump:
            debug["reject_reasons"].append(f"curvature_jump_{curvature_jump:.3f}")

        if debug["reject_reasons"]:
            debug["accepted_current"] = False
            confidence *= 0.40
            reason = str(debug["reject_reasons"][0])
            return previous_y, confidence, reason + "_" + state, debug

        alpha = self.cfg.center_filter_alpha if confidence >= 0.50 else self.cfg.low_confidence_center_filter_alpha
        max_step = self.cfg.max_center_step_m if confidence >= 0.50 else self.cfg.low_confidence_max_center_step_m
        alpha = clamp(alpha, 0.0, 1.0)
        max_step = max(0.0, float(max_step))
        stepped_y = previous_y + np.clip(center_y - previous_y, -max_step, max_step)
        filtered_y = alpha * stepped_y + (1.0 - alpha) * previous_y
        debug["alpha"] = float(alpha)
        debug["max_step_m"] = float(max_step)
        debug["center_jump_m"] = float(center_jump)
        debug["curvature_jump"] = float(curvature_jump)
        return filtered_y, confidence, state, debug

    def _extract_best_scanline_chain(
        self,
        bev_mask: np.ndarray,
        boundary: str,
        expected: str,
        previous: PolynomialFit | None,
    ) -> np.ndarray:
        """Link sparse horizontal scanline hits into a continuous lane chain.

        Every contiguous run on a sampled BEV row is a candidate.  Candidates
        are associated from the near field toward the far field using a
        constant-slope prediction, so a curve keeps the same identity even
        after it crosses the image centre.  Only the best connected chain is
        sent to RANSAC.
        """
        h, _w = bev_mask.shape[:2]
        step = max(2, int(self.cfg.scanline_step_px))
        max_missed = max(0, int(self.cfg.scanline_max_missed_slices))
        active: list[dict[str, Any]] = []
        completed: list[dict[str, Any]] = []

        for v in range(h - 1, -1, -step):
            cols = np.flatnonzero(bev_mask[v] > 0)
            runs: list[np.ndarray] = []
            if cols.size:
                split_at = np.flatnonzero(np.diff(cols) > 1) + 1
                runs = [run for run in np.split(cols, split_at) if run.size]

            candidates: list[tuple[float, float]] = []
            for run in runs:
                if boundary in {"yellow_right_edge", "road_right_edge"}:
                    u = float(run[-1])
                elif boundary == "white_left_edge":
                    u = float(run[0])
                else:
                    continue
                x, y = self._bev_pixel_to_ground(u, float(v))
                if self.cfg.fit_min_x_m <= x <= self.cfg.fit_max_x_m:
                    candidates.append((x, y))

            for chain in active:
                chain["missed"] += 1

            unmatched = set(range(len(candidates)))
            proposals: list[tuple[float, int, int]] = []
            for chain_index, chain in enumerate(active):
                points = chain["points"]
                last_x, last_y = points[-1]
                slope = 0.0
                if len(points) >= 2:
                    prev_x, prev_y = points[-2]
                    slope = (last_y - prev_y) / max(1e-6, last_x - prev_x)
                for candidate_index, (x, y) in enumerate(candidates):
                    predicted_y = last_y + slope * (x - last_x)
                    gap = abs(y - predicted_y)
                    allowed = self.cfg.scanline_link_max_gap_m * (1.0 + 0.18 * chain["missed"])
                    if gap <= allowed:
                        proposals.append((gap, chain_index, candidate_index))

            used_chains: set[int] = set()
            for _gap, chain_index, candidate_index in sorted(proposals):
                if chain_index in used_chains or candidate_index not in unmatched:
                    continue
                active[chain_index]["points"].append(candidates[candidate_index])
                active[chain_index]["missed"] = 0
                used_chains.add(chain_index)
                unmatched.remove(candidate_index)

            for candidate_index in sorted(unmatched):
                active.append({"points": [candidates[candidate_index]], "missed": 0})

            survivors: list[dict[str, Any]] = []
            for chain in active:
                if chain["missed"] > max_missed:
                    completed.append(chain)
                else:
                    survivors.append(chain)
            active = survivors

        completed.extend(active)
        min_slices = max(2, int(self.cfg.scanline_min_slices))
        chains = [np.asarray(chain["points"], dtype=np.float32) for chain in completed if len(chain["points"]) >= min_slices]
        if not chains:
            return np.zeros((0, 2), dtype=np.float32)

        expected_y = 0.5 * self.estimated_lane_width_m if expected == "left" else -0.5 * self.estimated_lane_width_m

        def chain_score(points: np.ndarray) -> float:
            order = np.argsort(points[:, 0])
            points = points[order]
            span = float(np.ptp(points[:, 0])) if points.shape[0] > 1 else 0.0
            near_index = int(np.argmin(np.abs(points[:, 0] - 1.2)))
            score = -2.0 * span - 0.015 * float(points.shape[0]) + 0.35 * abs(float(points[near_index, 1]) - expected_y)
            if previous is not None and self.last_real_detection_timestamp is not None:
                predicted = np.asarray(previous.y(points[:, 0]), dtype=np.float64)
                score += 3.5 * float(np.median(np.abs(points[:, 1] - predicted)))
            return score

        best = min(chains, key=chain_score)
        return best[np.argsort(best[:, 0])].astype(np.float32)

    def _extract_boundary_points(self, bev_mask: np.ndarray, boundary: str) -> np.ndarray:
        h, _w = bev_mask.shape[:2]
        points: list[tuple[float, float]] = []
        step = max(1, int(self.cfg.boundary_row_step_px))
        for v in range(0, h, step):
            cols = np.flatnonzero(bev_mask[v] > 0)
            if cols.size == 0:
                continue
            if boundary == "yellow_right_edge":
                u = float(np.max(cols))
            elif boundary == "white_left_edge":
                center_u = self.cfg.bev_width_px * 0.5
                cols = cols[cols >= center_u - self.cfg.bev_width_px * 0.08]
                if cols.size == 0:
                    continue
                u = float(np.min(cols))
            elif boundary == "road_right_edge":
                u = float(np.max(cols))
            else:
                continue
            x, y = self._bev_pixel_to_ground(u, float(v))
            if self.cfg.fit_min_x_m <= x <= self.cfg.fit_max_x_m:
                points.append((x, y))
        if not points:
            return np.zeros((0, 2), dtype=np.float32)
        return np.asarray(points, dtype=np.float32)

    def _filter_ground_points(self, points: np.ndarray, expected: str) -> np.ndarray:
        if points.size == 0:
            return points.reshape((0, 2)).astype(np.float32)
        x = points[:, 0]
        y = points[:, 1]
        valid = np.isfinite(x) & np.isfinite(y)
        if expected == "left":
            valid &= y > -0.45
        elif expected == "right":
            valid &= y < 0.80
        valid &= x >= self.cfg.fit_min_x_m
        valid &= x <= self.cfg.fit_max_x_m
        return points[valid].astype(np.float32)

    def _get_homography(
        self,
        image_w: int,
        image_h: int,
        wide_search: bool = False,
    ) -> tuple[np.ndarray, np.ndarray]:
        key = (int(image_w), int(image_h), bool(wide_search))
        if self._homography_key == key and self._homography is not None and self._inv_homography is not None:
            return self._homography, self._inv_homography

        cx = self.cfg.source_center_x_ratio * image_w
        top_y_ratio = self.cfg.wide_source_top_y_ratio if wide_search else self.cfg.source_top_y_ratio
        top_half_width_ratio = (
            self.cfg.wide_source_top_half_width_ratio if wide_search else self.cfg.source_top_half_width_ratio
        )
        top_y = top_y_ratio * image_h
        bottom_y = self.cfg.source_bottom_y_ratio * image_h
        top_hw = top_half_width_ratio * image_w
        bottom_hw = self.cfg.source_bottom_half_width_ratio * image_w
        src = np.float32(
            [
                [cx - top_hw, top_y],
                [cx + top_hw, top_y],
                [cx + bottom_hw, bottom_y],
                [cx - bottom_hw, bottom_y],
            ]
        )
        dst = np.float32(
            [
                [0.0, 0.0],
                [self.cfg.bev_width_px - 1.0, 0.0],
                [self.cfg.bev_width_px - 1.0, self.cfg.bev_height_px - 1.0],
                [0.0, self.cfg.bev_height_px - 1.0],
            ]
        )
        self._homography = cv2.getPerspectiveTransform(src, dst)
        self._inv_homography = cv2.getPerspectiveTransform(dst, src)
        self._homography_key = key
        return self._homography, self._inv_homography

    def _bev_pixel_to_ground(self, u: float, v: float) -> tuple[float, float]:
        w = max(1.0, float(self.cfg.bev_width_px - 1))
        h = max(1.0, float(self.cfg.bev_height_px - 1))
        x = self.cfg.ground_x_max_m * (1.0 - v / h)
        y = self.cfg.ground_y_left_m + (u / w) * (self.cfg.ground_y_right_m - self.cfg.ground_y_left_m)
        return float(x), float(y)

    def _ground_to_bev_pixel(self, x: float, y: float) -> tuple[float, float]:
        w = max(1.0, float(self.cfg.bev_width_px - 1))
        h = max(1.0, float(self.cfg.bev_height_px - 1))
        u = (y - self.cfg.ground_y_left_m) / (self.cfg.ground_y_right_m - self.cfg.ground_y_left_m) * w
        v = (1.0 - x / max(1e-6, self.cfg.ground_x_max_m)) * h
        return float(u), float(v)

    def _ground_polyline_to_image(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        inv_homography: np.ndarray,
    ) -> np.ndarray:
        pts = []
        for x, y in zip(xs, ys):
            u, v = self._ground_to_bev_pixel(float(x), float(y))
            pts.append([u, v])
        if not pts:
            return np.zeros((0, 2), dtype=np.int32)
        bev_pts = np.asarray(pts, dtype=np.float32).reshape((-1, 1, 2))
        img_pts = cv2.perspectiveTransform(bev_pts, inv_homography).reshape((-1, 2))
        return np.round(img_pts).astype(np.int32)

    def _draw_debug(
        self,
        bgr: np.ndarray,
        roi_mask: np.ndarray,
        yellow_mask: np.ndarray,
        white_mask: np.ndarray,
        yellow_bev: np.ndarray,
        white_bev: np.ndarray,
        road_bev: np.ndarray,
        result: LaneDetectionResult,
        inv_homography: np.ndarray,
        bev_debug: np.ndarray | None = None,
    ) -> np.ndarray:
        overlay = bgr.copy()
        roi_tint = np.zeros_like(overlay)
        roi_tint[:, :, 1] = roi_mask
        overlay = cv2.addWeighted(overlay, 1.0, roi_tint, 0.12, 0.0)

        yellow_color = np.zeros_like(overlay)
        yellow_color[:, :, 1] = yellow_mask
        yellow_color[:, :, 2] = yellow_mask
        white_color = cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR)
        overlay = cv2.addWeighted(overlay, 1.0, yellow_color, 0.35, 0.0)
        overlay = cv2.addWeighted(overlay, 1.0, white_color, 0.25, 0.0)

        if result.left_fit is not None and result.sample_x.size:
            ys = np.asarray(result.left_fit.y(result.sample_x), dtype=np.float32)
            pts = self._ground_polyline_to_image(result.sample_x, ys, inv_homography)
            if pts.shape[0] > 1:
                cv2.polylines(overlay, [pts], False, (0, 220, 255), 2, cv2.LINE_AA)
        if result.right_fit is not None and result.sample_x.size:
            ys = np.asarray(result.right_fit.y(result.sample_x), dtype=np.float32)
            pts = self._ground_polyline_to_image(result.sample_x, ys, inv_homography)
            if pts.shape[0] > 1:
                cv2.polylines(overlay, [pts], False, (255, 255, 255), 2, cv2.LINE_AA)
        if result.sample_x.size:
            left_pts = self._ground_polyline_to_image(result.sample_x, result.safe_left_y, inv_homography)
            right_pts = self._ground_polyline_to_image(result.sample_x, result.safe_right_y, inv_homography)
            raw_center_pts = self._ground_polyline_to_image(result.sample_x, result.raw_center_y, inv_homography)
            previous_center_pts = self._ground_polyline_to_image(result.sample_x, result.previous_center_y, inv_homography)
            center_pts = self._ground_polyline_to_image(result.path_x, result.path_y, inv_homography)
            if left_pts.shape[0] > 1:
                cv2.polylines(overlay, [left_pts], False, (0, 160, 255), 1, cv2.LINE_AA)
            if right_pts.shape[0] > 1:
                cv2.polylines(overlay, [right_pts], False, (255, 180, 80), 1, cv2.LINE_AA)
            if raw_center_pts.shape[0] > 1:
                cv2.polylines(overlay, [raw_center_pts], False, (0, 80, 255), 1, cv2.LINE_AA)
            if previous_center_pts.shape[0] > 1:
                cv2.polylines(overlay, [previous_center_pts], False, (255, 0, 255), 1, cv2.LINE_AA)
            if center_pts.shape[0] > 1:
                cv2.polylines(overlay, [center_pts], False, (0, 255, 80), 3, cv2.LINE_AA)

        self._draw_scanline_lane_id(
            overlay,
            result.scanline_left_points,
            inv_homography,
            label=f"L#{result.left_lane_id}",
            color=(0, 210, 255),
            observed=result.observed_left,
        )
        self._draw_scanline_lane_id(
            overlay,
            result.scanline_right_points,
            inv_homography,
            label=f"R#{result.right_lane_id}",
            color=(255, 210, 80),
            observed=result.observed_right,
        )

        text = (
            f"{result.state} conf={result.overall_lane_confidence:.2f} "
            f"width={result.estimated_lane_width:.2f}m err={result.lateral_error_near:+.2f}m"
        )
        cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (255, 255, 255), 1, cv2.LINE_AA)
        reasons = ",".join(result.debug.get("tracking", {}).get("reject_reasons", []))
        text2 = (
            f"search={result.search_mode} ids=L{result.left_lane_id}/R{result.right_lane_id} "
            f"intersection={int(result.intersection_like)} rejected={result.rejected_candidate_count} "
            f"reasons={reasons[:48]}"
        )
        cv2.putText(overlay, text2, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, text2, (12, 54), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (220, 255, 220), 1, cv2.LINE_AA)

        if bev_debug is None:
            bev_debug = self._make_bev_debug(yellow_bev, white_bev, road_bev, result)
        bev_inset = cv2.resize(bev_debug, (overlay.shape[1] // 3, overlay.shape[0] // 2))
        y0 = overlay.shape[0] - bev_inset.shape[0] - 8
        x0 = overlay.shape[1] - bev_inset.shape[1] - 8
        overlay[y0 : y0 + bev_inset.shape[0], x0 : x0 + bev_inset.shape[1]] = bev_inset
        return overlay

    def overlay_vehicle_prediction(
        self,
        image: np.ndarray | None,
        curvature: float,
        color: tuple[int, int, int] = (255, 180, 0),
    ) -> np.ndarray | None:
        """Draw the current steering trajectory like a reversing-camera guide."""
        if image is None:
            return None
        xs: list[float] = []
        ys: list[float] = []
        for distance in np.linspace(0.2, min(5.5, self.cfg.ground_x_max_m), 44):
            if abs(curvature) < 1e-5:
                x = float(distance)
                y = 0.0
            else:
                theta = float(curvature * distance)
                x = float(math.sin(theta) / curvature)
                y = float((1.0 - math.cos(theta)) / curvature)
            xs.append(x)
            ys.append(y)
        _homography, inv_homography = self._get_homography(
            image.shape[1],
            image.shape[0],
            wide_search=self._real_lane_age(self.last_capture_timestamp or 0.0) >= self.cfg.wide_search_after_s,
        )
        points = self._ground_polyline_to_image(np.asarray(xs), np.asarray(ys), inv_homography)
        visible = points[
            (points[:, 0] >= 0)
            & (points[:, 0] < image.shape[1])
            & (points[:, 1] >= 0)
            & (points[:, 1] < image.shape[0])
        ]
        if visible.shape[0] > 1:
            cv2.polylines(image, [visible], False, color, 3, cv2.LINE_AA)
            cv2.putText(image, "vehicle prediction", (12, image.shape[0] - 18), cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 1, cv2.LINE_AA)
        return image

    def _draw_contours(
        self,
        image: np.ndarray,
        mask: np.ndarray,
        color: tuple[int, int, int],
        thickness: int,
        min_area: int,
    ) -> int:
        contours, _hierarchy = cv2.findContours((mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        kept = 0
        for contour in contours:
            if cv2.contourArea(contour) < min_area:
                continue
            cv2.drawContours(image, [contour], -1, color, thickness, cv2.LINE_AA)
            kept += 1
        return kept

    def _make_front_detection_overlay(
        self,
        bgr: np.ndarray,
        roi_mask: np.ndarray,
        road_mask: np.ndarray,
        white_mask: np.ndarray,
        yellow_mask: np.ndarray,
        result: LaneDetectionResult,
        inv_homography: np.ndarray,
    ) -> np.ndarray:
        overlay = bgr.copy()
        tint = np.zeros_like(overlay)
        tint[:, :, 1] = np.maximum(tint[:, :, 1], (road_mask > 0).astype(np.uint8) * 120)
        tint[:, :, 0] = np.maximum(tint[:, :, 0], (white_mask > 0).astype(np.uint8) * 230)
        tint[:, :, 1] = np.maximum(tint[:, :, 1], (white_mask > 0).astype(np.uint8) * 230)
        tint[:, :, 2] = np.maximum(tint[:, :, 2], (white_mask > 0).astype(np.uint8) * 230)
        tint[:, :, 1] = np.maximum(tint[:, :, 1], (yellow_mask > 0).astype(np.uint8) * 220)
        tint[:, :, 2] = np.maximum(tint[:, :, 2], (yellow_mask > 0).astype(np.uint8) * 255)
        overlay = cv2.addWeighted(overlay, 0.72, tint, 0.44, 0.0)

        roi_contours = self._draw_contours(overlay, roi_mask, (80, 160, 80), 1, 200)
        road_contours = self._draw_contours(overlay, road_mask, (60, 220, 120), 3, 350)
        white_contours = self._draw_contours(overlay, white_mask, (255, 255, 255), 2, 18)
        yellow_contours = self._draw_contours(overlay, yellow_mask, (0, 220, 255), 2, 18)

        self._draw_scanline_lane_id(
            overlay,
            result.scanline_left_points,
            inv_homography,
            label=f"L#{result.left_lane_id}",
            color=(0, 210, 255),
            observed=result.observed_left,
        )
        self._draw_scanline_lane_id(
            overlay,
            result.scanline_right_points,
            inv_homography,
            label=f"R#{result.right_lane_id}",
            color=(255, 210, 80),
            observed=result.observed_right,
        )

        text = (
            f"road={road_contours} white={white_contours} yellow={yellow_contours} "
            f"roi={roi_contours} state={result.state} search={result.search_mode}"
        )
        cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, text, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (235, 255, 235), 1, cv2.LINE_AA)
        return overlay

    def _make_mask_debug(
        self,
        bgr: np.ndarray,
        roi_mask: np.ndarray,
        road_mask: np.ndarray,
        white_mask: np.ndarray,
        yellow_mask: np.ndarray,
        result: LaneDetectionResult,
        inv_homography: np.ndarray,
    ) -> np.ndarray:
        mask_view = np.zeros_like(bgr)
        mask_view[:, :, 1] = np.maximum(mask_view[:, :, 1], (road_mask > 0).astype(np.uint8) * 80)
        white = (white_mask > 0).astype(np.uint8) * 255
        mask_view = np.maximum(mask_view, cv2.cvtColor(white, cv2.COLOR_GRAY2BGR))
        mask_view[:, :, 1] = np.maximum(mask_view[:, :, 1], (yellow_mask > 0).astype(np.uint8) * 220)
        mask_view[:, :, 2] = np.maximum(mask_view[:, :, 2], (yellow_mask > 0).astype(np.uint8) * 255)
        roi_contours, _ = cv2.findContours((roi_mask > 0).astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(mask_view, roi_contours, -1, (80, 130, 80), 1, cv2.LINE_AA)
        self._draw_scanline_lane_id(
            mask_view,
            result.scanline_left_points,
            inv_homography,
            label=f"L#{result.left_lane_id}",
            color=(0, 210, 255),
            observed=result.observed_left,
        )
        self._draw_scanline_lane_id(
            mask_view,
            result.scanline_right_points,
            inv_homography,
            label=f"R#{result.right_lane_id}",
            color=(255, 210, 80),
            observed=result.observed_right,
        )
        cv2.putText(
            mask_view,
            f"MASK / SCANLINES  search={result.search_mode}",
            (12, 26),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (230, 255, 230),
            1,
            cv2.LINE_AA,
        )
        return mask_view

    def _draw_scanline_lane_id(
        self,
        image: np.ndarray,
        points: np.ndarray,
        inv_homography: np.ndarray,
        label: str,
        color: tuple[int, int, int],
        observed: bool,
    ) -> None:
        if points.size == 0:
            return
        projected = self._ground_polyline_to_image(points[:, 0], points[:, 1], inv_homography)
        h, w = image.shape[:2]
        visible: list[tuple[int, int]] = []
        for index, (u, v) in enumerate(projected):
            if not (0 <= u < w and 0 <= v < h):
                continue
            visible.append((int(u), int(v)))
            if index % max(1, len(projected) // 18) == 0:
                cv2.line(image, (max(0, int(u) - 14), int(v)), (min(w - 1, int(u) + 14), int(v)), color, 1)
                cv2.circle(image, (int(u), int(v)), 3, color, -1, cv2.LINE_AA)
        if len(visible) > 1:
            cv2.polylines(image, [np.asarray(visible, dtype=np.int32)], False, color, 2, cv2.LINE_AA)
            anchor = max(visible, key=lambda point: point[1])
            text = label if observed else label + " inferred"
            cv2.putText(image, text, (anchor[0] + 8, max(20, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(image, text, (anchor[0] + 8, max(20, anchor[1] - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)

    def _make_bev_debug(
        self,
        yellow_bev: np.ndarray,
        white_bev: np.ndarray,
        road_bev: np.ndarray,
        result: LaneDetectionResult,
    ) -> np.ndarray:
        bev = np.zeros((self.cfg.bev_height_px, self.cfg.bev_width_px, 3), dtype=np.uint8)
        bev[:, :, 1] = np.maximum(bev[:, :, 1], (road_bev > 0).astype(np.uint8) * 50)
        bev[:, :, 1] = np.maximum(bev[:, :, 1], (yellow_bev > 0).astype(np.uint8) * 220)
        bev[:, :, 2] = np.maximum(bev[:, :, 2], (yellow_bev > 0).astype(np.uint8) * 220)
        bev = np.maximum(bev, cv2.cvtColor(white_bev, cv2.COLOR_GRAY2BGR))

        def draw_ground_points(points: np.ndarray, color: tuple[int, int, int]) -> None:
            for x, y in points[:: max(1, len(points) // 150) if len(points) else 1]:
                u, v = self._ground_to_bev_pixel(float(x), float(y))
                cv2.circle(bev, (int(round(u)), int(round(v))), 2, color, -1)

        draw_ground_points(result.yellow_points, (0, 180, 255))
        draw_ground_points(result.white_points, (230, 230, 230))
        draw_ground_points(result.road_edge_points, (255, 100, 0))

        if result.sample_x.size:
            for ys, color, thickness in [
                (result.raw_center_y, (0, 80, 255), 1),
                (result.previous_center_y, (255, 0, 255), 1),
                (result.safe_left_y, (0, 180, 255), 1),
                (result.safe_right_y, (255, 170, 60), 1),
                (result.path_y, (0, 255, 80), 2),
            ]:
                pts = []
                for x, y in zip(result.sample_x, ys):
                    u, v = self._ground_to_bev_pixel(float(x), float(y))
                    pts.append([int(round(u)), int(round(v))])
                if len(pts) > 1:
                    cv2.polylines(bev, [np.asarray(pts, dtype=np.int32)], False, color, thickness, cv2.LINE_AA)
        return bev
