from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np

from .preprocessor import AuturboPreprocessor, LanePixels


@dataclass
class LaneResult:
    target_x: int
    angle_error: float
    confidence: float
    state: str
    debug_image: np.ndarray | None
    bev_image: np.ndarray | None
    mask_image: np.ndarray | None
    curve_score: float = 999.0
    left_count: int = 0
    mid_count: int = 0
    right_count: int = 0


class LaneDetector:
    def __init__(
        self,
        image_width: int = 640,
        obstacle_margin_px: int = 70,
        side_margin_px: int = 140,
        min_lane_pair_width_px: int = 120,
        threshold_mode: str = "auturbo_white_hsv",
        white_s_max: int = 35,
        white_v_min: int = 190,
        target_lookahead_y_px: int = 75,
        visibility_mode: str = "all",
    ) -> None:
        self.image_width = image_width
        self.obstacle_margin_px = obstacle_margin_px
        self.side_margin_px = side_margin_px
        self.min_lane_pair_width_px = min_lane_pair_width_px
        self.target_lookahead_y_px = int(target_lookahead_y_px)
        self.visibility_mode = str(visibility_mode)
        self.preprocessor = AuturboPreprocessor(
            roi_width=image_width,
            threshold_mode=threshold_mode,
            white_s_max=white_s_max,
            white_v_min=white_v_min,
        )
        self.last_target_x = image_width // 2

    def detect(
        self,
        frame: np.ndarray,
        obstacle_info: str = "middle",
        make_debug: bool = True,
        target_offset_px: int = 0,
        bev_status_text: str = "",
    ) -> LaneResult:
        if frame is None or frame.size == 0:
            return LaneResult(self.last_target_x, 0.0, 0.0, "no_image", None, None, None)

        mask, bev, lane_pixels = self.preprocessor.process(frame)
        lane_pixels = self._apply_visibility_mode(lane_pixels)
        base_target_x = self._target_from_lanes(lane_pixels, obstacle_info)
        target_x = int(np.clip(base_target_x + int(target_offset_px), 0, self.image_width - 1))
        curve_score = self._curve_score(lane_pixels)
        self.last_target_x = target_x
        center_x = self.image_width // 2
        raw_error = target_x - center_x
        angle_error = self._map(raw_error, -100.0, 100.0, -50.0, 50.0)
        confidence = self._confidence(lane_pixels)
        state = "lane_found" if confidence > 0.25 else "low_confidence"

        debug_image = None
        bev_image = lane_pixels.debug_image
        if make_debug:
            debug_image = frame.copy()
            bev_image = lane_pixels.debug_image.copy()
            target_y = self._target_debug_y(lane_pixels, bev_image.shape[0])
            frame_target = self.preprocessor.bev_to_frame_point(target_x, target_y, frame.shape)
            cv2.circle(debug_image, frame_target, 8, (120, 0, 255), -1)
            cv2.line(bev_image, (center_x, max(0, target_y - 20)), (center_x, min(bev_image.shape[0] - 1, target_y + 20)), (0, 255, 255), 1)
            cv2.circle(bev_image, (int(target_x), int(target_y)), 4, (120, 0, 255), -1)
            cv2.putText(
                bev_image,
                f"target={target_x} y={target_y}",
                (max(0, int(target_x) - 60), max(14, int(target_y) - 8)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.45,
                (120, 0, 255),
                1,
                cv2.LINE_AA,
            )
            if bev_status_text:
                text_x = int(np.clip(int(target_x) - 120, 0, max(0, bev_image.shape[1] - 300)))
                text_y = int(np.clip(int(target_y) + 16, 14, max(14, bev_image.shape[0] - 6)))
                cv2.putText(
                    bev_image,
                    bev_status_text[:120],
                    (text_x, text_y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.42,
                    (0, 255, 255),
                    1,
                    cv2.LINE_AA,
                )
            cv2.putText(
                debug_image,
                f"target={target_x} err={angle_error:.1f} conf={confidence:.2f} curve={curve_score:.0f} obs={obstacle_info} off={int(target_offset_px)}",
                (12, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

        return LaneResult(
            target_x=target_x,
            angle_error=float(angle_error),
            confidence=float(confidence),
            state=state,
            debug_image=debug_image,
            bev_image=bev_image if make_debug else None,
            mask_image=cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR) if make_debug else None,
            curve_score=float(curve_score),
            left_count=len(lane_pixels.left_x),
            mid_count=len(lane_pixels.mid_x),
            right_count=len(lane_pixels.right_x),
        )

    def _apply_visibility_mode(self, lanes: LanePixels) -> LanePixels:
        mode = self.visibility_mode.strip().lower()
        if mode in {"all", "normal", ""}:
            return lanes

        left_x, left_y = lanes.left_x, lanes.left_y
        mid_x, mid_y = lanes.mid_x, lanes.mid_y
        right_x, right_y = lanes.right_x, lanes.right_y

        if mode in {"left_only", "left"}:
            mid_x, mid_y = [], []
            right_x, right_y = [], []
        elif mode in {"middle_only", "mid_only", "middle", "mid"}:
            left_x, left_y = [], []
            right_x, right_y = [], []
        elif mode in {"right_only", "right"}:
            left_x, left_y = [], []
            mid_x, mid_y = [], []
        else:
            return lanes

        debug = cv2.cvtColor(cv2.cvtColor(lanes.debug_image, cv2.COLOR_BGR2GRAY), cv2.COLOR_GRAY2BGR)
        self._draw_points(debug, left_x, left_y, (255, 0, 0))
        self._draw_points(debug, mid_x, mid_y, (0, 0, 255))
        self._draw_points(debug, right_x, right_y, (0, 255, 0))
        return LanePixels(debug, left_x, left_y, mid_x, mid_y, right_x, right_y)

    @staticmethod
    def _draw_points(image: np.ndarray, xs: list[int], ys: list[int], color: tuple[int, int, int]) -> None:
        for x, y in zip(xs, ys):
            cv2.circle(image, (int(x), int(y)), 2, color, -1)

    def _target_debug_y(self, lanes: LanePixels, bev_height: int) -> int:
        target_y = int(np.clip(self.target_lookahead_y_px, 0, max(0, bev_height - 1)))
        candidates: list[int] = []
        for ys in (lanes.left_y, lanes.mid_y, lanes.right_y):
            if ys:
                y_arr = np.asarray(ys, dtype=np.int32)
                candidates.append(int(y_arr[int(np.argmin(np.abs(y_arr - target_y)))]))
        if not candidates:
            return target_y
        return int(np.clip(round(float(np.mean(candidates))), 0, bev_height - 1))

    def _target_from_lanes(self, lanes: LanePixels, obstacle_info: str) -> int:
        lx = self._lane_x_at_y(lanes.left_x, lanes.left_y, self.target_lookahead_y_px)
        mx = self._lane_x_at_y(lanes.mid_x, lanes.mid_y, self.target_lookahead_y_px)
        rx = self._lane_x_at_y(lanes.right_x, lanes.right_y, self.target_lookahead_y_px)
        left_slope = self._lane_slope(lanes.left_x, lanes.left_y)
        mid_slope = self._lane_slope(lanes.mid_x, lanes.mid_y)
        right_slope = self._lane_slope(lanes.right_x, lanes.right_y)
        return self._target_from_lane_values(
            lx,
            mx,
            rx,
            obstacle_info,
            left_slope=left_slope,
            mid_slope=mid_slope,
            right_slope=right_slope,
        )

    def _target_from_lane_values(
        self,
        lx: int | None,
        mx: int | None,
        rx: int | None,
        obstacle_info: str,
        left_slope: float | None = None,
        mid_slope: float | None = None,
        right_slope: float | None = None,
    ) -> int:
        target = self.image_width // 2
        center_x = self.image_width // 2
        has_left = lx is not None
        has_mid = mx is not None
        has_right = rx is not None
        has_lane_pair = has_left and has_right and abs(rx - lx) >= self.min_lane_pair_width_px
        mid_left_duplicate = has_mid and has_left and abs(mx - lx) < self.min_lane_pair_width_px
        mid_right_duplicate = has_mid and has_right and abs(rx - mx) < self.min_lane_pair_width_px

        if has_lane_pair:
            target = (lx + rx) // 2
        elif has_left and has_right:
            boundary_x = (lx + rx) // 2
            target = self._target_from_boundary(
                boundary_x,
                self._mean_slope(left_slope, right_slope),
                center_x,
            )
        elif mid_left_duplicate:
            boundary_x = (mx + lx) // 2
            target = self._target_from_boundary(
                boundary_x,
                self._mean_slope(mid_slope, left_slope),
                center_x,
            )
        elif mid_right_duplicate:
            boundary_x = (mx + rx) // 2
            target = self._target_from_boundary(
                boundary_x,
                self._mean_slope(mid_slope, right_slope),
                center_x,
            )
        elif has_mid:
            target = mx
        elif has_left:
            target = lx + self.side_margin_px
        elif has_right:
            target = rx - self.side_margin_px
        else:
            target = self.last_target_x

        if obstacle_info == "right":
            if has_right and has_mid:
                target = (rx + mx) // 2
            elif has_right:
                target = rx - self.obstacle_margin_px
            elif has_mid:
                target = mx + self.obstacle_margin_px - 80
        elif obstacle_info == "left":
            if has_left and has_mid:
                target = (lx + mx) // 2
            elif has_left:
                target = lx + self.obstacle_margin_px
            elif has_mid:
                target = mx - self.obstacle_margin_px + 80

        return int(np.clip(target, 0, self.image_width - 1))

    def _target_from_boundary(self, boundary_x: int, slope: float | None, center_x: int) -> int:
        if slope is not None and abs(slope) > 0.35:
            if slope > 0.0:
                return int(boundary_x - self.side_margin_px)
            return int(boundary_x + self.side_margin_px)
        if boundary_x >= center_x:
            return int(boundary_x - self.side_margin_px)
        return int(boundary_x + self.side_margin_px)

    @staticmethod
    def _mean_slope(*slopes: float | None) -> float | None:
        values = [float(slope) for slope in slopes if slope is not None and np.isfinite(slope)]
        if not values:
            return None
        return float(np.mean(values))

    @staticmethod
    def _lane_slope(xs: list[int], ys: list[int]) -> float | None:
        if len(xs) < 4 or len(ys) < 4:
            return None
        count = min(len(xs), len(ys))
        x_arr = np.asarray(xs[:count], dtype=np.float32)
        y_arr = np.asarray(ys[:count], dtype=np.float32)
        mask = (y_arr >= 10.0) & (y_arr <= 125.0)
        if int(np.count_nonzero(mask)) < 4:
            return None
        try:
            slope, _ = np.polyfit(y_arr[mask], x_arr[mask], 1)
        except Exception:
            return None
        return float(slope)

    def _lane_x_at_y(self, xs: list[int], ys: list[int], target_y: int, max_distance_px: int = 18) -> int | None:
        if not xs or not ys:
            return None
        count = min(len(xs), len(ys))
        y_arr = np.asarray(ys[:count], dtype=np.int32)
        distances = np.abs(y_arr - int(target_y))
        idx = int(np.argmin(distances))
        if int(distances[idx]) > max_distance_px:
            return None
        return int(xs[idx])

    def _center_x_at_y(self, lanes: LanePixels, target_y: int) -> int | None:
        lx = self._lane_x_at_y(lanes.left_x, lanes.left_y, target_y)
        mx = self._lane_x_at_y(lanes.mid_x, lanes.mid_y, target_y)
        rx = self._lane_x_at_y(lanes.right_x, lanes.right_y, target_y)
        target = self._target_from_lane_values(lx, mx, rx, "middle")
        if lx is None and mx is None and rx is None:
            return None
        return target

    def _curve_score(self, lanes: LanePixels) -> float:
        centers: list[int] = []
        for y in (20, 40, 60, 80, 100, 120):
            center = self._center_x_at_y(lanes, y)
            if center is not None:
                centers.append(center)
        if len(centers) < 3:
            return 999.0
        return float(max(centers) - min(centers))

    def _confidence(self, lanes: LanePixels) -> float:
        usable = max(len(lanes.left_x), len(lanes.mid_x), len(lanes.right_x))
        paired_bonus = 0.15 if len(lanes.left_x) >= 2 and len(lanes.right_x) >= 2 else 0.0
        max_windows = max(1, self.preprocessor.max_windows_per_side)
        return float(np.clip(usable / max_windows + paired_bonus, 0.0, 1.0))

    @staticmethod
    def _map(x: float, input_min: float, input_max: float, output_min: float, output_max: float) -> float:
        x = float(np.clip(x, input_min, input_max))
        return (x - input_min) * (output_max - output_min) / (input_max - input_min) + output_min
