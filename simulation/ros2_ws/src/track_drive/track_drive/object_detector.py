from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass
class ObjectResult:
    direction: str = "none"
    detected: bool = False
    target_class: str = "sonata"
    box_height_px: float = 0.0


class VehicleObjectDetector:
    """Interprets YOLO BoundingBoxes messages; it does not run YOLO itself."""

    def __init__(self, target_class: str = "sonata", rest_class: str = "grandeur", min_height_px: float = 30.0) -> None:
        self.target_class = target_class
        self.rest_class = rest_class
        self.min_height_px = min_height_px
        self.detect = {"grandeur": 0, "avante": 0, "sonata": 0}
        self.x_mid = {"grandeur": 0.0, "avante": 0.0, "sonata": 0.0}
        self.height = {"grandeur": 0.0, "avante": 0.0, "sonata": 0.0}
        self.last_result = ObjectResult(target_class=target_class)

    def update_from_msg(self, msg: Any) -> ObjectResult:
        self.detect = {"grandeur": 0, "avante": 0, "sonata": 0}
        self.x_mid = {"grandeur": 0.0, "avante": 0.0, "sonata": 0.0}
        self.height = {"grandeur": 0.0, "avante": 0.0, "sonata": 0.0}
        for bbox in getattr(msg, "bounding_boxes", []):
            cls = str(getattr(bbox, "Class", getattr(bbox, "class_name", "")))
            if cls not in self.detect:
                continue
            xmin = float(getattr(bbox, "xmin", 0.0))
            xmax = float(getattr(bbox, "xmax", 0.0))
            ymin = float(getattr(bbox, "ymin", 0.0))
            ymax = float(getattr(bbox, "ymax", 0.0))
            self.detect[cls] = 1
            self.x_mid[cls] = (xmin + xmax) * 0.5
            self.height[cls] = abs(ymax - ymin)
        self.last_result = self._classify()
        return self.last_result

    def _classify(self) -> ObjectResult:
        target = self.target_class
        rest = self.rest_class
        target_height = self.height.get(target, 0.0)
        rest_height = self.height.get(rest, 0.0)
        if target_height <= self.min_height_px and rest_height <= self.min_height_px:
            return ObjectResult(direction="none", detected=False, target_class=target, box_height_px=0.0)

        target_x = self.x_mid.get(target, 0.0)
        rest_x = self.x_mid.get(rest, 0.0)
        if target_x != 0.0 and rest_x != 0.0:
            direction = "left" if target_x < rest_x else "right"
        elif target_x == 0.0 and rest_x != 0.0:
            direction = "right" if rest_x < 320.0 else "left"
        elif rest_x == 0.0 and target_x != 0.0:
            direction = "left" if target_x < 320.0 else "right"
        else:
            direction = "none"

        return ObjectResult(
            direction=direction,
            detected=direction != "none",
            target_class=target,
            box_height_px=max(target_height, rest_height),
        )


@dataclass
class HeadlightCandidate:
    x: int
    y: int
    w: int
    h: int
    area: float

    @property
    def cx(self) -> float:
        return self.x + self.w * 0.5

    @property
    def cy(self) -> float:
        return self.y + self.h * 0.5


@dataclass
class HeadlightResult:
    detected: bool = False
    direction: str = "none"
    light_count: int = 0
    pair_count: int = 0
    x_mid: float = 0.0
    y_mid: float = 0.0
    pair_width_px: float = 0.0
    debug_image: np.ndarray | None = None


class HeadlightDetector:
    """Detects white vehicle lights as paired low-saturation bright blobs."""

    def __init__(
        self,
        image_width: int = 640,
        roi_y_min_ratio: float = 0.46,
        roi_y_max_ratio: float = 0.72,
        min_pair_y_ratio: float = 0.46,
        roi_x_side: str = "right",
        roi_side_inner_ratio: float = 0.52,
        roi_side_outer_margin_ratio: float = 0.16,
        max_saturation: int = 80,
        min_value: int = 205,
        min_area_px: float = 8.0,
        max_area_px: float = 900.0,
        min_height_px: float = 12.0,
        max_height_px: float = 45.0,
        min_pair_gap_px: float = 14.0,
        max_pair_gap_px: float = 180.0,
        pair_y_tolerance_px: float = 28.0,
        target_side: str = "strongest",
    ) -> None:
        self.image_width = int(image_width)
        self.roi_y_min_ratio = float(roi_y_min_ratio)
        self.roi_y_max_ratio = float(roi_y_max_ratio)
        self.min_pair_y_ratio = float(min_pair_y_ratio)
        self.roi_x_side = str(roi_x_side)
        self.roi_side_inner_ratio = float(roi_side_inner_ratio)
        self.roi_side_outer_margin_ratio = float(roi_side_outer_margin_ratio)
        self.max_saturation = int(max_saturation)
        self.min_value = int(min_value)
        self.min_area_px = float(min_area_px)
        self.max_area_px = float(max_area_px)
        self.min_height_px = float(min_height_px)
        self.max_height_px = float(max_height_px)
        self.min_pair_gap_px = float(min_pair_gap_px)
        self.max_pair_gap_px = float(max_pair_gap_px)
        self.pair_y_tolerance_px = float(pair_y_tolerance_px)
        self.target_side = str(target_side)
        self.last_result = HeadlightResult()

    def detect(self, frame: np.ndarray, make_debug: bool = True) -> HeadlightResult:
        if frame is None or frame.size == 0:
            self.last_result = HeadlightResult()
            return self.last_result

        h, w = frame.shape[:2]
        y0 = int(np.clip(round(h * self.roi_y_min_ratio), 0, max(0, h - 1)))
        y1 = int(np.clip(round(h * self.roi_y_max_ratio), y0 + 1, h))
        roi = frame[y0:y1, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        white_mask = cv2.inRange(
            hsv,
            np.array([0, 0, self.min_value], dtype=np.uint8),
            np.array([180, self.max_saturation, 255], dtype=np.uint8),
        )
        bgr_min = np.min(roi, axis=2)
        bgr_max = np.max(roi, axis=2)
        balanced_white = ((bgr_min >= 145) & ((bgr_max - bgr_min) <= 85)).astype(np.uint8) * 255
        mask = cv2.bitwise_and(white_mask, balanced_white)
        mask = cv2.medianBlur(mask, 3)
        open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)

        candidates = self._extract_candidates(mask, y0, frame_shape=frame.shape)
        pairs = self._pair_candidates(candidates, frame_height=h)
        best_pair = self._select_pair(pairs, frame_width=w)

        if best_pair is not None:
            left, right = best_pair
            x_mid = (left.cx + right.cx) * 0.5
            y_mid = (left.cy + right.cy) * 0.5
            pair_width = abs(right.cx - left.cx)
            center_x = w * 0.5
            direction = "left" if x_mid < center_x - w * 0.08 else "right" if x_mid > center_x + w * 0.08 else "middle"
            result = HeadlightResult(
                detected=True,
                direction=direction,
                light_count=len(candidates),
                pair_count=len(pairs),
                x_mid=float(x_mid),
                y_mid=float(y_mid),
                pair_width_px=float(pair_width),
            )
        else:
            result = HeadlightResult(light_count=len(candidates), pair_count=0)

        if make_debug:
            result.debug_image = self._make_debug_image(frame, y0, y1, mask, candidates, pairs, best_pair, result)
        self.last_result = result
        return result

    def _extract_candidates(
        self, mask: np.ndarray, y_offset: int, frame_shape: tuple[int, int, int] | tuple[int, int]
    ) -> list[HeadlightCandidate]:
        frame_h, frame_w = frame_shape[:2]
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates: list[HeadlightCandidate] = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if not (self.min_area_px <= area <= self.max_area_px):
                continue
            x, y, bw, bh = cv2.boundingRect(contour)
            if bw < 3 or bh < 2:
                continue
            if not (self.min_height_px <= bh <= self.max_height_px):
                continue
            aspect = bw / max(1.0, float(bh))
            if not (0.7 <= aspect <= 8.0):
                continue
            fill = area / max(1.0, float(bw * bh))
            if fill < 0.18:
                continue
            global_y = y + y_offset
            if global_y < frame_h * self.roi_y_min_ratio or global_y + bh > frame_h * self.roi_y_max_ratio + 8:
                continue
            if not self._in_x_roi(x + bw * 0.5, frame_w):
                continue
            if x < 1 or x + bw > frame_w - 1:
                continue
            candidates.append(HeadlightCandidate(int(x), int(global_y), int(bw), int(bh), area))
        return sorted(candidates, key=lambda c: (c.y, c.x))

    def _in_x_roi(self, cx: float, frame_width: int) -> bool:
        x0, x1 = self._x_roi_bounds(frame_width)
        return float(x0) <= float(cx) <= float(x1)

    def _x_roi_bounds(self, frame_width: int) -> tuple[int, int]:
        side = self.roi_x_side.strip().lower()
        inner = float(np.clip(self.roi_side_inner_ratio, 0.50, 0.98))
        outer_margin = float(np.clip(self.roi_side_outer_margin_ratio, 0.0, 0.45))
        center_x = int(round(frame_width * 0.5))
        if side == "right":
            x0 = int(round(frame_width * inner))
            x1 = int(round(frame_width * (1.0 - outer_margin)))
            return max(center_x, x0), min(frame_width - 1, max(x0 + 1, x1))
        if side == "left":
            x0 = int(round(frame_width * outer_margin))
            x1 = int(round(frame_width * (1.0 - inner)))
            return max(0, min(x0, x1 - 1)), min(center_x, max(x0 + 1, x1))
        x0 = int(round(frame_width * outer_margin))
        x1 = int(round(frame_width * (1.0 - outer_margin)))
        return max(0, x0), min(frame_width - 1, max(x0 + 1, x1))

    def _pair_candidates(
        self, candidates: list[HeadlightCandidate], frame_height: int
    ) -> list[tuple[HeadlightCandidate, HeadlightCandidate]]:
        pairs: list[tuple[HeadlightCandidate, HeadlightCandidate]] = []
        for i, left in enumerate(candidates):
            for right in candidates[i + 1 :]:
                if right.cx <= left.cx:
                    continue
                gap = right.cx - left.cx
                if not (self.min_pair_gap_px <= gap <= self.max_pair_gap_px):
                    continue
                pair_y = (left.cy + right.cy) * 0.5
                if pair_y < frame_height * self.min_pair_y_ratio:
                    continue
                y_delta = abs(right.cy - left.cy)
                if y_delta > max(self.pair_y_tolerance_px, 1.6 * max(left.h, right.h)):
                    continue
                h_ratio = min(left.h, right.h) / max(1.0, float(max(left.h, right.h)))
                area_ratio = min(left.area, right.area) / max(1.0, float(max(left.area, right.area)))
                if h_ratio < 0.35 or area_ratio < 0.18:
                    continue
                pairs.append((left, right))
        return pairs

    @staticmethod
    def _pair_mid_x(pair: tuple[HeadlightCandidate, HeadlightCandidate]) -> float:
        left, right = pair
        return (left.cx + right.cx) * 0.5

    def _select_pair(
        self, pairs: list[tuple[HeadlightCandidate, HeadlightCandidate]], frame_width: int
    ) -> tuple[HeadlightCandidate, HeadlightCandidate] | None:
        if not pairs:
            return None
        target_side = self.target_side.lower()
        if target_side == "right":
            right_pairs = [pair for pair in pairs if self._pair_mid_x(pair) >= frame_width * 0.5]
            return max(right_pairs or pairs, key=self._pair_mid_x)
        if target_side == "left":
            left_pairs = [pair for pair in pairs if self._pair_mid_x(pair) <= frame_width * 0.5]
            return min(left_pairs or pairs, key=self._pair_mid_x)
        if target_side == "middle":
            center_x = frame_width * 0.5
            return min(pairs, key=lambda pair: abs(self._pair_mid_x(pair) - center_x))
        return max(pairs, key=self._pair_score)

    @staticmethod
    def _pair_score(pair: tuple[HeadlightCandidate, HeadlightCandidate]) -> float:
        left, right = pair
        y_alignment = 1.0 / (1.0 + abs(left.cy - right.cy))
        size_score = left.area + right.area
        gap_score = min(120.0, abs(right.cx - left.cx))
        lower_image_score = (left.cy + right.cy) * 0.25
        return size_score * 2.0 + gap_score + y_alignment * 20.0 + lower_image_score

    def _make_debug_image(
        self,
        frame: np.ndarray,
        y0: int,
        y1: int,
        mask: np.ndarray,
        candidates: list[HeadlightCandidate],
        pairs: list[tuple[HeadlightCandidate, HeadlightCandidate]],
        best_pair: tuple[HeadlightCandidate, HeadlightCandidate] | None,
        result: HeadlightResult,
    ) -> np.ndarray:
        debug = frame.copy()
        roi_x0, roi_x1 = self._x_roi_bounds(frame.shape[1])
        overlay = debug.copy()
        cv2.rectangle(overlay, (roi_x0, y0), (roi_x1, y1), (0, 180, 80), -1)
        debug = cv2.addWeighted(overlay, 0.16, debug, 0.84, 0.0)
        cv2.rectangle(debug, (roi_x0, y0), (roi_x1, y1), (0, 255, 80), 2)
        cv2.putText(
            debug,
            f"ROI x={roi_x0}-{roi_x1} y={y0}-{y1}",
            (max(8, roi_x0 + 8), max(18, y0 - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 80),
            2,
            cv2.LINE_AA,
        )
        for candidate in candidates:
            cv2.rectangle(
                debug,
                (candidate.x, candidate.y),
                (candidate.x + candidate.w, candidate.y + candidate.h),
                (255, 180, 0),
                1,
            )
            cv2.circle(debug, (int(candidate.cx), int(candidate.cy)), 2, (255, 180, 0), -1)

        for left, right in pairs:
            cv2.line(
                debug,
                (int(left.cx), int(left.cy)),
                (int(right.cx), int(right.cy)),
                (0, 200, 255),
                1,
            )

        if best_pair is not None:
            left, right = best_pair
            cv2.rectangle(
                debug,
                (min(left.x, right.x), min(left.y, right.y)),
                (max(left.x + left.w, right.x + right.w), max(left.y + left.h, right.y + right.h)),
                (0, 0, 255),
                2,
            )
            cv2.circle(debug, (int(result.x_mid), int(result.y_mid)), 5, (0, 0, 255), -1)

        status = (
            f"headlight detected={int(result.detected)} dir={result.direction} "
            f"lights={result.light_count} pairs={result.pair_count} x={result.x_mid:.0f} w={result.pair_width_px:.0f}"
        )
        cv2.putText(debug, status, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.58, (0, 255, 255), 2, cv2.LINE_AA)

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_bgr = cv2.resize(mask_bgr, (frame.shape[1] // 3, max(1, (y1 - y0) // 3)))
        mh, mw = mask_bgr.shape[:2]
        debug[0:mh, frame.shape[1] - mw : frame.shape[1]] = mask_bgr
        return debug
