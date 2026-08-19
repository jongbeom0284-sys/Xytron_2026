from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class LanePixels:
    debug_image: np.ndarray
    left_x: list[int]
    left_y: list[int]
    mid_x: list[int]
    mid_y: list[int]
    right_x: list[int]
    right_y: list[int]


class AuturboPreprocessor:
    """AuTURBO-style BEV and sliding-window lane extractor."""

    def __init__(
        self,
        roi_y: int = 280,
        roi_y0_ratio: float = 0.58,
        roi_height: int = 200,
        roi_width: int = 640,
        warp_bottom_margin_px: int = 50,
        threshold_mode: str = "auturbo_white_hsv",
        white_s_max: int = 35,
        white_v_min: int = 190,
        window_width_px: int = 30,
        window_height_px: int = 3,
        max_windows_per_side: int = 30,
        max_failures_per_side: int = 10,
    ) -> None:
        self.roi_y = roi_y
        self.roi_y0_ratio = float(roi_y0_ratio)
        self.roi_height = roi_height
        self.roi_width = roi_width
        self.warp_bottom_margin_px = int(warp_bottom_margin_px)
        self.threshold_mode = threshold_mode
        self.white_s_max = int(white_s_max)
        self.white_v_min = int(white_v_min)
        self.window_width = int(window_width_px)
        self.window_height = int(window_height_px)
        self.max_windows_per_side = int(max_windows_per_side)
        self.max_failures_per_side = int(max_failures_per_side)

    def threshold_lane(self, frame: np.ndarray) -> np.ndarray:
        if self.threshold_mode in {"auturbo_white_hsv", "white_hsv"}:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            white_mask = cv2.inRange(
                hsv,
                np.array([0, 0, self.white_v_min], dtype=np.uint8),
                np.array([180, self.white_s_max, 255], dtype=np.uint8),
            )
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            return cv2.morphologyEx(white_mask, cv2.MORPH_CLOSE, kernel)

        blurred = cv2.GaussianBlur(frame, (3, 3), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        return cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 9)

    def warp_perspective(self, binary: np.ndarray) -> np.ndarray:
        h, w = binary.shape[:2]
        width = w
        usable_height = max(10, h - self.warp_bottom_margin_px)
        width_margin = int(round(width * 215.0 / 640.0))
        src = np.float32([[0, 0], [0, usable_height], [width - 1, 0], [width - 1, usable_height]])
        dst = np.float32(
            [[0, 0], [width_margin, usable_height], [width - 1, 0], [width - 1 - width_margin, usable_height]]
        )
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(binary[:usable_height, :], matrix, (width, usable_height), flags=cv2.INTER_LINEAR)

    def bev_to_frame_point(self, x: int, y: int, frame_shape: tuple[int, int, int] | tuple[int, int]) -> tuple[int, int]:
        frame_h, frame_w = frame_shape[:2]
        roi_y0 = int(np.clip(frame_h * self.roi_y0_ratio, 0, max(0, frame_h - 1)))
        roi_h = int(np.clip(self.roi_height, 40, frame_h - roi_y0))
        usable_height = max(10, roi_h - self.warp_bottom_margin_px)
        width_margin = int(round(frame_w * 215.0 / 640.0))
        src = np.float32([[0, 0], [0, usable_height], [frame_w - 1, 0], [frame_w - 1, usable_height]])
        dst = np.float32(
            [[0, 0], [width_margin, usable_height], [frame_w - 1, 0], [frame_w - 1 - width_margin, usable_height]]
        )
        inverse = cv2.getPerspectiveTransform(dst, src)
        point = np.float32([[[float(x), float(y)]]])
        roi_point = cv2.perspectiveTransform(point, inverse)[0][0]
        frame_x = int(np.clip(round(float(roi_point[0])), 0, frame_w - 1))
        frame_y = int(np.clip(round(float(roi_point[1])) + roi_y0, 0, frame_h - 1))
        return frame_x, frame_y

    def edge_binary(self, warped: np.ndarray) -> np.ndarray:
        edge = cv2.Canny(warped, 70, 210)
        close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
        closed = cv2.morphologyEx(edge, cv2.MORPH_CLOSE, close_kernel)
        return self._suppress_checkerboard_edges(closed)

    def _suppress_checkerboard_edges(self, img: np.ndarray) -> np.ndarray:
        """Remove dense grid-like white patterns before sliding-window tracing."""
        _, w = img.shape[:2]
        num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(img, connectivity=8)
        lane_like = np.zeros_like(img)
        for label in range(1, num_labels):
            x, y, comp_w, comp_h, area = stats[label]
            if area < 8:
                continue
            density = float(area) / max(1.0, float(comp_w * comp_h))
            too_wide_grid = comp_w > max(140, int(w * 0.22)) and comp_h > 25 and density > 0.12
            if too_wide_grid:
                continue
            lane_like[labels == label] = 255
        return lane_like

    def hist_line_peak(self, img: np.ndarray) -> tuple[int, int, int]:
        h, w = img.shape[:2]
        histogram = np.sum(img[int(h * 0.6) :, :], axis=0)
        midpoint = int(histogram.shape[0] / 2)
        side_margin = 50
        mid_margin = 30

        left_slice = histogram[: max(1, midpoint - side_margin)]
        right_slice = histogram[min(w - 1, midpoint + side_margin) :]
        mid_slice = histogram[max(0, midpoint - mid_margin) : min(w, midpoint + mid_margin)]

        left_base = int(np.argmax(left_slice)) if left_slice.size else midpoint // 2
        right_base = int(midpoint + side_margin + np.argmax(right_slice)) if right_slice.size else midpoint + midpoint // 2
        mid_base = int(max(0, midpoint - mid_margin) + np.argmax(mid_slice)) if mid_slice.size else midpoint
        return left_base, mid_base, right_base

    def sliding_window(self, img: np.ndarray) -> LanePixels:
        debug = cv2.cvtColor(img.copy(), cv2.COLOR_GRAY2BGR)
        left_base, mid_base, right_base = self.hist_line_peak(img)
        left_x, left_y = self._trace_lane(
            img,
            debug,
            left_base,
            max_windows=self.max_windows_per_side,
            fail_limit=self.max_failures_per_side,
            color=(255, 0, 0),
        )
        mid_x, mid_y = self._trace_lane(
            img,
            debug,
            mid_base,
            max_windows=min(10, max(3, self.max_windows_per_side // 3)),
            fail_limit=min(3, self.max_failures_per_side),
            color=(0, 0, 255),
        )
        right_x, right_y = self._trace_lane(
            img,
            debug,
            right_base,
            max_windows=self.max_windows_per_side,
            fail_limit=self.max_failures_per_side,
            color=(0, 255, 0),
        )
        return LanePixels(debug, left_x, left_y, mid_x, mid_y, right_x, right_y)

    def _trace_lane(
        self,
        img: np.ndarray,
        debug: np.ndarray,
        base_x: int,
        max_windows: int,
        fail_limit: int,
        color: tuple[int, int, int],
    ) -> tuple[list[int], list[int]]:
        h, w = img.shape[:2]
        x = int(np.clip(base_x, 0, w - 1))
        y = min(h - 1, 120)
        fail_count = 0
        found_previous = False
        xs: list[int] = []
        ys: list[int] = []

        while y > 0 and len(xs) < max_windows and fail_count <= fail_limit:
            y0 = max(0, y - self.window_height)
            x0 = max(0, x - self.window_width)
            x1 = min(w, x + self.window_width + 1)
            window = img[y0:y, x0:x1]
            contours, _ = cv2.findContours(window, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                found_previous = False
                cx, cy = self._nonzero_center(window, x0, y0)
                if cx is None or cy is None:
                    fail_count += 1
                    y -= self.window_height
                    continue
            else:
                contour = max(contours, key=cv2.contourArea)
                moments = cv2.moments(contour)
                if moments["m00"] > 0:
                    cx = int(x0 + moments["m10"] / moments["m00"])
                    cy = int(y0 + moments["m01"] / moments["m00"])
                else:
                    cx, cy = self._nonzero_center(window, x0, y0)
                    if cx is None or cy is None:
                        fail_count += 1
                        found_previous = False
                        y -= self.window_height
                        continue
            xs.append(cx)
            ys.append(cy)
            x = int(np.clip(cx, 0, w - 1))
            found_previous = True
            fail_count = 0
            cv2.circle(debug, (cx, cy), 2, color, -1)
            y -= self.window_height
        return xs, ys

    @staticmethod
    def _nonzero_center(window: np.ndarray, x0: int, y0: int) -> tuple[int | None, int | None]:
        ys, xs = np.nonzero(window)
        if xs.size == 0:
            return None, None
        return int(x0 + np.mean(xs)), int(y0 + np.mean(ys))

    def process(self, frame: np.ndarray) -> tuple[np.ndarray, np.ndarray, LanePixels]:
        h, _ = frame.shape[:2]
        roi_y0 = int(np.clip(h * self.roi_y0_ratio, 0, max(0, h - 1)))
        roi_h = int(np.clip(self.roi_height, 40, h - roi_y0))
        roi = frame[roi_y0 : roi_y0 + roi_h, :]
        binary = self.threshold_lane(roi)
        warped = self.warp_perspective(binary)
        lane_binary = self.edge_binary(warped)
        pixels = self.sliding_window(lane_binary)
        return binary, lane_binary, pixels
