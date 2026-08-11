from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class StopLineResult:
    detected: bool = False
    debug_image: np.ndarray | None = None


class StopLineDetector:
    def __init__(self, roi_y: int = 280, roi_height: int = 200, roi_width: int = 640) -> None:
        self.roi_y = roi_y
        self.roi_height = roi_height
        self.roi_width = roi_width
        self.threshold = 180
        self.area_threshold = 2000
        self.length_threshold = 300

    def detect(self, frame: np.ndarray, make_debug: bool = False) -> StopLineResult:
        if frame is None or frame.size == 0:
            return StopLineResult(False, None)
        warped = self._warp(frame)
        blur = cv2.GaussianBlur(warped, (5, 5), 0)
        gray = cv2.cvtColor(blur, cv2.COLOR_BGR2GRAY)
        _, stopline = cv2.threshold(gray, self.threshold, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(stopline, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        detected = False
        debug = warped.copy() if make_debug else None
        for contour in contours:
            length = cv2.arcLength(contour, True)
            area = cv2.contourArea(contour)
            if area <= self.area_threshold or length <= self.length_threshold:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w // 2
            if 200 <= center_x <= self.roi_width - 200 and w > 260 and h < 50:
                detected = True
                if debug is not None:
                    cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 255), 2)
        return StopLineResult(detected=detected, debug_image=debug)

    def _warp(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        width = min(self.roi_width, w)
        y0 = min(max(0, self.roi_y), max(0, h - 1))
        usable_height = min(self.roi_height - 50, h - y0)
        roi = frame[y0 : y0 + usable_height, 0:width]
        width_margin = int(round(width * 215.0 / 640.0))
        src = np.float32([[0, 0], [0, usable_height], [width, 0], [width, usable_height]])
        dst = np.float32([[0, 0], [width_margin, usable_height], [width, 0], [width - width_margin, usable_height]])
        matrix = cv2.getPerspectiveTransform(src, dst)
        return cv2.warpPerspective(roi, matrix, (width, usable_height), flags=cv2.INTER_LINEAR)
