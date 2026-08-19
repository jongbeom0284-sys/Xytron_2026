#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
신호등 인식 전용 모듈.

대회 과제 #1 조건:
- 출발선 3구 신호등은 빨강/노랑/파랑(녹색)으로 바뀌며 파란불일 때만 출발한다.
- 좌회전 4구 신호등은 두 번째와 세 번째 바퀴에 등장한다.
- 좌회전 신호가 켜져도 경찰차가 차선을 막고 있으면 좌회전하면 안 된다.

이 파일은 카메라 이미지에서 신호등 색을 안정적으로 판단하는 역할만 담당한다.
경찰차 차단 여부와 실제 조향 제어는 다른 주행 모듈에서 결합한다.
"""

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class RoiRatio:
    """이미지 크기에 비례하는 ROI 영역.

    x1, y1, x2, y2는 0.0~1.0 사이 비율이다.
    신호등은 주로 화면 위쪽에 나오므로 기본값은 상단 중앙 영역으로 둔다.
    """

    x1: float
    y1: float
    x2: float
    y2: float

    def crop(self, image):
        height, width = image.shape[:2]
        left = max(0, min(width - 1, int(width * self.x1)))
        top = max(0, min(height - 1, int(height * self.y1)))
        right = max(left + 1, min(width, int(width * self.x2)))
        bottom = max(top + 1, min(height, int(height * self.y2)))
        return image[top:bottom, left:right], (left, top, right, bottom)


class TrafficLightDetector:
    """HSV 색상 기반 신호등 검출기.

    시뮬레이터 신호등은 색 대비가 비교적 뚜렷하므로, 첫 구현은 딥러닝보다
    HSV 색상 마스크와 연속 프레임 안정화 방식이 실전 튜닝에 유리하다.
    """

    START_RED = "RED"
    START_YELLOW = "YELLOW"
    START_GREEN = "GREEN"
    UNKNOWN = "UNKNOWN"
    LAYOUT_THREE = "THREE_LIGHT"
    LAYOUT_FOUR = "FOUR_LIGHT"

    def __init__(
        self,
        start_roi=RoiRatio(0.15, 0.00, 0.98, 0.42),
        left_roi=RoiRatio(0.10, 0.00, 0.98, 0.56),
        history_size=3,
        stable_count=2,
        min_area=35,
        four_light_aspect=3.05,
        left_turn_slot="right",
    ):
        self.start_roi = start_roi
        self.left_roi = left_roi
        self.history_size = history_size
        self.stable_count = stable_count
        self.min_area = min_area
        self.four_light_aspect = four_light_aspect
        self.left_turn_slot = left_turn_slot
        self.start_history = deque(maxlen=history_size)
        self.left_history = deque(maxlen=history_size)
        self.last_candidates = []
        self.last_left_detail = {
            "layout": self.UNKNOWN,
            "presence": False,
            "raw_signal": False,
            "stable_signal": False,
            "confidence": 0.0,
            "arrow_direction_score": 0.0,
            "candidate_count": 0,
            "candidate_source": "INIT",
            "candidate_box": None,
            "reason": "INIT",
        }

    def reset_left_history(self):
        """좌회전 구간이 아닐 때 이전 프레임 판정이 남지 않게 초기화한다."""

        self.left_history.clear()
        self.last_left_detail = {
            "layout": self.UNKNOWN,
            "presence": False,
            "raw_signal": False,
            "stable_signal": False,
            "confidence": 0.0,
            "arrow_direction_score": 0.0,
            "candidate_count": 0,
            "candidate_source": "RESET",
            "candidate_box": None,
            "reason": "RESET",
        }

    def detect_signal_candidates(self, image):
        """Find likely traffic-light housings in the upper camera frame.

        This keeps the old ROI path as a safe fallback, but exposes an explicit
        candidate-detection stage so the driver can log and gate decisions like
        a real perception pipeline.
        """

        roi, (offset_x, offset_y, _, _) = self.left_roi.crop(image)
        candidates = []
        for source, local_box in self._detect_housing_candidates(roi):
            x, y, width, height = local_box
            pad_x = max(6, int(width * 0.10))
            pad_y = max(6, int(height * 0.35))
            left = max(0, x - pad_x)
            top = max(0, y - pad_y)
            right = min(roi.shape[1], x + width + pad_x)
            bottom = min(roi.shape[0], y + height + pad_y)
            crop = roi[top:bottom, left:right]
            if crop.size == 0:
                continue
            layout, geometry = self._detect_light_layout(crop)
            color_score = self._housing_color_score(
                cv2.cvtColor(roi, cv2.COLOR_BGR2HSV),
                x,
                y,
                width,
                height,
            )
            aspect = width / max(1, height)
            candidates.append(
                {
                    "bbox": (offset_x + left, offset_y + top, right - left, bottom - top),
                    "housing": (offset_x + x, offset_y + y, width, height),
                    "layout": layout,
                    "score": float(color_score + width * height * 0.05),
                    "aspect": float(aspect),
                    "source": source,
                    "geometry": geometry,
                }
            )

        if not candidates:
            layout, geometry = self._detect_light_layout(roi)
            if layout != self.UNKNOWN:
                candidates.append(
                    {
                        "bbox": (offset_x, offset_y, roi.shape[1], roi.shape[0]),
                        "housing": None,
                        "layout": layout,
                        "score": float(self.min_area),
                        "aspect": 0.0,
                        "source": "ROI_FALLBACK",
                        "geometry": geometry,
                    }
                )

        candidates = sorted(candidates, key=lambda item: item["score"], reverse=True)
        self.last_candidates = candidates[:4]
        return list(self.last_candidates)

    def detect_start_light(self, image):
        """출발 신호등 색을 반환한다.

        반환값은 RED, YELLOW, GREEN, UNKNOWN 중 하나다.
        불확실하면 출발하지 않도록 UNKNOWN을 반환한다.
        """

        roi, _ = self.start_roi.crop(image)
        if self._has_left_arrow_pair(roi):
            self.start_history.append(self.UNKNOWN)
            return self.UNKNOWN

        layout, geometry = self._detect_light_layout(roi)
        if layout != self.LAYOUT_THREE or geometry is None:
            self.start_history.append(self.UNKNOWN)
            return self.UNKNOWN

        if isinstance(geometry, dict) and "circles" in geometry and len(geometry["circles"]) != 3:
            self.start_history.append(self.UNKNOWN)
            return self.UNKNOWN

        scores = self._color_scores(roi, geometry)

        if max(scores.values()) < self.min_area:
            signal = self.UNKNOWN
        else:
            signal = max(scores, key=scores.get)

        self.start_history.append(signal)
        if signal in (self.START_RED, self.START_YELLOW):
            return signal
        return self._stable_value(self.start_history, signal)

    def can_start(self, image):
        """파란/녹색 신호가 안정적으로 감지된 경우에만 True를 반환한다."""

        return self.detect_start_light(image) == self.START_GREEN

    def detect_four_light_state(self, image):
        """Return the visible 4-light traffic state for the driving signal zone."""

        roi, _ = self.left_roi.crop(image)
        candidates = self.detect_signal_candidates(image)
        analysis_roi = roi
        if candidates:
            x, y, width, height = candidates[0]["bbox"]
            height_img, width_img = image.shape[:2]
            left = max(0, int(x))
            top = max(0, int(y))
            right = min(width_img, int(x + width))
            bottom = min(height_img, int(y + height))
            if right > left and bottom > top:
                analysis_roi = image[top:bottom, left:right]
        layout, geometry = self._detect_light_layout(analysis_roi)
        has_housing = layout == self.LAYOUT_FOUR or self._has_four_light_housing(analysis_roi) or bool(candidates)
        stats = self._color_stats(analysis_roi, geometry if isinstance(geometry, dict) else None)
        raw_scores = {color: stat["area"] for color, stat in stats.items()}
        lamp_candidates = self._compact_lamp_candidates(analysis_roi)
        compact_scores = {
            color: lamp_candidates[color]["area"] if color in lamp_candidates else 0.0
            for color in (self.START_RED, self.START_YELLOW, self.START_GREEN)
        }
        best_signal = self.UNKNOWN
        housing_scores = {}
        geometry_confirmed = layout != self.UNKNOWN
        if has_housing and geometry_confirmed:
            housing_scores = {
                color: stat["area"]
                for color, stat in stats.items()
                if self._is_lamp_like_blob(stat, analysis_roi.shape, area_scale=0.55)
            }
            if housing_scores:
                best_signal = max(housing_scores, key=housing_scores.get)
        # Do not let broad raw HSV area override lamp candidates.  In this map,
        # yellow lane markings/trees can enter the ROI and make raw_yellow
        # larger than the actual lit green lamp.
        bar_supported_scores = {
            color: lamp["area"]
            for color, lamp in lamp_candidates.items()
            if self._has_dark_signal_bar_support(analysis_roi, lamp, color)
        }
        if not bar_supported_scores and analysis_roi is not roi:
            roi_lamp_candidates = self._compact_lamp_candidates(roi)
            bar_supported_scores = {
                color: lamp["area"]
                for color, lamp in roi_lamp_candidates.items()
                if self._has_dark_signal_bar_support(roi, lamp, color)
            }
        if bar_supported_scores:
            best_signal = max(bar_supported_scores, key=bar_supported_scores.get)
            housing_scores = bar_supported_scores
        elif best_signal == self.UNKNOWN and lamp_candidates and not candidates:
            best_signal = max(lamp_candidates, key=lambda color: lamp_candidates[color]["area"])
        color_presence = bool(lamp_candidates)
        debug_scores = compact_scores if color_presence else housing_scores
        if bar_supported_scores:
            candidate_source = "SIGNAL_BAR_COLOR"
        elif candidates:
            candidate_source = candidates[0]["source"]
        elif has_housing:
            candidate_source = "HOUSING_FALLBACK"
        elif color_presence:
            candidate_source = "COLOR_FALLBACK"
        else:
            candidate_source = "NONE"
        return {
            "signal": best_signal,
            "layout": layout,
            "presence": bool(has_housing or candidates or color_presence),
            "candidate_count": len(candidates) if candidates else len(lamp_candidates),
            "candidate_source": candidate_source,
            "red_score": float(debug_scores.get(self.START_RED, 0.0)),
            "yellow_score": float(debug_scores.get(self.START_YELLOW, 0.0)),
            "green_score": float(debug_scores.get(self.START_GREEN, 0.0)),
            "raw_red_score": float(raw_scores.get(self.START_RED, 0.0)),
            "raw_yellow_score": float(raw_scores.get(self.START_YELLOW, 0.0)),
            "raw_green_score": float(raw_scores.get(self.START_GREEN, 0.0)),
        }

    def detect_left_turn_signal(self, image):
        """좌회전 신호가 켜졌는지 판단한다.

        이 함수는 전체 주행 상태머신이 좌회전 신호등 구간에 진입했다고
        판단한 뒤에만 호출해야 한다. 출발선 3구 신호등의 녹색불과
        좌회전 신호등의 녹색 계열은 색상만으로 완전히 구분하기 어렵기 때문이다.

        4구 배치가 확인되고 녹색 신호가 원형등이 아닌 화살표 형태로 보일 때만
        좌회전 가능으로 판단한다.
        """

        detail = self.detect_left_turn_detail(image, update_history=True)
        return detail["stable_signal"]

    def detect_left_turn_detail(self, image, update_history=True):
        """좌회전 신호 판단 근거를 함께 반환한다.

        직진 녹색 원형등과 좌회전 화살표를 구분하기 위해, 4구 구조와
        화살표 형태가 함께 확인될 때만 raw_signal을 True로 둔다.
        """

        roi, _ = self.left_roi.crop(image)
        candidates = self.detect_signal_candidates(image)
        best_candidate = candidates[0] if candidates else None
        candidate_layout = best_candidate.get("layout") if best_candidate else self.UNKNOWN
        candidate_roi = roi
        if best_candidate:
            x, y, width, height = best_candidate["bbox"]
            img_h, img_w = image.shape[:2]
            left = max(0, int(x))
            top = max(0, int(y))
            right = min(img_w, int(x + width))
            bottom = min(img_h, int(y + height))
            if right > left and bottom > top:
                candidate_roi = image[top:bottom, left:right]

        layout = candidate_layout if candidate_layout == self.LAYOUT_FOUR else self.infer_layout(image)
        arrow_pair = self._has_verified_left_arrow_pair(roi) or self._has_verified_left_arrow_pair(candidate_roi)
        arrow_in_housing, housing_score = self._has_arrow_shaped_green_in_housing(candidate_roi)
        if not arrow_in_housing:
            arrow_in_housing, housing_score = self._has_arrow_shaped_green_in_housing(roi)
        signal_bar_arrow, bar_score = self._has_signal_bar_with_arrow_green(candidate_roi)
        if not signal_bar_arrow:
            signal_bar_arrow, bar_score = self._has_signal_bar_with_arrow_green(roi)
        arrow_direction_score = max(
            self._left_arrow_pair_direction_score(roi) if arrow_pair else 0.0,
            housing_score,
            bar_score,
        )
        candidate_four = candidate_layout == self.LAYOUT_FOUR
        presence = (
            arrow_pair
            or arrow_in_housing
            or signal_bar_arrow
            or candidate_four
            or self._has_four_light_housing(roi)
            or self._has_three_dark_plus_green_slot(roi)
        )

        if arrow_pair:
            reason = "RED_GREEN_ARROW_PAIR"
        elif arrow_in_housing:
            reason = "ARROW_IN_FOUR_LIGHT_HOUSING"
        elif signal_bar_arrow:
            reason = "SIGNAL_BAR_ARROW"
        elif candidate_four:
            reason = "FOUR_LIGHT_CANDIDATE"
        elif presence:
            reason = "FOUR_LIGHT_NO_LEFT_ARROW"
        else:
            reason = "NO_FOUR_LIGHT"

        raw_signal = layout == self.LAYOUT_FOUR and (arrow_pair or arrow_in_housing)
        confidence = self._left_signal_confidence(
            layout,
            arrow_pair=arrow_pair,
            arrow_in_housing=arrow_in_housing,
            signal_bar_arrow=signal_bar_arrow,
            presence=presence,
            arrow_direction_score=arrow_direction_score,
        )
        if update_history:
            self.left_history.append(raw_signal)
            stable_signal = raw_signal and self._stable_bool(self.left_history, True)
        else:
            stable_signal = raw_signal and self._stable_bool(self.left_history, True)

        self.last_left_detail = {
            "layout": layout,
            "presence": presence,
            "raw_signal": raw_signal,
            "stable_signal": stable_signal,
            "confidence": confidence,
            "arrow_direction_score": arrow_direction_score,
            "candidate_count": len(candidates),
            "candidate_source": best_candidate["source"] if best_candidate else "NONE",
            "candidate_box": best_candidate["bbox"] if best_candidate else None,
            "reason": reason,
        }
        return self.last_left_detail.copy()

    def _left_signal_confidence(
        self,
        layout,
        arrow_pair=False,
        arrow_in_housing=False,
        signal_bar_arrow=False,
        presence=False,
        arrow_direction_score=0.0,
    ):
        """Return a small confidence score for logging and conservative gating."""

        score = 0.0
        if layout == self.LAYOUT_FOUR:
            score += 0.30
        if presence:
            score += 0.10
        if arrow_pair:
            score += 0.35
        if arrow_in_housing:
            score += 0.34
        if signal_bar_arrow:
            score += 0.30
        score += min(0.20, max(0.0, arrow_direction_score) * 0.20)
        return min(score, 1.0)

    def detect_left_turn_presence(self, image):
        """좌회전 4구 신호등이 보이는지만 보수적으로 판단한다.

        이 값은 대기 상태 진입에만 사용하고, 좌회전 허용 판정에는 절대 사용하지 않는다.
        실제 좌회전 허용은 detect_left_turn_signal()의 연속 프레임 판정을 통과해야 한다.
        """

        return self.detect_left_turn_detail(image, update_history=False)["presence"]

    def infer_layout(self, image):
        """디버그 로그용 신호등 배치를 추정한다."""

        roi, _ = self.left_roi.crop(image)
        if (
            self._has_verified_left_arrow_pair(roi)
            or self._has_four_light_housing(roi)
            or self._has_three_dark_plus_green_slot(roi)
            or self._has_signal_bar_with_arrow_green(roi)[0]
            or self._has_arrow_shaped_green_in_housing(roi)[0]
        ):
            return self.LAYOUT_FOUR
        layout, _ = self._detect_light_layout(roi)
        return layout

    def _has_four_light_housing(self, roi):
        """4구 신호등의 검은 하우징을 보수적으로 찾는다.

        좌회전 신호 대기 진입용 보조 판정이다. 실제 좌회전 허용은
        별도의 초록 화살표 판정과 라이다 차단물 판단을 다시 통과해야 한다.
        """

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 95, 80]))
        dark[int(roi.shape[0] * 0.82):, :] = 0
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8), iterations=2)
        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        roi_height, roi_width = roi.shape[:2]
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 900:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            aspect = width / max(1, height)
            # 3구 신호등도 검은 하우징의 가로세로비가 2.7 안팎까지 나올 수 있다.
            # 4구 대기 구간 진입 보조 판정은 보수적으로 두어 출발선 빨간불 오판을 막는다.
            if aspect < 3.05 or aspect > 4.8:
                continue
            if width < roi_width * 0.16 or height < roi_height * 0.10:
                continue
            if y > roi_height * 0.68:
                continue
            return True
        return False

    def _has_arrow_shaped_green_in_housing(self, roi):
        """4구 하우징 안의 녹색 좌회전 화살표만 좌회전 신호로 인정한다.

        단순히 4구 신호등 오른쪽에 녹색 원형 램프가 켜진 경우는 직진 신호일 수
        있으므로 좌회전으로 보지 않는다. 녹색 blob이 하우징 안에 있고, 원형등보다
        가로로 길게 퍼진 화살표 형태일 때만 True를 반환한다.
        """

        layout, geometry = self._detect_light_layout(roi)
        if layout != self.LAYOUT_FOUR or not isinstance(geometry, dict):
            return False, 0.0
        housing = geometry.get("housing")
        if housing is None:
            return False, 0.0

        hx, hy, hw, hh = housing
        green_blobs = self._colored_blobs(roi, self.START_GREEN)
        best_score = 0.0
        for blob in green_blobs:
            cx = blob["cx"]
            cy = blob["cy"]
            if cx is None or cy is None:
                continue
            in_left_turn_slots = hx + hw * 0.18 <= cx <= hx + hw * 0.64
            in_housing_band = hy - hh * 0.10 <= cy <= hy + hh * 1.18
            area_ok = blob["area"] >= self.min_area * 2.0
            size_ok = blob["width"] <= hw * 0.32 and blob["height"] <= hh * 0.78
            direction_score = self._left_arrow_direction_score(blob)
            best_score = max(best_score, direction_score)
            shape_ok = self._is_left_arrow_like_blob(blob, roi.shape, direction_score=direction_score)
            if in_left_turn_slots and in_housing_band and area_ok and size_ok and shape_ok:
                return True, direction_score
        return False, best_score

    def _has_verified_left_arrow_pair(self, roi):
        """4구 하우징/바 후보와 함께 보이는 좌회전 화살표만 인정한다."""

        if not self._has_left_arrow_pair(roi):
            return False
        return (
            self._has_four_light_housing(roi)
            or self._has_signal_bar_with_arrow_green(roi)[0]
            or self._has_three_dark_plus_green_slot(roi)
        )

    def _has_signal_bar_with_arrow_green(self, roi):
        """검은 가로 바 주변의 녹색 좌회전 화살표 후보를 찾는다."""

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 110, 90]))
        green = cv2.inRange(hsv, np.array([50, 70, 70]), np.array([125, 255, 255]))
        dark[int(roi.shape[0] * 0.84):, :] = 0
        green[int(roi.shape[0] * 0.84):, :] = 0
        dark_contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        bars = []
        for contour in dark_contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area < 120 or width < roi.shape[1] * 0.15:
                continue
            if height > roi.shape[0] * 0.16:
                continue
            bars.append((x, y, width, height))

        lamps = []
        for contour in green_contours:
            area = cv2.contourArea(contour)
            x, y, width, height = cv2.boundingRect(contour)
            if area < 80 or width < 12 or height < 10:
                continue
            if y > roi.shape[0] * 0.76:
                continue
            lamps.append((x, y, width, height))

        for bx, by, bw, bh in bars:
            bar_cy = by + bh / 2.0
            for gx, gy, gw, gh in lamps:
                lamp_cy = gy + gh / 2.0
                close_to_bar = bx + bw * 0.15 < gx < bx + bw * 0.82
                same_band = abs(lamp_cy - bar_cy) < max(roi.shape[0] * 0.22, gh)
                blob = {
                    "area": float(gw * gh),
                    "x": gx,
                    "y": gy,
                    "width": gw,
                    "height": gh,
                    "fill_ratio": 1.0,
                    "cx": gx + gw / 2.0,
                    "cy": lamp_cy,
                }
                direction_score = self._left_arrow_direction_score(blob)
                if close_to_bar and same_band and self._is_left_arrow_like_blob(
                    blob,
                    roi.shape,
                    direction_score=direction_score,
                ):
                    return True, direction_score
        return False, 0.0

    def _has_three_dark_plus_green_slot(self, roi):
        """검은 3칸 옆에 초록 램프가 붙은 4구 신호등 후보를 찾는다."""

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 100, 85]))
        green = cv2.inRange(hsv, np.array([50, 70, 70]), np.array([125, 255, 255]))
        dark[int(roi.shape[0] * 0.82):, :] = 0
        green[int(roi.shape[0] * 0.82):, :] = 0
        dark_contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        green_contours, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        dark_boxes = []
        for contour in dark_contours:
            area = cv2.contourArea(contour)
            if area < 120:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width < 10 or height < 10:
                continue
            dark_boxes.append((x, y, width, height, area))

        green_boxes = []
        for contour in green_contours:
            area = cv2.contourArea(contour)
            if area < 90:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width < 10 or height < 10:
                continue
            green_boxes.append((x, y, width, height, area))

        if not dark_boxes or not green_boxes:
            return False

        dark = max(dark_boxes, key=lambda item: item[4])
        dx, dy, dw, dh, _ = dark
        for gx, gy, gw, gh, _ in green_boxes:
            # 주변 나무/잔디가 하우징 위쪽에서 크게 잡히는 경우를 배제한다.
            # 실제 4번째 슬롯은 검은 램프들과 같은 행에 있고 하우징 오른쪽 끝 근처에 붙는다.
            same_row = abs((gy + gh / 2.0) - (dy + dh / 2.0)) < min(dh, gh) * 0.35
            right_side = gx > dx + dw * 0.86 and gx < dx + dw + roi.shape[1] * 0.10
            size_ok = gh > dh * 0.35 and gh < dh * 1.20 and gw > 8 and gw < dw * 0.55
            if same_row and right_side and size_ok:
                return True
        return False

    def _has_left_arrow_pair(self, roi):
        """빨간 램프와 같은 높이대의 초록 좌회전 화살표 쌍을 찾는다.

        4구 신호등이 화면 위에 잘리면 검은 하우징 전체를 안정적으로 잡기 어렵다.
        이때는 켜진 빨간 램프와 초록 화살표의 상대 위치가 가장 안정적인 단서다.
        """

        red_blobs = self._colored_blobs(roi, self.START_RED)
        green_blobs = self._colored_blobs(roi, self.START_GREEN)
        if not red_blobs or not green_blobs:
            return False

        roi_height, roi_width = roi.shape[:2]
        for red in red_blobs:
            for green in green_blobs:
                dx = green["cx"] - red["cx"]
                dy = abs(green["cy"] - red["cy"])
                if dx < roi_width * 0.12 or dx > roi_width * 0.55:
                    continue
                # 좌회전 화살표는 빨간 램프와 거의 같은 높이대에 있어야 한다.
                # 넓은 ROI 안의 나무/잔디/표지판 녹색이 같은 신호로 묶이지 않도록
                # 이미지 비율 대신 실제 램프 크기 기준으로 세로 오차를 제한한다.
                max_lamp_height = max(red["height"], green["height"])
                if dy > max_lamp_height * 0.55:
                    continue
                if green["area"] < max(self.min_area * 8.0, red["area"] * 0.35):
                    continue
                if green["width"] > roi_width * 0.24 or green["height"] > roi_height * 0.42:
                    continue
                if green["width"] < 18 or green["height"] < 16:
                    continue
                direction_score = self._left_arrow_direction_score(green)
                # In the simulator the red lamp can be cropped by the ROI edge,
                # while the left-arrow lamp remains fully visible and nearly square.
                # For a red/green same-row pair, accept a weaker shape score than
                # the generic green-blob filter because the pair geometry is already
                # strong evidence that this is the protected left-turn signal.
                if direction_score < 0.28:
                    continue
                if green["width"] / max(1.0, float(green["height"])) < 0.95:
                    continue
                return True
        return False

    def _left_arrow_pair_direction_score(self, roi):
        """Return the strongest left-arrow direction score among red/green pairs."""

        red_blobs = self._colored_blobs(roi, self.START_RED)
        green_blobs = self._colored_blobs(roi, self.START_GREEN)
        if not red_blobs or not green_blobs:
            return 0.0
        roi_height, roi_width = roi.shape[:2]
        best_score = 0.0
        for red in red_blobs:
            for green in green_blobs:
                dx = green["cx"] - red["cx"]
                dy = abs(green["cy"] - red["cy"])
                if dx < roi_width * 0.12 or dx > roi_width * 0.55:
                    continue
                if dy > max(red["height"], green["height"]) * 0.65:
                    continue
                if green["width"] > roi_width * 0.24 or green["height"] > roi_height * 0.42:
                    continue
                best_score = max(best_score, self._left_arrow_direction_score(green))
        return best_score

    def _colored_blobs(self, bgr_image, color):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        if color == self.START_RED:
            mask = cv2.inRange(hsv, np.array([0, 65, 65]), np.array([10, 255, 255]))
            mask |= cv2.inRange(hsv, np.array([170, 65, 65]), np.array([180, 255, 255]))
        elif color == self.START_YELLOW:
            mask = cv2.inRange(hsv, np.array([15, 60, 65]), np.array([48, 255, 255]))
        else:
            mask = cv2.inRange(hsv, np.array([50, 55, 60]), np.array([125, 255, 255]))

        roi_height, roi_width = bgr_image.shape[:2]
        mask[int(roi_height * 0.78):, :] = 0
        mask = self._clean_color_mask(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        blobs = []
        for contour in contours:
            area = float(cv2.contourArea(contour))
            if area < self.min_area * 2.0:
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if width <= 0 or height <= 0:
                continue
            if width > roi_width * 0.34 or height > roi_height * 0.48:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] == 0:
                continue
            fill_ratio = area / max(1.0, float(width * height))
            if fill_ratio < 0.22:
                continue
            blobs.append(
                {
                    "area": area,
                    "cx": moments["m10"] / moments["m00"],
                    "cy": moments["m01"] / moments["m00"],
                    "x": x,
                    "y": y,
                    "width": width,
                    "height": height,
                    "fill_ratio": fill_ratio,
                    "contour": contour,
                }
            )
        return sorted(blobs, key=lambda blob: blob["area"], reverse=True)

    def _is_left_arrow_like_blob(self, blob, image_shape, direction_score=None):
        """원형 녹색등과 좌회전 화살표를 구분하기 위한 형태 필터."""

        image_height, image_width = image_shape[:2]
        width = float(blob.get("width", 0))
        height = float(blob.get("height", 0))
        area = float(blob.get("area", 0.0))
        cy = blob.get("cy")
        if width <= 0 or height <= 0 or cy is None:
            return False
        aspect = width / max(1.0, height)
        if aspect < 1.32:
            return False
        if width < 18 or height < 10:
            return False
        if area < self.min_area * 2.0:
            return False
        if width > image_width * 0.28 or height > image_height * 0.32:
            return False
        if cy > image_height * 0.78:
            return False
        if direction_score is None:
            direction_score = self._left_arrow_direction_score(blob)
        if direction_score < 0.42:
            return False
        return True

    def _left_arrow_direction_score(self, blob):
        """Score whether a green blob has a left arrow head and right stem.

        A circular green lamp can pass a color test, but it should not have the
        left-heavy vertical spread and right-side stem that a left arrow has.
        """

        width = int(blob.get("width", 0))
        height = int(blob.get("height", 0))
        if width <= 0 or height <= 0:
            return 0.0

        contour = blob.get("contour")
        if contour is None:
            aspect = width / max(1.0, float(height))
            return float(np.clip((aspect - 1.25) / 0.75, 0.0, 0.55))

        x = int(blob.get("x", 0))
        y = int(blob.get("y", 0))
        shifted = contour.copy()
        shifted[:, 0, 0] -= x
        shifted[:, 0, 1] -= y
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.drawContours(mask, [shifted], -1, 255, -1)

        if cv2.countNonZero(mask) < self.min_area * 2:
            return 0.0

        third = max(1, width // 3)
        left = mask[:, :third]
        middle = mask[:, third: max(third + 1, width - third)]
        right = mask[:, width - third:]
        left_count = cv2.countNonZero(left)
        middle_count = cv2.countNonZero(middle)
        right_count = cv2.countNonZero(right)
        if left_count == 0 or middle_count == 0 or right_count == 0:
            return 0.0

        def vertical_span(part):
            ys = np.where(part > 0)[0]
            if ys.size == 0:
                return 0
            return int(ys.max() - ys.min() + 1)

        left_span = vertical_span(left)
        right_span = vertical_span(right)
        local_cx = (float(blob.get("cx", x + width / 2.0)) - x) / max(1.0, float(width))

        spread_score = np.clip((left_span / max(1.0, float(right_span)) - 1.05) / 0.55, 0.0, 1.0)
        balance_score = np.clip(min(left_count, right_count) / max(left_count, right_count), 0.0, 1.0)
        centroid_score = np.clip((0.66 - local_cx) / 0.18, 0.0, 1.0)
        aspect_score = np.clip((width / max(1.0, float(height)) - 1.25) / 0.55, 0.0, 1.0)
        return float(0.35 * spread_score + 0.20 * balance_score + 0.25 * centroid_score + 0.20 * aspect_score)

    def _fast_start_candidate(self, roi):
        """멀리 있는 출발 신호등을 빠르게 잡기 위한 색상 기반 보조 판정."""

        stats = self._color_stats(roi)
        candidates = {
            color: stat
            for color, stat in stats.items()
            if self._is_lamp_like_blob(stat, roi.shape)
        }
        if not candidates:
            return self.UNKNOWN
        return max(candidates, key=lambda color: candidates[color]["area"])

    def _compact_lamp_candidates(self, roi):
        """Return compact lamp-like color blobs and ignore broad scenery colors."""

        candidates = {}
        roi_height, roi_width = roi.shape[:2]
        for color in (self.START_RED, self.START_YELLOW, self.START_GREEN):
            blobs = []
            for blob in self._colored_blobs(roi, color):
                if blob["cy"] is None or blob["cx"] is None:
                    continue
                if blob["cy"] > roi_height * 0.72:
                    continue
                if blob["width"] < 8 or blob["height"] < 8:
                    continue
                if blob["width"] > roi_width * 0.18 or blob["height"] > roi_height * 0.30:
                    continue
                if blob["fill_ratio"] < 0.24:
                    continue
                blobs.append(blob)
            if blobs:
                candidates[color] = max(blobs, key=lambda item: item["area"])
        return candidates

    def _has_dark_signal_bar_support(self, roi, lamp, color=None):
        """Confirm a compact colored blob belongs to a traffic light bar."""

        cx = lamp.get("cx")
        cy = lamp.get("cy")
        if cx is None or cy is None:
            return False
        x = int(lamp.get("x", 0))
        y = int(lamp.get("y", 0))
        width = int(lamp.get("width", 0))
        height = int(lamp.get("height", 0))
        if width <= 0 or height <= 0:
            return False

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 120, 100]))
        roi_height, roi_width = roi.shape[:2]
        dark[int(roi_height * 0.78):, :] = 0

        band_top = max(0, int(y - height * 0.55))
        band_bottom = min(roi_height, int(y + height * 1.55))
        if color == self.START_RED:
            band_left = max(0, int(x - width * 0.55))
            band_right = min(roi_width, int(x + width * 5.2))
        elif color == self.START_GREEN:
            band_left = max(0, int(x - width * 5.2))
            band_right = min(roi_width, int(x + width * 0.55))
        else:
            band_left = max(0, int(x - width * 3.0))
            band_right = min(roi_width, int(x + width * 3.0))
        if band_right <= band_left or band_bottom <= band_top:
            return False

        band = dark[band_top:band_bottom, band_left:band_right]
        if cv2.countNonZero(band) < max(self.min_area * 8.0, width * height * 1.5):
            return False

        contours, _ = cv2.findContours(band, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            bx, by, bw, bh = cv2.boundingRect(contour)
            if area < max(self.min_area * 6.0, width * height * 0.75):
                continue
            if bw < width * 2.0 or bh < height * 0.40:
                continue
            global_left = band_left + bx
            global_right = global_left + bw
            if global_right < x - width * 0.20 or global_left > x + width * 1.20:
                continue
            return True
        return False

    def _fast_left_candidate(self, roi):
        """4구 배치가 아직 뚜렷하지 않을 때 좌회전 화살표를 빠르게 보조 판정한다."""

        stats = self._color_stats(roi)
        red = stats[self.START_RED]
        green = stats[self.START_GREEN]
        if not self._is_lamp_like_blob(red, roi.shape, area_scale=1.4):
            return False
        if not self._is_lamp_like_blob(green, roi.shape, area_scale=0.9):
            return False
        if red["cx"] is None or green["cx"] is None or red["cy"] is None or green["cy"] is None:
            return False
        if abs(green["cy"] - red["cy"]) > roi.shape[0] * 0.18:
            return False
        return green["cx"] > red["cx"] + roi.shape[1] * 0.12

    def _is_lamp_like_blob(self, stats, image_shape, area_scale=1.0):
        """나무/잔디처럼 넓게 퍼진 녹색 덩어리를 신호등 램프로 오인하지 않게 거른다."""

        image_height, image_width = image_shape[:2]
        if stats["area"] < self.min_area * area_scale:
            return False
        if stats["cx"] is None or stats["cy"] is None:
            return False
        if stats["width"] <= 0 or stats["height"] <= 0:
            return False
        if stats["width"] > image_width * 0.20 or stats["height"] > image_height * 0.45:
            return False
        if stats["fill_ratio"] < 0.28:
            return False
        if stats["cy"] > image_height * 0.78:
            return False
        return True

    def annotate(self, image):
        """디버그 화면용 ROI 박스와 판정 결과를 그린 이미지를 반환한다."""

        output = image.copy()
        start_signal = self.detect_start_light(image)
        left_detail = self.detect_left_turn_detail(image, update_history=True)
        left_signal = left_detail["stable_signal"]
        start_layout, _ = self._detect_light_layout(self.start_roi.crop(image)[0])
        left_layout = left_detail["layout"]

        self._draw_roi(
            output,
            self.start_roi,
            (255, 180, 0),
            f"START: {start_signal} {start_layout}",
        )
        self._draw_roi(
            output,
            self.left_roi,
            (0, 255, 120) if left_signal else (80, 80, 80),
            f"LEFT: {left_signal} {left_layout} {left_detail['reason']}",
            y_offset=28,
        )
        return output, start_signal, left_signal

    def _color_scores(self, bgr_image, geometry=None):
        return {
            color: stats["area"]
            for color, stats in self._color_stats(bgr_image, geometry).items()
        }

    def _color_stats(self, bgr_image, geometry=None):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

        # 빨간색은 HSV hue가 0 근처와 180 근처로 나뉘므로 두 마스크를 합친다.
        red1 = cv2.inRange(hsv, np.array([0, 65, 65]), np.array([10, 255, 255]))
        red2 = cv2.inRange(hsv, np.array([170, 65, 65]), np.array([180, 255, 255]))
        yellow = cv2.inRange(hsv, np.array([15, 60, 65]), np.array([48, 255, 255]))

        # 노란불을 출발 허용으로 오인하지 않도록 녹색/청록/파랑 범위는 보수적으로 잡는다.
        green = cv2.inRange(hsv, np.array([50, 55, 60]), np.array([125, 255, 255]))

        lamp_mask = self._geometry_mask(bgr_image, geometry)
        if lamp_mask is not None:
            red1 = cv2.bitwise_and(red1, red1, mask=lamp_mask)
            red2 = cv2.bitwise_and(red2, red2, mask=lamp_mask)
            yellow = cv2.bitwise_and(yellow, yellow, mask=lamp_mask)
            green = cv2.bitwise_and(green, green, mask=lamp_mask)

        return {
            self.START_RED: self._largest_blob_stats(red1 | red2),
            self.START_YELLOW: self._largest_blob_stats(yellow),
            self.START_GREEN: self._largest_blob_stats(green),
        }

    def _largest_area(self, mask):
        return self._largest_blob_stats(mask)["area"]

    def _largest_blob_stats(self, mask):
        mask = self._clean_color_mask(mask)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return {
                "area": 0.0,
                "cx": None,
                "cy": None,
                "width": 0,
                "height": 0,
                "fill_ratio": 0.0,
            }

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        x, y, width, height = cv2.boundingRect(contour)
        fill_ratio = area / max(1.0, float(width * height))
        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            return {
                "area": area,
                "cx": None,
                "cy": None,
                "width": width,
                "height": height,
                "fill_ratio": fill_ratio,
            }
        return {
            "area": area,
            "cx": moments["m10"] / moments["m00"],
            "cy": moments["m01"] / moments["m00"],
            "width": width,
            "height": height,
            "fill_ratio": fill_ratio,
        }

    def _clean_color_mask(self, mask):
        """Remove tiny color speckles without erasing compact signal blobs."""

        mask = cv2.medianBlur(mask, 5)
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        return mask

    def _detect_light_layout(self, bgr_image):
        housing = self._detect_housing_rect(bgr_image)
        if housing is not None:
            _, _, width, height = housing
            aspect = width / max(1, height)
            if aspect >= self.four_light_aspect:
                return self.LAYOUT_FOUR, {"housing": housing}

        circles = self._detect_lamp_circles(bgr_image)
        if len(circles) >= 4:
            if housing is None:
                return self.UNKNOWN, None
            return self.LAYOUT_FOUR, {"circles": circles, "housing": housing}
        if len(circles) == 3:
            housing = self._detect_housing_rect(bgr_image)
            if housing is not None:
                _, _, width, height = housing
                if width / max(1, height) >= self.four_light_aspect:
                    return self.LAYOUT_FOUR, {"circles": circles, "housing": housing}
            return self.LAYOUT_THREE, {"circles": circles}

        if housing is None:
            return self.UNKNOWN, None

        _, _, width, height = housing
        aspect = width / max(1, height)
        if aspect >= self.four_light_aspect:
            return self.LAYOUT_FOUR, {"housing": housing}
        return self.LAYOUT_THREE, housing

    def _detect_lamp_circles(self, bgr_image):
        gray = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.medianBlur(gray, 5)
        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.2,
            minDist=35,
            param1=60,
            param2=30,
            minRadius=20,
            maxRadius=42,
        )
        if circles is None:
            return []

        image_height, image_width = bgr_image.shape[:2]
        candidates = []
        for circle in np.round(circles[0]).astype(int):
            x, y, radius = tuple(map(int, circle))
            if radius < 20:
                continue
            if x - radius < 3 or x + radius > image_width - 3:
                continue
            if y - radius < 3 or y + radius > image_height - 3:
                continue
            candidates.append((x, y, radius))
        candidates = sorted(candidates, key=lambda circle: circle[0])

        best_group = []
        for _, y_anchor, _ in candidates:
            group = [
                circle
                for circle in candidates
                if abs(circle[1] - y_anchor) <= 28
            ]
            group = self._dedupe_circles_by_x(group)
            if len(group) > len(best_group):
                best_group = group

        if len(best_group) < 3:
            return []

        xs = [circle[0] for circle in best_group]
        span = max(xs) - min(xs)
        if span < 70:
            return []

        if len(best_group) >= 4:
            gaps = np.diff([circle[0] for circle in sorted(best_group, key=lambda circle: circle[0])[:4]])
            median_gap = float(np.median(gaps))
            if median_gap <= 0 or max(gaps) > median_gap * 1.9:
                return []

        return sorted(best_group, key=lambda circle: circle[0])[:4]

    def _dedupe_circles_by_x(self, circles):
        deduped = []
        for circle in sorted(circles, key=lambda item: item[0]):
            if deduped and abs(circle[0] - deduped[-1][0]) < 22:
                if circle[2] > deduped[-1][2]:
                    deduped[-1] = circle
            else:
                deduped.append(circle)
        return deduped

    def _geometry_x_span(self, geometry):
        if isinstance(geometry, dict):
            if "circles" in geometry:
                xs = [circle[0] for circle in geometry["circles"]]
                radii = [circle[2] for circle in geometry["circles"]]
                left = min(xs) - max(radii)
                right = max(xs) + max(radii)
                return left, right - left
            if "housing" in geometry:
                x, _, width, _ = geometry["housing"]
                return x, width

        x, _, width, _ = geometry
        return x, width

    def _geometry_mask(self, bgr_image, geometry):
        mask = np.zeros(bgr_image.shape[:2], dtype=np.uint8)
        if not isinstance(geometry, dict):
            if geometry is None:
                return None
            x, y, width, height = geometry
            pad_x = max(1, int(width * 0.02))
            pad_y = max(1, int(height * 0.05))
            cv2.rectangle(
                mask,
                (max(0, x + pad_x), max(0, y + pad_y)),
                (min(bgr_image.shape[1] - 1, x + width - pad_x), min(bgr_image.shape[0] - 1, y + height - pad_y)),
                255,
                -1,
            )
            return mask

        if "housing" in geometry:
            x, y, width, height = geometry["housing"]
            pad_x = max(1, int(width * 0.02))
            pad_y = max(1, int(height * 0.05))
            cv2.rectangle(
                mask,
                (max(0, x + pad_x), max(0, y + pad_y)),
                (min(bgr_image.shape[1] - 1, x + width - pad_x), min(bgr_image.shape[0] - 1, y + height - pad_y)),
                255,
                -1,
            )
            return mask

        if "circles" in geometry:
            for x, y, radius in geometry["circles"]:
                cv2.circle(mask, (int(x), int(y)), max(8, int(radius * 0.82)), 255, -1)
            return mask

        return None

    def _detect_housing_rect(self, bgr_image):
        candidates = self._detect_housing_candidates(bgr_image)
        if not candidates:
            return None
        _, box = candidates[0]
        return box

    def _detect_housing_candidates(self, bgr_image):
        hsv = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        dark = cv2.inRange(hsv, np.array([0, 0, 0]), np.array([180, 120, 95]))
        image_height, image_width = bgr_image.shape[:2]
        dark[int(image_height * 0.72):, :] = 0
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 5))
        dark = cv2.morphologyEx(dark, cv2.MORPH_CLOSE, kernel, iterations=2)
        contours, _ = cv2.findContours(dark, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        candidates = []
        for contour in contours:
            x, y, width, height = cv2.boundingRect(contour)
            area = width * height
            aspect = width / max(1, height)
            if area < 800:
                continue
            if not 1.3 <= aspect <= 5.8:
                continue
            if height < 18 or height > image_height * 0.55:
                continue
            if width < image_width * 0.10 or width > image_width * 0.92:
                continue
            if y > image_height * 0.58:
                continue
            color_score = self._housing_color_score(hsv, x, y, width, height)
            if color_score < self.min_area:
                continue
            four_like_bonus = 50000 if aspect >= self.four_light_aspect else 0
            candidates.append((four_like_bonus + area, color_score, x, y, width, height))

        if not candidates:
            return []

        ranked = sorted(candidates, key=lambda item: (item[0], item[1]), reverse=True)
        return [
            ("HOUSING", (x, y, width, height))
            for _, _, x, y, width, height in ranked[:4]
        ]

    def _housing_color_score(self, hsv, x, y, width, height):
        roi = hsv[y:y + height, x:x + width]
        if roi.size == 0:
            return 0.0
        red1 = cv2.inRange(roi, np.array([0, 65, 65]), np.array([10, 255, 255]))
        red2 = cv2.inRange(roi, np.array([170, 65, 65]), np.array([180, 255, 255]))
        yellow = cv2.inRange(roi, np.array([15, 60, 65]), np.array([48, 255, 255]))
        green = cv2.inRange(roi, np.array([50, 55, 60]), np.array([125, 255, 255]))
        return float(cv2.countNonZero(red1 | red2 | yellow | green))

    def _stable_value(self, history, candidate):
        if candidate == self.UNKNOWN:
            return self.UNKNOWN
        if sum(1 for value in history if value == candidate) >= self.stable_count:
            return candidate
        return self.UNKNOWN

    def _stable_bool(self, history, target):
        if not history or history[-1] is not target:
            return False
        return sum(1 for value in history if value is target) >= self.stable_count

    def _draw_roi(self, image, roi, color, label, y_offset=0):
        _, (left, top, right, bottom) = roi.crop(image)
        cv2.rectangle(image, (left, top), (right, bottom), color, 2)
        cv2.putText(
            image,
            label,
            (left, max(18, top + 20 + y_offset)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            color,
            2,
            cv2.LINE_AA,
        )






